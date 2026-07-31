// Copyright 2024, DCS Digital / Scott Bowler
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  ALVR compositor target — captures Monado frames, encodes with NVENC,
 *         streams to Apple Vision Pro via alvr_server_core.
 *
 * Frame path:
 *   Monado compositor (Vulkan) → VkImage
 *       → vkCmdCopyImage → CUDA-mapped staging buffer
 *       → FFmpeg AVFrame (nv12, CUDA)
 *       → h265_nvenc → NAL units
 *       → alvr_send_video_nal() → Vision Pro
 *
 * @ingroup drv_alvr
 */

extern "C" {
#include "alvr_interface.h"
#include "alvr_server_core.h"

// Monado internal headers (available from source tree build)
#include "../../compositor/main/comp_compositor.h"
#include "../../compositor/main/comp_target.h"
#include "../../compositor/util/comp_vulkan.h"

#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "os/os_time.h"
}

#include <vulkan/vulkan.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_cuda.h>
}

#define ALVR_TARGET_MAX_IMAGES 3
#define AVP_EYE_WIDTH  1832u
#define AVP_EYE_HEIGHT 1920u
#define AVP_REFRESH_HZ  90.0f
#define LOG_D(...) U_LOG_D("alvr_target: " __VA_ARGS__)
#define LOG_I(...) U_LOG_I("alvr_target: " __VA_ARGS__)
#define LOG_E(...) U_LOG_E("alvr_target: " __VA_ARGS__)

// ── per-image state ───────────────────────────────────────────────────────────

struct alvr_target_image {
	VkImage          image;
	VkDeviceMemory   memory;
	VkImageView      view;
	// CPU-visible staging for readback
	VkImage          staging_image;
	VkDeviceMemory   staging_memory;
	VkCommandBuffer  cmd;
};

// ── target struct ─────────────────────────────────────────────────────────────

struct alvr_target {
	struct comp_target      base;
	struct comp_compositor *c;

	uint32_t width;
	uint32_t height;

	uint32_t                   image_count;
	struct alvr_target_image   images[ALVR_TARGET_MAX_IMAGES];
	uint32_t                   current_index;

	VkCommandPool  cmd_pool;
	VkFence        fence;

	// FFmpeg NVENC encoder
	AVCodecContext  *enc_ctx;
	AVBufferRef     *hw_device_ctx;
	AVFrame         *hw_frame;
	AVFrame         *sw_frame;
	AVPacket        *pkt;
	bool             encoder_ready;
	bool             sent_config_nals;

	// Frame timing
	int64_t          last_present_ns;
};

static inline struct alvr_target *
alvr_target(struct comp_target *ct)
{
	return (struct alvr_target *)ct;
}

// ── encoder ───────────────────────────────────────────────────────────────────

static bool
alvr_encoder_init(struct alvr_target *at)
{
	const AVCodec *codec = avcodec_find_encoder_by_name("hevc_nvenc");
	if (!codec) {
		codec = avcodec_find_encoder_by_name("h264_nvenc");
		if (!codec) {
			LOG_E("No NVENC encoder found");
			return false;
		}
	}

	// CUDA hardware context
	if (av_hwdevice_ctx_create(&at->hw_device_ctx, AV_HWDEVICE_TYPE_CUDA,
	                           NULL, NULL, 0) < 0) {
		LOG_E("Failed to create CUDA hw device context");
		return false;
	}

	at->enc_ctx = avcodec_alloc_context3(codec);
	if (!at->enc_ctx)
		return false;

	at->enc_ctx->width     = (int)at->width;
	at->enc_ctx->height    = (int)at->height;
	at->enc_ctx->pix_fmt   = AV_PIX_FMT_CUDA;
	at->enc_ctx->hw_device_ctx = av_buffer_ref(at->hw_device_ctx);

	// NVENC options for low-latency streaming
	av_opt_set(at->enc_ctx->priv_data, "preset",    "p1",          0); // fastest
	av_opt_set(at->enc_ctx->priv_data, "tune",      "ull",         0); // ultra-low latency
	av_opt_set(at->enc_ctx->priv_data, "rc",        "cbr",         0);
	av_opt_set(at->enc_ctx->priv_data, "zerolatency","1",           0);
	at->enc_ctx->bit_rate  = 50000000; // 50 Mbps — adjustable from ALVR settings
	at->enc_ctx->gop_size  = 0;        // IDR only when requested
	at->enc_ctx->time_base = {1, 90000};
	at->enc_ctx->framerate = {90, 1};

	// Create HW frame pool
	AVBufferRef *hw_frames_ref = av_hwframe_ctx_alloc(at->hw_device_ctx);
	AVHWFramesContext *frames_ctx = (AVHWFramesContext *)hw_frames_ref->data;
	frames_ctx->format    = AV_PIX_FMT_CUDA;
	frames_ctx->sw_format = AV_PIX_FMT_NV12;
	frames_ctx->width     = (int)at->width;
	frames_ctx->height    = (int)at->height;
	frames_ctx->initial_pool_size = 4;

	if (av_hwframe_ctx_init(hw_frames_ref) < 0) {
		LOG_E("Failed to init hw frames context");
		av_buffer_unref(&hw_frames_ref);
		return false;
	}
	at->enc_ctx->hw_frames_ctx = av_buffer_ref(hw_frames_ref);
	av_buffer_unref(&hw_frames_ref);

	if (avcodec_open2(at->enc_ctx, codec, NULL) < 0) {
		LOG_E("Failed to open NVENC encoder");
		return false;
	}

	at->hw_frame = av_frame_alloc();
	at->sw_frame = av_frame_alloc();
	at->pkt      = av_packet_alloc();

	if (av_hwframe_get_buffer(at->enc_ctx->hw_frames_ctx, at->hw_frame, 0) < 0) {
		LOG_E("Failed to alloc hw frame");
		return false;
	}

	LOG_I("NVENC encoder ready: %s %ux%u @ 50Mbps",
	      codec->name, at->width, at->height);
	return true;
}

static void
alvr_encoder_destroy(struct alvr_target *at)
{
	if (at->hw_frame)   av_frame_free(&at->hw_frame);
	if (at->sw_frame)   av_frame_free(&at->sw_frame);
	if (at->pkt)        av_packet_free(&at->pkt);
	if (at->enc_ctx)    avcodec_free_context(&at->enc_ctx);
	if (at->hw_device_ctx) av_buffer_unref(&at->hw_device_ctx);
}

// ── image readback + encode + send ───────────────────────────────────────────

static void
alvr_encode_and_send(struct alvr_target *at, uint32_t index, uint64_t timestamp_ns)
{
	if (!at->encoder_ready)
		return;

	struct alvr_target_image *img = &at->images[index];
	struct vk_bundle *vk = &at->c->base.vk;

	// Submit readback command buffer (image → staging)
	VkSubmitInfo si = {VK_STRUCTURE_TYPE_SUBMIT_INFO};
	si.commandBufferCount = 1;
	si.pCommandBuffers    = &img->cmd;

	vk->vkResetFences(vk->device, 1, &at->fence);
	vk->vkQueueSubmit(vk->graphics_queue->queue, 1, &si, at->fence);
	vk->vkWaitForFences(vk->device, 1, &at->fence, VK_TRUE, UINT64_MAX);

	// Map staging memory → CPU
	void *mapped = NULL;
	VkResult res = vk->vkMapMemory(vk->device, img->staging_memory,
	                                0, VK_WHOLE_SIZE, 0, &mapped);
	if (res != VK_SUCCESS || !mapped) {
		LOG_E("Failed to map staging memory");
		return;
	}

	// Copy BGRA staging → sw_frame (NV12 conversion done by FFmpeg scale_cuda)
	at->sw_frame->format = AV_PIX_FMT_BGRA;
	at->sw_frame->width  = (int)at->width;
	at->sw_frame->height = (int)at->height;
	at->sw_frame->data[0]     = (uint8_t *)mapped;
	at->sw_frame->linesize[0] = (int)(at->width * 4);

	// Upload to CUDA hw frame (triggers BGRA→NV12 on GPU)
	if (av_hwframe_transfer_data(at->hw_frame, at->sw_frame, 0) < 0) {
		LOG_E("Failed to transfer frame to CUDA");
		vk->vkUnmapMemory(vk->device, img->staging_memory);
		return;
	}
	vk->vkUnmapMemory(vk->device, img->staging_memory);

	at->hw_frame->pts = (int64_t)(timestamp_ns / 1000); // microseconds

	// Encode
	if (avcodec_send_frame(at->enc_ctx, at->hw_frame) < 0) {
		LOG_E("avcodec_send_frame failed");
		return;
	}

	while (avcodec_receive_packet(at->enc_ctx, at->pkt) == 0) {
		// Send config NALs on first frame
		if (!at->sent_config_nals) {
			AlvrCodecType codec = ALVR_CODEC_TYPE_HEVC;
			if (at->enc_ctx->codec_id == AV_CODEC_ID_H264)
				codec = ALVR_CODEC_TYPE_H264;

			alvr_set_video_config_nals(codec,
			    at->enc_ctx->extradata,
			    at->enc_ctx->extradata_size);
			at->sent_config_nals = true;
		}

		bool is_idr = (at->pkt->flags & AV_PKT_FLAG_KEY) != 0;
		// Provide view params snapshot for latency correction
		struct AlvrViewParams vp[2] = {};
		alvr_send_video_nal(timestamp_ns,
		                    vp,
		                    is_idr,
		                    at->pkt->data,
		                    (int32_t)at->pkt->size);

		av_packet_unref(at->pkt);
	}

	alvr_report_composed(timestamp_ns, 0);
}

// ── comp_target function table ────────────────────────────────────────────────

static bool
alvr_target_init_pre_vulkan(struct comp_target *ct)
{
	(void)ct;
	return true;
}

static bool
alvr_target_init_post_vulkan(struct comp_target *ct,
                              uint32_t preferred_width,
                              uint32_t preferred_height)
{
	struct alvr_target *at = alvr_target(ct);
	struct vk_bundle   *vk = &at->c->base.vk;

	at->width  = preferred_width  ? preferred_width  : (AVP_EYE_WIDTH * 2);
	at->height = preferred_height ? preferred_height : AVP_EYE_HEIGHT;

	// Create command pool (use graphics queue family)
	VkCommandPoolCreateInfo cpci = {VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
	cpci.queueFamilyIndex = vk->graphics_queue->family_index;
	cpci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	vk->vkCreateCommandPool(vk->device, &cpci, NULL, &at->cmd_pool);

	// Create fence
	VkFenceCreateInfo fci = {VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
	vk->vkCreateFence(vk->device, &fci, NULL, &at->fence);

	// Initialise encoder
	at->encoder_ready = alvr_encoder_init(at);

	return true;
}

static bool
alvr_target_check_ready(struct comp_target *ct)
{
	return true;
}

static bool
alvr_target_is_shared_presentable_image(struct comp_target *ct)
{
	return false;
}

static void
alvr_target_create_images(struct comp_target *ct,
                           const struct comp_target_create_images_info *info,
                           struct vk_bundle_queue *present_queue)
{
	struct alvr_target *at = alvr_target(ct);
	struct vk_bundle   *vk = &at->c->base.vk;

	at->image_count = ALVR_TARGET_MAX_IMAGES;
	ct->image_count = at->image_count;
	ct->images      = U_TYPED_ARRAY_CALLOC(struct comp_target_image, at->image_count);

	VkCommandBufferAllocateInfo cbai = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
	cbai.commandPool        = at->cmd_pool;
	cbai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	cbai.commandBufferCount = 1;

	for (uint32_t i = 0; i < at->image_count; i++) {
		struct alvr_target_image *img = &at->images[i];

		// Render target image (GPU-local)
		VkImageCreateInfo ici = {VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
		ici.imageType   = VK_IMAGE_TYPE_2D;
		ici.format      = VK_FORMAT_B8G8R8A8_UNORM;
		ici.extent      = {at->width, at->height, 1};
		ici.mipLevels   = 1;
		ici.arrayLayers = 1;
		ici.samples     = VK_SAMPLE_COUNT_1_BIT;
		ici.tiling      = VK_IMAGE_TILING_OPTIMAL;
		ici.usage       = info->image_usage |
		                  VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
		                  VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
		vk->vkCreateImage(vk->device, &ici, NULL, &img->image);

		VkMemoryRequirements mr;
		vk->vkGetImageMemoryRequirements(vk->device, img->image, &mr);
		VkMemoryAllocateInfo mai = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
		mai.allocationSize  = mr.size;
		uint32_t mem_type_idx = 0;
		vk_get_memory_type(vk, mr.memoryTypeBits,
		    VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, &mem_type_idx);
		mai.memoryTypeIndex = mem_type_idx;
		vk->vkAllocateMemory(vk->device, &mai, NULL, &img->memory);
		vk->vkBindImageMemory(vk->device, img->image, img->memory, 0);

		// Host-visible staging image for CPU readback
		VkImageCreateInfo sci = ici;
		sci.tiling = VK_IMAGE_TILING_LINEAR;
		sci.usage  = VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		vk->vkCreateImage(vk->device, &sci, NULL, &img->staging_image);

		VkMemoryRequirements smr;
		vk->vkGetImageMemoryRequirements(vk->device, img->staging_image, &smr);
		VkMemoryAllocateInfo smai = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
		smai.allocationSize  = smr.size;
		uint32_t smem_type_idx = 0;
		vk_get_memory_type(vk, smr.memoryTypeBits,
		    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		    &smem_type_idx);
		smai.memoryTypeIndex = smem_type_idx;
		vk->vkAllocateMemory(vk->device, &smai, NULL, &img->staging_memory);
		vk->vkBindImageMemory(vk->device, img->staging_image, img->staging_memory, 0);

		// Pre-record readback command buffer
		vk->vkAllocateCommandBuffers(vk->device, &cbai, &img->cmd);

		VkCommandBufferBeginInfo cbbi = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
		vk->vkBeginCommandBuffer(img->cmd, &cbbi);

		// Transition render image to TRANSFER_SRC
		VkImageMemoryBarrier b0 = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
		b0.srcAccessMask    = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		b0.dstAccessMask    = VK_ACCESS_TRANSFER_READ_BIT;
		b0.oldLayout        = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		b0.newLayout        = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
		b0.image            = img->image;
		b0.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
		vk->vkCmdPipelineBarrier(img->cmd,
		    VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
		    VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, NULL, 0, NULL, 1, &b0);

		// Transition staging to TRANSFER_DST
		VkImageMemoryBarrier b1 = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
		b1.srcAccessMask    = 0;
		b1.dstAccessMask    = VK_ACCESS_TRANSFER_WRITE_BIT;
		b1.oldLayout        = VK_IMAGE_LAYOUT_UNDEFINED;
		b1.newLayout        = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		b1.image            = img->staging_image;
		b1.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
		vk->vkCmdPipelineBarrier(img->cmd,
		    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
		    VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, NULL, 0, NULL, 1, &b1);

		// Copy
		VkImageCopy region = {};
		region.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
		region.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
		region.extent = {at->width, at->height, 1};
		vk->vkCmdCopyImage(img->cmd,
		    img->image,   VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
		    img->staging_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
		    1, &region);

		vk->vkEndCommandBuffer(img->cmd);

		// Fill comp_target_image
		VkImageViewCreateInfo ivci = {VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
		ivci.image    = img->image;
		ivci.viewType = VK_IMAGE_VIEW_TYPE_2D;
		ivci.format   = VK_FORMAT_B8G8R8A8_UNORM;
		ivci.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
		vk->vkCreateImageView(vk->device, &ivci, NULL, &img->view);

		ct->images[i].handle = img->image;
		ct->images[i].view   = img->view;
	}

	ct->width  = at->width;
	ct->height = at->height;
	ct->format = VK_FORMAT_B8G8R8A8_UNORM;
}

static bool
alvr_target_has_images(struct comp_target *ct)
{
	return alvr_target(ct)->image_count > 0;
}

static VkResult
alvr_target_acquire(struct comp_target *ct, uint32_t *out_index)
{
	struct alvr_target *at = alvr_target(ct);
	at->current_index = (at->current_index + 1) % at->image_count;
	*out_index = at->current_index;
	return VK_SUCCESS;
}

static VkResult
alvr_target_present(struct comp_target *ct,
                     struct vk_bundle_queue *present_queue,
                     uint32_t index,
                     uint64_t timeline_semaphore_value,
                     int64_t desired_present_time_ns,
                     int64_t present_slop_ns)
{
	struct alvr_target *at = alvr_target(ct);
	uint64_t ts_ns = (uint64_t)os_monotonic_get_ns();

	// Wait for GPU work to finish before readback
	(void)present_queue;
	{
		struct vk_bundle *vk = &at->c->base.vk;
		vk->vkQueueWaitIdle(vk->graphics_queue->queue);
	}

	alvr_encode_and_send(at, index, ts_ns);
	alvr_report_present(ts_ns, 0);

	at->last_present_ns = (int64_t)ts_ns;
	return VK_SUCCESS;
}

static VkResult
alvr_target_wait_for_present(struct comp_target *ct, time_duration_ns timeout_ns)
{
	return VK_SUCCESS;
}

static void
alvr_target_flush(struct comp_target *ct)
{
	// Nothing to flush
}

static void
alvr_target_calc_frame_pacing(struct comp_target *ct,
                               int64_t *out_frame_id,
                               int64_t *out_wake_up_time_ns,
                               int64_t *out_desired_present_time_ns,
                               int64_t *out_present_slop_ns,
                               int64_t *out_predicted_display_time_ns)
{
	// Simple fixed 90Hz pacing
	int64_t now = (int64_t)os_monotonic_get_ns();
	int64_t frame_ns = (int64_t)(1000000000.0 / 90.0);

	static int64_t frame_id = 0;
	*out_frame_id                    = frame_id++;
	*out_desired_present_time_ns     = now + frame_ns;
	*out_wake_up_time_ns             = now;
	*out_present_slop_ns             = frame_ns / 4;
	*out_predicted_display_time_ns   = now + frame_ns * 2;
}

static void
alvr_target_mark_timing_point(struct comp_target *ct,
                               enum comp_target_timing_point point,
                               int64_t frame_id,
                               int64_t when_ns)
{
	// Could feed into ALVR's dynamic encoder params in the future
}

static VkResult
alvr_target_update_timings(struct comp_target *ct)
{
	return VK_SUCCESS;
}

static void
alvr_target_info_gpu(struct comp_target *ct,
                      int64_t frame_id,
                      int64_t gpu_start_ns,
                      int64_t gpu_end_ns,
                      int64_t when_ns)
{
	// Timing data — could use for adaptive bitrate
}

static void
alvr_target_set_title(struct comp_target *ct, const char *title)
{
	(void)ct; (void)title;
}

static xrt_result_t
alvr_target_get_refresh_rates(struct comp_target *ct,
                               uint32_t *out_count,
                               float *out_rates)
{
	if (out_rates && out_count && *out_count > 0)
		out_rates[0] = 90.0f;
	if (out_count)
		*out_count = 1;
	return XRT_SUCCESS;
}

static xrt_result_t
alvr_target_get_current_refresh_rate(struct comp_target *ct, float *out_hz)
{
	*out_hz = 90.0f;
	return XRT_SUCCESS;
}

static xrt_result_t
alvr_target_request_refresh_rate(struct comp_target *ct, float hz)
{
	return XRT_SUCCESS;
}

static VkResult
alvr_target_queue_supports_present(struct comp_target *ct,
                                    struct vk_bundle_queue *queue,
                                    VkBool32 *out_supported)
{
	*out_supported = VK_TRUE;
	return VK_SUCCESS;
}

static void
alvr_target_destroy(struct comp_target *ct)
{
	struct alvr_target *at = alvr_target(ct);
	struct vk_bundle   *vk = &at->c->base.vk;

	alvr_encoder_destroy(at);

	for (uint32_t i = 0; i < at->image_count; i++) {
		struct alvr_target_image *img = &at->images[i];
		if (img->view)           vk->vkDestroyImageView(vk->device, img->view, NULL);
		if (img->image)          vk->vkDestroyImage(vk->device, img->image, NULL);
		if (img->memory)         vk->vkFreeMemory(vk->device, img->memory, NULL);
		if (img->staging_image)  vk->vkDestroyImage(vk->device, img->staging_image, NULL);
		if (img->staging_memory) vk->vkFreeMemory(vk->device, img->staging_memory, NULL);
	}

	if (at->cmd_pool) vk->vkDestroyCommandPool(vk->device, at->cmd_pool, NULL);
	if (at->fence)    vk->vkDestroyFence(vk->device, at->fence, NULL);

	free(ct->images);
	free(at);
}

// ── constructor ───────────────────────────────────────────────────────────────

extern "C" struct comp_target *
alvr_target_create(struct comp_compositor *c)
{
	struct alvr_target *at = U_TYPED_CALLOC(struct alvr_target);
	if (!at)
		return NULL;

	at->c = c;

	struct comp_target *ct = &at->base;
	ct->name = "ALVR";

	ct->init_pre_vulkan            = alvr_target_init_pre_vulkan;
	ct->init_post_vulkan           = alvr_target_init_post_vulkan;
	ct->check_ready                = alvr_target_check_ready;
	ct->is_shared_presentable_image = alvr_target_is_shared_presentable_image;
	ct->create_images              = alvr_target_create_images;
	ct->has_images                 = alvr_target_has_images;
	ct->acquire                    = alvr_target_acquire;
	ct->present                    = alvr_target_present;
	ct->wait_for_present           = alvr_target_wait_for_present;
	ct->flush                      = alvr_target_flush;
	ct->calc_frame_pacing          = alvr_target_calc_frame_pacing;
	ct->mark_timing_point          = alvr_target_mark_timing_point;
	ct->update_timings             = alvr_target_update_timings;
	ct->info_gpu                   = alvr_target_info_gpu;
	ct->set_title                  = alvr_target_set_title;
	ct->get_refresh_rates          = alvr_target_get_refresh_rates;
	ct->get_current_refresh_rate   = alvr_target_get_current_refresh_rate;
	ct->request_refresh_rate       = alvr_target_request_refresh_rate;
	ct->queue_supports_present     = alvr_target_queue_supports_present;
	ct->destroy                    = alvr_target_destroy;

	LOG_I("ALVR compositor target created (%ux%u)",
	      AVP_EYE_WIDTH * 2, AVP_EYE_HEIGHT);
	return ct;
}

// ── comp_target_factory ───────────────────────────────────────────────────────

static bool
alvr_factory_detect(const struct comp_target_factory *ctf,
                    struct comp_compositor *c)
{
	// Always prefer ALVR when it's compiled in
	(void)ctf; (void)c;
	return true;
}

static bool
alvr_factory_create_target(const struct comp_target_factory *ctf,
                            struct comp_compositor *c,
                            struct comp_target **out_ct)
{
	(void)ctf;
	*out_ct = alvr_target_create(c);
	return *out_ct != NULL;
}

extern "C" const struct comp_target_factory comp_target_factory_alvr = {
	.name                            = "ALVR (Vision Pro streaming)",
	.identifier                      = "alvr",
	.requires_vulkan_for_create      = true,
	.is_deferred                     = false,
	.required_instance_version       = 0,
	.required_instance_extensions    = NULL,
	.required_instance_extension_count = 0,
	.optional_device_extensions      = NULL,
	.optional_device_extension_count = 0,
	.detect                          = alvr_factory_detect,
	.create_target                   = alvr_factory_create_target,
};

// Strong definition of the weak symbol declared in comp_compositor.c.
// When drv_alvr is linked, this overrides the NULL weak default and injects
// comp_target_factory_alvr as the first compositor target.
extern "C" const struct comp_target_factory *comp_target_factory_alvr_ptr =
    &comp_target_factory_alvr;
