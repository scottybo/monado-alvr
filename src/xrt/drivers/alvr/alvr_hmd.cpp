// Copyright 2024, DCS Digital / Scott Bowler
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  ALVR HMD xrt_device — pose tracking fed from Vision Pro via ALVR.
 *
 * Architecture:
 *   alvr_server_core (Rust) ←→ network ←→ Vision Pro (ALVR app)
 *       ↓ alvr_poll_event / alvr_get_device_motion
 *   alvr_hmd (this file) implements xrt_device
 *       ↓ get_tracked_pose / get_view_poses
 *   Monado compositor → alvr_target → NVENC → Vision Pro
 *
 * @ingroup drv_alvr
 */

#include "alvr_interface.h"
#include "alvr_server_core.h"

#include "os/os_time.h"
#include "os/os_threading.h"

#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include "math/m_relation_history.h"
#include "math/m_api.h"
#include "math/m_mathinclude.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_var.h"
#include "util/u_visibility_mask.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

DEBUG_GET_ONCE_LOG_OPTION(alvr_log, "ALVR_LOG", U_LOGGING_INFO)

#define ALVR_TRACE(hmd, ...) U_LOG_XDEV_IFL_T(&(hmd)->base, (hmd)->log_level, __VA_ARGS__)
#define ALVR_DEBUG(hmd, ...) U_LOG_XDEV_IFL_D(&(hmd)->base, (hmd)->log_level, __VA_ARGS__)
#define ALVR_INFO(hmd, ...)  U_LOG_XDEV_IFL_I(&(hmd)->base, (hmd)->log_level, __VA_ARGS__)
#define ALVR_ERROR(hmd, ...) U_LOG_XDEV_IFL_E(&(hmd)->base, (hmd)->log_level, __VA_ARGS__)

// Apple Vision Pro display spec
#define AVP_REFRESH_HZ      90
#define AVP_EYE_WIDTH       1832
#define AVP_EYE_HEIGHT      1920
#define AVP_IPD_M           0.064f

struct alvr_hmd {
	struct xrt_device         base;
	enum u_logging_level      log_level;
	struct m_relation_history *relation_hist;

	// Eye-specific view poses / FOVs (updated on ALVR_EVENT_HMD_SET_STREAM_DESC)
	struct os_mutex           view_mutex;
	struct xrt_pose           view_poses[2];
	struct xrt_fov            view_fovs[2];

	// Background event thread
	pthread_t                 event_thread;
	volatile bool             running;
};

static inline struct alvr_hmd *
alvr_hmd(struct xrt_device *xdev)
{
	return (struct alvr_hmd *)xdev;
}

// ── helpers ──────────────────────────────────────────────────────────────────

static struct xrt_pose
xrt_pose_from_alvr(const AlvrPose *ap)
{
	struct xrt_pose p;
	p.orientation.x = ap->orientation.x;
	p.orientation.y = ap->orientation.y;
	p.orientation.z = ap->orientation.z;
	p.orientation.w = ap->orientation.w;
	p.position.x    = ap->position[0];
	p.position.y    = ap->position[1];
	p.position.z    = ap->position[2];
	return p;
}

static struct xrt_fov
xrt_fov_from_alvr(const AlvrFov *af)
{
	struct xrt_fov f;
	f.angle_left  = af->left;
	f.angle_right = af->right;
	f.angle_up    = af->up;
	f.angle_down  = af->down;
	return f;
}

static struct xrt_space_relation
xrt_rel_from_alvr_motion(const AlvrDeviceMotion *m)
{
	struct xrt_space_relation r = XRT_SPACE_RELATION_ZERO;
	r.relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	    XRT_SPACE_RELATION_POSITION_VALID_BIT |
	    XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
	    XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT |
	    XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);

	r.pose = xrt_pose_from_alvr(&m->pose);
	r.linear_velocity.x  = m->linear_velocity[0];
	r.linear_velocity.y  = m->linear_velocity[1];
	r.linear_velocity.z  = m->linear_velocity[2];
	r.angular_velocity.x = m->angular_velocity[0];
	r.angular_velocity.y = m->angular_velocity[1];
	r.angular_velocity.z = m->angular_velocity[2];
	return r;
}

// ── event polling thread ──────────────────────────────────────────────────────

static void *
alvr_event_thread(void *arg)
{
	struct alvr_hmd *hmd = (struct alvr_hmd *)arg;

	ALVR_INFO(hmd, "Event thread started");

	while (hmd->running) {
		AlvrEvent ev;
		// Block up to 16ms waiting for an event
		if (!alvr_poll_event(&ev, 16000000ULL))
			continue;

		switch (ev.tag) {

		case ALVR_EVENT_CLIENT_CONNECTED:
			ALVR_INFO(hmd, "Vision Pro client connected");
			break;

		case ALVR_EVENT_CLIENT_DISCONNECTED:
			ALVR_INFO(hmd, "Vision Pro client disconnected");
			break;

		case ALVR_EVENT_TRACKING_UPDATED: {
			// Pull head pose from the device motion store
			AlvrDeviceMotion motion;
			uint64_t hmd_id = alvr_path_to_id("/user/head");
			uint64_t sample_ts = ev.TRACKING_UPDATED.sample_timestamp_ns;
			if (alvr_get_device_motion(hmd_id, sample_ts, &motion)) {
				struct xrt_space_relation rel = xrt_rel_from_alvr_motion(&motion);
				int64_t now = (int64_t)os_monotonic_get_ns();
				m_relation_history_push(hmd->relation_hist, &rel, now);
			}
			break;
		}

		case ALVR_EVENT_LOCAL_VIEW_PARAMS: {
			// Update per-eye FOVs and poses from the event payload
			const AlvrViewParams *vp = ev.local_view_params;
			os_mutex_lock(&hmd->view_mutex);
			hmd->view_fovs[0]  = xrt_fov_from_alvr(&vp[0].fov);
			hmd->view_fovs[1]  = xrt_fov_from_alvr(&vp[1].fov);
			hmd->view_poses[0] = xrt_pose_from_alvr(&vp[0].pose);
			hmd->view_poses[1] = xrt_pose_from_alvr(&vp[1].pose);
			os_mutex_unlock(&hmd->view_mutex);
			ALVR_INFO(hmd, "View params updated from client");
			break;
		}

		default:
			break;
		}
	}

	ALVR_INFO(hmd, "Event thread exiting");
	return NULL;
}

// ── xrt_device implementation ─────────────────────────────────────────────────

static void
alvr_hmd_destroy(struct xrt_device *xdev)
{
	struct alvr_hmd *hmd = alvr_hmd(xdev);

	hmd->running = false;
	pthread_join(hmd->event_thread, NULL);

	alvr_shutdown();

	os_mutex_destroy(&hmd->view_mutex);
	m_relation_history_destroy(&hmd->relation_hist);
	u_var_remove_root(hmd);
	u_device_free(&hmd->base);
}

static xrt_result_t
alvr_hmd_update_inputs(struct xrt_device *xdev)
{
	return XRT_SUCCESS;
}

static xrt_result_t
alvr_hmd_get_tracked_pose(struct xrt_device *xdev,
                           enum xrt_input_name name,
                           int64_t at_timestamp_ns,
                           struct xrt_space_relation *out_relation)
{
	struct alvr_hmd *hmd = alvr_hmd(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		ALVR_ERROR(hmd, "unknown input name %u", name);
		return XRT_ERROR_POSE_NOT_ACTIVE;
	}

	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	m_relation_history_get(hmd->relation_hist, at_timestamp_ns, &rel);

	if (rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT)
		math_quat_normalize(&rel.pose.orientation);

	*out_relation = rel;
	return XRT_SUCCESS;
}

static xrt_result_t
alvr_hmd_get_view_poses(struct xrt_device *xdev,
                         const struct xrt_vec3 *default_eye_relation,
                         int64_t at_timestamp_ns,
                         enum xrt_view_type view_type,
                         uint32_t view_count,
                         struct xrt_space_relation *out_head_relation,
                         struct xrt_fov *out_fovs,
                         struct xrt_pose *out_poses)
{
	struct alvr_hmd *hmd = alvr_hmd(xdev);

	xrt_device_get_tracked_pose(xdev, XRT_INPUT_GENERIC_HEAD_POSE,
	                            at_timestamp_ns, out_head_relation);

	os_mutex_lock(&hmd->view_mutex);
	for (uint32_t i = 0; i < view_count && i < 2; i++) {
		out_fovs[i]  = hmd->view_fovs[i];
		out_poses[i] = hmd->view_poses[i];
	}
	os_mutex_unlock(&hmd->view_mutex);
	return XRT_SUCCESS;
}

static xrt_result_t
alvr_hmd_get_visibility_mask(struct xrt_device *xdev,
                              enum xrt_visibility_mask_type type,
                              uint32_t view_index,
                              struct xrt_visibility_mask **out_mask)
{
	struct xrt_fov fov = xdev->hmd->distortion.fov[view_index];
	u_visibility_mask_get_default(type, &fov, out_mask);
	return XRT_SUCCESS;
}

// ── constructor ───────────────────────────────────────────────────────────────

extern "C" struct xrt_device *
alvr_hmd_create(void)
{
	// Initialise ALVR server core (config dir picked up from env / defaults)
	const char *config_dir = getenv("ALVR_CONFIG_DIR");
	char default_config[256];
	if (!config_dir) {
		const char *home = getenv("HOME");
		snprintf(default_config, sizeof(default_config),
		         "%s/.config/alvr", home ? home : "/tmp");
		config_dir = default_config;
	}

	char log_dir[256];
	snprintf(log_dir, sizeof(log_dir), "%s/../local/share/alvr/logs", config_dir);

	alvr_initialize_environment(config_dir, log_dir);
	alvr_start_connection();

	// Allocate device
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct alvr_hmd *hmd = U_DEVICE_ALLOCATE(struct alvr_hmd, flags, 1, 0);
	if (!hmd)
		return NULL;

	hmd->log_level = debug_get_log_option_alvr_log();

	// Identity initial poses
	struct xrt_quat ident_q = {0, 0, 0, 1};
	hmd->view_poses[0].orientation = ident_q;
	hmd->view_poses[0].position    = {-AVP_IPD_M/2, 0, 0};
	hmd->view_poses[1].orientation = ident_q;
	hmd->view_poses[1].position    = { AVP_IPD_M/2, 0, 0};

	// Default Vision Pro FOV (±47° H, ±40° V approx)
	for (int i = 0; i < 2; i++) {
		hmd->view_fovs[i].angle_left  = -0.820f;  // ~47°
		hmd->view_fovs[i].angle_right =  0.820f;
		hmd->view_fovs[i].angle_up    =  0.698f;  // ~40°
		hmd->view_fovs[i].angle_down  = -0.698f;
	}

	os_mutex_init(&hmd->view_mutex);
	m_relation_history_create(&hmd->relation_hist);

	// Identity bootstrap pose
	struct xrt_space_relation identity = XRT_SPACE_RELATION_ZERO;
	identity.relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
	struct xrt_quat boot_q = {0, 0, 0, 1};
	identity.pose.orientation = boot_q;
	m_relation_history_push(hmd->relation_hist, &identity, 0);

	// xrt_device metadata
	hmd->base.name        = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	hmd->base.supported.orientation_tracking = true;
	hmd->base.supported.position_tracking    = true;

	snprintf(hmd->base.str,    XRT_DEVICE_NAME_LEN, "Apple Vision Pro (ALVR)");
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "ALVR-AVP-001");

	// HMD display config (Vision Pro per-eye)
	uint32_t w = AVP_EYE_WIDTH, h = AVP_EYE_HEIGHT;
	hmd->base.hmd->screens[0].w_pixels = w * 2;
	hmd->base.hmd->screens[0].h_pixels = h;
	hmd->base.hmd->screens[0].nominal_frame_interval_ns =
	    (uint64_t)(1000000000.0 / AVP_REFRESH_HZ);

	hmd->base.hmd->blend_modes[0]   = XRT_BLEND_MODE_OPAQUE;
	hmd->base.hmd->blend_mode_count = 1;

	for (int eye = 0; eye < 2; eye++) {
		hmd->base.hmd->views[eye].display.w_pixels = w;
		hmd->base.hmd->views[eye].display.h_pixels = h;
		hmd->base.hmd->views[eye].viewport.x_pixels = eye * w;
		hmd->base.hmd->views[eye].viewport.y_pixels = 0;
		hmd->base.hmd->views[eye].viewport.w_pixels = w;
		hmd->base.hmd->views[eye].viewport.h_pixels = h;
		hmd->base.hmd->views[eye].rot = u_device_rotation_ident;
		hmd->base.hmd->distortion.fov[eye] = hmd->view_fovs[eye];
	}

	u_distortion_mesh_set_none(&hmd->base);

	// Function pointers
	hmd->base.update_inputs       = alvr_hmd_update_inputs;
	hmd->base.get_tracked_pose    = alvr_hmd_get_tracked_pose;
	hmd->base.get_view_poses      = alvr_hmd_get_view_poses;
	hmd->base.get_visibility_mask = alvr_hmd_get_visibility_mask;
	hmd->base.destroy             = alvr_hmd_destroy;

	// Debug variable tracking
	u_var_add_root(hmd, "ALVR HMD (Vision Pro)", true);
	u_var_add_log_level(hmd, &hmd->log_level, "log_level");

	// Start event thread
	hmd->running = true;
	pthread_create(&hmd->event_thread, NULL, alvr_event_thread, hmd);

	U_LOG_I("ALVR HMD created — waiting for Vision Pro client on port 9943");
	return &hmd->base;
}
