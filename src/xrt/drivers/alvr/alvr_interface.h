// Copyright 2024, DCS Digital / Scott Bowler
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  ALVR driver interface — Monado ↔ ALVR streaming bridge.
 * @ingroup drv_alvr
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

struct xrt_auto_prober;
struct xrt_device;
struct comp_compositor;
struct comp_target;

/*!
 * Create the ALVR auto-prober (discovers the virtual HMD).
 * @ingroup drv_alvr
 */
struct xrt_auto_prober *
alvr_create_auto_prober(void);

/*!
 * Create the ALVR HMD xrt_device.
 * Initialises the alvr_server_core library, starts the network connection,
 * and spawns the event-polling thread.
 * @ingroup drv_alvr
 */
struct xrt_device *
alvr_hmd_create(void);

/*!
 * Create the ALVR compositor target.
 * After Monado composites each frame this target captures the Vulkan image,
 * encodes it with NVENC, and ships it to the connected Vision Pro client.
 * @ingroup drv_alvr
 */
struct comp_target *
alvr_target_create(struct comp_compositor *c);

/*! Pre-initialised factory — add to ctfs[] in comp_compositor.c */
extern const struct comp_target_factory comp_target_factory_alvr;

/*!
 * @dir drivers/alvr
 * @brief @ref drv_alvr files.
 */

#ifdef __cplusplus
}
#endif
