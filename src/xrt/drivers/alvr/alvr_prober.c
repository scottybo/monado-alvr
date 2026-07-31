// Copyright 2024, DCS Digital / Scott Bowler
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  ALVR auto-prober — discovers the virtual HMD unconditionally.
 * @ingroup drv_alvr
 */

#include "xrt/xrt_prober.h"
#include "util/u_misc.h"
#include "alvr_interface.h"

/*!
 * @implements xrt_auto_prober
 */
struct alvr_auto_prober {
	struct xrt_auto_prober base;
};

static inline struct alvr_auto_prober *
alvr_auto_prober(struct xrt_auto_prober *xap)
{
	return (struct alvr_auto_prober *)xap;
}

static void
alvr_auto_prober_destroy(struct xrt_auto_prober *p)
{
	free(alvr_auto_prober(p));
}

static int
alvr_auto_prober_autoprobe(struct xrt_auto_prober *xap,
                            cJSON *attached_data,
                            bool no_hmds,
                            struct xrt_prober *xp,
                            struct xrt_device **out_xdevs)
{
	(void)alvr_auto_prober(xap);
	if (no_hmds)
		return 0;

	out_xdevs[0] = alvr_hmd_create();
	return out_xdevs[0] ? 1 : 0;
}

struct xrt_auto_prober *
alvr_create_auto_prober(void)
{
	struct alvr_auto_prober *ap = U_TYPED_CALLOC(struct alvr_auto_prober);
	ap->base.name                    = "ALVR HMD";
	ap->base.destroy                 = alvr_auto_prober_destroy;
	ap->base.lelo_dallas_autoprobe   = alvr_auto_prober_autoprobe;
	return &ap->base;
}
