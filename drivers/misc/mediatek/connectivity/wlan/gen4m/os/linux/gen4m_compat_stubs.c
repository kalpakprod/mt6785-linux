// SPDX-License-Identifier: GPL-2.0
/*
 * gen4m_compat_stubs.c - begonia (MT6785) gen4m forward-port link stubs.
 *
 * The vendor gen4m driver declares (in gl_os.h / gl_rst.h) a handful of
 * symbols as `extern` that are *defined in other vendor connectivity modules*
 * (the connsys/WMT integration glue, MET tracing, and the gps/md coex cdevs)
 * which are not part of this forward-port. gen4m *calls* them; without a
 * provider modpost fails with "undefined!".
 *
 * These are bring-up-time link stubs so the module loads. They are inert
 * no-ops: none touch a connectivity register, an EMI seed, or a PMIC/clk
 * rail. The real wiring (e.g. routing wifi_reset_start() into the Stage-1
 * wmt_drv reset path) is a later integration step, intentionally out of scope
 * for the Stage-2 "gen4m compiles + links" milestone.
 *
 * Each is __weak: if a real provider is linked in later, it overrides these.
 */

#include "gl_os.h"

/* connsys -> gen4m: request a WiFi subsystem reset. The vendor connsys driver
 * calls these; the bodies live in gen4m's gl_rst.c reset state machine only
 * when CFG_WMT_RESET_API_SUPPORT pulls them in. Stub returns "reset complete"
 * so callers proceed; no actual reset is performed here. */
int __weak wifi_reset_start(void)
{
	return 0;
}

int __weak wifi_reset_end(enum ENUM_RESET_STATUS status)
{
	(void)status;
	return 0;
}

/* connsys/cdev -> gen4m: low-latency-mode query. Default: not in LL mode. */
uint32_t __weak get_low_latency_mode(void)
{
	return 0;
}

/* wmtWifi cdev <-> gen4m registration glue (provided by the vendor wmt char
 * device in production). No-op: the begonia bring-up path enables WiFi via the
 * Stage-1 wmt_drv / /dev/wmtWifi, not these gen4m-registered callbacks. */
void __weak update_driver_loaded_status(uint8_t loaded)
{
	(void)loaded;
}

void __weak register_set_p2p_mode_handler(set_p2p_mode handler)
{
	(void)handler;
}

void __weak register_file_buf_handler(file_buf_handler handler,
				      void *ctx, uint8_t ucType)
{
	(void)handler;
	(void)ctx;
	(void)ucType;
}

/* connectivity_export_show_stack(): a debug helper that dumps a task's kernel
 * stack; its body lives in the Stage-1 connectivity_build_in_adapter, which is
 * not linked into this module. Stub to a no-op (debug-only path). */
void __weak connectivity_export_show_stack(struct task_struct *tsk,
					   unsigned long *sp)
{
	(void)tsk;
	(void)sp;
}
