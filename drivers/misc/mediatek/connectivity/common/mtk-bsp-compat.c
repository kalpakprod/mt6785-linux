// SPDX-License-Identifier: GPL-2.0
/*
 * mtk-bsp-compat.c - connadp build-in-adapter bridge for wmt_drv (begonia fwport).
 *
 * Single remaining role after the full vendor forward-port: provide no-op bodies
 * for the connectivity_build_in_adapter / wmt_build_in_adapter export wrappers
 * that wmt_drv references via the unconditional KERNEL_* macros and the platform
 * bridge. The real bodies live in the separate connadp module
 * (connectivity_build_in_adapter.c / wmt_build_in_adapter.c), which pulls a large
 * BSP surface (private scheduler / ppm / spm headers) and is NOT built into this
 * standalone wmt_drv. These no-ops let wmt_drv link without that BSP surface.
 *
 * All earlier weak stubs for *pruned* TUs (SDIO/UART transports, hif_sdio, the
 * combo IC ops, wmt_detect_get_chip_type) and their data symbols (gpio_ctrl_info,
 * g_stp_sdio_host_info, ...) were REMOVED: those TUs are now compiled into
 * wmt_drv with their real vendor logic and provide the strong definitions.
 *
 * RULE: no register address / value / bitfield. These calls are no-op at the API
 * level - no hardware is touched here. These are the genuinely-unportable connadp
 * BSP bridge points (PMIC/MT6306/sysboost/ppm/spm), off the begonia bring-up
 * critical path; flagged in the port report.
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/mmc/card.h>
#include <linux/pm.h>

#include "connectivity_build_in_adapter.h"
#include "wmt_build_in_adapter.h"

/* ---- connectivity_build_in_adapter export wrappers (connadp no-op) ------- */
/* NOTE: the begonia conn bring-up path no longer routes A-die PMIC ops through
 * these wrappers. consys_hw_vcn18_ctrl / consys_hw_vcn28_ctrl (mt6785.c) now do
 * the REAL mt6359 regmap ops directly (wmt_pmic_ldo_lp / wmt_pmic_set_ldo_lp via
 * g_regmap). These two bodies are therefore dead on the begonia path and kept
 * only so the unconditional KERNEL_pmic_*_register_value adapter macro still
 * links for any non-begonia caller. They touch no hardware. */
void connectivity_export_pmic_set_register_value(int flagname, unsigned int val)
{ (void)flagname; (void)val; }

unsigned short connectivity_export_pmic_get_register_value(int flagname)
{ (void)flagname; return 0; }

void connectivity_export_mt6306_set_gpio_out(unsigned long pin, unsigned long output)
{ (void)pin; (void)output; }
void connectivity_export_mt6306_set_gpio_out(unsigned long pin, unsigned long output);

void connectivity_export_mt6306_set_gpio_dir(unsigned long pin, unsigned long dir)
{ (void)pin; (void)dir; }
void connectivity_export_mt6306_set_gpio_dir(unsigned long pin, unsigned long dir);

void connectivity_export_show_stack(struct task_struct *tsk, unsigned long *sp)
{ (void)tsk; (void)sp; }

void connectivity_export_dump_thread_state(const char *name)
{ (void)name; }

void connectivity_export_dump_gpio_info(int start, int end)
{ (void)start; (void)end; }

/* SDIO CMD52 bridge (KERNEL_mmc_io_rw_direct): the real body is in connadp,
 * which owns the mmc core glue. The restored SDIO transport references it; no-op
 * (returns -ENODEV) here since begonia uses the BTIF transport, not SDIO. */
int connectivity_export_mmc_io_rw_direct(struct mmc_card *card, int write,
					 unsigned int fn, unsigned int addr,
					 u8 in, u8 *out)
{ (void)card; (void)write; (void)fn; (void)addr; (void)in; if (out) *out = 0; return -ENODEV; }

/* ---- wmt_build_in_adapter platform-bridge (connadp no-op) ---------------- *
 * stub_alps.c / hif_sdio.c register/unregister their callback tables through
 * these; the real connadp owns the platform bridge. No-op keeps the headless
 * bring-up self-sufficient. */
void wmt_export_platform_bridge_register(struct wmt_platform_bridge *cb) { (void)cb; }
void wmt_export_platform_bridge_unregister(void) { }
void wmt_export_mtk_wcn_cmb_sdio_disable_eirq(void) { }
int  wmt_export_mtk_wcn_sdio_irq_flag_set(int flag) { (void)flag; return 0; }

/* ---- connadp data symbols referenced by the restored SDIO/eint TUs -------- *
 * These three live in wmt_build_in_adapter.c (the connadp TU, not built into
 * wmt_drv). The restored mtk_wcn_stub_alps.c / hif_sdio.c reference them through
 * the wmt_build_in_adapter.h externs, so wmt_drv must provide the definitions.
 * Plain zero-initialised storage - the SDIO/eint paths they feed are not on the
 * begonia BTIF bring-up path. */
unsigned int wifi_irq = 0xffffffff;
pm_callback_t mtk_wcn_cmb_sdio_pm_cb;
void *mtk_wcn_cmb_sdio_pm_data;

/* wmt_drv module metadata: the vendor placed MODULE_LICENSE in the SDIO transport
 * TUs; keep it here as the module's single license/description record. */
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek WMT/connsys (wmt_drv) vendor forward-port for begonia MT6785");

/*
 * Load order, the way Android's init.rc does it by hand.
 *
 * wmt_drv owns the STP transport and the bring-up sequence; every other module
 * of this stack talks to it through exported symbols. wlan_gen4m must be
 * present before the Wi-Fi function is switched on, because its probe is what
 * registers the Wi-Fi callbacks with WMT (mtk_wcn_wmt_wlan_reg() in hifAxiProbe)
 * and without them FUNC_CTRL(WIFI on) has no probe to run. The softdep makes
 * modprobe bring it in with wmt_drv, so the order is right by construction.
 *
 * btmtkbtif and stpgps do not need this: both bind to their own DT nodes and
 * ask WMT for their channel inside open(), long after wmt_drv is up.
 */
MODULE_SOFTDEP("pre: wlan_gen4m");
