# SPDX-License-Identifier: GPL-2.0
#
# wmt_drv: vendor WMT/STP/connsys core, forward-ported to mainline for begonia (MT6785).
# Stage-1 of the begonia vendor forward-port. BSP -I paths from the vendor
# Makefile are intentionally dropped; the external BSP symbol surface is
# backed by the force-included compat shim (compat/include/mtk-bsp-compat.h).

ccflags-y += -DMT6785
ccflags-y += -DCONFIG_MTK_WCN_ARM64
ccflags-y += -DMTK_WCN_SOC_CHIP_SUPPORT
ccflags-y += -DMTK_WCN_WLAN_GEN4
ccflags-y += -DWMT_PLAT_ALPS=1
ccflags-y += -DWMT_IDC_SUPPORT=0
ccflags-y += -DWMT_UART_RX_MODE_WORK=0
ccflags-y += -DWMT_SDIO_MODE=1
ccflags-y += -DWMT_CREATE_NODE_DYNAMIC=1
ccflags-y += -DWMT_DBG_SUPPORT=0
ccflags-y += -DWMT_DEVAPC_DBG_SUPPORT=0
ccflags-y += -DCONSYS_PMIC_CTRL_6635=1
ccflags-y += -DMTK_WCN_WMT_STP_EXP_SYMBOL_ABSTRACT
ccflags-y += -DCONSYS_BUILD_IN

# Internal include dirs (replace all vendor BSP -I lines)
ccflags-y += -I$(src)/common_main/include
ccflags-y += -I$(src)/common_main/core/include
ccflags-y += -I$(src)/common_main/linux/include
ccflags-y += -I$(src)/common_main/platform/include
ccflags-y += -I$(src)/common_detect
ccflags-y += -I$(src)/common_detect/drv_init/inc
ccflags-y += -I$(src)/debug_utility
ccflags-y += -I$(src)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/btif/common/inc

# Compat shim: header dir + force-include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/compat/include
ccflags-y += -include $(srctree)/drivers/misc/mediatek/connectivity/compat/include/mtk-bsp-compat.h

obj-$(CONFIG_MTK_WMT_DRV) += wmt_drv.o

# compat: connadp build-in-adapter export wrappers (the connadp BSP bridge).
# All pruned-TU weak stubs were removed once the real TUs were restored below.
wmt_drv-objs += mtk-bsp-compat.o

# ---------------------------------------------------------------------------
# common_detect: real chip-detect + power + gpio + per-subsystem drv_init.
# wmt_detect.c owns the single module_init (creates /dev/wmtdetect and, headless,
# issues the DO_MODULE_INIT bring-up in-kernel). mtk_wcn_stub_alps.c provides the
# cmb_stub reg/unreg/aif_ctrl surface.
# ---------------------------------------------------------------------------
wmt_drv-objs += common_detect/wmt_detect_pwr.o
wmt_drv-objs += common_detect/wmt_detect.o
wmt_drv-objs += common_detect/sdio_detect.o
wmt_drv-objs += common_detect/mtk_wcn_stub_alps.o
wmt_drv-objs += common_detect/wmt_gpio.o
wmt_drv-objs += common_detect/drv_init/fm_drv_init.o
wmt_drv-objs += common_detect/drv_init/conn_drv_init.o
wmt_drv-objs += common_detect/drv_init/bluetooth_drv_init.o
wmt_drv-objs += common_detect/drv_init/wlan_drv_init.o
wmt_drv-objs += common_detect/drv_init/common_drv_init.o
wmt_drv-objs += common_detect/drv_init/gps_drv_init.o

# platform
wmt_drv-objs += common_main/platform/mt6785.o
wmt_drv-objs += common_main/platform/wmt_plat_alps.o
wmt_drv-objs += common_main/platform/mtk_wcn_consys_hw.o
wmt_drv-objs += common_main/platform/mtk_wcn_cmb_hw.o

# core
wmt_drv-objs += common_main/core/wmt_conf.o
wmt_drv-objs += common_main/core/stp_core.o
wmt_drv-objs += common_main/core/wmt_ctrl.o
wmt_drv-objs += common_main/core/wmt_func.o
wmt_drv-objs += common_main/core/wmt_core.o
wmt_drv-objs += common_main/core/psm_core.o
wmt_drv-objs += common_main/core/wmt_ic_soc.o
wmt_drv-objs += common_main/core/wmt_lib.o
wmt_drv-objs += common_main/core/stp_exp.o
wmt_drv-objs += common_main/core/wmt_exp.o
wmt_drv-objs += common_main/core/btm_core.o

# linux
# begonia: stp_uart.o / stp_sdio.o / hif_sdio.o removed — UART/SDIO STP transports
# are dead on this SoC (live transport is BTIF, if_tx=2). SoC chip-id comes via
# GEN_HCR + wmt_plat_get_soc_chipid(), not the SDIO query path.
wmt_drv-objs += common_main/linux/stp_dbg_soc.o
wmt_drv-objs += common_main/linux/stp_dbg_combo.o
wmt_drv-objs += common_main/linux/osal.o
wmt_drv-objs += common_main/linux/wmt_dev.o
wmt_drv-objs += common_main/linux/bgw_desense.o
wmt_drv-objs += common_main/linux/wmt_idc.o
wmt_drv-objs += common_main/linux/wmt_dbg.o
wmt_drv-objs += common_main/linux/stp_dbg.o
wmt_drv-objs += common_main/linux/wmt_user_proc.o
wmt_drv-objs += common_main/linux/wmt_proc_dbg.o
wmt_drv-objs += common_main/linux/wmt_alarm.o
wmt_drv-objs += common_main/linux/fw_log_wmt.o
wmt_drv-objs += common_main/linux/wmt_step.o
wmt_drv-objs += common_main/linux/stp_btif.o
# begonia fwport: /dev/wmtWifi WiFi on/off trigger (reconstructs the stock node)
wmt_drv-objs += common_main/linux/wmt_wifi_trigger.o

# debug_utility
wmt_drv-objs += debug_utility/ring.o
wmt_drv-objs += debug_utility/ring_emi.o
wmt_drv-objs += debug_utility/connsys_debug_utility.o
