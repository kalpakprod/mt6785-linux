# SPDX-License-Identifier: GPL-2.0
#
# gen4m fullmac WLAN driver (wlan0), vendor forward-port for begonia (MT6785).
# Stage-2 of the begonia vendor forward-port. Sits on top of the Stage-1
# WMT/connsys (wmt_drv) stack. Scoped to the begonia config:
#   MTK_COMBO_CHIP=CONNAC, HIF=axi, WLAN_CHIP_ID=6785, WIFI_IP_SET=2,
#   WIFI_ECO_VER=1, MTK_ANDROID_WMT=y, MTK_ANDROID_EMI=y.
#
# All BSP -I paths from the vendor gen4m Makefile are intentionally dropped;
# the external BSP symbol surface is backed by the force-included compat shim
# (compat/include/mtk-bsp-compat.h, shared with Stage-1, extended for gen4m).
#
# RULE: no register address / value / bitfield changed in any .c to build;
# type / signature / header / API churn only.

MODULE_NAME := wlan_gen4m

# ---------------------------------------------------------------------------
# Chip-select macros (begonia subset: CONNAC AXI 6785). Mirrors the vendor
# gen4m Makefile's WLAN_CHIP_LIST -U.../-D... resolution for MTK_COMBO_CHIP=CONNAC.
# ---------------------------------------------------------------------------
ccflags-y += -UMT6620 -UMT6628 -UMT5931 -UMT6630 -UMT6632 -UMT7663
ccflags-y += -UCONNAC2X2 -UUT_TEST_MODE -UMT7915 -USOC3_0 -UMT7961
ccflags-y += -DCONNAC
ccflags-y += -DLINUX

# config-var derived defines (begonia)
ccflags-y += -DCFG_SUPPORT_DEBUG_FS=0
ccflags-y += -DWLAN_INCLUDE_PROC
ccflags-y += -DCFG_SUPPORT_AGPS_ASSIST=0
ccflags-y += -DCFG_SUPPORT_TSF_USING_BOOTTIME=1
ccflags-y += -DARP_MONITER_ENABLE=1
ccflags-y += -DCFG_DRIVER_INITIAL_RUNNING_MODE=3
ccflags-y += -DDBG=0

# CONNAC1 => CONNAC2X off, 11AX off, TWT off
ccflags-y += -DCFG_SUPPORT_CONNAC2X=0
ccflags-y += -DCFG_SUPPORT_CONNAC2X_2x2=0
ccflags-y += -DCFG_SUPPORT_802_11AX=0
ccflags-y += -DCFG_SUPPORT_TWT=0

# conninfra is the connac2/soc3_0 path; CONNAC1 begonia uses the WMT stack.
ccflags-y += -DCFG_SUPPORT_CONNINFRA=0
ccflags-y += -DCFG_ANDORID_CONNINFRA_SUPPORT=0
ccflags-y += -DCFG_ANDORID_CONNINFRA_COREDUMP_SUPPORT=0
ccflags-y += -DCFG_SUPPORT_PRE_ON_PHY_ACTION=0

# WMT + EMI (begonia: MTK_ANDROID_WMT=y, MTK_ANDROID_EMI=y)
ccflags-y += -DCFG_MTK_ANDROID_WMT=1
ccflags-y += -DCFG_MTK_ANDROID_EMI=1
ccflags-y += -DWMT_IDC_SUPPORT=0
ccflags-y += -DMTK_WCN_WMT_STP_EXP_SYMBOL_ABSTRACT

# WIFI_IP_SET=2 (begonia)
ccflags-y += -DCFG_WIFI_IP_SET=2

# misc vendor feature gates (off)
ccflags-y += -DCFG_BUILT_IN_DRIVER=0
ccflags-y += -DCONFIG_WLAN_DRV_BUILD_IN=0
ccflags-y += -DCFG_ENABLE_HOST_BUS_TIMEOUT=0
ccflags-y += -DCFG_SUPPORT_DUAL_STA=1
ccflags-y += -DCFG_SUPPORT_CHNL_CONFLICT_REVISE=0
ccflags-y += -DCFG_SUPPORT_AEE=0
ccflags-y += -DBUILD_QA_DBG=0
ccflags-y += -DCFG_SUPPORT_PASSPOINT=0
ccflags-y += -DCFG_HS20_DEBUG=0
ccflags-y += -DCFG_ENABLE_GTK_FRAME_FILTER=0
ccflags-y += -DCFG_MET_PACKET_TRACE_SUPPORT=0
ccflags-y += -DCFG_MET_TAG_SUPPORT=0
ccflags-y += -DCFG_TC3_FEATURE=0
ccflags-y += -DCFG_TC10_FEATURE=0
ccflags-y += -DCFG_TC1_FEATURE=0
ccflags-y += -DCFG_MTK_MCIF_WIFI_SUPPORT=0
ccflags-y += -DCFG_MTK_MDDP_WH_SUPPORT=0
ccflags-y += -DCONFIG_WLAN_SERVICE=0
ccflags-y += -DCONFIG_TEST_ENGINE_OFFLOAD=0
ccflags-y += -DCONFIG_NUM_OF_WFDMA_TX_RING=1
ccflags-y += -DCONFIG_NUM_OF_WFDMA_RX_RING=1

# AXI HIF
ccflags-y += -D_HIF_AXI=1

# silence vendor warnings (it builds with -Werror upstream; we drop -Werror
# during the port and keep the warning suppressions it relied on)
ccflags-y += -Wno-unused-value -Wno-unused-result -Wno-format -Wno-parentheses
ccflags-y += -Wno-unused-function -Wno-unused-variable

# ---------------------------------------------------------------------------
# Internal include dirs (replace ALL vendor BSP -I lines)
# ---------------------------------------------------------------------------
ccflags-y += -I$(src)/include -I$(src)/include/nic -I$(src)/include/mgmt -I$(src)/include/chips
ccflags-y += -I$(src)/os -I$(src)/os/linux/include
ccflags-y += -I$(src)/os/linux/hif/common/include
ccflags-y += -I$(src)/os/linux/hif/axi/include

# net/ : gen4m reaches cfg80211's private wireless/core.h (scan-cache path).
# In-tree path, not a BSP path.
ccflags-y += -I$(srctree)/net

# WMT bridge: pull the Stage-1 wmt_drv export headers (wmt_exp.h, stp_exp.h ...)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/common/common_main/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/common/common_main/linux/include

# Compat shim: shared with Stage-1, force-included into every gen4m TU
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/compat/include
ccflags-y += -include $(srctree)/drivers/misc/mediatek/connectivity/compat/include/mtk-bsp-compat.h

# ---------------------------------------------------------------------------
# Object list (begonia CONNAC/AXI/6785 subset). 93 TUs.
# ---------------------------------------------------------------------------
obj-$(CONFIG_MTK_WLAN_GEN4M) += $(MODULE_NAME).o

# common/
$(MODULE_NAME)-objs += common/dump.o common/wlan_lib.o common/wlan_oid.o \
	common/wlan_bow.o common/debug.o common/wlan_p2p.o common/wlan_he.o

# nic/
$(MODULE_NAME)-objs += nic/nic.o nic/nic_tx.o nic/nic_txd_v1.o nic/nic_rxd_v1.o \
	nic/nic_rx.o nic/nic_pwr_mgt.o nic/nic_rate.o nic/cmd_buf.o nic/que_mgt.o \
	nic/nic_cmd_event.o nic/nic_umac.o nic/p2p_nic.o

# os/linux/
$(MODULE_NAME)-objs += os/linux/gl_init.o os/linux/gl_kal.o os/linux/gl_bow.o \
	os/linux/gl_wext.o os/linux/gl_wext_priv.o os/linux/gl_ate_agent.o \
	os/linux/gl_qa_agent.o os/linux/gl_hook_api.o os/linux/gl_rst.o \
	os/linux/gl_cfg80211.o os/linux/gl_proc.o os/linux/gl_sys.o \
	os/linux/gl_vendor.o os/linux/platform.o \
	os/linux/gl_p2p.o os/linux/gl_p2p_cfg80211.o os/linux/gl_p2p_init.o \
	os/linux/gl_p2p_kal.o

# mgmt/
$(MODULE_NAME)-objs += mgmt/ais_fsm.o mgmt/aaa_fsm.o mgmt/assoc.o mgmt/auth.o \
	mgmt/bss.o mgmt/cnm.o mgmt/cnm_timer.o mgmt/cnm_mem.o mgmt/hem_mbox.o \
	mgmt/mib.o mgmt/privacy.o mgmt/rate.o mgmt/rlm.o mgmt/rlm_domain.o \
	mgmt/reg_rule.o mgmt/rlm_obss.o mgmt/rlm_protection.o mgmt/rrm.o mgmt/rsn.o \
	mgmt/saa_fsm.o mgmt/scan.o mgmt/scan_fsm.o mgmt/scan_cache.o mgmt/swcr.o \
	mgmt/roaming_fsm.o mgmt/tkip_mic.o mgmt/hs20.o mgmt/tdls.o mgmt/wnm.o \
	mgmt/qosmap.o mgmt/ap_selection.o mgmt/wmm.o mgmt/mddp.o mgmt/stats.o \
	mgmt/p2p_dev_fsm.o mgmt/p2p_dev_state.o mgmt/p2p_role_fsm.o \
	mgmt/p2p_role_state.o mgmt/p2p_func.o mgmt/p2p_scan.o mgmt/p2p_ie.o \
	mgmt/p2p_rlm.o mgmt/p2p_assoc.o mgmt/p2p_bss.o mgmt/p2p_rlm_obss.o \
	mgmt/p2p_fsm.o mgmt/wapi.o

# chips/ (CONNAC1 only)
$(MODULE_NAME)-objs += chips/common/cmm_asic_connac.o chips/common/dbg_connac.o \
	chips/common/fw_dl.o chips/connac/connac.o

# os/linux/hif/{common,axi}
$(MODULE_NAME)-objs += os/linux/hif/common/hal_pdma.o os/linux/hif/common/kal_pdma.o \
	os/linux/hif/common/dbg_pdma.o os/linux/hif/axi/axi.o

# os/linux/plat/mt6785
$(MODULE_NAME)-objs += os/linux/plat/mt6785/plat_priv.o

# forward-port link stubs for vendor cross-module bridge symbols
$(MODULE_NAME)-objs += os/linux/gen4m_compat_stubs.o
