# SPDX-License-Identifier: GPL-2.0
#
# Vendor WMT/connsys (wmt_drv) forward-port for begonia (MT6785).
# All BSP -I paths from the vendor Makefile are dropped; replaced with the
# fork-internal include dirs + the compat shim dir, and a force-included
# mtk-bsp-compat.h that backs the ~dozen external BSP symbols.
#
# The warning silencing lives one level up, in drivers/misc/mediatek/Makefile
# (subdir-ccflags-y), so that it covers this directory and btif/ alike.
#
obj-$(CONFIG_MTK_WMT_DRV) += common/
obj-$(CONFIG_MTK_WLAN_GEN4M) += wlan/gen4m/
