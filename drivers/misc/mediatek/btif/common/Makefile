# SPDX-License-Identifier: GPL-2.0
#
# Vendor MediaTek BTIF forward-port for begonia (MT6785).
# Module named mtk-vendor-btif so it does not collide with the reimpl
# drivers/misc/mtk-btif.o (CONFIG_MTK_BTIF).
#
ccflags-y += -I$(src)/inc
ccflags-y += -I$(src)/plat_inc

# Reuse the connectivity compat shim for the external BSP headers the
# vendor BTIF pulls in (mach/*, mtk_lpae.h, sync_write.h, etc.).
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/compat/include
ccflags-y += -include $(srctree)/drivers/misc/mediatek/connectivity/compat/include/mtk-bsp-compat.h

obj-$(CONFIG_MTK_WMT_FWPORT_BTIF) += mtk-vendor-btif.o
mtk-vendor-btif-objs := mtk_btif.o mtk_btif_exp.o btif_dma_plat.o btif_plat.o
