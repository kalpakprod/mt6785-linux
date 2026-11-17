# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the GNSS subsystem.
#

obj-$(CONFIG_GNSS)			+= gnss.o
gnss-y := core.o

obj-$(CONFIG_GNSS_SERIAL)		+= gnss-serial.o
gnss-serial-y := serial.o

obj-$(CONFIG_GNSS_MTK_SERIAL)		+= gnss-mtk.o
gnss-mtk-y := mtk.o

obj-$(CONFIG_GNSS_SIRF_SERIAL)		+= gnss-sirf.o
gnss-sirf-y := sirf.o

obj-$(CONFIG_GNSS_UBX_SERIAL)		+= gnss-ubx.o
gnss-ubx-y := ubx.o

obj-$(CONFIG_GNSS_USB)			+= gnss-usb.o
gnss-usb-y := usb.o

# The GNSS bridge talks to the WMT/STP stack, whose public headers live in the
# connectivity driver; use the same defines the WMT core is built with.
CFLAGS_stpgps.o += -DMTK_WCN_WMT_STP_EXP_SYMBOL_ABSTRACT
CFLAGS_stpgps.o += -DWMT_IDC_SUPPORT=0
CFLAGS_stpgps.o += -DWMT_PLAT_ALPS=1
CFLAGS_stpgps.o += -I$(srctree)/drivers/misc/mediatek/connectivity/common/common_main/include
CFLAGS_stpgps.o += -I$(srctree)/drivers/misc/mediatek/connectivity/common/common_main/linux/include
CFLAGS_stpgps.o += -I$(srctree)/drivers/misc/mediatek/connectivity/compat/include
CFLAGS_stpgps.o += -include $(srctree)/drivers/misc/mediatek/connectivity/compat/include/mtk-bsp-compat.h

obj-$(CONFIG_MTK_STPGPS)		+= stpgps.o
