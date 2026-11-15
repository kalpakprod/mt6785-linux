# SPDX-License-Identifier: GPL-2.0
#
# MediaTek SCP sensor hub: the BSP sensor stack of the MT6885 mainline port
# (scratch/native/4-modem/src/cezanne-linux-port/linux/drivers/scp/sensor),
# which is MediaTek's sensors-1.0 stack, adapted to the MT6785 CM4 firmware.
#
# One module on purpose: the four BSP modules have a strict load order
# (hf-manager before mtk-nanohub before scp-iio) and splitting them again only
# buys that order a modules-load.d file.
#
obj-$(CONFIG_MTK_SCP_SENSORS) += mtk-scp-sensors.o
mtk-scp-sensors-y := hub.o hf_manager.o main.o bl.o comms.o nanohub-mtk.o
mtk-scp-sensors-y += mtk_nanohub.o mtk_nanohub_ipi.o sensor_list.o scp_iio.o

ccflags-y += -I$(src)/include -include $(src)/compat_sensor.h
ccflags-y += -DCONFIG_NANOHUB_MTK_IPI
#
# The hub's packets are unions of differently typed views of the same payload and
# the BSP declares the views it does not size itself as zero-length arrays; every
# index into one of those is reported as out of bounds.  The shapes are the
# firmware's (guarded by the BUILD_BUG_ONs on both packet sizes), so the warning
# is switched off here rather than reshaping the protocol structures.
ccflags-y += -Wno-array-bounds
