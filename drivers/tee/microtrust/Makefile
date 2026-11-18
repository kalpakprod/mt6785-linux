# SPDX-License-Identifier: GPL-2.0
#
# Microtrust TEEI client driver (iSee 4.0 / SOTER) for the POCO M5s, carried over
# from the cezanne-linux-port tree (linux/drivers/fingerprint/teei/Kbuild), which
# itself came from Xiaomi's pissarro-s-oss kernel (drivers/misc/mediatek/teei/).
# This is forward-ported vendor code, not upstream-bound: it is built with -w and
# listed in scratch/wt/fingerprint/vendor-dirs so checkpatch skips it.
#
# Feature switches, decided against this phone's own kernel config
# (poco-m5s/device-dump-2026-11-13/stock/proc-config.gz) and against the port's
# mainline fixes:
#   CONFIG_MICROTRUST_TZ_LOG      on here, off in stock: it forwards the
#                                 secure-world log ([TZ_LOG]) and is how the lead
#                                 sees the TEE boot on a new port.
#   CONFIG_MICROTRUST_FP_DRIVER   on, as in stock: without it nothing wakes the
#                                 /dev/teei_fp opener once the TEE is up
#                                 (teei_client_main.c:632) and the fp client lock
#                                 is not accounted for.
#   CONFIG_MICROTRUST_DYNAMIC_CORE stays OFF although stock builds it on: the
#                                 port replaced it with a mainline fix that pins
#                                 the switch thread to one core (kthread_bind() in
#                                 teei_client_main.c:1108-1113, "an SMC from
#                                 another core hangs the phone") and runs the CPU
#                                 hotplug handler instead. Turning it on would drop
#                                 that fix.
#   CONFIG_MICROTRUST_TEST_DRIVERS, CONFIG_MICROTRUST_TZDRIVER_DYNAMICAL_DEBUG and
#   CONFIG_MICROTRUST_TZ_DRIVER_MTK_TRACING stay unset, as in stock.
ccflags-y += -I$(src)/common/include -I$(src)/public -I$(src)/tz_dcih \
	-I$(src)/tz_driver/include -I$(src)/tz_driver -I$(src)/tee
ccflags-y += -DDEBUG -DCONFIG_MICROTRUST_TZ_LOG -DCONFIG_MICROTRUST_FP_DRIVER
ccflags-y += -include $(src)/cz_compat.h -DUT_DMA_ZONE
ccflags-y += -w
obj-$(CONFIG_MICROTRUST_TEEI) += microtrust_teei.o
microtrust_teei-y := teei_main.o \
	tz_driver/teei_client_main.o tz_driver/fdrv.o tz_driver/backward_driver.o \
	tz_driver/irq_register.o tz_driver/notify_queue.o tz_driver/teei_fp.o \
	tz_driver/teei_smc_call.o tz_driver/tz_log.o tz_driver/switch_queue.o \
	tz_driver/teei_keymaster.o tz_driver/teei_cancel_cmd.o \
	tz_driver/teei_client_transfer_data.o tz_driver/sysfs.o \
	tz_driver/teei_task_link.o \
	tee/tee_core.o tee/tee_shm.o tee/tee_shm_pool.o tee/tee_client_api.o \
	tee/soter/core.o tee/soter/call.o \
	teei_fp/fp_func.o tz_vfs/vfsFun.o tz_vfs/fp_vendor.o \
	tz_dcih/tz_dcih.o tz_dcih/tz_dcih_test.o ut_keymaster/ut_keymaster.o
