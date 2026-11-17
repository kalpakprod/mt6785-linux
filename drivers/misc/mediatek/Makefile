# SPDX-License-Identifier: GPL-2.0
#
# MediaTek vendor connectivity forward-port (begonia MT6785), Stage-1.
#
# The vendor code is a 4.14 BSP tree and carries ~200 warnings of its own kind
# (-Wmissing-prototypes on internal helpers, -Wundef on the BSP
# BTIF_DBG_SUPPORT switch, counters that are set but only read in vendor debug
# paths). The card allows silencing them above 50 findings, and the
# 20261115-084416 build reported 197. subdir-ccflags-y reaches this directory
# and everything below it; it does not touch any in-tree driver.
#
subdir-ccflags-y += -w

obj-$(CONFIG_MTK_WMT_FWPORT) += connectivity/
obj-$(CONFIG_MTK_WMT_FWPORT_BTIF) += btif/
