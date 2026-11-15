/* SPDX-License-Identifier: GPL-2.0 */
/*
 * compat topo_ctrl.h - stub for the begonia gen4m forward-port.
 *
 * The vendor plat_priv.c queries the CPU cluster count to size its per-cluster
 * frequency-floor request (a throughput optimisation - see cpu_ctrl.h). The
 * MTK BSP performance/topology service is absent on mainline. Returning the
 * MT6785 cluster count keeps the (no-op) freq-set loop well-formed. No
 * hardware is touched. Reached via the compat/include -I path.
 */
#ifndef _COMPAT_TOPO_CTRL_H
#define _COMPAT_TOPO_CTRL_H

/* MT6785 (Helio G90T): 2 clusters (2x A76 + 6x A55). The value only bounds the
 * loop in kalSetCpuFreq(); the result is discarded by the cpu_ctrl stub. */
static inline unsigned int topo_ctrl_get_nr_clusters(void)
{
	return 2;
}

#endif /* _COMPAT_TOPO_CTRL_H */
