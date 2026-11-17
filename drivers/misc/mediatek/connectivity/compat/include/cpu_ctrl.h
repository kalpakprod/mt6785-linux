/* SPDX-License-Identifier: GPL-2.0 */
/*
 * compat cpu_ctrl.h - stub for the begonia gen4m forward-port.
 *
 * The vendor plat_priv.c (kalBoostCpu) raises the CPU frequency floor during
 * high WiFi throughput via the MTK BSP CPU-DVFS "userlimit" interface. That
 * interface is not present in mainline. Stubbing it is correctness-neutral:
 * it is a throughput optimisation only, touches no connectivity hardware, and
 * writes no device register. Reached via the compat/include -I path.
 */
#ifndef _COMPAT_CPU_CTRL_H
#define _COMPAT_CPU_CTRL_H

#include <linux/types.h>

/* DVFS kicker ids - only CPU_KIR_WIFI is referenced. */
enum ppm_kicker {
	CPU_KIR_WIFI = 0,
};

/* Per-cluster min/max frequency request. Layout matches the field names the
 * vendor uses (.min/.max); values are ignored by the stub. */
struct ppm_limit_data {
	int min;
	int max;
};

static inline int update_userlimit_cpu_freq(int kicker, int num_cluster,
					    struct ppm_limit_data *freq_limit)
{
	(void)kicker; (void)num_cluster; (void)freq_limit;
	return 0;	/* no-op: CPU DVFS floor not modelled on mainline */
}

#endif /* _COMPAT_CPU_CTRL_H */
