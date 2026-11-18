/* SPDX-License-Identifier: GPL-2.0 */
/* compat stub: upmu_common.h (begonia fwport). PMIC flag tokens only; the
 * KERNEL_pmic_* wrappers are build-time no-ops (see mtk-bsp-compat.h).
 * No PMIC register is read or written. */
#ifndef _COMPAT_STUB_UPMU_COMMON_H
#define _COMPAT_STUB_UPMU_COMMON_H
enum {
	PMIC_RG_LDO_VCN18_LP = 0,
	PMIC_RG_LDO_VCN33_2_LP,
};
#endif
