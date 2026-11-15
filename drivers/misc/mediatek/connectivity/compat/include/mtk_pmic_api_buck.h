/* SPDX-License-Identifier: GPL-2.0 */
/* compat stub: mtk_pmic_api_buck.h (begonia fwport). Faithful copy of the
 * vendor BUCK_LDO_EN_USER selector enum + HW_/SW_ mode tokens (selectors, not
 * register values). Consumed only as args to the build-time-no-op
 * KERNEL_pmic_ldo_*_lp() wrappers; no PMIC hardware is touched. */
#ifndef _COMPAT_STUB_MTK_PMIC_API_BUCK_H
#define _COMPAT_STUB_MTK_PMIC_API_BUCK_H
enum BUCK_LDO_EN_USER {
	SRCLKEN0,
	SRCLKEN1, SRCLKEN2, SRCLKEN3, SRCLKEN4, SRCLKEN5, SRCLKEN6, SRCLKEN7,
	SRCLKEN8, SRCLKEN9, SRCLKEN10, SRCLKEN11, SRCLKEN12, SRCLKEN13, SRCLKEN14,
	SW,
	SPM = SW,
};
#define HW_OFF  0
#define HW_LP   1
#define HW_ULP  2
#define SW_OFF  0
#define SW_ON   1
#define SW_LP   3
#endif
