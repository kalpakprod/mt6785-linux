/* SPDX-License-Identifier: GPL-2.0 */
/* compat stub for BSP header mt_emi_api.h (begonia fwport).
 *
 * FLAGGED: there is NO mainline EMI-MPU API in 6.16. The vendor BSP
 * mt_emi_api.h provided emi_mpu_set_protection(struct emi_region_info_t *) plus
 * the emi_region_info_t type and the SET_ACCESS_PERMISSION() macro, backed by
 * the MTK EMI driver gated on CONFIG_MTK_EMI. Mainline drivers/memory/mediatek/
 * ships only mtk-smi.o (no mtk-emi.c, no mtk_emimpu_* symbol), and CONFIG_MTK_EMI
 * is undefined, so consys_emi_mpu_set_region_protection() (mt6785.c) cannot set
 * the conn EMI carveout access-protection table.
 *
 * This is an access-protection feature, NOT the conn firmware-fetch mechanism
 * (which is the conn 0xF0000000 EMI-window remapping to the carveout phys base -
 * see consys_emi_set_remapping_reg()). The proven reimpl mtk-conninfra-mt6785.c
 * reaches conn idle 0x1d1e with NO EMI-MPU call, so the missing MPU is not a
 * bring-up gate. Left as an empty stub and explicitly flagged in
 * consys_emi_mpu_set_region_protection(); wire a real API here if one ever lands
 * in mainline. */
#ifndef _COMPAT_STUB_MT_EMI_API_H
#define _COMPAT_STUB_MT_EMI_API_H
#endif
