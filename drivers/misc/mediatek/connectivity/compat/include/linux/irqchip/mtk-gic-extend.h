/* SPDX-License-Identifier: GPL-2.0 */
/* compat stub: linux/irqchip/mtk-gic-extend.h (begonia fwport).
 * Vendor BTIF includes this only for mt_irq_dump_status(); no mainline
 * equivalent. Provide a no-op so the IRQ-dump debug path compiles. */
#ifndef _COMPAT_STUB_MTK_GIC_EXTEND_H
#define _COMPAT_STUB_MTK_GIC_EXTEND_H
static inline void mt_irq_dump_status(int irq) { (void)irq; }
#endif
