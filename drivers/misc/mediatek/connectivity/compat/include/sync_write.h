/* SPDX-License-Identifier: GPL-2.0 */
/* compat: sync_write.h (begonia fwport). Faithful copy of the vendor MMIO
 * sync-write macros (raw write + full barrier). These are write *primitives*,
 * not register values. */
#ifndef _COMPAT_SYNC_WRITE_H
#define _COMPAT_SYNC_WRITE_H
#include <linux/io.h>
#include <asm/barrier.h>

#define mt_reg_sync_writel(v, a) 	do { __raw_writel((v), (void __force __iomem *)((a))); mb(); } while (0)
#define mt_reg_sync_writew(v, a) 	do { __raw_writew((v), (void __force __iomem *)((a))); mb(); } while (0)
#define mt_reg_sync_writeb(v, a) 	do { __raw_writeb((v), (void __force __iomem *)((a))); mb(); } while (0)
#ifdef CONFIG_64BIT
#define mt_reg_sync_writeq(v, a) 	do { __raw_writeq((v), (void __force __iomem *)((a))); mb(); } while (0)
#endif
#endif
