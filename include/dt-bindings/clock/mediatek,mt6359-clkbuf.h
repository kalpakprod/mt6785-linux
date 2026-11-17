/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Clock buffer (XO) outputs of the MediaTek MT6359 PMIC.
 *
 * The list is the vendor bridge's xo_id minus the two buffers the vendor marks
 * disabled (AUD, PD): see mtk-clkbuf-bridge.h in the vendor 4.19 tree, and
 * pmic/upmu_hw.h for the registers.
 */
#ifndef _DT_BINDINGS_CLOCK_MEDIATEK_MT6359_CLKBUF_H
#define _DT_BINDINGS_CLOCK_MEDIATEK_MT6359_CLKBUF_H

#define MT6359_CLKBUF_XO_SOC	0
#define MT6359_CLKBUF_XO_WCN	1
#define MT6359_CLKBUF_XO_NFC	2
#define MT6359_CLKBUF_XO_CEL	3
#define MT6359_CLKBUF_XO_EXT	4

#define MT6359_CLKBUF_XO_NR	5

#endif /* _DT_BINDINGS_CLOCK_MEDIATEK_MT6359_CLKBUF_H */
