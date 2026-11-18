/* SPDX-License-Identifier: GPL-2.0 */
/* compat stub: mtk_clkbuf_ctl.h (begonia fwport). Selector tokens only; the
 * KERNEL_clk_buf_ctrl() wrapper is a build-time no-op (see mtk-bsp-compat.h).
 * No clk-buf/XO hardware is touched. */
#ifndef _COMPAT_STUB_MTK_CLKBUF_CTL_H
#define _COMPAT_STUB_MTK_CLKBUF_CTL_H
enum clk_buf_id {
	CLK_BUF_CONN = 0,
};
#endif
