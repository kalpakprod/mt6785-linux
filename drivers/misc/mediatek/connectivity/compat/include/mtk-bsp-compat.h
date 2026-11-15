/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mtk-bsp-compat.h - compat shim for the begonia (MT6785) vendor connectivity
 * forward-port. Force-included into every wmt_drv / vendor-btif TU.
 *
 * Backs the external MTK-BSP symbols the 4.14 vendor stack references that
 * have no mainline equivalent, as no-op stubs or mainline-framework shims.
 * Grown from compile errors, not predicted.
 *
 * RULE: no register address / value / bitfield is altered here. Type /
 * signature / symbol shims only.
 *
 * UPDATE (fwport-pmic): the conn A-die PMIC path is NO LONGER stubbed. The conn
 * 26MHz XO buffer (consys_clock_buffer_ctrl) and the VCN18/VCN33_2 A-die LDO
 * low-power setup (consys_hw_vcn18_ctrl / consys_hw_vcn28_ctrl) now do the REAL
 * mt6359 regmap ops directly in mt6785.c (via g_regmap). The KERNEL_clk_buf_ctrl
 * and KERNEL_pmic_ldo_vcn*_lp no-op macros below are consequently dead on the
 * begonia path; they remain only so any non-begonia TU that still references the
 * macro keeps building. A no-op here touches no hardware.
 *
 * Split of responsibility for the connectivity_build_in_adapter KERNEL_*
 * macros (the connadp module is NOT linked into wmt_drv here):
 *  - UNCONDITIONAL macros in connectivity_build_in_adapter.h (pmic_set_register
 *    _value, mt6306 gpio, show_stack, dump_thread_state, gpio_dump_regs_range)
 *    map to connectivity_export_*(); we provide no-op bodies in
 *    mtk-bsp-compat.c. Do NOT redefine those KERNEL_* macros here.
 *  - GATED-OFF macros (clk_buf*, pmic_ldo_vcn*_lp - gated on CONFIG_MACH_* /
 *    CONFIG_MTK_PMIC_CHIP_* which are unset in mainline) are not defined by the
 *    adapter header, so we define them directly as no-op inlines here.
 */
#ifndef _MTK_BSP_COMPAT_H
#define _MTK_BSP_COMPAT_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/io.h>

/* sched_clock() is declared in <linux/sched/clock.h>; the 4.14 vendor code
 * called it expecting an implicit prototype. Pull the header so gen4m's
 * debug/wlan_lib timestamp paths resolve it. */
#include <linux/sched/clock.h>

/* --- 4.14 -> 6.16 API churn --------------------------------------------- */
/* ioremap_nocache() was removed in 5.6; ioremap() is now always non-cached. */
#ifndef ioremap_nocache
#define ioremap_nocache(addr, size) ioremap((addr), (size))
#endif

/* struct timeval was removed (Y2038). The vendor osal exposes
 * osal_do_gettimeofday(struct timeval *) and several TUs use struct timeval as
 * a plain {sec,usec} pair. Provide the classic layout so those compile
 * unchanged; osal_do_gettimeofday() is backed by ktime_get_real_ts64(). */
#ifndef _COMPAT_STRUCT_TIMEVAL
#define _COMPAT_STRUCT_TIMEVAL
struct timeval {
	long	tv_sec;
	long	tv_usec;
};
#endif

/* struct timespec removed (Y2038); restore classic layout for vendor log
 * structs that embed it (mtk_btif _btif_log_buf_t_.ts). */
#ifndef _COMPAT_STRUCT_TIMESPEC
#define _COMPAT_STRUCT_TIMESPEC
struct timespec {
	long	tv_sec;
	long	tv_nsec;
};
#endif

/* do_gettimeofday(struct timeval *) removed; back with ktime_get_real_ts64. */
#ifndef do_gettimeofday
#include <linux/ktime.h>
#include <linux/timekeeping.h>
static inline void do_gettimeofday(struct timeval *tv)
{
	struct timespec64 _n;

	ktime_get_real_ts64(&_n);
	tv->tv_sec = _n.tv_sec;
	tv->tv_usec = _n.tv_nsec / 1000;
}
#endif

/* ktime_to_timespec() removed; provide a wrapper returning the compat
 * struct timespec via the 64-bit variant. */
#ifndef ktime_to_timespec
static inline struct timespec ktime_to_timespec(ktime_t kt)
{
	struct timespec64 _t = ktime_to_timespec64(kt);
	struct timespec _r;

	_r.tv_sec = _t.tv_sec;
	_r.tv_nsec = _t.tv_nsec;
	return _r;
}
#endif

/* dma_zalloc_coherent() removed; dma_alloc_coherent() now zeroes. */
#ifndef dma_zalloc_coherent
#define dma_zalloc_coherent(d, s, h, f) dma_alloc_coherent((d), (s), (h), (f))
#endif

/* arm64's internal cache-maintenance helpers __dma_unmap_area/__dma_flush_area
 * are not exported to drivers on mainline. The vendor BTIF used them around its
 * RX/TX vFIFO buffers - but those buffers are allocated with dma_alloc_coherent()
 * (hal_btif_vfifo_init(), mtk_btif.c). For a *coherent* allocation the DMA API
 * guarantees CPU/device coherency WITHOUT explicit cache maintenance; the only
 * requirement is ordering. An ordering barrier is therefore the correct and
 * sufficient mainline primitive here - dma_sync_single_*() is for streaming
 * (dma_map_single) mappings and must NOT be used on a coherent buffer.
 *   __dma_unmap_area(.., DMA_FROM_DEVICE): order the CPU read after DMA done -> dma_rmb()
 *   __dma_flush_area(): order CPU writes before the device reads them   -> dma_wmb()
 * (The TX path already issues mb() before __dma_flush_area, so this is belt-and-braces.) */
#ifndef __dma_unmap_area
#include <asm/barrier.h>
#include <linux/dma-direction.h>
static inline void __dma_unmap_area(const void *addr, size_t size, int dir)
{
	(void)addr; (void)size;
	if (dir == DMA_FROM_DEVICE)
		dma_rmb();
	else
		dma_wmb();
}
#endif
#ifndef __dma_flush_area
static inline void __dma_flush_area(const void *addr, size_t size)
{ (void)addr; (void)size; dma_wmb(); }
#endif

/* from_timer() was renamed to timer_container_of() in 6.16. */
#ifndef from_timer
#define from_timer(var, callback_timer, timer_fieldname) 	timer_container_of(var, callback_timer, timer_fieldname)
#endif

/* rtc_time_to_tm() (32-bit) removed; use the 64-bit variant. */
#ifndef rtc_time_to_tm
#define rtc_time_to_tm(time, tm) rtc_time64_to_tm((time64_t)(time), (tm))
#endif

/* del_timer()/del_timer_sync() removed in 6.15; renamed timer_delete*(). */
#ifndef del_timer
#define del_timer(t)      timer_delete(t)
#endif
#ifndef del_timer_sync
#define del_timer_sync(t) timer_delete_sync(t)
#endif

/* The set_fs()/get_fs()/KERNEL_DS uaccess address-limit mechanism was removed
 * in 5.10 (arm64 had it gone earlier). The vendor's kalFileRead/Write already
 * use kernel_read()/kernel_write(), which operate on kernel addresses without
 * any address-limit override - so the surrounding set_fs() dance is now a
 * no-op. Provide opaque stubs so the 4.14 source compiles unchanged. These
 * touch no hardware and change no behaviour (kernel_read/write are correct). */
#ifndef _COMPAT_SET_FS
#define _COMPAT_SET_FS
typedef struct { unsigned long seg; } mm_segment_t;
static inline mm_segment_t get_fs(void) { mm_segment_t s = { 0 }; return s; }
static inline mm_segment_t get_ds(void) { mm_segment_t s = { 0 }; return s; }
static inline void set_fs(mm_segment_t s) { (void)s; }
#ifndef KERNEL_DS
#define KERNEL_DS ((mm_segment_t){ 0 })
#endif
#endif

/* strlcpy() was removed in 6.8 in favour of strscpy() (which has the same
 * truncate-and-NUL-terminate semantics; return value differs but the vendor
 * uses it fire-and-forget). */
#ifndef strlcpy
#include <linux/string.h>
static inline size_t __compat_strlcpy(char *dst, const char *src, size_t size)
{
	size_t srclen = strlen(src);

	if (size) {
		size_t len = (srclen >= size) ? size - 1 : srclen;

		memcpy(dst, src, len);
		dst[len] = '\0';
	}
	return srclen;
}
#define strlcpy(d, s, n) __compat_strlcpy((d), (s), (n))
#endif

/* netif_rx_ni() was folded into netif_rx() in 5.18 (netif_rx() now handles
 * both process and softirq context). */
#ifndef netif_rx_ni
#define netif_rx_ni(skb) netif_rx(skb)
#endif

/* fbdev FB_EVENT_BLANK notifier was removed. The wmt fb notifier is never
 * registered on mainline (no fb_register_client); define the sentinel so the
 * callback compiles and always early-returns. */
#ifndef FB_EVENT_BLANK
#define FB_EVENT_BLANK 0x09
#endif

/* --- adapter KERNEL_* macros the adapter header leaves UNDEFINED ---------- *
 * (clk_buf / pmic_ldo blocks are gated on CONFIG_MACH_* / CONFIG_MTK_PMIC_CHIP_*
 * which are unset on mainline). No-op at the API level - no hardware touched.  */
static inline void __wmt_compat_clk_buf_ctrl(int id, bool onoff) { (void)id; (void)onoff; }
static inline void __wmt_compat_clk_buf_show_status_info(void) { }
static inline int  __wmt_compat_pmic_ldo_lp(int a, int b, int c, int d) { (void)a; (void)b; (void)c; (void)d; return 0; }

#ifndef KERNEL_clk_buf_ctrl
#define KERNEL_clk_buf_ctrl(id, onoff)           __wmt_compat_clk_buf_ctrl((id), (onoff))
#endif
#ifndef KERNEL_clk_buf_show_status_info
#define KERNEL_clk_buf_show_status_info()        __wmt_compat_clk_buf_show_status_info()
#endif
#ifndef KERNEL_pmic_ldo_vcn18_lp
#define KERNEL_pmic_ldo_vcn18_lp(a, b, c, d)     __wmt_compat_pmic_ldo_lp((a), (b), (c), (d))
#endif
#ifndef KERNEL_pmic_ldo_vcn33_2_lp
#define KERNEL_pmic_ldo_vcn33_2_lp(a, b, c, d)   __wmt_compat_pmic_ldo_lp((a), (b), (c), (d))
#endif

#endif /* __KERNEL__ */

#endif /* _MTK_BSP_COMPAT_H */
