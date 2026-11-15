// SPDX-License-Identifier: GPL-2.0-only
/*
 * Poco M5s (rosemary): the modem's runtime-data handshake.
 *
 * At the first handshake the modem asks, through the CCIF SRAM, which of its
 * 64 runtime features the AP supports; the AP answers with one TLV per feature
 * in the non-cacheable runtime-data region plus a summary structure in the
 * SRAM. Every address in that answer is the modem's own view of the shared
 * windows, and every value is this phone's own.
 *
 * Adapted from the cezanne port's rtdata.c (github.com/DrShadow34/
 * cezanne-linux-port, linux/drivers/modem-ccci/rtdata.c). The one deliberate
 * difference is the world-mode bitmap, and it is the reason this file is not a
 * copy:
 *
 *   cezanne hard-codes 0x7d for MISC_INFO_SBP_ID. In
 *   ccci_util_md_rat.c:14-23 that is bit0 G | bit2 C | bit3 W | bit4 Lt |
 *   bit5 Lf | bit6 N -- i.e. it tells the modem that cdma2000 AND NR are
 *   available. rosemary is a 4G-only MT6785 with no C2K subsystem driver and
 *   no second modem; its own vendor config asks for ro.vendor.mtk_md1_support=9
 *   (ulwg = Lf|Lt|W|G = 0x39). This driver therefore takes the bitmap from the
 *   image's own capability, which tags.c read out of the loader's md1_chk tag
 *   and md_img_capability_map[] (ccci_util_md_rat.c:33-86), and only falls back
 *   to 0x39 when the tag was not readable.
 */
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/random.h>
#include <linux/string.h>
#include <linux/timekeeping.h>

#include "rosemary_ccci.h"

/* the world-mode bitmap for a 4G-only phone: Lf|Lt|W|G */
#define SBP_ULWG		0x39

enum {
	FEATURE_NOT_EXIST,
	FEATURE_NOT_SUPPORT,
	FEATURE_MUST_SUPPORT,
	FEATURE_OPTIONAL_SUPPORT,
	FEATURE_SUPPORT_BACKWARD_COMPAT,
};

/* runtime feature ids (vendor enum MD_CCCI_RUNTIME_FEATURE_ID) */
enum {
	BOOT_INFO = 0,
	EXCEPTION_SHARE_MEMORY,
	CCIF_SHARE_MEMORY,
	MISC_INFO_HIF_DMA_REMAP = 5,
	MISC_INFO_RTC_32K_LESS,
	MISC_INFO_RANDOM_SEED_NUM,
	MISC_INFO_GPS_COCLOCK,
	MISC_INFO_SBP_ID,
	MISC_INFO_CCCI,
	MISC_INFO_CLIB_TIME,
	MISC_INFO_C2K,
	MD_IMAGE_START_MEMORY,
	CCB_SHARE_MEMORY = 15,
	DHL_RAW_SHARE_MEMORY,
	DT_NETD_SHARE_MEMORY,
	DT_USB_SHARE_MEMORY,
	EE_AFTER_EPOF,
	AP_CCMNI_MTU,
	LWA_SHARE_MEMORY = 25,
	AUDIO_RAW_SHARE_MEMORY,
	MULTI_MD_MPU,
	CCISM_SHARE_MEMORY_EXP,
	MD_CONSYS_SHARE_MEMORY = 30,
	MD_USIP_SHARE_MEMORY,
	MD_MTEE_SHARE_MEMORY_ENABLE,
	MD_WIFI_PROXY_SHARE_MEMORY = 35,
	NVRAM_CACHE_SHARE_MEMORY,
	MD_MEM_AP_VIEW_INF = 38,
};

struct rt_feature {
	u8 id;
	u8 support;		/* support_mask:4, version:4 */
	u8 reserved[2];
	u32 len;
} __packed;

/* the AP's receive buffer size minus the CCCI header (CCCI_MTU + 16 - 128) */
#define AP_CCMNI_MTU_VAL	(3584 - 16)

static u8 *put(u8 *p, int id, u8 support, const void *data, u32 len)
{
	struct rt_feature f = { .id = id, .support = support, .len = len };

	memcpy(p, &f, sizeof(f));
	p += sizeof(f);
	if (len)
		memcpy(p, data, len);
	return p + len;
}

/* the data for one feature the AP supports; its length, or -1 if it has none */
static int feature_data(struct ccci_md *md, int id, u32 *d)
{
	struct timespec64 ts;

	memset(d, 0, 48);
	switch (id) {
	case BOOT_INFO:		/* boot channel CONTROL_RX, normal boot */
	case MISC_INFO_HIF_DMA_REMAP:
	case MISC_INFO_RTC_32K_LESS:
	case MISC_INFO_GPS_COCLOCK:
	case EE_AFTER_EPOF:
		return 16;
	case EXCEPTION_SHARE_MEMORY:
		d[0] = NC_MD_VIEW + NC_MDCCCI_DBG;
		d[1] = 2 * 1024;
		return 8;
	case CCIF_SHARE_MEMORY:
		d[0] = NC_MD_VIEW + NC_CCISM_MCU;
		d[1] = md->rings_size;
		return 8;
	case MISC_INFO_RANDOM_SEED_NUM:
		d[0] = get_random_u32();
		return 16;
	case MISC_INFO_SBP_ID:
		d[0] = 0;		/* the SBP code, 0 = the default bin */
		d[1] = md->md_img_cap ?: SBP_ULWG;
		return 16;
	case MISC_INFO_CCCI:
		d[0] = BIT(0) | BIT(1);	/* sequence check, MD status polling */
		return 16;
	case MISC_INFO_CLIB_TIME:
		ktime_get_real_ts64(&ts);
		d[0] = lower_32_bits(ts.tv_sec);
		d[1] = upper_32_bits(ts.tv_sec);
		return 16;		/* time zone and DST stay 0 */
	case MISC_INFO_C2K:
		/*
		 * There is no C2K subsystem on this SoC and the phone's own
		 * configuration is 4G-only, so the feature is answered as not
		 * supported by leaving it out of the answer. Its absence is
		 * reported here rather than silently.
		 */
		return -1;
	case MD_IMAGE_START_MEMORY:
		d[0] = lower_32_bits(md->md_mem_base);
		d[1] = md->md_mem_size;
		return 8;
	case CCB_SHARE_MEMORY: {
		/*
		 * The CCB buffers are where the loader put them, not at a fixed
		 * offset: this phone's ccb_info tag says 0x8c100000 + 0x1600000
		 * inside a 0x1780000 window. If the loader did not describe
		 * them the feature is answered as unsupported rather than with
		 * a guessed address.
		 */
		const struct ccci_c_item *ccb =
			ccci_smem_region(md, SMEM_USER_ID_CCB_DHL);

		if (!ccb)
			return -1;
		d[0] = NC_MD_VIEW + NC_CCB_CTRL;
		d[1] = NC_CCB_CTRL_SIZE;
		d[2] = NC_MD_VIEW + md->c_view_off + ccb->md_off;
		d[3] = ccb->size;
		return 16;
	}
	case DHL_RAW_SHARE_MEMORY:
		/*
		 * Declined: the modem's raw trace log (tens of MiB), optional
		 * for the modem and read only by the vendor's modem logger.
		 * On this phone it sits inside a no-map reservation, so
		 * declining costs nothing.
		 */
		return -1;
	case DT_NETD_SHARE_MEMORY:
		d[0] = NC_MD_VIEW + NC_NETD;
		d[1] = NC_NETD_SIZE;
		return 8;
	case DT_USB_SHARE_MEMORY:
		d[0] = NC_MD_VIEW + NC_USB;
		d[1] = NC_USB_SIZE;
		return 8;
	case AP_CCMNI_MTU:
		d[0] = AP_CCMNI_MTU_VAL;
		return 4;
	case LWA_SHARE_MEMORY:	/* no LWA region on this phone */
		return 8;
	case AUDIO_RAW_SHARE_MEMORY:
		d[0] = NC_MD_VIEW + NC_AUDIO;
		d[1] = NC_AUDIO_SIZE;
		return 8;
	case MULTI_MD_MPU:
		return 0;
	case CCISM_SHARE_MEMORY_EXP:
		d[0] = NC_MD_VIEW + NC_CCISM_MCU_EXP;
		d[1] = NC_CCISM_MCU_EXP_SIZE;
		return 8;
	case MD_CONSYS_SHARE_MEMORY: {
		const struct ccci_c_item *r =
			ccci_smem_region(md, SMEM_USER_ID_MD_CONSYS);

		if (!r)
			return -1;
		d[0] = NC_MD_VIEW + md->c_view_off + r->md_off;
		d[1] = r->size;
		return 8;
	}
	case MD_USIP_SHARE_MEMORY: {
		const struct ccci_c_item *r = ccci_smem_region(md, SMEM_USER_ID_USIP);

		if (!r)
			return -1;
		d[0] = NC_MD_VIEW + md->c_view_off + r->md_off;
		d[1] = r->size;
		return 8;
	}
	case MD_MTEE_SHARE_MEMORY_ENABLE:
		return 4;		/* no MTEE on this phone */
	case MD_WIFI_PROXY_SHARE_MEMORY:
		return -1;		/* no Wi-Fi proxy region */
	case NVRAM_CACHE_SHARE_MEMORY: {
		const struct ccci_c_item *r =
			ccci_smem_region(md, SMEM_USER_ID_MD_NVRAM_CACHE);

		if (!r)
			return -1;
		d[0] = NC_MD_VIEW + md->c_view_off + r->md_off;
		d[1] = r->size;
		return 8;
	}
	case MD_MEM_AP_VIEW_INF:
		/* { MD view, size, AP physical } per window */
		d[0] = 0;
		d[1] = md->md_mem_size;
		d[2] = lower_32_bits(md->md_mem_base);
		d[4] = NC_MD_VIEW;
		d[5] = NC_SIZE;
		d[6] = lower_32_bits(md->nc_phys);
		d[8] = NC_MD_VIEW + md->c_view_off;
		d[9] = C_SIZE;
		d[10] = lower_32_bits(md->c_phys);
		return 48;
	}
	return -1;
}

/* the vendor's merge rule: what the modem asks for against what the AP has */
static u8 merge(u8 md_set, bool ap_has)
{
	u8 mask = md_set & 0xf, ver = md_set >> 4;

	switch (mask) {
	case FEATURE_NOT_EXIST:
	case FEATURE_MUST_SUPPORT:
		return md_set;
	case FEATURE_NOT_SUPPORT:
		return 0;
	case FEATURE_OPTIONAL_SUPPORT:
		return ap_has && ver == 0 ? FEATURE_MUST_SUPPORT :
					    FEATURE_NOT_SUPPORT;
	case FEATURE_SUPPORT_BACKWARD_COMPAT:
		return ap_has ? FEATURE_MUST_SUPPORT : FEATURE_NOT_SUPPORT;
	}
	return 0;
}

static int prepare(struct ccci_md *md, const struct md_query_ap_feature *q)
{
	u8 *base = md->nc + NC_RUNTIME_DATA, *p = base;
	u32 d[12];
	int i, len;

	if (q->head_pattern != MD_FEATURE_QUERY_PATTERN ||
	    q->tail_pattern != MD_FEATURE_QUERY_PATTERN) {
		dev_err(md->dev, "HS1: bad feature query %#x/%#x\n",
			q->head_pattern, q->tail_pattern);
		return -EINVAL;
	}
	dev_info(md->dev, "HS1: MD features %*phN\n", FEATURE_COUNT,
		 q->feature_set);
	for (i = 0; i < FEATURE_COUNT; i++) {
		u8 sup;

		len = feature_data(md, i, d);
		sup = merge(q->feature_set[i], len >= 0);
		if ((q->feature_set[i] & 0xf) == FEATURE_MUST_SUPPORT && len < 0) {
			dev_err(md->dev, "HS1: the modem needs feature %d\n", i);
			return -EINVAL;
		}
		if ((sup & 0xf) != FEATURE_MUST_SUPPORT)
			len = 0;
		if (p + sizeof(struct rt_feature) + len > base + NC_RUNTIME_AP_SIZE)
			return -ENOSPC;
		p = put(p, i, sup, d, len);
	}
	wmb();
	return p - base;
}

/**
 * ccci_send_runtime_data() - answer the modem's feature query
 * @md: the modem
 * @query: the &struct md_query_ap_feature the modem left in the SRAM
 */
void ccci_send_runtime_data(struct ccci_md *md, const void *query)
{
	struct ap_query_md_feature_v2_1 a = {
		.head_pattern = AP_FEATURE_QUERY_PATTERN,
		.share_memory_support = 2,	/* MULTI_MD_MPU_SUPPORT */
		.ap_runtime_data_addr = NC_MD_VIEW + NC_RUNTIME_DATA,
		.ap_runtime_data_size = NC_RUNTIME_AP_SIZE,
		.md_runtime_data_addr = NC_MD_VIEW + NC_RUNTIME_DATA +
					NC_RUNTIME_AP_SIZE,
		.md_runtime_data_size = NC_RUNTIME_MD_SIZE,
		.noncached_mpu_start_addr = NC_MD_VIEW,
		.noncached_mpu_total_size = NC_SIZE,
		.cached_mpu_start_addr = NC_MD_VIEW + md->c_view_off,
		.cached_mpu_total_size = C_SIZE,
		.tail_pattern = AP_FEATURE_QUERY_PATTERN,
	};
	struct ccci_header h = {
		.data = { 0, sizeof(h) + sizeof(a) },
		.channel = CCCI_CONTROL_TX,
		.reserved = MD_INIT_CHK_ID,
	};
	int n = prepare(md, query);

	if (n < 0) {
		dev_err(md->dev, "HS1: no runtime data (%d)\n", n);
		return;
	}
	/* the MPU size covers the AP<->MD1 and MD1<->MD3 windows */
	a.feature_set[1] = 1 << 4;
	ccci_sram_write(md, SRAM_UP_HEADER, &h, sizeof(h));
	ccci_sram_write(md, SRAM_AP_RT_DATA, &a, sizeof(a));
	wmb();
	ccci_ccif_send_ch(md, CCIF_CH_SRAM);
	dev_info(md->dev, "HS1: runtime data sent (%d bytes)\n", n);
}
