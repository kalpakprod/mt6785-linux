/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Poco M5s (rosemary): MediaTek MT6785 modem, CCCI over CCIF -- shared state.
 *
 * The modem is MD1, generation 6295: control runs over CCIF (0x10209000 on the
 * AP side, 0x1020a000 on the modem's), packet data over DPMAIF (0x10014000).
 * There is no CLDMA and no second modem on this part.
 *
 * Sources for every hardware fact below: the phone's live device tree
 * (poco-m5s/device-dump-2026-11-12/device-tree.dts, ccifdriver@10209000 and
 * mddriver), the vendor GPL kernel 4.19 for this SoC
 * (scratch/native/4-modem/src/los-mt6785, drivers/misc/mediatek/eccci/), and
 * the port plan (poco-m5s/research/native/4-modem/). Where they disagree the
 * live device tree wins, and where the vendor reads its own generation-gated
 * tables the generation decides -- see the note on the share-memory offsets.
 */
#ifndef __ROSEMARY_CCCI_H
#define __ROSEMARY_CCCI_H

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

struct ccci_md;
struct wwan_port;
struct ccci_dpmaif;

/* ---- CCIF, the control transport (AP side) ---------------------------- */

/* ccif_hif_platform.h:32-38, identical on both sides */
#define APCCIF_CON		0x00
#define APCCIF_BUSY		0x04	/* per-channel busy bitmap */
#define APCCIF_START		0x08
#define APCCIF_TCHNUM		0x0c
#define APCCIF_RCHNUM		0x10
#define APCCIF_ACK		0x14
#define APCCIF_CHDATA		0x100
#define CCIF_SRAM_SIZE		512

/*
 * Channel layout for MD_GENERATION >= 6295 (ccif_hif_platform.h:41-57:
 * RINGQ_BASE 0, RINGQ_SRAM 15, RINGQ_EXP_BASE 15, CCIF_CH_NUM 24,
 * AP_MD_CCB_WAKEUP 7): ring queue n is channel n, the runtime-data exchange
 * uses channel 15, the exception channels start at 15.
 */
#define CCIF_QUEUES		16
#define CCIF_CH_SRAM		15
#define CCIF_CH_CCB_WAKEUP	7
#define CCIF_CH_EXP_BASE	15

/*
 * The exception channels, one per stage of the modem's crash handshake
 * (hif/ccif_hif_platform.h:39-66). The first and third expect an answer from
 * the AP on the same two numbers the modem rings; see ee.c.
 */
#define CCIF_CH_EX_INIT		(CCIF_CH_EXP_BASE + 1)	/* +0x10000 */
#define CCIF_CH_EX_INIT_DONE	(CCIF_CH_EXP_BASE + 2)	/* +0x20000 */
#define CCIF_CH_EX_CLEARQ_DONE	(CCIF_CH_EXP_BASE + 3)	/* +0x40000 */
#define CCIF_CH_EX_ALLQ_RESET	(CCIF_CH_EXP_BASE + 4)	/* +0x80000 */
#define CCIF_CH_EX_PEER_WAKEUP	(CCIF_CH_EXP_BASE + 5)
#define CCIF_CH_EX_SEQ_ERROR	(CCIF_CH_EXP_BASE + 6)
#define CCIF_CH_MAX		24	/* CCIF_CH_NUM: bits above this are not ours */

/* CCCI logical channels (eccci/inc/ccci_core.h:314-420) */
#define CCCI_CONTROL_RX		0
#define CCCI_CONTROL_TX		1
#define CCCI_SYSTEM_RX		2
#define CCCI_SYSTEM_TX		3
#define CCCI_PCM_RX		4
#define CCCI_PCM_TX		5
#define CCCI_UART1_RX		6
#define CCCI_UART1_TX		8
#define CCCI_UART2_RX		10	/* AT */
#define CCCI_UART2_TX		12
#define CCCI_FS_RX		14
#define CCCI_FS_TX		15
#define CCCI_RPC_RX		32
#define CCCI_RPC_TX		33
#define CCCI_IPC_RX		34
#define CCCI_IPC_TX		36
#define CCCI_IPC_UART_RX	38	/* AGPS */
#define CCCI_IPC_UART_TX	40
#define CCCI_MD_LOG_RX		42
#define CCCI_MD_LOG_TX		43
#define CCCI_STATUS_RX		67
#define CCCI_STATUS_TX		68
#define CCCI_MAX_CH		256

/* control messages (ccci_header.data[1]) */
#define MD_INIT_START_BOOT	0x0
#define MD_EX			0x4
#define MD_EX_REC_OK		0x6
#define MD_EX_PASS		0x8
#define MD_INIT_CHK_ID		0x5555ffff

struct ccci_header {
	u32 data[2];
	u16 channel;
	u16 seq_num:15;
	u16 assert_bit:1;
	u32 reserved;
} __packed;

/* ring framing (hif/ccci_ringbuf.c) */
#define RBF_HEADER		0xee0000ee
#define RBF_FOOTER		0xff0000ff
#define PKG_HEADER		0xaabbaabb
#define PKG_FOOTER		0xccddeeff

struct ccci_ringbuf_ctl {
	u32 read;
	u32 write;
	u32 length;
};

struct ccci_ringbuf {
	struct ccci_ringbuf_ctl rx, tx;
	u8 buffer[];
};

#define RINGBUF_CTL_LEN		(8 + sizeof(struct ccci_ringbuf) + 8)

/*
 * Non-cacheable share memory, offsets in KiB from the window base.
 *
 * These are the vendor's own numbers for THIS generation:
 * ccci_modem.c:113-141 (md1_6293_noncacheable_fat), which is the table
 * ccci_6295_md_smem_layout_config() at ccci_modem.c:705 walks -- generation
 * 6295 uses the 6293 layout, not the 6297 one at ccci_modem.c:70. The plan's
 * warning about cezanne's constants applies here too: cezanne's ccci.h puts
 * RAW_USB at 108 KiB, RAW_AUDIO at 112 KiB and CCISM_MCU at 164 KiB, i.e.
 * every region from RAW_USB on is 4 KiB away from where this phone has it.
 */
#define NC_MDCCCI_DBG		0		/* + 2 KiB  */
#define NC_MDSS_DBG		0x800		/* + 10 KiB */
#define NC_MDSS_DBG_SIZE	0x2800
#define NC_RESERVED		0x3000		/* + 46 KiB */
#define NC_RUNTIME_DATA		0xe800		/* + 4 KiB: AP 2, MD 2 */
#define NC_RUNTIME_AP_SIZE	0x800
#define NC_RUNTIME_MD_SIZE	0x800
#define NC_FORCE_ASSERT		0xf800		/* + 1 KiB */
/*
 * RAW_DBM is the last block before 64 KiB: CCCI_SMEM_SIZE_DBM 160 plus two
 * 8-byte guards (ccci_common_config.h:30-31, ccci_modem.c:53), so 176 bytes
 * ending exactly at the CCISM_SCP base.
 */
#define NC_DBM			0xff50		/* + 176 bytes */
#define NC_DBM_SIZE		176
#define NC_CCISM_SCP		0x10000		/* + 32 KiB */
#define NC_CCB_CTRL		0x18000		/* + 4 KiB */
#define NC_CCB_CTRL_SIZE	0x1000
#define NC_NETD			0x19000		/* + 4 KiB */
#define NC_NETD_SIZE		0x1000
#define NC_USB			0x1a000		/* + 4 KiB */
#define NC_USB_SIZE		0x1000
#define NC_AUDIO		0x1b000		/* + 52 KiB */
#define NC_AUDIO_SIZE		0xd000
#define NC_CCISM_MCU		0x28000		/* 160 KiB, 721 KiB of rings */
#define NC_CCISM_MCU_SIZE	0x2d1000
#define NC_CCISM_MCU_EXP	0xdc400		/* 881 KiB, 121 KiB */
#define NC_CCISM_MCU_EXP_SIZE	0x1e400
#define NC_DFD			0x100000	/* 1 MiB, sized by tag */
#define NC_SIZE			0x1770000	/* the window LK reserved */

/* MD view of the non-cacheable window (LK: "Map 0x40000000 for MD1") */
#define NC_MD_VIEW		0x40000000

/*
 * Cacheable share memory. There are NO fixed offsets here on purpose: the
 * window's sub-regions are described by the loader in md1_bank4_cache_info and
 * md1_bank4_cache_layout, and they are read into struct ccci_md::c_item at
 * probe. This phone's own trial printed "buffer 0x8c000000 + 0x1780000,
 * md offset 0xc000000, 5 items", which no constant table from another phone
 * matches -- cezanne's CCB offset of 0x4900000 is 64 MiB, i.e. nearly three
 * times this window, and using it oopsed the first boot.
 *
 * Use ccci_smem_region() to find a region.
 */
#define C_SIZE			0x1780000	/* the window LK reserved */

/* ---- runtime data (handshake 1) -------------------------------------- */

#define FEATURE_COUNT		64
#define MD_FEATURE_QUERY_PATTERN 0x49434343
#define AP_FEATURE_QUERY_PATTERN 0x43434349

struct md_query_ap_feature {
	u32 head_pattern;
	u8 feature_set[FEATURE_COUNT];
	u32 tail_pattern;
	u8 reserved[152];
} __packed;

struct ap_query_md_feature_v2_1 {
	u32 head_pattern;
	u8 feature_set[FEATURE_COUNT];
	u32 share_memory_support;
	u32 ap_runtime_data_addr;
	u32 ap_runtime_data_size;
	u32 md_runtime_data_addr;
	u32 md_runtime_data_size;
	u32 noncached_mpu_start_addr;
	u32 noncached_mpu_total_size;
	u32 cached_mpu_start_addr;
	u32 cached_mpu_total_size;
	u32 reserve_addr[12];
	u32 tail_pattern;
} __packed;

/* CCIF SRAM layout during the handshakes (ccif-regs.md, +0x100 on) */
#define SRAM_DL_HEADER		0
#define SRAM_MD_RT_DATA		16
#define SRAM_UP_HEADER		(16 + sizeof(struct md_query_ap_feature))
#define SRAM_AP_RT_DATA		(SRAM_UP_HEADER + 16)

/* ---- SPM and infracfg, the power sequence ---------------------------- */

#define SPM_PHYS		0x10006000
#define INFRA_PHYS		0x10001000
#define SEC_AO_PHYS		0x1001a000
#define MD_PLL_PHYS		0x1020e300
#define MD_BOOT_VECTOR_PHYS	0x20000024

#define SPM_POWERON_CONFIG_EN	0x000	/* write 0xb16 << 16 | 1 first */
#define SPM_CLK_CON		0x008	/* BIT(21): srclken_o1 */
/*
 * PCM_REG13_DATA, the SPM register that carries the MD-side sleep handshake
 * (base/power/spm/mt6785/mtk_spm_reg.h:88, bits from pcm_def.h:148-151):
 * R13_MD_SRCCLKENA_0 BIT(2) and R13_MD_APSRC_REQ_0 BIT(3) for MD1. The vendor
 * reads them to decide whether MD1 is asleep (spm_is_md1_sleep(),
 * mtk_spm_utils.c:46-50), so they are the modem's own view of the same request.
 */
#define SPM_PCM_REG13_DATA	0x110
#define SPM_R13_MD_SRCCLKENA_0	BIT(2)
#define SPM_R13_MD_APSRC_REQ_0	BIT(3)
/* the buck isolation the vendor's MD1 sequence clears before power-on */
#define SPM_MD_EXT_BUCK_ISO_CON	0x3b0
#define SPM_PWR_STATUS		0x160	/* BIT(0): the md domain is on */
#define SPM_PWR_STATUS_2ND	0x164	/* the same, one SPM stage later */
/* the domain's control register: PWR_RST_B BIT(0), PWR_ON BIT(2) */
#define SPM_MD1_PWR_CON		0x318
#define SPM_AP_MDSRC_REQ	0x434	/* ack in BIT(4) (mtk_spm_reg.h:265,
					 * AP_MDSMSRC_ACK_LSB at :1851)
					 */
#define SPM_AP_MDSRC_ACK	BIT(4)
#define SPM_PROJECT_KEY		0xb1600000

#define INFRA_MD_SRCCLKENA	0xf0c	/* low byte 0x21 on, 0 off */
#define INFRA_RST0_AO		0x140
#define INFRA_RST1_AO		0x144
/* the MD bus protection: step1 BIT(7), step2 BIT(3)|BIT(4) (vendor clk-mt6785-pg.c) */
#define INFRA_TOPAXI_PROTECTEN	0x220
#define INFRA_RST0_PD		0x150
#define INFRA_RST1_PD		0x154

/* the boot-ROM gate: security_ao (0x1001a000) + 0x824..0x830 */
#define SEC_AO_BROM0		0x824
#define SEC_AO_BROM1		0x828
#define SEC_AO_BROM2		0x82c
#define SEC_AO_BROM3		0x830

/*
 * The modem's boot statistics: 0x1020e300 selected by 0x1020e700. Both are in
 * the block the pll window maps from MD_PLL_PHYS, so they are used as offsets.
 */
#define MD1_BOOT_STATS_SELECT	0x400
#define MD1_CFG_BOOT_STATS	0x0

/* ---- one modem ------------------------------------------------------- */

enum ccci_state {
	MD_STATE_OFF,
	MD_STATE_BOOTING,
	MD_STATE_HS1,
	MD_STATE_READY,
	MD_STATE_EXCEPTION,
};

/**
 * struct ccci_md - the modem, as the AP sees it
 *
 * Everything the bootloader decides -- the share-memory addresses, the modem's
 * DRAM window, the CCB buffers, the capability bitmap -- is filled by tags.c at
 * probe. None of it is compiled in, because the loader moves those regions
 * between boots and renumbers its reservations.
 */
struct ccci_md {
	struct device *dev;

	/*
	 * The loader's CCCI boot information. The header does not live in the
	 * reserved block: it arrives as the /chosen property
	 * ccci,modem_info_v2, and it names the block that holds the tag
	 * records. See lk_header_from_chosen() in tags.c.
	 */
	u64 lk_base;
	u32 lk_size;
	u32 lk_err_no;
	u32 lk_version;
	u32 lk_tag_num;
	u32 lk_ld_flag;
	u32 lk_ld_md_errno;

	/* the tag block the header names, mapped for the walk */
	phys_addr_t tag_base;
	resource_size_t tag_size;
	void __iomem *tags;

	/* reservations the loader made, found by name (tags.c) */
	phys_addr_t nc_phys;
	resource_size_t nc_size;
	phys_addr_t c_phys;
	resource_size_t c_size;
	phys_addr_t md_mem_phys;	/* the loader's md_mem_usage region */
	resource_size_t md_mem_region_size;

	/* the modem's DRAM window, from the hdr_tbl_inf tag */
	phys_addr_t md_mem_base;
	u32 md_mem_size;

	/* share-memory layout, from the smem_layout tag */
	u64 smem_base;
	u32 smem_size;
	u32 c_view_off;

	/* the cacheable window's real layout, from md1_bank4_cache_layout */
	struct ccci_c_item {
		u64 addr;		/* AP address of the region */
		u32 md_off;		/* its offset in the modem's view */
		u32 size;
		u32 id;			/* the SMEM_USER_ID it belongs to */
	} c_item[32];
	u32 c_item_num;

	/* logging buffers, from the ccb_info tag */
	u64 ccb_data_base;
	u32 ccb_data_size;
	u32 ccb_gear_id;
	u32 ccb_cap_gear;

	/* the image the loader started: check-header type and capability */
	u8 md_img_type;
	u32 md_img_cap;
	u32 hdr_count;

	/* mapped register windows */
	void __iomem *ccif_ap;
	void __iomem *ccif_md;
	void __iomem *infra;		/* infracfg 0x10001000 */
	void __iomem *spm;		/* SPM 0x10006000, writable copy */
	void __iomem *sec_ao;		/* security_ao 0x1001a000 */
	void __iomem *pll;		/* 0x1020e300: the modem's boot stats */
	void __iomem *boot_vector;	/* 0x20000024: MD_BOOT_VECTOR_EN */
	struct regmap *spm_map;		/* the SCPSYS syscon, for PWR_STATUS */
	void *nc;			/* AP view of the non-cacheable window */
	void *c;			/* ... and of the cacheable one */

	/* supplies and clocks the power sequence drives */
	struct regulator *vmodem;
	struct regulator *vsram_md;
	struct clk *md_clk;		/* scp-sys-md1-main */
	struct clk_bulk_data *clks;
	int num_clks;

	/* transport */
	struct ccci_ringbuf *ring[CCIF_QUEUES];
	struct ccci_ringbuf *ring_exp[CCIF_QUEUES];
	u32 rings_size;
	spinlock_t tx_lock[CCIF_QUEUES];
	struct work_struct rx_work;
	unsigned long pending;
	u16 seq[CCCI_MAX_CH];
	int irq_ccif, irq_ccif_exp, irq_wdt;

	/* boot */
	struct work_struct boot_work;
	bool rails_on;		/* this driver enabled vmodem and vsram_md */
	struct mutex lock;
	enum ccci_state state;
	bool first_boot;
	struct completion hs1, hs2;
	unsigned long last_fs;
	unsigned long ex_channels;
	bool ee_printed;		/* the exception record was reported */

	struct ccci_port *ports;
	int num_ports;
	struct mutex ports_lock;
	dev_t devt;

	struct ccci_dpmaif *dp;		/* the packet data path */
	bool no_boot_info;		/* the loader gave us nothing to boot */
};

/* a channel pair handed to userspace */
struct ccci_port {
	const char *name;
	u16 rx_ch, tx_ch;
	u8 txq;
	bool user_header;
	bool cdev_added;		/* the char device went into the kernel */
	struct ccci_md *md;
	struct cdev cdev;
	struct device *dev;
	struct sk_buff_head rxq;
	wait_queue_head_t wq;
	atomic_t users;
	unsigned long rx_drops;
	u32 smem_off, smem_size;
	struct wwan_port *wwan;		/* instead of the char device */
};

/* the SMEM_USER_ID values the driver looks regions up by (mtk_ccci_common.h:531) */
#define SMEM_USER_ID_CCB_DHL		2
#define SMEM_USER_ID_CCB_MD_MONITOR	3
#define SMEM_USER_ID_CCB_META		4
#define SMEM_USER_ID_MD_CONSYS		22
#define SMEM_USER_ID_USIP		24
#define SMEM_USER_ID_MD_NVRAM_CACHE	32

/* tags.c */
int rosemary_ccci_tags_init(struct ccci_md *md);

/*
 * The loader's cacheable layout, looked up by region id. Returns the AP
 * address of the region and its size, or NULL when the loader did not
 * describe that region -- callers must handle the NULL, because a region the
 * loader did not report is a region the driver must not touch.
 */
const struct ccci_c_item *ccci_smem_region(struct ccci_md *md, u32 id);

/* ccif.c */
int ccci_ccif_init(struct ccci_md *md);
void ccci_ccif_reset(struct ccci_md *md);
irqreturn_t ccci_ccif_isr(int irq, void *data);
int ccci_send_raw(struct ccci_md *md, int txq, void *pkt, size_t len);
int ccci_send(struct ccci_md *md, int txq, u16 channel, u32 data0, u32 data1,
	      u32 reserved, const void *payload, size_t len);
void ccci_sram_write(struct ccci_md *md, u32 off, const void *src, size_t len);
void ccci_sram_read(struct ccci_md *md, u32 off, void *dst, size_t len);
void ccci_ccif_send_ch(struct ccci_md *md, unsigned int ch);
void ccci_rx_work(struct work_struct *work);

/* rtdata.c */
void ccci_send_runtime_data(struct ccci_md *md, const void *query);

/* port.c */
int ccci_ports_init(struct ccci_md *md);
void ccci_ports_exit(struct ccci_md *md);
void ccci_ports_online(struct ccci_md *md, bool on);
bool ccci_port_recv(struct ccci_md *md, const struct ccci_header *h,
		    const void *pkt, size_t len);

/* dpmaif.c */
int rosemary_ccci_dpmaif_init(struct ccci_md *md);
void rosemary_ccci_dpmaif_exit(struct ccci_md *md);
int ccci_dpmaif_start(struct ccci_md *md);
void ccci_dpmaif_stop(struct ccci_md *md);

/* core.c */
void ccci_recv_control(struct ccci_md *md, const struct ccci_header *h,
		       const void *pkt, size_t len);
void ccci_ee_on_channel(struct ccci_md *md, unsigned int ch);
void ccci_md_stop(struct ccci_md *md);
void ccci_set_state(struct ccci_md *md, enum ccci_state state);

#endif /* __ROSEMARY_CCCI_H */
