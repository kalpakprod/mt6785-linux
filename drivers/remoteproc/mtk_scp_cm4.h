/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MT6785 SCP (tinysys, Cortex-M4) for mainline: the interface between the
 * loader/IPI driver and its clients.
 *
 * The SCP has no mailbox and no virtio.  Both directions of the IPI are a
 * fixed pair of objects in its SRAM (vendor scp/cm4/scp_ipi.h struct
 * share_obj), one for AP to SCP at SRAM+0x6e0 and one for SCP to AP at
 * SRAM+0x5c0, and the interrupt is bit 0 of the host-to-SCP register for the
 * way in and bit 0 of the SCP-to-host register for the way out (vendor
 * scp/cm4/v01/scp_ipi.c, scp_reg.h).
 *
 * IPI ids, memory block ids and feature ids are the ones of this SoC's
 * firmware: vendor scp/cm4/include/scp_cm4.h.
 */
#ifndef __MTK_SCP_CM4_H
#define __MTK_SCP_CM4_H

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct clk;
struct device;
struct dentry;

/* SCP SRAM layout, vendor scp/cm4/scp_helper.h */
#define SCP_CM4_REGION_INFO_OFFSET	0x400
#define SCP_CM4_RTOS_START		0x800
#define SCP_CM4_SHARE_BUF_SIZE		288	/* struct share_obj */
#define SCP_CM4_SHARE_RECV_OFFSET \
	(SCP_CM4_RTOS_START - SCP_CM4_SHARE_BUF_SIZE * 2)
#define SCP_CM4_SHARE_SEND_OFFSET \
	(SCP_CM4_SHARE_RECV_OFFSET + SCP_CM4_SHARE_BUF_SIZE)

/* fields of struct share_obj, in bytes from the start of the object */
#define SCP_CM4_SHARE_ID		0x0
#define SCP_CM4_SHARE_LEN		0x4
#define SCP_CM4_SHARE_DATA		0x10

/* bus turnaround, so a sender waits for the firmware to take the object */
#define SCP_CM4_IPI_BUSY_TIMEOUT_US	1000000

enum scp_core_id {
	SCP_A_ID = 0,
	SCP_CORE_TOTAL = 1,
};

/*
 * IPI ids of the MT6785 CM4 firmware (vendor scp/cm4/include/scp_cm4.h).  They
 * are message ids, not mailbox pins: the object carries the id, so unlike the
 * RISC-V SCP of later SoCs there is no pin table and no offset arithmetic.
 */
enum scp_cm4_ipi_id {
	IPI_WDT = 0,			/* AP to SCP: watchdog ping */
	IPI_TEST1,
	IPI_LOGGER_ENABLE,
	IPI_LOGGER_WAKEUP,
	IPI_LOGGER_INIT_A,
	IPI_VOW,
	IPI_AUDIO,
	IPI_DVT_TEST,
	IPI_SENSOR,			/* sensor hub protocol, both ways */
	IPI_TIME_SYNC,
	IPI_SHF,
	IPI_CONSYS,
	IPI_SCP_A_READY,		/* SCP to AP: ready, image size */
	IPI_APCCCI,
	IPI_SCP_A_RAM_DUMP,
	IPI_DVFS_DEBUG,
	IPI_DVFS_FIX_OPP_SET,
	IPI_DVFS_FIX_OPP_EN,
	IPI_DVFS_LIMIT_OPP_SET,
	IPI_DVFS_LIMIT_OPP_EN,
	IPI_DVFS_DISABLE,
	IPI_DVFS_SLEEP,
	IPI_PMICW_MODE_DEBUG,
	IPI_DVFS_SET_FREQ,
	IPI_CHRE,			/* CHRE transport, sensors-1.0 */
	IPI_CHREX,
	IPI_SCP_PLL_CTRL,
	IPI_DO_AP_MSG,
	IPI_DO_SCP_MSG,
	IPI_MET_SCP,
	IPI_SCP_TIMER,
	IPI_SCP_ERROR_INFO,		/* SCP to AP: error report */
	IPI_SCPCTL,
	IPI_SCP_LOG_FILTER,
	IPI_ELLIPTIC = 34,
	IPI_SENSOR_INIT_START = 35,
	SCP_CM4_NR_IPI,
};

enum scp_ipi_status {
	SCP_IPI_NOT_READY = -2,
	SCP_IPI_ERROR = -1,
	SCP_IPI_DONE,
	SCP_IPI_BUSY,
};

enum SCP_NOTIFY_EVENT {
	SCP_EVENT_READY = 0,
	SCP_EVENT_STOP,
};

/* DRAM blocks the firmware expects inside the scp_share carriage-out */
enum scp_reserve_mem_id_t {
	VOW_MEM_ID,
	SENS_MEM_ID,
	MP3_MEM_ID,
	FLP_MEM_ID,
	SCP_A_LOGGER_MEM_ID,
	AUDIO_IPI_MEM_ID,
	SPK_PROTECT_MEM_ID,
	SPK_PROTECT_DUMP_MEM_ID,
	VOW_BARGEIN_MEM_ID,
	SCP_DRV_PARAMS_MEM_ID,
	ULTRA_MEM_ID,
	SCP_ELLIPTIC_DEBUG_MEM,
	SCP_CM4_NUMS_MEM_ID,
};

enum feature_id {
	VOW_FEATURE_ID,
	OPEN_DSP_FEATURE_ID,
	SENS_FEATURE_ID,
	MP3_FEATURE_ID,
	FLP_FEATURE_ID,
	RTOS_FEATURE_ID,
	SPEAKER_PROTECT_FEATURE_ID,
	VCORE_TEST_FEATURE_ID,
	VOW_BARGEIN_FEATURE_ID,
	VOW_DUMP_FEATURE_ID,
	VOW_VENDOR_M_FEATURE_ID,
	VOW_VENDOR_A_FEATURE_ID,
	VOW_VENDOR_G_FEATURE_ID,
	ULTRA_FEATURE_ID,
	SCP_CM4_NUM_FEATURE_ID,
};

/* one DRAM block inside the carriage-out */
struct scp_cm4_mblock {
	phys_addr_t phys;
	void *virt;
	size_t size;
};

struct scp_cm4 {
	struct device *dev;

	void __iomem *sram;
	void __iomem *cfg;
	void __iomem *clkctrl;
	void __iomem *l1cctrl;
	size_t sram_size;

	struct clk *clk_main;
	struct clk *clk_infra;
	struct regmap *infracfg;	/* awake lock registers */

	/* carriage-out of the image, the bootloader's SCP-reserved region */
	phys_addr_t dram_phys;
	size_t dram_size;
	void *dram;
	phys_addr_t dram_fw_phys;
	size_t dram_fw_size;
	phys_addr_t dram_data_phys;
	size_t dram_data_size;
	phys_addr_t dram_backup_phys;

	/* scp_share carriage-out the firmware splits into the blocks below */
	phys_addr_t share_phys;
	size_t share_size;
	void *share;
	struct scp_cm4_mblock mblock[SCP_CM4_NUMS_MEM_ID];

	struct completion ready;
	bool ready_seen;
	u32 image_size_reported;
	struct work_struct ready_work;

	/* IPI: one message in flight per direction, the id picks the handler */
	struct mutex ipi_lock;
	void (*ipi_handler[SCP_CM4_NR_IPI])(int id, void *data, unsigned int len);
	const char *ipi_name[SCP_CM4_NR_IPI];
	u8 *ipi_recv_buf;
	struct blocking_notifier_head notifier;
	int awake_count;
	spinlock_t awake_lock;		/* guards awake_count and the ack polls */

	struct dentry *debugfs;
};

int scp_cm4_ipi_init(struct scp_cm4 *scp);
void scp_cm4_ipi_isr(struct scp_cm4 *scp);
void scp_cm4_ipi_reset(struct scp_cm4 *scp);

/* the one SCP of this SoC, set by the loader's probe */
extern struct scp_cm4 *scp_cm4_scp;

/* the API the sensor hub and other SCP clients use */
enum scp_ipi_status scp_cm4_ipi_registration(enum scp_cm4_ipi_id id,
	void (*handler)(int id, void *data, unsigned int len),
	const char *name);
enum scp_ipi_status scp_cm4_ipi_unregistration(enum scp_cm4_ipi_id id);
enum scp_ipi_status scp_cm4_ipi_send(enum scp_cm4_ipi_id id, void *buf,
	unsigned int len, unsigned int wait, enum scp_core_id core);
unsigned int scp_cm4_is_ready(enum scp_core_id core);
void scp_cm4_notify_register(struct notifier_block *nb);
void scp_cm4_notify_unregister(struct notifier_block *nb);
int scp_cm4_awake_lock(void *unused);
int scp_cm4_awake_unlock(void *unused);
phys_addr_t scp_cm4_reserve_mem_phys(enum scp_reserve_mem_id_t id);
phys_addr_t scp_cm4_reserve_mem_virt(enum scp_reserve_mem_id_t id);
phys_addr_t scp_cm4_reserve_mem_size(enum scp_reserve_mem_id_t id);
void scp_cm4_register_feature(enum feature_id id);
void scp_cm4_deregister_feature(enum feature_id id);
void scp_cm4_wdt_reset(int cpu_id);

#endif /* __MTK_SCP_CM4_H */
