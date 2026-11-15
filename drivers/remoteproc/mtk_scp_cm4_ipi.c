// SPDX-License-Identifier: GPL-2.0
/*
 * MT6785 SCP (tinysys, Cortex-M4) IPI for mainline.
 *
 * The firmware has no mailbox: an IPI is one object in SCP SRAM carrying an
 * id, a length and up to 272 bytes of payload, and the interrupt is a single
 * bit per direction.  That is the whole transport, and it is what the vendor
 * driver of this SoC does in drivers/misc/mediatek/scp/cm4/v01/scp_ipi.c
 * (port scp_ipi_send()/scp_A_ipi_handler()), whose register map and sharing
 * rules this file keeps; the ids come from the same tree's
 * scp/cm4/include/scp_cm4.h.
 *
 * The API the clients see is the BSP one the sensor hub is written against
 * (scp/A_register_notify, ipi send/register, DRAM blocks, awake lock), with
 * the names prefixed scp_cm4_ because the in-tree MT8183 driver already
 * exports scp_ipi_send() for its own rpmsg use.
 *
 * The awake lock is the same hardware as on the MT6885 port
 * (cezanne-linux-port/linux/drivers/scp/scp_cezanne.c): AP sets and clears
 * bit 0 (lock) and 1 (unlock) of the 0xa0 group in infracfg and waits for the
 * SCP to acknowledge through the same register (vendor scp/cm4/scp_awake.c
 * scp_awake_lock()/scp_awake_unlock()).
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include "mtk_scp_cm4.h"

/* scp_reg.h: interrupt and awake registers */
#define SCP_A_TO_HOST_REG	0x001c
#define SCP_GIPC_IN_REG		0x0028
#define SCP_TO_SPM_REG		0x0020
#define SCP_HOST_TO_SCP_BIT	BIT(0)
#define SCP_A_IPI_AWAKE_BIT	BIT(2)

/* infracfg, vendor scp_reg.h INFRA_IRQ_SET/CLEAR */
#define INFRA_IRQ_SET		0x0b14
#define INFRA_IRQ_CLEAR		0x0b18
#define AP_AWAKE_LOCK		0
#define AP_AWAKE_UNLOCK		1
#define AP_AWAKE_MAGIC		0xa0
#define SCP_AWAKE_TIMEOUT	100000

/*
 * DRAM block sizes of the scp_share carriage-out, vendor device tree
 * scp_mem_tbl of this SoC (live DT scp@10500000): sensor 1 MiB, logger 1.5 MiB,
 * audio ipi 100 KiB, vow bargein, and the ones only the vendor drivers use.
 */
static const u32 scp_cm4_mblock_size[SCP_CM4_NUMS_MEM_ID] = {
	[VOW_MEM_ID] = 0x49200,
	[SENS_MEM_ID] = 0x100000,
	[FLP_MEM_ID] = 0x1000,
	[SCP_A_LOGGER_MEM_ID] = 0x180000,
	[AUDIO_IPI_MEM_ID] = 0x19000,
	[VOW_BARGEIN_MEM_ID] = 0x5a00,
};

static void scp_cm4_fromio(void *dst, const void __iomem *src,
			   unsigned int len)
{
	/* SRAM reads are word based; a plain copy is fine on the AP side */
	memcpy_fromio(dst, src, len);
}

/* vendor scp/cm4/v01/scp_ipi.c scp_ipi_send() */
enum scp_ipi_status scp_cm4_ipi_send(enum scp_cm4_ipi_id id, void *buf,
	unsigned int len, unsigned int wait, enum scp_core_id core)
{
	struct scp_cm4 *scp;
	void __iomem *obj;
	u32 rb_id;
	int ret = SCP_IPI_ERROR;

	if (!scp_cm4_scp || core != SCP_A_ID)
		return SCP_IPI_ERROR;
	scp = scp_cm4_scp;

	if (id >= SCP_CM4_NR_IPI || !buf || len > SCP_CM4_SHARE_BUF_SIZE - 16)
		return SCP_IPI_ERROR;
	if (!scp_cm4_is_ready(core))
		return SCP_IPI_NOT_READY;

	/* one message at a time, the firmware has a single object per way */
	if (!mutex_trylock(&scp->ipi_lock))
		return SCP_IPI_BUSY;
	if (scp_cm4_awake_lock(NULL))
		goto out_unlock;

	/* the previous message must have been taken before the next one */
	if (readl(scp->cfg + SCP_GIPC_IN_REG) & SCP_HOST_TO_SCP_BIT) {
		ret = SCP_IPI_BUSY;
		goto out_awake;
	}

	obj = scp->sram + SCP_CM4_SHARE_SEND_OFFSET;
	memcpy_toio(obj + SCP_CM4_SHARE_DATA, buf, len);
	writel(len, obj + SCP_CM4_SHARE_LEN);
	writel(id, obj + SCP_CM4_SHARE_ID);

	/* read the id back: the SCP's SRAM may not have been awake yet */
	rb_id = readl(obj + SCP_CM4_SHARE_ID);
	if (rb_id != id) {
		dev_err(scp->dev, "ipi %d: id read back %d\n", id, rb_id);
		ret = SCP_IPI_ERROR;
		goto out_awake;
	}

	dsb(sy);
	writel(SCP_HOST_TO_SCP_BIT, scp->cfg + SCP_GIPC_IN_REG);

	if (wait) {
		u32 left = SCP_CM4_IPI_BUSY_TIMEOUT_US;

		while ((readl(scp->cfg + SCP_GIPC_IN_REG) &
			SCP_HOST_TO_SCP_BIT) && --left)
			udelay(1);
		if (!left) {
			dev_err(scp->dev, "ipi %d: not taken by the SCP\n", id);
			ret = SCP_IPI_BUSY;
			goto out_awake;
		}
	}

	ret = SCP_IPI_DONE;
out_awake:
	scp_cm4_awake_unlock(NULL);
out_unlock:
	mutex_unlock(&scp->ipi_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(scp_cm4_ipi_send);

enum scp_ipi_status scp_cm4_ipi_registration(enum scp_cm4_ipi_id id,
	void (*handler)(int id, void *data, unsigned int len),
	const char *name)
{
	struct scp_cm4 *scp = scp_cm4_scp;

	if (!scp || id >= SCP_CM4_NR_IPI || !handler)
		return SCP_IPI_ERROR;

	scp->ipi_handler[id] = handler;
	scp->ipi_name[id] = name;
	dev_dbg(scp->dev, "ipi %d registered for %s\n", id, name);

	return SCP_IPI_DONE;
}
EXPORT_SYMBOL_GPL(scp_cm4_ipi_registration);

enum scp_ipi_status scp_cm4_ipi_unregistration(enum scp_cm4_ipi_id id)
{
	struct scp_cm4 *scp = scp_cm4_scp;

	if (!scp || id >= SCP_CM4_NR_IPI)
		return SCP_IPI_ERROR;

	scp->ipi_handler[id] = NULL;
	scp->ipi_name[id] = NULL;
	return SCP_IPI_DONE;
}
EXPORT_SYMBOL_GPL(scp_cm4_ipi_unregistration);

unsigned int scp_cm4_is_ready(enum scp_core_id core)
{
	struct scp_cm4 *scp = scp_cm4_scp;

	return scp && core == SCP_A_ID && scp->ready_seen;
}
EXPORT_SYMBOL_GPL(scp_cm4_is_ready);

void scp_cm4_notify_register(struct notifier_block *nb)
{
	struct scp_cm4 *scp = scp_cm4_scp;

	if (!scp)
		return;
	blocking_notifier_chain_register(&scp->notifier, nb);
	if (scp->ready_seen)
		nb->notifier_call(nb, SCP_EVENT_READY, NULL);
}
EXPORT_SYMBOL_GPL(scp_cm4_notify_register);

void scp_cm4_notify_unregister(struct notifier_block *nb)
{
	struct scp_cm4 *scp = scp_cm4_scp;

	if (!scp)
		return;
	blocking_notifier_chain_unregister(&scp->notifier, nb);
}
EXPORT_SYMBOL_GPL(scp_cm4_notify_unregister);

/*
 * Wake the SCP before the AP touches its mailbox SRAM, and let it sleep again
 * afterwards; the firmware answers on the same register, so a timeout means it
 * is not running (vendor scp/cm4/scp_awake.c).
 */
int scp_cm4_awake_lock(void *unused)
{
	struct scp_cm4 *scp = scp_cm4_scp;
	unsigned long flags;
	u32 status;
	int ret = -1, count = 0;

	if (!scp || !scp->infracfg || !scp->ready_seen)
		return -1;

	spin_lock_irqsave(&scp->awake_lock, flags);
	if (scp->awake_count > 0) {
		scp->awake_count++;
		spin_unlock_irqrestore(&scp->awake_lock, flags);
		return 0;
	}

	regmap_write(scp->infracfg, INFRA_IRQ_SET,
		     AP_AWAKE_MAGIC | BIT(AP_AWAKE_LOCK));
	while (++count != SCP_AWAKE_TIMEOUT) {
		regmap_read(scp->infracfg, INFRA_IRQ_SET, &status);
		if ((status & 0xf0) != AP_AWAKE_MAGIC) {
			dev_notice(scp->dev, "awake: status %#x\n", status);
			break;
		}
		if (!((status & 0x0f) & BIT(AP_AWAKE_LOCK))) {
			ret = 0;
			break;
		}
		udelay(10);
	}
	regmap_write(scp->infracfg, INFRA_IRQ_CLEAR,
		     AP_AWAKE_MAGIC | BIT(AP_AWAKE_LOCK));

	if (!ret)
		scp->awake_count++;
	else
		dev_notice_ratelimited(scp->dev, "awake lock timeout\n");
	spin_unlock_irqrestore(&scp->awake_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(scp_cm4_awake_lock);

int scp_cm4_awake_unlock(void *unused)
{
	struct scp_cm4 *scp = scp_cm4_scp;
	unsigned long flags;
	u32 status;
	int ret = -1, count = 0;

	if (!scp || !scp->infracfg || !scp->ready_seen)
		return -1;

	spin_lock_irqsave(&scp->awake_lock, flags);
	if (scp->awake_count > 1) {
		scp->awake_count--;
		spin_unlock_irqrestore(&scp->awake_lock, flags);
		return 0;
	}

	regmap_write(scp->infracfg, INFRA_IRQ_SET,
		     AP_AWAKE_MAGIC | BIT(AP_AWAKE_UNLOCK));
	while (++count != SCP_AWAKE_TIMEOUT) {
		regmap_read(scp->infracfg, INFRA_IRQ_SET, &status);
		if ((status & 0xf0) != AP_AWAKE_MAGIC)
			break;
		if (!((status & 0x0f) & BIT(AP_AWAKE_UNLOCK))) {
			ret = 0;
			break;
		}
		udelay(10);
	}
	regmap_write(scp->infracfg, INFRA_IRQ_CLEAR,
		     AP_AWAKE_MAGIC | BIT(AP_AWAKE_UNLOCK));

	if (scp->awake_count > 0)
		scp->awake_count--;
	spin_unlock_irqrestore(&scp->awake_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(scp_cm4_awake_unlock);

phys_addr_t scp_cm4_reserve_mem_phys(enum scp_reserve_mem_id_t id)
{
	struct scp_cm4 *scp = scp_cm4_scp;

	if (!scp || id >= SCP_CM4_NUMS_MEM_ID)
		return 0;
	return scp->mblock[id].phys;
}
EXPORT_SYMBOL_GPL(scp_cm4_reserve_mem_phys);

phys_addr_t scp_cm4_reserve_mem_virt(enum scp_reserve_mem_id_t id)
{
	struct scp_cm4 *scp = scp_cm4_scp;

	if (!scp || id >= SCP_CM4_NUMS_MEM_ID)
		return 0;
	return (phys_addr_t)(uintptr_t)scp->mblock[id].virt;
}
EXPORT_SYMBOL_GPL(scp_cm4_reserve_mem_virt);

phys_addr_t scp_cm4_reserve_mem_size(enum scp_reserve_mem_id_t id)
{
	struct scp_cm4 *scp = scp_cm4_scp;

	if (!scp || id >= SCP_CM4_NUMS_MEM_ID)
		return 0;
	return scp->mblock[id].size;
}
EXPORT_SYMBOL_GPL(scp_cm4_reserve_mem_size);

/*
 * The firmware runs at whatever OPP the bootloader left it at, and the sensor
 * hub only needs tens of MHz, so feature bookkeeping is a no-op here (the
 * MT6885 port folds it the same way in scp_cezanne.c).
 */
void scp_cm4_register_feature(enum feature_id id)
{
}
EXPORT_SYMBOL_GPL(scp_cm4_register_feature);

void scp_cm4_deregister_feature(enum feature_id id)
{
}
EXPORT_SYMBOL_GPL(scp_cm4_deregister_feature);

void scp_cm4_wdt_reset(int cpu_id)
{
	struct scp_cm4 *scp = scp_cm4_scp;

	if (scp)
		dev_warn(scp->dev, "wdt reset requested for core %d\n", cpu_id);
}
EXPORT_SYMBOL_GPL(scp_cm4_wdt_reset);

/* SCP to AP: take the object, hand the payload to its owner, answer the SCP */
void scp_cm4_ipi_isr(struct scp_cm4 *scp)
{
	void __iomem *obj = scp->sram + SCP_CM4_SHARE_RECV_OFFSET;
	u32 id = readl(obj + SCP_CM4_SHARE_ID);
	u32 len = readl(obj + SCP_CM4_SHARE_LEN);

	if (id < SCP_CM4_NR_IPI && len <= SCP_CM4_SHARE_BUF_SIZE - 16) {
		scp_cm4_fromio(scp->ipi_recv_buf, obj + SCP_CM4_SHARE_DATA, len);
		if (scp->ipi_handler[id])
			scp->ipi_handler[id](id, scp->ipi_recv_buf, len);
		else
			dev_dbg(scp->dev, "ipi %d (%u bytes) has no handler\n",
				id, len);
	} else {
		dev_notice(scp->dev, "ipi with id %u len %u ignored\n", id, len);
	}

	/* let the SCP go back to idle, it stays busy otherwise (vendor code) */
	writel(SCP_A_IPI_AWAKE_BIT, scp->cfg + SCP_GIPC_IN_REG);
	writel(0x1, scp->cfg + SCP_TO_SPM_REG);
}

void scp_cm4_ipi_reset(struct scp_cm4 *scp)
{
	writel(0, scp->cfg + SCP_GIPC_IN_REG);
	memset_io(scp->sram + SCP_CM4_SHARE_RECV_OFFSET, 0,
		  SCP_CM4_SHARE_BUF_SIZE * 2);
}

/*
 * The sensor FIFO and the logger live in the scp_share carriage-out, which the
 * firmware splits into fixed blocks in this order (vendor
 * scp_reserve_memory_ioremap() + the board's scp_mem_tbl).
 */
int scp_cm4_ipi_init(struct scp_cm4 *scp)
{
	struct device_node *node;
	struct reserved_mem *rmem = NULL;
	phys_addr_t ofs = 0;
	int i;

	mutex_init(&scp->ipi_lock);
	spin_lock_init(&scp->awake_lock);
	BLOCKING_INIT_NOTIFIER_HEAD(&scp->notifier);
	scp->ipi_recv_buf = devm_kmalloc(scp->dev, SCP_CM4_SHARE_BUF_SIZE,
					GFP_KERNEL);
	if (!scp->ipi_recv_buf)
		return -ENOMEM;

	scp->infracfg = ERR_PTR(-ENODEV);
	node = of_parse_phandle(scp->dev->of_node, "mediatek,infracfg", 0);
	if (node) {
		scp->infracfg = syscon_node_to_regmap(node);
		of_node_put(node);
	}
	if (IS_ERR(scp->infracfg)) {
		dev_warn(scp->dev,
			 "no mediatek,infracfg: the awake lock is disabled\n");
		scp->infracfg = NULL;
	}

	node = of_parse_phandle(scp->dev->of_node, "memory-region", 0);
	if (node) {
		rmem = of_reserved_mem_lookup(node);
		of_node_put(node);
	}
	if (!rmem) {
		node = of_find_compatible_node(NULL, NULL,
					       "mediatek,reserve-memory-scp_share");
		rmem = node ? of_reserved_mem_lookup(node) : NULL;
		if (node)
			of_node_put(node);
	}
	if (rmem) {
		scp->share_phys = rmem->base;
		scp->share_size = rmem->size;
		scp->share = devm_memremap(scp->dev, rmem->base, rmem->size,
					   MEMREMAP_WC);
		if (IS_ERR(scp->share)) {
			dev_err(scp->dev, "cannot map scp_share\n");
			scp->share = NULL;
		}
	}

	for (i = 0; i < SCP_CM4_NUMS_MEM_ID; i++) {
		scp->mblock[i].size = scp_cm4_mblock_size[i];
		if (!scp->share)
			continue;
		if (ofs + scp->mblock[i].size > scp->share_size) {
			dev_err(scp->dev,
				"scp_share is %#zx, the block table needs %pa\n",
				scp->share_size, &ofs);
			break;
		}
		scp->mblock[i].phys = scp->share_phys + ofs;
		scp->mblock[i].virt = scp->share + ofs;
		ofs += scp->mblock[i].size;
	}

	if (!scp->share)
		dev_err(scp->dev,
			"no reserved-memory node mediatek,reserve-memory-scp_share: no DRAM for the sensor hub\n");

	return 0;
}
