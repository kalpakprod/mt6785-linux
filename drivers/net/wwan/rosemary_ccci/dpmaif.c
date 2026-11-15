// SPDX-License-Identifier: GPL-2.0-only
/*
 * Poco M5s (rosemary): the modem's packet data path (DPMAIF).
 *
 * DPMAIF is the hardware between the modem's network interfaces and the AP's
 * DRAM: 22 hardware links, one per modem interface (ccmni0..ccmni20 plus the
 * LAN interface). Downlink the AP lends receive buffers to the hardware through
 * the BAT ring; per packet the hardware writes a message PIT (interface,
 * checksum verdict) followed by payload PITs (buffer index, length, address)
 * into the PIT ring. Uplink the AP posts a message DRB and a payload DRB into
 * queue 0's DRB ring; the hardware's read index frees them.
 *
 * Each link becomes a WWAN network device. The modem picks a fresh interface
 * for every PDP context activation ("+EIF: <tx><if>, ifup" on the AT port), so
 * the bearer moves a link to that interface through
 * /sys/class/net/<link>/md_interface.
 *
 * Adapted from the cezanne port's dpmaif.c (github.com/DrShadow34/
 * cezanne-linux-port, linux/drivers/modem-ccci/dpmaif.c) with this phone's own
 * register windows, taken from the device tree rather than compiled in, and
 * with one whole mechanism removed:
 *
 *   the MT6297 dummy-PIT reorder workaround. That silicon reorders downlink
 *   packets and the AP has to hand "ignore" PIT entries back; MT6785 has no
 *   reorder engine, and the vendor gates all of it behind #ifdef MT6297
 *   (hif/dpmaif_reg.h and hif/dpmaif_drv.c). Keeping cezanne's code would
 *   enable a workaround for hardware that is not here, on a chip whose
 *   capability word says 6 rather than cezanne's 14.
 *
 * The register offsets are the vendor's for this generation: with MT6297
 * undefined, hif/dpmaif_reg.h:52-90 puts the AP's groups at AO base
 * 0x10014000 (UL at +0, DL at +0x400), PD base 0x1022d000 (UL at +0, DL at
 * +0x100, AP misc at +0x400) and the modem's own view at 0x1022c000, which is
 * exactly the seven windows the board's device tree describes.
 */
#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/if_arp.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/wwan.h>

#include "rosemary_ccci.h"

/* AO group: uplink at +0, downlink at +0x400 (dpmaif_reg.h:63-77) */
#define AO_UL_CH_STA(q)		(0x000 + 4 * (q))	/* [31:16] read, words */
#define AO_UL_CHNL_ARB0		0x01c
#define AO_UL_CHNL_CON0(q)	(0x020 + 0x10 * (q))
#define AO_UL_CHNL_CON1(q)	(0x024 + 0x10 * (q))
#define AO_UL_AP_L2TIMR0	0x080
#define AO_UL_AP_L2TIMCR0	0x084
#define AO_UL_AP_L2TIMSR0	0x088
#define AO_UL_APDL_L2TIMR0	0x090
#define AO_UL_APDL_L2TIMCR0	0x094
#define AO_UL_APDL_L2TIMSR0	0x098
#define AO_UL_IP_BUSY_MASK	0x09c
#define AO_DL_PKTINFO_CON0	0x400
#define AO_DL_PKTINFO_CON1	0x404
#define AO_DL_PKTINFO_CON2	0x408
#define AO_DL_RDY_CHK_THRES	0x40c
#define AO_DL_PIT_STA0		0x420
#define AO_DL_PIT_STA1		0x424
#define AO_DL_PIT_STA2		0x428	/* [31:16] read, [15:0] write */
#define AO_MISC_CFG1		0x804
#define AO_REMAP_DOMAIN		0x820
#define AO_REMAP_CACHE		0x824
#define AO_REMAP_BANK(i)	(0x830 + 4 * (i))

/* PD group: uplink at +0, downlink at +0x100, AP misc at +0x400 */
#define PD_UL_ADD_DESC_CH(q)	(0x0b0 + 4 * (q))
#define PD_DL_BAT_INIT		0x100
#define PD_DL_BAT_ADD		0x104
#define PD_DL_BAT_INIT_CON0	0x108
#define PD_DL_BAT_INIT_CON1	0x10c
#define PD_DL_PIT_INIT		0x120
#define PD_DL_PIT_ADD		0x124
#define PD_DL_PIT_INIT_CON0	0x128
#define PD_DL_PIT_INIT_CON1	0x12c
#define PD_DL_PIT_INIT_CON2	0x130
#define PD_DL_DBG_STA1		0x1b4
#define PD_DL_DBG_STA7		0x1cc
#define PD_DL_FIFO_STA		0x358
#define PD_WDMA_WR_CMD_CON0	0x300
#define PD_WDMA_WR_CHNL_CON3	0x314
#define PD_AP_UL_L2TISAR0	0x400
#define PD_AP_DL_L2TISAR0	0x450
#define PD_AP_IP_BUSY		0x460
#define PD_AP_CG_EN		0x468

#define INFRA_PROT_DPMAIF	BIT(10)		/* infracfg_ao_mem + 0 */

#define HW_BUSY			BIT(31)
#define INIT_ALLSET		BIT(0)

#define UL_INT_DONE(q)		BIT(q)
#define UL_INT_LEN_ERR		GENMASK(24, 20)
#define DL_INT_DONE		BIT(0)
#define UL_INT_EN		(GENMASK(4, 0) | GENMASK(24, 10))
#define DL_INT_EN		(BIT(0) | BIT(1) | BIT(6))

/*
 * Sizing, identical to the MT6295 vendor code (hif/dpmaif_drv.h and the
 * cezanne port's values, which the study cross-checked for this generation).
 */
#define BAT_ENTRIES		8192
#define PIT_ENTRIES		(2 * BAT_ENTRIES)
#define DRB_ENTRIES		2048
#define RX_BUFS			1024
#define RX_BUF_SIZE		3584
#define DL_MTU			(3 * 1024 + 8)
#define LINK_MTU_MAX		3000
#define PIT_CHK_NUM		124
#define BAT_CHK_NUM		62
#define RB_PIT_CHK_NUM		64
#define NUM_LINKS		22	/* ccmni0..20 plus ccmni-lan */
#define HW_TIMEOUT_US		20000

/* PIT: 4 words, bit 0 of word 0 tells a message PIT from a payload PIT */
struct dpmaif_pit {
	__le32 w[4];
};
#define PIT_MSG			BIT(0)
#define PIT_BID			GENMASK(15, 3)
#define PIT_LEN			GENMASK(31, 16)
#define PIT_MSG_CSUM		GENMASK(3, 2)
#define PIT_MSG_CH		GENMASK(23, 16)
#define PIT_MSG_DROP		BIT(31)

struct dpmaif_bat {
	__le32 addr, addr_ext;
};

struct dpmaif_drb {
	__le32 w0, w1;
};
#define DRB_MSG			1
#define DRB_CONT		BIT(2)
#define DRB_ADDR_EXT		GENMASK(15, 8)
#define DRB_LEN			GENMASK(31, 16)
#define DRB_MSG_CH		GENMASK(23, 16)

struct ccci_dpmaif {
	struct ccci_md *md;
	struct device *dev;
	void __iomem *ao;		/* AO group: UL and DL */
	void __iomem *pd;		/* PD group: UL, DL and AP misc */
	void __iomem *md_misc;		/* the modem's own view */
	void __iomem *infra_mem;	/* infracfg_ao_mem, one protection bit */
	struct clk_bulk_data *clks;
	int num_clks;
	int irq;
	bool running;

	struct net_device *napi_dev;
	struct napi_struct napi;

	/* downlink */
	struct dpmaif_pit *pit;
	dma_addr_t pit_dma;
	u32 pit_rd;
	struct dpmaif_bat *bat;
	dma_addr_t bat_dma;
	u32 bat_wr, bat_rel;
	struct sk_buff **bat_skb;
	dma_addr_t *bat_skb_dma;
	u8 rx_ch, rx_csum;
	bool rx_drop;

	/* uplink, queue 0 */
	struct dpmaif_drb *drb;
	dma_addr_t drb_dma;
	u32 drb_wr, drb_rel;
	struct sk_buff **drb_skb;
	dma_addr_t *drb_skb_dma;
	spinlock_t tx_lock;
	bool tx_stopped;

	struct net_device __rcu *links[NUM_LINKS];
};

/* private data of one link's netdev */
struct dpmaif_link {
	struct ccci_dpmaif *dp;
	u32 ch;
};

static u32 ring_next(u32 i, u32 size)
{
	return i + 1 == size ? 0 : i + 1;
}

static u32 ring_used(u32 rd, u32 wr, u32 size)
{
	return wr >= rd ? wr - rd : size - rd + wr;
}

static void rmw(void __iomem *reg, u32 clr, u32 set)
{
	writel((readl(reg) & ~clr) | set, reg);
}

/* an *_ADD / *_INIT write holds BUSY until the hardware has taken it */
static int hw_write(void __iomem *reg, u32 val, bool wait_done)
{
	u32 v;
	int ret;

	ret = readl_poll_timeout_atomic(reg, v, !(v & HW_BUSY), 1, HW_TIMEOUT_US);
	if (ret)
		return ret;
	writel(val | HW_BUSY, reg);
	if (!wait_done)
		return 0;
	return readl_poll_timeout_atomic(reg, v, !(v & HW_BUSY), 1, HW_TIMEOUT_US);
}

static void clear_ip_busy(struct ccci_dpmaif *dp)
{
	u32 v = readl(dp->pd + PD_AP_IP_BUSY);

	if (v)
		writel(v, dp->pd + PD_AP_IP_BUSY);
}

static void dl_irq_mask(struct ccci_dpmaif *dp, bool mask)
{
	writel(DL_INT_DONE, dp->ao + (mask ? AO_UL_APDL_L2TIMSR0 :
					   AO_UL_APDL_L2TIMCR0));
	rmw(dp->ao + AO_DL_RDY_CHK_THRES, mask ? 0 : DL_INT_DONE,
	    mask ? DL_INT_DONE : 0);
}

static void ul_irq_mask(struct ccci_dpmaif *dp, bool mask)
{
	writel(UL_INT_DONE(0), dp->ao + (mask ? AO_UL_AP_L2TIMSR0 :
						AO_UL_AP_L2TIMCR0));
}

/* ---- downlink ---------------------------------------------------------- */

static int rx_buf_lend(struct ccci_dpmaif *dp, gfp_t gfp)
{
	u32 i = dp->bat_wr;
	struct sk_buff *skb;
	dma_addr_t addr;

	skb = __dev_alloc_skb(RX_BUF_SIZE, gfp);
	if (!skb)
		return -ENOMEM;
	addr = dma_map_single(dp->dev, skb->data, RX_BUF_SIZE, DMA_FROM_DEVICE);
	if (dma_mapping_error(dp->dev, addr)) {
		dev_kfree_skb_any(skb);
		return -ENOMEM;
	}
	dp->bat[i].addr = cpu_to_le32(lower_32_bits(addr));
	dp->bat[i].addr_ext = cpu_to_le32(upper_32_bits(addr) & 0xff);
	dp->bat_skb[i] = skb;
	dp->bat_skb_dma[i] = addr;
	dp->bat_wr = ring_next(i, BAT_ENTRIES);
	return 0;
}

static u32 rx_refill(struct ccci_dpmaif *dp)
{
	u32 n = 0;

	while (dp->bat_rel != dp->bat_wr && !dp->bat_skb[dp->bat_rel]) {
		if (rx_buf_lend(dp, GFP_ATOMIC))
			break;
		dp->bat_rel = ring_next(dp->bat_rel, BAT_ENTRIES);
		n++;
	}
	return n;
}

static void rx_free_bufs(struct ccci_dpmaif *dp)
{
	u32 i;

	for (i = 0; i < BAT_ENTRIES; i++) {
		if (!dp->bat_skb[i])
			continue;
		dma_unmap_single(dp->dev, dp->bat_skb_dma[i], RX_BUF_SIZE,
				 DMA_FROM_DEVICE);
		dev_kfree_skb_any(dp->bat_skb[i]);
		dp->bat_skb[i] = NULL;
	}
}

/* return consumed PITs and refilled BAT slots to the hardware */
static void rx_notify(struct ccci_dpmaif *dp, u32 pits)
{
	u32 bats = rx_refill(dp);

	if (bats && hw_write(dp->pd + PD_DL_BAT_ADD, bats, false))
		dev_err_ratelimited(dp->dev, "BAT add timeout\n");
	if (pits && hw_write(dp->pd + PD_DL_PIT_ADD, pits, true))
		dev_err_ratelimited(dp->dev, "PIT add timeout\n");
}

static void rx_deliver(struct ccci_dpmaif *dp, struct sk_buff *skb)
{
	struct net_device *ndev = NULL;

	rcu_read_lock();
	if (dp->rx_ch < NUM_LINKS)
		ndev = rcu_dereference(dp->links[dp->rx_ch]);
	if (!ndev) {
		rcu_read_unlock();
		dev_info_ratelimited(dp->dev, "rx for interface %u, no link\n",
				     dp->rx_ch);
		dev_kfree_skb_any(skb);
		return;
	}
	switch (skb->data[0] & 0xf0) {
	case 0x40:
		skb->protocol = htons(ETH_P_IP);
		break;
	case 0x60:
		skb->protocol = htons(ETH_P_IPV6);
		break;
	default:
		DEV_STATS_INC(ndev, rx_errors);
		rcu_read_unlock();
		dev_kfree_skb_any(skb);
		return;
	}
	skb->dev = ndev;
	skb_reset_network_header(skb);
	if (!dp->rx_csum && (ndev->features & NETIF_F_RXCSUM))
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	DEV_STATS_INC(ndev, rx_packets);
	DEV_STATS_ADD(ndev, rx_bytes, skb->len);
	napi_gro_receive(&dp->napi, skb);
	rcu_read_unlock();
}

static void rx_payload(struct ccci_dpmaif *dp, const struct dpmaif_pit *pit)
{
	u32 w0 = le32_to_cpu(pit->w[0]), bid = FIELD_GET(PIT_BID, w0);
	u32 len = FIELD_GET(PIT_LEN, w0);
	u64 addr = (u64)le32_to_cpu(pit->w[2]) << 32 | le32_to_cpu(pit->w[1]);
	struct sk_buff *skb;
	u64 off;

	if (bid >= BAT_ENTRIES || !dp->bat_skb[bid]) {
		dev_err_ratelimited(dp->dev, "PIT %u: bad buffer %u\n",
				    dp->pit_rd, bid);
		return;
	}
	skb = dp->bat_skb[bid];
	dp->bat_skb[bid] = NULL;
	dma_unmap_single(dp->dev, dp->bat_skb_dma[bid], RX_BUF_SIZE,
			 DMA_FROM_DEVICE);
	off = addr - dp->bat_skb_dma[bid];
	if (off + len > RX_BUF_SIZE || dp->rx_drop) {
		if (!dp->rx_drop)
			dev_err_ratelimited(dp->dev,
					    "PIT %u: %#x len %u off %llu dropped\n",
					    dp->pit_rd, w0, len, off);
		dev_kfree_skb_any(skb);
		return;
	}
	skb_reserve(skb, off);
	skb_put(skb, len);
	rx_deliver(dp, skb);
}

static int rx_poll(struct ccci_dpmaif *dp, int budget)
{
	u32 wr = readl(dp->ao + AO_DL_PIT_STA2) & 0xffff, pits = 0;
	int done = 0;

	if (wr >= PIT_ENTRIES)
		return 0;
	while (dp->pit_rd != wr && done < budget) {
		const struct dpmaif_pit *pit = &dp->pit[dp->pit_rd];
		u32 w0 = le32_to_cpu(READ_ONCE(pit->w[0]));

		if (w0 & PIT_MSG) {
			dp->rx_ch = FIELD_GET(PIT_MSG_CH, w0);
			dp->rx_csum = FIELD_GET(PIT_MSG_CSUM, w0);
			dp->rx_drop = w0 & PIT_MSG_DROP;
		} else {
			rx_payload(dp, pit);
			done++;
		}
		dp->pit_rd = ring_next(dp->pit_rd, PIT_ENTRIES);
		if (++pits == 128) {
			rx_notify(dp, pits);
			pits = 0;
		}
	}
	rx_notify(dp, pits);
	return done;
}

/* ---- uplink ------------------------------------------------------------ */

static void tx_release(struct ccci_dpmaif *dp)
{
	u32 rd = (readl(dp->ao + AO_UL_CH_STA(0)) >> 16) / 2;

	if (rd >= DRB_ENTRIES)
		return;
	spin_lock(&dp->tx_lock);
	while (dp->drb_rel != rd) {
		u32 i = dp->drb_rel;
		struct sk_buff *skb = dp->drb_skb[i];

		if (skb) {
			dma_unmap_single(dp->dev, dp->drb_skb_dma[i], skb->len,
					 DMA_TO_DEVICE);
			dev_consume_skb_any(skb);
			dp->drb_skb[i] = NULL;
		}
		dp->drb_rel = ring_next(i, DRB_ENTRIES);
	}
	if (dp->tx_stopped) {
		int i;

		dp->tx_stopped = false;
		rcu_read_lock();
		for (i = 0; i < NUM_LINKS; i++) {
			struct net_device *ndev = rcu_dereference(dp->links[i]);

			if (ndev)
				netif_wake_queue(ndev);
		}
		rcu_read_unlock();
	}
	spin_unlock(&dp->tx_lock);
}

static void tx_free_all(struct ccci_dpmaif *dp)
{
	u32 i;

	for (i = 0; i < DRB_ENTRIES; i++) {
		if (!dp->drb_skb[i])
			continue;
		dma_unmap_single(dp->dev, dp->drb_skb_dma[i],
				 dp->drb_skb[i]->len, DMA_TO_DEVICE);
		dev_kfree_skb_any(dp->drb_skb[i]);
		dp->drb_skb[i] = NULL;
	}
}

static netdev_tx_t link_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct dpmaif_link *l = wwan_netdev_drvpriv(ndev);
	struct ccci_dpmaif *dp = l->dp;
	u32 i, j, len;
	dma_addr_t addr;

	if (!READ_ONCE(dp->running) || skb_linearize(skb) ||
	    skb->len > LINK_MTU_MAX)
		goto drop;
	len = skb->len;
	addr = dma_map_single(dp->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(dp->dev, addr))
		goto drop;

	spin_lock_bh(&dp->tx_lock);
	if (DRB_ENTRIES - 1 - ring_used(dp->drb_rel, dp->drb_wr, DRB_ENTRIES) < 2) {
		dp->tx_stopped = true;
		netif_stop_queue(ndev);
		spin_unlock_bh(&dp->tx_lock);
		dma_unmap_single(dp->dev, addr, len, DMA_TO_DEVICE);
		return NETDEV_TX_BUSY;
	}
	i = dp->drb_wr;
	j = ring_next(i, DRB_ENTRIES);
	dp->drb[i].w0 = cpu_to_le32(DRB_MSG | DRB_CONT |
				    FIELD_PREP(DRB_LEN, len));
	dp->drb[i].w1 = cpu_to_le32(FIELD_PREP(DRB_MSG_CH, READ_ONCE(l->ch)));
	dp->drb[j].w0 = cpu_to_le32(FIELD_PREP(DRB_ADDR_EXT,
					       upper_32_bits(addr) & 0xff) |
				    FIELD_PREP(DRB_LEN, len));
	dp->drb[j].w1 = cpu_to_le32(lower_32_bits(addr));
	dp->drb_skb[j] = skb;
	dp->drb_skb_dma[j] = addr;
	dp->drb_wr = ring_next(j, DRB_ENTRIES);
	/* two DRBs, counted in words */
	if (hw_write(dp->pd + PD_UL_ADD_DESC_CH(0), 4, true))
		dev_err_ratelimited(dp->dev, "DRB add timeout\n");
	spin_unlock_bh(&dp->tx_lock);
	DEV_STATS_INC(ndev, tx_packets);
	DEV_STATS_ADD(ndev, tx_bytes, len);
	return NETDEV_TX_OK;

drop:
	DEV_STATS_INC(ndev, tx_dropped);
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

/* ---- interrupt and NAPI ------------------------------------------------ */

static irqreturn_t dpmaif_isr(int irq, void *data)
{
	struct ccci_dpmaif *dp = data;
	u32 ul = readl(dp->pd + PD_AP_UL_L2TISAR0) &
		 ~readl(dp->ao + AO_UL_AP_L2TIMR0);
	u32 dl = readl(dp->pd + PD_AP_DL_L2TISAR0) &
		 ~readl(dp->ao + AO_UL_APDL_L2TIMR0);
	bool poll = false;

	clear_ip_busy(dp);
	if (ul) {
		writel(ul, dp->pd + PD_AP_UL_L2TISAR0);
		if (ul & UL_INT_LEN_ERR)
			dev_warn_ratelimited(dp->dev, "UL length error %#x\n", ul);
		if (ul & UL_INT_DONE(0)) {
			ul_irq_mask(dp, true);
			poll = true;
		}
	}
	if (dl) {
		writel(dl, dp->pd + PD_AP_DL_L2TISAR0);
		if (dl & ~DL_INT_DONE)
			dev_warn_ratelimited(dp->dev, "DL interrupt %#x\n", dl);
		if (dl & DL_INT_DONE) {
			dl_irq_mask(dp, true);
			poll = true;
		}
	}
	if (poll)
		napi_schedule(&dp->napi);
	return ul || dl ? IRQ_HANDLED : IRQ_NONE;
}

static int dpmaif_poll(struct napi_struct *napi, int budget)
{
	struct ccci_dpmaif *dp = container_of(napi, struct ccci_dpmaif, napi);
	int done;

	tx_release(dp);
	done = rx_poll(dp, budget);
	if (done < budget && napi_complete_done(napi, done)) {
		clear_ip_busy(dp);
		ul_irq_mask(dp, false);
		dl_irq_mask(dp, false);
	}
	return done;
}

/* ---- the WWAN links ---------------------------------------------------- */

static const struct net_device_ops link_ops = {
	.ndo_start_xmit = link_xmit,
};

static void link_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &link_ops;
	ndev->type = ARPHRD_NONE;
	ndev->flags = IFF_POINTOPOINT | IFF_NOARP;
	ndev->mtu = ETH_DATA_LEN;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = LINK_MTU_MAX;
	ndev->tx_queue_len = 1000;
	ndev->features = NETIF_F_RXCSUM | NETIF_F_GRO;
	ndev->hw_features = NETIF_F_RXCSUM;
	ndev->needs_free_netdev = true;
}

static ssize_t md_interface_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dpmaif_link *l = wwan_netdev_drvpriv(to_net_dev(dev));

	return sysfs_emit(buf, "%u\n", READ_ONCE(l->ch));
}

static ssize_t md_interface_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct net_device *ndev = to_net_dev(dev);
	struct dpmaif_link *l = wwan_netdev_drvpriv(ndev);
	struct ccci_dpmaif *dp = l->dp;
	struct net_device *other;
	u32 ch;
	int ret = kstrtou32(buf, 0, &ch);

	if (ret)
		return ret;
	if (ch >= NUM_LINKS)
		return -EINVAL;
	rtnl_lock();
	other = rtnl_dereference(dp->links[ch]);
	if (other && other != ndev) {
		ret = -EBUSY;
	} else if (!other) {
		RCU_INIT_POINTER(dp->links[l->ch], NULL);
		WRITE_ONCE(l->ch, ch);
		rcu_assign_pointer(dp->links[ch], ndev);
	}
	rtnl_unlock();
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(md_interface);

static struct attribute *link_attrs[] = {
	&dev_attr_md_interface.attr,
	NULL,
};

static const struct attribute_group link_attr_group = {
	.attrs = link_attrs,
};

static int link_new(void *ctxt, struct net_device *ndev, u32 if_id,
		    struct netlink_ext_ack *extack)
{
	struct ccci_dpmaif *dp = ctxt;
	struct dpmaif_link *l = wwan_netdev_drvpriv(ndev);
	int ret;

	if (if_id >= NUM_LINKS)
		return -EINVAL;
	if (rtnl_dereference(dp->links[if_id]))
		return -EBUSY;
	l->dp = dp;
	l->ch = if_id;
	ndev->sysfs_groups[0] = &link_attr_group;
	ret = register_netdevice(ndev);
	if (!ret)
		rcu_assign_pointer(dp->links[if_id], ndev);
	return ret;
}

static void link_del(void *ctxt, struct net_device *ndev, struct list_head *head)
{
	struct ccci_dpmaif *dp = ctxt;
	struct dpmaif_link *l = wwan_netdev_drvpriv(ndev);

	RCU_INIT_POINTER(dp->links[l->ch], NULL);
	unregister_netdevice_queue(ndev, head);
}

static const struct wwan_ops dpmaif_wwan_ops = {
	.priv_size = sizeof(struct dpmaif_link),
	.setup = link_setup,
	.newlink = link_new,
	.dellink = link_del,
};

/* ---- start and stop ---------------------------------------------------- */

static void hw_reset(struct ccci_dpmaif *dp)
{
	rmw(dp->infra_mem, INFRA_PROT_DPMAIF, 0);
	/* DPMAIF AO reset BIT(6), PD reset BIT(2): infracfg +0x140..0x154 */
	writel(BIT(6) | BIT(2), dp->md->infra + INFRA_RST0_AO);
	writel(BIT(6) | BIT(2), dp->md->infra + INFRA_RST1_AO);
}

/*
 * The modem's view of its own DRAM for the downlink path: eight banks of
 * 64 MiB counted from its DRAM window, which the loader placed.
 */
static void hw_md_remap(struct ccci_dpmaif *dp)
{
	u32 base = dp->md->md_mem_base;
	int i;

	writel(1 << 8 | 1, dp->ao + AO_REMAP_DOMAIN);
	writel(7 << 8 | 6, dp->ao + AO_REMAP_CACHE);
	for (i = 0; i < 8; i++, base += SZ_64M)
		writel((base >> 24 & 0xff) |
		       ((base + SZ_32M) >> 8 & 0xff0000) | BIT(16) | BIT(0),
		       dp->ao + AO_REMAP_BANK(i));
	rmw(dp->ao + AO_MISC_CFG1, 0, BIT(0));
}

static int hw_irq_init(struct ccci_dpmaif *dp)
{
	u32 v;

	writel(~0, dp->pd + PD_AP_UL_L2TISAR0);
	writel(UL_INT_EN, dp->ao + AO_UL_AP_L2TIMCR0);
	writel((u32)~UL_INT_EN, dp->ao + AO_UL_AP_L2TIMSR0);
	if (readl_poll_timeout(dp->ao + AO_UL_AP_L2TIMR0, v,
			       (v & UL_INT_EN) != UL_INT_EN, 10, HW_TIMEOUT_US))
		return -ETIMEDOUT;
	writel(~0, dp->pd + PD_AP_DL_L2TISAR0);
	writel(DL_INT_EN, dp->ao + AO_UL_APDL_L2TIMCR0);
	writel((u32)~DL_INT_EN, dp->ao + AO_UL_APDL_L2TIMSR0);
	rmw(dp->ao + AO_DL_RDY_CHK_THRES, 0x7f, (u32)~DL_INT_EN & 0x7f);
	if (readl_poll_timeout(dp->ao + AO_UL_APDL_L2TIMR0, v,
			       (v & DL_INT_EN) != DL_INT_EN, 10, HW_TIMEOUT_US))
		return -ETIMEDOUT;
	writel(~0, dp->pd + PD_AP_IP_BUSY);
	writel(0, dp->ao + AO_UL_IP_BUSY_MASK);
	return 0;
}

static int hw_rx_init(struct ccci_dpmaif *dp)
{
	void __iomem *ao = dp->ao, *pd = dp->pd;
	int i, ret;

	rmw(ao + AO_DL_PKTINFO_CON0, GENMASK(15, 8), (64 / 16) << 8);
	rmw(ao + AO_DL_PKTINFO_CON2, GENMASK(16, 8) | GENMASK(7, 0),
	    (RX_BUF_SIZE / 128) << 8);
	rmw(ao + AO_DL_PKTINFO_CON0, GENMASK(31, 16), 1 << 16);
	/* 64-byte aligned packets */
	rmw(ao + AO_DL_RDY_CHK_THRES, GENMASK(23, 22), BIT(23));
	writel(DL_MTU, ao + AO_DL_PKTINFO_CON1);
	rmw(ao + AO_DL_PKTINFO_CON2, GENMASK(31, 24), PIT_CHK_NUM << 24);
	rmw(ao + AO_DL_RDY_CHK_THRES, GENMASK(21, 16), BAT_CHK_NUM << 16);
	rmw(ao + AO_DL_PKTINFO_CON0, GENMASK(7, 0), RB_PIT_CHK_NUM);
	rmw(pd + PD_DL_BAT_INIT_CON1, 0, BIT(22));
	rmw(ao + AO_DL_RDY_CHK_THRES, 0, BIT(13));

	writel(lower_32_bits(dp->bat_dma), pd + PD_DL_BAT_INIT_CON0);
	rmw(pd + PD_DL_BAT_INIT_CON1, GENMASK(31, 24) | GENMASK(16, 0),
	    (upper_32_bits(dp->bat_dma) & 0xff) << 24 | BAT_ENTRIES);
	writel(lower_32_bits(dp->pit_dma), pd + PD_DL_PIT_INIT_CON0);
	rmw(pd + PD_DL_PIT_INIT_CON1, GENMASK(31, 24) | GENMASK(15, 0),
	    (upper_32_bits(dp->pit_dma) & 0xff) << 24 | PIT_ENTRIES);
	ret = hw_write(pd + PD_DL_BAT_INIT, INIT_ALLSET, true);
	if (!ret)
		ret = hw_write(pd + PD_DL_PIT_INIT, INIT_ALLSET, true);
	if (ret)
		return ret;
	/* the hardware checks the checksum */
	rmw(ao + AO_DL_RDY_CHK_THRES, 0, BIT(31));

	for (i = 0; i < RX_BUFS; i++) {
		ret = rx_buf_lend(dp, GFP_KERNEL);
		if (ret)
			return ret;
	}
	ret = hw_write(pd + PD_DL_BAT_ADD, RX_BUFS, false);
	if (ret)
		return ret;
	rmw(pd + PD_DL_BAT_INIT_CON1, 0, BIT(16));
	return hw_write(pd + PD_DL_BAT_INIT, 0, true);
}

static void hw_tx_init(struct ccci_dpmaif *dp)
{
	writel(lower_32_bits(dp->drb_dma), dp->ao + AO_UL_CHNL_CON0(0));
	rmw(dp->ao + AO_UL_CHNL_CON1(0), GENMASK(31, 24) | GENMASK(15, 0),
	    (upper_32_bits(dp->drb_dma) & 0xff) << 24 | DRB_ENTRIES * 2);
	rmw(dp->ao + AO_UL_CHNL_ARB0, 0, BIT(0) | BIT(8));
}

static int hw_init(struct ccci_dpmaif *dp)
{
	int ret;

	hw_reset(dp);
	rx_free_bufs(dp);
	tx_free_all(dp);
	memset(dp->pit, 0, PIT_ENTRIES * sizeof(*dp->pit));
	memset(dp->bat, 0, BAT_ENTRIES * sizeof(*dp->bat));
	memset(dp->drb, 0, DRB_ENTRIES * sizeof(*dp->drb));
	dp->pit_rd = dp->bat_wr = dp->bat_rel = 0;
	dp->drb_wr = dp->drb_rel = 0;
	dp->rx_drop = dp->tx_stopped = false;

	rmw(dp->infra_mem, 0, INFRA_PROT_DPMAIF);
	writel(0x7f, dp->pd + PD_AP_CG_EN);
	rmw(dp->pd + PD_WDMA_WR_CHNL_CON3, GENMASK(31, 28), 0xe << 28);
	rmw(dp->pd + PD_WDMA_WR_CMD_CON0, GENMASK(15, 0), 0);
	hw_md_remap(dp);
	writel(BIT(9) | BIT(10) | BIT(15) | BIT(16), dp->ao + 0x08c);
	ret = hw_irq_init(dp);
	if (!ret)
		ret = hw_rx_init(dp);
	return ret;
}

int ccci_dpmaif_start(struct ccci_md *md)
{
	struct ccci_dpmaif *dp = md->dp;
	int ret, i;

	if (!dp || dp->running)
		return 0;
	ret = clk_bulk_prepare_enable(dp->num_clks, dp->clks);
	if (ret)
		return ret;
	/*
	 * About one boot in five the first downlink BAT init write stays busy
	 * (0x80000001); the polls do not clear it, a reset and a second init
	 * later do. Without a data path the modem never reaches its second
	 * handshake, so it is worth retrying here.
	 */
	for (i = 0; i < 10; i++) {
		ret = hw_init(dp);
		if (ret != -ETIMEDOUT)
			break;
		if (!i)
			dev_warn(dp->dev, "DPMAIF init timed out, retrying\n");
		msleep(200);
	}
	if (i && !ret)
		dev_info(dp->dev, "DPMAIF init done at attempt %d\n", i + 1);
	if (ret) {
		dev_err(dp->dev, "DPMAIF init: %d\n", ret);
		hw_reset(dp);
		rx_free_bufs(dp);
		clk_bulk_disable_unprepare(dp->num_clks, dp->clks);
		return ret;
	}
	hw_tx_init(dp);
	napi_enable(&dp->napi);
	WRITE_ONCE(dp->running, true);
	enable_irq(dp->irq);
	return 0;
}

void ccci_dpmaif_stop(struct ccci_md *md)
{
	struct ccci_dpmaif *dp = md->dp;

	if (!dp || !dp->running)
		return;
	WRITE_ONCE(dp->running, false);
	disable_irq(dp->irq);
	napi_disable(&dp->napi);
	spin_lock_bh(&dp->tx_lock);	/* let an xmit in flight finish */
	spin_unlock_bh(&dp->tx_lock);
	hw_reset(dp);
	rx_free_bufs(dp);
	tx_free_all(dp);
	clk_bulk_disable_unprepare(dp->num_clks, dp->clks);
}

static int dpmaif_probe_one(struct ccci_md *md, struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ccci_dpmaif *dp;
	int ret;

	dp = devm_kzalloc(dev, sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;
	dp->md = md;
	dp->dev = dev;
	spin_lock_init(&dp->tx_lock);
	dp->ao = devm_platform_ioremap_resource_byname(pdev, "ao-ul");
	if (IS_ERR(dp->ao))
		return PTR_ERR(dp->ao);
	dp->pd = devm_platform_ioremap_resource_byname(pdev, "pd-ul");
	if (IS_ERR(dp->pd))
		return PTR_ERR(dp->pd);
	dp->md_misc = devm_platform_ioremap_resource_byname(pdev, "pd-md-misc");
	if (IS_ERR(dp->md_misc))
		return PTR_ERR(dp->md_misc);
	dp->infra_mem = devm_ioremap(dev, 0x10002000, 0x1000);
	if (!dp->infra_mem)
		return -ENOMEM;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret)
		return ret;
	dp->pit = dmam_alloc_coherent(dev, PIT_ENTRIES * sizeof(*dp->pit),
				      &dp->pit_dma, GFP_KERNEL);
	dp->bat = dmam_alloc_coherent(dev, BAT_ENTRIES * sizeof(*dp->bat),
				      &dp->bat_dma, GFP_KERNEL);
	dp->drb = dmam_alloc_coherent(dev, DRB_ENTRIES * sizeof(*dp->drb),
				      &dp->drb_dma, GFP_KERNEL);
	dp->bat_skb = devm_kcalloc(dev, BAT_ENTRIES, sizeof(*dp->bat_skb),
				   GFP_KERNEL);
	dp->bat_skb_dma = devm_kcalloc(dev, BAT_ENTRIES,
				       sizeof(*dp->bat_skb_dma), GFP_KERNEL);
	dp->drb_skb = devm_kcalloc(dev, DRB_ENTRIES, sizeof(*dp->drb_skb),
				   GFP_KERNEL);
	dp->drb_skb_dma = devm_kcalloc(dev, DRB_ENTRIES,
				       sizeof(*dp->drb_skb_dma), GFP_KERNEL);
	if (!dp->pit || !dp->bat || !dp->drb || !dp->bat_skb ||
	    !dp->bat_skb_dma || !dp->drb_skb || !dp->drb_skb_dma)
		return -ENOMEM;

	dp->num_clks = devm_clk_bulk_get_all(dev, &dp->clks);
	if (dp->num_clks < 0)
		return dev_err_probe(dev, dp->num_clks, "the DPMAIF clocks\n");
	dp->irq = platform_get_irq(pdev, 0);
	if (dp->irq < 0)
		return dp->irq;
	ret = devm_request_irq(dev, dp->irq, dpmaif_isr, IRQF_NO_AUTOEN,
			       "dpmaif", dp);
	if (ret)
		return ret;

	dp->napi_dev = alloc_netdev_dummy(0);
	if (!dp->napi_dev)
		return -ENOMEM;
	netif_napi_add(dp->napi_dev, &dp->napi, dpmaif_poll);
	ret = wwan_register_ops(dev, &dpmaif_wwan_ops, dp, 0);
	if (ret) {
		netif_napi_del(&dp->napi);
		free_netdev(dp->napi_dev);
		return ret;
	}
	md->dp = dp;
	return 0;
}

/* the DPMAIF node is the modem's data path: bind it with its parent's probe */
int rosemary_ccci_dpmaif_init(struct ccci_md *md)
{
	struct platform_device *pdev;

	pdev = of_find_device_by_node(of_parse_phandle(md->dev->of_node,
						       "mediatek,dpmaif", 0));
	if (!pdev)
		return -ENODEV;
	return dpmaif_probe_one(md, pdev);
}

void rosemary_ccci_dpmaif_exit(struct ccci_md *md)
{
	struct ccci_dpmaif *dp = md->dp;

	if (!dp)
		return;
	wwan_unregister_ops(dp->dev);
	netif_napi_del(&dp->napi);
	free_netdev(dp->napi_dev);
	rx_free_bufs(dp);
	tx_free_all(dp);
	md->dp = NULL;
}
