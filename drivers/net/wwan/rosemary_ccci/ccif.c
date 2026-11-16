// SPDX-License-Identifier: GPL-2.0-only
/*
 * Poco M5s (rosemary): the CCIF control transport.
 *
 * CCIF0 is the AP<->modem doorbell: 16 ring queues in the non-cacheable share
 * memory, one hardware channel per queue, a 512-byte SRAM for the handshake
 * exchange. The protocol (packet framing, ring layout, channel numbers) is
 * MediaTek's and is identical between the MT6785 (generation 6295) and the
 * MT6885 (6297); only the addresses differ.
 *
 * Adapted from the cezanne port's ccif.c (github.com/DrShadow34/
 * cezanne-linux-port, linux/drivers/modem-ccci/ccif.c), which is a working
 * mainline implementation of the same protocol, with every SoC fact replaced
 * by this phone's and checked against the vendor kernel's GPL source for
 * MT6785 (scratch/native/4-modem/src/los-mt6785,
 * drivers/misc/mediatek/eccci/hif/ccci_hif_ccif.c and hif/ccci_ringbuf.c).
 *
 * Three details a fresh implementation gets wrong, kept from cezanne because
 * the vendor source confirms each:
 *  - CCIF SRAM takes 32-bit accesses only; a byte write corrupts the word.
 *  - A channel must not be rung again while APCCIF_BUSY still has its bit, or
 *    the queue wedges.
 *  - RPC and FS replies after the first handshake must carry a clear
 *    assert_bit; the modem reads a set bit on those as an exception.
 */
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "rosemary_ccci.h"

/*
 * Ring sizes per queue, generation >= 6295: ccci_hif_ccif.c:164-190
 * (rx_queue_buffer_size_up_95 / tx_queue_buffer_size_up_95 and the two
 * exception tables), byte for byte.
 */
static const u16 rx_kib[CCIF_QUEUES] = { 80, 80, 40, 80, 20, 20, 64, 0, 8 };
static const u16 tx_kib[CCIF_QUEUES] = { 128, 40, 8, 40, 20, 20, 64, 0, 8 };
static const u16 rx_exp_kib[CCIF_QUEUES] = { 12, 32, 8, 0, 0, 0, 8 };
static const u16 tx_exp_kib[CCIF_QUEUES] = { 12, 32, 8, 0, 0, 0, 8 };

static u32 wrap(u32 pos, u32 len)
{
	return pos >= len ? pos - len : pos;
}

static void ring_copy_in(u8 *ring, u32 len, u32 pos, const void *src, u32 n)
{
	u32 first = min(n, len - pos);

	memcpy(ring + pos, src, first);
	memcpy(ring, src + first, n - first);
}

static void ring_copy_out(void *dst, const u8 *ring, u32 len, u32 pos, u32 n)
{
	u32 first = min(n, len - pos);

	memcpy(dst, ring + pos, first);
	memcpy(dst + first, ring, n - first);
}

static struct ccci_ringbuf *ring_create(void *buf, u32 rx, u32 tx)
{
	u32 len = RINGBUF_CTL_LEN + rx + tx;
	struct ccci_ringbuf *rb = buf + 8;
	u32 *w = buf;

	memset(buf, 0, len);
	w[0] = RBF_HEADER;
	w[1] = RBF_HEADER;
	w = buf + len - 8;
	w[0] = RBF_FOOTER;
	w[1] = RBF_FOOTER;
	rb->rx.length = rx;
	rb->tx.length = tx;
	return rb;
}

/**
 * ccci_ccif_init() - build the ring buffers in the share memory
 * @md: the modem
 *
 * The rings live in the CCISM_MCU region of the non-cacheable window and the
 * exception rings in CCISM_MCU_EXP, at the vendor's offsets for this
 * generation (rosemary_ccci.h: NC_CCISM_MCU, NC_CCISM_MCU_EXP).
 */
/* the bytes the two ring areas need, computed from the tables above */
static u32 ring_area_size(const u16 *rx, const u16 *tx)
{
	u32 total = 0;
	int i;

	for (i = 0; i < CCIF_QUEUES; i++)
		total += RINGBUF_CTL_LEN + (rx[i] + tx[i]) * 1024;
	return total;
}

int ccci_ccif_init(struct ccci_md *md)
{
	void *buf;
	u32 need;
	int i;

	/*
	 * Both ring areas are written blind, so both are bounds-checked first:
	 * a ring that does not fit its region is not built at all, with the
	 * reason in the log, instead of running off the end of the mapping.
	 */
	need = ring_area_size(rx_kib, tx_kib);
	if (!md->nc || NC_CCISM_MCU + need > md->nc_size) {
		dev_err(md->dev, "the CCIF rings need %#x at %#x, outside the mapped %#llx\n",
			need, NC_CCISM_MCU, (u64)md->nc_size);
		return -EINVAL;
	}
	need = ring_area_size(rx_exp_kib, tx_exp_kib);
	if (NC_CCISM_MCU_EXP + need > md->nc_size) {
		dev_err(md->dev, "the CCIF exception rings need %#x at %#x, outside the mapped %#llx\n",
			need, NC_CCISM_MCU_EXP, (u64)md->nc_size);
		return -EINVAL;
	}

	buf = md->nc + NC_CCISM_MCU;
	md->rings_size = 0;
	for (i = 0; i < CCIF_QUEUES; i++) {
		u32 rx = rx_kib[i] * 1024, tx = tx_kib[i] * 1024;
		struct ccci_ringbuf *rb = ring_create(buf, rx, tx);

		md->ring[i] = rb;
		buf += RINGBUF_CTL_LEN + rx + tx;
		md->rings_size += RINGBUF_CTL_LEN + rx + tx;
	}
	buf = md->nc + NC_CCISM_MCU_EXP;
	for (i = 0; i < CCIF_QUEUES; i++) {
		u32 rx = rx_exp_kib[i] * 1024, tx = tx_exp_kib[i] * 1024;

		md->ring_exp[i] = ring_create(buf, rx, tx);
		buf += RINGBUF_CTL_LEN + rx + tx;
	}
	wmb();
	return 0;
}

/* CCIF SRAM takes 32-bit accesses only (vendor hif/ccci_hif_ccif.c) */
void ccci_sram_write(struct ccci_md *md, u32 off, const void *src, size_t len)
{
	const u32 *w = src;
	size_t i;

	for (i = 0; i < len / 4; i++)
		writel(w[i], md->ccif_ap + APCCIF_CHDATA + off + 4 * i);
}

void ccci_sram_read(struct ccci_md *md, u32 off, void *dst, size_t len)
{
	u32 *w = dst;
	size_t i;

	for (i = 0; i < len / 4; i++)
		w[i] = readl(md->ccif_ap + APCCIF_CHDATA + off + 4 * i);
}

/**
 * ccci_ccif_reset() - clear the CCIF state and the SRAM before a boot
 * @md: the modem
 *
 * The trailer the modem reads is three words at the end of the SRAM: a magic,
 * the address of the MDSS debug region in the modem's own view, and its size.
 * The modem writes its boot trace there, which is what makes a silent
 * handshake timeout diagnosable.
 */
void ccci_ccif_reset(struct ccci_md *md)
{
	u32 last = CCIF_SRAM_SIZE - 12;
	int i;

	/* CCIF0/CCIF1 reset, infracfg +0x150/+0x154 (ccif-regs.md) */
	writel(BIT(8) | BIT(7), md->infra + INFRA_RST0_PD);
	writel(BIT(8) | BIT(7), md->infra + INFRA_RST1_PD);
	for (i = 0; i < CCIF_SRAM_SIZE; i += 4) {
		writel(0, md->ccif_ap + APCCIF_CHDATA + i);
		writel(0, md->ccif_md + APCCIF_CHDATA + i);
	}
	writel(0x7274626e, md->ccif_ap + APCCIF_CHDATA + last);
	writel(NC_MD_VIEW + NC_MDSS_DBG, md->ccif_ap + APCCIF_CHDATA + last + 4);
	writel(NC_MDSS_DBG_SIZE, md->ccif_ap + APCCIF_CHDATA + last + 8);
	memset(md->seq, 0, sizeof(md->seq));
}

/* ring one channel; never re-ring one the modem has not taken yet */
void ccci_ccif_send_ch(struct ccci_md *md, unsigned int ch)
{
	if (readl(md->ccif_ap + APCCIF_BUSY) & BIT(ch))
		return;
	writel(BIT(ch), md->ccif_ap + APCCIF_BUSY);
	writel(ch, md->ccif_ap + APCCIF_TCHNUM);
}

/**
 * ccci_send_raw() - put one whole packet into a transmit ring
 * @md: the modem
 * @txq: the ring queue to use
 * @pkt: the packet, starting with its &struct ccci_header
 * @len: packet length
 *
 * Returns @len, -EAGAIN when the ring is full, or -EINVAL.
 */
int ccci_send_raw(struct ccci_md *md, int txq, void *pkt, size_t len)
{
	struct ccci_ringbuf *rb;
	struct ccci_header *h = pkt;
	u32 hdr[2] = { PKG_HEADER, len }, ftr[2] = { PKG_FOOTER, PKG_FOOTER };
	u32 rlen, rd, wr, need, room;
	u8 *ring;
	unsigned long flags;

	if (txq < 0 || txq >= CCIF_QUEUES || len < sizeof(*h))
		return -EINVAL;
	rb = md->ring[txq];
	if (!rb || !rb->tx.length)
		return -EINVAL;
	spin_lock_irqsave(&md->tx_lock[txq], flags);
	rlen = rb->tx.length;
	rd = READ_ONCE(rb->tx.read);
	wr = rb->tx.write;
	need = ALIGN(len + 16, 8);
	room = rd > wr ? rd - wr - 1 : rlen - wr + rd - 1;
	if (need >= room) {
		spin_unlock_irqrestore(&md->tx_lock[txq], flags);
		return -EAGAIN;
	}
	if (h->channel < CCCI_MAX_CH) {
		h->seq_num = md->seq[h->channel]++;
		/* FS and RPC replies after the first handshake: no assert bit */
		h->assert_bit = !((h->channel == CCCI_RPC_TX ||
				   h->channel == CCCI_FS_TX) &&
				  md->state != MD_STATE_HS1);
	}
	ring = rb->buffer + rb->rx.length;
	ring_copy_in(ring, rlen, wr, hdr, 8);
	wr = wrap(wr + 8, rlen);
	ring_copy_in(ring, rlen, wr, pkt, len);
	wr = wrap(wr + ALIGN(len, 8), rlen);
	ring_copy_in(ring, rlen, wr, ftr, 8);
	wr = wrap(wr + 8, rlen);
	mb();
	WRITE_ONCE(rb->tx.write, wr);
	ccci_ccif_send_ch(md, txq);
	spin_unlock_irqrestore(&md->tx_lock[txq], flags);
	return len;
}

int ccci_send(struct ccci_md *md, int txq, u16 channel, u32 data0, u32 data1,
	      u32 reserved, const void *payload, size_t len)
{
	struct ccci_header *h;
	int ret;

	h = kzalloc(sizeof(*h) + len, GFP_KERNEL);
	if (!h)
		return -ENOMEM;
	h->data[0] = data0;
	h->data[1] = data1;
	h->channel = channel;
	h->reserved = reserved;
	if (len)
		memcpy(h + 1, payload, len);
	ret = ccci_send_raw(md, txq, h, sizeof(*h) + len);
	kfree(h);
	return ret;
}

/* one packet out of receive ring @qno: its length, 0 if empty, <0 if broken */
static int ring_read_pkt(struct ccci_md *md, int qno, void **out)
{
	struct ccci_ringbuf *rb = md->ring[qno];
	u32 rlen, rd, wr, hdr[2], ftr[2], avail, plen, total;
	void *pkt;

	if (!rb)
		return 0;
	rlen = rb->rx.length;
	if (!rlen)
		return 0;
	rd = rb->rx.read;
	wr = READ_ONCE(rb->rx.write);
	avail = wr >= rd ? wr - rd : rlen - rd + wr;
	if (avail < 16 + sizeof(struct ccci_header))
		return 0;
	rmb();
	ring_copy_out(hdr, rb->buffer, rlen, rd, 8);
	if (hdr[0] != PKG_HEADER) {
		dev_err(md->dev, "rxq%d: bad header %#x at %u\n", qno, hdr[0], rd);
		return -EIO;
	}
	plen = hdr[1];
	total = ALIGN(plen + 16, 8);
	if (plen < sizeof(struct ccci_header) || total >= rlen) {
		dev_err(md->dev, "rxq%d: bad length %u\n", qno, plen);
		return -EIO;
	}
	if (total > avail)
		return 0;
	ring_copy_out(ftr, rb->buffer, rlen, wrap(rd + total - 8, rlen), 8);
	if (ftr[0] != PKG_FOOTER || ftr[1] != PKG_FOOTER) {
		dev_err(md->dev, "rxq%d: bad footer\n", qno);
		return -EIO;
	}
	pkt = kmalloc(plen, GFP_KERNEL);
	if (!pkt)
		return -ENOMEM;
	ring_copy_out(pkt, rb->buffer, rlen, wrap(rd + 8, rlen), plen);
	mb();
	WRITE_ONCE(rb->rx.read, wrap(rd + total, rlen));
	*out = pkt;
	return plen;
}

static void dispatch(struct ccci_md *md, void *pkt, size_t len)
{
	struct ccci_header *h = pkt;

	if (h->channel == CCCI_CONTROL_RX) {
		ccci_recv_control(md, h, pkt, len);
		return;
	}
	if (!ccci_port_recv(md, h, pkt, len))
		dev_info(md->dev, "rx ch %u len %zu %08x %08x %08x (no port)\n",
			 h->channel, len, h->data[0], h->data[1], h->reserved);
}

void ccci_rx_work(struct work_struct *work)
{
	struct ccci_md *md = container_of(work, struct ccci_md, rx_work);
	int i, n;

	for (i = 0; i < CCIF_CH_MAX; i++) {
		if (!test_and_clear_bit(i, &md->pending))
			continue;
		/*
		 * Above the rings are the modem's crash channels: the record it
		 * wrote, then the handshake stages that expect an answer.
		 */
		if (i > CCIF_CH_SRAM) {
			ccci_ee_on_channel(md, i);
			continue;
		}
		if (i == CCIF_CH_SRAM) {
			/*
			 * Handshake 1 arrives through the SRAM, not a ring: the
			 * modem's feature query sits at SRAM_DL_HEADER.
			 */
			struct {
				struct ccci_header h;
				struct md_query_ap_feature f;
			} __packed q;

			ccci_sram_read(md, SRAM_DL_HEADER, &q, sizeof(q));
			ccci_recv_control(md, &q.h, &q, sizeof(q));
			continue;
		}
		if (i == CCIF_CH_CCB_WAKEUP)
			continue;
		for (;;) {
			void *pkt;

			n = ring_read_pkt(md, i, &pkt);
			if (n <= 0)
				break;
			dispatch(md, pkt, n);
			kfree(pkt);
		}
	}
}

/**
 * ccci_ccif_isr() - the CCIF data interrupt (GIC SPI 174, level low)
 *
 * Reads the channels the modem rang, acknowledges them and defers the work.
 * Level-low means the interrupt stays asserted until APCCIF_ACK is written,
 * which is the first thing done here.
 */
irqreturn_t ccci_ccif_isr(int irq, void *data)
{
	struct ccci_md *md = data;
	u32 ch = readl(md->ccif_ap + APCCIF_RCHNUM);

	if (!ch)
		return IRQ_NONE;
	writel(ch, md->ccif_ap + APCCIF_ACK);
	if (ch & ~GENMASK(CCIF_CH_SRAM, 0)) {
		dev_warn(md->dev, "exception channels %#x\n", ch);
		md->ex_channels |= ch;
	}
	/*
	 * The exception channels above the rings are carried to the work item as
	 * well, or the record the modem left would never be read. The mask keeps
	 * the bitmap inside the bits this driver defines.
	 */
	ch &= GENMASK(CCIF_CH_MAX - 1, 0);
	while (ch) {
		set_bit(__ffs(ch), &md->pending);
		ch &= ch - 1;
	}
	schedule_work(&md->rx_work);
	return IRQ_HANDLED;
}
