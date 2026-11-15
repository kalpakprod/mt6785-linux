// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CAMSV capture driver for MT6785 (Poco M5s, rosemary).
 *
 * CAMSV is the small capture engine next to the closed ISP: a timing
 * generator, a packer and one DMA (IMGO) that writes a raw frame into memory.
 * It has no demosaic and no statistics; it exists to get sensor frames out of
 * the SoC without the vendor ISP firmware. On MT6785 six engines sit at
 * 0x1a050000 + 0x1000 * n (live device tree camsv1@1a050000 ... camsv6@1a055000),
 * which is the ISP5-family block the cezanne MT6885 port described (there at
 * 0x1a090000); the register layout and every offset below are from that port's
 * drivers/camera/cezanne_camsv.c and its README, whose register map came from
 * the vendor HAL binary (libimageio_plat_drv.so, CAMSV_TOP_CTRL / CAMSV_BUF_CTRL).
 *
 * Scope of this first version, decided by the card (step 4) and the study
 * (poco-m5s/research/native/5-cameras.md section 4.5):
 *
 *  - physical addressing: buffers come from CMA below 4 GiB through vb2
 *    dma-contig with a 32-bit DMA mask, and the SMI larb port is left in MMU
 *    bypass. No IOMMU, no IOVA, nothing to map. Which larb/port a CAMSV IMGO
 *    uses is not documented for MT6785, so this avoids the question entirely
 *    for the first milestone.
 *  - one video node per engine, and one engine per streaming channel, claimed
 *    at stream-on, so two cameras can stream at once on two engines.
 *  - the format is not free: a CAMSV engine captures what the sensor and the
 *    SENINF route send it, so the frame size is set by the sensor mode. The
 *    three sparseness the hardware offers are RAW8 (the top 8 of 10 bits) and
 *    RAW10 unpacked (16 bits per pixel), never MediaTek's packed RAW10, which
 *    nothing in userspace reads.
 *
 * Three operational traps are handled explicitly, all of them paid for in the
 * cezanne port and none of them obvious from the register map:
 *  1. DMA and format registers latch only when the engine starts, so the setup
 *     runs before the timing generator is released.
 *  2. The IMGO error bits are level interrupts; an unmasked one storms the
 *     interrupt line, so they are reported once and then masked.
 *  3. Something writes to CAMSV_FRAME_WRITE_BASE once per frame. At 0 that is
 *     a write per frame to physical 0, so a scratch buffer is handed to it.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <media/media-device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#define CAMSV_MODULE_EN		0x0010
#define   MODULE_TG_EN		BIT(0)
#define   MODULE_PAK_EN		BIT(2)
#define   MODULE_IMGO_EN	BIT(4)
#define   MODULE_DEFAULT	0x50000000
#define CAMSV_FMT_SEL		0x0014	/* 0 RAW8, 1 RAW10, 2 RAW12 */
#define CAMSV_INT_EN		0x0018
#define CAMSV_INT_STATUS	0x001c	/* read clear */
#define   INT_VS1		BIT(0)
#define   INT_TG_ERR		BIT(4)
#define   INT_TG_GBERR		BIT(5)
#define   INT_SOF		BIT(6)
#define   INT_PASS1_DONE	(BIT(10) | BIT(11) | BIT(12))
#define   INT_IMGO_ERR		BIT(16)
#define   INT_IMGO_OVERRUN	BIT(17)
#define   INT_ERRORS		(INT_TG_ERR | INT_TG_GBERR | INT_IMGO_ERR | \
				 INT_IMGO_OVERRUN)
#define CAMSV_SW_CTL		0x0020
#define   SW_RST_TRIG		BIT(0)
#define   SW_RST_ST		BIT(1)	/* set: the DMA is idle */
#define   SW_HW_RST		BIT(2)
#define CAMSV_IMGO_FBC		0x002c	/* bit 0: RCNT_INC */
#define CAMSV_CLK_EN		0x0030
#define   CLK_EN_TG_PAK_DMA	0x8005
#define CAMSV_PAK		0x004c	/* [4:0] mode, [6:5] pixel mode */
#define   PAK_RAW8		8
#define   PAK_RAW10_UNPACKED	13	/* 16 bits per pixel, LSB aligned */
#define CAMSV_TG_SEN_MODE	0x0130
#define   SEN_MODE_CMOS_EN	BIT(0)
#define   SEN_MODE_TIME_STP_EN	BIT(16)
#define CAMSV_TG_VF_CON		0x0134
#define CAMSV_TG_GRAB_PXL	0x0138
#define CAMSV_TG_GRAB_LIN	0x013c
#define CAMSV_TG_PATH_CFG	0x0140
#define CAMSV_TG_INTER_ST	0x016c
#define   TG_STATE		GENMASK(13, 8)
#define   TG_STATE_IDLE		0x100
#define CAMSV_IMGO_BASE		0x0420
#define CAMSV_IMGO_BASE_MSB	0x0424
#define CAMSV_IMGO_OFST		0x0428
#define CAMSV_IMGO_XSIZE	0x0430	/* bytes per line minus one */
#define CAMSV_IMGO_YSIZE	0x0434
#define CAMSV_IMGO_STRIDE	0x0438
#define CAMSV_IMGO_ERR_STAT	0x0760
#define CAMSV_DMA_DB_EN		0x0800	/* bit 0: per-frame address latch */
#define CAMSV_FRAME_WRITE_BASE	0x0804	/* written once per frame */
#define CAMSV_FBC_IMGO_CTL1	0x0a00
#define   FBC_MODE		BIT(11)
#define   FBC_EN		BIT(15)
#define CAMSV_FBC_IMGO_CTL2	0x0a04	/* [15:8] frame write count */

/* 64: what the software ISP and the GPU import linear buffers with */
#define CAMSV_STRIDE(w)		ALIGN(w, 64)

/* the frame in progress when the engine is stopped arrives within one frame */
#define CAMSV_TG_IDLE_TIMEOUT_US	500000

/**
 * struct camsv_fmt - one pixel format the engine can produce
 * @fourcc: V4L2 pixel format
 * @raw8: the engine is set to RAW8 (the top 8 of the 10 bits)
 * @bytes: bytes per pixel written to memory
 */
struct camsv_fmt {
	u32 fourcc;
	bool raw8;
	unsigned int bytes;
};

static const struct camsv_fmt camsv_formats[] = {
	{ .fourcc = V4L2_PIX_FMT_SBGGR10, .raw8 = false, .bytes = 2 },
	{ .fourcc = V4L2_PIX_FMT_SBGGR8,  .raw8 = true,  .bytes = 1 },
	{ .fourcc = V4L2_PIX_FMT_SGBRG10, .raw8 = false, .bytes = 2 },
	{ .fourcc = V4L2_PIX_FMT_SGBRG8,  .raw8 = true,  .bytes = 1 },
	{ .fourcc = V4L2_PIX_FMT_SGRBG10, .raw8 = false, .bytes = 2 },
	{ .fourcc = V4L2_PIX_FMT_SGRBG8,  .raw8 = true,  .bytes = 1 },
	{ .fourcc = V4L2_PIX_FMT_SRGGB10, .raw8 = false, .bytes = 2 },
	{ .fourcc = V4L2_PIX_FMT_SRGGB8,  .raw8 = true,  .bytes = 1 },
};

struct camsv_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	void *va;			/* kernel mapping, for the frame markers */
};

/**
 * struct camsv_dev - one CAMSV engine
 * @dev: the platform device
 * @base: register window
 * @irq: the engine's interrupt
 * @clks: the engine's clock (cam_sv0 / cam_sv1 on MT6785)
 * @vdev: the video node
 * @pad: the media pad the SENINF entity feeds
 * @queue: vb2 queue, dma-contig
 * @vlock: serialises ioctls and stream on/off
 * @qlock: protects the buffer lists and the streaming state (irq context)
 * @queued: buffers waiting for the DMA
 * @inflight: buffers the DMA was given, in the order it takes them
 * @streaming: the engine is capturing
 * @latch_pending: the last address written has not latched at SOF yet
 * @skip: completions to drop after stream-on (the first one is empty)
 * @wcnt: last frame write count read from the engine
 * @sequence: frame counter handed to userspace
 * @errors: error bits already reported (they are level, report once)
 * @unwritten: completions dropped because the buffer was not written
 * @fmt: the selected format
 * @width, @height: the frame geometry, from the DT or from try_fmt
 * @scratch: the buffer CAMSV_FRAME_WRITE_BASE points at
 * @scratch_dma: its DMA address
 */
struct camsv_dev {
	struct device *dev;
	void __iomem *base;
	int irq;
	struct clk_bulk_data *clks;
	int num_clks;

	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct media_pad pad;
	struct vb2_queue queue;
	struct mutex vlock;	/* serialises ioctls and stream on/off */

	spinlock_t qlock;	/* the two lists and the streaming state, irq */
	struct list_head queued;
	struct list_head inflight;
	bool streaming;
	bool latch_pending;
	u8 skip;
	u8 wcnt;
	u32 sequence;
	u32 errors;
	u32 unwritten;

	const struct camsv_fmt *fmt;
	unsigned int width;
	unsigned int height;

	void *scratch;
	dma_addr_t scratch_dma;

};

static inline struct camsv_buffer *to_camsv_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct camsv_buffer, vb);
}

static unsigned int camsv_line_bytes(const struct camsv_dev *camsv)
{
	return camsv->width * camsv->fmt->bytes;
}

static unsigned int camsv_sizeimage(const struct camsv_dev *camsv)
{
	return CAMSV_STRIDE(camsv_line_bytes(camsv)) * camsv->height;
}

/*
 * The first buffer handed to the DMA after stream-on is never written: it is
 * dropped and requeued rather than delivered, because with the buffers a camera
 * gets, an untouched buffer is leftover memory, not a frame. Later completions
 * are checked with a marker instead, a pattern no written frame can contain.
 */
static const u16 camsv_mark_words[] = {
	0xffff, 0xdead, 0xbeef, 0xfeed, 0xcafe, 0xf00d, 0xd00d, 0xface,
	0xc0de, 0xb0b0, 0xe1e1, 0xd1ce, 0xbabe, 0xfade, 0xc0c0, 0xdeaf,
};

#define CAMSV_MARK_BYTES	sizeof(camsv_mark_words)

static unsigned int camsv_mark_last(const struct camsv_dev *camsv)
{
	return CAMSV_STRIDE(camsv_line_bytes(camsv)) * (camsv->height - 1) +
	       camsv_line_bytes(camsv) - CAMSV_MARK_BYTES;
}

static void camsv_mark_buffer(struct camsv_dev *camsv,
			      struct camsv_buffer *buf)
{
	struct vb2_buffer *vb = &buf->vb.vb2_buf;
	dma_addr_t dma = vb2_dma_contig_plane_dma_addr(vb, 0);
	unsigned int last = camsv_mark_last(camsv);

	if (!buf->va)
		return;
	memcpy(buf->va, camsv_mark_words, CAMSV_MARK_BYTES);
	memcpy(buf->va + last, camsv_mark_words, CAMSV_MARK_BYTES);
	dma_sync_single_range_for_device(vb->vb2_queue->dev, dma, 0,
					 CAMSV_MARK_BYTES, DMA_FROM_DEVICE);
	dma_sync_single_range_for_device(vb->vb2_queue->dev, dma, last,
					 CAMSV_MARK_BYTES, DMA_FROM_DEVICE);
}

static bool camsv_buffer_written(struct camsv_dev *camsv,
				 struct camsv_buffer *buf)
{
	struct vb2_buffer *vb = &buf->vb.vb2_buf;
	dma_addr_t dma = vb2_dma_contig_plane_dma_addr(vb, 0);
	unsigned int last = camsv_mark_last(camsv);

	if (!buf->va)
		return true;
	dma_sync_single_range_for_cpu(vb->vb2_queue->dev, dma, 0,
				      CAMSV_MARK_BYTES, DMA_FROM_DEVICE);
	dma_sync_single_range_for_cpu(vb->vb2_queue->dev, dma, last,
				      CAMSV_MARK_BYTES, DMA_FROM_DEVICE);
	return memcmp(buf->va, camsv_mark_words, CAMSV_MARK_BYTES) &&
	       memcmp(buf->va + last, camsv_mark_words, CAMSV_MARK_BYTES);
}

/* -------------------------------------------------------------------------
 * Hardware
 */

/* Hands the DMA the next queued buffer. qlock held. */
static void camsv_program_next(struct camsv_dev *camsv)
{
	struct camsv_buffer *buf;
	dma_addr_t addr;

	if (!camsv->streaming || camsv->latch_pending || list_empty(&camsv->queued))
		return;

	buf = list_first_entry(&camsv->queued, struct camsv_buffer, list);
	list_move_tail(&buf->list, &camsv->inflight);
	addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
	writel(lower_32_bits(addr), camsv->base + CAMSV_IMGO_BASE);
	writel(upper_32_bits(addr), camsv->base + CAMSV_IMGO_BASE_MSB);
	/* ask for the address to be latched at the next start of frame */
	writel(readl(camsv->base + CAMSV_IMGO_FBC) | BIT(0),
	       camsv->base + CAMSV_IMGO_FBC);
	camsv->latch_pending = true;
}

static irqreturn_t camsv_irq(int irq, void *data)
{
	struct camsv_dev *camsv = data;
	struct camsv_buffer *buf;
	u32 st;
	u8 wcnt, n;

	spin_lock(&camsv->qlock);

	/*
	 * Read-clear and level-triggered: the register must be read on every
	 * interrupt, streaming or not, or the line stays asserted and the core
	 * reports "nobody cared" a hundred thousand times before disabling the
	 * IRQ.
	 */
	st = readl(camsv->base + CAMSV_INT_STATUS);
	if (!st) {
		spin_unlock(&camsv->qlock);
		return IRQ_NONE;
	}

	if (!camsv->streaming) {
		/* idle: the read above acknowledged whatever was pending */
		camsv->errors |= st & INT_ERRORS;
		spin_unlock(&camsv->qlock);
		return IRQ_HANDLED;
	}

	if (st & INT_ERRORS) {
		/*
		 * Level-triggered: acknowledge by reporting once and masking,
		 * otherwise the line stays up and the phone livelocks.
		 */
		if (!(camsv->errors & st))
			dev_warn_ratelimited(camsv->dev,
					     "error status %#x, IMGO %#x\n", st,
					     readl(camsv->base + CAMSV_IMGO_ERR_STAT));
		camsv->errors |= st & INT_ERRORS;
		writel(readl(camsv->base + CAMSV_INT_EN) & ~(st & INT_ERRORS),
		       camsv->base + CAMSV_INT_EN);
	}

	if (st & INT_SOF) {
		camsv->sequence++;
		camsv->latch_pending = false;
	}

	/* completed frames arrive in the order the DMA was given them */
	wcnt = readl(camsv->base + CAMSV_FBC_IMGO_CTL2) >> 8;
	n = wcnt - camsv->wcnt;
	camsv->wcnt = wcnt;

	while (n-- && !list_empty(&camsv->inflight)) {
		buf = list_first_entry(&camsv->inflight, struct camsv_buffer, list);

		if (camsv->skip) {
			camsv->skip--;
			camsv_mark_buffer(camsv, buf);
			list_move_tail(&buf->list, &camsv->queued);
			continue;
		}
		if (!camsv_buffer_written(camsv, buf)) {
			camsv->unwritten++;
			camsv_mark_buffer(camsv, buf);
			list_move_tail(&buf->list, &camsv->queued);
			continue;
		}

		list_del(&buf->list);
		buf->vb.vb2_buf.timestamp = ktime_get_ns();
		buf->vb.sequence = camsv->sequence;
		buf->vb.field = V4L2_FIELD_NONE;
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}

	camsv_program_next(camsv);
	spin_unlock(&camsv->qlock);

	return IRQ_HANDLED;
}

/*
 * Bring the engine up with the first buffer already programmed. Everything
 * that latches at start (format, sizes, address) is written here, before the
 * timing generator is released: none of it can be changed while streaming.
 */
static void camsv_hw_start(struct camsv_dev *camsv)
{
	void __iomem *base = camsv->base;
	unsigned int line = camsv_line_bytes(camsv);

	/* reset the block, and wait for the DMA to report itself idle */
	writel(SW_HW_RST, base + CAMSV_SW_CTL);
	writel(0, base + CAMSV_SW_CTL);
	writel(SW_RST_TRIG, base + CAMSV_SW_CTL);
	readl_poll_timeout(base + CAMSV_SW_CTL, line, line & SW_RST_ST, 10,
			   10000);

	writel(SEN_MODE_TIME_STP_EN, base + CAMSV_TG_SEN_MODE);
	writel(camsv->width << 16, base + CAMSV_TG_GRAB_PXL);
	writel(camsv->height << 16, base + CAMSV_TG_GRAB_LIN);
	writel(0, base + CAMSV_TG_PATH_CFG);

	readl(base + CAMSV_INT_STATUS);
	writel(INT_VS1 | INT_SOF | INT_PASS1_DONE | INT_ERRORS,
	       base + CAMSV_INT_EN);

	writel(camsv->fmt->raw8 ? 0 : 1, base + CAMSV_FMT_SEL);
	writel(CLK_EN_TG_PAK_DMA, base + CAMSV_CLK_EN);
	writel(camsv->fmt->raw8 ? PAK_RAW8 : PAK_RAW10_UNPACKED, base + CAMSV_PAK);

	writel(1, base + CAMSV_DMA_DB_EN);
	writel(0, base + CAMSV_IMGO_OFST);
	writel(line - 1, base + CAMSV_IMGO_XSIZE);
	writel(camsv->height - 1, base + CAMSV_IMGO_YSIZE);
	writel(CAMSV_STRIDE(line), base + CAMSV_IMGO_STRIDE);
	/* the per-frame write CAMSV does on its own goes to scratch memory */
	writel(lower_32_bits(camsv->scratch_dma), base + CAMSV_FRAME_WRITE_BASE);

	writel(MODULE_DEFAULT | MODULE_TG_EN | MODULE_PAK_EN,
	       base + CAMSV_MODULE_EN);
	writel(SEN_MODE_TIME_STP_EN | SEN_MODE_CMOS_EN, base + CAMSV_TG_SEN_MODE);
	writel(MODULE_DEFAULT | MODULE_TG_EN | MODULE_PAK_EN | MODULE_IMGO_EN,
	       base + CAMSV_MODULE_EN);

	writel(FBC_MODE, base + CAMSV_FBC_IMGO_CTL1);
	writel(FBC_EN | FBC_MODE, base + CAMSV_FBC_IMGO_CTL1);
}

/*
 * Stop in the vendor's order: the timing generator first, waiting for the frame
 * in progress to end, then the DMA reset handshake. Clearing the enables first
 * leaves a frame half-written.
 */
static void camsv_hw_stop(struct camsv_dev *camsv)
{
	void __iomem *base = camsv->base;
	u32 st;

	writel(0, base + CAMSV_INT_EN);
	readl(base + CAMSV_INT_STATUS);
	writel(0, base + CAMSV_TG_VF_CON);

	if (readl_poll_timeout(base + CAMSV_TG_INTER_ST, st,
			       (st & TG_STATE) == TG_STATE_IDLE, 1000,
			       CAMSV_TG_IDLE_TIMEOUT_US))
		dev_warn(camsv->dev, "timing generator not idle (%#x)\n", st);

	writel(0, base + CAMSV_TG_SEN_MODE);
	writel(0, base + CAMSV_SW_CTL);
	writel(SW_RST_TRIG, base + CAMSV_SW_CTL);
	if (readl_poll_timeout(base + CAMSV_SW_CTL, st, st & SW_RST_ST, 10, 10000))
		dev_warn(camsv->dev, "DMA reset timed out (%#x)\n", st);

	writel(SW_HW_RST, base + CAMSV_SW_CTL);
	writel(0, base + CAMSV_SW_CTL);
	writel(MODULE_DEFAULT, base + CAMSV_MODULE_EN);
	writel(0, base + CAMSV_FBC_IMGO_CTL1);
	readl(base + CAMSV_INT_STATUS);
}

/* -------------------------------------------------------------------------
 * vb2
 */

static int camsv_queue_setup(struct vb2_queue *q, unsigned int *nbuf,
			     unsigned int *nplanes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	struct camsv_dev *camsv = vb2_get_drv_priv(q);
	unsigned int size = camsv_sizeimage(camsv);

	if (*nplanes) {
		if (sizes[0] < size)
			return -EINVAL;
		return 0;
	}
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int camsv_buf_prepare(struct vb2_buffer *vb)
{
	struct camsv_dev *camsv = vb2_get_drv_priv(vb->vb2_queue);
	unsigned int size = camsv_sizeimage(camsv);

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(camsv->dev, "buffer too small: %lu < %u\n",
			vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}
	vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static void camsv_buf_queue(struct vb2_buffer *vb)
{
	struct camsv_dev *camsv = vb2_get_drv_priv(vb->vb2_queue);
	struct camsv_buffer *buf = to_camsv_buffer(to_vb2_v4l2_buffer(vb));
	unsigned long flags;

	spin_lock_irqsave(&camsv->qlock, flags);
	camsv_mark_buffer(camsv, buf);
	list_add_tail(&buf->list, &camsv->queued);
	camsv_program_next(camsv);
	spin_unlock_irqrestore(&camsv->qlock, flags);
}

static void camsv_return_buffers(struct camsv_dev *camsv,
				 enum vb2_buffer_state state)
{
	struct camsv_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&camsv->qlock, flags);
	camsv->streaming = false;
	list_for_each_entry_safe(buf, tmp, &camsv->inflight, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	list_for_each_entry_safe(buf, tmp, &camsv->queued, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&camsv->qlock, flags);
}

static int camsv_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct camsv_dev *camsv = vb2_get_drv_priv(q);
	unsigned long flags;
	int ret;

	ret = pm_runtime_resume_and_get(camsv->dev);
	if (ret)
		goto err_return;

	ret = clk_bulk_prepare_enable(camsv->num_clks, camsv->clks);
	if (ret)
		goto err_pm;

	/*
	 * The line is level-triggered and the status is read-clear, so the
	 * engine can assert it before anyone is streaming - the bootloader or
	 * the vendor kernel leaves sources pending, and on the first phone boot
	 * that meant 100000 interrupts in about a second and "Disabling IRQ".
	 * Two things keep that from happening: every source is masked and the
	 * status cleared before the handler is installed, and the handler
	 * itself is only requested while a stream is running. The vendor guards
	 * the same way - it reads the status first and does nothing while its
	 * clock is off (camera_isp.c, ISP_Irq_CAMSV).
	 */
	writel(0, camsv->base + CAMSV_INT_EN);
	readl(camsv->base + CAMSV_INT_STATUS);

	ret = request_irq(camsv->irq, camsv_irq, 0, dev_name(camsv->dev), camsv);
	if (ret) {
		dev_err(camsv->dev, "failed to request irq %d: %d\n",
			camsv->irq, ret);
		goto err_clock;
	}

	camsv_hw_start(camsv);

	spin_lock_irqsave(&camsv->qlock, flags);
	camsv->wcnt = readl(camsv->base + CAMSV_FBC_IMGO_CTL2) >> 8;
	camsv->errors = 0;
	camsv->latch_pending = false;
	camsv->skip = 1;
	camsv->sequence = 0;
	camsv->streaming = true;
	camsv_program_next(camsv);	/* first buffer before the TG runs */
	spin_unlock_irqrestore(&camsv->qlock, flags);

	writel(1, camsv->base + CAMSV_TG_VF_CON);
	return 0;

err_clock:
	clk_bulk_disable_unprepare(camsv->num_clks, camsv->clks);
err_pm:
	pm_runtime_put(camsv->dev);
err_return:
	camsv_return_buffers(camsv, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void camsv_stop_streaming(struct vb2_queue *q)
{
	struct camsv_dev *camsv = vb2_get_drv_priv(q);
	unsigned long flags;

	spin_lock_irqsave(&camsv->qlock, flags);
	camsv->streaming = false;
	spin_unlock_irqrestore(&camsv->qlock, flags);

	camsv_hw_stop(camsv);
	synchronize_irq(camsv->irq);
	free_irq(camsv->irq, camsv);

	clk_bulk_disable_unprepare(camsv->num_clks, camsv->clks);
	pm_runtime_put(camsv->dev);

	camsv_return_buffers(camsv, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops camsv_vb2_ops = {
	.queue_setup = camsv_queue_setup,
	.buf_prepare = camsv_buf_prepare,
	.buf_queue = camsv_buf_queue,
	.start_streaming = camsv_start_streaming,
	.stop_streaming = camsv_stop_streaming,
};

/* -------------------------------------------------------------------------
 * V4L2
 */

static void camsv_fill_pix(struct camsv_dev *camsv, struct v4l2_pix_format *pix)
{
	unsigned int line = camsv_line_bytes(camsv);

	pix->width = camsv->width;
	pix->height = camsv->height;
	pix->pixelformat = camsv->fmt->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = CAMSV_STRIDE(line);
	pix->sizeimage = CAMSV_STRIDE(line) * camsv->height;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int camsv_querycap(struct file *file, void *fh,
			  struct v4l2_capability *cap)
{
	strscpy(cap->driver, "mtk-camsv", sizeof(cap->driver));
	strscpy(cap->card, "MT6785 CAMSV", sizeof(cap->card));
	return 0;
}

static int camsv_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(camsv_formats))
		return -EINVAL;
	f->pixelformat = camsv_formats[f->index].fourcc;
	return 0;
}

static int camsv_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camsv_dev *camsv = video_drvdata(file);

	camsv_fill_pix(camsv, &f->fmt.pix);
	return 0;
}

static int camsv_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camsv_dev *camsv = video_drvdata(file);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(camsv_formats); i++) {
		if (camsv_formats[i].fourcc == f->fmt.pix.pixelformat) {
			camsv->fmt = &camsv_formats[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(camsv_formats))
		camsv->fmt = &camsv_formats[0];

	/*
	 * The size is not negotiable: the engine captures exactly the frame
	 * the sensor sends, so the DT size is reported whatever was asked.
	 */
	camsv_fill_pix(camsv, &f->fmt.pix);
	return 0;
}

static int camsv_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camsv_dev *camsv = video_drvdata(file);
	unsigned int i;

	if (vb2_is_busy(&camsv->queue))
		return -EBUSY;

	for (i = 0; i < ARRAY_SIZE(camsv_formats); i++) {
		if (camsv_formats[i].fourcc == f->fmt.pix.pixelformat) {
			camsv->fmt = &camsv_formats[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(camsv_formats))
		camsv->fmt = &camsv_formats[0];

	camsv_fill_pix(camsv, &f->fmt.pix);
	return 0;
}

static int camsv_enum_framesizes(struct file *file, void *fh,
				 struct v4l2_frmsizeenum *fsize)
{
	struct camsv_dev *camsv = video_drvdata(file);
	unsigned int i;

	if (fsize->index)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(camsv_formats); i++) {
		if (camsv_formats[i].fourcc == fsize->pixel_format)
			break;
	}
	if (i == ARRAY_SIZE(camsv_formats))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = camsv->width;
	fsize->discrete.height = camsv->height;
	return 0;
}

static const struct v4l2_ioctl_ops camsv_ioctl_ops = {
	.vidioc_querycap = camsv_querycap,
	.vidioc_enum_fmt_vid_cap = camsv_enum_fmt,
	.vidioc_g_fmt_vid_cap = camsv_g_fmt,
	.vidioc_try_fmt_vid_cap = camsv_try_fmt,
	.vidioc_s_fmt_vid_cap = camsv_s_fmt,
	.vidioc_enum_framesizes = camsv_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations camsv_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

/* -------------------------------------------------------------------------
 * Probe
 */

static int camsv_get_geometry(struct camsv_dev *camsv)
{
	u32 v;
	int ret;

	/*
	 * The frame the engine captures is the sensor's mode; until SENINF and
	 * the bindings exist it is described on the engine node itself, as the
	 * live device tree does for the ISP nodes it owns.
	 */
	ret = of_property_read_u32(camsv->dev->of_node, "width", &v);
	if (ret)
		return dev_err_probe(camsv->dev, ret, "missing width\n");
	camsv->width = v;

	ret = of_property_read_u32(camsv->dev->of_node, "height", &v);
	if (ret)
		return dev_err_probe(camsv->dev, ret, "missing height\n");
	camsv->height = v;

	camsv->fmt = &camsv_formats[0];
	return 0;
}

static int camsv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct camsv_dev *camsv;
	struct vb2_queue *q;
	int ret;

	camsv = devm_kzalloc(dev, sizeof(*camsv), GFP_KERNEL);
	if (!camsv)
		return -ENOMEM;

	camsv->dev = dev;
	spin_lock_init(&camsv->qlock);
	INIT_LIST_HEAD(&camsv->queued);
	INIT_LIST_HEAD(&camsv->inflight);
	mutex_init(&camsv->vlock);

	camsv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(camsv->base))
		return PTR_ERR(camsv->base);

	camsv->irq = platform_get_irq(pdev, 0);
	if (camsv->irq < 0)
		return camsv->irq;

	ret = devm_clk_bulk_get_all(dev, &camsv->clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get clocks\n");
	camsv->num_clks = ret;

	ret = camsv_get_geometry(camsv);
	if (ret)
		return ret;

	/*
	 * Physical addressing: the buffers must live below 4 GiB, because the
	 * SMI larb port stays in MMU bypass (study section 4.5). A 32-bit mask
	 * on the device is what makes vb2 hand out such addresses.
	 */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA\n");

	camsv->scratch = dma_alloc_coherent(dev, SZ_64K, &camsv->scratch_dma,
					    GFP_KERNEL);
	if (!camsv->scratch)
		return -ENOMEM;

	platform_set_drvdata(pdev, camsv);

	q = &camsv->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->dev = dev;
	q->ops = &camsv_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->lock = &camsv->vlock;
	q->min_queued_buffers = 2;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	ret = vb2_queue_init(q);
	if (ret) {
		dev_err_probe(dev, ret, "vb2 queue init failed\n");
		goto err_free_scratch;
	}

	camsv->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&camsv->vdev.entity, 1, &camsv->pad);
	if (ret) {
		dev_err_probe(dev, ret, "media entity init failed\n");
		goto err_free_scratch;
	}

	ret = v4l2_device_register(dev, &camsv->v4l2_dev);
	if (ret) {
		dev_err_probe(dev, ret, "v4l2 device register failed\n");
		goto err_entity;
	}

	camsv->vdev.queue = q;
	camsv->vdev.lock = &camsv->vlock;
	camsv->vdev.v4l2_dev = &camsv->v4l2_dev;
	camsv->vdev.fops = &camsv_fops;
	camsv->vdev.ioctl_ops = &camsv_ioctl_ops;
	camsv->vdev.release = video_device_release_empty;
	camsv->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	snprintf(camsv->vdev.name, sizeof(camsv->vdev.name), "mtk-camsv-%s",
		 dev_name(dev));
	video_set_drvdata(&camsv->vdev, camsv);

	pm_runtime_enable(dev);

	ret = video_register_device(&camsv->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err_probe(dev, ret, "video device register failed\n");
		goto err_v4l2;
	}

	dev_info(dev, "%s: %ux%u, engine at %pR\n", video_device_node_name(&camsv->vdev),
		 camsv->width, camsv->height, &pdev->resource[0]);
	return 0;

err_v4l2:
	pm_runtime_disable(dev);
	v4l2_device_unregister(&camsv->v4l2_dev);
err_entity:
	media_entity_cleanup(&camsv->vdev.entity);
err_free_scratch:
	dma_free_coherent(dev, SZ_64K, camsv->scratch, camsv->scratch_dma);
	return ret;
}

static void camsv_remove(struct platform_device *pdev)
{
	struct camsv_dev *camsv = platform_get_drvdata(pdev);

	video_unregister_device(&camsv->vdev);
	pm_runtime_disable(camsv->dev);
	v4l2_device_unregister(&camsv->v4l2_dev);
	media_entity_cleanup(&camsv->vdev.entity);
	dma_free_coherent(camsv->dev, SZ_64K, camsv->scratch, camsv->scratch_dma);
}

static const struct of_device_id camsv_of_match[] = {
	{ .compatible = "mediatek,mt6785-camsv" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, camsv_of_match);

static struct platform_driver camsv_driver = {
	.probe = camsv_probe,
	.remove = camsv_remove,
	.driver = {
		.name = "mtk-camsv",
		.of_match_table = camsv_of_match,
	},
};
module_platform_driver(camsv_driver);

MODULE_DESCRIPTION("MediaTek MT6785 CAMSV raw capture driver");
MODULE_LICENSE("GPL");
