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
 * 0x1a090000). That port's drivers/camera/cezanne_camsv.c was the starting
 * point, but its register offsets are the sibling SoC's; the offsets here are
 * the MT6785 ones from this phone's own vendor driver, and the ones no source
 * gives are gated instead of guessed - see struct camsv_regs and the stock
 * register diff written up in scratch/wt/camera/REPORT.md.
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
 *  3. Something writes to the frame-write address register once per frame. At
 *     zero that is a write per frame to physical 0, so the driver hands it a
 *     scratch buffer.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <media/media-device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

/*
 * Registers, with the MT6785 offsets.
 *
 * Source: drivers/misc/mediatek/cameraisp/src/mt6785/inc/cam_regs.h and its
 * user camera_isp.c in the Lunaris/Xiaomi 4.19 tree
 * (scratch/poco-lunaris-kernel) - the driver this phone's stock firmware runs.
 * The line of every known offset is quoted beside it. MT6785 is the odd one
 * out in its own family: MT6765, MT6853, MT6873 and MT6885 (the cezanne port
 * this file was first built from) put the top bank at 0x0010..0x004c, the
 * timing generator at 0x0130 and IMGO at 0x0420, while MT6785 puts the top
 * bank at 0x0510, the timing generator at 0x0230 and IMGO at 0x0020. The
 * stock register diff of 18.11.2026 says the same: 0x1a050510 reads
 * 0x40000000, 0x1a050520 reads 0x2 and 0x1a05026c reads 0x102 - the module
 * enable with nothing but the debug block on, the DMA reset status idle and
 * the timing generator state idle - at offsets the sibling table never names.
 *
 * The rest of the map is not in the GPL drop: the vendor kernel touches only
 * the registers named below, and the format, the packer, the DMA sizes and the
 * frame-write address are programmed by the closed HAL. Those offsets are left
 * unknown here, and start_streaming() refuses to run while one of them is
 * missing instead of writing a sibling SoC's offset into a register that means
 * something else on this one. The stock register snapshot of study 4.4 fills
 * them in, and nothing else needs to change then.
 */
struct camsv_regs {
	u32 module_en;		/* 0x0510, cam_regs.h:362 */
	u32 int_en;		/* 0x0518, cam_regs.h:363 */
	u32 int_status;		/* 0x051c, cam_regs.h:364 */
	u32 sw_ctl;		/* 0x0520, cam_regs.h:365 */
	u32 fbc_imgo_ctl1;	/* 0x0110, cam_regs.h:366 */
	u32 fbc_imgo_ctl2;	/* 0x0114, cam_regs.h:367 */
	u32 imgo_base;		/* 0x0020, cam_regs.h:368 */
	u32 tg_sen_mode;	/* 0x0230, cam_regs.h:369 */
	u32 tg_vf_con;		/* 0x0234, cam_regs.h:370 */
	u32 tg_inter_st;	/* 0x026c, cam_regs.h:371 */
	u32 tg_grab_pxl;
	u32 tg_grab_lin;
	u32 tg_path_cfg;
	u32 imgo_base_msb;
	u32 imgo_fbc;
	u32 imgo_ofst;
	u32 imgo_xsize;
	u32 imgo_ysize;
	u32 imgo_stride;
	u32 imgo_err;
	u32 fmt_sel;
	u32 pak;
	u32 clk_en;
	u32 dma_db_en;
	u32 frame_write;
};

#define CAMSV_REG_UNKNOWN	0xffffffffu

static const struct camsv_regs mt6785_camsv_regs = {
	.module_en	= 0x0510,
	.int_en		= 0x0518,
	.int_status	= 0x051c,
	.sw_ctl		= 0x0520,
	.fbc_imgo_ctl1	= 0x0110,
	.fbc_imgo_ctl2	= 0x0114,
	.imgo_base	= 0x0020,
	.tg_sen_mode	= 0x0230,
	.tg_vf_con	= 0x0234,
	.tg_inter_st	= 0x026c,
	.tg_grab_pxl	= CAMSV_REG_UNKNOWN,
	.tg_grab_lin	= CAMSV_REG_UNKNOWN,
	.tg_path_cfg	= CAMSV_REG_UNKNOWN,
	.imgo_base_msb	= CAMSV_REG_UNKNOWN,
	.imgo_fbc	= CAMSV_REG_UNKNOWN,
	.imgo_ofst	= CAMSV_REG_UNKNOWN,
	.imgo_xsize	= CAMSV_REG_UNKNOWN,
	.imgo_ysize	= CAMSV_REG_UNKNOWN,
	.imgo_stride	= CAMSV_REG_UNKNOWN,
	.imgo_err	= CAMSV_REG_UNKNOWN,
	.fmt_sel	= CAMSV_REG_UNKNOWN,
	.pak		= CAMSV_REG_UNKNOWN,
	.clk_en		= CAMSV_REG_UNKNOWN,
	.dma_db_en	= CAMSV_REG_UNKNOWN,
	.frame_write	= CAMSV_REG_UNKNOWN,
};

#define CAMSV_REG(camsv, name)	((camsv)->base + (camsv)->regs->name)

/*
 * Module enables. Only the IMGO DMA bit is in the MT6785 source, which reads
 * it back with "& 0x10" (camera_isp.c:3462-3467); the timing generator and
 * packer bits and the reserved 0x50000000 are the sibling SoC's and wait for
 * the same snapshot as the offsets.
 */
#define   MODULE_TG_EN		BIT(0)
#define   MODULE_PAK_EN		BIT(2)
#define   MODULE_IMGO_EN	BIT(4)
#define   MODULE_DEFAULT	0x50000000

/*
 * Interrupt bits, cam_regs.h:69-83 (SV_*). The MT6885 port's values differ -
 * it has SOF at bit 6 and pass1-done at bits 10..12 - so its mask would wait
 * for a source this SoC never raises.
 */
#define   INT_VS1		BIT(0)	/* SV_VS1_ST */
#define   INT_TG_ST1		BIT(1)	/* SV_TG_ST1 */
#define   INT_TG_ST2		BIT(2)	/* SV_TG_ST2 */
#define   INT_EXPDON		BIT(3)	/* SV_EXPDON_ST */
#define   INT_TG_ERR		BIT(4)	/* SV_TG_ERR */
#define   INT_TG_GBERR		BIT(5)	/* SV_TG_GBERR */
#define   INT_SOF		BIT(7)	/* SV_SOF_INT_ST */
#define   INT_HW_PASS1_DONE	BIT(10)	/* SV_HW_PASS1_DON_ST */
#define   INT_IMGO_ERR		BIT(16)	/* SV_IMGO_ERR */
#define   INT_IMGO_OVERRUN	BIT(17)	/* SV_IMGO_OVERRUN */
#define   INT_SW_PASS1_DONE	BIT(20)	/* SV_SW_PASS1_DON_ST */
#define   INT_ERRORS		(INT_TG_ERR | INT_TG_GBERR | INT_IMGO_ERR | \
				 INT_IMGO_OVERRUN)

/*
 * Reset handshake, camera_isp.c:5045-5060: write 4, then 0, then 1, and wait
 * for a read of 3 - trigger set, status idle.
 */
#define   SW_RST_TRIG		BIT(0)
#define   SW_RST_ST		BIT(1)	/* set: the DMA is idle */
#define   SW_HW_RST		BIT(2)

/* Timing generator: CMOS enable (camera_isp.c:5066) and the state field
 * (camera_isp.c:5003, 5061), which reads 1 when idle.
 */
#define   SEN_MODE_CMOS_EN	BIT(0)
#define   SEN_MODE_TIME_STP_EN	BIT(16)
#define   TG_STATE		GENMASK(13, 8)
#define   TG_STATE_IDLE		0x100

/*
 * Values for registers whose MT6785 offset is unknown, taken from the sibling
 * SoCs the cezanne port documented: only the semantics are used, never their
 * offset.
 */
#define   CLK_EN_TG_PAK_DMA	0x8005
#define   PAK_RAW8		8
#define   PAK_RAW10_UNPACKED	13	/* 16 bits per pixel, LSB aligned */
#define   FBC_MODE		BIT(11)
#define   FBC_EN		BIT(15)

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
 * @regs: this SoC's register offsets, see struct camsv_regs
 * @dbgfs: debugfs directory holding the register dump
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
 * @scratch: the buffer the frame-write address register points at
 * @scratch_dma: its DMA address
 */
struct camsv_dev {
	struct device *dev;
	void __iomem *base;
	const struct camsv_regs *regs;
	struct dentry *dbgfs;
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
	writel(lower_32_bits(addr), CAMSV_REG(camsv, imgo_base));
	writel(upper_32_bits(addr), CAMSV_REG(camsv, imgo_base_msb));
	/* ask for the address to be latched at the next start of frame */
	writel(readl(CAMSV_REG(camsv, imgo_fbc)) | BIT(0),
	       CAMSV_REG(camsv, imgo_fbc));
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
	st = readl(CAMSV_REG(camsv, int_status));
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
					     readl(CAMSV_REG(camsv, imgo_err)));
		camsv->errors |= st & INT_ERRORS;
		writel(readl(CAMSV_REG(camsv, int_en)) & ~(st & INT_ERRORS),
		       CAMSV_REG(camsv, int_en));
	}

	if (st & INT_SOF) {
		camsv->sequence++;
		camsv->latch_pending = false;
	}

	/* completed frames arrive in the order the DMA was given them */
	wcnt = readl(CAMSV_REG(camsv, fbc_imgo_ctl2)) >> 8;
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
	unsigned int line = camsv_line_bytes(camsv);
	u32 st;

	/* reset the block, and wait for the DMA to report itself idle */
	writel(SW_HW_RST, CAMSV_REG(camsv, sw_ctl));
	writel(0, CAMSV_REG(camsv, sw_ctl));
	writel(SW_RST_TRIG, CAMSV_REG(camsv, sw_ctl));
	readl_poll_timeout(CAMSV_REG(camsv, sw_ctl), st, st & SW_RST_ST, 10,
			   10000);

	writel(SEN_MODE_TIME_STP_EN, CAMSV_REG(camsv, tg_sen_mode));
	writel(camsv->width << 16, CAMSV_REG(camsv, tg_grab_pxl));
	writel(camsv->height << 16, CAMSV_REG(camsv, tg_grab_lin));
	writel(0, CAMSV_REG(camsv, tg_path_cfg));

	readl(CAMSV_REG(camsv, int_status));
	writel(INT_VS1 | INT_SOF | INT_HW_PASS1_DONE | INT_ERRORS,
	       CAMSV_REG(camsv, int_en));

	writel(camsv->fmt->raw8 ? 0 : 1, CAMSV_REG(camsv, fmt_sel));
	writel(CLK_EN_TG_PAK_DMA, CAMSV_REG(camsv, clk_en));
	writel(camsv->fmt->raw8 ? PAK_RAW8 : PAK_RAW10_UNPACKED,
	       CAMSV_REG(camsv, pak));

	writel(1, CAMSV_REG(camsv, dma_db_en));
	writel(0, CAMSV_REG(camsv, imgo_ofst));
	writel(line - 1, CAMSV_REG(camsv, imgo_xsize));
	writel(camsv->height - 1, CAMSV_REG(camsv, imgo_ysize));
	writel(CAMSV_STRIDE(line), CAMSV_REG(camsv, imgo_stride));
	/* the per-frame write CAMSV does on its own goes to scratch memory */
	writel(lower_32_bits(camsv->scratch_dma),
	       CAMSV_REG(camsv, frame_write));

	writel(MODULE_DEFAULT | MODULE_TG_EN | MODULE_PAK_EN,
	       CAMSV_REG(camsv, module_en));
	writel(SEN_MODE_TIME_STP_EN | SEN_MODE_CMOS_EN,
	       CAMSV_REG(camsv, tg_sen_mode));
	writel(MODULE_DEFAULT | MODULE_TG_EN | MODULE_PAK_EN | MODULE_IMGO_EN,
	       CAMSV_REG(camsv, module_en));

	writel(FBC_MODE, CAMSV_REG(camsv, fbc_imgo_ctl1));
	writel(FBC_EN | FBC_MODE, CAMSV_REG(camsv, fbc_imgo_ctl1));
}

/*
 * Stop in the vendor's order: the timing generator first, waiting for the frame
 * in progress to end, then the DMA reset handshake. Clearing the enables first
 * leaves a frame half-written.
 */
static void camsv_hw_stop(struct camsv_dev *camsv)
{
	u32 st;

	writel(0, CAMSV_REG(camsv, int_en));
	readl(CAMSV_REG(camsv, int_status));
	writel(0, CAMSV_REG(camsv, tg_vf_con));

	if (readl_poll_timeout(CAMSV_REG(camsv, tg_inter_st), st,
			       (st & TG_STATE) == TG_STATE_IDLE, 1000,
			       CAMSV_TG_IDLE_TIMEOUT_US))
		dev_warn(camsv->dev, "timing generator not idle (%#x)\n", st);

	writel(0, CAMSV_REG(camsv, tg_sen_mode));
	writel(0, CAMSV_REG(camsv, sw_ctl));
	writel(SW_RST_TRIG, CAMSV_REG(camsv, sw_ctl));
	if (readl_poll_timeout(CAMSV_REG(camsv, sw_ctl), st, st & SW_RST_ST, 10,
			       10000))
		dev_warn(camsv->dev, "DMA reset timed out (%#x)\n", st);

	writel(SW_HW_RST, CAMSV_REG(camsv, sw_ctl));
	writel(0, CAMSV_REG(camsv, sw_ctl));
	writel(MODULE_DEFAULT, CAMSV_REG(camsv, module_en));
	writel(0, CAMSV_REG(camsv, fbc_imgo_ctl1));
	readl(CAMSV_REG(camsv, int_status));
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

/*
 * The registers the capture path needs but whose MT6785 offset no source
 * gives. Returns the first one still missing, NULL when the table is complete.
 */
static const char *camsv_regs_missing(const struct camsv_regs *r)
{
	const struct {
		u32 ofs;
		const char *name;
	} needed[] = {
		{ r->tg_grab_pxl, "TG_GRAB_PXL" },
		{ r->tg_grab_lin, "TG_GRAB_LIN" },
		{ r->tg_path_cfg, "TG_PATH_CFG" },
		{ r->fmt_sel, "FMT_SEL" },
		{ r->pak, "PAK" },
		{ r->clk_en, "CLK_EN" },
		{ r->dma_db_en, "DMA_DB" },
		{ r->imgo_base_msb, "IMGO_BASE_MSB" },
		{ r->imgo_ofst, "IMGO_OFST" },
		{ r->imgo_xsize, "IMGO_XSIZE" },
		{ r->imgo_ysize, "IMGO_YSIZE" },
		{ r->imgo_stride, "IMGO_STRIDE" },
		{ r->imgo_fbc, "IMGO_FBC" },
		{ r->imgo_err, "IMGO_ERR_STAT" },
		{ r->frame_write, "FRAME_WRITE_BASE" },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(needed); i++)
		if (needed[i].ofs == CAMSV_REG_UNKNOWN)
			return needed[i].name;

	return NULL;
}

static int camsv_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct camsv_dev *camsv = vb2_get_drv_priv(q);
	unsigned long flags;
	const char *missing;
	int ret;

	/*
	 * A wrong offset here is not a slow path, it is a write into a register
	 * that means something else: the MT6885 layout puts the DMA base
	 * address where this SoC has IMGO_... and the reverse, which can point
	 * the DMA at physical 0 or trip the DEVAPC the vendor driver warns
	 * about. Refuse and say what is missing.
	 */
	missing = camsv_regs_missing(camsv->regs);
	if (missing) {
		dev_err(camsv->dev, "cannot stream: no MT6785 offset for %s\n",
			missing);
		dev_err(camsv->dev,
			"the stock register snapshot fills in what the closed HAL programs\n");
		ret = -EOPNOTSUPP;
		goto err_return;
	}

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
	writel(0, CAMSV_REG(camsv, int_en));
	readl(CAMSV_REG(camsv, int_status));

	ret = request_irq(camsv->irq, camsv_irq, 0, dev_name(camsv->dev), camsv);
	if (ret) {
		dev_err(camsv->dev, "failed to request irq %d: %d\n",
			camsv->irq, ret);
		goto err_clock;
	}

	camsv_hw_start(camsv);

	spin_lock_irqsave(&camsv->qlock, flags);
	camsv->wcnt = readl(CAMSV_REG(camsv, fbc_imgo_ctl2)) >> 8;
	camsv->errors = 0;
	camsv->latch_pending = false;
	camsv->skip = 1;
	camsv->sequence = 0;
	camsv->streaming = true;
	camsv_program_next(camsv);	/* first buffer before the TG runs */
	spin_unlock_irqrestore(&camsv->qlock, flags);

	writel(1, CAMSV_REG(camsv, tg_vf_con));
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
 * Debugfs
 */

/*
 * The engine's registers as offsets from the node's base, in the windows of
 * both known layouts, so one boot can say which of them this silicon has and
 * what the closed HAL leaves behind. Compare with the stock dump in
 * poco-m5s/evidence/stock-session-2026-11/camera-streaming.txt, which lists
 * absolute addresses from the same window. Reads only; the clocks and the
 * power domain are on, because a register read of a gated block returns zero.
 */
static int camsv_regs_show(struct seq_file *s, void *unused)
{
	static const unsigned int windows[][2] = {
		{ 0x0000, 0x0060 },	/* IMGO bank, then the sibling top bank */
		{ 0x0100, 0x0120 },	/* frame write counter of both layouts */
		{ 0x0200, 0x02b0 },	/* MT6785 timing generator */
		{ 0x0420, 0x0440 },	/* IMGO bank of the sibling SoCs */
		{ 0x0500, 0x0530 },	/* MT6785 top bank */
		{ 0x0800, 0x0810 },	/* sibling DMA address latch */
		{ 0x0a00, 0x0a10 },	/* sibling frame write counter */
	};
	struct camsv_dev *camsv = s->private;
	unsigned int i, ofs;
	int ret;

	ret = pm_runtime_resume_and_get(camsv->dev);
	if (ret)
		return ret;
	ret = clk_bulk_prepare_enable(camsv->num_clks, camsv->clks);
	if (ret)
		goto out_pm;

	for (i = 0; i < ARRAY_SIZE(windows); i++)
		for (ofs = windows[i][0]; ofs < windows[i][1]; ofs += 4)
			seq_printf(s, "%04x %08x\n", ofs,
				   readl(camsv->base + ofs));

	clk_bulk_disable_unprepare(camsv->num_clks, camsv->clks);
out_pm:
	pm_runtime_put(camsv->dev);
	return ret;
}

static int camsv_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, camsv_regs_show, inode->i_private);
}

static const struct file_operations camsv_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= camsv_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
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
	camsv->regs = &mt6785_camsv_regs;
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

	camsv->dbgfs = debugfs_create_dir(dev_name(dev), NULL);
	if (!IS_ERR(camsv->dbgfs))
		debugfs_create_file("regs", 0444, camsv->dbgfs, camsv,
				    &camsv_regs_fops);

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

	debugfs_remove_recursive(camsv->dbgfs);
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
