// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CAMSV: raw Bayer capture.
 *
 * The CAMSV engines take the camera mux output of the CSI-2 receiver (SENINF) and write raw
 * frames to memory. This driver implements one engine as a V4L2 video device with a single sink
 * pad, connected to the receiver's source pad through the media controller, and a vb2 queue
 * that allocates through the camera larb's IOMMU (M4U_PORT_CAM_IMGO, larb 6 port 0 -
 * include/dt-bindings/memory/mediatek,mt6785-memory-port.h:96).
 *
 * Register offsets are the cezanne (MT6885) map, the only complete one for this IP generation
 * (poco-m5s/research/native/5-cameras.md §3.2, read there from cezanne_camsv.c): the engines'
 * layout is shared with MT6785 while their bases differ (ours: camsv1..6 at 0x1a050000 +
 * 0x1000*n, vendor device tree). They are not proven on this silicon until a stock-phone
 * snapshot exists, which is the same caveat the receiver carries.
 *
 * Clocks: the camsys gates CLK_CAM_SV0/CLK_CAM_SV1 (drivers/clk/mediatek/clk-mt6785-cam.c:29-30,
 * ids at include/dt-bindings/clock/mediatek,mt6785-clk.h:98-99) and the CAM power domain, the one
 * larb6 uses.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-async.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

#include "../cam/mtk_cam_core.h"

/*
 * CAMSV registers, from this phone's own GPL kernel
 * (scratch/native/5-cameras/vendor-rosemary-r-oss/cam_regs.h:369-379, the CAMSV block, and the
 * device tree's camsv1 at 0x1a050000 with a 0x1000 window). The cezanne MT6885 offsets this
 * driver used before (INT_EN 0x18, SW_CTL 0x20) belong to the ISP6 layout and simply do not exist
 * on this block - 0x18 reads back 0 while the block is clocked.
 */
#define CAMSV_MODULE_EN			0x0510	/* cam_regs.h:369 */
#define CAMSV_INT_EN			0x0518	/* cam_regs.h:370 */
#define CAMSV_INT_STATUS		0x051c	/* cam_regs.h:371 */
#define CAMSV_SW_CTL			0x0520	/* cam_regs.h:372 */
#define CAMSV_FBC_IMGO_CTL1		0x0110	/* cam_regs.h:373 */
#define CAMSV_FBC_IMGO_CTL2		0x0114	/* cam_regs.h:374 */
#define CAMSV_IMGO_BASE_ADDR		0x0020	/* cam_regs.h:375 */
#define CAMSV_TG_SEN_MODE		0x0230	/* cam_regs.h:376 */
#define CAMSV_TG_VF_CON			0x0234	/* cam_regs.h:377 */
#define CAMSV_TG_INTER_ST		0x026c	/* cam_regs.h:378 */
#define CAMSV_TG_TIME_STAMP		0x02a0	/* cam_regs.h:379 */

/*
 * The rest of the engine, from the stock HAL's CAMSV code (libimageio_plat_drv.so from this phone's
 * vendor image: CAMSV_TG_CTRL::_config 0xb92b8, CAMSV_TOP_CTRL::_config 0xba218 / ::_enable 0xbb390,
 * DMAO_CAMSV_B::_config 0xbc880 / ::_enable 0xbd5c8 / ::setBaseAddr 0xbda78, CAMSV_BUF_CTRL::_config
 * 0xbee28 / ::_enable 0xbf0b0 / ::enqueueHwBuf 0xbfbd0; disassembly in scratch/native/5-cameras/
 * hal-dis/camsv/). The order is the one of the mainline-style driver for the next MediaTek generation
 * (scratch/native/5-cameras/tmp/cez-cam/cezanne_camsv.c, cam_hw_setup), whose block is the same
 * engine at other offsets: TOP +0x500, TG +0x100, DMA -0x400 here.
 */
#define CAMSV_FMT_SEL			0x0514	/* TOP_CTRL: [x19+0x44] written whole; 1 = RAW10 */
#define CAMSV_IMGO_FBC			0x052c	/* enqueueHwBuf 0xc0204: bit 0 = RCNT_INC */
/*
 * With FBC on, the DMA takes each frame's address from the FBC queue, not from IMGO_BASE_ADDR:
 * enqueueHwBuf (0xc0148-0xc0204) writes the buffer to 0x118, its header to 0x404, then RCNT_INC.
 * comb46 measured it: with only 0x020 set the engine wrote at 0 + offset (IOMMU faults, larb 6 port 11).
 */
#define CAMSV_FBC_IMGO_ENQ_ADDR		0x0118
#define CAMSV_CLK_EN			0x0530	/* TOP_CTRL::_enable: bits 0, 2, 15 */
#define CAMSV_PAK			0x054c	/* bits 2:0 mode, 5:4 pixel mode */
#define CAMSV_TG_GRAB_PXL		0x0238	/* start | end << 16 */
#define CAMSV_TG_GRAB_LIN		0x023c
#define CAMSV_TG_PATH_CFG		0x0240
#define CAMSV_IMGO_OFST			0x0028
#define CAMSV_IMGO_XSIZE		0x0030	/* bytes per line - 1 */
#define CAMSV_IMGO_YSIZE		0x0034	/* lines - 1 */
#define CAMSV_IMGO_STRIDE		0x0038
#define CAMSV_IMGO_CON			0x003c	/* DMAO config: 0x80000080 */
#define CAMSV_IMGO_CON2			0x0040	/* 0x00020002 */
#define CAMSV_IMGO_CON3			0x0044	/* 0x00020002 */
#define CAMSV_DMA_ERR_CTL		0x03c4	/* TOP_CTRL writes all ones */
#define CAMSV_DMA_DB_EN			0x0400	/* DMAO::_enable: bit 0, address latches at SOF */
#define CAMSV_FRAME_WRITE_BASE		0x0404	/* per-frame header the engine writes; needs memory */
#define CAMSV_SCRATCH_SIZE		SZ_64K

#define CAMSV_SEN_MODE_CMOS_EN		BIT(0)
#define CAMSV_SEN_MODE_TIME_STP_EN	BIT(16)
#define CAMSV_MODULE_EN_DEFAULT		BIT(30)	/* TOP_CTRL ORs it in */
#define CAMSV_CLK_EN_ALL		0x8005
#define CAMSV_FBC_MODE			BIT(16)	/* union FBC_CTRL_1, cam_regs.h */
#define CAMSV_FBC_BIT22			BIT(22)	/* BUF_CTRL::_enable sets it after FBC_EN */
#define CAMSV_FMT_RAW10			1

/* INT_STATUS bits of this chip (cam_regs.h:80-92), read-clear (camera_isp.c:8074-8076). */
#define CAMSV_INT_VS1			BIT(0)
#define CAMSV_INT_TG_ERR		BIT(4)
#define CAMSV_INT_TG_GBERR		BIT(5)
#define CAMSV_INT_SOF			BIT(7)
#define CAMSV_INT_HW_PASS1_DONE		BIT(10)
#define CAMSV_INT_PASS1_DONE		BIT(20)	/* SW pass1 done: the vendor dequeues on it (:8153) */
#define CAMSV_INT_IMGO_ERR		BIT(16)
#define CAMSV_INT_IMGO_OVERRUN		BIT(17)
#define CAMSV_INT_MASK			(CAMSV_INT_VS1 | CAMSV_INT_TG_ERR | CAMSV_INT_TG_GBERR | \
					 CAMSV_INT_SOF | CAMSV_INT_HW_PASS1_DONE | CAMSV_INT_PASS1_DONE | \
					 CAMSV_INT_IMGO_ERR | CAMSV_INT_IMGO_OVERRUN)
#define CAMSV_INT_ERRS			(CAMSV_INT_TG_ERR | CAMSV_INT_TG_GBERR | CAMSV_INT_IMGO_ERR | \
					 CAMSV_INT_IMGO_OVERRUN)
/* the vendor masks an error that repeats too often so it cannot hold the CPU (:8079-8104) */
#define CAMSV_ERR_STORM			64

/* FBC_IMGO_CTL1: FBC_EN is bit15 (union FBC_CTRL_1, cam_regs.h:383-390). */
#define CAMSV_FBC_EN			BIT(15)

/*
 * The interrupt mask: the vendor kernel never writes a constant - the HAL supplies it through an
 * ioctl and the kernel only ever clears bits from it (camera_isp.c:8117-8123). Enable everything
 * the block can report and let the handler read the status word, which is the safe direction: a
 * missed frame interrupt is worse than an extra one.
 */
#define CAMSV_INT_EN_ALL		0xffffffff

/* MODULE_EN bits: the vendor's own enable for TG, PAK, IMGO and DB (union in cam_regs.h). */
#define CAMSV_MODULE_EN_TG		BIT(0)
#define CAMSV_MODULE_EN_PAK		BIT(2)
#define CAMSV_MODULE_EN_IMGO		BIT(4)

#define CAMSV_MAX_WIDTH			4208
#define CAMSV_MAX_HEIGHT		3120
#define CAMSV_DEF_WIDTH			3280	/* the IMX355 full frame, measured at the seninf mux */
#define CAMSV_DEF_HEIGHT		2464

/**
 * struct mtk_camsv - one capture engine
 * @dev: platform device
 * @base: engine registers
 * @vdev: the video device
 * @subdev: the sink side of the pipeline
 * @pads: one sink pad, linked from the receiver
 * @mdev: the media device the pipeline is described in
 * @vdev_lock: serialises queue and streaming state
 * @queue: the vb2 queue, contiguous DMA through the camera IOMMU
 * @cur_fmt: current capture format
 * @sequence: frame counter handed to userspace
 */
struct mtk_camsv {
	struct device *dev;
	void __iomem *base;
	struct clk_bulk_data clks[1];
	struct clk *main_clk;
	struct clk *tg_clk;
	struct clk *gals_clk;
	int irq;

	struct video_device vdev;
	struct v4l2_subdev subdev;
	struct media_pad pads[2];
	struct media_pad vdev_pad;
	struct mtk_cam_core *cam;
	struct mutex vdev_lock;

	struct vb2_queue queue;
	struct media_pipeline pipeline;
	phys_addr_t base_pa;
	bool stream_dump;
	unsigned int irq_count;
	struct v4l2_format cur_fmt;
	struct v4l2_mbus_framefmt mbus_fmt;
	struct vb2_v4l2_buffer *cur_buf;
	u32 sequence;
	void *scratch;
	dma_addr_t scratch_dma;
	struct list_head bufs;		/* queued to the driver, not yet given to the engine */
	spinlock_t buf_lock;
	u32 status_seen;
	unsigned int sof_count, err_count;
	u32 err_status;
};

/* RAW10 as the engine packs it: four pixels in five bytes, lines 64-byte aligned. */
static unsigned int camsv_line_bytes(u32 w)
{
	return DIV_ROUND_UP(w * 10, 8);
}

static unsigned int camsv_stride(u32 w)
{
	return ALIGN(camsv_line_bytes(w), 64);
}

struct camsv_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

static inline struct camsv_buf *to_camsv_buf(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct camsv_buf, vb);
}

static struct vb2_v4l2_buffer *camsv_take_buf(struct mtk_camsv *camsv)
{
	struct camsv_buf *b = list_first_entry_or_null(&camsv->bufs, struct camsv_buf, list);

	if (!b)
		return NULL;
	list_del(&b->list);
	return &b->vb;
}

static inline struct mtk_camsv *to_camsv(struct video_device *vdev)
{
	return container_of(vdev, struct mtk_camsv, vdev);
}

/* DIAG (not for upstream): the engine's first 0x100, read-only, eight words a line. */
static void camsv_dump_engine(struct mtk_camsv *camsv)
{
	unsigned int off;

	static const unsigned int ranges[][2] = {
		{ 0x000, 0x100 }, { 0x100, 0x140 }, { 0x200, 0x2c0 }, { 0x500, 0x540 },
	};
	unsigned int r;

	for (r = 0; r < ARRAY_SIZE(ranges); r++)
	for (off = ranges[r][0]; off < ranges[r][1]; off += 0x20)
		dev_info(camsv->dev,
			 "camsv 0x%08llx: %08x %08x %08x %08x %08x %08x %08x %08x\n",
			 (unsigned long long)camsv->base_pa + off,
			 readl(camsv->base + off), readl(camsv->base + off + 4),
			 readl(camsv->base + off + 8), readl(camsv->base + off + 0xc),
			 readl(camsv->base + off + 0x10), readl(camsv->base + off + 0x14),
			 readl(camsv->base + off + 0x18), readl(camsv->base + off + 0x1c));
}

/* ---- register programming, cezanne map ---- */

static void camsv_hw_reset(struct mtk_camsv *camsv)
{
	u32 val;
	int i;

	/*
	 * The vendor's own flow for this block, register for register
	 * (camera_isp.c:1687-1702): SW_RST 1, SW_RST 0, IMGO_RST_TRIG 1, poll until the register
	 * reads 0x3 (reset done), then IMGO_RST_TRIG 0.
	 */
	writel(0x4, camsv->base + CAMSV_SW_CTL);
	writel(0x0, camsv->base + CAMSV_SW_CTL);
	writel(0x1, camsv->base + CAMSV_SW_CTL);

	for (i = 0; i < 100; i++) {
		val = readl(camsv->base + CAMSV_SW_CTL);
		if (val == 0x3)
			break;
		usleep_range(100, 200);
	}

	if (val != 0x3)
		dev_warn(camsv->dev, "reset did not reach 0x3: SW_CTL 0x%08x (0x%08llx)\n",
			 val, (unsigned long long)camsv->base_pa + CAMSV_SW_CTL);

	writel(0x0, camsv->base + CAMSV_SW_CTL);

	dev_info(camsv->dev, "reset: SW_CTL 0x%08x (0x%08llx)\n",
		 readl(camsv->base + CAMSV_SW_CTL),
		 (unsigned long long)camsv->base_pa + CAMSV_SW_CTL);
}


/* ---- register programming, cezanne map ---- */


/* One buffer into the FBC queue (enqueueHwBuf): its address, its frame header, then RCNT_INC. */
static void camsv_hw_next(struct mtk_camsv *camsv, dma_addr_t base)
{
	writel(lower_32_bits(base), camsv->base + CAMSV_FBC_IMGO_ENQ_ADDR);
	writel(lower_32_bits(camsv->scratch_dma), camsv->base + CAMSV_FRAME_WRITE_BASE);
	writel(readl(camsv->base + CAMSV_IMGO_FBC) | BIT(0), camsv->base + CAMSV_IMGO_FBC);
}

static void camsv_hw_start(struct mtk_camsv *camsv, dma_addr_t base, u32 w, u32 h)
{
	void __iomem *sv = camsv->base;

	/* TG: timestamps on, the window from (0, 0) to (w, h), no path options */
	writel(CAMSV_SEN_MODE_TIME_STP_EN, sv + CAMSV_TG_SEN_MODE);
	writel(w << 16, sv + CAMSV_TG_GRAB_PXL);
	writel(h << 16, sv + CAMSV_TG_GRAB_LIN);
	writel(0, sv + CAMSV_TG_PATH_CFG);

	readl(sv + CAMSV_INT_STATUS);
	writel(CAMSV_INT_MASK, sv + CAMSV_INT_EN);

	/* TOP: RAW10, the engine's clocks, packer mode 0 at one pixel per clock */
	writel(CAMSV_FMT_RAW10, sv + CAMSV_FMT_SEL);
	writel(CAMSV_CLK_EN_ALL, sv + CAMSV_CLK_EN);
	writel(readl(sv + CAMSV_PAK) & ~0x37u, sv + CAMSV_PAK);
	writel(0xffffffff, sv + CAMSV_DMA_ERR_CTL);

	/* IMGO: one frame per buffer, the engine's per-frame header into the scratch page */
	writel(1, sv + CAMSV_DMA_DB_EN);
	writel(0, sv + CAMSV_IMGO_OFST);
	writel(camsv_line_bytes(w) - 1, sv + CAMSV_IMGO_XSIZE);
	writel(h - 1, sv + CAMSV_IMGO_YSIZE);
	writel(camsv_stride(w), sv + CAMSV_IMGO_STRIDE);
	writel(0x80000080, sv + CAMSV_IMGO_CON);
	writel(0x00020002, sv + CAMSV_IMGO_CON2);
	writel(0x00020002, sv + CAMSV_IMGO_CON3);
	writel(lower_32_bits(base), sv + CAMSV_IMGO_BASE_ADDR);

	/* enables in the vendor's order: TG and packer, the TG's CMOS input, then IMGO */
	writel(CAMSV_MODULE_EN_DEFAULT | CAMSV_MODULE_EN_TG | CAMSV_MODULE_EN_PAK,
	       sv + CAMSV_MODULE_EN);
	writel(CAMSV_SEN_MODE_TIME_STP_EN | CAMSV_SEN_MODE_CMOS_EN, sv + CAMSV_TG_SEN_MODE);
	writel(CAMSV_MODULE_EN_DEFAULT | CAMSV_MODULE_EN_TG | CAMSV_MODULE_EN_PAK |
	       CAMSV_MODULE_EN_IMGO, sv + CAMSV_MODULE_EN);

	/* FBC: mode, then enable (BUF_CTRL::_config / ::_enable), and hand over the first buffer */
	writel(CAMSV_FBC_MODE, sv + CAMSV_FBC_IMGO_CTL1);
	writel(CAMSV_FBC_MODE | CAMSV_FBC_EN | CAMSV_FBC_BIT22, sv + CAMSV_FBC_IMGO_CTL1);
	camsv_hw_next(camsv, base);

	writel(BIT(0), sv + CAMSV_TG_VF_CON);		/* VF start */

	dev_info(camsv->dev,
		 "engine armed %ux%u: MODULE_EN 0x%08x SEN_MODE 0x%08x VF 0x%08x GRAB 0x%08x/0x%08x XSIZE 0x%08x STRIDE 0x%08x FBC 0x%08x ENQ 0x%08x\n",
		 w, h, readl(sv + CAMSV_MODULE_EN), readl(sv + CAMSV_TG_SEN_MODE),
		 readl(sv + CAMSV_TG_VF_CON), readl(sv + CAMSV_TG_GRAB_PXL),
		 readl(sv + CAMSV_TG_GRAB_LIN), readl(sv + CAMSV_IMGO_XSIZE),
		 readl(sv + CAMSV_IMGO_STRIDE), readl(sv + CAMSV_FBC_IMGO_CTL1),
		 readl(sv + CAMSV_FBC_IMGO_ENQ_ADDR));
}

static void camsv_hw_stop(struct mtk_camsv *camsv)
{
	writel(0, camsv->base + CAMSV_INT_EN);
	readl(camsv->base + CAMSV_INT_STATUS);
	writel(0, camsv->base + CAMSV_TG_VF_CON);
	writel(0, camsv->base + CAMSV_TG_SEN_MODE);
	writel(CAMSV_MODULE_EN_DEFAULT, camsv->base + CAMSV_MODULE_EN);
	writel(0, camsv->base + CAMSV_FBC_IMGO_CTL1);
}

/* ---- vb2 ---- */

static int camsv_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			     unsigned int *num_planes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	struct mtk_camsv *camsv = vb2_get_drv_priv(q);
	unsigned int size = camsv->cur_fmt.fmt.pix.sizeimage;

	if (*num_planes) {
		if (sizes[0] < size)
			return -EINVAL;
		return 0;
	}

	*num_planes = 1;
	sizes[0] = size;

	return 0;
}

static int camsv_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_camsv *camsv = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = camsv->cur_fmt.fmt.pix.sizeimage;

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

static void camsv_buf_queue(struct vb2_buffer *vb)
{
	struct mtk_camsv *camsv = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	unsigned long flags;

	vbuf->field = V4L2_FIELD_NONE;

	/*
	 * vb2 hands every buffer queued before STREAMON to this callback first and only then calls
	 * start_streaming, so the list is the one source of buffers; the engine gets one at a time.
	 * While streaming with the engine starved, the new buffer goes straight to it.
	 */
	spin_lock_irqsave(&camsv->buf_lock, flags);
	if (vb2_start_streaming_called(vb->vb2_queue) && !camsv->cur_buf) {
		camsv->cur_buf = vbuf;
		camsv_hw_next(camsv, vb2_dma_contig_plane_dma_addr(vb, 0));
	} else {
		list_add_tail(&to_camsv_buf(vbuf)->list, &camsv->bufs);
	}
	spin_unlock_irqrestore(&camsv->buf_lock, flags);
}

static int camsv_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mtk_camsv *camsv = vb2_get_drv_priv(q);
	int ret;

	ret = media_pipeline_start(&camsv->subdev.entity.pads[0], &camsv->pipeline);
	dev_info(camsv->dev, "media_pipeline_start: %d\n", ret);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(camsv->dev);
	if (ret)
		goto err_pipeline;

	if (camsv->main_clk) {
		ret = clk_prepare_enable(camsv->main_clk);
		dev_info(camsv->dev, "cam main clock: %d, %lu Hz\n", ret,
			 clk_get_rate(camsv->main_clk));
	}
	if (camsv->tg_clk) {
		ret = clk_prepare_enable(camsv->tg_clk);
		dev_info(camsv->dev, "cam tg clock: %d, %lu Hz\n", ret,
			 clk_get_rate(camsv->tg_clk));
	}
	if (camsv->gals_clk) {
		ret = clk_prepare_enable(camsv->gals_clk);
		dev_info(camsv->dev, "cam gals clock: %d, %lu Hz\n", ret,
			 clk_get_rate(camsv->gals_clk));
	}

	/*
	 * Start the chain from the source end: the receiver's s_stream() programs the CSI-2 side and
	 * calls its own upstream, which is the sensor. Without this the engine is armed on an idle
	 * bus - the rate that disp42 measured (0 frames, 0 interrupts).
	 */
	{
		struct media_pad *remote =
			media_pad_remote_pad_unique(&camsv->subdev.entity.pads[0]);

		if (IS_ERR(remote)) {
			dev_err(camsv->dev, "no remote pad: %ld\n", PTR_ERR(remote));
		} else {
			struct v4l2_subdev *sd =
				media_entity_to_v4l2_subdev(remote->entity);

			ret = v4l2_subdev_call(sd, video, s_stream, 1);
			dev_info(camsv->dev, "s_stream upstream %s: %d\n", sd->name, ret);
		}
	}

	camsv->sequence = 0;
	camsv->irq_count = 0;
	camsv->sof_count = 0;
	camsv->err_count = 0;
	camsv->err_status = 0;
	camsv->cur_buf = NULL;
	camsv_hw_reset(camsv);

	/* The vendor mask is written here, not only when a buffer happens to be armed first
	 * (cezanne map, 5-cameras.md §3.2: INT_EN 0x0018 = 0x31871), so the engine can report a
	 * frame even if userspace queues late. */
	/* DIAG: which of the engine's first 0x100 offsets are live on this generation. */
	if (camsv->stream_dump)
		camsv_dump_engine(camsv);

	dev_info(camsv->dev, "stream on: %u buffers, %ux%u, INT_EN 0x%08x (0x%08llx)\n", count,
		 camsv->cur_fmt.fmt.pix.width, camsv->cur_fmt.fmt.pix.height,
		 readl(camsv->base + CAMSV_INT_EN),
		 (unsigned long long)camsv->base_pa + CAMSV_INT_EN);

	/* A buffer queued before streamon starts the engine, the rest follow one per frame. */
	camsv->status_seen = 0;
	spin_lock_irq(&camsv->buf_lock);
	camsv->cur_buf = camsv_take_buf(camsv);
	spin_unlock_irq(&camsv->buf_lock);
	if (camsv->cur_buf) {
		dma_addr_t addr = vb2_dma_contig_plane_dma_addr(&camsv->cur_buf->vb2_buf, 0);

		dev_info(camsv->dev, "arm buffer %u at %pad, size %zu\n",
			 camsv->cur_buf->vb2_buf.index, &addr,
			 vb2_plane_size(&camsv->cur_buf->vb2_buf, 0));
		camsv_hw_start(camsv, addr, camsv->cur_fmt.fmt.pix.width,
			       camsv->cur_fmt.fmt.pix.height);
	}

	return 0;

err_pipeline:
	media_pipeline_stop(&camsv->subdev.entity.pads[0]);

	return ret;
}

static void camsv_stop_streaming(struct vb2_queue *q)
{
	struct mtk_camsv *camsv = vb2_get_drv_priv(q);

	struct vb2_v4l2_buffer *buf;

	{
		struct media_pad *remote =
			media_pad_remote_pad_unique(&camsv->subdev.entity.pads[0]);

		if (!IS_ERR(remote))
			v4l2_subdev_call(media_entity_to_v4l2_subdev(remote->entity),
					 video, s_stream, 0);
	}

	camsv_hw_stop(camsv);
	if (camsv->gals_clk)
		clk_disable_unprepare(camsv->gals_clk);
	if (camsv->tg_clk)
		clk_disable_unprepare(camsv->tg_clk);
	if (camsv->main_clk)
		clk_disable_unprepare(camsv->main_clk);
	media_pipeline_stop(&camsv->subdev.entity.pads[0]);

	/*
	 * Hand every buffer the queue still owns back exactly once. `vb2_buffer_done()` removes the
	 * buffer from queued_list, so this walk terminates; an index-based accessor would not (it
	 * returns the same buffer for ever, which is what made a killed stream spin in the kernel).
	 */
	spin_lock_irq(&camsv->buf_lock);
	while ((buf = camsv_take_buf(camsv)))
		vb2_buffer_done(&buf->vb2_buf, VB2_BUF_STATE_ERROR);
	buf = camsv->cur_buf;
	camsv->cur_buf = NULL;
	spin_unlock_irq(&camsv->buf_lock);
	if (buf)
		vb2_buffer_done(&buf->vb2_buf, VB2_BUF_STATE_ERROR);

	dev_info(camsv->dev, "stream off, %u frames, %u interrupts, %u SOF, %u errors (status 0x%08x, all seen 0x%08x), TG 0x%08x, FBC 0x%08x/0x%08x\n",
		 camsv->sequence, camsv->irq_count, camsv->sof_count, camsv->err_count,
		 camsv->err_status, camsv->status_seen, readl(camsv->base + CAMSV_TG_INTER_ST),
		 readl(camsv->base + CAMSV_FBC_IMGO_CTL1), readl(camsv->base + CAMSV_FBC_IMGO_CTL2));

	pm_runtime_put_autosuspend(camsv->dev);
}

static const struct vb2_ops camsv_queue_ops = {
	.queue_setup = camsv_queue_setup,
	.buf_prepare = camsv_buf_prepare,
	.buf_queue = camsv_buf_queue,
	.start_streaming = camsv_start_streaming,
	.stop_streaming = camsv_stop_streaming,
};

/* ---- video node ---- */

static int camsv_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	struct mtk_camsv *camsv = video_drvdata(file);

	strscpy(cap->driver, "mtk-camsv", sizeof(cap->driver));
	strscpy(cap->card, dev_name(camsv->dev), sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", dev_name(camsv->dev));

	return 0;
}

static int camsv_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_SGRBG10P;
	strscpy(f->description, "10-bit Bayer GRBG packed", sizeof(f->description));

	return 0;
}

static int camsv_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mtk_camsv *camsv = video_drvdata(file);

	*f = camsv->cur_fmt;

	return 0;
}

static int camsv_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width = clamp_t(u32, pix->width, 16, CAMSV_MAX_WIDTH) & ~1U;
	pix->height = clamp_t(u32, pix->height, 16, CAMSV_MAX_HEIGHT) & ~1U;
	/*
	 * RAW10 packed by the engine, four pixels in five bytes (FMT_SEL 1, packer mode 0); the IMX355
	 * reads out GRBG. Whether the byte order is the MIPI one this fourcc names or MediaTek's own
	 * bit stream is checked on a captured frame, not assumed.
	 */
	pix->pixelformat = V4L2_PIX_FMT_SGRBG10P;
	pix->bytesperline = camsv_stride(pix->width);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_RAW;

	return 0;
}

static int camsv_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mtk_camsv *camsv = video_drvdata(file);
	int ret;

	ret = camsv_try_fmt(file, priv, f);
	if (ret)
		return ret;

	if (vb2_is_busy(&camsv->queue))
		return -EBUSY;

	camsv->cur_fmt = *f;

	return 0;
}

static const struct v4l2_ioctl_ops camsv_ioctl_ops = {
	.vidioc_querycap = camsv_querycap,
	.vidioc_enum_fmt_vid_cap = camsv_enum_fmt,
	.vidioc_g_fmt_vid_cap = camsv_g_fmt,
	.vidioc_try_fmt_vid_cap = camsv_try_fmt,
	.vidioc_s_fmt_vid_cap = camsv_s_fmt,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations camsv_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

static const struct v4l2_subdev_ops camsv_subdev_ops = { };

/* ---- interrupt ---- */

static irqreturn_t mtk_camsv_irq(int irq, void *data)
{
	struct mtk_camsv *camsv = data;
	u32 status;

	camsv->irq_count++;

	status = readl(camsv->base + CAMSV_INT_STATUS);	/* read-clear */
	if (!status)
		return IRQ_NONE;

	camsv->status_seen |= status;
	if (status & CAMSV_INT_SOF)
		camsv->sof_count++;
	if (status & CAMSV_INT_ERRS) {
		camsv->err_count++;
		camsv->err_status |= status;
		if (camsv->err_count == CAMSV_ERR_STORM) {
			writel(readl(camsv->base + CAMSV_INT_EN) & ~CAMSV_INT_ERRS,
			       camsv->base + CAMSV_INT_EN);
			dev_warn(camsv->dev, "error interrupts masked after %u (0x%08x), %u frames\n",
				 camsv->err_count, camsv->err_status, camsv->sequence);
		}
	}

	/* Frame written: complete it and give the engine the next queued buffer. */
	spin_lock(&camsv->buf_lock);
	if ((status & CAMSV_INT_PASS1_DONE) && camsv->cur_buf) {
		struct vb2_v4l2_buffer *done = camsv->cur_buf;
		struct vb2_v4l2_buffer *next = camsv_take_buf(camsv);

		camsv->cur_buf = next;
		if (next)
			camsv_hw_next(camsv, vb2_dma_contig_plane_dma_addr(&next->vb2_buf, 0));
		done->vb2_buf.timestamp = ktime_get_ns();
		done->sequence = camsv->sequence++;
		vb2_buffer_done(&done->vb2_buf, VB2_BUF_STATE_DONE);
	}
	spin_unlock(&camsv->buf_lock);

	return IRQ_HANDLED;
}

/* ---- power ---- */

static int mtk_camsv_runtime_resume(struct device *dev)
{
	struct mtk_camsv *camsv = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(ARRAY_SIZE(camsv->clks), camsv->clks);
}

static int mtk_camsv_runtime_suspend(struct device *dev)
{
	struct mtk_camsv *camsv = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(ARRAY_SIZE(camsv->clks), camsv->clks);

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(mtk_camsv_pm_ops, mtk_camsv_runtime_suspend,
				 mtk_camsv_runtime_resume, NULL);

/* ---- probe ---- */

static int mtk_camsv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_camsv *camsv;
	int ret;

	camsv = devm_kzalloc(dev, sizeof(*camsv), GFP_KERNEL);
	if (!camsv)
		return -ENOMEM;

	camsv->dev = dev;
	camsv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(camsv->base))
		return PTR_ERR(camsv->base);

	camsv->base_pa = platform_get_resource(pdev, IORESOURCE_MEM, 0)->start;
	camsv->stream_dump = true;
	camsv->irq = platform_get_irq(pdev, 0);
	if (camsv->irq < 0)
		return camsv->irq;

	camsv->clks[0].id = "sv";

	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(camsv->clks), camsv->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get the engine clock\n");

	/*
	 * The engine runs on the camera subsystem's gates: `main` for the block and `tg` for the
	 * timing generator (the vendor's CAMSV path enables both; without them its SW_CTL reset never
	 * reports idle, which is what the phone measured as "reset did not settle").
	 */
	camsv->main_clk = devm_clk_get_optional(dev, "main");
	camsv->tg_clk = devm_clk_get_optional(dev, "tg");
	camsv->gals_clk = devm_clk_get_optional(dev, "gals");

	/* the engine writes a per-frame header to FRAME_WRITE_BASE: give it memory, not address 0 */
	camsv->scratch = dmam_alloc_coherent(dev, CAMSV_SCRATCH_SIZE, &camsv->scratch_dma, GFP_KERNEL);
	if (!camsv->scratch)
		return -ENOMEM;

	mutex_init(&camsv->vdev_lock);
	INIT_LIST_HEAD(&camsv->bufs);
	spin_lock_init(&camsv->buf_lock);
	platform_set_drvdata(pdev, camsv);

	camsv->cur_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	camsv->cur_fmt.fmt.pix.width = CAMSV_DEF_WIDTH;
	camsv->cur_fmt.fmt.pix.height = CAMSV_DEF_HEIGHT;
	camsv_try_fmt(NULL, NULL, &camsv->cur_fmt);

	camsv->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	camsv->queue.io_modes = VB2_MMAP | VB2_DMABUF;
	/*
	 * vb2_queue_init() warns when the timestamp flavour is left UNKNOWN
	 * (videobuf2-v4l2.c:909-911) and fails the probe with -EINVAL: a capture queue has to say
	 * where its timestamps come from. The engine's frame-done interrupt takes ktime_get_ns(),
	 * so they are monotonic and start-of-frame.
	 */
	camsv->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
				       V4L2_BUF_FLAG_TSTAMP_SRC_SOE;
	camsv->queue.drv_priv = camsv;
	camsv->queue.buf_struct_size = sizeof(struct camsv_buf);
	camsv->queue.ops = &camsv_queue_ops;
	camsv->queue.mem_ops = &vb2_dma_contig_memops;
	camsv->queue.dev = dev;
	camsv->queue.lock = &camsv->vdev_lock;

	ret = vb2_queue_init(&camsv->queue);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialise the queue\n");

	/* The sink side of the pipeline: linked from the receiver's source pad. */
	camsv->subdev.dev = dev;
	v4l2_subdev_init(&camsv->subdev, &camsv_subdev_ops);
	camsv->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	camsv->subdev.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	strscpy(camsv->subdev.name, dev_name(dev), sizeof(camsv->subdev.name));
	/*
	 * Pad 0 takes the stream from the receiver, pad 1 carries it to the video node: a link needs a
	 * source pad on the source side (mc-entity.c:1147 warns otherwise).
	 */
	camsv->pads[0].flags = MEDIA_PAD_FL_SINK;
	camsv->pads[1].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&camsv->subdev.entity, 2, camsv->pads);
	if (ret)
		return ret;

	/*
	 * The video device needs the pipeline's v4l2_device: video_register_device() warns and fails
	 * on a NULL one (drivers/media/v4l2-core/v4l2-dev.c:929). The core is a separate driver, so
	 * before it has probed this one defers rather than registers without it.
	 */
	camsv->cam = mtk_cam_core_get(dev);
	if (!camsv->cam)
		return -EPROBE_DEFER;

	strscpy(camsv->vdev.name, "mtk-camsv", sizeof(camsv->vdev.name));
	camsv->vdev.v4l2_dev = NULL;
	camsv->vdev.fops = &camsv_fops;
	camsv->vdev.ioctl_ops = &camsv_ioctl_ops;
	camsv->vdev.release = video_device_release_empty;
	camsv->vdev.lock = &camsv->vdev_lock;
	camsv->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
				  V4L2_CAP_READWRITE;
	camsv->vdev.queue = &camsv->queue;
	video_set_drvdata(&camsv->vdev, camsv);

	camsv->vdev.v4l2_dev = &camsv->cam->v4l2_dev;

	/*
	 * The video node is an entity of the graph: it needs a sink pad of its own before it is
	 * registered, or the link from the engine's subdevice is rejected (mc-entity.c:1143-1146
	 * warns on a pad index past num_pads).
	 */
	camsv->vdev_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&camsv->vdev.entity, 1, &camsv->vdev_pad);
	if (ret)
		goto err_subdev;

	ret = video_register_device(&camsv->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_subdev;

	/* The pipeline's async notifier collects the entities and makes the links. */
	ret = v4l2_async_register_subdev(&camsv->subdev);
	if (ret)
		goto err_video;

	ret = devm_request_irq(dev, camsv->irq, mtk_camsv_irq, 0, dev_name(dev), camsv);
	if (ret)
		goto err_video;

	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		goto err_video;

	/*
	 * The video node is an entity of its own in the graph; without a link from the engine's
	 * subdevice the pipeline cannot be validated at STREAMON.
	 */
	if (!media_create_pad_link(&camsv->subdev.entity, 1, &camsv->vdev.entity, 0,
				   MEDIA_LNK_FL_ENABLED))
		dev_info(dev, "link %s -> video%d\n", camsv->subdev.name,
			 camsv->vdev.num);

	dev_info(dev, "raw capture on /dev/video%d\n", camsv->vdev.num);

	return 0;

err_video:
	video_unregister_device(&camsv->vdev);
err_subdev:
	media_entity_cleanup(&camsv->subdev.entity);

	return ret;
}

static void mtk_camsv_remove(struct platform_device *pdev)
{
	struct mtk_camsv *camsv = platform_get_drvdata(pdev);

	video_unregister_device(&camsv->vdev);
	v4l2_async_unregister_subdev(&camsv->subdev);
	media_entity_cleanup(&camsv->subdev.entity);
}

static const struct of_device_id mtk_camsv_of_ids[] = {
	{ .compatible = "mediatek,mt6785-camsv" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_camsv_of_ids);

static struct platform_driver mtk_camsv_driver = {
	.probe = mtk_camsv_probe,
	.remove = mtk_camsv_remove,
	.driver = {
		.name = "mtk-camsv",
		.of_match_table = mtk_camsv_of_ids,
		.pm = pm_ptr(&mtk_camsv_pm_ops),
	},
};
module_platform_driver(mtk_camsv_driver);

MODULE_DESCRIPTION("MediaTek CAMSV raw capture");
MODULE_LICENSE("GPL");
