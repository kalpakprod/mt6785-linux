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
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

/* CAMSV registers, cezanne map (§3.2). */
#define CAMSV_MODULE_EN			0x0010
#define CAMSV_FMT_SEL			0x0014
#define CAMSV_INT_EN			0x0018
#define CAMSV_INT_STATUS		0x001c
#define CAMSV_SW_CTL			0x0020
#define CAMSV_IMGO_FBC			0x002c
#define CAMSV_CLK_EN			0x0030
#define CAMSV_PAK			0x004c
#define CAMSV_TG_SEN_MODE		0x0130
#define CAMSV_TG_VF_CON			0x0134
#define CAMSV_TG_GRAB_PXL		0x0138
#define CAMSV_TG_GRAB_LIN		0x013c
#define CAMSV_TG_PATH_CFG		0x0140
#define CAMSV_TG_INTER_ST		0x016c
#define CAMSV_IMGO_BASE			0x0420
#define CAMSV_IMGO_BASE_MSB		0x0424
#define CAMSV_IMGO_OFST			0x0428

/* MODULE_EN bits (§3.2): TG, PAK, unpack, IMGO, DB. */
#define CAMSV_MODULE_EN_TG		BIT(0)
#define CAMSV_MODULE_EN_PAK		BIT(2)
#define CAMSV_MODULE_EN_UNPACK		BIT(3)
#define CAMSV_MODULE_EN_IMGO		BIT(4)

#define CAMSV_CLK_EN_VAL		0x8005
/* PAK modes (§3.2): 8 RAW8, 9 packed RAW10, 13 unpacked RAW10. */
#define CAMSV_PAK_RAW10			9
/* TG_INTER_ST: bits [13:8] carry the TG state, 0x100 means idle. */
#define CAMSV_TG_IDLE			0x100

#define CAMSV_MAX_WIDTH			4208
#define CAMSV_MAX_HEIGHT		3120
#define CAMSV_DEF_WIDTH			1920
#define CAMSV_DEF_HEIGHT		1080

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
	int irq;

	struct video_device vdev;
	struct v4l2_subdev subdev;
	struct media_pad pads[1];
	struct media_device mdev;
	struct mutex vdev_lock;

	struct vb2_queue queue;
	struct v4l2_format cur_fmt;
	struct v4l2_mbus_framefmt mbus_fmt;
	u32 sequence;
};

static inline struct mtk_camsv *to_camsv(struct video_device *vdev)
{
	return container_of(vdev, struct mtk_camsv, vdev);
}

/* ---- register programming, cezanne map ---- */

static void camsv_hw_reset(struct mtk_camsv *camsv)
{
	u32 val;
	int i;

	/* SW_CTL: bit0 reset trigger, bit1 reset status (1 = idle), bit2 hw reset. */
	writel(BIT(2) | BIT(0), camsv->base + CAMSV_SW_CTL);

	for (i = 0; i < 100; i++) {
		val = readl(camsv->base + CAMSV_SW_CTL);
		if (val & BIT(1))
			return;
		usleep_range(100, 200);
	}

	dev_warn(camsv->dev, "reset did not settle: SW_CTL 0x%08x\n", val);
}

static void camsv_hw_start(struct mtk_camsv *camsv, dma_addr_t base, u32 w, u32 h)
{
	/* Frame size first, then the packing, then the destination. */
	writel((w << 16), camsv->base + CAMSV_TG_GRAB_PXL);
	writel((h << 16), camsv->base + CAMSV_TG_GRAB_LIN);
	writel(CAMSV_PAK_RAW10, camsv->base + CAMSV_PAK);
	writel(CAMSV_CLK_EN_VAL, camsv->base + CAMSV_CLK_EN);

	writel(lower_32_bits(base), camsv->base + CAMSV_IMGO_BASE);
	writel(upper_32_bits(base), camsv->base + CAMSV_IMGO_BASE_MSB);
	writel(0, camsv->base + CAMSV_IMGO_OFST);

	writel(BIT(0) | BIT(16), camsv->base + CAMSV_TG_SEN_MODE);
	writel(CAMSV_MODULE_EN_TG | CAMSV_MODULE_EN_PAK | CAMSV_MODULE_EN_UNPACK |
	       CAMSV_MODULE_EN_IMGO, camsv->base + CAMSV_MODULE_EN);
	writel(BIT(0), camsv->base + CAMSV_TG_VF_CON);
	writel(0x1, camsv->base + CAMSV_INT_EN);
}

static void camsv_hw_stop(struct mtk_camsv *camsv)
{
	writel(0, camsv->base + CAMSV_TG_VF_CON);
	writel(0, camsv->base + CAMSV_MODULE_EN);
	writel(0, camsv->base + CAMSV_INT_EN);
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
	dma_addr_t addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	u32 w = camsv->cur_fmt.fmt.pix.width;
	u32 h = camsv->cur_fmt.fmt.pix.height;

	/* The first buffer owns the engine: program it and let the frame arrive. */
	if (vb->index == 0 || vb2_is_streaming(vb->vb2_queue)) {
		camsv_hw_start(camsv, addr, w, h);
		vbuf->field = V4L2_FIELD_NONE;
	}
}

static int camsv_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mtk_camsv *camsv = vb2_get_drv_priv(q);
	int ret;

	ret = pm_runtime_resume_and_get(camsv->dev);
	if (ret)
		return ret;

	camsv->sequence = 0;
	camsv_hw_reset(camsv);

	dev_dbg(camsv->dev, "streaming on, %u buffers\n", count);

	return 0;
}

static void camsv_stop_streaming(struct vb2_queue *q)
{
	struct mtk_camsv *camsv = vb2_get_drv_priv(q);

	camsv_hw_stop(camsv);
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

	f->pixelformat = V4L2_PIX_FMT_SBGGR10;
	strscpy(f->description, "10-bit Bayer BGGR", sizeof(f->description));

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
	pix->pixelformat = V4L2_PIX_FMT_SBGGR10;
	pix->bytesperline = pix->width * 2;
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

	status = readl(camsv->base + CAMSV_INT_STATUS);
	if (!status)
		return IRQ_NONE;

	writel(status, camsv->base + CAMSV_INT_STATUS);

	dev_dbg(camsv->dev, "frame done, INT_STATUS 0x%08x\n", status);

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

	camsv->irq = platform_get_irq(pdev, 0);
	if (camsv->irq < 0)
		return camsv->irq;

	camsv->clks[0].id = "sv";

	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(camsv->clks), camsv->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get the engine clock\n");

	mutex_init(&camsv->vdev_lock);
	platform_set_drvdata(pdev, camsv);

	camsv->cur_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	camsv->cur_fmt.fmt.pix.width = CAMSV_DEF_WIDTH;
	camsv->cur_fmt.fmt.pix.height = CAMSV_DEF_HEIGHT;
	camsv_try_fmt(NULL, NULL, &camsv->cur_fmt);

	camsv->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	camsv->queue.io_modes = VB2_MMAP | VB2_DMABUF;
	camsv->queue.drv_priv = camsv;
	camsv->queue.buf_struct_size = sizeof(struct vb2_v4l2_buffer);
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
	strscpy(camsv->subdev.name, dev_name(dev), sizeof(camsv->subdev.name));
	camsv->pads[0].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&camsv->subdev.entity, 1, camsv->pads);
	if (ret)
		return ret;

	camsv->mdev.dev = dev;
	strscpy(camsv->mdev.model, "mtk-camsv", sizeof(camsv->mdev.model));
	media_device_init(&camsv->mdev);

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

	ret = video_register_device(&camsv->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_media;

	ret = media_device_register(&camsv->mdev);
	if (ret)
		goto err_video;

	ret = devm_request_irq(dev, camsv->irq, mtk_camsv_irq, 0, dev_name(dev), camsv);
	if (ret)
		goto err_mdev;

	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		goto err_mdev;

	dev_info(dev, "raw capture on /dev/video%d\n", camsv->vdev.num);

	return 0;

err_mdev:
	media_device_unregister(&camsv->mdev);
err_video:
	video_unregister_device(&camsv->vdev);
err_media:
	media_device_cleanup(&camsv->mdev);

	return ret;
}

static void mtk_camsv_remove(struct platform_device *pdev)
{
	struct mtk_camsv *camsv = platform_get_drvdata(pdev);

	media_device_unregister(&camsv->mdev);
	video_unregister_device(&camsv->vdev);
	media_device_cleanup(&camsv->mdev);
	v4l2_device_unregister_subdev(&camsv->subdev);
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
