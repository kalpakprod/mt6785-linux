// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek SCP sensorhub: the IPI client.
 *
 * The preload steps, in the order the vendor's stack does them
 * (scratch/wt/sensors/tmp/vendor/sensors/SCP_nanoHub.c, and the cezanne port at
 * cezanne-linux-port/linux/drivers/scp/sensor/mtk_nanohub_ipi.c):
 *
 *  1. the AP maps a ring in DRAM and tells the SCP where it is with
 *     SENSOR_HUB_SET_CONFIG (SCP_sensorHub.h:340-354) - the firmware keeps the
 *     address and writes samples there;
 *  2. per sensor, SENSOR_HUB_ACTIVATE and SENSOR_HUB_SET_DELAY or the newer
 *     ConfigCmd with CONFIG_CMD_ENABLE (:58-70), and SENSOR_HUB_SET_CUST for
 *     calibration (:357-370);
 *  3. the SCP raises IPI 8 with SENSOR_HUB_NOTIFY (:495-511) carrying its write
 *     pointer, and the AP copies the frames from its read pointer to that write
 *     pointer and advances the read pointer.
 *
 * This file is the transport and the bookkeeping; the IIO devices that consume
 * the frames come in the next step, which is why every stage logs its own line:
 * on a trial that stalls, dmesg then says which of the three stopped.
 */
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/remoteproc/mtk_scp.h>

#include "mtk-sensorhub.h"

/* the vendor's IPI id for the hub: enum ipi_id, scp_cm4.h:17-38 */
#define MTK_SH_IPI_ID		8
/* how long an ACK may take before the client gives up on that message */
#define MTK_SH_ACK_TIMEOUT_MS	200
/* the ring the vendor reserves for the hub: SENS_MEM_ID, 1 MiB */
#define MTK_SH_RING_MIN_SIZE	SZ_4K

static void mtk_sh_note_first_frame(struct mtk_sh *sh, u8 type)
{
	if (type >= MTK_SH_TYPE_MAX || test_and_set_bit(type, sh->first_seen))
		return;

	dev_info(sh->dev, "first frame of sensor type %u arrived\n", type);
}

/*
 * Read the bytes the SCP has written since our read pointer into sh->buf and
 * walk the frames there: the parser needs whole frames, and the ring can wrap,
 * so the copy is what makes the walk linear.
 */
static void mtk_sh_drain(struct mtk_sh *sh)
{
	struct mtk_sh_ring __iomem *hdr = sh->ring;
	u32 rp = readl(&hdr->rp), wp = readl(&hdr->wp), size = readl(&hdr->size);
	size_t len, pos = 0;
	u32 next;
	struct mtk_sh_frame frame;
	int ret;

	if (mtk_sh_ring_valid(rp, wp, size)) {
		dev_warn_once(sh->dev, "firmware reported an unusable ring\n");
		return;
	}

	if (wp > rp) {
		len = wp - rp;
	} else if (wp < rp) {
		/* wrapped: take the tail now, the head on the next round */
		len = size - rp;
	} else {
		return;
	}

	if (len > sh->ring_size)
		len = sh->ring_size;

	memcpy_fromio(sh->buf, sh->ring + rp, len);

	while (pos < len) {
		ret = mtk_sh_next_frame(sh->buf, len, pos, &frame, &next);
		if (ret) {
			sh->errors++;
			dev_warn_ratelimited(sh->dev,
					     "dropping the ring at %zu (%d)\n",
					     pos, ret);
			pos = (wp < rp) ? size : wp;
			break;
		}
		pos = next;
		sh->frames++;
		mtk_sh_note_first_frame(sh, frame.type);
		mtk_sh_iio_push(sh, &frame);
	}

	if (pos >= len)
		writel((wp < rp) ? 0 : wp, &hdr->rp);

	sh->last_wp = wp;
	dev_dbg(sh->dev, "ring drained to %zu (rp %u wp %u, %u frames, %u errors)\n",
		pos, rp, wp, sh->frames, sh->errors);
}

static void mtk_sh_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct mtk_sh *sh = priv;

	dev_dbg(sh->dev, "notify, %u bytes\n", len);
	mtk_sh_drain(sh);
}

int mtk_sh_send(struct mtk_sh *sh, const u8 *msg, int len, const char *what)
{
	int ret;

	if (len < 0)
		return len;

	ret = scp_ipi_send(sh->scp, MTK_SH_IPI_ID, msg, len,
			   MTK_SH_ACK_TIMEOUT_MS);
	if (ret)
		dev_warn(sh->dev, "%s: ipi_send failed %d\n", what, ret);
	else
		dev_dbg(sh->dev, "%s sent (%d bytes)\n", what, len);

	return ret;
}

static int mtk_sh_set_config(struct mtk_sh *sh, u8 type)
{
	u8 msg[MTK_SH_SET_CONFIG_LEN];
	int len;

	len = mtk_sh_encode_set_config(msg, sizeof(msg), type,
				       (u32)sh->ring_phys, (u32)sh->ring_size,
				       0, ktime_get_ns());
	return mtk_sh_send(sh, msg, len, "SET_CONFIG");
}

int mtk_sh_activate(struct mtk_sh *sh, u8 type, bool enable)
{
	u8 msg[MTK_SH_ACTIVATE_LEN];
	int len;

	len = mtk_sh_encode_activate(msg, sizeof(msg), type, enable);
	return mtk_sh_send(sh, msg, len, enable ? "ACTIVATE on" : "ACTIVATE off");
}

int mtk_sh_set_delay(struct mtk_sh *sh, u8 type, u32 delay_ms)
{
	u8 msg[MTK_SH_SET_DELAY_LEN];
	int len;

	len = mtk_sh_encode_set_delay(msg, sizeof(msg), type, delay_ms);
	return mtk_sh_send(sh, msg, len, "SET_DELAY");
}

/*
 * The sensors this board has, from the firmware's own strings: icm40607 (accel
 * and gyro), akm09918 (magnetometer), tsl2540 (light and proximity). The type
 * numbers are MediaTek's SENSOR_TYPE_* from the vendor's hwmsensor.h - branch
 * rosemary-r-oss of MiCode/Xiaomi_Kernel_OpenSource, commit
 * 7094834e4fca19af2ca43a752debf95c0022b949, lines 30-37 - which follow Android's
 * numbering; mtk-sensorhub-ring.h carries them with the same citation.
 */
static struct mtk_sh_sensor mtk_sh_sensors[] = {
	{ "accel", MTK_SH_TYPE_ACCEL, 50 },
	{ "gyro", MTK_SH_TYPE_GYRO, 50 },
	{ "mag", MTK_SH_TYPE_MAG, 10 },
	{ "als", MTK_SH_TYPE_LIGHT, 5 },
	{ "ps", MTK_SH_TYPE_PROXIMITY, 5 },
};

static int mtk_sh_debugfs_show(struct seq_file *s, void *unused)
{
	struct mtk_sh *sh = s->private;
	struct mtk_sh_ring __iomem *hdr = sh->ring;

	seq_printf(s, "ring phys %pa size %zu\n", &sh->ring_phys, sh->ring_size);
	seq_printf(s, "rp %u wp %u size %u last_wp %u\n",
		   readl(&hdr->rp), readl(&hdr->wp), readl(&hdr->size),
		   sh->last_wp);
	seq_printf(s, "frames %u errors %u\n", sh->frames, sh->errors);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_sh_debugfs);

static int mtk_sh_map_ring(struct mtk_sh *sh, struct platform_device *pdev)
{
	struct device_node *np;
	struct resource res;
	int ret;

	np = of_parse_phandle(sh->dev->of_node, "memory-region", 0);
	if (!np)
		return dev_err_probe(sh->dev, -EINVAL,
				     "no memory-region for the sample ring\n");

	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret)
		return dev_err_probe(sh->dev, ret, "cannot read the ring's region\n");

	sh->ring_size = resource_size(&res);
	if (sh->ring_size < MTK_SH_RING_MIN_SIZE)
		return dev_err_probe(sh->dev, -EINVAL, "ring too small: %zu\n",
				     sh->ring_size);

	sh->buf = devm_kzalloc(sh->dev, sh->ring_size, GFP_KERNEL);
	if (!sh->buf)
		return -ENOMEM;

	sh->ring_phys = res.start;
	sh->ring = devm_ioremap(sh->dev, sh->ring_phys, sh->ring_size);
	if (!sh->ring)
		return dev_err_probe(sh->dev, -ENOMEM, "cannot map the ring\n");

	/* a ring left over from a previous boot may hold stale pointers */
	writel(0, &((struct mtk_sh_ring __iomem *)sh->ring)->rp);
	writel(0, &((struct mtk_sh_ring __iomem *)sh->ring)->wp);
	writel(sh->ring_size, &((struct mtk_sh_ring __iomem *)sh->ring)->size);

	dev_info(sh->dev, "ring mapped: %pa, %zu bytes\n", &sh->ring_phys,
		 sh->ring_size);

	return 0;
}

static int mtk_sh_probe(struct platform_device *pdev)
{
	struct mtk_sh *sh;
	size_t i;
	int ret;

	sh = devm_kzalloc(&pdev->dev, sizeof(*sh), GFP_KERNEL);
	if (!sh)
		return -ENOMEM;

	sh->dev = &pdev->dev;
	sh->sensors = mtk_sh_sensors;
	sh->num_sensors = ARRAY_SIZE(mtk_sh_sensors);
	mutex_init(&sh->lock);

	sh->scp = scp_get(pdev);
	if (!sh->scp)
		return -EPROBE_DEFER;

	ret = mtk_sh_map_ring(sh, pdev);
	if (ret)
		return ret;

	ret = scp_ipi_register(sh->scp, MTK_SH_IPI_ID, mtk_sh_ipi_handler, sh);
	if (ret) {
		dev_err_probe(sh->dev, ret, "cannot register IPI %d\n",
			      MTK_SH_IPI_ID);
		return ret;
	}
	dev_info(sh->dev, "IPI %d registered\n", MTK_SH_IPI_ID);

	sh->debugfs = debugfs_create_dir(dev_name(sh->dev), NULL);
	debugfs_create_file("ring", 0444, sh->debugfs, sh, &mtk_sh_debugfs_fops);

	platform_set_drvdata(pdev, sh);

	/* the ring is handed to the firmware once, before any sensor is used */
	for (i = 0; i < sh->num_sensors; i++) {
		ret = mtk_sh_set_config(sh, sh->sensors[i].type);
		if (ret)
			return ret;
	}

	/*
	 * The IIO devices are what the user sees; each sensor is activated when
	 * its buffer is enabled, so nothing is switched on at probe.
	 */
	return mtk_sh_iio_register(sh);
}

static void mtk_sh_remove(struct platform_device *pdev)
{
	struct mtk_sh *sh = platform_get_drvdata(pdev);

	debugfs_remove_recursive(sh->debugfs);
	scp_ipi_unregister(sh->scp, MTK_SH_IPI_ID);
}

static const struct of_device_id mtk_sh_of_match[] = {
	{ .compatible = "mediatek,mt6785-sensorhub" },
	{}
};
MODULE_DEVICE_TABLE(of, mtk_sh_of_match);

static struct platform_driver mtk_sh_driver = {
	.probe = mtk_sh_probe,
	.remove = mtk_sh_remove,
	.driver = {
		.name = "mtk6785-sensorhub",
		.of_match_table = mtk_sh_of_match,
	},
};
module_platform_driver(mtk_sh_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek SCP sensorhub IPI client");
