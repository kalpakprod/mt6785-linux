// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek camera pipeline core.
 *
 * The pipeline is a chain of subdevices described by the device tree graph: the sensor on its
 * I2C bus, the CSI-2 receiver (SENINF) and the capture engine (CAMSV). A media device has to
 * own all of them for the links to exist, and exactly one v4l2 device registers them; this
 * driver is that owner. The per-SoC drivers find it through the "mediatek,cam" phandle of
 * their device tree node.
 *
 * Links are created from the graph: the receiver's sink endpoint names the sensor endpoint it
 * is wired to, and its source endpoint names the engine. Nothing about a particular board is
 * in this file - the graphs are the board's.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "mtk_cam_core.h"

static struct mtk_cam_core *mtk_cam_core_from_np(struct device_node *np)
{
	struct platform_device *pdev;
	struct device_node *cam_np;
	struct device *dev;

	if (!np)
		return NULL;

	cam_np = of_parse_phandle(np, "mediatek,cam", 0);
	if (!cam_np)
		return NULL;

	pdev = of_find_device_by_node(cam_np);
	of_node_put(cam_np);
	if (!pdev)
		return NULL;

	dev = &pdev->dev;
	put_device(dev);

	return dev_get_drvdata(dev);
}

struct mtk_cam_core *mtk_cam_core_get(struct device *dev)
{
	return mtk_cam_core_from_np(dev->of_node);
}

void mtk_cam_core_put(struct mtk_cam_core *cam)
{
	(void)cam;
}

/* The registered subdevice whose device node is @np, or NULL. */
static struct v4l2_subdev *cam_subdev_by_node(struct mtk_cam_core *cam,
					      struct device_node *np)
{
	struct v4l2_subdev *sd;

	if (!np)
		return NULL;

	v4l2_device_for_each_subdev(sd, &cam->v4l2_dev) {
		if (sd->dev && sd->dev->of_node == np)
			return sd;
	}

	return NULL;
}

/*
 * Create the pad link from the entity described by @from_np's endpoint @from_port to the entity
 * of @to_np, using @to_pad. Returns 0 when the link exists or was made, -ENODEV when one side is
 * not registered yet (the caller retries once its peer probes).
 */
static int cam_link(struct mtk_cam_core *cam, struct device_node *from_np,
		    unsigned int from_port, unsigned int from_pad,
		    struct device_node *to_np, unsigned int to_pad)
{
	struct v4l2_subdev *from_sd, *to_sd;
	struct media_entity *from, *to;

	from_sd = cam_subdev_by_node(cam, from_np);
	to_sd = cam_subdev_by_node(cam, to_np);
	if (!from_sd || !to_sd)
		return -ENODEV;

	from = &from_sd->entity;
	to = &to_sd->entity;

	if (media_create_pad_link(from, from_pad, to, to_pad, 0)) {
		dev_dbg(cam->dev, "link %s:%u -> %s:%u already exists\n",
			from->name, from_pad, to->name, to_pad);
		return 0;
	}

	dev_info(cam->dev, "%s:%u -> %s:%u\n", from->name, from_pad, to->name, to_pad);

	return 0;
}

/*
 * Walk every subdevice whose node has an input port and link it to whatever its endpoint names.
 * Called from each driver after it registers, so a link appears as soon as both ends exist.
 */
int mtk_cam_core_link(struct mtk_cam_core *cam)
{
	struct v4l2_subdev *sd;
	int ret = 0;

	if (!cam)
		return -ENODEV;

	v4l2_device_for_each_subdev(sd, &cam->v4l2_dev) {
		struct device_node *np = sd->dev ? sd->dev->of_node : NULL;
		struct device_node *ep, *remote;
		unsigned int port;

		if (!np)
			continue;

		/* The sensor has one output port; the receiver's ports are handled below. */
		for (port = 0; port < 2; port++) {
			struct device_node *ports = of_get_child_by_name(np, "ports");

			if (!ports)
				continue;

			for_each_endpoint_of_node(ports, ep) {
				u32 reg = 0;

				of_property_read_u32(of_get_parent(ep), "reg", &reg);
				if (reg != port)
					continue;

				remote = of_graph_get_remote_endpoint(ep);
				if (!remote)
					continue;

				/* A link exists when the remote's parent entity is registered. */
				ret = cam_link(cam, of_graph_get_port_parent(remote), port,
					       port ? 0 : 0,
					       np, port ? 0 : 0);
				of_node_put(remote);
			}
			of_node_put(ports);
		}
	}

	return ret;
}

static int mtk_cam_core_probe(struct platform_device *pdev)
{
	struct mtk_cam_core *cam;
	int ret;

	cam = devm_kzalloc(&pdev->dev, sizeof(*cam), GFP_KERNEL);
	if (!cam)
		return -ENOMEM;

	cam->dev = &pdev->dev;

	cam->v4l2_dev.dev = &pdev->dev;
	strscpy(cam->v4l2_dev.name, "mtk-cam", sizeof(cam->v4l2_dev.name));
	ret = v4l2_device_register(&pdev->dev, &cam->v4l2_dev);
	if (ret)
		return ret;

	cam->mdev.dev = &pdev->dev;
	strscpy(cam->mdev.model, "mtk-cam", sizeof(cam->mdev.model));
	cam->v4l2_dev.mdev = &cam->mdev;
	media_device_init(&cam->mdev);

	ret = media_device_register(&cam->mdev);
	if (ret)
		goto err_v4l2;

	platform_set_drvdata(pdev, cam);

	dev_info(&pdev->dev, "camera pipeline core registered\n");

	return 0;

err_v4l2:
	v4l2_device_unregister(&cam->v4l2_dev);

	return ret;
}

static void mtk_cam_core_remove(struct platform_device *pdev)
{
	struct mtk_cam_core *cam = platform_get_drvdata(pdev);

	media_device_unregister(&cam->mdev);
	v4l2_device_unregister(&cam->v4l2_dev);
	media_device_cleanup(&cam->mdev);
}

static const struct of_device_id mtk_cam_core_of_ids[] = {
	{ .compatible = "mediatek,mt6785-cam" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_cam_core_of_ids);

static struct platform_driver mtk_cam_core_driver = {
	.probe = mtk_cam_core_probe,
	.remove = mtk_cam_core_remove,
	.driver = {
		.name = "mtk-cam-core",
		.of_match_table = mtk_cam_core_of_ids,
	},
};
module_platform_driver(mtk_cam_core_driver);

MODULE_DESCRIPTION("MediaTek camera pipeline core");
MODULE_LICENSE("GPL");
