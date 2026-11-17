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
#include <media/v4l2-async.h>
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
EXPORT_SYMBOL_GPL(mtk_cam_core_get);

void mtk_cam_core_put(struct mtk_cam_core *cam)
{
	(void)cam;
}
EXPORT_SYMBOL_GPL(mtk_cam_core_put);

static struct v4l2_subdev *cam_subdev_by_node(struct mtk_cam_core *cam,
					      struct device_node *np)
{
	unsigned int i;

	if (!np)
		return NULL;

	for (i = 0; i < cam->n_entities; i++)
		if (cam->entities[i].np == np)
			return cam->entities[i].sd;

	return NULL;
}

static struct device_node *cam_remote_of(struct device_node *np, unsigned int port)
{
	struct device_node *ports = of_get_child_by_name(np, "ports");
	struct device_node *ep, *remote = NULL;

	if (!ports) {
		/* A single-port node spells it "port". */
		ports = of_get_child_by_name(np, "port");
		if (ports) {
			ep = of_get_child_by_name(ports, "endpoint");
			if (ep) {
				remote = of_graph_get_remote_endpoint(ep);
				of_node_put(ep);
			}
			of_node_put(ports);
		}
		return remote;
	}

	for_each_endpoint_of_node(ports, ep) {
		u32 reg = 0;

		of_property_read_u32(of_graph_get_port_parent(ep), "reg", &reg);
		if (reg != port)
			continue;

		remote = of_graph_get_remote_endpoint(ep);
		break;
	}

	of_node_put(ports);

	return remote;
}

/*
 * One entity bound: remember it, and when every entity of the pipeline is present, wire the
 * chain the device tree describes - sensor -> receiver -> engine - with pad links on the media
 * device. The graph is the board's; this file only follows it.
 */
static int mtk_cam_core_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *sd,
			      struct v4l2_async_connection *asc)
{
	struct mtk_cam_core *cam =
		container_of(notifier, struct mtk_cam_core, notifier);
	struct device_node *in_np, *remote;
	struct v4l2_subdev *from;
	unsigned int i;

	if (cam->n_entities >= ARRAY_SIZE(cam->entities))
		return -ENOSPC;

	for (i = 0; i < cam->n_entities; i++)
		if (cam->entities[i].sd == sd)
			return 0;

	cam->entities[cam->n_entities].sd = sd;
	cam->entities[cam->n_entities].np = sd->dev ? sd->dev->of_node : NULL;
	cam->n_entities++;

	dev_info(cam->dev, "%s bound (%u entities)\n", sd->name, cam->n_entities);

	/*
	 * For every entity that has an input port, look at the endpoint that port names and link
	 * the two entities through the media device.
	 */
	for (i = 0; i < cam->n_entities; i++) {
		in_np = cam->entities[i].np;
		if (!in_np)
			continue;

		/* A sensor has one output; receivers and engines have their input on port 0. */
		remote = cam_remote_of(in_np, 0);
		if (!remote)
			continue;

		from = cam_subdev_by_node(cam, remote);
		if (from && media_create_pad_link(&from->entity, from->entity.num_pads - 1,
						  &cam->entities[i].sd->entity, 0, 0) == 0)
			dev_info(cam->dev, "link %s -> %s\n", from->name,
				 cam->entities[i].sd->name);

		of_node_put(remote);
	}

	return 0;
}

static const struct v4l2_async_notifier_operations mtk_cam_core_nf_ops = {
	.bound = mtk_cam_core_bound,
};

static const struct of_device_id mtk_cam_core_async_matches[] = {
	{ .compatible = "sony,imx355" },
	{ .compatible = "ovti,ov13b10" },
	{ .compatible = "mediatek,mt6785-seninf" },
	{ .compatible = "mediatek,mt6785-camsv" },
	{ /* sentinel */ }
};

/* The links are made by the notifier; nothing to walk here. */
int mtk_cam_core_link(struct mtk_cam_core *cam)
{
	return cam ? 0 : -ENODEV;
}
EXPORT_SYMBOL_GPL(mtk_cam_core_link);

static int mtk_cam_core_probe(struct platform_device *pdev)
{
	struct mtk_cam_core *cam;
	unsigned int i, n_matches = 0;
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

	v4l2_async_nf_init(&cam->notifier, &cam->v4l2_dev);
	cam->notifier.ops = &mtk_cam_core_nf_ops;

	/* Watch every camera entity of the SoC: the sensors, the receiver and the engine. */
	for (i = 0; i < ARRAY_SIZE(mtk_cam_core_async_matches) - 1; i++) {
		struct device_node *np, *next;

		/* Keep the reference until the next search starts: it uses this node as its base. */
		for (np = of_find_compatible_node(NULL, NULL,
						  mtk_cam_core_async_matches[i].compatible);
		     np; np = next) {
			struct v4l2_async_connection *asc;

			next = of_find_compatible_node(np, NULL,
						       mtk_cam_core_async_matches[i].compatible);
			asc = v4l2_async_nf_add_fwnode(&cam->notifier,
						       of_fwnode_handle(np),
						       struct v4l2_async_connection);
			of_node_put(np);
			if (IS_ERR(asc)) {
				ret = PTR_ERR(asc);
				goto err_nf;
			}
			n_matches++;
		}
	}

	dev_info(cam->dev, "notifier watching %u camera entities\n", n_matches);

	ret = v4l2_async_nf_register(&cam->notifier);
	if (ret)
		goto err_nf;

	dev_info(&pdev->dev, "camera pipeline core registered\n");

	return 0;

err_nf:
	v4l2_async_nf_cleanup(&cam->notifier);
	media_device_unregister(&cam->mdev);
err_v4l2:
	v4l2_device_unregister(&cam->v4l2_dev);

	return ret;
}

static void mtk_cam_core_remove(struct platform_device *pdev)
{
	struct mtk_cam_core *cam = platform_get_drvdata(pdev);

	v4l2_async_nf_unregister(&cam->notifier);
	v4l2_async_nf_cleanup(&cam->notifier);
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
