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
#include <media/v4l2-mc.h>
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

/*
 * A pipeline entity bound. Links are made per entity, from the source side, so that an absent
 * neighbour costs only its own link:
 *
 *  - for each of this entity's *sink* pads, the firmware endpoint that pad is wired to names the
 *    entity that feeds it; that entity is the source and this pad the destination, which is
 *    exactly `v4l2_create_fwnode_links_to_pad(source_sd, sink_pad, flags)`;
 *  - the subdevice nodes are registered once the receiver and the engine are present, rather than
 *    in `.complete()`, which upstream reaches only when every match has bound.
 */
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

/*
 * The node this entity's pad @pad is wired to, through the endpoint the device tree declares.
 * Pad numbering follows the ports: pad N is port N of the entity's node, whether the node spells
 * it as one "port" or as "ports/port@N" (of_graph_get_port_by_id() handles both).
 */
static struct device_node *cam_pad_remote_of(struct device_node *np, unsigned int pad)
{
	struct device_node *port, *ep, *remote;

	port = of_graph_get_port_by_id(np, pad);
	if (!port)
		return NULL;

	ep = of_get_child_by_name(port, "endpoint");
	of_node_put(port);
	if (!ep)
		return NULL;

	remote = of_graph_get_remote_endpoint(ep);
	of_node_put(ep);

	return remote;
}

/*
 * Make every link whose two ends are registered by now, skipping the ones already there. Called
 * from each .bound(), so the last arrival completes the graph whatever the bind order is.
 */
static void mtk_cam_core_make_links(struct mtk_cam_core *cam)
{
	unsigned int i;

	for (i = 0; i < cam->n_entities; i++) {
		struct v4l2_subdev *sd = cam->entities[i].sd;
		unsigned int p;

		for (p = 0; p < sd->entity.num_pads; p++) {
			struct media_pad *pad = &sd->entity.pads[p];
			struct device_node *remote_np;
			struct v4l2_subdev *src;
			int ret;

			if (!(pad->flags & MEDIA_PAD_FL_SINK))
				continue;

			remote_np = cam_pad_remote_of(sd->dev->of_node, p);
			if (!remote_np) {
				dev_info(cam->dev, "link try %s:%u: no endpoint in the tree\n",
					 sd->name, p);
				continue;
			}

			src = cam_subdev_by_node(cam, remote_np);
			if (!src) {
				dev_info(cam->dev, "link try %s:%u: remote %pOF not bound yet\n",
					 sd->name, p, remote_np);
				of_node_put(remote_np);
				continue;
			}
			of_node_put(remote_np);

			ret = v4l2_create_fwnode_links_to_pad(src, pad, 0);
			dev_info(cam->dev, "link try %s:0 -> %s:%u: %d\n", src->name,
				 sd->name, p, ret);
		}
	}
}

static int mtk_cam_core_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *sd,
			      struct v4l2_async_connection *asc)
{
	struct mtk_cam_core *cam =
		container_of(notifier, struct mtk_cam_core, notifier);
	struct v4l2_subdev *seninf = NULL, *camsv = NULL;
	unsigned int i;

	if (cam->n_entities < ARRAY_SIZE(cam->entities)) {
		cam->entities[cam->n_entities].sd = sd;
		cam->entities[cam->n_entities].np = sd->dev ? sd->dev->of_node : NULL;
		cam->n_entities++;
	}

	dev_info(cam->dev, "%s bound (%u entities)\n", sd->name, cam->n_entities);

	mtk_cam_core_make_links(cam);

	for (i = 0; i < cam->n_entities; i++) {
		if (!strncmp(cam->entities[i].sd->name, "1a040000", 8))
			seninf = cam->entities[i].sd;
		if (!strncmp(cam->entities[i].sd->name, "1a050000", 8))
			camsv = cam->entities[i].sd;
	}

	/*
	 * Nodes for whoever has bound so far. The helper skips subdevices that already have one
	 * (drivers/media/v4l2-core/v4l2-device.c:201-206), so calling it on every bind is what gives
	 * a late arrival - imx355 was last in disp38 - its node too.
	 */
	if (seninf && camsv) {
		int ret = v4l2_device_register_subdev_nodes(&cam->v4l2_dev);

		if (ret && ret != -EEXIST)
			dev_dbg(cam->dev, "subdev nodes: %d\n", ret);
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
