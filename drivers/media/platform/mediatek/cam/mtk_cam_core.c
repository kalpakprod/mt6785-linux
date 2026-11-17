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
 * A pipeline entity bound. Two rules make a missing neighbour harmless:
 *
 *  - the link for this entity is created from its own firmware endpoint as soon as it appears
 *    (`v4l2_create_fwnode_links_to_pad()` does the walk and the link), so one sensor failing
 *    never leaves the other without its link;
 *  - the subdevice nodes are registered once the receiver and the engine are there - the two
 *    entities the pipeline cannot work without - rather than in `.complete`, which upstream
 *    reaches only when every match has bound and which an optional sensor would hold open for
 *    ever.
 */
static int mtk_cam_core_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *sd,
			      struct v4l2_async_connection *asc)
{
	struct mtk_cam_core *cam =
		container_of(notifier, struct mtk_cam_core, notifier);
	struct v4l2_subdev *seninf = NULL, *camsv = NULL;
	struct fwnode_handle *ep;
	unsigned int i;

	if (cam->n_entities < ARRAY_SIZE(cam->entities)) {
		cam->entities[cam->n_entities].sd = sd;
		cam->entities[cam->n_entities].np = sd->dev ? sd->dev->of_node : NULL;
		cam->n_entities++;
	}

	dev_info(cam->dev, "%s bound (%u entities)\n", sd->name, cam->n_entities);

	/* The subdevice's endpoint names the entity it feeds; make that link now. */
	fwnode_graph_for_each_endpoint(dev_fwnode(sd->dev), ep) {
		fwnode_handle_get(ep);
		if (v4l2_create_fwnode_links_to_pad(sd, &sd->entity.pads[0], 0))
			dev_dbg(cam->dev, "%s: no link from endpoint %pfwf\n", sd->name, ep);
		fwnode_handle_put(ep);
	}

	for (i = 0; i < cam->n_entities; i++) {
		if (!strncmp(cam->entities[i].sd->name, "1a040000", 8))
			seninf = cam->entities[i].sd;
		if (!strncmp(cam->entities[i].sd->name, "1a050000", 8))
			camsv = cam->entities[i].sd;
	}

	if (seninf && camsv && !cam->nodes_registered) {
		/* The engine's sink pad is fed by the receiver's source pad. */
		if (!media_create_pad_link(&seninf->entity, seninf->entity.num_pads - 1,
					   &camsv->entity, 0, MEDIA_LNK_FL_ENABLED))
			dev_info(cam->dev, "link %s -> %s\n", seninf->name, camsv->name);

		if (!v4l2_device_register_subdev_nodes(&cam->v4l2_dev))
			cam->nodes_registered = true;
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
