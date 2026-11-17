/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MTK_CAM_CORE_H__
#define __MTK_CAM_CORE_H__

#include <linux/device.h>
#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>

/**
 * struct mtk_cam_core - the shared camera pipeline objects
 * @dev: the platform device described as "mediatek,mt6785-cam"
 * @v4l2_dev: the v4l2 device the sensor, receiver and engine subdevices register into
 * @mdev: the media device their entities appear in
 */
struct mtk_cam_core {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	/* Every pipeline entity is collected here, and the links are made once all have bound. */
	struct v4l2_async_notifier notifier;
	struct { struct v4l2_subdev *sd; struct device_node *np; } entities[8];
	unsigned int n_entities;
};

#if IS_ENABLED(CONFIG_VIDEO_MTK_CAM)
struct mtk_cam_core *mtk_cam_core_get(struct device *dev);
void mtk_cam_core_put(struct mtk_cam_core *cam);
int mtk_cam_core_link(struct mtk_cam_core *cam);
#else
static inline struct mtk_cam_core *mtk_cam_core_get(struct device *dev)
{
	return NULL;
}
static inline void mtk_cam_core_put(struct mtk_cam_core *cam) { }
static inline int mtk_cam_core_link(struct mtk_cam_core *cam)
{
	return -ENODEV;
}
#endif

#endif /* __MTK_CAM_CORE_H__ */
