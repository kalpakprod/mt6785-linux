/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek SCP sensorhub: the conversion from the hub's raw values to IIO units.
 */
#ifndef __MTK_SENSORHUB_SCALE_H
#define __MTK_SENSORHUB_SCALE_H

#include <linux/types.h>

#define IIO_VAL_INT		1
#define IIO_VAL_INT_PLUS_NANO	3

/**
 * mtk_sh_scale() - scale of a sensor type, in the form IIO reports
 * @type: frame's sensor type byte
 * @val: integral part
 * @val2: nano part for IIO_VAL_INT_PLUS_NANO
 *
 * Return: the IIO value format, or -EINVAL for a type without a scale. The
 * numbers are the vendor's, cited in mtk-sensorhub-scale.c.
 */
int mtk_sh_scale(u8 type, s32 *val, s32 *val2);

#endif /* __MTK_SENSORHUB_SCALE_H */
