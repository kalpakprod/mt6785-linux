// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek SCP sensorhub: scales and the IIO devices.
 *
 * The scale factors are the vendor's, as the cezanne port documents them
 * (cezanne-linux-port/linux/drivers/scp/sensor/scp_iio.c:43-67, from
 * mtk_nanohub_init_sensor_info): the hub reports accelerometer in m/s^2 * 1000,
 * gyroscope in rad/s * 1e6, magnetic field in uT * 100, light in lux, proximity
 * as 0 = near / 1 = far, orientation in degrees * 1000. IIO wants m/s^2, rad/s,
 * gauss, lux, "bigger is nearer" and thousandths of a degree, so each of those
 * becomes an IIO_VAL_INT_PLUS_NANO scale - and that conversion is a pure
 * function, so a unit test can hold it to the numbers above.
 */
#include <linux/errno.h>
#include <linux/types.h>

#include "mtk-sensorhub-ring.h"
#include "mtk-sensorhub-scale.h"

int mtk_sh_scale(u8 type, s32 *val, s32 *val2)
{
	switch (type) {
	case MTK_SH_TYPE_ACCEL:
		/* raw = m/s^2 * 1000 */
		*val = 0;
		*val2 = 1000000;	/* 0.001 */
		return IIO_VAL_INT_PLUS_NANO;
	case MTK_SH_TYPE_GYRO:
		/* raw = rad/s * 1e6 */
		*val = 0;
		*val2 = 1000;		/* 0.000001 */
		return IIO_VAL_INT_PLUS_NANO;
	case MTK_SH_TYPE_MAG:
		/* raw = uT * 100, and 1 uT = 0.01 G */
		*val = 0;
		*val2 = 100000;		/* 0.0001 */
		return IIO_VAL_INT_PLUS_NANO;
	case MTK_SH_TYPE_ORIENTATION:
		/* raw = degrees * 1000 */
		*val = 0;
		*val2 = 1000000;	/* 0.001 */
		return IIO_VAL_INT_PLUS_NANO;
	case MTK_SH_TYPE_LIGHT:
		/* lux, already in IIO units */
		*val = 1;
		*val2 = 0;
		return IIO_VAL_INT;
	case MTK_SH_TYPE_PROXIMITY:
		/* 0 = near, 1 = far: the driver inverts it on the way out */
		*val = 1;
		*val2 = 0;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}
