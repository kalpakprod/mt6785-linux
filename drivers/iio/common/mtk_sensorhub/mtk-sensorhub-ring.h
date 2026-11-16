/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek SCP sensorhub: the ring buffer the SCP writes sample frames into.
 *
 * The protocol lives in the vendor's SCP_sensorHub.h (the sensors area keeps an
 * extract at scratch/wt/sensors/tmp/vendor/sensors/):
 *
 *	struct sensorFIFO {		S_sensorshub.h:264-270
 *		u32 rp;			read pointer, the AP owns it
 *		u32 wp;			write pointer, the SCP owns it
 *		u32 FIFOSize;
 *		u32 reserve;
 *		struct data_unit_t data[];
 *	};
 *
 *	struct data_unit_t {		S_sensorshub.h:224-253, packed
 *		u8 sensor_type;
 *		u8 flush_action;
 *		u8 reserve[2];
 *		u64 time_stamp;
 *		union { ... };
 *	};
 *
 * The AP announces the buffer with SENSOR_HUB_SET_CONFIG and the SCP answers
 * each batch of samples with SENSOR_HUB_NOTIFY carrying its write pointer. The
 * parsing below is deliberately free of I/O so it can be unit tested on a plain
 * byte array (see mtk-sensorhub-ring-test.c).
 */
#ifndef __MTK_SENSORHUB_RING_H
#define __MTK_SENSORHUB_RING_H

#include <linux/build_bug.h>
#include <linux/errno.h>
#include <linux/types.h>

/* the ring's own header, SCP_sensorHub.h:264-270 */
struct mtk_sh_ring {
	u32 rp;
	u32 wp;
	u32 size;
	u32 reserve;
	u8 data[];
} __packed;

/* one frame, as far as the parser needs it */
struct mtk_sh_frame {
	u8 type;
	u8 flush;
	u64 timestamp;
	const u8 *payload;
	size_t payload_len;
};

/*
 * The sensor type byte of a frame is MediaTek's SENSOR_TYPE_*, which follows
 * Android's numbering. Taken from the vendor's own header for this phone:
 * MiCode/Xiaomi_Kernel_OpenSource, branch rosemary-r-oss, commit
 * 7094834e4fca19af2ca43a752debf95c0022b949,
 * drivers/misc/mediatek/sensors-1.0/hwmon/include/hwmsensor.h:30-37 -
 * ACCELEROMETER 1, MAGNETIC_FIELD 2, ORIENTATION 3, GYROSCOPE 4, LIGHT 5,
 * PROXIMITY 8. The sensor hub's handle ids are the same numbers minus one
 * (hwmsensor.h:96-103: ID_ACCELEROMETER = ID_BASE + SENSOR_TYPE_ACCELEROMETER - 1),
 * which is why the type byte and the handle must not be confused. The SCP
 * firmware is built from the same vendor tree (its strings name
 * middleware/contexthub/...), so it speaks these numbers.
 */
enum mtk_sh_type {
	MTK_SH_TYPE_ACCEL = 1,		/* SENSOR_TYPE_ACCELEROMETER */
	MTK_SH_TYPE_MAG = 2,		/* SENSOR_TYPE_MAGNETIC_FIELD */
	MTK_SH_TYPE_ORIENTATION = 3,	/* SENSOR_TYPE_ORIENTATION */
	MTK_SH_TYPE_GYRO = 4,		/* SENSOR_TYPE_GYROSCOPE */
	MTK_SH_TYPE_LIGHT = 5,		/* SENSOR_TYPE_LIGHT */
	MTK_SH_TYPE_PROXIMITY = 8,	/* SENSOR_TYPE_PROXIMITY */
	MTK_SH_TYPE_MAX = 9,
};

/**
 * mtk_sh_frame_size() - size of a whole frame of @type, header included
 * @type: sensor type byte from the frame
 *
 * Return: the frame's size in bytes, or 0 for a type this client doesn't know.
 */
size_t mtk_sh_frame_size(u8 type);

/**
 * mtk_sh_ring_valid() - are the ring's pointers inside its own buffer?
 * @rp: read pointer
 * @wp: write pointer
 * @size: ring capacity reported by the SCP
 *
 * Return: 0 when both pointers are smaller than @size, -EINVAL otherwise. A ring
 * whose size is zero or larger than the mapping is caught by the caller.
 */
int mtk_sh_ring_valid(u32 rp, u32 wp, u32 size);

/**
 * mtk_sh_next_frame() - read the frame at @pos
 * @buf: the ring's bytes, starting at the ring header
 * @buf_size: how many bytes of the ring are valid
 * @pos: offset of the frame, counted from the start of @buf
 * @frame: filled with the frame's type, timestamp and payload
 * @next: set to the offset of the following frame
 *
 * A frame must lie entirely inside the buffer and hold a whole header plus a
 * payload of its type's size; anything else is -EBADMSG so the caller can drop
 * the ring's content instead of parsing garbage.
 *
 * Return: 0 on success, -EINVAL for a position outside the buffer, -EBADMSG for
 * a frame that is truncated or of an unknown type.
 */
int mtk_sh_next_frame(const u8 *buf, size_t buf_size, u32 pos,
		      struct mtk_sh_frame *frame, u32 *next);

#endif /* __MTK_SENSORHUB_RING_H */
