// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek SCP sensorhub: parsing of the sample ring.
 *
 * The sizes below are the vendor's data_unit_t payloads
 * (scratch/wt/sensors/tmp/vendor/sensors/SCP_sensorHub.h:83-105, :224-253): the
 * frame header is four bytes of type/flush/reserve plus a packed 64-bit
 * timestamp, and a vector payload is six 32-bit words with biases plus the
 * temperature-or-status word (mtk_sensor_vec_t below).
 */
#include <linux/unaligned.h>

#include "mtk-sensorhub-ring.h"

/* struct sensor_vec_t: x, y, z, x_bias, y_bias, z_bias, packed t/status */
struct mtk_sh_vec {
	s32 x, y, z;
	s32 x_bias, y_bias, z_bias;
	u32 status;
} __packed;

#define MTK_SH_HEADER_SIZE	4	/* type, flush, reserve[2] */
#define MTK_SH_FRAME_HDR_SIZE	(MTK_SH_HEADER_SIZE + sizeof(u64))

size_t mtk_sh_frame_size(u8 type)
{
	size_t payload;

	switch (type) {
	case MTK_SH_TYPE_ACCEL:
	case MTK_SH_TYPE_GYRO:
	case MTK_SH_TYPE_MAG:
	case MTK_SH_TYPE_ORIENTATION:
		payload = sizeof(struct mtk_sh_vec);
		break;
	case MTK_SH_TYPE_LIGHT:
	case MTK_SH_TYPE_PROXIMITY:
		/* int32_t light, struct proximity_vec_t */
		payload = sizeof(s32);
		break;
	default:
		return 0;
	}

	return MTK_SH_FRAME_HDR_SIZE + payload;
}

int mtk_sh_ring_valid(u32 rp, u32 wp, u32 size)
{
	if (!size)
		return -EINVAL;
	if (rp >= size || wp >= size)
		return -EINVAL;

	return 0;
}

int mtk_sh_next_frame(const u8 *buf, size_t buf_size, u32 pos,
		      struct mtk_sh_frame *frame, u32 *next)
{
	const u8 *p = buf + pos;
	size_t frame_size;
	u8 type;

	if (pos >= buf_size)
		return -EINVAL;

	if (buf_size - pos < MTK_SH_FRAME_HDR_SIZE)
		return -EBADMSG;

	type = p[0];
	frame_size = mtk_sh_frame_size(type);
	if (!frame_size || frame_size > buf_size - pos)
		return -EBADMSG;

	frame->type = type;
	frame->flush = p[1];
	frame->timestamp = get_unaligned_le64(p + MTK_SH_HEADER_SIZE);
	frame->payload = p + MTK_SH_FRAME_HDR_SIZE;
	frame->payload_len = frame_size - MTK_SH_FRAME_HDR_SIZE;
	*next = pos + frame_size;

	return 0;
}
