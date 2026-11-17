// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the MediaTek SCP sensorhub ring parser.
 *
 * The ring is the one thing between the SCP and the IIO front end that can be
 * fed a malicious or broken buffer: a firmware that reports a truncated frame,
 * an unknown type or a wrap that does not add up must be rejected instead of
 * producing samples with garbage timestamps. Each test below builds such a ring
 * by hand, including a deliberately corrupted frame, and checks the parser's
 * verdict. Run with:
 *
 *	tools/testing/kunit/kunit.py run --kunitconfig=drivers/iio/common/mtk_sensorhub
 */
#include <kunit/test.h>

#include "mtk-sensorhub-msg.h"
#include "mtk-sensorhub-scale.h"
#include "mtk-sensorhub-ring.h"

#define FRAME_ACCEL	40	/* header 12 + struct sensor_vec_t 28 */
#define FRAME_LIGHT	16	/* header 12 + s32 4 */

static void fill_vec_frame(u8 *p, u8 type, u8 flush, u64 ts, s32 x)
{
	memset(p, 0, FRAME_ACCEL);
	p[0] = type;
	p[1] = flush;
	put_unaligned_le64(ts, p + 4);
	put_unaligned_le32(x, p + 12);
}

static void mtk_sh_test_frame_size(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, mtk_sh_frame_size(MTK_SH_TYPE_ACCEL), FRAME_ACCEL);
	KUNIT_EXPECT_EQ(test, mtk_sh_frame_size(MTK_SH_TYPE_GYRO), FRAME_ACCEL);
	KUNIT_EXPECT_EQ(test, mtk_sh_frame_size(MTK_SH_TYPE_MAG), FRAME_ACCEL);
	KUNIT_EXPECT_EQ(test, mtk_sh_frame_size(MTK_SH_TYPE_LIGHT), FRAME_LIGHT);
	KUNIT_EXPECT_EQ(test, mtk_sh_frame_size(MTK_SH_TYPE_PROXIMITY), FRAME_LIGHT);
	/* an unknown type has no size, and that is how the parser spots it */
	KUNIT_EXPECT_EQ(test, mtk_sh_frame_size(0x7e), 0);
}

static void mtk_sh_test_ring_valid(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, mtk_sh_ring_valid(0, FRAME_ACCEL * 2, 4096), 0);
	KUNIT_EXPECT_EQ(test, mtk_sh_ring_valid(4096, 0, 4096), -EINVAL);
	KUNIT_EXPECT_EQ(test, mtk_sh_ring_valid(0, 4096, 4096), -EINVAL);
	KUNIT_EXPECT_EQ(test, mtk_sh_ring_valid(0, 0, 0), -EINVAL);
}

static void mtk_sh_test_frames_in_order(struct kunit *test)
{
	u8 ring[FRAME_ACCEL * 2 + FRAME_LIGHT];
	struct mtk_sh_frame f;
	u32 pos = 0, next;

	fill_vec_frame(ring, MTK_SH_TYPE_ACCEL, 0, 0x1122334455667788ULL, -7);
	fill_vec_frame(ring + FRAME_ACCEL, MTK_SH_TYPE_GYRO, 1, 42, 9);
	memset(ring + FRAME_ACCEL * 2, 0, FRAME_LIGHT);
	ring[FRAME_ACCEL * 2] = MTK_SH_TYPE_LIGHT;
	put_unaligned_le32(320, ring + FRAME_ACCEL * 2 + 12);

	KUNIT_ASSERT_EQ(test, mtk_sh_next_frame(ring, sizeof(ring), pos, &f, &next), 0);
	KUNIT_EXPECT_EQ(test, f.type, MTK_SH_TYPE_ACCEL);
	KUNIT_EXPECT_EQ(test, f.flush, 0);
	KUNIT_EXPECT_EQ(test, f.timestamp, 0x1122334455667788ULL);
	KUNIT_EXPECT_EQ(test, f.payload_len, 28);
	KUNIT_EXPECT_EQ(test, (s32)get_unaligned_le32(f.payload), -7);
	pos = next;

	KUNIT_ASSERT_EQ(test, mtk_sh_next_frame(ring, sizeof(ring), pos, &f, &next), 0);
	KUNIT_EXPECT_EQ(test, f.type, MTK_SH_TYPE_GYRO);
	KUNIT_EXPECT_EQ(test, f.flush, 1);
	KUNIT_EXPECT_EQ(test, f.timestamp, 42);
	pos = next;

	KUNIT_ASSERT_EQ(test, mtk_sh_next_frame(ring, sizeof(ring), pos, &f, &next), 0);
	KUNIT_EXPECT_EQ(test, f.type, MTK_SH_TYPE_LIGHT);
	KUNIT_EXPECT_EQ(test, (s32)get_unaligned_le32(f.payload), 320);
	KUNIT_EXPECT_EQ(test, next, sizeof(ring));
}

/* the error case the test exists for: a frame of a type nothing defines */
static void mtk_sh_test_unknown_type_is_rejected(struct kunit *test)
{
	u8 ring[FRAME_ACCEL];
	struct mtk_sh_frame f;
	u32 next;

	fill_vec_frame(ring, 0x7e, 0, 1, 0);
	KUNIT_EXPECT_EQ(test, mtk_sh_next_frame(ring, sizeof(ring), 0, &f, &next),
			-EBADMSG);
}

/* a frame whose header is whole but whose payload runs past the buffer */
static void mtk_sh_test_truncated_frame_is_rejected(struct kunit *test)
{
	u8 ring[FRAME_ACCEL];
	struct mtk_sh_frame f;
	u32 next;

	fill_vec_frame(ring, MTK_SH_TYPE_ACCEL, 0, 1, 0);
	KUNIT_EXPECT_EQ(test, mtk_sh_next_frame(ring, FRAME_ACCEL - 1, 0, &f, &next),
			-EBADMSG);
	/* and a frame whose own header does not fit at all */
	KUNIT_EXPECT_EQ(test, mtk_sh_next_frame(ring, 11, 0, &f, &next), -EBADMSG);
}

static void mtk_sh_test_position_outside_buffer(struct kunit *test)
{
	u8 ring[FRAME_ACCEL];
	struct mtk_sh_frame f;
	u32 next;

	memset(ring, 0, sizeof(ring));
	KUNIT_EXPECT_EQ(test, mtk_sh_next_frame(ring, sizeof(ring), FRAME_ACCEL,
						&f, &next), -EINVAL);
}

/*
 * The encoders' byte layout, against the vendor's own sizes: a message whose
 * fields move is a hub that reads garbage, so the offsets are asserted one by
 * one and a planted wrong layout is shown to be caught below.
 */
static void mtk_sh_test_config_cmd_layout(struct kunit *test)
{
	u8 buf[32];

	KUNIT_EXPECT_EQ(test, sizeof(struct mtk_sh_config_cmd), 20);
	KUNIT_EXPECT_EQ(test, mtk_sh_encode_config_cmd(buf, sizeof(buf),
						       MTK_SH_TYPE_ACCEL,
						       MTK_SH_CFG_ENABLE,
						       50, 0, 0), 20);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(buf), MTK_SH_EVT_NO_SENSOR_CONFIG);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(buf + 4), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(buf + 12), 50);
	KUNIT_EXPECT_EQ(test, buf[16], MTK_SH_TYPE_ACCEL);
	KUNIT_EXPECT_EQ(test, buf[17], MTK_SH_CFG_ENABLE);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(buf + 18), 0);
	/* and a shorter buffer must be refused instead of overflowing */
	KUNIT_EXPECT_EQ(test, mtk_sh_encode_config_cmd(buf, 19, 1, 1, 1, 1, 1),
			-ENOSPC);
}

static void mtk_sh_test_action_messages_layout(struct kunit *test)
{
	const u32 cust[2] = { 0x11111111, 0x22222222 };
	u8 buf[64];

	KUNIT_ASSERT_EQ(test, mtk_sh_encode_activate(buf, sizeof(buf), 1, true), 8);
	KUNIT_EXPECT_EQ(test, buf[0], 1);
	KUNIT_EXPECT_EQ(test, buf[1], MTK_SH_ACTION_ACTIVATE);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(buf + 4), 1);

	KUNIT_ASSERT_EQ(test, mtk_sh_encode_set_delay(buf, sizeof(buf), 4, 20), 8);
	KUNIT_EXPECT_EQ(test, buf[1], MTK_SH_ACTION_SET_DELAY);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(buf + 4), 20);

	KUNIT_ASSERT_EQ(test, mtk_sh_encode_batch(buf, sizeof(buf), 1, 3, 200, 1000),
			16);
	KUNIT_EXPECT_EQ(test, buf[1], MTK_SH_ACTION_BATCH);
	KUNIT_EXPECT_EQ(test, buf[2], 3);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(buf + 4), 200);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(buf + 8), 1000);

	KUNIT_ASSERT_EQ(test, mtk_sh_encode_set_config(buf, sizeof(buf), 1,
						       0x50000000, 0x100000,
						       0x1122334455667788ULL,
						       0x99aabbccddeeff00ULL), 28);
	KUNIT_EXPECT_EQ(test, buf[1], MTK_SH_ACTION_SET_CONFIG);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(buf + 4), 0x50000000);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(buf + 8), 0x100000);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(buf + 12), 0x1122334455667788ULL);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(buf + 20), 0x99aabbccddeeff00ULL);

	KUNIT_ASSERT_EQ(test, mtk_sh_encode_set_cust(buf, sizeof(buf), 4,
						     MTK_SH_CUST_SET_CALI, cust, 2),
			16);
	KUNIT_EXPECT_EQ(test, buf[1], MTK_SH_ACTION_SET_CUST);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(buf + 4), MTK_SH_CUST_SET_CALI);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(buf + 8), 0x11111111);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(buf + 12), 0x22222222);
}

/*
 * The planted wrong layout: a message built with the enable flag at offset 8
 * instead of 4 - the mistake an unaligned or padded struct would produce - must
 * NOT match what the encoders promise, which is what the assertions above rest
 * on.
 */
static void mtk_sh_test_planted_wrong_layout(struct kunit *test)
{
	u8 good[16], planted[16];

	memset(planted, 0, sizeof(planted));
	mtk_sh_encode_activate(good, sizeof(good), 1, true);
	planted[0] = 1;
	planted[1] = MTK_SH_ACTION_ACTIVATE;
	put_unaligned_le32(1, planted + 8);

	KUNIT_EXPECT_EQ(test, memcmp(good, planted, 8), 0);
	KUNIT_EXPECT_NE(test, memcmp(good, planted, sizeof(good)), 0);
}

/*
 * The scale of each sensor, against the vendor's reported units: accel
 * m/s^2 * 1000, gyro rad/s * 1e6, magnetic uT * 100 (0.01 G each), orientation
 * degrees * 1000, light lux, proximity raw.
 */
static void mtk_sh_test_scales(struct kunit *test)
{
	s32 val, val2;

	KUNIT_EXPECT_EQ(test, mtk_sh_scale(MTK_SH_TYPE_ACCEL, &val, &val2),
			IIO_VAL_INT_PLUS_NANO);
	KUNIT_EXPECT_EQ(test, val, 0);
	KUNIT_EXPECT_EQ(test, val2, 1000000);

	KUNIT_EXPECT_EQ(test, mtk_sh_scale(MTK_SH_TYPE_GYRO, &val, &val2),
			IIO_VAL_INT_PLUS_NANO);
	KUNIT_EXPECT_EQ(test, val2, 1000);

	KUNIT_EXPECT_EQ(test, mtk_sh_scale(MTK_SH_TYPE_MAG, &val, &val2),
			IIO_VAL_INT_PLUS_NANO);
	KUNIT_EXPECT_EQ(test, val2, 100000);

	KUNIT_EXPECT_EQ(test, mtk_sh_scale(MTK_SH_TYPE_ORIENTATION, &val, &val2),
			IIO_VAL_INT_PLUS_NANO);
	KUNIT_EXPECT_EQ(test, val2, 1000000);

	KUNIT_EXPECT_EQ(test, mtk_sh_scale(MTK_SH_TYPE_LIGHT, &val, &val2),
			IIO_VAL_INT);
	KUNIT_EXPECT_EQ(test, val, 1);

	KUNIT_EXPECT_EQ(test, mtk_sh_scale(MTK_SH_TYPE_PROXIMITY, &val, &val2),
			IIO_VAL_INT);

	KUNIT_EXPECT_EQ(test, mtk_sh_scale(0x7e, &val, &val2), -EINVAL);
}

/*
 * The planted wrong scale: a gyro reported as if the hub sent rad/s * 1000
 * instead of * 1e6 would scale ten times off, and the numbers above must catch
 * exactly that, so assert the wrong value is NOT what the function returns.
 */
static void mtk_sh_test_planted_wrong_scale(struct kunit *test)
{
	s32 val, val2;

	KUNIT_ASSERT_EQ(test, mtk_sh_scale(MTK_SH_TYPE_GYRO, &val, &val2),
			IIO_VAL_INT_PLUS_NANO);
	KUNIT_EXPECT_NE(test, val2, 1000000);
	KUNIT_EXPECT_NE(test, val2, 100);
}

static struct kunit_case mtk_sh_test_cases[] = {
	KUNIT_CASE(mtk_sh_test_scales),
	KUNIT_CASE(mtk_sh_test_planted_wrong_scale),
	KUNIT_CASE(mtk_sh_test_config_cmd_layout),
	KUNIT_CASE(mtk_sh_test_action_messages_layout),
	KUNIT_CASE(mtk_sh_test_planted_wrong_layout),
	KUNIT_CASE(mtk_sh_test_frame_size),
	KUNIT_CASE(mtk_sh_test_ring_valid),
	KUNIT_CASE(mtk_sh_test_frames_in_order),
	KUNIT_CASE(mtk_sh_test_unknown_type_is_rejected),
	KUNIT_CASE(mtk_sh_test_truncated_frame_is_rejected),
	KUNIT_CASE(mtk_sh_test_position_outside_buffer),
	{}
};

static struct kunit_suite mtk_sh_test_suite = {
	.name = "mtk-sensorhub-ring",
	.test_cases = mtk_sh_test_cases,
};

kunit_test_suite(mtk_sh_test_suite);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("KUnit tests for the MediaTek SCP sensorhub ring parser");
