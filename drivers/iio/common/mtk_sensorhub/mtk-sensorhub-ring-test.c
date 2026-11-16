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

static struct kunit_case mtk_sh_test_cases[] = {
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
