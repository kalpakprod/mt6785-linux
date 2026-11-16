// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek SCP sensorhub: encoders for the messages to the hub.
 *
 * The vendor userland and HAL build these structures in host order and pass
 * them to the SCP build of the same header, which is little-endian ARM, so the
 * encoders write little-endian explicitly: the same buffers then also work from
 * a big-endian host and can be checked byte by byte in a test.
 */
#include <linux/string.h>
#include <linux/unaligned.h>

#include "mtk-sensorhub-msg.h"

#define MTK_SH_HDR_LEN	4	/* sensorType, action, errCode/event, reserve */

static int mtk_sh_check(u8 *buf, size_t size, size_t need)
{
	if (!buf || size < need)
		return -ENOSPC;

	memset(buf, 0, need);
	return 0;
}

int mtk_sh_encode_config_cmd(u8 *buf, size_t size, u8 sensor_type, u8 cmd,
			     u32 rate, u64 latency, u16 flags)
{
	int ret = mtk_sh_check(buf, size, sizeof(struct mtk_sh_config_cmd));

	if (ret)
		return ret;

	put_unaligned_le32(MTK_SH_EVT_NO_SENSOR_CONFIG, buf);
	put_unaligned_le64(latency, buf + 4);
	put_unaligned_le32(rate, buf + 12);
	buf[16] = sensor_type;
	buf[17] = cmd;
	put_unaligned_le16(flags, buf + 18);

	return sizeof(struct mtk_sh_config_cmd);
}

int mtk_sh_encode_activate(u8 *buf, size_t size, u8 sensor_type, bool enable)
{
	int ret = mtk_sh_check(buf, size, MTK_SH_ACTIVATE_LEN);

	if (ret)
		return ret;

	buf[0] = sensor_type;
	buf[1] = MTK_SH_ACTION_ACTIVATE;
	put_unaligned_le32(enable ? 1 : 0, buf + 4);

	return MTK_SH_ACTIVATE_LEN;
}

int mtk_sh_encode_set_delay(u8 *buf, size_t size, u8 sensor_type, u32 delay_ms)
{
	int ret = mtk_sh_check(buf, size, MTK_SH_SET_DELAY_LEN);

	if (ret)
		return ret;

	buf[0] = sensor_type;
	buf[1] = MTK_SH_ACTION_SET_DELAY;
	put_unaligned_le32(delay_ms, buf + 4);

	return MTK_SH_SET_DELAY_LEN;
}

int mtk_sh_encode_batch(u8 *buf, size_t size, u8 sensor_type, u8 flag,
			u32 period_ms, u32 timeout_ms)
{
	int ret = mtk_sh_check(buf, size, MTK_SH_BATCH_LEN);

	if (ret)
		return ret;

	buf[0] = sensor_type;
	buf[1] = MTK_SH_ACTION_BATCH;
	buf[2] = flag;
	put_unaligned_le32(period_ms, buf + 4);
	put_unaligned_le32(timeout_ms, buf + 8);

	return MTK_SH_BATCH_LEN;
}

int mtk_sh_encode_set_config(u8 *buf, size_t size, u8 sensor_type, u32 base,
			     u32 ring_size, u64 ap_ts, u64 arch_counter)
{
	int ret = mtk_sh_check(buf, size, MTK_SH_SET_CONFIG_LEN);

	if (ret)
		return ret;

	buf[0] = sensor_type;
	buf[1] = MTK_SH_ACTION_SET_CONFIG;
	put_unaligned_le32(base, buf + 4);
	put_unaligned_le32(ring_size, buf + 8);
	put_unaligned_le64(ap_ts, buf + 12);
	put_unaligned_le64(arch_counter, buf + 20);

	return MTK_SH_SET_CONFIG_LEN;
}

int mtk_sh_encode_set_cust(u8 *buf, size_t size, u8 sensor_type,
			   enum mtk_sh_cust_action cust,
			   const u32 *data, size_t words)
{
	size_t len = MTK_SH_SET_CUST_LEN(words);
	size_t i;
	int ret;

	ret = mtk_sh_check(buf, size, len);
	if (ret)
		return ret;

	buf[0] = sensor_type;
	buf[1] = MTK_SH_ACTION_SET_CUST;
	put_unaligned_le32(cust, buf + MTK_SH_HDR_LEN);
	for (i = 0; i < words; i++)
		put_unaligned_le32(data[i], buf + MTK_SH_HDR_LEN + 4 + 4 * i);

	return len;
}
