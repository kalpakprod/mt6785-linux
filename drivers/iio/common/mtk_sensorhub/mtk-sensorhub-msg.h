/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek SCP sensorhub: the messages the AP sends to the hub.
 *
 * Every structure below is the vendor's: MiCode/Xiaomi_Kernel_OpenSource,
 * branch rosemary-r-oss, commit 7094834e4fca19af2ca43a752debf95c0022b949,
 * drivers/misc/mediatek/sensors-1.0/sensorHub/inc_v1/SCP_sensorHub.h (the copy
 * the sensors area keeps at scratch/wt/sensors/tmp/vendor/sensors/ is the same
 * file). The encoders are the only place in this driver that knows their byte
 * layout, so that layout can be checked against the vendor's sizes in a unit
 * test instead of being trusted in the middle of the driver.
 *
 * The transport is IPI id 8 (IPI_SENSOR, the vendor's enum ipi_id in
 * scratch/wt/sensors/tmp/vendor/scp_cm4.h:17-38). Each message starts with the
 * same four bytes - { u8 sensorType; u8 action; u8 errCode or event; u8
 * reserve; } - and the action decides the body (SCP_sensorHub.h:283-370 and
 * :495-533).
 */
#ifndef __MTK_SENSORHUB_MSG_H
#define __MTK_SENSORHUB_MSG_H

#include <linux/errno.h>
#include <linux/types.h>

/* SCP_sensorHub.h:75-81 */
enum mtk_sh_action {
	MTK_SH_ACTION_ACTIVATE = 0,
	MTK_SH_ACTION_SET_DELAY = 1,
	MTK_SH_ACTION_GET_DATA = 2,
	MTK_SH_ACTION_BATCH = 3,
	MTK_SH_ACTION_SET_CONFIG = 4,
	MTK_SH_ACTION_SET_CUST = 5,
	MTK_SH_ACTION_NOTIFY = 6,
	MTK_SH_ACTION_BATCH_TIMEOUT = 7,
	MTK_SH_ACTION_SET_TIMESTAMP = 8,
	MTK_SH_ACTION_POWER_NOTIFY = 9,
};

/* SCP_sensorHub.h:31-36: the newer client's single-message form, where cmd is
 * one of these; named differently from struct mtk_sh_config_cmd because C has a
 * single tag namespace for both.
 */
enum mtk_sh_cfg_cmd {
	MTK_SH_CFG_DISABLE = 0,
	MTK_SH_CFG_ENABLE = 1,
	MTK_SH_CFG_FLUSH = 2,
	MTK_SH_CFG_DATA = 3,
	MTK_SH_CFG_CALIBRATE = 4,
	MTK_SH_CFG_SELF_TEST = 5,
};

/* SCP_sensorHub.h:27-28 */
#define MTK_SH_RATE_ONCHANGE	0xffffff01UL
#define MTK_SH_RATE_ONESHOT	0xffffff02UL

/* SCP_sensorHub.h:26 */
#define MTK_SH_EVT_NO_SENSOR_CONFIG	0x00000300

/* SCP_sensorHub.h:367-379 */
enum mtk_sh_cust_action {
	MTK_SH_CUST_SET_CUST = 1,
	MTK_SH_CUST_SET_CALI,
	MTK_SH_CUST_RESET_CALI,
	MTK_SH_CUST_SET_TRACE,
	MTK_SH_CUST_SET_DIRECTION,
	MTK_SH_CUST_SHOW_REG,
	MTK_SH_CUST_GET_RAW_DATA,
	MTK_SH_CUST_SET_PS_THRESHOLD,
	MTK_SH_CUST_SHOW_ALSLV,
	MTK_SH_CUST_SHOW_ALSVAL,
	MTK_SH_CUST_SET_FACTORY,
	MTK_SH_CUST_GET_SENSOR_INFO,
};

/* SCP_sensorHub.h:39-47, packed: 4 + 8 + 4 + 1 + 1 + 2 = 20 bytes */
struct mtk_sh_config_cmd {
	u32 evt_type;
	u64 latency;
	u32 rate;
	u8 sensor_type;
	u8 cmd;
	u16 flags;
	u8 data[];
} __packed;

/* SCP_sensorHub.h:296-304: { hdr, u32 enable } */
#define MTK_SH_ACTIVATE_LEN	8
/* SCP_sensorHub.h:306-315: { hdr, u32 delay } */
#define MTK_SH_SET_DELAY_LEN	8
/* SCP_sensorHub.h:338-346: { hdr, u8 flag, u8 reserve, u32 period, u32 timeout } */
#define MTK_SH_BATCH_LEN	16
/* SCP_sensorHub.h:351-361: { hdr, u32 base, u32 size, u64 ap_ts, u64 arch } */
#define MTK_SH_SET_CONFIG_LEN	28
/*
 * SCP_sensorHub.h:381+: { hdr, u32 data[] }; the vendor's callers put the
 * CUST_ACTION_* value in data[0], so mtk_sh_encode_set_cust() writes @cust there
 * and @data after it.
 */
#define MTK_SH_SET_CUST_LEN(n)	(8 + 4 * (n))

int mtk_sh_encode_config_cmd(u8 *buf, size_t size, u8 sensor_type, u8 cmd,
			     u32 rate, u64 latency, u16 flags);
int mtk_sh_encode_activate(u8 *buf, size_t size, u8 sensor_type, bool enable);
int mtk_sh_encode_set_delay(u8 *buf, size_t size, u8 sensor_type, u32 delay_ms);
int mtk_sh_encode_batch(u8 *buf, size_t size, u8 sensor_type, u8 flag,
			u32 period_ms, u32 timeout_ms);
int mtk_sh_encode_set_config(u8 *buf, size_t size, u8 sensor_type, u32 base,
			     u32 ring_size, u64 ap_ts, u64 arch_counter);
int mtk_sh_encode_set_cust(u8 *buf, size_t size, u8 sensor_type,
			   enum mtk_sh_cust_action cust,
			   const u32 *data, size_t words);

#endif /* __MTK_SENSORHUB_MSG_H */
