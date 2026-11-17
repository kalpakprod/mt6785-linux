/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek SCP sensorhub: private state shared by the client and the IIO layer.
 */
#ifndef __MTK_SENSORHUB_H
#define __MTK_SENSORHUB_H

#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/remoteproc/mtk_scp.h>
#include <linux/types.h>

#include "mtk-sensorhub-msg.h"
#include "mtk-sensorhub-ring.h"

/* one sensor of the board, as the client enables it and IIO exposes it */
struct mtk_sh_sensor {
	const char *name;
	u8 type;
	u32 rate_hz;
	struct iio_dev *indio;
};

struct mtk_sh {
	struct device *dev;
	struct mtk_scp *scp;
	void __iomem *ring;
	u8 *buf;			/* a copy of the ring, for the parser */
	phys_addr_t ring_phys;
	size_t ring_size;
	struct mutex lock;		/* serialises messages and ring updates */
	struct mtk_sh_sensor *sensors;
	unsigned int num_sensors;
	u32 last_wp;
	unsigned int frames;
	unsigned int errors;
	unsigned long first_seen[BITS_TO_LONGS(MTK_SH_TYPE_MAX)];
	struct dentry *debugfs;
};

int mtk_sh_send(struct mtk_sh *sh, const u8 *msg, int len, const char *what);
int mtk_sh_set_delay(struct mtk_sh *sh, u8 type, u32 delay_ms);
int mtk_sh_activate(struct mtk_sh *sh, u8 type, bool enable);

int mtk_sh_iio_register(struct mtk_sh *sh);
void mtk_sh_iio_push(struct mtk_sh *sh, const struct mtk_sh_frame *frame);

#endif /* __MTK_SENSORHUB_H */
