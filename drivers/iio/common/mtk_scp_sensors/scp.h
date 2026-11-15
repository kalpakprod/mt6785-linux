/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The SCP side of the sensor hub, as the ported BSP sources spell it.
 *
 * The sensor hub sources below are the MT6885 mainline port of MediaTek's BSP
 * sensor stack (scratch/native/4-modem/src/cezanne-linux-port/linux/drivers/
 * scp/sensor/), which in turn is the 4.14 BSP code.  They call the SCP API by
 * its BSP names (scp_ipi_send(), scp_get_reserve_mem_virt(), ...).  On MT6785
 * that API is provided by the Cortex-M4 SCP driver, whose functions are
 * prefixed scp_cm4_ because the in-tree MT8183 SCP driver already exports
 * scp_ipi_send() for its own rpmsg use; these wrappers keep the ported sources
 * readable and their diff against the originals small.
 */
#ifndef __MTK_SCP_SENSORS_SCP_H
#define __MTK_SCP_SENSORS_SCP_H

#include "../../../remoteproc/mtk_scp_cm4.h"

static inline enum scp_ipi_status scp_ipi_registration(enum scp_cm4_ipi_id id,
	void (*handler)(int id, void *data, unsigned int len),
	const char *name)
{
	return scp_cm4_ipi_registration(id, handler, name);
}

static inline enum scp_ipi_status scp_ipi_unregistration(enum scp_cm4_ipi_id id)
{
	return scp_cm4_ipi_unregistration(id);
}

static inline enum scp_ipi_status scp_ipi_send(enum scp_cm4_ipi_id id,
	void *buf, unsigned int len, unsigned int wait,
	enum scp_core_id core)
{
	return scp_cm4_ipi_send(id, buf, len, wait, core);
}

static inline unsigned int is_scp_ready(enum scp_core_id core)
{
	return scp_cm4_is_ready(core);
}

static inline void scp_A_register_notify(struct notifier_block *nb)
{
	scp_cm4_notify_register(nb);
}

static inline void scp_A_unregister_notify(struct notifier_block *nb)
{
	scp_cm4_notify_unregister(nb);
}

static inline int scp_awake_lock(void *core)
{
	return scp_cm4_awake_lock(core);
}

static inline int scp_awake_unlock(void *core)
{
	return scp_cm4_awake_unlock(core);
}

static inline phys_addr_t scp_get_reserve_mem_phys(enum scp_reserve_mem_id_t id)
{
	return scp_cm4_reserve_mem_phys(id);
}

static inline phys_addr_t scp_get_reserve_mem_virt(enum scp_reserve_mem_id_t id)
{
	return scp_cm4_reserve_mem_virt(id);
}

static inline phys_addr_t scp_get_reserve_mem_size(enum scp_reserve_mem_id_t id)
{
	return scp_cm4_reserve_mem_size(id);
}

static inline void scp_register_feature(enum feature_id id)
{
	scp_cm4_register_feature(id);
}

static inline void scp_deregister_feature(enum feature_id id)
{
	scp_cm4_deregister_feature(id);
}

static inline void scp_wdt_reset(int cpu_id)
{
	scp_cm4_wdt_reset(cpu_id);
}

#endif /* __MTK_SCP_SENSORS_SCP_H */
