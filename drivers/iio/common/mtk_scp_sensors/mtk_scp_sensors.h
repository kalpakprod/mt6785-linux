/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The pieces of the SCP sensor hub, in the order they have to start.
 *
 * The BSP stack is four drivers (hf_manager, nanohub, mtk_nanohub, scp_iio)
 * with a load order; here they are one module and hub.c starts them in that
 * order, because a module's own initcalls in the .initcall sections never run
 * (they are only executed for built-in code).
 */
#ifndef __MTK_SCP_SENSORS_H
#define __MTK_SCP_SENSORS_H

int hf_manager_init(void);
int nanohub_init(void);
int mtk_nanohub_init(void);
void mtk_nanohub_exit(void);
int scp_iio_init(void);
void scp_iio_exit(void);

#endif /* __MTK_SCP_SENSORS_H */
