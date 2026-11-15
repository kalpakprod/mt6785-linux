// SPDX-License-Identifier: GPL-2.0
/*
 * The sensor hub behind the MT6785 SCP, as one module.
 *
 * Ported from the MT6885 mainline port,
 * scratch/native/4-modem/src/cezanne-linux-port/linux/drivers/scp/sensor/
 * (MediaTek BSP sensors-1.0: hf_manager, nanohub, mtk_nanohub, scp_iio), with
 * three deliberate changes for this phone:
 *
 *  - the SCP API is the Cortex-M4 one (scp.h here maps the BSP names onto the
 *    scp_cm4_* functions of the remoteproc driver), and the IPI ids are this
 *    firmware's (IPI_SENSOR = 8 and friends, vendor scp/cm4/include/scp_cm4.h);
 *  - the IIO table lists this phone's sensors: the front light sensor is a
 *    second, separate sensor on the MT6785 too (TSL2540 behind the firmware's
 *    lightSecondary module, while the rear BU27030NUC is the main one);
 *  - the four BSP drivers are one module whose init calls them in load order,
 *    because initcalls in a module never run: only the module's own init does.
 */
#include <linux/init.h>
#include <linux/module.h>

#include "mtk_scp_sensors.h"

/*
 * Order matters: hf_manager creates the sensor core and /dev/hf_manager,
 * mtk_nanohub is the driver that fills it from the SCP and needs the SCP to be
 * up (it waits for the ready notification), scp_iio then reads from the core.
 * nanohub (the CHRE char device) binds to its own device tree node and stays
 * idle without one.
 */
static int __init mtk_scp_sensors_init(void)
{
	int ret;

	ret = hf_manager_init();
	if (ret)
		return ret;

	ret = nanohub_init();
	if (ret)
		return ret;

	ret = mtk_nanohub_init();
	if (ret)
		return ret;

	return scp_iio_init();
}

static void __exit mtk_scp_sensors_exit(void)
{
	scp_iio_exit();
	mtk_nanohub_exit();
}

module_init(mtk_scp_sensors_init);
module_exit(mtk_scp_sensors_exit);

MODULE_DESCRIPTION("Mediatek SCP sensor hub (MT6785)");
MODULE_LICENSE("GPL");
