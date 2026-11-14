// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek CPU hotplug: power a core down through the ATF after PSCI CPU_OFF.
 *
 * On MediaTek phone SoCs of the MT6785 / MT6885 generation the ATF's PSCI
 * CPU_OFF leaves the core in a state from which the next CPU_ON never
 * completes, unless the kernel also asks the ATF to power the core down with
 * the SIP call MTK_SIP_POWER_DOWN_CORE (0x212). The vendor 4.14 / 4.19 kernels
 * do this in drivers/misc/mediatek/base/power/cpuhotplug (poweroff_cpu() from
 * cpuhp_platform_cpuoff()); the same fix for the MT6885 is in the cezanne
 * mainline port (linux/patches/0033). Without it, onlining a core that was
 * taken down (sysfs, kexec, suspend) hangs the phone until the watchdog.
 */
#include <linux/arm-smccc.h>
#include <linux/cpuhotplug.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>

#define MTK_SIP_POWER_DOWN_CORE		MTK_SIP_SMC_CMD(0x212)

static int mtk_cpuhp_sip_dead(unsigned int cpu)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_POWER_DOWN_CORE, cpu, 0, 0, 0, 0, 0, 0, &res);
	return 0;
}

static const char *const mtk_cpuhp_sip_machines[] __initconst = {
	"mediatek,mt6785",
	"mediatek,mt6885",
	NULL
};

static int __init mtk_cpuhp_sip_init(void)
{
	int ret;

	if (!of_machine_compatible_match(mtk_cpuhp_sip_machines))
		return 0;

	ret = cpuhp_setup_state_nocalls(CPUHP_BP_PREPARE_DYN, "soc/mediatek/cpuhp-sip:dead",
					NULL, mtk_cpuhp_sip_dead);
	return ret < 0 ? ret : 0;
}
early_initcall(mtk_cpuhp_sip_init);
