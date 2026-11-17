// SPDX-License-Identifier: GPL-2.0
/*
 * MT6785 SPM firmware kick
 *
 * Why this exists, in the vendor's own terms: the SPM's PCM firmware is what makes the
 * modem's 26 MHz request and the AP's own MD source request observable. PCM_REG13_DATA is
 * a PCM-block data register (SPM 0x110, vendor drivers/misc/mediatek/base/power/spm/mt6785/
 * mtk_spm_reg.h:88) and the vendor decides "MD1 is asleep" from its bit 2
 * (spm_is_md1_sleep(), mtk_spm_utils.c:36-39, bit R13_MD_SRCCLKENA_0, pcm_def.h:148); the
 * AP's MD source request is acknowledged from the same block (AP_MDSMSRC_REQ bit 0 /
 * AP_MDSMSRC_ACK bit 4, mtk_spm_reg.h:1847-1852, waited for in check_ap_mdsrc_ack(),
 * mtk_spm_utils.c:59-72).
 *
 * The image itself is the loader's: LK reads the spmfw partition and hands it to the secure
 * monitor in one SMC (our LK log: "[spmfw] before smc call" / "after smc call", 1 ms,
 * poco-m5s/evidence/try-safe1/pl_lk.txt:2293-2304; GPT entries at :1051 and :1071). What
 * the vendor kernel adds afterwards is the *kick*, and it is a plain SMC:
 *
 *	late_initcall(mtk_sleep_init)                    spm/common/mtk_sleep_init.c:19-30
 *	  -> mtk_spm_init()                              mt6785/mtk_spm.c:415-421
 *	    -> spm_module_init()                         mt6785/mtk_spm.c:330
 *	      -> SMC_CALL(ARGS, SPM_ARGS_SPMFW_IDX_KICK, spm_get_spmfw_idx(),
 *	                  dram_steps_freq(0))            mt6785/mtk_spm.c:405-408
 *
 * with, on this SoC: MTK_SIP_KERNEL_SPM_ARGS = 0x8200022A | AARCH_BIT
 * (mt-plat/mtk_secure_api.h:90-91), SPM_ARGS_SPMFW_IDX_KICK = 0
 * (mt6785/mtk_spm_internal.h:143-146), the firmware index from spm_get_spmfw_idx()
 * (mt6785/mtk_spm_dram.c:41-47, 0 unless the DRAM type selects another) and
 * dram_steps_freq(0), 4266 when CONFIG_MTK_DRAMC is absent (mt6785/mtk_spm.c:31-36).
 *
 * The status call that reports the result is MTK_SIP_KERNEL_SPM_FIRMWARE_STATUS
 * (0x82000221 | AARCH_BIT, mt-plat/mtk_secure_api.h:66), whose return value the vendor's
 * comment fixes: -1 not initialised, 0 not loaded, 1 loaded, 2 loaded and kicked
 * (mt6785/mtk_spm_utils.c:29-36).
 *
 * What this driver deliberately does NOT do: none of the SPM register programming that
 * precedes the kick in the vendor's spm_module_init() (wakeup masks, PCM timers, power-on
 * values). Whether the secure monitor needs any of it is not knowable from the sources we
 * have -- ATF is not among them -- which is exactly what this build measures: it kicks,
 * then reads the status back and says what happened.
 */

#include <linux/arm-smccc.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define MTK_SIP_KERNEL_SPM_ARGS			0xc200022a	/* 0x8200022A | AARCH_BIT */
#define MTK_SIP_KERNEL_SPM_FIRMWARE_STATUS	0xc2000221	/* 0x82000221 | AARCH_BIT */

#define SPM_ARGS_SPMFW_IDX_KICK			0
#define SPMFW_IDX_DEFAULT			0
#define DRAM_STEPS_FREQ_DEFAULT			4266		/* mt6785/mtk_spm.c:31-36 */

static int mt6785_spm_kick_probe(struct platform_device *pdev)
{
	struct arm_smccc_res status, kick;

	arm_smccc_smc(MTK_SIP_KERNEL_SPM_FIRMWARE_STATUS, 0, 0, 0, 0, 0, 0, 0, &status);
	dev_info(&pdev->dev, "spm firmware before the kick: %ld (%s)\n", (long)status.a0,
		 status.a0 == 2 ? "loaded and kicked" :
		 status.a0 == 1 ? "loaded" : status.a0 == 0 ? "NOT loaded" : "not init / refused");

	/*
	 * The vendor's kick, argument for argument: (ARGS, SPMFW_IDX_KICK, firmware index,
	 * DRAM step frequency).
	 */
	arm_smccc_smc(MTK_SIP_KERNEL_SPM_ARGS, SPM_ARGS_SPMFW_IDX_KICK,
		      SPMFW_IDX_DEFAULT, DRAM_STEPS_FREQ_DEFAULT, 0, 0, 0, 0, &kick);
	dev_info(&pdev->dev,
		 "spm kick: SMC %#x args %u/%u/%u returned %ld (a1 %ld a2 %ld a3 %ld)\n",
		 MTK_SIP_KERNEL_SPM_ARGS, SPM_ARGS_SPMFW_IDX_KICK, SPMFW_IDX_DEFAULT,
		 DRAM_STEPS_FREQ_DEFAULT, (long)kick.a0, (long)kick.a1, (long)kick.a2,
		 (long)kick.a3);

	arm_smccc_smc(MTK_SIP_KERNEL_SPM_FIRMWARE_STATUS, 0, 0, 0, 0, 0, 0, 0, &status);
	dev_info(&pdev->dev, "spm firmware after the kick: %ld (%s)\n", (long)status.a0,
		 status.a0 == 2 ? "loaded and kicked" :
		 status.a0 == 1 ? "still only loaded" :
		 status.a0 == 0 ? "NOT loaded" : "not init / refused");
	return 0;
}

static const struct of_device_id mt6785_spm_kick_of_match[] = {
	/* the vendor's own node: scpsys@10006000, "mediatek,mt6785-scpsys" (mt6785.dtsi:943) */
	{ .compatible = "mediatek,mt6785-scpsys" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6785_spm_kick_of_match);

static struct platform_driver mt6785_spm_kick_driver = {
	.probe = mt6785_spm_kick_probe,
	.driver = {
		.name = "mtk-spm-mt6785-kick",
		.of_match_table = mt6785_spm_kick_of_match,
	},
};
module_platform_driver(mt6785_spm_kick_driver);

MODULE_DESCRIPTION("MT6785 SPM firmware kick (PROBE: does the PCM firmware run without the vendor's register setup?)");
MODULE_LICENSE("GPL");
