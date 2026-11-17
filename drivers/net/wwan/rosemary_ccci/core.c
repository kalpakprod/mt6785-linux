// SPDX-License-Identifier: GPL-2.0-only
/*
 * Poco M5s (rosemary): MediaTek MT6785 modem -- probe, power and boot.
 *
 * The modem is MD1, generation 6295: a second processor behind the SCPSYS
 * MTCMOS gate at 0x10006000, reaching the AP over CCIF for control and DPMAIF
 * for packet data. This file runs its life cycle.
 *
 *   probe  takes the domain through runtime PM and reads back its power status
 *          (M0), maps the register windows and the two share-memory windows and
 *          reads the bootloader's boot information (tags.c, M1). Nothing is
 *          written to the modem.
 *   boot   (write 1 to the device's "boot" attribute) clears and rebuilds the
 *          share memory, powers the modem, releases it and answers its first
 *          handshake. stop_after decides where the build stops: m2 leaves the
 *          modem at the first handshake, m3 goes on to the second one once
 *          userspace has served its file service.
 *
 * The watchdog and the exception path are wired from the first boot: a modem
 * that panics must say so instead of going silent, which is the difference
 * between a day and a week spent hunting a handshake timeout.
 *
 * Where the facts come from: the vendor's own MT6785 platform code
 * (drivers/misc/mediatek/eccci/mt6785/md_sys1_platform.c and modem_reg_base.h
 * in LineageOS android_kernel_xiaomi_mt6785 30485853fa89), the port plan
 * (poco-m5s/research/native/4-modem/5-port-plan.md §1), and for the part
 * mainline already owns, drivers/pmdomain/mediatek/mt6785-pm-domains.h.
 */
#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>

#include "rosemary_ccci.h"

/*
 * The AP's clock request to the modem: one SMC, then the SPM's ack at
 * SPM+0x434 bit 4 (vendor base/power/spm/mt6785/mtk_spm_utils.c:64-118 and
 * mtk_spm_reg.h:265,1851); the id is MTK_SIP_KERNEL_SPM_AP_MDSRC_REQ with the
 * 64-bit call bit.
 */
#define SIP_AP_MDSRC_REQ	0xc2000223

/* the modem's SRAM supply: PMIC_RG_LDO_VSRAM_MD_VOSEL = 0x4f, 993 750 uV */
#define VSRAM_MD_UV		993750

#define HS1_TIMEOUT		(30 * HZ)
#define HS2_TIMEOUT		(30 * HZ)

static bool m0_only;
module_param(m0_only, bool, 0444);
MODULE_PARM_DESC(m0_only, "stop at the M0 power-domain check");

static char *stop_after = "m2";
module_param(stop_after, charp, 0444);
MODULE_PARM_DESC(stop_after,
		 "the last stage this build runs: m0, m1, m2, m3 or m4");

static const char *const state_names[] = {
	[MD_STATE_OFF] = "off",
	[MD_STATE_BOOTING] = "booting",
	[MD_STATE_HS1] = "handshake",
	[MD_STATE_READY] = "ready",
	[MD_STATE_EXCEPTION] = "exception",
};

const char *ccci_state_name(enum ccci_state state)
{
	if (state >= ARRAY_SIZE(state_names))
		return "?";
	return state_names[state];
}

/* the stage the module was told to stop at; m0_only overrides it */
static int stage_limit(void)
{
	if (m0_only || !stop_after)
		return 0;
	if (!strcmp(stop_after, "m1"))
		return 1;
	if (!strcmp(stop_after, "m2"))
		return 2;
	if (!strcmp(stop_after, "m3"))
		return 3;
	if (!strcmp(stop_after, "m4"))
		return 4;
	return 0;
}

void ccci_set_state(struct ccci_md *md, enum ccci_state state)
{
	if (md->state == state)
		return;
	if (md->state == MD_STATE_READY)
		ccci_ports_online(md, false);
	md->state = state;
	if (state == MD_STATE_READY)
		ccci_ports_online(md, true);
	dev_info(md->dev, "state %s\n", state_names[state]);
	sysfs_notify(&md->dev->kobj, NULL, "state");
}

/* ---- clocks and the power sequence ------------------------------------ */

/*
 * The AP clock request to the modem. Without it the modem's registers answer
 * nothing; the vendor waits 3 ms for the 26 MHz to settle, then for the ack.
 */
static void ap_mdsrc_req(struct ccci_md *md, bool on)
{
	struct arm_smccc_res r;
	u32 v;

	arm_smccc_smc(SIP_AP_MDSRC_REQ, on ? 1 : 0, 0, 0, 0, 0, 0, 0, &r);
	if (r.a0)
		dev_warn(md->dev, "AP_MDSRC_REQ(%d) returned %#lx\n", on, r.a0);
	if (!on)
		return;
	usleep_range(3000, 4000);
	if (readl_poll_timeout(md->spm + SPM_AP_MDSRC_REQ, v,
			       v & SPM_AP_MDSRC_ACK, 100, 20000))
		dev_warn(md->dev, "no AP_MDSRC ack (%#x)\n", v);
}

/*
 * What the vendor does around mainline's genpd sequence: the two supplies, the
 * AP clock request, the modem's 26 MHz source request and the SPM register
 * unlock. The MTCMOS gate itself (0x318, 0x3b0 and the bus protection) belongs
 * to the genpd driver and is not written here.
 */
/*
 * The two halves of the modem's power state, kept apart because the vendor keeps
 * them apart and the order between them matters.
 *
 * Vendor md_cd_power_on() for this SoC (mt6785/md_sys1_platform.c:705-770) is, in
 * this order: the PMIC (:713), then the modem's 26 MHz source request
 * INFRA_AO_MD_SRCCLKENA low byte 0x21 (:727-733), then the MTCMOS gate (:744-745).
 * md_cd_power_off() (:794-828) is the same three steps backwards: the gate first
 * (:809), then srclkena (:812-818), then the PMIC (:821-828).
 *
 * The AP's own clock request to the modem -- the SMC in spm_ap_mdsrc_req() plus its
 * ack poll, base/power/spm/mt6785/mtk_spm_utils.c:64-120 -- belongs to that same
 * middle step: the vendor sets it while the modem is powered and holds it until the
 * power-off.
 *
 * Splitting them here is what lets the boot keep the vendor's order without ever
 * touching the rails while the domain is on: the clock can be asked for with the
 * domain live, the rails cannot.
 */
static bool md_domain_is_on(struct ccci_md *md);

static void md_rails_cut(struct ccci_md *md)
{
	/*
	 * force_disable(), not disable(): on the loader's boot this driver never took
	 * a reference to the rails, so regulator_disable() would be an unbalanced put.
	 * The bookkeeping is not the question here either -- the question is whether the
	 * modem still has its supply, and after a stop the answer must be no.
	 *
	 * Called only with the domain already off: that is the only state the vendor
	 * ever reaches its PMIC step in (md_cd_power_off() cuts the gate first,
	 * mt6785/md_sys1_platform.c:809, and the PMIC step is :823-824).
	 */
	if (!md->vmodem || !md->vsram_md)
		return;
	regulator_force_disable(md->vmodem);
	regulator_force_disable(md->vsram_md);
	md->rails_on = false;
}

static int md_rails_prepare(struct ccci_md *md)
{
	int ret;

	if (md->rails_on)
		return 0;
	ret = regulator_set_voltage(md->vsram_md, VSRAM_MD_UV, VSRAM_MD_UV);
	if (ret)
		dev_warn(md->dev, "vsram_md voltage: %d\n", ret);
	ret = regulator_enable(md->vsram_md);
	if (ret)
		return dev_err_probe(md->dev, ret, "vsram_md\n");
	ret = regulator_enable(md->vmodem);
	if (ret) {
		regulator_disable(md->vsram_md);
		return dev_err_probe(md->dev, ret, "vmodem\n");
	}
	md->rails_on = true;
	return 0;
}

/*
 * The runtime-PM reference the driver owns.
 *
 * The modem's domain must stay on between probe and stop -- the vendor holds the
 * same thing with clk_prepare_enable(clk_table[0]) and gives it back only in
 * md_cd_power_off() (mt6785/md_sys1_platform.c:744-745, :809) -- and genpd will
 * not power a domain off while one of its devices is in use. So exactly one
 * reference is held for the driver's lifetime, and this flag is what makes that
 * exact: a bare put that assumed one was held underflowed (trial 24), while a
 * get/put pair meant to drop the domain left the count at one and the clamp never
 * ran at all (trial 25: power_off reported dev_count=1 and did nothing).
 */
static void md_pm_get(struct ccci_md *md)
{
	int ret;

	if (md->pm_held)
		return;
	ret = pm_runtime_resume_and_get(md->dev);
	if (ret) {
		dev_err(md->dev, "cannot take the md domain's reference: %d\n", ret);
		return;
	}
	md->pm_held = true;
}

static void md_pm_put(struct ccci_md *md)
{
	if (!md->pm_held)
		return;
	md->pm_held = false;
	pm_runtime_put_sync(md->dev);
}

/*
 * The modem's watchdog interrupt, enabled exactly while a released modem may
 * assert it. The vendor disables it before it stops a running modem
 * (md_cd_pre_stop(), modem_sys1.c:496) so the handler cannot race the stop, and
 * this flag keeps our enable/disable balanced against the interrupt's own
 * IRQF_NO_AUTOEN depth.
 */
static void md_wdt_irq(struct ccci_md *md, bool on)
{
	if (on == md->wdt_irq_on)
		return;
	md->wdt_irq_on = on;
	if (on) {
		enable_irq(md->irq_wdt);
		dev_info(md->dev, "wdt: the modem's watchdog interrupt is enabled\n");
	} else {
		disable_irq_nosync(md->irq_wdt);
		dev_info(md->dev, "wdt: the modem's watchdog interrupt is disabled, as the vendor's pre-stop does (md_cd_pre_stop(), modem_sys1.c:496)\n");
	}
}

/*
 * The session's grant of the modem's 26 MHz source clock: INFRA_AO_MD_SRCCLKENA's low
 * byte, 0x21 for on. The vendor sets it in md_cd_power_on() (mt6785/md_sys1_platform.c:
 * 713-726) and clears it only after the clamp has been acknowledged (:813-816), so it
 * is a session-wide thing and not part of any single register access.
 *
 * Two things that used to live here and no longer do, both because the vendor does not
 * do them on this SoC:
 *
 *  - the AP's own MD source request (SPM_AP_MDSRC_REQ, 0x434). The vendor holds it only
 *    around an AP access to the modem's registers, through
 *    md_cd_lock_modem_clock_src(1/0) (:332-360, used by modem_sys1.c:1047-1059,
 *    mdee_dumper_v3.c:371-376 and :912-935) and never across its power-off. Holding it
 *    for the whole session was one of the two things our stop did that the vendor's
 *    does not; it is md_md_access() now;
 *  - SPM_CLK_CON bit 21 (SRCLKEN_O1). That is the vendor's MT6781 path only:
 *    mtk_ccci_cfg_srclken_o1_on() (md_sys1_platform.c:660-676) is called under
 *    #ifdef CCCI_PLATFORM_MT6781 inside md_cd_power_on() (:713-724). MT6785 does not
 *    set it, and this port is MT6785 only.
 */
static void md_srcclkena_grant(struct ccci_md *md, bool on)
{
	if (!on) {
		writel(readl(md->infra + INFRA_MD_SRCCLKENA) & ~0xff,
		       md->infra + INFRA_MD_SRCCLKENA);
		return;
	}
	/* the modem's 26 MHz source clock request: low byte 0x21 */
	writel((readl(md->infra + INFRA_MD_SRCCLKENA) & ~0xff) | 0x21,
	       md->infra + INFRA_MD_SRCCLKENA);
}

/*
 * md_md_access() - hold the AP's MD source request across one access to the modem
 * @md: the modem
 * @on: true before the access, false after it
 *
 * This is the vendor's md_cd_lock_modem_clock_src() in behaviour: the AP asks the SPM
 * for the modem's SM clock, the SPM acknowledges (SPM_AP_MDSRC_REQ bits 0 and 4,
 * mtk_spm_reg.h:1847-1852), the AP touches the modem's registers, and the request is
 * released. Nothing may hold it for longer: the vendor never does, and at the moment
 * its bus protect is set the request is free.
 */
static void md_md_access(struct ccci_md *md, bool on)
{
	ap_mdsrc_req(md, on);
}

/*
 * The CCIF clocks, with the two counters a trial reads to prove they are where they
 * should be. The vendor has no hif at the moment it sets the bus protect: ccci_hif_start()
 * runs after the power cycle, and ccif_set_clk_on() only inside ccif_start()
 * (modem_sys1.c:396; ccci_hif_ccif.c:2075). So the clocks come off before the clamp and
 * go back on after the power-on.
 */
static void md_clks(struct ccci_md *md, bool on)
{
	if (on == md->clks_on)
		return;
	if (on) {
		if (clk_bulk_prepare_enable(md->num_clks, md->clks))
			return;
		md->clks_on = true;
		md->clk_on_count++;
	} else {
		clk_bulk_disable_unprepare(md->num_clks, md->clks);
		md->clks_on = false;
		md->clk_off_count++;
	}
}

/* the vendor's power-on: rails and clock, then the gate by the caller */
static int md_power_seq_prepare(struct ccci_md *md)
{
	int ret = md_rails_prepare(md);

	if (ret)
		return ret;
	md_srcclkena_grant(md, true);
	return 0;
}

/* the vendor's power-off: clock and rails, the gate already off by the caller */
static void md_power_seq_release(struct ccci_md *md)
{
	/*
	 * The vendor's last two steps of the power-off, in its order: the 26 MHz
	 * request, then the PMIC (md_cd_power_off(), mt6785/md_sys1_platform.c:813-824).
	 * Both come after the gate by construction, so if the gate is still on these
	 * steps are not reached at all -- the vendor never reaches them either, because
	 * it waits for the bus-protect acknowledgement before (:1009-1042 of
	 * clk-mt6785-pg.c).
	 */
	if (md_domain_is_on(md)) {
		dev_warn(md->dev,
			 "supplies: not releasing the 26 MHz request or the rails while the md domain is on (PWR_STATUS %#x)\n",
			 readl(md->spm + SPM_PWR_STATUS));
		return;
	}
	md_srcclkena_grant(md, false);
	md_rails_cut(md);
}

/*
 * First boot: the loader already powered the modem and ran its boot ROM once to
 * verify the image. Wait for the boot ROM to report, read the two boot
 * statistics words and leave the modem off -- our boot is a warm restart of it
 * (vendor md_start_platform and md_cd_dump_md_bootup_status,
 * mt6785/md_sys1_platform.c:473-500 and :583-660).
 */
static int md_first_boot_check(struct ccci_md *md)
{
	unsigned long deadline = jiffies + 2 * HZ;
	u32 v0, v1, v2, v3, stats;

	md_md_access(md, true);
	for (;;) {
		v0 = readl(md->sec_ao + SEC_AO_BROM0);
		v1 = readl(md->sec_ao + SEC_AO_BROM1);
		v2 = readl(md->sec_ao + SEC_AO_BROM2);
		v3 = readl(md->sec_ao + SEC_AO_BROM3);
		if (v0 == 1 && v1 == 1 && v2 == 1 && v3 == 1) {
			dev_info(md->dev, "boot ROM pass\n");
			break;
		}
		if (time_after(jiffies, deadline)) {
			md_md_access(md, false);
			dev_err(md->dev, "boot ROM did not pass: %#x %#x %#x %#x\n",
				v0, v1, v2, v3);
			return -ETIMEDOUT;
		}
		msleep(20);
	}
	/*
	 * Read three times and believe the third: the vendor's own note says the
	 * AP/MD interface needs the delay.
	 */
	writel(0, md->pll + MD1_BOOT_STATS_SELECT);
	readl(md->pll + MD1_CFG_BOOT_STATS);
	readl(md->pll + MD1_CFG_BOOT_STATS);
	stats = readl(md->pll + MD1_CFG_BOOT_STATS);
	dev_info(md->dev, "md_boot_stats0 %#x\n", stats);
	writel(1, md->pll + MD1_BOOT_STATS_SELECT);
	readl(md->pll + MD1_CFG_BOOT_STATS);
	readl(md->pll + MD1_CFG_BOOT_STATS);
	stats = readl(md->pll + MD1_CFG_BOOT_STATS);
	dev_info(md->dev, "md_boot_stats1 %#x\n", stats);
	return 0;
}

/*
 * The CCB logging buffers' control headers: one 64-byte block each, page size,
 * buffer size and end guard for the downlink and the uplink half, written before
 * every boot. Without them the modem asserts while checking the buffers. The
 * slot table is the vendor's ccb_configs[] for this generation
 * (eccci/port/port_smem.c:78-129), the guard value libccci_util's.
 */
static const struct {
	u32 dl_page, ul_page, dl_size, ul_size;
} ccb_slots[] = {
	{ 1024, 1024, 1024 * 32, 1024 * 32 },			/* DHL control */
	{ 20 * 1024, 128, 20 * 1024 * 6, 128 },			/* MD exception */
	{ 512, 1024, 512 * 72, 1024 * 32 },			/* PS */
	{ 512, 128, 512 * 64, 128 },				/* HW logger 1 */
	{ 512, 128, 512 * 64, 128 },				/* HW logger 2 */
	{ 512, 128, 512 * 256, 128 },				/* HW logger 3 */
	{ 128, 128, 128, 128 },					/* dummies */
	{ 128, 128, 128, 128 },
	{ 128, 128, 128, 128 },
	{ 128, 128, 128, 128 },
	{ 128, 128, 128, 128 },
	{ 128, 128, 128, 128 },
	{ 128, 128, 128, 128 },
	{ 128, 128, 128, 128 },
	{ 128, 128, 128, 128 },
	{ 128, 128, 128, 128 },
	{ 128, 128, 128, 128 },
	{ 128, 128, 128 * 5, 128 },				/* padding */
	{ 512, 1024, 512 * 32, 1024 * 16 },			/* MD monitor */
	{ 65 * 1024, 63 * 1024, 65 * 1024 * 8, 63 * 1024 * 8 },	/* META */
};

#define CCB_GUARD_END	0xeeff0011

static int ccb_ctrl_init(struct ccci_md *md)
{
	u32 *h = md->nc + NC_CCB_CTRL;
	int i;

	if (!md->nc || NC_CCB_CTRL + NC_CCB_CTRL_SIZE > md->nc_size) {
		dev_err(md->dev, "the CCB control block %#x + %#x is outside the mapped %#llx\n",
			NC_CCB_CTRL, NC_CCB_CTRL_SIZE, (u64)md->nc_size);
		return -EINVAL;
	}
	memset(h, 0, NC_CCB_CTRL_SIZE);
	for (i = 0; i < ARRAY_SIZE(ccb_slots); i++, h += 16) {
		h[5] = ccb_slots[i].dl_page;
		h[6] = ccb_slots[i].dl_size;
		h[7] = CCB_GUARD_END;
		h[13] = ccb_slots[i].ul_page;
		h[14] = ccb_slots[i].ul_size;
		h[15] = CCB_GUARD_END;
	}
	wmb();
	return 0;
}

/*
 * Every write into a share-memory window goes through this: a region that is
 * not entirely inside the mapped window is refused with a message naming it,
 * and nothing is written. The first boot on the phone had a constant table
 * from another phone put the CCB buffers 64 MiB into a 23.5 MiB window; the
 * memset ran off the mapping and oopsed at the first page past it.
 */
static int smem_clear(struct ccci_md *md, const char *window, void *base,
		      resource_size_t mapped, u32 off, u32 size)
{
	if (!size)
		return 0;
	if (!base || off > mapped || size > mapped - off) {
		dev_err(md->dev, "%s: %#x + %#x is outside the mapped %#llx, not cleared\n",
			window, off, size, (u64)mapped);
		return -EINVAL;
	}
	memset(base + off, 0, size);
	return 0;
}

/**
 * md_dump_state() - print what a failed handshake needs to be diagnosed
 * @md: the modem
 * @when: a short word for the moment this is printed at
 *
 * The first handshake attempt on the phone timed out with PWR_STATUS reading
 * 0x83f7fffe -- bit 0 clear, i.e. the modem's MTCMOS domain was OFF for the
 * whole wait. Nothing in the old log could tell that apart from "the domain is
 * on but the modem never started", which are different bugs with different
 * fixes. So every release now prints, on one line per fact:
 *
 *  - PWR_STATUS and PWR_STATUS_2ND, with the md domain's bit called out;
 *  - the CCIF doorbell: BUSY and RCHNUM, which say whether the modem has
 *    rung anything at all;
 *  - the first bytes of the MD image at the address the loader reported, so
 *    an empty or unmapped region is visible as zeros rather than as a silent
 *    timeout;
 *  - the boot-stage register set, read the way the vendor's
 *    md_cd_dump_md_bootup_status() reads it (select 0 and 1, three reads,
 *    believe the third).
 */
static void md_dump_state(struct ccci_md *md, const char *when)
{
	u32 sta, sta_map, sta2, busy, rchnum, stage0, stage1;
	void __iomem *img;

	sta = readl(md->spm + SPM_PWR_STATUS);
	sta2 = readl(md->spm + SPM_PWR_STATUS_2ND);
	busy = md->ccif_ap ? readl(md->ccif_ap + APCCIF_BUSY) : 0;
	rchnum = md->ccif_ap ? readl(md->ccif_ap + APCCIF_RCHNUM) : 0;
	/*
	 * Three independent views of the domain, because they disagreed once and
	 * the disagreement is the whole diagnosis:
	 *  - the SPM register through the syscon the genpd driver itself uses,
	 *  - the same register through the driver's own mapping,
	 *  - what runtime PM believes, which is a cached opinion, not hardware.
	 * bit 0 is the md domain: see mt6785-pm-domains.h (MT6785_POWER_DOMAIN_MD,
	 * sta_mask BIT(0), pwr_sta_offs 0x160) and the vendor clk-mt6785-pg.c:488
	 * (MD1_PWR_STA_MASK (0x1 << 0), PWR_STATUS SPM_REG(0x0160)).
	 */
	regmap_read(md->spm_map, SPM_PWR_STATUS, &sta_map);
	dev_info(md->dev,
		 "%s: PWR_STATUS via syscon %#x / via ioremap %#x (md bit %u vs %u), PWR_STATUS_2ND %#x, runtime PM %s, CCIF busy %#x rchnum %#x\n",
		 when, sta_map, sta, (u32)(sta_map & BIT(0)),
		 (u32)(sta & BIT(0)), sta2,
		 pm_runtime_active(md->dev) ? "active" : "suspended", busy,
		 rchnum);

	writel(0, md->pll + MD1_BOOT_STATS_SELECT);
	readl(md->pll + MD1_CFG_BOOT_STATS);
	readl(md->pll + MD1_CFG_BOOT_STATS);
	stage0 = readl(md->pll + MD1_CFG_BOOT_STATS);
	writel(1, md->pll + MD1_BOOT_STATS_SELECT);
	readl(md->pll + MD1_CFG_BOOT_STATS);
	readl(md->pll + MD1_CFG_BOOT_STATS);
	stage1 = readl(md->pll + MD1_CFG_BOOT_STATS);
	dev_info(md->dev, "%s: md_boot_stats0 %#x, md_boot_stats1 %#x\n",
		 when, stage0, stage1);

	if (!md->md_mem_base)
		return;
	/*
	 * The modem's DRAM window is ordinary system memory, already mapped by
	 * the linear map: ioremap() refuses it ("ioremap attempted on RAM pfn")
	 * and the trial printed a WARN instead of the image. memremap() is the
	 * call that is meant for RAM, and it returns the linear-map address.
	 */
	img = memremap(md->md_mem_base, 32, MEMREMAP_WB);
	if (!img)
		return;
	dev_info(md->dev, "%s: md image at %#llx: %*phN\n", when,
		 md->md_mem_base, 32, img);
	memunmap(img);
}

/**
 * clear_smem() - clear what the modem does not keep, before a boot
 * @md: the modem
 *
 * Vendor clear_smem_region() semantics for a first boot: every region without
 * SMF_NCLR_FIRST is zeroed at its own base and its own size. The regions here
 * are this generation's (ccci_modem.c:113-155): the non-cacheable debug heads,
 * NETD, USB and the ring area, and in the cacheable window the NVRAM cache and
 * the CCB buffers -- and those two are taken from the layout the LOADER
 * reported, not from a constant table, because the offsets differ per phone
 * and per boot.
 */
static int clear_smem(struct ccci_md *md)
{
	static const struct {
		const char *name;
		u32 off, size;
	} nc[] = {
		{ "nc debug..ccism_scp", 0, NC_CCB_CTRL },
		{ "nc netd..audio", NC_NETD, NC_AUDIO - NC_NETD },
		{ "nc ccism_mcu..exp", NC_CCISM_MCU,
		  NC_CCISM_MCU_EXP + NC_CCISM_MCU_EXP_SIZE - NC_CCISM_MCU },
	};
	int i, errors = 0;

	for (i = 0; i < ARRAY_SIZE(nc); i++)
		if (smem_clear(md, nc[i].name, md->nc, md->nc_size, nc[i].off,
			       nc[i].size))
			errors++;

	/*
	 * The cacheable window: exactly the CCB buffers, at the address the
	 * loader's ccb_info tag reports. This generation's table
	 * (md1_6293_cacheable, ccci_modem.c:142-155) has no separately located
	 * NVRAM cache -- its CCB_DHL/CCB_MD_MONITOR/CCB_META entries ARE the
	 * whole cacheable clearing, and the NVRAM cache offset that cezanne's
	 * table carries would here land on top of the CCB region.
	 */
	if (md->ccb_data_base >= md->c_phys &&
	    md->ccb_data_base < md->c_phys + md->c_size) {
		u32 off = md->ccb_data_base - md->c_phys;

		if (smem_clear(md, "c ccb buffers", md->c, md->c_size, off,
			       md->ccb_data_size))
			errors++;
	} else if (md->ccb_data_base) {
		dev_err(md->dev, "the loader's CCB buffers at %#llx are outside the cacheable window %#llx + %#llx, not cleared\n",
			md->ccb_data_base, (u64)md->c_phys, (u64)md->c_size);
		errors++;
	}
	wmb();
	return errors ? -EINVAL : 0;
}

/* ---- the boot state machine ------------------------------------------- */

/*
 * One line per attempt, with the registers that decide it, so a trial can be read
 * instead of argued about. PWR_CON is the domain's own control register (0x318),
 * the two status registers are the pair scpsys_domain_is_on() consults
 * (mtk-pm-domains.c:90-107), and INFRA_TOPAXI_PROTECTEN is the bus protection the
 * vendor's own MD1 sequence writes (clk-mt6785-pg.c:1015-1105).
 */
static void md_domain_trace(struct ccci_md *md, const char *when)
{
	unsigned int sta = 0, sta2 = 0, con = 0, prot = 0;

	regmap_read(md->spm_map, SPM_PWR_STATUS, &sta);
	regmap_read(md->spm_map, SPM_PWR_STATUS_2ND, &sta2);
	regmap_read(md->spm_map, SPM_MD1_PWR_CON, &con);
	/* the infra window is mapped after the probe's first check, so it may be absent */
	if (md->infra)
		prot = readl(md->infra + INFRA_TOPAXI_PROTECTEN);
	dev_dbg(md->dev,
		"domain %-16s: PWR_STATUS=%#x PWR_STATUS_2ND=%#x md_bit=%u PWR_CON(%#x)=%#x PROTECTEN=%#x runtime_status=%d usage=%d\n",
		 when, sta, sta2, (u32)(sta & BIT(0)), SPM_MD1_PWR_CON, con, prot,
		 md->dev->power.runtime_status,
		 atomic_read(&md->dev->power.usage_count));
}

/* the domain's own status bit, the one the vendor polls after power-on */
static bool md_domain_is_on(struct ccci_md *md)
{
	unsigned int sta = 0;

	regmap_read(md->spm_map, SPM_PWR_STATUS, &sta);
	return sta & BIT(0);
}

/**
 * md_boot_stage() - the two words the modem's boot code leaves behind
 * @md: the modem
 * @stage0: first word, select 0
 * @stage1: second word, select 1
 *
 * MD1_BOOT_STATS_SELECT (0x1020e700) picks the word, MD1_CFG_BOOT_STATS
 * (0x1020e300) carries it, read three times with the vendor's own delay note
 * (mt6785/md_sys1_platform.c:361-383: "dump 3 times, and buy-in the 3rd dump
 * value"). Both are AP-side registers, so they answer whether the loader has
 * already run the modem even when nothing else about it is reachable.
 */
/*
 * md_loader_state() - what the loader left, in the four numbers a trial needs
 * @md: the modem
 * @when: a short tag for the log line
 *
 * The vendor's first-boot path decides everything from the boot ROM's four
 * completion words: md_start_platform() polls security_ao 0x824, 0x828, 0x82C and
 * 0x830 for 1 each, and calls that "BROM Pass" (mt6785/md_sys1_platform.c:615-623).
 * Two things about that check are worth having in the log at the earliest moment,
 * because they are what separates "the loader left a modem in its ROM" from "the
 * loader started a modem that has since run":
 *
 *  - those four words are latched once the ROM hands off, so they read 1 on every
 *    boot of ours even when the modem is minutes past the ROM (trial 24 read them
 *    at 26.9 s);
 *  - PCM_REG13_DATA bit 30 (R13_MD_VRF18_REQ_0, base/power/spm/mt6785/pcm_def.h:177)
 *    says the modem holds its 1.8 V request and bit 2 (R13_MD_SRCCLKENA_0, :148) that
 *    it holds its 26 MHz request -- i.e. that it is powered and running, not parked.
 *
 * So this prints the ROM words, the domain bit, the two request bits and the boot
 * stats together, once at probe and once at the clamp.
 */
/*
 * md_ap_snapshot() - the AP-side state at one moment, with the domains that changed
 * @md: the modem
 * @when: a short tag for the log line
 *
 * Trial 33 left one hypothesis standing: the modem's bus master may have a transaction
 * stuck toward a target the AP took away -- a slave inside a power domain that the
 * forced genpd sync powered off (PWR_STATUS went 0x83f7ffff -> 0x27dffd there, and the
 * modem's own boot stats moved 0x2c04000a -> 0x2c04010a at the same moment). A master
 * with an outstanding transaction cannot acknowledge the bus protect, which is exactly
 * what we measure.
 *
 * So the snapshot is taken three times -- at probe, just before the clamp and just
 * after the timeout -- and the important column is the diff: which domain bits were on
 * at the previous moment and are off now.
 *
 * Everything printed here is AP-side and citable:
 *  - PWR_STATUS 0x160 and PWR_STATUS_2ND 0x164 (base/power/spm/mt6785/mtk_spm_reg.h;
 *    mt6785-pm-domains.h:13-14), bit 0 being the modem's own domain
 *    (clk-mt6785-pg.c:488);
 *  - MD1_PWR_CON 0x318, PCM_REG13_DATA 0x110 with bits 2 and 30 named
 *    (pcm_def.h:148, :177), MD_EXT_BUCK_ISO_CON 0x3b0;
 *  - the three bus-protect windows and their statuses (0x220/0x228, 0x250/0x258);
 *  - BUS_DBG_CON at infracfg 0x2fc, whose offset the vendor's own device tree states
 *    (mt6785.dts:1575-1580).
 *
 * Deliberately not read, and named here so the gap is visible: the EMI MPU violation
 * registers (emi_mpu@10226000, mt6785.dts:1842-1844) and the DEVAPC violation registers
 * (devapc@10207000 with its AO windows, mt6785.dts:1565-1572; devapc_ao_md@10019000,
 * :1067-1070). Their register layouts live in the vendor's emi_mpu and devapc drivers,
 * which are in none of the sources we have -- the whole project was searched -- and
 * reading undocumented offsets in those blocks is the same class of risk that hung the
 * phone in trial 31. They go in when the layout can be cited.
 */
static void md_boot_stage(struct ccci_md *md, u32 *stage0, u32 *stage1);

/*
 * md_reset_state() - what the MD's TOPRGU reset bit really reads
 * @md: the modem
 * @when: a short tag
 *
 * A master held in reset must acknowledge a protect, so if the acknowledgement does not
 * come while we hold the reset asserted, the first thing to know is whether the reset is
 * actually asserted. The vendor's own sequence says it is (mtk_wdt.c:155-183 writes
 * SWSYSRST |= BIT(id) | key), but a write that does not take -- wrong key, a locked
 * register, or a layout that differs on this SoC -- would look exactly like a modem that
 * refuses. Reading the register back after the write and again after the retried clamp is
 * the difference between those two stories.
 */
static void md_reset_state(struct ccci_md *md, const char *when)
{
	u32 rst, en;
	bool has_line = md->md_reset != NULL;

	if (!md->toprgu) {
		dev_info(md->dev, "toprgu %s: not mapped, cannot read the reset bit back\n", when);
		return;
	}
	rst = readl(md->toprgu + TOPRGU_SWSYSRST);
	en = readl(md->toprgu + TOPRGU_SWSYSRST_EN);
	dev_info(md->dev,
		 "toprgu %s: SWSYSRST %#x (MD bit %u %s, key %#x), SWSYSRST_EN %#x (MD bit %u), reset line %s\n",
		 when, rst, TOPRGU_MD_RST_BIT,
		 (rst & BIT(TOPRGU_MD_RST_BIT)) ? "SET" : "clear",
		 (u32)(rst & GENMASK(31, 28)), en, (u32)!!(en & BIT(TOPRGU_MD_RST_BIT)),
		 has_line ? "present" : "ABSENT");
}

/*
 * md_vendor_power_on() - the vendor's MD1 power-on, run immediately before the clamp
 *
 * This is the one ordering difference the trials have left (plan doc §2r, O1). On stock
 * the AP does not clamp a domain that has been up for minutes: md_start_platform()
 * enables the MTCMOS gate clock one statement before it disables it again
 * (mt6785/md_sys1_platform.c:629 then :636 via md_cd_power_off() at :809), and enabling
 * that gate runs spm_mtcmos_ctrl_md1(STA_POWER_ON), which clears all three MD protect
 * bits and only then leaves the domain to be clamped microseconds later
 * (clk-mt6785-pg.c:1069-1098).
 *
 * So this performs the same eight writes in the same order, from the driver, at the same
 * point in the chain. It is a deliberate exception to "the domain belongs to genpd": the
 * point of the experiment is to reproduce the vendor's sequence exactly, and genpd has no
 * way to run its power-on for a domain that is already on.
 *
 * The writes, in vendor order (clk-mt6785-pg.c):
 *   1069-1071  MD_EXT_BUCK_ISO_CON bit 0 = 0
 *   1072-1074  MD_EXT_BUCK_ISO_CON bit 1 = 0
 *   1075-1077  MD1_PWR_CON |= PWR_RST_B
 *   1079-1081  MD1_PWR_CON |= PWR_ON
 *   1082-1090  wait, PWR_STATUS bit 0 == 1
 *   1093-1094  PROTECTEN_CLR = MD1_PROT_STEP2_0_MASK   (bits 3|4)
 *   1095-1096  PROTECTEN_1_CLR = MD1_PROT_STEP2_1_MASK (bit 6)
 *   1097-1098  PROTECTEN_CLR = MD1_PROT_STEP1_0_MASK   (bit 7)
 */
static void md_vendor_power_on(struct ccci_md *md)
{
	unsigned int v;
	int ret;

	regmap_clear_bits(md->spm_map, SPM_MD_EXT_BUCK_ISO_CON, BIT(0));
	regmap_clear_bits(md->spm_map, SPM_MD_EXT_BUCK_ISO_CON, BIT(1));
	/*
	 * PWR_RST_B is BIT(0) and PWR_ON is BIT(2) (mtk-pm-domains.c:46-50). The first
	 * version of this wrote BIT(1), which is PWR_ISO: it isolated the domain instead
	 * of releasing its reset, and trial 38's 0xd -> 0xf is that isolation bit
	 * appearing. Corrected here.
	 */
	regmap_set_bits(md->spm_map, SPM_MD1_PWR_CON, MD1_PWR_RST_B);
	regmap_set_bits(md->spm_map, SPM_MD1_PWR_CON, MD1_PWR_ON);
	ret = regmap_read_poll_timeout(md->spm_map, SPM_PWR_STATUS, v, v & BIT(0),
				       10, 1000 * 1000);
	dev_info(md->dev,
		 "vendor power-on: ISO cleared, PWR_RST_B and PWR_ON set, PWR_STATUS %#x (%s), MD1_PWR_CON %#x\n",
		 v, ret ? "the domain did not come back" : "domain on",
		 ({ unsigned int c = 0; regmap_read(md->spm_map, SPM_MD1_PWR_CON, &c); c; }));
	writel(BIT(3) | BIT(4), md->infra + INFRA_TOPAXI_PROTECTEN_CLR);
	writel(BIT(6), md->infra + INFRA_TOPAXI_PROTECTEN_1_CLR);
	writel(BIT(7), md->infra + INFRA_TOPAXI_PROTECTEN_CLR);
	md->vendor_on_ns = ktime_get_ns();
	dev_info(md->dev,
		 "vendor power-on: the three protect clears written at %llu ns (PROTECTEN %#x, PROTECTEN_1 %#x), now the clamp\n",
		 md->vendor_on_ns, readl(md->infra + INFRA_TOPAXI_PROTECTEN),
		 readl(md->infra + INFRA_TOPAXI_PROTECTEN_1));
}

static void md_ap_snapshot(struct ccci_md *md, const char *when)
{
	unsigned int sta = 0, sta2 = 0, con = 0, r13 = 0, iso = 0, prot = 0, prot_sta = 0;
	unsigned int prot1 = 0, prot_sta1 = 0, dbg = 0, changed = 0;
	bool had = md->snap_valid;

	if (!md->spm)
		return;
	regmap_read(md->spm_map, SPM_PWR_STATUS, &sta);
	regmap_read(md->spm_map, SPM_PWR_STATUS_2ND, &sta2);
	regmap_read(md->spm_map, SPM_MD1_PWR_CON, &con);
	regmap_read(md->spm_map, SPM_MD_EXT_BUCK_ISO_CON, &iso);
	r13 = readl(md->spm + SPM_PCM_REG13_DATA);
	if (md->infra) {
		prot = readl(md->infra + INFRA_TOPAXI_PROTECTEN);
		prot_sta = readl(md->infra + INFRA_TOPAXI_PROTECT_STA1);
		prot1 = readl(md->infra + INFRA_TOPAXI_PROTECTEN_1);
		prot_sta1 = readl(md->infra + INFRA_TOPAXI_PROTECT_STA1_1);
		dbg = readl(md->infra + INFRA_BUS_DBG_CON);
	}
	if (had)
		changed = md->snap_pwr_status ^ sta;
	dev_info(md->dev,
		 "%s: PWR_STATUS %#x (was %#x, domains that changed %#x)%s, 2ND %#x, PCM_REG13_DATA %#x (1.8V %u, 26MHz %u), MD1_PWR_CON %#x, ISO %#x, PROTECTEN %#x STA1 %#x, PROTECTEN_1 %#x STA1_1 %#x, BUS_DBG_CON %#x\n",
		 when, sta, md->snap_pwr_status, changed,
		 had ? "" : " (first snapshot)", sta2, r13,
		 (u32)!!(r13 & SPM_R13_MD_VRF18_REQ), (u32)!!(r13 & SPM_R13_MD_SRCCLKENA_0),
		 con, iso, prot, prot_sta, prot1, prot_sta1, dbg);
	md->snap_pwr_status = sta;
	md->snap_pwr_status_2nd = sta2;
	md->snap_valid = true;
}

static void md_loader_state(struct ccci_md *md, const char *when)
{
	unsigned int r13, sta = 0;
	u32 s0 = 0, s1 = 0;

	if (!md->sec_ao || !md->spm)
		return;
	regmap_read(md->spm_map, SPM_PWR_STATUS, &sta);
	r13 = readl(md->spm + SPM_PCM_REG13_DATA);
	if (md->pll)
		md_boot_stage(md, &s0, &s1);
	dev_info(md->dev,
		 "%s: loader left brom %#x %#x %#x %#x, PWR_STATUS md bit %u, PCM_REG13_DATA %#x (1.8V request %u, 26MHz request %u), boot stats %#x/%#x\n",
		 when,
		 readl(md->sec_ao + SEC_AO_BROM0), readl(md->sec_ao + SEC_AO_BROM1),
		 readl(md->sec_ao + SEC_AO_BROM2), readl(md->sec_ao + SEC_AO_BROM3),
		 (u32)(sta & BIT(0)), r13,
		 (u32)!!(r13 & SPM_R13_MD_VRF18_REQ),
		 (u32)!!(r13 & SPM_R13_MD_SRCCLKENA_0), s0, s1);
}

static void md_boot_stage(struct ccci_md *md, u32 *stage0, u32 *stage1)
{
	/* an AP access to the modem's registers: the vendor holds its source request
	 * across exactly these (md_cd_lock_modem_clock_src(), md_sys1_platform.c:332-360) */
	md_md_access(md, true);
	writel(0, md->pll + MD1_BOOT_STATS_SELECT);
	readl(md->pll + MD1_CFG_BOOT_STATS);
	readl(md->pll + MD1_CFG_BOOT_STATS);
	*stage0 = readl(md->pll + MD1_CFG_BOOT_STATS);
	writel(1, md->pll + MD1_BOOT_STATS_SELECT);
	readl(md->pll + MD1_CFG_BOOT_STATS);
	readl(md->pll + MD1_CFG_BOOT_STATS);
	*stage1 = readl(md->pll + MD1_CFG_BOOT_STATS);
	md_md_access(md, false);
}

/* the two helpers debugfs.c needs, so it does not reach into this file's statics */
void ccci_md_boot_stage(struct ccci_md *md, u32 *stage0, u32 *stage1)
{
	md_boot_stage(md, stage0, stage1);
}

void ccci_md_src_req(struct ccci_md *md, bool on)
{
	ap_mdsrc_req(md, on);
}

/*
 * One line per step of the boot prefix.
 *
 * The trial needed to see WHERE power went when the boot started early, and this
 * is what showed it: twelve lines per boot, each with the md bit, PWR_STATUS and
 * the domain's control register. They did their job, so they are debug-level now:
 * enable them for a trial with
 *
 *   echo 'file core.c +p' > /sys/kernel/debug/dynamic_debug/control
 */
static void md_step(struct ccci_md *md, const char *step)
{
	unsigned int sta = 0, con = 0;

	regmap_read(md->spm_map, SPM_PWR_STATUS, &sta);
	regmap_read(md->spm_map, SPM_MD1_PWR_CON, &con);
	/*
	 * dev_info, not dev_dbg: the step trace is what a trial reads to know which
	 * branch ran, and four trials were read without it because dynamic debug is
	 * not on by default (trial 28 asked "did the pre-stop run?" -- the answer was
	 * in a line nobody could see).
	 */
	dev_info(md->dev, "step %-22s: md_bit=%u STA=%#x CON=%#x\n",
		 step, (u32)(sta & BIT(0)), sta, con);
}


/*
 * md_dump_pm_state() - why genpd did or did not power the domain off, from the
 * real state rather than from a guess
 * @md: the modem
 * @when: a short tag for the log line
 *
 * Three facts decide it, and each is read, not inferred:
 *
 *  - the device's runtime-PM usage count. genpd does not power a domain off while
 *    one of its devices is in use, so a count above zero means the power-off was
 *    never attempted -- and the driver's own reference is the usual suspect
 *    (trial 25: "power_off ... dev_count=1", the reference the probe holds);
 *  - the domain's stay_on. A genpd keeps a domain powered until its provider has
 *    run sync_state (of_genpd_sync_state(), drivers/pmdomain/core.c), so a set
 *    stay_on means "the provider has not synced" and nothing the driver does will
 *    move the domain;
 *  - PWR_STATUS bit 0, the hardware's own answer, for when genpd and the hardware
 *    disagree.
 *
 * Printing all three is what turns "the clamp was never attempted" into a named
 * cause; the earlier text guessed "provider unsynced" and was wrong twice.
 */
static void md_dump_pm_state(struct ccci_md *md, const char *when)
{
#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
	struct generic_pm_domain *genpd = NULL;
	const char *genpd_name = "none";
	bool stay_on = false;
#endif
	unsigned int sta = 0;

	regmap_read(md->spm_map, SPM_PWR_STATUS, &sta);
#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
	if (md->dev->pm_domain) {
		genpd = container_of(md->dev->pm_domain, struct generic_pm_domain,
				     domain);
		genpd_name = genpd->name;
		stay_on = genpd->stay_on;
	}
#endif
	dev_info(md->dev,
		 "%s: runtime PM %s, usage %d, driver holds %u, genpd %s%s, PWR_STATUS md bit %u\n",
		 when, pm_runtime_active(md->dev) ? "active" : "suspended",
		 atomic_read(&md->dev->power.usage_count), (u32)md->pm_held,
		 genpd_name, stay_on ? " (stay_on: provider not synced)" : "",
		 (u32)(sta & BIT(0)));
}

/**
 * md_power_cycle() - the vendor's restart, for a modem the loader already released
 * @md: the modem
 *
 * The vendor's power-off is the gate first, then the 26 MHz request, then the
 * PMIC (mt6785/md_sys1_platform.c:809-828); its power-on is the same three steps
 * the other way round (:713-745). This performs that, through the two layers that
 * own each step: runtime PM for the gate, this driver for the request and the rails.
 *
 * The gate is the part that can refuse. genpd keeps a domain with stay_on set until
 * its provider's sync_state runs (of_genpd_sync_state(), drivers/pmdomain/core.c:
 * 3684-3702), and while it does, pm_runtime_put_sync() leaves the hardware on. That
 * case is reported and NOT papered over: writing the release into a modem that was
 * never power-cycled is exactly the failure trial 18 measured, so the caller must
 * not continue. Raw MTCMOS writes are not the answer either -- the domain is genpd's.
 *
 * Returns 0 when the modem has really been through the cycle, -EAGAIN when the gate
 * could not be moved, or another negative error.
 */
/*
 * md_epof_check_flag() - what the modem says about the power-off check itself
 * @md: the modem
 *
 * The vendor reads one word out of the MD's SS debug region before it decides how
 * to treat a modem that will not park: `check_power_off_en()` returns non-zero,
 * i.e. "the check is on", when the word is not zero (modem_sys1.c:430-455), and
 * only then does it dump the MD's state and assert; otherwise it calls the weak
 * md1_sleep_timeout_proc() (modem_sys1.c:469-472, :584). The offset is
 * CCCI_EE_OFFSET_EPOF_MD1, "flag to enable MD power off checking or not, in MD SS
 * debug region" (mt6785/ccci_config.h:43).
 *
 * Reading it is what tells a later reader whether this modem's own configuration
 * even expects the handshake we are waiting for.
 */
#define MDSS_DBG_EPOF_CHECK	(7 * 1024 + 0x234)

static void md_epof_check_flag(struct ccci_md *md)
{
	const unsigned int off = NC_MDSS_DBG + MDSS_DBG_EPOF_CHECK;
	unsigned int flag;

	if (!md->nc || off + 4 > NC_MDSS_DBG + NC_MDSS_DBG_SIZE) {
		dev_info(md->dev,
			 "park: the MDSS debug region is too small for the modem's power-off check flag\n");
		return;
	}
	flag = readl(md->nc + off);
	dev_info(md->dev,
		 "park: the modem's own power-off check flag (MDSS debug %#x) is %#x: the check is %s\n",
		 MDSS_DBG_EPOF_CHECK, flag, flag ? "on" : "off");
}

/*
 * md_park() - wait for the modem to let go of its 26 MHz clock, as stock does
 * @md: the modem
 *
 * This is the whole of what the vendor's kernel does before it powers a running
 * modem off, and it is worth being exact about it because it is *not* a message:
 *
 *  - `md_cd_pre_stop()` (modem_sys1.c:474-535) takes a mutex and disables the MD
 *    watchdog interrupt (:496) -- the only AP-side action;
 *  - for flight mode it then calls `debug_in_flight_mode()` (modem_sys1.c:504-590),
 *    which loops up to four times on `spm_is_md1_sleep()` -- `PCM_REG13_DATA`
 *    bit `R13_MD1_SRCCLKENA` clear (base/power/spm/mt6765/mtk_spm_utils.c:36-39,
 *    the same shape on mt6785) -- sleeping one second between tries, and dumps and
 *    asserts on timeout;
 *  - `md_cd_stop()` (modem_sys1.c:1052-1095) then disables the CCIF interrupt and
 *    powers off.
 *
 * Nothing is written into the CCIF SRAM, no CLDMA or DPMAIF message is queued and
 * no register tells the modem to stop: `md_cd_lock_cldma_clock_src()` is an empty
 * function on this SoC (mt6785/md_sys1_platform.c:327-330), the AP to MD system
 * channel's `MD_SLP_REQUEST` is marked deprecated (mt-plat/mtk_ccci_common.h:370),
 * and the flight-mode ioctls only notify userspace through a virtual monitor
 * message (fsm/ccci_fsm_ioctl.c:534-561). A modem parks because its own software
 * decided to -- flight mode reaches it from the framework over a CCCI AT or system
 * port, not from this driver -- and the kernel's part is to wait for that and then
 * to clamp.
 *
 * The bit is the modem's own declaration that it has parked, and it is what the
 * MTCMOS clamp needs: a modem that is issuing transactions cannot acknowledge the
 * bus protection, which is why the clamp of a running modem timed out in the
 * trials (PROTECTEN bit 7 set, PROTECT_STA bit 7 clear) while a halted one
 * acknowledged at once. The vendor proceeds even when the modem never sleeps,
 * because it can wait for the acknowledgement forever; mainline gives the bus
 * protect one second (MTK_POLL_TIMEOUT, mtk-pm-domains.c:38), so this driver must
 * know the answer before it clamps.
 *
 * Returns true when the modem declared sleep within the vendor's five seconds.
 */
static bool md_park(struct ccci_md *md)
{
	unsigned int v;
	int i;

	md_epof_check_flag(md);
	for (i = 0; i < 5; i++) {
		v = readl(md->spm + SPM_PCM_REG13_DATA);
		dev_info(md->dev,
			 "park %d/5: PCM_REG13_DATA %#x, the modem %s its 26 MHz request\n",
			 i + 1, v,
			 (v & SPM_R13_MD_SRCCLKENA_0) ? "still holds" : "dropped");
		if (!(v & SPM_R13_MD_SRCCLKENA_0))
			return true;
		msleep(1000);
	}
	v = readl(md->spm + SPM_PCM_REG13_DATA);
	dev_info(md->dev,
		 "park 6/6: PCM_REG13_DATA %#x after five seconds, the modem %s its 26 MHz request\n",
		 v, (v & SPM_R13_MD_SRCCLKENA_0) ? "still holds" : "dropped");
	return !(v & SPM_R13_MD_SRCCLKENA_0);
}

static int md_power_cycle(struct ccci_md *md)
{
	bool reset_held = false;
	unsigned int prot = 0, sta = 0;
	int ret;

	/*
	 * The clamp comes FIRST, on the modem as it is -- that is the vendor's order
	 * and the only one that can work, because the acknowledgement is not the AP's
	 * to give. MD1_PROT_STEP1_0_MASK is bit 7 of INFRA_TOPAXI_PROTECTEN and its
	 * acknowledgement is read from INFRA_TOPAXI_PROTECTEN_STA1
	 * (clk-mt6785-pg.c:1009-1020, and the mask at :267-268), i.e. the request goes
	 * to the interconnect and the ack says the protected port has gone quiet. A
	 * modem held in reset cannot quieten anything: trial 23 measured the reset
	 * reaching it (PCM_REG13_DATA 0xc400192c -> 0x84000800) while PROTECT_STA bit 7
	 * never came. So the reset is the last resort AFTER a clamp that timed out, not
	 * a preparation for it.
	 */
	md_loader_state(md, "clamp");
	md_ap_snapshot(md, "before clamp");
	if (!md_park(md))
		dev_info(md->dev,
			 "the modem did not park: trying the clamp first, as the vendor does\n");
	else
		md_step(md, "cycle: modem asleep");

	/*
	 * Give up the driver's own reference: that is the request for genpd to power
	 * the domain off, and it is the only reference in the count at this point
	 * (md_pm_get() is idempotent, so no path can hold a second one). A get/put
	 * pair here instead would leave the count at one -- trial 25: power_off at
	 * 17.59 s reported dev_count=1 and did nothing.
	 */
	/*
	 * What the vendor holds at this instant, and nothing more (plan doc §2j): the
	 * session grant is on, the AP's own source request is free, and the CCIF clocks
	 * are off because the vendor has no hif yet at its power-off.
	 */
	md_clks(md, false);
	/*
	 * O1, and this time in the chain: the vendor's power-on with its three protect
	 * clears runs AFTER the park wait and immediately before the power-off request, so
	 * the clear of step1_0 and its set are microseconds apart, as on stock
	 * (md_sys1_platform.c:629 then :636). Trial 38 had it before the park wait, 5.09 s
	 * early, which repeated the previous experiment instead of testing this one.
	 */
	md_vendor_power_on(md);
	dev_info(md->dev,
		 "clamp: AP_MDSRC_REQ %#x (request %u ack %u), CCIF clocks %s (%u on / %u off), PWR_STATUS %#x md bit %u, PCM_REG13_DATA %#x\n",
		 readl(md->spm + SPM_AP_MDSRC_REQ),
		 (u32)!!(readl(md->spm + SPM_AP_MDSRC_REQ) & BIT(0)),
		 (u32)!!(readl(md->spm + SPM_AP_MDSRC_REQ) & SPM_AP_MDSRC_ACK),
		 md->clks_on ? "on" : "off", md->clk_on_count, md->clk_off_count,
		 readl(md->spm + SPM_PWR_STATUS),
		 (u32)!!(readl(md->spm + SPM_PWR_STATUS) & BIT(0)),
		 readl(md->spm + SPM_PCM_REG13_DATA));
	{
		ktime_t t0 = ktime_get();
		u64 t_ns = ktime_get_ns();

		dev_info(md->dev,
			 "clamp: power-off requested at %llu ns (%llu ns after the protect clears)\n",
			 t_ns, t_ns - md->vendor_on_ns);
		md_pm_put(md);
		md->clamp_wait_ms = ktime_ms_delta(ktime_get(), t0);
	}
	prot = md->infra ? readl(md->infra + INFRA_TOPAXI_PROTECTEN) : 0;
	sta = md->infra ? readl(md->infra + INFRA_TOPAXI_PROTECT_STA1) : 0;
	md_ap_snapshot(md, "after clamp");
	dev_info(md->dev,
		 "clamp: waited %u ms, PROTECTEN %#x, PROTECT_STA1 %#x (bit 7 %u), md domain %s\n",
		 md->clamp_wait_ms, prot, sta, (u32)!!(sta & BIT(7)),
		 md_domain_is_on(md) ? "on" : "off");
	if (md_domain_is_on(md)) {

		/*
		 * Two different failures end here, and the registers tell them apart --
		 * guessing "the provider is unsynced" was wrong twice already (trial 19:
		 * stay_on was 0 and the power-off ran into the bus protect; trial 25: the
		 * driver's own reference was still held, so genpd returned without
		 * trying):
		 *
		 *  - PROTECTEN bit 7 clear: genpd never attempted the power-off, and
		 *    md_dump_pm_state() names why (usage count, stay_on, hardware bit);
		 *  - PROTECTEN bit 7 set with PROTECT_STA bit 7 clear: the clamp was
		 *    attempted and not acknowledged inside the driver's second
		 *    (MTK_POLL_TIMEOUT, mtk-pm-domains.c:38).
		 */
		if (!(prot & BIT(7))) {
			dev_err(md->dev,
				"genpd did not power the domain off: the clamp was never attempted (PROTECTEN %#x)\n",
				prot);
			md_dump_pm_state(md, "clamp not attempted");
			goto restore;
		}
		if (!(sta & BIT(7))) {
			/*
			 * Last resort: take the modem down without its cooperation, and give
			 * the interconnect the chance to acknowledge now that the port is
			 * quiet. The reset stays asserted through the rest of the cycle.
			 */
			if (!md->md_reset) {
				dev_err(md->dev,
					"the bus protect was not acknowledged within a second (PROTECTEN %#x, PROTECT_STA %#x) and there is no reset line to quieten the modem\n",
					prot, sta);
				goto restore;
			}
			dev_info(md->dev,
				 "the clamp timed out (PROTECTEN %#x, PROTECT_STA %#x): asserting the MD reset as the last resort\n",
				 prot, sta);
			reset_control_assert(md->md_reset);
			reset_held = true;
			md_reset_state(md, "after assert");
			if (readl_poll_timeout(md->infra + INFRA_TOPAXI_PROTECT_STA1,
					       sta, sta & BIT(7), 100, 200000))
				dev_warn(md->dev,
					 "the bus protect is still not acknowledged after the reset (PROTECT_STA %#x)\n",
					 sta);
			else
				dev_info(md->dev, "the bus protect is acknowledged now (PROTECT_STA %#x)\n", sta);
			/*
			 * Ask genpd to try again: with the acknowledgement in place the
			 * power-off sequence can finish. Take the reference and give it
			 * straight back, which is a real request for the power-off.
			 */
			md_pm_get(md);
			md_pm_put(md);
			md_reset_state(md, "after the second clamp");
			if (md_domain_is_on(md)) {
				dev_err(md->dev,
					"the domain is still on after the clamp and the reset (PROTECTEN %#x, PROTECT_STA %#x)\n",
					readl(md->infra + INFRA_TOPAXI_PROTECTEN), sta);
				goto restore;
			}
		} else {
			dev_err(md->dev,
				"the bus protect was acknowledged but the domain is still on (PROTECTEN %#x, PROTECT_STA %#x)\n",
				prot, sta);
			goto restore;
		}
	}
	md_step(md, "cycle: gate off");
	md_power_seq_release(md);
	md_step(md, "cycle: supplies off");

	/*
	 * Bring the rails back and take the driver's reference again, in that order:
	 * the vendor's power-on is the PMIC, then the gate
	 * (mt6785/md_sys1_platform.c:713-745).
	 */
	ret = md_power_seq_prepare(md);
	if (ret)
		goto restore;
	md_pm_get(md);
	md_clks(md, true);
	md_step(md, "cycle: gate on");
	/*
	 * Release the line only after the domain is back, so the modem starts from the
	 * state a loader-started one would rather than half-started.
	 */
	if (reset_held) {
		reset_control_deassert(md->md_reset);
		md_reset_state(md, "after deassert");
	}
	if (!md_domain_is_on(md)) {
		dev_err(md->dev, "the md domain did not come back on after the cycle\n");
		return -EIO;
	}
	dev_info(md->dev, "power cycle done: the modem is freshly powered and waiting for the release\n");
	return 0;

restore:
	/* the clocks go back on whatever happened, so the caller sees the state it left */
	md_clks(md, true);
	/*
	 * Every failing branch comes here. The domain is left exactly as it was found,
	 * with the driver's one reference held again, or the next attempt starts from
	 * a count that is one too low (the underflow of trial 24) or one too high (the
	 * clamp that never ran in trial 25).
	 */
	if (reset_held) {
		reset_control_deassert(md->md_reset);
		/* on the failure path too, so a reset that will not clear is visible */
		md_reset_state(md, "after deassert (failure path)");
	}
	md_pm_get(md);
	if (!md->pm_held)
		dev_err(md->dev, "the md domain's reference could not be taken back\n");
	return -EAGAIN;
}

static int md_boot(struct ccci_md *md)
{
	int limit = stage_limit();
	unsigned long deadline;
	u32 before = 0, after = 0;
	struct md_query_ap_feature hs1_query;
	bool hs1_pending = false;
	bool modem_in_ee = false;
	int ret;

	if (md->no_boot_info) {
		dev_err(md->dev, "cannot boot the modem: the loader left no CCCI boot information\n");
		return -ENODEV;
	}
	if (limit < 2) {
		dev_info(md->dev, "stop_after %s: booting is not part of this stage\n",
			 stop_after ?: "?");
		return 0;
	}

	ccci_set_state(md, MD_STATE_BOOTING);
	reinit_completion(&md->hs1);
	reinit_completion(&md->hs2);
	md->pending = 0;
	md->ex_channels = 0;
	md->ee_printed = false;

	ret = clear_smem(md);
	md_step(md, "clear_smem");
	if (ret)
		goto err_off;
	ret = ccb_ctrl_init(md);
	md_step(md, "ccb_ctrl_init");
	if (ret)
		goto err_off;
	ret = ccci_ccif_init(md);
	md_step(md, "ccif_init");
	if (ret)
		goto err_off;
	/*
	 * The CCIF clocks come on before anything reads or writes the CCIF
	 * registers. The vendor's ccif_start() does exactly this:
	 * ccif_set_clk_on() at drivers/misc/mediatek/eccci/hif/ccci_hif_ccif.c:2075,
	 * and only then ccci_reset_ccif_hw() at :2087. With the gate shut the
	 * reset never lands and the SRAM is cleared into a dead block: the
	 * trailer the modem reads is missing and the handshake it rings never
	 * arrives.
	 */
	md_clks(md, true);
	if (!md->clks_on)
		goto err_off;
	md_step(md, "ccif_clocks_on");
	/*
	 * Look into the CCIF SRAM BEFORE the reset below wipes it. A modem the loader
	 * already released asks for the runtime data once and then waits; the vendor
	 * never sees that case because it restarts the modem, whose query therefore
	 * arrives after the restart. Here the modem is not restarted, so its query is
	 * still in the SRAM -- and the reset would throw it away and leave the modem
	 * waiting for an answer that can no longer come, which is what trials 18 and 20
	 * measured (boot stage 0x2c04010a, no second release, and the modem awake and
	 * holding its clock request rather than parked).
	 */
	/*
	 * What the modem left behind, before anything is wiped. Both are printed
	 * every boot so a trial never has to guess again:
	 *
	 *  - the SRAM header words: a pending first handshake shows as
	 *    data[1] = MD_INIT_START_BOOT and reserved = MD_INIT_CHK_ID;
	 *  - RCHNUM and BUSY: which channels the modem rang that nobody answered.
	 *    A modem already in its exception handler rings the exception channels
	 *    and then waits, exactly like a modem waiting for its runtime data.
	 */
	{
		struct ccci_header h = { };
		u32 rchnum = readl(md->ccif_ap + APCCIF_RCHNUM);
		u32 busy = readl(md->ccif_ap + APCCIF_BUSY);

		ccci_sram_read(md, SRAM_DL_HEADER, &h, sizeof(h));
		dev_info(md->dev,
			 "CCIF before the reset: RCHNUM %#x BUSY %#x, SRAM header %08x %08x ch %u seq %u assert %u reserved %08x\n",
			 rchnum, busy, h.data[0], h.data[1], h.channel, h.seq_num,
			 h.assert_bit, h.reserved);
		if (md->first_boot)
			hs1_pending = ccci_ccif_pending_hs1(md, &hs1_query);
		if (hs1_pending)
			dev_info(md->dev, "the modem is already asking for the runtime data: answering it instead of restarting it\n");
		else if (rchnum & ~GENMASK(CCIF_CH_SRAM, 0)) {
			/*
			 * The modem rang the exception channels before this driver was
			 * listening, so nobody answered and it is sitting in its own
			 * exception handler -- which is why it keeps its clock request and
			 * never sleeps (trial 21: PCM_REG13_DATA still asking, accept no
			 * clamp). The vendor's stop path handles this the same way: before it
			 * stops the hardware it checks for an exception and runs the EE flow
			 * (fsm_routine_stop() calls ccci_md_pre_exception ->
			 * fsm_check_for_ee(), fsm/ccci_fsm.c:486-522). Answering the stages it
			 * is waiting for lets it hand over and stop, which is what makes the
			 * power-off below possible at all.
			 */
			u32 ex = rchnum & ~GENMASK(CCIF_CH_SRAM, 0);

			dev_info(md->dev, "the modem is in its exception handler (%#x): answering its stages before stopping it\n",
				 ex);
			writel(ex, md->ccif_ap + APCCIF_ACK);
			md->ex_channels |= ex;
			while (ex) {
				unsigned int ch = __ffs(ex);

				if (ch != CCIF_CH_EX_PEER_WAKEUP &&
				    ch != CCIF_CH_EX_SEQ_ERROR)
					ccci_ee_on_channel(md, ch);
				ex &= ex - 1;
			}
			modem_in_ee = true;
		}
	}
	/*
	 * The order here is the vendor's, and it matters.
	 *
	 * On a first boot the loader has already powered the modem and run its boot
	 * ROM, so the domain is ON and the rails are the loader's. The vendor does
	 * not touch the PMIC in that state: md_start_platform() verifies the boot
	 * ROM and then powers the modem off -- gate first, then srcclkena, then the
	 * PMIC (mt6785/md_sys1_platform.c:591-653, md_cd_power_off() :794-828) --
	 * and md_cd_start() powers it on again in the opposite order (PMIC, then
	 * srcclkena, then the gate, :705-770).
	 *
	 * Trial 15 run 2 measured why that matters: with the rails re-applied
	 * while the domain was on, `power_seq_prepare` read back md_bit=0
	 * (STA 0x27dffc, only bit 0 dropped) at 35.951 s -- the rail was disturbed
	 * under a live domain. The same signature appears in the -modem13 trace,
	 * where the first `before get` after power_seq_prepare read bit 0 clear.
	 * Which sub-step inside power_seq_prepare does it is not measured; what is
	 * measured is that the vendor never performs that step with the gate on,
	 * and this driver now does not either.
	 *
	 * The domain is already powered: the probe holds its reference (see
	 * rosemary_ccci_probe()), so genpd has not powered it off since, and the
	 * modem is in the state the loader left it in.
	 *
	 * On the loader's boot this driver now touches NO power at all -- no gate
	 * cycle, no rails, no clock request -- and the reasons are measured, not
	 * assumed:
	 *
	 *  1. The release works from the loader's state. Kernel -modem4 (commit
	 *     aae062530) reached `state ready` and answered AT with no power cycle
	 *     in the first-boot path at all; its first_boot block was the boot-ROM
	 *     check and then the release. Trial 15 run 2, which did cycle, reached
	 *     READY the same way. So the cycle is not what makes the modem start.
	 *
	 *  2. The cycle is what this port cannot do safely. The vendor's restart is
	 *     real: md_start_platform() powers the modem off after the boot-ROM
	 *     check (mt6785/md_sys1_platform.c:591-653, ending in
	 *     md_cd_power_off()), and md_cd_start() powers it on again before
	 *     let_md_go() (modem_sys1.c:409-417 -> md_cd_power_on(), :705-770).
	 *     But its MTCMOS power-off waits for the bus-protection acknowledgement
	 *     with no timeout at all -- three `while (...) ram_console_update();`
	 *     loops (clk-mt6785-pg.c:1010-1042) -- while the mainline power-domain
	 *     driver polls for one second and returns -ETIMEDOUT
	 *     (MTK_POLL_TIMEOUT, drivers/pmdomain/mediatek/mtk-pm-domains.c:38).
	 *
	 *  3. A running modem needs seconds to acknowledge, not milliseconds. Trial
	 *     15 run 2 asked at 31.759 s and got -110 after its one second; by
	 *     35.953 s the same bit read back acknowledged in PROTECT_STA
	 *     (0x961c7080). Trial 17 run 2 asked at the boot and gave up, and run 1
	 *     of the same trial shows a modem that was already halted acknowledging
	 *     at once (both directions ret=0). Nothing in the vendor's mt6785 path
	 *     quiesces the modem first: no sleep request, no watchdog disable, no
	 *     PLL handover -- md_cd_power_off() is gate, srclkena, PMIC
	 *     (:809-828) and nothing else.
	 *
	 *  4. A failed attempt is worse than none. It leaves PROTECTEN bit 7 set
	 *     with PWR_ON still 1 -- a half-protected bus -- and the release write
	 *     then does not land (trial 16: `before 0x0, after 0x0`).
	 *
	 * So the driver releases the modem the loader handed it, and only powers a
	 * modem it brought up itself. That also removes the last dependency on the
	 * power-domain provider being synced: nothing here needs the domain switched.
	 */
	if (modem_in_ee) {
		/*
		 * It answered the stages it was waiting for and handed over, so it is
		 * stopped now rather than running: take it down and bring it back up, and
		 * the fresh modem waits for the release like the loader's would have.
		 */
		ret = md_power_cycle(md);
		if (ret)
			goto err_seq;
	} else if (md->first_boot && !hs1_pending) {
		u32 stage0, stage1;

		ret = md_first_boot_check(md);
		md_step(md, "brom_check");
		if (ret)
			goto err_seq;
		md_boot_stage(md, &stage0, &stage1);
		/*
		 * What the two words say about the state the loader left behind, and
		 * why the driver has to act on it:
		 *
		 * The loader runs the modem's boot ROM once to verify the image. If
		 * it released the modem afterwards, the boot-vector enable it wrote
		 * has already been consumed -- the modem is past the point where the
		 * ROM waits for it, and a second write of the same bit is ignored.
		 * That is what trial 18 measured: stats 0x2c04010a (non-zero, so the
		 * modem had run), then
		 * `MD_BOOT_VECTOR_EN at 0x20000024: before 0x0, after 0x0`, an HS1
		 * timeout with CCIF idle (busy 0, rchnum 0) and no exception.
		 *
		 * Trials 10 and 15 read `0x0 / 0x0` at the same point and their write
		 * did stick: those boots the loader had left the ROM waiting, and the
		 * enable was still ours to give.
		 *
		 * The vendor only ever writes that bit on a modem it has just powered
		 * on: md_start_platform() powers the modem off after the boot-ROM check
		 * (mt6785/md_sys1_platform.c:591-653) and md_cd_start() powers it on
		 * again before md_cd_let_md_go() writes the bit (:777-791). It has no
		 * path that releases a modem the loader already released -- because
		 * from that state the write does nothing.
		 *
		 * Which of the vendor's stage codes these are is decoded only for a few
		 * values, and only where it dumps an exception: 0 means "MD poweron
		 * failed, offender DVFS", 0x5443000C and 0x53310000-0x533100FF mean an
		 * HS1 failure (fsm/mdee_dumper_v3.c:783-808). Anything else, including
		 * this one, the vendor logs raw. What matters here is only zero versus
		 * not: zero is a modem that has not run.
		 */
		if (stage0 || stage1) {
			dev_info(md->dev,
				 "the loader already released the modem (boot stage %#x/%#x): its boot-vector enable is spent, so this boot needs the vendor's power cycle\n",
				 stage0, stage1);
			ret = md_power_cycle(md);
			if (ret)
				goto err_seq;
		}
	} else {
		/*
		 * Not the loader's boot: nothing has powered this modem, so the rails
		 * and the clock request are ours, and the vendor's power_on order
		 * applies (PMIC, srclkena, gate).
		 */
		ret = md_power_seq_prepare(md);
		if (ret)
			goto err_clk;
		/*
		 * The driver's reference is already held, so this only makes sure the
		 * domain is on (md_pm_get() is idempotent); a second reference here is
		 * one the cycle would later have to drop twice.
		 */
		md_pm_get(md);
		if (!md->pm_held)
			goto err_seq;
	}
	if (hs1_pending) {
		/*
		 * The modem is already released and waiting: answer the query it left in
		 * the SRAM, from where it is. No restarts and no power cycle -- the two
		 * things this port cannot do to a running modem (it does not park, so it
		 * cannot acknowledge the bus protection) are exactly what this path
		 * avoids.
		 */
		md_dump_state(md, "pending HS1");
		md->first_boot = false;
		enable_irq(md->irq_ccif);
		enable_irq(md->irq_ccif_exp);
		md_wdt_irq(md, true);
		ccci_send_runtime_data(md, &hs1_query);
		ccci_set_state(md, MD_STATE_HS1);
		dev_info(md->dev, "HS1 answered from the query the loader's modem left behind\n");
		goto hs1_done;
	}
	/*
	 * The CCIF is reset only NOW, after the modem has been brought to a known
	 * power state -- and that is the vendor's order, not a detail.
	 *
	 * On stock the AP never touches the CCIF before it clamps the modem the
	 * loader released: md_start_platform() (mt6785/md_sys1_platform.c:591-653)
	 * polls the boot ROM and powers the modem off, and the CCIF is only brought
	 * up afterwards, in md_cd_start()'s hif start, where ccif_start() turns the
	 * clocks on and resets the block (ccci_hif_ccif.c:2075-2087). This driver
	 * used to reset the CCIF first and clamp second, i.e. it reset the block and
	 * wiped the SRAM header under a modem that was live in its boot ROM and
	 * polling that very SRAM.
	 *
	 * What that costs is visible in trial 24: the SRAM header read all zeros in
	 * the second request while the modem's stage words had not moved, and the
	 * clamp then timed out on step 1 with every other bit acknowledged. A master
	 * left with an outstanding transaction cannot go idle, and nothing the AP can
	 * write will make it -- so the hypothesis this fixes is that we were
	 * creating that transaction ourselves. With the order below a modem that has
	 * never been touched is clamped, and the block is reset on a modem that is
	 * either fresh or dead, in both cases with nothing outstanding.
	 */
	ccci_ccif_reset(md);
	md_step(md, "ccif_reset");
	/*
	 * The data path starts after the block is reset and before the release (the
	 * vendor is in BOOT_WAITING_FOR_HS1 here), and a modem without one never
	 * reaches its second handshake -- so a failure is reported and the boot goes
	 * on.
	 */
	if (ccci_dpmaif_start(md))
		dev_warn(md->dev, "no packet data path\n");
	md_step(md, "dpmaif_start");
	md_dump_state(md, "before release");

	/*
	 * Let the modem go: MD_BOOT_VECTOR_EN, physical 0x20000024
	 * (mt6785/modem_reg_base.h:14), written exactly as the vendor's
	 * md_cd_let_md_go() writes it -- one write of 1, no retry
	 * (mt6785/md_sys1_platform.c:777-791). The value is read before and after,
	 * so a write that does not take is reported with both numbers instead of
	 * only being noticed.
	 */
	/*
	 * A modem that is not powered cannot take the release, and that is worth a
	 * line of its own: trial 14 wrote the vector into a powered-off modem and the
	 * write simply did not stick, which looked like a handshake failure 30 s
	 * later.
	 */
	if (!md_domain_is_on(md)) {
		/*
		 * With the domain off there is nothing to release into: say so and stop
		 * before writing, because a write into an unpowered modem is silently
		 * lost and then looks like a handshake failure 30 s later (trial 14).
		 */
		dev_err(md->dev, "the md domain is not powered: not writing the release\n");
		ret = -EAGAIN;
		goto err_seq;
	}
	md_md_access(md, true);
	before = readl(md->boot_vector);
	writel(1, md->boot_vector);
	after = readl(md->boot_vector);
	md_md_access(md, false);
	dev_info(md->dev, "MD_BOOT_VECTOR_EN at %#x: before %#x, after %#x\n",
		 MD_BOOT_VECTOR_PHYS, before, after);
	if (after != 1)
		dev_warn(md->dev, "MD_BOOT_VECTOR_EN did not stick\n");
	md->first_boot = false;
	enable_irq(md->irq_ccif);
	enable_irq(md->irq_ccif_exp);
	md_wdt_irq(md, true);
	ccci_set_state(md, MD_STATE_HS1);
	dev_info(md->dev, "MD released, waiting for the first handshake\n");

	if (!wait_for_completion_timeout(&md->hs1, HS1_TIMEOUT)) {
		md_dump_state(md, "at HS1 timeout");
		dev_err(md->dev, "HS1 timeout: power status %#x, exception channels %#lx\n",
			readl(md->spm + SPM_PWR_STATUS), md->ex_channels);
		ccci_set_state(md, MD_STATE_EXCEPTION);
		return 0;
	}
hs1_done:
	if (limit < 3) {
		dev_info(md->dev, "stop_after m2: the modem stays at the first handshake\n");
		return 0;
	}
	/* HS2 comes after the modem has read its NVRAM through the file service */
	deadline = jiffies + HS2_TIMEOUT;
	while (!wait_for_completion_timeout(&md->hs2, HZ)) {
		if (md->state == MD_STATE_EXCEPTION)
			return 0;
		if (time_before(deadline, READ_ONCE(md->last_fs) + 10 * HZ))
			deadline = READ_ONCE(md->last_fs) + 10 * HZ;
		if (time_after(jiffies, deadline)) {
			dev_err(md->dev, "HS2 timeout\n");
			ccci_set_state(md, MD_STATE_EXCEPTION);
			return 0;
		}
	}
	return 0;

err_seq:
	md_power_seq_release(md);
err_clk:
	ccci_dpmaif_stop(md);
	md_clks(md, false);
err_off:
	ccci_set_state(md, MD_STATE_OFF);
	return ret;
}

static void boot_work(struct work_struct *work)
{
	struct ccci_md *md = container_of(work, struct ccci_md, boot_work);

	mutex_lock(&md->lock);
	md_boot(md);
	mutex_unlock(&md->lock);
}

void ccci_recv_control(struct ccci_md *md, const struct ccci_header *h,
		       const void *pkt, size_t len)
{
	switch (h->data[1]) {
	case MD_INIT_START_BOOT:
		if (h->reserved == MD_INIT_CHK_ID) {
			if (len < sizeof(*h) + sizeof(struct md_query_ap_feature)) {
				dev_err(md->dev, "HS1: short feature query (%zu)\n",
					len);
				break;
			}
			ccci_send_runtime_data(md, pkt + sizeof(*h));
			ccci_set_state(md, MD_STATE_HS1);
			complete(&md->hs1);
		} else {
			ccci_set_state(md, MD_STATE_READY);
			complete(&md->hs2);
		}
		break;
	case MD_EX:
	case MD_EX_REC_OK:
	case MD_EX_PASS:
		dev_err(md->dev, "MD exception message %#x %#x\n", h->data[1],
			h->reserved);
		ccci_set_state(md, MD_STATE_EXCEPTION);
		break;
	default:
		dev_info(md->dev, "control message %#x %#x\n", h->data[1],
			 h->reserved);
	}
}

/*
 * The modem's own watchdog: it asserts this when its software has died. This is
 * what turns a silent hang into a diagnosable crash, and the reason the
 * interrupt is wired before anyone tries to boot the modem.
 */
static irqreturn_t wdt_isr(int irq, void *data)
{
	struct ccci_md *md = data;

	dev_err(md->dev, "MD watchdog: the modem has asserted\n");
	ccci_set_state(md, MD_STATE_EXCEPTION);
	return IRQ_HANDLED;
}

/* ---- sysfs, probe ------------------------------------------------------ */

/* the SCPSYS syscon, the only mapping M0 needs */
static int md_get_spm(struct ccci_md *md)
{
	md->spm_map = syscon_regmap_lookup_by_phandle(md->dev->of_node,
						     "mediatek,spm");
	if (IS_ERR(md->spm_map))
		return dev_err_probe(md->dev, PTR_ERR(md->spm_map),
				     "the SPM syscon\n");
	return 0;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct ccci_md *md = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", state_names[md->state]);
}
static DEVICE_ATTR_RO(state);

static ssize_t boot_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct ccci_md *md = dev_get_drvdata(dev);
	bool on;
	int ret = kstrtobool(buf, &on);

	if (ret)
		return ret;
	if (on)
		queue_work(system_long_wq, &md->boot_work);
	return count;
}
static DEVICE_ATTR_WO(boot);

static struct attribute *ccci_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_boot.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ccci);

static int md_map_regs(struct ccci_md *md)
{
	struct platform_device *pdev = to_platform_device(md->dev);

	md->ccif_ap = devm_platform_ioremap_resource_byname(pdev, "ap-ccif");
	if (IS_ERR(md->ccif_ap))
		return PTR_ERR(md->ccif_ap);
	md->ccif_md = devm_platform_ioremap_resource_byname(pdev, "md-ccif");
	if (IS_ERR(md->ccif_md))
		return PTR_ERR(md->ccif_md);
	md->infra = devm_ioremap(md->dev, INFRA_PHYS, 0x1000);
	md->spm = devm_ioremap(md->dev, SPM_PHYS, 0x1000);
	md->sec_ao = devm_ioremap(md->dev, SEC_AO_PHYS, 0x1000);
	md->toprgu = devm_ioremap(md->dev, TOPRGU_PHYS, 0x1000);
	md->pll = devm_ioremap(md->dev, MD_PLL_PHYS, 0x1000);
	md->boot_vector = devm_ioremap(md->dev, MD_BOOT_VECTOR_PHYS, 4);
	if (!md->infra || !md->spm || !md->sec_ao || !md->pll || !md->boot_vector)
		return -ENOMEM;
	return 0;
}

static int md_request_irqs(struct ccci_md *md)
{
	struct platform_device *pdev = to_platform_device(md->dev);

	md->irq_ccif = platform_get_irq_byname(pdev, "ccif");
	md->irq_ccif_exp = platform_get_irq_byname(pdev, "ccif-exception");
	md->irq_wdt = platform_get_irq_byname(pdev, "md-wdt");
	if (md->irq_ccif < 0 || md->irq_ccif_exp < 0 || md->irq_wdt < 0)
		return -EINVAL;

	/*
	 * Both CCIF lines are level-low and stay asserted until the handler has
	 * written APCCIF_ACK, so they are requested without auto-enable: the
	 * handler only runs once the share memory exists.
	 */
	return devm_request_threaded_irq(md->dev, md->irq_wdt, NULL, wdt_isr,
					 IRQF_ONESHOT | IRQF_NO_AUTOEN, "md-wdt",
					 md);
}

static int rosemary_ccci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ccci_md *md;
	unsigned int status;
	int ret, i;

	md = devm_kzalloc(dev, sizeof(*md), GFP_KERNEL);
	if (!md)
		return -ENOMEM;
	md->dev = dev;
	md->first_boot = true;
	mutex_init(&md->lock);
	init_completion(&md->hs1);
	init_completion(&md->hs2);
	for (i = 0; i < CCIF_QUEUES; i++)
		spin_lock_init(&md->tx_lock[i]);
	INIT_WORK(&md->rx_work, ccci_rx_work);
	INIT_WORK(&md->boot_work, boot_work);
	platform_set_drvdata(pdev, md);

	/* M0: the domain must be powerable and PWR_STATUS must read 1 */
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	ret = md_get_spm(md);
	if (ret)
		return ret;
	md_domain_trace(md, "probe before get");
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power the md domain\n");
	md->pm_held = true;
	md_domain_trace(md, "probe after get");
	md_loader_state(md, "probe");
	regmap_read(md->spm_map, SPM_PWR_STATUS, &status);
	/*
	 * The reference is HELD from here until the modem is stopped, on purpose.
	 *
	 * Trial 16 measured what happens when it is not: the provider was synced at
	 * 26.87 s, the domain had no user, and genpd powered it off by itself while
	 * the modem was still in the state the loader left it in -- and the MTCMOS
	 * bus-protection acknowledgement never came (PROTECT_STA 0x16181000 against
	 * PROTECTEN bit 7), so scpsys returned -ETIMEDOUT and left the bit set for
	 * every later attempt.
	 *
	 * genpd will not power off a domain whose device is in use
	 * (genpd_power_off() returns when a device is not suspended,
	 * drivers/pmdomain/core.c), so holding the reference is what keeps the modem's
	 * domain alive between probe and the boot -- exactly what the vendor does with
	 * clk_prepare_enable() in md_cd_power_on() and clk_disable_unprepare() in
	 * md_cd_power_off() (mt6785/md_sys1_platform.c:744-745, :809).
	 */
	dev_info(dev, "rosemary_ccci: M0 md power status %u (reference held until stop)\n",
		 (u32)(status & BIT(0)));
	/* BIT() is unsigned long, so the expression is narrowed for the format */
	dev_info(dev, "rosemary_ccci: M0 md power status %u\n",
		 (u32)(status & BIT(0)));

	if (stage_limit() < 1)
		return 0;

	/* M1: read-only. Map the windows and walk the loader's tag list. */
	ret = md_map_regs(md);
	if (ret)
		return ret;
	/*
	 * The reference AP-state snapshot, and it has to be here: md_map_regs() is what maps
	 * the SPM and infracfg windows this reads. Trial 34 had the call queued above the
	 * mapping, where the guard returned early and the diff was empty by construction --
	 * the log's first snapshot said "(first snapshot)" with "was 0x0" after the sync had
	 * already happened.
	 */
	md_ap_snapshot(md, "probe");
	ret = rosemary_ccci_tags_init(md);
	if (ret) {
		/*
		 * The loader left no CCCI boot information: no modem this boot
		 * (charger or recovery boot, or a boot whose loader ran no image).
		 *
		 * Bind anyway, with no channels and no boot work, on purpose. The
		 * power controller is this device's supplier, and the kit builds
		 * with FW_DEVLINK_SYNC_STATE_STRICT, where a supplier whose
		 * consumer never binds keeps sync_state() pending for good:
		 * "mtk-power-controller ...: sync_state() pending due to
		 * 10209000.modem", exactly what the first phone trial printed.
		 * A driver that binds and then refuses to do anything is the
		 * honest answer: the modem is reported as unavailable and the
		 * power controller can finish its own bookkeeping.
		 */
		dev_warn(dev, "no modem this boot (%d): the driver binds, the modem stays off\n",
			 ret);
		md->no_boot_info = true;
		return 0;
	}

	/* the two share-memory windows, mapped uncached like the modem sees them */
	md->nc = devm_memremap(dev, md->nc_phys, md->nc_size, MEMREMAP_WC);
	if (IS_ERR(md->nc))
		return dev_err_probe(dev, PTR_ERR(md->nc), "ap_md_nc_smem\n");
	md->c = devm_memremap(dev, md->c_phys, md->c_size, MEMREMAP_WC);
	if (IS_ERR(md->c))
		return dev_err_probe(dev, PTR_ERR(md->c), "ap_md_c_smem\n");

	md->num_clks = devm_clk_bulk_get_all(dev, &md->clks);
	if (md->num_clks < 0)
		return dev_err_probe(dev, md->num_clks, "the CCIF clocks\n");

	md->vmodem = devm_regulator_get(dev, "vmodem");
	if (IS_ERR(md->vmodem))
		return dev_err_probe(dev, PTR_ERR(md->vmodem), "vmodem\n");
	md->vsram_md = devm_regulator_get(dev, "vsram-md");
	if (IS_ERR(md->vsram_md))
		return dev_err_probe(dev, PTR_ERR(md->vsram_md), "vsram-md\n");

	/*
	 * The modem's reset line, from the SoC's TOPRGU: the only way to take down a
	 * modem that neither answers nor parks. Optional, because a board that does
	 * not describe it loses the recovery and not the driver.
	 */
	md->md_reset = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(md->md_reset))
		return dev_err_probe(dev, PTR_ERR(md->md_reset), "md reset line\n");
	if (!md->md_reset)
		dev_info(dev, "no md reset line in the device tree: a modem that will not park cannot be reset\n");

	ret = md_request_irqs(md);
	if (ret)
		return ret;
	ret = devm_request_irq(dev, md->irq_ccif, ccci_ccif_isr,
			       IRQF_NO_AUTOEN, "ccif", md);
	if (ret)
		return ret;
	ret = devm_request_irq(dev, md->irq_ccif_exp, ccci_ccif_isr,
			       IRQF_NO_AUTOEN, "ccif-exp", md);
	if (ret)
		return ret;

	ret = ccci_ports_init(md);
	if (ret)
		return dev_err_probe(dev, ret, "the modem's channels\n");

	ret = rosemary_ccci_dpmaif_init(md);
	if (ret)
		dev_warn(dev, "no packet data path: %d\n", ret);

	/*
	 * The read-only dumps: /sys/kernel/debug/rosemary_ccci/state and .../regions.
	 * The modem on this port never parks, and the four clamp variants that tried
	 * to explain it produced the same refusal, so what is left is reading the
	 * state the vendor reads when its modem misbehaves (mt6785/ap_md_reg_dump.c).
	 */
	rosemary_ccci_debugfs_add(md);

	dev_info(dev, "probed: stop_after %s, the modem stays %s until userspace boots it\n",
		 stop_after ?: "?", ccci_state_name(md->state));
	return 0;
}

static const struct of_device_id rosemary_ccci_of_match[] = {
	{ .compatible = "xiaomi,rosemary-ccci" },
	{ }
};
MODULE_DEVICE_TABLE(of, rosemary_ccci_of_match);

/**
 * ccci_md_stop() - give the modem's domain, clock request and rails back
 * @md: the modem
 *
 * The mirror of the probe and the boot: the reference the probe took is put back,
 * then the 26 MHz request and the rails go, in the vendor's power-off order with
 * the gate already off (mt6785/md_sys1_platform.c:809-828).
 */
void ccci_md_stop(struct ccci_md *md)
{
	/*
	 * The one put that gives the driver's reference back. The rails and the clock
	 * request follow, in the vendor's order (md_cd_power_off(),
	 * mt6785/md_sys1_platform.c:809-828), which leaves the modem in the state a
	 * cold boot presents to the loader: MTCMOS off, PWR_ON 0, the external buck
	 * isolated, PWR_RST_B low -- the state the loader powers up from itself on
	 * every boot ("MD1 MTCMOS power on done!" in our own LK log). Whether the gate
	 * really moved is printed, because it is the one fact the next boot depends on.
	 */
	/* the vendor's pre-stop, first thing (md_cd_pre_stop(), modem_sys1.c:496) */
	md_wdt_irq(md, false);
	/* and its state at the power-off: no hif clocks, no AP source request (plan §2j) */
	md_clks(md, false);
	dev_info(md->dev,
		 "stop: AP_MDSRC_REQ %#x (request %u ack %u), CCIF clocks %s (%u on / %u off), PCM_REG13_DATA %#x\n",
		 readl(md->spm + SPM_AP_MDSRC_REQ),
		 (u32)!!(readl(md->spm + SPM_AP_MDSRC_REQ) & BIT(0)),
		 (u32)!!(readl(md->spm + SPM_AP_MDSRC_REQ) & SPM_AP_MDSRC_ACK),
		 md->clks_on ? "on" : "off", md->clk_on_count, md->clk_off_count,
		 readl(md->spm + SPM_PCM_REG13_DATA));
	{
		ktime_t t0 = ktime_get();

		md_pm_put(md);
		md->clamp_wait_ms = ktime_ms_delta(ktime_get(), t0);
	}
	dev_info(md->dev,
		 "stop 1/3 gate: the md domain is %s (PWR_STATUS %#x), waited %u ms, PROTECT_STA1 %#x (bit 7 %u)\n",
		 md_domain_is_on(md) ? "STILL ON" : "off",
		 readl(md->spm + SPM_PWR_STATUS), md->clamp_wait_ms,
		 md->infra ? readl(md->infra + INFRA_TOPAXI_PROTECT_STA1) : 0,
		 md->infra ? (u32)!!(readl(md->infra + INFRA_TOPAXI_PROTECT_STA1) & BIT(7)) : 0);
	if (md_domain_is_on(md)) {
		/*
		 * The clamp is the step that can refuse, and the vendor's order has
		 * nothing after it that makes sense without it: cutting the rails under a
		 * live domain is the one thing neither tree ever does. So stop here, say
		 * exactly why, and leave them alone.
		 */
		dev_warn(md->dev,
			 "stop: the clamp was not acknowledged (PROTECTEN %#x, PROTECT_STA %#x), so the 26 MHz request and the rails are left alone -- the loader will find a running modem\n",
			 md->infra ? readl(md->infra + INFRA_TOPAXI_PROTECTEN) : 0,
			 md->infra ? readl(md->infra + INFRA_TOPAXI_PROTECT_STA1) : 0);
	} else {
		md_srcclkena_grant(md, false);
		dev_info(md->dev, "stop 2/3 26 MHz request: released (INFRA_MD_SRCCLKENA %#x)\n",
			 readl(md->infra + INFRA_MD_SRCCLKENA));
		md_rails_cut(md);
		dev_info(md->dev, "stop 3/3 supplies: vmodem and vsram_md cut, so the loader starts a fresh modem on the next boot\n");
	}
	ccci_dpmaif_stop(md);
	md_clks(md, false);
	ccci_set_state(md, MD_STATE_OFF);
}

/**
 * rosemary_ccci_shutdown() - power the modem off before the machine restarts
 * @pdev: the modem's platform device
 *
 * kexec runs device_shutdown(), not .remove, so without this hook a warm reboot
 * carries a running modem into the next kernel: its MTCMOS domain, its clocks and
 * its 26 MHz request all survive, and the new kernel finds a modem that is neither
 * in boot-ROM wait nor asleep -- the state trials 18 to 21 kept failing on. A real
 * SoC reset would clear it (mainline's mtk_wdt_restart() issues WDT_SWRST, and the
 * MT6785 TOPRGU has an MD reset bit at all, MT6785_TOPRGU_MD_RST = 7,
 * include/dt-bindings/reset/mediatek,mt6785-resets.h:22), but kexec does not reset
 * the SoC.
 *
 * The vendor never needed this hook -- its .shutdown() is an empty function
 * (mt6785/md_sys1_platform.c) because a phone reboot is a real reset there -- so
 * this is the one place where a kexec-based port has to do more than the vendor.
 *
 * The stop itself is the same sequence the driver already uses: the reference is
 * put back, the clock request and the rails go, and everything happens after the
 * EE handover if the modem was in exception (see md_boot()).
 */
static void rosemary_ccci_shutdown(struct platform_device *pdev)
{
	struct ccci_md *md = platform_get_drvdata(pdev);

	if (!md)
		return;
	dev_info(md->dev, "shutdown: powering the modem off for the next kernel\n");
	ccci_md_stop(md);
}

static void rosemary_ccci_remove(struct platform_device *pdev)
{
	struct ccci_md *md = platform_get_drvdata(pdev);

	if (!md)
		return;
	/*
	 * Everything the probe registered has to go back, in the reverse order: the
	 * modem's power, then the three things that hold kernel objects --
	 * the WWAN data-path ops, the channel devices and the class itself.
	 * Trial 17 measured what a half-undone remove costs: the class stayed
	 * registered, and the next probe failed with
	 * "sysfs: cannot create duplicate filename '/class/ccci'" (-EEXIST), so the
	 * driver could not be reloaded after a single rmmod.
	 */
	rosemary_ccci_debugfs_remove(md);
	ccci_md_stop(md);
	rosemary_ccci_dpmaif_exit(md);
	ccci_ports_exit(md);
}

static struct platform_driver rosemary_ccci_driver = {
	.probe = rosemary_ccci_probe,
	.remove = rosemary_ccci_remove,
	.shutdown = rosemary_ccci_shutdown,
	.driver = {
		.name = "rosemary-ccci",
		.dev_groups = ccci_groups,
		.of_match_table = rosemary_ccci_of_match,
	},
};
module_platform_driver(rosemary_ccci_driver);

MODULE_DESCRIPTION("MediaTek MT6785 modem (CCCI over CCIF), Poco M5s");
MODULE_LICENSE("GPL");
