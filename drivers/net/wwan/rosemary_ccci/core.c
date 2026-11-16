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
static int md_power_seq_prepare(struct ccci_md *md)
{
	int ret;

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
	ap_mdsrc_req(md, true);
	/* the modem's 26 MHz source clock request: low byte 0x21 */
	writel((readl(md->infra + INFRA_MD_SRCCLKENA) & ~0xff) | 0x21,
	       md->infra + INFRA_MD_SRCCLKENA);
	/* SPM register control unlock, then srclken_o1 for the 32 kHz request */
	writel(SPM_PROJECT_KEY | 1, md->spm + SPM_POWERON_CONFIG_EN);
	writel(readl(md->spm + SPM_CLK_CON) | BIT(21), md->spm + SPM_CLK_CON);
	return 0;
}

static void md_power_seq_release(struct ccci_md *md)
{
	writel(readl(md->infra + INFRA_MD_SRCCLKENA) & ~0xff,
	       md->infra + INFRA_MD_SRCCLKENA);
	ap_mdsrc_req(md, false);
	regulator_disable(md->vmodem);
	regulator_disable(md->vsram_md);
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

/**
 * md_domain_on() - power the modem's MTCMOS domain and prove it is on
 * @md: the modem
 *
 * Taking the runtime-PM reference is not the same as having a powered domain.
 * Trial 8a booted the modem at 11 s, early in the boot, and the domain read
 * OFF with runtime PM reporting "active": genpd's cached status said on, its
 * power_on() returned early on that belief ("if (genpd_status_on(genpd))
 * return 0", drivers/pmdomain/core.c:1043), and the hardware was never asked.
 * A reference alone therefore proves nothing, so this asks twice: take the
 * reference, then read PWR_STATUS bit 0 -- the md domain's own bit -- and if it
 * is clear, force the domain down and up again through genpd itself, and read
 * again. If it still reads clear the boot says so with the value instead of
 * failing 30 s later at the handshake.
 */
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
	dev_dbg(md->dev, "step %-22s: md_bit=%u STA=%#x CON=%#x\n",
		step, (u32)(sta & BIT(0)), sta, con);
}

static int md_domain_on(struct ccci_md *md)
{
	unsigned int status = 0;
	int ret;

	md_domain_trace(md, "before get");
	ret = pm_runtime_resume_and_get(md->dev);
	if (ret) {
		md_domain_trace(md, "get failed");
		return ret;
	}
	md_domain_trace(md, "after get");

	regmap_read(md->spm_map, SPM_PWR_STATUS, &status);
	if (status & BIT(0))
		return 0;

	dev_warn(md->dev,
		 "the md domain is off while runtime PM reports it on (%#x): powering it again\n",
		 status);
	/*
	 * Drop the reference so genpd really suspends the domain, then take it
	 * again. pm_runtime_put_sync() waits for the suspend, so the second
	 * resume cannot be answered by the same stale belief that caused this.
	 */
	pm_runtime_put_sync(md->dev);
	usleep_range(1000, 2000);
	ret = pm_runtime_resume_and_get(md->dev);
	if (ret)
		return ret;

	status = 0;
	regmap_read(md->spm_map, SPM_PWR_STATUS, &status);
	if (!(status & BIT(0))) {
		/*
		 * genpd reported success and the hardware says otherwise. Record what
		 * the hardware says after a short wait too: if the bit appears on its
		 * own the domain was still settling, and if it does not the power-on
		 * genuinely did not take. Both are facts a trial can use; neither is
		 * assumed here.
		 */
		msleep(20);
		md_domain_trace(md, "20ms later");
		pm_runtime_put_sync(md->dev);
		md_domain_trace(md, "after put");
		return dev_err_probe(md->dev, -EIO,
				     "the md domain will not power on (PWR_STATUS %#x)\n",
				     status);
	}
	return 0;
}

static int md_boot(struct ccci_md *md)
{
	int limit = stage_limit();
	unsigned long deadline;
	u32 before = 0, after = 0;
	u32 gate_off_bit = 0;
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
	ret = clk_bulk_prepare_enable(md->num_clks, md->clks);
	if (ret)
		goto err_off;
	md_step(md, "ccif_clocks_on");
	ccci_ccif_reset(md);
	md_step(md, "ccif_reset");
	/*
	 * The data path starts before the modem (the vendor is in
	 * BOOT_WAITING_FOR_HS1 here), and a modem without one never reaches its
	 * second handshake -- so a failure is reported and the boot goes on.
	 */
	if (ccci_dpmaif_start(md))
		dev_warn(md->dev, "no packet data path\n");
	md_step(md, "dpmaif_start");
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
	 */
	if (md->first_boot) {
		ret = md_domain_on(md);
		if (ret) {
			dev_err(md->dev, "cannot power the md domain: %d\n", ret);
			goto err_clk;
		}
	} else {
		ret = md_power_seq_prepare(md);
		if (ret)
			goto err_clk;
		ret = md_domain_on(md);
		if (ret) {
			dev_err(md->dev, "cannot power the md domain: %d\n", ret);
			goto err_seq;
		}
	}

	if (md->first_boot) {
		ret = md_first_boot_check(md);
		md_step(md, "brom_check");
		if (ret)
			goto err_pm;
		/*
		 * The loader ran the boot ROM once; the modem is started again from
		 * here, and the vendor restarts it by powering it down and up again,
		 * in this order:
		 *
		 *   md_start_platform() on first boot waits for the boot ROM, dumps
		 *   the boot status, and then powers the modem off:
		 *   mt6785/md_sys1_platform.c:591-653, ending in md_cd_power_off().
		 *   md_cd_power_off() disables the MTCMOS gate clock first
		 *   (clk_disable_unprepare(clk_table[0]), i.e. "scp-sys-md1-main",
		 *   :809), then clears INFRA_AO_MD_SRCCLKENA (:812-818), then the
		 *   PMIC (:821-828).
		 *   md_cd_start() then calls md_cd_power_on() before let_md_go()
		 *   (modem_sys1.c:409-417); md_cd_power_on() is the same three steps
		 *   the other way round: PMIC (:713), srclkena = 0x21 (:727-733),
		 *   MTCMOS gate (:744-745).
		 *
		 * Here the gate is the md power domain's, and only runtime PM may
		 * touch it: pm_runtime_put_sync() is the vendor's
		 * clk_disable_unprepare(), pm_runtime_resume_and_get() is its
		 * clk_prepare_enable(). The supplies and the 26 MHz request are ours
		 * to drive, and they follow the gate exactly as the vendor orders it.
		 *
		 * Whether the gate really moves is not assumed: the two checks below
		 * report it. genpd keeps domains with stay_on set (until its provider
		 * syncs), and when that happens the gate does not move at all -- which
		 * is a fact the trial has to show, not something to paper over with a
		 * raw write to 0x318.
		 */
		pm_runtime_put_sync(md->dev);
		gate_off_bit = md_domain_is_on(md) ? 1 : 0;
		md_step(md, "restart: gate off");
		md_power_seq_release(md);
		md_step(md, "restart: supplies off");
		ret = md_power_seq_prepare(md);
		if (ret)
			goto err_clk;
		ret = pm_runtime_resume_and_get(md->dev);
		md_step(md, "restart: gate on");
		if (ret)
			goto err_seq;
		/*
		 * One line, because this is the one thing a trial has to see at a
		 * glance: the vendor restarts the modem by taking the domain down and
		 * bringing it back, and genpd can refuse to move it. `gate off 0` and
		 * `gate on 1` means the restart was real.
		 */
		dev_info(md->dev, "restart before the release: gate off md_bit=%u, gate on md_bit=%u\n",
			 gate_off_bit, (u32)md_domain_is_on(md));
		if (!md_domain_is_on(md))
			dev_warn(md->dev,
				 "the md domain did not come back on; the vendor's restart could not run here\n");
	}
	md_dump_state(md, "before release");

	/*
	 * Let the modem go: MD_BOOT_VECTOR_EN, physical 0x20000024
	 * (mt6785/modem_reg_base.h:14), written exactly as the vendor's
	 * md_cd_let_md_go() writes it -- one write of 1, no retry
	 * (mt6785/md_sys1_platform.c:777-791). The value is read before and after,
	 * so a write that does not take is reported with both numbers instead of
	 * only being noticed.
	 */
	before = readl(md->boot_vector);
	writel(1, md->boot_vector);
	after = readl(md->boot_vector);
	dev_info(md->dev, "MD_BOOT_VECTOR_EN at %#x: before %#x, after %#x\n",
		 MD_BOOT_VECTOR_PHYS, before, after);
	if (after != 1)
		dev_warn(md->dev, "MD_BOOT_VECTOR_EN did not stick\n");
	md->first_boot = false;
	enable_irq(md->irq_ccif);
	enable_irq(md->irq_ccif_exp);
	enable_irq(md->irq_wdt);
	ccci_set_state(md, MD_STATE_HS1);
	dev_info(md->dev, "MD released, waiting for the first handshake\n");

	if (!wait_for_completion_timeout(&md->hs1, HS1_TIMEOUT)) {
		md_dump_state(md, "at HS1 timeout");
		dev_err(md->dev, "HS1 timeout: power status %#x, exception channels %#lx\n",
			readl(md->spm + SPM_PWR_STATUS), md->ex_channels);
		ccci_set_state(md, MD_STATE_EXCEPTION);
		return 0;
	}
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

err_pm:
	/*
	 * A boot that already took the domain reference must give it back before
	 * the supplies go away, or the domain stays powered with nothing using it.
	 */
	pm_runtime_put_sync(md->dev);
err_seq:
	md_power_seq_release(md);
err_clk:
	ccci_dpmaif_stop(md);
	clk_bulk_disable_unprepare(md->num_clks, md->clks);
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
	md_domain_trace(md, "probe after get");
	regmap_read(md->spm_map, SPM_PWR_STATUS, &status);
	pm_runtime_put_sync(dev);
	md_domain_trace(md, "probe after put");
	/* BIT() is unsigned long, so the expression is narrowed for the format */
	dev_info(dev, "rosemary_ccci: M0 md power status %u\n",
		 (u32)(status & BIT(0)));

	if (stage_limit() < 1)
		return 0;

	/* M1: read-only. Map the windows and walk the loader's tag list. */
	ret = md_map_regs(md);
	if (ret)
		return ret;
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

	dev_info(dev, "probed: stop_after %s, the modem stays %s until userspace boots it\n",
		 stop_after ?: "?", state_names[md->state]);
	return 0;
}

static const struct of_device_id rosemary_ccci_of_match[] = {
	{ .compatible = "xiaomi,rosemary-ccci" },
	{ }
};
MODULE_DEVICE_TABLE(of, rosemary_ccci_of_match);

static struct platform_driver rosemary_ccci_driver = {
	.probe = rosemary_ccci_probe,
	.driver = {
		.name = "rosemary-ccci",
		.dev_groups = ccci_groups,
		.of_match_table = rosemary_ccci_of_match,
	},
};
module_platform_driver(rosemary_ccci_driver);

MODULE_DESCRIPTION("MediaTek MT6785 modem (CCCI over CCIF), Poco M5s");
MODULE_LICENSE("GPL");
