// SPDX-License-Identifier: GPL-2.0
/*
 * rosemary_ccci: read-only debugfs dumps of the modem's state
 *
 * Why this exists: the MD on this port never parks (its 26 MHz and 1.8 V requests
 * stay asserted, PCM_REG13_DATA 0xc400192c, for as long as the domain is on), so
 * the bus protection of its MTCMOS power-off never acknowledges and the clamp
 * cannot complete. Four trials of clamp variants produced the same refusal, so the
 * next step is not another variant but reading what the modem is actually doing --
 * which is exactly what the vendor does when its modem misbehaves: it dumps these
 * register blocks and reads them (mt6785/ap_md_reg_dump.c, the EE / dump path) and
 * the boot-status and CCIF words (mt6785/md_sys1_platform.c).
 *
 * Everything here is a read. No register is ever written, no power state is
 * touched, and the file is only reachable through debugfs on a kernel that has it
 * mounted.
 *
 * Two files:
 *
 *   state    -- the AP-side scalars that decide the questions the trials keep
 *               asking: is the domain on, does the modem hold its clock request,
 *               was the clamp attempted and what did it acknowledge, what did the
 *               loader leave, and what is in the CCIF. Every address here is one
 *               the vendor reads while the modem runs.
 *   md_regs  -- the modem's own peripheral blocks (TOPSM, RGU, OST, CSC, ECT, bus,
 *               PLL, USIP at 0x0d0d_xxxx and 0x0d0c_xxxx). ONE block per run,
 *               selected through the md_block parameter, and ONLY in the state in
 *               which the vendor reads them. Read the rest of this comment before
 *               touching it.
 *
 * Why md_regs is fenced in: trial 31 read those blocks in the running state and
 * hung the AP -- no panic, `androidboot.bootreason=SSPM_reboot`, i.e. the AP's
 * transaction never completed and the SSPM watchdog reset the chip. The vendor
 * knew: md_cd_dump_debug_register() opens with the comment "MD no need dump
 * because of bus hang happened - open for debug" (mt6785/md_sys1_platform.c:467)
 * and only reaches the MD-side blocks when the modem's own boot stage says it
 * failed to come up (`:476-483`: only stage 0, 0x5443000C, or 0x53310000-0x5331_00FF),
 * never while it is running normally. It also takes the AP's MD source request
 * around the access (`md_cd_lock_modem_clock_src(1)`, `:492-496`, which is
 * spm_ap_mdsrc_req(), :332-360), so the modem's bus is clocked while the AP reads.
 *
 * Both of those are reproduced here: the same stage gate, and the request held
 * across the read. Blocks the vendor cannot read without writing first, or warns
 * hang the bus, are not offered at all: the PC monitor (it stops and restarts the
 * monitor, ap_md_reg_dump.c:59-85), the bus-history selector (eight pre-action
 * writes, :192-294) and ELM ("might cause bus hang so enable it only when needed",
 * :373-378).
 */

#include <linux/bitops.h>
#include <linux/byteorder/generic.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/pm_runtime.h>
#include <linux/iopoll.h>
#include <linux/seq_file.h>

#include "rosemary_ccci.h"

/*
 * Which of the modem's own blocks `md_regs` will read. Off by default, and it
 * cannot be read at all in a state the vendor would not read either -- see the
 * header comment.
 */
static char *md_block = "";
module_param(md_block, charp, 0644);
MODULE_PARM_DESC(md_block, "the modem block md_regs reads: topsm, rgu, ost, csc, ect, bus, pll or usip");

/* one line per scalar: name, the register, its vendor citation, and its meaning */
struct md_scalar {
	const char *name;
	const char *where;
	unsigned int off;
	unsigned int mask;
	const char *what;
};

static void md_scalar_dump(struct seq_file *s, const char *group, void __iomem *base,
			   size_t size, const struct md_scalar *tab, size_t n)
{
	size_t i;

	if (!base) {
		seq_printf(s, "%s: not mapped\n", group);
		return;
	}
	seq_printf(s, "[%s]\n", group);
	for (i = 0; i < n; i++) {
		const struct md_scalar *r = &tab[i];
		unsigned int v;

		if (r->off + 4 > size) {
			seq_printf(s, "  %-28s out of the mapped window\n", r->name);
			continue;
		}
		v = readl(base + r->off);
		seq_printf(s, "  %-28s %#010x", r->name, v);
		if (r->mask)
			seq_printf(s, "  bit%s %u", r->mask & (r->mask - 1) ? "s" : "",
				   (unsigned int)!!(v & r->mask));
		seq_printf(s, "\n      %s\n      %s\n", r->where, r->what);
	}
}

/*
 * The AP-side blocks. Citations are the vendor's own headers and code:
 *  - SPM (0x10006000): base/power/spm/mt6785/mtk_spm_reg.h
 *  - PCM R13 fields: base/power/spm/mt6785/pcm_def.h:146-179
 *  - INFRA (0x10001000): mt6785/md_sys1_platform.c (:719-726 for srcclkena) and
 *    drivers/pmdomain/mediatek/mt6785-pm-domains.h:14-21 for the protect windows
 */
static const struct md_scalar spatterns[] = {
	{ "PCM_REG13_DATA", "spm/mt6785/mtk_spm_reg.h, PCM_REG13_DATA 0x110",
	  SPM_PCM_REG13_DATA, 0,
	  "the SPM's view of every subsystem's clock request; bit 2 = MD1 26 MHz request, bit 30 = MD1 1.8 V request (pcm_def.h:148, :177)" },
	{ "PWR_STATUS", "spm/mt6785/mtk_spm_reg.h, PWR_STATUS 0x160",
	  SPM_PWR_STATUS, BIT(0),
	  "bit 0 is the MD1 domain's own power status (clk-mt6785-pg.c:488 MD1_PWR_STA_MASK, mt6785-pm-domains.h:116 sta_mask)" },
	{ "MD1_PWR_CON", "spm/mt6785/mtk_spm_reg.h, MD1_PWR_CON 0x318",
	  SPM_MD1_PWR_CON, 0,
	  "PWR_ON and PWR_RST_B: what the vendor's spm_mtcmos_ctrl_md1() leaves on a powered domain (clk-mt6785-pg.c:1079-1081)" },
	{ "MD_EXT_BUCK_ISO_CON", "spm/mt6785/mtk_spm_reg.h, 0x3b0",
	  SPM_MD_EXT_BUCK_ISO_CON, 0,
	  "bits 0 and 1 isolate the MD's external buck; the vendor sets them only after the domain is off (clk-mt6785-pg.c:1055-1060)" },
	{ "AP_MDSRC_REQ", "spm/mt6785/mtk_spm_reg.h, 0x434 (:1847-1852)",
	  SPM_AP_MDSRC_REQ, SPM_AP_MDSRC_ACK,
	  "bit 0 = the AP's request for the MD source clock, bit 4 = the SPM's acknowledgement; the vendor waits for bit 4 (mt6765/mtk_spm_utils.c:46-72)" },
};

static const struct md_scalar infra[] = {
	{ "INFRA_MD_SRCCLKENA", "mt6785/md_sys1_platform.c:719-726 (kernel), LK writes 0x1006321",
	  INFRA_MD_SRCCLKENA, 0x21,
	  "the AP side of the MD 26 MHz request; low byte 0x21 means requested, the vendor clears it after the clamp (:813-816)" },
	{ "TOPAXI_PROTECTEN", "mt6785-pm-domains.h:16-17, clk-mt6785-pg.c:1009",
	  INFRA_TOPAXI_PROTECTEN, BIT(7),
	  "bit 7 = MD1_PROT_STEP1_0: set means the power-off asked the MD's port to go quiet" },
	{ "TOPAXI_PROTECTEN_STA1", "mt6785-pm-domains.h:16, clk-mt6785-pg.c:1011-1017",
	  INFRA_TOPAXI_PROTECT_STA1, BIT(7),
	  "bit 7 = MD1_PROT_STEP1_0_ACK: set means it acknowledged; clear with the request set is the refusal every trial measured" },
	{ "TOPAXI_PROTECTEN_1", "mt6785-pm-domains.h:20, clk-mt6785-pg.c:1033",
	  INFRA_TOPAXI_PROTECTEN_1, BIT(6),
	  "bit 6 = MD1_PROT_STEP2_1" },
	{ "TOPAXI_PROTECT_STA1_1", "mt6785-pm-domains.h:20, clk-mt6785-pg.c:1035-1041",
	  INFRA_TOPAXI_PROTECT_STA1_1, BIT(6),
	  "bit 6 = MD1_PROT_STEP2_1_ACK" },
};

/*
 * The MD-side scalars, reached through the MD register window the vendor maps in
 * md_cd_io_remap_md_side_register() (mt6785/md_sys1_platform.c:299-322): the boot
 * vector, the boot-stats selector and the stats word itself.
 */
static int md_state_show(struct seq_file *s, void *unused)
{
	struct ccci_md *md = s->private;
	unsigned int v, rchnum, busy, ack;

	seq_printf(s, "driver state: %s, first_boot %u, runtime PM %s, holds %u\n",
		   ccci_state_name(md->state),
		   (unsigned int)md->first_boot,
		   pm_runtime_active(md->dev) ? "active" : "suspended",
		   (unsigned int)md->pm_held);

	md_scalar_dump(s, "SPM (0x10006000)", md->spm, 0x1000, spatterns,
		       ARRAY_SIZE(spatterns));
	md_scalar_dump(s, "INFRA (0x10001000)", md->infra, 0x1000, infra,
		       ARRAY_SIZE(infra));

	seq_puts(s, "[MD side]\n");
	if (md->boot_vector) {
		v = readl(md->boot_vector);
		seq_printf(s, "  %-28s %#010x\n      %s\n      %s\n", "MD_BOOT_VECTOR_EN", v,
			   "mt6785/modem_reg_base.h, written by the loader and by md_cd_let_md_go() (md_sys1_platform.c:777-791)",
			   "1 = the release is offered; the modem clears it when it takes it, so 0 after a write means it consumed it");
	}
	if (md->sec_ao) {
		seq_printf(s, "  %-28s %#x %#x %#x %#x\n      %s\n      %s\n",
			   "security_ao BROM words", readl(md->sec_ao + SEC_AO_BROM0),
			   readl(md->sec_ao + SEC_AO_BROM1), readl(md->sec_ao + SEC_AO_BROM2),
			   readl(md->sec_ao + SEC_AO_BROM3),
			   "mt6785/md_sys1_platform.c:615-623 polls 0x824, 0x828, 0x82C, 0x830 for 1 each",
			   "all 1 = the boot ROM handed off (latched); it does NOT mean the modem is still in the ROM");
	}
	if (md->pll) {
		writel(0, md->pll + MD1_BOOT_STATS_SELECT);
		seq_printf(s, "  %-28s %#010x\n", "MD1 boot stats [0]",
			   readl(md->pll + MD1_CFG_BOOT_STATS));
		writel(1, md->pll + MD1_BOOT_STATS_SELECT);
		seq_printf(s, "  %-28s %#010x\n      %s\n      %s\n", "MD1 boot stats [1]",
			   readl(md->pll + MD1_CFG_BOOT_STATS),
			   "md_cd_get_md_bootup_status(), mt6785/md_sys1_platform.c:312-314",
			   "the stage code the modem's own code leaves; non-zero = it has run, zeros = it has not");
	}

	seq_puts(s, "[CCIF (0x10209000)]\n");
	if (md->ccif_ap) {
		busy = readl(md->ccif_ap + APCCIF_BUSY);
		rchnum = readl(md->ccif_ap + APCCIF_RCHNUM);
		ack = readl(md->ccif_ap + APCCIF_ACK);
		seq_printf(s, "  %-28s %#010x %s\n", "APCCIF_BUSY", busy,
			   "channels the AP has not taken yet");
		seq_printf(s, "  %-28s %#010x %s\n", "APCCIF_RCHNUM", rchnum,
			   "channels the modem rang; bits 16+ are the exception channels (ccif_hif_platform.h:32-38)");
		seq_printf(s, "  %-28s %#010x %s\n", "APCCIF_ACK", ack,
			   "what the AP has acknowledged; a channel left set here is one the modem is still waiting on");
		seq_printf(s, "  %-28s %lu\n", "exception channels held", (unsigned long)hweight_long(md->ex_channels));
	} else {
		seq_puts(s, "  CCIF not mapped\n");
	}

	return 0;
}

/*
 * The modem's own blocks, one per run, gated exactly as the vendor gates them.
 *
 * Selection through the writable module parameter `md_block` (below) instead of a
 * parameter to the file: a debugfs file has no arguments, and the selector is what
 * a reader sets *before* the read, so if the read hangs the phone's last known
 * intent is in the log and in `state` -- which is how trial 31's hang would have
 * been named in one line instead of guessed.
 */
struct md_region {
	const char *name;
	const char *where;
	phys_addr_t base;
	size_t len;
	const char *what;
};

static const struct md_region md_regions[] = {
	{ "topsm", "ap_md_reg_dump.c:325-336 (0x0d0d0000, 0x8e4)", 0x0d0d0000, 0x8e4,
	  "the modem's top power state machine: its own sleep requests and masks" },
	{ "rgu", "ap_md_reg_dump.c:338-352 (0x0d0d2100 + 0x200)", 0x0d0d2100, 0x25c,
	  "the modem's reset generation unit: what reset it last, and what is held" },
	{ "ost", "ap_md_reg_dump.c:354-368 (0x0d0d1000 + 0x200)", 0x0d0d1000, 0x208,
	  "the modem's own timer: a running counter means its software is alive and clocked" },
	{ "csc", "ap_md_reg_dump.c:370-383 (0x0d0d3000)", 0x0d0d3000, 0x214,
	  "modem core status capture" },
	{ "ect", "ap_md_reg_dump.c:302-322 (0x0d0cc130)", 0x0d0cc130, 0x1ee8,
	  "the modem's event/exception trace block" },
	{ "bus", "ap_md_reg_dump.c:154-170 (0x0d0c7000 head)", 0x0d0c7000, 0x19098,
	  "the modem's internal bus status: a stuck transaction shows up here" },
	{ "pll", "ap_md_reg_dump.c:85-153 (0x0d0c3800 head)", 0x0d0c3800, 0x1c85c,
	  "the modem's clock control: which PLL it runs from" },
	{ "usip", "ap_md_reg_dump.c:384-420 (0x0d0c4400 head)", 0x0d0c4400, 0x3500,
	  "the modem's serial interface block" },
};

#define REGION_WINDOW	0x80

/*
 * The vendor's gate, verbatim in behaviour: only a modem whose own boot stage says
 * it failed to come up is dumped (md_sys1_platform.c:476-483). A running modem --
 * ours, stage 0x2c04010a -- is never read, because that is the state trial 31 hung
 * the phone in.
 */
static bool md_regs_may_read(struct ccci_md *md, const char **why)
{
	u32 s0 = 0, s1 = 0;

	if (md->state != MD_STATE_EXCEPTION) {
		*why = "the modem is not in exception; the vendor dumps these blocks only for a modem that failed to come up (md_sys1_platform.c:476-483)";
		return false;
	}
	if (md->pll)
		ccci_md_boot_stage(md, &s0, &s1);
	if (!(s0 == 0 || s0 == 0x5443000c || (s0 >= 0x53310000 && s0 <= 0x533100ff))) {
		*why = "the boot stage is not one the vendor dumps on (only 0, 0x5443000C or 0x53310000-0x533100FF)";
		return false;
	}
	*why = NULL;
	return true;
}

static int md_regs_show(struct seq_file *s, void *unused)
{
	struct ccci_md *md = s->private;
	const struct md_region *r = NULL;
	const char *why = NULL;
	void __iomem *p;
	size_t len, off;

	/*
	 * sysfs_streq(), not strcmp(): `echo topsm > md_block` writes "topsm\n", and a
	 * plain compare then reports "no such block" while the value in sysfs reads
	 * exactly right -- trial 32 hit that.
	 */
	for (size_t i = 0; i < ARRAY_SIZE(md_regions); i++)
		if (md_block && sysfs_streq(md_regions[i].name, md_block))
			r = &md_regions[i];

	seq_printf(s, "selected block: %s\n",
		   md_block && *md_block ? strim(md_block) : "(none)");
	seq_printf(s, "driver state: %s, boot stats available: %u\n",
		   ccci_state_name(md->state), (unsigned int)!!md->pll);
	if (!r) {
		seq_puts(s, "no such block; choose one of: topsm rgu ost csc ect bus pll usip\n");
		return 0;
	}
	seq_printf(s, "block %s, base %#llx, window %#zx of %#zx\n  %s\n  %s\n",
		   r->name, (unsigned long long)r->base,
		   min_t(size_t, r->len, REGION_WINDOW), r->len, r->where, r->what);
	if (!md_regs_may_read(md, &why)) {
		seq_printf(s, "refusing to read: %s\n", why);
		seq_puts(s, "trial 31 read these blocks while the modem was running and the AP's\n"
			    "transaction never completed (bootreason SSPM_reboot).\n");
		return 0;
	}

	/*
	 * The vendor holds the AP's MD source request across the access
	 * (md_cd_lock_modem_clock_src(1), md_sys1_platform.c:492-496, which is
	 * spm_ap_mdsrc_req()). Do the same, and say so before the read, because a
	 * hang here leaves no output.
	 */
	dev_info(md->dev, "md_regs: reading %s at %#llx (window %#zx), holding the AP MD source request\n",
		 r->name, (unsigned long long)r->base,
		 min_t(size_t, r->len, REGION_WINDOW));
	ccci_md_src_req(md, true);
	len = min_t(size_t, r->len, REGION_WINDOW);
	p = ioremap(r->base, r->len);
	if (!p) {
		ccci_md_src_req(md, false);
		seq_printf(s, "cannot map %#llx\n", (unsigned long long)r->base);
		return 0;
	}
	for (off = 0; off < len; off += 16) {
		seq_printf(s, "  %08llx:", (unsigned long long)(r->base + off));
		for (size_t k = 0; k < 16 && off + k < len; k += 4)
			seq_printf(s, " %08x", readl(p + off + k));
		seq_putc(s, '\n');
	}
	iounmap(p);
	ccci_md_src_req(md, false);
	return 0;
}

static int md_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, md_state_show, inode->i_private);
}

/*
 * The loader's tag block, exactly as the stock kernel reads it
 *
 * /chosen/ccci,modem_info_v2 names a 64 KiB block of tag records; the vendor kernel walks them with
 * find_ccci_tag_inf() (ccci_util/ccci_util_lib_fo.c:127-200) and asks for these names: opt_using_lk_val,
 * nc_smem_info_ext_num, nc_smem_info_ext, md1_bank4_cache_info, md1_bank4_cache_layout, smem_layout,
 * ccb_info, udc_layout, md1_sib_info, md1_phy_cap, smem_dfd_size, smem_amms_pos_size,
 * smem_align_padding_size, md1_smem_cahce_offset, mtee_support, hdr_count, hdr_tbl_inf, md1_chk,
 * md1img, md3_chk, md3img, md_mpu_num, md_mpu_inf. Every one of them feeds memory layout, image type
 * or a status string -- which is why the block is dumped here and nothing branches on it: the question
 * was whether the kernel has a branch that skips the clamp for a modem the loader already started, and
 * reading the source says it does not.
 */
static int md_tags_show(struct seq_file *s, void *unused)
{
	struct ccci_md *md = s->private;
	u32 off = 0, n = 0;

	if (!md->tags) {
		seq_puts(s, "no tag block mapped (the loader gave none this boot)\n");
		return 0;
	}
	seq_printf(s, "block %#llx + %#x, version %u, %u tags\n",
		   (unsigned long long)md->lk_base, md->lk_size, md->lk_version,
		   md->lk_tag_num);
	if (md->lk_version < 2) {
		seq_puts(s, "tag version 1: the record layout differs from v2 and is not decoded here\n");
		return 0;
	}
	for (n = 0; n < md->lk_tag_num && off + 76 < md->lk_size; n++) {
		char name[65] = { };
		char *d = kmalloc(512, GFP_KERNEL);
		u32 doff, dsize, next;

		memcpy_fromio(name, md->tags + off, 64);
		{
			u32 v[3];

			memcpy_fromio(v, md->tags + off + 64, sizeof(v));
			doff = le32_to_cpu(v[0]);
			dsize = le32_to_cpu(v[1]);
			next = le32_to_cpu(v[2]);
		}
		seq_printf(s, "  %2u %-32s data %#x + %-6u (%u bytes)\n",
			   n, name, doff, dsize, dsize);
		if (d && doff < md->lk_size && dsize <= 512 && !(doff & 3)) {
			memcpy_fromio(d, md->tags + doff, dsize);
			for (u32 i = 0; i < dsize && i < 64; i += 4) {
				u32 w;
				char txt[5] = "    ";

				memcpy(&w, d + i, 4);
				for (int b = 0; b < 4; b++) {
					u8 c = (w >> (8 * b)) & 0xff;

					txt[b] = (c >= 0x20 && c < 0x7f) ? c : '.';
				}
				seq_printf(s, "        +%02x %08x  |%s|\n", i,
					   le32_to_cpu(w), txt);
			}
		}
		kfree(d);
		/*
		 * next_tag_offset is an ABSOLUTE offset from the block base, not a delta:
		 * the vendor does curr = base + next_tag_offset (find_ccci_tag_inf(),
		 * ccci_util/ccci_util_lib_fo.c:183-185). Trial 40's first dump added it
		 * instead, which is why tag 2 came out with an empty name and a nonsense
		 * size.
		 */
		if (!next || next == off)
			break;
		off = next;
	}
	return 0;
}

/*
 * The infra bus-debug block, read-only
 *
 * bus_dbg@10208000, compatible "mediatek,bus_dbg-v2", with the configuration/status register
 * in the infracfg window at 0x2fc ("mediatek,bus_dbg_con_offset = <0x2fc>",
 * arch/arm64/boot/dts/mediatek/mt6785.dts:1575-1580). It is the block that latches a hung
 * transaction on the interconnect, which is the shape trial 31 measured from the other side
 * (an AP read on the MD path that never completed) and the shape a bus protect that never
 * acknowledges would leave.
 *
 * What it prints is the raw content: the register bit layout lives in the vendor's bus_dbg
 * driver, which is in none of the sources this project has, so the values are printed as
 * they are and named by offset -- no invented bit names, and nothing written.
 */
#define BUS_DBG_PHYS	0x10208000
#define BUS_DBG_WINDOW	0x100

static int md_bus_dbg_show(struct seq_file *s, void *unused)
{
	struct ccci_md *md = s->private;
	void __iomem *p = ioremap(BUS_DBG_PHYS, BUS_DBG_WINDOW);
	size_t off;

	seq_printf(s, "BUS_DBG_CON (infracfg 0x2fc): %#010x\n",
		   md->infra ? readl(md->infra + INFRA_BUS_DBG_CON) : 0);
	seq_printf(s, "bus_dbg block at %#x, window %#zx (raw; the bit layout is in the vendor's\n"
		      "bus_dbg driver, which is in no source we have -- nothing is decoded here)\n",
		   BUS_DBG_PHYS, (size_t)BUS_DBG_WINDOW);
	if (!p) {
		seq_printf(s, "cannot map %#x\n", BUS_DBG_PHYS);
		return 0;
	}
	for (off = 0; off < BUS_DBG_WINDOW; off += 16) {
		seq_printf(s, "  %08x:", (unsigned int)(BUS_DBG_PHYS + off));
		for (size_t k = 0; k < 16; k += 4)
			seq_printf(s, " %08x", readl(p + off + k));
		seq_putc(s, '\n');
	}
	iounmap(p);
	return 0;
}

static int md_bus_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, md_bus_dbg_show, inode->i_private);
}

static const struct file_operations md_bus_dbg_fops = {
	.owner = THIS_MODULE,
	.open = md_bus_dbg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * The modem's own debug view, from shared memory
 *
 * Everything else the AP can read has now been checked against stock: the SPM firmware is kicked
 * (status 2), the SPM request router is configured (SPM_SRC_MASK 0x586f403d, MD1 bits 0x3d), the bus
 * protect mask, offsets, order and reset match the vendor line for line and the reset lands and holds,
 * the CCIF state at the clamp is the vendor's (no hif yet), and both supplies read the values the
 * vendor's own PMIC code writes. What is left is inside the modem, and the only window the AP has into
 * it is the memory the modem writes for us:
 *
 *  - NC_MDCCCI_DBG (2 KiB at the NC share-memory base): the modem's CCCI debug words;
 *  - NC_MDSS_DBG (10 KiB at +0x800): the modem's own exception/log region, which is exactly what the
 *    vendor's dumper reads as "the MD's own log" (fsm/mdee_dumper_v1.c:76-77:
 *    ex_log_addr = mdss_dbg->base_ap_view_vir; ex_log_len = mdss_dbg->size). The power-off check flag
 *    we have been reading sits at +0x1e34 inside it (mt6785/ccci_config.h:43).
 *
 * This is AP-side shared DRAM, not the modem's register or bus space, so it cannot hang anything: the
 * trial-31 hazard was reading MD registers through a port that never answers, which this is not.
 */
static int md_shmem_show(struct seq_file *s, void *unused)
{
	struct ccci_md *md = s->private;

	if (!md->nc) {
		seq_puts(s, "share memory not mapped\n");
		return 0;
	}
	for (size_t r = 0; r < 2; r++) {
		const u32 base = r ? NC_MDSS_DBG : NC_MDCCCI_DBG;
		const u32 size = r ? NC_MDSS_DBG_SIZE : 0x800;
		const char *name = r ? "NC_MDSS_DBG" : "NC_MDCCCI_DBG";

		seq_printf(s, "[%s] NC offset %#x, %#x bytes\n", name, base, size);
		for (u32 off = 0; off < size; off += 16) {
			char txt[17];

			seq_printf(s, "  %04x:", off);
			for (u32 k = 0; k < 16; k += 4) {
				u32 w = readl(md->nc + base + off + k);
				u8 b0 = w & 0xff, b1 = (w >> 8) & 0xff;
				u8 b2 = (w >> 16) & 0xff, b3 = (w >> 24) & 0xff;

				seq_printf(s, " %08x", w);
				txt[k + 0] = (b0 >= 0x20 && b0 < 0x7f) ? b0 : '.';
				txt[k + 1] = (b1 >= 0x20 && b1 < 0x7f) ? b1 : '.';
				txt[k + 2] = (b2 >= 0x20 && b2 < 0x7f) ? b2 : '.';
				txt[k + 3] = (b3 >= 0x20 && b3 < 0x7f) ? b3 : '.';
				txt[k + 4] = '\0';
			}
			seq_printf(s, "  |%s|\n", txt);
		}
	}
	return 0;
}

/*
 * A map of the two share-memory windows: which 4 KiB pages hold anything at all
 *
 * The modem's own debug regions read all zero in trial 45, which is expected once the vendor's
 * dumper is read properly: it touches SMEM_USER_RAW_MDSS_DBG only in the exception path, to hand
 * the modem's own crash log to the AEE dumper (fsm/mdee_dumper_v1.c:76-77,
 * aed_md_exception_api()), not to sample a healthy modem. So zeros there say nothing about the
 * window -- and the question becomes whether the modem has written anywhere in the windows at all.
 *
 * This walks both windows a page at a time and prints only the pages that contain anything,
 * with the number of non-zero words and the first one. It is a map, not a dump: a few hundred
 * lines at most, and it is shared DRAM, so reading it cannot hang the modem's bus.
 */
static int md_shmem_map_show(struct seq_file *s, void *unused)
{
	struct ccci_md *md = s->private;

	if (!md->nc || !md->c) {
		seq_puts(s, "share memory not mapped\n");
		return 0;
	}
	for (size_t w = 0; w < 2; w++) {
		void __iomem *base = w ? md->c : md->nc;
		u32 size = w ? md->c_size : md->nc_size;
		phys_addr_t phys = w ? md->c_phys : md->nc_phys;
		u32 pages = 0, used = 0;

		seq_printf(s, "[%s] %#llx + %#x (%u pages of %#x)\n",
			   w ? "C smem" : "NC smem", (unsigned long long)phys, size,
			   size / 4096, 4096);
		for (u32 off = 0; off + 4096 <= size; off += 4096) {
			u32 nz = 0, first = 0, i;

			for (i = 0; i < 4096; i += 4) {
				u32 v = readl(base + off + i);

				if (v) {
					if (!nz)
						first = i;
					nz++;
				}
			}
			pages++;
			if (!nz)
				continue;
			used++;
			seq_printf(s, "  page %06x  %5u non-zero words, first at +%#05x = %#010x\n",
				   off, nz, first, readl(base + off + first));
		}
		seq_printf(s, "  %u pages, %u with data\n", pages, used);
	}
	return 0;
}

/*
 * One named page, on demand
 *
 * The map says which pages hold something; this says what is in one of them, and only one per
 * read: echo the window and the offset into the file, then read it back. Bounded to a single
 * 4 KiB page on purpose -- a bulk dump is how trial 31 hung the phone.
 *
 *	echo "nc 0x28000" > /sys/kernel/debug/rosemary_ccci/page
 *	cat /sys/kernel/debug/rosemary_ccci/page
 */
static ssize_t md_page_write(struct file *file, const char __user *buf, size_t len,
			     loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct ccci_md *md = m->private;
	char tmp[32], win[8];
	unsigned int off;

	if (len >= sizeof(tmp))
		return -EINVAL;
	if (copy_from_user(tmp, buf, len))
		return -EFAULT;
	tmp[len] = 0;
	if (sscanf(tmp, "%7s %x", win, &off) != 2)
		return -EINVAL;
	if (sysfs_streq(win, "nc")) {
		if (off + 4096 > md->nc_size)
			return -ERANGE;
		md->page_win = 0;
	} else if (sysfs_streq(win, "c")) {
		if (off + 4096 > md->c_size)
			return -ERANGE;
		md->page_win = 1;
	} else {
		return -EINVAL;
	}
	md->page_off = off;
	return len;
}

static int md_page_show(struct seq_file *s, void *unused)
{
	struct ccci_md *md = s->private;
	void __iomem *base;
	u32 off = md->page_off;

	if (md->page_win == 2) {
		seq_puts(s, "no page selected: echo \"nc 0x28000\" or \"c 0x34000\" first\n");
		return 0;
	}
	base = md->page_win ? md->c : md->nc;
	if (!base) {
		seq_puts(s, "share memory not mapped\n");
		return 0;
	}
	seq_printf(s, "[%s + %#x]\n", md->page_win ? "C" : "NC", off);
	for (u32 i = 0; i < 4096; i += 16) {
		char txt[17];

		seq_printf(s, "  %05x:", off + i);
		for (u32 k = 0; k < 16; k += 4) {
			u32 w = readl(base + off + i + k);

			seq_printf(s, " %08x", w);
			txt[k + 0] = (w & 0xff) >= 0x20 && (w & 0xff) < 0x7f ? (w & 0xff) : '.';
			txt[k + 1] = ((w >> 8) & 0xff) >= 0x20 && ((w >> 8) & 0xff) < 0x7f ? ((w >> 8) & 0xff) : '.';
			txt[k + 2] = ((w >> 16) & 0xff) >= 0x20 && ((w >> 16) & 0xff) < 0x7f ? ((w >> 16) & 0xff) : '.';
			txt[k + 3] = ((w >> 24) & 0xff) >= 0x20 && ((w >> 24) & 0xff) < 0x7f ? ((w >> 24) & 0xff) : '.';
			txt[k + 4] = '\0';
		}
		seq_printf(s, "  |%s|\n", txt);
	}
	return 0;
}

static int md_page_open(struct inode *inode, struct file *file)
{
	return single_open(file, md_page_show, inode->i_private);
}

static const struct file_operations md_page_fops = {
	.owner = THIS_MODULE,
	.open = md_page_open,
	.write = md_page_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int md_shmem_map_open(struct inode *inode, struct file *file)
{
	return single_open(file, md_shmem_map_show, inode->i_private);
}

static const struct file_operations md_shmem_map_fops = {
	.owner = THIS_MODULE,
	.open = md_shmem_map_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int md_shmem_open(struct inode *inode, struct file *file)
{
	return single_open(file, md_shmem_show, inode->i_private);
}

static const struct file_operations md_shmem_fops = {
	.owner = THIS_MODULE,
	.open = md_shmem_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int md_tags_open(struct inode *inode, struct file *file)
{
	return single_open(file, md_tags_show, inode->i_private);
}

static const struct file_operations md_tags_fops = {
	.owner = THIS_MODULE,
	.open = md_tags_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int md_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, md_regs_show, inode->i_private);
}

static const struct file_operations md_state_fops = {
	.owner = THIS_MODULE,
	.open = md_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations md_regs_fops = {
	.owner = THIS_MODULE,
	.open = md_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void rosemary_ccci_debugfs_add(struct ccci_md *md)
{
	md->page_win = 2;	/* nothing selected yet */
	md->debugfs = debugfs_create_dir("rosemary_ccci", NULL);
	if (IS_ERR_OR_NULL(md->debugfs)) {
		md->debugfs = NULL;
		return;
	}
	debugfs_create_file("state", 0444, md->debugfs, md, &md_state_fops);
	debugfs_create_file("md_regs", 0444, md->debugfs, md, &md_regs_fops);
	debugfs_create_file("tags", 0444, md->debugfs, md, &md_tags_fops);
	debugfs_create_file("bus_dbg", 0444, md->debugfs, md, &md_bus_dbg_fops);
	debugfs_create_file("md_shmem", 0444, md->debugfs, md, &md_shmem_fops);
	debugfs_create_file("md_shmem_map", 0444, md->debugfs, md, &md_shmem_map_fops);
	debugfs_create_file("page", 0644, md->debugfs, md, &md_page_fops);
}

void rosemary_ccci_debugfs_remove(struct ccci_md *md)
{
	debugfs_remove_recursive(md->debugfs);
	md->debugfs = NULL;
}
