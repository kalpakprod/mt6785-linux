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
 *   state   -- the scalars that decide the questions the trials keep asking: is the
 *              domain on, does the modem hold its clock request, was the clamp
 *              attempted and what did it acknowledge, what did the loader leave.
 *   regions -- the vendor's read-only dump ranges (TOPSM, MD RGU, OST, CSC, ECT,
 *              the MD bus and PLL blocks), each line naming the block, its base and
 *              the vendor line it comes from, dumped as bounded windows because
 *              the vendor's own ranges run to tens of kilobytes.
 *
 * Deliberately left out, because the vendor's own code cannot read them without
 * writing first, or warns that they hang the bus:
 *
 *  - the PC monitor block (0x0d0d9000): the vendor stops and restarts the monitor
 *    around the dump (ap_md_reg_dump.c:59-85);
 *  - the bus-history block (0x0d0c6000): eight pre-action writes select the entry
 *    (ap_md_reg_dump.c:192-294);
 *  - ELM (0x20350000): "This dump might cause bus hang so enable it only when
 *    needed" (ap_md_reg_dump.c:373-378).
 */

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/pm_runtime.h>
#include <linux/iopoll.h>
#include <linux/seq_file.h>

#include "rosemary_ccci.h"

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
 * The vendor's read-only dump ranges, bounded.
 *
 * The vendor dumps whole blocks (TOPSM is 0x8e4 bytes, the bus block 0x19098); a
 * debugfs read that produces a hundred kilobytes of hex is not usable on a phone
 * terminal, so each block is dumped as a window with its name, base and the vendor
 * line it comes from. The head of each block is where the status words live.
 */
struct md_region {
	const char *name;
	const char *where;
	phys_addr_t base;
	size_t len;
	const char *what;
};

#define REGION_WINDOW	0x80

static const struct md_region regions[] = {
	{ "TOPSM", "ap_md_reg_dump.c:325-336 (0x0d0d0000, 0x8e4)", 0x0d0d0000, 0x8e4,
	  "the modem's own top power state machine: sleep requests and their masks live here, which is why the AP can only read it (MD_DBG_DUMP_TOPSM, inc/ccci_modem.h:54)" },
	{ "MD_RGU", "ap_md_reg_dump.c:338-352 (0x0d0d2100 + 0x200, 0xcc and 0x5c)", 0x0d0d2100, 0x25c,
	  "the modem's reset generation unit: what reset it last, and what is held" },
	{ "OST", "ap_md_reg_dump.c:354-368 (0x0d0d1000 + 0x200)", 0x0d0d1000, 0x208,
	  "the modem's own timer: a running counter means its software is alive and clocked" },
	{ "CSC", "ap_md_reg_dump.c:370-383 (0x0d0d3000)", 0x0d0d3000, 0x214,
	  "modem core status capture" },
	{ "ECT", "ap_md_reg_dump.c:302-322 (0x0d0cc130, selected words)", 0x0d0cc130, 0x1ee8,
	  "the modem's event/exception trace block" },
	{ "MD_BUS", "ap_md_reg_dump.c:154-170 (0x0d0c7000, 0x0d0c9000, 0x0d0e0000)", 0x0d0c7000, 0x19098,
	  "the modem's internal bus: a stuck transaction shows up as a non-idle status here, which is the state that would explain a bus protect that never acks" },
	{ "MD_PLL", "ap_md_reg_dump.c:85-153 (0x0d0c3800)", 0x0d0c3800, 0x1c85c,
	  "the modem's clock control: whether its PLLs are up and which one it is running from" },
	{ "USIP", "ap_md_reg_dump.c:384-… (0x0d0c4400)", 0x0d0c4400, 0x3500,
	  "the modem's serial interface block" },
};

static int md_regions_show(struct seq_file *s, void *unused)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(regions); i++) {
		const struct md_region *r = &regions[i];
		void __iomem *p;
		size_t len = min_t(size_t, r->len, REGION_WINDOW);
		size_t off;

		seq_printf(s, "[%s] base %#llx, window %#zx of %#zx\n  %s\n  %s\n",
			   r->name, (unsigned long long)r->base, len, r->len,
			   r->where, r->what);
		p = ioremap(r->base, r->len);
		if (!p) {
			seq_printf(s, "  cannot map %#llx\n", (unsigned long long)r->base);
			continue;
		}
		for (off = 0; off < len; off += 16) {
			seq_printf(s, "  %08llx:", (unsigned long long)(r->base + off));
			for (size_t k = 0; k < 16 && off + k < len; k += 4)
				seq_printf(s, " %08x", readl(p + off + k));
			seq_putc(s, '\n');
		}
		iounmap(p);
	}
	return 0;
}

static int md_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, md_state_show, inode->i_private);
}

static int md_regions_open(struct inode *inode, struct file *file)
{
	return single_open(file, md_regions_show, inode->i_private);
}

static const struct file_operations md_state_fops = {
	.owner = THIS_MODULE,
	.open = md_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations md_regions_fops = {
	.owner = THIS_MODULE,
	.open = md_regions_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void rosemary_ccci_debugfs_add(struct ccci_md *md)
{
	md->debugfs = debugfs_create_dir("rosemary_ccci", NULL);
	if (IS_ERR_OR_NULL(md->debugfs)) {
		md->debugfs = NULL;
		return;
	}
	debugfs_create_file("state", 0444, md->debugfs, md, &md_state_fops);
	debugfs_create_file("regions", 0444, md->debugfs, md, &md_regions_fops);
}

void rosemary_ccci_debugfs_remove(struct ccci_md *md)
{
	debugfs_remove_recursive(md->debugfs);
	md->debugfs = NULL;
}
