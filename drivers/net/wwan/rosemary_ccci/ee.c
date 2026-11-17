// SPDX-License-Identifier: GPL-2.0-only
/*
 * Poco M5s (rosemary): the modem's exception record and its handshake.
 *
 * When the modem fails it does not go quiet: it writes a record of what went
 * wrong into the share memory it was given for exactly that purpose, and then
 * rings the exception channels. This file reads that record and answers the
 * handshake, so a failed boot says why instead of ending in a timeout.
 *
 * The record is a struct ex_overview_t at the start of the MDSS debug region,
 * and the handshake is four channels above CCIF_CH_SRAM. Both are the vendor's
 * own layout, taken from the GPL source for this SoC:
 *
 *  - the record: drivers/misc/mediatek/eccci/fsm/mdee_dumper_v3.h:31-97
 *    (struct ex_assert_v3, struct ex_fatal_v3, struct ex_brief_maininfo,
 *    struct ex_overview_t); MD1 on a 6295 uses the v3 dumper, which is chosen
 *    by mdee_dumper_v3_alloc() for MD_GENERATION >= 6292 in
 *    fsm/ccci_fsm_ee.c:385-394, and it reads the buffer from
 *    SMEM_USER_RAW_MDSS_DBG in md_ee_get_buf_ptr() (mdee_dumper_v3.c:412-437).
 *  - the handshake: hif/ccif_hif_platform.h:39-66 (RINGQ_EXP_BASE 15,
 *    D2H_EXCEPTION_INIT 16, D2H_EXCEPTION_INIT_DONE 17,
 *    D2H_EXCEPTION_CLEARQ_DONE 18, D2H_EXCEPTION_ALLQ_RESET 19,
 *    AP_MD_PEER_WAKEUP 20, AP_MD_SEQ_ERROR 21, and the two answers on 16 and 17)
 *    driven by md_cd_ee_handshake()/md_cd_exception() in modem_sys1.c:144-260.
 */
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "rosemary_ccci.h"

/* exception type classes, mdee_dumper_v3.h:566-600 */
#define MD_EX_CLASS_ASSERT	0
#define MD_EX_CLASS_FATAL	1
#define MD_EX_CLASS_CUSTOM	2

/* the ranges the v3 dumper names its exception types by, mdee_dumper_v3.h:100 */
#define EX_CPU_MAX		0x2F	/* MIPS cause codes */
#define EX_STACKACCESS		0x30
#define EX_SYS_FATALERR_MAX	0x33
#define EX_ASSERT_FAIL		0x50
#define EX_ASSERT_FAIL_MAX	0x55
#define EX_CC_INVALID		0x60
#define EX_CC_MAX		0x67
#define EX_EMI_MPU_VIOLATION	0x70

/* struct ex_step_v3, mdee_dumper_v3.h:31 */
struct ee_step {
	u32 step;
	u32 timestap;
} __packed;

/* struct ex_assert_v3, mdee_dumper_v3.h:36 */
struct ee_assert {
	char filepath[256];
	u32 line_number;
	u32 para1;
	u32 para2;
	u32 para3;
	u32 lr;
} __packed;

/* struct ex_fatal_v3, mdee_dumper_v3.h:45; filename[] is variable length */
struct ee_fatal {
	u32 code1;
	u32 code2;
	u32 code3;
	char offender[8];
	u8 is_cadefa_supported;
	u8 is_filename_supported;
	u8 error_section;
	u8 pad[5];
	u32 error_status;
	u32 error_sp;
	u32 error_pc;
	u32 error_lr;
	u32 error_address;
	u32 error_cause;
	/*
	 * The vendor declares this char[0] and reads it as a string after the
	 * fixed part. A flexible array cannot live inside the union that follows,
	 * so it is spelled with room for the name the vendor prints (32 bytes is
	 * what its own formatter reserves); the union's size is set by the assert
	 * record either way.
	 */
	char filename[32];
} __packed;

/* union ex_main_content, mdee_dumper_v3.h:54 */
union ee_content {
	struct ee_fatal fatal;
	struct ee_assert assert;
} __packed;

/* struct ex_brief_maininfo, mdee_dumper_v3.h:70 */
struct ee_brief_maininfo {
	u16 ex_type;
	u8 e_type_format;
	u8 maincontent_type;
	u8 elm_status;
	u8 system_info1;
	u8 system_info2;
	u8 pad;
	union ee_content info;
} __packed;

/* struct ex_main_reason_v3, mdee_dumper_v3.h:82 */
struct ee_main_reason {
	char core_name[11];
	u8 is_offender;
} __packed;

#define EE_CORE_TOTAL_NUM	8

/* struct ex_overview_t, mdee_dumper_v3.h:86 */
struct ee_overview {
	u32 overview_verno;
	u32 core_num;
	struct ee_main_reason main_reson[EE_CORE_TOTAL_NUM];
	struct ee_brief_maininfo ex_info;
	u32 mips_vpe_num;
	struct ee_step ex_step_logging[6];
	u32 ect_status;
	u32 cs_offending_core;
	u32 md32_offending_status;
	u32 pad;
	u32 core_offset[EE_CORE_TOTAL_NUM];
} __packed;

/*
 * The epoch of an exception type, as the v3 dumper groups them
 * (mdee_dumper_v3.h:100-178). Only the range is named: the full strings are the
 * modem's own vocabulary and the numbers are what a maintainer compares.
 */
static const char *ee_type_name(u16 t)
{
	if (t < EX_CPU_MAX)
		return "cpu exception (MIPS cause code)";
	if (t == EX_STACKACCESS)
		return "Fatal error(STACKACCESS)";
	if (t < EX_SYS_FATALERR_MAX)
		return "Fatal error(task/buf)";
	if (t == EX_ASSERT_FAIL)
		return "ASSERT";
	if (t < EX_ASSERT_FAIL_MAX)
		return "ASSERT (custom/native)";
	if (t < EX_CC_MAX)
		return "Fatal error (cross-core)";
	if (t == EX_EMI_MPU_VIOLATION)
		return "Fatal error (rmpu violation)";
	return "unknown";
}

static const char *ee_class_name(u8 c)
{
	switch (c) {
	case MD_EX_CLASS_ASSERT:
		return "assert";
	case MD_EX_CLASS_FATAL:
		return "fatal error";
	case MD_EX_CLASS_CUSTOM:
		return "custom fatal error";
	}
	return "invalid";
}

static void ee_print(struct ccci_md *md)
{
	const struct ee_overview *o;
	const struct ee_brief_maininfo *b;
	int i, offender = -1;

	if (!md->nc)
		return;
	o = md->nc + NC_MDSS_DBG;

	/* an empty region says the modem died before it could write anything */
	if (!o->core_num && !o->overview_verno && !o->ex_info.ex_type) {
		dev_err(md->dev,
			"exception record empty at %#x (the modem wrote nothing)\n",
			NC_MDSS_DBG);
		return;
	}
	if (o->core_num > EE_CORE_TOTAL_NUM)
		return;

	for (i = 0; i < o->core_num; i++)
		if (o->main_reson[i].is_offender)
			offender = i;

	b = &o->ex_info;
	dev_err(md->dev,
		"exception record: version %u, %u cores, offender core %d, type %#x (%s), class %s (%u), elm status %#x\n",
		o->overview_verno, o->core_num, offender, b->ex_type,
		ee_type_name(b->ex_type), ee_class_name(b->maincontent_type),
		b->maincontent_type, b->elm_status);
	if (offender >= 0)
		dev_err(md->dev, "exception record: offending core named \"%.11s\"\n",
			o->main_reson[offender].core_name);

	if (b->maincontent_type == MD_EX_CLASS_FATAL ||
	    b->maincontent_type == MD_EX_CLASS_CUSTOM) {
		const struct ee_fatal *f = &b->info.fatal;

		dev_err(md->dev,
			"exception record: fatal codes %#x %#x %#x, error section %u, pc %#x, address %#x, cause %#x, offender \"%.8s\"\n",
			f->code1, f->code2, f->code3, f->error_section,
			f->error_pc, f->error_address, f->error_cause,
			f->offender);
		if (f->is_filename_supported)
			dev_err(md->dev, "exception record: offending file \"%.*s\"\n",
				200, f->filename);
	} else {
		const struct ee_assert *a = &b->info.assert;

		dev_err(md->dev,
			"exception record: assert at \"%.200s\":%u, parameters %#x %#x %#x, lr %#x\n",
			a->filepath, a->line_number, a->para1, a->para2,
			a->para3, a->lr);
	}

	/*
	 * The rest of the record, the way the vendor's dumper keeps it: the
	 * overview is only the head of the buffer. struct ex_overview_t ends with
	 * core_offset[], one offset per core, and past it the per-core detail
	 * lives. The layout of that per-core part is not reproduced here -- a
	 * wrong guess would be worse than none -- so the bytes after the overview
	 * are printed raw, for the next trial to read with the same struct at hand.
	 */
	dev_err(md->dev, "exception record: core_offset[] %#x %#x %#x %#x %#x %#x %#x %#x (record %zu bytes)\n",
		o->core_offset[0], o->core_offset[1], o->core_offset[2],
		o->core_offset[3], o->core_offset[4], o->core_offset[5],
		o->core_offset[6], o->core_offset[7], sizeof(*o));
	dev_err(md->dev, "exception record: after the overview %*phN\n", 128,
		(const u8 *)o + sizeof(*o));
}

/**
 * ccci_ee_on_channel() - one exception channel, handled the vendor's way
 * @md: the modem
 * @ch: the channel number the modem rang
 *
 * The modem walks four stages and waits for the AP at two of them
 * (md_cd_ee_handshake(), modem_sys1.c:226-241):
 *   16 D2H_EXCEPTION_INIT       -> the AP answers on 16 (H2D_EXCEPTION_ACK)
 *   17 D2H_EXCEPTION_INIT_DONE  -> nothing to do
 *   18 D2H_EXCEPTION_CLEARQ_DONE-> the AP answers on 17 (H2D_EXCEPTION_CLEARQ_ACK)
 *   19 D2H_EXCEPTION_ALLQ_RESET -> nothing to do, the dump is complete
 * The record itself is written before the first ring, so it is read there.
 */
void ccci_ee_on_channel(struct ccci_md *md, unsigned int ch)
{
	switch (ch) {
	case CCIF_CH_EX_INIT:
		if (!md->ee_printed) {
			md->ee_printed = true;
			ee_print(md);
		}
		ccci_set_state(md, MD_STATE_EXCEPTION);
		ccci_ccif_send_ch(md, CCIF_CH_EX_INIT);
		break;
	case CCIF_CH_EX_INIT_DONE:
		break;
	case CCIF_CH_EX_CLEARQ_DONE:
		/*
		 * The vendor sleeps 2 s here to let its DMA path flush before it
		 * acknowledges; this driver has no queue left to flush, but the
		 * modem's timeout is measured from its own ring, so the answer
		 * goes out at once.
		 */
		ccci_ccif_send_ch(md, CCIF_CH_EX_INIT_DONE);
		break;
	case CCIF_CH_EX_ALLQ_RESET:
		md->ee_printed = true;
		break;
	case CCIF_CH_EX_PEER_WAKEUP:
		break;
	case CCIF_CH_EX_SEQ_ERROR:
		dev_err(md->dev, "the modem found a bad sequence number\n");
		break;
	default:
		dev_err(md->dev, "exception channel %u\n", ch);
	}
}
