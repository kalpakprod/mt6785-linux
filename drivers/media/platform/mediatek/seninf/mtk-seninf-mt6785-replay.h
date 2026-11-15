/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Register replay tables for the MT6785 SENINF, empty on purpose.
 *
 * The vendor BSP programs SENINF and the MIPI RX analog block from closed
 * userspace (libcam.* and the CCU firmware), so the values that make a given
 * module stream are not in any GPL source. The cezanne MT6885 port solved that
 * by snapshotting those registers on the stock phone while each module was
 * previewing, keeping only the words that stayed the same across two runs, and
 * replaying them (its README, "Snapshot & replay"; study
 * poco-m5s/research/native/5-cameras.md sections 3.1 and 4.4).
 *
 * That snapshot is a phone action: the lead runs it on the stock phone with the
 * camera app streaming each module, dumping the SENINF window at 0x1a040000
 * (+0xa000) and the analog window at 0x11c80000 (+0xc000). Until it exists,
 * this header defines the table shape and no tables: the driver then programs
 * SENINF from the fields it knows (see mtk-seninf-mt6785.c), which is what the
 * ChromeOS reference does, and the replay adds the rest.
 *
 * When the snapshots arrive, one entry per receiver, analog words first:
 *
 *	static const struct seninf_reg replay_main_ana[] = { ... };
 *	static const struct seninf_reg replay_main_dig[] = { ... };
 *	static const struct seninf_replay seninf_replay_tables[] = {
 *		{ .receiver = 1, .ana = replay_main_ana,
 *		  .n_ana = ARRAY_SIZE(replay_main_ana),
 *		  .dig = replay_main_dig,
 *		  .n_dig = ARRAY_SIZE(replay_main_dig) },
 *	};
 */

#ifndef __MTK_SENINF_MT6785_REPLAY_H__
#define __MTK_SENINF_MT6785_REPLAY_H__

#include <linux/types.h>

/**
 * struct seninf_reg - one register write
 * @ofs: offset inside the block the write belongs to
 * @val: value to write
 */
struct seninf_reg {
	u32 ofs;
	u32 val;
};

/**
 * struct seninf_replay - the writes that make one receiver work
 * @receiver: SENINF receiver number, 1-based
 * @ana: analog block writes, replayed first
 * @n_ana: number of analog writes
 * @dig: digital block writes, replayed in address order
 * @n_dig: number of digital writes
 */
struct seninf_replay {
	unsigned int receiver;
	const struct seninf_reg *ana;
	unsigned int n_ana;
	const struct seninf_reg *dig;
	unsigned int n_dig;
};

/* No snapshot yet: no table. The driver then programs SENINF by itself. */
static const struct seninf_replay seninf_replay_tables[] = { };

#endif /* __MTK_SENINF_MT6785_REPLAY_H__ */
