// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 Bluetooth over the CONNSYS BTIF transport.
 *
 * Copyright (C) 2026 The poco-m5s port
 *
 * The MT6631 Bluetooth controller has no bus of its own on this phone: it is a
 * task on the CONNSYS MCU, reached over BTIF through the WMT/STP stack that
 * wmt_drv provides. HCI is carried as H4 inside STP frames of type 0
 * (BT_TASK_INDX), one STP frame per HCI packet.
 *
 * The shape of this driver follows drivers/bluetooth/btmtkuart.c, which speaks
 * the same WMT protocol over a real UART on other MediaTek parts. The three
 * workarounds in btmtkbtif_send() and btmtkbtif_open() come from the begonia
 * port, the only place this stack has run on mainline (study
 * 2-connectivity.md 4.5, results #13-#22).
 */

#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/wait.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "h4_recv.h"

/* The WMT/STP public headers, in the order the vendor code includes them. */
#include "osal_typedef.h"
#include "wmt_exp.h"
#include "stp_exp.h"
#include "nvdata_mac.h"

#define BTMTKBTIF_DEV_NAME	"btmtkbtif"

/* One H4 packet per STP frame, so this only has to hold one packet. */
#define BTMTKBTIF_RX_SIZE	HCI_MAX_FRAME_SIZE

/*
 * Workaround 1 - the LE event mask.
 *
 * Claim, from begonia's firmware of the same family: the controller does not
 * declare "LE Set Scan Enable" in Read Local Supported Commands (octets 26/27
 * read 0). The core asks that bit before it enables the advertising-report
 * event (net/bluetooth/hci_sync.c:4511: "if (hdev->commands[26] & 0x08)
 * events[0] |= 0x02"), so without it the host never sees an advertisement. On
 * begonia, ORing the four low bits took a scan from 0 to 43 devices
 * (RESULT #18/#19).
 *
 * NOT YET PROVEN ON THIS PHONE. Trial 13 run 3 does answer 0x1002, but the
 * trace prints 32 bytes and the reply needs 35 to reach octet 26, so this
 * phone's own bits are not in that log. btmtkbtif_note_supported_cmds() below
 * logs octets 26 and 27 at INFO as soon as the reply passes through, which is
 * what settles it: if 26 & 0x08 is set, this workaround is dead weight and goes.
 */
#define BTMTKBTIF_LE_MASK_OR	0x0f

struct btmtkbtif_data {
	struct hci_dev *hdev;

	/* Serialises STP sends: the STP layer expects whole frames. */
	struct mutex tx_mutex;

	/* RX: STP signals new data, this thread drains the type-0 queue. */
	struct task_struct *rx_thread;
	wait_queue_head_t rx_wait;

	u8 *rx_buf;
	struct sk_buff *rx_skb;

	bool bt_on;

	/* Raw byte tracing, bounded so a healthy link does not flood the log. */
	unsigned int traced_tx;
	unsigned int traced_rx;
};

static unsigned int btmtkbtif_le_mask_or = BTMTKBTIF_LE_MASK_OR;
module_param_named(le_mask_or, btmtkbtif_le_mask_or, uint, 0644);
MODULE_PARM_DESC(le_mask_or,
		 "Bits forced into byte 0 of LE Set Event Mask; 0 disables the workaround");

/*
 * How many packets at each end of the link are dumped byte by byte.
 *
 * The point is to answer one question with evidence rather than theory: does the
 * controller ever receive what we put on the BT channel, and does it ever answer
 * on it? A packet that goes out but never comes back, with the WMT channel still
 * working, means the BT task itself is not running; a packet that never reaches
 * the wire at all means the STP/hif path is wrong. Both look identical from HCI.
 * Set to 0 to silence; the log then carries nothing per packet.
 */
static unsigned int btmtkbtif_trace_frames = 8;
module_param_named(trace_frames, btmtkbtif_trace_frames, uint, 0644);
MODULE_PARM_DESC(trace_frames,
		 "Dump this many raw TX and RX frames on the BT channel at open, 0 to disable");

/* One line per packet: the STP payload is exactly the H4 frame. */
static void btmtkbtif_trace(struct hci_dev *hdev, const char *dir,
			    const u8 *buf, unsigned int len)
{
	char hex[3 * 32 + 4];
	unsigned int i, shown = min(len, 32u);

	for (i = 0; i < shown; i++)
		snprintf(hex + 3 * i, 4, "%02x ", buf[i]);

	bt_dev_info(hdev, "BT %s %u bytes: %s%s", dir, len, hex,
		    len > shown ? "..." : "");
}

/* The STP event callback takes no argument, so the instance lives here. */
static struct btmtkbtif_data *btmtkbtif_active;

static const struct h4_recv_pkt btmtkbtif_recv_pkts[] = {
	{ H4_RECV_ACL,   .recv = hci_recv_frame },
	{ H4_RECV_SCO,   .recv = hci_recv_frame },
	{ H4_RECV_EVENT, .recv = hci_recv_frame },
	{ H4_RECV_ISO,   .recv = hci_recv_frame },
};

/*
 * Diagnostic, not a workaround: say what this controller declares about LE
 * scanning, from its own Read Local Supported Commands reply (0x1002). The bit
 * the core asks about is commands[26] & 0x08 ("LE Set Scan Enable",
 * net/bluetooth/hci_sync.c:4511); octet 27 carries the connection-update and
 * read-remote-features bits. One INFO line per boot, only when the reply is
 * complete, and it decides whether workaround 1 above is still needed.
 */
static void btmtkbtif_note_supported_cmds(struct hci_dev *hdev, const u8 *buf,
					  unsigned int len)
{
	if (len < 35)
		return;
	if (buf[0] != HCI_EVENT_PKT || buf[1] != HCI_EV_CMD_COMPLETE)
		return;
	if (get_unaligned_le16(buf + 4) != HCI_OP_READ_LOCAL_COMMANDS)
		return;
	if (buf[6] != 0x00)	/* status */
		return;

	bt_dev_info(hdev,
		    "local commands: octet 26 = 0x%02x (LE Set Scan Enable %s), octet 27 = 0x%02x",
		    buf[7 + 26], (buf[7 + 26] & 0x08) ? "declared" : "absent",
		    buf[7 + 27]);
}

static int btmtkbtif_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtkbtif_data *btdev = hci_get_drvdata(hdev);
	u8 pkt_type = hci_skb_pkt_type(skb);
	int err;

	if (!btdev->bt_on) {
		err = -ENODEV;
		goto drop;
	}

	/*
	 * hdev->send() hands over the packet WITHOUT its H4 type byte; the
	 * transport adds it. Both in-tree H4 transports do exactly this -
	 * h4_enqueue() in hci_h4.c and btmtkuart_send_frame() in btmtkuart.c
	 * each start with "memcpy(skb_push(skb, 1), &type, 1)".
	 *
	 * This driver sent skb->data unchanged, so the controller received a
	 * command whose first byte was the opcode LSB instead of 0x01. Its HCI
	 * transport parsed that as a garbage packet type and asserted
	 * (system/transport/hcit_mtk_uart.c #449), which is why the very first
	 * command after FUNC_CTRL(BT on) - the BD_ADDR read - timed out with
	 * -110 and every later command and the reset timed out too.
	 */
	if (skb_headroom(skb) < 1) {
		err = pskb_expand_head(skb, 1, 0, GFP_ATOMIC);
		if (err < 0)
			goto drop;
	}
	memcpy(skb_push(skb, 1), &pkt_type, 1);

	if (btdev->traced_tx < btmtkbtif_trace_frames) {
		btdev->traced_tx++;
		btmtkbtif_trace(hdev, "tx", skb->data, skb->len);
	}

	mutex_lock(&btdev->tx_mutex);
	err = mtk_wcn_stp_send_data(skb->data, skb->len, BT_TASK_INDX);
	mutex_unlock(&btdev->tx_mutex);

	if (err < 0)
		goto drop;

	hdev->stat.byte_tx += skb->len;
	kfree_skb(skb);

	return 0;

drop:
	hdev->stat.err_tx++;
	/*
	 * Do NOT free here. hci_send_frame() frees the skb itself when ->send()
	 * returns an error (net/bluetooth/hci_core.c:3051-3056), so freeing it
	 * on this path as well is a double free: the slab is handed back, then
	 * reused by the network stack, and the second free is the "refcount_t:
	 * underflow; use-after-free" that arrived in the USB gadget's TX
	 * completion 1.5 s later. On success the driver owns the skb and frees
	 * it exactly once, which is what the branch above does.
	 */
	return err;
}

/*
 * Workaround 2 - the BD_ADDR.
 *
 * The firmware resets the address to its default 00:00:46:76:63:01 on every
 * WMT function-cycle of the Bluetooth task, so writing it once during setup()
 * is not enough: it has to be re-applied after each FUNC_CTRL(BT on), which is
 * what open() does (RESULT #22). The write itself is MediaTek's own command
 * 0xfc1a, the same one drivers/bluetooth/btmtk.c uses.
 */
static int btmtkbtif_set_bdaddr(struct hci_dev *hdev, const bdaddr_t *bdaddr)
{
	struct sk_buff *skb;
	long err;

	skb = __hci_cmd_sync(hdev, 0xfc1a, sizeof(*bdaddr), bdaddr,
			     HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		err = PTR_ERR(skb);
		bt_dev_err(hdev, "BD_ADDR write failed (%ld)", err);
		return err;
	}
	kfree_skb(skb);

	return 0;
}

/*
 * Send the address to the controller, but never a zero one.
 *
 * Trial 4 showed what a zero address does: the TX stream carried
 * 1a fc 06 00 00 00 00 00 00 00, and the firmware asserted
 * (hcit_mtk_uart.c #449), after which HCI_Reset timed out and hci0 went down.
 * A zero address is what hdev->bdaddr holds when the factory nvdata copy is
 * absent and the core has not yet read the controller's own address, so in
 * that case the right move is to learn it from the controller instead of
 * writing it.
 */
static int btmtkbtif_apply_bdaddr(struct hci_dev *hdev)
{
	static const u8 any[ETH_ALEN];
	bdaddr_t ctrl_addr;
	struct sk_buff *skb;
	u8 *p;
	int err = 0;

	if (ether_addr_equal((const u8 *)&hdev->bdaddr, any)) {
		/* Read BD_ADDR (0x1009) and adopt what the controller reports. */
		skb = __hci_cmd_sync(hdev, HCI_OP_READ_BD_ADDR, 0, NULL,
				     HCI_INIT_TIMEOUT);
		if (IS_ERR(skb)) {
			bt_dev_warn(hdev, "cannot read BD_ADDR (%ld), not writing one",
				    PTR_ERR(skb));
			return 0;
		}

		p = skb->data + sizeof(struct hci_ev_cmd_complete);
		if (skb->len >= sizeof(struct hci_ev_cmd_complete) +
				sizeof(struct hci_rp_read_bd_addr) &&
		    p[0] == 0x00) {
			memcpy(&ctrl_addr,
			       &((struct hci_rp_read_bd_addr *)p)->bdaddr,
			       sizeof(ctrl_addr));
			if (!ether_addr_equal((u8 *)&ctrl_addr, any))
				bacpy(&hdev->bdaddr, &ctrl_addr);
		}
		kfree_skb(skb);

		if (ether_addr_equal((u8 *)&hdev->bdaddr, any)) {
			bt_dev_info(hdev, "controller reports no address either, leaving it alone");
			return 0;
		}
	}

	err = btmtkbtif_set_bdaddr(hdev, &hdev->bdaddr);
	if (!err)
		bt_dev_info(hdev, "BD_ADDR %pM applied", &hdev->bdaddr);
	else
		bt_dev_warn(hdev, "BD_ADDR write failed (%d)", err);

	return err;
}

static void btmtkbtif_rx_cb(void)
{
	struct btmtkbtif_data *btdev = READ_ONCE(btmtkbtif_active);

	if (btdev) {
		bt_dev_dbg(btdev->hdev, "RXTRACE cb: stp event -> wake kthread");
		wake_up_interruptible(&btdev->rx_wait);
	}
}

/*
 * Drain everything the STP layer queued for the Bluetooth task. One queue read
 * is one STP frame, i.e. one H4 packet; h4_recv_buf() validates the framing and
 * re-assembles if the firmware ever splits a packet.
 */
static int btmtkbtif_rx_kthread(void *data)
{
	struct btmtkbtif_data *btdev = data;
	struct hci_dev *hdev = btdev->hdev;

	while (!kthread_should_stop()) {
		wait_event_interruptible(btdev->rx_wait,
					 kthread_should_stop() ||
					 !mtk_wcn_stp_is_rxqueue_empty(BT_TASK_INDX));
		if (kthread_should_stop())
			break;

		if (!btdev->bt_on)
			continue;

		while (!mtk_wcn_stp_is_rxqueue_empty(BT_TASK_INDX)) {
			int cnt;

			/*
			 * Lead note 12, step 2: the STP task ring is emptied here and the
			 * bytes go straight into the H4 assembler, so this line and the
			 * "BT rx" trace below bracket hci_recv_frame().
			 */
			bt_dev_dbg(hdev, "RXTRACE kthread: stp task has data");
			cnt = mtk_wcn_stp_receive_data(btdev->rx_buf,
						       BTMTKBTIF_RX_SIZE,
						       BT_TASK_INDX);
			if (cnt <= 0)
				break;
			bt_dev_dbg(hdev, "RXTRACE kthread: got %d bytes", cnt);

			btmtkbtif_note_supported_cmds(hdev, btdev->rx_buf, cnt);

			hdev->stat.byte_rx += cnt;

			if (btdev->traced_rx < btmtkbtif_trace_frames) {
				btdev->traced_rx++;
				btmtkbtif_trace(hdev, "rx", btdev->rx_buf, cnt);
			}

			btdev->rx_skb = h4_recv_buf(hdev, btdev->rx_skb,
						    btdev->rx_buf, cnt,
						    btmtkbtif_recv_pkts,
						    ARRAY_SIZE(btmtkbtif_recv_pkts));
			if (IS_ERR(btdev->rx_skb)) {
				bt_dev_err(hdev, "RX frame failed (%ld)",
					   PTR_ERR(btdev->rx_skb));
				btdev->rx_skb = NULL;
				hdev->stat.err_rx++;
			}
		}
	}

	return 0;
}

static int btmtkbtif_open(struct hci_dev *hdev)
{
	struct btmtkbtif_data *btdev = hci_get_drvdata(hdev);
	u8 addr[ETH_ALEN];
	int err;

	/*
	 * FUNC_CTRL(BT on) returns MTK_WCN_BOOL, not an errno: TRUE (1) is
	 * success and FALSE (0) is failure. Comparing it against zero inverted
	 * the test and turned every successful power-on into "failed (1)" - the
	 * line the phone log showed.
	 */
	if (mtk_wcn_wmt_func_on(WMTDRV_TYPE_BT) != MTK_WCN_BOOL_TRUE) {
		bt_dev_err(hdev, "FUNC_CTRL(BT on) failed");
		return -EIO;
	}

	btdev->bt_on = true;
	btdev->rx_skb = NULL;

	/*
	 * The address is back at the firmware default; take the one the factory
	 * nvdata copy holds, or the core's if there is no copy. Only the local
	 * copy happens here: writing it is setup()'s job, because no HCI
	 * command may be sent from open().
	 */
	if (mtk_nvdata_get_bt_addr(addr))
		bacpy(&hdev->bdaddr, (bdaddr_t *)addr);

	btdev->rx_thread = kthread_run(btmtkbtif_rx_kthread, btdev,
				       "btmtkbtif-rx");
	if (IS_ERR(btdev->rx_thread)) {
		err = PTR_ERR(btdev->rx_thread);
		btdev->rx_thread = NULL;
		btdev->bt_on = false;
		mtk_wcn_wmt_func_off(WMTDRV_TYPE_BT);
		return err;
	}

	/*
	 * HCI_RUNNING belongs to hci_dev_init_sync(), which sets it right after
	 * this function returns. Setting it here made the driver and the core
	 * disagree about the device state.
	 */
	bt_dev_info(hdev, "Bluetooth on");

	return 0;
}

static int btmtkbtif_close(struct hci_dev *hdev)
{
	struct btmtkbtif_data *btdev = hci_get_drvdata(hdev);

	if (btdev->rx_thread) {
		kthread_stop(btdev->rx_thread);
		btdev->rx_thread = NULL;
	}

	btdev->bt_on = false;
	mtk_wcn_wmt_func_off(WMTDRV_TYPE_BT);
	bt_dev_info(hdev, "Bluetooth off");

	return 0;
}

/*
 * hci_dev_init_sync() calls setup() after open() returned and after it set
 * HCI_RUNNING, so this is the first moment an HCI command may be sent at all.
 * Doing that work in open() - as this driver did - failed before it was sent
 * ("cannot read BD_ADDR (-110)" logged ahead of "Bluetooth on", with no
 * transmit line for it) and left the core's command queue unable to carry the
 * next init command: 0x1003 was never transmitted and timed out two seconds
 * later. Both symptoms are one bug, and the fix is the standard order.
 *
 * The LE event mask widening stays in send() where the packet is visible.
 */
static int btmtkbtif_setup(struct hci_dev *hdev)
{
	int err;

	err = btmtkbtif_apply_bdaddr(hdev);
	if (err)
		bt_dev_warn(hdev, "BD_ADDR could not be re-applied (%d)", err);

	return 0;
}

static int btmtkbtif_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	u16 opcode;

	if (hci_skb_pkt_type(skb) != HCI_COMMAND_PKT || skb->len < 3)
		return btmtkbtif_send_frame(hdev, skb);

	opcode = get_unaligned_le16(skb->data);

	/* Workaround 1: widen the LE event mask the core computed. */
	if (opcode == HCI_OP_LE_SET_EVENT_MASK && btmtkbtif_le_mask_or &&
	    skb->len >= 4 + 8)
		skb->data[3] |= btmtkbtif_le_mask_or;

	return btmtkbtif_send_frame(hdev, skb);
}

static int btmtkbtif_probe(struct platform_device *pdev)
{
	struct btmtkbtif_data *btdev;
	struct hci_dev *hdev;
	int err;

	btdev = devm_kzalloc(&pdev->dev, sizeof(*btdev), GFP_KERNEL);
	if (!btdev)
		return -ENOMEM;

	btdev->rx_buf = devm_kzalloc(&pdev->dev, BTMTKBTIF_RX_SIZE, GFP_KERNEL);
	if (!btdev->rx_buf)
		return -ENOMEM;

	mutex_init(&btdev->tx_mutex);
	init_waitqueue_head(&btdev->rx_wait);

	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;

	hdev->bus = HCI_UART;
	hdev->open = btmtkbtif_open;
	hdev->close = btmtkbtif_close;
	hdev->setup = btmtkbtif_setup;
	hdev->send = btmtkbtif_send;
	hdev->manufacturer = 70;	/* MediaTek */
	hdev->set_bdaddr = btmtkbtif_set_bdaddr;
	hci_set_quirk(hdev, HCI_QUIRK_NON_PERSISTENT_SETUP);

	btdev->hdev = hdev;
	hci_set_drvdata(hdev, btdev);
	SET_HCIDEV_DEV(hdev, &pdev->dev);

	err = hci_register_dev(hdev);
	if (err < 0) {
		bt_dev_err(hdev, "hci_register_dev failed (%d)", err);
		hci_free_dev(hdev);
		return err;
	}

	platform_set_drvdata(pdev, btdev);
	WRITE_ONCE(btmtkbtif_active, btdev);

	err = mtk_wcn_stp_register_event_cb(BT_TASK_INDX, btmtkbtif_rx_cb);
	if (err) {
		bt_dev_err(hdev, "cannot register the STP RX callback (%d)", err);
		WRITE_ONCE(btmtkbtif_active, NULL);
		hci_unregister_dev(hdev);
		hci_free_dev(hdev);
		platform_set_drvdata(pdev, NULL);
		return err;
	}

	dev_info(&pdev->dev, "Bluetooth over BTIF ready as %s\n", hdev->name);

	return 0;
}

static void btmtkbtif_remove(struct platform_device *pdev)
{
	struct btmtkbtif_data *btdev = platform_get_drvdata(pdev);
	struct hci_dev *hdev = btdev->hdev;

	WRITE_ONCE(btmtkbtif_active, NULL);
	mtk_wcn_stp_register_event_cb(BT_TASK_INDX, NULL);

	hci_unregister_dev(hdev);
	hci_free_dev(hdev);
	platform_set_drvdata(pdev, NULL);
}

static const struct of_device_id btmtkbtif_of_match[] = {
	{ .compatible = "mediatek,mt6785-bt" },
	{ }
};
MODULE_DEVICE_TABLE(of, btmtkbtif_of_match);

static struct platform_driver btmtkbtif_driver = {
	.probe = btmtkbtif_probe,
	.remove = btmtkbtif_remove,
	.driver = {
		.name = BTMTKBTIF_DEV_NAME,
		.of_match_table = btmtkbtif_of_match,
	},
};
module_platform_driver(btmtkbtif_driver);

MODULE_AUTHOR("The poco-m5s port");
MODULE_DESCRIPTION("MediaTek MT6785 Bluetooth over the CONNSYS BTIF transport");
MODULE_LICENSE("GPL");
