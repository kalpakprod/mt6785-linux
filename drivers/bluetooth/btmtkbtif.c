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
 * The controller does not declare "LE Set Scan Enable" and its neighbours in
 * Read Local Supported Commands (octets 26/27 read 0), so
 * hci_le_set_event_mask_sync() (net/bluetooth/hci_sync.c) builds an LE event
 * mask without the advertising-report bit and the host never sees a single
 * advertisement. This driver ORs the four low bits into the mask the core
 * sends. On begonia that took a scan from 0 to 43 devices (RESULT #18/#19).
 */
#define BTMTKBTIF_LE_MASK_OR	0x0f

/*
 * Workaround 3 - opcode 0x0c77 (Read Synchronization Train Parameters).
 *
 * This firmware rejects the command with status 0x01, which the HCI core
 * treats as fatal and the setup sequence stops there. The driver answers it
 * itself with a Command Complete carrying status 0 (RESULT #14).
 */
#define BTMTKBTIF_SYNC_TRAIN_OPCODE	0x0c77

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

static bool btmtkbtif_sync_train_fake = true;
module_param_named(sync_train_fake, btmtkbtif_sync_train_fake, bool, 0644);
MODULE_PARM_DESC(sync_train_fake,
		 "Answer opcode 0x0c77 locally instead of letting the firmware reject it");

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

/* Answer a command ourselves, the way the controller would have. */
static int btmtkbtif_cmd_complete(struct hci_dev *hdev, u16 opcode)
{
	struct hci_ev_cmd_complete *ev;
	struct sk_buff *skb;
	u8 *sent_opcode;

	skb = bt_skb_alloc(sizeof(*ev) + sizeof(opcode), GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	hci_skb_pkt_type(skb) = HCI_EV_CMD_COMPLETE;
	ev = skb_put(skb, sizeof(*ev));
	ev->ncmd = 1;
	ev->opcode = cpu_to_le16(opcode);
	sent_opcode = skb_put(skb, sizeof(opcode));
	put_unaligned_le16(opcode, sent_opcode);

	/* hci_recv_frame() takes ownership of the skb. */
	return hci_recv_frame(hdev, skb);
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
	kfree_skb(skb);

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

	if (btdev)
		wake_up_interruptible(&btdev->rx_wait);
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

			cnt = mtk_wcn_stp_receive_data(btdev->rx_buf,
						       BTMTKBTIF_RX_SIZE,
						       BT_TASK_INDX);
			if (cnt <= 0)
				break;

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
	 * Keep BTIF clocked for as long as Bluetooth is up.
	 *
	 * The BTIF driver gates its clocks in _btif_enter_dpidle_from_on() as
	 * soon as btif_enter_dpidle() is called, and nothing calls it in this
	 * port: the STP power-saving path that would is dead, because
	 * mtk_wcn_stp_psm_enable() requires mtk_wcn_stp_is_btif_fullset_mode()
	 * and this SoC runs STP in mandatory mode (mtk6785 keeps its patch in
	 * EMI, pda_dl_patch_flag = 1, so the full-mode switch is skipped).
	 *
	 * Trial 7 shows what that costs: the answer to the first HCI command
	 * arrives while BTIF is still clocked and gets through, then the
	 * controller's next event lands while BTIF is in DPIDLE and is never
	 * seen - no rx interrupt, no read, just the HCI timeout (-110).
	 *
	 * Stock does not have that gap because its deep-idle entry is the tail
	 * of the STP sleep handshake, with the host wake-up line armed first
	 * (wmt_lib.c:947 wmt_plat_eirq_ctrl(PIN_BGF_EINT, PIN_STA_EINT_EN)
	 * right after the SLEEP command). This port never arms it, so the
	 * only correct behaviour left is to keep BTIF awake while the link
	 * is up and let it sleep when Bluetooth goes down.
	 */
	mtk_wcn_stp_dpidle_ctrl(0);

	/*
	 * The address is back at the firmware default; restore the one the
	 * factory nvdata copy holds, or the core's if there is no copy.
	 */
	if (mtk_nvdata_get_bt_addr(addr))
		bacpy(&hdev->bdaddr, (bdaddr_t *)addr);

	err = btmtkbtif_apply_bdaddr(hdev);
	if (err)
		bt_dev_warn(hdev, "BD_ADDR could not be re-applied (%d)", err);

	btdev->rx_thread = kthread_run(btmtkbtif_rx_kthread, btdev,
				       "btmtkbtif-rx");
	if (IS_ERR(btdev->rx_thread)) {
		err = PTR_ERR(btdev->rx_thread);
		btdev->rx_thread = NULL;
		btdev->bt_on = false;
		mtk_wcn_wmt_func_off(WMTDRV_TYPE_BT);
		return err;
	}

	if (!test_and_set_bit(HCI_RUNNING, &hdev->flags))
		bt_dev_info(hdev, "Bluetooth on");

	return 0;
}

static int btmtkbtif_close(struct hci_dev *hdev)
{
	struct btmtkbtif_data *btdev = hci_get_drvdata(hdev);

	clear_bit(HCI_RUNNING, &hdev->flags);

	if (btdev->rx_thread) {
		kthread_stop(btdev->rx_thread);
		btdev->rx_thread = NULL;
	}

	btdev->bt_on = false;

	/* Let BTIF enter deep idle again, now that nothing needs it. */
	mtk_wcn_stp_dpidle_ctrl(1);

	mtk_wcn_wmt_func_off(WMTDRV_TYPE_BT);
	bt_dev_info(hdev, "Bluetooth off");

	return 0;
}

static int btmtkbtif_setup(struct hci_dev *hdev)
{
	/* The address and the LE mask are handled in open() and send(). */
	return 0;
}

static int btmtkbtif_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	u16 opcode;

	if (hci_skb_pkt_type(skb) != HCI_COMMAND_PKT || skb->len < 3)
		return btmtkbtif_send_frame(hdev, skb);

	opcode = get_unaligned_le16(skb->data);

	/* Workaround 3: the firmware rejects this one with status 0x01. */
	if (opcode == BTMTKBTIF_SYNC_TRAIN_OPCODE && btmtkbtif_sync_train_fake) {
		kfree_skb(skb);
		return btmtkbtif_cmd_complete(hdev, opcode);
	}

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
