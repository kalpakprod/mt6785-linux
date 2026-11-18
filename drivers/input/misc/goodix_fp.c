// SPDX-License-Identifier: GPL-2.0
/*
 * Goodix side (power-button) fingerprint sensor, GF3626 oswego_m family, on the
 * POCO M5s (rosemary, MT6785).
 *
 * The sensor hangs on SPI0 (0x1100a000, chip select 0) and is driven by the
 * Goodix trusted application inside the Microtrust SOTER secure world: the TA
 * owns the SPI controller and performs every sensor transfer itself. The
 * normal world only powers the sensor (VFP = MT6360 LDO1, 3.3 V), pulses its
 * reset line, keeps the SPI controller's clocks running while the TA needs
 * them, and forwards the sensor interrupt to user space. No fingerprint data
 * ever passes through this driver.
 *
 * /dev/goodix_fp keeps the vendor gf_spi_tee ABI (ioctl magic 'g', netlink
 * events on route 29) that Goodix's HAL and its /vendor/bin/hw fingerprint
 * service expect.
 *
 * Ported from the cezanne-linux-port tree (DrShadow34/cezanne-linux-port,
 * linux/drivers/fingerprint/goodix_fp.c) which was written for the Redmi K30
 * Ultra (cezanne, MT6885) with the same TEE-driven architecture, and checked
 * against the vendor driver of this phone (Xiaomi_Kernel_OpenSource branch
 * rosemary-r-oss, drivers/input/fingerprint/goodix/gf_spi_tee.c).
 */
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/netlink.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <net/net_namespace.h>
#include <net/netlink.h>
#include <net/sock.h>

/*
 * The ioctl set of the vendor driver, unchanged: the Goodix HAL opens
 * /dev/goodix_fp and speaks exactly this. Numbers and structures are copied
 * from gf_spi_tee.h of the vendor tree so that a prebuilt HAL works as is.
 */
#define GF_IOC_MAGIC		'g'
#define GF_IOC_INIT		_IOR(GF_IOC_MAGIC, 0, u8)
#define GF_IOC_EXIT		_IO(GF_IOC_MAGIC, 1)
#define GF_IOC_RESET		_IO(GF_IOC_MAGIC, 2)
#define GF_IOC_ENABLE_IRQ	_IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ	_IO(GF_IOC_MAGIC, 4)
#define GF_IOC_ENABLE_SPI_CLK	_IOW(GF_IOC_MAGIC, 5, u32)
#define GF_IOC_DISABLE_SPI_CLK	_IO(GF_IOC_MAGIC, 6)
#define GF_IOC_ENABLE_POWER	_IO(GF_IOC_MAGIC, 7)
#define GF_IOC_DISABLE_POWER	_IO(GF_IOC_MAGIC, 8)
#define GF_IOC_INPUT_KEY_EVENT	_IOW(GF_IOC_MAGIC, 9, struct gf_key)
#define GF_IOC_ENTER_SLEEP_MODE	_IO(GF_IOC_MAGIC, 10)
#define GF_IOC_GET_FW_INFO	_IOR(GF_IOC_MAGIC, 11, u8)
#define GF_IOC_REMOVE		_IO(GF_IOC_MAGIC, 12)
#define GF_IOC_CHIP_INFO	_IOW(GF_IOC_MAGIC, 13, struct gf_ioc_chip_info)
#define GF_IOC_NAV_EVENT	_IOW(GF_IOC_MAGIC, 14, u32)
/*
 * 15/16/17 are the normal-world (REE) SPI paths of the vendor tree. The vendor
 * driver compiles them out on this phone too (SUPPORT_REE_SPI is undefined and
 * 16/17 are commented out in gf_spi_tee.c), because the TA does all sensor
 * transfers; they are listed here only so the ABI table stays complete and a
 * caller gets a clear error instead of a wrong ioctl.
 */
#define GF_IOC_TRANSFER_CMD	_IOWR(GF_IOC_MAGIC, 15, struct gf_ioc_transfer)
#define GF_IOC_TRANSFER_RAW_CMD	_IOWR(GF_IOC_MAGIC, 16, struct gf_ioc_transfer_raw)
#define GF_IOC_SPI_INIT_CFG_CMD	_IOW(GF_IOC_MAGIC, 17, struct gf_spi_cfg)

/* Netlink route the vendor Goodix driver uses on this phone (gf_spi_tee.c:80). */
#define GF_NETLINK_ROUTE	29
#define GF_NETLINK_IRQ		1
#define GF_NETLINK_SCREEN_OFF	2
#define GF_NETLINK_SCREEN_ON	3
#define GF_NL_MSG_LEN		16

struct gf_key {
	u32 key;
	int value;
};

struct gf_ioc_chip_info {
	u8 vendor_id;
	u8 mode;
	u8 operation;
	u8 reserved[5];
};

struct gf_ioc_transfer {
	u8 cmd;			/* 0 = read, 1 = write */
	u8 reserved;
	u16 addr;
	u32 len;
	u8 *buf;
};

struct gf_ioc_transfer_raw {
	u32 len;
	u8 *read_buf;
	u8 *write_buf;
	u32 high_time;
	u32 low_time;
};

struct gf_spi_cfg {
	unsigned int cs_setuptime;
	unsigned int cs_holdtime;
	unsigned int cs_idletime;
	unsigned int speed_hz;
	unsigned int duty_cycle;
	unsigned int cpol;
	unsigned int cpha;
};

/* the interrupt watchdog and the wakeup it reports need a device */
#define GF_WAKEUP_MS		500

struct gf_dev {
	struct spi_device *spi;
	struct regulator *vfp;
	struct gpio_desc *reset;
	struct cdev cdev;
	struct class *class;
	dev_t devno;
	struct sock *nl_sk;
	u32 nl_pid;
	int irq;
	bool irq_enabled;
	bool irq_requested;
	int clk_on;
	struct mutex lock;	/* serialises ioctls and the sysfs chip-id read */
};

static struct gf_dev *gf_global;	/* netlink input callback and sysfs */

/*
 * Reset pulse: the sensor's RESET line is active low at the pin (the DT declares
 * it GPIO_ACTIVE_LOW), so a logical 1 drives it low. The vendor driver holds it
 * low for 15 ms then releases it (gf_hw_power_enable(), gf_spi_tee.c:289 with
 * pinctrl states fpsensor_fpc_rst_low/_high, mdelay(15)).
 */
static void gf_reset(struct gf_dev *gf, unsigned int delay_ms)
{
	gpiod_set_value_cansleep(gf->reset, 1);
	usleep_range(15000, 16000);
	gpiod_set_value_cansleep(gf->reset, 0);
	if (delay_ms)
		msleep(delay_ms);
}

/*
 * The TA does the transfers, but it does not own the controller's clocks: the
 * AP must keep them on while the secure world talks to the sensor, exactly as
 * the vendor's gf_spi_clk_enable()/mt_spi_enable_master_clk() pair does.
 */
static void gf_spi_clk(struct gf_dev *gf, bool on)
{
	struct device *ctrl = gf->spi->controller->dev.parent;

	if (on && !gf->clk_on) {
		if (pm_runtime_resume_and_get(ctrl) == 0)
			gf->clk_on = 1;
	} else if (!on && gf->clk_on) {
		pm_runtime_put(ctrl);
		gf->clk_on = 0;
	}
}

static void gf_netlink_send(struct gf_dev *gf, u8 cmd)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;

	if (!gf->nl_sk || !gf->nl_pid)
		return;
	skb = nlmsg_new(GF_NL_MSG_LEN, GFP_ATOMIC);
	if (!skb)
		return;
	nlh = nlmsg_put(skb, 0, 0, 0, GF_NL_MSG_LEN, 0);
	if (!nlh) {
		kfree_skb(skb);
		return;
	}
	memset(nlmsg_data(nlh), 0, GF_NL_MSG_LEN);
	*(u8 *)nlmsg_data(nlh) = cmd;
	netlink_unicast(gf->nl_sk, skb, gf->nl_pid, MSG_DONTWAIT);
}

/* the HAL registers its port id by sending any message to the route */
static void gf_netlink_recv(struct sk_buff *skb)
{
	struct nlmsghdr *nlh = nlmsg_hdr(skb);

	if (gf_global && skb->len >= NLMSG_HDRLEN)
		gf_global->nl_pid = nlh->nlmsg_pid;
}

static irqreturn_t gf_irq(int irq, void *data)
{
	struct gf_dev *gf = data;

	pm_wakeup_event(&gf->spi->dev, GF_WAKEUP_MS);
	gf_netlink_send(gf, GF_NETLINK_IRQ);
	return IRQ_HANDLED;
}

static void gf_irq_enable(struct gf_dev *gf, bool on)
{
	if (!gf->irq_requested || gf->irq_enabled == on)
		return;
	if (on)
		enable_irq(gf->irq);
	else
		disable_irq(gf->irq);
	gf->irq_enabled = on;
}

static long gf_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf = f->private_data;
	void __user *uarg = (void __user *)arg;
	u8 route = GF_NETLINK_ROUTE, zero = 0;
	long ret = 0;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENOTTY;

	mutex_lock(&gf->lock);
	switch (cmd) {
	case GF_IOC_INIT:
		if (copy_to_user(uarg, &route, 1)) {
			ret = -EFAULT;
			break;
		}
		if (gf->irq_requested)
			break;
		ret = request_threaded_irq(gf->irq, NULL, gf_irq,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   "goodix_fp_irq", gf);
		if (ret)
			break;
		gf->irq_requested = true;
		gf->irq_enabled = true;
		gf_irq_enable(gf, false);
		break;
	case GF_IOC_EXIT:
		if (gf->irq_requested) {
			gf_irq_enable(gf, true);
			free_irq(gf->irq, gf);
			gf->irq_requested = false;
			gf->irq_enabled = false;
		}
		break;
	case GF_IOC_RESET:
		gf_reset(gf, 60);
		break;
	case GF_IOC_ENABLE_IRQ:
		gf_irq_enable(gf, true);
		break;
	case GF_IOC_DISABLE_IRQ:
		gf_irq_enable(gf, false);
		break;
	case GF_IOC_ENABLE_SPI_CLK:
		gf_spi_clk(gf, true);
		break;
	case GF_IOC_DISABLE_SPI_CLK:
		gf_spi_clk(gf, false);
		break;
	case GF_IOC_ENABLE_POWER:
		/* the vendor driver only pulses reset here; VFP stays on */
		gf_reset(gf, 0);
		break;
	case GF_IOC_DISABLE_POWER:
	case GF_IOC_ENTER_SLEEP_MODE:
	case GF_IOC_INPUT_KEY_EVENT:
	case GF_IOC_NAV_EVENT:
	case GF_IOC_CHIP_INFO:
	case GF_IOC_REMOVE:
		break;
	case GF_IOC_GET_FW_INFO:
		/* the vendor driver reports "no free-fall/feature flag" */
		if (copy_to_user(uarg, &zero, 1))
			ret = -EFAULT;
		break;
	case GF_IOC_TRANSFER_CMD:
	case GF_IOC_TRANSFER_RAW_CMD:
	case GF_IOC_SPI_INIT_CFG_CMD:
		/* normal-world SPI is compiled out on this phone, as in the vendor tree */
		ret = -EOPNOTSUPP;
		break;
	default:
		ret = -ENOTTY;
	}
	mutex_unlock(&gf->lock);
	return ret;
}

static int gf_open(struct inode *inode, struct file *f)
{
	f->private_data = container_of(inode->i_cdev, struct gf_dev, cdev);
	return nonseekable_open(inode, f);
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	.open = gf_open,
	.unlocked_ioctl = gf_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

/*
 * Chip id, read over the normal world SPI path exactly as the vendor's
 * check_hwid() does (gf_spi_tee.c:2181 and gf_spi_read_bytes() at :1674): write
 * 0xF0, address high, address low, wait 5 us, then transfer 0xF1 plus the
 * requested bytes and take the bytes that follow the 0xF1. The transfer speed
 * comes from the DT (spi-max-frequency = 1 MHz, the speed the vendor's REE path
 * runs at).
 */
static int gf_chip_id_read(struct gf_dev *gf, u8 id[4])
{
	u8 cmd[3] = { 0xf0, 0x00, 0x00 };
	u8 buf[5] = { 0xf1, 0x00, 0x00, 0x00, 0x00 };
	struct spi_transfer xfer[2] = {
		{ .tx_buf = cmd, .len = sizeof(cmd),
		  .speed_hz = gf->spi->max_speed_hz },
		{ .tx_buf = buf, .rx_buf = buf, .len = sizeof(buf),
		  .speed_hz = gf->spi->max_speed_hz },
	};
	int ret;

	ret = spi_sync_transfer(gf->spi, &xfer[0], 1);
	if (ret)
		return ret;
	udelay(5);
	ret = spi_sync_transfer(gf->spi, &xfer[1], 1);
	if (!ret)
		memcpy(id, buf + 1, sizeof(buf) - 1);

	return ret;
}

/*
 * The vendor's acceptance test for this sensor family: the chip id starts with
 * 0x07, 0x08 or 0x10 and ends with 0x25 (check_hwid(), gf_spi_tee.c:2205).
 * A sensor that answers with anything else is either absent or held in reset.
 */
static bool gf_chip_id_known(const u8 id[4])
{
	return id[3] == 0x25 && (id[0] == 0x07 || id[0] == 0x08 || id[0] == 0x10);
}

static ssize_t chip_id_show(struct device *dev, struct device_attribute *a,
			    char *out)
{
	struct gf_dev *gf = dev_get_drvdata(dev);
	u8 id[4];
	int ret;

	mutex_lock(&gf->lock);
	ret = gf_chip_id_read(gf, id);
	mutex_unlock(&gf->lock);
	return ret ?: sysfs_emit(out, "%*phN\n", 4, id);
}
static DEVICE_ATTR_RO(chip_id);

static int gf_probe(struct spi_device *spi)
{
	struct netlink_kernel_cfg cfg = { .input = gf_netlink_recv };
	struct device *dev = &spi->dev;
	struct gf_dev *gf;
	struct device *cdev;
	u8 id[4] = { 0 };
	int ret;

	gf = devm_kzalloc(dev, sizeof(*gf), GFP_KERNEL);
	if (!gf)
		return -ENOMEM;
	gf->spi = spi;
	mutex_init(&gf->lock);
	spi_set_drvdata(spi, gf);

	gf->irq = spi->irq;
	if (gf->irq <= 0)
		return dev_err_probe(dev, -ENXIO, "no interrupt\n");
	irq_set_status_flags(gf->irq, IRQ_NOAUTOEN);

	gf->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gf->reset))
		return dev_err_probe(dev, PTR_ERR(gf->reset), "reset gpio\n");

	gf->vfp = devm_regulator_get(dev, "vfp");
	if (IS_ERR(gf->vfp))
		return dev_err_probe(dev, PTR_ERR(gf->vfp), "vfp supply\n");
	ret = regulator_enable(gf->vfp);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable vfp\n");
	usleep_range(1000, 2000);
	gf_reset(gf, 10);

	ret = gf_chip_id_read(gf, id);
	if (ret)
		dev_warn(dev, "chip id read failed: %d\n", ret);
	else if (!gf_chip_id_known(id))
		dev_warn(dev, "chip id %*phN is not a known oswego_m id\n", 4, id);
	else
		dev_info(dev, "Goodix fingerprint sensor, chip id %*phN\n", 4, id);

	gf->nl_sk = netlink_kernel_create(&init_net, GF_NETLINK_ROUTE, &cfg);
	if (!gf->nl_sk) {
		ret = -ENOMEM;
		goto err_power;
	}
	gf_global = gf;

	ret = alloc_chrdev_region(&gf->devno, 0, 1, "goodix_fp");
	if (ret)
		goto err_nl;
	cdev_init(&gf->cdev, &gf_fops);
	gf->cdev.owner = THIS_MODULE;
	ret = cdev_add(&gf->cdev, gf->devno, 1);
	if (ret)
		goto err_region;
	gf->class = class_create("goodix_fp");
	if (IS_ERR(gf->class)) {
		ret = PTR_ERR(gf->class);
		goto err_cdev;
	}
	cdev = device_create(gf->class, dev, gf->devno, gf, "goodix_fp");
	if (IS_ERR(cdev)) {
		ret = PTR_ERR(cdev);
		goto err_class;
	}
	device_init_wakeup(dev, true);
	ret = device_create_file(dev, &dev_attr_chip_id);
	if (ret)
		goto err_device;
	return 0;

err_device:
	device_destroy(gf->class, gf->devno);
err_class:
	class_destroy(gf->class);
err_cdev:
	cdev_del(&gf->cdev);
err_region:
	unregister_chrdev_region(gf->devno, 1);
err_nl:
	gf_global = NULL;
	netlink_kernel_release(gf->nl_sk);
err_power:
	regulator_disable(gf->vfp);
	return ret;
}

static void gf_remove(struct spi_device *spi)
{
	struct gf_dev *gf = spi_get_drvdata(spi);

	device_remove_file(&spi->dev, &dev_attr_chip_id);
	device_destroy(gf->class, gf->devno);
	class_destroy(gf->class);
	cdev_del(&gf->cdev);
	unregister_chrdev_region(gf->devno, 1);
	if (gf->irq_requested)
		free_irq(gf->irq, gf);
	gf_global = NULL;
	netlink_kernel_release(gf->nl_sk);
	gf_spi_clk(gf, false);
	gpiod_set_value_cansleep(gf->reset, 1);
	regulator_disable(gf->vfp);
}

static const struct of_device_id gf_of_match[] = {
	{ .compatible = "goodix,gf3626-fp" },
	{ }
};
MODULE_DEVICE_TABLE(of, gf_of_match);

static const struct spi_device_id gf_spi_ids[] = {
	{ "gf3626-fp" },
	{ }
};
MODULE_DEVICE_TABLE(spi, gf_spi_ids);

static struct spi_driver gf_driver = {
	.driver = {
		.name = "goodix_fp",
		.of_match_table = gf_of_match,
	},
	.id_table = gf_spi_ids,
	.probe = gf_probe,
	.remove = gf_remove,
};
module_spi_driver(gf_driver);

MODULE_DESCRIPTION("Goodix side fingerprint sensor (TEE-driven) on the POCO M5s");
MODULE_LICENSE("GPL");
