/* SPDX-License-Identifier: GPL-2.0 */
/*
 * compat shim: linux/of_gpio.h
 *
 * The vendor connectivity code (wmt_gpio.c, mtk_wcn_consys_hw.c) resolves a
 * single GPIO number from a device-tree property with the legacy
 * of_get_named_gpio() helper. Linux 7.3.4 has no <linux/of_gpio.h> and no such
 * helper: it was replaced by the descriptor API in <linux/gpio/consumer.h>.
 *
 * This shim is a signature/type shim only, as mtk-bsp-compat.h requires: the
 * property is looked up with exactly the same semantics the old helper had
 * (same property name, same index, no active-low translation) and converted
 * back to the integer number the vendor code stores. The resulting line stays
 * requested for the caller, which is what the vendor code expects because it
 * then drives it through the legacy gpio_* accessors.
 *
 * On rosemary none of these properties exist, so the helper returns a negative
 * errno and the vendor code takes its "pin not defined" path, as on stock.
 */
#ifndef _COMPAT_LINUX_OF_GPIO_H
#define _COMPAT_LINUX_OF_GPIO_H

#include <linux/types.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>

/**
 * of_get_named_gpio() - resolve a GPIO number from a device-tree property
 * @np:		device node to get the GPIO from
 * @propname:	property name holding the gpio specifier
 * @index:	index of the GPIO inside the property
 *
 * Returns the global GPIO number, or a negative errno when the property is
 * absent or malformed.
 */
static inline int of_get_named_gpio(struct device_node *np,
				    const char *propname, int index)
{
	struct gpio_desc *desc;

	desc = fwnode_gpiod_get_index(of_fwnode_handle(np), propname, index,
				      GPIOD_ASIS, propname);
	if (IS_ERR(desc))
		return PTR_ERR(desc);

	return desc_to_gpio(desc);
}

#endif /* _COMPAT_LINUX_OF_GPIO_H */
