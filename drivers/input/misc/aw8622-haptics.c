// SPDX-License-Identifier: GPL-2.0-only
/*
 * Awinic AW8622 LRA haptics driver
 *
 * The AW8622 is a linear-resonant-actuator driver with a PWM input and an
 * enable pin: a square wave at the motor's resonance frequency drives it and
 * the duty cycle sets the strength.
 *
 * The shape of this driver (input device, FF_RUMBLE through
 * input_ff_create_memless, a PWM plus an enable GPIO) is that of
 * drivers/input/misc/pwm-vibra.c, Copyright (C) 2017 Collabora Ltd. The part's
 * wiring and its resonance frequency come from the vendor driver
 * drivers/misc/aw8622_haptic/aw8622_haptic.c of LineageOS'
 * android_kernel_xiaomi_mt6785 (scratch/native/4-modem/src/los-mt6785),
 * Copyright (c) 2021 MediaTek Inc.; see the note on wave playback below.
 *
 * The vendor driver plays the phone's own aw8622_*.bin waveforms through a
 * MediaTek-only PWM memory mode (a DMA-fed waveform generator) that this
 * tree's PWM driver does not implement, so the actuator is driven the way the
 * vendor driver drives it whenever no waveform is playing
 * (aw8622_set_pwm_defalut_state()): a continuous square wave at the resonance
 * frequency with the amplitude set by the duty cycle. That is also what
 * pwm-vibra does for every other PWM-input actuator.
 */

#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/time64.h>
#include <linux/workqueue.h>

#define AW8622_DEFAULT_FREQ_HZ		208
#define AW8622_MAX_MAGNITUDE		0xffff

/**
 * struct aw8622_haptic - one AW8622 instance
 * @dev: the platform device
 * @input: the input device the FF effects arrive on
 * @pwm: the PWM channel feeding the actuator
 * @enable_gpio: the chip's HWEN pin
 * @period_ns: one cycle at the actuator's resonance frequency
 * @level: last requested magnitude, 0 when idle
 * @lock: serialises the PWM and GPIO updates
 * @play_work: applies @level outside the FF callback
 * @active: whether the actuator is currently driven
 */
struct aw8622_haptic {
	struct device *dev;
	struct input_dev *input;
	struct pwm_device *pwm;
	struct gpio_desc *enable_gpio;
	u64 period_ns;
	u16 level;
	struct mutex lock; /* serialises the pwm and gpio updates */
	struct work_struct play_work;
	bool active;
};

static int aw8622_start(struct aw8622_haptic *haptic, u16 level)
{
	struct pwm_state state;
	int ret;

	dev_dbg(haptic->dev, "start, level %u\n", level);

	gpiod_set_value_cansleep(haptic->enable_gpio, 1);

	pwm_init_state(haptic->pwm, &state);
	state.period = haptic->period_ns;
	/*
	 * The actuator is driven by a square wave at its resonance frequency and
	 * follows the AC part of it: half a period is the strongest drive, no
	 * drive and a constant level are both silence, and strength falls away
	 * on either side of 50 %. Measured on the phone with a debugfs look at
	 * this PWM while an FF_RUMBLE played (15.11): a magnitude of 0xffff gave
	 * "4807692/4807692 ns" - 100 % duty, a constant level - and the owner
	 * felt nothing, twice; 0x8000 gave "2403882/4807692 ns" - 50 % - and he
	 * felt the vibration. The vendor driver runs the same way: its
	 * aw8622_set_pwm_defalut_state() programs a fixed 50 % duty
	 * (THRESH = clock / (frequency * 2)) and leaves the strength to the
	 * waveform it plays.
	 *
	 * So the magnitude is mapped onto 0..50 % of the period, and a request
	 * that rounds below the PWM's smallest step still has to move something.
	 */
	state.duty_cycle = div_u64(haptic->period_ns * level,
				   2 * AW8622_MAX_MAGNITUDE);
	if (level && !state.duty_cycle)
		state.duty_cycle = haptic->period_ns / 100;
	state.enabled = true;

	ret = pwm_apply_might_sleep(haptic->pwm, &state);
	if (ret) {
		dev_err(haptic->dev, "cannot apply the pwm state: %d\n", ret);
		gpiod_set_value_cansleep(haptic->enable_gpio, 0);
		return ret;
	}

	haptic->active = true;

	return 0;
}

static void aw8622_stop(struct aw8622_haptic *haptic)
{
	if (!haptic->active)
		return;

	dev_dbg(haptic->dev, "stop\n");

	pwm_disable(haptic->pwm);
	gpiod_set_value_cansleep(haptic->enable_gpio, 0);
	haptic->active = false;
}

static void aw8622_play_work(struct work_struct *work)
{
	struct aw8622_haptic *haptic = container_of(work, struct aw8622_haptic,
						    play_work);

	mutex_lock(&haptic->lock);
	if (haptic->level)
		aw8622_start(haptic, haptic->level);
	else
		aw8622_stop(haptic);
	mutex_unlock(&haptic->lock);
}

static int aw8622_play_effect(struct input_dev *dev, void *data,
			      struct ff_effect *effect)
{
	struct aw8622_haptic *haptic = input_get_drvdata(dev);

	haptic->level = effect->u.rumble.strong_magnitude;
	if (!haptic->level)
		haptic->level = effect->u.rumble.weak_magnitude;

	schedule_work(&haptic->play_work);

	return 0;
}

static void aw8622_close(struct input_dev *input)
{
	struct aw8622_haptic *haptic = input_get_drvdata(input);

	cancel_work_sync(&haptic->play_work);
	mutex_lock(&haptic->lock);
	aw8622_stop(haptic);
	mutex_unlock(&haptic->lock);
}

static int aw8622_probe(struct platform_device *pdev)
{
	struct aw8622_haptic *haptic;
	struct pwm_state state;
	u32 freq_hz = AW8622_DEFAULT_FREQ_HZ;
	int ret;

	haptic = devm_kzalloc(&pdev->dev, sizeof(*haptic), GFP_KERNEL);
	if (!haptic)
		return -ENOMEM;

	haptic->dev = &pdev->dev;
	mutex_init(&haptic->lock);
	INIT_WORK(&haptic->play_work, aw8622_play_work);

	/*
	 * The enable pin is the chip's HWEN input, GPIO202 on this board (the
	 * board's own tree calls it hwen-gpio) [L: device-tree.dts, aw8622
	 * node]. The PWM channel is 0, muxed to GPIO26 [L: same node, pwm_ch,
	 * and the haptic_gpio_aw8622_set pin group, which muxes GPIO26 to
	 * PINMUX_GPIO26__FUNC_PWM_0].
	 */
	haptic->enable_gpio = devm_gpiod_get(&pdev->dev, "hwen", GPIOD_OUT_LOW);
	if (IS_ERR(haptic->enable_gpio))
		return dev_err_probe(&pdev->dev, PTR_ERR(haptic->enable_gpio),
				     "cannot get the hwen gpio\n");

	haptic->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(haptic->pwm))
		return dev_err_probe(&pdev->dev, PTR_ERR(haptic->pwm),
				     "cannot get the pwm channel\n");

	device_property_read_u32(&pdev->dev, "awinic,vibration-frequency-hz",
				 &freq_hz);
	if (!freq_hz)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "vibration frequency must not be zero\n");
	haptic->period_ns = DIV_ROUND_CLOSEST(NSEC_PER_SEC, freq_hz);

	/* Start from a known state: off, with the actuator released. */
	pwm_init_state(haptic->pwm, &state);
	state.enabled = false;
	ret = pwm_apply_might_sleep(haptic->pwm, &state);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "cannot set the initial pwm state\n");

	haptic->input = devm_input_allocate_device(&pdev->dev);
	if (!haptic->input)
		return -ENOMEM;

	haptic->input->name = "aw8622-haptics";
	haptic->input->id.bustype = BUS_HOST;
	haptic->input->dev.parent = &pdev->dev;
	haptic->input->close = aw8622_close;

	input_set_drvdata(haptic->input, haptic);
	input_set_capability(haptic->input, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(haptic->input, NULL, aw8622_play_effect);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "cannot create the ff device\n");

	ret = input_register_device(haptic->input);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "cannot register the input device\n");

	platform_set_drvdata(pdev, haptic);

	return 0;
}

static int aw8622_suspend(struct device *dev)
{
	struct aw8622_haptic *haptic = dev_get_drvdata(dev);

	cancel_work_sync(&haptic->play_work);
	mutex_lock(&haptic->lock);
	aw8622_stop(haptic);
	mutex_unlock(&haptic->lock);

	return 0;
}

static int aw8622_resume(struct device *dev)
{
	struct aw8622_haptic *haptic = dev_get_drvdata(dev);

	mutex_lock(&haptic->lock);
	if (haptic->level)
		aw8622_start(haptic, haptic->level);
	mutex_unlock(&haptic->lock);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(aw8622_pm_ops, aw8622_suspend, aw8622_resume);

static const struct of_device_id aw8622_of_match[] = {
	{ .compatible = "awinic,aw8622" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw8622_of_match);

static struct platform_driver aw8622_driver = {
	.probe = aw8622_probe,
	.driver = {
		.name = "aw8622-haptics",
		.of_match_table = aw8622_of_match,
		.pm = pm_sleep_ptr(&aw8622_pm_ops),
	},
};
module_platform_driver(aw8622_driver);

MODULE_AUTHOR("MediaTek Inc.");
MODULE_DESCRIPTION("Awinic AW8622 LRA haptics driver");
MODULE_LICENSE("GPL");
