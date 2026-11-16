// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek SCP sensorhub: one IIO device per sensor.
 *
 * The shape follows the two upstream examples that already do this through a
 * remote processor: cros_ec_sensors builds the channels from the sensor type
 * with a shared scale and sampling frequency
 * (drivers/iio/common/cros_ec_sensors/cros_ec_sensors.c:240-275), and it pushes
 * every sample from wherever the EC's data arrives with
 * iio_push_to_buffers_with_timestamp()
 * (cros_ec_sensors_core.c:200-208, :730). The cezanne port does the same for
 * this very hub (cezanne-linux-port/linux/drivers/scp/sensor/scp_iio.c:43-67 for
 * the descriptors and the scales, :250-262 for the scale callback).
 *
 * The samples arrive unsolicited from the SCP, so the buffer needs no trigger of
 * its own: the client pushes each frame as it drains the ring, and the frame's
 * own timestamp goes with it. What the buffer's lifetime controls is the sensor:
 * enabling a buffer ACTIVATEs the sensor on the hub and disabling it switches the
 * sensor off again, which is what keeps the hub from sampling when nothing reads.
 */
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "mtk-sensorhub.h"
#include "mtk-sensorhub-scale.h"

/* a frame's payload is little-endian s32 words: up to three of them */
#define MTK_SH_MAX_CHANNELS	3

struct mtk_sh_iio {
	const struct mtk_sh_sensor *sensor;
	struct mtk_sh *sh;
	struct iio_chan_spec chans[MTK_SH_MAX_CHANNELS];
};

static const char * const mtk_sh_axis_name[MTK_SH_MAX_CHANNELS] = {
	"x", "y", "z",
};

static int mtk_sh_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct mtk_sh_iio *priv = iio_priv(indio_dev);
	s32 sval, sval2;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		ret = mtk_sh_scale(priv->sensor->type, &sval, &sval2);
		if (ret < 0)
			return ret;
		*val = sval;
		*val2 = sval2;
		return ret;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = priv->sensor->rate_hz;
		*val2 = 0;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int mtk_sh_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct mtk_sh_iio *priv = iio_priv(indio_dev);
	struct mtk_sh *sh = priv->sh;
	u32 delay_ms;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;
	if (val <= 0)
		return -EINVAL;

	/* the hub takes a period in milliseconds, the way SET_DELAY carries it */
	delay_ms = DIV_ROUND_CLOSEST_ULL(1000000000ULL, (u64)val * 1000000ULL);
	if (!delay_ms)
		delay_ms = 1;

	priv->sensor->rate_hz = val;
	return mtk_sh_set_delay(sh, priv->sensor->type, delay_ms);
}

static const struct iio_info mtk_sh_info = {
	.read_raw = mtk_sh_read_raw,
	.write_raw = mtk_sh_write_raw,
};

/*
 * The hub only samples what is enabled, so the buffer is the switch: preenable
 * ACTIVATEs the sensor (SCP_sensorHub.h:296-304) and sets its first period,
 * postdisable ACTIVATEs it off. Both run from the process context of the
 * buffer's ioctls, so the IPI's wait is fine here.
 */
static int mtk_sh_buffer_preenable(struct iio_dev *indio_dev)
{
	struct mtk_sh_iio *priv = iio_priv(indio_dev);
	int ret;

	ret = mtk_sh_activate(priv->sh, priv->sensor->type, true);
	if (ret)
		return ret;

	return mtk_sh_set_delay(priv->sh, priv->sensor->type,
				DIV_ROUND_CLOSEST_ULL(1000000000ULL,
						      (u64)priv->sensor->rate_hz
						      * 1000000ULL));
}

static int mtk_sh_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct mtk_sh_iio *priv = iio_priv(indio_dev);

	return mtk_sh_activate(priv->sh, priv->sensor->type, false);
}

static const struct iio_buffer_setup_ops mtk_sh_buffer_ops = {
	.preenable = mtk_sh_buffer_preenable,
	.postdisable = mtk_sh_buffer_postdisable,
};

static int mtk_sh_build_channels(struct mtk_sh_iio *priv, unsigned int *n)
{
	const struct mtk_sh_sensor *sensor = priv->sensor;
	unsigned int i, chans = 1;
	enum iio_chan_type type;

	switch (sensor->type) {
	case MTK_SH_TYPE_ACCEL:
		type = IIO_ACCEL;
		chans = MTK_SH_MAX_CHANNELS;
		break;
	case MTK_SH_TYPE_GYRO:
		type = IIO_ANGL_VEL;
		chans = MTK_SH_MAX_CHANNELS;
		break;
	case MTK_SH_TYPE_MAG:
		type = IIO_MAGN;
		chans = MTK_SH_MAX_CHANNELS;
		break;
	case MTK_SH_TYPE_ORIENTATION:
		type = IIO_ROT;
		chans = MTK_SH_MAX_CHANNELS;
		break;
	case MTK_SH_TYPE_LIGHT:
		type = IIO_LIGHT;
		break;
	case MTK_SH_TYPE_PROXIMITY:
		type = IIO_PROXIMITY;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < chans; i++) {
		struct iio_chan_spec *chan = &priv->chans[i];

		chan->type = type;
		chan->channel = i;
		chan->scan_index = i;
		chan->scan_type.sign = 's';
		chan->scan_type.realbits = 32;
		chan->scan_type.storagebits = 32;
		chan->scan_type.endianness = IIO_LE;
		chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
		chan->info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SCALE) |
						BIT(IIO_CHAN_INFO_SAMP_FREQ);
		chan->info_mask_shared_by_all_available =
			BIT(IIO_CHAN_INFO_SAMP_FREQ);
		if (chans > 1) {
			chan->modified = 1;
			chan->channel2 = IIO_MOD_X + i;
			chan->datasheet_name = mtk_sh_axis_name[i];
		}
	}

	*n = chans;
	return 0;
}

int mtk_sh_iio_register(struct mtk_sh *sh)
{
	unsigned int i;
	int ret;

	for (i = 0; i < sh->num_sensors; i++) {
		struct mtk_sh_sensor *sensor = &sh->sensors[i];
		struct mtk_sh_iio *priv;
		struct iio_dev *indio;
		unsigned int nchans = 0;
		char *name;

		indio = devm_iio_device_alloc(sh->dev, sizeof(*priv));
		if (!indio)
			return -ENOMEM;

		priv = iio_priv(indio);
		priv->sensor = sensor;
		priv->sh = sh;

		ret = mtk_sh_build_channels(priv, &nchans);
		if (ret)
			return ret;

		name = devm_kasprintf(sh->dev, GFP_KERNEL, "mtk-sensorhub-%s",
				      sensor->name);
		if (!name)
			return -ENOMEM;

		indio->name = name;
		indio->info = &mtk_sh_info;
		indio->modes = INDIO_BUFFER_TRIGGERED;
		indio->channels = priv->chans;
		indio->num_channels = nchans;

		/*
		 * No trigger of its own: the frames come from the SCP, and the
		 * client pushes them as it drains the ring (cros_ec_sensors
		 * does the same from the EC's data path,
		 * cros_ec_sensors_core.c:200-208).
		 */
		ret = devm_iio_triggered_buffer_setup(sh->dev, indio, NULL, NULL,
						      &mtk_sh_buffer_ops);
		if (ret)
			return ret;

		ret = devm_iio_device_register(sh->dev, indio);
		if (ret)
			return ret;

		sensor->indio = indio;
		dev_info(sh->dev, "iio device %s created\n", name);
	}

	return 0;
}

/*
 * Push one frame into the buffer of its sensor. The payload is already the
 * little-endian s32 words the scan type describes; proximity is inverted because
 * the hub reports 0 for near and IIO wants a bigger value for nearer.
 */
void mtk_sh_iio_push(struct mtk_sh *sh, const struct mtk_sh_frame *frame)
{
	unsigned int i;

	for (i = 0; i < sh->num_sensors; i++) {
		struct mtk_sh_sensor *sensor = &sh->sensors[i];
		s32 sample[MTK_SH_MAX_CHANNELS] = { 0 };
		size_t words, w;

		if (sensor->type != frame->type || !sensor->indio)
			continue;
		if (!iio_buffer_enabled(sensor->indio))
			return;

		words = frame->payload_len / sizeof(s32);
		if (words > MTK_SH_MAX_CHANNELS)
			words = MTK_SH_MAX_CHANNELS;
		for (w = 0; w < words; w++)
			sample[w] = (s32)get_unaligned_le32(frame->payload + 4 * w);
		if (frame->type == MTK_SH_TYPE_PROXIMITY)
			sample[0] = sample[0] ? 0 : 1;

		iio_push_to_buffers_with_timestamp(sensor->indio, sample,
						   frame->timestamp);
		return;
	}
}
