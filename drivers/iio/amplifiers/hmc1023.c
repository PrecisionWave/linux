/*
 * hmc1023 SPI Dual-Digital Variable Gain Amplifier (VGA)
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/bitrev.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

enum hmc1023_type {
	ID_HMC1023
};

struct hmc1023_state {
	struct spi_device	*spi;
	struct regulator		*reg;
	struct gpio_desc		*reset_gpio;
	enum hmc1023_type	type;
	unsigned char fc; 
	unsigned char g;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	unsigned char		data[4] ____cacheline_aligned;
};

static int hmc1023_write(struct iio_dev *indio_dev,
			unsigned char fc, unsigned char g)
{
	struct hmc1023_state *st = iio_priv(indio_dev);
	int ret;

	fc = fc & 0xF;

	st->data[0] = 0;
	st->data[1] = fc >> 2;
	st->data[2] = ((fc & 0x3) << 6) | (g << 4);
	st->data[3] = (0x2 << 3) | 0x5;


	ret = spi_write(st->spi, st->data, 4);
	if (ret < 0)
		dev_err(&indio_dev->dev, "write failed (%d)", ret);

	return ret;
}

static int hmc1023_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct hmc1023_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:

		/* Values in dB */
		*val = st->g * 10;

		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:

		/* Values in Hz */
		switch (st->fc) {
		case 0:
			*val = 5000000;
			break;
		case 1:
			*val = 7000000;
			break;
		case 2:
			*val = 10000000;
			break;
		case 3:
			*val = 14000000;
			break;
		case 4:
			*val = 20000000;
			break;
		case 5:
			*val = 28000000;
			break;
		case 6:
			*val = 40000000;
			break;
		case 7:
			*val = 50000000;
			break;
		case 8:
			*val = 72000000;
			break;
		}

		ret = IIO_VAL_INT;
		break;

	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
};

static int hmc1023_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct hmc1023_state *st = iio_priv(indio_dev);
	int ret;
	/* Values in Hz */

	mutex_lock(&indio_dev->mlock);
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if(val >= 5)		
			st->g = 1;
		else
			st->g = 0;
		ret = hmc1023_write(indio_dev, st->fc, st->g);
		break;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		if(val <= 5000000)		
			st->fc = 0;
		else if(val <= 7000000)		
			st->fc = 1;
		else if(val <= 10000000)		
			st->fc = 2;
		else if(val <= 14000000)		
			st->fc = 3;
		else if(val <= 20000000)		
			st->fc = 4;
		else if(val <= 28000000)		
			st->fc = 5;
		else if(val <= 40000000)		
			st->fc = 6;
		else if(val <= 50000000)		
			st->fc = 7;
		else
			st->fc = 8;
		ret = hmc1023_write(indio_dev, st->fc, st->g);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const struct iio_info hmc1023_info = {
	.read_raw = &hmc1023_read_raw,
	.write_raw = &hmc1023_write_raw,
};

static const struct iio_chan_spec hmc1023_channels[] = {
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),
	}, {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
	}
};

static int hmc1023_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct hmc1023_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->reg = devm_regulator_get(&spi->dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			return ret;
	}

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;

	/* try to get a unique name */
	if (spi->dev.platform_data)
		indio_dev->name = spi->dev.platform_data;
	else if (spi->dev.of_node)
		indio_dev->name = spi->dev.of_node->name;
	else
		indio_dev->name = spi_get_device_id(spi)->name;

	st->type = spi_get_device_id(spi)->driver_data;
	switch (st->type) {
	case ID_HMC1023:
		indio_dev->channels = hmc1023_channels;
		indio_dev->num_channels = ARRAY_SIZE(hmc1023_channels);
		break;
	default:
		dev_err(&spi->dev, "Invalid device ID\n");
		return -EINVAL;
	}

	indio_dev->info = &hmc1023_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = hmc1023_write(indio_dev, 0 , 0);
	if (ret < 0)
		goto error_disable_reg;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return ret;
}

static int hmc1023_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct hmc1023_state *st = iio_priv(indio_dev);
	struct regulator *reg = st->reg;

	iio_device_unregister(indio_dev);

	if (!IS_ERR(reg))
		regulator_disable(reg);

	return 0;
}

static const struct spi_device_id hmc1023_id[] = {
	{"hmc1023", ID_HMC1023},
	{}
};
MODULE_DEVICE_TABLE(spi, hmc1023_id);

static struct spi_driver hmc1023_driver = {
	.driver = {
		.name	= KBUILD_MODNAME,
	},
	.probe		= hmc1023_probe,
	.remove		= hmc1023_remove,
	.id_table	= hmc1023_id,
};

module_spi_driver(hmc1023_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("Analog Devices HMC1023 adjustable lowpass filter");
MODULE_LICENSE("GPL v2");
