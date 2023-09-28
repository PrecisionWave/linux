/*
 * PE4312 SPI Digital Variable Attenuation
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


struct pe4312_state {
	struct spi_device	*spi;
	struct regulator	*reg;
	struct gpio_desc	*reset_gpio;
	uint8_t				gain;
	const char*			unique_id;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	unsigned char		data[2] ____cacheline_aligned;
};

static int pe4312_write(struct iio_dev *indio_dev, uint8_t val)
{
	struct pe4312_state *st = iio_priv(indio_dev);
	int ret;

	st->data[0] = val;

	ret = spi_write(st->spi, st->data, indio_dev->num_channels);
	if (ret < 0)
		dev_err(&indio_dev->dev, "write failed (%d)", ret);

	return ret;
}

static int pe4312_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct pe4312_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);

	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		/* Values in dB */
		if(st->gain == 127){
			*val = -100;
			*val2 = 0;
		}else{
			*val = -(st->gain - 128)/2;
			*val2 = -((st->gain - 128)%2)*500000;
		}

		ret = IIO_VAL_INT_PLUS_MICRO_DB;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
};

static int pe4312_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct pe4312_state *st = iio_priv(indio_dev);
	int ret;

	val2 = abs(val2);
	if((val<-31 || val>0) && val!=-100)
		return -EINVAL;
	if(val2!=0 && val2!=500000)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if(val == 0)
			st->gain = (2*val2)/1000000 + 128;
		else if (val == -100)
			st->gain = 127;
		else
			st->gain = 2*(-val) + (2*val2)/1000000 + 128;
		ret = pe4312_write(indio_dev, st->gain);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const struct iio_info pe4312_info = {
	.read_raw = &pe4312_read_raw,
	.write_raw = &pe4312_write_raw,
};

#define PE4312_CHAN(_channel) {\
	.type = IIO_VOLTAGE,\
	.output = 1,\
	.indexed = 1,\
	.channel = _channel,\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),\
}

static const struct iio_chan_spec pe4312_channels[] = {				// add more channels here if desired
	PE4312_CHAN(0),
};

static int pe4312_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct device_node *np = spi->dev.of_node;
	struct pe4312_state *st;
	int ret;

	if (!np)
		return -ENODEV;

	dev_dbg(&spi->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));		// alloc iio device (contain the spi device and enough space for the private data)
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);										// link pointer to private data

	st->reg = devm_regulator_get(&spi->dev, "vcc");					// add regulator
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			return ret;
	}

	spi_set_drvdata(spi, indio_dev);								// do the following:   spi->dev->driver_data = indio_dev
	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;


	if(of_property_read_string(np, "optional,unique-id", &st->unique_id))
		st->unique_id = "";	// default

	/* try to get unique id */
	if(!strlen(st->unique_id)){
		pr_warning("PE4312 >> unique-id not found! check devicetree ..\n");
		indio_dev->name = np->name;									// set non-unique id of the device
	}
	else if(strlen(st->unique_id) > 59){
		pr_warning("PE4312 >> unique-id is too long! check devicetree ..\n");
		indio_dev->name = np->name;									// set non-unique id of the device
	}
	else
		indio_dev->name = st->unique_id;							// set unique id of the device


	indio_dev->channels = pe4312_channels;							// link channels structure
	indio_dev->num_channels = ARRAY_SIZE(pe4312_channels);			// define number of channels available
	indio_dev->info = &pe4312_info;									// link callbacks and constant info from driver
	indio_dev->modes = INDIO_DIRECT_MODE;							// define operating modes supported by device


//	ret = pe4312_write(indio_dev, 0);								// initialize to min attenuation
//	if (ret < 0)
//		goto error_disable_reg;
//	pe4312_write_raw(indio_dev, &indio_dev->channels[0], 0, 0, IIO_CHAN_INFO_HARDWAREGAIN);
//	pe4312_write_raw(indio_dev, &indio_dev->channels[0], -31, 500000, IIO_CHAN_INFO_HARDWAREGAIN);
	pe4312_write_raw(indio_dev, &indio_dev->channels[0], -100, 0, IIO_CHAN_INFO_HARDWAREGAIN);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return ret;
}

static int pe4312_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct pe4312_state *st = iio_priv(indio_dev);
	struct regulator *reg = st->reg;

	iio_device_unregister(indio_dev);

	if (!IS_ERR(reg))
		regulator_disable(reg);

	return 0;
}

static const struct spi_device_id pe4312_id[] = {
	{"pe4312", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, pe4312_id);

static struct spi_driver pe4312_driver = {
	.driver = {
		.name	= KBUILD_MODNAME,
	},
	.probe		= pe4312_probe,
	.remove		= pe4312_remove,
	.id_table	= pe4312_id,
};

module_spi_driver(pe4312_driver);

MODULE_AUTHOR("Cyril Zwahlen <zwahlen@precisionwave.ch>");
MODULE_DESCRIPTION("Peregrine Semiconductor 4312 Attenuator");
MODULE_LICENSE("GPL v2");
