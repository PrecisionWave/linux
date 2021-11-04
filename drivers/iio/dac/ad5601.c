/*
 * Generic Device Driver
 *
 * Copyright 2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/gcd.h>
#include <linux/gpio.h>
#include <asm/div64.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include <linux/clk-provider.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>


#define DRIVER_NAME		"ad5601"
#define NAME_(x)		ad5601_##x
#define SPI_BUFFER_SIZE		2	// in bytes

#define RESERVED_SPI		0x200


struct NAME_(platform_data){
	char name[SPI_NAME_SIZE];
};

struct NAME_(state){
	struct spi_device *spi;
	const char* unique_id;

	uint32_t spi_rd_buffer;
	uint32_t spi_wr_buffer;

	uint32_t reserved_spi;
};

static ssize_t NAME_(spi_transfer)(struct iio_dev *indio_dev)
{
	struct NAME_(state) *st = iio_priv(indio_dev);
	int ret;

	struct spi_transfer t = {
	    .tx_buf = &st->spi_wr_buffer,
	    .rx_buf = &st->spi_rd_buffer,
	    .len = SPI_BUFFER_SIZE,
	};

	ret = spi_sync_transfer(st->spi, &t, 1); // write buffer

	return ret;
}

static int NAME_(reg_access)(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct NAME_(state) *st = iio_priv(indio_dev);
	int ret;

	if(reg != 0)
		return -ENODEV;

	if (readval == NULL) {
		st->spi_wr_buffer = cpu_to_be32( (writeval & 0xFFFF) << 16 );
		ret = NAME_(spi_transfer)(indio_dev);
	} else {
		return -ENODEV;
	}

	return ret;
}

static ssize_t NAME_(store)(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct NAME_(state) *st = iio_priv(indio_dev);
	long val;
	int ret = 0;

	ret = kstrtol(buf, 0, &val);
	if (ret < 0)
		return ret;
	
	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case RESERVED_SPI:
		st->reserved_spi = (u32)val;
		st->spi_wr_buffer = cpu_to_be32( (st->reserved_spi & 0xFFFF) << 16 );
		ret = NAME_(spi_transfer)(indio_dev);
		if(ret)
			break;
		st->reserved_spi = ( be32_to_cpu(st->spi_rd_buffer) >> 16);
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t NAME_(show)(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct NAME_(state) *st = iio_priv(indio_dev);
	u32 val;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case RESERVED_SPI:
		val = st->reserved_spi;
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0){
		ret = sprintf(buf, "%d\n", val);
	}
	return ret;
}

static IIO_DEVICE_ATTR(reserved_spi, S_IRUGO | S_IWUSR,
			NAME_(show),
			NAME_(store),
			RESERVED_SPI);

static struct attribute *NAME_(attributes)[] = {
	&iio_dev_attr_reserved_spi.dev_attr.attr,
	NULL
};

static const struct attribute_group NAME_(attribute_group) = {
	.attrs = NAME_(attributes),
};

static const struct iio_info NAME_(info) = {
	.debugfs_reg_access = &NAME_(reg_access),
	.attrs = &NAME_(attribute_group),
};

static int NAME_(probe)(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct device_node *np = spi->dev.of_node;
	struct NAME_(state) *st;
	int ret;

	if (!np)
		return -ENODEV;

	dev_dbg(&spi->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	spi_set_drvdata(spi, indio_dev);

	st = iio_priv(indio_dev);
	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;

	if(of_property_read_string(np, "optional,unique-id", &st->unique_id))
		st->unique_id = "";	// default

	/* try to get unique id */
	if(!strlen(st->unique_id)){
		pr_warning(DRIVER_NAME" >> unique-id not found! check devicetree ..\n");
		indio_dev->name = np->name;									// set non-unique id of the device
	}
	else if(strlen(st->unique_id) > 59){
		pr_warning(DRIVER_NAME" >> unique-id is too long! check devicetree ..\n");
		indio_dev->name = np->name;									// set non-unique id of the device
	}
	else
		indio_dev->name = st->unique_id;							// set unique id of the device

	indio_dev->info = &NAME_(info);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(indio_dev);

	return ret;
}

static int NAME_(remove)(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	iio_device_unregister(indio_dev);
	return 0;
}

static const struct spi_device_id NAME_(id)[] = {
	{DRIVER_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(spi, NAME_(id));

static struct spi_driver NAME_(driver) = {
	.driver = {
		.name	= KBUILD_MODNAME,
		.owner	= THIS_MODULE,
	},
	.probe		= NAME_(probe),
	.remove		= NAME_(remove),
	.id_table	= NAME_(id),
};
module_spi_driver(NAME_(driver));

MODULE_AUTHOR("Cyril Zwahlen <zwahlen@precisionwave.com>");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_LICENSE("GPL v2");
