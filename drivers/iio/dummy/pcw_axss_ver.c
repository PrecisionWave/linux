// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Bitstream timestamp readout device
 *
 * Copyright 2024 PrecisionWave AG
 *
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
 #include <linux/iio/sysfs.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

enum attributes{
	BITSTREAM_TIMESTAMP,
};

struct axss_ver_state {
	void __iomem		*base;
	resource_size_t		length;
	struct resource		*res;
};

static void axss_ver_write(struct axss_ver_state *st, unsigned reg, uint32_t val)
{
	iowrite32(val, st->base + reg);
}

static uint32_t axss_ver_read(struct axss_ver_state *st, unsigned reg)
{
	return ioread32(st->base + reg);
}

static int axss_ver_reg_access(struct iio_dev *indio_dev, u32 reg, u32 writeval, u32 *readval)
{
	struct axss_ver_state *st = iio_priv(indio_dev);
	int ret;

	if(reg > st->length)
		return -EINVAL;

	/* Unaligned access is disallowed */
	if((reg & 0x3) != 0)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);

	if (!readval) {
		axss_ver_write(st, reg, writeval);
		ret = 0;
	} else {
		*readval = axss_ver_read(st, reg);
		ret = 0;
	}

	mutex_unlock(&indio_dev->mlock);
	return ret;
}


static ssize_t axss_ver_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	return -EINVAL;
}

static ssize_t axss_ver_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct axss_ver_state *st = iio_priv(indio_dev);
	int ret = -ENODEV;

	mutex_lock(&indio_dev->mlock);

	switch (this_attr->address) {
	case BITSTREAM_TIMESTAMP: {
		u32 u32val = axss_ver_read(st, 0);
		if (u32val == 0) {
			ret = -EINVAL;
			break;
		}

		// FPGA Bitstream generation Date / time
		// Format:
		//            ddddd_MMMM_yyyyyy_hhhhh_mmmmmm_ssssss
		//   (bit 31)                                       (bit 0)
		// Bits 22..17  yyyyyy = 6 bits to represent years 0-63 (2000 to 2063)
		// Bits 26..23  MMMM   = 4 bits to represent months 1-12 in a year
		// Bits 23..27  ddddd  = 5 bits to represent days 1-31 in a month
		// Bits 16..12  hhhhh  = 5 bits to represent hours 0-23 in a day
		// Bits 11.. 6  mmmmmm = 6 bits to represent minutes 0-59 in an hour
		// Bits  5.. 0  ssssss = 6 bits to represent seconds 0-59 in a minute
		ret = sprintf(buf, "20%02u-%02u-%02u %02u:%02u:%02u\n",
			(u32val >> 17) & 0x3f,	// Year
			(u32val >> 23) & 0x0f,	// Month
			(u32val >> 27) & 0x1f,	// Day
			(u32val >> 12) & 0x1f,	// hour
			(u32val >> 6) & 0x3f,	// minute
			(u32val >> 0) & 0x3f);	// second
		} break;

	default:
		ret = -ENODEV;
		break;
	}

	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static IIO_DEVICE_ATTR(bitstream_timestamp, S_IRUGO,
			axss_ver_show,
			axss_ver_store,
			BITSTREAM_TIMESTAMP);

static struct attribute *axss_ver_attributes[] = {
	&iio_dev_attr_bitstream_timestamp.dev_attr.attr,
	NULL
};

static const struct attribute_group axss_ver_attribute_group = {
	.attrs = axss_ver_attributes,
};

static const struct iio_info axss_ver_info = {
	.attrs = &axss_ver_attribute_group,
	.debugfs_reg_access = &axss_ver_reg_access,
};

/* Match table for of_platform binding */
static const struct of_device_id axss_ver_of_match[] = {
	{ .compatible = "pcw,axss-ver", },
	{ }
};
MODULE_DEVICE_TABLE(of, axss_ver_of_match);

static int axss_ver_probe(struct platform_device *pdev)
{
	struct axss_ver_state *st;
	struct iio_dev *indio_dev;
	int ret;
	u32 timestamp = 0;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->base = devm_ioremap_resource(&pdev->dev, st->res);
	if (IS_ERR(st->base))
		return PTR_ERR(st->base);
	st->length = st->res->end - st->res->start;

	indio_dev->dev.parent = &pdev->dev;
	if (pdev->dev.of_node)
		indio_dev->name = pdev->dev.of_node->name;
	else
		indio_dev->name = "axss-ver";
	indio_dev->info = &axss_ver_info;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret < 0)
		return ret;

	timestamp = axss_ver_read(st, 0);
	dev_info(&pdev->dev, "Bitstream timestamp is 0x%08x (20%02u-%02u-%02u %02u:%02u:%02u)",
		timestamp,
		(timestamp >> 17) & 0x3f,	// Year
		(timestamp >> 23) & 0x0f,	// Month
		(timestamp >> 27) & 0x1f,	// Day
		(timestamp >> 12) & 0x1f,	// hour
		(timestamp >> 6) & 0x3f,	// minute
		(timestamp >> 0) & 0x3f);	// second

	return 0;
}

static struct platform_driver axss_ver_driver = {
	.driver = {
		.name = "axss_ver",
		.of_match_table = axss_ver_of_match,
	},
	.probe		= axss_ver_probe,
};
module_platform_driver(axss_ver_driver);

MODULE_AUTHOR("Philipp Diethelm <philipp.diethelm@precisionwave.com>");
MODULE_DESCRIPTION("PCW bitstream timestamp version readout device");
MODULE_LICENSE("Dual BSD/GPL");
