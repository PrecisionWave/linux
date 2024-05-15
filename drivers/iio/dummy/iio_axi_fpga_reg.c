// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * TDD HDL CORE driver
 *
 * Copyright 2016 Analog Devices Inc.
 *
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

struct iio_axi_fpga_reg_state {
	void __iomem		*base;
	resource_size_t		length;
	struct resource		*res;
	struct mutex		lock;
};

static int iio_axi_fpga_reg_reg_access(struct iio_dev *indio_dev, u32 reg, u32 writeval, u32 *readval)
{
	struct iio_axi_fpga_reg_state *st = iio_priv(indio_dev);
	int ret;

	if(reg > st->length)
		return -EINVAL;

	/* Unaligned access is disallowed */
	if((reg & 0x3) != 0)
		return -EINVAL;

	mutex_lock(&st->lock);

	if (!readval) {
		iowrite32(writeval, st->base + reg);
		ret = 0;
	} else {
		*readval = ioread32(st->base + reg);
		ret = 0;
	}

	mutex_unlock(&st->lock);
	return ret;
}

static const struct iio_info iio_axi_fpga_reg_info = {
	.debugfs_reg_access = &iio_axi_fpga_reg_reg_access,
};


/* Match table for of_platform binding */
static const struct of_device_id iio_axi_fpga_reg_of_match[] = {
	{ .compatible = "pcw,iio-axi-fpga-reg", .data = 0},
	{ }
};
MODULE_DEVICE_TABLE(of, iio_axi_fpga_reg_of_match);

static int iio_axi_fpga_reg_probe(struct platform_device *pdev)
{
	struct iio_axi_fpga_reg_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->base = devm_ioremap_resource(&pdev->dev, st->res);
	if (IS_ERR(st->base))
		return PTR_ERR(st->base);
	st->length = st->res->end - st->res->start;

	mutex_init(&st->lock);

	indio_dev->dev.parent = &pdev->dev;
	if (pdev->dev.of_node)
		indio_dev->name = pdev->dev.of_node->name;
	else
		indio_dev->name = "iio-axi-fpga-reg";
	indio_dev->info = &iio_axi_fpga_reg_info;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret < 0)
		return ret;

	dev_info(&pdev->dev, "PCW AXI FPGA REG");

	return 0;
}

static struct platform_driver iio_axi_fpga_reg_driver = {
	.driver = {
		.name = "iio_axi_fpga_reg",
		.of_match_table = iio_axi_fpga_reg_of_match,
	},
	.probe		= iio_axi_fpga_reg_probe,
};
module_platform_driver(iio_axi_fpga_reg_driver);

MODULE_AUTHOR("Philipp Diethelm <philipp.diethelm@precisionwave.com>");
MODULE_DESCRIPTION("PCW IIO AXI FPGA register access");
MODULE_LICENSE("Dual BSD/GPL");
