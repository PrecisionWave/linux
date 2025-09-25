// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Dexter APU management driver
 *
 * Copyright 2025 PrecisionWave AG
 *
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mm_types.h>

/* Device and char device-related information */
static dev_t dexter_apu_devt;
static struct class *dexter_apu_class = NULL;
static int dexter_apu_count = 0;
#define DEXTER_APU_DEV_MAX 16
#define APU_DMA_SIZE (1 * 1024 * 1024)

struct dexter_apu_priv {
	struct platform_device *pdev;
	struct device *device;
	struct cdev cdev;
	void __iomem *base;
	resource_size_t length;
	struct resource *res;
	size_t apu_dma_size;
	dma_addr_t apu_dma_addr;
	void *apu_dma;
};

/* Match table for of_platform binding */
static const struct of_device_id dexter_apu_of_match[] = {
	{
		.compatible = "pcw,dexter-apu",
	},
	{}
};
MODULE_DEVICE_TABLE(of, dexter_apu_of_match);

static const struct file_operations fops = {
	.owner = THIS_MODULE,
};

static int dexter_apu_probe(struct platform_device *pdev)
{
	struct dexter_apu_priv *priv;
	int ret = 0;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->pdev = pdev;
	priv->apu_dma_size = APU_DMA_SIZE;

	priv->base =
		devm_platform_get_and_ioremap_resource(pdev, 0, &priv->res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);
	priv->length = priv->res->end - priv->res->start;

	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	priv->apu_dma = dma_alloc_coherent(&pdev->dev, priv->apu_dma_size,
					   &priv->apu_dma_addr, GFP_KERNEL);
	if (IS_ERR(priv->apu_dma))
		return PTR_ERR(priv->apu_dma);

	cdev_init(&priv->cdev, &fops);
	priv->cdev.owner = THIS_MODULE;
	ret = cdev_add(&priv->cdev, dexter_apu_devt, 1);
	if (ret < 0)
		goto cleanup_dma;

	priv->device = device_create(dexter_apu_class, NULL, dexter_apu_devt,
				     priv, "apu%d", dexter_apu_count++);
	if (IS_ERR(priv->device)) {
		ret = PTR_ERR(priv->device);
		goto cleanup_cdev;
	}

	dev_info(priv->device, "Dexter APU attached for device %d.",
		 dexter_apu_count);
	dev_info(priv->device, "DMA addr: 0x%08x - 0x%08x", priv->apu_dma_addr,
		 priv->apu_dma_addr + priv->apu_dma_size - 1);
	return 0;

cleanup_cdev:
	cdev_del(&priv->cdev);
cleanup_dma:
	dma_free_coherent(&pdev->dev, priv->apu_dma_size, priv->apu_dma,
			  priv->apu_dma_addr);
	return ret;
}

static struct platform_driver dexter_apu_driver = {
	.driver = {
		.name = "dexter_apu",
		.of_match_table = dexter_apu_of_match,
	},
	.probe		= dexter_apu_probe,
};
module_platform_driver(dexter_apu_driver);

static int __init dexter_apu_init(void)
{
	int ret = -EINVAL;

	dexter_apu_class = class_create(THIS_MODULE, "dexter_apu");
	if (IS_ERR(dexter_apu_class))
		return PTR_ERR(dexter_apu_class);

	ret = alloc_chrdev_region(&dexter_apu_devt, 0, DEXTER_APU_DEV_MAX,
				  "apu");
	if (ret < 0)
		goto cleanup_class;

	dexter_apu_count = 0;
	return 0;

cleanup_class:
	class_destroy(dexter_apu_class);
	return ret;
}
module_init(dexter_apu_init);

static void __exit dexter_apu_exit(void)
{
	unregister_chrdev_region(dexter_apu_devt, DEXTER_APU_DEV_MAX);
	class_destroy(dexter_apu_class);
}
module_exit(dexter_apu_exit);

MODULE_AUTHOR("Philipp Diethelm <philipp.diethelm@precisionwave.com>");
MODULE_DESCRIPTION("PCW Dexter APU driver");
MODULE_LICENSE("Dual BSD/GPL");
