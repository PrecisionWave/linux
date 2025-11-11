// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * DRAS CPRI port access
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
#include <linux/pgtable.h>
#include <linux/init.h>
#include <linux/stat.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/buffer.h>

#include "dras_cpri.h"

/* Device and char device-related information */
static dev_t dras_cpri_devt;
static struct class *dras_cpri_class = NULL;
static int dras_cpri_minor_count = 0;

// Internal limits
#define DRAS_CPRI_DEV_MAX 16
#define DRAS_CPRI_MMAP_PORT_COUNT_MAX 16

struct dras_cpri_reg {
	struct resource *res;
	void __iomem *iomem;
};

struct dras_cpri_priv {
	struct platform_device *pdev;
	struct device *dev;
	struct cdev cdev;
	struct dras_cpri_reg *xlnx_regs;
	struct dras_cpri_reg *pcw_regs;
	struct dras_cpri_reg portid_reg;
	int minor;
	int port_count;
};

static struct dras_cpri_priv *dras_cpri_devices;
static int dras_cpri_devices_max = DRAS_CPRI_DEV_MAX;
static int dras_cpri_port_count_max = DRAS_CPRI_MMAP_PORT_COUNT_MAX;

#define DRAS_CPRI_MMAP_XLNX_PORT_REG_START (DRAS_CPRI_MMAP_XLNX_PORT_REG)
#define DRAS_CPRI_MMAP_XLNX_PORT_REG_END                                       \
	(DRAS_CPRI_MMAP_XLNX_PORT_REG_START + dras_cpri_port_count_max)

#define DRAS_CPRI_MMAP_PCW_PORT_REG_START (DRAS_CPRI_MMAP_PCW_PORT_REG)
#define DRAS_CPRI_MMAP_PCW_PORT_REG_END                                        \
	(DRAS_CPRI_MMAP_PCW_PORT_REG_START + dras_cpri_port_count_max)

static int dras_cpri_mmap_portid_reg(struct dras_cpri_priv *priv,
				     struct vm_area_struct *vma)
{
	size_t len;
	unsigned long vm_pgoff;

	len = vma->vm_end - vma->vm_start;
	if (len > PAGE_SIZE)
		return -EINVAL;

	vm_pgoff = vma->vm_pgoff;
	vma->vm_pgoff = 0;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    priv->portid_reg.res->start >> PAGE_SHIFT,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		vma->vm_pgoff = vm_pgoff;
		return -EAGAIN;
	}

	vma->vm_pgoff = vm_pgoff;
	return 0;
}

static int dras_cpri_mmap_xlnx_reg(struct dras_cpri_priv *priv,
				   struct vm_area_struct *vma)
{
	size_t len;
	unsigned long vm_pgoff;
	int index;

	if (vma->vm_pgoff < DRAS_CPRI_MMAP_XLNX_PORT_REG_START)
		return -EINVAL;
	if (vma->vm_pgoff >= DRAS_CPRI_MMAP_XLNX_PORT_REG_END)
		return -EINVAL;

	index = vma->vm_pgoff - DRAS_CPRI_MMAP_XLNX_PORT_REG_START;
	if (index >= dras_cpri_port_count_max)
		return -EINVAL;
	if (index >= priv->port_count)
		return -EINVAL;

	len = vma->vm_end - vma->vm_start;
	if (len > PAGE_SIZE)
		return -EINVAL;

	vm_pgoff = vma->vm_pgoff;
	vma->vm_pgoff = 0;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    (priv->xlnx_regs[index].res->start) >> PAGE_SHIFT,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		vma->vm_pgoff = vm_pgoff;
		return -EAGAIN;
	}

	vma->vm_pgoff = vm_pgoff;
	return 0;
}

static int dras_cpri_mmap_pcw_reg(struct dras_cpri_priv *priv,
				  struct vm_area_struct *vma)
{
	size_t len;
	unsigned long vm_pgoff;
	int index;

	if (vma->vm_pgoff < DRAS_CPRI_MMAP_PCW_PORT_REG_START)
		return -EINVAL;
	if (vma->vm_pgoff >= DRAS_CPRI_MMAP_PCW_PORT_REG_END)
		return -EINVAL;

	index = vma->vm_pgoff - DRAS_CPRI_MMAP_PCW_PORT_REG_START;
	if (index >= dras_cpri_port_count_max)
		return -EINVAL;
	if (index >= priv->port_count)
		return -EINVAL;

	len = vma->vm_end - vma->vm_start;
	if (len > PAGE_SIZE)
		return -EINVAL;

	vm_pgoff = vma->vm_pgoff;
	vma->vm_pgoff = 0;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    (priv->pcw_regs[index].res->start) >> PAGE_SHIFT,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		vma->vm_pgoff = vm_pgoff;
		return -EAGAIN;
	}

	vma->vm_pgoff = vm_pgoff;
	return 0;
}

static int dras_cpri_mmap(struct file *filep, struct vm_area_struct *vma)
{
	struct dras_cpri_priv *priv = filep->private_data;

	// dev_info(priv->dev, "mmap for vma->vm_pgoff %lx\n", vma->vm_pgoff);

	if (vma->vm_pgoff == DRAS_CPRI_MMAP_PORTID_REG)
		return dras_cpri_mmap_portid_reg(priv, vma);

	if ((vma->vm_pgoff >= DRAS_CPRI_MMAP_XLNX_PORT_REG_START) &&
	    (vma->vm_pgoff < DRAS_CPRI_MMAP_XLNX_PORT_REG_END))
		return dras_cpri_mmap_xlnx_reg(priv, vma);

	if ((vma->vm_pgoff >= DRAS_CPRI_MMAP_PCW_PORT_REG_START) &&
	    (vma->vm_pgoff < DRAS_CPRI_MMAP_PCW_PORT_REG_END))
		return dras_cpri_mmap_pcw_reg(priv, vma);

	return -EINVAL;
}

static int dras_cpri_open(struct inode *inode, struct file *filep)
{
	int minor = iminor(inode);
	if (minor >= dras_cpri_devices_max)
		return -ENXIO;

	filep->private_data = &dras_cpri_devices[minor];

	return 0;
}

static int put_u32(u32 __user *argp, u32 val)
{
	return put_user(val, argp);
}

static long dras_cpri_ioctl(struct file *filep, unsigned int cmd,
			    unsigned long arg)
{
	struct dras_cpri_priv *priv = filep->private_data;
	void __user *argp = (void __user *)arg;
	//int err = -EINVAL;
	//int int_param;

	switch (cmd) {
	case DRAS_CPRI_IOCTL_GET_CPRI_PORT_COUNT:
		return put_u32(argp, priv->port_count);
	default:
		return -EINVAL;
	}
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = dras_cpri_open,
	.mmap = dras_cpri_mmap,
	.unlocked_ioctl = dras_cpri_ioctl,
};

static int dras_cpri_probe(struct platform_device *pdev)
{
	int minor = dras_cpri_minor_count++;
	struct dras_cpri_priv *priv = &dras_cpri_devices[minor];
	int ret = 0;
	int index;

	memset(priv, 0, sizeof(*priv));

	priv->minor = minor;
	priv->pdev = pdev;
	platform_set_drvdata(pdev, priv);

	priv->xlnx_regs = devm_kzalloc(&pdev->dev,
				       array_size(dras_cpri_port_count_max,
						  sizeof(struct dras_cpri_reg)),
				       GFP_KERNEL);
	if (IS_ERR(priv->xlnx_regs))
		return PTR_ERR(priv->xlnx_regs);

	priv->pcw_regs = devm_kzalloc(&pdev->dev,
				      array_size(dras_cpri_port_count_max,
						 sizeof(struct dras_cpri_reg)),
				      GFP_KERNEL);
	if (IS_ERR(priv->pcw_regs))
		return PTR_ERR(priv->pcw_regs);

	priv->portid_reg.iomem = devm_platform_get_and_ioremap_resource(
		pdev, 0, &priv->portid_reg.res);
	if (IS_ERR(priv->portid_reg.iomem))
		return PTR_ERR(priv->portid_reg.iomem);

	for (index = 0; index < dras_cpri_port_count_max; index++) {
		/* no more resources */
		if (pdev->num_resources < 1 + index * 2 + 1)
			break;

		priv->xlnx_regs[index].iomem =
			devm_platform_get_and_ioremap_resource(
				pdev, 1 + index * 2 + 0,
				&priv->xlnx_regs[index].res);
		if (IS_ERR(priv->xlnx_regs[index].iomem)) {
			dev_err(&pdev->dev,
				"Error mapping xlnx resource for port %d\n",
				index);
			return PTR_ERR(priv->xlnx_regs[index].iomem);
		}

		priv->pcw_regs[index].iomem =
			devm_platform_get_and_ioremap_resource(
				pdev, 1 + index * 2 + 1,
				&priv->pcw_regs[index].res);

		if (IS_ERR(priv->pcw_regs[index].iomem)) {
			dev_err(&pdev->dev,
				"Error mapping pcw resource for port %d\n",
				index);
			return PTR_ERR(priv->pcw_regs[index].iomem);
		}

		priv->port_count++;
	}

	if (priv->port_count == 0) {
		dev_err(&pdev->dev, "No CPRI ports detected!");
		return -ENODEV;
	}

	cdev_init(&priv->cdev, &fops);
	priv->cdev.owner = THIS_MODULE;
	ret = cdev_add(&priv->cdev, dras_cpri_devt, 1);
	if (ret < 0)
		return ret;

	priv->dev = device_create(dras_cpri_class, NULL, dras_cpri_devt, priv,
				  "cpri%d", priv->minor);
	if (IS_ERR(priv->dev)) {
		ret = PTR_ERR(priv->dev);
		goto cleanup_cdev;
	}

	dev_info(priv->dev, "DRAS CPRI attached for device %d with %d ports",
		 priv->minor, priv->port_count);
	for (index = 0; index < priv->port_count; index++) {
		if (IS_ERR(priv->xlnx_regs[index].iomem) ||
		    IS_ERR(priv->pcw_regs[index].iomem))
			continue;
		dev_info(priv->dev,
			 "CPRI port %d @ 0x%08x - 0x%08x, %08x - %08x\n", index,
			 priv->xlnx_regs[index].res->start,
			 priv->xlnx_regs[index].res->end,
			 priv->pcw_regs[index].res->start,
			 priv->pcw_regs[index].res->end);
	}

	return 0;

cleanup_cdev:
	cdev_del(&priv->cdev);
	return ret;
}

/* Match table for of_platform binding */
static const struct of_device_id dras_cpri_of_match[] = {
	{
		.compatible = "pcw,dras-cpri",
	},
	{}
};
MODULE_DEVICE_TABLE(of, dras_cpri_of_match);

static struct platform_driver dras_cpri_driver = {
	.driver = {
		.name = "dras_cpri",
		.of_match_table = dras_cpri_of_match,
	},
	.probe		= dras_cpri_probe,
};
module_platform_driver(dras_cpri_driver);

static int __init dras_cpri_init(void)
{
	int ret = 0;

	dras_cpri_devices = vzalloc(array_size(dras_cpri_devices_max,
					       sizeof(struct dras_cpri_priv)));
	if (!dras_cpri_devices)
		return -ENOMEM;

	dras_cpri_class = class_create(THIS_MODULE, "dras_cpri");
	if (IS_ERR(dras_cpri_class)) {
		ret = PTR_ERR(dras_cpri_class);
		goto cleanup_devices;
	}

	ret = alloc_chrdev_region(&dras_cpri_devt, 0, dras_cpri_devices_max,
				  "cpri");
	if (ret < 0)
		goto cleanup_class;

	dras_cpri_minor_count = 0;
	return 0;

cleanup_class:
	class_destroy(dras_cpri_class);
cleanup_devices:
	vfree(dras_cpri_devices);
	return ret;
}
module_init(dras_cpri_init);

static void __exit dras_cpri_exit(void)
{
	unregister_chrdev_region(dras_cpri_devt, dras_cpri_devices_max);
	class_destroy(dras_cpri_class);
	vfree(dras_cpri_devices);
}
module_exit(dras_cpri_exit);

module_param(dras_cpri_devices_max, int, 0);
module_param(dras_cpri_port_count_max, int, 0);

MODULE_AUTHOR("Philipp Diethelm <philipp.diethelm@precisionwave.com>");
MODULE_DESCRIPTION("PCW DRAS CPRI MMIO driver");
MODULE_LICENSE("Dual BSD/GPL");
