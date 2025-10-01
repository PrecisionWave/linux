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
#include <linux/pgtable.h>
#include <linux/init.h>
#include <linux/stat.h>

#include "dexter_apu.h"

/* Device and char device-related information */
static dev_t dexter_apu_devt;
static struct class *dexter_apu_class = NULL;
static int dexter_apu_count = 0;

// APU controls
#define APU_CTRL_LENGTH 0x30000U
#define APU_CTRL_SRAM_OFFSET 0x00000U
#define APU_CTRL_SRAM_LENGTH 0x04000U
#define APU_CTRL_GPIO_OFFSET 0x10000U
#define APU_CTRL_MBOX_OFFSET 0x20000U

struct dexter_apu_priv {
	struct platform_device *pdev;
	struct device *device;
	struct cdev cdev;
	struct resource *res;
	int minor;
	// APU MMIO registers
	phys_addr_t reg_addr;
	void __iomem *reg;
	resource_size_t reg_length;
	// DMA memory for APU DDR
	size_t apu_ddr_size;
	dma_addr_t apu_ddr_addr;
	void *apu_ddr;
};

static struct dexter_apu_priv *dexter_apu_devices;
static int dexter_apu_devices_max = DEXTER_APU_DEV_MAX;

static void dexter_apu_reset(struct dexter_apu_priv *priv, int assert_reset)
{
	if (assert_reset) {
		// sleep
		iowrite32(0, priv->reg + APU_CTRL_GPIO_OFFSET + 0x0);
		// assert reset
		iowrite32(1, priv->reg + APU_CTRL_GPIO_OFFSET + 0x8);
	} else {
		// assert reset
		iowrite32(0, priv->reg + APU_CTRL_GPIO_OFFSET + 0x8);
		// wakeup
		iowrite32(1, priv->reg + APU_CTRL_GPIO_OFFSET + 0x0);
	}
}

static int dexter_apu_mmap_regs(struct dexter_apu_priv *priv,
				struct vm_area_struct *vma)
{
	size_t len;
	unsigned long vm_pgoff;

	if (priv->reg_addr & ~PAGE_MASK)
		return -ENODEV;

	len = vma->vm_end - vma->vm_start;
	if (len > priv->reg_length)
		return -EINVAL;

	vm_pgoff = vma->vm_pgoff;
	vma->vm_pgoff = 0;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, priv->reg_addr >> PAGE_SHIFT,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		vma->vm_pgoff = vm_pgoff;
		return -EAGAIN;
	}

	vma->vm_pgoff = vm_pgoff;
	return 0;
}

static int dexter_apu_mmap_sram(struct dexter_apu_priv *priv,
				struct vm_area_struct *vma)
{
	size_t len;
	unsigned long vm_pgoff;

	if ((priv->reg_addr + APU_CTRL_SRAM_OFFSET) & ~PAGE_MASK)
		return -ENODEV;

	len = vma->vm_end - vma->vm_start;
	if (len > APU_CTRL_SRAM_LENGTH)
		return -EINVAL;

	vm_pgoff = vma->vm_pgoff;
	vma->vm_pgoff = 0;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    (priv->reg_addr + APU_CTRL_SRAM_OFFSET) >>
				    PAGE_SHIFT,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		vma->vm_pgoff = vm_pgoff;
		return -EAGAIN;
	}

	vma->vm_pgoff = vm_pgoff;
	return 0;
}

static int dexter_apu_mmap_apu_ddr(struct dexter_apu_priv *priv,
				   struct vm_area_struct *vma)
{
	unsigned long vm_pgoff;
	int ret;
	size_t len = vma->vm_end - vma->vm_start;

	if (!priv->apu_ddr)
		return -ENODEV;

	if (len > priv->apu_ddr_size)
		return -EINVAL;

	vm_pgoff = vma->vm_pgoff;
	vma->vm_pgoff = 0;
	ret = dma_mmap_coherent(priv->device, vma, priv->apu_ddr,
				priv->apu_ddr_addr, len);

	vma->vm_pgoff = vm_pgoff;
	return ret;
}

static int dexter_apu_mmap(struct file *filep, struct vm_area_struct *vma)
{
	struct dexter_apu_priv *priv = filep->private_data;

	switch (vma->vm_pgoff) {
	case DEXTER_APU_MMAP_REGS:
		return dexter_apu_mmap_regs(priv, vma);

	case DEXTER_APU_MMAP_DDR:
		return dexter_apu_mmap_apu_ddr(priv, vma);

	case DEXTER_APU_MMAP_SRAM:
		return dexter_apu_mmap_sram(priv, vma);

	default:
		break;
	}

	return -EINVAL;
}

static int dexter_apu_open(struct inode *inode, struct file *filep)
{
	int minor = iminor(inode);
	if (minor >= DEXTER_APU_DEV_MAX)
		return -ENXIO;

	filep->private_data = &dexter_apu_devices[minor];

	return 0;
}

static int put_u32(u32 __user *argp, u32 val)
{
	return put_user(val, argp);
}

static long dexter_apu_ioctl(struct file *filep, unsigned int cmd,
			     unsigned long arg)
{
	struct dexter_apu_priv *priv = filep->private_data;
	void __user *argp = (void __user *)arg;
	int err = -EINVAL;
	int int_param;

	// dev_info(priv->device, "ioctl: cmd = %08x\n", cmd);

	switch (cmd) {
	case DEXTER_APU_IOCTL_APU_RESET:
		err = get_user(int_param, (int __user *)arg);
		if (err)
			return err;
		dexter_apu_reset(priv, int_param);
		return 0;

	case DEXTER_APU_IOCTL_GET_SRAM_SIZE:
		return put_u32(argp, APU_CTRL_SRAM_LENGTH);

	case DEXTER_APU_IOCTL_GET_DDR_SIZE:
		return put_u32(argp, priv->apu_ddr_size);

	case DEXTER_APU_IOCTL_GET_DDR_PHYS:
		return put_u32(argp, priv->apu_ddr_addr);
	}

	return -EINVAL;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = dexter_apu_open,
	.mmap = dexter_apu_mmap,
	.unlocked_ioctl = dexter_apu_ioctl,
};

/* Match table for of_platform binding */
static const struct of_device_id dexter_apu_of_match[] = {
	{
		.compatible = "pcw,dexter-apu",
	},
	{}
};
MODULE_DEVICE_TABLE(of, dexter_apu_of_match);

static int dexter_apu_probe(struct platform_device *pdev)
{
	int minor = dexter_apu_count++;
	struct dexter_apu_priv *priv = &dexter_apu_devices[minor];
	int ret = 0;

	memset(priv, 0, sizeof(*priv));

	priv->minor = minor;
	priv->pdev = pdev;
	priv->apu_ddr_size = DEXTER_APU_DDR_SIZE_DEFAULT;
	platform_set_drvdata(pdev, priv);

	priv->reg = devm_platform_get_and_ioremap_resource(pdev, 0, &priv->res);
	if (IS_ERR(priv->reg))
		return PTR_ERR(priv->reg);
	priv->reg_addr = priv->res->start;
	priv->reg_length = priv->res->end - priv->res->start + 1;

	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	priv->apu_ddr = dma_alloc_coherent(&pdev->dev, priv->apu_ddr_size,
					   &priv->apu_ddr_addr, GFP_KERNEL);
	if (IS_ERR(priv->apu_ddr))
		return PTR_ERR(priv->apu_ddr);

	cdev_init(&priv->cdev, &fops);
	priv->cdev.owner = THIS_MODULE;
	ret = cdev_add(&priv->cdev, dexter_apu_devt, 1);
	if (ret < 0)
		goto cleanup_dma;

	priv->device = device_create(dexter_apu_class, NULL, dexter_apu_devt,
				     priv, "apu%d", priv->minor);
	if (IS_ERR(priv->device)) {
		ret = PTR_ERR(priv->device);
		goto cleanup_cdev;
	}

	dev_info(priv->device, "Dexter APU attached for device %d.",
		 priv->minor);
	dev_info(priv->device, "Phys reg: 0x%08x - 0x%08x", priv->reg_addr,
		 priv->reg_addr + priv->reg_length - 1);
	dev_info(priv->device, "Phys DMA: 0x%08x - 0x%08x", priv->apu_ddr_addr,
		 priv->apu_ddr_addr + priv->apu_ddr_size - 1);
	return 0;

cleanup_cdev:
	cdev_del(&priv->cdev);
cleanup_dma:
	dma_free_coherent(&pdev->dev, priv->apu_ddr_size, priv->apu_ddr,
			  priv->apu_ddr_addr);
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
	int ret = 0;

	dexter_apu_devices = vzalloc(array_size(
		dexter_apu_devices_max, sizeof(struct dexter_apu_priv)));
	if (!dexter_apu_devices)
		return -ENOMEM;

	dexter_apu_class = class_create(THIS_MODULE, "dexter_apu");
	if (IS_ERR(dexter_apu_class)) {
		ret = PTR_ERR(dexter_apu_class);
		goto cleanup_devices;
	}

	ret = alloc_chrdev_region(&dexter_apu_devt, 0, dexter_apu_devices_max,
				  "apu");
	if (ret < 0)
		goto cleanup_class;

	dexter_apu_count = 0;
	return 0;

cleanup_class:
	class_destroy(dexter_apu_class);
cleanup_devices:
	vfree(dexter_apu_devices);
	return ret;
}
module_init(dexter_apu_init);

static void __exit dexter_apu_exit(void)
{
	unregister_chrdev_region(dexter_apu_devt, dexter_apu_devices_max);
	class_destroy(dexter_apu_class);
	vfree(dexter_apu_devices);
}
module_exit(dexter_apu_exit);

module_param(dexter_apu_devices_max, int, 0);

MODULE_AUTHOR("Philipp Diethelm <philipp.diethelm@precisionwave.com>");
MODULE_DESCRIPTION("PCW Dexter APU driver");
MODULE_LICENSE("Dual BSD/GPL");
