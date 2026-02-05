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
#include <linux/delay.h>

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
	struct device *dev;
	struct cdev cdev;
	int minor;
	// APU MMIO registers
	struct resource *reg_res;
	void __iomem *reg_virt;
	// APU MMIO registers #2 (optional)
	struct resource *reg2_res;
	void __iomem *reg2_virt;
	// DMA memory for APU DDR
	size_t apu_ddr_size;
	dma_addr_t apu_ddr_addr;
	void *apu_ddr;
};

static int dexter_apu_devices_max = DEXTER_APU_DEV_MAX;
static int dexter_apu_register_class(void);

static void dexter_apu_reset(struct dexter_apu_priv *priv, int assert_reset)
{
	if (assert_reset) {
		// sleep
		iowrite32(0, priv->reg_virt + APU_CTRL_GPIO_OFFSET + 0x0);
		msleep(1);
		// assert reset
		iowrite32(1, priv->reg_virt + APU_CTRL_GPIO_OFFSET + 0x8);
		msleep(1);

		// sync memory
		dma_sync_single_for_cpu(priv->dev, priv->apu_ddr_addr,
					priv->apu_ddr_size, DMA_FROM_DEVICE);
	} else {
		// sync memory
		dma_sync_single_for_device(priv->dev, priv->apu_ddr_addr,
					   priv->apu_ddr_size, DMA_TO_DEVICE);
		msleep(1);

		// de-assert reset
		iowrite32(0, priv->reg_virt + APU_CTRL_GPIO_OFFSET + 0x8);
		msleep(1);

		// wakeup
		iowrite32(1, priv->reg_virt + APU_CTRL_GPIO_OFFSET + 0x0);
	}
}

static void dexter_apu_start_from(struct dexter_apu_priv *priv,
				  uint32_t start_address)
{
	// sync memory
	dma_sync_single_for_device(priv->dev, priv->apu_ddr_addr,
				   priv->apu_ddr_size, DMA_TO_DEVICE);
	msleep(1);

	// assert reset
	iowrite32(1, priv->reg_virt + APU_CTRL_GPIO_OFFSET + 0x8);
	msleep(1);

	// some implementations use reset_vector instead of wakeup
	iowrite32(start_address, priv->reg_virt + APU_CTRL_GPIO_OFFSET + 0x0);
	msleep(1);

	// de-assert reset
	iowrite32(0, priv->reg_virt + APU_CTRL_GPIO_OFFSET + 0x8);
	msleep(1);
}

static int dexter_apu_mmap_page(struct dexter_apu_priv *priv,
				struct vm_area_struct *vma,
				const resource_size_t res_start,
				const size_t res_len)
{
	size_t len;
	unsigned long vm_pgoff;

	len = vma->vm_end - vma->vm_start;
	if (len > res_len)
		return -EINVAL;

	vm_pgoff = vma->vm_pgoff;
	vma->vm_pgoff = 0;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, res_start >> PAGE_SHIFT,
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
	ret = dma_mmap_coherent(priv->dev, vma, priv->apu_ddr,
				priv->apu_ddr_addr, len);

	vma->vm_pgoff = vm_pgoff;
	return ret;
}

static int dexter_apu_mmap(struct file *filep, struct vm_area_struct *vma)
{
	struct dexter_apu_priv *priv = filep->private_data;

	switch (vma->vm_pgoff) {
	case DEXTER_APU_MMAP_REGS:
		return dexter_apu_mmap_page(
			priv, vma, priv->reg_res->start,
			(priv->reg_res->end - priv->reg_res->start + 1));

	case DEXTER_APU_MMAP_REGS2:
		if (!priv->reg2_virt)
			return -EINVAL;

		return dexter_apu_mmap_page(
			priv, vma, priv->reg2_res->start,
			(priv->reg2_res->end - priv->reg2_res->start + 1));

	case DEXTER_APU_MMAP_DDR:
		return dexter_apu_mmap_apu_ddr(priv, vma);

	case DEXTER_APU_MMAP_SRAM:
		return dexter_apu_mmap_page(
			priv, vma, priv->reg_res->start + APU_CTRL_SRAM_OFFSET,
			APU_CTRL_SRAM_LENGTH);

	default:
		break;
	}

	return -EINVAL;
}

static int dexter_apu_open(struct inode *inode, struct file *filep)
{
	struct dexter_apu_priv *priv;
	priv = container_of(inode->i_cdev, struct dexter_apu_priv, cdev);

	filep->private_data = priv;

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
	uint32_t u32_param;

	switch (cmd) {
	case DEXTER_APU_IOCTL_APU_RESET:
		err = get_user(int_param, (int __user *)arg);
		if (err)
			return err;
		dexter_apu_reset(priv, int_param);
		return 0;

	case DEXTER_APU_IOCTL_APU_START:
		err = get_user(u32_param, (int __user *)arg);
		if (err)
			return err;
		dexter_apu_start_from(priv, u32_param);
		return 0;

	case DEXTER_APU_IOCTL_GET_SRAM_SIZE:
		return put_u32(argp, APU_CTRL_SRAM_LENGTH);

	case DEXTER_APU_IOCTL_GET_DDR_SIZE:
		return put_u32(argp, priv->apu_ddr_size);

	case DEXTER_APU_IOCTL_GET_DDR_PHYS:
		return put_u32(argp, priv->apu_ddr_addr);

	case DEXTER_APU_IOCTL_SYNC_FOR_CPU:
		err = get_user(int_param, (int __user *)arg);
		if (err)
			return err;
		switch (int_param) {
		case DEXTER_APU_DMA_FROM_DEVICE:
			dma_sync_single_for_cpu(priv->dev, priv->apu_ddr_addr,
						priv->apu_ddr_size,
						DMA_FROM_DEVICE);
			break;
		case DEXTER_APU_DMA_TO_DEVICE:
			dma_sync_single_for_cpu(priv->dev, priv->apu_ddr_addr,
						priv->apu_ddr_size,
						DMA_FROM_DEVICE);
			break;
		case DEXTER_APU_DMA_BIDIR:
			dma_sync_single_for_cpu(priv->dev, priv->apu_ddr_addr,
						priv->apu_ddr_size,
						DMA_BIDIRECTIONAL);
			break;
		}
		return 0;

	case DEXTER_APU_IOCTL_SYNC_FOR_DEVICE:
		err = get_user(int_param, (int __user *)arg);
		if (err)
			return err;
		switch (int_param) {
		case DEXTER_APU_DMA_FROM_DEVICE:
			dma_sync_single_for_device(priv->dev,
						   priv->apu_ddr_addr,
						   priv->apu_ddr_size,
						   DMA_FROM_DEVICE);
			break;
		case DEXTER_APU_DMA_TO_DEVICE:
			dma_sync_single_for_device(priv->dev,
						   priv->apu_ddr_addr,
						   priv->apu_ddr_size,
						   DMA_FROM_DEVICE);
			break;
		case DEXTER_APU_DMA_BIDIR:
			dma_sync_single_for_device(priv->dev,
						   priv->apu_ddr_addr,
						   priv->apu_ddr_size,
						   DMA_BIDIRECTIONAL);
			break;
		}
		return 0;
	}

	return -EINVAL;
}

static ssize_t dexter_apu_read(struct file *filep, char __user *to,
			       size_t count, loff_t *ppos)
{
	struct dexter_apu_priv *priv = filep->private_data;
	loff_t available = priv->apu_ddr_size;
	loff_t pos = *ppos;
	size_t ret;

	dev_info(priv->dev, "read: count=%zu, pos=%lld", count, pos);

	if (pos < 0)
		return -EINVAL;
	if (pos >= available || !count)
		return 0;
	if (count > available - pos)
		count = available - pos;

	dma_sync_single_for_cpu(priv->dev, priv->apu_ddr_addr + pos, count,
				DMA_FROM_DEVICE);
	ret = copy_to_user(to, priv->apu_ddr + pos, count);
	if (ret == count)
		return -EFAULT;
	count -= ret;
	*ppos = pos + count;

	return count;
}

static ssize_t dexter_apu_write(struct file *filep, const char __user *from,
				size_t count, loff_t *ppos)
{
	struct dexter_apu_priv *priv = filep->private_data;
	loff_t available = priv->apu_ddr_size;
	loff_t pos = *ppos;
	size_t res;

	dev_info(priv->dev, "write: count=%zu, pos=%lld", count, pos);

	if (pos < 0)
		return -EINVAL;
	if (pos >= available || !count)
		return 0;
	if (count > available - pos)
		count = available - pos;
	res = copy_from_user(priv->apu_ddr + pos, from, count);
	if (res == count)
		return -EFAULT;

	dma_sync_single_for_device(priv->dev, priv->apu_ddr_addr + pos, count,
				   DMA_TO_DEVICE);

	count -= res;
	*ppos = pos + count;

	return count;
}

static loff_t dexter_apu_llseek(struct file *filep, loff_t offset, int whence)
{
	struct dexter_apu_priv *priv = filep->private_data;
	return fixed_size_llseek(filep, offset, whence, priv->apu_ddr_size);
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = dexter_apu_open,
	.mmap = dexter_apu_mmap,
	.write = dexter_apu_write,
	.llseek = dexter_apu_llseek,
	.read = dexter_apu_read,
	.unlocked_ioctl = dexter_apu_ioctl,
};

static int dexter_apu_probe(struct platform_device *pdev)
{
	int minor = dexter_apu_count++;
	struct dexter_apu_priv *priv;
	int ret = 0;

	ret = dexter_apu_register_class();
	if (ret < 0)
		return ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	priv->minor = minor;
	priv->pdev = pdev;
	priv->apu_ddr_size = DEXTER_APU_DDR_SIZE_DEFAULT;
	platform_set_drvdata(pdev, priv);

	priv->reg_virt =
		devm_platform_get_and_ioremap_resource(pdev, 0, &priv->reg_res);
	if (IS_ERR(priv->reg_virt))
		return PTR_ERR(priv->reg_virt);

	priv->reg2_virt = devm_platform_get_and_ioremap_resource(
		pdev, 1, &priv->reg2_res);
	if (IS_ERR(priv->reg2_virt))
		priv->reg2_virt = NULL;

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

	priv->dev = device_create(dexter_apu_class, NULL, dexter_apu_devt, priv,
				  "apu%d", priv->minor);
	if (IS_ERR(priv->dev)) {
		ret = PTR_ERR(priv->dev);
		goto cleanup_cdev;
	}

	dev_info(priv->dev, "Dexter APU attached for device %d.", priv->minor);
	dev_info(priv->dev, "Phys reg: %pR", priv->reg_res);
	dev_info(priv->dev, "Phys reg2: %pR", priv->reg2_res);
	dev_info(priv->dev, "Phys DMA: 0x%08x - 0x%08x", priv->apu_ddr_addr,
		 priv->apu_ddr_addr + priv->apu_ddr_size - 1);

	return 0;

cleanup_cdev:
	cdev_del(&priv->cdev);
cleanup_dma:
	dma_free_coherent(&pdev->dev, priv->apu_ddr_size, priv->apu_ddr,
			  priv->apu_ddr_addr);
	return ret;
}

/* Match table for of_platform binding */
static const struct of_device_id dexter_apu_of_match[] = {
	{
		.compatible = "pcw,dexter-apu",
	},
	{}
};
MODULE_DEVICE_TABLE(of, dexter_apu_of_match);

static struct platform_driver dexter_apu_driver = {
	.driver = {
		.name = "dexter_apu",
		.of_match_table = dexter_apu_of_match,
	},
	.probe		= dexter_apu_probe,
};
module_platform_driver(dexter_apu_driver);

static int dexter_apu_register_class(void)
{
	int ret = 0;

	if (dexter_apu_class)
		return 0;

	dexter_apu_class = class_create(THIS_MODULE, "dexter_apu");
	if (IS_ERR(dexter_apu_class))
		return PTR_ERR(dexter_apu_class);

	ret = alloc_chrdev_region(&dexter_apu_devt, 0, dexter_apu_devices_max,
				  "apu");
	if (ret < 0)
		goto cleanup_class;

	dexter_apu_count = 0;
	return 0;

cleanup_class:
	class_destroy(dexter_apu_class);
	return ret;
}

static int __init dexter_apu_init(void)
{
	return dexter_apu_register_class();
}
module_init(dexter_apu_init);

static void __exit dexter_apu_exit(void)
{
	unregister_chrdev_region(dexter_apu_devt, dexter_apu_devices_max);
	class_destroy(dexter_apu_class);
}
module_exit(dexter_apu_exit);

module_param(dexter_apu_devices_max, int, 0);

MODULE_AUTHOR("Philipp Diethelm <philipp.diethelm@precisionwave.com>");
MODULE_DESCRIPTION("PCW Dexter APU driver");
MODULE_LICENSE("Dual BSD/GPL");
