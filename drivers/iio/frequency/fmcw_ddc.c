/*
 * FMCW DDC COREFPGA Module
 * FMCW digital down converter
 *
 * Copyright 2018 PrecisionWave AG
 *
 * Licensed under the GPL-2.
 * 
 * Device Parameters
 * -----------------
 
 */

#include <linux/module.h>
#include <linux/errno.h> 
#include <linux/platform_device.h> 
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>


#define DRIVER_NAME			"fmcw-ddc"

#define ADDR_VALID_SELECT		0*4
#define ADDR_SKIP_AT_BEGINNING		1*4
#define ADDR_PERIOD			2*4
#define ADDR_FIRST_SKIP			3*4
#define ADDR_OVERFLOWS_TX		4*4
#define ADDR_OVERFLOWS_RX		5*4
#define ADDR_SAMPLES_SNT		6*4
#define ADDR_FRAMES_SNT			7*4
#define ADDR_SAMPLES_RCVD		8*4
#define ADDR_FRAMES_RCVD		9*4
#define ADDR_CHANNEL2_OFFSET		10*4

#define MIN_GAIN		0x0000
#define MAX_GAIN		0xFFFF

enum chan_num{
	CH_VALID_SELECT,
	CH_PERIOD,
	CH_OFFSET,
	CH_FIRST_OFFSET,
	CH_CHANNEL2_OFFSET,
	CH_OVERFLOWS_RX,
	CH_OVERFLOWS_TX
};

struct vbi_fm_dsp_state {
	struct iio_info					iio_info;
	void __iomem					*regs;
	struct mutex					lock;

	uint32_t						fs_adc;
	uint32_t						nb_of_blocks;
	uint32_t						ch_nb;
};

static void vbi_fm_dsp_write(struct vbi_fm_dsp_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 vbi_fm_dsp_read(struct vbi_fm_dsp_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int vbi_fm_dsp_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	int ret;

	mutex_lock(&indio_dev->mlock);

	switch (mask) {
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int vbi_fm_dsp_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long mask)
{
	int ret;

	mutex_lock(&indio_dev->mlock);

	switch (mask) {
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t vbi_fm_dsp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct vbi_fm_dsp_state *st = iio_priv(indio_dev);
	long val;
	int ret;

	/* convert to long
	 * auto-detect decimal,
	 * octal (beginning with 0) and
	 * hexadecimal (beginning with 0x)
	 */
	ret = kstrtol(buf, 0, &val);
	if (ret < 0)
		return ret;

	
	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case CH_VALID_SELECT:
		if(val<0 || val>8){
			ret = -EINVAL;
			break;
		}
		vbi_fm_dsp_write(st, ADDR_VALID_SELECT, (u32)val);
		break;
	case CH_PERIOD:
		if(val<0 || val>8191){
			ret = -EINVAL;
			break;
		}
		vbi_fm_dsp_write(st, ADDR_PERIOD, (u32)val);
		break;
	case CH_OFFSET:
		if(val<0 || val>8191){
			ret = -EINVAL;
			break;
		}
		vbi_fm_dsp_write(st, ADDR_SKIP_AT_BEGINNING, (u32)val);
		break;
	case CH_FIRST_OFFSET:
		vbi_fm_dsp_write(st, ADDR_FIRST_SKIP, (u32)val);
		break;
	case CH_CHANNEL2_OFFSET:
		if(val<0 || val>8191){
			ret = -EINVAL;
			break;
		}
		vbi_fm_dsp_write(st, ADDR_CHANNEL2_OFFSET, (u32)val);
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t vbi_fm_dsp_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct vbi_fm_dsp_state *st = iio_priv(indio_dev);
	u32 val;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);

	switch ((u32)this_attr->address) {
	case CH_VALID_SELECT:
		val = vbi_fm_dsp_read(st, ADDR_VALID_SELECT);
		break;
	case CH_PERIOD:
		val = vbi_fm_dsp_read(st, ADDR_PERIOD);
		break;
	case CH_OFFSET:
		val = vbi_fm_dsp_read(st, ADDR_SKIP_AT_BEGINNING);
		break;
	case CH_FIRST_OFFSET:
		val = vbi_fm_dsp_read(st, ADDR_FIRST_SKIP);
		break;
	case CH_CHANNEL2_OFFSET:
		val = vbi_fm_dsp_read(st, ADDR_CHANNEL2_OFFSET);
		break;
	case CH_OVERFLOWS_RX:
		val = vbi_fm_dsp_read(st, ADDR_OVERFLOWS_RX);
		break;
	case CH_OVERFLOWS_TX:
		val = vbi_fm_dsp_read(st, ADDR_OVERFLOWS_TX);
		break;

	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0)
		ret = sprintf(buf, "%d\n", val);

	return ret;
}


static IIO_DEVICE_ATTR(valid_select, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			CH_VALID_SELECT);

static IIO_DEVICE_ATTR(period, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			CH_PERIOD);

static IIO_DEVICE_ATTR(offset, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			CH_OFFSET);

static IIO_DEVICE_ATTR(first_offset, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			CH_FIRST_OFFSET);

static IIO_DEVICE_ATTR(channel2_offset, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			CH_CHANNEL2_OFFSET);

static IIO_DEVICE_ATTR(overflows_rx, S_IRUGO,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			CH_OVERFLOWS_RX);

static IIO_DEVICE_ATTR(overflows_tx, S_IRUGO,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			CH_OVERFLOWS_TX);



static struct attribute *vbi_fm_dsp_attributes[] = {
	&iio_dev_attr_valid_select.dev_attr.attr,
	&iio_dev_attr_period.dev_attr.attr,
	&iio_dev_attr_offset.dev_attr.attr,
	&iio_dev_attr_first_offset.dev_attr.attr,
	&iio_dev_attr_channel2_offset.dev_attr.attr,
	&iio_dev_attr_overflows_rx.dev_attr.attr,
	&iio_dev_attr_overflows_tx.dev_attr.attr,
	NULL,
};


static const struct attribute_group vbi_fm_dsp_attribute_group = {
	.attrs = vbi_fm_dsp_attributes,
};

static const struct iio_info vbi_fm_dsp_info = {
	.read_raw = &vbi_fm_dsp_read_raw,
	.write_raw = &vbi_fm_dsp_write_raw,
	.attrs = &vbi_fm_dsp_attribute_group,
};

static const struct iio_chan_spec vbi_fm_dsp_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id vbi_fm_dsp_of_match[] = {
	{ .compatible = "fpga,fmcw-ddc", },
	{ },
};

MODULE_DEVICE_TABLE(of, vbi_fm_dsp_of_match);

static int vbi_fm_dsp_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct vbi_fm_dsp_state *st;
	struct iio_dev *indio_dev;
	int ret; //, i, n;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(vbi_fm_dsp_of_match, &pdev->dev);
	if (!id)
		return -ENODEV;

	/* allocate some kernel space for the driver attributes
	 * devm_kzalloc: When the device is detached from the system
	 *               or the driver for the device is unloaded,
	 *               that memory is freed automatically
	 */
	indio_dev = iio_device_alloc(sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

//	st->adc_freq = pdata->adc_freq;

	/* get information about the structure of the device resource,
	 * map device resource to kernel space
	 * devm_ioremap_resource: When the device is detached from the system
	 *                        or the driver for the device is unloaded,
	 *                        that memory is unmapped automatically
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, res);
	if (!st->regs) {
		ret = -ENOMEM;
		goto err_iio_device_free;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->channels = vbi_fm_dsp_channels;
	indio_dev->num_channels = ARRAY_SIZE(vbi_fm_dsp_channels);
	indio_dev->info = &vbi_fm_dsp_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	vbi_fm_dsp_write(st, ADDR_VALID_SELECT, 0);
	vbi_fm_dsp_write(st, ADDR_SKIP_AT_BEGINNING, 0x900);
	vbi_fm_dsp_write(st, ADDR_PERIOD, 0x100);
	vbi_fm_dsp_write(st, ADDR_FIRST_SKIP, 0x823);


	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, indio_dev);
	return 0;

err_iio_device_free:
	iio_device_free(indio_dev);
	return ret;
}

static int vbi_fm_dsp_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver vbi_fm_dsp_driver = {
	.probe		= vbi_fm_dsp_probe,
	.remove		= vbi_fm_dsp_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vbi_fm_dsp_of_match,
	},
};

module_platform_driver(vbi_fm_dsp_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("FMCW DDC FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);

