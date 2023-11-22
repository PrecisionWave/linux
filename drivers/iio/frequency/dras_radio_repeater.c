/*
 * DRAS DSP COREFPGA Module
 * DRAS FM TETRA ADC DAC DSP Core Driver
 *
 * Copyright 2023 PrecisionWave AG
 *
 * Licensed under the GPL-2.

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


#define DRIVER_NAME			"dras-radio-repeater"
#define NB_OF_TETRA_CHANNELS		8

// global attributes
#define ADDR_MAXGAIN			16*4
#define ADDR_TARGET_PWR			17*4
#define ADDR_SQUELCH			18*4
#define ADDR_CHANNEL_EN			19*4
#define ADDR_DSP_VERSION		20*4

// channel attributes
#define ADDR_RSSI(x)			(0+x)*4 // 16bit LSB first rssi, second 16bit second rssi
#define ADDR_BEST_SOURCE(x)		(8+x)*4


// expands to:
//   CH0_<REG> + CHANNEL
// example:
//   REG_CH(0, REG_GAIN_TX1)
//     expansion:
//     CH0_REG_GAIN_TX1 + 0
#define REG_CH(CHANNEL, REG) \
	(CH0_##REG + CHANNEL)

// expands to:
//   CH0_<REG>,
//   CH1_<REG>,
//    ::   ::
//   CH31_<REG>
// example:
//   REG_ALL_CH(REG_GAIN_TX1)
//     expansion:
//     CH0_REG_GAIN_TX1
//     CH1_REG_GAIN_TX1,
//      ::   ::
//     CH31_REG_GAIN_TX1
#define REG_ALL_CH(REG) \
	CH0_##REG, \
	CH1_##REG, \
	CH2_##REG, \
	CH3_##REG, \
	CH4_##REG, \
	CH5_##REG, \
	CH6_##REG, \
	CH7_##REG

// expands to:
//   static IIO_DEVICE_ATTR(ch0_<ATTR>, <RW>, <SHOW>, <STORE>, CH0_<REG>);
//   static IIO_DEVICE_ATTR(ch1_<ATTR>, <RW>, <SHOW>, <STORE>, CH1_<REG>);
//    ::   ::
//   static IIO_DEVICE_ATTR(ch31_<ATTR>, <RW>, <SHOW>, <STORE>, CH31_<REG>);
// example:
//   IIO_DEVICE_ATTR_ALL_CH(gain_tx1, S_IRUGO | S_IWUSR, dras_radio_repeater_show, dras_radio_repeater_store, REG_GAIN_TX1)
//     expansion:
//     static IIO_DEVICE_ATTR(ch0_gain_tx1, S_IRUGO | S_IWUSR, dras_radio_repeater_show, dras_radio_repeater_store, CH0_REG_GAIN_TX1);
//     static IIO_DEVICE_ATTR(ch1_gain_tx1, S_IRUGO | S_IWUSR, dras_radio_repeater_show, dras_radio_repeater_store, CH1_REG_GAIN_TX1);
//      ::   ::
//     static IIO_DEVICE_ATTR(ch31_gain_tx1, S_IRUGO | S_IWUSR, dras_radio_repeater_show, dras_radio_repeater_store, CH31_REG_GAIN_TX1);
#define IIO_DEVICE_ATTR_ALL_CH(ATTR, RW, SHOW, STORE, REG) \
	static IIO_DEVICE_ATTR(ch0_##ATTR, RW, SHOW, STORE, CH0_##REG); \
	static IIO_DEVICE_ATTR(ch1_##ATTR, RW, SHOW, STORE, CH1_##REG); \
	static IIO_DEVICE_ATTR(ch2_##ATTR, RW, SHOW, STORE, CH2_##REG); \
	static IIO_DEVICE_ATTR(ch3_##ATTR, RW, SHOW, STORE, CH3_##REG); \
	static IIO_DEVICE_ATTR(ch4_##ATTR, RW, SHOW, STORE, CH4_##REG); \
	static IIO_DEVICE_ATTR(ch5_##ATTR, RW, SHOW, STORE, CH5_##REG); \
	static IIO_DEVICE_ATTR(ch6_##ATTR, RW, SHOW, STORE, CH6_##REG); \
	static IIO_DEVICE_ATTR(ch7_##ATTR, RW, SHOW, STORE, CH7_##REG);

// expands to:
//   &iio_dev_attr_ch0_<ATTR>.dev_attr.attr,
//   &iio_dev_attr_ch1_<ATTR>.dev_attr.attr,
//    ::   ::
//   &iio_dev_attr_ch31_<ATTR>.dev_attr.attr,
// example:
//   IIO_ATTR_ALL_CH(gain_tx1)
//     expansion:
//     &iio_dev_attr_ch0_gain_tx1.dev_attr.attr,
//     &iio_dev_attr_ch1_gain_tx1.dev_attr.attr,
//      ::   ::
//     &iio_dev_attr_ch31_gain_tx1.dev_attr.attr
#define IIO_ATTR_ALL_CH(ATTR) \
	&iio_dev_attr_ch0_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch1_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch2_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch3_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch4_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch5_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch6_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch7_##ATTR.dev_attr.attr

enum chan_num{
	REG_ALL_CH(REG_RSSI),	// being expanded for all channels
	REG_ALL_CH(REG_CHANNEL_ENABLE),	// being expanded for all channels
	REG_ALL_CH(REG_BEST_SOURCE),	// being expanded for all channels
	REG_DSP_VERSION,
	REG_TARGET_POWER,
	REG_SQUELCH,
	REG_MAX_GAIN
};

struct dras_radio_repeater_state {
	struct iio_info		iio_info;
	void __iomem		*regs;
	struct mutex		lock;
};

static void dras_radio_repeater_write(struct dras_radio_repeater_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 dras_radio_repeater_read(struct dras_radio_repeater_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int dras_radio_repeater_write_raw(struct iio_dev *indio_dev,
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

static int dras_radio_repeater_read_raw(struct iio_dev *indio_dev,
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

static ssize_t dras_radio_repeater_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dras_radio_repeater_state *st = iio_priv(indio_dev);
	long val;
	int ret;
	u32 temp32;
	u64 temp64;
	u32 ch;
	int match;

	/* convert to long
	 * auto-detect decimal,
	 * octal (beginning with 0) and
	 * hexadecimal (beginning with 0x)
	 */
	ret = kstrtol(buf, 0, &val);
	if (ret < 0)
		return ret;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<NB_OF_TETRA_CHANNELS; ch++){
		if((u32)this_attr->address == REG_CH(ch, REG_CHANNEL_ENABLE)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = dras_radio_repeater_read(st, ADDR_CHANNEL_EN) & ~(1<<ch);
			temp32 += ((uint32_t)val)<<ch;
			dras_radio_repeater_write(st, ADDR_CHANNEL_EN, temp32);
			break;
		}
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		return ret ? ret : len;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_TARGET_POWER:
		if(val<0 || val>0xFFFF){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_radio_repeater_read(st, ADDR_TARGET_PWR) & ~0xFFFF;
		temp32 += (u32)val;
		dras_radio_repeater_write(st, ADDR_TARGET_PWR, temp32);
		break;
	case REG_SQUELCH:
		if(val<0 || val>0xFFFFFFF){
			ret = -EINVAL;
			break;
		}
		dras_radio_repeater_write(st, ADDR_SQUELCH, (u32)val);
		break;
	case REG_MAX_GAIN:
		if(val<0 || val>0xFFFFFF){
			ret = -EINVAL;
			break;
		}
		dras_radio_repeater_write(st, ADDR_MAXGAIN, (u32)val);
		break;
	default:
		ret = -ENODEV;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t dras_radio_repeater_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dras_radio_repeater_state *st = iio_priv(indio_dev);
	int val;
	int ret = 0;
	int64_t temp64;
	u32 ch;
	int match;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<NB_OF_TETRA_CHANNELS; ch++){
		if((u32)this_attr->address == REG_CH(ch, REG_RSSI)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_RSSI(ch)) & 0x1FFFFFF;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_CHANNEL_ENABLE)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_CHANNEL_EN) >> ch) & 1;
			break;
		}else if((u32)this_attr->address == REG_CH(ch, REG_BEST_SOURCE)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_BEST_SOURCE(ch)) & 0xF;
		}
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		if(ret==0)
			ret = sprintf(buf, "%d\n", val);
		return ret;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_TARGET_POWER:
		val = dras_radio_repeater_read(st, ADDR_TARGET_PWR);
		break;
	case REG_SQUELCH:
		val = dras_radio_repeater_read(st, ADDR_SQUELCH);
		break;
	case REG_MAX_GAIN:
		val = dras_radio_repeater_read(st, ADDR_MAXGAIN);
		break;
	case REG_DSP_VERSION:
		val = dras_radio_repeater_read(st, ADDR_DSP_VERSION);
		break;
	default:
		ret = -ENODEV;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0)
		ret = sprintf(buf, "%d\n", val);

	return ret;
}

IIO_DEVICE_ATTR_ALL_CH(rssi, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_RSSI);

IIO_DEVICE_ATTR_ALL_CH(channel_enable, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_CHANNEL_ENABLE);

IIO_DEVICE_ATTR_ALL_CH(best_source, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_BEST_SOURCE);

static IIO_DEVICE_ATTR(target_power, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_TARGET_POWER);

static IIO_DEVICE_ATTR(squelch, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_SQUELCH);

static IIO_DEVICE_ATTR(max_gain, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_MAX_GAIN);

static IIO_DEVICE_ATTR(dsp_version, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_DSP_VERSION);


static struct attribute *dras_radio_repeater_attributes[] = {
	IIO_ATTR_ALL_CH(rssi),
	IIO_ATTR_ALL_CH(channel_enable),
	IIO_ATTR_ALL_CH(best_source),
	&iio_dev_attr_dsp_version.dev_attr.attr,
	&iio_dev_attr_target_power.dev_attr.attr,
	&iio_dev_attr_squelch.dev_attr.attr,
	&iio_dev_attr_max_gain.dev_attr.attr,
	NULL,
};


static const struct attribute_group dras_radio_repeater_attribute_group = {
	.attrs = dras_radio_repeater_attributes,
};

static const struct iio_info dras_radio_repeater_info = {
	.read_raw = &dras_radio_repeater_read_raw,
	.write_raw = &dras_radio_repeater_write_raw,
	.attrs = &dras_radio_repeater_attribute_group,
};

static const struct iio_chan_spec dras_radio_repeater_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id dras_radio_repeater_of_match[] = {
	{ .compatible = "fpga,dras-radio-repeater", },
	{ },
};

MODULE_DEVICE_TABLE(of, dras_radio_repeater_of_match);

static int dras_radio_repeater_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct dras_radio_repeater_state *st;
	struct iio_dev *indio_dev;
	int ret; //, i, n;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(dras_radio_repeater_of_match, &pdev->dev);
	if (!id)
		return -ENODEV;

	/* allocate some kernel space for the driver attributes
	 * devm_kzalloc: When the device is detached from the system
	 *               or the driver for the device is unloaded,
	 *               that memory is freed automatically
	 */
	indio_dev = iio_device_alloc(&pdev->dev, sizeof(*st));
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

	indio_dev->name = np->name;
	indio_dev->channels = dras_radio_repeater_channels;
	indio_dev->num_channels = ARRAY_SIZE(dras_radio_repeater_channels);
	indio_dev->info = &dras_radio_repeater_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, indio_dev);
	return 0;

err_iio_device_free:
	iio_device_free(indio_dev);
	return ret;
}

static int dras_radio_repeater_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver dras_radio_repeater_driver = {
	.probe		= dras_radio_repeater_probe,
	.remove		= dras_radio_repeater_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = dras_radio_repeater_of_match,
	},
};

module_platform_driver(dras_radio_repeater_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("DRAS RADIO REPEATER FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
