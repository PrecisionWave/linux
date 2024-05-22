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
#include <linux/clk.h>


#define DRIVER_NAME			"dras-tetra-repeater"
#define NB_OF_TETRA_CHANNELS		16

// global attributes
#define ADDR_DSP_VERSION		16*4
#define ADDR_TARGET_PWR			17*4
#define ADDR_MAXGAIN			18*4
#define ADDR_SQUELCH			19*4
#define ADDR_SUM_RSSI			20*4

// channel attributes
#define ADDR_RSSI(x)			(0+x)*4 // 16bit LSB first rssi, second 16bit second rssi


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
	CH7_##REG, \
	CH8_##REG, \
	CH9_##REG, \
	CH10_##REG, \
	CH11_##REG, \
	CH12_##REG, \
	CH13_##REG, \
	CH14_##REG, \
	CH15_##REG

// expands to:
//   static IIO_DEVICE_ATTR(ch0_<ATTR>, <RW>, <SHOW>, <STORE>, CH0_<REG>);
//   static IIO_DEVICE_ATTR(ch1_<ATTR>, <RW>, <SHOW>, <STORE>, CH1_<REG>);
//    ::   ::
//   static IIO_DEVICE_ATTR(ch31_<ATTR>, <RW>, <SHOW>, <STORE>, CH31_<REG>);
// example:
//   IIO_DEVICE_ATTR_ALL_CH(gain_tx1, S_IRUGO | S_IWUSR, dras_tetra_repeater_show, dras_tetra_repeater_store, REG_GAIN_TX1)
//     expansion:
//     static IIO_DEVICE_ATTR(ch0_gain_tx1, S_IRUGO | S_IWUSR, dras_tetra_repeater_show, dras_tetra_repeater_store, CH0_REG_GAIN_TX1);
//     static IIO_DEVICE_ATTR(ch1_gain_tx1, S_IRUGO | S_IWUSR, dras_tetra_repeater_show, dras_tetra_repeater_store, CH1_REG_GAIN_TX1);
//      ::   ::
//     static IIO_DEVICE_ATTR(ch31_gain_tx1, S_IRUGO | S_IWUSR, dras_tetra_repeater_show, dras_tetra_repeater_store, CH31_REG_GAIN_TX1);
#define IIO_DEVICE_ATTR_ALL_CH(ATTR, RW, SHOW, STORE, REG) \
	static IIO_DEVICE_ATTR(ch0_##ATTR, RW, SHOW, STORE, CH0_##REG); \
	static IIO_DEVICE_ATTR(ch1_##ATTR, RW, SHOW, STORE, CH1_##REG); \
	static IIO_DEVICE_ATTR(ch2_##ATTR, RW, SHOW, STORE, CH2_##REG); \
	static IIO_DEVICE_ATTR(ch3_##ATTR, RW, SHOW, STORE, CH3_##REG); \
	static IIO_DEVICE_ATTR(ch4_##ATTR, RW, SHOW, STORE, CH4_##REG); \
	static IIO_DEVICE_ATTR(ch5_##ATTR, RW, SHOW, STORE, CH5_##REG); \
	static IIO_DEVICE_ATTR(ch6_##ATTR, RW, SHOW, STORE, CH6_##REG); \
	static IIO_DEVICE_ATTR(ch7_##ATTR, RW, SHOW, STORE, CH7_##REG); \
	static IIO_DEVICE_ATTR(ch8_##ATTR, RW, SHOW, STORE, CH8_##REG); \
	static IIO_DEVICE_ATTR(ch9_##ATTR, RW, SHOW, STORE, CH9_##REG); \
	static IIO_DEVICE_ATTR(ch10_##ATTR, RW, SHOW, STORE, CH10_##REG); \
	static IIO_DEVICE_ATTR(ch11_##ATTR, RW, SHOW, STORE, CH11_##REG); \
	static IIO_DEVICE_ATTR(ch12_##ATTR, RW, SHOW, STORE, CH12_##REG); \
	static IIO_DEVICE_ATTR(ch13_##ATTR, RW, SHOW, STORE, CH13_##REG); \
	static IIO_DEVICE_ATTR(ch14_##ATTR, RW, SHOW, STORE, CH14_##REG); \
	static IIO_DEVICE_ATTR(ch15_##ATTR, RW, SHOW, STORE, CH15_##REG);

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
	&iio_dev_attr_ch7_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch8_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch9_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch10_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch11_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch12_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch13_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch14_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch15_##ATTR.dev_attr.attr

enum chan_num{
	REG_ALL_CH(REG_RSSI),	// being expanded for all channels
	REG_DSP_VERSION,
	REG_TARGET_POWER,
	REG_SQUELCH,
	REG_SUM_RSSI,
	REG_MAX_GAIN
};

struct dras_tetra_repeater_state {
	struct iio_info		iio_info;
	void __iomem		*regs;
	struct mutex		lock;

	struct device		*dev;
	struct clk		*adrv_clk;
	struct notifier_block	adrv_clk_rate_change_nb;
	uint32_t		adrv_clk_rate;
};

static void dras_tetra_repeater_write(struct dras_tetra_repeater_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 dras_tetra_repeater_read(struct dras_tetra_repeater_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int dras_tetra_repeater_write_raw(struct iio_dev *indio_dev,
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

static int dras_tetra_repeater_read_raw(struct iio_dev *indio_dev,
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

static ssize_t dras_tetra_repeater_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dras_tetra_repeater_state *st = iio_priv(indio_dev);
	long val;
	int ret;
	u32 temp32;

	/* convert to long
	 * auto-detect decimal,
	 * octal (beginning with 0) and
	 * hexadecimal (beginning with 0x)
	 */
	ret = kstrtol(buf, 0, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&indio_dev->mlock);

	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_TARGET_POWER:
		if(val<0 || val>0xFFFF){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_repeater_read(st, ADDR_TARGET_PWR) & ~0xFFFF;
		temp32 += (u32)val;
		dras_tetra_repeater_write(st, ADDR_TARGET_PWR, temp32);
		break;
	case REG_SQUELCH:
		if(val<0 || val>0xFFFFFFF){
			ret = -EINVAL;
			break;
		}
		dras_tetra_repeater_write(st, ADDR_SQUELCH, (u32)val);
		break;
	case REG_MAX_GAIN:
		if(val<0 || val>0xFFFFFF){
			ret = -EINVAL;
			break;
		}
		dras_tetra_repeater_write(st, ADDR_MAXGAIN, (u32)val);
		break;
	default:
		ret = -ENODEV;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t dras_tetra_repeater_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dras_tetra_repeater_state *st = iio_priv(indio_dev);
	int val;
	int ret = 0;
	u32 ch;
	int match;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<NB_OF_TETRA_CHANNELS; ch++){
		if((u32)this_attr->address == REG_CH(ch, REG_RSSI)){
			match = 1;
			val = dras_tetra_repeater_read(st, ADDR_RSSI(ch)) & 0x1FFFFFF;
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
		val = dras_tetra_repeater_read(st, ADDR_TARGET_PWR);
		break;
	case REG_SQUELCH:
		val = dras_tetra_repeater_read(st, ADDR_SQUELCH);
		break;
	case REG_MAX_GAIN:
		val = dras_tetra_repeater_read(st, ADDR_MAXGAIN);
		break;
	case REG_DSP_VERSION:
		val = dras_tetra_repeater_read(st, ADDR_DSP_VERSION);
		break;
	case REG_SUM_RSSI:
		val = dras_tetra_repeater_read(st, ADDR_SUM_RSSI);
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
			dras_tetra_repeater_show,
			dras_tetra_repeater_store,
			REG_RSSI);

static IIO_DEVICE_ATTR(target_power, S_IRUGO | S_IWUSR,
			dras_tetra_repeater_show,
			dras_tetra_repeater_store,
			REG_TARGET_POWER);

static IIO_DEVICE_ATTR(squelch, S_IRUGO | S_IWUSR,
			dras_tetra_repeater_show,
			dras_tetra_repeater_store,
			REG_SQUELCH);

static IIO_DEVICE_ATTR(max_gain, S_IRUGO | S_IWUSR,
			dras_tetra_repeater_show,
			dras_tetra_repeater_store,
			REG_MAX_GAIN);

static IIO_DEVICE_ATTR(dsp_version, S_IRUGO,
			dras_tetra_repeater_show,
			dras_tetra_repeater_store,
			REG_DSP_VERSION);

static IIO_DEVICE_ATTR(sum_rssi, S_IRUGO,
			dras_tetra_repeater_show,
			dras_tetra_repeater_store,
			REG_SUM_RSSI);


static struct attribute *dras_tetra_repeater_attributes[] = {
	IIO_ATTR_ALL_CH(rssi),
	&iio_dev_attr_dsp_version.dev_attr.attr,
	&iio_dev_attr_target_power.dev_attr.attr,
	&iio_dev_attr_squelch.dev_attr.attr,
	&iio_dev_attr_max_gain.dev_attr.attr,
	&iio_dev_attr_sum_rssi.dev_attr.attr,
	NULL,
};


static const struct attribute_group dras_tetra_repeater_attribute_group = {
	.attrs = dras_tetra_repeater_attributes,
};

static const struct iio_info dras_tetra_repeater_info = {
	.read_raw = &dras_tetra_repeater_read_raw,
	.write_raw = &dras_tetra_repeater_write_raw,
	.attrs = &dras_tetra_repeater_attribute_group,
};

static const struct iio_chan_spec dras_tetra_repeater_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id dras_tetra_repeater_of_match[] = {
	{ .compatible = "fpga,dras-tetra-repeater", },
	{ },
};

MODULE_DEVICE_TABLE(of, dras_tetra_repeater_of_match);

static int adrv_clk_clock_notifier(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct clk_notifier_data *ndata = data;
	struct dras_tetra_repeater_state *st = container_of(nb, struct dras_tetra_repeater_state, adrv_clk_rate_change_nb);

	dev_info(st->dev, "adrv_clk rate change: new rate = %lu Hz\n", ndata->new_rate);

	if (event == POST_RATE_CHANGE) {
		st->adrv_clk_rate = ndata->new_rate;
	}

	return NOTIFY_DONE;
}

static int dras_tetra_repeater_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct dras_tetra_repeater_state *st;
	struct iio_dev *indio_dev;
	int ret; //, i, n;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(dras_tetra_repeater_of_match, &pdev->dev);
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
	st->dev = &pdev->dev;

	st->adrv_clk = devm_clk_get(&pdev->dev, "adrv_clk");
	if (IS_ERR_OR_NULL(st->adrv_clk)) {
		ret = PTR_ERR(st->adrv_clk);
		dev_err(&pdev->dev, "Failed to get ADRV clock (%d)\n", ret);
		goto err_iio_device_free;
	}

	ret = clk_prepare_enable(st->adrv_clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable ADRV clock\n");
		goto err_iio_device_free;
	}

	st->adrv_clk_rate = clk_get_rate(st->adrv_clk);
	if (st->adrv_clk_rate == 0) {
		dev_warn(&pdev->dev, "ADRV clk equal to 0 Hz\n");
		//ret = -EINVAL;
		//goto err_iio_device_free;
	}

	dev_info(&pdev->dev, "ADRV clk rate is %u Hz", st->adrv_clk_rate);

	st->adrv_clk_rate_change_nb.notifier_call = adrv_clk_clock_notifier;
	clk_notifier_register(st->adrv_clk, &st->adrv_clk_rate_change_nb);

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
	indio_dev->channels = dras_tetra_repeater_channels;
	indio_dev->num_channels = ARRAY_SIZE(dras_tetra_repeater_channels);
	indio_dev->info = &dras_tetra_repeater_info;
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

static int dras_tetra_repeater_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver dras_tetra_repeater_driver = {
	.probe		= dras_tetra_repeater_probe,
	.remove		= dras_tetra_repeater_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = dras_tetra_repeater_of_match,
	},
};

module_platform_driver(dras_tetra_repeater_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("DRAS TETRA REPEATER FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
