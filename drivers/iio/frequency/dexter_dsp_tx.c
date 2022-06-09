/*
 * ZEBU FMCW DSP COREFPGA Module
 *
 * Copyright 2022 PrecisionWave AG
 *
 * Licensed under the GPL-2.
 *
 * Device Parameters
 * -----------------
 * frequency: channel frequency in Hz
 * gain_tx: output gain TX 0..65535, 65535=0dB

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

#define DRIVER_NAME			"dexter-dsp-tx"
#define NB_OF_BLOCKS			1
#define DDS_PHASEWIDTH			25


#define ADDR_DSP_VERSION		0*4
#define ADDR_PPS_SETTINGS		1*4
#define ADDR_PPS_ERROR			2*4
#define ADDR_PPS_CNT			3*4
#define ADDR_PPS_DELAY			4*4
#define ADDR_PPS_CLKS_LSB		5*4
#define ADDR_PPS_CLKS_MSB		6*4
#define ADDR_DDS_INC0			16*4
#define ADDR_GAIN_DC0			17*4
#define ADDR_BUFF_UFLOWS0		18*4
#define ADDR_STREAM_START0_LSB		19*4
#define ADDR_STREAM_START0_MSB		20*4
#define ADDR_DDS_INC1			32*4
#define ADDR_GAIN_DC1			33*4
#define ADDR_BUFF_UFLOWS1		34*4
#define ADDR_STREAM_START1_LSB		35*4
#define ADDR_STREAM_START1_MSB		36*4


enum chan_num{
  	REG_FREQUENCY0,
  	REG_GAIN0,
  	REG_DC0,
	REG_STREAM0_START_CLKS,
	REG_BUFFER_UNDERFLOWS0,
  	REG_FREQUENCY1,
  	REG_GAIN1,
  	REG_DC1,
	REG_STREAM1_START_CLKS,
	REG_BUFFER_UNDERFLOWS1,
	REG_PPS_DIRECTION_OUT_N_IN,
	REG_PPS_CLK_ERROR,
	REG_PPS_CLK_ERROR_NS,
	REG_PPS_CLK_ERROR_HZ,
	REG_PPS_CNT,
	REG_PPS_CLKS,
	REG_PPS_REFERENCE_FREQUENCY,
	REG_PPS_DELAY,
	REG_GPSDO_LOCKED,
	REG_DSP_VERSION
};

struct dexter_dsp_tx_state {
	struct iio_info	iio_info;
	void __iomem	*regs;
	struct mutex	lock;
	struct clk		*dac_clk;
	struct notifier_block	clk_rate_change_nb;
	struct device	*dev;
	uint32_t	fs_if_dac;
	bool 		gpsdo_locked;
  	uint32_t	pps_clk_error_ns;
  	uint32_t	pps_clk_error_hz;
};

static void dexter_dsp_tx_write(struct dexter_dsp_tx_state *st, unsigned reg, uint32_t val)
{
	iowrite32(val, st->regs + reg);
}

static uint32_t dexter_dsp_tx_read(struct dexter_dsp_tx_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int dexter_dsp_tx_write_raw(struct iio_dev *indio_dev,
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

static int dexter_dsp_tx_read_raw(struct iio_dev *indio_dev,
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



static ssize_t dexter_dsp_tx_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dexter_dsp_tx_state *st = iio_priv(indio_dev);
	long val;
	int32_t ret;
	int64_t temp64;
	uint64_t tempu64;
	uint32_t temp32;

	unsigned long long readin;

	/* convert to long
	 * auto-detect decimal,
	 * octal (beginning with 0) and
	 * hexadecimal (beginning with 0x)
	 */
	if ((uint32_t)this_attr->address == REG_STREAM0_START_CLKS || (uint32_t)this_attr->address == REG_STREAM1_START_CLKS){
		ret = kstrtoull(buf, 10, &readin);
		if (ret)
			return ret;
	} else {
		ret = kstrtol(buf, 0, &val);
		if (ret < 0)
			return ret;
	}

	mutex_lock(&indio_dev->mlock);
	switch ((uint32_t)this_attr->address) {

	case REG_FREQUENCY0:
    		temp64 = (int64_t)val  << DDS_PHASEWIDTH;
    		temp32 = (int32_t)div_s64(temp64,st->fs_if_dac);
		dexter_dsp_tx_write(st, ADDR_DDS_INC0, temp32);
		break;

	case REG_GAIN0:
		temp32 = dexter_dsp_tx_read(st, ADDR_GAIN_DC0) & 0xFFFF0000;
		temp32 += (uint32_t)val & 0xFFFF;
		dexter_dsp_tx_write(st, ADDR_GAIN_DC0, temp32);
		break;

	case REG_DC0:
		temp32 = dexter_dsp_tx_read(st, ADDR_GAIN_DC0) & 0xFFFF;
		temp32 += (uint32_t)val << 16;
		dexter_dsp_tx_write(st, ADDR_GAIN_DC0, temp32);
		break;

	case REG_STREAM0_START_CLKS:
		tempu64 = ((uint64_t)dexter_dsp_tx_read(st, ADDR_PPS_CLKS_MSB) << 32) + dexter_dsp_tx_read(st, ADDR_PPS_CLKS_LSB);
		if(readin!=0 && (readin < tempu64 || readin > tempu64+20*(uint64_t)st->fs_if_dac)) // start must be in the future and not more than 10s in the future
			ret = -EINVAL;
		temp32 = (uint32_t)readin;
		dexter_dsp_tx_write(st, ADDR_STREAM_START0_LSB, temp32);
		temp32 = (uint32_t)(readin>>32);
		dexter_dsp_tx_write(st, ADDR_STREAM_START0_MSB, temp32);
		break;

	case REG_FREQUENCY1:
    		temp64 = (int64_t)val  << DDS_PHASEWIDTH;
    		temp32 = (int32_t)div_s64(temp64,st->fs_if_dac);
		dexter_dsp_tx_write(st, ADDR_DDS_INC1, temp32);
		break;

	case REG_GAIN1:
		temp32 = dexter_dsp_tx_read(st, ADDR_GAIN_DC1) & 0xFFFF0000;
		temp32 += (uint32_t)val & 0xFFFF;
		dexter_dsp_tx_write(st, ADDR_GAIN_DC1, temp32);
		break;

	case REG_DC1:
		temp32 = dexter_dsp_tx_read(st, ADDR_GAIN_DC1) & 0xFFFF;
		temp32 += (uint32_t)val << 16;
		dexter_dsp_tx_write(st, ADDR_GAIN_DC1, temp32);
		break;

	case REG_STREAM1_START_CLKS:
		tempu64 = ((uint64_t)dexter_dsp_tx_read(st, ADDR_PPS_CLKS_MSB) << 32) + dexter_dsp_tx_read(st, ADDR_PPS_CLKS_LSB);
		if(readin!=0 && (readin < tempu64 || readin > tempu64+20*(uint64_t)st->fs_if_dac)) // start must be in the future and not more than 10s in the future
			ret = -EINVAL;
		temp32 = (uint32_t)readin;
		dexter_dsp_tx_write(st, ADDR_STREAM_START1_LSB, temp32);
		temp32 = (uint32_t)(readin>>32);
		dexter_dsp_tx_write(st, ADDR_STREAM_START1_MSB, temp32);
		break;

	case REG_PPS_DIRECTION_OUT_N_IN:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dexter_dsp_tx_read(st, ADDR_PPS_SETTINGS) & ~(1<<29);
		temp32 += (u32)val<<29;
		dexter_dsp_tx_write(st, ADDR_PPS_SETTINGS, temp32);
		break;

	case REG_PPS_REFERENCE_FREQUENCY:
		temp32 = dexter_dsp_tx_read(st, ADDR_PPS_SETTINGS) & ~(0x1FFFFFFF);
		temp32 += (u32)val & 0x1FFFFFFF;
		dexter_dsp_tx_write(st, ADDR_PPS_SETTINGS, temp32);
		break;

	case REG_PPS_DELAY:
		temp32 = dexter_dsp_tx_read(st, ADDR_PPS_DELAY) & ~(0x1FFFFFFF);
		temp32 += (u32)val & 0x1FFFFFFF;
		dexter_dsp_tx_write(st, ADDR_PPS_DELAY, temp32);
		break;

	case REG_PPS_CLK_ERROR_NS:
		st->pps_clk_error_ns = (u32)val;
		break;

	case REG_PPS_CLK_ERROR_HZ:
		st->pps_clk_error_hz = (u32)val;
		break;

	case REG_GPSDO_LOCKED:
		st->gpsdo_locked = (u32)val & 0x1;
		break;

	default:
		ret = -ENODEV;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}




static ssize_t dexter_dsp_tx_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dexter_dsp_tx_state *st = iio_priv(indio_dev);
	int ret = 0;
	int32_t val = 0;
	int64_t temp64;
	uint64_t tempu64 = 0;

	mutex_lock(&indio_dev->mlock);

	switch ((uint32_t)this_attr->address) {

	case REG_FREQUENCY0:
		temp64 = (int32_t)dexter_dsp_tx_read(st, ADDR_DDS_INC0);
		temp64 = temp64 * st->fs_if_dac;
		val = (int32_t)(temp64 >> DDS_PHASEWIDTH);
		break;

	case REG_GAIN0:
		val = dexter_dsp_tx_read(st, ADDR_GAIN_DC0) & 0xFFFF;
		break;

	case REG_DC0:
		val = (dexter_dsp_tx_read(st, ADDR_GAIN_DC0) & 0xFFFF0000) >> 16;
		break;

	case REG_STREAM0_START_CLKS:
		tempu64 = ((uint64_t)dexter_dsp_tx_read(st, ADDR_STREAM_START0_MSB) << 32) + dexter_dsp_tx_read(st, ADDR_STREAM_START0_LSB);
		break;

	case REG_BUFFER_UNDERFLOWS0:
		val = dexter_dsp_tx_read(st, ADDR_BUFF_UFLOWS0);
		break;

	case REG_FREQUENCY1:
		temp64 = (int32_t)dexter_dsp_tx_read(st, ADDR_DDS_INC1);
		temp64 = temp64 * st->fs_if_dac;
		val = (int32_t)(temp64 >> DDS_PHASEWIDTH);
		break;

	case REG_GAIN1:
		val = dexter_dsp_tx_read(st, ADDR_GAIN_DC1) & 0xFFFF;
		break;

	case REG_DC1:
		val = (dexter_dsp_tx_read(st, ADDR_GAIN_DC1) & 0xFFFF0000) >> 16;
		break;

	case REG_STREAM1_START_CLKS:
		tempu64 = ((uint64_t)dexter_dsp_tx_read(st, ADDR_STREAM_START1_MSB) << 32) + dexter_dsp_tx_read(st, ADDR_STREAM_START1_LSB);
		break;

	case REG_BUFFER_UNDERFLOWS1:
		val = dexter_dsp_tx_read(st, ADDR_BUFF_UFLOWS1);
		break;

	case REG_PPS_DIRECTION_OUT_N_IN:
		val = (dexter_dsp_tx_read(st, ADDR_PPS_SETTINGS) >>29) & 1;
		break;

	case REG_PPS_CLK_ERROR:
		val = dexter_dsp_tx_read(st, ADDR_PPS_ERROR) & 0x1FFFFFFF;
		break;

	case REG_PPS_CLK_ERROR_HZ:
		val = st->pps_clk_error_hz;
		break;

	case REG_PPS_CLK_ERROR_NS:
		val = st->pps_clk_error_ns;
		break;

	case REG_PPS_REFERENCE_FREQUENCY:
		val = dexter_dsp_tx_read(st, ADDR_PPS_SETTINGS) & 0x1FFFFFFF;
		break;

	case REG_PPS_DELAY:
		val = dexter_dsp_tx_read(st, ADDR_PPS_DELAY) & 0x1FFFFFFF;
		break;

	case REG_PPS_CNT:
		val = dexter_dsp_tx_read(st, ADDR_PPS_CNT);
		break;

	case REG_PPS_CLKS:
		tempu64 = ((uint64_t)dexter_dsp_tx_read(st, ADDR_PPS_CLKS_MSB) << 32) + dexter_dsp_tx_read(st, ADDR_PPS_CLKS_LSB);
		break;

	case REG_GPSDO_LOCKED:
		val = st->gpsdo_locked;
		break;

	case REG_DSP_VERSION:
		val = dexter_dsp_tx_read(st, ADDR_DSP_VERSION);
		break;

	default:
		ret = -ENODEV;

	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0){
		if((uint32_t)this_attr->address == REG_PPS_CLKS || (uint32_t)this_attr->address == REG_STREAM0_START_CLKS || (uint32_t)this_attr->address == REG_STREAM1_START_CLKS)
			ret = sprintf(buf, "%llu\n", tempu64);
		else
			ret = sprintf(buf, "%d\n", val);
	}
	return ret;
}



static IIO_DEVICE_ATTR(frequency0, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_FREQUENCY0);

static IIO_DEVICE_ATTR(gain0, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_GAIN0);

static IIO_DEVICE_ATTR(dc0, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_DC0);

static IIO_DEVICE_ATTR(stream0_start_clks, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_STREAM0_START_CLKS);

static IIO_DEVICE_ATTR(buffer_underflows0, S_IRUGO,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_BUFFER_UNDERFLOWS0);

static IIO_DEVICE_ATTR(frequency1, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_FREQUENCY1);

static IIO_DEVICE_ATTR(gain1, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_GAIN1);

static IIO_DEVICE_ATTR(dc1, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_DC1);

static IIO_DEVICE_ATTR(stream1_start_clks, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_STREAM1_START_CLKS);

static IIO_DEVICE_ATTR(buffer_underflows1, S_IRUGO,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_BUFFER_UNDERFLOWS1);

static IIO_DEVICE_ATTR(pps_direction_out_n_in, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_PPS_DIRECTION_OUT_N_IN);

static IIO_DEVICE_ATTR(pps_clk_error, S_IRUGO,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_PPS_CLK_ERROR);

static IIO_DEVICE_ATTR(pps_clk_error_ns, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_PPS_CLK_ERROR_NS);

static IIO_DEVICE_ATTR(pps_clk_error_hz, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_PPS_CLK_ERROR_HZ);

static IIO_DEVICE_ATTR(pps_reference_frequency, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_PPS_REFERENCE_FREQUENCY);

static IIO_DEVICE_ATTR(pps_delay, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_PPS_DELAY);

static IIO_DEVICE_ATTR(gpsdo_locked, S_IRUGO | S_IWUSR,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_GPSDO_LOCKED);

static IIO_DEVICE_ATTR(pps_cnt, S_IRUGO,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_PPS_CNT);

static IIO_DEVICE_ATTR(pps_clks, S_IRUGO,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_PPS_CLKS);

static IIO_DEVICE_ATTR(dsp_version, S_IRUGO,
			dexter_dsp_tx_show,
			dexter_dsp_tx_store,
			REG_DSP_VERSION);



static struct attribute *dexter_dsp_tx_attributes[] = {
	&iio_dev_attr_frequency0.dev_attr.attr,
	&iio_dev_attr_gain0.dev_attr.attr,
	&iio_dev_attr_dc0.dev_attr.attr,
	&iio_dev_attr_stream0_start_clks.dev_attr.attr,
	&iio_dev_attr_buffer_underflows0.dev_attr.attr,
	&iio_dev_attr_frequency1.dev_attr.attr,
	&iio_dev_attr_gain1.dev_attr.attr,
	&iio_dev_attr_dc1.dev_attr.attr,
	&iio_dev_attr_stream1_start_clks.dev_attr.attr,
	&iio_dev_attr_buffer_underflows1.dev_attr.attr,
	&iio_dev_attr_pps_direction_out_n_in.dev_attr.attr,
	&iio_dev_attr_pps_clk_error.dev_attr.attr,
	&iio_dev_attr_pps_clk_error_ns.dev_attr.attr,
	&iio_dev_attr_pps_clk_error_hz.dev_attr.attr,
	&iio_dev_attr_pps_reference_frequency.dev_attr.attr,
	&iio_dev_attr_pps_delay.dev_attr.attr,
	&iio_dev_attr_pps_cnt.dev_attr.attr,
	&iio_dev_attr_pps_clks.dev_attr.attr,
	&iio_dev_attr_gpsdo_locked.dev_attr.attr,
	&iio_dev_attr_dsp_version.dev_attr.attr,
	NULL
};


static const struct attribute_group dexter_dsp_tx_attribute_group = {
	.attrs = dexter_dsp_tx_attributes,
};

static const struct iio_info dexter_dsp_tx_info = {
	.read_raw = &dexter_dsp_tx_read_raw,
	.write_raw = &dexter_dsp_tx_write_raw,
	.attrs = &dexter_dsp_tx_attribute_group,
};

static const struct iio_chan_spec dexter_dsp_tx_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id dexter_dsp_tx_of_match[] = {
	{ .compatible = "fpga,dexter-dsp-tx", },
	{ },
};

MODULE_DEVICE_TABLE(of, dexter_dsp_tx_of_match);

#define to_dexter_dsp_tx_state(x) \
		container_of(x, struct dexter_dsp_tx_state, clk_rate_change_nb)

static int dac_clk_clock_notifier(struct notifier_block *nb,
				  unsigned long event, void *data)
{
	struct clk_notifier_data *ndata = data;
	struct dexter_dsp_tx_state *st = to_dexter_dsp_tx_state(nb);

	dev_info(st->dev, "dac_clk_clock_notifier: event %lu, Old rate %lu, New rate = %lu\n", event, ndata->old_rate, ndata->new_rate);

	st->fs_if_dac = ndata->new_rate;

	switch (event) {
	case PRE_RATE_CHANGE:
	case POST_RATE_CHANGE:
	case ABORT_RATE_CHANGE:
	default:
		return NOTIFY_DONE;
	}
}

static int dexter_dsp_tx_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct dexter_dsp_tx_state *st;
	struct iio_dev *indio_dev;
	int ret;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(dexter_dsp_tx_of_match, &pdev->dev);
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
	st->dev = &pdev->dev;

	st->dac_clk = devm_clk_get(&pdev->dev, "dac_clk");
	if (IS_ERR_OR_NULL(st->dac_clk)) {
		ret = PTR_ERR(st->dac_clk);
		dev_err(&pdev->dev, "Failed to get DAC clock (%d)\n", ret);
		goto err_iio_device_free;
	}

	ret = clk_prepare_enable(st->dac_clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable DAC clock\n");
		goto err_iio_device_free;
	}

	ret = clk_get_rate(st->dac_clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to query DAC clock\n");
		goto err_iio_device_free;
	}

	st->fs_if_dac = ret;

	if (st->fs_if_dac == 0) {
		dev_warn(&pdev->dev, "dac_clk equal to 0 Hz\n");
		//ret = -EINVAL;
		//goto err_iio_device_free;
	}

	dev_info(&pdev->dev, "fs_if_dac rate is %u Hz", st->fs_if_dac);

	st->clk_rate_change_nb.notifier_call = dac_clk_clock_notifier;
	clk_notifier_register(st->dac_clk, &st->clk_rate_change_nb);

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
	indio_dev->channels = dexter_dsp_tx_channels;
	indio_dev->num_channels = ARRAY_SIZE(dexter_dsp_tx_channels);
	indio_dev->info = &dexter_dsp_tx_info;
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

static int dexter_dsp_tx_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver dexter_dsp_tx_driver = {
	.probe		= dexter_dsp_tx_probe,
	.remove		= dexter_dsp_tx_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = dexter_dsp_tx_of_match,
	},
};

module_platform_driver(dexter_dsp_tx_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("DEXTER SDR DSP FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
