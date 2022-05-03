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
 * gain_tx1/2: output gain TX1 0..65535, 65535=0dB

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


#define DRIVER_NAME			"zebu-fmcw-dsp"
#define NB_OF_BLOCKS			1
#define DDS_PHASEWIDTH			24
#define CHIRP_DDS_PHASEWIDTH		25
#define CHIRP_INC_STEP_SHIFT		7


#define ADDR_DSP_VERSION		0*4
#define ADDR_CHIRP_LEN_DAC_SPS		1*4
#define ADDR_CHIRP_INC_STEP		2*4
#define ADDR_CHIRP_INC_START0		3*4
#define ADDR_CHIRP_INC_START1		4*4
#define ADDR_CHIRP_GAIN01		5*4
#define ADDR_RX_DDS_INC			6*4
#define ADDR_RX_DECIMATION		7*4
#define ADDR_RX_BURST_PERIOD		8*4
#define ADDR_RX_BURST_LENGTH		9*4
#define ADDR_RX_OVERFLOWS		10*4


enum chan_num{
  	CH_TX_CHIRP_LEN_SPS,
  	CH_TX_CHIRP_START_FREQ,
  	CH_TX_CHIRP_STOP_FREQ,
  	CH_TX2_CHIRP_OFFSET_FREQ,
  	CH_TX1_CHIRP_GAIN,
  	CH_TX2_CHIRP_GAIN,
  	CH_RX_FREQ,
  	CH_RX_DECIMATION,
  	CH_RX_BURST_PERIOD,
  	CH_RX_BURST_LENGTH,
  	CH_RX_OVERFLOWS,
	CH_DSP_VERSION
};

struct zebu_fmcw_dsp_state {
	struct iio_info	iio_info;
	void __iomem	*regs;
	struct mutex	lock;

	uint32_t	fs_if_dac;
	uint32_t	fs_if_adc;
  	int32_t		chirp_start_freq0;
  	int32_t		chirp_start_freq1;
  	int32_t		chirp_stop_freq0;
  	int32_t		chirp_start_inc0;
  	int32_t		chirp_start_inc1;
  	int32_t		chirp_stop_inc0;
  	int32_t	      	chirp_offset_freq;
};

static void zebu_fmcw_dsp_write(struct zebu_fmcw_dsp_state *st, unsigned reg, uint32_t val)
{
	iowrite32(val, st->regs + reg);
}

static uint32_t zebu_fmcw_dsp_read(struct zebu_fmcw_dsp_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int zebu_fmcw_dsp_write_raw(struct iio_dev *indio_dev,
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

static int zebu_fmcw_dsp_read_raw(struct iio_dev *indio_dev,
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



static ssize_t zebu_fmcw_dsp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct zebu_fmcw_dsp_state *st = iio_priv(indio_dev);
	long val;
	int32_t ret;
	int64_t temp64;
	uint32_t temp32;
	int32_t temps32;

	/* convert to long
	 * auto-detect decimal,
	 * octal (beginning with 0) and
	 * hexadecimal (beginning with 0x)
	 */
	ret = kstrtol(buf, 0, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch ((uint32_t)this_attr->address) {

	case CH_TX_CHIRP_LEN_SPS:
    		temp64 = (int64_t)st->chirp_stop_inc0 - (int64_t)st->chirp_start_inc0; // inc_stop - inc_start0
    		temp64 <<= CHIRP_INC_STEP_SHIFT+2;
    		temp64 = div_s64(temp64,val);
		zebu_fmcw_dsp_write(st, ADDR_CHIRP_INC_STEP, (int32_t)temp64);
		zebu_fmcw_dsp_write(st, ADDR_CHIRP_LEN_DAC_SPS, (uint32_t)val);
		break;

	case CH_TX_CHIRP_START_FREQ:
    		st->chirp_start_freq0 = val; // inc_start0
    		temp64 = (int64_t)st->chirp_start_freq0  << CHIRP_DDS_PHASEWIDTH;
    		st->chirp_start_inc0 = (int32_t)div_s64(temp64,st->fs_if_dac);
		zebu_fmcw_dsp_write(st, ADDR_CHIRP_INC_START0, st->chirp_start_inc0);

    		temp64 = (int64_t)st->chirp_stop_inc0 - (int64_t)st->chirp_start_inc0; // inc_step
    		temp64 <<= CHIRP_INC_STEP_SHIFT+2;
    		temps32 = (int32_t)div_s64(temp64,zebu_fmcw_dsp_read(st, ADDR_CHIRP_LEN_DAC_SPS));
    		zebu_fmcw_dsp_write(st, ADDR_CHIRP_INC_STEP, temps32);

    		st->chirp_start_freq1 = st->chirp_start_freq0 + st->chirp_offset_freq; // chirp_start_freq1
    		temp64 = (int64_t)st->chirp_start_freq1  << CHIRP_DDS_PHASEWIDTH;
    		st->chirp_start_inc1 = (int32_t)div_s64(temp64,st->fs_if_dac); // inc_start1
    		zebu_fmcw_dsp_write(st, ADDR_CHIRP_INC_START1, st->chirp_start_inc1);
		break;

	case CH_TX_CHIRP_STOP_FREQ:
		st->chirp_stop_freq0 = val; // inc_stop
		temp64 = (int64_t)st->chirp_stop_freq0  << CHIRP_DDS_PHASEWIDTH;
		st->chirp_stop_inc0 = (int32_t)div_s64(temp64,st->fs_if_dac);

		temp64 = (int64_t)st->chirp_stop_inc0 - (int64_t)st->chirp_start_inc0; // inc_step
		temp64 <<= CHIRP_INC_STEP_SHIFT+2;
		temps32 = (int32_t)div_s64(temp64, zebu_fmcw_dsp_read(st, ADDR_CHIRP_LEN_DAC_SPS));
		zebu_fmcw_dsp_write(st, ADDR_CHIRP_INC_STEP, temps32);
		break;

	case CH_TX2_CHIRP_OFFSET_FREQ:
		st->chirp_offset_freq = val;
		st->chirp_start_freq1 = st->chirp_start_freq0 + (int32_t)val; // chirp_start_freq1
		temp64 = (int64_t)st->chirp_start_freq1  << CHIRP_DDS_PHASEWIDTH;
		st->chirp_start_inc1 = (int32_t)div_s64(temp64,st->fs_if_dac); // inc_start1
		zebu_fmcw_dsp_write(st, ADDR_CHIRP_INC_START1, st->chirp_start_inc1);
		break;

	case CH_TX1_CHIRP_GAIN:
		temp32 = zebu_fmcw_dsp_read(st, ADDR_CHIRP_GAIN01) & 0xFFFF0000;
		temp32 += (uint32_t)val;
		zebu_fmcw_dsp_write(st, ADDR_CHIRP_GAIN01, temp32);
		break;

	case CH_TX2_CHIRP_GAIN:
		temp32 = zebu_fmcw_dsp_read(st, ADDR_CHIRP_GAIN01) & 0xFFFF;
		temp32 += (uint32_t)val << 16;
		zebu_fmcw_dsp_write(st, ADDR_CHIRP_GAIN01, temp32);
		break;

	case CH_RX_FREQ:
    		temp64 = (int64_t)val  << CHIRP_DDS_PHASEWIDTH;
    		temp32 = (int32_t)div_s64(temp64,st->fs_if_adc);
		zebu_fmcw_dsp_write(st, ADDR_RX_DDS_INC, temp32);
		break;

	case CH_RX_DECIMATION:
		if(val>64)
			val = 64;
		val >>= 1;
		temp32 = 0;
		while(val>0){ // LOG2
			val >>= 1;
			temp32++;
		}
		zebu_fmcw_dsp_write(st, ADDR_RX_DECIMATION, temp32);
		break;

	case CH_RX_BURST_PERIOD:
		zebu_fmcw_dsp_write(st, ADDR_RX_BURST_PERIOD, val);
		break;

	case CH_RX_BURST_LENGTH:
		zebu_fmcw_dsp_write(st, ADDR_RX_BURST_LENGTH, val);
		break;

	default:
		ret = -ENODEV;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}




static ssize_t zebu_fmcw_dsp_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct zebu_fmcw_dsp_state *st = iio_priv(indio_dev);
	int32_t temps32;
	int ret = 0;
	int32_t val;
	int64_t temp64;

	mutex_lock(&indio_dev->mlock);

	switch ((uint32_t)this_attr->address) {

	case CH_TX_CHIRP_LEN_SPS:
		val = zebu_fmcw_dsp_read(st, ADDR_CHIRP_LEN_DAC_SPS);
		break;

	case CH_TX_CHIRP_START_FREQ:
		temp64 = (int32_t)zebu_fmcw_dsp_read(st, ADDR_CHIRP_INC_START0);
		temp64 = temp64 * st->fs_if_dac;
		val = (int32_t)(temp64 >> CHIRP_DDS_PHASEWIDTH);
		break;

	case CH_TX_CHIRP_STOP_FREQ:
		temp64 = (int64_t)zebu_fmcw_dsp_read(st, ADDR_CHIRP_INC_STEP) * (int64_t)zebu_fmcw_dsp_read(st, ADDR_CHIRP_LEN_DAC_SPS);
		temp64 >>= CHIRP_INC_STEP_SHIFT+2;
		temp64 += zebu_fmcw_dsp_read(st, ADDR_CHIRP_INC_START0); // inc stop
		temp64 = temp64 * st->fs_if_dac;
		val = (int32_t)(temp64 >> CHIRP_DDS_PHASEWIDTH);
		break;

	case CH_TX2_CHIRP_OFFSET_FREQ:
		temp64 = (int32_t)zebu_fmcw_dsp_read(st, ADDR_CHIRP_INC_START1);
		temp64 = temp64 * st->fs_if_dac;
		temps32 = (int32_t)(temp64 >> CHIRP_DDS_PHASEWIDTH);
		temp64 = (int32_t)zebu_fmcw_dsp_read(st, ADDR_CHIRP_INC_START0);
		temp64 = temp64 * st->fs_if_dac;
		val = temps32 - (int32_t)(temp64 >> CHIRP_DDS_PHASEWIDTH);
		break;

	case CH_TX1_CHIRP_GAIN:
		val = zebu_fmcw_dsp_read(st, ADDR_CHIRP_GAIN01) & 0xFFFF;
		break;

	case CH_TX2_CHIRP_GAIN:
		val = (zebu_fmcw_dsp_read(st, ADDR_CHIRP_GAIN01) & 0xFFFF0000) >> 16;
		break;

	case CH_RX_FREQ:
		temp64 = (int32_t)zebu_fmcw_dsp_read(st, ADDR_RX_DDS_INC);
		temp64 = temp64 * st->fs_if_adc;
		val = (int32_t)(temp64 >> CHIRP_DDS_PHASEWIDTH);
		break;

	case CH_RX_DECIMATION:
		temps32 = zebu_fmcw_dsp_read(st, ADDR_RX_DECIMATION) & 0x7;
		val = 1<<temps32;
		break;

	case CH_RX_BURST_PERIOD:
		val = zebu_fmcw_dsp_read(st, ADDR_RX_BURST_PERIOD);
		break;

	case CH_RX_BURST_LENGTH:
		val = zebu_fmcw_dsp_read(st, ADDR_RX_BURST_LENGTH);
		break;

	case CH_RX_OVERFLOWS:
		val = zebu_fmcw_dsp_read(st, ADDR_RX_OVERFLOWS);
		break;

	case CH_DSP_VERSION:
		val = zebu_fmcw_dsp_read(st, ADDR_DSP_VERSION);
		break;

	default:
		ret = -ENODEV;

	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0){
		ret = sprintf(buf, "%d\n", val);
	}
	return ret;
}



static IIO_DEVICE_ATTR(tx_chirp_len_sps, S_IRUGO | S_IWUSR,
			zebu_fmcw_dsp_show,
			zebu_fmcw_dsp_store,
			CH_TX_CHIRP_LEN_SPS);

static IIO_DEVICE_ATTR(tx_chirp_start_freq, S_IRUGO | S_IWUSR,
			zebu_fmcw_dsp_show,
			zebu_fmcw_dsp_store,
			CH_TX_CHIRP_START_FREQ);

static IIO_DEVICE_ATTR(tx_chirp_stop_freq, S_IRUGO | S_IWUSR,
			zebu_fmcw_dsp_show,
			zebu_fmcw_dsp_store,
			CH_TX_CHIRP_STOP_FREQ);

static IIO_DEVICE_ATTR(tx2_chirp_offset_freq, S_IRUGO | S_IWUSR,
			zebu_fmcw_dsp_show,
			zebu_fmcw_dsp_store,
			CH_TX2_CHIRP_OFFSET_FREQ);

static IIO_DEVICE_ATTR(tx1_chirp_gain, S_IRUGO | S_IWUSR,
			zebu_fmcw_dsp_show,
			zebu_fmcw_dsp_store,
			CH_TX1_CHIRP_GAIN);

static IIO_DEVICE_ATTR(tx2_chirp_gain, S_IRUGO | S_IWUSR,
			zebu_fmcw_dsp_show,
			zebu_fmcw_dsp_store,
			CH_TX2_CHIRP_GAIN);

static IIO_DEVICE_ATTR(rx_frequency, S_IRUGO | S_IWUSR,
			zebu_fmcw_dsp_show,
			zebu_fmcw_dsp_store,
			CH_RX_FREQ);

static IIO_DEVICE_ATTR(rx_decimation, S_IRUGO | S_IWUSR,
			zebu_fmcw_dsp_show,
			zebu_fmcw_dsp_store,
			CH_RX_DECIMATION);

static IIO_DEVICE_ATTR(rx_burst_period, S_IRUGO | S_IWUSR,
			zebu_fmcw_dsp_show,
			zebu_fmcw_dsp_store,
			CH_RX_BURST_PERIOD);

static IIO_DEVICE_ATTR(rx_burst_length, S_IRUGO | S_IWUSR,
			zebu_fmcw_dsp_show,
			zebu_fmcw_dsp_store,
			CH_RX_BURST_LENGTH);

static IIO_DEVICE_ATTR(rx_overflows, S_IRUGO,
			zebu_fmcw_dsp_show,
			zebu_fmcw_dsp_store,
			CH_RX_OVERFLOWS);

static IIO_DEVICE_ATTR(dsp_version, S_IRUGO,
			zebu_fmcw_dsp_show,
			zebu_fmcw_dsp_store,
			CH_DSP_VERSION);



static struct attribute *zebu_fmcw_dsp_attributes[] = {
	&iio_dev_attr_tx_chirp_len_sps.dev_attr.attr,
	&iio_dev_attr_tx_chirp_start_freq.dev_attr.attr,
	&iio_dev_attr_tx_chirp_stop_freq.dev_attr.attr,
	&iio_dev_attr_tx2_chirp_offset_freq.dev_attr.attr,
	&iio_dev_attr_tx1_chirp_gain.dev_attr.attr,
	&iio_dev_attr_tx2_chirp_gain.dev_attr.attr,
	&iio_dev_attr_rx_frequency.dev_attr.attr,
	&iio_dev_attr_rx_decimation.dev_attr.attr,
	&iio_dev_attr_rx_burst_period.dev_attr.attr,
	&iio_dev_attr_rx_burst_length.dev_attr.attr,
	&iio_dev_attr_rx_overflows.dev_attr.attr,
	&iio_dev_attr_dsp_version.dev_attr.attr,
	NULL
};


static const struct attribute_group zebu_fmcw_dsp_attribute_group = {
	.attrs = zebu_fmcw_dsp_attributes,
};

static const struct iio_info zebu_fmcw_dsp_info = {
	.read_raw = &zebu_fmcw_dsp_read_raw,
	.write_raw = &zebu_fmcw_dsp_write_raw,
	.attrs = &zebu_fmcw_dsp_attribute_group,
};

static const struct iio_chan_spec zebu_fmcw_dsp_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id zebu_fmcw_dsp_of_match[] = {
	{ .compatible = "fpga,zebu-fmcw-dsp", },
	{ },
};

MODULE_DEVICE_TABLE(of, zebu_fmcw_dsp_of_match);

static int zebu_fmcw_dsp_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct zebu_fmcw_dsp_state *st;
	struct iio_dev *indio_dev;
	int ret;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(zebu_fmcw_dsp_of_match, &pdev->dev);
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

	if(of_property_read_u32(np, "required,fs-if-dac", &st->fs_if_dac)){
		printk("ZEBU-FMCW-DSP: ***ERROR! \"required,fs-if-dac\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->fs_if_dac == 0){
		printk("ZEBU-FMCW-DSP: ***ERROR! \"required,fs-if-dac\" equal to 0 Hz\n");
		goto err_iio_device_free;
	}

	if(of_property_read_u32(np, "required,fs-if-adc", &st->fs_if_adc)){
		printk("ZEBU-FMCW-DSP: ***ERROR! \"required,fs-if-adc\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->fs_if_adc == 0){
		printk("ZEBU-FMCW-DSP: ***ERROR! \"required,fs-if-adc\" equal to 0 Hz\n");
		goto err_iio_device_free;
	}
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->channels = zebu_fmcw_dsp_channels;
	indio_dev->num_channels = ARRAY_SIZE(zebu_fmcw_dsp_channels);
	indio_dev->info = &zebu_fmcw_dsp_info;
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

static int zebu_fmcw_dsp_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver zebu_fmcw_dsp_driver = {
	.probe		= zebu_fmcw_dsp_probe,
	.remove		= zebu_fmcw_dsp_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = zebu_fmcw_dsp_of_match,
	},
};

module_platform_driver(zebu_fmcw_dsp_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("ZEBU SDR DSP FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
