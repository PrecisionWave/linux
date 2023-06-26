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


#define DRIVER_NAME			"dras-tetra"
#define NB_OF_TETRA_CHANNELS		4

// common DSP addresses
#define ADDR_DSP_VERSION		(0*4)
#define ADDR_CHANNEL_ASSIGNMENT		(1*4)
#define ADDR_TX21_GAIN			(2*4)
#define ADDR_RX_BURST_LENGTH		(3*4)
#define ADDR_RX_BURST_PERIOD		(4*4)

// TETRA channels
#define ADDR_PER_TETRA_CHANNELS		16
#define ADDR_RX_TETRA_DDSINC(x)		(1*16+x*ADDR_PER_TETRA_CHANNELS+0)*4
#define ADDR_TX_TETRA_DDSINC(x)		(1*16+x*ADDR_PER_TETRA_CHANNELS+1)*4
#define ADDR_TETRA_GAIN(x)		(1*16+x*ADDR_PER_TETRA_CHANNELS+2)*4
#define ADDR_TX_TETRA_TESTTONE_AMPL(x)	(1*16+x*ADDR_PER_TETRA_CHANNELS+3)*4

#define MIN_GAIN			0x0000
#define MAX_GAIN			0x3FFFF
#define MIN_AMPL			0x0000
#define MAX_AMPL			0xFFFF
#define MAX_TETRA_FREQUENCY		20000000
#define MIN_TETRA_FREQUENCY		-20000000


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
	CH3_##REG
//	CH4_##REG, \
//	CH5_##REG, \
//	CH6_##REG, \
//	CH7_##REG

// expands to:
//   static IIO_DEVICE_ATTR(ch0_<ATTR>, <RW>, <SHOW>, <STORE>, CH0_<REG>);
//   static IIO_DEVICE_ATTR(ch1_<ATTR>, <RW>, <SHOW>, <STORE>, CH1_<REG>);
//    ::   ::
//   static IIO_DEVICE_ATTR(ch31_<ATTR>, <RW>, <SHOW>, <STORE>, CH31_<REG>);
// example:
//   IIO_DEVICE_ATTR_ALL_CH(gain_tx1, S_IRUGO | S_IWUSR, dras_tetra_show, dras_tetra_store, REG_GAIN_TX1)
//     expansion:
//     static IIO_DEVICE_ATTR(ch0_gain_tx1, S_IRUGO | S_IWUSR, dras_tetra_show, dras_tetra_store, CH0_REG_GAIN_TX1);
//     static IIO_DEVICE_ATTR(ch1_gain_tx1, S_IRUGO | S_IWUSR, dras_tetra_show, dras_tetra_store, CH1_REG_GAIN_TX1);
//      ::   ::
//     static IIO_DEVICE_ATTR(ch31_gain_tx1, S_IRUGO | S_IWUSR, dras_tetra_show, dras_tetra_store, CH31_REG_GAIN_TX1);
#define IIO_DEVICE_ATTR_ALL_CH(ATTR, RW, SHOW, STORE, REG) \
	static IIO_DEVICE_ATTR(ch0_##ATTR, RW, SHOW, STORE, CH0_##REG); \
	static IIO_DEVICE_ATTR(ch1_##ATTR, RW, SHOW, STORE, CH1_##REG); \
	static IIO_DEVICE_ATTR(ch2_##ATTR, RW, SHOW, STORE, CH2_##REG); \
	static IIO_DEVICE_ATTR(ch3_##ATTR, RW, SHOW, STORE, CH3_##REG); 
//	static IIO_DEVICE_ATTR(ch4_##ATTR, RW, SHOW, STORE, CH4_##REG); \
//	static IIO_DEVICE_ATTR(ch5_##ATTR, RW, SHOW, STORE, CH5_##REG); \
//	static IIO_DEVICE_ATTR(ch6_##ATTR, RW, SHOW, STORE, CH6_##REG); \
//	static IIO_DEVICE_ATTR(ch7_##ATTR, RW, SHOW, STORE, CH7_##REG);

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
	&iio_dev_attr_ch3_##ATTR.dev_attr.attr
//	&iio_dev_attr_ch4_##ATTR.dev_attr.attr, \
//	&iio_dev_attr_ch5_##ATTR.dev_attr.attr, \
//	&iio_dev_attr_ch6_##ATTR.dev_attr.attr, \
//	&iio_dev_attr_ch7_##ATTR.dev_attr.attr

enum chan_num{
	REG_ALL_CH(REG_RX_TETRA_CHANNEL_FREQUENCY),	// being expanded for all channels
	REG_ALL_CH(REG_TX_TETRA_CHANNEL_FREQUENCY),	// being expanded for all channels
	REG_ALL_CH(REG_TETRA_CHANNEL_GAIN),	// being expanded for all channels
	REG_ALL_CH(REG_TX_TETRA_CHANNEL_TESTTONE_AMPLITUDE),	// being expanded for all channels
	REG_ALL_CH(REG_RX_TETRA_CHANNEL_SELECTION),	// being expanded for all channels
	REG_ALL_CH(REG_TX1_TETRA_CHANNEL_OUTPUT_ENABLE),	// being expanded for all channels
	REG_ALL_CH(REG_TX2_TETRA_CHANNEL_OUTPUT_ENABLE),	// being expanded for all channels
	REG_TX1_GAIN,
	REG_TX2_GAIN,
	REG_CH0_WIDEBAND_MODE,
	REG_CH2_WIDEBAND_MODE,
	REG_RX_BURST_LENGTH,
	REG_RX_BURST_PERIOD,
	REG_RX_DMA_FULLRATE_ADC,
	REG_RX_DMA_FULLRATE_ADC_SELECTION,
	REG_DSP_VERSION
};

struct dras_tetra_state {
	struct iio_info		iio_info;
	void __iomem		*regs;
	struct mutex		lock;

	uint32_t		fs_adc;
};

static void dras_tetra_write(struct dras_tetra_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 dras_tetra_read(struct dras_tetra_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int dras_tetra_write_raw(struct iio_dev *indio_dev,
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

static int dras_tetra_read_raw(struct iio_dev *indio_dev,
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

static ssize_t dras_tetra_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dras_tetra_state *st = iio_priv(indio_dev);
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
		if((u32)this_attr->address == REG_CH(ch, REG_RX_TETRA_CHANNEL_FREQUENCY)){
			match = 1;
			if(val<MIN_TETRA_FREQUENCY || val>MAX_TETRA_FREQUENCY){
				ret = -EINVAL;
				break;
			}
			temp64 = val << 24;
			temp64 = div_s64(temp64,st->fs_adc);
			val = (int)temp64 & 0xFFFFFF;
			dras_tetra_write(st, ADDR_RX_TETRA_DDSINC(ch), val);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX_TETRA_CHANNEL_FREQUENCY)){
			match = 1;
			if(val<MIN_TETRA_FREQUENCY || val>MAX_TETRA_FREQUENCY){
				ret = -EINVAL;
				break;
			}
			temp64 = val << 24;
			temp64 = div_s64(temp64,st->fs_adc);
			val = (int)temp64 & 0xFFFFFF;
			dras_tetra_write(st, ADDR_TX_TETRA_DDSINC(ch), val);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TETRA_CHANNEL_GAIN)){
			match = 1;
			if(val<MIN_GAIN || val>MAX_GAIN){
				ret = -EINVAL;
				break;
			}
			dras_tetra_write(st, ADDR_TETRA_GAIN(ch), val);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX_TETRA_CHANNEL_TESTTONE_AMPLITUDE)){
			match = 1;
			if(val<MIN_AMPL || val>MAX_AMPL){
				ret = -EINVAL;
				break;
			}
			dras_tetra_write(st, ADDR_TX_TETRA_TESTTONE_AMPL(ch), val);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_TETRA_CHANNEL_SELECTION)){
			match = 1;
			if(val<1 || val>2){
				ret = -EINVAL;
				break;
			}
			temp32 = dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) & ~(1<<ch);
			temp32 += ((uint32_t)val-1)<<ch;
			dras_tetra_write(st, ADDR_CHANNEL_ASSIGNMENT, temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX1_TETRA_CHANNEL_OUTPUT_ENABLE)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) & ~(1<<(ch+16));
			temp32 += ((uint32_t)val)<<(ch+16);
			dras_tetra_write(st, ADDR_CHANNEL_ASSIGNMENT, temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX2_TETRA_CHANNEL_OUTPUT_ENABLE)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) & ~(1<<(ch+24));
			temp32 += ((uint32_t)val)<<(ch+24);
			dras_tetra_write(st, ADDR_CHANNEL_ASSIGNMENT, temp32);
			break;
		}
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		return ret ? ret : len;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_CH0_WIDEBAND_MODE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) & ~(1<<8);
		temp32 += ((uint32_t)val)<<8;
		dras_tetra_write(st, ADDR_CHANNEL_ASSIGNMENT, temp32);
		break;
	case REG_CH2_WIDEBAND_MODE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) & ~(1<<10);
		temp32 += ((uint32_t)val)<<10;
		dras_tetra_write(st, ADDR_CHANNEL_ASSIGNMENT, temp32);
		break;
	case REG_TX1_GAIN:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_TX21_GAIN) & 0xFFFF0000;
		temp32 += ((uint32_t)val);
		dras_tetra_write(st, ADDR_TX21_GAIN, temp32);
		break;
	case REG_TX2_GAIN:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_TX21_GAIN) & 0xFFFF;
		temp32 += ((uint32_t)val) << 16;
		dras_tetra_write(st, ADDR_TX21_GAIN, temp32);
		break;
	case REG_RX_BURST_LENGTH:
		dras_tetra_write(st, ADDR_RX_BURST_LENGTH, (uint32_t)val);
		break;
	case REG_RX_BURST_PERIOD:
		dras_tetra_write(st, ADDR_RX_BURST_PERIOD, (uint32_t)val);
		break;
	case REG_RX_DMA_FULLRATE_ADC:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) & ~(1<<12);
		temp32 += ((uint32_t)val)<<12;
		dras_tetra_write(st, ADDR_CHANNEL_ASSIGNMENT, temp32);
		break;
	case REG_RX_DMA_FULLRATE_ADC_SELECTION:
		if(val<1 || val>2){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) & ~(1<<13);
		temp32 += ((uint32_t)val-1)<<13;
		dras_tetra_write(st, ADDR_CHANNEL_ASSIGNMENT, temp32);
		break;
	default:
		ret = -ENODEV;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t dras_tetra_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dras_tetra_state *st = iio_priv(indio_dev);
	int val;
	int ret = 0;
	int64_t temp64;
	u32 ch;
	int match;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<NB_OF_TETRA_CHANNELS; ch++){
		if((u32)this_attr->address == REG_CH(ch, REG_RX_TETRA_CHANNEL_FREQUENCY)){
			match = 1;
			temp64 = (int32_t)dras_tetra_read(st, ADDR_RX_TETRA_DDSINC(ch)) & 0xFFFFFF;

			temp64 = temp64 * st->fs_adc;
			val = (int32_t)(temp64 >> 24);
			if(val > (st->fs_adc >>1))
				val -= st->fs_adc;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX_TETRA_CHANNEL_FREQUENCY)){
			match = 1;
			temp64 = (int32_t)dras_tetra_read(st, ADDR_TX_TETRA_DDSINC(ch)) & 0xFFFFFF;

			temp64 = temp64 * st->fs_adc;
			val = (int32_t)(temp64 >> 24);
			if(val > (st->fs_adc >>1))
				val -= st->fs_adc;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TETRA_CHANNEL_GAIN)){
			match = 1;
			val = dras_tetra_read(st, ADDR_TETRA_GAIN(ch));
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX_TETRA_CHANNEL_TESTTONE_AMPLITUDE)){
			match = 1;
			val = dras_tetra_read(st, ADDR_TX_TETRA_TESTTONE_AMPL(ch));
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_TETRA_CHANNEL_SELECTION)){
			match = 1;
			val = ((dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) >> ch) & 1)+1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX1_TETRA_CHANNEL_OUTPUT_ENABLE)){
			match = 1;
			val = (dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) >> (ch+16)) & 1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX2_TETRA_CHANNEL_OUTPUT_ENABLE)){
			match = 1;
			val = (dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) >> (ch+24)) & 1;
			break;
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
	case REG_CH0_WIDEBAND_MODE:
		val = (dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) >> 8) & 1;
		break;
	case REG_CH2_WIDEBAND_MODE:
		val = (dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) >> 10) & 1;
		break;
	case REG_TX1_GAIN:
		val = (uint32_t)dras_tetra_read(st, ADDR_TX21_GAIN) & 0xFFFF;
		break;
	case REG_TX2_GAIN:
		val = ((uint32_t)dras_tetra_read(st, ADDR_TX21_GAIN) & 0xFFFF0000) >> 16;
		break;
	case REG_RX_BURST_LENGTH:
		val = (uint32_t)dras_tetra_read(st, ADDR_RX_BURST_LENGTH);
		break;
	case REG_RX_BURST_PERIOD:
		val = (uint32_t)dras_tetra_read(st, ADDR_RX_BURST_PERIOD);
		break;
	case REG_RX_DMA_FULLRATE_ADC:
		val = (dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) >> 12) & 1;
		break;
	case REG_RX_DMA_FULLRATE_ADC_SELECTION:
		val = 1+((dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) >> 13) & 1);
		break;
	case REG_DSP_VERSION:
		val = dras_tetra_read(st, ADDR_DSP_VERSION);
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

IIO_DEVICE_ATTR_ALL_CH(rx_tetra_frequency, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_TETRA_CHANNEL_FREQUENCY);

IIO_DEVICE_ATTR_ALL_CH(tx_tetra_frequency, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX_TETRA_CHANNEL_FREQUENCY);

IIO_DEVICE_ATTR_ALL_CH(tetra_channel_gain, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TETRA_CHANNEL_GAIN);

IIO_DEVICE_ATTR_ALL_CH(tx_tetra_testtone_amplitude, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX_TETRA_CHANNEL_TESTTONE_AMPLITUDE);

IIO_DEVICE_ATTR_ALL_CH(rx_tetra_channel_input_selection, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_TETRA_CHANNEL_SELECTION);

IIO_DEVICE_ATTR_ALL_CH(tx1_tetra_channel_output_enable, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX1_TETRA_CHANNEL_OUTPUT_ENABLE);

IIO_DEVICE_ATTR_ALL_CH(tx2_tetra_channel_output_enable, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX2_TETRA_CHANNEL_OUTPUT_ENABLE);

static IIO_DEVICE_ATTR(tx1_gain, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX1_GAIN);

static IIO_DEVICE_ATTR(tx2_gain, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX2_GAIN);

static IIO_DEVICE_ATTR(ch0_wideband_mode, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_CH0_WIDEBAND_MODE);

static IIO_DEVICE_ATTR(ch2_wideband_mode, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_CH2_WIDEBAND_MODE);

static IIO_DEVICE_ATTR(rx_burst_length, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_BURST_LENGTH);

static IIO_DEVICE_ATTR(rx_burst_period, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_BURST_PERIOD);

static IIO_DEVICE_ATTR(rx_dma_fullrate_adc, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_DMA_FULLRATE_ADC);

static IIO_DEVICE_ATTR(rx_dma_fullrate_adc_selection, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_DMA_FULLRATE_ADC_SELECTION);

static IIO_DEVICE_ATTR(dsp_version, S_IRUGO,
			dras_tetra_show,
			dras_tetra_store,
			REG_DSP_VERSION);


static struct attribute *dras_tetra_attributes[] = {
	IIO_ATTR_ALL_CH(rx_tetra_frequency),
	IIO_ATTR_ALL_CH(tx_tetra_frequency),
	IIO_ATTR_ALL_CH(tx_tetra_testtone_amplitude),
	IIO_ATTR_ALL_CH(tetra_channel_gain),
	IIO_ATTR_ALL_CH(rx_tetra_channel_input_selection),
	IIO_ATTR_ALL_CH(tx1_tetra_channel_output_enable),
	IIO_ATTR_ALL_CH(tx2_tetra_channel_output_enable),
	&iio_dev_attr_tx1_gain.dev_attr.attr,
	&iio_dev_attr_tx2_gain.dev_attr.attr,
	&iio_dev_attr_ch0_wideband_mode.dev_attr.attr,
	&iio_dev_attr_ch2_wideband_mode.dev_attr.attr,
	&iio_dev_attr_rx_burst_length.dev_attr.attr,
	&iio_dev_attr_rx_burst_period.dev_attr.attr,
	&iio_dev_attr_rx_dma_fullrate_adc.dev_attr.attr,
	&iio_dev_attr_rx_dma_fullrate_adc_selection.dev_attr.attr,
	&iio_dev_attr_dsp_version.dev_attr.attr,
	NULL,
};


static const struct attribute_group dras_tetra_attribute_group = {
	.attrs = dras_tetra_attributes,
};

static const struct iio_info dras_tetra_info = {
	.read_raw = &dras_tetra_read_raw,
	.write_raw = &dras_tetra_write_raw,
	.attrs = &dras_tetra_attribute_group,
};

static const struct iio_chan_spec dras_tetra_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id dras_tetra_of_match[] = {
	{ .compatible = "fpga,dras-tetra", },
	{ },
};

MODULE_DEVICE_TABLE(of, dras_tetra_of_match);

static int dras_tetra_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct dras_tetra_state *st;
	struct iio_dev *indio_dev;
	int ret; //, i, n;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(dras_tetra_of_match, &pdev->dev);
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
//	printk("\nDDC-DUC at 0x%08llX mapped to 0x%p\n",
//			(unsigned long long)res->start, st->regs);


	if(of_property_read_u32(np, "required,fs-adc", &st->fs_adc)){
		printk("DRAS-TETRA: ***ERROR! \"required,fs-adc\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->fs_adc == 0){
		printk("DRAS-TETRA: ***ERROR! \"required,fs-adc\" equal to 0 Hz\n");
		goto err_iio_device_free;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->channels = dras_tetra_channels;
	indio_dev->num_channels = ARRAY_SIZE(dras_tetra_channels);
	indio_dev->info = &dras_tetra_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	//dras_tetra_write(st, ADDR_RX_FM_BAND_BURST_PERIOD, 2389333); 	// Fs/10 > 10Hz update rate
	//dras_tetra_write(st, ADDR_RX_FM_BAND_BURST_LENGTH, 2048);	// 11.7kHz RBW @ 2k FFT
	//dras_tetra_write(st, ADDR_RX_TETRA_BAND_BURST_PERIOD, 14336000); // Fs/10 > 10Hz update rate
	//dras_tetra_write(st, ADDR_RX_TETRA_BAND_BURST_LENGTH, 4096);	// 35kHz RBW @ 4k FFT

	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, indio_dev);
	return 0;

err_iio_device_free:
	iio_device_free(indio_dev);
	return ret;
}

static int dras_tetra_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver dras_tetra_driver = {
	.probe		= dras_tetra_probe,
	.remove		= dras_tetra_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = dras_tetra_of_match,
	},
};

module_platform_driver(dras_tetra_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("DRAS TETRA FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
