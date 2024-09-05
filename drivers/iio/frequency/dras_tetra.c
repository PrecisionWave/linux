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


#define DRIVER_NAME			"dras-tetra"
#define NB_OF_TETRA_CHANNELS		8

// common DSP addresses
#define ADDR_DSP_VERSION		(0*4)
#define ADDR_CHANNEL_ASSIGNMENT		(1*4)
#define ADDR_TX21_GAIN			(2*4)
#define ADDR_RX_BURST_LENGTH		(3*4)
#define ADDR_RX_BURST_PERIOD		(4*4)
#define ADDR_CHANNEL_ASSIGNMENT2	(5*4)
#define ADDR_DL_ORDER			(6*4) // 4bits per channel, 8 channels
#define ADDR_EN_UL_TEST_ID_OFFSET	(7*4) // EN_ULTEST, 4bit offset tlast, 12bit ID
#define ADDR_DL_SYNC			(8*4) // sync
//#define ADDR_UL_GAIN			(9*4) // 16bit MSB CH1 + 16bit LSB CH0

// TETRA channels
#define ADDR_PER_TETRA_CHANNELS		16
#define ADDR_RX_TETRA_DDSINC(x)		(1*16+x*ADDR_PER_TETRA_CHANNELS+0)*4
#define ADDR_TX_TETRA_DDSINC(x)		(1*16+x*ADDR_PER_TETRA_CHANNELS+1)*4
//#define ADDR_TETRA_GAIN(x)		(1*16+x*ADDR_PER_TETRA_CHANNELS+2)*4
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
	REG_ALL_CH(REG_RX_TETRA_CHANNEL_FREQUENCY),	// being expanded for all channels
	REG_ALL_CH(REG_TX_TETRA_CHANNEL_FREQUENCY),	// being expanded for all channels
	//REG_ALL_CH(REG_TETRA_CHANNEL_GAIN),	// being expanded for all channels
	REG_ALL_CH(REG_TX_TETRA_CHANNEL_TESTTONE_AMPLITUDE),	// being expanded for all channels
	REG_ALL_CH(REG_RX_TETRA_CHANNEL_SELECTION),	// being expanded for all channels
	REG_ALL_CH(REG_TX1_TETRA_CHANNEL_OUTPUT_ENABLE),	// being expanded for all channels
	REG_ALL_CH(REG_TX2_TETRA_CHANNEL_OUTPUT_ENABLE),	// being expanded for all channels
	REG_TX1_GAIN,
	REG_TX2_GAIN,
	REG_CH0_WIDEBAND_MODE,
	REG_CH4_WIDEBAND_MODE,
	//REG_CH0_UL_GAIN,
	//REG_CH4_UL_GAIN,
	REG_RX_BURST_LENGTH,
	REG_RX_BURST_PERIOD,
	REG_RX_DMA_FULLRATE_ADC,
	REG_RX_DMA_FULLRATE_ADC_SELECTION,
	REG_EN_UL_TEST,
	REG_UL_ID,
	REG_DL_ORDER,
	REG_DL_OFFSET_TLAST,
	REG_DL_SYNC,
	REG_DSP_VERSION,
	REG_RF_MUTE
};

struct dras_tetra_state {
	struct iio_info		iio_info;
	void __iomem		*regs;
	struct mutex		lock;

	uint32_t		fs_adc;
	u32			gain_tx1;
	u32			gain_tx2;
	bool			rf_mute;

	struct device		*dev;
	struct clk		*adrv_clk;
	struct notifier_block	adrv_clk_rate_change_nb;
	uint32_t		adrv_clk_rate;
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
			temp64 = (u64)val << 24;
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
			temp64 = (u64)val << 24;
			temp64 = div_s64(temp64,st->fs_adc);
			val = (int)temp64 & 0xFFFFFF;
			dras_tetra_write(st, ADDR_TX_TETRA_DDSINC(ch), val);
			break;
		}
/*
		else if((u32)this_attr->address == REG_CH(ch, REG_TETRA_CHANNEL_GAIN)){
			match = 1;
			if(val<MIN_GAIN || val>MAX_GAIN){
				ret = -EINVAL;
				break;
			}
			dras_tetra_write(st, ADDR_TETRA_GAIN(ch), val);
			break;
		}
*/
		else if((u32)this_attr->address == REG_CH(ch, REG_TX_TETRA_CHANNEL_TESTTONE_AMPLITUDE)){
			match = 1;
			if((ch%2)==1){ // only channel ch0,2,4,6 have this reg
				ret = -ENODEV;
				break;
			}
			if(val<MIN_AMPL || val>MAX_AMPL){
				ret = -EINVAL;
				break;
			}
			dras_tetra_write(st, ADDR_TX_TETRA_TESTTONE_AMPL(ch/2), val);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_TETRA_CHANNEL_SELECTION)){
			match = 1;
			if(val<1 || val>3){
				ret = -EINVAL;
				break;
			}
			temp32 = dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT2) & ~(1<<ch);
			if(val==3)
				temp32 += 1<<ch;
			else
				temp32 += 0<<ch;
			dras_tetra_write(st, ADDR_CHANNEL_ASSIGNMENT2, temp32);
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
		if(val<0 || val>2){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) & ~(3<<8);
		temp32 += ((uint32_t)val)<<8;
		dras_tetra_write(st, ADDR_CHANNEL_ASSIGNMENT, temp32);
		break;
	case REG_CH4_WIDEBAND_MODE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) & ~(1<<10);
		temp32 += ((uint32_t)val)<<10;
		dras_tetra_write(st, ADDR_CHANNEL_ASSIGNMENT, temp32);
		break;
/*
	case REG_CH0_UL_GAIN:
		if(val<0 || val>65535){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_UL_GAIN) & 0xFFFF0000;
		temp32 += ((uint32_t)val);
		dras_tetra_write(st, ADDR_UL_GAIN, temp32);
		break;
	case REG_CH4_UL_GAIN:
		if(val<0 || val>65535){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_UL_GAIN) & 0xFFFF;
		temp32 += ((uint32_t)val)<<16;
		dras_tetra_write(st, ADDR_UL_GAIN, temp32);
		break;
*/
	case REG_TX1_GAIN:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		st->gain_tx1 = val;
		if(st->rf_mute)
			break;
		dras_tetra_write(st, ADDR_TX21_GAIN,
			(st->gain_tx2 << 16) | st->gain_tx1);
		break;
	case REG_TX2_GAIN:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		st->gain_tx2 = val;
		if(st->rf_mute)
			break;
		dras_tetra_write(st, ADDR_TX21_GAIN,
			(st->gain_tx2 << 16) | st->gain_tx1);
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
	case REG_RF_MUTE:
		if((bool)val == st->rf_mute){
			break;
		}
		st->rf_mute = (bool)val;
		if(st->rf_mute){
			dras_tetra_write(st, ADDR_TX21_GAIN, 0);
		}
		else{
			dras_tetra_write(st, ADDR_TX21_GAIN,
				(st->gain_tx2 << 16) | st->gain_tx1);
		}
		break;
	case REG_EN_UL_TEST:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_EN_UL_TEST_ID_OFFSET) & ~(1<<16);
		temp32 += ((uint32_t)val)<<16;
		dras_tetra_write(st, ADDR_EN_UL_TEST_ID_OFFSET, temp32);
		break;
	case REG_UL_ID:
		if(val<0 || val>0xFFF){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_EN_UL_TEST_ID_OFFSET) & ~(0xFFF);
		temp32 += (uint32_t)val;
		dras_tetra_write(st, ADDR_EN_UL_TEST_ID_OFFSET, temp32);
		break;
	case REG_DL_OFFSET_TLAST:
		//if(val<0 || val>0x7){
		if(val<0 || val>0xF){
			ret = -EINVAL;
			break;
		}
		//val = val*2;
		temp32 = dras_tetra_read(st, ADDR_EN_UL_TEST_ID_OFFSET) & ~(0xF<<12);
		temp32 += ((uint32_t)val)<<12;
		dras_tetra_write(st, ADDR_EN_UL_TEST_ID_OFFSET, temp32);
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
	int val = 0;
	int ret = 0;
	//int power10 = 1;
	int64_t temp64;
	u32 temp32;
	//int shift;
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
/*
		else if((u32)this_attr->address == REG_CH(ch, REG_TETRA_CHANNEL_GAIN)){
			match = 1;
			val = dras_tetra_read(st, ADDR_TETRA_GAIN(ch));
			break;
		}
*/
		else if((u32)this_attr->address == REG_CH(ch, REG_TX_TETRA_CHANNEL_TESTTONE_AMPLITUDE)){
			match = 1;			
			if((ch%2)==1){ // only channel ch0,2,4,6 have this reg
				val = 0;
			}else{
				val = dras_tetra_read(st, ADDR_TX_TETRA_TESTTONE_AMPL(ch/2));
			}
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_TETRA_CHANNEL_SELECTION)){
			match = 1;
			if((dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT2) >> ch) & 1)
				val = 3;
			else
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
		val = (dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) >> 8) & 3;
		break;
	case REG_CH4_WIDEBAND_MODE:
		val = (dras_tetra_read(st, ADDR_CHANNEL_ASSIGNMENT) >> 10) & 1;
		break;
/*
	case REG_CH0_UL_GAIN:
		val = dras_tetra_read(st, ADDR_UL_GAIN) & 0xFFFF;
		break;
	case REG_CH4_UL_GAIN:
		val = (dras_tetra_read(st, ADDR_UL_GAIN)>>16) & 0xFFFF;
		break;
*/
	case REG_TX1_GAIN:
		val = st->gain_tx1;
		break;
	case REG_TX2_GAIN:
		val = st->gain_tx2;
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
	case REG_RF_MUTE:
		val = st->rf_mute;
		break;
	case REG_EN_UL_TEST:
		val = (dras_tetra_read(st, ADDR_EN_UL_TEST_ID_OFFSET)>>16) & 1;
		break;
	case REG_UL_ID:
		val = dras_tetra_read(st, ADDR_EN_UL_TEST_ID_OFFSET) & 0xFFF;
		break;
	case REG_DL_OFFSET_TLAST:
		//val = ((dras_tetra_read(st, ADDR_EN_UL_TEST_ID_OFFSET)>>12) & 0xF)/2;
		val = (dras_tetra_read(st, ADDR_EN_UL_TEST_ID_OFFSET)>>12) & 0xF;
		break;
	case REG_DL_ORDER:
		temp32 = dras_tetra_read(st, ADDR_DL_ORDER);
		//for(shift=0; shift<8; shift++){
		//	val += power10 * (((temp32>>(4*shift)) & 0xF)/2); // register contains 4bits per channel, but only each 2nd channel is used
		//	power10 = power10 * 10;
		//}
		ret = sprintf(buf, "%08x\n", temp32);
		break;
	case REG_DL_SYNC:
		val = dras_tetra_read(st, ADDR_DL_SYNC) & 1;
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

/*
IIO_DEVICE_ATTR_ALL_CH(tetra_channel_gain, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TETRA_CHANNEL_GAIN);
*/

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

static IIO_DEVICE_ATTR(ch4_wideband_mode, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_CH4_WIDEBAND_MODE);

/*
static IIO_DEVICE_ATTR(ch0_ul_gain, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_CH0_UL_GAIN);

static IIO_DEVICE_ATTR(ch4_ul_gain, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_CH4_UL_GAIN);
*/

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

static IIO_DEVICE_ATTR(rf_mute, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RF_MUTE);

static IIO_DEVICE_ATTR(enable_uplink_test, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_EN_UL_TEST);

static IIO_DEVICE_ATTR(uplink_id, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_UL_ID);

static IIO_DEVICE_ATTR(downlink_order, S_IRUGO,
			dras_tetra_show,
			dras_tetra_store,
			REG_DL_ORDER);

static IIO_DEVICE_ATTR(downlink_offset_tlast, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_DL_OFFSET_TLAST);

static IIO_DEVICE_ATTR(downlink_sync, S_IRUGO,
			dras_tetra_show,
			dras_tetra_store,
			REG_DL_SYNC);


static struct attribute *dras_tetra_attributes[] = {
	IIO_ATTR_ALL_CH(rx_tetra_frequency),
	IIO_ATTR_ALL_CH(tx_tetra_frequency),
	IIO_ATTR_ALL_CH(tx_tetra_testtone_amplitude),
	//IIO_ATTR_ALL_CH(tetra_channel_gain),
	IIO_ATTR_ALL_CH(rx_tetra_channel_input_selection),
	IIO_ATTR_ALL_CH(tx1_tetra_channel_output_enable),
	IIO_ATTR_ALL_CH(tx2_tetra_channel_output_enable),
	&iio_dev_attr_tx1_gain.dev_attr.attr,
	&iio_dev_attr_tx2_gain.dev_attr.attr,
	&iio_dev_attr_ch0_wideband_mode.dev_attr.attr,
	&iio_dev_attr_ch4_wideband_mode.dev_attr.attr,
	//&iio_dev_attr_ch0_ul_gain.dev_attr.attr,
	//&iio_dev_attr_ch4_ul_gain.dev_attr.attr,
	&iio_dev_attr_rx_burst_length.dev_attr.attr,
	&iio_dev_attr_rx_burst_period.dev_attr.attr,
	&iio_dev_attr_rx_dma_fullrate_adc.dev_attr.attr,
	&iio_dev_attr_rx_dma_fullrate_adc_selection.dev_attr.attr,
	&iio_dev_attr_dsp_version.dev_attr.attr,
	&iio_dev_attr_rf_mute.dev_attr.attr,
	&iio_dev_attr_enable_uplink_test.dev_attr.attr,
	&iio_dev_attr_uplink_id.dev_attr.attr,
	&iio_dev_attr_downlink_order.dev_attr.attr,
	&iio_dev_attr_downlink_offset_tlast.dev_attr.attr,
	&iio_dev_attr_downlink_sync.dev_attr.attr,
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


static int adrv_clk_clock_notifier(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct clk_notifier_data *ndata = data;
	struct dras_tetra_state *st = container_of(nb, struct dras_tetra_state, adrv_clk_rate_change_nb);

	dev_info(st->dev, "adrv_clk rate change: new rate = %lu Hz\n", ndata->new_rate);

	if (event == POST_RATE_CHANGE) {
		st->adrv_clk_rate = ndata->new_rate;
	}

	return NOTIFY_DONE;
}


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

	indio_dev->name = np->name;
	indio_dev->channels = dras_tetra_channels;
	indio_dev->num_channels = ARRAY_SIZE(dras_tetra_channels);
	indio_dev->info = &dras_tetra_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	/* initially mute TX of both TX */
	st->rf_mute = true;

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
