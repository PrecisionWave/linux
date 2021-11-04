/*
 * VBI FM DSP COREFPGA Module
 * FM Voice Break In DSP
 *
 * Copyright 2018 PrecisionWave AG
 *
 * Licensed under the GPL-2.
 *
 * Device Parameters
 * -----------------
 * sel: selection of channel 0..4*number_of_blocks from devicetree
 * frequency: channel frequency in Hz
 * gain_tx1: output gain TX1 0..65536, 512=0dB
 * gain_tx2: output gain TX1 0..65536, 512=0dB
 * rf_input_selection: select

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


#define DRIVER_NAME				"vbi-fsk-dsp"
#define NB_OF_BLOCKS			8
#define ADDR_PER_BLOCK			9*4

#define ADDR_PPS_CLKS			(ADDR_PER_BLOCK*NB_OF_BLOCKS)
#define ADDR_BURST_RX_INPUT			(ADDR_PER_BLOCK*NB_OF_BLOCKS+1*4)
#define ADDR_DMA_BURST_PERIOD	(ADDR_PER_BLOCK*NB_OF_BLOCKS+2*4)
#define ADDR_DMA_BURST_LENGTH	(ADDR_PER_BLOCK*NB_OF_BLOCKS+3*4)
#define ADDR_DSP_VERSION		(ADDR_PER_BLOCK*NB_OF_BLOCKS+12*4)
#define ADDR_ADC_PEAK			(ADDR_PER_BLOCK*NB_OF_BLOCKS+13*4)
#define ADDR_ADC_BER_TESTER		(ADDR_PER_BLOCK*NB_OF_BLOCKS+14*4)
#define ADDR_PPS_CNT		(ADDR_PER_BLOCK*NB_OF_BLOCKS+15*4)
#define ADDR_PPS_SETTINGS		(ADDR_PER_BLOCK*NB_OF_BLOCKS+17*4)

#define ADDR_INC01(x)			(0*4+x*ADDR_PER_BLOCK)
#define ADDR_INC23(x)			(1*4+x*ADDR_PER_BLOCK)
#define ADDR_ROUTING(x)			(8*4+x*ADDR_PER_BLOCK)


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
	CH15_##REG, \
	CH16_##REG, \
	CH17_##REG, \
	CH18_##REG, \
	CH19_##REG, \
	CH20_##REG, \
	CH21_##REG, \
	CH22_##REG, \
	CH23_##REG, \
	CH24_##REG, \
	CH25_##REG, \
	CH26_##REG, \
	CH27_##REG, \
	CH28_##REG, \
	CH29_##REG, \
	CH30_##REG, \
	CH31_##REG

// expands to:
//   static IIO_DEVICE_ATTR(ch0_<ATTR>, <RW>, <SHOW>, <STORE>, CH0_<REG>);
//   static IIO_DEVICE_ATTR(ch1_<ATTR>, <RW>, <SHOW>, <STORE>, CH1_<REG>);
//    ::   ::
//   static IIO_DEVICE_ATTR(ch31_<ATTR>, <RW>, <SHOW>, <STORE>, CH31_<REG>);
// example:
//   IIO_DEVICE_ATTR_ALL_CH(gain_tx1, S_IRUGO | S_IWUSR, vbi_fsk_dsp_show, vbi_fsk_dsp_store, REG_GAIN_TX1)
//     expansion:
//     static IIO_DEVICE_ATTR(ch0_gain_tx1, S_IRUGO | S_IWUSR, vbi_fsk_dsp_show, vbi_fsk_dsp_store, CH0_REG_GAIN_TX1);
//     static IIO_DEVICE_ATTR(ch1_gain_tx1, S_IRUGO | S_IWUSR, vbi_fsk_dsp_show, vbi_fsk_dsp_store, CH1_REG_GAIN_TX1);
//      ::   ::
//     static IIO_DEVICE_ATTR(ch31_gain_tx1, S_IRUGO | S_IWUSR, vbi_fsk_dsp_show, vbi_fsk_dsp_store, CH31_REG_GAIN_TX1);
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
	static IIO_DEVICE_ATTR(ch15_##ATTR, RW, SHOW, STORE, CH15_##REG); \
	static IIO_DEVICE_ATTR(ch16_##ATTR, RW, SHOW, STORE, CH16_##REG); \
	static IIO_DEVICE_ATTR(ch17_##ATTR, RW, SHOW, STORE, CH17_##REG); \
	static IIO_DEVICE_ATTR(ch18_##ATTR, RW, SHOW, STORE, CH18_##REG); \
	static IIO_DEVICE_ATTR(ch19_##ATTR, RW, SHOW, STORE, CH19_##REG); \
	static IIO_DEVICE_ATTR(ch20_##ATTR, RW, SHOW, STORE, CH20_##REG); \
	static IIO_DEVICE_ATTR(ch21_##ATTR, RW, SHOW, STORE, CH21_##REG); \
	static IIO_DEVICE_ATTR(ch22_##ATTR, RW, SHOW, STORE, CH22_##REG); \
	static IIO_DEVICE_ATTR(ch23_##ATTR, RW, SHOW, STORE, CH23_##REG); \
	static IIO_DEVICE_ATTR(ch24_##ATTR, RW, SHOW, STORE, CH24_##REG); \
	static IIO_DEVICE_ATTR(ch25_##ATTR, RW, SHOW, STORE, CH25_##REG); \
	static IIO_DEVICE_ATTR(ch26_##ATTR, RW, SHOW, STORE, CH26_##REG); \
	static IIO_DEVICE_ATTR(ch27_##ATTR, RW, SHOW, STORE, CH27_##REG); \
	static IIO_DEVICE_ATTR(ch28_##ATTR, RW, SHOW, STORE, CH28_##REG); \
	static IIO_DEVICE_ATTR(ch29_##ATTR, RW, SHOW, STORE, CH29_##REG); \
	static IIO_DEVICE_ATTR(ch30_##ATTR, RW, SHOW, STORE, CH30_##REG); \
	static IIO_DEVICE_ATTR(ch31_##ATTR, RW, SHOW, STORE, CH31_##REG);

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
	&iio_dev_attr_ch15_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch16_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch17_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch18_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch19_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch20_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch21_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch22_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch23_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch24_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch25_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch26_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch27_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch28_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch29_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch30_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch31_##ATTR.dev_attr.attr

enum chan_num{
	REG_ALL_CH(REG_FREQUENCY),	// being expanded for all channels
	REG_ALL_CH(REG_RF_INPUT_SELECTION),	// being expanded for all channels
	REG_DSP_VERSION,
	REG_ADC1_PEAK_HOLD_VAL,
	REG_ADC2_PEAK_HOLD_VAL,
	REG_ADC_BER_ALTERNATE,
	REG_ADC_BER_CHECKER,
	REG_BURST_RX_INPUT,
	REG_DMA_BURST_PERIOD,
	REG_DMA_BURST_LENGTH,
	REG_PPS_CLK_ERROR,
	REG_PPS_CLK_ERROR_NS,
	REG_PPS_CLK_ERROR_HZ,
	REG_PPS_DIRECTION_OUT_N_IN,
	REG_RX_SAMP_RATE_ADC,
	REG_GPSDO_LOCKED,
	REG_PPS_REFERENCE_FREQUENCY,
	REG_PPS_CLKS,
	REG_PPS_CNT
};

struct vbi_fsk_dsp_state {
	struct iio_info		iio_info;
	void __iomem		*regs;
	struct mutex		lock;

	uint32_t		fs_adc;
	bool			gpsdo_locked;
  	uint32_t		pps_clk_error_ns;
  	uint32_t		pps_clk_error_hz;
	uint32_t		nb_of_blocks;
};

static void vbi_fsk_dsp_write(struct vbi_fsk_dsp_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 vbi_fsk_dsp_read(struct vbi_fsk_dsp_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int vbi_fsk_dsp_write_raw(struct iio_dev *indio_dev,
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

static int vbi_fsk_dsp_read_raw(struct iio_dev *indio_dev,
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

static ssize_t vbi_fsk_dsp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct vbi_fsk_dsp_state *st = iio_priv(indio_dev);
	long val;
	int ret;
	u64 temp64;
	int tempint;
	u32 temp32;
	int subchannel;
	int block_nb;
	u32 ch;
	int match;
	u64 fcenter2;

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
	for(ch=0; ch<32; ch++){
		subchannel = ((ch+1) & 0x3);
		block_nb = (ch >> 2);
		if((u32)this_attr->address == REG_CH(ch, REG_FREQUENCY)){
			match = 1;
			fcenter2 = (u64)st->fs_adc/2*(11+3*16)/16;
			temp64 = fcenter2 - val;
			temp64 = temp64 << 16;
			temp64 = div_s64(temp64,st->fs_adc/8);
			tempint = (int)temp64;
			tempint = tempint & 0xFFFF;
			switch(subchannel){
			case 0:
				temp32 = vbi_fsk_dsp_read(st, ADDR_INC01(block_nb)) & 0xFFFF0000;
				temp32 += (u32)tempint;
				vbi_fsk_dsp_write(st, ADDR_INC01(block_nb), temp32);
				break;
			case 1:
				temp32 = vbi_fsk_dsp_read(st, ADDR_INC01(block_nb)) & 0xFFFF;
				temp32 += (u32)tempint << 16;
				vbi_fsk_dsp_write(st, ADDR_INC01(block_nb), temp32);
				break;
			case 2:
				temp32 = vbi_fsk_dsp_read(st, ADDR_INC23(block_nb)) & 0xFFFF0000;
				temp32 += (u32)tempint;
				vbi_fsk_dsp_write(st, ADDR_INC23(block_nb), temp32);
				break;
			case 3:
				temp32 = vbi_fsk_dsp_read(st, ADDR_INC23(block_nb)) & 0xFFFF;
				temp32 += (u32)tempint << 16;
				vbi_fsk_dsp_write(st, ADDR_INC23(block_nb), temp32);
				break;
			default:
				break;
			}
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RF_INPUT_SELECTION)){
			match = 1;
			if(val<1 || val>2){
				ret = -EINVAL;
				break;
			}
			val -= 1;
			temp32 = vbi_fsk_dsp_read(st, ADDR_ROUTING(block_nb)) & ~(1<<(0+subchannel));
			temp32 += (u32)val << (0+subchannel);
			vbi_fsk_dsp_write(st, ADDR_ROUTING(block_nb), temp32);
			break;
		}
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		return ret ? ret : len;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_BURST_RX_INPUT:
		if(val<1 || val>2){
			ret = -EINVAL;
			break;
		}
		val -= 1;
		vbi_fsk_dsp_write(st, ADDR_BURST_RX_INPUT, (u32)val);
		break;
	case REG_DMA_BURST_PERIOD:
		vbi_fsk_dsp_write(st, ADDR_DMA_BURST_PERIOD, (u32)val);
		break;
	case REG_DMA_BURST_LENGTH:
		vbi_fsk_dsp_write(st, ADDR_DMA_BURST_LENGTH, (u32)val);
		break;
	case REG_PPS_CLK_ERROR_NS:
		st->pps_clk_error_ns = (u32)val;
		break;
	case REG_PPS_CLK_ERROR_HZ:
		st->pps_clk_error_hz = (u32)val;
		break;
	case REG_PPS_DIRECTION_OUT_N_IN:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_fsk_dsp_read(st, ADDR_PPS_SETTINGS) & ~(1<<29);
		temp32 += (u32)val<<29;
		vbi_fsk_dsp_write(st, ADDR_PPS_SETTINGS, temp32);
		break;
	case REG_PPS_REFERENCE_FREQUENCY:
		temp32 = vbi_fsk_dsp_read(st, ADDR_PPS_SETTINGS) & ~(0x1FFFFFFF);
		temp32 += (u32)val & 0x1FFFFFFF;
		vbi_fsk_dsp_write(st, ADDR_PPS_SETTINGS, temp32);
		break;
	case REG_GPSDO_LOCKED:
		st->gpsdo_locked = (u32)val & 0x1;
		break;
	default:
		ret = -ENODEV;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t vbi_fsk_dsp_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct vbi_fsk_dsp_state *st = iio_priv(indio_dev);
	u32 val;
	int ret = 0;
	int tempint;
	u64 fcenter2;
	u64 temp64;
	u32 subchannel;
	u32 block_nb;
	u32 ch;
	int match;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<32; ch++){
		subchannel = ((ch+1) & 0x3);
		block_nb = (ch >> 2);
		if((u32)this_attr->address == REG_CH(ch, REG_FREQUENCY)){
			match = 1;
			switch (subchannel){
			case 0:
				val = vbi_fsk_dsp_read(st, ADDR_INC01(block_nb)) & 0xFFFF;
				break;
			case 1:
				val = (vbi_fsk_dsp_read(st, ADDR_INC01(block_nb)) & 0xFFFF0000) >> 16;
				break;
			case 2:
				val = vbi_fsk_dsp_read(st, ADDR_INC23(block_nb)) & 0xFFFF;
				break;
			case 3:
				val = (vbi_fsk_dsp_read(st, ADDR_INC23(block_nb)) & 0xFFFF0000) >> 16;
				break;
			default:
				break;
			}
			fcenter2 = (u64)st->fs_adc/2*(11+3*16)/16;
			if(val>(1<<15))
				tempint = (int)(val - (1<<16));
			else
				tempint = (int)val;
			temp64 = (u64)tempint * st->fs_adc/8;
			temp64 = temp64 >> 16;
			val = (u32)(fcenter2 - temp64);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RF_INPUT_SELECTION)){
			match = 1;
			val = (vbi_fsk_dsp_read(st, ADDR_ROUTING(block_nb)) & (1<<(0+subchannel))) >> (0+subchannel);
			val += 1;
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
	case REG_DSP_VERSION:
		val = vbi_fsk_dsp_read(st, ADDR_DSP_VERSION);
		break;
	case REG_RX_SAMP_RATE_ADC:
		val = st->fs_adc;
		break;
	case REG_ADC1_PEAK_HOLD_VAL:
		val = (vbi_fsk_dsp_read(st, ADDR_ADC_PEAK) & 0xFFFF);
		break;
	case REG_ADC2_PEAK_HOLD_VAL:
		val = (vbi_fsk_dsp_read(st, ADDR_ADC_PEAK) >> 16);
		break;
	case REG_ADC_BER_ALTERNATE:
		val = (vbi_fsk_dsp_read(st, ADDR_ADC_BER_TESTER) & 0xFFFF);
		break;
	case REG_ADC_BER_CHECKER:
		val = (vbi_fsk_dsp_read(st, ADDR_ADC_BER_TESTER) >> 16);
		break;
	case REG_BURST_RX_INPUT:
		val = vbi_fsk_dsp_read(st, ADDR_BURST_RX_INPUT);
		val += 1;
		break;
	case REG_DMA_BURST_PERIOD:
		val = vbi_fsk_dsp_read(st, ADDR_DMA_BURST_PERIOD);
		break;
	case REG_DMA_BURST_LENGTH:
		val = vbi_fsk_dsp_read(st, ADDR_DMA_BURST_LENGTH);
		break;
	case REG_PPS_DIRECTION_OUT_N_IN:
		val = (vbi_fsk_dsp_read(st, ADDR_PPS_SETTINGS) >>29) & 1;
		break;
	case REG_PPS_CLK_ERROR:
		val = vbi_fsk_dsp_read(st, ADDR_PPS_CLKS) & 0x1FFFFFFF;
		break;
	case REG_PPS_CLK_ERROR_HZ:
		val = st->pps_clk_error_hz;
		break;
	case REG_PPS_CLK_ERROR_NS:
		val = st->pps_clk_error_ns;
		break;
	case REG_PPS_REFERENCE_FREQUENCY:
		val = vbi_fsk_dsp_read(st, ADDR_PPS_SETTINGS) & 0x1FFFFFFF;
		break;
	case REG_PPS_CNT:
		val = vbi_fsk_dsp_read(st, ADDR_PPS_CNT);
		break;
	case REG_GPSDO_LOCKED:
		val = st->gpsdo_locked;
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

IIO_DEVICE_ATTR_ALL_CH(frequency, S_IRUGO | S_IWUSR,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_FREQUENCY);

IIO_DEVICE_ATTR_ALL_CH(rf_input_selection, S_IRUGO | S_IWUSR,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_RF_INPUT_SELECTION);

static IIO_DEVICE_ATTR(dsp_version, S_IRUGO,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_DSP_VERSION);

static IIO_DEVICE_ATTR(adc1_peak_hold_value, S_IRUGO,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_ADC1_PEAK_HOLD_VAL);

static IIO_DEVICE_ATTR(adc2_peak_hold_value, S_IRUGO,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_ADC2_PEAK_HOLD_VAL);

static IIO_DEVICE_ATTR(adc_ber_alternate, S_IRUGO,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_ADC_BER_ALTERNATE);

static IIO_DEVICE_ATTR(adc_ber_checker, S_IRUGO,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_ADC_BER_CHECKER);

static IIO_DEVICE_ATTR(burst_rx_input, S_IRUGO | S_IWUSR,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_BURST_RX_INPUT);

static IIO_DEVICE_ATTR(dma_burst_period, S_IRUGO | S_IWUSR,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_DMA_BURST_PERIOD);

static IIO_DEVICE_ATTR(dma_burst_length, S_IRUGO | S_IWUSR,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_DMA_BURST_LENGTH);

static IIO_DEVICE_ATTR(pps_direction_out_n_in, S_IRUGO | S_IWUSR,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_PPS_DIRECTION_OUT_N_IN);

static IIO_DEVICE_ATTR(pps_clk_error, S_IRUGO,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_PPS_CLK_ERROR);

static IIO_DEVICE_ATTR(pps_clk_error_ns, S_IRUGO | S_IWUSR,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_PPS_CLK_ERROR_NS);

static IIO_DEVICE_ATTR(pps_clk_error_hz, S_IRUGO | S_IWUSR,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_PPS_CLK_ERROR_HZ);

static IIO_DEVICE_ATTR(pps_reference_frequency, S_IRUGO | S_IWUSR,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_PPS_REFERENCE_FREQUENCY);

static IIO_DEVICE_ATTR(gpsdo_locked, S_IRUGO | S_IWUSR,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_GPSDO_LOCKED);

static IIO_DEVICE_ATTR(pps_cnt, S_IRUGO,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_PPS_CNT);

static IIO_DEVICE_ATTR(rx_samp_rate_adc, S_IRUGO,
			vbi_fsk_dsp_show,
			vbi_fsk_dsp_store,
			REG_RX_SAMP_RATE_ADC);


static struct attribute *vbi_fsk_dsp_attributes[] = {
	IIO_ATTR_ALL_CH(frequency),
	IIO_ATTR_ALL_CH(rf_input_selection),
	&iio_dev_attr_dsp_version.dev_attr.attr,
	&iio_dev_attr_adc1_peak_hold_value.dev_attr.attr,
	&iio_dev_attr_adc2_peak_hold_value.dev_attr.attr,
	&iio_dev_attr_adc_ber_alternate.dev_attr.attr,
	&iio_dev_attr_adc_ber_checker.dev_attr.attr,
	&iio_dev_attr_burst_rx_input.dev_attr.attr,
	&iio_dev_attr_dma_burst_period.dev_attr.attr,
	&iio_dev_attr_dma_burst_length.dev_attr.attr,
	&iio_dev_attr_rx_samp_rate_adc.dev_attr.attr,
	&iio_dev_attr_pps_direction_out_n_in.dev_attr.attr,
	&iio_dev_attr_pps_clk_error.dev_attr.attr,
	&iio_dev_attr_pps_clk_error_ns.dev_attr.attr,
	&iio_dev_attr_pps_clk_error_hz.dev_attr.attr,
	&iio_dev_attr_pps_reference_frequency.dev_attr.attr,
	&iio_dev_attr_pps_cnt.dev_attr.attr,
	&iio_dev_attr_gpsdo_locked.dev_attr.attr,
	NULL,
};


static const struct attribute_group vbi_fsk_dsp_attribute_group = {
	.attrs = vbi_fsk_dsp_attributes,
};

static const struct iio_info vbi_fsk_dsp_info = {
	.read_raw = &vbi_fsk_dsp_read_raw,
	.write_raw = &vbi_fsk_dsp_write_raw,
	.attrs = &vbi_fsk_dsp_attribute_group,
};

static const struct iio_chan_spec vbi_fsk_dsp_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id vbi_fsk_dsp_of_match[] = {
	{ .compatible = "fpga,vbi-fsk-dsp", },
	{ },
};

MODULE_DEVICE_TABLE(of, vbi_fsk_dsp_of_match);

static int vbi_fsk_dsp_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct vbi_fsk_dsp_state *st;
	struct iio_dev *indio_dev;
	int ret; //, i, n;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(vbi_fsk_dsp_of_match, &pdev->dev);
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
		printk("VBI-FM-DSP: ***ERROR! \"required,fs-adc\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->fs_adc == 0){
		printk("VBI-FM-DSP: ***ERROR! \"required,fs-adc\" equal to 0 Hz\n");
		goto err_iio_device_free;
	}
	if(of_property_read_u32(np, "required,nb-of-blocks", &st->nb_of_blocks)){
		printk("VBI-FM-DSP: ***ERROR! \"required,nb-of-blocks\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->nb_of_blocks == 0){
		printk("VBI-FM-DSP: ***ERROR! \"required,nb-of-blocks\" equal to 0\n");
		goto err_iio_device_free;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->channels = vbi_fsk_dsp_channels;
	indio_dev->num_channels = ARRAY_SIZE(vbi_fsk_dsp_channels);
	indio_dev->info = &vbi_fsk_dsp_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	vbi_fsk_dsp_write(st, ADDR_DMA_BURST_PERIOD, 16384);
	vbi_fsk_dsp_write(st, ADDR_DMA_BURST_LENGTH, 2048);


	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, indio_dev);
	return 0;

err_iio_device_free:
	iio_device_free(indio_dev);
	return ret;
}

static int vbi_fsk_dsp_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver vbi_fsk_dsp_driver = {
	.probe		= vbi_fsk_dsp_probe,
	.remove		= vbi_fsk_dsp_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vbi_fsk_dsp_of_match,
	},
};

module_platform_driver(vbi_fsk_dsp_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("FM Voice Break-In (VBI) FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
