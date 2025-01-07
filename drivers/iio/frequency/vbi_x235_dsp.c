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


#define DRIVER_NAME			"vbi-x235-dsp"
#define NB_OF_BLOCKS			1

#define ADDR_RX_DDS_INC		(0*4)
#define ADDR_LO_FREQ		(1*4)
#define ADDR_GAIN		(2*4)
#define ADDR_ADC_PEAK		(3*4)
#define ADDR_FREQ_READ		(4*4)
#define ADDR_LO_CONFIG_DELAY	(5*4)
#define ADDR_SETTINGS		(6*4)
#define ADDR_VERSION		(7*4)


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
//   IIO_DEVICE_ATTR_ALL_CH(gain_tx1, S_IRUGO | S_IWUSR, vbi_x235_dsp_show, vbi_x235_dsp_store, REG_GAIN_TX1)
//     expansion:
//     static IIO_DEVICE_ATTR(ch0_gain_tx1, S_IRUGO | S_IWUSR, vbi_x235_dsp_show, vbi_x235_dsp_store, CH0_REG_GAIN_TX1);
//     static IIO_DEVICE_ATTR(ch1_gain_tx1, S_IRUGO | S_IWUSR, vbi_x235_dsp_show, vbi_x235_dsp_store, CH1_REG_GAIN_TX1);
//      ::   ::
//     static IIO_DEVICE_ATTR(ch31_gain_tx1, S_IRUGO | S_IWUSR, vbi_x235_dsp_show, vbi_x235_dsp_store, CH31_REG_GAIN_TX1);
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
	REG_DSP_VERSION,
	REG_ADC1_PEAK_HOLD_VAL,
	REG_ADC2_PEAK_HOLD_VAL,
	REG_RX_ZF1_FREQ,
	REG_LO_MAN_FREQ,
	REG_LO_EN_MAN_TUNING,
	REG_LO_FREQ_READ,
	REG_TX_ENABLE_READ,
	REG_LO_CONF_DELAY,
	REG_GAIN_TX1,
	REG_GAIN_TX2,
	REG_LNA_RX1_EN,
	REG_LNA_RX2_EN,
	REG_PREFILSEL,
	REG_RX_BYP_DIS,
	REG_TX_BYP_DIS,
	REG_RX_ADC_SEL,
	REG_SPI_FREQ_CLK_RISING,
	REG_SPI_FREQ_CS_HIACTIVE,
	REG_SPI_FREQ_D_INVERT,
	REG_SPI_FREQ_MSB_FIRST,
	REG_SPI_DATA_CLK_FALLING,
	REG_SPI_DATA_CS_HIACTIVE,
	REG_SPI_DATA_D_INVERT
};

struct vbi_x235_dsp_state {
	struct iio_info		iio_info;
	void __iomem		*regs;
	struct mutex		lock;

	uint32_t		dsp_clk;
};

static void vbi_x235_dsp_write(struct vbi_x235_dsp_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 vbi_x235_dsp_read(struct vbi_x235_dsp_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int vbi_x235_dsp_write_raw(struct iio_dev *indio_dev,
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

static int vbi_x235_dsp_read_raw(struct iio_dev *indio_dev,
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

static ssize_t vbi_x235_dsp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct vbi_x235_dsp_state *st = iio_priv(indio_dev);
	long val;
	int ret;
	u64 temp64;
	u32 temp32 = 0;
	int quotient;
	//u32 ch;
	//int match;
	//u64 fcenter2;

	/* convert to long
	 * auto-detect decimal,
	 * octal (beginning with 0) and
	 * hexadecimal (beginning with 0x)
	 */
	ret = kstrtol(buf, 0, &val);
	if (ret < 0)
		return ret;


	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_RX_ADC_SEL:
		if(val<1 || val>2){
			ret = -EINVAL;
			break;
		}
		val -= 1;
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<11);
		temp32 += (u32)val<<11;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	case REG_RX_ZF1_FREQ:
		if(val<0 || val>130000000){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val << 24;
		temp64 = div_s64(temp64,st->dsp_clk);
		val = (int)temp64 & 0xFFFFFF;
		vbi_x235_dsp_write(st, ADDR_RX_DDS_INC, (u32)val);
		break;
	case REG_LO_MAN_FREQ:
		if(val<30000000 || val>89975000){
			ret = -EINVAL;
			break;
		}
		val -= 30000000;
		quotient = val/10000000; // 10M-Schritte Bit13..10
		temp32 = ((u32)quotient & 0xF)<<10;
		val = val - quotient * 10000000;
		quotient = val/1000000; // 1M-Schritte Bit9..6
		temp32 += ((u32)quotient & 0xF)<<6;
		val = val - quotient * 1000000;
		quotient = val/25000; // 25k-Schritte Bit5..0
		temp32 += ((u32)quotient & 0x3F)<<0;
		vbi_x235_dsp_write(st, ADDR_LO_FREQ, temp32);
		break;
	case REG_LO_EN_MAN_TUNING:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<10);
		temp32 += (u32)val<<10;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	case REG_LO_CONF_DELAY:
		vbi_x235_dsp_write(st, ADDR_LO_CONFIG_DELAY, (u32)val);
		break;
	case REG_GAIN_TX1:
		if(val<0 || val>0xFFFF){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_GAIN) & 0xFFFF0000;
		temp32 += ((uint32_t)val) & 0xFFFF;
		vbi_x235_dsp_write(st, ADDR_GAIN, temp32);
		break;
	case REG_GAIN_TX2:
		if(val<0 || val>0xFFFF){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_GAIN) & 0xFFFF;
		temp32 += ((uint32_t)val) <<16;
		vbi_x235_dsp_write(st, ADDR_GAIN, temp32);
		break;
	case REG_LNA_RX1_EN:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<12);
		temp32 += (u32)val<<12;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	case REG_LNA_RX2_EN:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<13);
		temp32 += (u32)val<<13;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	case REG_PREFILSEL:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<14);
		temp32 += (u32)val<<14;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	case REG_RX_BYP_DIS:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<9);
		temp32 += (u32)val<<9;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	case REG_TX_BYP_DIS:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<8);
		temp32 += (u32)val<<8;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	case REG_SPI_FREQ_CLK_RISING:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<0);
		temp32 += (u32)val<<0;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	case REG_SPI_FREQ_CS_HIACTIVE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<1);
		temp32 += (u32)val<<1;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	case REG_SPI_FREQ_D_INVERT:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<2);
		temp32 += (u32)val<<2;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	case REG_SPI_FREQ_MSB_FIRST:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<3);
		temp32 += (u32)val<<3;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	case REG_SPI_DATA_CLK_FALLING:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<5);
		temp32 += (u32)val<<5;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	case REG_SPI_DATA_CS_HIACTIVE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<6);
		temp32 += (u32)val<<6;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	case REG_SPI_DATA_D_INVERT:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_x235_dsp_read(st, ADDR_SETTINGS) & ~(1<<7);
		temp32 += (u32)val<<7;
		vbi_x235_dsp_write(st, ADDR_SETTINGS, temp32);
		break;
	default:
		ret = -ENODEV;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t vbi_x235_dsp_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct vbi_x235_dsp_state *st = iio_priv(indio_dev);
	u32 val;
	u32 temp32;
	int ret = 0;
	//int tempint;
	//u64 fcenter2;
	u64 temp64;
	//u32 subchannel;
	//u32 block_nb;
	//u32 ch;
	//int match;


	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_DSP_VERSION:
		val = vbi_x235_dsp_read(st, ADDR_VERSION);
		break;
	case REG_ADC1_PEAK_HOLD_VAL:
		val = (vbi_x235_dsp_read(st, ADDR_ADC_PEAK) & 0xFFFF);
		break;
	case REG_ADC2_PEAK_HOLD_VAL:
		val = (vbi_x235_dsp_read(st, ADDR_ADC_PEAK) >> 16);
		break;
	case REG_LO_FREQ_READ:
		temp32 = vbi_x235_dsp_read(st, ADDR_FREQ_READ);
		val = (temp32 & 0x3F)*25000;
		val += ((temp32>>6) & 0xF)*1000000;
		val += ((temp32>>10) & 0xF)*10000000;
		val += 30000000;
		break;
	case REG_TX_ENABLE_READ:
		val = (vbi_x235_dsp_read(st, ADDR_FREQ_READ) >>14) & 1;
		break;
	case REG_RX_ADC_SEL:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>11) & 1;
		val++;
		break;
	case REG_RX_ZF1_FREQ:
		temp64 = (int32_t)vbi_x235_dsp_read(st, ADDR_RX_DDS_INC);
		temp64 = temp64 * st->dsp_clk;
		val = (int32_t)(temp64 >> 24);
		if(val > (st->dsp_clk >>1))
			val -= st->dsp_clk;
		break;
	case REG_LO_MAN_FREQ:
		temp32 = vbi_x235_dsp_read(st, ADDR_LO_FREQ);
		val = (temp32 & 0x3F)*25000;
		val += ((temp32>>6) & 0xF)*1000000;
		val += ((temp32>>10) & 0xF)*10000000;
		val += 30000000;
		break;
	case REG_LO_EN_MAN_TUNING:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>10) & 1;
		break;
	case REG_LO_CONF_DELAY:
		val = vbi_x235_dsp_read(st, ADDR_LO_CONFIG_DELAY);
		break;
	case REG_GAIN_TX1:
		val = (vbi_x235_dsp_read(st, ADDR_GAIN) & 0xFFFF);
		break;
	case REG_GAIN_TX2:
		val = (vbi_x235_dsp_read(st, ADDR_GAIN) >> 16);
		break;
	case REG_LNA_RX1_EN:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>12) & 1;
		break;
	case REG_LNA_RX2_EN:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>13) & 1;
		break;
	case REG_PREFILSEL:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>14) & 1;
		break;
	case REG_RX_BYP_DIS:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>9) & 1;
		break;
	case REG_TX_BYP_DIS:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>8) & 1;
		break;
	case REG_SPI_FREQ_CLK_RISING:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>0) & 1;
		break;
	case REG_SPI_FREQ_CS_HIACTIVE:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>1) & 1;
		break;
	case REG_SPI_FREQ_D_INVERT:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>2) & 1;
		break;
	case REG_SPI_FREQ_MSB_FIRST:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>3) & 1;
		break;
	case REG_SPI_DATA_CLK_FALLING:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>5) & 1;
		break;
	case REG_SPI_DATA_CS_HIACTIVE:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>6) & 1;
		break;
	case REG_SPI_DATA_D_INVERT:
		val = (vbi_x235_dsp_read(st, ADDR_SETTINGS)>>7) & 1;
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


static IIO_DEVICE_ATTR(dsp_version, S_IRUGO,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_DSP_VERSION);

static IIO_DEVICE_ATTR(adc1_peak_hold_value, S_IRUGO,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_ADC1_PEAK_HOLD_VAL);

static IIO_DEVICE_ATTR(adc2_peak_hold_value, S_IRUGO,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_ADC2_PEAK_HOLD_VAL);

static IIO_DEVICE_ATTR(lo_frequency_read, S_IRUGO,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_LO_FREQ_READ);

static IIO_DEVICE_ATTR(lo_tx_enable_read, S_IRUGO,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_TX_ENABLE_READ);

static IIO_DEVICE_ATTR(rx_zf1_frequency, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_RX_ZF1_FREQ);

static IIO_DEVICE_ATTR(lo_manual_frequency, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_LO_MAN_FREQ);

static IIO_DEVICE_ATTR(lo_enable_manual_tuning, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_LO_EN_MAN_TUNING);

static IIO_DEVICE_ATTR(lo_config_delay, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_LO_CONF_DELAY);

static IIO_DEVICE_ATTR(tx1_gain, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_GAIN_TX1);

static IIO_DEVICE_ATTR(tx2_gain, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_GAIN_TX2);

static IIO_DEVICE_ATTR(rx1_lna_enable, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_LNA_RX1_EN);

static IIO_DEVICE_ATTR(rx2_lna_enable, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_LNA_RX2_EN);

static IIO_DEVICE_ATTR(rx_prefilter_select, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_PREFILSEL);

static IIO_DEVICE_ATTR(rx_bypass_disable, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_RX_BYP_DIS);

static IIO_DEVICE_ATTR(tx_bypass_disable, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_TX_BYP_DIS);

static IIO_DEVICE_ATTR(rx_adc_selection, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_RX_ADC_SEL);

static IIO_DEVICE_ATTR(spi_freq_clk_rising, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_SPI_FREQ_CLK_RISING);

static IIO_DEVICE_ATTR(spi_freq_cs_hiactive, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_SPI_FREQ_CS_HIACTIVE);

static IIO_DEVICE_ATTR(spi_freq_d_invert, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_SPI_FREQ_D_INVERT);

static IIO_DEVICE_ATTR(spi_freq_msb_first, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_SPI_FREQ_MSB_FIRST);

static IIO_DEVICE_ATTR(spi_data_clk_falling, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_SPI_DATA_CLK_FALLING);

static IIO_DEVICE_ATTR(spi_data_cs_hiactive, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_SPI_DATA_CS_HIACTIVE);

static IIO_DEVICE_ATTR(spi_data_d_invert, S_IRUGO | S_IWUSR,
			vbi_x235_dsp_show,
			vbi_x235_dsp_store,
			REG_SPI_DATA_D_INVERT);


static struct attribute *vbi_x235_dsp_attributes[] = {
	&iio_dev_attr_dsp_version.dev_attr.attr,
	&iio_dev_attr_adc1_peak_hold_value.dev_attr.attr,
	&iio_dev_attr_adc2_peak_hold_value.dev_attr.attr,
	&iio_dev_attr_lo_frequency_read.dev_attr.attr,
	&iio_dev_attr_lo_tx_enable_read.dev_attr.attr,
	&iio_dev_attr_rx_zf1_frequency.dev_attr.attr,
	&iio_dev_attr_lo_manual_frequency.dev_attr.attr,
	&iio_dev_attr_lo_enable_manual_tuning.dev_attr.attr,
	&iio_dev_attr_lo_config_delay.dev_attr.attr,
	&iio_dev_attr_tx1_gain.dev_attr.attr,
	&iio_dev_attr_tx2_gain.dev_attr.attr,
	&iio_dev_attr_rx1_lna_enable.dev_attr.attr,
	&iio_dev_attr_rx2_lna_enable.dev_attr.attr,
	&iio_dev_attr_rx_prefilter_select.dev_attr.attr,
	&iio_dev_attr_rx_bypass_disable.dev_attr.attr,
	&iio_dev_attr_tx_bypass_disable.dev_attr.attr,
	&iio_dev_attr_rx_adc_selection.dev_attr.attr,
	&iio_dev_attr_spi_freq_clk_rising.dev_attr.attr,
	&iio_dev_attr_spi_freq_cs_hiactive.dev_attr.attr,
	&iio_dev_attr_spi_freq_d_invert.dev_attr.attr,
	&iio_dev_attr_spi_freq_msb_first.dev_attr.attr,
	&iio_dev_attr_spi_data_clk_falling.dev_attr.attr,
	&iio_dev_attr_spi_data_cs_hiactive.dev_attr.attr,
	&iio_dev_attr_spi_data_d_invert.dev_attr.attr,
	NULL,
};


static const struct attribute_group vbi_x235_dsp_attribute_group = {
	.attrs = vbi_x235_dsp_attributes,
};

static const struct iio_info vbi_x235_dsp_info = {
	.read_raw = &vbi_x235_dsp_read_raw,
	.write_raw = &vbi_x235_dsp_write_raw,
	.attrs = &vbi_x235_dsp_attribute_group,
};

static const struct iio_chan_spec vbi_x235_dsp_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id vbi_x235_dsp_of_match[] = {
	{ .compatible = "fpga,vbi-x235-dsp", },
	{ },
};

MODULE_DEVICE_TABLE(of, vbi_x235_dsp_of_match);

static int vbi_x235_dsp_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct vbi_x235_dsp_state *st;
	struct iio_dev *indio_dev;
	int ret; //, i, n;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(vbi_x235_dsp_of_match, &pdev->dev);
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


	if(of_property_read_u32(np, "required,dsp-clk", &st->dsp_clk)){
		printk("VBI-X235-DSP: ***ERROR! \"required,dsp-clk\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->dsp_clk == 0){
		printk("VBI-X235-DSP: ***ERROR! \"required,dsp-clk\" equal to 0 Hz\n");
		goto err_iio_device_free;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->channels = vbi_x235_dsp_channels;
	indio_dev->num_channels = ARRAY_SIZE(vbi_x235_dsp_channels);
	indio_dev->info = &vbi_x235_dsp_info;
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

static int vbi_x235_dsp_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver vbi_x235_dsp_driver = {
	.probe		= vbi_x235_dsp_probe,
	.remove		= vbi_x235_dsp_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vbi_x235_dsp_of_match,
	},
};

module_platform_driver(vbi_x235_dsp_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("x235 (VBI) FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
