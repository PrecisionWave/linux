/*
 * DRAS DSP COREFPGA Module
 * DRAS FM DAB ADC DAC DSP Core Driver
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


#define DRIVER_NAME			"dras-fm-dab-adc-dac"
#define NB_OF_DAB_CHANNELS		2

// common DSP addresses
#define ADDR_DSP_VERSION		(0*4)
#define ADDR_ADC_PEAK			(1*4)
#define ADDR_PPS_CLKS			(3*4)
#define ADDR_PPS_CNT			(4*4)
#define ADDR_ADC_BER_TESTER		(5*4)
#define ADDR_WATCHDOG			(9*4)
#define ADDR_PPS_SETTINGS		(12*4)

// FM Band
#define ADDR_RX_FM_BAND_BURST_LENGTH	(1*16+1)*4
#define ADDR_RX_FM_BAND_BURST_PERIOD	(1*16+2)*4
#define ADDR_TX_FM_BAND_GAIN		(1*16+3)*4
#define ADDR_TX_FM_TESTTONE_DDSINC21	(1*16+4)*4
#define ADDR_TX_FM_TESTTONE_DDSINC43	(1*16+5)*4
#define ADDR_TX_FM_TESTTONE_AMPL21	(1*16+6)*4
#define ADDR_TX_FM_TESTTONE_AMPL43	(1*16+7)*4

// DAB Band
#define ADDR_RX_DAB_CHANNEL_FREQUENCY	(2*16+0)*4
#define ADDR_RX_DAB_BAND_BURST_LENGTH	(2*16+1)*4
#define ADDR_RX_DAB_BAND_BURST_PERIOD	(2*16+2)*4
#define ADDR_TX_DAB_BAND_GAIN		(2*16+3)*4

// DAB channels
#define ADDR_PER_DAB_CHANNELS		2
#define ADDR_TX_DAB_DDSINC(x)		(3*16+0+x*ADDR_PER_DAB_CHANNELS)*4
#define ADDR_TX_DAB_TESTTONE_AMPL(x)	(3*16+1+x*ADDR_PER_DAB_CHANNELS)*4

#define MIN_GAIN			0x0000
#define MAX_GAIN			0xFFFF
#define MAX_DAB_FREQUENCY		240000000
#define MIN_DAB_FREQUENCY		174000000
#define MAX_FM_FREQUENCY		108100000
#define MIN_FM_FREQUENCY		87400000


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
	CH1_##REG
//	CH2_##REG, \
//	CH3_##REG, \
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
//   IIO_DEVICE_ATTR_ALL_CH(gain_tx1, S_IRUGO | S_IWUSR, dras_fm_dab_adc_dac_show, dras_fm_dab_adc_dac_store, REG_GAIN_TX1)
//     expansion:
//     static IIO_DEVICE_ATTR(ch0_gain_tx1, S_IRUGO | S_IWUSR, dras_fm_dab_adc_dac_show, dras_fm_dab_adc_dac_store, CH0_REG_GAIN_TX1);
//     static IIO_DEVICE_ATTR(ch1_gain_tx1, S_IRUGO | S_IWUSR, dras_fm_dab_adc_dac_show, dras_fm_dab_adc_dac_store, CH1_REG_GAIN_TX1);
//      ::   ::
//     static IIO_DEVICE_ATTR(ch31_gain_tx1, S_IRUGO | S_IWUSR, dras_fm_dab_adc_dac_show, dras_fm_dab_adc_dac_store, CH31_REG_GAIN_TX1);
#define IIO_DEVICE_ATTR_ALL_CH(ATTR, RW, SHOW, STORE, REG) \
	static IIO_DEVICE_ATTR(ch0_##ATTR, RW, SHOW, STORE, CH0_##REG); \
	static IIO_DEVICE_ATTR(ch1_##ATTR, RW, SHOW, STORE, CH1_##REG); 
//	static IIO_DEVICE_ATTR(ch2_##ATTR, RW, SHOW, STORE, CH2_##REG); \
//	static IIO_DEVICE_ATTR(ch3_##ATTR, RW, SHOW, STORE, CH3_##REG); \
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
	&iio_dev_attr_ch1_##ATTR.dev_attr.attr
//	&iio_dev_attr_ch2_##ATTR.dev_attr.attr, \
//	&iio_dev_attr_ch3_##ATTR.dev_attr.attr, \
//	&iio_dev_attr_ch4_##ATTR.dev_attr.attr, \
//	&iio_dev_attr_ch5_##ATTR.dev_attr.attr, \
//	&iio_dev_attr_ch6_##ATTR.dev_attr.attr, \
//	&iio_dev_attr_ch7_##ATTR.dev_attr.attr

enum chan_num{
	REG_ALL_CH(REG_TX_DAB_CHANNEL_FREQUENCY),	// being expanded for all channels
	REG_ALL_CH(REG_TX_DAB_CHANNEL_TESTTONE_AMPLITUDE),	// being expanded for all channels
	REG_DSP_VERSION,
	REG_ADC1_PEAK_HOLD_VAL,
	REG_ADC2_PEAK_HOLD_VAL,
	REG_ADC_BER_ALTERNATE,
	REG_ADC_BER_CHECKER,
	REG_RX_DAB_BAND_BURST_LENGTH,
	REG_RX_DAB_BAND_BURST_PERIOD,
	REG_RX_DAB_CHANNEL_FREQUENCY,
	REG_TX1_DAB_BAND_GAIN,
	REG_TX2_DAB_BAND_GAIN,
	REG_RX_FM_BAND_BURST_LENGTH,
	REG_RX_FM_BAND_BURST_PERIOD,
	REG_TX1_FM_BAND_GAIN,
	REG_TX2_FM_BAND_GAIN,
	REG_TX_FM_TESTTONE_FREQUENCY0,
	REG_TX_FM_TESTTONE_FREQUENCY1,
	REG_TX_FM_TESTTONE_FREQUENCY2,
	REG_TX_FM_TESTTONE_FREQUENCY3,
	REG_TX_FM_TESTTONE_AMPLITUDE0,
	REG_TX_FM_TESTTONE_AMPLITUDE1,
	REG_TX_FM_TESTTONE_AMPLITUDE2,
	REG_TX_FM_TESTTONE_AMPLITUDE3,
	REG_WATCHDOG_ENABLE,
	REG_WATCHDOG_TRIGGER,
	REG_SATURATION_MUTING_OCCURRENCE,
	REG_PPS_CLK_ERROR,
	REG_PPS_CLK_ERROR_NS,
	REG_PPS_CLK_ERROR_HZ,
	REG_PPS_DIRECTION_OUT_N_IN,
	REG_RX_SAMP_RATE_ADC,
	REG_GPSDO_LOCKED,
	REG_PPS_REFERENCE_FREQUENCY,
	REG_PPS_CLKS,
	REG_PPS_CNT,
	REG_RF_MUTE
};

struct dras_fm_dab_adc_dac_state {
	struct iio_info		iio_info;
	void __iomem		*regs;
	struct mutex		lock;

	uint32_t		fs_adc;
	bool			gpsdo_locked;
  	uint32_t		pps_clk_error_ns;
  	uint32_t		pps_clk_error_hz;

	u32				gain_dab_tx1;
	u32				gain_dab_tx2;
	u32				gain_fm_tx1;
	u32				gain_fm_tx2;
	bool			rf_mute;
};

static void dras_fm_dab_adc_dac_write(struct dras_fm_dab_adc_dac_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 dras_fm_dab_adc_dac_read(struct dras_fm_dab_adc_dac_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int dras_fm_dab_adc_dac_write_raw(struct iio_dev *indio_dev,
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

static int dras_fm_dab_adc_dac_read_raw(struct iio_dev *indio_dev,
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

static ssize_t dras_fm_dab_adc_dac_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dras_fm_dab_adc_dac_state *st = iio_priv(indio_dev);
	long val;
	int ret;
	u64 temp64;
	u32 temp32;
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
	for(ch=0; ch<NB_OF_DAB_CHANNELS; ch++){
		if((u32)this_attr->address == REG_CH(ch, REG_TX_DAB_CHANNEL_TESTTONE_AMPLITUDE)){
			match = 1;
			if(val<MIN_GAIN || val>MAX_GAIN){
				ret = -EINVAL;
				break;
			}
			dras_fm_dab_adc_dac_write(st, ADDR_TX_DAB_TESTTONE_AMPL(ch), val);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX_DAB_CHANNEL_FREQUENCY)){
			match = 1;
			if(val<MIN_DAB_FREQUENCY || val>MAX_DAB_FREQUENCY){
				ret = -EINVAL;
				break;
			}
			val += st->fs_adc>>1;
			temp64 = (u64)val << 24;
			temp64 = div_s64(temp64,st->fs_adc);
			val = (int)temp64 & 0xFFFFFF;
			dras_fm_dab_adc_dac_write(st, ADDR_TX_DAB_DDSINC(ch), val);
			break;
		}
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		return ret ? ret : len;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {

	case REG_TX1_DAB_BAND_GAIN:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		st->gain_dab_tx1 = val;
		if(st->rf_mute)
			break;
		dras_fm_dab_adc_dac_write(st, ADDR_TX_DAB_BAND_GAIN,
				(st->gain_dab_tx2 << 16) | st->gain_dab_tx1);
		break;
	case REG_TX2_DAB_BAND_GAIN:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		st->gain_dab_tx2 = val;
		if(st->rf_mute)
			break;
		dras_fm_dab_adc_dac_write(st, ADDR_TX_DAB_BAND_GAIN,
				(st->gain_dab_tx2 << 16) | st->gain_dab_tx1);
		break;
	case REG_RX_DAB_CHANNEL_FREQUENCY:
		if(val<MIN_DAB_FREQUENCY || val>MAX_DAB_FREQUENCY){
			ret = -EINVAL;
			break;
		}
		val = st->fs_adc - val;
		temp64 = (u64)val << 24;
		temp64 = div_s64(temp64,st->fs_adc);
		val = (int)temp64 & 0xFFFFFF;
		dras_fm_dab_adc_dac_write(st, ADDR_RX_DAB_CHANNEL_FREQUENCY, val);
		break;
	case REG_RX_DAB_BAND_BURST_LENGTH:
		dras_fm_dab_adc_dac_write(st, ADDR_RX_DAB_BAND_BURST_LENGTH, (u32)val);
		break;
	case REG_RX_DAB_BAND_BURST_PERIOD:
		dras_fm_dab_adc_dac_write(st, ADDR_RX_DAB_BAND_BURST_PERIOD, (u32)val);
		break;
	case REG_TX1_FM_BAND_GAIN:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		st->gain_fm_tx1 = val;
		if(st->rf_mute)
			break;
		dras_fm_dab_adc_dac_write(st, ADDR_TX_FM_BAND_GAIN,
				(st->gain_fm_tx2 << 16) | st->gain_fm_tx1);
		break;
	case REG_TX2_FM_BAND_GAIN:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		st->gain_fm_tx2 = val;
		if(st->rf_mute)
			break;
		dras_fm_dab_adc_dac_write(st, ADDR_TX_FM_BAND_GAIN,
				(st->gain_fm_tx2 << 16) | st->gain_fm_tx1);
		break;
	case REG_TX_FM_TESTTONE_FREQUENCY0:
		if(val<MIN_FM_FREQUENCY || val>MAX_FM_FREQUENCY){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)st->fs_adc * 15;
		temp64 = div_s64(temp64,44); // fm_f_mix = clk*15/44
		val -= (int)temp64;
		val = 3*val;
		temp64 = (u64)val << 18;
		temp64 = div_s64(temp64,st->fs_adc);
		val = (int)temp64 & 0xFFFF;
		temp32 = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_DDSINC21) & 0xFFFF0000;
		temp32 += (u32)val;
		dras_fm_dab_adc_dac_write(st, ADDR_TX_FM_TESTTONE_DDSINC21, temp32);
		break;
	case REG_TX_FM_TESTTONE_FREQUENCY1:
		if(val<MIN_FM_FREQUENCY || val>MAX_FM_FREQUENCY){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)st->fs_adc * 15;
		temp64 = div_s64(temp64,44); // fm_f_mix = clk*15/44
		val -= (int)temp64;
		val = 3*val;
		temp64 = (u64)val << 18;
		temp64 = div_s64(temp64,st->fs_adc);
		val = (int)temp64 & 0xFFFF;
		temp32 = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_DDSINC21) & 0xFFFF;
		temp32 += (u32)val << 16;
		dras_fm_dab_adc_dac_write(st, ADDR_TX_FM_TESTTONE_DDSINC21, temp32);
		break;
	case REG_TX_FM_TESTTONE_FREQUENCY2:
		if(val<MIN_FM_FREQUENCY || val>MAX_FM_FREQUENCY){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)st->fs_adc * 15;
		temp64 = div_s64(temp64,44); // fm_f_mix = clk*15/44
		val -= (int)temp64;
		val = 3*val;
		temp64 = (u64)val << 18;
		temp64 = div_s64(temp64,st->fs_adc);
		val = (int)temp64 & 0xFFFF;
		temp32 = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_DDSINC43) & 0xFFFF0000;
		temp32 += (u32)val;
		dras_fm_dab_adc_dac_write(st, ADDR_TX_FM_TESTTONE_DDSINC43, temp32);
		break;
	case REG_TX_FM_TESTTONE_FREQUENCY3:
		if(val<MIN_FM_FREQUENCY || val>MAX_FM_FREQUENCY){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)st->fs_adc * 15;
		temp64 = div_s64(temp64,44); // fm_f_mix = clk*15/44
		val -= (int)temp64;
		val = 3*val;
		temp64 = (u64)val << 18;
		temp64 = div_s64(temp64,st->fs_adc);
		val = (int)temp64 & 0xFFFF;
		temp32 = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_DDSINC43) & 0xFFFF;
		temp32 += (u32)val << 16;
		dras_fm_dab_adc_dac_write(st, ADDR_TX_FM_TESTTONE_DDSINC43, temp32);
		break;
	case REG_TX_FM_TESTTONE_AMPLITUDE0:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_AMPL21) & 0xFFFF0000;
		temp32 += (u32)val;
		dras_fm_dab_adc_dac_write(st, ADDR_TX_FM_TESTTONE_AMPL21, temp32);
		break;
	case REG_TX_FM_TESTTONE_AMPLITUDE1:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_AMPL21) & 0xFFFF;
		temp32 += (u32)val << 16;
		dras_fm_dab_adc_dac_write(st, ADDR_TX_FM_TESTTONE_AMPL21, temp32);
		break;
	case REG_TX_FM_TESTTONE_AMPLITUDE2:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_AMPL43) & 0xFFFF0000;
		temp32 += (u32)val;
		dras_fm_dab_adc_dac_write(st, ADDR_TX_FM_TESTTONE_AMPL43, temp32);
		break;
	case REG_TX_FM_TESTTONE_AMPLITUDE3:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_AMPL43) & 0xFFFF;
		temp32 += (u32)val << 16;
		dras_fm_dab_adc_dac_write(st, ADDR_TX_FM_TESTTONE_AMPL43, temp32);
		break;
	case REG_RX_FM_BAND_BURST_LENGTH:
		dras_fm_dab_adc_dac_write(st, ADDR_RX_FM_BAND_BURST_LENGTH, (u32)val);
		break;
	case REG_RX_FM_BAND_BURST_PERIOD:
		dras_fm_dab_adc_dac_write(st, ADDR_RX_FM_BAND_BURST_PERIOD, (u32)val);
		break;
	case REG_WATCHDOG_ENABLE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_fm_dab_adc_dac_read(st, ADDR_WATCHDOG) & ~(0x1<<2);
		temp32 += (u32)val << 2;
		dras_fm_dab_adc_dac_write(st, ADDR_WATCHDOG, temp32);
		break;
	case REG_WATCHDOG_TRIGGER:
		temp32 = dras_fm_dab_adc_dac_read(st, ADDR_WATCHDOG) & ~(0x1<<3);
		temp32 += 1 << 3;
		dras_fm_dab_adc_dac_write(st, ADDR_WATCHDOG, temp32);
		temp32 &= ~(1 << 3);
		dras_fm_dab_adc_dac_write(st, ADDR_WATCHDOG, temp32);
		break;
	case REG_SATURATION_MUTING_OCCURRENCE:
		if(val<0 || val>4095){
			val=4095;
			break;
		}
		temp32 = dras_fm_dab_adc_dac_read(st, ADDR_WATCHDOG) & ~(0xfff<<4);
		temp32 += (u32)val << 4;
		dras_fm_dab_adc_dac_write(st, ADDR_WATCHDOG, temp32);
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
		temp32 = dras_fm_dab_adc_dac_read(st, ADDR_PPS_SETTINGS) & ~(1<<29);
		temp32 += (u32)val<<29;
		dras_fm_dab_adc_dac_write(st, ADDR_PPS_SETTINGS, temp32);
		break;
	case REG_PPS_REFERENCE_FREQUENCY:
		temp32 = dras_fm_dab_adc_dac_read(st, ADDR_PPS_SETTINGS) & ~(0x1FFFFFFF);
		temp32 += (u32)val & 0x1FFFFFFF;
		dras_fm_dab_adc_dac_write(st, ADDR_PPS_SETTINGS, temp32);
		break;
	case REG_GPSDO_LOCKED:
		st->gpsdo_locked = (u32)val & 0x1;
		break;
	case REG_RF_MUTE:
		if((bool)val == st->rf_mute){
			break;
		}
		st->rf_mute = (bool)val;
		if(st->rf_mute){
			dras_fm_dab_adc_dac_write(st, ADDR_TX_DAB_BAND_GAIN, 0);
			dras_fm_dab_adc_dac_write(st, ADDR_TX_FM_BAND_GAIN, 0);
		}
		else{
			dras_fm_dab_adc_dac_write(st, ADDR_TX_DAB_BAND_GAIN,
					(st->gain_dab_tx2 << 16) | st->gain_dab_tx1);
			dras_fm_dab_adc_dac_write(st, ADDR_TX_FM_BAND_GAIN,
					(st->gain_fm_tx2 << 16) | st->gain_fm_tx1);
		}
		break;
	default:
		ret = -ENODEV;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t dras_fm_dab_adc_dac_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dras_fm_dab_adc_dac_state *st = iio_priv(indio_dev);
	int val;
	int ret = 0;
	u64 temp64;
	u32 ch;
	int match;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<NB_OF_DAB_CHANNELS; ch++){
		if((u32)this_attr->address == REG_CH(ch, REG_TX_DAB_CHANNEL_TESTTONE_AMPLITUDE)){
			match = 1;
			val = dras_fm_dab_adc_dac_read(st, ADDR_TX_DAB_TESTTONE_AMPL(ch));
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX_DAB_CHANNEL_FREQUENCY)){
			match = 1;
			val = dras_fm_dab_adc_dac_read(st, ADDR_TX_DAB_DDSINC(ch));

			temp64 = (u64)val * st->fs_adc;
			val = (u32)(temp64 >> 24);
			val += st->fs_adc>>1;
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
		val = dras_fm_dab_adc_dac_read(st, ADDR_DSP_VERSION);
		break;
	case REG_RX_SAMP_RATE_ADC:
		val = st->fs_adc;
		break;
	case REG_ADC1_PEAK_HOLD_VAL:
		val = (dras_fm_dab_adc_dac_read(st, ADDR_ADC_PEAK) & 0xFFFF);
		break;
	case REG_ADC2_PEAK_HOLD_VAL:
		val = (dras_fm_dab_adc_dac_read(st, ADDR_ADC_PEAK) >> 16);
		break;
	case REG_ADC_BER_ALTERNATE:
		val = (dras_fm_dab_adc_dac_read(st, ADDR_ADC_BER_TESTER) & 0xFFFF);
		break;
	case REG_ADC_BER_CHECKER:
		val = (dras_fm_dab_adc_dac_read(st, ADDR_ADC_BER_TESTER) >> 16);
		break;
	case REG_RX_DAB_CHANNEL_FREQUENCY:
		val = dras_fm_dab_adc_dac_read(st, ADDR_RX_DAB_CHANNEL_FREQUENCY);
		temp64 = (u64)val * st->fs_adc;
		val = (u32)(temp64 >> 24);
		val = st->fs_adc - val;
		break;
	case REG_TX1_DAB_BAND_GAIN:
		val = st->gain_dab_tx1;
		break;
	case REG_TX2_DAB_BAND_GAIN:
		val = st->gain_dab_tx2;
		break;
	case REG_RX_DAB_BAND_BURST_LENGTH:
		val = dras_fm_dab_adc_dac_read(st, ADDR_RX_DAB_BAND_BURST_LENGTH);
		break;
	case REG_RX_DAB_BAND_BURST_PERIOD:
		val = dras_fm_dab_adc_dac_read(st, ADDR_RX_DAB_BAND_BURST_PERIOD);
		break;
	case REG_RX_FM_BAND_BURST_LENGTH:
		val = dras_fm_dab_adc_dac_read(st, ADDR_RX_FM_BAND_BURST_LENGTH);
		break;
	case REG_RX_FM_BAND_BURST_PERIOD:
		val = dras_fm_dab_adc_dac_read(st, ADDR_RX_FM_BAND_BURST_PERIOD);
		break;
	case REG_TX1_FM_BAND_GAIN:
		val = st->gain_fm_tx1;
		break;
	case REG_TX2_FM_BAND_GAIN:
		val = st->gain_fm_tx2;
		break;
	case REG_TX_FM_TESTTONE_FREQUENCY0:
		val = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_DDSINC21) & 0xFFFF;
		if(val>1<<15){
			temp64 = (u64)val * st->fs_adc;
			val = ((int)(temp64 >> 18)) - (st->fs_adc>>2); // f_test = fm_f_mix+(fm_dds_inc*clk/2^18-clk/4)/3
		}else{
			temp64 = (u64)val * st->fs_adc;
			val = (u32)(temp64 >> 18); // f_test = fm_f_mix+fm_dds_inc*clk/2^18/3
		}
		val = val/3;
		temp64 = (u64)st->fs_adc * 15;
		temp64 = div_s64(temp64,44); // fm_f_mix = clk*15/44
		val += (int)temp64;
		break;
	case REG_TX_FM_TESTTONE_FREQUENCY1:
		val = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_DDSINC21) >> 16;
		if(val>1<<15){
			temp64 = (u64)val * st->fs_adc;
			val = ((int)(temp64 >> 18)) - (st->fs_adc>>2); // f_test = fm_f_mix+(fm_dds_inc*clk/2^18-clk/4)/3
		}else{
			temp64 = (u64)val * st->fs_adc;
			val = (u32)(temp64 >> 18); // f_test = fm_f_mix+fm_dds_inc*clk/2^18/3
		}
		val = val/3;
		temp64 = (u64)st->fs_adc * 15;
		temp64 = div_s64(temp64,44); // fm_f_mix = clk*15/44
		val += (int)temp64;
		break;
	case REG_TX_FM_TESTTONE_FREQUENCY2:
		val = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_DDSINC43) & 0xFFFF;
		if(val>1<<15){
			temp64 = (u64)val * st->fs_adc;
			val = ((int)(temp64 >> 18)) - (st->fs_adc>>2); // f_test = fm_f_mix+(fm_dds_inc*clk/2^18-clk/4)/3
		}else{
			temp64 = (u64)val * st->fs_adc;
			val = (u32)(temp64 >> 18); // f_test = fm_f_mix+fm_dds_inc*clk/2^18/3
		}
		val = val/3;
		temp64 = (u64)st->fs_adc * 15;
		temp64 = div_s64(temp64,44); // fm_f_mix = clk*15/44
		val += (int)temp64;
		break;
	case REG_TX_FM_TESTTONE_FREQUENCY3:
		val = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_DDSINC43) >> 16;
		if(val>1<<15){
			temp64 = (u64)val * st->fs_adc;
			val = ((int)(temp64 >> 18)) - (st->fs_adc>>2); // f_test = fm_f_mix+(fm_dds_inc*clk/2^18-clk/4)/3
		}else{
			temp64 = (u64)val * st->fs_adc;
			val = (u32)(temp64 >> 18); // f_test = fm_f_mix+fm_dds_inc*clk/2^18/3
		}
		val = val/3;
		temp64 = (u64)st->fs_adc * 15;
		temp64 = div_s64(temp64,44); // fm_f_mix = clk*15/44
		val += (int)temp64;
		break;
	case REG_TX_FM_TESTTONE_AMPLITUDE0:
		val = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_AMPL21) & 0xFFFF;
		break;
	case REG_TX_FM_TESTTONE_AMPLITUDE1:
		val = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_AMPL21) >> 16;
		break;
	case REG_TX_FM_TESTTONE_AMPLITUDE2:
		val = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_AMPL43) & 0xFFFF;
		break;
	case REG_TX_FM_TESTTONE_AMPLITUDE3:
		val = dras_fm_dab_adc_dac_read(st, ADDR_TX_FM_TESTTONE_AMPL43) >> 16;
		break;
	case REG_WATCHDOG_ENABLE:
		val = (dras_fm_dab_adc_dac_read(st, ADDR_WATCHDOG) >> 2) & 0x1;
		break;
	case REG_WATCHDOG_TRIGGER:
		val = (dras_fm_dab_adc_dac_read(st, ADDR_WATCHDOG) >> 3) & 0x1;
		break;
	case REG_SATURATION_MUTING_OCCURRENCE:
		val = (dras_fm_dab_adc_dac_read(st, ADDR_WATCHDOG) >> 4) & 0xfff;
		break;
	case REG_PPS_DIRECTION_OUT_N_IN:
		val = (dras_fm_dab_adc_dac_read(st, ADDR_PPS_SETTINGS) >>29) & 1;
		break;
	case REG_PPS_CLK_ERROR:
		val = dras_fm_dab_adc_dac_read(st, ADDR_PPS_CLKS) & 0x1FFFFFFF;
		break;
	case REG_PPS_CLK_ERROR_HZ:
		val = st->pps_clk_error_hz;
		break;
	case REG_PPS_CLK_ERROR_NS:
		val = st->pps_clk_error_ns;
		break;
	case REG_PPS_REFERENCE_FREQUENCY:
		val = dras_fm_dab_adc_dac_read(st, ADDR_PPS_SETTINGS) & 0x1FFFFFFF;
		break;
	case REG_PPS_CNT:
		val = dras_fm_dab_adc_dac_read(st, ADDR_PPS_CNT);
		break;
	case REG_GPSDO_LOCKED:
		val = st->gpsdo_locked;
		break;
	case REG_RF_MUTE:
		val = st->rf_mute;
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


IIO_DEVICE_ATTR_ALL_CH(tx_dab_testtone_amplitude, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX_DAB_CHANNEL_TESTTONE_AMPLITUDE);

IIO_DEVICE_ATTR_ALL_CH(tx_dab_frequency, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX_DAB_CHANNEL_FREQUENCY);

static IIO_DEVICE_ATTR(dsp_version, S_IRUGO,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_DSP_VERSION);

static IIO_DEVICE_ATTR(adc1_peak_hold_value, S_IRUGO,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_ADC1_PEAK_HOLD_VAL);

static IIO_DEVICE_ATTR(adc2_peak_hold_value, S_IRUGO,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_ADC2_PEAK_HOLD_VAL);

static IIO_DEVICE_ATTR(adc_ber_alternate, S_IRUGO,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_ADC_BER_ALTERNATE);

static IIO_DEVICE_ATTR(adc_ber_checker, S_IRUGO,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_ADC_BER_CHECKER);

static IIO_DEVICE_ATTR(rx_dab_band_burst_length, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_RX_DAB_BAND_BURST_LENGTH);

static IIO_DEVICE_ATTR(rx_dab_band_burst_period, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_RX_DAB_BAND_BURST_PERIOD);

static IIO_DEVICE_ATTR(rx_dab_channel_frequency, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_RX_DAB_CHANNEL_FREQUENCY);

static IIO_DEVICE_ATTR(tx1_dab_band_gain, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX1_DAB_BAND_GAIN);

static IIO_DEVICE_ATTR(tx2_dab_band_gain, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX2_DAB_BAND_GAIN);

static IIO_DEVICE_ATTR(rx_fm_band_burst_length, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_RX_FM_BAND_BURST_LENGTH);

static IIO_DEVICE_ATTR(rx_fm_band_burst_period, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_RX_FM_BAND_BURST_PERIOD);

static IIO_DEVICE_ATTR(tx1_fm_band_gain, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX1_FM_BAND_GAIN);

static IIO_DEVICE_ATTR(tx2_fm_band_gain, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX2_FM_BAND_GAIN);

static IIO_DEVICE_ATTR(ch0_tx_fm_testtone_frequency, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX_FM_TESTTONE_FREQUENCY0);

static IIO_DEVICE_ATTR(ch1_tx_fm_testtone_frequency, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX_FM_TESTTONE_FREQUENCY1);

static IIO_DEVICE_ATTR(ch2_tx_fm_testtone_frequency, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX_FM_TESTTONE_FREQUENCY2);

static IIO_DEVICE_ATTR(ch3_tx_fm_testtone_frequency, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX_FM_TESTTONE_FREQUENCY3);

static IIO_DEVICE_ATTR(ch0_tx_fm_testtone_amplitude, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX_FM_TESTTONE_AMPLITUDE0);

static IIO_DEVICE_ATTR(ch1_tx_fm_testtone_amplitude, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX_FM_TESTTONE_AMPLITUDE1);

static IIO_DEVICE_ATTR(ch2_tx_fm_testtone_amplitude, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX_FM_TESTTONE_AMPLITUDE2);

static IIO_DEVICE_ATTR(ch3_tx_fm_testtone_amplitude, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_TX_FM_TESTTONE_AMPLITUDE3);

static IIO_DEVICE_ATTR(watchdog_enable, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_WATCHDOG_ENABLE);

static IIO_DEVICE_ATTR(watchdog_trigger, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_WATCHDOG_TRIGGER);

static IIO_DEVICE_ATTR(saturation_muting_occurrence, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_SATURATION_MUTING_OCCURRENCE);

static IIO_DEVICE_ATTR(pps_direction_out_n_in, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_PPS_DIRECTION_OUT_N_IN);

static IIO_DEVICE_ATTR(pps_clk_error, S_IRUGO,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_PPS_CLK_ERROR);

static IIO_DEVICE_ATTR(pps_clk_error_ns, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_PPS_CLK_ERROR_NS);

static IIO_DEVICE_ATTR(pps_clk_error_hz, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_PPS_CLK_ERROR_HZ);

static IIO_DEVICE_ATTR(pps_reference_frequency, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_PPS_REFERENCE_FREQUENCY);

static IIO_DEVICE_ATTR(gpsdo_locked, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_GPSDO_LOCKED);

static IIO_DEVICE_ATTR(pps_cnt, S_IRUGO,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_PPS_CNT);

static IIO_DEVICE_ATTR(rx_samp_rate_adc, S_IRUGO,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_RX_SAMP_RATE_ADC);

static IIO_DEVICE_ATTR(rf_mute, S_IRUGO | S_IWUSR,
			dras_fm_dab_adc_dac_show,
			dras_fm_dab_adc_dac_store,
			REG_RF_MUTE);


static struct attribute *dras_fm_dab_adc_dac_attributes[] = {
	IIO_ATTR_ALL_CH(tx_dab_testtone_amplitude),
	IIO_ATTR_ALL_CH(tx_dab_frequency),
	&iio_dev_attr_dsp_version.dev_attr.attr,
	&iio_dev_attr_adc1_peak_hold_value.dev_attr.attr,
	&iio_dev_attr_adc2_peak_hold_value.dev_attr.attr,
	&iio_dev_attr_adc_ber_alternate.dev_attr.attr,
	&iio_dev_attr_adc_ber_checker.dev_attr.attr,
	&iio_dev_attr_rx_dab_band_burst_length.dev_attr.attr,
	&iio_dev_attr_rx_dab_band_burst_period.dev_attr.attr,
	&iio_dev_attr_rx_dab_channel_frequency.dev_attr.attr,
	&iio_dev_attr_tx1_dab_band_gain.dev_attr.attr,
	&iio_dev_attr_tx2_dab_band_gain.dev_attr.attr,
	&iio_dev_attr_rx_fm_band_burst_length.dev_attr.attr,
	&iio_dev_attr_rx_fm_band_burst_period.dev_attr.attr,
	&iio_dev_attr_tx1_fm_band_gain.dev_attr.attr,
	&iio_dev_attr_tx2_fm_band_gain.dev_attr.attr,
	&iio_dev_attr_ch0_tx_fm_testtone_frequency.dev_attr.attr,
	&iio_dev_attr_ch1_tx_fm_testtone_frequency.dev_attr.attr,
	&iio_dev_attr_ch2_tx_fm_testtone_frequency.dev_attr.attr,
	&iio_dev_attr_ch3_tx_fm_testtone_frequency.dev_attr.attr,
	&iio_dev_attr_ch0_tx_fm_testtone_amplitude.dev_attr.attr,
	&iio_dev_attr_ch1_tx_fm_testtone_amplitude.dev_attr.attr,
	&iio_dev_attr_ch2_tx_fm_testtone_amplitude.dev_attr.attr,
	&iio_dev_attr_ch3_tx_fm_testtone_amplitude.dev_attr.attr,
	&iio_dev_attr_watchdog_enable.dev_attr.attr,
	&iio_dev_attr_watchdog_trigger.dev_attr.attr,
	&iio_dev_attr_saturation_muting_occurrence.dev_attr.attr,
	&iio_dev_attr_rx_samp_rate_adc.dev_attr.attr,
	&iio_dev_attr_pps_direction_out_n_in.dev_attr.attr,
	&iio_dev_attr_pps_clk_error.dev_attr.attr,
	&iio_dev_attr_pps_clk_error_ns.dev_attr.attr,
	&iio_dev_attr_pps_clk_error_hz.dev_attr.attr,
	&iio_dev_attr_pps_reference_frequency.dev_attr.attr,
	&iio_dev_attr_pps_cnt.dev_attr.attr,
	&iio_dev_attr_gpsdo_locked.dev_attr.attr,
	&iio_dev_attr_rf_mute.dev_attr.attr,
	NULL,
};


static const struct attribute_group dras_fm_dab_adc_dac_attribute_group = {
	.attrs = dras_fm_dab_adc_dac_attributes,
};

static const struct iio_info dras_fm_dab_adc_dac_info = {
	.read_raw = &dras_fm_dab_adc_dac_read_raw,
	.write_raw = &dras_fm_dab_adc_dac_write_raw,
	.attrs = &dras_fm_dab_adc_dac_attribute_group,
};

static const struct iio_chan_spec dras_fm_dab_adc_dac_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id dras_fm_dab_adc_dac_of_match[] = {
	{ .compatible = "fpga,dras-fm-dab-adc-dac", },
	{ },
};

MODULE_DEVICE_TABLE(of, dras_fm_dab_adc_dac_of_match);

static int dras_fm_dab_adc_dac_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct dras_fm_dab_adc_dac_state *st;
	struct iio_dev *indio_dev;
	int ret; //, i, n;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(dras_fm_dab_adc_dac_of_match, &pdev->dev);
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
		printk("DRAS-FM-DAB-ADC-DAC: ***ERROR! \"required,fs-adc\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->fs_adc == 0){
		printk("DRAS-FM-DAB-ADC-DAC: ***ERROR! \"required,fs-adc\" equal to 0 Hz\n");
		goto err_iio_device_free;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->channels = dras_fm_dab_adc_dac_channels;
	indio_dev->num_channels = ARRAY_SIZE(dras_fm_dab_adc_dac_channels);
	indio_dev->info = &dras_fm_dab_adc_dac_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	/* initially mute TX of both RF bands */
	st->rf_mute = true;
   	// dras_fm_dab_adc_dac_write(st, ADDR_TX_DAB_BAND_GAIN, 0);
   	// dras_fm_dab_adc_dac_write(st, ADDR_TX_FM_BAND_GAIN, 0);

	//dras_fm_dab_adc_dac_write(st, ADDR_RX_FM_BAND_BURST_PERIOD, 2389333); 	// Fs/10 > 10Hz update rate
	//dras_fm_dab_adc_dac_write(st, ADDR_RX_FM_BAND_BURST_LENGTH, 2048);	// 11.7kHz RBW @ 2k FFT
	//dras_fm_dab_adc_dac_write(st, ADDR_RX_DAB_BAND_BURST_PERIOD, 14336000); // Fs/10 > 10Hz update rate
	//dras_fm_dab_adc_dac_write(st, ADDR_RX_DAB_BAND_BURST_LENGTH, 4096);	// 35kHz RBW @ 4k FFT

	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, indio_dev);
	return 0;

err_iio_device_free:
	iio_device_free(indio_dev);
	return ret;
}

static int dras_fm_dab_adc_dac_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver dras_fm_dab_adc_dac_driver = {
	.probe		= dras_fm_dab_adc_dac_probe,
	.remove		= dras_fm_dab_adc_dac_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = dras_fm_dab_adc_dac_of_match,
	},
};

module_platform_driver(dras_fm_dab_adc_dac_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("DRAS FM DAB ADC DAC FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
