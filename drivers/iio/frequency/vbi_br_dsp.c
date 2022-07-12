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
#include "br_vbi_dsp_fir_coef.h"


#define DRIVER_NAME			"vbi-br-dsp"
#define NB_OF_BLOCKS			4
#define DDS_PHASEWIDTH			32
#define MIN_RX_DECIMATION		1
#define MAX_RX_DECIMATION		20


#define ADDR_DSP_VERSION		0*4
#define ADDR_GPI			3*4
#define ADDR_GPO			4*4
#define ADDR_PPS_SETTINGS		5*4
#define ADDR_PPS_ERROR			6*4
#define ADDR_PPS_CNT			7*4
#define ADDR_PPS_DELAY			8*4
#define ADDR_RX_ADC_PEAK		10*4

#define ADDR_OFFSET_DSP			0x100
#define ADDR_PER_BLOCK			0x100
#define ADDR_SETTINGS(x)		(ADDR_OFFSET_DSP+0*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_INC(x)			(ADDR_OFFSET_DSP+1*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_FIR_COEFS(x)		(ADDR_OFFSET_DSP+2*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_DECIMATION(x)		(ADDR_OFFSET_DSP+3*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_PERIOD(x)		(ADDR_OFFSET_DSP+4*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_FRAME_LENGTH(x)		(ADDR_OFFSET_DSP+5*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_OVERFLOW_SAMPLES(x)	(ADDR_OFFSET_DSP+6*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_STREAM_TO_PPS_OFFSET(x)	(ADDR_OFFSET_DSP+7*4+x*ADDR_PER_BLOCK)


#define REG_CH(CHANNEL, REG) \
	(CH0_##REG + CHANNEL)

#define REG_ALL_CH(REG) \
	CH0_##REG, \
	CH1_##REG, \
	CH2_##REG, \
	CH3_##REG

#define IIO_DEVICE_ATTR_ALL_CH(ATTR, RW, SHOW, STORE, REG) \
	static IIO_DEVICE_ATTR(ch0_##ATTR, RW, SHOW, STORE, CH0_##REG); \
	static IIO_DEVICE_ATTR(ch1_##ATTR, RW, SHOW, STORE, CH1_##REG); \
	static IIO_DEVICE_ATTR(ch2_##ATTR, RW, SHOW, STORE, CH2_##REG); \
	static IIO_DEVICE_ATTR(ch3_##ATTR, RW, SHOW, STORE, CH3_##REG);

#define IIO_ATTR_ALL_CH(ATTR) \
	&iio_dev_attr_ch0_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch1_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch2_##ATTR.dev_attr.attr, \
	&iio_dev_attr_ch3_##ATTR.dev_attr.attr

enum chan_num{
	REG_ALL_CH(REG_RX_SAMP_RATE_BASEBAND),
	REG_ALL_CH(REG_RX_FREQUENCY),
	REG_ALL_CH(REG_RX_BYPASS_DECIMATOR),
	REG_ALL_CH(REG_RX_PERIOD),
	REG_ALL_CH(REG_RX_FRAME_LENGTH),
	REG_ALL_CH(REG_RX_INIT_STREAMING),
	REG_ALL_CH(REG_RX_ENABLE_HEADER),
	REG_ALL_CH(REG_RX_SWAP_IQ),
	REG_ALL_CH(REG_RX_ENABLE_IQ_TESTDATA),
	REG_ALL_CH(REG_RX_ENABLE_IQ_TESTDATA_PER_FRAME),
	REG_ALL_CH(REG_RX_FIR_DECIMATION),
	REG_ALL_CH(REG_RX_FIR_COEFFICIENTS),
	REG_ALL_CH(REG_RX_BUFFER_OVERFLOW_SAMPLES),
	REG_ALL_CH(REG_RX_STREAM_TO_PPS_OFFSET_CLOCKCYCLES),
	REG_RX_SAMP_RATE_ADC,
	REG_RX_ADC1_PEAK_HOLD_VAL,
	REG_RX_ADC2_PEAK_HOLD_VAL,
	REG_RX_ADC_PEAK_HOLD_RESET,
	REG_RX_LNA_PEAK_DETECT_FLAG,
	REG_RX_LED_GREEN,
	REG_RX_LED_RED,
	REG_TX_LED_GREEN,
	REG_TX_LED_RED,
	REG_GPO_VALUE,
	REG_GPI_VALUE,
	REG_PPS_DIRECTION_OUT_N_IN,
	REG_PPS_CLK_ERROR,
	REG_PPS_CLK_ERROR_NS,
	REG_PPS_CLK_ERROR_HZ,
	REG_PPS_CNT,
	REG_PPS_REFERENCE_FREQUENCY,
	REG_PPS_DELAY,
	REG_GPSDO_LOCKED,
	REG_INT_TO_VOLT_SCALAR,
	REG_DSP_VERSION
};

struct vbi_br_dsp_state {
	struct iio_info	iio_info;
	void __iomem	*regs;
	struct mutex	lock;

	uint32_t	fs_adc;
	bool 		gpsdo_locked;
	bool 		rx_swap_iq[NB_OF_BLOCKS];
  	uint64_t	rx_frequency[NB_OF_BLOCKS];
  	uint32_t	pps_clk_error_ns;
  	uint32_t	pps_clk_error_hz;
	uint32_t	rx_nyquist_zone[NB_OF_BLOCKS];
	uint32_t	nb_of_blocks_dt;
	uint64_t	int_to_volt_scalar;
};

static void vbi_br_dsp_write(struct vbi_br_dsp_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 vbi_br_dsp_read(struct vbi_br_dsp_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int vbi_br_dsp_write_raw(struct iio_dev *indio_dev,
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

static int vbi_br_dsp_read_raw(struct iio_dev *indio_dev,
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



static ssize_t vbi_br_dsp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct vbi_br_dsp_state *st = iio_priv(indio_dev);
	long val;
	int ret, match;
	uint64_t temp64;
	uint32_t temp32, i, ch;
	unsigned long long readin;
	int attr_is_64bits = 0;


	/* convert to long
	 * auto-detect decimal,
	 * octal (beginning with 0) and
	 * hexadecimal (beginning with 0x)
	 */
	attr_is_64bits = (u32)this_attr->address == REG_INT_TO_VOLT_SCALAR;
	for (ch=0; ch < st->nb_of_blocks_dt; ch++) {
		if ((u32)this_attr->address == REG_CH(ch, REG_RX_FREQUENCY)) {
			attr_is_64bits = 1;
		}
	}

	if (attr_is_64bits) {
		ret = kstrtoull(buf, 10, &readin);
		if (ret < 0)
			return ret;
	} else {
		ret = kstrtol(buf, 0, &val);
		if (ret < 0)
			return ret;
	}

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<st->nb_of_blocks_dt; ch++){

		if((u32)this_attr->address == REG_CH(ch, REG_RX_FREQUENCY)){
			match = 1;
			// calculate phase increment word

	// version freq in Hz
	//		st->rx_nyquist_zone = (u32)val/(st->fs_adc/2) + 1;
	//		if(val > st->fs_adc/2)
	//			val %= st->fs_adc/2;
	//		if((st->rx_nyquist_zone & 1) == 0)
	//			val = st->fs_adc/2 - val;
	//		temp64 = (u64)val << DDS_PHASEWIDTH;
	//		temp64 = div_s64(temp64,st->fs_adc);

	// version freq in milliherz
			temp64 = readin;
			st->rx_nyquist_zone[ch] = (u32)div64_u64(temp64,500ULL*(u64)st->fs_adc) + 1; // calculate and store nyquist zone
			temp64 = div_u64(temp64,50); // scale milliherz frequency to 1/25 Hz steps, otherwise 64bit is not enough
			//printk("nyquist=%d, scaled readin=%llu\n", st->rx_nyquist_zone, temp64);
			if(temp64 > 10*(u64)st->fs_adc)
				div64_u64_rem(temp64, 10*(u64)st->fs_adc, &temp64);
				//temp64 %= 10*(u64)st->fs_adc;
			if((st->rx_nyquist_zone[ch] & 1) == 0)
				temp64 = 10*(u64)st->fs_adc - temp64;
			temp64 = temp64 << DDS_PHASEWIDTH;
			temp64 = div64_u64(temp64,20*(u64)st->fs_adc);

			vbi_br_dsp_write(st, ADDR_RX_INC(ch), (u32)temp64); // write phase increment word

			// swap IQ on even nyquist zones
			val = (st->rx_nyquist_zone[ch] & 1) ^ st->rx_swap_iq[ch];
			temp32 = vbi_br_dsp_read(st, ADDR_SETTINGS(ch)) & ~(1<<1);
			temp32 += (u32)val << 1;
			vbi_br_dsp_write(st, ADDR_SETTINGS(ch), temp32);

	    		// recalculate frequency
	// version freq in Hz
	//    		temp64 = (u64)tempint * st->fs_adc;
	//		if(st->rx_nyquist_zone & 1)
	//	    		st->rx_frequency = (u32)(temp64 >> DDS_PHASEWIDTH) + st->fs_adc/2 * (st->rx_nyquist_zone - 1);
	//		else
	//			st->rx_frequency = st->fs_adc/2 * st->rx_nyquist_zone - (u32)(temp64 >> DDS_PHASEWIDTH);

	// version freq in milliherz
	    		temp64 = temp64 * 20 * (u64)st->fs_adc;
			if(st->rx_nyquist_zone[ch] & 1)
		    		st->rx_frequency[ch] = 50*(temp64 >> DDS_PHASEWIDTH) + 500*(u64)st->fs_adc * (u64)(st->rx_nyquist_zone[ch] - 1);
			else
				st->rx_frequency[ch] = 500*(u64)st->fs_adc * (u64)st->rx_nyquist_zone[ch] - 50*(temp64 >> DDS_PHASEWIDTH);
			//printk("nyquist=%d, freq=%llu\n", st->rx_nyquist_zone[ch], st->rx_frequency[ch]);
			break;
		}
		if((u32)this_attr->address == REG_CH(ch, REG_RX_BYPASS_DECIMATOR)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_br_dsp_read(st, ADDR_SETTINGS(ch)) & ~(1<<2);
			temp32 += (u32)val << 2;
			vbi_br_dsp_write(st, ADDR_SETTINGS(ch), temp32);
			break;
		}
		if((u32)this_attr->address == REG_CH(ch, REG_RX_INIT_STREAMING)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_br_dsp_read(st, ADDR_SETTINGS(ch)) & ~(1<<3);
			temp32 += (u32)val<<3;
			vbi_br_dsp_write(st, ADDR_SETTINGS(ch), temp32);
			break;
		}
		if((u32)this_attr->address == REG_CH(ch, REG_RX_ENABLE_HEADER)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_br_dsp_read(st, ADDR_SETTINGS(ch)) & ~(1<<0);
			temp32 += (u32)val<<0;
			vbi_br_dsp_write(st, ADDR_SETTINGS(ch), temp32);
			break;
		}
		if((u32)this_attr->address == REG_CH(ch, REG_RX_SWAP_IQ)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			st->rx_swap_iq[ch] = val;
			// swap IQ on even nyquist zones
			val = (st->rx_nyquist_zone[ch] & 1) ^ st->rx_swap_iq[ch];
			temp32 = vbi_br_dsp_read(st, ADDR_SETTINGS(ch)) & ~(1<<1);
			temp32 += (u32)val << 1;
			vbi_br_dsp_write(st, ADDR_SETTINGS(ch), temp32);
			break;
		}
		if((u32)this_attr->address == REG_CH(ch, REG_RX_ENABLE_IQ_TESTDATA)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_br_dsp_read(st, ADDR_SETTINGS(ch)) & ~(1<<4);
			temp32 += (u32)val<<4;
			vbi_br_dsp_write(st, ADDR_SETTINGS(ch), temp32);
			break;
		}
		if((u32)this_attr->address == REG_CH(ch, REG_RX_ENABLE_IQ_TESTDATA_PER_FRAME)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_br_dsp_read(st, ADDR_SETTINGS(ch)) & ~(1<<5);
			temp32 += (u32)val<<5;
			vbi_br_dsp_write(st, ADDR_SETTINGS(ch), temp32);
			break;
		}
		if((u32)this_attr->address == REG_CH(ch, REG_RX_PERIOD)){
			match = 1;
			vbi_br_dsp_write(st, ADDR_RX_PERIOD(ch), (u32)val);
			break;
		}
		if((u32)this_attr->address == REG_CH(ch, REG_RX_FRAME_LENGTH)){
			match = 1;
			vbi_br_dsp_write(st, ADDR_RX_FRAME_LENGTH(ch), (u32)val);
			break;
		}
		if((u32)this_attr->address == REG_CH(ch, REG_RX_FIR_DECIMATION)){
			match = 1;
			if(val<MIN_RX_DECIMATION || val>MAX_RX_DECIMATION){
				ret = -EINVAL;
				break;
			}
			vbi_br_dsp_write(st, ADDR_RX_DECIMATION(ch), (u32)val);
			
			for(i=0; i<180; i++){
				vbi_br_dsp_write(st, ADDR_RX_FIR_COEFS(ch), (u32)fir_coefs[(u32)(val - 1)][i] + (u32)((i+1)<<16));
			}
			
			break;
		}
		if((u32)this_attr->address == REG_CH(ch, REG_RX_FIR_COEFFICIENTS)){
			match = 1;
			vbi_br_dsp_write(st, ADDR_RX_FIR_COEFS(ch), (u32)val);
			break;
		}
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		return ret ? ret : len;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_INT_TO_VOLT_SCALAR:
		st->int_to_volt_scalar = readin;
		break;
	
  	case REG_RX_SAMP_RATE_ADC:
		match = 1;		
		if(val<1){
			ret = -EINVAL;
			break;
		}
		st->fs_adc = val;

    		// recalculate downconverter frequency
// version freq in Hz
//    		val = st->rx_frequency;
//		st->rx_nyquist_zone = (u32)val/(st->fs_adc/2) + 1;
//		if(val > st->fs_adc/2)
//			val %= st->fs_adc/2;
//		if((st->rx_nyquist_zone & 1) == 0)
//			val = st->fs_adc/2 - val;
//		temp64 = (u64)val << DDS_PHASEWIDTH;
//		temp64 = div_s64(temp64,st->fs_adc);

// version freq in milliherz
		for(i=0; i<st->nb_of_blocks_dt; i++){
	    		temp64 = st->rx_frequency[i];
			st->rx_nyquist_zone[i] = (u32)div64_u64(temp64,500ULL*(u64)st->fs_adc) + 1; // calculate and store nyquist zone
			temp64 = div_u64(temp64,50); // scale milliherz frequency to 1/25 Hz steps, otherwise 64bit is not enough
			if(temp64 > 10*(u64)st->fs_adc)
				div64_u64_rem(temp64, 10*(u64)st->fs_adc, &temp64);
				//temp64 %= 10*(u64)st->fs_adc;
			if((st->rx_nyquist_zone[i] & 1) == 0)
				temp64 = 10*(u64)st->fs_adc - temp64;
			temp64 = temp64 << DDS_PHASEWIDTH;
			temp64 = div64_u64(temp64,20*(u64)st->fs_adc);

			vbi_br_dsp_write(st, ADDR_RX_INC(i), (u32)temp64); // write phase increment word

			// swap IQ on even nyquist zones
			val = (st->rx_nyquist_zone[i] & 1) ^ st->rx_swap_iq[i];
			temp32 = vbi_br_dsp_read(st, ADDR_SETTINGS(i)) & ~(1<<1);
			temp32 += (u32)val << 1;
			vbi_br_dsp_write(st, ADDR_SETTINGS(i), temp32);
		}
    		break;

	case REG_RX_ADC_PEAK_HOLD_RESET:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_br_dsp_read(st, ADDR_PPS_SETTINGS) & ~(1<<31);
		temp32 += (u32)val<<31;
		vbi_br_dsp_write(st, ADDR_PPS_SETTINGS, temp32);
		break;

	case REG_RX_LED_GREEN:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_br_dsp_read(st, ADDR_GPO) & ~(1<<2);
		temp32 += (u32)val<<2;
		vbi_br_dsp_write(st, ADDR_GPO, temp32);
		break;

	case REG_RX_LED_RED:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_br_dsp_read(st, ADDR_GPO) & ~(1<<3);
		temp32 += (u32)val<<3;
		vbi_br_dsp_write(st, ADDR_GPO, temp32);
		break;

	case REG_TX_LED_GREEN:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_br_dsp_read(st, ADDR_GPO) & ~(1<<1);
		temp32 += (u32)val<<1;
		vbi_br_dsp_write(st, ADDR_GPO, temp32);
		break;

	case REG_TX_LED_RED:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_br_dsp_read(st, ADDR_GPO) & ~(1<<0);
		temp32 += (u32)val<<0;
		vbi_br_dsp_write(st, ADDR_GPO, temp32);
		break;

	case REG_GPO_VALUE:
		if(val<0 || val>65535){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_br_dsp_read(st, ADDR_GPO) & 0xFFFF0000;
		temp32 += (u32)val;
		vbi_br_dsp_write(st, ADDR_GPO, temp32);
		break;

	case REG_PPS_DIRECTION_OUT_N_IN:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_br_dsp_read(st, ADDR_PPS_SETTINGS) & ~(1<<29);
		temp32 += (u32)val<<29;
		vbi_br_dsp_write(st, ADDR_PPS_SETTINGS, temp32);
		break;

	case REG_PPS_REFERENCE_FREQUENCY:
		temp32 = vbi_br_dsp_read(st, ADDR_PPS_SETTINGS) & ~(0x1FFFFFFF);
		temp32 += (u32)val & 0x1FFFFFFF;
		vbi_br_dsp_write(st, ADDR_PPS_SETTINGS, temp32);
		break;

	case REG_PPS_DELAY:
		temp32 = vbi_br_dsp_read(st, ADDR_PPS_DELAY) & ~(0x1FFFFFFF);
		temp32 += (u32)val & 0x1FFFFFFF;
		vbi_br_dsp_write(st, ADDR_PPS_DELAY, temp32);
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




static ssize_t vbi_br_dsp_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct vbi_br_dsp_state *st = iio_priv(indio_dev);
	uint32_t temp32, ch;
	int ret = 0, val, match;
	uint64_t temp64;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<st->nb_of_blocks_dt; ch++){
		if((u32)this_attr->address == REG_CH(ch, REG_RX_FREQUENCY)){
			match = 1;
	    		temp64 = st->rx_frequency[ch];
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_SAMP_RATE_BASEBAND)){
			match = 1;
			if((vbi_br_dsp_read(st, ADDR_SETTINGS(ch)) >> 2 ) & 1) // read bypass decimator bit
				val = st->fs_adc/4;
			else
				temp32 = vbi_br_dsp_read(st, ADDR_RX_DECIMATION(ch));
				if(temp32==0)
					temp32 = 1;
				val = st->fs_adc/9/temp32;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_BYPASS_DECIMATOR)){
			match = 1;
			val = (vbi_br_dsp_read(st, ADDR_SETTINGS(ch)) >> 2 ) & 1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_INIT_STREAMING)){
			match = 1;
			val = (vbi_br_dsp_read(st, ADDR_SETTINGS(ch)) >> 3) & 1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_ENABLE_HEADER)){
			match = 1;
			val = (vbi_br_dsp_read(st, ADDR_SETTINGS(ch)) >> 0) & 1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_SWAP_IQ)){
			match = 1;
			val = st->rx_swap_iq[ch];
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_ENABLE_IQ_TESTDATA)){
			match = 1;
			val = (vbi_br_dsp_read(st, ADDR_SETTINGS(ch)) >> 4) & 1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_ENABLE_IQ_TESTDATA_PER_FRAME)){
			match = 1;
			val = (vbi_br_dsp_read(st, ADDR_SETTINGS(ch)) >> 5) & 1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_PERIOD)){
			match = 1;
			val = vbi_br_dsp_read(st, ADDR_RX_PERIOD(ch));
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_FRAME_LENGTH)){
			match = 1;
			val = vbi_br_dsp_read(st, ADDR_RX_FRAME_LENGTH(ch));
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_FIR_DECIMATION)){
			match = 1;
			val = vbi_br_dsp_read(st, ADDR_RX_DECIMATION(ch));
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_FIR_COEFFICIENTS)){
			match = 1;
			val = vbi_br_dsp_read(st, ADDR_RX_FIR_COEFS(ch));
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_BUFFER_OVERFLOW_SAMPLES)){
			match = 1;
			val = vbi_br_dsp_read(st, ADDR_RX_OVERFLOW_SAMPLES(ch));
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_STREAM_TO_PPS_OFFSET_CLOCKCYCLES)){
			match = 1;
			val = vbi_br_dsp_read(st, ADDR_RX_STREAM_TO_PPS_OFFSET(ch));
			break;
		}
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		if(ret==0){
			if((u32)this_attr->address == REG_CH(ch, REG_RX_FREQUENCY))
				ret = sprintf(buf, "%llu\n", temp64);
			else
				ret = sprintf(buf, "%d\n", val);
		}
		return ret;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
		case REG_RX_SAMP_RATE_ADC:
			val = st->fs_adc;
			break;
		case REG_RX_ADC_PEAK_HOLD_RESET:
			val = (vbi_br_dsp_read(st, ADDR_PPS_SETTINGS) >> 31) & 1;
			break;
		case REG_RX_ADC1_PEAK_HOLD_VAL:
			val = (vbi_br_dsp_read(st, ADDR_RX_ADC_PEAK) & 0xFFFF);
			break;
		case REG_RX_ADC2_PEAK_HOLD_VAL:
			val = (vbi_br_dsp_read(st, ADDR_RX_ADC_PEAK) >> 16);
			break;
		case REG_RX_LNA_PEAK_DETECT_FLAG:
			val = (vbi_br_dsp_read(st, ADDR_GPI) >> 3) & 0x1;
			break;
		case REG_RX_LED_GREEN:
			val = (vbi_br_dsp_read(st, ADDR_GPO) >> 2) & 0x1;
			break;
		case REG_RX_LED_RED:
			val = (vbi_br_dsp_read(st, ADDR_GPO) >> 3) & 0x1;
			break;
		case REG_TX_LED_GREEN:
			val = (vbi_br_dsp_read(st, ADDR_GPO) >> 1) & 0x1;
			break;
		case REG_TX_LED_RED:
			val = (vbi_br_dsp_read(st, ADDR_GPO) >> 0) & 0x1;
			break;
		case REG_PPS_DIRECTION_OUT_N_IN:
			val = (vbi_br_dsp_read(st, ADDR_PPS_SETTINGS) >>29) & 1;
			break;
		case REG_PPS_CLK_ERROR:
			val = vbi_br_dsp_read(st, ADDR_PPS_ERROR) & 0x1FFFFFFF;
			break;
		case REG_PPS_CLK_ERROR_HZ:
			val = st->pps_clk_error_hz;
			break;
		case REG_PPS_CLK_ERROR_NS:
			val = st->pps_clk_error_ns;
			break;
		case REG_PPS_REFERENCE_FREQUENCY:
			val = vbi_br_dsp_read(st, ADDR_PPS_SETTINGS) & 0x1FFFFFFF;
			break;
		case REG_PPS_DELAY:
			val = vbi_br_dsp_read(st, ADDR_PPS_DELAY) & 0x1FFFFFFF;
			break;
		case REG_PPS_CNT:
			val = vbi_br_dsp_read(st, ADDR_PPS_CNT);
			break;
		case REG_GPSDO_LOCKED:
			val = st->gpsdo_locked;
			break;
		case REG_GPO_VALUE:
			val = vbi_br_dsp_read(st, ADDR_GPO) & 0xFFFF;
			break;
		case REG_GPI_VALUE:
			val = vbi_br_dsp_read(st, ADDR_GPI) & 0xFFFF;
			break;
		case REG_INT_TO_VOLT_SCALAR:
			temp64 = st->int_to_volt_scalar;
			break;
		case REG_DSP_VERSION:
			val = vbi_br_dsp_read(st, ADDR_DSP_VERSION);
			break;
		default:
			ret = -ENODEV;
			break;
	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0){
		if((u32)this_attr->address == REG_INT_TO_VOLT_SCALAR)
			ret = sprintf(buf, "%llu\n", temp64);
		else
			ret = sprintf(buf, "%d\n", val);
	}
	return ret;
}



IIO_DEVICE_ATTR_ALL_CH(rx_samp_rate_baseband, S_IRUGO,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_SAMP_RATE_BASEBAND);

IIO_DEVICE_ATTR_ALL_CH(rx_frequency_milliherz, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_FREQUENCY);

IIO_DEVICE_ATTR_ALL_CH(rx_bypass_decimator, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_BYPASS_DECIMATOR);

IIO_DEVICE_ATTR_ALL_CH(rx_init_streaming, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_INIT_STREAMING);

IIO_DEVICE_ATTR_ALL_CH(rx_enable_header, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_ENABLE_HEADER);

IIO_DEVICE_ATTR_ALL_CH(rx_enable_iq_testdata, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_ENABLE_IQ_TESTDATA);

IIO_DEVICE_ATTR_ALL_CH(rx_enable_iq_testdata_per_frame, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_ENABLE_IQ_TESTDATA_PER_FRAME);

IIO_DEVICE_ATTR_ALL_CH(rx_swap_iq, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_SWAP_IQ);

IIO_DEVICE_ATTR_ALL_CH(rx_period, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_PERIOD);

IIO_DEVICE_ATTR_ALL_CH(rx_frame_length, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_FRAME_LENGTH);

IIO_DEVICE_ATTR_ALL_CH(rx_fir_decimation, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_FIR_DECIMATION);

IIO_DEVICE_ATTR_ALL_CH(rx_fir_coefficients, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_FIR_COEFFICIENTS);

IIO_DEVICE_ATTR_ALL_CH(rx_buffer_overflow_samples, S_IRUGO,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_BUFFER_OVERFLOW_SAMPLES);

IIO_DEVICE_ATTR_ALL_CH(rx_stream_to_pps_offset_clockcycles, S_IRUGO,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_STREAM_TO_PPS_OFFSET_CLOCKCYCLES);

static IIO_DEVICE_ATTR(rx_samp_rate_adc, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_SAMP_RATE_ADC);

static IIO_DEVICE_ATTR(rx_adc_peak_hold_reset, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_ADC_PEAK_HOLD_RESET);

static IIO_DEVICE_ATTR(rx_adc1_peak_hold_val, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_ADC1_PEAK_HOLD_VAL);

static IIO_DEVICE_ATTR(rx_adc2_peak_hold_val, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_ADC2_PEAK_HOLD_VAL);

static IIO_DEVICE_ATTR(rx_lna_peak_detect_flag, S_IRUGO,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_LNA_PEAK_DETECT_FLAG);

static IIO_DEVICE_ATTR(rx_led_green, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_LED_GREEN);

static IIO_DEVICE_ATTR(rx_led_red, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_RX_LED_RED);

static IIO_DEVICE_ATTR(tx_led_green, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_TX_LED_GREEN);

static IIO_DEVICE_ATTR(tx_led_red, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_TX_LED_RED);

static IIO_DEVICE_ATTR(gpo_value, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_GPO_VALUE);

static IIO_DEVICE_ATTR(gpi_value, S_IRUGO,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_GPI_VALUE);

static IIO_DEVICE_ATTR(pps_direction_out_n_in, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_PPS_DIRECTION_OUT_N_IN);

static IIO_DEVICE_ATTR(pps_clk_error, S_IRUGO,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_PPS_CLK_ERROR);

static IIO_DEVICE_ATTR(pps_clk_error_ns, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_PPS_CLK_ERROR_NS);

static IIO_DEVICE_ATTR(pps_clk_error_hz, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_PPS_CLK_ERROR_HZ);

static IIO_DEVICE_ATTR(pps_reference_frequency, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_PPS_REFERENCE_FREQUENCY);

static IIO_DEVICE_ATTR(pps_delay, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_PPS_DELAY);

static IIO_DEVICE_ATTR(gpsdo_locked, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_GPSDO_LOCKED);

static IIO_DEVICE_ATTR(pps_cnt, S_IRUGO,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_PPS_CNT);

static IIO_DEVICE_ATTR(int_to_volt_scalar, S_IRUGO | S_IWUSR,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_INT_TO_VOLT_SCALAR);

static IIO_DEVICE_ATTR(dsp_version, S_IRUGO,
			vbi_br_dsp_show,
			vbi_br_dsp_store,
			REG_DSP_VERSION);



static struct attribute *vbi_br_dsp_attributes[] = {
	IIO_ATTR_ALL_CH(rx_samp_rate_baseband),
	IIO_ATTR_ALL_CH(rx_frequency_milliherz),
	IIO_ATTR_ALL_CH(rx_bypass_decimator),
	IIO_ATTR_ALL_CH(rx_init_streaming),
	IIO_ATTR_ALL_CH(rx_enable_header),
	IIO_ATTR_ALL_CH(rx_swap_iq),
	IIO_ATTR_ALL_CH(rx_enable_iq_testdata),
	IIO_ATTR_ALL_CH(rx_enable_iq_testdata_per_frame),
	IIO_ATTR_ALL_CH(rx_period),
	IIO_ATTR_ALL_CH(rx_frame_length),
	IIO_ATTR_ALL_CH(rx_fir_decimation),
	IIO_ATTR_ALL_CH(rx_fir_coefficients),
	IIO_ATTR_ALL_CH(rx_buffer_overflow_samples),
	IIO_ATTR_ALL_CH(rx_stream_to_pps_offset_clockcycles),
	&iio_dev_attr_rx_samp_rate_adc.dev_attr.attr,
	&iio_dev_attr_rx_adc_peak_hold_reset.dev_attr.attr,
	&iio_dev_attr_rx_adc1_peak_hold_val.dev_attr.attr,
	&iio_dev_attr_rx_adc2_peak_hold_val.dev_attr.attr,
	&iio_dev_attr_rx_lna_peak_detect_flag.dev_attr.attr,
	&iio_dev_attr_rx_led_green.dev_attr.attr,
	&iio_dev_attr_rx_led_red.dev_attr.attr,
	&iio_dev_attr_tx_led_green.dev_attr.attr,
	&iio_dev_attr_tx_led_red.dev_attr.attr,
	&iio_dev_attr_gpo_value.dev_attr.attr,
	&iio_dev_attr_gpi_value.dev_attr.attr,
	&iio_dev_attr_pps_direction_out_n_in.dev_attr.attr,
	&iio_dev_attr_pps_clk_error.dev_attr.attr,
	&iio_dev_attr_pps_clk_error_ns.dev_attr.attr,
	&iio_dev_attr_pps_clk_error_hz.dev_attr.attr,
	&iio_dev_attr_pps_reference_frequency.dev_attr.attr,
	&iio_dev_attr_pps_delay.dev_attr.attr,
	&iio_dev_attr_pps_cnt.dev_attr.attr,
	&iio_dev_attr_gpsdo_locked.dev_attr.attr,
	&iio_dev_attr_int_to_volt_scalar.dev_attr.attr,
	&iio_dev_attr_dsp_version.dev_attr.attr,
	NULL
};


static const struct attribute_group vbi_br_dsp_attribute_group = {
	.attrs = vbi_br_dsp_attributes,
};

static const struct iio_info vbi_br_dsp_info = {
	//.driver_module = THIS_MODULE,
	.read_raw = &vbi_br_dsp_read_raw,
	.write_raw = &vbi_br_dsp_write_raw,
	.attrs = &vbi_br_dsp_attribute_group,
};

static const struct iio_chan_spec vbi_br_dsp_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id vbi_br_dsp_of_match[] = {
	{ .compatible = "fpga,vbi-br-dsp", },
	{ },
};

MODULE_DEVICE_TABLE(of, vbi_br_dsp_of_match);

static int vbi_br_dsp_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct vbi_br_dsp_state *st;
	struct iio_dev *indio_dev;
	u32 temp32,temp32_1;
	int ret;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(vbi_br_dsp_of_match, &pdev->dev);
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


	if(of_property_read_u32(np, "required,fs-adc", &st->fs_adc)){
		printk("VBI-BR-DSP: ***ERROR! \"required,fs-adc\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->fs_adc == 0){
		printk("VBI-BR-DSP: ***ERROR! \"required,fs-adc\" equal to 0 Hz\n");
		goto err_iio_device_free;
	}
	if(of_property_read_u32(np, "required,nb-of-blocks", &st->nb_of_blocks_dt)){
		printk("VBI-BR-DSP: ***ERROR! \"required,nb-of-blocks\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->nb_of_blocks_dt == 0){
		printk("VBI-BR-DSP: ***ERROR! \"required,nb-of-blocks\" equal to 0\n");
		goto err_iio_device_free;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->channels = vbi_br_dsp_channels;
	indio_dev->num_channels = ARRAY_SIZE(vbi_br_dsp_channels);
	indio_dev->info = &vbi_br_dsp_info;
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

static int vbi_br_dsp_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver vbi_br_dsp_driver = {
	.probe		= vbi_br_dsp_probe,
	.remove		= vbi_br_dsp_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vbi_br_dsp_of_match,
	},
};

module_platform_driver(vbi_br_dsp_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("PCW BR-VBI DSP FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
