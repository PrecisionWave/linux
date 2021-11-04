/*
 * DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2014 Analog Devices Inc.
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


#define DRIVER_NAME			"ddc-duc"

#define ADDR_IO_SEL0			0x0000
#define ADDR_IO_SEL1			0x00b8
#define ADDR_DEMOD0_INP_GAIN		0x0004
#define ADDR_MOD0_OUTP_GAIN_TX0		0x0008
#define ADDR_MOD0_OUTP_GAIN_TX1		0x0098
#define ADDR_BYP0_GAIN_TX0			0x000C
#define ADDR_BYP0_GAIN_TX1			0x009c
#define ADDR_INC_IN0			0x0010
#define ADDR_RSSI_OUT0			0x0014
#define ADDR_DEMOD1_INP_GAIN		0x0018
#define ADDR_MOD1_OUTP_GAIN_TX0		0x001C
#define ADDR_MOD1_OUTP_GAIN_TX1		0x00a0
#define ADDR_BYP1_GAIN_TX0			0x0020
#define ADDR_BYP1_GAIN_TX1			0x00a4
#define ADDR_INC_IN1			0x0024
#define ADDR_RSSI_OUT1			0x0028
#define ADDR_DEMOD2_INP_GAIN		0x002C
#define ADDR_MOD2_OUTP_GAIN_TX0		0x0030
#define ADDR_MOD2_OUTP_GAIN_TX1		0x00a8
#define ADDR_BYP2_GAIN_TX0			0x0034
#define ADDR_BYP2_GAIN_TX1			0x00ac
#define ADDR_INC_IN2			0x0038
#define ADDR_RSSI_OUT2			0x003C
#define ADDR_DEMOD3_INP_GAIN		0x0040
#define ADDR_MOD3_OUTP_GAIN_TX0		0x0044
#define ADDR_MOD3_OUTP_GAIN_TX1		0x00b0
#define ADDR_BYP3_GAIN_TX0			0x0048
#define ADDR_BYP3_GAIN_TX1			0x00b4
#define ADDR_INC_IN3			0x004C
#define ADDR_RSSI_OUT3			0x0050

#define ADDR_DEMOD4_INP_GAIN		0x00bc
#define ADDR_MOD4_OUTP_GAIN_TX0		0x00c0
#define ADDR_MOD4_OUTP_GAIN_TX1		0x00f4
#define ADDR_BYP4_GAIN_TX0		0x00c4
#define ADDR_BYP4_GAIN_TX1		0x00f8
#define ADDR_INC_IN4			0x00c8
#define ADDR_RSSI_OUT4			0x00cc

#define ADDR_DEMOD5_INP_GAIN		0x00d0
#define ADDR_MOD5_OUTP_GAIN_TX0		0x00d4
#define ADDR_MOD5_OUTP_GAIN_TX1		0x00fc
#define ADDR_BYP5_GAIN_TX0		0x00d8
#define ADDR_BYP5_GAIN_TX1		0x0100
#define ADDR_INC_IN5			0x00dc
#define ADDR_RSSI_OUT5			0x00e0

#define ADDR_DEMOD6_INP_GAIN		0x0054
#define ADDR_INC_IN6			0x0058
#define ADDR_RSSI_OUT6			0x005c

#define DFLT_IO_SEL0			0x00
#define DFLT_IO_SEL1			0x00
#define DFLT_DEMOD0_INP_GAIN		0x1FF
#define DFLT_MOD0_OUTP_GAIN_TX0		0x0 //0x1FF
#define DFLT_MOD0_OUTP_GAIN_TX1		0x0 //0x1FF
#define DFLT_BYP0_GAIN_TX0			0x0 //0x1FF
#define DFLT_BYP0_GAIN_TX1			0x0 //0x1FF
#define DFLT_INC_IN0			0xDA8AF9
#define DFLT_DEMOD1_INP_GAIN		0x1FF
#define DFLT_MOD1_OUTP_GAIN_TX0		0x0 //0x1FF
#define DFLT_MOD1_OUTP_GAIN_TX1		0x0 //0x1FF
#define DFLT_BYP1_GAIN_TX0			0x0 //0x1FF
#define DFLT_BYP1_GAIN_TX1			0x0 //0x1FF
#define DFLT_INC_IN1			0xDA8AF9
#define DFLT_DEMOD2_INP_GAIN		0x1FF
#define DFLT_MOD2_OUTP_GAIN_TX0		0x0 //0x1FF
#define DFLT_MOD2_OUTP_GAIN_TX1		0x0 //0x1FF
#define DFLT_BYP2_GAIN_TX0			0x0 //0x1FF
#define DFLT_BYP2_GAIN_TX1			0x0 //0x1FF
#define DFLT_INC_IN2			0xDA8AF9
#define DFLT_DEMOD3_INP_GAIN		0x1FF
#define DFLT_MOD3_OUTP_GAIN_TX0		0x0 //0x1FF
#define DFLT_MOD3_OUTP_GAIN_TX1		0x0 //0x1FF
#define DFLT_BYP3_GAIN_TX0			0x0 //0x1FF
#define DFLT_BYP3_GAIN_TX1			0x0 //0x1FF
#define DFLT_INC_IN3			0xDA8AF9

#define DFLT_DEMOD4_INP_GAIN		0x1FF
#define DFLT_MOD4_OUTP_GAIN_TX0		0x0 //0x1FF
#define DFLT_MOD4_OUTP_GAIN_TX1		0x0 //0x1FF
#define DFLT_BYP4_GAIN_TX0			0x0 //0x1FF
#define DFLT_BYP4_GAIN_TX1			0x0 //0x1FF
#define DFLT_INC_IN4			0xDA8AF9

#define DFLT_DEMOD5_INP_GAIN		0x1FF
#define DFLT_MOD5_OUTP_GAIN_TX0		0x0 //0x1FF
#define DFLT_MOD5_OUTP_GAIN_TX1		0x0 //0x1FF
#define DFLT_BYP5_GAIN_TX0			0x0 //0x1FF
#define DFLT_BYP5_GAIN_TX1			0x0 //0x1FF
#define DFLT_INC_IN5			0xDA8AF9

#define DFLT_DEMOD6_INP_GAIN		0x1FF
#define DFLT_INC_IN6			0xDA8AF9

#define MIN_MOD_OUTP_GAIN		0x0000
#define MAX_MOD_OUTP_GAIN		0xFFFF
#define MIN_MOD_INP_GAIN		0x0000
#define MAX_MOD_INP_GAIN		0xFFFF
#define MIN_IO_SEL			0x000
#define MAX_IO_SEL			0xFFFFFFFF
#define MIN_BYP_GAIN			0x0000
#define MAX_BYP_GAIN			0xFFFF

#define MAXVAL_U32			4294967295	// pow(2,32)-1
#define BASE2_POWER25		33554432	// pow(2,25)
#define BASE2_POWER24		16777216	// pow(2,24)

enum chan_num{
	CH_IO_SEL0,
	CH_IO_SEL1,
	CH_DEMOD0_INP_GAIN,
	CH_MOD0_OUTP_GAIN_TX0,
	CH_MOD0_OUTP_GAIN_TX1,
	CH_BYP0_GAIN_TX0,
	CH_BYP0_GAIN_TX1,
	CH_DDS0_FREQ,
	CH_RSSI_OUT0,
	CH_DEMOD1_INP_GAIN,
	CH_MOD1_OUTP_GAIN_TX0,
	CH_MOD1_OUTP_GAIN_TX1,
	CH_BYP1_GAIN_TX0,
	CH_BYP1_GAIN_TX1,
	CH_DDS1_FREQ,
	CH_RSSI_OUT1,
	CH_DEMOD2_INP_GAIN,
	CH_MOD2_OUTP_GAIN_TX0,
	CH_MOD2_OUTP_GAIN_TX1,
	CH_BYP2_GAIN_TX0,
	CH_BYP2_GAIN_TX1,
	CH_DDS2_FREQ,
	CH_RSSI_OUT2,
	CH_DEMOD3_INP_GAIN,
	CH_MOD3_OUTP_GAIN_TX0,
	CH_MOD3_OUTP_GAIN_TX1,
	CH_BYP3_GAIN_TX0,
	CH_BYP3_GAIN_TX1,
	CH_DDS3_FREQ,
	CH_RSSI_OUT3,
	CH_DEMOD4_INP_GAIN,
	CH_MOD4_OUTP_GAIN_TX0,
	CH_MOD4_OUTP_GAIN_TX1,
	CH_BYP4_GAIN_TX0,
	CH_BYP4_GAIN_TX1,
	CH_DDS4_FREQ,
	CH_RSSI_OUT4,
	CH_DEMOD5_INP_GAIN,
	CH_MOD5_OUTP_GAIN_TX0,
	CH_MOD5_OUTP_GAIN_TX1,
	CH_BYP5_GAIN_TX0,
	CH_BYP5_GAIN_TX1,
	CH_DDS5_FREQ,
	CH_RSSI_OUT5,
	CH_DEMOD6_INP_GAIN,
	CH_DDS6_FREQ,
	CH_RSSI_OUT6
};

struct ddc_duc_state {
	struct iio_info					iio_info;
	void __iomem					*regs;
//	struct device 					*dev;
//	struct clk 						*clk;
//	struct ddc_duc_platform_data	*pdata;
	struct mutex					lock;

	uint32_t						freq_adc;
	uint32_t						freq_dds;
	uint32_t						gain_tx;
	uint32_t						gain_rx;
};

static void ddc_duc_write(struct ddc_duc_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 ddc_duc_read(struct ddc_duc_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int ddc_duc_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
//	struct ddc_duc_state *st = iio_priv(indio_dev);
	int ret;
//	u64 temp64;
//	u32 freq;

	mutex_lock(&indio_dev->mlock);

	switch (mask) {
//	case IIO_CHAN_INFO_FREQUENCY:
//		/* Values in Hz */
//		if(val<0 || val>st->freq_adc)
//			return -EINVAL;
//
//		temp64 = (u64)val;
//		temp64 += st->freq_adc/4;
//		temp64 *= BASE2_POWER25;
//		temp64 = div_s64(temp64,st->freq_adc);
//		temp64 &= 0x0000000000FFFFFF;
////		div_s64_rem(temp64, BASE2_POWER24, (s32*)&temp64);
//		freq = (u32)temp64;
//
//		ddc_duc_write(st, DDS_FREQ, freq);
//		ret = 0;
//		break;
//	case IIO_CHAN_INFO_SCALE:
//		if(val>0xFFFF)
//			val = 0xFFFF;
//		if(chan->channel == CH_GAIN_TX)
//			ddc_duc_write(st, GAIN_TX, val);
//		if(chan->channel == CH_GAIN_RX)
//			ddc_duc_write(st, GAIN_RX, val);
//		ret = 0;
//		break;
//	case IIO_CHAN_INFO_RAW:
////		if(*chan == CH_FRAMELEN)
////			ddc_duc_write(st, CH_FRAMELEN, val);
////		if(*chan == CH_RSSI)
////			ddc_duc_write(st, CH_RSSI, val);
//		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int ddc_duc_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long mask)
{
//	struct ddc_duc_state *st = iio_priv(indio_dev);
	int ret;
//	u32 temp;
//	u64 temp64;

	mutex_lock(&indio_dev->mlock);

	switch (mask) {
//	case IIO_CHAN_INFO_FREQUENCY:
//		/* Values in Hz */
//		temp = ddc_duc_read(st, DDS_FREQ);
//
//		temp64 = (u64)temp;
//		temp64 *= st->freq_adc;
//		temp64 = div_u64(temp64,BASE2_POWER25);
//		temp64 -= st->freq_adc/4;
//		*val = (int)temp64;
//
//		ret = IIO_VAL_INT;
//		break;
//	case IIO_CHAN_INFO_SCALE:
//		if(chan->channel == CH_GAIN_TX)
//			*val = ddc_duc_read(st, GAIN_TX);
//		if(chan->channel == CH_GAIN_RX)
//			*val = ddc_duc_read(st, GAIN_RX);
//		ret = IIO_VAL_INT;
//		break;
//	case IIO_CHAN_INFO_RAW:
//		if(chan->channel == CH_FRAMELEN)
//			*val = ddc_duc_read(st, FRAMELEN_OUT);
//		if(chan->channel == CH_RSSI)
//			*val = ddc_duc_read(st, RSSI_OUT);
//		ret = IIO_VAL_INT;
//		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t ddc_duc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ddc_duc_state *st = iio_priv(indio_dev);
	long val;
	int ret;
	u64 temp64;
	u32 freq;

	/* convert to long
	 * auto-detect decimal,
	 * octal (beginning with 0) and
	 * hexadecimal (beginning with 0x)
	 */
	ret = kstrtol(buf, 0, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case CH_IO_SEL0:
		if(val<MIN_IO_SEL || val>MAX_IO_SEL){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_IO_SEL0, (u32)val);
		break;
	case CH_IO_SEL1:
		if(val<MIN_IO_SEL || val>MAX_IO_SEL){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_IO_SEL1, 0x1 | (u32)val);
		break;
	case CH_DEMOD0_INP_GAIN:
		if(val<MIN_MOD_INP_GAIN || val>MAX_MOD_INP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_DEMOD0_INP_GAIN, (u32)val);
		break;
	case CH_MOD0_OUTP_GAIN_TX0:
		if(val<MIN_MOD_OUTP_GAIN || val>MAX_MOD_OUTP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_MOD0_OUTP_GAIN_TX0, (u32)val);
		break;
	case CH_MOD0_OUTP_GAIN_TX1:
		if(val<MIN_MOD_OUTP_GAIN || val>MAX_MOD_OUTP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_MOD0_OUTP_GAIN_TX1, (u32)val);
		break;
	case CH_BYP0_GAIN_TX0:
		if(val<MIN_BYP_GAIN || val>MAX_BYP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_BYP0_GAIN_TX0, (u32)val);
		break;
	case CH_BYP0_GAIN_TX1:
		if(val<MIN_BYP_GAIN || val>MAX_BYP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_BYP0_GAIN_TX1, (u32)val);
		break;
	case CH_DDS0_FREQ:
		if(val<0 || val>st->freq_adc){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val;
		temp64 += st->freq_adc/4;
		temp64 *= BASE2_POWER25;
		temp64 = div_s64(temp64,st->freq_adc);
		temp64 &= 0x0000000000FFFFFF;
//		div_s64_rem(temp64, BASE2_POWER24, (s32*)&temp64);
		freq = (u32)temp64;
		ddc_duc_write(st, ADDR_INC_IN0, freq);
		break;
	case CH_DEMOD1_INP_GAIN:
		if(val<MIN_MOD_INP_GAIN || val>MAX_MOD_INP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_DEMOD1_INP_GAIN, (u32)val);
		break;
	case CH_MOD1_OUTP_GAIN_TX0:
		if(val<MIN_MOD_OUTP_GAIN || val>MAX_MOD_OUTP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_MOD1_OUTP_GAIN_TX0, (u32)val);
		break;
	case CH_MOD1_OUTP_GAIN_TX1:
		if(val<MIN_MOD_OUTP_GAIN || val>MAX_MOD_OUTP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_MOD1_OUTP_GAIN_TX1, (u32)val);
		break;
	case CH_BYP1_GAIN_TX0:
		if(val<MIN_BYP_GAIN || val>MAX_BYP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_BYP1_GAIN_TX0, (u32)val);
		break;
	case CH_BYP1_GAIN_TX1:
		if(val<MIN_BYP_GAIN || val>MAX_BYP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_BYP1_GAIN_TX1, (u32)val);
		break;
	case CH_DDS1_FREQ:
		if(val<0 || val>st->freq_adc){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val;
		temp64 += st->freq_adc/4;
		temp64 *= BASE2_POWER25;
		temp64 = div_s64(temp64,st->freq_adc);
		temp64 &= 0x0000000000FFFFFF;
//		div_s64_rem(temp64, BASE2_POWER24, (s32*)&temp64);
		freq = (u32)temp64;
		ddc_duc_write(st, ADDR_INC_IN1, freq);
		break;
	case CH_DEMOD2_INP_GAIN:
		if(val<MIN_MOD_INP_GAIN || val>MAX_MOD_INP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_DEMOD2_INP_GAIN, (u32)val);
		break;
	case CH_MOD2_OUTP_GAIN_TX0:
		if(val<MIN_MOD_OUTP_GAIN || val>MAX_MOD_OUTP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_MOD2_OUTP_GAIN_TX0, (u32)val);
		break;
	case CH_MOD2_OUTP_GAIN_TX1:
		if(val<MIN_MOD_OUTP_GAIN || val>MAX_MOD_OUTP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_MOD2_OUTP_GAIN_TX1, (u32)val);
		break;
	case CH_BYP2_GAIN_TX0:
		if(val<MIN_BYP_GAIN || val>MAX_BYP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_BYP2_GAIN_TX0, (u32)val);
		break;
	case CH_BYP2_GAIN_TX1:
		if(val<MIN_BYP_GAIN || val>MAX_BYP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_BYP2_GAIN_TX1, (u32)val);
		break;
	case CH_DDS2_FREQ:
		if(val<0 || val>st->freq_adc){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val;
		temp64 += st->freq_adc/4;
		temp64 *= BASE2_POWER25;
		temp64 = div_s64(temp64,st->freq_adc);
		temp64 &= 0x0000000000FFFFFF;
//		div_s64_rem(temp64, BASE2_POWER24, (s32*)&temp64);
		freq = (u32)temp64;
		ddc_duc_write(st, ADDR_INC_IN2, freq);
		break;
	case CH_DEMOD3_INP_GAIN:
		if(val<MIN_MOD_INP_GAIN || val>MAX_MOD_INP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_DEMOD3_INP_GAIN, (u32)val);
		break;
	case CH_MOD3_OUTP_GAIN_TX0:
		if(val<MIN_MOD_OUTP_GAIN || val>MAX_MOD_OUTP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_MOD3_OUTP_GAIN_TX0, (u32)val);
		break;
	case CH_MOD3_OUTP_GAIN_TX1:
		if(val<MIN_MOD_OUTP_GAIN || val>MAX_MOD_OUTP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_MOD3_OUTP_GAIN_TX1, (u32)val);
		break;
	case CH_BYP3_GAIN_TX0:
		if(val<MIN_BYP_GAIN || val>MAX_BYP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_BYP3_GAIN_TX0, (u32)val);
		break;
	case CH_BYP3_GAIN_TX1:
		if(val<MIN_BYP_GAIN || val>MAX_BYP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_BYP3_GAIN_TX1, (u32)val);
		break;
	case CH_DDS3_FREQ:
		if(val<0 || val>st->freq_adc){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val;
		temp64 += st->freq_adc/4;
		temp64 *= BASE2_POWER25;
		temp64 = div_s64(temp64,st->freq_adc);
		temp64 &= 0x0000000000FFFFFF;
//		div_s64_rem(temp64, BASE2_POWER24, (s32*)&temp64);
		freq = (u32)temp64;
		ddc_duc_write(st, ADDR_INC_IN3, freq);
		break;
	case CH_DEMOD4_INP_GAIN:
		if(val<MIN_MOD_INP_GAIN || val>MAX_MOD_INP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_DEMOD4_INP_GAIN, (u32)val);
		break;
	case CH_MOD4_OUTP_GAIN_TX0:
		if(val<MIN_MOD_OUTP_GAIN || val>MAX_MOD_OUTP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_MOD4_OUTP_GAIN_TX0, (u32)val);
		break;
	case CH_MOD4_OUTP_GAIN_TX1:
		if(val<MIN_MOD_OUTP_GAIN || val>MAX_MOD_OUTP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_MOD4_OUTP_GAIN_TX1, (u32)val);
		break;
	case CH_BYP4_GAIN_TX0:
		if(val<MIN_BYP_GAIN || val>MAX_BYP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_BYP4_GAIN_TX0, (u32)val);
		break;
	case CH_BYP4_GAIN_TX1:
		if(val<MIN_BYP_GAIN || val>MAX_BYP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_BYP4_GAIN_TX1, (u32)val);
		break;
	case CH_DDS4_FREQ:
		if(val<0 || val>st->freq_adc){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val;
		temp64 += st->freq_adc/4;
		temp64 *= BASE2_POWER25;
		temp64 = div_s64(temp64,st->freq_adc);
		temp64 &= 0x0000000000FFFFFF;
//		div_s64_rem(temp64, BASE2_POWER24, (s32*)&temp64);
		freq = (u32)temp64;
		ddc_duc_write(st, ADDR_INC_IN4, freq);
		break;
	case CH_DEMOD5_INP_GAIN:
		if(val<MIN_MOD_INP_GAIN || val>MAX_MOD_INP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_DEMOD5_INP_GAIN, (u32)val);
		break;
	case CH_MOD5_OUTP_GAIN_TX0:
		if(val<MIN_MOD_OUTP_GAIN || val>MAX_MOD_OUTP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_MOD5_OUTP_GAIN_TX0, (u32)val);
		break;
	case CH_MOD5_OUTP_GAIN_TX1:
		if(val<MIN_MOD_OUTP_GAIN || val>MAX_MOD_OUTP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_MOD5_OUTP_GAIN_TX1, (u32)val);
		break;
	case CH_BYP5_GAIN_TX0:
		if(val<MIN_BYP_GAIN || val>MAX_BYP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_BYP5_GAIN_TX0, (u32)val);
		break;
	case CH_BYP5_GAIN_TX1:
		if(val<MIN_BYP_GAIN || val>MAX_BYP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_BYP5_GAIN_TX1, (u32)val);
		break;
	case CH_DDS5_FREQ:
		if(val<0 || val>st->freq_adc){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val;
		temp64 += st->freq_adc/4;
		temp64 *= BASE2_POWER25;
		temp64 = div_s64(temp64,st->freq_adc);
		temp64 &= 0x0000000000FFFFFF;
//		div_s64_rem(temp64, BASE2_POWER24, (s32*)&temp64);
		freq = (u32)temp64;
		ddc_duc_write(st, ADDR_INC_IN5, freq);
		break;
	case CH_DEMOD6_INP_GAIN:
		if(val<MIN_MOD_INP_GAIN || val>MAX_MOD_INP_GAIN){
			ret = -EINVAL;
			break;
		}
		ddc_duc_write(st, ADDR_DEMOD6_INP_GAIN, (u32)val);
		break;
	case CH_DDS6_FREQ:
		if(val<0 || val>st->freq_adc){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val;
		temp64 += st->freq_adc/4;
		temp64 *= BASE2_POWER25;
		temp64 = div_s64(temp64,st->freq_adc);
		temp64 &= 0x0000000000FFFFFF;
//		div_s64_rem(temp64, BASE2_POWER24, (s32*)&temp64);
		freq = (u32)temp64;
		ddc_duc_write(st, ADDR_INC_IN6, freq);
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ddc_duc_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ddc_duc_state *st = iio_priv(indio_dev);
	u32 val;
	int ret;
	u64 temp64;
	u32 freq;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case CH_IO_SEL0:
		val = ddc_duc_read(st, ADDR_IO_SEL0);
		ret = 0;
		break;
	case CH_IO_SEL1:
		val = ddc_duc_read(st, ADDR_IO_SEL1);
		ret = 0;
		break;
	case CH_DEMOD0_INP_GAIN:
		val = ddc_duc_read(st, ADDR_DEMOD0_INP_GAIN);
		ret = 0;
		break;
	case CH_MOD0_OUTP_GAIN_TX0:
		val = ddc_duc_read(st, ADDR_MOD0_OUTP_GAIN_TX0);
		ret = 0;
		break;
	case CH_MOD0_OUTP_GAIN_TX1:
		val = ddc_duc_read(st, ADDR_MOD0_OUTP_GAIN_TX1);
		ret = 0;
		break;
	case CH_BYP0_GAIN_TX0:
		val = ddc_duc_read(st, ADDR_BYP0_GAIN_TX0);
		ret = 0;
		break;
	case CH_BYP0_GAIN_TX1:
		val = ddc_duc_read(st, ADDR_BYP0_GAIN_TX1);
		ret = 0;
		break;
	case CH_DDS0_FREQ:
		freq = ddc_duc_read(st, ADDR_INC_IN0);
		temp64 = (u64)freq;
		temp64 *= st->freq_adc;
		temp64 = div_u64(temp64,BASE2_POWER25);
		temp64 -= st->freq_adc/4;
		val = (int)temp64;
		ret = IIO_VAL_INT;
		ret = 0;
		break;
	case CH_RSSI_OUT0:
		val = ddc_duc_read(st, ADDR_RSSI_OUT0);
		ret = 0;
		break;
	case CH_DEMOD1_INP_GAIN:
		val = ddc_duc_read(st, ADDR_DEMOD1_INP_GAIN);
		ret = 0;
		break;
	case CH_MOD1_OUTP_GAIN_TX0:
		val = ddc_duc_read(st, ADDR_MOD1_OUTP_GAIN_TX0);
		ret = 0;
		break;
	case CH_MOD1_OUTP_GAIN_TX1:
		val = ddc_duc_read(st, ADDR_MOD1_OUTP_GAIN_TX1);
		ret = 0;
		break;
	case CH_BYP1_GAIN_TX0:
		val = ddc_duc_read(st, ADDR_BYP1_GAIN_TX0);
		ret = 0;
		break;
	case CH_BYP1_GAIN_TX1:
		val = ddc_duc_read(st, ADDR_BYP1_GAIN_TX1);
		ret = 0;
		break;
	case CH_DDS1_FREQ:
		freq = ddc_duc_read(st, ADDR_INC_IN1);
		temp64 = (u64)freq;
		temp64 *= st->freq_adc;
		temp64 = div_u64(temp64,BASE2_POWER25);
		temp64 -= st->freq_adc/4;
		val = (int)temp64;
		ret = IIO_VAL_INT;
		ret = 0;
		break;
	case CH_RSSI_OUT1:
		val = ddc_duc_read(st, ADDR_RSSI_OUT1);
		ret = 0;
		break;
	case CH_DEMOD2_INP_GAIN:
		val = ddc_duc_read(st, ADDR_DEMOD2_INP_GAIN);
		ret = 0;
		break;
	case CH_MOD2_OUTP_GAIN_TX0:
		val = ddc_duc_read(st, ADDR_MOD2_OUTP_GAIN_TX0);
		ret = 0;
		break;
	case CH_MOD2_OUTP_GAIN_TX1:
		val = ddc_duc_read(st, ADDR_MOD2_OUTP_GAIN_TX1);
		ret = 0;
		break;
	case CH_BYP2_GAIN_TX0:
		val = ddc_duc_read(st, ADDR_BYP2_GAIN_TX0);
		ret = 0;
		break;
	case CH_BYP2_GAIN_TX1:
		val = ddc_duc_read(st, ADDR_BYP2_GAIN_TX1);
		ret = 0;
		break;
	case CH_DDS2_FREQ:
		freq = ddc_duc_read(st, ADDR_INC_IN2);
		temp64 = (u64)freq;
		temp64 *= st->freq_adc;
		temp64 = div_u64(temp64,BASE2_POWER25);
		temp64 -= st->freq_adc/4;
		val = (int)temp64;
		ret = IIO_VAL_INT;
		ret = 0;
		break;
	case CH_RSSI_OUT2:
		val = ddc_duc_read(st, ADDR_RSSI_OUT2);
		ret = 0;
		break;
	case CH_DEMOD3_INP_GAIN:
		val = ddc_duc_read(st, ADDR_DEMOD3_INP_GAIN);
		ret = 0;
		break;
	case CH_MOD3_OUTP_GAIN_TX0:
		val = ddc_duc_read(st, ADDR_MOD3_OUTP_GAIN_TX0);
		ret = 0;
		break;
	case CH_MOD3_OUTP_GAIN_TX1:
		val = ddc_duc_read(st, ADDR_MOD3_OUTP_GAIN_TX1);
		ret = 0;
		break;
	case CH_BYP3_GAIN_TX0:
		val = ddc_duc_read(st, ADDR_BYP3_GAIN_TX0);
		ret = 0;
		break;
	case CH_BYP3_GAIN_TX1:
		val = ddc_duc_read(st, ADDR_BYP3_GAIN_TX1);
		ret = 0;
		break;
	case CH_DDS3_FREQ:
		freq = ddc_duc_read(st, ADDR_INC_IN3);
		temp64 = (u64)freq;
		temp64 *= st->freq_adc;
		temp64 = div_u64(temp64,BASE2_POWER25);
		temp64 -= st->freq_adc/4;
		val = (int)temp64;
		ret = IIO_VAL_INT;
		ret = 0;
		break;
	case CH_RSSI_OUT3:
		val = ddc_duc_read(st, ADDR_RSSI_OUT3);
		ret = 0;
		break;
	case CH_DEMOD4_INP_GAIN:
		val = ddc_duc_read(st, ADDR_DEMOD4_INP_GAIN);
		ret = 0;
		break;
	case CH_MOD4_OUTP_GAIN_TX0:
		val = ddc_duc_read(st, ADDR_MOD4_OUTP_GAIN_TX0);
		ret = 0;
		break;
	case CH_MOD4_OUTP_GAIN_TX1:
		val = ddc_duc_read(st, ADDR_MOD4_OUTP_GAIN_TX1);
		ret = 0;
		break;
	case CH_BYP4_GAIN_TX0:
		val = ddc_duc_read(st, ADDR_BYP4_GAIN_TX0);
		ret = 0;
		break;
	case CH_BYP4_GAIN_TX1:
		val = ddc_duc_read(st, ADDR_BYP4_GAIN_TX1);
		ret = 0;
		break;
	case CH_DDS4_FREQ:
		freq = ddc_duc_read(st, ADDR_INC_IN4);
		temp64 = (u64)freq;
		temp64 *= st->freq_adc;
		temp64 = div_u64(temp64,BASE2_POWER25);
		temp64 -= st->freq_adc/4;
		val = (int)temp64;
		ret = IIO_VAL_INT;
		ret = 0;
		break;
	case CH_RSSI_OUT4:
		val = ddc_duc_read(st, ADDR_RSSI_OUT4);
		ret = 0;
		break;
	case CH_DEMOD5_INP_GAIN:
		val = ddc_duc_read(st, ADDR_DEMOD5_INP_GAIN);
		ret = 0;
		break;
	case CH_MOD5_OUTP_GAIN_TX0:
		val = ddc_duc_read(st, ADDR_MOD5_OUTP_GAIN_TX0);
		ret = 0;
		break;
	case CH_MOD5_OUTP_GAIN_TX1:
		val = ddc_duc_read(st, ADDR_MOD5_OUTP_GAIN_TX1);
		ret = 0;
		break;
	case CH_BYP5_GAIN_TX0:
		val = ddc_duc_read(st, ADDR_BYP5_GAIN_TX0);
		ret = 0;
		break;
	case CH_BYP5_GAIN_TX1:
		val = ddc_duc_read(st, ADDR_BYP5_GAIN_TX1);
		ret = 0;
		break;
	case CH_DDS5_FREQ:
		freq = ddc_duc_read(st, ADDR_INC_IN5);
		temp64 = (u64)freq;
		temp64 *= st->freq_adc;
		temp64 = div_u64(temp64,BASE2_POWER25);
		temp64 -= st->freq_adc/4;
		val = (int)temp64;
		ret = IIO_VAL_INT;
		ret = 0;
		break;
	case CH_RSSI_OUT5:
		val = ddc_duc_read(st, ADDR_RSSI_OUT5);
		ret = 0;
		break;
	case CH_DEMOD6_INP_GAIN:
		val = ddc_duc_read(st, ADDR_DEMOD6_INP_GAIN);
		ret = 0;
		break;
	case CH_DDS6_FREQ:
		freq = ddc_duc_read(st, ADDR_INC_IN6);
		temp64 = (u64)freq;
		temp64 *= st->freq_adc;
		temp64 = div_u64(temp64,BASE2_POWER25);
		temp64 -= st->freq_adc/4;
		val = (int)temp64;
		ret = IIO_VAL_INT;
		ret = 0;
		break;
	case CH_RSSI_OUT6:
		val = ddc_duc_read(st, ADDR_RSSI_OUT6);
		ret = 0;
		break;

	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0)
		ret = sprintf(buf, "%d\n", val);

	return ret;
}

static IIO_DEVICE_ATTR(io_sel0, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_IO_SEL0);

static IIO_DEVICE_ATTR(io_sel1, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_IO_SEL1);

static IIO_DEVICE_ATTR(demod0_inp_gain, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DEMOD0_INP_GAIN);
static IIO_DEVICE_ATTR(mod0_outp_gain_tx0, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_MOD0_OUTP_GAIN_TX0);
static IIO_DEVICE_ATTR(mod0_outp_gain_tx1, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_MOD0_OUTP_GAIN_TX1);
static IIO_DEVICE_ATTR(byp0_gain_tx0, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_BYP0_GAIN_TX0);
static IIO_DEVICE_ATTR(byp0_gain_tx1, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_BYP0_GAIN_TX1);
static IIO_DEVICE_ATTR(dds0_freq, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DDS0_FREQ);
static IIO_DEVICE_ATTR(rssi_out0, S_IRUGO,
			ddc_duc_show,
			ddc_duc_store,
			CH_RSSI_OUT0);

static IIO_DEVICE_ATTR(demod1_inp_gain, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DEMOD1_INP_GAIN);
static IIO_DEVICE_ATTR(mod1_outp_gain_tx0, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_MOD1_OUTP_GAIN_TX0);
static IIO_DEVICE_ATTR(mod1_outp_gain_tx1, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_MOD1_OUTP_GAIN_TX1);
static IIO_DEVICE_ATTR(byp1_gain_tx0, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_BYP1_GAIN_TX0);
static IIO_DEVICE_ATTR(byp1_gain_tx1, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_BYP1_GAIN_TX1);
static IIO_DEVICE_ATTR(dds1_freq, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DDS1_FREQ);
static IIO_DEVICE_ATTR(rssi_out1, S_IRUGO,
			ddc_duc_show,
			ddc_duc_store,
			CH_RSSI_OUT1);

static IIO_DEVICE_ATTR(demod2_inp_gain, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DEMOD2_INP_GAIN);
static IIO_DEVICE_ATTR(mod2_outp_gain_tx0, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_MOD2_OUTP_GAIN_TX0);
static IIO_DEVICE_ATTR(mod2_outp_gain_tx1, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_MOD2_OUTP_GAIN_TX1);
static IIO_DEVICE_ATTR(byp2_gain_tx0, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_BYP2_GAIN_TX0);
static IIO_DEVICE_ATTR(byp2_gain_tx1, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_BYP2_GAIN_TX1);
static IIO_DEVICE_ATTR(dds2_freq, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DDS2_FREQ);
static IIO_DEVICE_ATTR(rssi_out2, S_IRUGO,
			ddc_duc_show,
			ddc_duc_store,
			CH_RSSI_OUT2);

static IIO_DEVICE_ATTR(demod3_inp_gain, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DEMOD3_INP_GAIN);
static IIO_DEVICE_ATTR(mod3_outp_gain_tx0, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_MOD3_OUTP_GAIN_TX0);
static IIO_DEVICE_ATTR(mod3_outp_gain_tx1, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_MOD3_OUTP_GAIN_TX1);
static IIO_DEVICE_ATTR(byp3_gain_tx0, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_BYP3_GAIN_TX0);
static IIO_DEVICE_ATTR(byp3_gain_tx1, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_BYP3_GAIN_TX1);
static IIO_DEVICE_ATTR(dds3_freq, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DDS3_FREQ);
static IIO_DEVICE_ATTR(rssi_out3, S_IRUGO,
			ddc_duc_show,
			ddc_duc_store,
			CH_RSSI_OUT3);

static IIO_DEVICE_ATTR(demod4_inp_gain, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DEMOD4_INP_GAIN);
static IIO_DEVICE_ATTR(mod4_outp_gain_tx0, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_MOD4_OUTP_GAIN_TX0);
static IIO_DEVICE_ATTR(mod4_outp_gain_tx1, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_MOD4_OUTP_GAIN_TX1);
static IIO_DEVICE_ATTR(byp4_gain_tx0, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_BYP4_GAIN_TX0);
static IIO_DEVICE_ATTR(byp4_gain_tx1, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_BYP4_GAIN_TX1);
static IIO_DEVICE_ATTR(dds4_freq, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DDS4_FREQ);
static IIO_DEVICE_ATTR(rssi_out4, S_IRUGO,
			ddc_duc_show,
			ddc_duc_store,
			CH_RSSI_OUT4);

static IIO_DEVICE_ATTR(demod5_inp_gain, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DEMOD5_INP_GAIN);
static IIO_DEVICE_ATTR(mod5_outp_gain_tx0, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_MOD5_OUTP_GAIN_TX0);
static IIO_DEVICE_ATTR(mod5_outp_gain_tx1, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_MOD5_OUTP_GAIN_TX1);
static IIO_DEVICE_ATTR(byp5_gain_tx0, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_BYP5_GAIN_TX0);
static IIO_DEVICE_ATTR(byp5_gain_tx1, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_BYP5_GAIN_TX1);
static IIO_DEVICE_ATTR(dds5_freq, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DDS5_FREQ);
static IIO_DEVICE_ATTR(rssi_out5, S_IRUGO,
			ddc_duc_show,
			ddc_duc_store,
			CH_RSSI_OUT5);

static IIO_DEVICE_ATTR(demod6_inp_gain, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DEMOD6_INP_GAIN);
static IIO_DEVICE_ATTR(dds6_freq, S_IRUGO | S_IWUSR,
			ddc_duc_show,
			ddc_duc_store,
			CH_DDS6_FREQ);
static IIO_DEVICE_ATTR(rssi_out6, S_IRUGO,
			ddc_duc_show,
			ddc_duc_store,
			CH_RSSI_OUT6);

static struct attribute *ddc_duc_attributes[] = {
	&iio_dev_attr_io_sel0.dev_attr.attr,
	&iio_dev_attr_io_sel1.dev_attr.attr,
	&iio_dev_attr_demod0_inp_gain.dev_attr.attr,
	&iio_dev_attr_mod0_outp_gain_tx0.dev_attr.attr,
	&iio_dev_attr_mod0_outp_gain_tx1.dev_attr.attr,
	&iio_dev_attr_byp0_gain_tx0.dev_attr.attr,
	&iio_dev_attr_byp0_gain_tx1.dev_attr.attr,
	&iio_dev_attr_dds0_freq.dev_attr.attr,
	&iio_dev_attr_rssi_out0.dev_attr.attr,
	&iio_dev_attr_demod1_inp_gain.dev_attr.attr,
	&iio_dev_attr_mod1_outp_gain_tx0.dev_attr.attr,
	&iio_dev_attr_mod1_outp_gain_tx1.dev_attr.attr,
	&iio_dev_attr_byp1_gain_tx0.dev_attr.attr,
	&iio_dev_attr_byp1_gain_tx1.dev_attr.attr,
	&iio_dev_attr_dds1_freq.dev_attr.attr,
	&iio_dev_attr_rssi_out1.dev_attr.attr,
	&iio_dev_attr_demod2_inp_gain.dev_attr.attr,
	&iio_dev_attr_mod2_outp_gain_tx0.dev_attr.attr,
	&iio_dev_attr_mod2_outp_gain_tx1.dev_attr.attr,
	&iio_dev_attr_byp2_gain_tx0.dev_attr.attr,
	&iio_dev_attr_byp2_gain_tx1.dev_attr.attr,
	&iio_dev_attr_dds2_freq.dev_attr.attr,
	&iio_dev_attr_rssi_out2.dev_attr.attr,
	&iio_dev_attr_demod3_inp_gain.dev_attr.attr,
	&iio_dev_attr_mod3_outp_gain_tx0.dev_attr.attr,
	&iio_dev_attr_mod3_outp_gain_tx1.dev_attr.attr,
	&iio_dev_attr_byp3_gain_tx0.dev_attr.attr,
	&iio_dev_attr_byp3_gain_tx1.dev_attr.attr,
	&iio_dev_attr_dds3_freq.dev_attr.attr,
	&iio_dev_attr_rssi_out3.dev_attr.attr,
	&iio_dev_attr_demod4_inp_gain.dev_attr.attr,
	&iio_dev_attr_mod4_outp_gain_tx0.dev_attr.attr,
	&iio_dev_attr_mod4_outp_gain_tx1.dev_attr.attr,
	&iio_dev_attr_byp4_gain_tx0.dev_attr.attr,
	&iio_dev_attr_byp4_gain_tx1.dev_attr.attr,
	&iio_dev_attr_dds4_freq.dev_attr.attr,
	&iio_dev_attr_rssi_out4.dev_attr.attr,
	&iio_dev_attr_demod5_inp_gain.dev_attr.attr,
	&iio_dev_attr_mod5_outp_gain_tx0.dev_attr.attr,
	&iio_dev_attr_mod5_outp_gain_tx1.dev_attr.attr,
	&iio_dev_attr_byp5_gain_tx0.dev_attr.attr,
	&iio_dev_attr_byp5_gain_tx1.dev_attr.attr,
	&iio_dev_attr_dds5_freq.dev_attr.attr,
	&iio_dev_attr_rssi_out5.dev_attr.attr,
	&iio_dev_attr_demod6_inp_gain.dev_attr.attr,
	&iio_dev_attr_dds6_freq.dev_attr.attr,
	&iio_dev_attr_rssi_out6.dev_attr.attr,
	NULL,
};

static const struct attribute_group ddc_duc_attribute_group = {
	.attrs = ddc_duc_attributes,
};

static const struct iio_info ddc_duc_info = {
	.read_raw = &ddc_duc_read_raw,
	.write_raw = &ddc_duc_write_raw,
	.attrs = &ddc_duc_attribute_group,
};

static const struct iio_chan_spec ddc_duc_channels[] = {				// add more channels here if desired
//	{
//		.type = IIO_VOLTAGE,
//		.output = 1,
//		.indexed = 1,
//		.channel = CH_FREQ_DDS,
//		.info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY),
//	},
//	{
//		.type = IIO_VOLTAGE,
//		.output = 1,
//		.indexed = 1,
//		.channel = CH_GAIN_TX,
//		.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),
//	},
//	{
//		.type = IIO_VOLTAGE,
//		.output = 1,
//		.indexed = 1,
//		.channel = CH_GAIN_RX,
//		.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),
//	},
//	{
//		.type = IIO_VOLTAGE,
//		.output = 0,
//		.indexed = 1,
//		.channel = CH_FRAMELEN,
//		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
//	},
//	{
//		.type = IIO_VOLTAGE,
//		.output = 0,
//		.indexed = 1,
//		.channel = CH_RSSI,
//		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
//	},
};

/* Match table for of_platform binding */
static const struct of_device_id ddc_duc_of_match[] = {
	{ .compatible = "fpga,ddc-duc", },
	{ },
};

MODULE_DEVICE_TABLE(of, ddc_duc_of_match);

static int ddc_duc_probe(struct platform_device *pdev)
{
//	struct ddc_duc_platform_data *pdata = dev_get_platdata(&pdev->dev);
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
//	struct ddc_duc *ddc_duc;
	struct resource *res;
	struct ddc_duc_state *st;
	struct iio_dev *indio_dev;
	int ret; //, i, n;
//	const char *name;
//	char tmp[20];

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(ddc_duc_of_match, &pdev->dev);
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


	if(of_property_read_u32(np, "required,freq-adc", &st->freq_adc)){
		printk("DDC-DUC: ***ERROR! \"required,freq-adc\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->freq_adc == 0){
		printk("DDC-DUC: ***ERROR! \"required,freq-adc\" equal to 0 Hz\n");
		goto err_iio_device_free;
	}

	if(of_property_read_u32(np, "required,freq-dds", &st->freq_dds)){
		printk("DDC-DUC: ***ERROR! \"required,freq-dds\" missing in devicetree?");
		ret = -ERANGE;
	}

	if(of_property_read_u32(np, "optional,gain-tx", &st->gain_tx))
		st->gain_tx = 0xFF;	// default = 0dB

	if(of_property_read_u32(np, "optional,gain-rx", &st->gain_rx))
		st->gain_rx = 0xFF;	// default = 0dB

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->channels = ddc_duc_channels;
	indio_dev->num_channels = ARRAY_SIZE(ddc_duc_channels);
	indio_dev->info = &ddc_duc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

#if 0
	// init all attributes
	ddc_duc_write(st, ADDR_IO_SEL0, 		(u32)DFLT_IO_SEL0);
	ddc_duc_write(st, ADDR_IO_SEL1, 		0x1 | (u32)DFLT_IO_SEL1);
	ddc_duc_write(st, ADDR_DEMOD0_INP_GAIN, 	(u32)DFLT_DEMOD0_INP_GAIN);
	ddc_duc_write(st, ADDR_MOD0_OUTP_GAIN_TX0, 	(u32)DFLT_MOD0_OUTP_GAIN_TX0);
	ddc_duc_write(st, ADDR_MOD0_OUTP_GAIN_TX1, 	(u32)DFLT_MOD0_OUTP_GAIN_TX1);
	ddc_duc_write(st, ADDR_BYP0_GAIN_TX0,		(u32)DFLT_BYP0_GAIN_TX0);
	ddc_duc_write(st, ADDR_BYP0_GAIN_TX1,		(u32)DFLT_BYP0_GAIN_TX1);
	ddc_duc_write(st, ADDR_INC_IN0, 		(u32)DFLT_INC_IN0);
	ddc_duc_write(st, ADDR_DEMOD1_INP_GAIN, 	(u32)DFLT_DEMOD1_INP_GAIN);
	ddc_duc_write(st, ADDR_MOD1_OUTP_GAIN_TX0, 	(u32)DFLT_MOD1_OUTP_GAIN_TX0);
	ddc_duc_write(st, ADDR_MOD1_OUTP_GAIN_TX1, 	(u32)DFLT_MOD1_OUTP_GAIN_TX1);
	ddc_duc_write(st, ADDR_BYP1_GAIN_TX0,		(u32)DFLT_BYP1_GAIN_TX0);
	ddc_duc_write(st, ADDR_BYP1_GAIN_TX1,		(u32)DFLT_BYP1_GAIN_TX1);
	ddc_duc_write(st, ADDR_INC_IN2, 		(u32)DFLT_INC_IN2);
	ddc_duc_write(st, ADDR_DEMOD2_INP_GAIN, 	(u32)DFLT_DEMOD2_INP_GAIN);
	ddc_duc_write(st, ADDR_MOD2_OUTP_GAIN_TX0, 	(u32)DFLT_MOD2_OUTP_GAIN_TX0);
	ddc_duc_write(st, ADDR_MOD2_OUTP_GAIN_TX1, 	(u32)DFLT_MOD2_OUTP_GAIN_TX1);
	ddc_duc_write(st, ADDR_BYP2_GAIN_TX0,		(u32)DFLT_BYP2_GAIN_TX0);
	ddc_duc_write(st, ADDR_BYP2_GAIN_TX1,		(u32)DFLT_BYP2_GAIN_TX1);
	ddc_duc_write(st, ADDR_INC_IN2, 		(u32)DFLT_INC_IN2);
	ddc_duc_write(st, ADDR_DEMOD3_INP_GAIN, 	(u32)DFLT_DEMOD3_INP_GAIN);
	ddc_duc_write(st, ADDR_MOD3_OUTP_GAIN_TX0, 	(u32)DFLT_MOD3_OUTP_GAIN_TX0);
	ddc_duc_write(st, ADDR_MOD3_OUTP_GAIN_TX1, 	(u32)DFLT_MOD3_OUTP_GAIN_TX1);
	ddc_duc_write(st, ADDR_BYP3_GAIN_TX0,		(u32)DFLT_BYP3_GAIN_TX0);
	ddc_duc_write(st, ADDR_BYP3_GAIN_TX1,		(u32)DFLT_BYP3_GAIN_TX1);
	ddc_duc_write(st, ADDR_INC_IN3, 		(u32)DFLT_INC_IN3);
	ddc_duc_write(st, ADDR_DEMOD4_INP_GAIN, 	(u32)DFLT_DEMOD4_INP_GAIN);
	ddc_duc_write(st, ADDR_MOD4_OUTP_GAIN_TX0, 	(u32)DFLT_MOD4_OUTP_GAIN_TX0);
	ddc_duc_write(st, ADDR_MOD4_OUTP_GAIN_TX1, 	(u32)DFLT_MOD4_OUTP_GAIN_TX1);
	ddc_duc_write(st, ADDR_BYP4_GAIN_TX0,		(u32)DFLT_BYP4_GAIN_TX0);
	ddc_duc_write(st, ADDR_BYP4_GAIN_TX1,		(u32)DFLT_BYP4_GAIN_TX1);
	ddc_duc_write(st, ADDR_INC_IN4, 		(u32)DFLT_INC_IN4);
	ddc_duc_write(st, ADDR_DEMOD5_INP_GAIN, 	(u32)DFLT_DEMOD5_INP_GAIN);
	ddc_duc_write(st, ADDR_MOD5_OUTP_GAIN_TX0, 	(u32)DFLT_MOD5_OUTP_GAIN_TX0);
	ddc_duc_write(st, ADDR_MOD5_OUTP_GAIN_TX1, 	(u32)DFLT_MOD5_OUTP_GAIN_TX1);
	ddc_duc_write(st, ADDR_BYP5_GAIN_TX0,		(u32)DFLT_BYP5_GAIN_TX0);
	ddc_duc_write(st, ADDR_BYP5_GAIN_TX1,		(u32)DFLT_BYP5_GAIN_TX1);
	ddc_duc_write(st, ADDR_INC_IN5, 		(u32)DFLT_INC_IN5);
	ddc_duc_write(st, ADDR_DEMOD6_INP_GAIN, 	(u32)DFLT_DEMOD6_INP_GAIN);
	ddc_duc_write(st, ADDR_INC_IN6, 		(u32)DFLT_INC_IN6);
#endif // 0

//	for(i=0;i<ARRAY_SIZE(ddc_duc_attributes);i++){
//		if(!ddc_duc_attribute_group.attrs[i])
//			continue;
//		name = ddc_duc_attribute_group.attrs[i]->name;
//		if(!name){
//			printk(">>> NO NAME\n");
//			continue;
//		}
//		printk("> NAME: %s\n",name);
//		if(strcmp(name, "io_sel") == 0){
//			n = sprintf(tmp, "%d\n", DFLT_IO_SEL);
//			ddc_duc_store(indio_dev->dev.parent, &dev_attr_io_sel, tmp, n);
//		}
//	}



	//		if(strcmp(name, "mod_outp_gain") == 0)
	//			ddc_duc_write(st, ADDR_MOD_OUTP_GAIN, (u32)DFLT_MOD_OUTP_GAIN);
	//		if(strcmp(name, "mod_inp_gain") == 0)
	//			ddc_duc_write(st, ADDR_MOD_INP_GAIN, (u32)DFLT_MOD_INP_GAIN);
	//		if(strcmp(name, "dds_freq") == 0)
	//			ddc_duc_write(st, ADDR_INC_IN0, (u32)DFLT_INC_IN0);
	//		if(strcmp(name, "byp_gain") == 0)
	//			ddc_duc_write(st, ADDR_BYP_GAIN, (u32)DFLT_BYP_GAIN);


	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, indio_dev);
	return 0;

err_iio_device_free:
	iio_device_free(indio_dev);
	return ret;
}

static int ddc_duc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver ddc_duc_driver = {
	.probe		= ddc_duc_probe,
	.remove		= ddc_duc_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ddc_duc_of_match,
	},
};

module_platform_driver(ddc_duc_driver);

MODULE_AUTHOR("Cyril Zwahlen <zlc3@bfh.ch>");
MODULE_DESCRIPTION("Digital Down Converter (DDC) and Digital Up Converter (DUC) FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);

