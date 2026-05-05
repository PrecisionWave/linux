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


#define DRIVER_NAME			"dras-radio-repeater"
#define NB_OF_TETRA_CHANNELS		16
#define NB_OF_TETRA_PORTS		16

// global attributes
#define ADDR_DSP_VERSION		(0*4)
#define ADDR_RANDOMNUMBER		(1*4)
#define ADDR_UL_SFP_SYNC		(2*4)
#define ADDR_GAIN_LIMIT			(160*4)
#define ADDR_TARGET_PWR			(161*4)
#define ADDR_SQUELCH			(162*4)
#define ADDR_CHANNEL_EN			(163*4)
#define ADDR_MUTE_LEN			(164*4)
#define ADDR_GAIN_LIMIT_DL		(172*4)
#define ADDR_TARGET_PWR_DL		(173*4)
#define ADDR_SQUELCH_DL			(174*4)
#define ADDR_OFFSET_TLAST0		(180*4)		// 4bits per port, port0..7
#define ADDR_OFFSET_TLAST1		(181*4)		// 4bits per port, port8..15
#define ADDR_HASH			(188*4)

// channel attributes
#define ADDR_RSSI_UL(x)			((8+(x))*4)	// 16bit LSB first rssi, second 16bit second rssi
#define ADDR_RSSI_PEAK_UL(x)		((24+(x))*4)
#define ADDR_UL_ORDER(x)		((40+(x))*4)	// 8 channels per port, each with 4bits, 16 ports, port x from 0..15
#define ADDR_PORT_ID(x)			((56+(x))*4)	// 12bit port id, port x from 0..15
#define ADDR_RSSI_DL(x)			((72+(x))*4)	// 16bit LSB first rssi, second 16bit second rssi
#define ADDR_RSSI_PEAK_DL(x)		((88+(x))*4)
#define ADDR_GAIN_UL(x)			((104+(x))*4)
#define ADDR_GAIN_DL(x)			((120+(x))*4)


// expands to:
//   CH0_<REG> + CHANNEL
// example:
//   REG_CH(0, REG_GAIN_TX1)
//     expansion:
//     CH0_REG_GAIN_TX1 + 0
#define REG_CH(CHANNEL, REG) \
	(CH0_##REG + CHANNEL)

#define REG_PORT(PORT, REG) \
	(PORT0_##REG + PORT)

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

#define REG_ALL_PORT(PORT) \
	PORT0_##PORT, \
	PORT1_##PORT, \
	PORT2_##PORT, \
	PORT3_##PORT, \
	PORT4_##PORT, \
	PORT5_##PORT, \
	PORT6_##PORT, \
	PORT7_##PORT, \
	PORT8_##PORT, \
	PORT9_##PORT, \
	PORT10_##PORT, \
	PORT11_##PORT, \
	PORT12_##PORT, \
	PORT13_##PORT, \
	PORT14_##PORT, \
	PORT15_##PORT

// expands to:
//   static IIO_DEVICE_ATTR(ch0_<ATTR>, <RW>, <SHOW>, <STORE>, CH0_<REG>);
//   static IIO_DEVICE_ATTR(ch1_<ATTR>, <RW>, <SHOW>, <STORE>, CH1_<REG>);
//    ::   ::
//   static IIO_DEVICE_ATTR(ch31_<ATTR>, <RW>, <SHOW>, <STORE>, CH31_<REG>);
// example:
//   IIO_DEVICE_ATTR_ALL_CH(gain_tx1, S_IRUGO | S_IWUSR, dras_radio_repeater_show, dras_radio_repeater_store, REG_GAIN_TX1)
//     expansion:
//     static IIO_DEVICE_ATTR(ch0_gain_tx1, S_IRUGO | S_IWUSR, dras_radio_repeater_show, dras_radio_repeater_store, CH0_REG_GAIN_TX1);
//     static IIO_DEVICE_ATTR(ch1_gain_tx1, S_IRUGO | S_IWUSR, dras_radio_repeater_show, dras_radio_repeater_store, CH1_REG_GAIN_TX1);
//      ::   ::
//     static IIO_DEVICE_ATTR(ch31_gain_tx1, S_IRUGO | S_IWUSR, dras_radio_repeater_show, dras_radio_repeater_store, CH31_REG_GAIN_TX1);
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

#define IIO_DEVICE_ATTR_ALL_PORT(ATTR, RW, SHOW, STORE, PORT) \
	static IIO_DEVICE_ATTR(port0_##ATTR, RW, SHOW, STORE, PORT0_##PORT); \
	static IIO_DEVICE_ATTR(port1_##ATTR, RW, SHOW, STORE, PORT1_##PORT); \
	static IIO_DEVICE_ATTR(port2_##ATTR, RW, SHOW, STORE, PORT2_##PORT); \
	static IIO_DEVICE_ATTR(port3_##ATTR, RW, SHOW, STORE, PORT3_##PORT); \
	static IIO_DEVICE_ATTR(port4_##ATTR, RW, SHOW, STORE, PORT4_##PORT); \
	static IIO_DEVICE_ATTR(port5_##ATTR, RW, SHOW, STORE, PORT5_##PORT); \
	static IIO_DEVICE_ATTR(port6_##ATTR, RW, SHOW, STORE, PORT6_##PORT); \
	static IIO_DEVICE_ATTR(port7_##ATTR, RW, SHOW, STORE, PORT7_##PORT); \
	static IIO_DEVICE_ATTR(port8_##ATTR, RW, SHOW, STORE, PORT8_##PORT); \
	static IIO_DEVICE_ATTR(port9_##ATTR, RW, SHOW, STORE, PORT9_##PORT); \
	static IIO_DEVICE_ATTR(port10_##ATTR, RW, SHOW, STORE, PORT10_##PORT); \
	static IIO_DEVICE_ATTR(port11_##ATTR, RW, SHOW, STORE, PORT11_##PORT); \
	static IIO_DEVICE_ATTR(port12_##ATTR, RW, SHOW, STORE, PORT12_##PORT); \
	static IIO_DEVICE_ATTR(port13_##ATTR, RW, SHOW, STORE, PORT13_##PORT); \
	static IIO_DEVICE_ATTR(port14_##ATTR, RW, SHOW, STORE, PORT14_##PORT); \
	static IIO_DEVICE_ATTR(port15_##ATTR, RW, SHOW, STORE, PORT15_##PORT);

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

#define IIO_ATTR_ALL_PORT(ATTR) \
	&iio_dev_attr_port0_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port1_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port2_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port3_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port4_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port5_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port6_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port7_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port8_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port9_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port10_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port11_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port12_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port13_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port14_##ATTR.dev_attr.attr, \
	&iio_dev_attr_port15_##ATTR.dev_attr.attr

enum chan_num{
	REG_ALL_CH(REG_CHANNEL_ENABLE),	// being expanded for all channels
	REG_ALL_CH(REG_EN_FREQ_TRANSLATION),	// being expanded for all channels
	REG_ALL_CH(REG_BEST_SOURCE_MAX),	// being expanded for all channels
	REG_ALL_CH(REG_BEST_SOURCE_MIN),	// being expanded for all channels
	REG_ALL_CH(REG_UL_RSSI),	// being expanded for all channels
	REG_ALL_CH(REG_UL_RSSI_MAX),	// being expanded for all channels
	REG_ALL_CH(REG_UL_RSSI_MIN),	// being expanded for all channels
	REG_ALL_CH(REG_UL_GAIN_MAX),	// being expanded for all channels
	REG_ALL_CH(REG_UL_GAIN_MIN),	// being expanded for all channels
	REG_ALL_CH(REG_UL_TARGET_POWER),	// being expanded for all channels
	REG_ALL_CH(REG_UL_SQUELCH),	// being expanded for all channels
	REG_ALL_CH(REG_UL_GAIN_LIMIT),	// being expanded for all channels
	REG_ALL_CH(REG_UL_MUTE),	// being expanded for all channels
	REG_ALL_CH(REG_DL_RSSI),	// being expanded for all channels
	REG_ALL_CH(REG_DL_RSSI_MAX),	// being expanded for all channels
	REG_ALL_CH(REG_DL_RSSI_MIN),	// being expanded for all channels
	REG_ALL_CH(REG_DL_GAIN_MAX),	// being expanded for all channels
	REG_ALL_CH(REG_DL_GAIN_MIN),	// being expanded for all channels
	REG_ALL_CH(REG_DL_TARGET_POWER),	// being expanded for all channels
	REG_ALL_CH(REG_DL_SQUELCH),	// being expanded for all channels
	REG_ALL_CH(REG_DL_GAIN_LIMIT),	// being expanded for all channels
	REG_ALL_CH(REG_DL_MUTE),	// being expanded for all channels
	//REG_ALL_PORT(REG_OFFSET_TLAST),	// being expanded for all channels
	//REG_ALL_PORT(REG_ENABLE_DL_TEST),	// being expanded for all channels
	//REG_ALL_PORT(REG_UL_ORDER),	// being expanded for all channels
	//REG_ALL_PORT(REG_PORT_ID),	// being expanded for all channels
	REG_ALL_PORT(REG_UL_SYNC),	// being expanded for all channels
	REG_DSP_VERSION,
	REG_RANDOMNUMBER,
	REG_HASH,
	REG_WIDEBAND_MU_RXTX4_FOR_COVERAGE
};

struct dras_radio_repeater_state {
	struct iio_info		iio_info;
	void __iomem		*regs;
	struct mutex		lock;

	struct device		*dev;
	struct clk		*adrv_clk;
	struct notifier_block	adrv_clk_rate_change_nb;
	uint32_t		adrv_clk_rate;
	u32			ul_target[NB_OF_TETRA_CHANNELS];
	u32			ul_squelch[NB_OF_TETRA_CHANNELS];
	u32			ul_gain_limit[NB_OF_TETRA_CHANNELS];
	u32			dl_target[NB_OF_TETRA_CHANNELS];
	u32			dl_squelch[NB_OF_TETRA_CHANNELS];
	u32			dl_gain_limit[NB_OF_TETRA_CHANNELS];
};

static void dras_radio_repeater_write(struct dras_radio_repeater_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 dras_radio_repeater_read(struct dras_radio_repeater_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int dras_radio_repeater_write_raw(struct iio_dev *indio_dev,
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

static int dras_radio_repeater_read_raw(struct iio_dev *indio_dev,
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

static ssize_t dras_radio_repeater_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dras_radio_repeater_state *st = iio_priv(indio_dev);
	long val;
	int ret;
	u32 temp32;
	u32 ch;
	int shift;
	u32 port;
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
		if((u32)this_attr->address == REG_CH(ch, REG_CHANNEL_ENABLE)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = dras_radio_repeater_read(st, ADDR_CHANNEL_EN) & ~(1<<ch);
			temp32 += ((uint32_t)val)<<ch;
			dras_radio_repeater_write(st, ADDR_CHANNEL_EN, temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_EN_FREQ_TRANSLATION)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = dras_radio_repeater_read(st, ADDR_MUTE_LEN) & ~(1<<(ch+16));
			temp32 += ((uint32_t)val)<<(ch+16);
			dras_radio_repeater_write(st, ADDR_MUTE_LEN, temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_UL_TARGET_POWER)){
			match = 1;
			if(val<0 || val>0x1FF){
				ret = -EINVAL;
				break;
			}
			st->ul_target[ch] = (u32)val;
			temp32 = (u32)val | (ch<<9);
			dras_radio_repeater_write(st, ADDR_TARGET_PWR, temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_UL_SQUELCH)){
			match = 1;
			if(val<0 || val>0x7FFF){
				ret = -EINVAL;
				break;
			}
			st->ul_squelch[ch] = (u32)val;
			temp32 = (u32)val | (ch<<15);
			dras_radio_repeater_write(st, ADDR_SQUELCH, temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_UL_GAIN_LIMIT)){
			match = 1;
			if(val<0 || val>0x7FFFFF){
				ret = -EINVAL;
				break;
			}
			st->ul_gain_limit[ch] = (u32)val;
			temp32 = (u32)val | (ch<<23);
			dras_radio_repeater_write(st, ADDR_GAIN_LIMIT, temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DL_TARGET_POWER)){
			match = 1;
			if(val<0 || val>0x1FF){
				ret = -EINVAL;
				break;
			}
			st->dl_target[ch] = (u32)val;
			temp32 = (u32)val | (ch<<9);
			dras_radio_repeater_write(st, ADDR_TARGET_PWR_DL, temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DL_SQUELCH)){
			match = 1;
			if(val<0 || val>0x7FFF){
				ret = -EINVAL;
				break;
			}
			st->dl_squelch[ch] = (u32)val;
			temp32 = (u32)val | (ch<<15);
			dras_radio_repeater_write(st, ADDR_SQUELCH_DL, temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DL_GAIN_LIMIT)){
			match = 1;
			if(val<0 || val>0x7FFFFF){
				ret = -EINVAL;
				break;
			}
			st->dl_gain_limit[ch] = (u32)val;
			temp32 = (u32)val | (ch<<23);
			dras_radio_repeater_write(st, ADDR_GAIN_LIMIT_DL, temp32);
			break;
		}
	}
/*
	for(port=0; port<NB_OF_TETRA_PORTS; port++){
		if((u32)this_attr->address == REG_PORT(port, REG_OFFSET_TLAST)){
			match = 1;
			//if(val<0 || val>7){
			if(val<0 || val>0xF){
				ret = -EINVAL;
				break;
			}
			//val = val*2; // only odd channels used
			if(port<8){
				shift = port*4;
				temp32 = dras_radio_repeater_read(st, ADDR_OFFSET_TLAST0) & ~(0xF<<shift);
				temp32 += ((uint32_t)val)<<shift;
				dras_radio_repeater_write(st, ADDR_OFFSET_TLAST0, temp32);
			}else{
				shift = (port-8)*4;
				temp32 = dras_radio_repeater_read(st, ADDR_OFFSET_TLAST1) & ~(0xF<<shift);
				temp32 += ((uint32_t)val)<<shift;
				dras_radio_repeater_write(st, ADDR_OFFSET_TLAST1, temp32);
			}
			break;
		}
		else if((u32)this_attr->address == REG_PORT(port, REG_ENABLE_DL_TEST)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = dras_radio_repeater_read(st, ADDR_CHANNEL_EN) & ~(1<<(port+16));
			temp32 += ((uint32_t)val)<<(port+16);
			dras_radio_repeater_write(st, ADDR_CHANNEL_EN, temp32);
			break;
		}
	}
*/
	if(match){
		mutex_unlock(&indio_dev->mlock);
		return ret ? ret : len;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_WIDEBAND_MU_RXTX4_FOR_COVERAGE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_radio_repeater_read(st, ADDR_MUTE_LEN) & ~(1<<15);
		temp32 += ((uint32_t)val)<<15;
		dras_radio_repeater_write(st, ADDR_MUTE_LEN, temp32);
		break;
	case REG_HASH:
		dras_radio_repeater_write(st, ADDR_HASH, (u32)val);
		break;
	default:
		ret = -ENODEV;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t dras_radio_repeater_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dras_radio_repeater_state *st = iio_priv(indio_dev);
	int val = 0;
	int shift;
	int ret = 0;
	u32 ch;
	u32 port;
	u32 temp32;
	//int power10 = 1;
	int match;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<NB_OF_TETRA_CHANNELS; ch++){
		if((u32)this_attr->address == REG_CH(ch, REG_UL_RSSI)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_RSSI_UL(ch)) & 0x7FFF;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_UL_RSSI_MAX)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_RSSI_PEAK_UL(ch)) & 0x7FFF;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_UL_RSSI_MIN)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_RSSI_PEAK_UL(ch))>>15) & 0x7FFF;
			if(val==32767)
				val=0;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DL_RSSI)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_RSSI_DL(ch)) & 0x7FFF;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DL_RSSI_MAX)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_RSSI_PEAK_DL(ch)) & 0x7FFF;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DL_RSSI_MIN)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_RSSI_PEAK_DL(ch))>>15) & 0x7FFF;
			if(val==32767)
				val=0;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_UL_GAIN_MAX)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_GAIN_UL(ch)) & 0xFFFF;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DL_GAIN_MAX)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_GAIN_DL(ch)) & 0xFFFF;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_UL_GAIN_MIN)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_GAIN_UL(ch))>>16;
			if(val==65535)
				val=0;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DL_GAIN_MIN)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_GAIN_DL(ch))>>16;
			if(val==65535)
				val=0;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_UL_TARGET_POWER)){
			match = 1;
			val = st->ul_target[ch];
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_UL_SQUELCH)){
			match = 1;
			val = st->ul_squelch[ch];
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_UL_GAIN_LIMIT)){
			match = 1;
			val = st->ul_gain_limit[ch];
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DL_TARGET_POWER)){
			match = 1;
			val = st->dl_target[ch];
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DL_SQUELCH)){
			match = 1;
			val = st->dl_squelch[ch];
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DL_GAIN_LIMIT)){
			match = 1;
			val = st->dl_gain_limit[ch];
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_CHANNEL_ENABLE)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_CHANNEL_EN) >> ch) & 1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_EN_FREQ_TRANSLATION)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_MUTE_LEN) >> (ch+16)) & 1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_UL_MUTE)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_RSSI_UL(ch))>>15) & 0x1;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DL_MUTE)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_RSSI_DL(ch))>>15) & 0x1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_BEST_SOURCE_MAX)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_RSSI_UL(ch))>>16) & 0xF;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_BEST_SOURCE_MIN)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_RSSI_UL(ch))>>20) & 0xF;
			break;
		}
	}

	for(port=0; port<NB_OF_TETRA_PORTS; port++){
		if((u32)this_attr->address == REG_PORT(port, REG_UL_SYNC)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_UL_SFP_SYNC)>>port) & 0x1;
			//val = (dras_radio_repeater_read(st, ADDR_PORT_ID(port))>>12) & 0x1;
			break;
		}
/*
		else if((u32)this_attr->address == REG_PORT(port, REG_OFFSET_TLAST)){
			match = 1;
			if(port<8){
				shift = port*4;
				val = (dras_radio_repeater_read(st, ADDR_OFFSET_TLAST0)>>shift) & 0xF;
			}else{
				shift = (port-8)*4;
				val = (dras_radio_repeater_read(st, ADDR_OFFSET_TLAST1)>>shift) & 0xF;
			}
			//val = val/2; // only odd channels used
			break;
		}
		else if((u32)this_attr->address == REG_PORT(port, REG_ENABLE_DL_TEST)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_CHANNEL_EN) >> (port+16)) & 1;
			break;
		}
		else if((u32)this_attr->address == REG_PORT(port, REG_UL_ORDER)){
			match = 1;
			temp32 = dras_radio_repeater_read(st, ADDR_UL_ORDER(port));
			//for(shift=0; shift<8; shift++){
			//	val += power10 * (((temp32>>(4*shift)) & 0xF)/2); // register contains 4bits per channel, but only each 2nd channel is used
			//	power10 = power10 * 10;
			//}
			ret = sprintf(buf, "%08x\n", temp32);
			break;
		}
		else if((u32)this_attr->address == REG_PORT(port, REG_PORT_ID)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_PORT_ID(port)) & 0xFFF;
			break;
		}
*/
	}

	if(match){
		mutex_unlock(&indio_dev->mlock);
		if(ret==0)
			ret = sprintf(buf, "%d\n", val);
		return ret;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_WIDEBAND_MU_RXTX4_FOR_COVERAGE:
		val = (dras_radio_repeater_read(st, ADDR_MUTE_LEN)>>15) & 0x1;
		break;
	case REG_HASH:
		val = dras_radio_repeater_read(st, ADDR_HASH);
		break;
	case REG_RANDOMNUMBER:
		val = dras_radio_repeater_read(st, ADDR_RANDOMNUMBER);
		break;
	case REG_DSP_VERSION:
		val = dras_radio_repeater_read(st, ADDR_DSP_VERSION);
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

IIO_DEVICE_ATTR_ALL_CH(uplink_rssi, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_UL_RSSI);

IIO_DEVICE_ATTR_ALL_CH(downlink_rssi, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_DL_RSSI);

IIO_DEVICE_ATTR_ALL_CH(uplink_rssi_max, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_UL_RSSI_MAX);

IIO_DEVICE_ATTR_ALL_CH(downlink_rssi_max, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_DL_RSSI_MAX);

IIO_DEVICE_ATTR_ALL_CH(uplink_rssi_min, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_UL_RSSI_MIN);

IIO_DEVICE_ATTR_ALL_CH(downlink_rssi_min, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_DL_RSSI_MIN);

IIO_DEVICE_ATTR_ALL_CH(uplink_gain_max, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_UL_GAIN_MAX);

IIO_DEVICE_ATTR_ALL_CH(downlink_gain_max, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_DL_GAIN_MAX);

IIO_DEVICE_ATTR_ALL_CH(uplink_gain_min, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_UL_GAIN_MIN);

IIO_DEVICE_ATTR_ALL_CH(downlink_gain_min, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_DL_GAIN_MIN);

IIO_DEVICE_ATTR_ALL_CH(uplink_target_power, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_UL_TARGET_POWER);

IIO_DEVICE_ATTR_ALL_CH(uplink_squelch, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_UL_SQUELCH);

IIO_DEVICE_ATTR_ALL_CH(uplink_gain_limit, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_UL_GAIN_LIMIT);

IIO_DEVICE_ATTR_ALL_CH(downlink_target_power, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_DL_TARGET_POWER);

IIO_DEVICE_ATTR_ALL_CH(downlink_squelch, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_DL_SQUELCH);

IIO_DEVICE_ATTR_ALL_CH(downlink_gain_limit, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_DL_GAIN_LIMIT);

IIO_DEVICE_ATTR_ALL_CH(channel_enable, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_CHANNEL_ENABLE);

IIO_DEVICE_ATTR_ALL_CH(enable_frequency_translation, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_EN_FREQ_TRANSLATION);

IIO_DEVICE_ATTR_ALL_CH(uplink_best_source_max, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_BEST_SOURCE_MAX);

IIO_DEVICE_ATTR_ALL_CH(uplink_best_source_min, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_BEST_SOURCE_MIN);

IIO_DEVICE_ATTR_ALL_CH(uplink_unmute, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_UL_MUTE);

IIO_DEVICE_ATTR_ALL_CH(downlink_unmute, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_DL_MUTE);
/*
IIO_DEVICE_ATTR_ALL_PORT(offset_tlast, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_OFFSET_TLAST);

IIO_DEVICE_ATTR_ALL_PORT(enable_downlink_test, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_ENABLE_DL_TEST);

IIO_DEVICE_ATTR_ALL_PORT(uplink_order, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_UL_ORDER);

IIO_DEVICE_ATTR_ALL_PORT(id, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_PORT_ID);
*/
IIO_DEVICE_ATTR_ALL_PORT(uplink_sync, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_UL_SYNC);

static IIO_DEVICE_ATTR(wideband_mu_rxtx4_for_coverage, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_WIDEBAND_MU_RXTX4_FOR_COVERAGE);

static IIO_DEVICE_ATTR(hash, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_HASH);

static IIO_DEVICE_ATTR(randomnumber, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_RANDOMNUMBER);

static IIO_DEVICE_ATTR(dsp_version, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_DSP_VERSION);


static struct attribute *dras_radio_repeater_attributes[] = {
	IIO_ATTR_ALL_CH(uplink_rssi),
	IIO_ATTR_ALL_CH(downlink_rssi),
	IIO_ATTR_ALL_CH(uplink_rssi_max),
	IIO_ATTR_ALL_CH(downlink_rssi_max),
	IIO_ATTR_ALL_CH(uplink_rssi_min),
	IIO_ATTR_ALL_CH(downlink_rssi_min),
	IIO_ATTR_ALL_CH(uplink_gain_max),
	IIO_ATTR_ALL_CH(downlink_gain_max),
	IIO_ATTR_ALL_CH(uplink_gain_min),
	IIO_ATTR_ALL_CH(downlink_gain_min),
	IIO_ATTR_ALL_CH(uplink_target_power),
	IIO_ATTR_ALL_CH(downlink_target_power),
	IIO_ATTR_ALL_CH(uplink_squelch),
	IIO_ATTR_ALL_CH(downlink_squelch),
	IIO_ATTR_ALL_CH(uplink_gain_limit),
	IIO_ATTR_ALL_CH(downlink_gain_limit),
	IIO_ATTR_ALL_CH(uplink_unmute),
	IIO_ATTR_ALL_CH(downlink_unmute),
	IIO_ATTR_ALL_CH(channel_enable),
	IIO_ATTR_ALL_CH(enable_frequency_translation),
	IIO_ATTR_ALL_CH(uplink_best_source_max),
	IIO_ATTR_ALL_CH(uplink_best_source_min),
	//IIO_ATTR_ALL_PORT(offset_tlast),
	//IIO_ATTR_ALL_PORT(enable_downlink_test),
	//IIO_ATTR_ALL_PORT(uplink_order),
	//IIO_ATTR_ALL_PORT(id),
	IIO_ATTR_ALL_PORT(uplink_sync),
	&iio_dev_attr_hash.dev_attr.attr,
	&iio_dev_attr_randomnumber.dev_attr.attr,
	&iio_dev_attr_dsp_version.dev_attr.attr,
	&iio_dev_attr_wideband_mu_rxtx4_for_coverage.dev_attr.attr,
	NULL,
};


static const struct attribute_group dras_radio_repeater_attribute_group = {
	.attrs = dras_radio_repeater_attributes,
};

static const struct iio_info dras_radio_repeater_info = {
	.read_raw = &dras_radio_repeater_read_raw,
	.write_raw = &dras_radio_repeater_write_raw,
	.attrs = &dras_radio_repeater_attribute_group,
};

static const struct iio_chan_spec dras_radio_repeater_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id dras_radio_repeater_of_match[] = {
	{ .compatible = "fpga,dras-radio-repeater", },
	{ },
};

MODULE_DEVICE_TABLE(of, dras_radio_repeater_of_match);

static int adrv_clk_clock_notifier(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct clk_notifier_data *ndata = data;
	struct dras_radio_repeater_state *st = container_of(nb, struct dras_radio_repeater_state, adrv_clk_rate_change_nb);

	dev_info(st->dev, "adrv_clk rate change: new rate = %lu Hz\n", ndata->new_rate);

	if (event == POST_RATE_CHANGE) {
		st->adrv_clk_rate = ndata->new_rate;
	}

	return NOTIFY_DONE;
}

static int dras_radio_repeater_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct dras_radio_repeater_state *st;
	struct iio_dev *indio_dev;
	int ret; //, i, n;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(dras_radio_repeater_of_match, &pdev->dev);
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
	indio_dev->channels = dras_radio_repeater_channels;
	indio_dev->num_channels = ARRAY_SIZE(dras_radio_repeater_channels);
	indio_dev->info = &dras_radio_repeater_info;
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

static int dras_radio_repeater_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver dras_radio_repeater_driver = {
	.probe		= dras_radio_repeater_probe,
	.remove		= dras_radio_repeater_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = dras_radio_repeater_of_match,
	},
};

module_platform_driver(dras_radio_repeater_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("DRAS RADIO REPEATER FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
