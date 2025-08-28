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
#define NB_OF_TETRA_CHANNELS		16

// common DSP addresses
#define ADDR_DSP_VERSION		(0*4)
#define ADDR_WB_ROUTING_FILTERSEL	(1*4)
#define ADDR_TX21_GAIN			(2*4)
#define ADDR_RX_BURST_LENGTH		(3*4)
#define ADDR_RX_BURST_PERIOD		(4*4)
#define ADDR_WB_DDS_INC			(5*4)
#define ADDR_DL_ORDER			(6*4) // 4bits per channel, 8 channels
#define ADDR_EN_UL_TEST_ID_OFFSET	(7*4) // EN_ULTEST, 4bit offset tlast, 12bit ID
#define ADDR_DL_SYNC			(8*4) // sync
#define ADDR_NB_FILTER_SEL0		(9*4)
#define ADDR_NB_FILTER_SEL1		(10*4)
#define ADDR_NB_IN_SEL			(11*4)
#define ADDR_NB_OUT_SEL			(12*4)
#define ADDR_NB_DDS_INC			(13*4)
#define ADDR_TESTTONE_INC		(14*4)
#define ADDR_TESTTONE_AMPL_TX1		(15*4)
#define ADDR_TESTTONE_AMPL_TX2		(16*4)
#define ADDR_BAND1_AGC_TARGET		(17*4)
#define ADDR_BAND1_AGC_MAXGAIN		(18*4)
#define ADDR_BAND1_AGC_SQUELCH		(19*4)
#define ADDR_BAND2_AGC_TARGET		(20*4)
#define ADDR_BAND2_AGC_MAXGAIN		(21*4)
#define ADDR_BAND2_AGC_SQUELCH		(22*4)
#define ADDR_BAND1_RSSI			(23*4)
#define ADDR_BAND2_RSSI			(24*4)
#define ADDR_RX_BURST_LENGTH2		(25*4)
#define ADDR_RX_BURST_PERIOD2		(26*4)
#define ADDR_TX_AVG_PWR			(27*4)
#define ADDR_TX_PEAK_PWR		(28*4)

#define MAX_BAND_FREQUENCY		20000000
#define MIN_BAND_FREQUENCY		-20000000
#define MAX_CH_FREQUENCY		3250000
#define MIN_CH_FREQUENCY		-3250000


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
	CH15_##REG

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
	static IIO_DEVICE_ATTR(ch7_##ATTR, RW, SHOW, STORE, CH7_##REG); \
	static IIO_DEVICE_ATTR(ch8_##ATTR, RW, SHOW, STORE, CH8_##REG); \
	static IIO_DEVICE_ATTR(ch9_##ATTR, RW, SHOW, STORE, CH9_##REG); \
	static IIO_DEVICE_ATTR(ch10_##ATTR, RW, SHOW, STORE, CH10_##REG); \
	static IIO_DEVICE_ATTR(ch11_##ATTR, RW, SHOW, STORE, CH11_##REG); \
	static IIO_DEVICE_ATTR(ch12_##ATTR, RW, SHOW, STORE, CH12_##REG); \
	static IIO_DEVICE_ATTR(ch13_##ATTR, RW, SHOW, STORE, CH13_##REG); \
	static IIO_DEVICE_ATTR(ch14_##ATTR, RW, SHOW, STORE, CH14_##REG); \
	static IIO_DEVICE_ATTR(ch15_##ATTR, RW, SHOW, STORE, CH15_##REG);

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

enum chan_num{
	REG_ALL_CH(REG_RX_FREQUENCY),	// being expanded for all channels
	REG_ALL_CH(REG_TX_FREQUENCY),	// being expanded for all channels
	REG_ALL_CH(REG_RX_BAND_SELECTION),	// being expanded for all channels
	REG_ALL_CH(REG_TX_BAND_SELECTION),	// being expanded for all channels
	REG_ALL_CH(REG_FILTER_SELECTION),	// being expanded for all channels
	REG_TX1_GAIN,
	REG_TX2_GAIN,
	REG_TX1_PA_COMP_GAIN,
	REG_TX2_PA_COMP_GAIN,
	REG_BAND1_RX_FREQUENCY,
	REG_BAND2_RX_FREQUENCY,
	REG_BAND1_TX_FREQUENCY,
	REG_BAND2_TX_FREQUENCY,
	REG_BAND1_RX_SELECTION,
	REG_BAND2_RX_SELECTION,
	REG_BAND1_FILTER_SELECTION,
	REG_BAND2_FILTER_SELECTION,
	REG_BAND1_TX1_ENABLE,
	REG_BAND1_TX2_ENABLE,
	REG_BAND2_TX1_ENABLE,
	REG_BAND2_TX2_ENABLE,
	REG_BAND1_WIDEBAND_MODE,
	REG_BAND2_WIDEBAND_MODE,
	REG_BAND1_AGC_TARGET,
	REG_BAND1_AGC_MAXGAIN,
	REG_BAND1_AGC_SQUELCH,
	REG_BAND2_AGC_TARGET,
	REG_BAND2_AGC_MAXGAIN,
	REG_BAND2_AGC_SQUELCH,
	REG_BAND1_RSSI,
	REG_BAND2_RSSI,
	REG_TX1_TESTTONE_FREQUENCY1,
	REG_TX1_TESTTONE_FREQUENCY2,
	REG_TX2_TESTTONE_FREQUENCY1,
	REG_TX2_TESTTONE_FREQUENCY2,
	REG_TX1_TESTTONE_AMPLITUDE1,
	REG_TX1_TESTTONE_AMPLITUDE2,
	REG_TX2_TESTTONE_AMPLITUDE1,
	REG_TX2_TESTTONE_AMPLITUDE2,
	REG_TX1_AVG_PWR,
	REG_TX2_AVG_PWR,
	REG_TX1_PEAK_PWR,
	REG_TX2_PEAK_PWR,
	REG_RX_BURST_LENGTH1,
	REG_RX_BURST_PERIOD1,
	REG_RX_DMA1_SOURCE_BAND1_RX1_BAND2_RX2,
	REG_RX_BURST_LENGTH2,
	REG_RX_BURST_PERIOD2,
	REG_RX_DMA2_SOURCE_BAND1_RX1_BAND2_RX2,
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

	uint32_t		tetra_clk;
	u32			gain_tx1;
	u32			gain_tx2;
	u32			pa_comp_gain_tx1;
	u32			pa_comp_gain_tx2;
	u32			gain_tx1_reg;
	u32			gain_tx2_reg;
	u32			testtone_ampl[4];
	bool			rf_mute;
	u32			nb_dds_inc[2*NB_OF_TETRA_CHANNELS];
	u32			wb_dds_inc[4];
	u32			testtone_dds_inc[4];

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
	int i;
	u32 temp32;
	u32 temp32_1;
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
		if((u32)this_attr->address == REG_CH(ch, REG_RX_FREQUENCY)){
			match = 1;
			if(val<MIN_CH_FREQUENCY || val>MAX_CH_FREQUENCY){
				ret = -EINVAL;
				break;
			}
			temp64 = (u64)val << 18;
			temp64 = div_s64(temp64,st->tetra_clk>>5);
			st->nb_dds_inc[ch*2] = (int)temp64 & 0x3FFFF; // rx dds auf geraden nummern, tx auf ungeraden
			val = ((int)temp64 & 0x3FFFF) | ((ch*2)<<18);
			dras_tetra_write(st, ADDR_NB_DDS_INC, val);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX_FREQUENCY)){
			match = 1;
			if(val<MIN_CH_FREQUENCY || val>MAX_CH_FREQUENCY){
				ret = -EINVAL;
				break;
			}
			temp64 = (u64)val << 18;
			temp64 = div_s64(temp64,st->tetra_clk>>5);
			st->nb_dds_inc[ch*2+1] = (int)temp64 & 0x3FFFF; // rx dds auf geraden nummern, tx auf ungeraden
			val = ((int)temp64 & 0x3FFFF) | ((ch*2+1)<<18);
			dras_tetra_write(st, ADDR_NB_DDS_INC, val);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_BAND_SELECTION)){
			match = 1;
			if(val<0 || val>2){
				ret = -EINVAL;
				break;
			}
			temp32 = dras_tetra_read(st, ADDR_NB_IN_SEL) & ~(3<<(2*ch));
			temp32 += ((uint32_t)val)<<(2*ch);
			dras_tetra_write(st, ADDR_NB_IN_SEL, temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX_BAND_SELECTION)){
			match = 1;
			if(val<0 || val>2){
				ret = -EINVAL;
				break;
			}
			temp32 = dras_tetra_read(st, ADDR_NB_OUT_SEL) & ~(3<<(2*ch));
			temp32 += ((uint32_t)val)<<(2*ch);
			dras_tetra_write(st, ADDR_NB_OUT_SEL, temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FILTER_SELECTION)){
			match = 1;
			if(val<1 || val>8){
				ret = -EINVAL;
				break;
			}
			val--;
			if(ch<8){
				temp32 = dras_tetra_read(st, ADDR_NB_FILTER_SEL0) & ~(7<<(4*ch));
				temp32 += ((uint32_t)val)<<(4*ch);
				dras_tetra_write(st, ADDR_NB_FILTER_SEL0, temp32);
			}else{
				temp32 = dras_tetra_read(st, ADDR_NB_FILTER_SEL1) & ~(7<<(4*ch-32));
				temp32 += ((uint32_t)val)<<(4*ch-32);
				dras_tetra_write(st, ADDR_NB_FILTER_SEL1, temp32);
			}
			break;
		}
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		return ret ? ret : len;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_BAND1_RX_FREQUENCY:
		if(val<MIN_BAND_FREQUENCY || val>MAX_BAND_FREQUENCY){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val << 18;
		temp64 = div_s64(temp64,st->tetra_clk>>2);
		st->wb_dds_inc[0] = (int)temp64 & 0x3FFFF; // rx dds auf geraden nummern, tx auf ungeraden
		val = ((int)temp64 & 0x3FFFF) | 0<<18;
		dras_tetra_write(st, ADDR_WB_DDS_INC, val);
		break;
	case REG_BAND2_RX_FREQUENCY:
		if(val<MIN_BAND_FREQUENCY || val>MAX_BAND_FREQUENCY){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val << 18;
		temp64 = div_s64(temp64,st->tetra_clk>>2);
		st->wb_dds_inc[2] = (int)temp64 & 0x3FFFF; // rx dds auf geraden nummern, tx auf ungeraden
		val = ((int)temp64 & 0x3FFFF) | 2<<18;
		dras_tetra_write(st, ADDR_WB_DDS_INC, val);
		break;
	case REG_BAND1_TX_FREQUENCY:
		if(val<MIN_BAND_FREQUENCY || val>MAX_BAND_FREQUENCY){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val << 18;
		temp64 = div_s64(temp64,st->tetra_clk>>2);
		st->wb_dds_inc[1] = (int)temp64 & 0x3FFFF; // rx dds auf geraden nummern, tx auf ungeraden
		val = ((int)temp64 & 0x3FFFF) | 1<<18;
		dras_tetra_write(st, ADDR_WB_DDS_INC, val);
		break;
	case REG_BAND2_TX_FREQUENCY:
		if(val<MIN_BAND_FREQUENCY || val>MAX_BAND_FREQUENCY){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val << 18;
		temp64 = div_s64(temp64,st->tetra_clk>>2);
		st->wb_dds_inc[3] = (int)temp64 & 0x3FFFF; // rx dds auf geraden nummern, tx auf ungeraden
		val = ((int)temp64 & 0x3FFFF) | 3<<18;
		dras_tetra_write(st, ADDR_WB_DDS_INC, val);
		break;
	case REG_TX1_TESTTONE_FREQUENCY1:
		if(val<MIN_BAND_FREQUENCY || val>MAX_BAND_FREQUENCY){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val << 18;
		temp64 = div_s64(temp64,st->tetra_clk>>2);
		st->testtone_dds_inc[0] = (int)temp64 & 0x3FFFF; // rx dds auf geraden nummern, tx auf ungeraden
		val = ((int)temp64 & 0x3FFFF) | 0<<18;
		dras_tetra_write(st, ADDR_TESTTONE_INC, val);
		break;
	case REG_TX1_TESTTONE_FREQUENCY2:
		if(val<MIN_BAND_FREQUENCY || val>MAX_BAND_FREQUENCY){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val << 18;
		temp64 = div_s64(temp64,st->tetra_clk>>2);
		st->testtone_dds_inc[1] = (int)temp64 & 0x3FFFF; // rx dds auf geraden nummern, tx auf ungeraden
		val = ((int)temp64 & 0x3FFFF) | 1<<18;
		dras_tetra_write(st, ADDR_TESTTONE_INC, val);
		break;
	case REG_TX2_TESTTONE_FREQUENCY1:
		if(val<MIN_BAND_FREQUENCY || val>MAX_BAND_FREQUENCY){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val << 18;
		temp64 = div_s64(temp64,st->tetra_clk>>2);
		st->testtone_dds_inc[2] = (int)temp64 & 0x3FFFF; // rx dds auf geraden nummern, tx auf ungeraden
		val = ((int)temp64 & 0x3FFFF) | 2<<18;
		dras_tetra_write(st, ADDR_TESTTONE_INC, val);
		break;
	case REG_TX2_TESTTONE_FREQUENCY2:
		if(val<MIN_BAND_FREQUENCY || val>MAX_BAND_FREQUENCY){
			ret = -EINVAL;
			break;
		}
		temp64 = (u64)val << 18;
		temp64 = div_s64(temp64,st->tetra_clk>>2);
		st->testtone_dds_inc[3] = (int)temp64 & 0x3FFFF; // rx dds auf geraden nummern, tx auf ungeraden
		val = ((int)temp64 & 0x3FFFF) | 3<<18;
		dras_tetra_write(st, ADDR_TESTTONE_INC, val);
		break;
	case REG_TX1_TESTTONE_AMPLITUDE1:
		if(val<0 || val>46286){
			ret = -EINVAL;
			break;
		}
		st->testtone_ampl[0] = val;
		val = ((u32)val*46286)>>15; // val*10^(3/20)*2^15
		val = (st->pa_comp_gain_tx1 * val) >> 8;
		if(val>0xFFFF)
			val = 0xFFFF;
		temp32 = dras_tetra_read(st, ADDR_TESTTONE_AMPL_TX1) & 0xFFFF0000;
		temp32 += ((uint32_t)val) & 0xFFFF;
		dras_tetra_write(st, ADDR_TESTTONE_AMPL_TX1, temp32);
		break;
	case REG_TX1_TESTTONE_AMPLITUDE2:
		if(val<0 || val>46286){
			ret = -EINVAL;
			break;
		}
		st->testtone_ampl[1] = val;
		val = ((u32)val*46286)>>15; // val*10^(3/20)*2^15
		val = (st->pa_comp_gain_tx1 * val) >> 8;
		if(val>0xFFFF)
			val = 0xFFFF;
		temp32 = dras_tetra_read(st, ADDR_TESTTONE_AMPL_TX1) & 0xFFFF;
		temp32 += ((uint32_t)val) <<16;
		dras_tetra_write(st, ADDR_TESTTONE_AMPL_TX1, temp32);
		break;
	case REG_TX2_TESTTONE_AMPLITUDE1:
		if(val<0 || val>46286){
			ret = -EINVAL;
			break;
		}
		st->testtone_ampl[2] = val;
		val = ((u32)val*46286)>>15; // val*10^(3/20)*2^15
		val = (st->pa_comp_gain_tx2 * val) >> 8;
		if(val>0xFFFF)
			val = 0xFFFF;
		temp32 = dras_tetra_read(st, ADDR_TESTTONE_AMPL_TX2) & 0xFFFF0000;
		temp32 += ((uint32_t)val) & 0xFFFF;
		dras_tetra_write(st, ADDR_TESTTONE_AMPL_TX2, temp32);
		break;
	case REG_TX2_TESTTONE_AMPLITUDE2:
		if(val<0 || val>46286){
			ret = -EINVAL;
			break;
		}
		st->testtone_ampl[3] = val;
		val = ((u32)val*46286)>>15; // val*10^(3/20)*2^15
		val = (st->pa_comp_gain_tx2 * val) >> 8;
		if(val>0xFFFF)
			val = 0xFFFF;
		temp32 = dras_tetra_read(st, ADDR_TESTTONE_AMPL_TX2) & 0xFFFF;
		temp32 += ((uint32_t)val) <<16;
		dras_tetra_write(st, ADDR_TESTTONE_AMPL_TX2, temp32);
		break;
	case REG_BAND1_FILTER_SELECTION:
		if(val<1 || val>4){
			ret = -EINVAL;
			break;
		}
		val--;
		temp32 = dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) & ~(3<<0);
		temp32 += ((uint32_t)val)<<0;
		dras_tetra_write(st, ADDR_WB_ROUTING_FILTERSEL, temp32);
		break;
	case REG_BAND2_FILTER_SELECTION:
		if(val<1 || val>4){
			ret = -EINVAL;
			break;
		}
		val--;
		temp32 = dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) & ~(3<<2);
		temp32 += ((uint32_t)val)<<2;
		dras_tetra_write(st, ADDR_WB_ROUTING_FILTERSEL, temp32);
		break;
	case REG_BAND1_RX_SELECTION:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) & ~(3<<4);
		temp32 += ((uint32_t)val)<<4;
		dras_tetra_write(st, ADDR_WB_ROUTING_FILTERSEL, temp32);
		break;
	case REG_BAND2_RX_SELECTION:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) & ~(3<<6);
		temp32 += ((uint32_t)val)<<6;
		dras_tetra_write(st, ADDR_WB_ROUTING_FILTERSEL, temp32);
		break;
	case REG_BAND1_TX1_ENABLE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) & ~(1<<8);
		temp32 += ((uint32_t)val)<<8;
		dras_tetra_write(st, ADDR_WB_ROUTING_FILTERSEL, temp32);
		break;
	case REG_BAND2_TX1_ENABLE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) & ~(1<<9);
		temp32 += ((uint32_t)val)<<9;
		dras_tetra_write(st, ADDR_WB_ROUTING_FILTERSEL, temp32);
		break;
	case REG_BAND1_TX2_ENABLE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) & ~(1<<10);
		temp32 += ((uint32_t)val)<<10;
		dras_tetra_write(st, ADDR_WB_ROUTING_FILTERSEL, temp32);
		break;
	case REG_BAND2_TX2_ENABLE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) & ~(1<<11);
		temp32 += ((uint32_t)val)<<11;
		dras_tetra_write(st, ADDR_WB_ROUTING_FILTERSEL, temp32);
		break;
	case REG_BAND1_WIDEBAND_MODE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) & ~(1<<12);
		temp32 += ((uint32_t)val)<<12;
		dras_tetra_write(st, ADDR_WB_ROUTING_FILTERSEL, temp32);
		break;
	case REG_BAND2_WIDEBAND_MODE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) & ~(1<<13);
		temp32 += ((uint32_t)val)<<13;
		dras_tetra_write(st, ADDR_WB_ROUTING_FILTERSEL, temp32);
		break;
	case REG_TX1_GAIN:
		if(val<0 || val>0xFFFF){
			ret = -EINVAL;
			break;
		}
		st->gain_tx1 = val;
		st->gain_tx1_reg = (st->pa_comp_gain_tx1 * val)>>8;
		if(st->gain_tx1_reg > 0xFFFF)
			st->gain_tx1_reg = 0xFFFF;
		if(st->rf_mute)
			break;
		dras_tetra_write(st, ADDR_TX21_GAIN,
			(st->gain_tx2_reg << 16) | st->gain_tx1_reg);
		break;
	case REG_TX2_GAIN:
		if(val<0 || val>0xFFFF){
			ret = -EINVAL;
			break;
		}
		st->gain_tx2 = val;
		st->gain_tx2_reg = (st->pa_comp_gain_tx2 * val)>>8;
		if(st->gain_tx2_reg > 0xFFFF)
			st->gain_tx2_reg = 0xFFFF;
		if(st->rf_mute)
			break;
		dras_tetra_write(st, ADDR_TX21_GAIN,
			(st->gain_tx2_reg << 16) | st->gain_tx1_reg);
		break;
	case REG_TX1_PA_COMP_GAIN:
		if(val<0 || val>0xFFFF){
			ret = -EINVAL;
			break;
		}
		st->pa_comp_gain_tx1 = val;
		// testtones
		temp32 = 0;
		for(i=0; i<=1; i++){
			temp32_1 = (st->testtone_ampl[i]*46286)>>15; // val*10^(3/20)*2^15
			temp32_1 = (st->pa_comp_gain_tx1 * temp32_1) >> 8;
			if(temp32_1>0xFFFF)
				temp32_1 = 0xFFFF;
			temp32 += temp32_1 << (i*16);
		}
		dras_tetra_write(st, ADDR_TESTTONE_AMPL_TX1, temp32);
		// txgain
		st->gain_tx1_reg = (st->gain_tx1 * val)>>8;
		if(st->gain_tx1_reg > 0xFFFF)
			st->gain_tx1_reg = 0xFFFF;
		if(st->rf_mute)
			break;
		dras_tetra_write(st, ADDR_TX21_GAIN,
			(st->gain_tx2_reg << 16) | st->gain_tx1_reg);
		break;
	case REG_TX2_PA_COMP_GAIN:
		if(val<0 || val>0xFFFF){
			ret = -EINVAL;
			break;
		}
		st->pa_comp_gain_tx2 = val;
		// testtones
		temp32 = 0;
		for(i=0; i<=1; i++){
			temp32_1 = (st->testtone_ampl[i+2]*46286)>>15; // val*10^(3/20)*2^15
			temp32_1 = (st->pa_comp_gain_tx2 * temp32_1) >> 8;
			if(temp32_1>0xFFFF)
				temp32_1 = 0xFFFF;
			temp32 += temp32_1 << (i*16);
		}
		dras_tetra_write(st, ADDR_TESTTONE_AMPL_TX2, temp32);
		// txgain
		st->gain_tx2_reg = (st->gain_tx2 * val)>>8;
		if(st->gain_tx2_reg > 0xFFFF)
			st->gain_tx2_reg = 0xFFFF;
		if(st->rf_mute)
			break;
		dras_tetra_write(st, ADDR_TX21_GAIN,
			(st->gain_tx2_reg << 16) | st->gain_tx1_reg);
		break;
	case REG_BAND1_AGC_TARGET:
		if(val>0xFFF){
			ret = -EINVAL;
			break;
		}
		dras_tetra_write(st, ADDR_BAND1_AGC_TARGET, (u32)val);
		break;
	case REG_BAND1_AGC_MAXGAIN:
		if(val>0xFFFFFF){
			ret = -EINVAL;
			break;
		}
		dras_tetra_write(st, ADDR_BAND1_AGC_MAXGAIN, (u32)val);
		break;
	case REG_BAND1_AGC_SQUELCH:
		if(val>0xFFFF){
			ret = -EINVAL;
			break;
		}
		dras_tetra_write(st, ADDR_BAND1_AGC_SQUELCH, (u32)val);
		break;
	case REG_BAND2_AGC_TARGET:
		if(val>0xFFF){
			ret = -EINVAL;
			break;
		}
		dras_tetra_write(st, ADDR_BAND2_AGC_TARGET, (u32)val);
		break;
	case REG_BAND2_AGC_MAXGAIN:
		if(val>0xFFFFFF){
			ret = -EINVAL;
			break;
		}
		dras_tetra_write(st, ADDR_BAND2_AGC_MAXGAIN, (u32)val);
		break;
	case REG_BAND2_AGC_SQUELCH:
		if(val>0xFFFF){
			ret = -EINVAL;
			break;
		}
		dras_tetra_write(st, ADDR_BAND2_AGC_SQUELCH, (u32)val);
		break;
	case REG_RX_BURST_LENGTH1:
		dras_tetra_write(st, ADDR_RX_BURST_LENGTH, (u32)val);
		break;
	case REG_RX_BURST_PERIOD1:
		dras_tetra_write(st, ADDR_RX_BURST_PERIOD, (u32)val);
		break;
	case REG_RX_BURST_LENGTH2:
		dras_tetra_write(st, ADDR_RX_BURST_LENGTH2, (u32)val);
		break;
	case REG_RX_BURST_PERIOD2:
		dras_tetra_write(st, ADDR_RX_BURST_PERIOD2, (u32)val);
		break;
	case REG_RX_DMA1_SOURCE_BAND1_RX1_BAND2_RX2:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) & ~(3<<14);
		temp32 += ((uint32_t)val)<<14;
		dras_tetra_write(st, ADDR_WB_ROUTING_FILTERSEL, temp32);
		break;
	case REG_RX_DMA2_SOURCE_BAND1_RX1_BAND2_RX2:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) & ~(3<<15);
		temp32 += ((uint32_t)val)<<15;
		dras_tetra_write(st, ADDR_WB_ROUTING_FILTERSEL, temp32);
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
				(st->gain_tx2_reg << 16) | st->gain_tx1_reg);
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
		if((u32)this_attr->address == REG_CH(ch, REG_RX_FREQUENCY)){
			match = 1;
			temp64 = (int32_t)st->nb_dds_inc[ch*2];
			temp64 = temp64 * (st->tetra_clk>>5);
			val = (int32_t)(temp64 >> 18);
			if(val > (st->tetra_clk >>6))
				val -= st->tetra_clk>>5;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX_FREQUENCY)){
			match = 1;
			temp64 = (int32_t)st->nb_dds_inc[ch*2+1];
			temp64 = temp64 * (st->tetra_clk>>5);
			val = (int32_t)(temp64 >> 18);
			if(val > (st->tetra_clk >>6))
				val -= st->tetra_clk>>5;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RX_BAND_SELECTION)){
			match = 1;
			val = ((dras_tetra_read(st, ADDR_NB_IN_SEL) >> (2*ch)) & 3);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_TX_BAND_SELECTION)){
			match = 1;
			val = ((dras_tetra_read(st, ADDR_NB_OUT_SEL) >> (2*ch)) & 3);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FILTER_SELECTION)){
			match = 1;
			if(ch<8){
				val = 1+(((dras_tetra_read(st, ADDR_NB_FILTER_SEL0) >> (4*ch)) & 7));
			}else{
				val = 1+(((dras_tetra_read(st, ADDR_NB_FILTER_SEL1) >> (4*ch-32)) & 7));
			}
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
	case REG_BAND1_RX_FREQUENCY:
		temp64 = (int32_t)st->wb_dds_inc[0];
		temp64 = temp64 * (st->tetra_clk>>2);
		val = (int32_t)(temp64 >> 18);
		if(val > (st->tetra_clk >>3))
			val -= st->tetra_clk>>2;
		break;
	case REG_BAND2_RX_FREQUENCY:
		temp64 = (int32_t)st->wb_dds_inc[2];
		temp64 = temp64 * (st->tetra_clk>>2);
		val = (int32_t)(temp64 >> 18);
		if(val > (st->tetra_clk >>3))
			val -= st->tetra_clk>>2;
		break;
	case REG_BAND1_TX_FREQUENCY:
		temp64 = (int32_t)st->wb_dds_inc[1];
		temp64 = temp64 * (st->tetra_clk>>2);
		val = (int32_t)(temp64 >> 18);
		if(val > (st->tetra_clk >>3))
			val -= st->tetra_clk>>2;
		break;
	case REG_BAND2_TX_FREQUENCY:
		temp64 = (int32_t)st->wb_dds_inc[3];
		temp64 = temp64 * (st->tetra_clk>>2);
		val = (int32_t)(temp64 >> 18);
		if(val > (st->tetra_clk >>3))
			val -= st->tetra_clk>>2;
		break;
	case REG_TX1_TESTTONE_FREQUENCY1:
		temp64 = (int32_t)st->testtone_dds_inc[0];
		temp64 = temp64 * (st->tetra_clk>>2);
		val = (int32_t)(temp64 >> 18);
		if(val > (st->tetra_clk >>3))
			val -= st->tetra_clk>>2;
		break;
	case REG_TX1_TESTTONE_FREQUENCY2:
		temp64 = (int32_t)st->testtone_dds_inc[1];
		temp64 = temp64 * (st->tetra_clk>>2);
		val = (int32_t)(temp64 >> 18);
		if(val > (st->tetra_clk >>3))
			val -= st->tetra_clk>>2;
		break;
	case REG_TX2_TESTTONE_FREQUENCY1:
		temp64 = (int32_t)st->testtone_dds_inc[2];
		temp64 = temp64 * (st->tetra_clk>>2);
		val = (int32_t)(temp64 >> 18);
		if(val > (st->tetra_clk >>3))
			val -= st->tetra_clk>>2;
		break;
	case REG_TX2_TESTTONE_FREQUENCY2:
		temp64 = (int32_t)st->testtone_dds_inc[3];
		temp64 = temp64 * (st->tetra_clk>>2);
		val = (int32_t)(temp64 >> 18);
		if(val > (st->tetra_clk >>3))
			val -= st->tetra_clk>>2;
		break;
	case REG_TX1_TESTTONE_AMPLITUDE1:
		val = st->testtone_ampl[0];
		break;
	case REG_TX1_TESTTONE_AMPLITUDE2:
		val = st->testtone_ampl[1];
		break;
	case REG_TX2_TESTTONE_AMPLITUDE1:
		val = st->testtone_ampl[2];
		break;
	case REG_TX2_TESTTONE_AMPLITUDE2:
		val = st->testtone_ampl[3];
		break;
	case REG_TX1_AVG_PWR:
		val = dras_tetra_read(st, ADDR_TX_AVG_PWR) & 0xFFFF;
		break;
	case REG_TX2_AVG_PWR:
		val = dras_tetra_read(st, ADDR_TX_AVG_PWR) >>16;
		break;
	case REG_TX1_PEAK_PWR:
		val = dras_tetra_read(st, ADDR_TX_PEAK_PWR) & 0xFFFF;
		break;
	case REG_TX2_PEAK_PWR:
		val = dras_tetra_read(st, ADDR_TX_PEAK_PWR) >>16;
		break;
	case REG_BAND1_FILTER_SELECTION:
		val = (dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) >> 0) & 3;
		val++;
		break;
	case REG_BAND2_FILTER_SELECTION:
		val = (dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) >> 2) & 3;
		val++;
		break;
	case REG_BAND1_RX_SELECTION:
		val = (dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) >> 4) & 3;
		break;
	case REG_BAND2_RX_SELECTION:
		val = (dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) >> 6) & 3;
		break;
	case REG_BAND1_TX1_ENABLE:
		val = (dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) >> 8) & 1;
		break;
	case REG_BAND2_TX1_ENABLE:
		val = (dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) >> 9) & 1;
		break;
	case REG_BAND1_TX2_ENABLE:
		val = (dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) >> 10) & 1;
		break;
	case REG_BAND2_TX2_ENABLE:
		val = (dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) >> 11) & 1;
		break;
	case REG_BAND1_WIDEBAND_MODE:
		val = (dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) >> 12) & 1;
		break;
	case REG_BAND2_WIDEBAND_MODE:
		val = (dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) >> 13) & 1;
		break;
	case REG_BAND1_AGC_TARGET:
		val = dras_tetra_read(st, ADDR_BAND1_AGC_TARGET) & 0xFFF;
		break;
	case REG_BAND1_AGC_MAXGAIN:
		val = dras_tetra_read(st, ADDR_BAND1_AGC_MAXGAIN) & 0xFFFFFF;
		break;
	case REG_BAND1_AGC_SQUELCH:
		val = dras_tetra_read(st, ADDR_BAND1_AGC_SQUELCH) & 0x7FFF;
		break;
	case REG_BAND2_AGC_TARGET:
		val = dras_tetra_read(st, ADDR_BAND2_AGC_TARGET) & 0xFFF;
		break;
	case REG_BAND2_AGC_MAXGAIN:
		val = dras_tetra_read(st, ADDR_BAND2_AGC_MAXGAIN) & 0xFFFFFF;
		break;
	case REG_BAND2_AGC_SQUELCH:
		val = dras_tetra_read(st, ADDR_BAND2_AGC_SQUELCH) & 0x7FFF;
		break;
	case REG_BAND1_RSSI:
		val = dras_tetra_read(st, ADDR_BAND1_RSSI) & 0xFFFF;
		break;
	case REG_BAND2_RSSI:
		val = dras_tetra_read(st, ADDR_BAND2_RSSI) & 0xFFFF;
		break;
	case REG_TX1_GAIN:
		val = st->gain_tx1;
		break;
	case REG_TX2_GAIN:
		val = st->gain_tx2;
		break;
	case REG_TX1_PA_COMP_GAIN:
		val = st->pa_comp_gain_tx1;
		break;
	case REG_TX2_PA_COMP_GAIN:
		val = st->pa_comp_gain_tx2;
		break;
	case REG_RX_BURST_LENGTH1:
		val = (uint32_t)dras_tetra_read(st, ADDR_RX_BURST_LENGTH);
		break;
	case REG_RX_BURST_PERIOD1:
		val = (uint32_t)dras_tetra_read(st, ADDR_RX_BURST_PERIOD);
		break;
	case REG_RX_BURST_LENGTH2:
		val = (uint32_t)dras_tetra_read(st, ADDR_RX_BURST_LENGTH2);
		break;
	case REG_RX_BURST_PERIOD2:
		val = (uint32_t)dras_tetra_read(st, ADDR_RX_BURST_PERIOD2);
		break;
	case REG_RX_DMA1_SOURCE_BAND1_RX1_BAND2_RX2:
		val = (dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) >> 14) & 3;
		break;
	case REG_RX_DMA2_SOURCE_BAND1_RX1_BAND2_RX2:
		val = (dras_tetra_read(st, ADDR_WB_ROUTING_FILTERSEL) >> 15) & 3;
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

IIO_DEVICE_ATTR_ALL_CH(rx_frequency, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_FREQUENCY);

IIO_DEVICE_ATTR_ALL_CH(tx_frequency, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX_FREQUENCY);

IIO_DEVICE_ATTR_ALL_CH(rx_band_selection, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_BAND_SELECTION);

IIO_DEVICE_ATTR_ALL_CH(tx_band_selection, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX_BAND_SELECTION);

IIO_DEVICE_ATTR_ALL_CH(filter_selection, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_FILTER_SELECTION);

static IIO_DEVICE_ATTR(tx1_gain, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX1_GAIN);

static IIO_DEVICE_ATTR(tx2_gain, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX2_GAIN);

static IIO_DEVICE_ATTR(tx1_pa_comp_gain, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX1_PA_COMP_GAIN);

static IIO_DEVICE_ATTR(tx2_pa_comp_gain, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX2_PA_COMP_GAIN);

static IIO_DEVICE_ATTR(band1_rx_frequency, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND1_RX_FREQUENCY);

static IIO_DEVICE_ATTR(band2_rx_frequency, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND2_RX_FREQUENCY);

static IIO_DEVICE_ATTR(band1_tx_frequency, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND1_TX_FREQUENCY);

static IIO_DEVICE_ATTR(band2_tx_frequency, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND2_TX_FREQUENCY);

static IIO_DEVICE_ATTR(band1_rx_selection, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND1_RX_SELECTION);

static IIO_DEVICE_ATTR(band2_rx_selection, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND2_RX_SELECTION);

static IIO_DEVICE_ATTR(band1_filter_selection, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND1_FILTER_SELECTION);

static IIO_DEVICE_ATTR(band2_filter_selection, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND2_FILTER_SELECTION);

static IIO_DEVICE_ATTR(band1_tx1_enable, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND1_TX1_ENABLE);

static IIO_DEVICE_ATTR(band1_tx2_enable, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND1_TX2_ENABLE);

static IIO_DEVICE_ATTR(band2_tx1_enable, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND2_TX1_ENABLE);

static IIO_DEVICE_ATTR(band2_tx2_enable, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND2_TX2_ENABLE);

static IIO_DEVICE_ATTR(band1_wideband_mode, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND1_WIDEBAND_MODE);

static IIO_DEVICE_ATTR(band2_wideband_mode, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND2_WIDEBAND_MODE);

static IIO_DEVICE_ATTR(band1_target, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND1_AGC_TARGET);

static IIO_DEVICE_ATTR(band1_maxgain, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND1_AGC_MAXGAIN);

static IIO_DEVICE_ATTR(band1_squelch, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND1_AGC_SQUELCH);

static IIO_DEVICE_ATTR(band2_target, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND2_AGC_TARGET);

static IIO_DEVICE_ATTR(band2_maxgain, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND2_AGC_MAXGAIN);

static IIO_DEVICE_ATTR(band2_squelch, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND2_AGC_SQUELCH);

static IIO_DEVICE_ATTR(band1_rssi, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND1_RSSI);

static IIO_DEVICE_ATTR(band2_rssi, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_BAND2_RSSI);

static IIO_DEVICE_ATTR(tx1_testtone_frequency1, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX1_TESTTONE_FREQUENCY1);

static IIO_DEVICE_ATTR(tx1_testtone_frequency2, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX1_TESTTONE_FREQUENCY2);

static IIO_DEVICE_ATTR(tx2_testtone_frequency1, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX2_TESTTONE_FREQUENCY1);

static IIO_DEVICE_ATTR(tx2_testtone_frequency2, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX2_TESTTONE_FREQUENCY2);

static IIO_DEVICE_ATTR(tx1_testtone_amplitude1, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX1_TESTTONE_AMPLITUDE1);

static IIO_DEVICE_ATTR(tx1_testtone_amplitude2, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX1_TESTTONE_AMPLITUDE2);

static IIO_DEVICE_ATTR(tx2_testtone_amplitude1, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX2_TESTTONE_AMPLITUDE1);

static IIO_DEVICE_ATTR(tx2_testtone_amplitude2, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX2_TESTTONE_AMPLITUDE2);

static IIO_DEVICE_ATTR(tx1_avg_power, S_IRUGO,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX1_AVG_PWR);

static IIO_DEVICE_ATTR(tx2_avg_power, S_IRUGO,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX2_AVG_PWR);

static IIO_DEVICE_ATTR(tx1_peak_power, S_IRUGO,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX1_PEAK_PWR);

static IIO_DEVICE_ATTR(tx2_peak_power, S_IRUGO,
			dras_tetra_show,
			dras_tetra_store,
			REG_TX2_PEAK_PWR);

static IIO_DEVICE_ATTR(rx_burst_length1, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_BURST_LENGTH1);

static IIO_DEVICE_ATTR(rx_burst_period1, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_BURST_PERIOD1);

static IIO_DEVICE_ATTR(rx_dma1_source_band1_rx1_band2_rx2, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_DMA1_SOURCE_BAND1_RX1_BAND2_RX2);

static IIO_DEVICE_ATTR(rx_burst_length2, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_BURST_LENGTH2);

static IIO_DEVICE_ATTR(rx_burst_period2, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_BURST_PERIOD2);

static IIO_DEVICE_ATTR(rx_dma2_source_band1_rx1_band2_rx2, S_IRUGO | S_IWUSR,
			dras_tetra_show,
			dras_tetra_store,
			REG_RX_DMA2_SOURCE_BAND1_RX1_BAND2_RX2);

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
	IIO_ATTR_ALL_CH(rx_frequency),
	IIO_ATTR_ALL_CH(tx_frequency),
	IIO_ATTR_ALL_CH(rx_band_selection),
	IIO_ATTR_ALL_CH(tx_band_selection),
	IIO_ATTR_ALL_CH(filter_selection),
	&iio_dev_attr_tx1_gain.dev_attr.attr,
	&iio_dev_attr_tx2_gain.dev_attr.attr,
	&iio_dev_attr_tx1_pa_comp_gain.dev_attr.attr,
	&iio_dev_attr_tx2_pa_comp_gain.dev_attr.attr,
	&iio_dev_attr_band1_rx_frequency.dev_attr.attr,
	&iio_dev_attr_band2_rx_frequency.dev_attr.attr,
	&iio_dev_attr_band1_tx_frequency.dev_attr.attr,
	&iio_dev_attr_band2_tx_frequency.dev_attr.attr,
	&iio_dev_attr_band1_rx_selection.dev_attr.attr,
	&iio_dev_attr_band2_rx_selection.dev_attr.attr,
	&iio_dev_attr_band1_filter_selection.dev_attr.attr,
	&iio_dev_attr_band2_filter_selection.dev_attr.attr,
	&iio_dev_attr_band1_tx1_enable.dev_attr.attr,
	&iio_dev_attr_band1_tx2_enable.dev_attr.attr,
	&iio_dev_attr_band2_tx1_enable.dev_attr.attr,
	&iio_dev_attr_band2_tx2_enable.dev_attr.attr,
	&iio_dev_attr_band1_wideband_mode.dev_attr.attr,
	&iio_dev_attr_band2_wideband_mode.dev_attr.attr,
	&iio_dev_attr_band1_target.dev_attr.attr,
	&iio_dev_attr_band1_maxgain.dev_attr.attr,
	&iio_dev_attr_band1_squelch.dev_attr.attr,
	&iio_dev_attr_band2_target.dev_attr.attr,
	&iio_dev_attr_band2_maxgain.dev_attr.attr,
	&iio_dev_attr_band2_squelch.dev_attr.attr,
	&iio_dev_attr_band1_rssi.dev_attr.attr,
	&iio_dev_attr_band2_rssi.dev_attr.attr,
	&iio_dev_attr_tx1_testtone_frequency1.dev_attr.attr,
	&iio_dev_attr_tx1_testtone_frequency2.dev_attr.attr,
	&iio_dev_attr_tx2_testtone_frequency1.dev_attr.attr,
	&iio_dev_attr_tx2_testtone_frequency2.dev_attr.attr,
	&iio_dev_attr_tx1_testtone_amplitude1.dev_attr.attr,
	&iio_dev_attr_tx1_testtone_amplitude2.dev_attr.attr,
	&iio_dev_attr_tx2_testtone_amplitude1.dev_attr.attr,
	&iio_dev_attr_tx2_testtone_amplitude2.dev_attr.attr,
	&iio_dev_attr_tx1_avg_power.dev_attr.attr,
	&iio_dev_attr_tx2_avg_power.dev_attr.attr,
	&iio_dev_attr_tx1_peak_power.dev_attr.attr,
	&iio_dev_attr_tx2_peak_power.dev_attr.attr,
	&iio_dev_attr_rx_burst_length1.dev_attr.attr,
	&iio_dev_attr_rx_burst_period1.dev_attr.attr,
	&iio_dev_attr_rx_burst_length2.dev_attr.attr,
	&iio_dev_attr_rx_burst_period2.dev_attr.attr,
	&iio_dev_attr_rx_dma1_source_band1_rx1_band2_rx2.dev_attr.attr,
	&iio_dev_attr_rx_dma2_source_band1_rx1_band2_rx2.dev_attr.attr,
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


	if(of_property_read_u32(np, "required,tetra-clk", &st->tetra_clk)){
		printk("DRAS-TETRA: ***ERROR! \"required,tetra-clk\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->tetra_clk == 0){
		printk("DRAS-TETRA: ***ERROR! \"required,tetra-clk\" equal to 0 Hz\n");
		goto err_iio_device_free;
	}

	indio_dev->name = np->name;
	indio_dev->channels = dras_tetra_channels;
	indio_dev->num_channels = ARRAY_SIZE(dras_tetra_channels);
	indio_dev->info = &dras_tetra_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	/* initially mute TX of both TX */
	st->rf_mute = true;
	st->pa_comp_gain_tx1 = 256;
	st->pa_comp_gain_tx2 = 256;

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
