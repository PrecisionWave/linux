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


#define DRIVER_NAME				"vbi-fm-dsp"
#define NB_OF_BLOCKS			8
#define ADDR_PER_BLOCK			16*4

#define ADDR_DSP_VERSION		(0*4)
#define ADDR_ADC_PEAK			(1*4)
#define ADDR_AUDIO_METER		(2*4)
#define ADDR_PPS_CLKS			(3*4)
#define ADDR_PPS_CNT			(4*4)
#define ADDR_ADC_BER_TESTER		(5*4)
#define ADDR_DMA_BURST_LENGTH		(6*4)
#define ADDR_DMA_BURST_PERIOD		(7*4)
#define ADDR_DMA_DECIMATION		(8*4)
#define ADDR_DMA_SINK_WATCHDOG			(9*4)
#define ADDR_DMA_SOURCE_CHANNEL		(10*4)
#define ADDR_AUDIO_SEL			(11*4)
#define ADDR_PPS_SETTINGS		(12*4)
#define ADDR_GAIN_MOD			(13*4)
#define ADDR_MONO_SWP_SOURCE		(14*4)
#define ADDR_MOD_CH_SHIFT		(15*4)

#define ADDR_ROUTING(x)			(0*4+(x+1)*ADDR_PER_BLOCK)
#define ADDR_INC01(x)			(1*4+(x+1)*ADDR_PER_BLOCK)
#define ADDR_INC23(x)			(2*4+(x+1)*ADDR_PER_BLOCK)
#define ADDR_GAIN01_0(x)		(3*4+(x+1)*ADDR_PER_BLOCK)
#define ADDR_GAIN23_0(x)		(4*4+(x+1)*ADDR_PER_BLOCK)
#define ADDR_GAIN01_1(x)		(5*4+(x+1)*ADDR_PER_BLOCK)
#define ADDR_GAIN23_1(x)		(6*4+(x+1)*ADDR_PER_BLOCK)
#define ADDR_RSSI01(x)			(10*4+(x+1)*ADDR_PER_BLOCK)
#define ADDR_RSSI23(x)			(11*4+(x+1)*ADDR_PER_BLOCK)

#define MIN_GAIN				0x0000
#define MAX_GAIN				0xFFFF


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
//   IIO_DEVICE_ATTR_ALL_CH(gain_tx1, S_IRUGO | S_IWUSR, vbi_fm_dsp_show, vbi_fm_dsp_store, REG_GAIN_TX1)
//     expansion:
//     static IIO_DEVICE_ATTR(ch0_gain_tx1, S_IRUGO | S_IWUSR, vbi_fm_dsp_show, vbi_fm_dsp_store, CH0_REG_GAIN_TX1);
//     static IIO_DEVICE_ATTR(ch1_gain_tx1, S_IRUGO | S_IWUSR, vbi_fm_dsp_show, vbi_fm_dsp_store, CH1_REG_GAIN_TX1);
//      ::   ::
//     static IIO_DEVICE_ATTR(ch31_gain_tx1, S_IRUGO | S_IWUSR, vbi_fm_dsp_show, vbi_fm_dsp_store, CH31_REG_GAIN_TX1);
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
	REG_ALL_CH(REG_GAIN_TX1),	// being expanded for all channels
	REG_ALL_CH(REG_GAIN_TX2),	// being expanded for all channels
	REG_ALL_CH(REG_MODULATOR_GAIN_TX1),	// being expanded for all channels
	REG_ALL_CH(REG_MODULATOR_GAIN_TX2),	// being expanded for all channels
	REG_ALL_CH(REG_FREQUENCY),	// being expanded for all channels
	REG_ALL_CH(REG_SWAP_TX1),	// being expanded for all channels
	REG_ALL_CH(REG_SWAP_TX2),	// being expanded for all channels
	REG_ALL_CH(REG_DDS_MODE_TX1),	// being expanded for all channels
	REG_ALL_CH(REG_DDS_MODE_TX2),	// being expanded for all channels
	REG_ALL_CH(REG_VBI_MODE_TX1),	// being expanded for all channels
	REG_ALL_CH(REG_VBI_MODE_TX2),	// being expanded for all channels
	REG_ALL_CH(REG_RF_INPUT_SELECTION),	// being expanded for all channels
	REG_ALL_CH(REG_AUDIO_SELECTION),	// being expanded for all channels
	REG_ALL_CH(REG_RSSI),	// being expanded for all channels
	REG_DSP_VERSION,
	REG_ADC1_PEAK_HOLD_VAL,
	REG_ADC2_PEAK_HOLD_VAL,
	REG_ADC_BER_ALTERNATE,
	REG_ADC_BER_CHECKER,
	REG_DMA_SOURCE,
	REG_DMA_SOURCE_CHANNEL,
	REG_DMA_SINK,
	REG_DMA_BURST_PERIOD,
	REG_DMA_BURST_LENGTH,
	REG_DMA_DECIMATION,
	REG_WATCHDOG_ENABLE,
	REG_WATCHDOG_TRIGGER,
	REG_GAIN_AUDIO,
	REG_GAIN_RDS,
	REG_METER_AUDIO1,
	REG_METER_AUDIO2,
	REG_AUDIO_MIX2MONO,
	REG_AUDIO_SWAP,
	REG_AUDIO_FROM_INPUT,
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

struct vbi_fm_dsp_state {
	struct iio_info		iio_info;
	void __iomem		*regs;
	struct mutex		lock;

	uint32_t		fs_adc;
	bool			gpsdo_locked;
  	uint32_t		pps_clk_error_ns;
  	uint32_t		pps_clk_error_hz;
	uint32_t		nb_of_blocks;
	uint32_t		gain_tx1[32];
	uint32_t		gain_tx2[32];
	uint32_t		modulator_gain_tx1[32];
	uint32_t		modulator_gain_tx2[32];
	uint32_t		vbi_mode_tx1[32];
	uint32_t		vbi_mode_tx2[32];
};

static void vbi_fm_dsp_write(struct vbi_fm_dsp_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 vbi_fm_dsp_read(struct vbi_fm_dsp_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int vbi_fm_dsp_write_raw(struct iio_dev *indio_dev,
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

static int vbi_fm_dsp_read_raw(struct iio_dev *indio_dev,
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

static ssize_t vbi_fm_dsp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct vbi_fm_dsp_state *st = iio_priv(indio_dev);
	long val;
	int ret;
	u64 temp64;
	int tempint;
	u32 temp32;
	int subchannel;
	int block_nb;
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
	for(ch=0; ch<32; ch++){
		subchannel = (ch & 0x3);
		block_nb = (ch >> 2);
		if((u32)this_attr->address == REG_CH(ch, REG_GAIN_TX1)){
			match = 1;
			if(val<MIN_GAIN || val>MAX_GAIN){
				ret = -EINVAL;
				break;
			}
			switch(subchannel){
			case 0:
				st->gain_tx1[ch] = (u32)val;
				if(!st->vbi_mode_tx1[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN01_0(block_nb)) & 0xFFFF0000;
					temp32 += st->gain_tx1[ch];
					vbi_fm_dsp_write(st, ADDR_GAIN01_0(block_nb), temp32);
				}
				break;
			case 1:
				st->gain_tx1[ch] = (u32)val;
				if(!st->vbi_mode_tx1[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN01_0(block_nb)) & 0xFFFF;
					temp32 += st->gain_tx1[ch] << 16;
					vbi_fm_dsp_write(st, ADDR_GAIN01_0(block_nb), temp32);
				}
				break;
			case 2:
				st->gain_tx1[ch] = (u32)val;
				if(!st->vbi_mode_tx1[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN23_0(block_nb)) & 0xFFFF0000;
					temp32 += st->gain_tx1[ch];
					vbi_fm_dsp_write(st, ADDR_GAIN23_0(block_nb), temp32);
				}
				break;
			case 3:
				st->gain_tx1[ch] = (u32)val;
				if(!st->vbi_mode_tx1[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN23_0(block_nb)) & 0xFFFF;
					temp32 += st->gain_tx1[ch] << 16;
					vbi_fm_dsp_write(st, ADDR_GAIN23_0(block_nb), temp32);
				}
				break;
			default:
				break;
			}
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_GAIN_TX2)){
			match = 1;
			if(val<MIN_GAIN || val>MAX_GAIN){
				ret = -EINVAL;
				break;
			}
			switch(subchannel){
			case 0:
				st->gain_tx2[ch] = (u32)val;
				if(!st->vbi_mode_tx2[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN01_1(block_nb)) & 0xFFFF0000;
					temp32 += st->gain_tx2[ch];
					vbi_fm_dsp_write(st, ADDR_GAIN01_1(block_nb), temp32);
				}
				break;
			case 1:
				st->gain_tx2[ch] = (u32)val;
				if(!st->vbi_mode_tx2[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN01_1(block_nb)) & 0xFFFF;
					temp32 += st->gain_tx2[ch] << 16;
					vbi_fm_dsp_write(st, ADDR_GAIN01_1(block_nb), temp32);
				}
				break;
			case 2:
				st->gain_tx2[ch] = (u32)val;
				if(!st->vbi_mode_tx2[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN23_1(block_nb)) & 0xFFFF0000;
					temp32 += st->gain_tx2[ch];
					vbi_fm_dsp_write(st, ADDR_GAIN23_1(block_nb), temp32);
				}
				break;
			case 3:
				st->gain_tx2[ch] = (u32)val;
				if(!st->vbi_mode_tx2[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN23_1(block_nb)) & 0xFFFF;
					temp32 += st->gain_tx2[ch] << 16;
					vbi_fm_dsp_write(st, ADDR_GAIN23_1(block_nb), temp32);
				}
				break;
			default:
				break;
			}
			break;
		}
		if((u32)this_attr->address == REG_CH(ch, REG_MODULATOR_GAIN_TX1)){
			match = 1;
			if(val<MIN_GAIN || val>MAX_GAIN){
				ret = -EINVAL;
				break;
			}
			switch(subchannel){
			case 0:
				st->modulator_gain_tx1[ch] = (u32)val;
				if(st->vbi_mode_tx1[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN01_0(block_nb)) & 0xFFFF0000;
					temp32 += st->modulator_gain_tx1[ch];
					vbi_fm_dsp_write(st, ADDR_GAIN01_0(block_nb), temp32);
				}
				break;
			case 1:
				st->modulator_gain_tx1[ch] = (u32)val;
				if(st->vbi_mode_tx1[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN01_0(block_nb)) & 0xFFFF;
					temp32 += st->modulator_gain_tx1[ch] << 16;
					vbi_fm_dsp_write(st, ADDR_GAIN01_0(block_nb), temp32);
				}
				break;
			case 2:
				st->modulator_gain_tx1[ch] = (u32)val;
				if(st->vbi_mode_tx1[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN23_0(block_nb)) & 0xFFFF0000;
					temp32 += st->modulator_gain_tx1[ch];
					vbi_fm_dsp_write(st, ADDR_GAIN23_0(block_nb), temp32);
				}
				break;
			case 3:
				st->modulator_gain_tx1[ch] = (u32)val;
				if(st->vbi_mode_tx1[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN23_0(block_nb)) & 0xFFFF;
					temp32 += st->modulator_gain_tx1[ch] << 16;
					vbi_fm_dsp_write(st, ADDR_GAIN23_0(block_nb), temp32);
				}
				break;
			default:
				break;
			}
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_MODULATOR_GAIN_TX2)){
			match = 1;
			if(val<MIN_GAIN || val>MAX_GAIN){
				ret = -EINVAL;
				break;
			}
			switch(subchannel){
			case 0:
				st->modulator_gain_tx2[ch] = (u32)val;
				if(st->vbi_mode_tx2[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN01_1(block_nb)) & 0xFFFF0000;
					temp32 += st->modulator_gain_tx2[ch];
					vbi_fm_dsp_write(st, ADDR_GAIN01_1(block_nb), temp32);
				}
				break;
			case 1:
				st->modulator_gain_tx2[ch] = (u32)val;
				if(st->vbi_mode_tx2[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN01_1(block_nb)) & 0xFFFF;
					temp32 += st->modulator_gain_tx2[ch] << 16;
					vbi_fm_dsp_write(st, ADDR_GAIN01_1(block_nb), temp32);
				}
				break;
			case 2:
				st->modulator_gain_tx2[ch] = (u32)val;
				if(st->vbi_mode_tx2[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN23_1(block_nb)) & 0xFFFF0000;
					temp32 += st->modulator_gain_tx2[ch];
					vbi_fm_dsp_write(st, ADDR_GAIN23_1(block_nb), temp32);
				}
				break;
			case 3:
				st->modulator_gain_tx2[ch] = (u32)val;
				if(st->vbi_mode_tx2[ch]){
					temp32 = vbi_fm_dsp_read(st, ADDR_GAIN23_1(block_nb)) & 0xFFFF;
					temp32 += st->modulator_gain_tx2[ch] << 16;
					vbi_fm_dsp_write(st, ADDR_GAIN23_1(block_nb), temp32);
				}
				break;
			default:
				break;
			}
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FREQUENCY)){
			match = 1;
			if(val<0 || val>st->fs_adc/2){
				ret = -EINVAL;
				break;
			}
			temp64 = (u64)val << 18;
			temp64 = div_s64(temp64,st->fs_adc/2);
			tempint = (int)temp64;
			tempint -= (11*(1 << 18))>>4;
			if(tempint<0)
				tempint += 1<<16;
			tempint = tempint & 0xFFFF;
			switch(subchannel){
			case 0:
				temp32 = vbi_fm_dsp_read(st, ADDR_INC01(block_nb)) & 0xFFFF0000;
				temp32 += (u32)tempint;
				vbi_fm_dsp_write(st, ADDR_INC01(block_nb), temp32);
				break;
			case 1:
				temp32 = vbi_fm_dsp_read(st, ADDR_INC01(block_nb)) & 0xFFFF;
				temp32 += (u32)tempint << 16;
				vbi_fm_dsp_write(st, ADDR_INC01(block_nb), temp32);
				break;
			case 2:
				temp32 = vbi_fm_dsp_read(st, ADDR_INC23(block_nb)) & 0xFFFF0000;
				temp32 += (u32)tempint;
				vbi_fm_dsp_write(st, ADDR_INC23(block_nb), temp32);
				break;
			case 3:
				temp32 = vbi_fm_dsp_read(st, ADDR_INC23(block_nb)) & 0xFFFF;
				temp32 += (u32)tempint << 16;
				vbi_fm_dsp_write(st, ADDR_INC23(block_nb), temp32);
				break;
			default:
				break;
			}
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_SWAP_TX1)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			switch(subchannel){
			case 0:
			case 1:
				temp32 = vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & ~(1<<4);
				temp32 += (u32)val << 4;
				vbi_fm_dsp_write(st, ADDR_ROUTING(block_nb), temp32);
				break;
			case 2:
			case 3:
				temp32 = vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & ~(1<<5);
				temp32 += (u32)val << 5;
				vbi_fm_dsp_write(st, ADDR_ROUTING(block_nb), temp32);
				break;
			default:
				break;
			}
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_SWAP_TX2)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			switch(subchannel){
			case 0:
			case 1:
				temp32 = vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & ~(1<<14);
				temp32 += (u32)val << 14;
				vbi_fm_dsp_write(st, ADDR_ROUTING(block_nb), temp32);
				break;
			case 2:
			case 3:
				temp32 = vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & ~(1<<15);
				temp32 += (u32)val << 15;
				vbi_fm_dsp_write(st, ADDR_ROUTING(block_nb), temp32);
				break;
			default:
				break;
			}
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DDS_MODE_TX1)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & ~(1<<(6+subchannel));
			temp32 += (u32)val << (6+subchannel);
			vbi_fm_dsp_write(st, ADDR_ROUTING(block_nb), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DDS_MODE_TX2)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & ~(1<<(16+subchannel));
			temp32 += (u32)val << (16+subchannel);
			vbi_fm_dsp_write(st, ADDR_ROUTING(block_nb), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_VBI_MODE_TX1)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			st->vbi_mode_tx1[ch] = (u32)val;
			temp32 = vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & ~(1<<(10+subchannel));
			temp32 += st->vbi_mode_tx1[ch] << (10+subchannel);
			vbi_fm_dsp_write(st, ADDR_ROUTING(block_nb), temp32);
			switch(subchannel){
			case 0:
				temp32 = vbi_fm_dsp_read(st, ADDR_GAIN01_0(block_nb)) & 0xFFFF0000;
				if(st->vbi_mode_tx1[ch])
					temp32 += st->modulator_gain_tx1[ch];
				else
					temp32 += st->gain_tx1[ch];
				vbi_fm_dsp_write(st, ADDR_GAIN01_0(block_nb), temp32);
				break;
			case 1:
				temp32 = vbi_fm_dsp_read(st, ADDR_GAIN01_0(block_nb)) & 0xFFFF;
				if(st->vbi_mode_tx1[ch])
					temp32 += st->modulator_gain_tx1[ch] << 16;
				else
					temp32 += st->gain_tx1[ch] << 16;
				vbi_fm_dsp_write(st, ADDR_GAIN01_0(block_nb), temp32);
				break;
			case 2:
				temp32 = vbi_fm_dsp_read(st, ADDR_GAIN23_0(block_nb)) & 0xFFFF0000;
				if(st->vbi_mode_tx1[ch])
					temp32 += st->modulator_gain_tx1[ch];
				else
					temp32 += st->gain_tx1[ch];
				vbi_fm_dsp_write(st, ADDR_GAIN23_0(block_nb), temp32);
				break;
			case 3:
				temp32 = vbi_fm_dsp_read(st, ADDR_GAIN23_0(block_nb)) & 0xFFFF;
				if(st->vbi_mode_tx1[ch])
					temp32 += st->modulator_gain_tx1[ch] << 16;
				else
					temp32 += st->gain_tx1[ch] << 16;
				vbi_fm_dsp_write(st, ADDR_GAIN23_0(block_nb), temp32);
				break;
			default:
				break;
			}
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_VBI_MODE_TX2)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			st->vbi_mode_tx2[ch] = (u32)val;
			temp32 = vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & ~(1<<(20+subchannel));
			temp32 += st->vbi_mode_tx2[ch] << (20+subchannel);
			vbi_fm_dsp_write(st, ADDR_ROUTING(block_nb), temp32);
			switch(subchannel){
			case 0:
				temp32 = vbi_fm_dsp_read(st, ADDR_GAIN01_1(block_nb)) & 0xFFFF0000;
				if(st->vbi_mode_tx2[ch])
					temp32 += st->modulator_gain_tx2[ch];
				else
					temp32 += st->gain_tx2[ch];
				vbi_fm_dsp_write(st, ADDR_GAIN01_1(block_nb), temp32);
				break;
			case 1:
				temp32 = vbi_fm_dsp_read(st, ADDR_GAIN01_1(block_nb)) & 0xFFFF;
				if(st->vbi_mode_tx2[ch])
					temp32 += st->modulator_gain_tx2[ch] << 16;
				else
					temp32 += st->gain_tx2[ch] << 16;
				vbi_fm_dsp_write(st, ADDR_GAIN01_1(block_nb), temp32);
				break;
			case 2:
				temp32 = vbi_fm_dsp_read(st, ADDR_GAIN23_1(block_nb)) & 0xFFFF0000;
				if(st->vbi_mode_tx2[ch])
					temp32 += st->modulator_gain_tx2[ch];
				else
					temp32 += st->gain_tx2[ch];
				vbi_fm_dsp_write(st, ADDR_GAIN23_1(block_nb), temp32);
				break;
			case 3:
				temp32 = vbi_fm_dsp_read(st, ADDR_GAIN23_1(block_nb)) & 0xFFFF;
				if(st->vbi_mode_tx2[ch])
					temp32 += st->modulator_gain_tx2[ch] << 16;
				else
					temp32 += st->gain_tx2[ch] << 16;
				vbi_fm_dsp_write(st, ADDR_GAIN23_1(block_nb), temp32);
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
			temp32 = vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & ~(1<<(0+subchannel));
			temp32 += (u32)val << (0+subchannel);
			vbi_fm_dsp_write(st, ADDR_ROUTING(block_nb), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_AUDIO_SELECTION)){
			match = 1;
			if(val<1 || val>2){
				ret = -EINVAL;
				break;
			}
			val -= 1;
			temp32 = vbi_fm_dsp_read(st, ADDR_AUDIO_SEL) & ~(1<<ch);
			temp32 += (u32)val << ch;
			vbi_fm_dsp_write(st, ADDR_AUDIO_SEL, temp32);
			break;
		}
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		return ret ? ret : len;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_DMA_SOURCE:
		if(val<0 || val>2){
			ret = -EINVAL;
			break;
		}
		if(val>0)
			val += 1;
		temp32 = vbi_fm_dsp_read(st, ADDR_DMA_SOURCE_CHANNEL) & ~(0x3<<5);
		temp32 += (u32)val << 5;
		vbi_fm_dsp_write(st, ADDR_DMA_SOURCE_CHANNEL, temp32);
		break;
	case REG_DMA_SOURCE_CHANNEL:
		if(val<0 || val>31){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_fm_dsp_read(st, ADDR_DMA_SOURCE_CHANNEL) & ~(0x1f<<0);
		temp32 += (u32)val << 0;
		vbi_fm_dsp_write(st, ADDR_DMA_SOURCE_CHANNEL, temp32);
		break;
	case REG_DMA_SINK:
		if(val<0 || val>2){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_fm_dsp_read(st, ADDR_DMA_SINK_WATCHDOG) & ~(0x3<<0);
		temp32 += (u32)val << 0;
		vbi_fm_dsp_write(st, ADDR_DMA_SINK_WATCHDOG, (u32)val);
		break;
	case REG_DMA_BURST_PERIOD:
		vbi_fm_dsp_write(st, ADDR_DMA_BURST_PERIOD, (u32)val);
		break;
	case REG_DMA_BURST_LENGTH:
		vbi_fm_dsp_write(st, ADDR_DMA_BURST_LENGTH, (u32)val);
		break;
	case REG_DMA_DECIMATION:
		if(val<40 || val>16383){
			ret = -EINVAL;
			break;
		}
		vbi_fm_dsp_write(st, ADDR_DMA_DECIMATION, (u32)val);
		break;
	case REG_WATCHDOG_ENABLE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_fm_dsp_read(st, ADDR_DMA_SINK_WATCHDOG) & ~(0x1<<2);
		temp32 += (u32)val << 2;
		vbi_fm_dsp_write(st, ADDR_DMA_SINK_WATCHDOG, temp32);
		break;
	case REG_WATCHDOG_TRIGGER:
		temp32 = vbi_fm_dsp_read(st, ADDR_DMA_SINK_WATCHDOG) & ~(0x1<<3);
		temp32 += 1 << 3;
		vbi_fm_dsp_write(st, ADDR_DMA_SINK_WATCHDOG, temp32);
		temp32 &= ~(1 << 3);
		vbi_fm_dsp_write(st, ADDR_DMA_SINK_WATCHDOG, temp32);
		break;
	case REG_GAIN_AUDIO:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_fm_dsp_read(st, ADDR_GAIN_MOD) & 0xFFFF0000;
		temp32 += (u32)val;
		vbi_fm_dsp_write(st, ADDR_GAIN_MOD, temp32);
		break;
	case REG_GAIN_RDS:
		if(val<MIN_GAIN || val>MAX_GAIN){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_fm_dsp_read(st, ADDR_GAIN_MOD) & 0xFFFF;
		temp32 += (u32)val << 16;
		vbi_fm_dsp_write(st, ADDR_GAIN_MOD, temp32);
		break;
	case REG_AUDIO_MIX2MONO:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_fm_dsp_read(st, ADDR_MONO_SWP_SOURCE);
		temp32 = (temp32 & 0xFFFFFFFE) | val;
		vbi_fm_dsp_write(st, ADDR_MONO_SWP_SOURCE, temp32);
		break;
	case REG_AUDIO_SWAP:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_fm_dsp_read(st, ADDR_MONO_SWP_SOURCE);
		temp32 = (temp32 & 0xFFFFFFFD) | (val << 1);
		vbi_fm_dsp_write(st, ADDR_MONO_SWP_SOURCE, temp32);
		break;
	case REG_AUDIO_FROM_INPUT:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = vbi_fm_dsp_read(st, ADDR_MONO_SWP_SOURCE);
		temp32 = (temp32 & 0xFFFFFFFB) | (val << 2);
		vbi_fm_dsp_write(st, ADDR_MONO_SWP_SOURCE, temp32);
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
		temp32 = vbi_fm_dsp_read(st, ADDR_PPS_SETTINGS) & ~(1<<29);
		temp32 += (u32)val<<29;
		vbi_fm_dsp_write(st, ADDR_PPS_SETTINGS, temp32);
		break;
	case REG_PPS_REFERENCE_FREQUENCY:
		temp32 = vbi_fm_dsp_read(st, ADDR_PPS_SETTINGS) & ~(0x1FFFFFFF);
		temp32 += (u32)val & 0x1FFFFFFF;
		vbi_fm_dsp_write(st, ADDR_PPS_SETTINGS, temp32);
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

static ssize_t vbi_fm_dsp_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct vbi_fm_dsp_state *st = iio_priv(indio_dev);
	u32 val;
	int ret = 0;
	int tempint;
	u64 temp64;
	u32 subchannel;
	u32 block_nb;
	u32 ch;
	int match;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<32; ch++){
		subchannel = (ch & 0x3);
		block_nb = (ch >> 2);
		if((u32)this_attr->address == REG_CH(ch, REG_GAIN_TX1)){
			match = 1;
			val = st->gain_tx1[ch];
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_GAIN_TX2)){
			match = 1;
			val = st->gain_tx2[ch];
			break;
		}
		if((u32)this_attr->address == REG_CH(ch, REG_MODULATOR_GAIN_TX1)){
			match = 1;
			val = st->modulator_gain_tx1[ch];
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_MODULATOR_GAIN_TX2)){
			match = 1;
			val = st->modulator_gain_tx2[ch];
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FREQUENCY)){
			match = 1;
			switch (subchannel){
			case 0:
				val = vbi_fm_dsp_read(st, ADDR_INC01(block_nb)) & 0xFFFF;
				break;
			case 1:
				val = (vbi_fm_dsp_read(st, ADDR_INC01(block_nb)) & 0xFFFF0000) >> 16;
				break;
			case 2:
				val = vbi_fm_dsp_read(st, ADDR_INC23(block_nb)) & 0xFFFF;
				break;
			case 3:
				val = (vbi_fm_dsp_read(st, ADDR_INC23(block_nb)) & 0xFFFF0000) >> 16;
				break;
			default:
				break;
			}
			tempint = (int)val;
			if(tempint>=1<<15)
				tempint -= 1<<16;
			tempint += (11*(1<<18))>>4;
			temp64 = (u64)tempint * st->fs_adc/2;
			val = (u32)(temp64 >> 18);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_SWAP_TX1)){
			match = 1;
			switch (subchannel){
			case 0:
			case 1:
				val = (vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & (1<<4)) >> 4;
				break;
			case 2:
			case 3:
				val = (vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & (1<<5)) >> 5;
				break;
			default:
				break;
			}
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_SWAP_TX2)){
			match = 1;
			switch (subchannel){
			case 0:
			case 1:
				val = (vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & (1<<14)) >> 14;
				break;
			case 2:
			case 3:
				val = (vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & (1<<15)) >> 15;
				break;
			default:
				break;
			}
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DDS_MODE_TX1)){
			match = 1;
			val = (vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & (1<<(6+subchannel))) >> (6+subchannel);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DDS_MODE_TX2)){
			match = 1;
			val = (vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & (1<<(16+subchannel))) >> (16+subchannel);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_VBI_MODE_TX1)){
			match = 1;
			val = (vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & (1<<(10+subchannel))) >> (10+subchannel);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_VBI_MODE_TX2)){
			match = 1;
			val = (vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & (1<<(20+subchannel))) >> (20+subchannel);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RF_INPUT_SELECTION)){
			match = 1;
			val = (vbi_fm_dsp_read(st, ADDR_ROUTING(block_nb)) & (1<<(0+subchannel))) >> (0+subchannel);
			val += 1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_AUDIO_SELECTION)){
			match = 1;
			val = (vbi_fm_dsp_read(st, ADDR_AUDIO_SEL) & (1<<ch)) >> ch;
			val += 1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RSSI)){
			match = 1;
			switch (subchannel){
			case 0:
				val = vbi_fm_dsp_read(st, ADDR_RSSI01(block_nb)) & 0xFFFF;
				break;
			case 1:
				val = (vbi_fm_dsp_read(st, ADDR_RSSI01(block_nb)) & 0xFFFF0000) >> 16;
				break;
			case 2:
				val = vbi_fm_dsp_read(st, ADDR_RSSI23(block_nb)) & 0xFFFF;
				break;
			case 3:
				val = (vbi_fm_dsp_read(st, ADDR_RSSI23(block_nb)) & 0xFFFF0000) >> 16;
				break;
			default:
				break;
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
	case REG_DSP_VERSION:
		val = vbi_fm_dsp_read(st, ADDR_DSP_VERSION);
		break;
	case REG_RX_SAMP_RATE_ADC:
		val = st->fs_adc;
		break;
	case REG_ADC1_PEAK_HOLD_VAL:
		val = (vbi_fm_dsp_read(st, ADDR_ADC_PEAK) & 0xFFFF);
		break;
	case REG_ADC2_PEAK_HOLD_VAL:
		val = (vbi_fm_dsp_read(st, ADDR_ADC_PEAK) >> 16);
		break;
	case REG_ADC_BER_ALTERNATE:
		val = (vbi_fm_dsp_read(st, ADDR_ADC_BER_TESTER) & 0xFFFF);
		break;
	case REG_ADC_BER_CHECKER:
		val = (vbi_fm_dsp_read(st, ADDR_ADC_BER_TESTER) >> 16);
		break;
	case REG_DMA_SOURCE:
		val = (vbi_fm_dsp_read(st, ADDR_DMA_SOURCE_CHANNEL) >> 5) & 0x3;
		if(val>1)
			val -= 1;
		break;
	case REG_DMA_SOURCE_CHANNEL:
		val = vbi_fm_dsp_read(st, ADDR_DMA_SOURCE_CHANNEL) & 0x1f;
		break;
	case REG_DMA_SINK:
		val = vbi_fm_dsp_read(st, ADDR_DMA_SINK_WATCHDOG) & 0x3;
		break;
	case REG_DMA_BURST_PERIOD:
		val = vbi_fm_dsp_read(st, ADDR_DMA_BURST_PERIOD);
		break;
	case REG_DMA_BURST_LENGTH:
		val = vbi_fm_dsp_read(st, ADDR_DMA_BURST_LENGTH);
		break;
	case REG_DMA_DECIMATION:
		val = vbi_fm_dsp_read(st, ADDR_DMA_DECIMATION);
		break;
	case REG_WATCHDOG_ENABLE:
		val = (vbi_fm_dsp_read(st, ADDR_DMA_SINK_WATCHDOG) >> 2) & 0x1;
		break;
	case REG_WATCHDOG_TRIGGER:
		val = (vbi_fm_dsp_read(st, ADDR_DMA_SINK_WATCHDOG) >> 3) & 0x1;
		break;
	case REG_GAIN_AUDIO:
		val = vbi_fm_dsp_read(st, ADDR_GAIN_MOD) & 0xFFFF;
		break;
	case REG_GAIN_RDS:
		val = (vbi_fm_dsp_read(st, ADDR_GAIN_MOD) & 0xFFFF0000) >> 16;
		break;
	case REG_METER_AUDIO1:
		val = vbi_fm_dsp_read(st, ADDR_AUDIO_METER) & 0xFFFF;
		break;
	case REG_METER_AUDIO2:
		val = (vbi_fm_dsp_read(st, ADDR_AUDIO_METER) & 0xFFFF0000) >> 16;
		break;
	case REG_AUDIO_MIX2MONO:
		val = vbi_fm_dsp_read(st, ADDR_MONO_SWP_SOURCE) & 0x0001;
		break;
	case REG_AUDIO_SWAP:
		val = (vbi_fm_dsp_read(st, ADDR_MONO_SWP_SOURCE) >> 1) & 0x0001;
		break;
	case REG_AUDIO_FROM_INPUT:
		val = (vbi_fm_dsp_read(st, ADDR_MONO_SWP_SOURCE) >> 2) & 0x0001;
		break;
	case REG_PPS_DIRECTION_OUT_N_IN:
		val = (vbi_fm_dsp_read(st, ADDR_PPS_SETTINGS) >>29) & 1;
		break;
	case REG_PPS_CLK_ERROR:
		val = vbi_fm_dsp_read(st, ADDR_PPS_CLKS) & 0x1FFFFFFF;
		break;
	case REG_PPS_CLK_ERROR_HZ:
		val = st->pps_clk_error_hz;
		break;
	case REG_PPS_CLK_ERROR_NS:
		val = st->pps_clk_error_ns;
		break;
	case REG_PPS_REFERENCE_FREQUENCY:
		val = vbi_fm_dsp_read(st, ADDR_PPS_SETTINGS) & 0x1FFFFFFF;
		break;
	case REG_PPS_CNT:
		val = vbi_fm_dsp_read(st, ADDR_PPS_CNT);
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

IIO_DEVICE_ATTR_ALL_CH(gain_tx1, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_GAIN_TX1);

IIO_DEVICE_ATTR_ALL_CH(gain_tx2, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_GAIN_TX2);

IIO_DEVICE_ATTR_ALL_CH(modulator_gain_tx1, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_MODULATOR_GAIN_TX1);

IIO_DEVICE_ATTR_ALL_CH(modulator_gain_tx2, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_MODULATOR_GAIN_TX2);

IIO_DEVICE_ATTR_ALL_CH(frequency, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_FREQUENCY);

IIO_DEVICE_ATTR_ALL_CH(swap_tx1, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_SWAP_TX1);

IIO_DEVICE_ATTR_ALL_CH(swap_tx2, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_SWAP_TX2);

IIO_DEVICE_ATTR_ALL_CH(dds_mode_tx1, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_DDS_MODE_TX1);

IIO_DEVICE_ATTR_ALL_CH(dds_mode_tx2, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_DDS_MODE_TX2);

IIO_DEVICE_ATTR_ALL_CH(vbi_mode_tx1, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_VBI_MODE_TX1);

IIO_DEVICE_ATTR_ALL_CH(vbi_mode_tx2, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_VBI_MODE_TX2);

IIO_DEVICE_ATTR_ALL_CH(rf_input_selection, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_RF_INPUT_SELECTION);

IIO_DEVICE_ATTR_ALL_CH(audio_selection, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_AUDIO_SELECTION);

IIO_DEVICE_ATTR_ALL_CH(rssi, S_IRUGO,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_RSSI);

static IIO_DEVICE_ATTR(dsp_version, S_IRUGO,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_DSP_VERSION);

static IIO_DEVICE_ATTR(adc1_peak_hold_value, S_IRUGO,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_ADC1_PEAK_HOLD_VAL);

static IIO_DEVICE_ATTR(adc2_peak_hold_value, S_IRUGO,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_ADC2_PEAK_HOLD_VAL);

static IIO_DEVICE_ATTR(adc_ber_alternate, S_IRUGO,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_ADC_BER_ALTERNATE);

static IIO_DEVICE_ATTR(adc_ber_checker, S_IRUGO,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_ADC_BER_CHECKER);

static IIO_DEVICE_ATTR(dma_source, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_DMA_SOURCE);

static IIO_DEVICE_ATTR(dma_source_channel, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_DMA_SOURCE_CHANNEL);

static IIO_DEVICE_ATTR(dma_sink, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_DMA_SINK);

static IIO_DEVICE_ATTR(dma_burst_period, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_DMA_BURST_PERIOD);

static IIO_DEVICE_ATTR(dma_burst_length, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_DMA_BURST_LENGTH);

static IIO_DEVICE_ATTR(dma_decimation, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_DMA_DECIMATION);

static IIO_DEVICE_ATTR(watchdog_enable, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_WATCHDOG_ENABLE);

static IIO_DEVICE_ATTR(watchdog_trigger, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_WATCHDOG_TRIGGER);

static IIO_DEVICE_ATTR(gain_audio, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_GAIN_AUDIO);

static IIO_DEVICE_ATTR(gain_rds, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_GAIN_RDS);

static IIO_DEVICE_ATTR(meter_audio1, S_IRUGO,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_METER_AUDIO1);

static IIO_DEVICE_ATTR(meter_audio2, S_IRUGO,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_METER_AUDIO2);

static IIO_DEVICE_ATTR(audio_mix2mono, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_AUDIO_MIX2MONO);

static IIO_DEVICE_ATTR(audio_swap, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_AUDIO_SWAP);

static IIO_DEVICE_ATTR(audio_from_input, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_AUDIO_FROM_INPUT);

static IIO_DEVICE_ATTR(pps_direction_out_n_in, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_PPS_DIRECTION_OUT_N_IN);

static IIO_DEVICE_ATTR(pps_clk_error, S_IRUGO,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_PPS_CLK_ERROR);

static IIO_DEVICE_ATTR(pps_clk_error_ns, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_PPS_CLK_ERROR_NS);

static IIO_DEVICE_ATTR(pps_clk_error_hz, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_PPS_CLK_ERROR_HZ);

static IIO_DEVICE_ATTR(pps_reference_frequency, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_PPS_REFERENCE_FREQUENCY);

static IIO_DEVICE_ATTR(gpsdo_locked, S_IRUGO | S_IWUSR,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_GPSDO_LOCKED);

static IIO_DEVICE_ATTR(pps_cnt, S_IRUGO,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_PPS_CNT);

static IIO_DEVICE_ATTR(rx_samp_rate_adc, S_IRUGO,
			vbi_fm_dsp_show,
			vbi_fm_dsp_store,
			REG_RX_SAMP_RATE_ADC);


static struct attribute *vbi_fm_dsp_attributes[] = {
	IIO_ATTR_ALL_CH(gain_tx1),
	IIO_ATTR_ALL_CH(gain_tx2),
	IIO_ATTR_ALL_CH(modulator_gain_tx1),
	IIO_ATTR_ALL_CH(modulator_gain_tx2),
	IIO_ATTR_ALL_CH(frequency),
	IIO_ATTR_ALL_CH(swap_tx1),
	IIO_ATTR_ALL_CH(swap_tx2),
	IIO_ATTR_ALL_CH(dds_mode_tx1),
	IIO_ATTR_ALL_CH(dds_mode_tx2),
	IIO_ATTR_ALL_CH(vbi_mode_tx1),
	IIO_ATTR_ALL_CH(vbi_mode_tx2),
	IIO_ATTR_ALL_CH(rf_input_selection),
	IIO_ATTR_ALL_CH(audio_selection),
	IIO_ATTR_ALL_CH(rssi),
	&iio_dev_attr_dsp_version.dev_attr.attr,
	&iio_dev_attr_adc1_peak_hold_value.dev_attr.attr,
	&iio_dev_attr_adc2_peak_hold_value.dev_attr.attr,
	&iio_dev_attr_adc_ber_alternate.dev_attr.attr,
	&iio_dev_attr_adc_ber_checker.dev_attr.attr,
	&iio_dev_attr_dma_source.dev_attr.attr,
	&iio_dev_attr_dma_source_channel.dev_attr.attr,
	&iio_dev_attr_dma_sink.dev_attr.attr,
	&iio_dev_attr_dma_burst_period.dev_attr.attr,
	&iio_dev_attr_dma_burst_length.dev_attr.attr,
	&iio_dev_attr_dma_decimation.dev_attr.attr,
	&iio_dev_attr_watchdog_enable.dev_attr.attr,
	&iio_dev_attr_watchdog_trigger.dev_attr.attr,
	&iio_dev_attr_gain_audio.dev_attr.attr,
	&iio_dev_attr_gain_rds.dev_attr.attr,
	&iio_dev_attr_meter_audio1.dev_attr.attr,
	&iio_dev_attr_meter_audio2.dev_attr.attr,
	&iio_dev_attr_audio_mix2mono.dev_attr.attr,
	&iio_dev_attr_audio_swap.dev_attr.attr,
	&iio_dev_attr_audio_from_input.dev_attr.attr,
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


static const struct attribute_group vbi_fm_dsp_attribute_group = {
	.attrs = vbi_fm_dsp_attributes,
};

static const struct iio_info vbi_fm_dsp_info = {
	.read_raw = &vbi_fm_dsp_read_raw,
	.write_raw = &vbi_fm_dsp_write_raw,
	.attrs = &vbi_fm_dsp_attribute_group,
};

static const struct iio_chan_spec vbi_fm_dsp_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id vbi_fm_dsp_of_match[] = {
	{ .compatible = "fpga,vbi-fm-dsp", },
	{ },
};

MODULE_DEVICE_TABLE(of, vbi_fm_dsp_of_match);

static int vbi_fm_dsp_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct vbi_fm_dsp_state *st;
	struct iio_dev *indio_dev;
	int ret; //, i, n;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(vbi_fm_dsp_of_match, &pdev->dev);
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
	indio_dev->channels = vbi_fm_dsp_channels;
	indio_dev->num_channels = ARRAY_SIZE(vbi_fm_dsp_channels);
	indio_dev->info = &vbi_fm_dsp_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	vbi_fm_dsp_write(st, ADDR_DMA_DECIMATION, 360);
	vbi_fm_dsp_write(st, ADDR_DMA_BURST_PERIOD, 16384);
	vbi_fm_dsp_write(st, ADDR_DMA_BURST_LENGTH, 2048);
	vbi_fm_dsp_write(st, ADDR_GAIN_MOD, 0x000a06a4);
	vbi_fm_dsp_write(st, ADDR_MOD_CH_SHIFT, 0x2c);
	vbi_fm_dsp_write(st, ADDR_MONO_SWP_SOURCE, 0x5);
	// MONO_SWP_SOURCE: bit0 0: dual channel, 1: ch0=in0+in1; bit1 0: normal, 1: swap channels; bit2 0: audio from codec output, 1: audio from codec input


	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, indio_dev);
	return 0;

err_iio_device_free:
	iio_device_free(indio_dev);
	return ret;
}

static int vbi_fm_dsp_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver vbi_fm_dsp_driver = {
	.probe		= vbi_fm_dsp_probe,
	.remove		= vbi_fm_dsp_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vbi_fm_dsp_of_match,
	},
};

module_platform_driver(vbi_fm_dsp_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("FM Voice Break-In (VBI) FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
