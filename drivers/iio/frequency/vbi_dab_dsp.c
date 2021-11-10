/*
 * VBI dab DSP COREFPGA Module
 * dab Voice Break In DSP
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


#define DRIVER_NAME			"vbi-dab-dsp"
#define NB_OF_BLOCKS			12 
#define ADDR_PER_BLOCK			4*16
#define ADDR_START_BLOCK		4*16
#define ADDR_START_MONITOR		ADDR_START_BLOCK+ADDR_PER_BLOCK*NB_OF_BLOCKS


#define ADDR_VERSION			4*0
#define ADDR_ADC_PEAK			4*1
//#define ADDR_DEMOD_PHASE_ERR		4*2
//#define ADDR_DEMOD_SPWR			4*3
//#define ADDR_DEMOD_NPWR			4*4
#define ADDR_PPS_CLKS			4*5
#define ADDR_PPS_CNT			4*6
#define ADDR_ADC_BER_TESTER		4*8
#define ADDR_DEMOD_SETTINGS		4*11
#define ADDR_PPS_SETTINGS		4*12

#define ADDR_BLOCK_SETTINGS(x)		(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*0)
#define ADDR_BLOCK_DEMOD_GAIN(x)	(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*1)
#define ADDR_BLOCK_MOD_OUT_GAIN(x)	(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*2)
#define ADDR_BLOCK_BYP_GAIN(x)		(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*3)
#define ADDR_BLOCK_DDS_GAIN(x)		(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*4)
#define ADDR_BLOCK_DDS_INC(x)		(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*5)
#define ADDR_BLOCK_FREQ_SYNC_DELAY(x)	(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*6)
#define ADDR_BLOCK_MOD_STARTTDELAY(x)	(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*7)
#define ADDR_BLOCK_STREAM_LENGTH(x)	(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*8)
#define ADDR_BLOCK_STREAM_PERIOD(x)	(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*9)
#define ADDR_BLOCK_PHASE_ERR(x)		(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*10)
#define ADDR_BLOCK_RSSI(x)		(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*11)
#define ADDR_BLOCK_FRAMESYNC_ERR(x)	(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*12)
#define ADDR_BLOCK_MOD_FRAMES_SINCE_REQ(x)	(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*13)
#define ADDR_BLOCK_DEMOD_FRAMES_SINCE_SOT(x)	(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*14)
#define ADDR_BLOCK_UNDERRUN_FRAMES_SINCE_RST(x)	(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*15)
//#define ADDR_BLOCK_GUARD_OFFSET(x)	(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*14)
//#define ADDR_BLOCK_MON_BURST_LENGTH(x)	(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*13)
//#define ADDR_BLOCK_MON_BURST_PERIOD(x)	(ADDR_START_BLOCK+x*ADDR_PER_BLOCK+4*14)

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
	CH11_##REG
	// CH12_##REG
	// CH13_##REG, \
	// CH14_##REG, \
	// CH15_##REG, \
	// CH16_##REG, \
	// CH17_##REG, \
	// CH18_##REG, \
	// CH19_##REG, \
	// CH20_##REG, \
	// CH21_##REG, \
	// CH22_##REG, \
	// CH23_##REG, \
	// CH24_##REG, \
	// CH25_##REG, \
	// CH26_##REG, \
	// CH27_##REG, \
	// CH28_##REG, \
	// CH29_##REG, \
	// CH30_##REG, \
	// CH31_##REG

// expands to:
//   static IIO_DEVICE_ATTR(ch0_<ATTR>, <RW>, <SHOW>, <STORE>, CH0_<REG>);
//   static IIO_DEVICE_ATTR(ch1_<ATTR>, <RW>, <SHOW>, <STORE>, CH1_<REG>);
//    ::   ::
//   static IIO_DEVICE_ATTR(ch31_<ATTR>, <RW>, <SHOW>, <STORE>, CH31_<REG>);
// example:
//   IIO_DEVICE_ATTR_ALL_CH(gain_tx1, S_IRUGO | S_IWUSR, vbi_dab_dsp_show, vbi_dab_dsp_store, REG_GAIN_TX1)
//     expansion:
//     static IIO_DEVICE_ATTR(ch0_gain_tx1, S_IRUGO | S_IWUSR, vbi_dab_dsp_show, vbi_dab_dsp_store, CH0_REG_GAIN_TX1);
//     static IIO_DEVICE_ATTR(ch1_gain_tx1, S_IRUGO | S_IWUSR, vbi_dab_dsp_show, vbi_dab_dsp_store, CH1_REG_GAIN_TX1);
//      ::   ::
//     static IIO_DEVICE_ATTR(ch31_gain_tx1, S_IRUGO | S_IWUSR, vbi_dab_dsp_show, vbi_dab_dsp_store, CH31_REG_GAIN_TX1);
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
	static IIO_DEVICE_ATTR(ch11_##ATTR, RW, SHOW, STORE, CH11_##REG);
	//static IIO_DEVICE_ATTR(ch12_##ATTR, RW, SHOW, STORE, CH12_##REG); 
	// static IIO_DEVICE_ATTR(ch13_##ATTR, RW, SHOW, STORE, CH13_##REG); \
	// static IIO_DEVICE_ATTR(ch14_##ATTR, RW, SHOW, STORE, CH14_##REG); \
	// static IIO_DEVICE_ATTR(ch15_##ATTR, RW, SHOW, STORE, CH15_##REG); \
	// static IIO_DEVICE_ATTR(ch16_##ATTR, RW, SHOW, STORE, CH16_##REG); \
	// static IIO_DEVICE_ATTR(ch17_##ATTR, RW, SHOW, STORE, CH17_##REG); \
	// static IIO_DEVICE_ATTR(ch18_##ATTR, RW, SHOW, STORE, CH18_##REG); \
	// static IIO_DEVICE_ATTR(ch19_##ATTR, RW, SHOW, STORE, CH19_##REG); \
	// static IIO_DEVICE_ATTR(ch20_##ATTR, RW, SHOW, STORE, CH20_##REG); \
	// static IIO_DEVICE_ATTR(ch21_##ATTR, RW, SHOW, STORE, CH21_##REG); \
	// static IIO_DEVICE_ATTR(ch22_##ATTR, RW, SHOW, STORE, CH22_##REG); \
	// static IIO_DEVICE_ATTR(ch23_##ATTR, RW, SHOW, STORE, CH23_##REG); \
	// static IIO_DEVICE_ATTR(ch24_##ATTR, RW, SHOW, STORE, CH24_##REG); \
	// static IIO_DEVICE_ATTR(ch25_##ATTR, RW, SHOW, STORE, CH25_##REG); \
	// static IIO_DEVICE_ATTR(ch26_##ATTR, RW, SHOW, STORE, CH26_##REG); \
	// static IIO_DEVICE_ATTR(ch27_##ATTR, RW, SHOW, STORE, CH27_##REG); \
	// static IIO_DEVICE_ATTR(ch28_##ATTR, RW, SHOW, STORE, CH28_##REG); \
	// static IIO_DEVICE_ATTR(ch29_##ATTR, RW, SHOW, STORE, CH29_##REG); \
	// static IIO_DEVICE_ATTR(ch30_##ATTR, RW, SHOW, STORE, CH30_##REG); \
	// static IIO_DEVICE_ATTR(ch31_##ATTR, RW, SHOW, STORE, CH31_##REG);

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
	&iio_dev_attr_ch11_##ATTR.dev_attr.attr
	// &iio_dev_attr_ch12_##ATTR.dev_attr.attr
	// &iio_dev_attr_ch13_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch14_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch15_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch16_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch17_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch18_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch19_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch20_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch21_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch22_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch23_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch24_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch25_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch26_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch27_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch28_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch29_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch30_##ATTR.dev_attr.attr, \
	// &iio_dev_attr_ch31_##ATTR.dev_attr.attr

enum chan_num{
	REG_ALL_CH(REG_DEMODULATOR_GAIN),	// being expanded for all channels
	REG_ALL_CH(REG_MODULATOR_GAIN_TX1),	// being expanded for all channels
	REG_ALL_CH(REG_MODULATOR_GAIN_TX2),	// being expanded for all channels
	REG_ALL_CH(REG_BYPASS_GAIN_TX1),	// being expanded for all channels
	REG_ALL_CH(REG_BYPASS_GAIN_TX2),	// being expanded for all channels
	REG_ALL_CH(REG_FREQUENCY),	// being expanded for all channels
	REG_ALL_CH(REG_FREQUENCY_TARGET),	// being expanded for all channels
	REG_ALL_CH(REG_DISABLE_FREQ_CORRECTION),	// being expanded for all channels
	REG_ALL_CH(REG_DDS_GAIN_TX1),	// being expanded for all channels
	REG_ALL_CH(REG_DDS_GAIN_TX2),	// being expanded for all channels
	REG_ALL_CH(REG_VBI_MODE_TX1),	// being expanded for all channels
	REG_ALL_CH(REG_VBI_MODE_TX2),	// being expanded for all channels
	REG_ALL_CH(REG_RF_INPUT_SELECTION),	// being expanded for all channels
	REG_ALL_CH(REG_RSSI),	// being expanded for all channels
	REG_ALL_CH(REG_PHASE_ERROR),	// being expanded for all channels
	//REG_ALL_CH(REG_PHASE_ERROR_I),	// being expanded for all channels
	//REG_ALL_CH(REG_PHASE_ERROR_Q),	// being expanded for all channels
	REG_ALL_CH(REG_FRAMESYNC_ERROR),	// being expanded for all channels
	REG_ALL_CH(REG_FRAMESYNC_CORRECTION),	// being expanded for all channels
	REG_ALL_CH(REG_FRAMESYNC_RESET),	// being expanded for all channels
	REG_ALL_CH(REG_BLOCK_RESET),	// being expanded for all channels
	REG_ALL_CH(REG_MOD_START_DELAY),	// being expanded for all channels
	REG_ALL_CH(REG_FREQ_SYNC_DELAY),	// being expanded for all channels
	REG_ALL_CH(REG_GUARD_OFFSET),	// being expanded for all channels
	REG_ALL_CH(REG_MOD_FRAMES_SINCE_REQ),	// being expanded for all channels
	REG_ALL_CH(REG_DEMOD_FRAMES_SINCE_SOT),	// being expanded for all channels
	REG_ALL_CH(REG_UNDERRUN_FRAMES_SINCE_RESET),	// being expanded for all channels
	REG_ALL_CH(REG_STREAM_PERIOD),	// being expanded for all channels
	REG_ALL_CH(REG_STREAM_LENGTH),	// being expanded for all channels
	REG_ALL_CH(REG_STREAM_ENABLE_BURST_MODE),
	REG_ALL_CH(REG_STREAM_DISABLE_SYNC),
	REG_ALL_CH(REG_STREAM_SOURCE),
	REG_VERSION,
	REG_ADC1_PEAK_HOLD_VAL,
	REG_ADC2_PEAK_HOLD_VAL,
	REG_ADC_BER_ALTERNATE,
	REG_ADC_BER_CHECKER,
	REG_DEMOD_SOURCE_CHANNEL,
//	REG_DEMOD_PHASE_ERR,
//	REG_DEMOD_SPWR,
//	REG_DEMOD_NPWR,
//	REG_PPS_SRC_INT_N_EXT,
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

struct vbi_dab_dsp_state {
	struct iio_info		iio_info;
	void __iomem		*regs;
	struct mutex		lock;

	uint32_t		fs_adc;
	bool			gpsdo_locked;
  	uint32_t		pps_clk_error_ns;
  	uint32_t		pps_clk_error_hz;
	uint32_t		nb_of_blocks_dt;
	uint32_t		frequency_target[NB_OF_BLOCKS];
	uint8_t			disable_freq_correction[NB_OF_BLOCKS];
};

static void vbi_dab_dsp_write(struct vbi_dab_dsp_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 vbi_dab_dsp_read(struct vbi_dab_dsp_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int vbi_dab_dsp_write_raw(struct iio_dev *indio_dev,
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

static int vbi_dab_dsp_read_raw(struct iio_dev *indio_dev,
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

static ssize_t vbi_dab_dsp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct vbi_dab_dsp_state *st = iio_priv(indio_dev);
	long val;
	int ret;
	uint64_t temp64;
	uint32_t temp32;
	uint32_t ch;
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
	for(ch=0; ch<st->nb_of_blocks_dt; ch++){
		if((u32)this_attr->address == REG_CH(ch, REG_DEMODULATOR_GAIN)){
			match = 1;
			if(val<MIN_GAIN || val>MAX_GAIN){
				ret = -EINVAL;
				break;
			}
			vbi_dab_dsp_write(st, ADDR_BLOCK_DEMOD_GAIN(ch), (u32)val);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_MODULATOR_GAIN_TX1)){
			match = 1;
			if(val<MIN_GAIN || val>MAX_GAIN){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_MOD_OUT_GAIN(ch)) & 0xFFFF0000;
			temp32 += (u32)val;
			vbi_dab_dsp_write(st, ADDR_BLOCK_MOD_OUT_GAIN(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_MODULATOR_GAIN_TX2)){
			match = 1;
			if(val<MIN_GAIN || val>MAX_GAIN){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_MOD_OUT_GAIN(ch)) & 0xFFFF;
			temp32 += (u32)val <<16;
			vbi_dab_dsp_write(st, ADDR_BLOCK_MOD_OUT_GAIN(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_BYPASS_GAIN_TX1)){
			match = 1;
			if(val<MIN_GAIN || val>MAX_GAIN){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_BYP_GAIN(ch)) & 0xFFFF0000;
			temp32 += (u32)val;
			vbi_dab_dsp_write(st, ADDR_BLOCK_BYP_GAIN(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_BYPASS_GAIN_TX2)){
			match = 1;
			if(val<MIN_GAIN || val>MAX_GAIN){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_BYP_GAIN(ch)) & 0xFFFF;
			temp32 += (u32)val <<16;
			vbi_dab_dsp_write(st, ADDR_BLOCK_BYP_GAIN(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FREQUENCY)){
			match = 1;
			if(val<(st->fs_adc>>1) || val>st->fs_adc){
				ret = -EINVAL;
				break;
			}
			val += st->fs_adc>>2;
			temp64 = (u64)val << 25;
			temp64 = div_s64(temp64,st->fs_adc);
			val = (int)temp64 & 0xFFFFFF;
			vbi_dab_dsp_write(st, ADDR_BLOCK_DDS_INC(ch), (u32)val);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FREQUENCY_TARGET)){
			match = 1;
			if(val<(st->fs_adc>>1) || val>st->fs_adc){
				ret = -EINVAL;
				break;
			}
			st->frequency_target[ch] = (u32)val;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DISABLE_FREQ_CORRECTION)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			st->disable_freq_correction[ch] = (u8)val;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DDS_GAIN_TX1)){
			match = 1;
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_DDS_GAIN(ch)) & 0xFFFF0000;
			temp32 += (u32)val;
			vbi_dab_dsp_write(st, ADDR_BLOCK_DDS_GAIN(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DDS_GAIN_TX2)){
			match = 1;
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_DDS_GAIN(ch)) & 0xFFFF;
			temp32 += (u32)val <<16;
			vbi_dab_dsp_write(st, ADDR_BLOCK_DDS_GAIN(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_VBI_MODE_TX1)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & ~(1<<12);
			temp32 += (u32)val << 12;
			vbi_dab_dsp_write(st, ADDR_BLOCK_SETTINGS(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_VBI_MODE_TX2)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & ~(1<<13);
			temp32 += (u32)val << 13;
			vbi_dab_dsp_write(st, ADDR_BLOCK_SETTINGS(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RF_INPUT_SELECTION)){
			match = 1;
			if(val<1 || val>2){
				ret = -EINVAL;
				break;
			}
			val -= 1;
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & ~(1<<15);
			temp32 += (u32)val << 15;
			vbi_dab_dsp_write(st, ADDR_BLOCK_SETTINGS(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FRAMESYNC_CORRECTION)){
			match = 1;
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & ~(0xFF);
			temp32 += (u32)val & 0xFF;
			vbi_dab_dsp_write(st, ADDR_BLOCK_SETTINGS(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FRAMESYNC_RESET)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & ~(1<<9);
			temp32 += (u32)val << 9;
			vbi_dab_dsp_write(st, ADDR_BLOCK_SETTINGS(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_BLOCK_RESET)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & ~(1<<14);
			temp32 += (u32)val << 14;
			vbi_dab_dsp_write(st, ADDR_BLOCK_SETTINGS(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_MOD_START_DELAY)){
			match = 1;
			vbi_dab_dsp_write(st, ADDR_BLOCK_MOD_STARTTDELAY(ch), (u32)val);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FREQ_SYNC_DELAY)){
			match = 1;
			val &= 0x7FFFFF;
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_FREQ_SYNC_DELAY(ch)) & ~0x7FFFFF;
			temp32 += (u32)val;
			vbi_dab_dsp_write(st, ADDR_BLOCK_FREQ_SYNC_DELAY(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_GUARD_OFFSET)){
			match = 1;
			if(val<0 || val>504){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_FREQ_SYNC_DELAY(ch)) & ~(0x1FF<<23);
			temp32 += (u32)val << 23;
			vbi_dab_dsp_write(st, ADDR_BLOCK_FREQ_SYNC_DELAY(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_STREAM_LENGTH)){
			match = 1;
			vbi_dab_dsp_write(st, ADDR_BLOCK_STREAM_LENGTH(ch), (u32)val);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_STREAM_PERIOD)){
			match = 1;
			vbi_dab_dsp_write(st, ADDR_BLOCK_STREAM_PERIOD(ch), (u32)val);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_STREAM_ENABLE_BURST_MODE)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & ~(1<<16);
			temp32 += (u32)val << 16;
			vbi_dab_dsp_write(st, ADDR_BLOCK_SETTINGS(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_STREAM_DISABLE_SYNC)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & ~(1<<17);
			temp32 += (u32)val << 17;
			vbi_dab_dsp_write(st, ADDR_BLOCK_SETTINGS(ch), temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_STREAM_SOURCE)){
			match = 1;
			if(val<0 || val>3){
				ret = -EINVAL;
				break;
			}
			temp32 = vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & ~(3<<18);
			temp32 += (u32)val << 18;
			vbi_dab_dsp_write(st, ADDR_BLOCK_SETTINGS(ch), temp32);
			break;
		}
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		return ret ? ret : len;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_DEMOD_SOURCE_CHANNEL:
		if(val<0 || val>(NB_OF_BLOCKS-1)){
			ret = -EINVAL;
			break;
		}
		vbi_dab_dsp_write(st, ADDR_DEMOD_SETTINGS, (u32)val);
		break;
//	case REG_PPS_SRC_INT_N_EXT:
//		if(val<0 || val>1){
//			ret = -EINVAL;
//			break;
//		}
//		temp32 = vbi_dab_dsp_read(st, ADDR_PPS_SETTINGS) & ~(0x1);
//		temp32 += (u32)val;
//		vbi_dab_dsp_write(st, ADDR_PPS_SETTINGS, temp32);
//		break;
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
		temp32 = vbi_dab_dsp_read(st, ADDR_PPS_SETTINGS) & ~(1<<29);
		temp32 += (u32)val<<29;
		vbi_dab_dsp_write(st, ADDR_PPS_SETTINGS, temp32);
		break;
	case REG_PPS_REFERENCE_FREQUENCY:
		temp32 = vbi_dab_dsp_read(st, ADDR_PPS_SETTINGS) & ~(0x1FFFFFFF);
		temp32 += (u32)val & 0x1FFFFFFF;
		vbi_dab_dsp_write(st, ADDR_PPS_SETTINGS, temp32);
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

static ssize_t vbi_dab_dsp_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct vbi_dab_dsp_state *st = iio_priv(indio_dev);
	int32_t val;
	int ret = 0;
	uint64_t temp64;
	uint32_t ch;
	int match;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<st->nb_of_blocks_dt; ch++){
		if((u32)this_attr->address == REG_CH(ch, REG_DEMODULATOR_GAIN)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_DEMOD_GAIN(ch)) & 0xFFFF;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_MODULATOR_GAIN_TX1)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_MOD_OUT_GAIN(ch)) & 0xFFFF;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_MODULATOR_GAIN_TX2)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_MOD_OUT_GAIN(ch)) >> 16;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_BYPASS_GAIN_TX1)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_BYP_GAIN(ch)) & 0xFFFF;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_BYPASS_GAIN_TX2)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_BYP_GAIN(ch)) >> 16;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FREQUENCY)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_DDS_INC(ch)) & 0xFFFFFF;
			if(val<=1<<23)
				val += 1<<24;
			temp64 = (u64)val * st->fs_adc;
			val = (u32)(temp64 >> 25);
			val += st->fs_adc>>2;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FREQUENCY_TARGET)){
			match = 1;
			val = st->frequency_target[ch];
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DISABLE_FREQ_CORRECTION)){
			match = 1;
			val = (u32)st->disable_freq_correction[ch];
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DDS_GAIN_TX1)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_DDS_GAIN(ch)) & 0xFFFF;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DDS_GAIN_TX2)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_DDS_GAIN(ch)) >> 16;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_VBI_MODE_TX1)){
			match = 1;
			val = (vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & (1<<12)) >> 12;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_VBI_MODE_TX2)){
			match = 1;
			val = (vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & (1<<13)) >> 13;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RF_INPUT_SELECTION)){
			match = 1;
			val = (vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & (1<<15)) >> 15;
			val += 1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_RSSI)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_RSSI(ch));
		}
/*		else if((u32)this_attr->address == REG_CH(ch, REG_PHASE_ERROR)){*/
/*			match = 1;*/
/*			val = (int32_t)((int16_t)vbi_dab_dsp_read(st, ADDR_BLOCK_PHASE_ERR(ch)));*/
/*		}*/
		else if((u32)this_attr->address == REG_CH(ch, REG_PHASE_ERROR)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_PHASE_ERR(ch));
		}
/*		else if((u32)this_attr->address == REG_CH(ch, REG_PHASE_ERROR_Q)){*/
/*			match = 1;*/
/*			val = (int32_t)((int16_t)(vbi_dab_dsp_read(st, ADDR_BLOCK_PHASE_ERR(ch)) & 0xFFFF));*/
/*		}*/
/*		else if((u32)this_attr->address == REG_CH(ch, REG_PHASE_ERROR_I)){*/
/*			match = 1;*/
/*			val = (int32_t)((int16_t)(vbi_dab_dsp_read(st, ADDR_BLOCK_PHASE_ERR(ch)) >> 16));*/
/*		}*/
		else if((u32)this_attr->address == REG_CH(ch, REG_FRAMESYNC_ERROR)){
			match = 1;
			val = (int32_t)((int16_t)vbi_dab_dsp_read(st, ADDR_BLOCK_FRAMESYNC_ERR(ch)));
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FRAMESYNC_CORRECTION)){
			match = 1;
			val = (int32_t)((int16_t)(vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & 0xff));
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FRAMESYNC_RESET)){
			match = 1;
			val = (vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & (1<<9)) >> 9;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_BLOCK_RESET)){
			match = 1;
			val = (vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & (1<<14)) >> 14;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_MOD_START_DELAY)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_MOD_STARTTDELAY(ch));
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FREQ_SYNC_DELAY)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_FREQ_SYNC_DELAY(ch)) & 0x7FFFFF;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_GUARD_OFFSET)){
			match = 1;
			val = (vbi_dab_dsp_read(st, ADDR_BLOCK_FREQ_SYNC_DELAY(ch))>>23) & 0x1FF;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_MOD_FRAMES_SINCE_REQ)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_MOD_FRAMES_SINCE_REQ(ch));
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_DEMOD_FRAMES_SINCE_SOT)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_DEMOD_FRAMES_SINCE_SOT(ch));
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_UNDERRUN_FRAMES_SINCE_RESET)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_UNDERRUN_FRAMES_SINCE_RST(ch));
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_STREAM_LENGTH)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_STREAM_LENGTH(ch));
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_STREAM_PERIOD)){
			match = 1;
			val = vbi_dab_dsp_read(st, ADDR_BLOCK_STREAM_PERIOD(ch));
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_STREAM_ENABLE_BURST_MODE)){
			match = 1;
			val = (vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & (1<<16)) >> 16;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_STREAM_DISABLE_SYNC)){
			match = 1;
			val = (vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & (1<<17)) >> 17;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_STREAM_SOURCE)){
			match = 1;
			val = (vbi_dab_dsp_read(st, ADDR_BLOCK_SETTINGS(ch)) & (3<<18)) >> 18;
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
	case REG_VERSION:
		val = vbi_dab_dsp_read(st, ADDR_VERSION);
		break;
	case REG_RX_SAMP_RATE_ADC:
		val = st->fs_adc;
		break;
	case REG_ADC1_PEAK_HOLD_VAL:
		val = (vbi_dab_dsp_read(st, ADDR_ADC_PEAK) & 0xFFFF);
		break;
	case REG_ADC2_PEAK_HOLD_VAL:
		val = (vbi_dab_dsp_read(st, ADDR_ADC_PEAK) >> 16);
		break;
	case REG_ADC_BER_ALTERNATE:
		val = (vbi_dab_dsp_read(st, ADDR_ADC_BER_TESTER) & 0xFFFF);
		break;
	case REG_ADC_BER_CHECKER:
		val = (vbi_dab_dsp_read(st, ADDR_ADC_BER_TESTER) >> 16);
		break;
	case REG_DEMOD_SOURCE_CHANNEL:
		val = vbi_dab_dsp_read(st, ADDR_DEMOD_SETTINGS) & 0xF;
		break;
//	case REG_DEMOD_PHASE_ERR:
//		val = (int32_t)((int16_t)vbi_dab_dsp_read(st, ADDR_DEMOD_PHASE_ERR));
//		break;
//	case REG_DEMOD_SPWR:
//		val = vbi_dab_dsp_read(st, ADDR_DEMOD_SPWR);
//		break;
//	case REG_DEMOD_NPWR:
//		val = vbi_dab_dsp_read(st, ADDR_DEMOD_NPWR);
//		break;
//	case REG_PPS_SRC_INT_N_EXT:
//		val = vbi_dab_dsp_read(st, ADDR_PPS_SETTINGS) & 0x1;
//		break;
	case REG_PPS_DIRECTION_OUT_N_IN:
		val = (vbi_dab_dsp_read(st, ADDR_PPS_SETTINGS) >>29) & 1;
		break;
	case REG_PPS_CLK_ERROR:
		val = vbi_dab_dsp_read(st, ADDR_PPS_CLKS) & 0x1FFFFFFF;
		break;
	case REG_PPS_CLK_ERROR_HZ:
		val = st->pps_clk_error_hz;
		break;
	case REG_PPS_CLK_ERROR_NS:
		val = st->pps_clk_error_ns;
		break;
	case REG_PPS_REFERENCE_FREQUENCY:
		val = vbi_dab_dsp_read(st, ADDR_PPS_SETTINGS) & 0x1FFFFFFF;
		break;
	case REG_PPS_CNT:
		val = vbi_dab_dsp_read(st, ADDR_PPS_CNT);
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

IIO_DEVICE_ATTR_ALL_CH(demodulator_gain, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_DEMODULATOR_GAIN);

IIO_DEVICE_ATTR_ALL_CH(modulator_gain_tx1, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_MODULATOR_GAIN_TX1);

IIO_DEVICE_ATTR_ALL_CH(modulator_gain_tx2, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_MODULATOR_GAIN_TX2);

IIO_DEVICE_ATTR_ALL_CH(bypass_gain_tx1, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_BYPASS_GAIN_TX1);

IIO_DEVICE_ATTR_ALL_CH(bypass_gain_tx2, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_BYPASS_GAIN_TX2);

IIO_DEVICE_ATTR_ALL_CH(frequency, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_FREQUENCY);

IIO_DEVICE_ATTR_ALL_CH(frequency_target, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_FREQUENCY_TARGET);

IIO_DEVICE_ATTR_ALL_CH(disable_frequency_correction, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_DISABLE_FREQ_CORRECTION);

IIO_DEVICE_ATTR_ALL_CH(dds_gain_tx1, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_DDS_GAIN_TX1);

IIO_DEVICE_ATTR_ALL_CH(dds_gain_tx2, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_DDS_GAIN_TX2);

IIO_DEVICE_ATTR_ALL_CH(vbi_mode_tx1, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_VBI_MODE_TX1);

IIO_DEVICE_ATTR_ALL_CH(vbi_mode_tx2, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_VBI_MODE_TX2);

IIO_DEVICE_ATTR_ALL_CH(rf_input_selection, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_RF_INPUT_SELECTION);

IIO_DEVICE_ATTR_ALL_CH(rssi, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_RSSI);

IIO_DEVICE_ATTR_ALL_CH(phase_error, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_PHASE_ERROR);

/*IIO_DEVICE_ATTR_ALL_CH(phase_error_i, S_IRUGO,*/
/*			vbi_dab_dsp_show,*/
/*			vbi_dab_dsp_store,*/
/*			REG_PHASE_ERROR_I);*/

/*IIO_DEVICE_ATTR_ALL_CH(phase_error_q, S_IRUGO,*/
/*			vbi_dab_dsp_show,*/
/*			vbi_dab_dsp_store,*/
/*			REG_PHASE_ERROR_Q);*/

IIO_DEVICE_ATTR_ALL_CH(framesync_error, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_FRAMESYNC_ERROR);

IIO_DEVICE_ATTR_ALL_CH(framesync_correction, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_FRAMESYNC_CORRECTION);

IIO_DEVICE_ATTR_ALL_CH(framesync_reset, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_FRAMESYNC_RESET);

IIO_DEVICE_ATTR_ALL_CH(block_reset, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_BLOCK_RESET);

IIO_DEVICE_ATTR_ALL_CH(mod_start_delay, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_MOD_START_DELAY);

IIO_DEVICE_ATTR_ALL_CH(freq_sync_delay, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_FREQ_SYNC_DELAY);

IIO_DEVICE_ATTR_ALL_CH(guard_offset, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_GUARD_OFFSET);

IIO_DEVICE_ATTR_ALL_CH(mod_frames_since_req, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_MOD_FRAMES_SINCE_REQ);

IIO_DEVICE_ATTR_ALL_CH(demod_frames_since_sot, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_DEMOD_FRAMES_SINCE_SOT);

IIO_DEVICE_ATTR_ALL_CH(underrun_frames_since_reset, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_UNDERRUN_FRAMES_SINCE_RESET);

IIO_DEVICE_ATTR_ALL_CH(stream_period, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_STREAM_PERIOD);

IIO_DEVICE_ATTR_ALL_CH(stream_length, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_STREAM_LENGTH);

IIO_DEVICE_ATTR_ALL_CH(stream_enable_burst_mode, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_STREAM_ENABLE_BURST_MODE);

IIO_DEVICE_ATTR_ALL_CH(stream_disable_sync, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_STREAM_DISABLE_SYNC);

IIO_DEVICE_ATTR_ALL_CH(stream_source, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_STREAM_SOURCE);

static IIO_DEVICE_ATTR(version, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_VERSION);

static IIO_DEVICE_ATTR(adc1_peak_hold_val, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_ADC1_PEAK_HOLD_VAL);

static IIO_DEVICE_ATTR(adc2_peak_hold_val, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_ADC2_PEAK_HOLD_VAL);

static IIO_DEVICE_ATTR(adc_ber_alternate, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_ADC_BER_ALTERNATE);

static IIO_DEVICE_ATTR(adc_ber_checker, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_ADC_BER_CHECKER);

static IIO_DEVICE_ATTR(demod_source_channel, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_DEMOD_SOURCE_CHANNEL);

//static IIO_DEVICE_ATTR(demod_phase_err, S_IRUGO,
//			vbi_dab_dsp_show,
//			vbi_dab_dsp_store,
//			REG_DEMOD_PHASE_ERR);

//static IIO_DEVICE_ATTR(demod_spwr, S_IRUGO,
//			vbi_dab_dsp_show,
//			vbi_dab_dsp_store,
//			REG_DEMOD_SPWR);

//static IIO_DEVICE_ATTR(demod_npwr, S_IRUGO,
//			vbi_dab_dsp_show,
//			vbi_dab_dsp_store,
//			REG_DEMOD_NPWR);

//static IIO_DEVICE_ATTR(pps_src_int_n_ext, S_IRUGO | S_IWUSR,
//			vbi_dab_dsp_show,
//			vbi_dab_dsp_store,
//			REG_PPS_SRC_INT_N_EXT);

static IIO_DEVICE_ATTR(pps_direction_out_n_in, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_PPS_DIRECTION_OUT_N_IN);

static IIO_DEVICE_ATTR(pps_clk_error, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_PPS_CLK_ERROR);

static IIO_DEVICE_ATTR(pps_clk_error_ns, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_PPS_CLK_ERROR_NS);

static IIO_DEVICE_ATTR(pps_clk_error_hz, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_PPS_CLK_ERROR_HZ);

static IIO_DEVICE_ATTR(pps_reference_frequency, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_PPS_REFERENCE_FREQUENCY);

static IIO_DEVICE_ATTR(gpsdo_locked, S_IRUGO | S_IWUSR,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_GPSDO_LOCKED);

static IIO_DEVICE_ATTR(pps_cnt, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_PPS_CNT);

static IIO_DEVICE_ATTR(rx_samp_rate_adc, S_IRUGO,
			vbi_dab_dsp_show,
			vbi_dab_dsp_store,
			REG_RX_SAMP_RATE_ADC);


static struct attribute *vbi_dab_dsp_attributes[] = {
	IIO_ATTR_ALL_CH(demodulator_gain),
	IIO_ATTR_ALL_CH(modulator_gain_tx1),
	IIO_ATTR_ALL_CH(modulator_gain_tx2),
	IIO_ATTR_ALL_CH(bypass_gain_tx1),
	IIO_ATTR_ALL_CH(bypass_gain_tx2),
	IIO_ATTR_ALL_CH(frequency),
	IIO_ATTR_ALL_CH(frequency_target),
	IIO_ATTR_ALL_CH(disable_frequency_correction),
	IIO_ATTR_ALL_CH(dds_gain_tx1),
	IIO_ATTR_ALL_CH(dds_gain_tx2),
	IIO_ATTR_ALL_CH(vbi_mode_tx1),
	IIO_ATTR_ALL_CH(vbi_mode_tx2),
	IIO_ATTR_ALL_CH(rf_input_selection),
	IIO_ATTR_ALL_CH(rssi),
	IIO_ATTR_ALL_CH(phase_error),
	//IIO_ATTR_ALL_CH(phase_error_i),
	//IIO_ATTR_ALL_CH(phase_error_q),
	IIO_ATTR_ALL_CH(framesync_error),
	IIO_ATTR_ALL_CH(framesync_correction),
	IIO_ATTR_ALL_CH(framesync_reset),
	IIO_ATTR_ALL_CH(block_reset),
	IIO_ATTR_ALL_CH(mod_start_delay),
	IIO_ATTR_ALL_CH(freq_sync_delay),
	IIO_ATTR_ALL_CH(guard_offset),
	IIO_ATTR_ALL_CH(mod_frames_since_req),
	IIO_ATTR_ALL_CH(demod_frames_since_sot),
	IIO_ATTR_ALL_CH(underrun_frames_since_reset),
	IIO_ATTR_ALL_CH(stream_period),
	IIO_ATTR_ALL_CH(stream_length),
	IIO_ATTR_ALL_CH(stream_enable_burst_mode),
	IIO_ATTR_ALL_CH(stream_disable_sync),
	IIO_ATTR_ALL_CH(stream_source),
	&iio_dev_attr_version.dev_attr.attr,
	&iio_dev_attr_adc1_peak_hold_val.dev_attr.attr,
	&iio_dev_attr_adc2_peak_hold_val.dev_attr.attr,
	&iio_dev_attr_adc_ber_alternate.dev_attr.attr,
	&iio_dev_attr_adc_ber_checker.dev_attr.attr,
	&iio_dev_attr_demod_source_channel.dev_attr.attr,
//	&iio_dev_attr_demod_phase_err.dev_attr.attr,
//	&iio_dev_attr_demod_spwr.dev_attr.attr,
//	&iio_dev_attr_demod_npwr.dev_attr.attr,
//	&iio_dev_attr_pps_src_int_n_ext.dev_attr.attr,
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


static const struct attribute_group vbi_dab_dsp_attribute_group = {
	.attrs = vbi_dab_dsp_attributes,
};

static const struct iio_info vbi_dab_dsp_info = {
	.read_raw = &vbi_dab_dsp_read_raw,
	.write_raw = &vbi_dab_dsp_write_raw,
	.attrs = &vbi_dab_dsp_attribute_group,
};

static const struct iio_chan_spec vbi_dab_dsp_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id vbi_dab_dsp_of_match[] = {
	{ .compatible = "fpga,vbi-dab-dsp", },
	{ },
};

MODULE_DEVICE_TABLE(of, vbi_dab_dsp_of_match);

static int vbi_dab_dsp_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct vbi_dab_dsp_state *st;
	struct iio_dev *indio_dev;
	int ret; //, i, n;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(vbi_dab_dsp_of_match, &pdev->dev);
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
		printk("VBI-dab-DSP: ***ERROR! \"required,fs-adc\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->fs_adc == 0){
		printk("VBI-dab-DSP: ***ERROR! \"required,fs-adc\" equal to 0 Hz\n");
		goto err_iio_device_free;
	}
	if(of_property_read_u32(np, "required,nb-of-blocks", &st->nb_of_blocks_dt)){
		printk("VBI-dab-DSP: ***ERROR! \"required,nb-of-blocks\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->nb_of_blocks_dt == 0){
		printk("VBI-dab-DSP: ***ERROR! \"required,nb-of-blocks\" equal to 0\n");
		goto err_iio_device_free;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->channels = vbi_dab_dsp_channels;
	indio_dev->num_channels = ARRAY_SIZE(vbi_dab_dsp_channels);
	indio_dev->info = &vbi_dab_dsp_info;
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

static int vbi_dab_dsp_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver vbi_dab_dsp_driver = {
	.probe		= vbi_dab_dsp_probe,
	.remove		= vbi_dab_dsp_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vbi_dab_dsp_of_match,
	},
};

module_platform_driver(vbi_dab_dsp_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("DAB Voice Break-In (VBI) FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
