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


#define DRIVER_NAME			"pcw-sdr-dsp"
#define NB_OF_BLOCKS			1
#define ADDR_PER_BLOCK			6*4
#define DDS_PHASEWIDTH			24
#define MIN_RX_CIC_DECIMATION		8
#define MAX_RX_CIC_DECIMATION		8192
#define MIN_RX_STREAM_DECIMATION	2*MIN_RX_CIC_DECIMATION
#define MAX_RX_STREAM_DECIMATION	2*MAX_RX_CIC_DECIMATION
#define MAX_ANTENNA_PATTERNS		32
#define ANTENNA_SWITCH_DELAY_OFFSET	112
#define ANTENNA_SWITCH_DELAY_SLOPE	35
#define RX_CH2_OFFSET_MAX		8192
#define BW_1ST_HALFBAND			40000000


#define ADDR_ANTENNA_PATTERNS		0*4
#define ADDR_RX_SAMPLES_PER_PATTERN	1*4
#define ADDR_ENOUT_PATTERNLENGTH	2*4
#define ADDR_ANTENNA_SW_DELAY		3*4
#define ADDR_GPO			4*4
#define ADDR_PPS_SETTINGS		5*4
#define ADDR_PPS_EXT_LSB		6*4
#define ADDR_PPS_EXT_MSB		7*4
#define ADDR_PPS_EXT_CNT		8*4
#define ADDR_PPS_GPS_LSB		9*4
#define ADDR_PPS_GPS_MSB		10*4
#define ADDR_PPS_GPS_CNT		11*4
#define ADDR_TX_ARB_LEN			12*4
#define ADDR_TX_PPS_EN			13*4

#define DDC_START_ADDR			14*4
#define ADDR_RX_SETTINGS_DDC(x)		(DDC_START_ADDR+0*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_INC(x)			(DDC_START_ADDR+1*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_FIR_COEFS(x)		(DDC_START_ADDR+2*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_DECIMATION_STREAM_CIC(x)	(DDC_START_ADDR+3*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_FRAME_LENGTH(x)		(DDC_START_ADDR+4*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_PERIOD(x)		(DDC_START_ADDR+5*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_OFFSET(x)		(DDC_START_ADDR+6*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_OVERFLOW_COUNT(x)	(DDC_START_ADDR+7*4+x*ADDR_PER_BLOCK)
#define ADDR_RX_CH2_OFFSET(x)		(DDC_START_ADDR+8*4+x*ADDR_PER_BLOCK)

#define ADDR_RX_ADC_OVERRANGE_COUNT	DDC_START_ADDR + 10*4
#define ADDR_RX_IQ_IMBALANCE		DDC_START_ADDR + 11*4
#define ADDR_RX_IQ_DC_OFFSET		DDC_START_ADDR + 12*4
#define ADDR_RX_MATCHED_FILTER_OFFSET		DDC_START_ADDR + 13*4

enum chan_num{
	CH_SEL,
	CH_RX_FREQUENCY,
	CH_RX_SOURCE,
	CH_ENABLE_HEADER,
	CH_INSERT_DF_HEADER,
	CH_SYNC_DF_TO_DMA,
	CH_ENABLE_LOOPBACK_TEST,
	CH_RX_PERIOD,
	CH_RX_FRAME_LENGTH,
	CH_RX_OFFSET,
	CH_RX_CH2_OFFSET,
	CH_RX_OVERFLOW_COUNT,
	CH_RX_ADC1_OVERRANGE_COUNT,
	CH_RX_ADC2_OVERRANGE_COUNT,
	CH_RX_STREAM_DECIMATION,
	CH_RX_CIC_DECIMATION,
	CH_RX_FIR_COEFFICIENTS,
	CH_RX_SWAP_IQ_RF,
	CH_RX_SWAP_IQ_BASEBAND,
	CH_RX_SWAP_IQ_MATCHED_FMCW_FILTER,
	CH_RX_ENABLE_MATCHED_FMCW_FILTER,
	CH_RX_ENABLE_IQ_DC_CORRECTION,
	CH_RX_IQ_IMBALANCE_ALPHA,
	CH_RX_IQ_IMBALANCE_BETA,
	CH_RX_I_DC_OFFSET,
	CH_RX_Q_DC_OFFSET,
	CH_RX_MATCHED_FILTER_OFFSET,
	CH_RX_SYNC,
	CH_ANTENNA_PATTERNS,
	CH_RX_SAMPLES_PER_PATTERN,
	CH_NR_PATTERNS,
	CH_ANTENNA_SWITCH_DELAY,
	CH_TX_ARBITRARY_LENGTH,
	CH_TX_SYNC_TO_PPS,
	CH_GPO_MASK,
	CH_GPO_VALUE,
	CH_PPS_COUNTER,
	CH_PPS_CLOCK_CYCLES,
	CH_PPS_SOURCE,
	CH_INT_TO_VOLT_SCALAR
};

struct pcw_sdr_dsp_state {
	struct iio_info	iio_info;
	void __iomem	*regs;
	struct mutex	lock;

	uint32_t	fs_adc;
	uint32_t	rx_frequency;
	uint32_t	rx_nyquist_zone;
	uint32_t	rx_swap_iq_rf;
	uint32_t	rx_adc_overrange_count[2];
	uint32_t	arb_mem_depth;
	uint32_t	nb_of_blocks;
	uint32_t	ch_nb;
	uint64_t	int_to_volt_scalar;
};

static void pcw_sdr_dsp_write(struct pcw_sdr_dsp_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 pcw_sdr_dsp_read(struct pcw_sdr_dsp_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int pcw_sdr_dsp_write_raw(struct iio_dev *indio_dev,
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

static int pcw_sdr_dsp_read_raw(struct iio_dev *indio_dev,
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



static ssize_t pcw_sdr_dsp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct pcw_sdr_dsp_state *st = iio_priv(indio_dev);
	long val;
	int ret, rx_stream_decimation, rx_offset, rx_samples_per_pattern;
	u64 temp64;
	int tempint;
	u32 temp32;
	u32 block_nb;
	unsigned long long readin;
	

	/* convert to long
	 * auto-detect decimal,
	 * octal (beginning with 0) and
	 * hexadecimal (beginning with 0x)
	 */
	if (((u32)this_attr->address == CH_INT_TO_VOLT_SCALAR)){
		ret = kstrtoull(buf, 10, &readin);
		if (ret)
			return ret;
	} else {
		ret = kstrtol(buf, 0, &val);
		if (ret < 0)
			return ret;
	}

	block_nb = st->ch_nb;
	
	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case CH_SEL:
		if(val<1 || val>st->nb_of_blocks){
			ret = -EINVAL;
			break;
		}
		st->ch_nb = (u32)val-1;
		break;

	case CH_RX_FREQUENCY:
		if(val<0){
			ret = -EINVAL;
			break;
		}

		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) & ~(1<<14 | 1<<1);

		st->rx_nyquist_zone = (u32)val/(st->fs_adc/2) + 1;
		if(val > st->fs_adc/2)
			val %= st->fs_adc/2;

		temp64 = (u64)val << DDS_PHASEWIDTH;
		temp64 = div_s64(temp64,st->fs_adc/2);
		tempint = (int)temp64;

		if(!((st->rx_nyquist_zone + st->rx_swap_iq_rf) & 1))
			temp32 |= 1<<14; // swap IQ when in an even nyquist zone

		if(val < (st->fs_adc>>2) - BW_1ST_HALFBAND)
			temp32 |= 1<<1; // disable Fs/4 DDC when f < ~30MHz
		else
			tempint -= 1 << (DDS_PHASEWIDTH-1); // subtract Fs/4 when Fs/4 DDC is enabled

		if(tempint<0)
			tempint += 1<<DDS_PHASEWIDTH;

		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32); // write settings
		
		pcw_sdr_dsp_write(st, ADDR_RX_INC(block_nb), (u32)tempint & 0xFFFFFF); // write phase increment word

		// recalculate frequency
		if(tempint >= 1 << (DDS_PHASEWIDTH-1))
			tempint -= 1 << DDS_PHASEWIDTH;
		if((temp32>>1) & 0x1 == 0){ // consider previous fs/4 downconversion
			tempint += 1 << (DDS_PHASEWIDTH-1);
		}
		temp64 = (u64)tempint * (st->fs_adc / 2);
		st->rx_frequency = (u32)(temp64 >> DDS_PHASEWIDTH) + st->fs_adc/2 * (st->rx_nyquist_zone - 1);
		break;

	case CH_RX_SOURCE:
		if(val<0 || val>9){
			ret = -EINVAL;
			break;
		}
		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) & ~0x7D;
		// 0: stream RX1 fs/4 HB DDC iq
		// 1: stream RX2 fs/4 HB DDC iq
		// 2: reserved: stream RX1 HB real
		// 3: reserved: stream RX2 HB real
		// 4: stream IQ, RX1 as I, RX2 as Q
		// 5: burst RX1 fs/4 HB DDC iq
		// 6: burst RX2 fs/4 HB DDC iq
		// 7: burst RX1 HB real
		// 8: burst RX2 HB real
		// 9: burst IQ, RX1 as I, RX2 as Q
		if(val>4){
			temp32 += (u32)( (1<<6) + ((val-5) << 3) + (val-5));
		}else{
			temp32 += (u32)val & 0x5;
		}
		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32);
		break;

	case CH_RX_SYNC:
		if(val<0 || val>2){
			ret = -EINVAL;
			break;
		}
		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) & ~(3<<8);
		// 0: off
		// 1: pps
		// 2: TX
		if(val>1){
			val++;
		}
		temp32 += (u32)val<<8;
		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32);
		break;

	case CH_ENABLE_HEADER:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) & ~(1<<7);
		temp32 += (u32)val << 7;
		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32);
		break;

	case CH_INSERT_DF_HEADER:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) & ~(1<<10);
		temp32 += (u32)val << 10;
		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32);
		break;

	case CH_SYNC_DF_TO_DMA:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) & ~(1<<13);
		temp32 += (u32)val << 13;
		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32);
		break;

	case CH_ENABLE_LOOPBACK_TEST:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) & ~(1<<11);
		temp32 += (u32)val << 11;
		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32);
		break;

	case CH_RX_PERIOD:
		pcw_sdr_dsp_write(st, ADDR_RX_PERIOD(block_nb), (u32)val);
		break;

	case CH_RX_FRAME_LENGTH:
		pcw_sdr_dsp_write(st, ADDR_RX_FRAME_LENGTH(block_nb), (u32)val);
		break;

	case CH_RX_OFFSET:
		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_DECIMATION_STREAM_CIC(block_nb)) >> 16;
		val = temp32*(u32)val-temp32*temp32/71+49*temp32;
		pcw_sdr_dsp_write(st, ADDR_RX_OFFSET(block_nb), (u32)val);
		break;

	case CH_RX_CH2_OFFSET:
		if(val > RX_CH2_OFFSET_MAX)
			val = RX_CH2_OFFSET_MAX;
		pcw_sdr_dsp_write(st, ADDR_RX_CH2_OFFSET(block_nb), (u32)val);
		break;

	case CH_RX_STREAM_DECIMATION:
		if(val<MIN_RX_STREAM_DECIMATION || val>MAX_RX_STREAM_DECIMATION){
			ret = -EINVAL;
			break;
		}

		rx_stream_decimation = pcw_sdr_dsp_read(st, ADDR_RX_DECIMATION_STREAM_CIC(block_nb)) >> 16;
		rx_offset = pcw_sdr_dsp_read(st, ADDR_RX_OFFSET(block_nb));
		rx_offset = (rx_offset - 49*rx_stream_decimation + rx_stream_decimation*rx_stream_decimation/71) / rx_stream_decimation;
		rx_samples_per_pattern = pcw_sdr_dsp_read(st, ADDR_RX_SAMPLES_PER_PATTERN);

		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_DECIMATION_STREAM_CIC(block_nb)) & 0xFFFF;
		temp32 += (u32)val << 16;
		pcw_sdr_dsp_write(st, ADDR_RX_DECIMATION_STREAM_CIC(block_nb), temp32);	// set new decimation

		if(( pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) >> 13 ) & 1){ // correct antenna switching delay
			tempint = ANTENNA_SWITCH_DELAY_OFFSET + ANTENNA_SWITCH_DELAY_SLOPE * val;	// correct only when sync_df_to_dma=1
			temp32 = (u32)((rx_samples_per_pattern - val) % rx_samples_per_pattern);
			pcw_sdr_dsp_write(st, ADDR_ANTENNA_SW_DELAY, temp32);
		}else{
			pcw_sdr_dsp_write(st, ADDR_ANTENNA_SW_DELAY, 0);
		}

		temp32 = (u32)(val*rx_offset-val*val/71+49*val);	// correct offset
		pcw_sdr_dsp_write(st, ADDR_RX_OFFSET(block_nb), temp32);

		break;

	case CH_RX_CIC_DECIMATION:
		if(val<MIN_RX_CIC_DECIMATION || val>MAX_RX_CIC_DECIMATION){
			ret = -EINVAL;
			break;
		}
		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_DECIMATION_STREAM_CIC(block_nb)) & 0xFFFF0000;
		temp32 += (u32)val;
		pcw_sdr_dsp_write(st, ADDR_RX_DECIMATION_STREAM_CIC(block_nb), temp32);
		break;

	case CH_RX_FIR_COEFFICIENTS:
		pcw_sdr_dsp_write(st, ADDR_RX_FIR_COEFS(block_nb), (u32)val);
		break;

	case CH_RX_SWAP_IQ_RF:
		if(val<-1 || val>1){
			ret = -EINVAL;
			break;
		}

		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) & ~(1<<14);
		u32 swap = 0;
		if(val < 0) // auto select by nyquist zone
			swap = (!((st->rx_nyquist_zone) & 1)) & 1;// swap IQ when in an even nyquist zone
		else
			swap = val & 1;

		st->rx_swap_iq_rf = swap;
		temp32 |= swap << 14;

		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32); // write settings
		break;

	case CH_RX_SWAP_IQ_BASEBAND:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) & ~(1<<15);
		temp32 += (u32)val << 15;
		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32); // write settings
		break;

	case CH_RX_SWAP_IQ_MATCHED_FMCW_FILTER:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}

		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) & ~(1<<16);
		temp32 += (u32)val << 16;

		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32); // write settings
		break;

	case CH_RX_ENABLE_MATCHED_FMCW_FILTER:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}

		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) & ~(1<<17);
		temp32 += (u32)val << 17;

		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32); // write settings
		break;

	case CH_RX_ENABLE_IQ_DC_CORRECTION:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}

		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) & ~(1<<18);
		temp32 += (u32)val << 18;

		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32); // write settings
		break;

	case CH_RX_IQ_IMBALANCE_ALPHA:
		if(val<-32768 || val>32767){
			ret = -EINVAL;
			break;
		}
		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_IQ_IMBALANCE) & 0xFFFF0000;
		temp32 += (u32)val;
		pcw_sdr_dsp_write(st, ADDR_RX_IQ_IMBALANCE, temp32);
		break;

	case CH_RX_IQ_IMBALANCE_BETA:
		if(val<-32768 || val>32767){
			ret = -EINVAL;
			break;
		}
		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_IQ_IMBALANCE) & 0xFFFF;
		temp32 += (u32)val <<16;
		pcw_sdr_dsp_write(st, ADDR_RX_IQ_IMBALANCE, temp32);
		break;

	case CH_RX_MATCHED_FILTER_OFFSET:
		temp32 = (u32)val;
		pcw_sdr_dsp_write(st, ADDR_RX_MATCHED_FILTER_OFFSET, temp32);
		break;

	case CH_ANTENNA_PATTERNS:
		pcw_sdr_dsp_write(st, ADDR_ANTENNA_PATTERNS, (u32)val);
		break;

	case CH_RX_SAMPLES_PER_PATTERN: // antenna switch delay fix related to channel 0 decimation factor
		pcw_sdr_dsp_write(st, ADDR_RX_SAMPLES_PER_PATTERN, (u32)val);	

		if(( pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) >> 13 ) & 1){ // correct antenna switching delay
			rx_stream_decimation = pcw_sdr_dsp_read(st, ADDR_RX_DECIMATION_STREAM_CIC(block_nb)) >> 16;
			rx_samples_per_pattern = pcw_sdr_dsp_read(st, ADDR_RX_SAMPLES_PER_PATTERN);
			val -= ANTENNA_SWITCH_DELAY_OFFSET + ANTENNA_SWITCH_DELAY_SLOPE * rx_stream_decimation;
			val %= rx_stream_decimation;
			pcw_sdr_dsp_write(st, ADDR_ANTENNA_SW_DELAY, (u32)val);
		}else{
			pcw_sdr_dsp_write(st, ADDR_ANTENNA_SW_DELAY, 0);
		}

		break;

	case CH_RX_OVERFLOW_COUNT: // reset when write a 0 
		if(val!=0){
			ret = -EINVAL;
			break;
		}
		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) | (1<<12);
		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32);
		temp32 &= ~(1 << 12);
		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(block_nb), temp32);
		break;

	case CH_RX_ADC1_OVERRANGE_COUNT: // reset when write a 0 
		if(val!=0){
			ret = -EINVAL;
			break;
		}
		st->rx_adc_overrange_count[0] = pcw_sdr_dsp_read(st, ADDR_RX_ADC_OVERRANGE_COUNT) & 0xFFFF;
		break;

	case CH_RX_ADC2_OVERRANGE_COUNT: // reset when write a 0 
		if(val!=0){
			ret = -EINVAL;
			break;
		}
		st->rx_adc_overrange_count[1] = pcw_sdr_dsp_read(st, ADDR_RX_ADC_OVERRANGE_COUNT) >> 16;
		break;

	case CH_NR_PATTERNS:
		if(val<0 || val>MAX_ANTENNA_PATTERNS-1){
			ret = -EINVAL;
			break;
		}		
		temp32 = pcw_sdr_dsp_read(st, ADDR_ENOUT_PATTERNLENGTH) & 0xFFFF0000;
		temp32 += (u32)val;
		pcw_sdr_dsp_write(st, ADDR_ENOUT_PATTERNLENGTH, temp32);
		break;

	case CH_ANTENNA_SWITCH_DELAY:
		rx_stream_decimation = pcw_sdr_dsp_read(st, ADDR_RX_DECIMATION_STREAM_CIC(block_nb)) >> 16;
		val %= rx_stream_decimation;
		pcw_sdr_dsp_write(st, ADDR_ANTENNA_SW_DELAY, (u32)val);
		break;

	case CH_TX_ARBITRARY_LENGTH:
		if(val > st->arb_mem_depth)
			val = st->arb_mem_depth;
		pcw_sdr_dsp_write(st, ADDR_TX_ARB_LEN, (u32)val);
		break;

	case CH_TX_SYNC_TO_PPS:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		temp32 = pcw_sdr_dsp_read(st, ADDR_TX_PPS_EN) & ~(1<<0);
		temp32 += (u32)val << 0;
		pcw_sdr_dsp_write(st, ADDR_TX_PPS_EN, temp32);
		break;

	case CH_GPO_MASK:
		if(val<0 || val>65535){
			ret = -EINVAL;
			break;
		}		
		temp32 = pcw_sdr_dsp_read(st, ADDR_ENOUT_PATTERNLENGTH) & 0xFFFF;
		temp32 += (u32)val << 16;
		pcw_sdr_dsp_write(st, ADDR_ENOUT_PATTERNLENGTH, temp32);
		break;

	case CH_GPO_VALUE:
		if(val<0 || val>65535){
			ret = -EINVAL;
			break;
		}		
		temp32 = pcw_sdr_dsp_read(st, ADDR_GPO) & 0xFFFF0000;
		temp32 += (u32)val;
		pcw_sdr_dsp_write(st, ADDR_GPO, temp32);
		break;

	case CH_PPS_SOURCE:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		if(val==1){
			val=3;
		}
		temp32 = pcw_sdr_dsp_read(st, ADDR_PPS_SETTINGS) & ~(3<<0);
		temp32 += (u32)val << 0;
		pcw_sdr_dsp_write(st, ADDR_PPS_SETTINGS, temp32);
		break;

	case CH_INT_TO_VOLT_SCALAR:
		st->int_to_volt_scalar = readin;
		break;


	default:
		ret = -ENODEV;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}




static ssize_t pcw_sdr_dsp_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct pcw_sdr_dsp_state *st = iio_priv(indio_dev);
	u32 temp32;
	int ret = 0, val;
	u64 ppsval = 0;
	u32 block_nb;
	u64 temp64;

	mutex_lock(&indio_dev->mlock);

	block_nb = st->ch_nb;

	switch ((u32)this_attr->address) {
	case CH_SEL:
		val = st->ch_nb+1;
		break;
	case CH_RX_FREQUENCY:
		val = st->rx_frequency;
		break;
	case CH_RX_SOURCE:
		val = pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) & 0x7D;
		// 0: stream RX1 fs/4 HB DDC iq
		// 1: stream RX2 fs/4 HB DDC iq
		// 2: stream RX1 HB real
		// 3: stream RX2 HB real
		// 4: stream IQ, RX1 as I, RX2 as Q
		// 5: burst RX1 fs/4 HB DDC iq
		// 6: burst RX2 fs/4 HB DDC iq
		// 7: burst RX1 HB real
		// 8: burst RX2 HB real
		// 9: burst IQ, RX1 as I, RX2 as Q
		if((val>>6) & 0x1){
			val = 5 + ((val>>3) & 0x7);
		}else{
			val &= 0x7;
		}
		break;
	case CH_RX_SYNC:
		val = ( pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) >> 8) & 3;
		// 0: off
		// 1: pps
		// 2: TX
		if(val>1){
			val--;
		}
		break;
	case CH_ENABLE_HEADER:
		val = ( pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) >> 7 ) & 1;
		break;
	case CH_INSERT_DF_HEADER:
		val = ( pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) >> 10 ) & 1;
		break;
	case CH_SYNC_DF_TO_DMA:
		val = ( pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) >> 13 ) & 1;
		break;
	case CH_ENABLE_LOOPBACK_TEST:
		val = ( pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) >> 11 ) & 1;
		break;
	case CH_RX_PERIOD:
		val = pcw_sdr_dsp_read(st, ADDR_RX_PERIOD(block_nb));
		break;
	case CH_RX_FRAME_LENGTH:
		val = pcw_sdr_dsp_read(st, ADDR_RX_FRAME_LENGTH(block_nb));
		break;
	case CH_RX_OFFSET:
		val = pcw_sdr_dsp_read(st, ADDR_RX_OFFSET(block_nb));
		temp32 = pcw_sdr_dsp_read(st, ADDR_RX_DECIMATION_STREAM_CIC(block_nb)) >> 16;
		val = (val - 49*temp32 + temp32*temp32/71) / temp32;
		break;
	case CH_RX_CH2_OFFSET:
		val = pcw_sdr_dsp_read(st, ADDR_RX_CH2_OFFSET(block_nb));
		break;
	case CH_RX_OVERFLOW_COUNT:
		val = pcw_sdr_dsp_read(st, ADDR_RX_OVERFLOW_COUNT(block_nb));
		break;
	case CH_RX_ADC1_OVERRANGE_COUNT:
		val = (pcw_sdr_dsp_read(st, ADDR_RX_ADC_OVERRANGE_COUNT) & 0xFFFF) - st->rx_adc_overrange_count[0];
		break;
	case CH_RX_ADC2_OVERRANGE_COUNT:
		val = (pcw_sdr_dsp_read(st, ADDR_RX_ADC_OVERRANGE_COUNT) >> 16) - st->rx_adc_overrange_count[1];
		break;
	case CH_RX_STREAM_DECIMATION:
		val = pcw_sdr_dsp_read(st, ADDR_RX_DECIMATION_STREAM_CIC(block_nb)) >> 16;
		break;
	case CH_RX_CIC_DECIMATION:
		val = pcw_sdr_dsp_read(st, ADDR_RX_DECIMATION_STREAM_CIC(block_nb)) & 0xFFFF;
		break;
	case CH_RX_FIR_COEFFICIENTS:
		val = pcw_sdr_dsp_read(st, ADDR_RX_FIR_COEFS(block_nb));
		break;
	case CH_RX_SWAP_IQ_RF:
		val = ( pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) >> 14 ) & 1;
		break;
	case CH_RX_SWAP_IQ_BASEBAND:
		val = ( pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) >> 15 ) & 1;
		break;
	case CH_RX_SWAP_IQ_MATCHED_FMCW_FILTER:
		val = ( pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) >> 16 ) & 1;
		break;
	case CH_RX_ENABLE_MATCHED_FMCW_FILTER:
		val = ( pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) >> 17 ) & 1;
		break;
	case CH_RX_ENABLE_IQ_DC_CORRECTION:
		val = ( pcw_sdr_dsp_read(st, ADDR_RX_SETTINGS_DDC(block_nb)) >> 18 ) & 1;
		break;
	case CH_RX_IQ_IMBALANCE_ALPHA:
		val = pcw_sdr_dsp_read(st, ADDR_RX_IQ_IMBALANCE) & 0xFFFF;
		break;
	case CH_RX_IQ_IMBALANCE_BETA:
		val = pcw_sdr_dsp_read(st, ADDR_RX_IQ_IMBALANCE) >>16;
		break;
	case CH_RX_MATCHED_FILTER_OFFSET:
		val = pcw_sdr_dsp_read(st, ADDR_RX_MATCHED_FILTER_OFFSET);
		break;
	case CH_RX_I_DC_OFFSET:
		val = (int)((short)(pcw_sdr_dsp_read(st, ADDR_RX_IQ_DC_OFFSET) & 0xFFFF));
		break;
	case CH_RX_Q_DC_OFFSET:
		val = (int)((short)(pcw_sdr_dsp_read(st, ADDR_RX_IQ_DC_OFFSET) >> 16));
		break;
	case CH_TX_ARBITRARY_LENGTH:
		val = pcw_sdr_dsp_read(st, ADDR_TX_ARB_LEN);
		break;
	case CH_TX_SYNC_TO_PPS:
		val = ( pcw_sdr_dsp_read(st, ADDR_TX_PPS_EN) >> 0 ) & 1;
		break;
	case CH_ANTENNA_PATTERNS:
		val = pcw_sdr_dsp_read(st, ADDR_ANTENNA_PATTERNS);
		break;
	case CH_RX_SAMPLES_PER_PATTERN:
		val = pcw_sdr_dsp_read(st, ADDR_RX_SAMPLES_PER_PATTERN);
		break;
	case CH_NR_PATTERNS:
		val = pcw_sdr_dsp_read(st, ADDR_ENOUT_PATTERNLENGTH) & 0xFFFF;
		break;
	case CH_ANTENNA_SWITCH_DELAY:
		val = pcw_sdr_dsp_read(st, ADDR_ANTENNA_SW_DELAY);
		break;
	case CH_PPS_COUNTER:
		val = pcw_sdr_dsp_read(st, ADDR_PPS_SETTINGS) & 0x1;
		if(val==0){
			val = pcw_sdr_dsp_read(st, ADDR_PPS_GPS_CNT);		
		}else{
			val = pcw_sdr_dsp_read(st, ADDR_PPS_EXT_CNT);	
		}
		break;
	case CH_PPS_CLOCK_CYCLES:
		val = pcw_sdr_dsp_read(st, ADDR_PPS_SETTINGS) & 0x1;
		
		if(val==0){
			ppsval = (u64) pcw_sdr_dsp_read(st, ADDR_PPS_GPS_LSB) & 0x00000000FFFFFFFF;		
			ppsval += (((u64)pcw_sdr_dsp_read(st, ADDR_PPS_GPS_MSB) << 32) & 0xFFFFFFFF00000000);
		}else{
			ppsval = (u64) pcw_sdr_dsp_read(st, ADDR_PPS_EXT_LSB) & 0x00000000FFFFFFFF;		
			ppsval += (((u64)pcw_sdr_dsp_read(st, ADDR_PPS_EXT_MSB) << 32) & 0xFFFFFFFF00000000);
		}		
		break;
	case CH_PPS_SOURCE:
		val = pcw_sdr_dsp_read(st, ADDR_PPS_SETTINGS) & 0x1;		
		break;
	case CH_GPO_MASK:
		val = pcw_sdr_dsp_read(st, ADDR_ENOUT_PATTERNLENGTH) >> 16;
		break;
	case CH_GPO_VALUE:
		val = pcw_sdr_dsp_read(st, ADDR_GPO) & 0xFFFF;		
		break;
	case CH_INT_TO_VOLT_SCALAR:
		temp64 = st->int_to_volt_scalar;
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0){
		if((u32)this_attr->address == CH_PPS_CLOCK_CYCLES)
			ret = sprintf(buf, "%llu\n", ppsval);
		else if((u32)this_attr->address == CH_INT_TO_VOLT_SCALAR)
			ret = sprintf(buf, "%llu\n", temp64);
		else
			ret = sprintf(buf, "%d\n", val);
	}
	return ret;
}



static IIO_DEVICE_ATTR(sel, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_SEL);

static IIO_DEVICE_ATTR(rx_frequency, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_FREQUENCY);

static IIO_DEVICE_ATTR(rx_source, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_SOURCE);

static IIO_DEVICE_ATTR(rx_sync, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_SYNC);


static IIO_DEVICE_ATTR(rx_period, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_PERIOD);

static IIO_DEVICE_ATTR(rx_frame_length, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_FRAME_LENGTH);

static IIO_DEVICE_ATTR(rx_offset, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_OFFSET);

static IIO_DEVICE_ATTR(rx_ch2_offset, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_CH2_OFFSET);

static IIO_DEVICE_ATTR(rx_overflow_count, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_OVERFLOW_COUNT);

static IIO_DEVICE_ATTR(rx_adc1_overrange_count, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_ADC1_OVERRANGE_COUNT);

static IIO_DEVICE_ATTR(rx_adc2_overrange_count, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_ADC2_OVERRANGE_COUNT);

static IIO_DEVICE_ATTR(rx_stream_decimation, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_STREAM_DECIMATION);

static IIO_DEVICE_ATTR(enable_header, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_ENABLE_HEADER);

static IIO_DEVICE_ATTR(insert_df_header, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_INSERT_DF_HEADER);

static IIO_DEVICE_ATTR(sync_df_to_dma, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_SYNC_DF_TO_DMA);

static IIO_DEVICE_ATTR(enable_loopback_test, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_ENABLE_LOOPBACK_TEST);

static IIO_DEVICE_ATTR(rx_cic_decimation, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_CIC_DECIMATION);

static IIO_DEVICE_ATTR(rx_fir_coefficients, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_FIR_COEFFICIENTS);

static IIO_DEVICE_ATTR(rx_swap_iq_rf, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_SWAP_IQ_RF);

static IIO_DEVICE_ATTR(rx_swap_iq_baseband, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_SWAP_IQ_BASEBAND);

static IIO_DEVICE_ATTR(rx_swap_iq_matched_fmcw_filter, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_SWAP_IQ_MATCHED_FMCW_FILTER);

static IIO_DEVICE_ATTR(rx_enable_matched_fmcw_filter, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_ENABLE_MATCHED_FMCW_FILTER);

static IIO_DEVICE_ATTR(rx_enable_iq_dc_correction, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_ENABLE_IQ_DC_CORRECTION);

static IIO_DEVICE_ATTR(rx_iq_imbalance_alpha, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_IQ_IMBALANCE_ALPHA);

static IIO_DEVICE_ATTR(rx_iq_imbalance_beta, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_IQ_IMBALANCE_BETA);

static IIO_DEVICE_ATTR(rx_matched_filter_offset, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_MATCHED_FILTER_OFFSET);

static IIO_DEVICE_ATTR(rx_i_dc_offset, S_IRUGO,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_I_DC_OFFSET);

static IIO_DEVICE_ATTR(rx_q_dc_offset, S_IRUGO,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_Q_DC_OFFSET);

static IIO_DEVICE_ATTR(tx_arbitrary_length, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_TX_ARBITRARY_LENGTH);

static IIO_DEVICE_ATTR(tx_sync_to_pps, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_TX_SYNC_TO_PPS);

static IIO_DEVICE_ATTR(antenna_patterns, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_ANTENNA_PATTERNS);

static IIO_DEVICE_ATTR(rx_samples_per_pattern, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_RX_SAMPLES_PER_PATTERN);

static IIO_DEVICE_ATTR(nb_of_patterns, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_NR_PATTERNS);

static IIO_DEVICE_ATTR(antenna_switch_delay, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_ANTENNA_SWITCH_DELAY);

static IIO_DEVICE_ATTR(gpo_mask, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_GPO_MASK);

static IIO_DEVICE_ATTR(gpo_value, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_GPO_VALUE);

static IIO_DEVICE_ATTR(pps_counter, S_IRUGO,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_PPS_COUNTER);

static IIO_DEVICE_ATTR(pps_clock_cycles, S_IRUGO,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_PPS_CLOCK_CYCLES);

static IIO_DEVICE_ATTR(pps_source, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_PPS_SOURCE);

static IIO_DEVICE_ATTR(int_to_volt_scalar, S_IRUGO | S_IWUSR,
			pcw_sdr_dsp_show,
			pcw_sdr_dsp_store,
			CH_INT_TO_VOLT_SCALAR);



static struct attribute *pcw_sdr_dsp_attributes[] = {
	&iio_dev_attr_sel.dev_attr.attr,
	&iio_dev_attr_rx_frequency.dev_attr.attr,
	&iio_dev_attr_rx_source.dev_attr.attr,
	&iio_dev_attr_rx_sync.dev_attr.attr,
	&iio_dev_attr_rx_period.dev_attr.attr,
	&iio_dev_attr_rx_frame_length.dev_attr.attr,
	&iio_dev_attr_rx_offset.dev_attr.attr,
	&iio_dev_attr_rx_ch2_offset.dev_attr.attr,
	&iio_dev_attr_rx_overflow_count.dev_attr.attr,
	&iio_dev_attr_rx_adc1_overrange_count.dev_attr.attr,
	&iio_dev_attr_rx_adc2_overrange_count.dev_attr.attr,
	&iio_dev_attr_rx_stream_decimation.dev_attr.attr,
	&iio_dev_attr_enable_header.dev_attr.attr,
	&iio_dev_attr_insert_df_header.dev_attr.attr,
	&iio_dev_attr_sync_df_to_dma.dev_attr.attr,
	&iio_dev_attr_enable_loopback_test.dev_attr.attr,
	&iio_dev_attr_rx_cic_decimation.dev_attr.attr,
	&iio_dev_attr_rx_fir_coefficients.dev_attr.attr,
	&iio_dev_attr_rx_swap_iq_rf.dev_attr.attr,
	&iio_dev_attr_rx_swap_iq_baseband.dev_attr.attr,
	&iio_dev_attr_rx_swap_iq_matched_fmcw_filter.dev_attr.attr,
	&iio_dev_attr_rx_enable_matched_fmcw_filter.dev_attr.attr,
	&iio_dev_attr_rx_enable_iq_dc_correction.dev_attr.attr,
	&iio_dev_attr_rx_iq_imbalance_alpha.dev_attr.attr,
	&iio_dev_attr_rx_iq_imbalance_beta.dev_attr.attr,
	&iio_dev_attr_rx_matched_filter_offset.dev_attr.attr,
	&iio_dev_attr_rx_i_dc_offset.dev_attr.attr,
	&iio_dev_attr_rx_q_dc_offset.dev_attr.attr,
	&iio_dev_attr_tx_arbitrary_length.dev_attr.attr,
	&iio_dev_attr_tx_sync_to_pps.dev_attr.attr,
	&iio_dev_attr_antenna_patterns.dev_attr.attr,
	&iio_dev_attr_rx_samples_per_pattern.dev_attr.attr,
	&iio_dev_attr_nb_of_patterns.dev_attr.attr,
	&iio_dev_attr_antenna_switch_delay.dev_attr.attr,
	&iio_dev_attr_gpo_mask.dev_attr.attr,
	&iio_dev_attr_gpo_value.dev_attr.attr,
	&iio_dev_attr_pps_counter.dev_attr.attr,
	&iio_dev_attr_pps_clock_cycles.dev_attr.attr,
	&iio_dev_attr_pps_source.dev_attr.attr,
	&iio_dev_attr_int_to_volt_scalar.dev_attr.attr,
	NULL
};


static const struct attribute_group pcw_sdr_dsp_attribute_group = {
	.attrs = pcw_sdr_dsp_attributes,
};

static const struct iio_info pcw_sdr_dsp_info = {
	.read_raw = &pcw_sdr_dsp_read_raw,
	.write_raw = &pcw_sdr_dsp_write_raw,
	.attrs = &pcw_sdr_dsp_attribute_group,
};

static const struct iio_chan_spec pcw_sdr_dsp_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id pcw_sdr_dsp_of_match[] = {
	{ .compatible = "fpga,pcw-sdr-dsp", },
	{ },
};

MODULE_DEVICE_TABLE(of, pcw_sdr_dsp_of_match);

static int pcw_sdr_dsp_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct pcw_sdr_dsp_state *st;
	struct iio_dev *indio_dev;
	u32 temp32,temp32_1;
	int ret, i;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(pcw_sdr_dsp_of_match, &pdev->dev);
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
		printk("PCW-SDR-DSP: ***ERROR! \"required,fs-adc\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->fs_adc == 0){
		printk("PCW-SDR-DSP: ***ERROR! \"required,fs-adc\" equal to 0 Hz\n");
		goto err_iio_device_free;
	}
	if(of_property_read_u32(np, "required,arb-mem-depth", &st->arb_mem_depth)){
		printk("PCW-SDR-DSP: ***ERROR! \"required,arb-mem-depth\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(of_property_read_u32(np, "required,nb-of-blocks", &st->nb_of_blocks)){
		printk("PCW-SDR-DSP: ***ERROR! \"required,nb-of-blocks\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->nb_of_blocks == 0){
		printk("PCW-SDR-DSP: ***ERROR! \"required,nb-of-blocks\" equal to 0\n");
		goto err_iio_device_free;
	}
	
	of_property_read_u32(np, "gpo_init_value", &temp32);
	of_property_read_u32(np, "en_arm_gpo", &temp32_1);
	temp32 = (temp32 & 0xffff) | ((temp32_1 << 16) & 0xffff0000);
	pcw_sdr_dsp_write(st, ADDR_GPO, temp32);
	of_property_read_u32(np, "gpo_mask", &temp32);
	pcw_sdr_dsp_write(st, ADDR_ENOUT_PATTERNLENGTH, (temp32<<16) & 0xffff0000);
	pcw_sdr_dsp_write(st, ADDR_RX_MATCHED_FILTER_OFFSET, 0);
	for(i=0; i<st->nb_of_blocks; i++)
		pcw_sdr_dsp_write(st, ADDR_RX_SETTINGS_DDC(i), 1<<15); // enable baseband IQ swapping

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->channels = pcw_sdr_dsp_channels;
	indio_dev->num_channels = ARRAY_SIZE(pcw_sdr_dsp_channels);
	indio_dev->info = &pcw_sdr_dsp_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	pcw_sdr_dsp_write(st, ADDR_RX_DECIMATION_STREAM_CIC(0), (128<<16) + 64);
	pcw_sdr_dsp_write(st, ADDR_RX_PERIOD(0), 16384);
	pcw_sdr_dsp_write(st, ADDR_RX_FRAME_LENGTH(0), 2048);


	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, indio_dev);
	return 0;

err_iio_device_free:
	iio_device_free(indio_dev);
	return ret;
}

static int pcw_sdr_dsp_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver pcw_sdr_dsp_driver = {
	.probe		= pcw_sdr_dsp_probe,
	.remove		= pcw_sdr_dsp_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = pcw_sdr_dsp_of_match,
	},
};

module_platform_driver(pcw_sdr_dsp_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("PCW SDR DSP FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
