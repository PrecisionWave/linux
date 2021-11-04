/*
 * PCW_WBRX SPI Wideband Synthesizer driver
 *
 * Copyright 2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/gcd.h>
#include <linux/gpio.h>
#include <asm/div64.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include <linux/clk-provider.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
//#include <linux/iio/frequency/pcw_wbrx.h>

#include <linux/clk/clkscale.h>

#define PCW_WBRX_LMX_ADDR(x)		(((x) & 0x0000007F) << 16)
#define PCW_WBRX_LMX_VALUE(x)		((x) & 0xFFFF)
#define PCW_WBRX_REG_NUM 		256
#define PCW_WBRX_LO_REG_NUM 		128
#define DRIVER_NAME			"pcw-wbrx"
#define NB_OF_DSP_BLOCKS		1
#define NB_OF_FRONTENDS			2
#define MAX_SPI_TRIALS			10

struct pcw_wbrx_platform_data {

	/* Register Map */
	//uint32_t					reg_map[32];
	char						name[SPI_NAME_SIZE];
};

enum chan_num{
	CH_DSP_SELECTION,
	CH_DSP_OFFSET_FREQUENCY,
	CH_OVERALL_INT_TO_VOLT_SCALAR,
	CH_FRONTEND_SELECTION,
	CH_FRONTEND_FREQUENCY,
	CH_FRONTEND_REFERENCE_LEVEL,
	CH_FRONTEND_PRE_FILTER_LNA,
	CH_FRONTEND_PRE_FILTER_ATTENUATOR,
	CH_FRONTEND_POST_FILTER_LNA,
	CH_FRONTEND_POST_FILTER_ATTENUATOR,
	CH_FRONTEND_AVOID_TUNABLE_FILTERS,
	CH_FRONTEND_ADC_GAIN,
	CH_FRONTEND_ADC_ATTENUATION,
	CH_FRONTEND_GAIN_MODE,
	CH_FRONTEND_SINT16_TO_VOLT_SCALAR,
	CH_FRONTEND_MIXER_OVERLOADED,
	CH_IOBOARD_ANTENNA_PATTERNS,
	CH_IOBOARD_GPIO_MASK,
	CH_IOBOARD_GPIO_OUTPUT_VALUE,
	CH_IOBOARD_GPIO_INPUT_VALUE,
	CH_IOBOARD_GPIO_DIRECTION,
	CH_IOBOARD_RELAY_VALUE,
	CH_IOBOARD_RESET_IP_BUTTON_VALUE,
	CH_IOBOARD_LEDS,
	CH_IOBOARD_FPGA_VERSION,
	CH_FE_RESERVED_SELECT_I2C_MODULE,
	CH_FE_RESERVED_FILTER_PRE_LNA,
	CH_FE_RESERVED_FILTER_PRE_LNA_FILTER,
	CH_FE_RESERVED_FILTER_PRE_ATTENUATOR,
	CH_FE_RESERVED_FILTER_POST_LNA,
	CH_FE_RESERVED_FILTER_POST_ATTENUATOR,
	CH_FE_RESERVED_FILTER_LF_MIXERBYPASS,
	CH_FE_RESERVED_FILTER_BAND,
	CH_FE_RESERVED_FILTER_TUNABLE_FILTER,
	CH_FE_RESERVED_FILTER_LEDS,
	CH_FE_RESERVED_FILTER_FPGA_VERSION,
	CH_FE_RESERVED_MIXER_INVERSION,
	CH_FE_RESERVED_MIXER_IF_FILTER,
	CH_FE_RESERVED_MIXER_ENABLE_CLK_OUT,
	CH_FE_RESERVED_MIXER_ENABLE_EXTERNAL_LO1,
	CH_FE_RESERVED_MIXER_ENABLE_EXTERNAL_LO2,
	CH_FE_RESERVED_MIXER_DATA_LO1,
	CH_FE_RESERVED_MIXER_DATA_LO2,
	CH_FE_RESERVED_MIXER_LEDS,
	CH_FE_RESERVED_MIXER_FPGA_VERSION,
	CH_FE_RESERVED_SPI_BIT_ERRORS,
        CH_LOCK_SPI
};

struct pcw_wbrx_state {
	struct spi_device	*spi;
	struct pcw_wbrx_platform_data	*pdata;
	u32			regs[PCW_WBRX_REG_NUM];
	uint32_t spi_rd_buffer;
	uint32_t spi_wr_buffer;

	uint32_t	nb_of_dsp_blocks;
	uint32_t	dsp_selection;
	uint32_t	dsp_offset_frequency[NB_OF_DSP_BLOCKS];
	uint64_t	overall_int_to_volt_scalar[NB_OF_DSP_BLOCKS];
	uint32_t	frontend_selection;
	uint64_t	frontend_frequency[NB_OF_FRONTENDS];
	int32_t		frontend_reference_level[NB_OF_FRONTENDS];
	int8_t		frontend_pre_filter_lna[NB_OF_FRONTENDS];
	int8_t		frontend_pre_filter_attenuator[NB_OF_FRONTENDS];
	int8_t		frontend_post_filter_lna[NB_OF_FRONTENDS];
	int8_t		frontend_post_filter_attenuator[NB_OF_FRONTENDS];
	int8_t		frontend_avoid_tunable_filters[NB_OF_FRONTENDS];
	int8_t		frontend_adc_gain[NB_OF_FRONTENDS];
	int8_t		frontend_adc_attenuation[NB_OF_FRONTENDS];
	int8_t		frontend_gain_mode[NB_OF_FRONTENDS];
	uint64_t	frontend_sint16_to_volt_scalar[NB_OF_FRONTENDS];
	int8_t		frontend_mixer_overloaded[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_select_i2c_module;
	uint8_t		fe_reserved_filter_pre_lna[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_filter_pre_lna_filter[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_filter_pre_attenuator[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_filter_post_lna[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_filter_post_attenuator[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_filter_lf_mixerbypass[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_filter_band[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_filter_tunable_filter[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_filter_leds[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_mixer_inversion[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_mixer_if_filter[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_mixer_enable_clk_out[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_mixer_enable_external_lo1[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_mixer_enable_external_lo2[NB_OF_FRONTENDS];
	uint8_t		fe_reserved_mixer_leds[NB_OF_FRONTENDS];
	unsigned	fe_reserved_spi_bit_errors;
	uint8_t		lock_spi[NB_OF_FRONTENDS];

	uint32_t	id_ioboard;
	uint32_t	id_mixer[NB_OF_FRONTENDS];
	uint32_t	id_filter[NB_OF_FRONTENDS];

	__be32			val ____cacheline_aligned;
};

struct child_clk {
	struct clk_hw		hw;
	struct pcw_wbrx_state	*st;
	bool			enabled;
	struct clock_scale 	scale;
};

#define to_clk_priv(_hw) container_of(_hw, struct child_clk, hw)

static int pcw_wbrx_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct pcw_wbrx_state *st = iio_priv(indio_dev);
	
	struct spi_transfer t = {
	    .tx_buf = &st->spi_wr_buffer,
	    .rx_buf = &st->spi_rd_buffer,
	    .len = 4,
	};

	int ret = 0, i, reg_lo;


	for(i=0; i<NB_OF_FRONTENDS*2; i++){
		if(((reg >> 8) & 0xff) == (st->id_mixer[i>>1] << 5 | (1 + (i & 0x1))) ){ // check if address is pointing to one of the LMX synths
			reg_lo = reg & 0x7f; // calculate read LMX register address
#ifdef LMX_NO_READBACK
			if (readval == NULL) {
				mutex_lock(&indio_dev->mlock);
				st->spi_wr_buffer = cpu_to_be32( (reg & 0xFF7F)<<16 | (writeval & 0xFFFF) ); // write
				ret = spi_sync_transfer(st->spi,&t, 1);
				st->lo_regs[reg_lo+i*PCW_WBRX_LO_REG_NUM] = (u16)writeval;
				mutex_unlock(&indio_dev->mlock);
			} else {
				*readval = (u32)st->lo_regs[reg_lo+i*PCW_WBRX_LO_REG_NUM]; // Write only version, copy read buffer to LMX registers
			}
#else
			mutex_lock(&indio_dev->mlock);
			if (readval == NULL) {
				st->spi_wr_buffer = cpu_to_be32( (reg & 0xFF7F)<<16 | (writeval & 0xFFFF) ); // write
				ret = spi_sync_transfer(st->spi,&t, 1);
				st->spi_wr_buffer = cpu_to_be32( (reg & 0xFF7F)<<16 | (writeval & 0xFFFF) | 1<<23 ); // set read bit
				ret |= spi_sync_transfer(st->spi,&t, 1); // write buffer again and readback values from previous write
				st->regs[(reg>>8) & 0xff] = be32_to_cpu(st->spi_rd_buffer) & 0xFFFFFF; // copy read buffer to LMX registers
			} else {
				st->spi_wr_buffer = cpu_to_be32( (reg & 0xFF7F)<<16 | 1<<23 ); // set read bit
				ret = spi_sync_transfer(st->spi,&t, 1); // write read bit & address and readback values
				st->regs[(reg>>8) & 0xff] = be32_to_cpu(st->spi_rd_buffer) & 0x7fffff; // copy read buffer to LMX registers
				*readval = st->regs[(reg>>8) & 0xff];
			}
			mutex_unlock(&indio_dev->mlock);
#endif
			return ret;
		}
	}

	if (reg >= PCW_WBRX_REG_NUM)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		st->spi_wr_buffer = cpu_to_be32( reg<<24 | (writeval & 0xFFFFFF) ); // write
		ret = spi_sync_transfer(st->spi,&t, 1);
		ret |= spi_sync_transfer(st->spi,&t, 1);
		//ret = spi_write_then_read(st->spi, &st->spi_buffer, 4, &st->spi_buffer, 4);
		st->regs[reg] = be32_to_cpu(st->spi_rd_buffer);
	} else {
		st->spi_wr_buffer = cpu_to_be32( reg<<24 | (st->regs[reg] & 0xFFFFFF) ); // read
		ret = spi_sync_transfer(st->spi,&t, 1);
		ret |= spi_sync_transfer(st->spi,&t, 1);
		//ret = spi_write_then_read(st->spi, &st->spi_buffer, 4, &st->spi_buffer, 4);
		st->regs[reg] = be32_to_cpu(st->spi_rd_buffer) & 0xFFFFFF;
		*readval = st->regs[reg];
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t pcw_lo_read_spi_transfer(struct iio_dev *indio_dev, unsigned reg)
{
	struct pcw_wbrx_state *st = iio_priv(indio_dev);
	
	struct spi_transfer t = {
	    .tx_buf = &st->spi_wr_buffer,
	    .rx_buf = &st->spi_rd_buffer,
	    .len = 4,
	};
	st->spi_wr_buffer = cpu_to_be32( (reg & 0xFF)<<24 | (st->regs[reg] & 0xFFFFFF) | 1<<23); // set R/W to read (MSB of 24 LMX Bits)
	spi_sync_transfer(st->spi,&t, 1); // write buffer again and readback values from previous write
	st->regs[reg] = be32_to_cpu(st->spi_rd_buffer) & 0x7FFFFF; // copy read buffer to LMX registers
	return 0;
}

static ssize_t pcw_wbrx_spi_transfer(struct iio_dev *indio_dev, unsigned reg, unsigned write_only)
{
	struct pcw_wbrx_state *st = iio_priv(indio_dev);
	
	struct spi_transfer t = {
	    .tx_buf = &st->spi_wr_buffer,
	    .rx_buf = &st->spi_rd_buffer,
	    .len = 4,
	};

	int ret = 0, i;
	unsigned word_sent, rb_val, bit_errors, trial, xor_val;

	if (reg >= PCW_WBRX_REG_NUM)
		return -EINVAL;

	word_sent = (reg<<24) | (st->regs[reg] & 0xFFFFFF); // save for later readback comparison
	
	trial = 0;
	do{
		st->spi_wr_buffer = cpu_to_be32(word_sent); // copy 8 address bits and 24 data bits to buffer
		ret = spi_sync_transfer(st->spi, &t, 1); // write buffer
		
		if(write_only == 0){ // skip readback when write_only
			ret |= spi_sync_transfer(st->spi,&t, 1); // if no LMX reg: write buffer again and readback values from previous write
			//ret = spi_write_then_read(st->spi, &st->spi_buffer, 4, &st->spi_buffer, 4);
			st->regs[reg] = be32_to_cpu(st->spi_rd_buffer) & 0xFFFFFF;
		}
		
		// readback check
		st->spi_wr_buffer = cpu_to_be32( ((reg & 0xE0) | 31)<<24 ); // read 24 data bits from data readback register
		ret |= spi_sync_transfer(st->spi,&t, 1);
		rb_val = be32_to_cpu(st->spi_rd_buffer) & 0xFFFFFF;
		st->spi_wr_buffer = cpu_to_be32( ((reg & 0xE0) | 29)<<24 ); // read 8 addr bits from address readback register
		ret |= spi_sync_transfer(st->spi,&t, 1);
		rb_val |= (be32_to_cpu(st->spi_rd_buffer) & 0xFF) << 24;
		xor_val = word_sent ^ rb_val; // compare addr&data bits readback from Reg31 with saved word
		
		// count bit errors
		bit_errors = 0;
		while (xor_val) { // Kernighan's bit count algorithm
			xor_val &= (xor_val - 1); 
			bit_errors++; 
		} 
		st->fe_reserved_spi_bit_errors += bit_errors;
		trial++;
	}while(trial<=MAX_SPI_TRIALS && bit_errors!=0);

	if(bit_errors==0)
		return 0;
	else
		return -EIO;
}

static ssize_t pcw_wbrx_update_filter(struct iio_dev *indio_dev)
{
	struct pcw_wbrx_state *st = iio_priv(indio_dev);

	int fe_nb;
	unsigned reg;
	ssize_t ret;

	fe_nb = st->frontend_selection;


	reg = (st->id_filter[fe_nb] << 5) | 3;	// select & write other filter bits
	st->regs[reg] &= ~(0x3fff03);
	st->regs[reg] |= ((u32)st->fe_reserved_filter_band[fe_nb]) << 18;
	st->regs[reg] |= ((u32)st->fe_reserved_filter_post_attenuator[fe_nb]) << 16;
	st->regs[reg] |= ((u32)st->fe_reserved_filter_post_lna[fe_nb]) << 14;
	st->regs[reg] |= ((u32)st->fe_reserved_filter_pre_attenuator[fe_nb]) << 12;
	st->regs[reg] |= ((u32)st->fe_reserved_filter_pre_lna[fe_nb]) << 10;
	st->regs[reg] |= ((u32)st->fe_reserved_filter_pre_lna_filter[fe_nb]) << 9;
	st->regs[reg] |= ((u32)st->fe_reserved_filter_lf_mixerbypass[fe_nb]) << 8;
	st->regs[reg] |= ((u32)st->fe_reserved_filter_leds[fe_nb]) << 0;
	
	ret = pcw_wbrx_spi_transfer(indio_dev, reg, 1);		// write data bits to destination register

	reg = (st->id_filter[fe_nb] << 5) | 1; // write tunable filter
	st->regs[reg] = st->fe_reserved_filter_tunable_filter[fe_nb];
	ret |= pcw_wbrx_spi_transfer(indio_dev, reg, 1);

	return ret;
}

static ssize_t pcw_wbrx_update_mixer(struct iio_dev *indio_dev)
{
	struct pcw_wbrx_state *st = iio_priv(indio_dev);

	int ret=0, fe_nb;
	unsigned reg;

	fe_nb = st->frontend_selection;

	reg = st->id_mixer[fe_nb] << 5 | 3;	// select & write other mixer bits
	st->regs[reg] &= ~(1<<13 | 1<<12 | 1<<11 | 1<<10 | 1<<9 | 1<<1 | 1<<0);
	st->regs[reg] |= st->fe_reserved_mixer_inversion[fe_nb] << 13;
	st->regs[reg] |= st->fe_reserved_mixer_if_filter[fe_nb] << 12;
	st->regs[reg] |= st->fe_reserved_mixer_enable_external_lo1[fe_nb] << 11;
	st->regs[reg] |= st->fe_reserved_mixer_enable_clk_out[fe_nb] << 10;
	st->regs[reg] |= st->fe_reserved_mixer_enable_external_lo2[fe_nb] << 9;
	st->regs[reg] |= st->fe_reserved_mixer_leds[fe_nb];	// bit 0 & bit 1
	ret = pcw_wbrx_spi_transfer(indio_dev, reg, 1);

	return ret;
}


static ssize_t pcw_wbrx_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct pcw_wbrx_state *st = iio_priv(indio_dev);
	long val;
	int ret = 0, save_val;
	u32 dsp_nb, fe_nb, reg;
	unsigned long long readin;

	/* convert to long
	 * auto-detect decimal,
	 * octal (beginning with 0) and
	 * hexadecimal (beginning with 0x)
	 */
	if (	((u32)this_attr->address == CH_FRONTEND_FREQUENCY)
	||	((u32)this_attr->address == CH_FRONTEND_SINT16_TO_VOLT_SCALAR)
	||	((u32)this_attr->address == CH_OVERALL_INT_TO_VOLT_SCALAR)
	){
		ret = kstrtoull(buf, 10, &readin);
		if (ret)
			return ret;
	} else {
		ret = kstrtol(buf, 0, &val);
		if (ret < 0)
			return ret;
	}

	mutex_lock(&indio_dev->mlock);

	dsp_nb = st->dsp_selection;
	fe_nb = st->frontend_selection;
	
	switch ((u32)this_attr->address) {
	case CH_DSP_SELECTION:
		if(val<1 || val>st->nb_of_dsp_blocks){
			ret = -EINVAL;
			break;
		}
		st->dsp_selection = (u32)val-1;
		break;
	case CH_DSP_OFFSET_FREQUENCY:
		st->dsp_offset_frequency[dsp_nb] = val;
		break;
	case CH_OVERALL_INT_TO_VOLT_SCALAR:
		st->overall_int_to_volt_scalar[dsp_nb] = readin;
		break;
	case CH_FRONTEND_SELECTION:
		if(val<1 || val>NB_OF_FRONTENDS){
			ret = -EINVAL;
			break;
		}
		st->frontend_selection = (u32)val-1;
		break;
	case CH_FRONTEND_FREQUENCY:
		st->frontend_frequency[fe_nb] = readin;
		break;
	case CH_FRONTEND_REFERENCE_LEVEL:
		st->frontend_reference_level[fe_nb] = val;
		break;
	case CH_FRONTEND_PRE_FILTER_LNA:
		if(val<-1 || val>1){
			ret = -EINVAL;
			break;
		}
		st->frontend_pre_filter_lna[fe_nb] = val;
		break;
	case CH_FRONTEND_PRE_FILTER_ATTENUATOR:
		st->frontend_pre_filter_attenuator[fe_nb] = val;
		break;
	case CH_FRONTEND_POST_FILTER_LNA:
		if(val<-1 || val>1){
			ret = -EINVAL;
			break;
		}
		st->frontend_post_filter_lna[fe_nb] = val;
		break;
	case CH_FRONTEND_POST_FILTER_ATTENUATOR:
		st->frontend_post_filter_attenuator[fe_nb] = val;
		break;
	case CH_FRONTEND_AVOID_TUNABLE_FILTERS:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		st->frontend_avoid_tunable_filters[fe_nb] = val;
		break;
	case CH_FRONTEND_ADC_GAIN:
		st->frontend_adc_gain[fe_nb] = val;
		break;
	case CH_FRONTEND_ADC_ATTENUATION:
		st->frontend_adc_attenuation[fe_nb] = val;
		break;
	case CH_FRONTEND_GAIN_MODE:
		st->frontend_gain_mode[fe_nb] = val;
		break;
	case CH_FRONTEND_SINT16_TO_VOLT_SCALAR:
		st->frontend_sint16_to_volt_scalar[fe_nb] = readin;
		break;
	case CH_FRONTEND_MIXER_OVERLOADED:
		st->frontend_mixer_overloaded[fe_nb] = val;
		break;
	case CH_IOBOARD_ANTENNA_PATTERNS:
		reg = st->id_ioboard << 5 | 5;
		st->regs[reg] = (u32)val;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		break;
	case CH_IOBOARD_GPIO_MASK:
		reg = st->id_ioboard << 5 | 6;
		st->regs[reg] = (u32)val & 0xff;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		break;
	case CH_IOBOARD_GPIO_OUTPUT_VALUE:
		reg = st->id_ioboard << 5 | 4;
		st->regs[reg] = st->regs[reg] & ~0xff | (u32)val & 0xff;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		break;
	case CH_IOBOARD_GPIO_DIRECTION:
		reg = st->id_ioboard << 5 | 4;
		st->regs[reg] = st->regs[reg] & ~(0xff << 8) | ((u32)val & 0xff) << 8;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		break;
	case CH_IOBOARD_RELAY_VALUE:
		reg = st->id_ioboard << 5 | 3;
		st->regs[reg] = st->regs[reg] & ~0xf | (u32)val & 0xf;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		break;
	case CH_IOBOARD_LEDS:
		reg = st->id_ioboard << 5 | 3;
		st->regs[reg] = st->regs[reg] & ~(0xf << 9) | ((u32)val & 0xf) << 9;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		break;
	case CH_FE_RESERVED_SELECT_I2C_MODULE:
		// reset i2c_vcc_en_n on all devices, set EEPROM address to 0x56
		reg = st->id_ioboard << 5 | 3;
		st->regs[reg] = st->regs[reg] & ~0x1f0 | 0x160; // reset bit7 i2c_vcc_en=0, bit8 adc_addr_a0=1 (AD5593R Addr=0x10) and set a_eeprom=6 (bit6..4)
		pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		reg = st->id_filter[0] << 5 | 3;
		st->regs[reg] = st->regs[reg] & ~0x3c | 0x18; // reset bit5 i2c_vcc_en and set a_eeprom=6 (bit4..2)
		pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		reg = st->id_filter[1] << 5 | 3;
		st->regs[reg] = st->regs[reg] & ~0x3c | 0x18; // reset bit5 i2c_vcc_en and set a_eeprom=6 (bit4..2)
		pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		reg = st->id_mixer[0] << 5 | 3;
		st->regs[reg] = st->regs[reg] & ~0x15c | 0x118; // set bit8 adc_addr_a0=1 (AD5593R Addr=0x11), bit6 i2c_vcc_en=0 and set a_eeprom=6 (bit4..2)
		pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		reg = st->id_mixer[1] << 5 | 3;
		st->regs[reg] = st->regs[reg] & ~0x15c | 0x118; // set bit8 adc_addr_a0=1 (AD5593R Addr=0x11), bit6 i2c_vcc_en=0 and set a_eeprom=6 (bit4..2)
		pcw_wbrx_spi_transfer(indio_dev, reg, 1);

		// set i2c_vcc_en_n on selected device, set EEPROM address to 0x53
		st->fe_reserved_select_i2c_module = (u8)val & 0x7;
		reg = st->fe_reserved_select_i2c_module << 5 | 3;
		if((u8)val == st->id_ioboard){
			st->regs[reg] = st->regs[reg] & ~0x1f0 | 0xb0; // reset bit8 adc_addr_a0=0 (AD5593R Addr=0x10), set bit7 i2c_vcc_en=1 and set a_eeprom=3 (bit6..4), EEPROM address to 0x53
			ret = pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		}else if((u8)val == st->id_filter[0] || (u8)val == st->id_filter[1]){
			st->regs[reg] = st->regs[reg] & ~0x3c | 0x2c; // set bit5 i2c_vcc_en=1 and set a_eeprom=3 (bit4..2), EEPROM address to 0x53
			ret = pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		}else if((u8)val == st->id_mixer[0] || (u8)val == st->id_mixer[1]){
			st->regs[reg] = st->regs[reg] & ~0x15c | 0x4c; // reset bit8 adc_addr_a0=0 (AD5593R Addr=0x10), set bit6 i2c_vcc_en=1 and set a_eeprom=3 (bit4..2), EEPROM address to 0x53
			ret = pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		}
		break;
	case CH_FE_RESERVED_FILTER_PRE_LNA:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_filter_pre_lna[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x1) )
		    ret = pcw_wbrx_update_filter(indio_dev);
		break;
	case CH_FE_RESERVED_FILTER_PRE_LNA_FILTER:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_filter_pre_lna_filter[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x1) )
		    ret = pcw_wbrx_update_filter(indio_dev);
		break;
	case CH_FE_RESERVED_FILTER_PRE_ATTENUATOR:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_filter_pre_attenuator[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x1) )
		    ret = pcw_wbrx_update_filter(indio_dev);
		break;
	case CH_FE_RESERVED_FILTER_POST_LNA:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_filter_post_lna[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x1) )
		    ret = pcw_wbrx_update_filter(indio_dev);
		break;
	case CH_FE_RESERVED_FILTER_POST_ATTENUATOR:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_filter_post_attenuator[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x1) )
		    ret = pcw_wbrx_update_filter(indio_dev);
		break;
	case CH_FE_RESERVED_FILTER_LF_MIXERBYPASS:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_filter_lf_mixerbypass[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x1) )
		    ret = pcw_wbrx_update_filter(indio_dev);
		break;
	case CH_FE_RESERVED_FILTER_BAND:
		if(val<0 || val>15){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_filter_band[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x1) )
		    ret = pcw_wbrx_update_filter(indio_dev);
		break;
	case CH_FE_RESERVED_FILTER_TUNABLE_FILTER:
		if(val<0 || val>31){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_filter_tunable_filter[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x1) )
		    ret = pcw_wbrx_update_filter(indio_dev);
		break;
	case CH_FE_RESERVED_FILTER_LEDS:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_filter_leds[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x1) )
		    ret = pcw_wbrx_update_filter(indio_dev);
		break;
	case CH_FE_RESERVED_MIXER_INVERSION:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_mixer_inversion[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x2) )
		    ret = pcw_wbrx_update_mixer(indio_dev);
		break;
	case CH_FE_RESERVED_MIXER_IF_FILTER:
		if(val<0 || val>1){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_mixer_if_filter[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x2) )
		    ret = pcw_wbrx_update_mixer(indio_dev);
		break;
	case CH_FE_RESERVED_MIXER_ENABLE_CLK_OUT:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_mixer_enable_clk_out[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x2) )
		    ret = pcw_wbrx_update_mixer(indio_dev);
		break;
	case CH_FE_RESERVED_MIXER_ENABLE_EXTERNAL_LO1:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_mixer_enable_external_lo1[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x2) )
		    ret = pcw_wbrx_update_mixer(indio_dev);
		break;
	case CH_FE_RESERVED_MIXER_ENABLE_EXTERNAL_LO2:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_mixer_enable_external_lo2[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x2) )
		    ret = pcw_wbrx_update_mixer(indio_dev);
		break;
	case CH_FE_RESERVED_MIXER_DATA_LO1:
		reg = st->id_mixer[fe_nb] << 5 | 1;
		st->regs[reg] = (u32)val & 0x7fffff;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		break;
	case CH_FE_RESERVED_MIXER_DATA_LO2:
		reg = st->id_mixer[fe_nb] << 5 | 2;
		st->regs[reg] = (u32)val & 0x7fffff;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 1);
		break;
	case CH_FE_RESERVED_MIXER_LEDS:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
		st->fe_reserved_mixer_leds[fe_nb] = (u8)val;
                if( !(st->lock_spi[fe_nb] & 0x2) )
		    ret = pcw_wbrx_update_mixer(indio_dev);
		break;
	case CH_LOCK_SPI:
		if(val<0 || val>3){
			ret = -EINVAL;
			break;
		}
                save_val = st->lock_spi[fe_nb];
		st->lock_spi[fe_nb] = (u8)val;
                if( (save_val & 0x1) && !(val & 0x1) )
                    ret = pcw_wbrx_update_filter(indio_dev);
                if( (save_val & 0x2) && !(val & 0x2) )
                    ret |= pcw_wbrx_update_mixer(indio_dev);
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}





static ssize_t pcw_wbrx_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct pcw_wbrx_state *st = iio_priv(indio_dev);
	u32 val, temp32;
	int ret = 0;
	int tempint;
	u64 temp64;
	u32 dsp_nb, fe_nb, reg;

	mutex_lock(&indio_dev->mlock);

	dsp_nb = st->dsp_selection;
	fe_nb = st->frontend_selection;

	switch ((u32)this_attr->address) {
	case CH_DSP_SELECTION:
		val = st->dsp_selection+1;
		break;
	case CH_DSP_OFFSET_FREQUENCY:
		val = st->dsp_offset_frequency[dsp_nb];
		break;
	case CH_OVERALL_INT_TO_VOLT_SCALAR:
		temp64 = st->overall_int_to_volt_scalar[dsp_nb];
		break;
	case CH_FRONTEND_SELECTION:
		val = st->frontend_selection+1;
		break;
	case CH_FRONTEND_FREQUENCY:
		temp64 = st->frontend_frequency[fe_nb];
		break;
	case CH_FRONTEND_REFERENCE_LEVEL:
		val = st->frontend_reference_level[fe_nb];
		break;
	case CH_FRONTEND_PRE_FILTER_LNA:
		val = st->frontend_pre_filter_lna[fe_nb];
		break;
	case CH_FRONTEND_PRE_FILTER_ATTENUATOR:
		val = st->frontend_pre_filter_attenuator[fe_nb];
		break;
	case CH_FRONTEND_POST_FILTER_LNA:
		val = st->frontend_post_filter_lna[fe_nb];
		break;
	case CH_FRONTEND_POST_FILTER_ATTENUATOR:
		val = st->frontend_post_filter_attenuator[fe_nb];
		break;
	case CH_FRONTEND_AVOID_TUNABLE_FILTERS:
		val = st->frontend_avoid_tunable_filters[fe_nb];
		break;
	case CH_FRONTEND_ADC_GAIN:
		val = st->frontend_adc_gain[fe_nb];
		break;
	case CH_FRONTEND_ADC_ATTENUATION:
		val = st->frontend_adc_attenuation[fe_nb];
		break;
	case CH_FRONTEND_GAIN_MODE:
		val = st->frontend_gain_mode[fe_nb];
		break;
	case CH_FRONTEND_SINT16_TO_VOLT_SCALAR:
		temp64 = st->frontend_sint16_to_volt_scalar[fe_nb];
		break;
	case CH_FRONTEND_MIXER_OVERLOADED:
		val = st->frontend_mixer_overloaded[fe_nb];
		break;
	case CH_IOBOARD_ANTENNA_PATTERNS:
		reg = st->id_ioboard << 5 | 5;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 0);
		val = st->regs[reg];
		break;
	case CH_IOBOARD_GPIO_MASK:
		reg = st->id_ioboard << 5 | 6;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 0);
		val = st->regs[reg] & 0xff;
		break;
	case CH_IOBOARD_GPIO_OUTPUT_VALUE:
		reg = st->id_ioboard << 5 | 4;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 0);
		val = st->regs[reg] & 0xff;
		break;
	case CH_IOBOARD_GPIO_INPUT_VALUE:
		reg = st->id_ioboard << 5 | 4;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 0);
		val = st->regs[reg] >> 16 & 0xff;
		break;
	case CH_IOBOARD_GPIO_DIRECTION:
		reg = st->id_ioboard << 5 | 4;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 0);
		val = st->regs[reg] >> 8 & 0xff;
		break;
	case CH_IOBOARD_RELAY_VALUE:
		reg = st->id_ioboard << 5 | 3;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 0);
		val = st->regs[reg] & 0xf;
		break;
	case CH_IOBOARD_RESET_IP_BUTTON_VALUE:
		reg = st->id_ioboard << 5 | 3;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 0);
		val = st->regs[reg] >> 23 & 0x1;
		break;
	case CH_IOBOARD_LEDS:
		reg = st->id_ioboard << 5 | 3;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 0);
		val = st->regs[reg] >> 9 & 0xf;
		break;
	case CH_IOBOARD_FPGA_VERSION:
		reg = st->id_ioboard << 5 | 30;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 0);
		val = st->regs[reg] & 0xFFFFFF;
		break;
	case CH_FE_RESERVED_SELECT_I2C_MODULE:
		val = st->fe_reserved_select_i2c_module;
		break;
	case CH_FE_RESERVED_FILTER_PRE_LNA:
		val = st->fe_reserved_filter_pre_lna[fe_nb];
		break;
	case CH_FE_RESERVED_FILTER_PRE_LNA_FILTER:
		val = st->fe_reserved_filter_pre_lna_filter[fe_nb];
		break;
	case CH_FE_RESERVED_FILTER_PRE_ATTENUATOR:
		val = st->fe_reserved_filter_pre_attenuator[fe_nb];
		break;
	case CH_FE_RESERVED_FILTER_POST_LNA:
		val = st->fe_reserved_filter_post_lna[fe_nb];
		break;
	case CH_FE_RESERVED_FILTER_POST_ATTENUATOR:
		val = st->fe_reserved_filter_post_attenuator[fe_nb];
		break;
	case CH_FE_RESERVED_FILTER_LF_MIXERBYPASS:
		val = st->fe_reserved_filter_lf_mixerbypass[fe_nb];
		break;
	case CH_FE_RESERVED_FILTER_BAND:
		val = st->fe_reserved_filter_band[fe_nb];
		break;
	case CH_FE_RESERVED_FILTER_TUNABLE_FILTER:
		val = st->fe_reserved_filter_tunable_filter[fe_nb];
		break;
	case CH_FE_RESERVED_FILTER_LEDS:
		val = st->fe_reserved_filter_leds[fe_nb];
		break;
	case CH_FE_RESERVED_FILTER_FPGA_VERSION:
		reg = st->id_filter[fe_nb] << 5 | 30;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 0);
		val = st->regs[reg] & 0xFFFFFF;
		break;
	case CH_FE_RESERVED_MIXER_INVERSION:
		val = st->fe_reserved_mixer_inversion[fe_nb];
		break;
	case CH_FE_RESERVED_MIXER_IF_FILTER:
		val = st->fe_reserved_mixer_if_filter[fe_nb];
		break;
	case CH_FE_RESERVED_MIXER_ENABLE_CLK_OUT:
		val = st->fe_reserved_mixer_enable_clk_out[fe_nb];
		break;
	case CH_FE_RESERVED_MIXER_ENABLE_EXTERNAL_LO1:
		val = st->fe_reserved_mixer_enable_external_lo1[fe_nb];
		break;
	case CH_FE_RESERVED_MIXER_ENABLE_EXTERNAL_LO2:
		val = st->fe_reserved_mixer_enable_external_lo2[fe_nb];
		break;
	case CH_FE_RESERVED_MIXER_DATA_LO1:
		reg = st->id_mixer[fe_nb] << 5 | 1;
		pcw_lo_read_spi_transfer(indio_dev, reg);
		val = st->regs[reg];
		break;
	case CH_FE_RESERVED_MIXER_DATA_LO2:
		reg = st->id_mixer[fe_nb] << 5 | 2;
		pcw_lo_read_spi_transfer(indio_dev, reg);
		val = st->regs[reg];
		break;
	case CH_FE_RESERVED_MIXER_LEDS:
		val = st->fe_reserved_mixer_leds[fe_nb];
		break;
	case CH_FE_RESERVED_MIXER_FPGA_VERSION:
		reg = st->id_mixer[fe_nb] << 5 | 30;
		ret = pcw_wbrx_spi_transfer(indio_dev, reg, 0);
		val = st->regs[reg] & 0xFFFFFF;
		break;
	case CH_FE_RESERVED_SPI_BIT_ERRORS:
		val = st->fe_reserved_spi_bit_errors;
		break;
	case CH_LOCK_SPI:
		val = st->lock_spi[fe_nb];
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0){
		if(	((u32)this_attr->address == CH_FRONTEND_FREQUENCY)
		||	((u32)this_attr->address == CH_FRONTEND_SINT16_TO_VOLT_SCALAR)
		||	((u32)this_attr->address == CH_OVERALL_INT_TO_VOLT_SCALAR)
		){
			ret = sprintf(buf, "%llu\n", temp64);
		} else {
			ret = sprintf(buf, "%d\n", val);
		}
	}
	return ret;
}



static IIO_DEVICE_ATTR(dsp_selection, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_DSP_SELECTION);

static IIO_DEVICE_ATTR(dsp_offset_frequency, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_DSP_OFFSET_FREQUENCY);

static IIO_DEVICE_ATTR(overall_int_to_volt_scalar, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_OVERALL_INT_TO_VOLT_SCALAR);

static IIO_DEVICE_ATTR(frontend_selection, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FRONTEND_SELECTION);

static IIO_DEVICE_ATTR(frontend_frequency, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FRONTEND_FREQUENCY);

static IIO_DEVICE_ATTR(frontend_reference_level, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FRONTEND_REFERENCE_LEVEL);

static IIO_DEVICE_ATTR(frontend_pre_filter_lna, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FRONTEND_PRE_FILTER_LNA);

static IIO_DEVICE_ATTR(frontend_pre_filter_attenuator, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FRONTEND_PRE_FILTER_ATTENUATOR);

static IIO_DEVICE_ATTR(frontend_post_filter_lna, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FRONTEND_POST_FILTER_LNA);

static IIO_DEVICE_ATTR(frontend_post_filter_attenuator, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FRONTEND_POST_FILTER_ATTENUATOR);

static IIO_DEVICE_ATTR(frontend_avoid_tunable_filters, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FRONTEND_AVOID_TUNABLE_FILTERS);

static IIO_DEVICE_ATTR(frontend_adc_gain, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FRONTEND_ADC_GAIN);

static IIO_DEVICE_ATTR(frontend_adc_attenuation, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FRONTEND_ADC_ATTENUATION);

static IIO_DEVICE_ATTR(frontend_gain_mode, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FRONTEND_GAIN_MODE);

static IIO_DEVICE_ATTR(frontend_sint16_to_volt_scalar, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FRONTEND_SINT16_TO_VOLT_SCALAR);

static IIO_DEVICE_ATTR(frontend_mixer_overloaded, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FRONTEND_MIXER_OVERLOADED);

static IIO_DEVICE_ATTR(ioboard_antenna_patterns, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_IOBOARD_ANTENNA_PATTERNS);

static IIO_DEVICE_ATTR(ioboard_gpio_mask, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_IOBOARD_GPIO_MASK);

static IIO_DEVICE_ATTR(ioboard_gpio_output_value, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_IOBOARD_GPIO_OUTPUT_VALUE);

static IIO_DEVICE_ATTR(ioboard_gpio_input_value, S_IRUGO,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_IOBOARD_GPIO_INPUT_VALUE);

static IIO_DEVICE_ATTR(ioboard_gpio_direction, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_IOBOARD_GPIO_DIRECTION);

static IIO_DEVICE_ATTR(ioboard_relay_value, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_IOBOARD_RELAY_VALUE);

static IIO_DEVICE_ATTR(ioboard_reset_ip_button, S_IRUGO,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_IOBOARD_RESET_IP_BUTTON_VALUE);

static IIO_DEVICE_ATTR(ioboard_leds, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_IOBOARD_LEDS);

static IIO_DEVICE_ATTR(ioboard_fpga_version, S_IRUGO,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_IOBOARD_FPGA_VERSION);

static IIO_DEVICE_ATTR(fe_reserved_select_i2c_module, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_SELECT_I2C_MODULE);

static IIO_DEVICE_ATTR(fe_reserved_filter_pre_lna, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_FILTER_PRE_LNA);

static IIO_DEVICE_ATTR(fe_reserved_filter_pre_lna_filter, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_FILTER_PRE_LNA_FILTER);

static IIO_DEVICE_ATTR(fe_reserved_filter_pre_attenuator, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_FILTER_PRE_ATTENUATOR);

static IIO_DEVICE_ATTR(fe_reserved_filter_post_lna, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_FILTER_POST_LNA);

static IIO_DEVICE_ATTR(fe_reserved_filter_post_attenuator, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_FILTER_POST_ATTENUATOR);

static IIO_DEVICE_ATTR(fe_reserved_filter_lf_mixerbypass, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_FILTER_LF_MIXERBYPASS);

static IIO_DEVICE_ATTR(fe_reserved_filter_band, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_FILTER_BAND);

static IIO_DEVICE_ATTR(fe_reserved_filter_tunable_filter, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_FILTER_TUNABLE_FILTER);

static IIO_DEVICE_ATTR(fe_reserved_filter_leds, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_FILTER_LEDS);

static IIO_DEVICE_ATTR(fe_reserved_filter_fpga_version, S_IRUGO,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_FILTER_FPGA_VERSION);

static IIO_DEVICE_ATTR(fe_reserved_mixer_inversion, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_MIXER_INVERSION);

static IIO_DEVICE_ATTR(fe_reserved_mixer_if_filter, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_MIXER_IF_FILTER);

static IIO_DEVICE_ATTR(fe_reserved_mixer_enable_clk_out, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_MIXER_ENABLE_CLK_OUT);

static IIO_DEVICE_ATTR(fe_reserved_mixer_enable_external_lo1, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_MIXER_ENABLE_EXTERNAL_LO1);

static IIO_DEVICE_ATTR(fe_reserved_mixer_enable_external_lo2, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_MIXER_ENABLE_EXTERNAL_LO2);

static IIO_DEVICE_ATTR(fe_reserved_mixer_data_lo1, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_MIXER_DATA_LO1);

static IIO_DEVICE_ATTR(fe_reserved_mixer_data_lo2, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_MIXER_DATA_LO2);

static IIO_DEVICE_ATTR(fe_reserved_mixer_leds, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_MIXER_LEDS);

static IIO_DEVICE_ATTR(fe_reserved_mixer_fpga_version, S_IRUGO,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_MIXER_FPGA_VERSION);

static IIO_DEVICE_ATTR(fe_reserved_spi_bit_errors, S_IRUGO,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_FE_RESERVED_SPI_BIT_ERRORS);

static IIO_DEVICE_ATTR(lock_spi, S_IRUGO | S_IWUSR,
			pcw_wbrx_show,
			pcw_wbrx_store,
			CH_LOCK_SPI);


static struct attribute *pcw_wbrx_attributes[] = {
	&iio_dev_attr_dsp_selection.dev_attr.attr,
	&iio_dev_attr_dsp_offset_frequency.dev_attr.attr,
	&iio_dev_attr_overall_int_to_volt_scalar.dev_attr.attr,
	&iio_dev_attr_frontend_selection.dev_attr.attr,
	&iio_dev_attr_frontend_frequency.dev_attr.attr,
	&iio_dev_attr_frontend_reference_level.dev_attr.attr,
	&iio_dev_attr_frontend_pre_filter_lna.dev_attr.attr,
	&iio_dev_attr_frontend_pre_filter_attenuator.dev_attr.attr,
	&iio_dev_attr_frontend_post_filter_lna.dev_attr.attr,
	&iio_dev_attr_frontend_post_filter_attenuator.dev_attr.attr,
	&iio_dev_attr_frontend_avoid_tunable_filters.dev_attr.attr,
	&iio_dev_attr_frontend_adc_gain.dev_attr.attr,
	&iio_dev_attr_frontend_adc_attenuation.dev_attr.attr,
	&iio_dev_attr_frontend_gain_mode.dev_attr.attr,
	&iio_dev_attr_frontend_sint16_to_volt_scalar.dev_attr.attr,
	&iio_dev_attr_frontend_mixer_overloaded.dev_attr.attr,
	&iio_dev_attr_ioboard_antenna_patterns.dev_attr.attr,
	&iio_dev_attr_ioboard_gpio_mask.dev_attr.attr,
	&iio_dev_attr_ioboard_gpio_output_value.dev_attr.attr,
	&iio_dev_attr_ioboard_gpio_input_value.dev_attr.attr,
	&iio_dev_attr_ioboard_gpio_direction.dev_attr.attr,
	&iio_dev_attr_ioboard_relay_value.dev_attr.attr,
	&iio_dev_attr_ioboard_reset_ip_button.dev_attr.attr,
	&iio_dev_attr_ioboard_leds.dev_attr.attr,
	&iio_dev_attr_ioboard_fpga_version.dev_attr.attr,
	&iio_dev_attr_fe_reserved_select_i2c_module.dev_attr.attr,
	&iio_dev_attr_fe_reserved_filter_pre_lna.dev_attr.attr,
	&iio_dev_attr_fe_reserved_filter_pre_lna_filter.dev_attr.attr,
	&iio_dev_attr_fe_reserved_filter_pre_attenuator.dev_attr.attr,
	&iio_dev_attr_fe_reserved_filter_post_lna.dev_attr.attr,
	&iio_dev_attr_fe_reserved_filter_post_attenuator.dev_attr.attr,
	&iio_dev_attr_fe_reserved_filter_lf_mixerbypass.dev_attr.attr,
	&iio_dev_attr_fe_reserved_filter_band.dev_attr.attr,
	&iio_dev_attr_fe_reserved_filter_tunable_filter.dev_attr.attr,
	&iio_dev_attr_fe_reserved_filter_leds.dev_attr.attr,
	&iio_dev_attr_fe_reserved_filter_fpga_version.dev_attr.attr,
	&iio_dev_attr_fe_reserved_mixer_inversion.dev_attr.attr,
	&iio_dev_attr_fe_reserved_mixer_if_filter.dev_attr.attr,
	&iio_dev_attr_fe_reserved_mixer_enable_clk_out.dev_attr.attr,
	&iio_dev_attr_fe_reserved_mixer_enable_external_lo1.dev_attr.attr,
	&iio_dev_attr_fe_reserved_mixer_enable_external_lo2.dev_attr.attr,
	&iio_dev_attr_fe_reserved_mixer_data_lo1.dev_attr.attr,
	&iio_dev_attr_fe_reserved_mixer_data_lo2.dev_attr.attr,
	&iio_dev_attr_fe_reserved_mixer_leds.dev_attr.attr,
	&iio_dev_attr_fe_reserved_mixer_fpga_version.dev_attr.attr,
	&iio_dev_attr_fe_reserved_spi_bit_errors.dev_attr.attr,
	&iio_dev_attr_lock_spi.dev_attr.attr,
	NULL
};


static const struct attribute_group pcw_wbrx_attribute_group = {
	.attrs = pcw_wbrx_attributes,
};

static const struct iio_chan_spec_ext_info pcw_wbrx_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	{ },
};

static const struct iio_chan_spec pcw_wbrx_chan[] = {
};

static const struct iio_chan_spec_ext_info adf4355_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	{ },
};


static const struct iio_info pcw_wbrx_info = {
	.debugfs_reg_access = &pcw_wbrx_reg_access,
	.attrs = &pcw_wbrx_attribute_group,
};

#ifdef CONFIG_OF
static struct pcw_wbrx_platform_data *pcw_wbrx_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct pcw_wbrx_platform_data *pdata;
	unsigned int tmp;
	int ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}

	strncpy(&pdata->name[0], np->name, SPI_NAME_SIZE - 1);

	return pdata;
}
#else
static
struct pcw_wbrx_platform_data *pcw_wbrx_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static const struct clk_ops clkout_ops = {
};

static int pcw_wbrx_probe(struct spi_device *spi)
{
	struct pcw_wbrx_platform_data *pdata;
	struct iio_dev *indio_dev;
	struct pcw_wbrx_state *st;
	struct clk *clk = NULL;
	int ret, i;

	if (spi->dev.of_node)
		pdata = pcw_wbrx_parse_dt(&spi->dev);
	else
		pdata = spi->dev.platform_data;

	if (!pdata) {
		dev_warn(&spi->dev, "Error: no platform data\n");
		return -EINVAL;
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL) {
		ret =  -ENOMEM;
		return ret;
	}

	st = iio_priv(indio_dev);

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;
	st->pdata = pdata;

	for(i=0; i<NB_OF_FRONTENDS; i++){
		st->frontend_pre_filter_lna[i] = -1;
		st->frontend_pre_filter_attenuator[i] = -1;
		st->frontend_post_filter_lna[i] = -1;
		st->frontend_post_filter_attenuator[i] = -1;
		st->frontend_avoid_tunable_filters[i] = 0;
		st->frontend_adc_gain[i] = -10;
		st->frontend_adc_attenuation[i] = -1;
	}

	if(of_property_read_u32(spi->dev.of_node, "required,nb-of-dsp-blocks", &st->nb_of_dsp_blocks)){
		printk("pcw-wbrx: ***ERROR! \"required,nb-of-dsp-blocks\" missing in devicetree?\n");
		st->nb_of_dsp_blocks = 1;
	}
	if(st->nb_of_dsp_blocks == 0){
		printk("pcw-wbrx: ***ERROR! \"required,nb-of-dsp-blocks\" equal to 0\n");
		st->nb_of_dsp_blocks = 1;
	}

	if(of_property_read_u32(spi->dev.of_node, "required,id-ioboard", &st->id_ioboard)){
		printk("pcw-wbrx: ***ERROR! \"required,id-ioboard\" missing in devicetree?\n");
		st->id_ioboard = 1;
	}
	if(st->id_ioboard > 7){
		printk("pcw-wbrx: ***ERROR! \"required,id-ioboard\" must be between 0 and 7\n");
		st->id_ioboard = 1;
	}
	if(of_property_read_u32(spi->dev.of_node, "required,id-mixer1", &st->id_mixer[0])){
		printk("pcw-wbrx: ***ERROR! \"required,id-mixer1\" missing in devicetree?\n");
		st->id_mixer[0] = 2;
	}
	if(st->id_mixer[0] > 7){
		printk("pcw-wbrx: ***ERROR! \"required,id-mixer1\" must be between 0 and 7\n");
		st->id_mixer[0] = 2;
	}
	if(of_property_read_u32(spi->dev.of_node, "required,id-mixer2", &st->id_mixer[1])){
		printk("pcw-wbrx: ***ERROR! \"required,id-mixer2\" missing in devicetree?\n");
		st->id_mixer[1] = 3;
	}
	if(st->id_mixer[1] > 7){
		printk("pcw-wbrx: ***ERROR! \"required,id-mixer2\" must be between 0 and 7\n");
		st->id_mixer[1] = 3;
	}
	if(of_property_read_u32(spi->dev.of_node, "required,id-filter1", &st->id_filter[0])){
		printk("pcw-wbrx: ***ERROR! \"required,id-filter1\" missing in devicetree?\n");
		st->id_filter[0] = 4;
	}
	if(st->id_filter[0] > 7){
		printk("pcw-wbrx: ***ERROR! \"required,id-filter1\" must be between 0 and 7\n");
		st->id_filter[0] = 4;
	}
	if(of_property_read_u32(spi->dev.of_node, "required,id-filter2", &st->id_filter[1])){
		printk("pcw-wbrx: ***ERROR! \"required,id-filter2\" missing in devicetree?\n");
		st->id_filter[1] = 5;
	}
	if(st->id_filter[1] > 7){
		printk("pcw-wbrx: ***ERROR! \"required,id-filter2\" must be between 0 and 7\n");
		st->id_filter[1] = 5;
	}

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = (pdata->name[0] != 0) ? pdata->name :
		spi_get_device_id(spi)->name;

	indio_dev->info = &pcw_wbrx_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(indio_dev);


	return 0;
}

static int pcw_wbrx_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct pcw_wbrx_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	if (IS_ENABLED(CONFIG_OF))
		of_clk_del_provider(spi->dev.of_node);

	return 0;
}


static const struct spi_device_id pcw_wbrx_id[] = {
	{"pcw-wbrx", 0},
	{}
};

static struct spi_driver pcw_wbrx_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= pcw_wbrx_probe,
	.remove		= pcw_wbrx_remove,
	.id_table	= pcw_wbrx_id,
};
module_spi_driver(pcw_wbrx_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("PrecisionWave PCW_WBRX");
MODULE_LICENSE("GPL v2");

