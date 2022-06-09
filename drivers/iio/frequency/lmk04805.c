/*
 * LMK04805 SPI Low Jitter Clock Generator
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>


#define LMK04805_ADDR(x)		((x) & 0x0000001F)
#define LMK04805_VALUE(x)		((x) & (0xFFFFFFFF - 0x1F))

#define LMK04805_NUM_CHAN		12


#define LMK04805_GET_CLK_FORMAT_REG(ch)		(ch/4)+6

#define LMK04805_GET_CLK_DIVIDER_REG(ch)	(ch/2)
#define LMK04805_GET_CLK_DIVIDER(x)			((x >> 5) & 0x7FF)
#define LMK04805_SET_CLK_DIVIDER(x)			((x & 0x7FF) << 5)
#define LMK04805_CLK_FORMAT_POWERDOWN		0x00
#define LMK04805_CLK_FORMAT_LVDS			0x1
#define LMK04805_CLK_FORMAT_LVPECL_1600		0x4

const uint32_t lmk04805_reg_default[] = {
		0x00140600,
		0x00140041,
		0x00140142,
		0x00140083,
		0x00140144,
		0x00140105,
		0x40040006,
		0x10010007,
		0x40010008,
		0x55555549,
		0x9102410A,
		0x0000300B, //0x03F0300B
		0x1B0C006C, //0x1B0C01AC
		0x1B01002D, //0x1B01002D,
		0x1200000E,
		0x8009C40F,
		0xC1550410,
		0x00000011,
		0x00000012,
		0x00000013,
		0x00000014,
		0x00000015,
		0x00000016,
		0x00000017,
		0x00000058,
		0x0049C419, //0x02C9C419,
		0xAFA8001A,
		0x1800041B, //0x1C00019B
		0x0060101C, //0x0060061C
		0x0100039D, //0x0180039D
		0x0200039E,
		0x001F001F
};

enum outp_drv_mode {
	TRISTATE,
	LVPECL_8mA,
	LVDS_4mA,
	LVDS_7mA,
	HSTL0_16mA,
	HSTL1_8mA,
	CMOS_CONF1,
	CMOS_CONF2,
	CMOS_CONF3,
	CMOS_CONF4,
	CMOS_CONF5,
	CMOS_CONF6,
	CMOS_CONF7,
	CMOS_CONF8,
	CMOS_CONF9
};
//
//enum ref_sel_mode {
//	NONEREVERTIVE_STAY_ON_REFB,
//	REVERT_TO_REFA,
//	SELECT_REFA,
//	SELECT_REFB,
//	EXT_REF_SEL
//};

/**
 * struct lmk04805_channel_spec - Output channel configuration
 *
 * @channel_num: Output channel number.
 * @divider_output_invert_en: Invert the polarity of the output clock.
 * @sync_ignore_en: Ignore chip-level SYNC signal.
 * @low_power_mode_en: Reduce power used in the differential output modes.
 * @use_alt_clock_src: Channel divider uses alternative clk source.
 * @output_dis: Disables, powers down the entire channel.
 * @driver_mode: Output driver mode (logic level family).
 * @divider_phase: Divider initial phase after a SYNC. Range 0..63
		   LSB = 1/2 of a period of the divider input clock.
 * @channel_divider: 10-bit channel divider.
 * @extended_name: Optional descriptive channel name.
 */

struct lmk04805_channel_spec {
	uint32_t		channel_num;
//	bool			divider_output_invert_en;
//	bool			sync_ignore_en;
//	bool			low_power_mode_en;
				 /* CH0..CH3 VCXO, CH4..CH9 VCO2 */
//	bool			use_alt_clock_src;
	bool			powerdown;
	// enum outp_drv_mode	driver_mode;
//	uint8_t			divider_phase;
	uint8_t			out_type;
	uint16_t		clock_divider;
	char			extended_name[16];
};


/**
 * struct lmk04805_platform_data - platform specific information
 *
 */

struct lmk04805_platform_data {

	/* Clock Output Control */
	bool						RESET;
	bool						POWERDOWN;
	bool						CLKout0_1_PD;
	bool						CLKout2_3_PD;
	bool						CLKout4_5_PD;
	bool						CLKout6_7_PD;
	bool						CLKout8_9_PD;
	bool						CLKout10_11_PD;
	uint8_t						CLKout6_7_OSCin_Sel;
	uint8_t						CLKout8_9_OSCin_Sel;
	uint16_t 					CLKout0_1_DIV;
	uint16_t 					CLKout2_3_DIV;
	uint16_t 					CLKout4_5_DIV;
	uint16_t 					CLKout6_7_DIV;
	uint16_t 					CLKout8_9_DIV;
	uint16_t 					CLKout10_11_DIV;
	uint8_t						CLKout0_TYPE;
	uint8_t						CLKout1_TYPE;
	uint8_t						CLKout2_TYPE;
	uint8_t						CLKout3_TYPE;
	uint8_t						CLKout4_TYPE;
	uint8_t						CLKout5_TYPE;
	uint8_t						CLKout6_TYPE;
	uint8_t						CLKout7_TYPE;
	uint8_t						CLKout8_TYPE;
	uint8_t						CLKout9_TYPE;
	uint8_t						CLKout10_TYPE;
	uint8_t						CLKout11_TYPE;
//	bool						CLKoutX_ADLY_SEL;
//	uint16_t					CLKoutX_Y_DDLY;
//	bool						CLKoutX_Y_HS;
//	uint8_t						CLKoutX_Y_ADLY;
//
//	/* Osc Buffer Control */
//	uint8_t						OSCout1_LVPECL_AMP;
//	uint8_t						OSCout0_TYPE;
//	bool						EN_OSCout1;
//	bool						EN_OSCout0;
//	bool						OSCout1_MUX;
//	bool						OSCout0_MUX;
//	bool						PD_OSCin;
//	uint8_t						OSCout_DIV;
//
//	/* Mode */
	bool						VCO_MUX;
//	bool						EN_FEEDBACK_MUX;
	uint8_t						VCO_DIV;
//	uint8_t						FEEDBACK_MUX;
//	uint8_t						MODE;
//
//	/* Clock Synchronization */
//	bool						EN_SYNC;
//	bool						NO_SYNC_CLKout10_11;
//	bool						NO_SYNC_CLKout8_9;
//	bool						NO_SYNC_CLKout6_7;
//	bool						NO_SYNC_CLKout4_5;
//	bool						NO_SYNC_CLKout2_3;
//	bool						NO_SYNC_CLKout0_1;
//	uint8_t						SYNC_MUX;
//	bool						SYNC_QUAL;
//	bool						SYNC_POL_INV;
//	bool						SYNC_EN_AUTO;
//	uint8_t						SYNC_TYPE;
//
//	/* Other Mode Control */
//	bool						EN_PLL2_XTAL;
//	uint8_t						LD_MUX;
//	uint8_t						LD_TYPE;
//	bool						SYNC_PLL2_DLD;
//	bool						SYNC_PLL1_DLD;
//	bool						EN_TRACK;
//	uint8_t						HOLDOVER_MODE;
//	uint8_t						HOLDOVER_MUX;
//	uint8_t						HOLDOVER_TYPE;
//	uint8_t						Status_CLKin1_MUX;
//	uint8_t						Status_CLKin0_TYPE;
//	bool						DISABLE_DLD1_DET;
//	uint8_t						Status_CLKin0_MUX;
	uint8_t						CLKin_SELECT_MODE;
//	bool						CLKin_Sel_INV;
//
//	/* CLKin Control */
//	bool						EN_CLKin1;
//	bool						EN_CLKin0;
//	uint8_t						LOS_TIMEOUT;
//	bool						EN_LOS;
//	uint8_t						Status_CLKin1_TYPE;
//	bool						CLKin1_BUF_TYPE;
//	bool						CLKin0_BUF_TYPE;
//
//	/* DAC Control */
//	uint8_t						DAC_HIGH_TRIP;
//	uint8_t						DAC_LOW_TRIP;
//	bool						EN_VTUNE_RAIL_DET;
//	uint16_t					MAN_DAC;
//	bool						EN_MAN_DAC;
//
//	uint16_t					HOLDOVER_DLD_CNT;
//	bool						FORCE_HOLDOVER;
//	uint8_t						XTAL_LVL;
//
//	/* PLL Control */
//	uint8_t						PLL2_C4_LF;
//	uint8_t						PLL2_C3_LF;
//	uint8_t						PLL2_R4_LF;
//	uint8_t						PLL2_R3_LF;
//	uint8_t						PLL1_N_DLY;
//	uint8_t						PLL1_R_DLY;
//	uint8_t						PLL1_WND_SIZE;
//	uint16_t					DAC_CLK_DIV;
//	uint16_t					PLL1_DLD_CNT;
//	uint8_t						PLL2_WND_SIZE;
	bool						EN_PLL2_REF_2X;
//	bool						PLL2_CP_POL;
//	uint8_t						PLL2_CP_GAIN;
//	uint16_t					PLL2_DLD_CNT;
//	bool						PLL2_CP_TRI;
//	bool						PLL1_CP_POL;
//	uint8_t						PLL1_CP_GAIN;
//	uint8_t						CLKin1_PreR_DIV;
//	uint8_t						CLKin0_PreR_DIV;
	uint16_t					PLL1_R;
//	bool						PLL1_CP_TRI;
	uint16_t					PLL2_R;
	uint16_t					PLL1_N;
//	uint8_t						OSCin_FREQ;
//	bool						PLL2_FAST_PDF;
	uint32_t					PLL2_N_CAL;
	uint8_t						PLL2_P;
	uint8_t						VCO_MODE;
	uint32_t					PLL2_N;
//
//	bool						READBACK_LE;
//	uint8_t						READBACK_ADDR;
//	bool						uWire_LOCK;

	/* Register Map */
	uint32_t					reg_map[32];

	uint32_t 					vcxo_freq;

	/* Output Channel Configuration */
	int						num_channels;
	struct lmk04805_channel_spec*	channels;

	char						name[SPI_NAME_SIZE];
};

#define ATTR_REF(IIO_ATTR) \
	&iio_dev_attr_##IIO_ATTR.dev_attr.attr

#define CLK_ATTR(CLK, ATTR) \
	(ATTR_CLK0_##ATTR + CLK)

#define ALL_CLK_ATTR(ATTR) \
	ATTR_CLK0_##ATTR, \
	ATTR_CLK1_##ATTR, \
	ATTR_CLK2_##ATTR, \
	ATTR_CLK3_##ATTR, \
	ATTR_CLK4_##ATTR, \
	ATTR_CLK5_##ATTR, \
	ATTR_CLK6_##ATTR, \
	ATTR_CLK7_##ATTR, \
	ATTR_CLK8_##ATTR, \
	ATTR_CLK9_##ATTR, \
	ATTR_CLK10_##ATTR, \
	ATTR_CLK11_##ATTR

#define ALL_CLK_IIO_DEVICE_ATTR(IIO_ATTR, RW, SHOW, STORE, ATTR) \
	static IIO_DEVICE_ATTR(CLKout0_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK0_##ATTR); \
	static IIO_DEVICE_ATTR(CLKout1_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK1_##ATTR); \
	static IIO_DEVICE_ATTR(CLKout2_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK2_##ATTR); \
	static IIO_DEVICE_ATTR(CLKout3_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK3_##ATTR); \
	static IIO_DEVICE_ATTR(CLKout4_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK4_##ATTR); \
	static IIO_DEVICE_ATTR(CLKout5_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK5_##ATTR); \
	static IIO_DEVICE_ATTR(CLKout6_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK6_##ATTR); \
	static IIO_DEVICE_ATTR(CLKout7_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK7_##ATTR); \
	static IIO_DEVICE_ATTR(CLKout8_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK8_##ATTR); \
	static IIO_DEVICE_ATTR(CLKout9_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK9_##ATTR); \
	static IIO_DEVICE_ATTR(CLKout10_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK10_##ATTR); \
	static IIO_DEVICE_ATTR(CLKout11_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK11_##ATTR);

#define ALL_CLK_IIO_ATTR_REF(IIO_ATTR) \
	&iio_dev_attr_CLKout0_##IIO_ATTR.dev_attr.attr, \
	&iio_dev_attr_CLKout1_##IIO_ATTR.dev_attr.attr, \
	&iio_dev_attr_CLKout2_##IIO_ATTR.dev_attr.attr, \
	&iio_dev_attr_CLKout3_##IIO_ATTR.dev_attr.attr, \
	&iio_dev_attr_CLKout4_##IIO_ATTR.dev_attr.attr, \
	&iio_dev_attr_CLKout5_##IIO_ATTR.dev_attr.attr, \
	&iio_dev_attr_CLKout6_##IIO_ATTR.dev_attr.attr, \
	&iio_dev_attr_CLKout7_##IIO_ATTR.dev_attr.attr, \
	&iio_dev_attr_CLKout8_##IIO_ATTR.dev_attr.attr, \
	&iio_dev_attr_CLKout9_##IIO_ATTR.dev_attr.attr, \
	&iio_dev_attr_CLKout10_##IIO_ATTR.dev_attr.attr, \
	&iio_dev_attr_CLKout11_##IIO_ATTR.dev_attr.attr


enum attributes{
	// ATTR_RESET,
	// ATTR_POWERDOWN,
	ALL_CLK_ATTR(ATTR_CLK_PD),	// being expanded for all channels
	ALL_CLK_ATTR(ATTR_CLK_DIV),	// being expanded for all channels
	ALL_CLK_ATTR(ATTR_CLK_TYPE),	// being expanded for all channels
	// ATTR_CLK6_OSCIN,
	// ATTR_CLK7_OSCIN,
	// ATTR_CLK8_OSCIN,
	// ATTR_CLK9_OSCIN,
	ATTR_PLL1_R,
	ATTR_PLL1_N,
	ATTR_PLL2_R,
	ATTR_PLL2_N,
	ATTR_PLL2_P,
	ATTR_VCO_MODE,
	ATTR_CLKIN_SELECT_MODE
};

int rename_iio_attribute(struct attribute *attr, char *name){
	// indio_dev->channel_attr_list
	if(!attr)
		return -1;
	if(!attr->name)
		return -2;
	//kfree(attr->name);
	attr->name = kasprintf(GFP_KERNEL, "%s", name);

	return 0;
}

void lmk04805_inject_register_value(u32 *reg, u32 offset, u32 nbits, u32 value){
	int mask = (((1 << nbits) - 1) << offset);
	/* mask register bits */
	*reg &= ~mask;
	/* write register bits */
	*reg |= (value << offset) & mask;
}

void lmk04805_extract_register_value(u32 reg, u32 offset, u32 nbits, u32 *value){
	int mask = ((1 << nbits) - 1);
	*value = (reg >> offset) & mask;
}

uint32_t lmk04805_get_clk_format(uint32_t reg, int ch){
	return (reg >> (16+4*(ch%4))) & 0x0000000F;
}

void lmk04805_set_clk_format(int ch, uint8_t format, uint32_t* reg){
	*reg = (*reg & ~(0x0000000F << (16+4*(ch%4)))) | ((format&0x0000000F) << (16+4*(ch%4)));
}


struct lmk04805_outputs {
	struct clk_hw hw;
	struct iio_dev *indio_dev;
	unsigned num;
	bool is_enabled;
};

struct lmk04805_state {
	struct spi_device*		spi;
	struct regulator*		reg;
	struct lmk04805_platform_data*	pdata;
	struct lmk04805_outputs		output[LMK04805_NUM_CHAN];
	struct iio_chan_spec		lmk04805_channels[LMK04805_NUM_CHAN];
	struct clk_onecell_data		clk_data;
	struct clk*			clks[LMK04805_NUM_CHAN];

	uint8_t		clk_output_format[LMK04805_NUM_CHAN];
	unsigned long	vcxo_freq;
	unsigned long	vco_freq;
	unsigned long	vco_out_freq;

	struct mutex	lock;
//	/*
//	 * DMA (thus cache coherency maintenance) requires the
//	 * transfer buffers to live in their own cache lines.
//	 */
//	union {	//TODO
//		__be32 d32;
//		u8 d8[4];
//	} data[2] ____cacheline_aligned;

	uint32_t spi_buffer;

};

int lmk04805_spi_read(struct iio_dev *indio_dev, u32 addr, u32 *val)
{
	struct lmk04805_state *st = iio_priv(indio_dev);
	int ret;
	uint32_t address_reg = 31;	// readback address register

	struct spi_transfer address_transfer[] = {
			{
			.tx_buf = &address_reg,
			.len = 4
			}
	};
	struct spi_transfer t[] = {
		{
			.tx_buf = &address_reg,
			.rx_buf = &st->spi_buffer,
			.len = 4
		}
	};

	/* write requested address to readback adress register */
	lmk04805_inject_register_value(&address_reg, 16, 5, addr);
	lmk04805_inject_register_value(&address_reg, 21, 1, 0);  // READBACK_LE = 0
	address_reg = cpu_to_be32(address_reg);

	/* push readback address into the LMK Serial MICROWIRE shift register */
	st->spi->mode = SPI_MODE_0;	 // data bit on rising edge
	st->spi->bits_per_word = 8;
	ret = spi_sync_transfer(st->spi, address_transfer, ARRAY_SIZE(address_transfer));
	if(ret < 0){
		dev_err(&indio_dev->dev, "write failed (%d)", ret);
		return ret;
	}

	/* readback */
	st->spi->mode = SPI_MODE_1;	 // data bit on falling edge
	st->spi->bits_per_word = 8;
	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if(ret < 0){
		dev_err(&indio_dev->dev, "read failed (%d)", ret);
		return ret;
	}

	/* add address bits */
	*val = be32_to_cpu(st->spi_buffer);
	lmk04805_inject_register_value(val, 0, 5, addr);

	return 0;
}

int lmk04805_spi_write(struct iio_dev *indio_dev, u32 val)
{
	struct lmk04805_state *st = iio_priv(indio_dev);
	int ret;
	uint32_t buffer = 0;

	struct spi_transfer t[] = {
		{
			.tx_buf = &buffer,
			.len = 4
		}
	};

	/* write register data to SPI TX buffer */
	buffer = cpu_to_be32(val);

	/* push data into the LMK SPI shift register */
	st->spi->mode = SPI_MODE_0;
	st->spi->bits_per_word = 8;
	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if(ret < 0){
		dev_err(&indio_dev->dev, "write failed (%d)", ret);
		return ret;
	}

	return 0;
}

int lmk04805_read(struct iio_dev *indio_dev, u32 addr, u32 *val)
{
	struct lmk04805_state *st = iio_priv(indio_dev);

	if(addr > 31){
		dev_err(&indio_dev->dev, "read failed - address \"0x%08x\" out of range", addr);
		return -EINVAL;
	}
	else
		*val = st->pdata->reg_map[addr];

	return 0;
};

int lmk04805_write(struct iio_dev *indio_dev, u32 addr, u32 val)
{
	struct lmk04805_state *st = iio_priv(indio_dev);
	int ret;
	u32 reg;

	/* we never write to those registers */
	if( (addr>16 && addr<24) || (addr > 31) ){
		dev_err(&indio_dev->dev, "read failed - address \"0x%08x\" out of range", addr);
		return -EINVAL;
	}

	/* generate register val */
	reg = LMK04805_VALUE(val) | LMK04805_ADDR(addr);

	/* write to SPI device */
	ret = lmk04805_spi_write(indio_dev, reg);
	if(ret < 0){
		dev_err(&indio_dev->dev, "write failed (%d)", ret);
		return ret;
	}

	/* special programming case: write register twice */
	if(addr < 6){
		ret = lmk04805_spi_write(indio_dev, reg);
		if(ret < 0){
			dev_err(&indio_dev->dev, "write failed (%d)", ret);
			return ret;
		}
	}

	/* update local register map */
	st->pdata->reg_map[addr] = reg;

	return 0;
}

int lmk04805_write_all(struct iio_dev *indio_dev, u32 addr, u32 val)
{
	struct lmk04805_state *st = iio_priv(indio_dev);
	int ret = 0;
	u32 reg;
	int i;

	/* we never write to those registers */
	if( (addr>16 && addr<24) || (addr > 31) ){
		dev_err(&indio_dev->dev, "read failed - address \"0x%08x\" out of range", addr);
		return -EINVAL;
	}

	/* generate register val */
	reg = LMK04805_VALUE(val) | LMK04805_ADDR(addr);

	for(i=-1; i<32; i++){
		if(i == 17)
			i=24;
		if(i == -1)
			ret = lmk04805_spi_write(indio_dev, 0x80160140); // perform RESET
		else if( i == addr){
			ret = lmk04805_spi_write(indio_dev, reg);
			if(ret)
				return ret;
			/* update local register map */
			st->pdata->reg_map[i] = reg;
		}
		else
			ret = lmk04805_spi_write(indio_dev, st->pdata->reg_map[i]);
		if(ret)
			return ret;
	}

	return 0;
}

int lmk04805_sync_all_registers(struct iio_dev *indio_dev){
	struct lmk04805_state *st = iio_priv(indio_dev);
	int ret = 0;
	int i;

	for(i=-1; i<32; i++){
		if(i == 17)
			i=24;
		else if(i == -1)
			ret = lmk04805_spi_write(indio_dev, 0x80160140); // perform RESET
		else
			ret = lmk04805_spi_write(indio_dev, st->pdata->reg_map[i]);
		if(ret)
			return ret;
	}

	return 0;
}

static ssize_t lmk04805_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret = 0;
	u32 val = 0;
	u32 reg;
	u8 reg_num;
	u8 ch;
	u8 match;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<LMK04805_NUM_CHAN; ch++){
		if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_PD)){
			match = 1;
			reg_num = LMK04805_GET_CLK_DIVIDER_REG(ch);
			ret = lmk04805_read(indio_dev, reg_num, &reg);
			if(ret == 0)
				lmk04805_extract_register_value(reg, 31, 1, &val);
			break;
		}
		else if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_DIV)){
			match = 1;
			reg_num = LMK04805_GET_CLK_DIVIDER_REG(ch);
			ret = lmk04805_read(indio_dev, reg_num, &reg);
			if(ret == 0)
				lmk04805_extract_register_value(reg, 5, 11, &val);
			break;
		}
		else if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_TYPE)){
			match = 1;
			reg_num = LMK04805_GET_CLK_FORMAT_REG(ch);
			ret = lmk04805_read(indio_dev, reg_num, &reg);
			if(ret == 0)
				lmk04805_extract_register_value(reg, 16 + 4*(ch%4), 4, &val);
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
	case ATTR_PLL1_R:
		reg_num = 27;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret == 0)
			lmk04805_extract_register_value(reg, 6, 14, &val);
		break;
	case ATTR_PLL1_N:
		reg_num = 28;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret == 0)
			lmk04805_extract_register_value(reg, 6, 14, &val);
		break;
	case ATTR_PLL2_R:
		reg_num = 28;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret == 0)
			lmk04805_extract_register_value(reg, 20, 12, &val);
		break;
	case ATTR_PLL2_N:
		reg_num = 30;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret == 0)
			lmk04805_extract_register_value(reg, 5, 18, &val);
		break;
	case ATTR_PLL2_P:
		reg_num = 30;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret == 0)
			lmk04805_extract_register_value(reg, 24, 3, &val);
		break;
	case ATTR_VCO_MODE:
		reg_num = 11;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret == 0)
			lmk04805_extract_register_value(reg, 27, 5, &val);
		break;
	case ATTR_CLKIN_SELECT_MODE:
		reg_num = 13;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret == 0)
			lmk04805_extract_register_value(reg, 9, 3, &val);
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

static ssize_t lmk04805_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret = 0;
	long val;
	u32 reg;
	u8 reg_num;
	u8 ch;
	u8 match;

	ret = kstrtol(buf, 0, &val);
	if(ret)
		return ret;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<LMK04805_NUM_CHAN; ch++){
		if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_PD)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			reg_num = LMK04805_GET_CLK_DIVIDER_REG(ch);
			ret = lmk04805_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04805_inject_register_value(&reg, 31, 1, val);
			ret = lmk04805_write_all(indio_dev, reg_num, reg);
			break;
		}
		else if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_DIV)){
			match = 1;
			if(val<=0 || val>1045){
				ret = -EINVAL;
				break;
			}
			reg_num = LMK04805_GET_CLK_DIVIDER_REG(ch);
			ret = lmk04805_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04805_inject_register_value(&reg, 5, 11, val);
			ret = lmk04805_write_all(indio_dev, reg_num, reg);
			break;
		}
		else if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_TYPE)){
			match = 1;
			if(val<0 || val>14){
				ret = -EINVAL;
				break;
			}
			reg_num = LMK04805_GET_CLK_FORMAT_REG(ch);
			ret = lmk04805_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04805_inject_register_value(&reg, 16 + 4*(ch%4), 4, val);
			ret = lmk04805_write_all(indio_dev, reg_num, reg);
			break;
		}
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		return ret ? ret : len;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case ATTR_PLL1_R:
		if(val<1 || val>16383){
			ret = -EINVAL;
			break;
		}
		reg_num = 27;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret)
			break;
		lmk04805_inject_register_value(&reg, 6, 14, val);
		ret = lmk04805_write_all(indio_dev, reg_num, reg);
		break;
	case ATTR_PLL1_N:
		if(val<1 || val>16383){
			ret = -EINVAL;
			break;
		}
		reg_num = 28;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret)
			break;
		lmk04805_inject_register_value(&reg, 6, 14, val);
		ret = lmk04805_write_all(indio_dev, reg_num, reg);
		break;
	case ATTR_PLL2_R:
		if(val<1 || val>4095){
			ret = -EINVAL;
			break;
		}
		reg_num = 28;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret)
			break;
		lmk04805_inject_register_value(&reg, 20, 12, val);
		ret = lmk04805_write_all(indio_dev, reg_num, reg);
		break;
	case ATTR_PLL2_N:
		if(val<1 || val>262143){
			ret = -EINVAL;
			break;
		}
		reg_num = 30;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret)
			break;
		lmk04805_inject_register_value(&reg, 5, 18, val);
		ret = lmk04805_write_all(indio_dev, reg_num, reg);
		break;
	case ATTR_PLL2_P:
		if(val<2 || val>8){
			ret = -EINVAL;
			break;
		}
		reg_num = 30;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret)
			break;
		lmk04805_inject_register_value(&reg, 24, 3, val);
		ret = lmk04805_write_all(indio_dev, reg_num, reg);
		break;
	case ATTR_VCO_MODE:
		if(val<0 || val>16){
			ret = -EINVAL;
			break;
		}
		switch(val){
			case 1:
			case 4:
			case 7:
			case 9:
			case 10:
			case 12:
			case 13:
			case 14:
				ret = -EINVAL;
				break;
			default:
				break;
		}
		if(ret)
			break;
		reg_num = 11;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret)
			break;
		lmk04805_inject_register_value(&reg, 27, 5, val);
		ret = lmk04805_write_all(indio_dev, reg_num, reg);
		break;
	case ATTR_CLKIN_SELECT_MODE:
		if(val<0 || val==2 || val==5 || val>6){
			ret = -EINVAL;
			break;
		}
		reg_num = 13;
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if(ret)
			break;
		lmk04805_inject_register_value(&reg, 9, 3, val);
		ret = lmk04805_write_all(indio_dev, reg_num, reg);
		break;
	default:
		ret = -ENODEV;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static int lmk04805_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct lmk04805_state *st = iio_priv(indio_dev);
	int ret;
	uint32_t reg;

	mutex_lock(&indio_dev->mlock);
	switch(m){
	case IIO_CHAN_INFO_RAW:
		if(chan->channel == 1){
			ret = -EINVAL;
			break;
		}
		ret = lmk04805_read(indio_dev, LMK04805_GET_CLK_FORMAT_REG(chan->channel), &reg);
		if(ret < 0)
			break;
		if(lmk04805_get_clk_format(reg, chan->channel))
			*val = 0;
		else
			*val = 1;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_FREQUENCY:
		ret = lmk04805_read(indio_dev, LMK04805_GET_CLK_DIVIDER_REG(chan->channel), &reg);
		if(ret < 0)
			break;
		*val = st->vco_out_freq / LMK04805_GET_CLK_DIVIDER(reg);
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
};

static int lmk04805_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct lmk04805_state *st = iio_priv(indio_dev);
	uint32_t reg = 0;
	int ret, tmp, reg_num;

	mutex_lock(&indio_dev->mlock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		reg_num = LMK04805_GET_CLK_FORMAT_REG(chan->channel);
		ret = lmk04805_read(indio_dev, reg_num, &reg);

		if(val == 1)
			lmk04805_set_clk_format(chan->channel, LMK04805_CLK_FORMAT_POWERDOWN, &reg);
		else if(val == 0)
			lmk04805_set_clk_format(chan->channel, st->clk_output_format[chan->channel], &reg);
		else{
			ret = -EINVAL;
			goto end;
		}
		break;
	case IIO_CHAN_INFO_FREQUENCY:
		if (val <= 0) {
			ret = -EINVAL;
			goto end;
		}
		reg_num = LMK04805_GET_CLK_DIVIDER_REG(chan->channel);
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if (ret < 0){
			ret = -EINVAL;
			goto end;
		}

		tmp = st->vco_out_freq / val;
		tmp = clamp(tmp, 1, 1045);
		reg &= ~(0x7FF << 5);
		reg |= LMK04805_SET_CLK_DIVIDER(tmp);
		break;
	default:
		ret = -EINVAL;
		goto end;
	}

	ret = lmk04805_write_all(indio_dev, reg_num, reg);

end:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static int lmk04805_reg_access(struct iio_dev *indio_dev,
			      unsigned reg_num, unsigned writeval,
			      unsigned *readval)
{
	int ret;
	uint32_t reg;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		ret = lmk04805_write(indio_dev, reg_num, writeval);
	} else {
		ret = lmk04805_read(indio_dev, reg_num, &reg);
		if (ret < 0)
			goto out_unlock;
		*readval = reg;
		ret = 0;
	}

out_unlock:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

ALL_CLK_IIO_DEVICE_ATTR(PD, S_IRUGO | S_IWUSR,
			lmk04805_show,
			lmk04805_store,
			ATTR_CLK_PD);

ALL_CLK_IIO_DEVICE_ATTR(DIV, S_IRUGO | S_IWUSR,
			lmk04805_show,
			lmk04805_store,
			ATTR_CLK_DIV);

ALL_CLK_IIO_DEVICE_ATTR(TYPE, S_IRUGO | S_IWUSR,
			lmk04805_show,
			lmk04805_store,
			ATTR_CLK_TYPE);

static IIO_DEVICE_ATTR(PLL1_R, S_IRUGO | S_IWUSR,
			lmk04805_show,
			lmk04805_store,
			ATTR_PLL1_R);

static IIO_DEVICE_ATTR(PLL1_N, S_IRUGO | S_IWUSR,
			lmk04805_show,
			lmk04805_store,
			ATTR_PLL1_N);

static IIO_DEVICE_ATTR(PLL2_R, S_IRUGO | S_IWUSR,
			lmk04805_show,
			lmk04805_store,
			ATTR_PLL2_R);

static IIO_DEVICE_ATTR(PLL2_N, S_IRUGO | S_IWUSR,
			lmk04805_show,
			lmk04805_store,
			ATTR_PLL2_N);

static IIO_DEVICE_ATTR(PLL2_P, S_IRUGO | S_IWUSR,
			lmk04805_show,
			lmk04805_store,
			ATTR_PLL2_P);

static IIO_DEVICE_ATTR(VCO_Mode, S_IRUGO | S_IWUSR,
			lmk04805_show,
			lmk04805_store,
			ATTR_VCO_MODE);

static IIO_DEVICE_ATTR(CLKin_SELECT_MODE, S_IRUGO | S_IWUSR,
			lmk04805_show,
			lmk04805_store,
			ATTR_CLKIN_SELECT_MODE);

static struct attribute *lmk04805_attributes[] = {
	ALL_CLK_IIO_ATTR_REF(PD),
	ALL_CLK_IIO_ATTR_REF(DIV),
	ALL_CLK_IIO_ATTR_REF(TYPE),
	ATTR_REF(PLL1_R),
	ATTR_REF(PLL1_N),
	ATTR_REF(PLL2_R),
	ATTR_REF(PLL2_N),
	ATTR_REF(PLL2_P),
	ATTR_REF(VCO_Mode),
	ATTR_REF(CLKin_SELECT_MODE),
	NULL,
};

static const struct attribute_group lmk04805_attribute_group = {
	.attrs = lmk04805_attributes,
};

static const struct iio_info lmk04805_info = {
	.read_raw = &lmk04805_read_raw,
	.write_raw = &lmk04805_write_raw,
	.debugfs_reg_access = &lmk04805_reg_access,
	.attrs = &lmk04805_attribute_group,
};

#define to_lmk04805_clk_output(_hw) container_of(_hw, struct lmk04805_outputs, hw)

static long lmk04805_get_clk_attr(struct clk_hw *hw, long mask)
{
	struct iio_dev *indio_dev = to_lmk04805_clk_output(hw)->indio_dev;
	int val, ret;
	struct iio_chan_spec chan;

	chan.channel = to_lmk04805_clk_output(hw)->num;
	ret = lmk04805_read_raw(indio_dev, &chan, &val, NULL, mask);
	if (ret == IIO_VAL_INT)
		return val;

	return ret;
}

static long lmk04805_set_clk_attr(struct clk_hw *hw, long mask, unsigned long val)
{
	struct iio_dev *indio_dev = to_lmk04805_clk_output(hw)->indio_dev;
	struct iio_chan_spec chan;

	chan.channel = to_lmk04805_clk_output(hw)->num;

	return lmk04805_write_raw(indio_dev, &chan, val, 0, mask);
}

static unsigned long lmk04805_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	return lmk04805_get_clk_attr(hw, IIO_CHAN_INFO_FREQUENCY);
}

static int lmk04805_clk_is_enabled(struct clk_hw *hw)
{
	return to_lmk04805_clk_output(hw)->is_enabled;
}

static int lmk04805_clk_prepare(struct clk_hw *hw)
{
	return lmk04805_set_clk_attr(hw, IIO_CHAN_INFO_RAW, 0);
}

static void lmk04805_clk_unprepare(struct clk_hw *hw)
{
	lmk04805_set_clk_attr(hw, IIO_CHAN_INFO_RAW, 1);
}

static int lmk04805_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long prate)
{
	return lmk04805_set_clk_attr(hw, IIO_CHAN_INFO_FREQUENCY, rate);
}

static long lmk04805_clk_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *prate)
{
	struct iio_dev *indio_dev = to_lmk04805_clk_output(hw)->indio_dev;
	struct lmk04805_state *st = iio_priv(indio_dev);
	unsigned long tmp;

	if (!rate)
		return 0;

	tmp = DIV_ROUND_CLOSEST(st->vco_out_freq, rate);
	tmp = clamp(tmp, 1UL, 1045UL);

	return st->vco_out_freq / tmp;
}

static const struct clk_ops lmk04805_clk_ops = {
	.recalc_rate = lmk04805_clk_recalc_rate,
	.is_enabled = lmk04805_clk_is_enabled,
	.prepare = lmk04805_clk_prepare,
	.unprepare = lmk04805_clk_unprepare,
	.set_rate = lmk04805_clk_set_rate,
	.round_rate = lmk04805_clk_round_rate,
};

static struct clk *lmk04805_clk_register(struct iio_dev *indio_dev, unsigned num,
				bool is_enabled)
{
	struct lmk04805_state *st = iio_priv(indio_dev);
	struct clk_init_data init;
	struct lmk04805_outputs *output = &st->output[num];
	struct clk *clk;
	char name[SPI_NAME_SIZE + 8];

	sprintf(name, "%s_out%d", indio_dev->name, num);

	init.name = name;
	init.ops = &lmk04805_clk_ops;

	init.num_parents = 0;
	init.flags = CLK_IS_BASIC | CLK_GET_RATE_NOCACHE;
	output->hw.init = &init;
	output->indio_dev = indio_dev;
	output->num = num;
	output->is_enabled = is_enabled;

	/* register the clock */
	clk = clk_register(&st->spi->dev, &output->hw);
	st->clk_data.clks[num] = clk;

	return clk;
}


static int lmk04805_setup(struct iio_dev *indio_dev)
{
	struct lmk04805_state *st = iio_priv(indio_dev);
	struct lmk04805_platform_data *pdata = st->pdata;
	struct lmk04805_channel_spec *chan;
	unsigned long active_mask = 0;
	int addr, i, cnt;


	for(addr=0;addr<32;addr++)
		st->pdata->reg_map[addr] = lmk04805_reg_default[addr];
		//lmk04805_write(indio_dev, addr, lmk04805_reg_default[addr]);

	for(cnt=0;cnt<12;cnt++){
		st->clk_output_format[cnt] = lmk04805_get_clk_format(lmk04805_reg_default[LMK04805_GET_CLK_FORMAT_REG(cnt)],cnt);
		if(!st->clk_output_format[cnt]){
			st->clk_output_format[cnt] = LMK04805_CLK_FORMAT_LVDS;
			//printk(">>>set out%d type\n", cnt);
		}
	}

/*	lmk04805_set_clk_format(0, 0x00, &(lmk04805_reg_default[LMK04805_GET_CLK_FORMAT_REG(0)]));
	printk(">>>REG: 0x%08x\n",lmk04805_reg_default[LMK04805_GET_CLK_FORMAT_REG(0)]);
	lmk04805_set_clk_format(0, 0x04, &(lmk04805_reg_default[LMK04805_GET_CLK_FORMAT_REG(0)]));
	printk(">>>REG: 0x%08x\n",lmk04805_reg_default[LMK04805_GET_CLK_FORMAT_REG(0)]);

	lmk04805_write(indio_dev, LMK04805_GET_CLK_FORMAT_REG(0), lmk04805_reg_default[LMK04805_GET_CLK_FORMAT_REG(0)]);
*/

// TODO: folgende variable Werte müssen die Daten oben manipulieren bevor der LMK initialisiert wird !
	st->vco_freq = (pdata->vcxo_freq * (pdata->EN_PLL2_REF_2X ? 2 : 1)
			/ pdata->PLL2_R) * (pdata->VCO_MUX ? pdata->VCO_DIV : 1)
			* pdata->PLL2_P * pdata->PLL2_N;

	//printk(">>>vcxo_freq=%d, EN_PLL2_REF_2X=%d, PLL2_R=%d, VCO_MUX=%d, VCO_DIV=%d, PLL2_P=%d, PLL2_N=%d, vco_freq=%d\n", pdata->vcxo_freq, pdata->EN_PLL2_REF_2X, pdata->PLL2_R, pdata->VCO_MUX, pdata->VCO_DIV, pdata->PLL2_P, pdata->PLL2_N, st->vco_freq);

	st->vco_out_freq = st->vco_freq / (pdata->VCO_MUX ? pdata->VCO_DIV : 1);
	st->vco_out_freq /= pdata->channels[7].clock_divider;
	st->vco_out_freq *= pdata->channels[7].clock_divider;

	//printk(">>>vco_out_freq=%d\n", st->vco_out_freq);

	// TODO: let's do it that way for all attibutes that are parsed from the devicetree!
	lmk04805_inject_register_value(&st->pdata->reg_map[11], 27, 5, pdata->VCO_MODE);

	st->pdata->reg_map[26] = (st->pdata->reg_map[26] & ~(0x1 << 29)) | ((pdata->EN_PLL2_REF_2X & 0x1) << 29);
	st->pdata->reg_map[27] = (st->pdata->reg_map[27] & ~(0x3FFF << 6)) | ((pdata->PLL1_R & 0x3FFF) << 6);
	st->pdata->reg_map[28] = (st->pdata->reg_map[28] & ~(0x3FFF << 6)) | ((pdata->PLL1_N & 0x3FFF) << 6);
	st->pdata->reg_map[28] = (st->pdata->reg_map[28] & ~(0xFFF << 20)) | ((pdata->PLL2_R & 0xFFF) << 20);
	st->pdata->reg_map[29] = (st->pdata->reg_map[29] & ~(0x3FFFF << 5)) | ((pdata->PLL2_N & 0x3FFFF) << 5);
	st->pdata->reg_map[30] = (st->pdata->reg_map[30] & ~(0x3FFFF << 5)) | ((pdata->PLL2_N & 0x3FFFF) << 5);
	st->pdata->reg_map[30] = (st->pdata->reg_map[30] & ~(0x7 << 24)) | ((pdata->PLL2_P & 0x7) << 24);


	st->clk_data.clks = st->clks;
	st->clk_data.clk_num = LMK04805_NUM_CHAN;

	for (i = 0; i < pdata->num_channels; i++) {
		chan = &pdata->channels[i];
		if (chan->channel_num < LMK04805_NUM_CHAN) {
			__set_bit(chan->channel_num, &active_mask);
			if(pdata->channels[i].clock_divider==0)
				pdata->channels[i].clock_divider=100;

			st->pdata->reg_map[i>>1] = (st->pdata->reg_map[i>>1] & ~(0x7FF << 5)) | ((pdata->channels[i].clock_divider & 0x7FF) << 5);
			//printk(">>>clock_divide=%d\n", pdata->channels[i].clock_divider);
			if(pdata->channels[i].powerdown == 1)
				lmk04805_set_clk_format(i, LMK04805_CLK_FORMAT_POWERDOWN, &st->pdata->reg_map[LMK04805_GET_CLK_FORMAT_REG(i)]);
			else
				lmk04805_set_clk_format(i, st->clk_output_format[i], &st->pdata->reg_map[LMK04805_GET_CLK_FORMAT_REG(i)]);

		}
	}

	lmk04805_sync_all_registers(indio_dev);
	msleep(300);	// give the lmk04805 some time to setup the clocks

	st->clk_data.clks = st->clks;
	st->clk_data.clk_num = LMK04805_NUM_CHAN;

	for (i = 0; i < pdata->num_channels; i++) {
		chan = &pdata->channels[i];
		if (chan->channel_num < LMK04805_NUM_CHAN) {
			struct clk *clk;
			__set_bit(chan->channel_num, &active_mask);

//
//			ret = ad9523_vco_out_map(indio_dev, chan->channel_num,
//					   chan->use_alt_clock_src);

			st->lmk04805_channels[i].type = IIO_ALTVOLTAGE;
			st->lmk04805_channels[i].output = 1;
			st->lmk04805_channels[i].indexed = 1;
			st->lmk04805_channels[i].channel = chan->channel_num;
			//printk("CHANNEL = %d   @ i=%d\n",(st->lmk04805_channels)[i].channel,i);
			st->lmk04805_channels[i].extend_name = chan->extended_name;
			st->lmk04805_channels[i].info_mask_separate =
				BIT(IIO_CHAN_INFO_RAW) |
//				BIT(IIO_CHAN_INFO_PHASE) |
				BIT(IIO_CHAN_INFO_FREQUENCY);

			clk = lmk04805_clk_register(indio_dev, chan->channel_num, !chan->powerdown);
			if (IS_ERR(clk))
				return PTR_ERR(clk);
		}
	}
	of_clk_add_provider(st->spi->dev.of_node, of_clk_src_onecell_get, &st->clk_data);

	return 0;
}

static int lmk04805_parse_dt(struct device *dev, struct lmk04805_state *st){
	struct device_node *np = dev->of_node, *chan_np;
	struct lmk04805_platform_data *pdata;
	struct lmk04805_channel_spec *chan;
	unsigned int cnt = 0;
	const char *str;
	int ret;
	int i;
        uint32_t attr;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return -ENOMEM;
	}

	/* reset reg-map */
	for(i=0;i<32;i++)
		pdata->reg_map[i] = 0;

	/* SPI name */
	strncpy(&pdata->name[0], np->name, SPI_NAME_SIZE - 1);


	/* VCXO frequency */
	ret = of_property_read_u32(np, "lmk,vcxo-freq", &pdata->vcxo_freq);
	if(ret < 0){ printk("lmk04805: error - lmk,vcxo-freq\n"); return ret; }

	/* setting: PLL1_R */
	attr = 0;
	ret = of_property_read_u32(np, "lmk,pll1-r", &attr);
	if(ret < 0){
		printk("lmk04805: warning - lmk,pll1-r=16\n");
		pdata->PLL1_R = 16;
	}
	else{
		if(attr < 1){ pdata->PLL1_R = 1; printk("lmk04805: lmk,pll1-r=1\n"); }
		else if(attr > 16383){ pdata->PLL1_R = 16383; printk("lmk04805: lmk,pll1-r=16383\n"); }
		else{ pdata->PLL1_R = attr; }
	}

	/* setting: PLL1_N */
	attr = 0;
	ret = of_property_read_u32(np, "lmk,pll1-n", &attr);
	if(ret < 0){
		printk("lmk04805: warning - lmk,pll1-n=64\n");
		pdata->PLL1_N = 64;
	}
	else{
		if(attr < 1){ pdata->PLL1_N = 1; printk("lmk04805: lmk,pll1-n=1\n"); }
		else if(attr > 16383){ pdata->PLL1_N = 16383; printk("lmk04805: lmk,pll1-n=16383\n"); }
		else{ pdata->PLL1_N = attr; }
	}

	/* setting: EN_PLL2_REF_2X */
	pdata->EN_PLL2_REF_2X = of_property_read_bool(np, "lmk,en-pll2-ref-2x");

	/* setting: PLL2_R */
	attr = 0;
	ret = of_property_read_u32(np, "lmk,pll2-r", &attr);
	if(ret < 0){
		printk("lmk04805: warning - lmk,pll2-r=4\n");
		pdata->PLL2_R = 4;
	}
	else{
		if(attr < 1){ pdata->PLL2_R = 1; printk("lmk04805: lmk,pll2-r=1\n"); }
		else if(attr > 4095){ pdata->PLL2_R = 4095; printk("lmk04805: lmk,pll2-r=4095\n"); }
		else{ pdata->PLL2_R = attr; }
	}

	/* setting: VCO_MUX */
	pdata->VCO_MUX = of_property_read_bool(np, "lmk,vco-mux");

	/* setting: VCO_DIV */
	attr = 0;
	ret = of_property_read_u32(np, "lmk,vco-div", &attr);
	if(ret < 0){
		printk("lmk04805: warning - lmk,vco-div=2\n");
		pdata->VCO_DIV = 2;
	}
	else{
		if(attr < 2){ pdata->VCO_DIV = 2; printk("lmk04805: lmk,vco-div=2\n"); }
		else if(attr > 8){ pdata->VCO_DIV = 8; printk("lmk04805: lmk,vco-div=8\n"); }
		else{ pdata->VCO_DIV = attr; }
	}
	pdata->VCO_MUX = of_property_read_bool(np, "lmk,vco-mux");

	/* setting: VCO_MODE */
	attr = 0;
	ret = of_property_read_u32(np, "lmk,vco-mode", &attr);
	if(ret < 0){
		printk("lmk04805: warning - lmk,vco-mode=0\n");
		pdata->VCO_MODE = 0;
	}
	else{
		pdata->VCO_MODE = attr;
	}

	/* setting: PLL2_P */
	attr = 0;
	ret = of_property_read_u32(np, "lmk,pll2-p", &attr);
	if(ret < 0){
		printk("lmk04805: warning - lmk,pll2-p=2\n");
		pdata->PLL2_P = 2;
	}
	else{
		if(attr < 2){ pdata->PLL2_P = 2; printk("lmk04805: lmk,pll2-p=2\n"); }
		else if(attr > 8){ pdata->PLL2_P = 8; printk("lmk04805: lmk,pll2-p=8\n"); }
		else{ pdata->PLL2_P = attr; }
	}

	/* setting: PLL2_N */
	attr = 0;
	ret = of_property_read_u32(np, "lmk,pll2-n", &attr);
	if(ret < 0){
		printk("lmk04805: warning - lmk,pll2-n=48\n");
		pdata->PLL2_N = 48;
	}
	else{
		if(attr < 1){ pdata->PLL2_N = 1; printk("lmk04805: lmk,pll2-n=1\n"); }
		else if(attr > 262143){ pdata->PLL2_N = 262143; printk("lmk04805: lmk,pll2-n=262143\n"); }
		else{ pdata->PLL2_N = attr; }
	}

	/* Output Channel Configuration */
	for_each_child_of_node(np, chan_np)
		cnt++;
	pdata->num_channels = cnt;
	pdata->channels = devm_kzalloc(dev, sizeof(*chan) * cnt, GFP_KERNEL);
	if (!pdata->channels) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}
	cnt = 0;
	for_each_child_of_node(np, chan_np){
		/* register/channel */
		ret = of_property_read_u32(chan_np, "reg", &pdata->channels[cnt].channel_num);
		if(ret < 0){
			printk("lmk04805: error - CH[%d] - reg\n", pdata->channels[cnt].channel_num);
			return ret;
		}

		/* channel setting: powerdown */
		pdata->channels[cnt].powerdown = of_property_read_bool(chan_np, "lmk,powerdown");

		/* channel setting: CLKoutX_TYPE */
		attr = 0;
		ret = of_property_read_u32(chan_np, "lmk,out-type", &attr);
		if(ret < 0){
			printk("lmk04805: warning - CH[%d] - lmk,out-type=0 (powerdown)\n", pdata->channels[cnt].channel_num);
			pdata->channels[cnt].out_type = 0;
		}
		else{
			if(attr < 0){ pdata->channels[cnt].out_type = 0; printk("lmk04805: CH[%d] - lmk,out-type=0\n", pdata->channels[cnt].channel_num); }
			else if(attr > 14){ pdata->channels[cnt].out_type = 0; printk("lmk04805: CH[%d] - lmk,out-type=0\n", pdata->channels[cnt].channel_num); }
			else{ pdata->channels[cnt].out_type = attr; }
		}

		/* channel setting: CLKoutX_Y_DIV */
		attr = 0;
		ret = of_property_read_u32(chan_np, "lmk,clock-divider", &attr);
		if(ret < 0){
			printk("lmk04805: warning - CH[%d] - lmk,clock-divider=2\n", pdata->channels[cnt].channel_num);
			pdata->channels[cnt].clock_divider = 2;
		}
		else{
			if(attr < 1){ pdata->channels[cnt].clock_divider = 1; printk("lmk04805: CH[%d] - lmk,clock-divider=1\n", pdata->channels[cnt].channel_num); }
			else if(attr > 1045){ pdata->channels[cnt].clock_divider = 1045; printk("lmk04805: CH[%d] - lmk,clock-divider=1045\n", pdata->channels[cnt].channel_num); }
			else{ pdata->channels[cnt].clock_divider = attr; }
		}

		ret = of_property_read_string(chan_np, "lmk,extended-name", &str);
		if(ret < 0){pr_warning("lmk04805: CH[%d] - extended-name of clock channel not found! Please check devicetree ..\n", pdata->channels[cnt].channel_num);}
		if(ret >= 0){strlcpy(pdata->channels[cnt].extended_name, str, sizeof(pdata->channels[cnt].extended_name));}
		cnt++;
	}

	/* set platform data */
	st->pdata = pdata;

	return 0;
}

static int lmk04805_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct lmk04805_state *st;
	int ret;
	//printk("lmk04805: calling probe()\n");

	/* this device driver needs to be konfigured in the device-tree */
	if(!spi->dev.of_node){
		dev_err(&spi->dev, "no platform data!?\n");
		return -ENODEV;
	}

	/* state: alloc mem then initialize */
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if(!indio_dev)
		return -ENOMEM;
	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;

	// ret = rename_iio_attribute(ATTR_REF(pll1_locked), "pll1_locked1");
	// if(ret){
	// 	if(ret == -1)
	// 		printk("LMK04805: >>>> TEST: NULL Pointer 0");
	// 	if(ret == -2)
	// 		printk("LMK04805: >>>> TEST: NULL Pointer 1");
	// }

	/* parse device-tree */
	//printk("lmk04805: parsing device tree ..\n");
	ret = lmk04805_parse_dt(&spi->dev, st);
	if(ret)
		return ret;

	/* regulator .. ? */
	st->reg = devm_regulator_get(&spi->dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			return ret;
	}

	/* setup iio device */
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = (st->pdata->name[0] != 0) ? st->pdata->name : spi_get_device_id(spi)->name;
	indio_dev->info = &lmk04805_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->lmk04805_channels;
	indio_dev->num_channels = st->pdata->num_channels;

	/* initialize LMK04805 */
	ret = lmk04805_setup(indio_dev);
	if (ret < 0)
		goto error_disable_reg;

	/* register IIO device */
	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	/* valid SPI device? */
	if(&spi->dev == NULL)
		printk("\nlmk04805: &spi->dev = NULL\n\n");

	dev_info(&spi->dev, "probed %s\n", indio_dev->name);
	return 0;

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return ret;
}

static int lmk04805_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct lmk04805_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return 0;
}

static const struct spi_device_id lmk04805_id[] = {	//TODO
	{"lmk04805-1", 0},
	{}
};

MODULE_DEVICE_TABLE(spi, lmk04805_id);

static struct spi_driver lmk04805_driver = {
	.driver = {
		.name	= "lmk04805",
	},
	.probe		= lmk04805_probe,
	.remove		= lmk04805_remove,
	.id_table	= lmk04805_id,
};
module_spi_driver(lmk04805_driver);

MODULE_AUTHOR("Cyril Zwahlen <zwahlen@precisionwave.com>");
MODULE_DESCRIPTION("TI LMK04805");
MODULE_LICENSE("GPL v2");
