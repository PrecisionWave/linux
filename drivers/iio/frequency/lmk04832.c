/*
 * LMK04832 SPI Low Jitter Clock Generator
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

#include <linux/math64.h>


#define DISABLE_IIO_CLOCK_INTERFACE		1

#define LMK04832_NUM_CHAN		14
#define LMK04832_NUM_REG		124

#define LMK04832_ADDR(x)		((x << 8) & ~(0xFF8000FF))
#define LMK04832_VALUE(x)		((x) & 0x000000FF)

#define LMK04832_GET_CLK_PD_REG(ch)						0x102 + 0x008*(ch/2)
#define LMK04832_GET_CLK_FORMAT_REG(ch)				0x107 + 0x008*(ch/2)
#define LMK04832_GET_CLK_DIVIDER_LO_REG(ch)		0x100 + 0x008*(ch/2)
#define LMK04832_GET_CLK_DIVIDER_HI_REG(ch)		0x102 + 0x008*(ch/2)
#define LMK04832_GET_CLK_DELAY_LO_REG(ch)			0x101 + 0x008*(ch/2)
#define LMK04832_GET_CLK_DELAY_HI_REG(ch)			0x102 + 0x008*(ch/2)
#define LMK04832_CLK_FORMAT_POWERDOWN		0x00
#define LMK04832_CLK_FORMAT_LVDS			0x1
#define LMK04832_CLK_FORMAT_LVPECL_1600		0x4

struct lmk04832_outputs {
	struct clk_hw hw;
	struct iio_dev *indio_dev;
	unsigned num;
	bool is_enabled;
};

struct lmk04832_state {
	struct spi_device* spi;
	struct regulator* reg;
	struct clk_onecell_data clk_data;
	struct clk* clks[LMK04832_NUM_CHAN];
	struct lmk04832_outputs output[LMK04832_NUM_CHAN];
	struct iio_chan_spec lmk04832_channels[LMK04832_NUM_CHAN];
	struct mutex lock;

	char name[SPI_NAME_SIZE];
	u8 clk_output_format[LMK04832_NUM_CHAN];
	u8 reg_map[LMK04832_NUM_REG];

	u32 reference_frequency_hz;
	bool reference_ext_intn;
	bool vco_ext_intn;
	u32 pll1_r_divider;
	u32 pll1_n_divider;
	u32 pll2_r_divider;
	u32 pll2_n_divider;
	u32 pll2_n_prescaler;
	bool pll2_osc_doubler_en;
	u32 oscout_sel;
	bool sdrclk_sel_lmx_lmkn;
	bool lo2_lowspur_en;
	bool vcxo_sel_100n_125;


	u64 vcxo_freq;
	u64 vco_freq;
	u64 vco_out_freq;
};

const uint32_t lmk04832_reg_default[LMK04832_NUM_REG] = {
0x000004, 0x000200, 0x000306, 0x000463,
0x0005D1, 0x000670, 0x000C51, 0x000D04,
0x010003, 0x01010A, 0x010260, 0x010304,
0x010410, 0x010500, 0x010601, 0x010705,
0x010803, 0x010908, 0x010A60, 0x010B04,
0x010C10, 0x010D00, 0x010E01, 0x010F05,
0x011008, 0x01110A, 0x011280, 0x011340,
0x011410, 0x011500, 0x011601, 0x011700,
0x011808, 0x01190A, 0x011A80, 0x011B40,
0x011C10, 0x011D00, 0x011E01, 0x011F00,
0x012008, 0x01210A, 0x012280, 0x012340,
0x012410, 0x012500, 0x012601, 0x012700,
0x012808, 0x01290A, 0x012A80, 0x012B40,
0x012C10, 0x012D00, 0x012E01, 0x012F00,
0x013002, 0x01310A, 0x013280, 0x013340,
0x013410, 0x013500, 0x013601, 0x013700,
0x013825, 0x013900, 0x013A0C, 0x013B00,
0x013C00, 0x013D08, 0x013E03, 0x013F07,
0x01400B, 0x014100, 0x014200, 0x014300,
0x014400, 0x014500, 0x014618, 0x014702,
0x014803, 0x014943, 0x014A33, 0x014B02,
0x014C00, 0x014D00, 0x014E00, 0x014F7F,
0x015000, 0x015102, 0x015200, 0x015300,
0x015401, 0x015500, 0x015601, 0x015700,
0x015801, 0x015900, 0x015A0A, 0x015BDA,
0x015C20, 0x015D00, 0x015E1E, 0x015F1B,
0x016000, 0x016101, 0x0162AD, 0x016300,
0x016400, 0x016503, 0x016959, 0x016A20,
0x016B00, 0x016C00, 0x016D00, 0x016E03,
0x017310, 0x017700, 0x018200, 0x018300,
0x016600, 0x016700, 0x016803, 0x055500
};

const uint32_t lmk04832_reg_addr[LMK04832_NUM_REG] = {
	0x000, 0x002, 0x003, 0x004,
	0x005, 0x006, 0x00C, 0x00D,
	0x100, 0x101, 0x102, 0x103,
	0x104, 0x105, 0x106, 0x107,
	0x108, 0x109, 0x10A, 0x10B,
	0x10C, 0x10D, 0x10E, 0x10F,
	0x110, 0x111, 0x112, 0x113,
	0x114, 0x115, 0x116, 0x117,
	0x118, 0x119, 0x11A, 0x11B,
	0x11C, 0x11D, 0x11E, 0x11F,
	0x120, 0x121, 0x122, 0x123,
	0x124, 0x125, 0x126, 0x127,
	0x128, 0x129, 0x12A, 0x12B,
	0x12C, 0x12D, 0x12E, 0x12F,
	0x130, 0x131, 0x132, 0x133,
	0x134, 0x135, 0x136, 0x137,
	0x138, 0x139, 0x13A, 0x13B,
	0x13C, 0x13D, 0x13E, 0x13F,
	0x140, 0x141, 0x142, 0x143,
	0x144, 0x145, 0x146, 0x147,
	0x148, 0x149, 0x14A, 0x14B,
	0x14C, 0x14D, 0x14E, 0x14F,
	0x150, 0x151, 0x152, 0x153,
	0x154, 0x155, 0x156, 0x157,
	0x158, 0x159, 0x15A, 0x15B,
	0x15C, 0x15D, 0x15E, 0x15F,
	0x160, 0x161, 0x162, 0x163,
	0x164, 0x165, 0x169, 0x16A,
	0x16B, 0x16C, 0x16D, 0x16E,
	0x173, 0x177, 0x182, 0x183,
	0x166, 0x167, 0x168, 0x555
	// 0x000, 0x002, 0x003, 0x004,
	// 0x005, 0x006, 0x00C, 0x00D,
	// 0x100, 0x101, 0x102, 0x103,
	// 0x104, 0x105, 0x106, 0x107,
	// 0x108, 0x109, 0x10A, 0x10B,
	// 0x10C, 0x10D, 0x10E, 0x10F,
	// 0x110, 0x111, 0x112, 0x113,
	// 0x114, 0x115, 0x116, 0x117,
	// 0x118, 0x119, 0x11A, 0x11B,
	// 0x11C, 0x11D, 0x11E, 0x11F,
	// 0x120, 0x121, 0x122, 0x123,
	// 0x124, 0x125, 0x126, 0x127,
	// 0x128, 0x129, 0x12A, 0x12B,
	// 0x12C, 0x12D, 0x12E, 0x12F,
	// 0x130, 0x131, 0x132, 0x133,
	// 0x134, 0x135, 0x136, 0x137,
	// 0x138, 0x139, 0x13A, 0x13B,
	// 0x13C, 0x13D, 0x13E, 0x13F,
	// 0x140, 0x141, 0x142, 0x143,
	// 0x144, 0x145, 0x146, 0x147,
	// 0x148, 0x149, 0x14A, 0x14B,
	// 0x14C, 0x14D, 0x14E, 0x14F,
	// 0x150, 0x151, 0x152, 0x153,
	// 0x154, 0x155, 0x156, 0x157,
	// 0x158, 0x159, 0x15A, 0x15B,
	// 0x15C, 0x15D, 0x15E, 0x15F,
	// 0x160, 0x161, 0x162, 0x163,
	// 0x164, 0x165, 0x166, 0x167,
	// 0x168, 0x169, 0x16A, 0x16B,
	// 0x16C, 0x173, 0x177, 0x182,
	// 0x183, 0x184, 0x185, 0x188,
	// 0x555
};

int lmk04832_reg_lut(unsigned int reg){
	switch(reg){
		case 0x000: return 0;
		case 0x002: return 1;
		case 0x003: return 2;
		case 0x004: return 3;
		case 0x005: return 4;
		case 0x006: return 5;
		case 0x00C: return 6;
		case 0x00D: return 7;
		case 0x100: return 8;
		case 0x101: return 9;
		case 0x102: return 10;
		case 0x103: return 11;
		case 0x104: return 12;
		case 0x105: return 13;
		case 0x106: return 14;
		case 0x107: return 15;
		case 0x108: return 16;
		case 0x109: return 17;
		case 0x10A: return 18;
		case 0x10B: return 19;
		case 0x10C: return 20;
		case 0x10D: return 21;
		case 0x10E: return 22;
		case 0x10F: return 23;
		case 0x110: return 24;
		case 0x111: return 25;
		case 0x112: return 26;
		case 0x113: return 27;
		case 0x114: return 28;
		case 0x115: return 29;
		case 0x116: return 30;
		case 0x117: return 31;
		case 0x118: return 32;
		case 0x119: return 33;
		case 0x11A: return 34;
		case 0x11B: return 35;
		case 0x11C: return 36;
		case 0x11D: return 37;
		case 0x11E: return 38;
		case 0x11F: return 39;
		case 0x120: return 40;
		case 0x121: return 41;
		case 0x122: return 42;
		case 0x123: return 43;
		case 0x124: return 44;
		case 0x125: return 45;
		case 0x126: return 46;
		case 0x127: return 47;
		case 0x128: return 48;
		case 0x129: return 49;
		case 0x12A: return 50;
		case 0x12B: return 51;
		case 0x12C: return 52;
		case 0x12D: return 53;
		case 0x12E: return 54;
		case 0x12F: return 55;
		case 0x130: return 56;
		case 0x131: return 57;
		case 0x132: return 58;
		case 0x133: return 59;
		case 0x134: return 60;
		case 0x135: return 61;
		case 0x136: return 62;
		case 0x137: return 63;
		case 0x138: return 64;
		case 0x139: return 65;
		case 0x13A: return 66;
		case 0x13B: return 67;
		case 0x13C: return 68;
		case 0x13D: return 69;
		case 0x13E: return 70;
		case 0x13F: return 71;
		case 0x140: return 72;
		case 0x141: return 73;
		case 0x142: return 74;
		case 0x143: return 75;
		case 0x144: return 76;
		case 0x145: return 77;
		case 0x146: return 78;
		case 0x147: return 79;
		case 0x148: return 80;
		case 0x149: return 81;
		case 0x14A: return 82;
		case 0x14B: return 83;
		case 0x14C: return 84;
		case 0x14D: return 85;
		case 0x14E: return 86;
		case 0x14F: return 87;
		case 0x150: return 88;
		case 0x151: return 89;
		case 0x152: return 90;
		case 0x153: return 91;
		case 0x154: return 92;
		case 0x155: return 93;
		case 0x156: return 94;
		case 0x157: return 95;
		case 0x158: return 96;
		case 0x159: return 97;
		case 0x15A: return 98;
		case 0x15B: return 99;
		case 0x15C: return 100;
		case 0x15D: return 101;
		case 0x15E: return 102;
		case 0x15F: return 103;
		case 0x160: return 104;
		case 0x161: return 105;
		case 0x162: return 106;
		case 0x163: return 107;
		case 0x164: return 108;
		case 0x165: return 109;
		case 0x169: return 110;
		case 0x16A: return 111;
		case 0x16B: return 112;
		case 0x16C: return 113;
		case 0x16D: return 114;
		case 0x16E: return 115;
		case 0x173: return 116;
		case 0x177: return 117;
		case 0x182: return 118;
		case 0x183: return 119;
		case 0x166: return 120;
		case 0x167: return 121;
		case 0x168: return 122;
		case 0x555: return 123;
		// case 0x166: return 110;
		// case 0x167: return 111;
		// case 0x168: return 112;
		// case 0x169: return 113;
		// case 0x16A: return 114;
		// case 0x16B: return 115;
		// case 0x16C: return 116;
		// case 0x173: return 117;
		// case 0x177: return 118;
		// case 0x182: return 119;
		// case 0x183: return 120;
		// case 0x184: return 121;
		// case 0x185: return 122;
		// case 0x188: return 123;
		// case 0x555: return 124;
		default: return -1;
	}
}

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
	ATTR_CLK11_##ATTR, \
	ATTR_CLK12_##ATTR, \
	ATTR_CLK13_##ATTR

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
	static IIO_DEVICE_ATTR(CLKout11_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK11_##ATTR); \
	static IIO_DEVICE_ATTR(CLKout12_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK12_##ATTR); \
	static IIO_DEVICE_ATTR(CLKout13_##IIO_ATTR, RW, SHOW, STORE, ATTR_CLK13_##ATTR);

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
	&iio_dev_attr_CLKout11_##IIO_ATTR.dev_attr.attr, \
	&iio_dev_attr_CLKout12_##IIO_ATTR.dev_attr.attr, \
	&iio_dev_attr_CLKout13_##IIO_ATTR.dev_attr.attr


enum attributes{
	// ATTR_RESET,
	// ATTR_POWERDOWN,
	ALL_CLK_ATTR(ATTR_CLK_PD),	// being expanded for all channels
	ALL_CLK_ATTR(ATTR_CLK_DIV),	// being expanded for all channels
	ALL_CLK_ATTR(ATTR_CLK_TYPE),	// being expanded for all channels
	ALL_CLK_ATTR(ATTR_CLK_DDLY),	// being expanded for all channels
	ATTR_REFERENCE_FREQUENCY_HZ,
	ATTR_REFERENCE_EXT_INTN,
	ATTR_VCO_EXT_INTN,
	ATTR_PLL1_R_DIVIDER,
	ATTR_PLL1_N_DIVIDER,
	ATTR_PLL2_R_DIVIDER,
	ATTR_PLL2_N_DIVIDER,
	ATTR_PLL2_N_PRESCALER,
	ATTR_PLL2_OSC_DOUBLER_EN,
	ATTR_OSCOUT_SEL,
	ATTR_SDRCLK_SEL_LMX_LMKN,
	ATTR_LO2_LOWSPUR_EN,
	ATTR_VCXO_SEL_100N_125,
	ATTR_PLL1_LOCKED,
	ATTR_PLL2_LOCKED,
	ATTR_OSC_FREQUENCY_HZ,
	ATTR_VCO_FREQUENCY_HZ,
	ATTR_SYNC_ALL_REGISTERS
};

void lmk04832_inject_register_value(u32 *reg, u32 offset, u32 nbits, u32 value){
	int mask = (((1 << nbits) - 1) << offset);
	/* mask register bits */
	*reg &= ~mask;
	/* write register bits */
	*reg |= (value << offset) & mask;
}

void lmk04832_extract_register_value(u32 reg, u32 offset, u32 nbits, u32 *value){
	int mask = ((1 << nbits) - 1);
	*value = (reg >> offset) & mask;
}

uint32_t lmk04832_get_clk_format(uint32_t reg, int ch){
	return (reg >> (4*(ch%2))) & 0x0000000F;
}

void lmk04832_set_clk_format(int ch, uint8_t format, uint32_t* reg){
	*reg = (*reg & ~(0x0000000F << (4*(ch%2)))) | ((format&0x0000000F) << (4*(ch%2)));
}

int lmk04832_spi_read(struct iio_dev *indio_dev, u32 addr, u32 *val)
{
	struct lmk04832_state *st = iio_priv(indio_dev);
	int ret;
	uint32_t tx_buffer = 0;
	uint32_t rx_buffer = 0;

	struct spi_transfer t[] = {
		{
			.tx_buf = &tx_buffer,
			.rx_buf = &rx_buffer,
			.len = 3
		}
	};

	/* write requested address to SPI write buffer */
	lmk04832_inject_register_value(&tx_buffer, 23, 1, 1);  // set R/W bit to 1 (read operation)
	lmk04832_inject_register_value(&tx_buffer, 8, 15, addr);
	tx_buffer = cpu_to_be32(tx_buffer << 8);

	/* SPI write/read operation */
	st->spi->mode = SPI_MODE_0;
	st->spi->bits_per_word = 8;
	ret = spi_sync_transfer(st->spi, t, 1);
	if(ret < 0){
		dev_err(&indio_dev->dev, "write/read failed (%d)", ret);
		return ret;
	}

	/* get readback value */
	*val = (be32_to_cpu(rx_buffer) >> 8);

	return 0;
}

int lmk04832_spi_write(struct iio_dev *indio_dev, u32 val)
{
	struct lmk04832_state *st = iio_priv(indio_dev);
	int ret;
	uint32_t tx_buffer = 0;

	struct spi_transfer t[] = {
		{
			.tx_buf = &tx_buffer,
			.len = 3
		}
	};

	lmk04832_inject_register_value(&val, 23, 1, 0);  // set R/W bit to 0 (write operation)

	/* write register data to SPI TX buffer */
	tx_buffer = cpu_to_be32(val << 8);

	/* push data into the LMK SPI shift register */
	st->spi->mode = SPI_MODE_0;
	st->spi->bits_per_word = 8;
	ret = spi_sync_transfer(st->spi, t, 1);
	if(ret < 0){
		dev_err(&indio_dev->dev, "write failed (%d)", ret);
		return ret;
	}

	return 0;
}

int lmk04832_read(struct iio_dev *indio_dev, u32 addr, u32 *val)
{
	struct lmk04832_state *st = iio_priv(indio_dev);
	int i;

	i = lmk04832_reg_lut(addr);
	if(i < 0)
		return -EINVAL;

	*val = st->reg_map[i];

	return 0;
};

/* update the value of one register
 */
int lmk04832_write(struct iio_dev *indio_dev, u32 addr, u32 val)
{
	struct lmk04832_state *st = iio_priv(indio_dev);
	int ret;
	u32 reg;
	int i;

	/* register boundary check */
	i = lmk04832_reg_lut(addr);
	if(i < 0)
		return -EINVAL;

	/* generate register val */
	reg = LMK04832_ADDR(addr) | LMK04832_VALUE(val);

	/* write to SPI device */
	ret = lmk04832_spi_write(indio_dev, reg);
	if(ret < 0){
		dev_err(&indio_dev->dev, "write failed (%d)", ret);
		return ret;
	}
	st->reg_map[i] = LMK04832_VALUE(val);  // update local register map

	return 0;
}

/* update the value of one register, but write all
 * registers in accending order to the SPI bus
 */
int lmk04832_write_all(struct iio_dev *indio_dev, u32 addr, u32 val)
{
	struct lmk04832_state *st = iio_priv(indio_dev);
	int ret;
	u32 reg;
	int i;

	/* register boundary check */
	i = lmk04832_reg_lut(addr);
	if(i < 0)
		return -EINVAL;

	/* set reset bit */
	reg = LMK04832_ADDR(0x000) | LMK04832_VALUE(0x90);
	ret = lmk04832_spi_write(indio_dev, reg);
	if(ret < 0){
		dev_err(&indio_dev->dev, "write failed (%d)", ret);
		return ret;
	}

	/* generate register val */
	reg =  LMK04832_ADDR(addr) | LMK04832_VALUE(val);

	/* write all registers */
	for(i=0; i<LMK04832_NUM_REG; i++){
		/* write register that has changed */
		if(lmk04832_reg_addr[i] == addr){
			ret = lmk04832_spi_write(indio_dev, reg);
			if(ret < 0){
				dev_err(&indio_dev->dev, "write failed (%d)", ret);
				return ret;
			}
			st->reg_map[i] = LMK04832_VALUE(val);  // update local register map
		}
		/* write all other registers */
		else{
			/* write to SPI device */
			ret = lmk04832_spi_write(indio_dev, st->reg_map[i]);
			if(ret < 0){
				dev_err(&indio_dev->dev, "write failed (%d)", ret);
				return ret;
			}
		}
	}

	return 0;
}

/* this is used to initialize the chip
 * in the setup function (kernel boot)
 */
int lmk04832_sync_all_registers(struct iio_dev *indio_dev){
	struct lmk04832_state *st = iio_priv(indio_dev);
	int ret = 0;
	int i;

	/* set reset bit */
	ret = lmk04832_write(indio_dev, 0x000, 0x90);
	if(ret)
		return ret;

	/* write all registers */
	for(i=0; i<LMK04832_NUM_REG; i++){
		ret = lmk04832_write(indio_dev, lmk04832_reg_addr[i], st->reg_map[i]);
		if(ret)
			return ret;
	}

	return 0;
}

// int lmk04832_update_regmap(struct iio_dev *indio_dev, u32 addr, u32 val){
// 	int i;
//
// 	/* register boundary check */
// 	i = lmk04832_reg_lut(addr);
// 	if(i < 0)
// 		return -EINVAL;
//
// 	st->reg_map[i] = LMK04832_VALUE(val);  // update local register map
// }

int lmk04832_sync_pll2(struct iio_dev *indio_dev){
	struct lmk04832_state *st = iio_priv(indio_dev);
	int i, ret;
	/* write PLL2_N_DIVIDER registers to Sync PLL2 */
	i = lmk04832_reg_lut(0x166);
	if( (ret = lmk04832_write(indio_dev, 0x166, st->reg_map[i])) )
		return ret;
	i = lmk04832_reg_lut(0x167);
	if( (ret = lmk04832_write(indio_dev, 0x167, st->reg_map[i])) )
		return ret;
	i = lmk04832_reg_lut(0x168);
	if( (ret = lmk04832_write(indio_dev, 0x168, st->reg_map[i])) )
		return ret;
	return 0;
}

int lmk04832_update_vco(struct iio_dev *indio_dev, u32 ref_freq){
	struct lmk04832_state *st = iio_priv(indio_dev);
	int ret;
	u32 reg, val, fvco;
	u64 temp64;

	/* external VCO in use: do not set VCO_MUX */
	if(st->vco_ext_intn)
		return 0;

	temp64 = ref_freq;
	temp64 *= st->pll1_n_divider;
	temp64 *= (st->pll2_osc_doubler_en+1);
	temp64 *= st->pll2_n_divider;
	temp64 *= st->pll2_n_prescaler;
	temp64 = div_u64(temp64, st->pll1_r_divider);
	temp64 = div_u64(temp64, st->pll2_r_divider);
	fvco = temp64;

	if(fvco > 2762500000U){
		/* set VCO-1 */
		if( (ret = lmk04832_read(indio_dev, 0x138, &reg)) )
			return ret;
		lmk04832_extract_register_value(reg, 5, 2, &val);
		if(val != 1){
			lmk04832_inject_register_value(&reg, 5, 2, 1);  // VCO_MUX:	0=VCO-0, 1=VCO-1, 2=Fin1/CLKin1, 3=Reserved
			if( (ret = lmk04832_write(indio_dev, 0x138, reg)) )
				return ret;
		}
	}
	else{
		/* set VCO-0 */
		if( (ret = lmk04832_read(indio_dev, 0x138, &reg)) )
			return ret;
		lmk04832_extract_register_value(reg, 5, 2, &val);
		if(val != 0){
			lmk04832_inject_register_value(&reg, 5, 2, 0);  // VCO_MUX:	0=VCO-0, 1=VCO-1, 2=Fin1/CLKin1, 3=Reserved
			if( (ret = lmk04832_write(indio_dev, 0x138, reg)) )
				return ret;
		}
	}
	return 0;
}

static ssize_t lmk04832_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct lmk04832_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret = 0;
	u32 val = 0;
	u32 temp;
	u64 temp64;
	u32 reg;
	u32 reg_num;
	u8 ch;
	u8 match;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<LMK04832_NUM_CHAN; ch++){
		if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_PD)){
			match = 1;
			/* get CLKoutX_Y_PD */
			reg_num = LMK04832_GET_CLK_PD_REG(ch);
			ret = lmk04832_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04832_extract_register_value(reg, 7, 1, &val);
			break;
		}
		else if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_DIV)){
			match = 1;
			/* get CLKoutX_Y_DIV (low bits) */
			reg_num = LMK04832_GET_CLK_DIVIDER_LO_REG(ch);
			ret = lmk04832_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04832_extract_register_value(reg, 0, 8, &val);
			/* get CLKoutX_Y_DIV (high bits) */
			reg_num = LMK04832_GET_CLK_DIVIDER_HI_REG(ch);
			ret = lmk04832_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04832_extract_register_value(reg, 0, 2, &temp);
			val |= (temp << 8);
			break;
		}
		else if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_TYPE)){
			match = 1;
			/* get CLKoutX_FMT */
			reg_num = LMK04832_GET_CLK_FORMAT_REG(ch);
			ret = lmk04832_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04832_extract_register_value(reg, 4*(ch%2), 4, &val);
			break;
		}
		else if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_DDLY)){
			match = 1;
			/* get CLKoutX_Y_DDLY (low bits) */
			reg_num = LMK04832_GET_CLK_DELAY_LO_REG(ch);
			ret = lmk04832_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04832_extract_register_value(reg, 0, 8, &val);
			/* set CLKoutX_Y_DDLY (high bits) */
			reg_num = LMK04832_GET_CLK_DELAY_HI_REG(ch);
			ret = lmk04832_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04832_extract_register_value(reg, 2, 2, &temp);
			val |= (temp << 8);
			break;
		}
		// else if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_DDLY)){
		// 	match = 1;
		// 	/* get CLKoutX_Y_DDLY (low bits) */
		// 	reg_num = LMK04832_GET_CLK_DELAY_LO_REG(ch);
		// 	ret = lmk04832_read(indio_dev, reg_num, &reg);
		// 	if(ret)
		// 		break;
		// 	lmk04832_extract_register_value(reg, 0, 8, &val);
		// 	/* set CLKoutX_Y_DDLY (high bits) */
		// 	reg_num = LMK04832_GET_CLK_DELAY_HI_REG(ch);
		// 	ret = lmk04832_read(indio_dev, reg_num, &reg);
		// 	if(ret)
		// 		break;
		// 	lmk04832_extract_register_value(reg, 2, 2, &temp);
		// 	val |= (temp << 8);
		// 	break;
		// }
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		if(ret==0)
			ret = sprintf(buf, "%d\n", val);
		return ret;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case ATTR_REFERENCE_FREQUENCY_HZ:
		val = st->reference_frequency_hz;
		break;
	case ATTR_REFERENCE_EXT_INTN:
		val = st->reference_ext_intn;
		break;
	case ATTR_VCO_EXT_INTN:
		val = st->vco_ext_intn;
		break;
	case ATTR_PLL1_R_DIVIDER:
		val = st->pll1_r_divider;
		break;
	case ATTR_PLL1_N_DIVIDER:
		val = st->pll1_n_divider;
		break;
	case ATTR_PLL2_R_DIVIDER:
		val = st->pll2_r_divider;
		break;
	case ATTR_PLL2_N_DIVIDER:
		val = st->pll2_n_divider;
		break;
	case ATTR_PLL2_N_PRESCALER:
		val = st->pll2_n_prescaler;
		break;
	case ATTR_PLL2_OSC_DOUBLER_EN:
		val = st->pll2_osc_doubler_en;
		break;
	case ATTR_OSCOUT_SEL:
		val = st->oscout_sel;
		break;
	case ATTR_SDRCLK_SEL_LMX_LMKN:
		val = st->sdrclk_sel_lmx_lmkn;
		break;
	case ATTR_LO2_LOWSPUR_EN:
		val = st->lo2_lowspur_en;
		break;
	case ATTR_VCXO_SEL_100N_125:
		val = st->vcxo_sel_100n_125;
		break;
	case ATTR_PLL1_LOCKED:
		reg_num = 0x183;
		ret = lmk04832_spi_read(indio_dev, reg_num, &reg);
		if(ret)
			break;
		lmk04832_extract_register_value(reg, 2, 1, &val);  // RB_PLL1_LD
		break;
	case ATTR_PLL2_LOCKED:
		reg_num = 0x15F;
		ret = lmk04832_read(indio_dev, reg_num, &reg);
		if(ret)
			break;
		lmk04832_inject_register_value(&reg, 3, 5, 2);	// set PLL1_LD_MUX to 2 before readback RB_PLL2_LD
		ret = lmk04832_write(indio_dev, reg_num, reg);
		if(ret)
			break;
		reg_num = 0x183;
		ret = lmk04832_spi_read(indio_dev, reg_num, &reg);
		if(ret)
			break;
		lmk04832_extract_register_value(reg, 0, 1, &val);  // RB_PLL2_LD
		reg_num = 0x15F;
		ret = lmk04832_read(indio_dev, reg_num, &reg);
		if(ret)
			break;
		lmk04832_inject_register_value(&reg, 3, 5, 3);	// set PLL1_LD_MUX back to 3
		ret = lmk04832_write(indio_dev, reg_num, reg);
		break;
	case ATTR_OSC_FREQUENCY_HZ:
		val = st->reference_frequency_hz*st->pll1_n_divider/st->pll1_r_divider;
		break;
	case ATTR_VCO_FREQUENCY_HZ:
		temp64 = st->reference_frequency_hz;
		temp64 *= st->pll1_n_divider;
		temp64 *= (st->pll2_osc_doubler_en+1);
		temp64 *= st->pll2_n_divider;
		temp64 *= st->pll2_n_prescaler;
		temp64 = div_u64(temp64, st->pll1_r_divider);
		temp64 = div_u64(temp64, st->pll2_r_divider);
		val = temp64;
		// val = st->reference_frequency_hz*st->pll1_n_divider*(st->pll2_osc_doubler_en+1)*st->pll2_n_divider*st->pll2_n_prescaler/st->pll1_r_divider/st->pll2_r_divider;
		break;
	case ATTR_SYNC_ALL_REGISTERS:
		ret = -ENODEV;
		break;
	default:
		ret = -ENODEV;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0)
			ret = sprintf(buf, "%u\n", val);

	return ret;
}

int lmk04832_update_reference_frequency(struct iio_dev *indio_dev, u32 val){
	return lmk04832_update_vco(indio_dev, val);
}

int lmk04832_update_reference_ext_intn(struct iio_dev *indio_dev, u32 val){
	struct lmk04832_state *st = iio_priv(indio_dev);
	u32 reg;
	u32 reg_num;
	int ret;
	/* set CLKin1_DEMUX & CLKin_SEL_MANUAL */
	reg_num = 0x147;
	ret = lmk04832_read(indio_dev, reg_num, &reg);
	if(ret)
		return ret;
	if(val > 0){
		lmk04832_inject_register_value(&reg, 2, 2, 2);  // CLKin1_DEMUX:	0=Fin, 1=Feedback-MUX, 2=PLL1, 3=Off
		lmk04832_inject_register_value(&reg, 4, 2, 1);  // CLKin_SEL_MANUAL:	0=CLKin0, 1=CLKin1, 2=CLKin2, 3=Holdover
	}
	else{
		lmk04832_inject_register_value(&reg, 0, 2, 2);  // CLKin0_DEMUX:	0=SYSREF-MUX, 1=Reserved, 2=PLL1, 3=Off
		lmk04832_inject_register_value(&reg, 4, 2, 0);  // CLKin_SEL_MANUAL:	0=CLKin0, 1=CLKin1, 2=CLKin2, 3=Holdover
		if(st->vco_ext_intn == 0)
			lmk04832_inject_register_value(&reg, 2, 2, 1);  // CLKin1_DEMUX:	0=Fin, 1=Feedback-MUX, 2=PLL1, 3=Off
	}
	ret = lmk04832_write(indio_dev, reg_num, reg);
	return ret;
}

int lmk04832_update_vco_ext_intn(struct iio_dev *indio_dev, u32 val){
	struct lmk04832_state *st = iio_priv(indio_dev);
	u32 reg;
	u32 reg_num;
	int ret;
	if(val > 0){
		/* set VCO_MUX */
		reg_num = 0x138;
		ret = lmk04832_read(indio_dev, reg_num, &reg);
		if(ret)
			return ret;
		lmk04832_inject_register_value(&reg, 5, 2, 2);  // VCO_MUX:	0=VCO-0, 1=VCO-1, 2=Fin1/CLKin1, 3=Reserved
		ret = lmk04832_write(indio_dev, reg_num, reg);
		if(ret)
			return ret;
		/* set CLKin1_DEMUX */
		reg_num = 0x147;
		ret = lmk04832_read(indio_dev, reg_num, &reg);
		if(ret)
			return ret;
		lmk04832_inject_register_value(&reg, 2, 2, 0);  // CLKin1_DEMUX:	0=Fin, 1=Feedback-MUX, 2=PLL1, 3=Off
		ret = lmk04832_write(indio_dev, reg_num, reg);
	}
	else{
		/* set VCO_MUX */
		reg_num = 0x138;
		ret = lmk04832_update_vco(indio_dev, st->reference_frequency_hz);
		if(ret)
			return ret;
		/* set CLKin1_DEMUX */
		if(st->reference_ext_intn == 0){
			reg_num = 0x147;
			ret = lmk04832_read(indio_dev, reg_num, &reg);
			if(ret)
				return ret;
			lmk04832_inject_register_value(&reg, 2, 2, 1);  // CLKin1_DEMUX:	0=Fin, 1=Feedback-MUX, 2=PLL1, 3=Off
			ret = lmk04832_write(indio_dev, reg_num, reg);
		}
	}
	return ret;
}

int lmk04832_update_pll1_r_divider(struct iio_dev *indio_dev, u32 val){
	struct lmk04832_state *st = iio_priv(indio_dev);
	u32 reg;
	int ret;
	/* write high bits of CLKin0_R, CLKin1_R, CLKin2_R */
	reg = (val >> 8) & 0x3F;
	if( (ret = lmk04832_write(indio_dev, 0x153, reg)) )
		return ret;
	if( (ret = lmk04832_write(indio_dev, 0x155, reg)) )
		return ret;
	if( (ret = lmk04832_write(indio_dev, 0x157, reg)) )
		return ret;
	/* write low bits of CLKin0_R, CLKin1_R, CLKin2_R */
	reg = val & 0xFF;
	if( (ret = lmk04832_write(indio_dev, 0x154, reg)) )
		return ret;
	if( (ret = lmk04832_write(indio_dev, 0x156, reg)) )
		return ret;
	ret = lmk04832_write(indio_dev, 0x158, reg);
	if(ret)
		return ret;
	ret = lmk04832_update_vco(indio_dev, st->reference_frequency_hz);
	return ret;
}

int lmk04832_update_pll1_n_divider(struct iio_dev *indio_dev, u32 val){
	struct lmk04832_state *st = iio_priv(indio_dev);
	u32 reg;
	int ret;
	/* write high bits of PLL1_N */
	reg = (val >> 8) & 0x3F;
	if( (ret = lmk04832_write(indio_dev, 0x159, reg)) )
		return ret;
	/* write low bits of PLL1_N */
	reg = val & 0xFF;
	ret = lmk04832_write(indio_dev, 0x15A, reg);
	if(ret)
		return ret;
	ret = lmk04832_update_vco(indio_dev, st->reference_frequency_hz);
	return ret;
}

int lmk04832_update_pll2_r_divider(struct iio_dev *indio_dev, u32 val){
	struct lmk04832_state *st = iio_priv(indio_dev);
	u32 reg;
	int ret;
	/* write high bits of PLL2_R */
	reg = (val >> 8) & 0x0F;
	if( (ret = lmk04832_write(indio_dev, 0x160, reg)) )
		return ret;
	/* write low bits of PLL2_R */
	reg = val & 0xFF;
	ret = lmk04832_write(indio_dev, 0x161, reg);
	if(ret)
		return ret;
	ret = lmk04832_update_vco(indio_dev, st->reference_frequency_hz);
	return ret;
}

int lmk04832_update_pll2_n_divider(struct iio_dev *indio_dev, u32 val){
	struct lmk04832_state *st = iio_priv(indio_dev);
	u32 reg;
	int ret;
	/* write high bits of PLL2_N */
	reg = (val >> 16) & 0x03;
	if( (ret = lmk04832_write(indio_dev, 0x166, reg)) )
		return ret;
	/* write center bits of PLL2_N */
	reg = (val >> 8) & 0xFF;
	if( (ret = lmk04832_write(indio_dev, 0x167, reg)) )
		return ret;
	/* write low bits of PLL2_N */
	reg = val & 0xFF;
	ret = lmk04832_write(indio_dev, 0x168, reg);
	if(ret)
		return ret;
	ret = lmk04832_update_vco(indio_dev, st->reference_frequency_hz);
	return ret;
}

int lmk04832_update_pll2_n_prescaler(struct iio_dev *indio_dev, u32 val){
	struct lmk04832_state *st = iio_priv(indio_dev);
	u32 reg;
	u32 reg_num;
	int ret;
	/* set PLL2_P */
	reg_num = 0x162;
	ret = lmk04832_read(indio_dev, reg_num, &reg);
	if(ret)
		return ret;
	if(val == 8)
		lmk04832_inject_register_value(&reg, 5, 3, 0);  // prescaler value '8' equals register value '0'
	else
		lmk04832_inject_register_value(&reg, 5, 3, val);
	ret = lmk04832_write(indio_dev, reg_num, reg);
	if(ret)
		return ret;
	ret = lmk04832_update_vco(indio_dev, st->reference_frequency_hz);
	return ret;
}

int lmk04832_update_pll2_osc_doubler_en(struct iio_dev *indio_dev, u32 val){
	struct lmk04832_state *st = iio_priv(indio_dev);
	u32 reg;
	u32 reg_num;
	int ret;
	/* set PLL2_REF_2X_EN */
	reg_num = 0x162;
	ret = lmk04832_read(indio_dev, reg_num, &reg);
	if(ret)
		return ret;
	lmk04832_inject_register_value(&reg, 0, 1, val);
	ret = lmk04832_write(indio_dev, reg_num, reg);
	if(ret)
		return ret;
	ret = lmk04832_update_vco(indio_dev, st->reference_frequency_hz);
	return ret;
}

int lmk04832_update_oscout_sel(struct iio_dev *indio_dev, u32 val){
	u32 reg;
	u32 reg_num;
	int ret;

	reg_num = 0x138;
	ret = lmk04832_read(indio_dev, reg_num, &reg);
	if(ret)
		return ret;
	switch(val){
		/* powerdown */
	case 0:
		lmk04832_inject_register_value(&reg, 0, 4, 0);  // OSCout_FMT: 0=Power down, 1=LVDS;
		break;
		/* OSCout = Buffered OSCin */
	case 1:
		lmk04832_inject_register_value(&reg, 0, 4, 1);  // OSCout_FMT: 0=Power down, 1=LVDS;
		lmk04832_inject_register_value(&reg, 4, 1, 0);  // OSCout_MUX: 0=Buffered OSCin, 1=Feedback Mux
		break;
		/* OSCout = Feedback Mux */
	case 2:
		lmk04832_inject_register_value(&reg, 0, 4, 1);  // OSCout_FMT: 0=Power down, 1=LVDS;
		lmk04832_inject_register_value(&reg, 4, 1, 1);  // OSCout_MUX: 0=Buffered OSCin, 1=Feedback Mux
		break;
	default:
		break;
	}
	ret = lmk04832_write(indio_dev, reg_num, reg);
	return ret;
}

int lmk04832_update_sdrclk_sel_lmx_lmkn(struct iio_dev *indio_dev, u32 val){
	u32 reg;
	u32 reg_num;
	int ret;

	reg_num = 0x149;
	ret = lmk04832_read(indio_dev, reg_num, &reg);
	if(ret)
		return ret;
	if(val)
		lmk04832_inject_register_value(&reg, 0, 3, 4);  // CLKin_SEL0_TYPE: 3=output (push-pull), 4=output inverted (push-pull) --> logic 1
	else
		lmk04832_inject_register_value(&reg, 0, 3, 3);  // CLKin_SEL0_TYPE: 3=output (push-pull), 4=output (push-pull) --> logic 0
	ret = lmk04832_write(indio_dev, reg_num, reg);
	return ret;
}

int lmk04832_update_lo2_lowspur_en(struct iio_dev *indio_dev, u32 val){
	u32 reg;
	u32 reg_num;
	int ret;

	reg_num = 0x148;
	ret = lmk04832_read(indio_dev, reg_num, &reg);
	if(ret)
		return ret;
	if(val)
		lmk04832_inject_register_value(&reg, 0, 3, 4);  // CLKin_SEL0_TYPE: 3=output (push-pull), 4=output inverted (push-pull) --> logic 1
	else
		lmk04832_inject_register_value(&reg, 0, 3, 3);  // CLKin_SEL0_TYPE: 3=output (push-pull), 4=output (push-pull) --> logic 0
	ret = lmk04832_write(indio_dev, reg_num, reg);
	return ret;
}

int lmk04832_update_vcxo_sel_100n_125(struct iio_dev *indio_dev, u32 val){
	u32 reg;
	u32 reg_num;
	int ret;

	reg_num = 0x16E;
	ret = lmk04832_read(indio_dev, reg_num, &reg);
	if(ret)
		return ret;
	if(val)
		lmk04832_inject_register_value(&reg, 0, 3, 4);  // PLL2_LD_TYPE: 3=output (push-pull), 4=output inverted (push-pull) --> logic 1
	else
		lmk04832_inject_register_value(&reg, 0, 3, 3);  // PLL2_LD_TYPE: 3=output (push-pull), 4=output (push-pull) --> logic 0
	ret = lmk04832_write(indio_dev, reg_num, reg);
	return ret;
}

static ssize_t lmk04832_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct lmk04832_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret = 0;
	unsigned long val;
	u32 reg;
	u32 reg_num;
	u8 ch;
	u8 match;

	ret = kstrtoul(buf, 0, &val);
	if(ret)
		return ret;

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<LMK04832_NUM_CHAN; ch++){
		if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_PD)){
			match = 1;
			if(val>1){
				ret = -EINVAL;
				break;
			}
			/* set CLKoutX_Y_PD */
			reg_num = LMK04832_GET_CLK_PD_REG(ch);
			ret = lmk04832_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04832_inject_register_value(&reg, 7, 1, val);
			ret = lmk04832_write(indio_dev, reg_num, reg);
			break;
		}
		else if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_DIV)){
			match = 1;
			if(val<1 || val>1023){
				ret = -EINVAL;
				break;
			}
			/* set CLKoutX_Y_DIV (low bits) */
			reg_num = LMK04832_GET_CLK_DIVIDER_LO_REG(ch);
			ret = lmk04832_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04832_inject_register_value(&reg, 0, 8, val);
			ret = lmk04832_write(indio_dev, reg_num, reg);
			if(ret)
				break;
			/* set CLKoutX_Y_DIV (high bits) */
			reg_num = LMK04832_GET_CLK_DIVIDER_HI_REG(ch);
			ret = lmk04832_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04832_inject_register_value(&reg, 0, 2, (val>>8));
			ret = lmk04832_write(indio_dev, reg_num, reg);
			break;
		}
		else if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_TYPE)){
			match = 1;
			if(val>15){
				ret = -EINVAL;
				break;
			}
			/* set CLKoutX_FMT */
			reg_num = LMK04832_GET_CLK_FORMAT_REG(ch);
			ret = lmk04832_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04832_inject_register_value(&reg, 4*(ch%2), 4, val);
			ret = lmk04832_write(indio_dev, reg_num, reg);
			break;
		}
		else if((u32)this_attr->address == CLK_ATTR(ch, ATTR_CLK_DDLY)){
			match = 1;
			if(val<1 || val>1023){
				ret = -EINVAL;
				break;
			}
			/* set CLKoutX_Y_DDLY (low bits) */
			reg_num = LMK04832_GET_CLK_DELAY_LO_REG(ch);
			ret = lmk04832_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04832_inject_register_value(&reg, 0, 8, val);
			ret = lmk04832_write(indio_dev, reg_num, reg);
			if(ret)
				break;
			/* set CLKoutX_Y_DDLY (high bits) */
			reg_num = LMK04832_GET_CLK_DELAY_HI_REG(ch);
			ret = lmk04832_read(indio_dev, reg_num, &reg);
			if(ret)
				break;
			lmk04832_inject_register_value(&reg, 2, 2, (val>>8));
			ret = lmk04832_write(indio_dev, reg_num, reg);
			break;
		}
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		return ret ? ret : len;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case ATTR_REFERENCE_FREQUENCY_HZ:
		// TODO:
		// if(val > XXXX){
		// 	ret = -EINVAL;
		// 	break;
		// }
		ret = lmk04832_update_reference_frequency(indio_dev, val);
		if(ret)
			break;
		/* sync PLL2 */
		ret = lmk04832_sync_pll2(indio_dev);
		st->reference_frequency_hz = val;
		break;
	case ATTR_REFERENCE_EXT_INTN:
		if(val>1){
			ret = -EINVAL;
			break;
		}
		ret = lmk04832_update_reference_ext_intn(indio_dev, val);
		if(ret)
			break;
		st->reference_ext_intn = val;
		break;
	case ATTR_VCO_EXT_INTN:
		if(val>1){
			ret = -EINVAL;
			break;
		}
		ret = lmk04832_update_vco_ext_intn(indio_dev, val);
		if(ret)
			break;
		/* sync PLL2 */
		lmk04832_sync_pll2(indio_dev);
		st->vco_ext_intn = val;
		break;
	case ATTR_PLL1_R_DIVIDER:
		if(val<1 || val>16383){
			ret = -EINVAL;
			break;
		}
		ret = lmk04832_update_pll1_r_divider(indio_dev, val);
		if(ret)
			break;
		/* sync PLL2 */
		lmk04832_sync_pll2(indio_dev);
		st->pll1_r_divider = val;
		break;
	case ATTR_PLL1_N_DIVIDER:
		if(val<1 || val>16383){
			ret = -EINVAL;
			break;
		}
		ret = lmk04832_update_pll1_n_divider(indio_dev, val);
		if(ret)
			break;
		/* sync PLL2 */
		lmk04832_sync_pll2(indio_dev);
		st->pll1_n_divider = val;
		break;
	case ATTR_PLL2_R_DIVIDER:
		if(val<1 || val>4095){
			ret = -EINVAL;
			break;
		}
		ret = lmk04832_update_pll2_r_divider(indio_dev, val);
		if(ret)
			break;
		/* sync PLL2 */
		lmk04832_sync_pll2(indio_dev);
		st->pll2_r_divider = val;
		break;
	case ATTR_PLL2_N_DIVIDER:
		if(val<1 || val>262143){
			ret = -EINVAL;
			break;
		}
		ret = lmk04832_update_pll2_n_divider(indio_dev, val);
		if(ret)
			break;
		lmk04832_sync_pll2(indio_dev);
		st->pll2_n_divider = val;
		break;
	case ATTR_PLL2_N_PRESCALER:
		if(val<2 || val>8){
			ret = -EINVAL;
			break;
		}
		ret = lmk04832_update_pll2_n_prescaler(indio_dev, val);
		if(ret)
			break;
		/* sync PLL2 */
		lmk04832_sync_pll2(indio_dev);
		st->pll2_n_prescaler = val;
		break;
	case ATTR_PLL2_OSC_DOUBLER_EN:
		if(val>1){
			ret = -EINVAL;
			break;
		}
		ret = lmk04832_update_pll2_osc_doubler_en(indio_dev, val);
		if(ret)
			break;
		/* sync PLL2 */
		lmk04832_sync_pll2(indio_dev);
		st->pll2_osc_doubler_en = val;
		break;
	case ATTR_OSCOUT_SEL:
		if(val>2){
			ret = -EINVAL;
			break;
		}
		ret = lmk04832_update_oscout_sel(indio_dev, val);
		if(ret)
			break;
		st->oscout_sel = val;
		break;
	case ATTR_SDRCLK_SEL_LMX_LMKN:
		if(val>1){
			ret = -EINVAL;
			break;
		}
		ret = lmk04832_update_sdrclk_sel_lmx_lmkn(indio_dev, val);
		if(ret)
			break;
		st->sdrclk_sel_lmx_lmkn = val;
		break;
	case ATTR_LO2_LOWSPUR_EN:
		if(val>1){
			ret = -EINVAL;
			break;
		}
		ret = lmk04832_update_lo2_lowspur_en(indio_dev, val);
		if(ret)
			break;
		st->lo2_lowspur_en = val;
		break;
	case ATTR_VCXO_SEL_100N_125:
		if(val>1){
			ret = -EINVAL;
			break;
		}
		ret = lmk04832_update_vcxo_sel_100n_125(indio_dev, val);
		if(ret)
			break;
		st->vcxo_sel_100n_125 = val;
		break;
	case ATTR_SYNC_ALL_REGISTERS:
		lmk04832_sync_all_registers(indio_dev);
		break;
	default:
		ret = -ENODEV;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static int lmk04832_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch(m){
	case IIO_CHAN_INFO_RAW:
		break;
	case IIO_CHAN_INFO_FREQUENCY:
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
};

static int lmk04832_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		break;
	case IIO_CHAN_INFO_FREQUENCY:
		break;
	default:
		ret = -EINVAL;
		goto end;
	}

end:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static int lmk04832_reg_access(struct iio_dev *indio_dev,
			      unsigned reg_num, unsigned writeval,
			      unsigned *readval)
{
	struct lmk04832_state *st = iio_priv(indio_dev);
	int ret;
	uint32_t reg;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		ret = lmk04832_write(indio_dev, reg_num, LMK04832_VALUE(writeval));
		if(ret)
			goto out_unlock;
		st->reg_map[lmk04832_reg_lut(reg_num)] = LMK04832_VALUE(writeval);  // update local register map
	} else {
		ret = lmk04832_spi_read(indio_dev, reg_num, &reg);
		if(ret)
			goto out_unlock;
		*readval = reg;
		ret = 0;
	}

out_unlock:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

ALL_CLK_IIO_DEVICE_ATTR(PD, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_CLK_PD);

ALL_CLK_IIO_DEVICE_ATTR(DIV, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_CLK_DIV);

ALL_CLK_IIO_DEVICE_ATTR(TYPE, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_CLK_TYPE);

ALL_CLK_IIO_DEVICE_ATTR(DDLY, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_CLK_DDLY);

static IIO_DEVICE_ATTR(Reference_Frequency_Hz, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_REFERENCE_FREQUENCY_HZ);

static IIO_DEVICE_ATTR(Reference_EXT_INTn, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_REFERENCE_EXT_INTN);

static IIO_DEVICE_ATTR(VCO_EXT_INTn, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_VCO_EXT_INTN);

static IIO_DEVICE_ATTR(PLL1_R_Divider, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_PLL1_R_DIVIDER);

static IIO_DEVICE_ATTR(PLL1_N_Divider, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_PLL1_N_DIVIDER);

static IIO_DEVICE_ATTR(PLL2_R_Divider, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_PLL2_R_DIVIDER);

static IIO_DEVICE_ATTR(PLL2_N_Divider, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_PLL2_N_DIVIDER);

static IIO_DEVICE_ATTR(PLL2_N_Prescaler, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_PLL2_N_PRESCALER);

static IIO_DEVICE_ATTR(PLL2_OSC_Doubler_En, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_PLL2_OSC_DOUBLER_EN);

static IIO_DEVICE_ATTR(OSCOut_Sel, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_OSCOUT_SEL);

static IIO_DEVICE_ATTR(SDRCLK_Sel_LMX_LMKn, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_SDRCLK_SEL_LMX_LMKN);

static IIO_DEVICE_ATTR(LO2_LowSpur_En, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_LO2_LOWSPUR_EN);

static IIO_DEVICE_ATTR(VCXO_Sel_100N_125, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_VCXO_SEL_100N_125);

static IIO_DEVICE_ATTR(PLL1_locked, S_IRUGO,
			lmk04832_show,
			NULL,
			ATTR_PLL1_LOCKED);

static IIO_DEVICE_ATTR(PLL2_locked, S_IRUGO,
			lmk04832_show,
			NULL,
			ATTR_PLL2_LOCKED);

static IIO_DEVICE_ATTR(Osc_Frequency_Hz, S_IRUGO,
			lmk04832_show,
			NULL,
			ATTR_OSC_FREQUENCY_HZ);

static IIO_DEVICE_ATTR(VCO_Frequency_Hz, S_IRUGO,
			lmk04832_show,
			NULL,
			ATTR_VCO_FREQUENCY_HZ);

static IIO_DEVICE_ATTR(Sync_All_Registers, S_IRUGO | S_IWUSR,
			lmk04832_show,
			lmk04832_store,
			ATTR_SYNC_ALL_REGISTERS);

static struct attribute *lmk04832_attributes[] = {
	ALL_CLK_IIO_ATTR_REF(PD),
	ALL_CLK_IIO_ATTR_REF(DIV),
	ALL_CLK_IIO_ATTR_REF(TYPE),
	ALL_CLK_IIO_ATTR_REF(DDLY),
	ATTR_REF(Reference_Frequency_Hz),
	ATTR_REF(Reference_EXT_INTn),
	ATTR_REF(VCO_EXT_INTn),
	ATTR_REF(PLL1_R_Divider),
	ATTR_REF(PLL1_N_Divider),
	ATTR_REF(PLL2_R_Divider),
	ATTR_REF(PLL2_N_Divider),
	ATTR_REF(PLL2_N_Prescaler),
	ATTR_REF(PLL2_OSC_Doubler_En),
	ATTR_REF(OSCOut_Sel),
	ATTR_REF(SDRCLK_Sel_LMX_LMKn),
	ATTR_REF(LO2_LowSpur_En),
	ATTR_REF(VCXO_Sel_100N_125),
	ATTR_REF(PLL1_locked),
	ATTR_REF(PLL2_locked),
	ATTR_REF(Osc_Frequency_Hz),
	ATTR_REF(VCO_Frequency_Hz),
	ATTR_REF(Sync_All_Registers),
	NULL,
};

static const struct attribute_group lmk04832_attribute_group = {
	.attrs = lmk04832_attributes,
};

static const struct iio_info lmk04832_info = {
	.read_raw = &lmk04832_read_raw,
	.write_raw = &lmk04832_write_raw,
	.debugfs_reg_access = &lmk04832_reg_access,
	.attrs = &lmk04832_attribute_group,
};

#if DISABLE_IIO_CLOCK_INTERFACE
#define to_lmk04832_clk_output(_hw) container_of(_hw, struct lmk04832_outputs, hw)

static long lmk04832_get_clk_attr(struct clk_hw *hw, long mask)
{
	struct iio_dev *indio_dev = to_lmk04832_clk_output(hw)->indio_dev;
	int val, ret;
	struct iio_chan_spec chan;

	chan.channel = to_lmk04832_clk_output(hw)->num;
	ret = lmk04832_read_raw(indio_dev, &chan, &val, NULL, mask);
	if (ret == IIO_VAL_INT)
		return val;

	return ret;
}

static long lmk04832_set_clk_attr(struct clk_hw *hw, long mask, unsigned long val)
{
	struct iio_dev *indio_dev = to_lmk04832_clk_output(hw)->indio_dev;
	struct iio_chan_spec chan;

	chan.channel = to_lmk04832_clk_output(hw)->num;

	return lmk04832_write_raw(indio_dev, &chan, val, 0, mask);
}

static unsigned long lmk04832_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	return lmk04832_get_clk_attr(hw, IIO_CHAN_INFO_FREQUENCY);
}

static int lmk04832_clk_is_enabled(struct clk_hw *hw)
{
	return to_lmk04832_clk_output(hw)->is_enabled;
}

static int lmk04832_clk_prepare(struct clk_hw *hw)
{
	return lmk04832_set_clk_attr(hw, IIO_CHAN_INFO_RAW, 0);
}

static void lmk04832_clk_unprepare(struct clk_hw *hw)
{
	lmk04832_set_clk_attr(hw, IIO_CHAN_INFO_RAW, 1);
}

static int lmk04832_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long prate)
{
	return lmk04832_set_clk_attr(hw, IIO_CHAN_INFO_FREQUENCY, rate);
}

static long lmk04832_clk_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *prate)
{
	struct iio_dev *indio_dev = to_lmk04832_clk_output(hw)->indio_dev;
	struct lmk04832_state *st = iio_priv(indio_dev);
	unsigned long tmp;

	if (!rate)
		return 0;

	tmp = DIV_ROUND_CLOSEST(st->vco_out_freq, rate);
	tmp = clamp(tmp, 1UL, 1045UL);

	return st->vco_out_freq / tmp;
}

static const struct clk_ops lmk04832_clk_ops = {
	.recalc_rate = lmk04832_clk_recalc_rate,
	.is_enabled = lmk04832_clk_is_enabled,
	.prepare = lmk04832_clk_prepare,
	.unprepare = lmk04832_clk_unprepare,
	.set_rate = lmk04832_clk_set_rate,
	.round_rate = lmk04832_clk_round_rate,
};

static struct clk *lmk04832_clk_register(struct iio_dev *indio_dev, unsigned num,
				bool is_enabled)
{
	struct lmk04832_state *st = iio_priv(indio_dev);
	struct clk_init_data init;
	struct lmk04832_outputs *output = &st->output[num];
	struct clk *clk;
	char name[SPI_NAME_SIZE + 8];

	sprintf(name, "%s_out%d", indio_dev->name, num);

	init.name = name;
	init.ops = &lmk04832_clk_ops;

	init.num_parents = 0;
	init.flags = CLK_IS_BASIC;
	output->hw.init = &init;
	output->indio_dev = indio_dev;
	output->num = num;
	output->is_enabled = is_enabled;

	/* register the clock */
	clk = clk_register(&st->spi->dev, &output->hw);
	st->clk_data.clks[num] = clk;

	return clk;
}
#endif // DISABLE_IIO_CLOCK_INTERFACE

static int lmk04832_setup(struct iio_dev *indio_dev)
{
	struct lmk04832_state *st = iio_priv(indio_dev);
	int i, cnt;

	for(i=0; i<LMK04832_NUM_REG; i++)
		st->reg_map[i] = lmk04832_reg_default[i] & 0xFF;

	if( lmk04832_update_reference_frequency(indio_dev, st->reference_frequency_hz) )
		return -1;
	if( lmk04832_update_reference_ext_intn(indio_dev, st->reference_ext_intn) )
		return -1;
	if( lmk04832_update_vco_ext_intn(indio_dev, st->vco_ext_intn) )
		return -1;
	if( lmk04832_update_pll1_r_divider(indio_dev, st->pll1_r_divider) )
		return -1;
	if( lmk04832_update_pll1_n_divider(indio_dev, st->pll1_n_divider) )
		return -1;
	if( lmk04832_update_pll2_r_divider(indio_dev, st->pll2_r_divider) )
		return -1;
	if( lmk04832_update_pll2_n_divider(indio_dev, st->pll2_n_divider) )
		return -1;
	if( lmk04832_update_pll2_n_prescaler(indio_dev, st->pll2_n_prescaler) )
		return -1;
	if( lmk04832_update_pll2_osc_doubler_en(indio_dev, st->pll2_osc_doubler_en) )
		return -1;
	if( lmk04832_update_oscout_sel(indio_dev, st->oscout_sel) )
		return -1;
	if( lmk04832_update_sdrclk_sel_lmx_lmkn(indio_dev, st->sdrclk_sel_lmx_lmkn) )
		return -1;
	if( lmk04832_update_lo2_lowspur_en(indio_dev, st->lo2_lowspur_en) )
		return -1;
	if( lmk04832_update_vcxo_sel_100n_125(indio_dev, st->vcxo_sel_100n_125) )
		return -1;

	for(cnt=0; cnt<LMK04832_NUM_CHAN; cnt++){
		i = lmk04832_reg_lut( LMK04832_GET_CLK_FORMAT_REG(cnt) );
		st->clk_output_format[cnt] = lmk04832_get_clk_format(st->reg_map[i], cnt);
		if(!st->clk_output_format[cnt])
			st->clk_output_format[cnt] = LMK04832_CLK_FORMAT_LVDS;
	}

	lmk04832_sync_all_registers(indio_dev);
	msleep(300);	// give the lmk04832 some time to setup the clocks

	st->clk_data.clks = st->clks;
	st->clk_data.clk_num = LMK04832_NUM_CHAN;

	of_clk_add_provider(st->spi->dev.of_node, of_clk_src_onecell_get, &st->clk_data);

	return 0;
}

static int lmk04832_parse_dt(struct device *dev, struct lmk04832_state *st){
	struct device_node *np = dev->of_node;
	int ret;

	/* SPI name */
	strncpy(&st->name[0], np->name, SPI_NAME_SIZE - 1);

	/* reference frequency */
	ret = of_property_read_u32(np, "lmk,reference-frequency", &st->reference_frequency_hz);
	if(ret){
		st->reference_frequency_hz = 10000000;
		printk("lmk04832: warning - lmk,reference-frequency=%u\n", st->reference_frequency_hz);
	}

	/* reference frequency externeal/internal */
	st->reference_ext_intn = of_property_read_bool(np, "lmk,reference-ext-intn");

	/* VCO external/internal */
	st->vco_ext_intn = of_property_read_bool(np, "lmk,vco-ext-intn");

	/* PLL1 R divider */
	ret = of_property_read_u32(np, "lmk,pll1-r-divider", &st->pll1_r_divider);
	if(ret){
		st->pll1_r_divider = 1;
		printk("lmk04832: warning - lmk,pll1-r-divider=%u\n", st->pll1_r_divider);
	}

	/* PLL1 N divider */
	ret = of_property_read_u32(np, "lmk,pll1-n-divider", &st->pll1_n_divider);
	if(ret){
		st->pll1_n_divider = 10;
		printk("lmk04832: warning - lmk,pll1-n-divider=%u\n", st->pll1_n_divider);
	}

	/* PLL2 R divider */
	ret = of_property_read_u32(np, "lmk,pll2-r-divider", &st->pll2_r_divider);
	if(ret){
		st->pll2_r_divider = 1;
		printk("lmk04832: warning - lmk,pll2-r-divider=%u", st->pll2_r_divider);
	}

	/* PLL2 N divider */
	ret = of_property_read_u32(np, "lmk,pll2-n-divider", &st->pll2_n_divider);
	if(ret){
		st->pll2_n_divider = 3;
		printk("lmk04832: warning - lmk,pll2-n-divider=%u", st->pll2_n_divider);
	}

	/* PLL2 N prescaler */
	ret = of_property_read_u32(np, "lmk,pll2-n-prescaler", &st->pll2_n_prescaler);
	if(ret){
		st->pll2_n_prescaler = 5;
		printk("lmk04832: warning - lmk,pll2-n-prescaler=%u", st->pll2_n_prescaler);
	}

	/* PLL2 osc doubler enable/disable */
	st->pll2_osc_doubler_en = of_property_read_bool(np, "lmk,pll2-osc-doubler-en");

	/* PLL2 osc doubler enable/disable */
	ret = of_property_read_u32(np, "lmk,oscout-sel", &st->oscout_sel);
	if(ret){
		st->oscout_sel = 0;
		printk("lmk04832: warning - lmk,oscout-sel=%u", st->oscout_sel);
	}

	/* PLL2 osc doubler enable/disable */
	st->sdrclk_sel_lmx_lmkn = of_property_read_bool(np, "lmk,sdrclk-sel-lmxn-lmk");

	/* PLL2 osc doubler enable/disable */
	st->lo2_lowspur_en = of_property_read_bool(np, "lmk,lo2-lowspur-en");

	st->vcxo_sel_100n_125 = of_property_read_bool(np, "lmk,vcxo-sel-100n-125");

	return 0;
}

static int lmk04832_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct lmk04832_state *st;
	int ret;

	printk("lmk04832: calling probe()\n");

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

	/* parse device-tree */
	printk("lmk04832: parsing device tree ..\n");
	ret = lmk04832_parse_dt(&spi->dev, st);
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
	indio_dev->name = (st->name[0] != 0) ? st->name : spi_get_device_id(spi)->name;
	indio_dev->info = &lmk04832_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->lmk04832_channels;
	indio_dev->num_channels = LMK04832_NUM_CHAN;

	/* initialize LMK04832 */
	ret = lmk04832_setup(indio_dev);
	if (ret < 0)
		goto error_disable_reg;

	/* register IIO device */
	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	/* valid SPI device? */
	if(&spi->dev == NULL)
		printk("\nlmk04832: &spi->dev = NULL\n\n");

	dev_info(&spi->dev, "probed %s\n", indio_dev->name);
	return 0;

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return ret;
}

static int lmk04832_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct lmk04832_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	if(!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return 0;
}

static const struct spi_device_id lmk04832_id[] = {
	{"lmk04832-1", 0},
	{}
};

MODULE_DEVICE_TABLE(spi, lmk04832_id);

static struct spi_driver lmk04832_driver = {
	.driver = {
		.name	= "lmk04832",
	},
	.probe		= lmk04832_probe,
	.remove		= lmk04832_remove,
	.id_table	= lmk04832_id,
};
module_spi_driver(lmk04832_driver);

MODULE_AUTHOR("Cyril Zwahlen <zwahlen@precisionwave.com>");
MODULE_DESCRIPTION("TI LMK04832");
MODULE_LICENSE("GPL v2");
