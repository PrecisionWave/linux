/*
 * LTC2158 SPI ADC driver for DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"

#include <linux/clk.h>
#include <linux/clk-provider.h>

#define DCO_DEBUG


#define ALL_OUTPUTS_0			0x00
#define ALL_OUTPUTS_1			0x01
#define ALTER_OUTPUT_PATTERN		0x02
#define CHECKERBOARD_OUTPUT_PATTERN	0x04
#define SET_TESTMODE(x)			(x<<5)
#define TESTMODE_EN			0x04
#define TWOS_COMP			0x01
#define RANDOMIZER			0x02

#define DTESTON				0				// TEST MODE ENABLE/DISABLE
#define TEST_PATTERN			ALTER_OUTPUT_PATTERN		// TEST MODE PATTERN SELECT


/*
 * ADI High-Speed ADC common spi interface registers
 * See Application-Note AN-877
 */

#define ADC_REG_CHIP_PORT_CONF		0x00
#define ADC_REG_CHIP_ID			0x01
#define ADC_REG_CHIP_GRADE		0x02
#define ADC_REG_CHAN_INDEX		0x05
#define ADC_REG_TRANSFER		0xFF
#define ADC_REG_MODES			0x08
#define ADC_REG_TEST_IO			0x0D
#define ADC_REG_ADC_INPUT		0x0F
#define ADC_REG_OFFSET			0x10
#define ADC_REG_OUTPUT_MODE		0x14
#define ADC_REG_OUTPUT_ADJUST		0x15
#define ADC_REG_OUTPUT_PHASE		0x16
#define ADC_REG_OUTPUT_DELAY		0x17
#define ADC_REG_VREF			0x18
#define ADC_REG_ANALOG_INPUT		0x2C

/* ADC_REG_TEST_IO */
#define TESTMODE_OFF			0x0
#define TESTMODE_MIDSCALE_SHORT		0x1
#define TESTMODE_POS_FULLSCALE		0x2
#define TESTMODE_NEG_FULLSCALE		0x3
#define TESTMODE_ALT_CHECKERBOARD	0x4
#define TESTMODE_PN23_SEQ		0x5
#define TESTMODE_PN9_SEQ		0x6
#define TESTMODE_ONE_ZERO_TOGGLE	0x7
#define TESTMODE_USER			0x8
#define TESTMODE_BIT_TOGGLE		0x9
#define TESTMODE_SYNC			0xA
#define TESTMODE_ONE_BIT_HIGH		0xB
#define TESTMODE_MIXED_BIT_FREQUENCY	0xC
#define TESTMODE_RAMP			0xF

/* ADC_REG_TRANSFER */
#define TRANSFER_SYNC			0x1

/* ADC_REG_OUTPUT_MODE */
#define OUTPUT_MODE_OFFSET_BINARY	0x0
#define OUTPUT_MODE_TWOS_COMPLEMENT	0x1
#define OUTPUT_MODE_GRAY_CODE		0x2

/* ADC_REG_OUTPUT_PHASE */
#define OUTPUT_EVEN_ODD_MODE_EN		0x20
#define INVERT_DCO_CLK			0x80

/* ADC_REG_OUTPUT_DELAY */
#define DCO_DELAY_ENABLE 		0x80


/*
 * Analog Devices LTC2158 Dual 14-Bit, 170/210/250 MSPS ADC
 */

#define CHIPID_LTC2158			0x82
#define LTC2158_REG_VREF_MASK		0x1F
#define LTC2158_DEF_OUTPUT_MODE		0x00

enum {
	ID_LTC2158,
};

static int ltc2158_spi_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[3];
	int ret;

	if (spi) {
		buf[0] = reg;

		ret = spi_write_then_read(spi, &buf[0], 1, &buf[1], 1);

		dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
			__func__, reg, buf[2], ret);

		if (ret < 0)
			return ret;

		return buf[1];
	}
	return -ENODEV;
}

static int ltc2158_spi_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	unsigned char buf[3];
	int ret;

	if (spi) {
		buf[0] = reg;
		buf[1] = val;
		ret = spi_write_then_read(spi, buf, 2, NULL, 0);
		if (ret < 0)
			return ret;

		dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
			__func__, reg, val, ret);

		return 0;
	}

	return -ENODEV;
}

static int ltc2158_reg_access(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int writeval, unsigned int *readval)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret;

	if (readval == NULL) {
		ret = ltc2158_spi_write(spi, reg, writeval);
		ltc2158_spi_write(spi, ADC_REG_TRANSFER, TRANSFER_SYNC);
		return ret;
	} else {
		ret = ltc2158_spi_read(spi, reg);
		if (ret < 0)
			return ret;
		*readval = ret;
	}

	return 0;
}

static int ltc2158_testmode_set(struct iio_dev *indio_dev,
			       unsigned chan, unsigned mode)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	ltc2158_spi_write(conv->spi, ADC_REG_CHAN_INDEX, 1 << chan);
	ltc2158_spi_write(conv->spi, ADC_REG_TEST_IO, mode);
	ltc2158_spi_write(conv->spi, ADC_REG_CHAN_INDEX, 0x3);
	ltc2158_spi_write(conv->spi, ADC_REG_TRANSFER, TRANSFER_SYNC);
	conv->testmode[chan] = mode;
	return 0;
}

static unsigned int ltc2158_pnsel_to_testmode(enum adc_pn_sel sel)
{
	switch (sel) {
	case ADC_PN9:
		return TESTMODE_PN9_SEQ;
	case ADC_PN23A:
		return TESTMODE_PN23_SEQ;
	default:
		return TESTMODE_OFF;
	}
}

static int ltc2158_set_pnsel(struct iio_dev *indio_dev, unsigned int chan,
	enum adc_pn_sel sel)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int mode = ltc2158_pnsel_to_testmode(sel);
	int ret;

	if (mode == TESTMODE_OFF)
		ret = ltc2158_spi_write(conv->spi, ADC_REG_OUTPUT_MODE,
				       conv->adc_output_mode);
	else
		ret = ltc2158_spi_write(conv->spi, ADC_REG_OUTPUT_MODE,
				       conv->
				       adc_output_mode &
				       ~OUTPUT_MODE_TWOS_COMPLEMENT);

	if (ret < 0)
		return ret;

	return ltc2158_testmode_set(indio_dev, chan, mode);
}

static const int ltc2158_scale_table[][2] = {
	{2087, 0x0F}, {2065, 0x0E}, {2042, 0x0D}, {2020, 0x0C}, {1997, 0x0B},
	{1975, 0x0A}, {1952, 0x09}, {1930, 0x08}, {1907, 0x07}, {1885, 0x06},
	{1862, 0x05}, {1840, 0x04}, {1817, 0x03}, {1795, 0x02}, {1772, 0x01},
	{1750, 0x00}, {1727, 0x1F}, {1704, 0x1E}, {1681, 0x1D}, {1658, 0x1C},
	{1635, 0x1B}, {1612, 0x1A}, {1589, 0x19}, {1567, 0x18}, {1544, 0x17},
	{1521, 0x16}, {1498, 0x15}, {1475, 0x14}, {1452, 0x13}, {1429, 0x12},
	{1406, 0x11}, {1383, 0x10},
};

static void ltc2158_scale(struct axiadc_converter *conv, int index,
	unsigned int *val, unsigned int *val2)
{
	unsigned int tmp;

	if (index > conv->chip_info->num_scales) {
		*val = 0;
		*val2 = 0;
		return;
	}

	tmp = (conv->chip_info->scale_table[index][0] * 1000000ULL) >>
		    conv->chip_info->channel[0].scan_type.realbits;
	*val = tmp / 1000000;
	*val2 = tmp % 1000000;
}

static const char *const testmodes[] = {
	[TESTMODE_OFF] = "off",
	[TESTMODE_MIDSCALE_SHORT] = "midscale_short",
	[TESTMODE_POS_FULLSCALE] = "pos_fullscale",
	[TESTMODE_NEG_FULLSCALE] = "neg_fullscale",
	[TESTMODE_ALT_CHECKERBOARD] = "checkerboard",
	[TESTMODE_PN23_SEQ] = "pn_long",
	[TESTMODE_PN9_SEQ] = "pn_short",
	[TESTMODE_ONE_ZERO_TOGGLE] = "one_zero_toggle",
	[TESTMODE_USER] = "user",
	[TESTMODE_BIT_TOGGLE] = "bit_toggle",
	[TESTMODE_SYNC] = "sync",
	[TESTMODE_ONE_BIT_HIGH] = "one_bit_high",
	[TESTMODE_MIXED_BIT_FREQUENCY] = "mixed_bit_frequency",
	[TESTMODE_RAMP] = "ramp",
};

static bool ltc2158_valid_test_mode(struct axiadc_converter *conv,
	unsigned int mode)
{
	if (!testmodes[mode])
		return false;

	/*
	 * All converters that support the ramp testmode have a gap between USER and
	 * RAMP.
	 */
	if (conv->chip_info->max_testmode == TESTMODE_RAMP &&
	    mode > TESTMODE_USER && mode < TESTMODE_RAMP)
		return false;

	return true;
}

static ssize_t ltc2158_show_scale_available(struct iio_dev *indio_dev,
					   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int scale[2];
	int i, len = 0;

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		ltc2158_scale(conv, i, &scale[0], &scale[1]);
		len += sprintf(buf + len, "%u.%06u ", scale[0], scale[1]);
	}

	/* replace last space with a newline */
	buf[len - 1] = '\n';

	return len;
}

static ssize_t ltc2158_testmode_mode_available(struct iio_dev *indio_dev,
					      uintptr_t private,
					      const struct iio_chan_spec *chan,
					      char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	size_t len = 0;
	int i;

	for (i = 0; i <= conv->chip_info->max_testmode; ++i) {
		if (ltc2158_valid_test_mode(conv, i))
			len += sprintf(buf + len, "%s ", testmodes[i]);
	}

	/* replace last space with a newline */
	buf[len - 1] = '\n';

	return len;
}

static ssize_t axiadc_testmode_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	return sprintf(buf, "%s\n", testmodes[conv->testmode[chan->channel]]);
}

static ssize_t axiadc_testmode_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int mode, i;
	int ret;

	mode = 0;

	for (i = 0; i <= conv->chip_info->max_testmode; ++i) {
		if (ltc2158_valid_test_mode(conv, i) &&
		    sysfs_streq(buf, testmodes[i])) {
			mode = i;
			break;
		}
	}

	mutex_lock(&indio_dev->mlock);
	ret = ltc2158_testmode_set(indio_dev, chan->channel, mode);
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static struct iio_chan_spec_ext_info axiadc_ext_info[] = {
	{
	 .name = "test_mode",
	 .read = axiadc_testmode_read,
	 .write = axiadc_testmode_write,
	 },
	{
	 .name = "test_mode_available",
	 .read = ltc2158_testmode_mode_available,
	 .shared = true,
	 },
	{
	 .name = "scale_available",
	 .read = ltc2158_show_scale_available,
	 .shared = true,
	 },
	{},
};

#define AIM_CHAN(_chan, _si, _bits, _sign)				\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE) |		\
			BIT(IIO_CHAN_INFO_CALIBBIAS) |			\
			BIT(IIO_CHAN_INFO_CALIBPHASE) |			\
			BIT(IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY), \
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | 	\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	  .ext_info = axiadc_ext_info,					\
	  .scan_index = _si,						\
	  .scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 16,				\
			.shift = 0,					\
	  },								\
	}

#define AIM_CHAN_NOCALIB(_chan, _si, _bits, _sign, _shift)		\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | 	\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	  .ext_info = axiadc_ext_info,			\
	  .scan_index = _si,						\
	  .scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 16,				\
			.shift = _shift,				\
	  },								\
	}

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_LTC2158] = {
		       .name = "LTC2158",
		       .max_rate = 250000000UL,
		       .scale_table = ltc2158_scale_table,
		       .num_scales = ARRAY_SIZE(ltc2158_scale_table),
		       .max_testmode = TESTMODE_RAMP,
		       .num_channels = 2,
		       .channel[0] = AIM_CHAN(0, 0, 14, 'S'),
		       .channel[1] = AIM_CHAN(1, 1, 14, 'S'),
		       },
};

static int ltc2158_get_scale(struct axiadc_converter *conv, int *val, int *val2)
{
	unsigned vref_val, vref_mask;
	unsigned int i;

	vref_val = ltc2158_spi_read(conv->spi, ADC_REG_VREF);

	switch (conv->id) {
	case CHIPID_LTC2158:
		vref_mask = LTC2158_REG_VREF_MASK;
		break;
	default:
		vref_mask = 0xFFFF;
		break;
	}

	vref_val &= vref_mask;

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		if (vref_val == conv->chip_info->scale_table[i][1])
			break;
	}

	ltc2158_scale(conv, i, val, val2);

	return IIO_VAL_INT_PLUS_MICRO;
}

static int ltc2158_set_scale(struct axiadc_converter *conv, int val, int val2)
{
	unsigned int scale_val[2];
	unsigned int i;

	if (val != 0)
		return -EINVAL;

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		ltc2158_scale(conv, i, &scale_val[0], &scale_val[1]);
		if (scale_val[0] != val || scale_val[1] != val2)
			continue;

		ltc2158_spi_write(conv->spi, ADC_REG_VREF,
				 conv->chip_info->scale_table[i][1]);
		ltc2158_spi_write(conv->spi, ADC_REG_TRANSFER,
				 TRANSFER_SYNC);
		return 0;
	}

	return -EINVAL;
}

static int ltc2158_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		return ltc2158_get_scale(conv, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		*val = conv->adc_clk = clk_get_rate_scaled(conv->clk, &conv->adc_clkscale);

		return IIO_VAL_INT;

	}
	return -EINVAL;
}

static int ltc2158_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned long r_clk;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return ltc2158_set_scale(conv, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		if (chan->extend_name)
			return -ENODEV;

		if (conv->sample_rate_read_only)
			return -EPERM;

		r_clk = clk_round_rate(conv->clk, val);
		if (r_clk < 0 || r_clk > conv->chip_info->max_rate) {
			dev_warn(&conv->spi->dev,
				 "Error setting ADC sample rate %ld", r_clk);
			return -EINVAL;
		}

		ret = clk_set_rate(conv->clk, r_clk);
		if (ret < 0)
			return ret;

		if (conv->adc_clk != r_clk) {
			conv->adc_clk = r_clk;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ltc2158_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int i;
	for (i = 0; i < conv->chip_info->num_channels; i++) {
		axiadc_write(st, ADI_REG_CHAN_CNTRL_2(i),
			     (i & 1) ? 0x00004000 : 0x40000000);
		if (!(conv->adc_output_mode & 0x1)){
			axiadc_write(st, ADI_REG_CHAN_CNTRL(i), ADI_ENABLE);
		}else{
			axiadc_write(st, ADI_REG_CHAN_CNTRL(i), ADI_FORMAT_ENABLE | ADI_ENABLE);
		}
	}

	return 0;
}

static void ltc2158_clk_del_provider(void *data)
{
	struct axiadc_converter *conv = data;

	of_clk_del_provider(conv->spi->dev.of_node);
	clk_unregister_fixed_factor(conv->out_clk);
}

static int ltc2158_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct axiadc_converter *conv;
	int ret, clk_enabled = 0;
	struct clk *clk;
	unsigned char buf[2];
	uint32_t phase;

	clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	ret = clk_prepare_enable(clk);
	if (ret < 0)
		return ret;
	clk_enabled = 1;
	conv->adc_clk = clk_get_rate(clk);

	conv->adc_clkscale.mult = 1;
	conv->adc_clkscale.div = 1;

	spi_set_drvdata(spi, conv);
	conv->spi = spi;
	conv->clk = clk;

	conv->pwrdown_gpio = devm_gpiod_get_optional(&spi->dev, "powerdown",
		GPIOD_OUT_LOW);
	if (IS_ERR(conv->pwrdown_gpio))
		return PTR_ERR(conv->pwrdown_gpio);

	conv->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(conv->reset_gpio))
		return PTR_ERR(conv->reset_gpio);

	if (conv->reset_gpio) {
		udelay(1);
		ret = gpiod_direction_output(conv->reset_gpio, 1);
	}

	mdelay(10);

	conv->id = spi_get_device_id(spi)->driver_data;
	if (conv->id != spi_get_device_id(spi)->driver_data) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n",
			conv->id);
		ret = -ENODEV;
		goto out;
	}

	switch (conv->id) {
	case CHIPID_LTC2158:
		conv->chip_info = &axiadc_chip_info_tbl[ID_LTC2158];
		buf[0] = 0x04;	// reg[4]
		conv->adc_output_mode = 0;
		if(of_property_read_bool(np, "en-randomizer")){
			buf[1] = SET_TESTMODE(TEST_PATTERN) | RANDOMIZER | TWOS_COMP;
			conv->adc_output_mode += 1;
		}
		else{
			buf[1] = SET_TESTMODE(TEST_PATTERN) | TWOS_COMP;
		}
		spi_write(spi, buf, 2);
		buf[0] = 0x02;	// reg[2]
		buf[1] = 0x0; // 0x0: clock not inverted for zynq, 0x08 clock inverted for zynqmp;
		if(of_property_read_bool(np, "clock_inverted")){
			conv->adc_output_mode += 2;
			buf[1] += 0x08; // 0x0: clock not inverted for zynq, 0x08 clock inverted for zynqmp;
		}
		if(of_property_read_u32(np, "clock-phase", &phase)==0){
			buf[1] += 0x01; // enable clock duty stabilizer
			buf[1] += (phase & 0x3) << 1; // set clock phase
		}
		spi_write(spi, buf, 2);
		break;
	default:
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
		ret = -ENODEV;
		goto out;
	}

	if (ret < 0)
		goto out;

	conv->reg_access = ltc2158_reg_access;
	conv->write_raw = ltc2158_write_raw;
	conv->read_raw = ltc2158_read_raw;
	conv->post_setup = ltc2158_post_setup;
	conv->set_pnsel = ltc2158_set_pnsel;

	conv->out_clk = clk_register_fixed_factor(&spi->dev, "out_clk",
		__clk_get_name(conv->clk), 0, 1, 1);
	if (IS_ERR(conv->out_clk)) {
		dev_warn(&spi->dev, "Failed to register out_clk\n");
	} else {
		ret = of_clk_add_provider(spi->dev.of_node, of_clk_src_simple_get, conv->out_clk);
		if (ret)
			clk_unregister_fixed_factor(conv->out_clk);

		devm_add_action_or_reset(&spi->dev, ltc2158_clk_del_provider, conv);
	}

	return 0;

out:
	if (clk_enabled)
		clk_disable_unprepare(clk);

	return ret;
}

static int ltc2158_remove(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);

	clk_disable_unprepare(conv->clk);

	return 0;
}

static const struct spi_device_id ltc2158_id[] = {
	{"ltc2158", CHIPID_LTC2158},
	{}
};
MODULE_DEVICE_TABLE(spi, ltc2158_id);

static struct spi_driver ltc2158_driver = {
	.driver = {
		   .name = "ltc2158",
		   .owner = THIS_MODULE,
		   },
	.probe = ltc2158_probe,
	.remove = ltc2158_remove,
	.id_table = ltc2158_id,
};
module_spi_driver(ltc2158_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9467 ADC");
MODULE_LICENSE("GPL v2");
