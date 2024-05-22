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
#define NB_OF_TETRA_CHANNELS		8
#define NB_OF_TETRA_PORTS		16

// global attributes
#define ADDR_MAXGAIN			16*4
#define ADDR_TARGET_PWR			17*4
#define ADDR_SQUELCH			18*4
#define ADDR_CHANNEL_EN			19*4
#define ADDR_DSP_VERSION		20*4
#define ADDR_OFFSET_TLAST0		21*4 // 4bits per port, port0..7
#define ADDR_OFFSET_TLAST1		22*4 // 4bits per port, port8..15

// channel attributes
#define ADDR_RSSI(x)			(0+x)*4 // 16bit LSB first rssi, second 16bit second rssi
#define ADDR_BEST_SOURCE(x)		(8+x)*4
#define ADDR_UL_ORDER(x)		(32+x)*4 // 8 channels per port, each with 4bits, 16 ports, port x from 0..15
#define ADDR_PORT_ID(x)			(48+x)*4 // 12bit port id, port x from 0..15


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
	CH7_##REG

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
	static IIO_DEVICE_ATTR(ch7_##ATTR, RW, SHOW, STORE, CH7_##REG);

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
	&iio_dev_attr_ch7_##ATTR.dev_attr.attr

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
	REG_ALL_CH(REG_RSSI),	// being expanded for all channels
	REG_ALL_CH(REG_CHANNEL_ENABLE),	// being expanded for all channels
	REG_ALL_CH(REG_BEST_SOURCE),	// being expanded for all channels
	REG_ALL_PORT(REG_OFFSET_TLAST),	// being expanded for all channels
	REG_ALL_PORT(REG_ENABLE_DL_TEST),	// being expanded for all channels
	REG_ALL_PORT(REG_UL_ORDER),	// being expanded for all channels
	REG_ALL_PORT(REG_PORT_ID),	// being expanded for all channels
	REG_ALL_PORT(REG_UL_SYNC),	// being expanded for all channels
	REG_DSP_VERSION,
	REG_TARGET_POWER,
	REG_SQUELCH,
	REG_MAX_GAIN
};

struct dras_radio_repeater_state {
	struct iio_info		iio_info;
	void __iomem		*regs;
	struct mutex		lock;

	struct device		*dev;
	struct clk		*adrv_clk;
	struct notifier_block	adrv_clk_rate_change_nb;
	uint32_t		adrv_clk_rate;
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
	}

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
		}
		if((u32)this_attr->address == REG_PORT(port, REG_ENABLE_DL_TEST)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = dras_radio_repeater_read(st, ADDR_CHANNEL_EN) & ~(1<<(port+8));
			temp32 += ((uint32_t)val)<<(port+8);
			dras_radio_repeater_write(st, ADDR_CHANNEL_EN, temp32);
		}
	}

	if(match){
		mutex_unlock(&indio_dev->mlock);
		return ret ? ret : len;
	}

	/* unique registers */
	switch ((u32)this_attr->address) {
	case REG_TARGET_POWER:
		if(val<0 || val>0xFFFF){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_radio_repeater_read(st, ADDR_TARGET_PWR) & ~0xFFFF;
		temp32 += (u32)val;
		dras_radio_repeater_write(st, ADDR_TARGET_PWR, temp32);
		break;
	case REG_SQUELCH:
		if(val<0 || val>0xFFFFFFF){
			ret = -EINVAL;
			break;
		}
		dras_radio_repeater_write(st, ADDR_SQUELCH, (u32)val);
		break;
	case REG_MAX_GAIN:
		if(val<0 || val>0xFFFFFF){
			ret = -EINVAL;
			break;
		}
		dras_radio_repeater_write(st, ADDR_MAXGAIN, (u32)val);
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
		if((u32)this_attr->address == REG_CH(ch, REG_RSSI)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_RSSI(ch)) & 0x1FFFFFF;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_CHANNEL_ENABLE)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_CHANNEL_EN) >> ch) & 1;
			break;
		}else if((u32)this_attr->address == REG_CH(ch, REG_BEST_SOURCE)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_BEST_SOURCE(ch)) & 0xF;
		}
	}

	for(port=0; port<NB_OF_TETRA_PORTS; port++){
		if((u32)this_attr->address == REG_PORT(port, REG_OFFSET_TLAST)){
			match = 1;
			if(port<8){
				shift = port*4;
				val = (dras_radio_repeater_read(st, ADDR_OFFSET_TLAST0)>>shift) & 0xF;
			}else{
				shift = (port-8)*4;
				val = (dras_radio_repeater_read(st, ADDR_OFFSET_TLAST1)>>shift) & 0xF;
			}
			//val = val/2; // only odd channels used
		}
		else if((u32)this_attr->address == REG_PORT(port, REG_ENABLE_DL_TEST)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_CHANNEL_EN) >> (port+8)) & 1;
		}
		else if((u32)this_attr->address == REG_PORT(port, REG_UL_ORDER)){
			match = 1;
			temp32 = dras_radio_repeater_read(st, ADDR_UL_ORDER(port));
			//for(shift=0; shift<8; shift++){
			//	val += power10 * (((temp32>>(4*shift)) & 0xF)/2); // register contains 4bits per channel, but only each 2nd channel is used
			//	power10 = power10 * 10;
			//}
			ret = sprintf(buf, "%08x\n", temp32);
		}
		else if((u32)this_attr->address == REG_PORT(port, REG_PORT_ID)){
			match = 1;
			val = dras_radio_repeater_read(st, ADDR_PORT_ID(port)) & 0xFFF;
		}
		else if((u32)this_attr->address == REG_PORT(port, REG_UL_SYNC)){
			match = 1;
			val = (dras_radio_repeater_read(st, ADDR_PORT_ID(port))>>12) & 0x1;
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
	case REG_TARGET_POWER:
		val = dras_radio_repeater_read(st, ADDR_TARGET_PWR);
		break;
	case REG_SQUELCH:
		val = dras_radio_repeater_read(st, ADDR_SQUELCH);
		break;
	case REG_MAX_GAIN:
		val = dras_radio_repeater_read(st, ADDR_MAXGAIN);
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

IIO_DEVICE_ATTR_ALL_CH(rssi, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_RSSI);

IIO_DEVICE_ATTR_ALL_CH(channel_enable, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_CHANNEL_ENABLE);

IIO_DEVICE_ATTR_ALL_CH(best_source, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_BEST_SOURCE);

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

IIO_DEVICE_ATTR_ALL_PORT(uplink_sync, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_UL_SYNC);

static IIO_DEVICE_ATTR(target_power, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_TARGET_POWER);

static IIO_DEVICE_ATTR(squelch, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_SQUELCH);

static IIO_DEVICE_ATTR(max_gain, S_IRUGO | S_IWUSR,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_MAX_GAIN);

static IIO_DEVICE_ATTR(dsp_version, S_IRUGO,
			dras_radio_repeater_show,
			dras_radio_repeater_store,
			REG_DSP_VERSION);


static struct attribute *dras_radio_repeater_attributes[] = {
	IIO_ATTR_ALL_CH(rssi),
	IIO_ATTR_ALL_CH(channel_enable),
	IIO_ATTR_ALL_CH(best_source),
	IIO_ATTR_ALL_PORT(offset_tlast),
	IIO_ATTR_ALL_PORT(enable_downlink_test),
	IIO_ATTR_ALL_PORT(uplink_order),
	IIO_ATTR_ALL_PORT(id),
	IIO_ATTR_ALL_PORT(uplink_sync),
	&iio_dev_attr_dsp_version.dev_attr.attr,
	&iio_dev_attr_target_power.dev_attr.attr,
	&iio_dev_attr_squelch.dev_attr.attr,
	&iio_dev_attr_max_gain.dev_attr.attr,
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
