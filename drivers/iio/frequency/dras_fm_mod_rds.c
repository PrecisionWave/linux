/*
 * DRAS DSP COREFPGA Module
 * DRAS FM REPEATER DSP Core Driver
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


#define DRIVER_NAME			"dras-fm-mod-rds"
#define NB_OF_CHANNELS			32

// global attributes
#define ADDR_DSP_VERSION		0*4
#define ADDR_AUDIO_METER		1*4
#define ADDR_AUDIO_RDS_GAIN		2*4
#define ADDR_PILOT_MOD_GAIN		3*4
#define ADDR_CHANNEL_EN			4*4
#define ADDR_DDSINC(x)			(5+x)*4 
// DDSINC channel interleaving:
// 32 channels, ch range 0..31, two 16bit channels per address
// for ch range 0..3,8..11,16..19,24..27 use lower 16bit of address: 
// for ch range 4..7,12..15,20..23,28..31 use upper 16bit of address: 
// x=4*mod(ch,4)+mod(ch>>3,4)
// if((ch>>2) & 1) then higher 16bit else lower 16bit

#define MAX_FM_FREQUENCY		108100000
#define MIN_FM_FREQUENCY		87400000


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
//      ::   ::TARGET
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
//   IIO_DEVICE_ATTR_ALL_CH(gain_tx1, S_IRUGO | S_IWUSR, dras_fm_mod_rds_show, dras_fm_mod_rds_store, REG_GAIN_TX1)
//     expansion:
//     static IIO_DEVICE_ATTR(ch0_gain_tx1, S_IRUGO | S_IWUSR, dras_fm_mod_rds_show, dras_fm_mod_rds_store, CH0_REG_GAIN_TX1);
//     static IIO_DEVICE_ATTR(ch1_gain_tx1, S_IRUGO | S_IWUSR, dras_fm_mod_rds_show, dras_fm_mod_rds_store, CH1_REG_GAIN_TX1);
//      ::   ::
//     static IIO_DEVICE_ATTR(ch31_gain_tx1, S_IRUGO | S_IWUSR, dras_fm_mod_rds_show, dras_fm_mod_rds_store, CH31_REG_GAIN_TX1);
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
	REG_ALL_CH(REG_FREQUENCY),	// being expanded for all channels
	REG_ALL_CH(REG_CHANNEL_ENABLE),	// being expanded for all channels
	REG_DSP_VERSION,
	REG_AUDIO_METER,
	REG_AUDIO_GAIN,
	REG_RDS_GAIN,
	REG_PILOT_GAIN,
	REG_MOD_GAIN
};

struct dras_fm_mod_rds_state {
	struct iio_info		iio_info;
	void __iomem		*regs;
	struct mutex		lock;

	uint32_t		mod_clk;
	uint32_t		rep_clk;
	const char*		unique_id;
};

static void dras_fm_mod_rds_write(struct dras_fm_mod_rds_state *st, unsigned reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 dras_fm_mod_rds_read(struct dras_fm_mod_rds_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int dras_fm_mod_rds_write_raw(struct iio_dev *indio_dev,
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

static int dras_fm_mod_rds_read_raw(struct iio_dev *indio_dev,
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

static ssize_t dras_fm_mod_rds_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dras_fm_mod_rds_state *st = iio_priv(indio_dev);
	long val;
	int ret;
	u64 temp64;
	u32 temp32;
	u32 ch;
	int match;
	int regoffset;
	int subchannel;
	const uint8_t inc_addroff[32]={0,7,11,14,0,4,11,15,1,4,8,15,1,5,8,12,2,5,9,12,2,6,9,13,3,6,10,13,3,7,10,14};
	const bool inc_msb[32]={0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0};

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

	for(ch=0; ch<NB_OF_CHANNELS; ch++){

		if((u32)this_attr->address == REG_CH(ch, REG_CHANNEL_ENABLE)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			temp32 = dras_fm_mod_rds_read(st, ADDR_CHANNEL_EN) & ~(1<<ch);
			temp32 += (u32)val << ch;
			dras_fm_mod_rds_write(st, ADDR_CHANNEL_EN, temp32);
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FREQUENCY)){
			match = 1;
			regoffset = inc_addroff[ch];
			subchannel = inc_msb[ch];

			temp64 = (u64)st->rep_clk * 15;
			temp64 = div_s64(temp64,44); // fm_f_mix = clk*15/44
			val -= (int)temp64;
			temp64 = (u64)val << 19;
			temp64 = div_s64(temp64,st->mod_clk);
			val = (int)temp64 & 0xFFFF;
			if(subchannel==0){
				temp32 = dras_fm_mod_rds_read(st, ADDR_DDSINC(regoffset)) & 0xFFFF0000;
				temp32 += (u32)val;
				dras_fm_mod_rds_write(st, ADDR_DDSINC(regoffset), temp32+1);
				dras_fm_mod_rds_write(st, ADDR_DDSINC(regoffset), temp32);
			}else{
				temp32 = dras_fm_mod_rds_read(st, ADDR_DDSINC(regoffset)) & 0xFFFF;
				temp32 += (u32)val << 16;
				dras_fm_mod_rds_write(st, ADDR_DDSINC(regoffset), temp32+1);
				dras_fm_mod_rds_write(st, ADDR_DDSINC(regoffset), temp32);
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

	case REG_AUDIO_GAIN:
		if(val<0 || val>65535){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_fm_mod_rds_read(st, ADDR_AUDIO_RDS_GAIN) & 0xFFFF0000;
		temp32 += (u32)val;
		dras_fm_mod_rds_write(st, ADDR_AUDIO_RDS_GAIN, temp32);
		break;
	case REG_RDS_GAIN:
		if(val<0 || val>65535){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_fm_mod_rds_read(st, ADDR_AUDIO_RDS_GAIN) & 0xFFFF;
		temp32 += (u32)val << 16;
		dras_fm_mod_rds_write(st, ADDR_AUDIO_RDS_GAIN, temp32);
		break;
	case REG_PILOT_GAIN:
		if(val<0 || val>65535){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_fm_mod_rds_read(st, ADDR_PILOT_MOD_GAIN) & 0xFFFF0000;
		temp32 += (u32)val;
		dras_fm_mod_rds_write(st, ADDR_PILOT_MOD_GAIN, temp32);
		break;
	case REG_MOD_GAIN:
		if(val<0 || val>65535){
			ret = -EINVAL;
			break;
		}
		temp32 = dras_fm_mod_rds_read(st, ADDR_PILOT_MOD_GAIN) & 0xFFFF;
		temp32 += (u32)val << 16;
		dras_fm_mod_rds_write(st, ADDR_PILOT_MOD_GAIN, temp32);
		break;
	default:
		ret = -ENODEV;
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t dras_fm_mod_rds_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct dras_fm_mod_rds_state *st = iio_priv(indio_dev);
	int val;
	int ret = 0;
	u64 temp64;
	u32 ch;
	int regoffset;
	int subchannel;
	int match;
	const uint8_t inc_addroff[32]={0,7,11,14,0,4,11,15,1,4,8,15,1,5,8,12,2,5,9,12,2,6,9,13,3,6,10,13,3,7,10,14};
	const bool inc_msb[32]={0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0};

	/* channel registers */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(ch=0; ch<NB_OF_CHANNELS; ch++){
		if((u32)this_attr->address == REG_CH(ch, REG_CHANNEL_ENABLE)){
			match = 1;
			val = dras_fm_mod_rds_read(st, ADDR_CHANNEL_EN);
			val = (val >> ch) & 1;
			break;
		}
		else if((u32)this_attr->address == REG_CH(ch, REG_FREQUENCY)){
			match = 1;
			regoffset = inc_addroff[ch];
			subchannel = inc_msb[ch];

			if(subchannel==0)
				val = dras_fm_mod_rds_read(st, ADDR_DDSINC(regoffset)) & 0xFFFF;
			else
				val = dras_fm_mod_rds_read(st, ADDR_DDSINC(regoffset)) >> 16;
			if(val>1<<15){
				temp64 = (u64)val * st->mod_clk;
				val = ((int)(temp64 >> 19)) - (st->mod_clk>>3); // f_test = fm_f_mix+(fm_dds_inc*clk/2^18-clk/4)/3
			}else{
				temp64 = (u64)val * st->mod_clk;
				val = (u32)(temp64 >> 19); // f_test = fm_f_mix+fm_dds_inc*clk/2^18/3
			}
			temp64 = (u64)st->rep_clk * 15;
			temp64 = div_s64(temp64,44); // fm_f_mix = clk*15/44
			val += (int)temp64;
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
		val = dras_fm_mod_rds_read(st, ADDR_DSP_VERSION);
		break;
	case REG_AUDIO_METER:
		val = dras_fm_mod_rds_read(st, ADDR_AUDIO_METER) & 0xFFFF;
		break;
	case REG_AUDIO_GAIN:
		val = dras_fm_mod_rds_read(st, ADDR_AUDIO_RDS_GAIN) & 0xFFFF;
		break;
	case REG_RDS_GAIN:
		val = (dras_fm_mod_rds_read(st, ADDR_AUDIO_RDS_GAIN) >> 16) & 0xFFFF;
		break;
	case REG_PILOT_GAIN:
		val = dras_fm_mod_rds_read(st, ADDR_PILOT_MOD_GAIN) & 0xFFFF;
		break;
	case REG_MOD_GAIN:
		val = (dras_fm_mod_rds_read(st, ADDR_PILOT_MOD_GAIN) >> 16) & 0xFFFF;
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


IIO_DEVICE_ATTR_ALL_CH(frequency, S_IRUGO | S_IWUSR,
			dras_fm_mod_rds_show,
			dras_fm_mod_rds_store,
			REG_FREQUENCY);

IIO_DEVICE_ATTR_ALL_CH(channel_enable, S_IRUGO | S_IWUSR,
			dras_fm_mod_rds_show,
			dras_fm_mod_rds_store,
			REG_CHANNEL_ENABLE);

static IIO_DEVICE_ATTR(dsp_version, S_IRUGO,
			dras_fm_mod_rds_show,
			dras_fm_mod_rds_store,
			REG_DSP_VERSION);

static IIO_DEVICE_ATTR(audio_meter, S_IRUGO,
			dras_fm_mod_rds_show,
			dras_fm_mod_rds_store,
			REG_AUDIO_METER);

static IIO_DEVICE_ATTR(audio_gain, S_IRUGO | S_IWUSR,
			dras_fm_mod_rds_show,
			dras_fm_mod_rds_store,
			REG_AUDIO_GAIN);

static IIO_DEVICE_ATTR(rds_gain, S_IRUGO | S_IWUSR,
			dras_fm_mod_rds_show,
			dras_fm_mod_rds_store,
			REG_RDS_GAIN);

static IIO_DEVICE_ATTR(pilot_gain, S_IRUGO | S_IWUSR,
			dras_fm_mod_rds_show,
			dras_fm_mod_rds_store,
			REG_PILOT_GAIN);

static IIO_DEVICE_ATTR(mod_gain, S_IRUGO | S_IWUSR,
			dras_fm_mod_rds_show,
			dras_fm_mod_rds_store,
			REG_MOD_GAIN);


static struct attribute *dras_fm_mod_rds_attributes[] = {
	IIO_ATTR_ALL_CH(frequency),
	IIO_ATTR_ALL_CH(channel_enable),
	&iio_dev_attr_dsp_version.dev_attr.attr,
	&iio_dev_attr_audio_meter.dev_attr.attr,
	&iio_dev_attr_audio_gain.dev_attr.attr,
	&iio_dev_attr_rds_gain.dev_attr.attr,
	&iio_dev_attr_pilot_gain.dev_attr.attr,
	&iio_dev_attr_mod_gain.dev_attr.attr,
	NULL,
};


static const struct attribute_group dras_fm_mod_rds_attribute_group = {
	.attrs = dras_fm_mod_rds_attributes,
};

static const struct iio_info dras_fm_mod_rds_info = {
	.read_raw = &dras_fm_mod_rds_read_raw,
	.write_raw = &dras_fm_mod_rds_write_raw,
	.attrs = &dras_fm_mod_rds_attribute_group,
};

static const struct iio_chan_spec dras_fm_mod_rds_channels[] = {				// add more channels here if desired
};

/* Match table for of_platform binding */
static const struct of_device_id dras_fm_mod_rds_of_match[] = {
	{ .compatible = "fpga,dras-fm-mod-rds", },
	{ },
};

MODULE_DEVICE_TABLE(of, dras_fm_mod_rds_of_match);

static int dras_fm_mod_rds_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;						// return of of_match_node()
	struct device_node *np = pdev->dev.of_node;			// param of of_match_node()
	struct resource *res;
	struct dras_fm_mod_rds_state *st;
	struct iio_dev *indio_dev;
	int ret; //, i, n;

	if (!np)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	/* looking for "compatible" */
	id = of_match_device(dras_fm_mod_rds_of_match, &pdev->dev);
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

	if(of_property_read_u32(np, "required,mod-clk", &st->mod_clk)){
		printk("DRAS-FM-MOD-RDS: ***ERROR! \"required,mod-clk\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->mod_clk == 0){
		printk("DRAS-FM-MOD-RDS: ***ERROR! \"required,mod-clk\" equal to 0 Hz\n");
		goto err_iio_device_free;
	}

	if(of_property_read_u32(np, "required,rep-clk", &st->rep_clk)){
		printk("DRAS-FM-MOD-RDS: ***ERROR! \"required,rep-clk\" missing in devicetree?\n");
		goto err_iio_device_free;
	}
	if(st->rep_clk == 0){
		printk("DRAS-FM-MOD-RDS: ***ERROR! \"required,rep-clk\" equal to 0 Hz\n");
		goto err_iio_device_free;
	}


	if(of_property_read_string(np, "optional,unique-id", &st->unique_id))
		st->unique_id = "";	// default

	/* try to get unique id */
	if(!strlen(st->unique_id)){
		pr_warn("DRAS-FM-MOD-RDS: unique-id not found! check devicetree ..\n");
		indio_dev->name = np->name;									// set non-unique id of the device
	}
	else if(strlen(st->unique_id) > 59){
		pr_warn("DRAS-FM-MOD-RDS: unique-id is too long! check devicetree ..\n");
		indio_dev->name = np->name;									// set non-unique id of the device
	}
	else
		indio_dev->name = st->unique_id;							// set unique id of the device

	indio_dev->channels = dras_fm_mod_rds_channels;
	indio_dev->num_channels = ARRAY_SIZE(dras_fm_mod_rds_channels);
	indio_dev->info = &dras_fm_mod_rds_info;
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

static int dras_fm_mod_rds_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver dras_fm_mod_rds_driver = {
	.probe		= dras_fm_mod_rds_probe,
	.remove		= dras_fm_mod_rds_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = dras_fm_mod_rds_of_match,
	},
};

module_platform_driver(dras_fm_mod_rds_driver);

MODULE_AUTHOR("Andreas Zutter <zutter@precisionwave.com>");
MODULE_DESCRIPTION("DRAS FM REPEATER FPGA-IP driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRIVER_NAME);
