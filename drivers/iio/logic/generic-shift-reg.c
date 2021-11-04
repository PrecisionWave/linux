/*
 * Variable Device Driver
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

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>


#define DRIVER_NAME		"generic-shift-register"
#define NBITS_MAX			32

#define DEFAULT_NUM_BITS	NBITS_MAX

struct gen_shiftreg_state{
	struct spi_device *spi;

	const char* device_name;
	u32 spi_wr_buffer;
	u32 nbits;
	bool bit_state[NBITS_MAX];
	const char* bit_name[NBITS_MAX];
};

#define BIT_ATTR(BIT, ATTR) \
	(ATTR_BIT0_##ATTR + BIT)

#define ALL_BIT_ATTR(ATTR) \
	ATTR_BIT0_##ATTR, \
	ATTR_BIT1_##ATTR, \
	ATTR_BIT2_##ATTR, \
	ATTR_BIT3_##ATTR, \
	ATTR_BIT4_##ATTR, \
	ATTR_BIT5_##ATTR, \
	ATTR_BIT6_##ATTR, \
	ATTR_BIT7_##ATTR, \
	ATTR_BIT8_##ATTR, \
	ATTR_BIT9_##ATTR, \
	ATTR_BIT10_##ATTR, \
	ATTR_BIT11_##ATTR, \
	ATTR_BIT12_##ATTR, \
	ATTR_BIT13_##ATTR, \
	ATTR_BIT14_##ATTR, \
	ATTR_BIT15_##ATTR, \
	ATTR_BIT16_##ATTR, \
	ATTR_BIT17_##ATTR, \
	ATTR_BIT18_##ATTR, \
	ATTR_BIT19_##ATTR, \
	ATTR_BIT20_##ATTR, \
	ATTR_BIT21_##ATTR, \
	ATTR_BIT22_##ATTR, \
	ATTR_BIT23_##ATTR, \
	ATTR_BIT24_##ATTR, \
	ATTR_BIT25_##ATTR, \
	ATTR_BIT26_##ATTR, \
	ATTR_BIT27_##ATTR, \
	ATTR_BIT28_##ATTR, \
	ATTR_BIT29_##ATTR, \
	ATTR_BIT30_##ATTR, \
	ATTR_BIT31_##ATTR

enum attributes{
	ALL_BIT_ATTR(STATE)	// being expanded for all channels
};

int gen_shiftreg_rename_iio_attribute(struct attribute *attr, const char *name){
	// indio_dev->channel_attr_list
	if(!attr)
		return -1;
	if(!attr->name)
		return -2;
	//kfree(attr->name);
	attr->name = kasprintf(GFP_KERNEL, "%s", name);

	return 0;
}

static ssize_t gen_shiftreg_spi_write(struct iio_dev *indio_dev)
{
	struct gen_shiftreg_state *st = iio_priv(indio_dev);
	int i;

	struct spi_transfer t = {
	    .tx_buf = &st->spi_wr_buffer,
	    .len = 0,
	};

	/* write bits to spi buffer */
	st->spi_wr_buffer = 0;
	for(i=0; i<st->nbits; i++){
		st->spi_wr_buffer |= (st->bit_state[i] << i);
	}

	/* define number of bytes to write */
	t.len = (st->nbits+7)/8;

	st->spi_wr_buffer = cpu_to_be32(st->spi_wr_buffer << (NBITS_MAX - t.len*8));

	/* write spi buffer to spi */
	return spi_sync_transfer(st->spi, &t, 1); // write buffer
}

static ssize_t gen_shiftreg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct gen_shiftreg_state *st = iio_priv(indio_dev);
	long val;
	int ret = 0;
	int i;
	u8 match;

	ret = kstrtol(buf, 0, &val);
	if(ret)
		return ret;

	/* bit attributes */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(i=0; i<NBITS_MAX; i++){
		if((u32)this_attr->address == BIT_ATTR(i, STATE)){
			match = 1;
			if(val<0 || val>1){
				ret = -EINVAL;
				break;
			}
			st->bit_state[i] = val;
			ret = gen_shiftreg_spi_write(indio_dev);
		}
	}
	if(match){
		mutex_unlock(&indio_dev->mlock);
		return ret ? ret : len;
	}

	/* unique attributes */
	switch ((u32)this_attr->address){
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t gen_shiftreg_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct gen_shiftreg_state *st = iio_priv(indio_dev);
	u32 val = 0;
	int ret = 0;
	int i;
	u8 match;

	/* bit attributes */
	mutex_lock(&indio_dev->mlock);
	match = 0;
	for(i=0; i<NBITS_MAX; i++){
		if((u32)this_attr->address == BIT_ATTR(i, STATE)){
			match = 1;
			val = st->bit_state[i];
			break;
		}
	}
	if(match){
		if(ret==0)
			ret = sprintf(buf, "%d\n", val);
		mutex_unlock(&indio_dev->mlock);
		return ret;
	}

	/* unique attributes */
	switch ((u32)this_attr->address){
	default:
		ret = -ENODEV;
	}
	if(ret==0)
		ret = sprintf(buf, "%d\n", val);
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

struct attribute *gen_shiftreg_attributes[100];

static const struct attribute_group gen_shiftreg_attribute_group = {
	.attrs = gen_shiftreg_attributes,
};

static const struct iio_info gen_shiftreg_info = {
	.attrs = &gen_shiftreg_attribute_group,
};

static int gen_shiftreg_parse_dt(struct device *dev, struct gen_shiftreg_state *st){
	struct device_node *np = dev->of_node;
	int ret, i;
	char dt_bit_name[60];
	// const char* bit_name;

	/* try to get device name from devicetree */
	if(of_property_read_string(np, "optional,device-name", &st->device_name))
		st->device_name = NULL;  // default

	/* try to get number of bits */
	if(of_property_read_u32(np, "optional,num-bits", &st->nbits)){
		pr_warning(DRIVER_NAME" >> num-bits: using default value = %d\n", DEFAULT_NUM_BITS);
		st->nbits = DEFAULT_NUM_BITS;
	}

	for(i=0; i<NBITS_MAX; i++){
		ret = sprintf(dt_bit_name, "optional,name-bit%u", i);
		if(ret < 0)
			return ret;
		if(of_property_read_string(np, dt_bit_name, &st->bit_name[i])){
			st->bit_name[i] = NULL;
			// if( (gen_shiftreg_rename_iio_attribute(gen_shiftreg_attributes[i], bit_name)) ){ ret = -1; }
		}
	}

	return 0;
}

static int gen_shiftreg_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct device_node *np = spi->dev.of_node;
	struct gen_shiftreg_state *st;
	int ret, i, n;
	struct iio_dev_attr *attr;

	if (!np)
		return -ENODEV;

	dev_dbg(&spi->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	spi_set_drvdata(spi, indio_dev);

	st = iio_priv(indio_dev);
	st->spi = spi;
	indio_dev->dev.parent = &spi->dev;

	/* parse devicetree */
	ret = gen_shiftreg_parse_dt(&spi->dev, st);
	if(ret)
		printk("generic-shift-register: ERROR parsing devicetree!\n");

	/* remove bits that are not used */
	// TODO: this is ONLY working because we have no other attibutes!!
	//      -> the first element of gen_shiftreg_attributes[] equal 'NULL' limits the list on its bottom end!
	n = 0;
	for(i=0; i<st->nbits; i++){
		if(st->bit_name[i]){
			if(strcmp(st->bit_name[i], "") == 0)
				continue;
			attr = kzalloc(sizeof(struct iio_dev_attr), GFP_KERNEL);
			attr->dev_attr.attr.name = st->bit_name[i];
		}
		else{
			attr = kzalloc(sizeof(struct iio_dev_attr), GFP_KERNEL);
			attr->dev_attr.attr.name = kasprintf(GFP_KERNEL, "BIT-%u", i);
		}
		attr->dev_attr.attr.mode = VERIFY_OCTAL_PERMISSIONS(S_IRUGO | S_IWUSR);
		attr->dev_attr.show = gen_shiftreg_show;
		attr->dev_attr.store = gen_shiftreg_store;
		attr->address = BIT_ATTR(i, STATE);
		gen_shiftreg_attributes[n] = &attr->dev_attr.attr;
		n++;
	}
	gen_shiftreg_attributes[n] = NULL;

	/* set IIO device name */
	if(!st->device_name)
		indio_dev->name = np->name;									// set platform name of the device
	else if(strlen(st->device_name) > 59){
		pr_warning(DRIVER_NAME" >> device-name is too long! check devicetree ..\n");
		indio_dev->name = np->name;									// set platform name of the device
	}
	else
		indio_dev->name = st->device_name;					// set device name defined in the devicetree

	indio_dev->info = &gen_shiftreg_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(indio_dev);

	return ret;
}

static int gen_shiftreg_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	iio_device_unregister(indio_dev);
	return 0;
}

static const struct spi_device_id gen_shiftreg_id[] = {
	{DRIVER_NAME, 0},
	{}
};
//MODULE_DEVICE_TABLE(spi, gen_shiftreg_id);

static struct spi_driver gen_shiftreg_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= gen_shiftreg_probe,
	.remove		= gen_shiftreg_remove,
	.id_table	= gen_shiftreg_id,
};
module_spi_driver(gen_shiftreg_driver);

MODULE_AUTHOR("Cyril Zwahlen <zwahlen@precisionwave.com>");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_LICENSE("GPL v2");
