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

#include <linux/clk-provider.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>


#define DRIVER_NAME		"lmx2594"
//#define NAME_(x)		lmx2594_##x
#define DEFAULT_OUTPUT1_FREQUENCY_HZ		0
#define DEFAULT_OUTPUT2_FREQUENCY_HZ		0
#define DEFAULT_VCO_FREQUENCY_HZ		0
#define DEFAULT_OUTPUT1_ENABLE			0
#define DEFAULT_OUTPUT2_ENABLE			0
#define DEFAULT_PLL_LOCKED				0

enum attributes{
	REGISTER,
	DEVICE_NAME,
	OUTPUT1_FREQUENCY_HZ,
	OUTPUT2_FREQUENCY_HZ,
	VCO_FREQUENCY_HZ,
	SYNTH_ENABLE,
	OUTPUT1_ENABLE,
	OUTPUT2_ENABLE,
	PLL_LOCKED
};

struct lmx2594_state{
	struct spi_device *spi;
	const char* unique_id;

	uint32_t spi_rd_buffer;
	uint32_t spi_wr_buffer;

	/* spi register */
	uint32_t reg;

	/* dummy attributes */
	char* device_name;
	uint64_t output1_frequency_hz;
	uint64_t output1_frequency_hz_fraction;
	uint64_t output2_frequency_hz;
	uint64_t output2_frequency_hz_fraction;
	uint64_t vco_frequency_hz;
	uint64_t vco_frequency_hz_fraction;
	uint32_t synth_enable;
	uint32_t output1_enable;
	uint32_t output2_enable;
	uint32_t pll_locked;
};

static ssize_t lmx2594_spi_transfer(struct iio_dev *indio_dev)
{
	struct lmx2594_state *st = iio_priv(indio_dev);

	struct spi_transfer t = {
	    .tx_buf = &st->spi_wr_buffer,
	    .rx_buf = &st->spi_rd_buffer,
	    .len = 3,
	};

	return spi_sync_transfer(st->spi, &t, 1); // write buffer
}

static int lmx2594_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct lmx2594_state *st = iio_priv(indio_dev);

	int ret;
	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) { // write access
		st->spi_wr_buffer = cpu_to_be32( ((0 << 23) | ((reg & 0x7f) << 16) | (writeval & 0xffff)) <<8);
		ret = lmx2594_spi_transfer(indio_dev);
	} else { // read access
		st->spi_wr_buffer = cpu_to_be32( ((1 << 23) | ((reg & 0x7f) << 16) | (writeval & 0xffff)) <<8);
		ret = lmx2594_spi_transfer(indio_dev); // write read bit & address and readback values
		// ret = lmx2594_spi_transfer(indio_dev); // write read bit & address and readback values
		st->reg = (be32_to_cpu(st->spi_rd_buffer) >> 8) & 0xffff;
		*readval = st->reg;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t lmx2594_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct lmx2594_state *st = iio_priv(indio_dev);
	long val;
	unsigned long long val64;
	unsigned long long fraction;
	int ret = 0;
	char *fract_pos;
	char temp[20];

	if((u32)this_attr->address == DEVICE_NAME)
		;  // no conversion needed
	else if((u32)this_attr->address == OUTPUT1_FREQUENCY_HZ ||
					(u32)this_attr->address == OUTPUT2_FREQUENCY_HZ ||
					(u32)this_attr->address == VCO_FREQUENCY_HZ
		){
		fract_pos = strchr(buf, '.');
		if(fract_pos == NULL){
			ret = kstrtoull(buf, 10, &val64);
			fraction = 0;
		}
		else{
			strncpy(temp, buf, fract_pos-buf);
			temp[fract_pos-buf] = '\0';
			ret = kstrtoull(temp, 10, &val64);

			strncpy(temp, fract_pos+1, sizeof(temp));
			ret = kstrtoull(temp, 10, &fraction);
		}
	}
	else
		ret = kstrtol(buf, 0, &val);
	if(ret)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address){
	case REGISTER:
		st->reg = (u32)val;
		st->spi_wr_buffer = cpu_to_be32(st->reg << 8);	// we gonna send 24-bits beginning with MSB [31:8]
		lmx2594_spi_transfer(indio_dev);
		st->reg = be32_to_cpu(st->spi_rd_buffer) >> 8;
		break;
	case DEVICE_NAME:
		if(st->device_name)
			kfree(st->device_name);
		st->device_name = kasprintf(GFP_KERNEL, "%s", buf);
		break;
	case OUTPUT1_FREQUENCY_HZ:
		st->output1_frequency_hz = val64;
		st->output1_frequency_hz_fraction = fraction;
		break;
	case OUTPUT2_FREQUENCY_HZ:
		st->output2_frequency_hz = val64;
		st->output2_frequency_hz_fraction = fraction;
		break;
	case VCO_FREQUENCY_HZ:
		st->vco_frequency_hz = val64;
		st->vco_frequency_hz_fraction = fraction;
		break;
	case SYNTH_ENABLE:
		st->synth_enable = (u32)val;
		break;
	case OUTPUT1_ENABLE:
		st->output1_enable = (u32)val;
		break;
	case OUTPUT2_ENABLE:
		st->output2_enable = (u32)val;
		break;
	case PLL_LOCKED:
		st->pll_locked = (u32)val;
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t lmx2594_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct lmx2594_state *st = iio_priv(indio_dev);
	u32 val = 0;
	u64 val64 = 0;
	u64 fraction = 0;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address){
	case REGISTER:
		val = st->reg;
		break;
	case DEVICE_NAME:
		if(st->device_name)
			ret = sprintf(buf, "%s\n", st->device_name);
		else
			ret = sprintf(buf, "\n");
		mutex_unlock(&indio_dev->mlock);
		return ret;
	case OUTPUT1_FREQUENCY_HZ:
		val64 = st->output1_frequency_hz;
		fraction = st->output1_frequency_hz_fraction;
		break;
	case OUTPUT2_FREQUENCY_HZ:
		val64 = st->output2_frequency_hz;
		fraction = st->output2_frequency_hz_fraction;
		break;
	case VCO_FREQUENCY_HZ:
		val64 = st->vco_frequency_hz;
		fraction = st->vco_frequency_hz_fraction;
		break;
	case SYNTH_ENABLE:
		val = st->synth_enable;
		break;
	case OUTPUT1_ENABLE:
		val = st->output1_enable;
		break;
	case OUTPUT2_ENABLE:
		val = st->output2_enable;
		break;
	case PLL_LOCKED:
		val = st->pll_locked;
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0){
		if((u32)this_attr->address == OUTPUT1_FREQUENCY_HZ ||
				(u32)this_attr->address == OUTPUT2_FREQUENCY_HZ ||
				(u32)this_attr->address == VCO_FREQUENCY_HZ
			)
			ret = sprintf(buf, "%llu.%llu\n", val64, fraction);
		else
			ret = sprintf(buf, "%d\n", val);
	}
	return ret;
}

static IIO_DEVICE_ATTR(Register, S_IRUGO | S_IWUSR,
			lmx2594_show,
			lmx2594_store,
			REGISTER);

static IIO_DEVICE_ATTR(Device_Name, S_IRUGO | S_IWUSR,
			lmx2594_show,
			lmx2594_store,
			DEVICE_NAME);

static IIO_DEVICE_ATTR(Output1_Frequency_Hz, S_IRUGO | S_IWUSR,
			lmx2594_show,
			lmx2594_store,
			OUTPUT1_FREQUENCY_HZ);

static IIO_DEVICE_ATTR(Output2_Frequency_Hz, S_IRUGO | S_IWUSR,
			lmx2594_show,
			lmx2594_store,
			OUTPUT2_FREQUENCY_HZ);

static IIO_DEVICE_ATTR(VCO_Frequency_Hz, S_IRUGO | S_IWUSR,
			lmx2594_show,
			lmx2594_store,
			VCO_FREQUENCY_HZ);

static IIO_DEVICE_ATTR(Synth_Enable, S_IRUGO | S_IWUSR,
			lmx2594_show,
			lmx2594_store,
			SYNTH_ENABLE);

static IIO_DEVICE_ATTR(Output1_Enable, S_IRUGO | S_IWUSR,
			lmx2594_show,
			lmx2594_store,
			OUTPUT1_ENABLE);

static IIO_DEVICE_ATTR(Output2_Enable, S_IRUGO | S_IWUSR,
			lmx2594_show,
			lmx2594_store,
			OUTPUT2_ENABLE);

static IIO_DEVICE_ATTR(PLL_Locked, S_IRUGO | S_IWUSR,
			lmx2594_show,
			lmx2594_store,
			PLL_LOCKED);

static struct attribute *lmx2594_attributes[] = {
	&iio_dev_attr_Register.dev_attr.attr,
	&iio_dev_attr_Device_Name.dev_attr.attr,
	&iio_dev_attr_Output1_Frequency_Hz.dev_attr.attr,
	&iio_dev_attr_Output2_Frequency_Hz.dev_attr.attr,
	&iio_dev_attr_VCO_Frequency_Hz.dev_attr.attr,
	&iio_dev_attr_Synth_Enable.dev_attr.attr,
	&iio_dev_attr_Output1_Enable.dev_attr.attr,
	&iio_dev_attr_Output2_Enable.dev_attr.attr,
	&iio_dev_attr_PLL_Locked.dev_attr.attr,
	NULL
};

static const struct attribute_group lmx2594_attribute_group = {
	.attrs = lmx2594_attributes,
};

static const struct iio_info lmx2594_info = {
	.attrs = &lmx2594_attribute_group,
	.debugfs_reg_access = &lmx2594_reg_access,
};

static int lmx2594_parse_dt(struct device *dev, struct lmx2594_state *st){
	struct device_node *np = dev->of_node;

	/* try to get unique id */
	if(of_property_read_string(np, "optional,unique-id", &st->unique_id))
		st->unique_id = "";  // default

	/* try to get initial value for Output1_Frequency_Hz */
	if(of_property_read_u64(np, "optional,output1-frequency-hz", &st->output1_frequency_hz)){
		pr_warning(DRIVER_NAME" >> output1-frequency-hz: using default value = %d\n", DEFAULT_OUTPUT1_FREQUENCY_HZ);
		st->output1_frequency_hz = DEFAULT_OUTPUT1_FREQUENCY_HZ;
	}

	/* try to get initial value for Output2_Frequency_Hz */
	if(of_property_read_u64(np, "optional,output2-frequency-hz", &st->output2_frequency_hz)){
		pr_warning(DRIVER_NAME" >> output2-frequency-hz: using default value = %d\n", DEFAULT_OUTPUT2_FREQUENCY_HZ);
		st->output2_frequency_hz = DEFAULT_OUTPUT2_FREQUENCY_HZ;
	}

	/* try to get initial value for VCO_Frequency_Hz */
	if(of_property_read_u64(np, "optional,vco-frequency-hz", &st->vco_frequency_hz)){
		pr_warning(DRIVER_NAME" >> vco-frequency-hz: using default value = %d\n", DEFAULT_VCO_FREQUENCY_HZ);
		st->vco_frequency_hz = DEFAULT_VCO_FREQUENCY_HZ;
	}

	/* try to get initial value for Output1_Enable */
	if(of_property_read_u32(np, "optional,output1-enable", &st->output1_enable)){
		pr_warning(DRIVER_NAME" >> output1-enable: using default value = %d\n", DEFAULT_OUTPUT1_ENABLE);
		st->output1_enable = DEFAULT_OUTPUT1_ENABLE;
	}

	/* try to get initial value for Output2_Enable */
	if(of_property_read_u32(np, "optional,output2-enable", &st->output2_enable)){
		pr_warning(DRIVER_NAME" >> output2-enable: using default value = %d\n", DEFAULT_OUTPUT2_ENABLE);
		st->output2_enable = DEFAULT_OUTPUT2_ENABLE;
	}

	return 0;
}

static int lmx2594_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct device_node *np = spi->dev.of_node;
	struct lmx2594_state *st;
	int ret;

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
	ret = lmx2594_parse_dt(&spi->dev, st);
	if(ret)
		return ret;

	/* set IIO device name */
	if(!strlen(st->unique_id)){
		pr_warning(DRIVER_NAME" >> unique-id not found! check devicetree ..\n");
		indio_dev->name = np->name;									// set non-unique id of the device
	}
	else if(strlen(st->unique_id) > 59){
		pr_warning(DRIVER_NAME" >> unique-id is too long! check devicetree ..\n");
		indio_dev->name = np->name;									// set non-unique id of the device
	}
	else
		indio_dev->name = st->unique_id;							// set unique id of the device

	indio_dev->info = &lmx2594_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(indio_dev);

	return ret;
}

static int lmx2594_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	iio_device_unregister(indio_dev);
	return 0;
}

static const struct spi_device_id lmx2594_id[] = {
	{DRIVER_NAME, 0},
	{}
};
//MODULE_DEVICE_TABLE(spi, lmx2594_id);

static struct spi_driver lmx2594_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= lmx2594_probe,
	.remove		= lmx2594_remove,
	.id_table	= lmx2594_id,
};
module_spi_driver(lmx2594_driver);

MODULE_AUTHOR("Cyril Zwahlen <zwahlen@precisionwave.com>");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_LICENSE("GPL v2");
