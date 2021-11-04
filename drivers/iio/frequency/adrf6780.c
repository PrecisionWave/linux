/*
 * Generic Device Driver
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


#define DRIVER_NAME		"adrf6780"
#define NAME_(x)		adrf6780_##x
#define SPI_BUFFER_SIZE		3	// in bytes (1 .. 4)

#define RESERVED_SPI		0x200

#define CONTROL			0x00	// RW
#define ALARM_READBACK		0x01	// R
#define ALARM_MASK		0x02	// RW
#define ENABLE			0x03	// RW
#define LINEARIZE		0x04	// RW
#define LO_PATH			0x05	// RW
#define ADC_CONTROL		0x06	// RW
#define ADC_OUTPUT		0x0C	// R
//#define DEBUG			0xFF
#define RDAC_LINEARIZE          0x10    // RW
#define I_PATH_PHASE_AQQURACY	0x11	// RW
#define Q_PATH_PHASE_AQQURACY	0x12	// RW

struct NAME_(platform_data){
	char name[SPI_NAME_SIZE];
};

struct NAME_(state){
	struct spi_device *spi;
	const char* unique_id;

	uint32_t spi_rd_buffer;
	uint32_t spi_wr_buffer;

	uint32_t reserved_spi;
	uint16_t regs[13];

//	uint32_t debug;

	__be32			val ____cacheline_aligned;
};

static ssize_t NAME_(spi_transfer)(struct iio_dev *indio_dev, uint32_t val)
{
	struct NAME_(state) *st = iio_priv(indio_dev);
	int ret;

	struct spi_transfer t = {
	    .tx_buf = &st->spi_wr_buffer,
	    .rx_buf = &st->spi_rd_buffer,
	    .len = SPI_BUFFER_SIZE,
	};

	st->spi_wr_buffer = cpu_to_be32(val << 8);
	ret = spi_sync_transfer(st->spi, &t, 1); // write buffer

//	st->debug = st->spi_rd_buffer;

	return ret;
}


static ssize_t send_all_regs(struct iio_dev *indio_dev)
{
	struct NAME_(state) *st = iio_priv(indio_dev);
	int ret = 0;
	int temp;

	temp = 0;
	temp |= (0x0 << 23);			// R/W (R=1, W=0)
	temp |= (0x0 << 17);			// addr
	temp |= ((st->regs[0]&0xFFFF) << 1);	// data
	temp |= (0x0 << 0);			// parity
	ret = NAME_(spi_transfer)(indio_dev, temp);
	if(ret){ return ret; }

	temp = 0;
	temp |= (0x0 << 23);			// R/W (R=1, W=0)
	temp |= (0x2 << 17);			// addr
	temp |= ((st->regs[2]&0xFFFF) << 1);	// data
	temp |= (0x0 << 0);			// parity
	ret = NAME_(spi_transfer)(indio_dev, temp);
	if(ret){ return ret; }

	temp = 0;
	temp |= (0x0 << 23);			// R/W (R=1, W=0)
	temp |= (0x3 << 17);			// addr
	temp |= ((st->regs[3]&0xFFFF) << 1);	// data
	temp |= (0x0 << 0);			// parity
	ret = NAME_(spi_transfer)(indio_dev, temp);
	if(ret){ return ret; }

	temp = 0;
	temp |= (0x0 << 23);			// R/W (R=1, W=0)
	temp |= (0x4 << 17);			// addr
	temp |= ((st->regs[4]&0xFFFF) << 1);	// data
	temp |= (0x0 << 0);			// parity
	ret = NAME_(spi_transfer)(indio_dev, temp);
	if(ret){ return ret; }

	temp = 0;
	temp |= (0x0 << 23);			// R/W (R=1, W=0)
	temp |= (0x5 << 17);			// addr
	temp |= ((st->regs[5]&0xFFFF) << 1);	// data
	temp |= (0x0 << 0);			// parity
	ret = NAME_(spi_transfer)(indio_dev, temp);
	if(ret){ return ret; }

	temp = 0;
	temp |= (0x0 << 23);			// R/W (R=1, W=0)
	temp |= (0x6 << 17);			// addr
	temp |= ((st->regs[6]&0xFFFF) << 1);	// data
	temp |= (0x0 << 0);			// parity
	ret = NAME_(spi_transfer)(indio_dev, temp);

	return ret;
}

static uint32_t NAME_(spi_readback)(struct iio_dev *indio_dev){
	struct NAME_(state) *st = iio_priv(indio_dev);
	return (be32_to_cpu(st->spi_rd_buffer) >> 8);
}

static int NAME_(reg_access)(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct NAME_(state) *st = iio_priv(indio_dev);
	int ret;
	uint32_t temp;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		temp = 0;
		temp |= (0x0 << 23);			// R/W (R=1, W=0)
		temp |= ((reg & 0x3F) << 17);		// addr
		temp |= ((writeval & 0xFFFF) << 1);	// data
		temp |= (0x0 << 0);			// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);

	} else {
//		st->debug = 1234;
		temp = 0;
		temp |= (0x1 << 23);		// R/W (R=1, W=0)
		temp |= ((reg & 0x3F) << 17);	// addr
		temp |= (0x0 << 1);		// data
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ return ret; }
		*readval = NAME_(spi_readback)(indio_dev);
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t NAME_(store)(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct NAME_(state) *st = iio_priv(indio_dev);
	long val;
	int ret = 0;
//	uint32_t temp;

	ret = kstrtol(buf, 0, &val);
	if (ret < 0)
		return ret;
	
	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case RESERVED_SPI:
		st->reserved_spi = val;
		ret = NAME_(spi_transfer)(indio_dev, st->reserved_spi);
		msleep(10);
		st->reserved_spi = NAME_(spi_readback)(indio_dev);
		break;
	case CONTROL:
		st->regs[0] = val;
		send_all_regs(indio_dev);
//		temp = 0;
//		temp |= (0x0 << 23);			// R/W (R=1, W=0)
//		temp |= (0x0 << 17);			// addr
//		temp |= ((st->regs[0]&0xFFFF) << 1);	// data
//		temp |= (0x0 << 0);			// parity
//		ret = NAME_(spi_transfer)(indio_dev, temp);
		break;
	case ALARM_MASK:
		st->regs[2] = val;
		send_all_regs(indio_dev);
//		temp = 0;
//		temp |= (0x0 << 23);			// R/W (R=1, W=0)
//		temp |= (0x2 << 17);			// addr
//		temp |= ((st->regs[2]&0xFFFF) << 1);	// data
//		temp |= (0x0 << 0);			// parity
//		ret = NAME_(spi_transfer)(indio_dev, temp);
		break;
	case ENABLE:
		st->regs[3] = val;
		send_all_regs(indio_dev);
//		temp = 0;
//		temp |= (0x0 << 23);			// R/W (R=1, W=0)
//		temp |= (0x3 << 17);			// addr
//		temp |= ((st->regs[3]&0xFFFF) << 1);	// data
//		temp |= (0x0 << 0);			// parity
//		ret = NAME_(spi_transfer)(indio_dev, temp);
		break;
	case LINEARIZE:
		st->regs[4] = val;
		send_all_regs(indio_dev);
//		temp = 0;
//		temp |= (0x0 << 23);			// R/W (R=1, W=0)
//		temp |= (0x4 << 17);			// addr
//		temp |= ((st->regs[4]&0xFFFF) << 1);	// data
//		temp |= (0x0 << 0);			// parity
//		ret = NAME_(spi_transfer)(indio_dev, temp);
		break;
	case LO_PATH:
		st->regs[5] = val;
		send_all_regs(indio_dev);
//		temp = 0;
//		temp |= (0x0 << 23);			// R/W (R=1, W=0)
//		temp |= (0x5 << 17);			// addr
//		temp |= ((st->regs[5]&0xFFFF) << 1);	// data
//		temp |= (0x0 << 0);			// parity
//		ret = NAME_(spi_transfer)(indio_dev, temp);
		break;
	case ADC_CONTROL:
		st->regs[6] = val;
		send_all_regs(indio_dev);
//		temp = 0;
//		temp |= (0x0 << 23);			// R/W (R=1, W=0)
//		temp |= (0x6 << 17);			// addr
//		temp |= ((st->regs[6]&0xFFFF) << 1);	// data
//		temp |= (0x0 << 0);			// parity
//		ret = NAME_(spi_transfer)(indio_dev, temp);
		break;
	case RDAC_LINEARIZE:
		st->regs[4] = (st->regs[4] && ~0x00FF) || (val && 0x00FF);
		send_all_regs(indio_dev);
//		temp = 0;
//		temp |= (0x0 << 23);			// R/W (R=1, W=0)
//		temp |= (0x4 << 17);			// addr
//		temp |= ((st->regs[4]&0xFFFF) << 1);	// data
//		temp |= (0x0 << 0);			// parity
//		ret = NAME_(spi_transfer)(indio_dev, temp);
		break;
	case I_PATH_PHASE_AQQURACY:
		st->regs[5] = (st->regs[5] && ~0x000F) || (val && 0x000F);
		send_all_regs(indio_dev);
//		temp = 0;
//		temp |= (0x0 << 23);			// R/W (R=1, W=0)
//		temp |= (0x5 << 17);			// addr
//		temp |= ((st->regs[5]&0xFFFF) << 1);	// data
//		temp |= (0x0 << 0);			// parity
//		ret = NAME_(spi_transfer)(indio_dev, temp);
		break;
	case Q_PATH_PHASE_AQQURACY:
		st->regs[5] = (st->regs[5] && ~0x00F0) || ((val && 0x000F) << 4);
		send_all_regs(indio_dev);
//		temp = 0;
//		temp |= (0x0 << 23);			// R/W (R=1, W=0)
//		temp |= (0x5 << 17);			// addr
//		temp |= ((st->regs[5]&0xFFFF) << 1);	// data
//		temp |= (0x0 << 0);			// parity
//		ret = NAME_(spi_transfer)(indio_dev, temp);
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t NAME_(show)(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct NAME_(state) *st = iio_priv(indio_dev);
	u32 val;
	int ret = 0;
	uint32_t temp;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case RESERVED_SPI:
		val = st->reserved_spi;
		break;
	case CONTROL:
		temp = 0;
		temp |= (0x1 << 23);		// R/W (R=1, W=0)
		temp |= (0x0 << 17);		// addr
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }
		st->regs[0] = NAME_(spi_readback)(indio_dev);
		val = st->regs[0];
		break;
	case ALARM_READBACK:
		temp = 0;
		temp |= (0x1 << 23);		// R/W (R=1, W=0)
		temp |= (0x1 << 17);		// addr
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }
		st->regs[1] = NAME_(spi_readback)(indio_dev);
		val = st->regs[1];
		break;
	case ALARM_MASK:
		temp = 0;
		temp |= (0x1 << 23);		// R/W (R=1, W=0)
		temp |= (0x2 << 17);		// addr
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }
		st->regs[2] = NAME_(spi_readback)(indio_dev);
		val = st->regs[2];
		break;
	case ENABLE:
		temp = 0;
		temp |= (0x1 << 23);		// R/W (R=1, W=0)
		temp |= (0x3 << 17);		// addr
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }
		st->regs[3] = NAME_(spi_readback)(indio_dev);
		val = st->regs[3];
		break;
	case LINEARIZE:
		temp = 0;
		temp |= (0x1 << 23);		// R/W (R=1, W=0)
		temp |= (0x4 << 17);		// addr
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }
		st->regs[4] = NAME_(spi_readback)(indio_dev);
		val = st->regs[4];
		break;
	case LO_PATH:
		temp = 0;
		temp |= (0x1 << 23);		// R/W (R=1, W=0)
		temp |= (0x5 << 17);		// addr
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }
		st->regs[5] = NAME_(spi_readback)(indio_dev);
		val = st->regs[5];
		break;
	case ADC_CONTROL:
		temp = 0;
		temp |= (0x1 << 23);		// R/W (R=1, W=0)
		temp |= (0x6 << 17);		// addr
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }
		st->regs[6] = NAME_(spi_readback)(indio_dev);
		val = st->regs[6];
		break;
	case ADC_OUTPUT:
		temp = 0;
		temp |= (0x0 << 23);		// R/W (R=1, W=0)
		temp |= (0x6 << 17);		// addr
		temp |= (0x7 << 1);		// data
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }

		msleep(1);	// sleep more than 200us

		temp = 0;
		temp |= (0x1 << 23);		// R/W (R=1, W=0)
		temp |= (0xC << 17);		// addr
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }
		if( !(NAME_(spi_readback)(indio_dev) & (1 << 8)) ){	// check bit 8 of reg 0xC
			ret = -ENODEV;
			break;
		}

		temp = 0;
		temp |= (0x0 << 23);		// R/W (R=1, W=0)
		temp |= (0x6 << 17);		// addr
		temp |= (0x3 << 1);		// data
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }

		temp |= (0x1 << 23);		// R/W (R=1, W=0)
		temp |= (0xC << 17);		// addr
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }
		st->regs[0xC] = NAME_(spi_readback)(indio_dev);
		val = st->regs[0xC];
		break;
//	case DEBUG:
//		val = st->debug;
//		break;
	case RDAC_LINEARIZE:
		temp = 0;
		temp |= (0x1 << 23);		// R/W (R=1, W=0)
		temp |= (0x4 << 17);		// addr
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }
		st->regs[4] = NAME_(spi_readback)(indio_dev);
		val = st->regs[4] && 0x00FF;
		break;
	case I_PATH_PHASE_AQQURACY:
		temp = 0;
		temp |= (0x1 << 23);		// R/W (R=1, W=0)
		temp |= (0x5 << 17);		// addr
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }
		st->regs[5] = NAME_(spi_readback)(indio_dev);
		val = st->regs[5] && 0x000F;
		break;
	case Q_PATH_PHASE_AQQURACY:
		temp = 0;
		temp |= (0x1 << 23);		// R/W (R=1, W=0)
		temp |= (0x5 << 17);		// addr
		temp |= (0x0 << 0);		// parity
		ret = NAME_(spi_transfer)(indio_dev, temp);
		if(ret){ break; }
		st->regs[5] = NAME_(spi_readback)(indio_dev);
		val = ((st->regs[5] && 0x00F0) >> 4);
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0){
		ret = sprintf(buf, "%d\n", val);
	}
	return ret;
}

static IIO_DEVICE_ATTR(reserved_spi, S_IRUGO | S_IWUSR,
			NAME_(show),
			NAME_(store),
			RESERVED_SPI);

static IIO_DEVICE_ATTR(control, S_IRUGO | S_IWUSR,
			NAME_(show),
			NAME_(store),
			CONTROL);

static IIO_DEVICE_ATTR(alarm_readback, S_IRUGO | S_IWUSR,
			NAME_(show),
			NAME_(store),
			ALARM_READBACK);

static IIO_DEVICE_ATTR(alarm_mask, S_IRUGO | S_IWUSR,
			NAME_(show),
			NAME_(store),
			ALARM_MASK);

static IIO_DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
			NAME_(show),
			NAME_(store),
			ENABLE);

static IIO_DEVICE_ATTR(linearize, S_IRUGO | S_IWUSR,
			NAME_(show),
			NAME_(store),
			LINEARIZE);

static IIO_DEVICE_ATTR(lo_path, S_IRUGO | S_IWUSR,
			NAME_(show),
			NAME_(store),
			LO_PATH);

static IIO_DEVICE_ATTR(adc_control, S_IRUGO | S_IWUSR,
			NAME_(show),
			NAME_(store),
			ADC_CONTROL);

static IIO_DEVICE_ATTR(adc_output, S_IRUGO | S_IWUSR,
			NAME_(show),
			NAME_(store),
			ADC_OUTPUT);

//static IIO_DEVICE_ATTR(debug, S_IRUGO | S_IWUSR,
//			NAME_(show),
//			NAME_(store),
//			DEBUG);

static IIO_DEVICE_ATTR(rdac_linearize, S_IRUGO | S_IWUSR,
			NAME_(show),
			NAME_(store),
			RDAC_LINEARIZE);

static IIO_DEVICE_ATTR(i_path_phase_aqquracy, S_IRUGO | S_IWUSR,
			NAME_(show),
			NAME_(store),
			I_PATH_PHASE_AQQURACY);

static IIO_DEVICE_ATTR(q_path_phase_aqquracy, S_IRUGO | S_IWUSR,
			NAME_(show),
			NAME_(store),
			Q_PATH_PHASE_AQQURACY);

static struct attribute *NAME_(attributes)[] = {
	&iio_dev_attr_reserved_spi.dev_attr.attr,
	&iio_dev_attr_control.dev_attr.attr,
	&iio_dev_attr_alarm_readback.dev_attr.attr,
	&iio_dev_attr_alarm_mask.dev_attr.attr,
	&iio_dev_attr_enable.dev_attr.attr,
	&iio_dev_attr_linearize.dev_attr.attr,
	&iio_dev_attr_lo_path.dev_attr.attr,
	&iio_dev_attr_adc_control.dev_attr.attr,
	&iio_dev_attr_adc_output.dev_attr.attr,
//	&iio_dev_attr_debug.dev_attr.attr,
	&iio_dev_attr_rdac_linearize.dev_attr.attr,
	&iio_dev_attr_i_path_phase_aqquracy.dev_attr.attr,
	&iio_dev_attr_q_path_phase_aqquracy.dev_attr.attr,
	NULL
};

static const struct attribute_group NAME_(attribute_group) = {
	.attrs = NAME_(attributes),
};

static const struct iio_info NAME_(info) = {
	.debugfs_reg_access = &NAME_(reg_access),
	.attrs = &NAME_(attribute_group),
};

static int NAME_(probe)(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct device_node *np = spi->dev.of_node;
	struct NAME_(state) *st;
	int ret;
	uint32_t temp;

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

	if(of_property_read_string(np, "optional,unique-id", &st->unique_id))
		st->unique_id = "";	// default

	/* try to get unique id */
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

	indio_dev->info = &NAME_(info);
	indio_dev->modes = INDIO_DIRECT_MODE;

	// initialize to default
	st->regs[0] = 0x0075;
	temp = 0;
	temp |= (0x0 << 23);		// R/W (R=1, W=0)
	temp |= (0x0 << 17);		// addr
	temp |= (st->regs[0] << 1);	// data
	temp |= (0x0 << 0);		// parity
	ret = NAME_(spi_transfer)(indio_dev, temp);
	if(ret) {return ret;}

	st->regs[2] = 0xFFFF;
	temp = 0;
	temp |= (0x0 << 23);		// R/W (R=1, W=0)
	temp |= (0x2 << 17);		// addr
	temp |= (st->regs[2] << 1);	// data
	temp |= (0x0 << 0);		// parity
	ret = NAME_(spi_transfer)(indio_dev, temp);
	if(ret) {return ret;}

	st->regs[3] = 0x0157;
	st->regs[3] |= (1 << 7);	// DETECTOR_ENABLE
	temp = 0;
	temp |= (0x0 << 23);		// R/W (R=1, W=0)
	temp |= (0x3 << 17);		// addr
	temp |= (st->regs[3] << 1);	// data
	temp |= (0x0 << 0);		// parity
	ret = NAME_(spi_transfer)(indio_dev, temp);
	if(ret) {return ret;}

	st->regs[4] = 0x0080;
	temp = 0;
	temp |= (0x0 << 23);		// R/W (R=1, W=0)
	temp |= (0x4 << 17);		// addr
	temp |= (st->regs[4] << 1);	// data
	temp |= (0x0 << 0);		// parity
	ret = NAME_(spi_transfer)(indio_dev, temp);
	if(ret) {return ret;}

	st->regs[5] = 0x0000;
	temp = 0;
	temp |= (0x0 << 23);		// R/W (R=1, W=0)
	temp |= (0x5 << 17);		// addr
	temp |= (st->regs[5] << 1);	// data
	temp |= (0x0 << 0);		// parity
	ret = NAME_(spi_transfer)(indio_dev, temp);
	if(ret) {return ret;}

	st->regs[6] = 0x0000;
	st->regs[6] |= (1 << 1);	// ADC_ENABLE
	temp = 0;
	temp |= (0x0 << 23);		// R/W (R=1, W=0)
	temp |= (0x6 << 17);		// addr
	temp |= (st->regs[6] << 1);	// data
	temp |= (0x0 << 0);		// parity
	ret = NAME_(spi_transfer)(indio_dev, temp);
	if(ret) {return ret;}

	st->regs[6] |= (1 << 2);	// ADC_CLOCK_ENABLE
	temp |= (st->regs[6] << 1);	// data
	ret = NAME_(spi_transfer)(indio_dev, temp);
	if(ret) {return ret;}


	ret = iio_device_register(indio_dev);

	return ret;
}

static int NAME_(remove)(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	iio_device_unregister(indio_dev);
	return 0;
}

static const struct spi_device_id NAME_(id)[] = {
	{DRIVER_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(spi, NAME_(id));

static struct spi_driver NAME_(driver) = {
	.driver = {
		.name	= KBUILD_MODNAME,
		.owner	= THIS_MODULE,
	},
	.probe		= NAME_(probe),
	.remove		= NAME_(remove),
	.id_table	= NAME_(id),
};
module_spi_driver(NAME_(driver));

MODULE_AUTHOR("Cyril Zwahlen <zwahlen@precisionwave.com>");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_LICENSE("GPL v2");

