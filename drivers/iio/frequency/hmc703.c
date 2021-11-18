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

/*
 * HMC703 Specific Definitions
 */
#define DRIVER_NAME			"hmc703"
#define DEFAULT_CP_CURRENT_COUNTS	127
#define DEFAULT_N_DIV			20
#define DEFAULT_R_DIV			2

/*
 * HMC703 Default Register Values
 *
 * Default initial Values, MSB first
 * Buildup: [d23:d0][r4:r0]['000'] --> 32bit of data
 * Shifting the register >> 8 sets D0 at LSB-position
 * Last 3 bits in the data Reg are always '0'
 */
#define HMC703_REG_COUNT 	16
static uint32_t default_regs[16] = {
	0x00000000, // Write only, Soft-Reset, SPI-Reading-Command
	0x00000208, // RST-Register
	0x00000210, // rDiv - Reference Divider 'R', 1...16383
	0x00001418, // intg - VCO Divider 'N', 16...65535
	0x00000020, // frac
	0x65432128, // SEED
	0x001F3F30, // SD CFG Reg
	0x00486538, // Lock Detect Reg
	0x016FFF40, // Analog EN Reg
	0x003FFF48, // Charge Pump Reg
	0x00000050, // Modstep
	0x01E06158, // PD Register
	0x00001960, // ALTINT
	0x00000068, // ALTFRAC
	0x00000070, // SPI TRIG
	0x0000C178, // GPO Register
};

// Enum for show- and store-switch-case
enum attributes{
	DEVICE_NAME,
	CP_CURRENT_COUNTS,
	N_DIV,
	R_DIV,
};

// HMC703-Definition
struct hmc703_state{
	struct spi_device *spi;
	const char* unique_id;
	//uint32_t spi_rd_buffer;
	//uint32_t spi_wr_buffer;
	/* spi register */
	uint32_t hmc703_reg[16]; // Initial Values HMC703
	/* dummy attributes */
	char* device_name;
};



/*
 * Store-Function
 */
static ssize_t hmc703_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct hmc703_state *st = iio_priv(indio_dev);
	long val;
	int ret = 0;
	int temp = 0;

	// Convert input-string to a long and cast into a 32bit int
	ret = kstrtol(buf, 0, &val);
	if(ret){
		return ret;
	}

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address){
		case DEVICE_NAME:
			if(st->device_name)
				kfree(st->device_name);
			st->device_name = kasprintf(GFP_KERNEL, "%s", buf);
			break;

		case CP_CURRENT_COUNTS:
			// Ins Schattenregister speichern
			st->hmc703_reg[9] = (st->hmc703_reg[9] & ~(0x7F << (0+8))) | ((u32)val << (0+8)); // HMC703 Reg09h, Bit0...6, CPIdn, Same value for pos and neg Charge Pump
			st->hmc703_reg[9] = (st->hmc703_reg[9] & ~(0x7F << (7+8))) | ((u32)val << (7+8)); // HMC703 Reg09h, Bit7...13, CPIup
			// auf SPI Schreiben
			temp = cpu_to_be32(st->hmc703_reg[9]);
			ret = spi_write(st->spi, &temp, 4);	// write 4 bytes
			if (ret < 0)
				dev_err(&indio_dev->dev, "write failed (%d)", ret);
			break;

		case N_DIV:
			// Ins Schattenregister speichern
			st->hmc703_reg[3] = (st->hmc703_reg[3] & ~(0xFFFF << (0+8))) | ((u32)val << (0+8)); // HMC703 Reg03h, Bit0...15, intg
			// auf SPI Schreiben
			temp = cpu_to_be32(st->hmc703_reg[3]);
			ret = spi_write(st->spi, &temp, 4);	// write 4 bytes
			if (ret < 0)
				dev_err(&indio_dev->dev, "write failed (%d)", ret);
			break;

		case R_DIV:
			// Ins Schattenregister speichern
			st->hmc703_reg[2] = (st->hmc703_reg[2] & ~(0x3FFF << (0+8))) | ((u32)val << (0+8)); // HMC703 Reg02h, Bit0...13, rDiv
			// auf SPI Schreiben
			temp = cpu_to_be32(st->hmc703_reg[2]);
			ret = spi_write(st->spi, &temp, 4);	// write 4 bytes
			if (ret < 0)
				dev_err(&indio_dev->dev, "write failed (%d)", ret);
			break;

		default:
			ret = -ENODEV;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}


/*
 * Show-Function
 */
static ssize_t hmc703_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct hmc703_state *st = iio_priv(indio_dev);
	uint32_t val = 0;
	int ret = 0;
	int match = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address){
		case DEVICE_NAME:
			if(st->device_name)
				ret = sprintf(buf, "%s\n", st->device_name);
			else
				ret = sprintf(buf, "\n");
			mutex_unlock(&indio_dev->mlock);
			match = 1;
			return ret;

		case CP_CURRENT_COUNTS:
			val = (st->hmc703_reg[9] >> (0+8)) & 0x7F; // HMC703 Reg09h, Bit0...6, CPIdn, Same value for pos and neg Charge Pump
			match = 1;
			break;

		case N_DIV:
			val = (st->hmc703_reg[3] >> (0+8)) & 0xFFFF; // HMC703 Reg03h, Bit0...15, intg
			match = 1;
			break;

		case R_DIV:
			val = (st->hmc703_reg[2] >> (0+8)) & 0x3FFF; // HMC703 Reg02h, Bit0...13, rDiv
			match = 1;
			break;

		default:
			ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);
	if(match){
		if(ret==0){
			ret = sprintf(buf, "%d\n", val);
		}
	}
	return ret;
}


/*
 * Attribute Macros
 */
static IIO_DEVICE_ATTR(Device_Name, S_IRUGO | S_IWUSR,
			hmc703_show,
			hmc703_store,
			DEVICE_NAME);

static IIO_DEVICE_ATTR(cp_current, S_IRUGO | S_IWUSR,
			hmc703_show,
			hmc703_store,
			CP_CURRENT_COUNTS);

static IIO_DEVICE_ATTR(n_div, S_IRUGO | S_IWUSR,
			hmc703_show,
			hmc703_store,
			N_DIV);

static IIO_DEVICE_ATTR(r_div, S_IRUGO | S_IWUSR,
			hmc703_show,
			hmc703_store,
			R_DIV);


/*
 * Attribute Definitions
 */
static struct attribute *hmc703_attributes[] = {
	&iio_dev_attr_Device_Name.dev_attr.attr,
	&iio_dev_attr_cp_current.dev_attr.attr,
	&iio_dev_attr_n_div.dev_attr.attr,
	&iio_dev_attr_r_div.dev_attr.attr,
	NULL
};

static const struct attribute_group hmc703_attribute_group = {
	.attrs = hmc703_attributes,
};

static const struct iio_info hmc703_info = {
	.attrs = &hmc703_attribute_group,
};



/*
 * Device-Tree Parser
 */
static int hmc703_parse_dt(struct device *dev, struct hmc703_state *st){
	struct device_node *np = dev->of_node;
	int val;

	/* try to get unique id */
	if(of_property_read_string(np, "optional,unique-id", &st->unique_id))
		st->unique_id = "";  // default

	/* try to get initial value for charge pump output current */
	if(of_property_read_u32(np, "optional,CP-Current", &val)){
		val = (st->hmc703_reg[9] >> (0+8)) & 0x7F;	// If not defined, take default value out of register
		pr_warning(DRIVER_NAME" >> CP-Current: using default value = %d\n", val);
	}
	st->hmc703_reg[9] = (st->hmc703_reg[9] & ~(0x7F << (0+8))) | (val << (0+8)); // HMC703 Reg09h, Bit0...6, CPIdn, Same value for pos and neg Charge Pump
	st->hmc703_reg[9] = (st->hmc703_reg[9] & ~(0x7F << (7+8))) | (val << (7+8)); // HMC703 Reg09h, Bit7...13, CPIup

	/* try to get initial value for N Divider VCO-Input */
	if(of_property_read_u32(np, "optional,N-DIV-VCO", &val)){
		val = (st->hmc703_reg[3] >> (0+8)) & 0xFFFF;	// If not defined, take default value out of register
		pr_warning(DRIVER_NAME" >> N-DIV-VCO: using default value = %d\n", val);
	}
	st->hmc703_reg[3] = (st->hmc703_reg[3] & ~(0xFFFF << (0+8))) | (val << (0+8)); // HMC703 Reg03h, Bit0...15, intg

	/* try to get initial value for R Divider REF-Input */
	if(of_property_read_u32(np, "optional,R-DIV-REF", &val)){
		val = (st->hmc703_reg[2] >> (0+8)) & 0x3FFF;	// If not defined, take default value out of register
		pr_warning(DRIVER_NAME" >> R-DIV-REF: using default value = %d\n", val);
	}
	st->hmc703_reg[2] = (st->hmc703_reg[2] & ~(0x3FFF << (0+8))) | (val << (0+8)); // HMC703 Reg02h, Bit0...13, rDiv

	return 0;
}


/*
 * Probe Function
 */
static int hmc703_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct device_node *np = spi->dev.of_node;
	struct hmc703_state *st;
	int ret;
	int temp = 0; //Temporary for converting SPI order MSB<-->LSB
	int i; //For Loop Counter


	if (!np)
		return -ENODEV;
	printk("HMC703: Start Probing");
	dev_dbg(&spi->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	spi_set_drvdata(spi, indio_dev);

	st = iio_priv(indio_dev);
	st->spi = spi;
	indio_dev->dev.parent = &spi->dev;

	// Initialize shadow-reg with default values
	for(i=0; i<HMC703_REG_COUNT; i++){
		st->hmc703_reg[i] = default_regs[i];
	}

	/* parse devicetree */
	ret = hmc703_parse_dt(&spi->dev, st);
	if(ret){
		return ret;
	}

	printk("HMC703: Devicetree Parsing...");
	/* If all Registers are parsed and/or default-defined, write it once on the SPI */

	/*
	for(i=0; i<HMC703_REG_COUNT; i++){
		temp = cpu_to_be32(st->hmc703_reg[i]);
		ret = spi_write(st->spi, temp&, 4);	// write 4 bytes
		if (ret < 0)
			dev_err(&indio_dev->dev, "write failed (%d)", ret);
	}
	*/
	// re-write the non-default registers
	temp = cpu_to_be32(st->hmc703_reg[2]);
	ret = spi_write(st->spi, &temp, 4);	// R-Div
	temp = cpu_to_be32(st->hmc703_reg[3]);
	ret = spi_write(st->spi, &temp, 4);	// N-Div
	temp = cpu_to_be32(st->hmc703_reg[6]);
	ret = spi_write(st->spi, &temp, 4);	// Mode
	temp = cpu_to_be32(st->hmc703_reg[9]);
	ret = spi_write(st->spi, &temp, 4);	// Charge-Pump
	temp = cpu_to_be32(st->hmc703_reg[15]);
	ret = spi_write(st->spi, &temp, 4);	// GPO-Pin-Setting


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


	indio_dev->info = &hmc703_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	ret = iio_device_register(indio_dev);

	printk("HMC703: Finished Probing");
	return ret;
}




/*
 * Driver and Register Functions
 */
static int hmc703_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	iio_device_unregister(indio_dev);
	return 0;
}

static const struct spi_device_id hmc703_id[] = {
	{DRIVER_NAME, 0},
	{}
};

static struct spi_driver hmc703_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= hmc703_probe,
	.remove		= hmc703_remove,
	.id_table	= hmc703_id,
};
module_spi_driver(hmc703_driver);


MODULE_AUTHOR("Tobias Batt <batt@precisionwave.com>");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_LICENSE("GPL v2");
