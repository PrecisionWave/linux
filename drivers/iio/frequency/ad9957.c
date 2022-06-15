/*
 * AD9122 SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

 #include <linux/module.h>
 #include <linux/spi/spi.h>
 #include <linux/clk.h>
 #include <linux/clk-provider.h>
 #include <linux/dma-mapping.h>

 #include <linux/iio/iio.h>
 #include <linux/iio/sysfs.h>
 #include <linux/iio/buffer-dma.h>
 #include <linux/iio/buffer-dmaengine.h>


#define NUM_TX_CH       4

struct ad9957_state {
	struct device		*dev;
	struct spi_device	*spi;
	struct mutex 		spi_mtx;
	struct mutex 		attr_mtx;
	struct clk			*ref_clk;
	struct notifier_block	ref_clk_rate_change_nb;
	struct clk_hw			*pdclk_hw;
	unsigned long				sysclk_frequency;
	unsigned long				center_frequency;
	unsigned long				pdclk_frequency;
	u32					ftw;
};

struct ad9957_tx_state {
	char name[20];
};

enum {
	AD9957_CENTER_FREQUENCY,
	AD9957_FTW,
	AD9957_RESET,
};

#define AD9957_CHAN(index)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.output = 1,						\
		.channel = index,					\
		.address = index,					\
		.scan_index = index,					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 16,					\
			.storagebits = 16,				\
		},							\
	}

static struct iio_chan_spec ad9957_channels[] = {
		AD9957_CHAN(0),
		AD9957_CHAN(1),
};

static int ad9957_write_32(struct ad9957_state *st, u8 instruction, u32 word)
{
	u8 buffer[] = {
		instruction,
		(word >> 24) & 0xff,
		(word >> 16) & 0xff,
		(word >> 8) & 0xff,
		(word >> 0) & 0xff
	};

	struct spi_transfer xfer = {
		.len = sizeof(buffer),
		.tx_buf = &buffer,
	};

	int ret;

	mutex_lock(&st->spi_mtx);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		dev_dbg(&st->spi->dev, "Failed to write 0x%08x to register %d (ret was %d)", instruction, word, ret);

	mutex_unlock(&st->spi_mtx);
	return ret;
}

static int ad9957_write_64(struct ad9957_state *st, u8 instruction, u32 word1, u32 word2)
{
	u8 buffer[] = {
		instruction,

		(word1 >> 24) & 0xff,
		(word1 >> 16) & 0xff,
		(word1 >> 8) & 0xff,
		(word1 >> 0) & 0xff,

		(word2 >> 24) & 0xff,
		(word2 >> 16) & 0xff,
		(word2 >> 8) & 0xff,
		(word2 >> 0) & 0xff
	};

	struct spi_transfer xfer = {
		.len = sizeof(buffer),
		.tx_buf = &buffer,
	};

	int ret;

	mutex_lock(&st->spi_mtx);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		dev_dbg(&st->spi->dev, "Failed to write 0x%08x%08x to register %d (ret was %d)", instruction, word1, word2, ret);

	mutex_unlock(&st->spi_mtx);
	return ret;
}

static void ad9957_init(struct ad9957_state *st)
{
	int ret;

	// Control Function Register 2 CFR2 (0x01)
	ret = ad9957_write_32(st, 0x01, 0x00402820);
	if (ret) {
		dev_err(&st->spi->dev, "CFR2 setup failed, status=%d", ret);
	}

	// Control Function Register 3 CFR3 (0x02)
	ret = ad9957_write_32(st, 0x02, 0x1e3dc000);
	if (ret) {
		dev_err(&st->spi->dev, "CFR3 setup failed, status=%d", ret);
	}

	// Auxiliary DAC Control Register (0x03)
	ret = ad9957_write_32(st, 0x03, 0x0000ff7f);
	if (ret) {
		dev_err(&st->spi->dev, "AuxDac setup failed, status=%d", ret);
	}

	// Profile 0 Register - QDUC (0x0E)
	// 0x35 55 55 55 = Frequency tuning word for fc=204.8MHz when fdac=983.04 MHz
	ret = ad9957_write_64(st, 0x0e, 0x0cb00000, 0x35555555);
	if (ret) {
		dev_err(&st->spi->dev, "Profile 0 - QDUC setup failed, status=%d", ret);
	}
	else {
		st->ftw = 0x35555555;
	}

	// Control Function Register 1 CFR1 (0x00)
	ret = ad9957_write_32(st, 0x00, (1<<21));
	if (ret) {
		dev_err(&st->spi->dev, "CFR1 setup failed, status=%d", ret);
	}
}

static ssize_t ad9957_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9957_state *st = iio_priv(indio_dev);
	long val, val2;
	u64 val64, val2_64;
	u32 rem;
	int ret;
	char s[101], *s1, s2[4] = "000", *p = s;
	const char delim = '.';

	memset(s, '\0', 101);
	strncpy(s, buf, 100);

	/* handle float value */
	s1 = strsep(&p, &delim);
	if (!s1) {
		return -EINVAL;
	}
	if (p) {
		int l = 3;
		char* pos;
		pos = strchr(p, '\n');
		if (pos) {
			l = pos - p;
			if (l > 3)
				l = 3;
		}
		strncpy(s2, p, l);
		while (strlen(s2) < 3) {
			strcat(s2, "0");
		}
		ret = kstrtol(s1, 10, &val);
		if (ret < 0)
			return ret;
		ret = kstrtol(s2, 10, &val2);
		if (ret < 0)
			return ret;
	}
	else {
		ret = kstrtol(buf, 0, &val);
		if (ret < 0)
			return ret;
		val2 = 0;
	}

	// printk("val  =  %ld\n",val);
	// printk("val2 =  %ld\n",val2);

	mutex_lock(&st->attr_mtx);
	switch ((u32)this_attr->address) {
	case AD9957_CENTER_FREQUENCY:
		/* calc FTW: round(2^32 * (f_out / f_sysclk)) */
		val64 = 0x100000000ULL * val;
		val64 = div_u64_rem(val64, st->sysclk_frequency, &rem);

		val2_64 = 0x100000000ULL * val2;
		val2_64 += rem * 1000ull;
		val2_64 = DIV_ROUND_CLOSEST_ULL(val2_64, st->sysclk_frequency);
		val2_64 = DIV_ROUND_CLOSEST_ULL(val2_64, 1000);

		val64 += val2_64;

		if (val64 > 0xffffffff) {
			val64 = 0xffffffff;
		}

		ret = ad9957_write_64(st, 0x0e, 0x0cb00000, (u32)val64);
		if (ret) {
			dev_err(&st->spi->dev, "Profile 0 - DDS configuration failed, status=%d", ret);
		}
		else {
			st->ftw = (u32)val64;
		}
		break;
	case AD9957_RESET:
		ad9957_init(st);
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&st->attr_mtx);

	return ret ? ret : len;
}

static ssize_t ad9957_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9957_state *st = iio_priv(indio_dev);
	u64 val64, val2_64;
	int ret;

	mutex_lock(&st->attr_mtx);
	switch ((u32)this_attr->address) {
	case AD9957_CENTER_FREQUENCY:
		/* calc f_out: (FTW / 2^32) * f_sysclk */
		val64 = st->ftw;
		val64 *= st->sysclk_frequency;
		val64 = div64_u64_rem(val64, 0x100000000ULL, &val2_64);

		val2_64 *= 1000;
		val2_64 *= 2;
		val2_64 = div64_u64(val2_64, 0x100000000ULL);
		val2_64 += 1;
		val2_64 = div_u64(val2_64, 2);

		ret = sprintf(buf, "%u.%03u\n", (u32)val64, (u32)val2_64);
		break;
	case AD9957_FTW:
		ret = sprintf(buf, "0x%08X\n", st->ftw);
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&st->attr_mtx);

	return ret;
}

static IIO_DEVICE_ATTR(center_frequency, S_IRUGO | S_IWUSR,
			ad9957_show,
			ad9957_store,
			AD9957_CENTER_FREQUENCY);

static IIO_DEVICE_ATTR(ftw, S_IRUGO,
			ad9957_show,
			ad9957_store,
			AD9957_FTW);

static IIO_DEVICE_ATTR(reset, S_IWUSR,
			ad9957_show,
			ad9957_store,
			AD9957_RESET);

static struct attribute *ad9957_attributes[] = {
	&iio_dev_attr_center_frequency.dev_attr.attr,
	&iio_dev_attr_ftw.dev_attr.attr,
	&iio_dev_attr_reset.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9957_attribute_group = {
	.attrs = ad9957_attributes,
};

static const struct iio_info ad9957_info = {
	.attrs = &ad9957_attribute_group,
};

/* we need that dummy function to get a valid iio_info object */
static int ad9957_read_raw_dummy(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	return -EINVAL;
}

static const struct iio_info ad9957_tx_info = {
	.read_raw = &ad9957_read_raw_dummy,
};

static int hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	return iio_dmaengine_buffer_submit_block(queue, block, DMA_TO_DEVICE);
}

static const struct iio_dma_buffer_ops dma_buffer_ops = {
	.submit = hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static void ad5597_update_ref_clk(struct ad9957_state *st, unsigned long ref_clk_rate)
{
	/* Assume SYSCLK directly driven from REF_CLK input pin
	   - PLL ENABLE in CFR3, BIT 8 = 0
	   - REFCLK INPUT DIVIDER BYPASS CFR3, BIT 15 = 1
	*/
	st->sysclk_frequency = ref_clk_rate;
	dev_info(&st->spi->dev, "SYSCLK frequency is %lu Hz\n", st->sysclk_frequency);

	/* With our settings f_PDCLK is f_SYSCLK/12. */
	st->pdclk_frequency = st->sysclk_frequency / 12;
	dev_info(&st->spi->dev, "PDCLK frequency is %lu Hz\n", st->pdclk_frequency);
}

#define nb_to_ad9957_state(x) \
		container_of(x, struct ad9957_state, ref_clk_rate_change_nb)

static int ref_clk_clock_notifier(struct notifier_block *nb,
				  unsigned long event, void *data)
{
	struct clk_notifier_data *ndata = data;
	struct ad9957_state *st = nb_to_ad9957_state(nb);

	dev_info(st->dev, "ref_clk rate change: new rate = %lu Hz\n", ndata->new_rate);

	if (event == POST_RATE_CHANGE) {
		ad5597_update_ref_clk(st, ndata->new_rate);
		to_clk_fixed_rate(st->pdclk_hw)->fixed_rate = st->pdclk_frequency;
		/* HACK: Use reparent to trigger clk notifications */
		clk_hw_reparent(st->pdclk_hw, NULL);
		dev_dbg(st->dev, "recalc rate for pdclk: %lu", clk_hw_get_rate(st->pdclk_hw));
	}

	return NOTIFY_DONE;
}

static int ad9957_probe(struct spi_device *spi)
{
	struct ad9957_state *st;
	struct ad9957_tx_state *st_tx;
	struct iio_dev *indio_dev, *indio_dev_tx[NUM_TX_CH];
	struct iio_buffer *buffer[NUM_TX_CH];
	int ret, i, dma_count;


	/* allocate memory for iio devices & private data */
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(&spi->dev, "devm_iio_device_alloc failed");
		return -ENOMEM;
	}

	for(i=0;i<NUM_TX_CH;i++) {
		indio_dev_tx[i] = devm_iio_device_alloc(&spi->dev, sizeof(*st_tx));
		if (!indio_dev_tx[i]){
			dev_err(&spi->dev, "devm_iio_device_alloc failed");
			return -ENOMEM;
		}
	}


	/* configure & register main IIO device components (SPI) */
	st = iio_priv(indio_dev);

	mutex_init(&st->spi_mtx);
	mutex_init(&st->attr_mtx);

	st->ref_clk = devm_clk_get(&spi->dev, "clk");
	if (IS_ERR(st->ref_clk)) {
		dev_err(&spi->dev, "devm_clk_get failed");
		return PTR_ERR(st->ref_clk);
	}

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;
	st->dev = &spi->dev;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad9957_info;

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret < 0) {
		dev_err(&spi->dev, "devm_iio_device_register failed");
		return ret;
	}

	/* configure & register secondary IIO devices components */
	dma_count = of_property_count_strings(spi->dev.of_node, "dma-names");
	for(i=0; i<NUM_TX_CH && i<dma_count; i++) {
		char dma_ch[10];
		st_tx = iio_priv(indio_dev_tx[i]);
		sprintf(st_tx->name,"ad9957_tx%d",i);
		sprintf(dma_ch,"tx%d",i);
		indio_dev_tx[i]->dev.parent = &spi->dev;
		indio_dev_tx[i]->name = st_tx->name;
		indio_dev_tx[i]->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
		indio_dev_tx[i]->channels = ad9957_channels;
		indio_dev_tx[i]->num_channels = ARRAY_SIZE(ad9957_channels);
		indio_dev_tx[i]->direction = IIO_DEVICE_DIRECTION_OUT;
		indio_dev_tx[i]->info = &ad9957_tx_info;

		buffer[i] = iio_dmaengine_buffer_alloc(indio_dev_tx[i]->dev.parent, dma_ch,
							&dma_buffer_ops, indio_dev_tx[i]);
		if (IS_ERR(buffer[i])) {
			dev_err(&spi->dev, "iio_dmaengine_buffer_alloc failed");
			ret = PTR_ERR(buffer[i]);
			goto error_disable_buf;
		}

		iio_device_attach_buffer(indio_dev_tx[i], buffer[i]);

		iio_device_set_drvdata(indio_dev_tx[i], st);

		devm_iio_device_register(&spi->dev, indio_dev_tx[i]);
	}

	/* prepare / enable clock */
	ret = clk_prepare_enable(st->ref_clk);
	if (ret < 0) {
		dev_err(&spi->dev, "clk_prepare_enable failed");
		goto error_disable_clk;
	}

	/* calculate derived lcoks */
	ad5597_update_ref_clk(st, clk_get_rate(st->ref_clk));

	/* register pdclk as clock provider */
	st->pdclk_hw = clk_hw_register_fixed_rate_with_accuracy(NULL, "pdclk", NULL, 0, st->pdclk_frequency, 0);
	if (IS_ERR(st->pdclk_hw)) {
		dev_err(&spi->dev, "Failed to register pdclk (error %ld)", PTR_ERR(st->pdclk_hw));
		goto error_disable_clk;
	}

	of_clk_add_provider(spi->dev.of_node, of_clk_src_simple_get, st->pdclk_hw->clk);

	/* register notifier to get ref_clk change notifications */
	st->ref_clk_rate_change_nb.notifier_call = ref_clk_clock_notifier;
	clk_notifier_register(st->ref_clk, &st->ref_clk_rate_change_nb);

	/* finally setup registers */
	ad9957_init(st);

	return 0;


error_disable_clk:
	clk_disable_unprepare(st->ref_clk);
error_disable_buf:
	i--;
	for(;i>=0;i--)
		iio_dmaengine_buffer_free(indio_dev_tx[i]->buffer);

	return ret;
}

static int ad9957_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad9957_state *st = iio_priv(indio_dev);

	iio_dmaengine_buffer_free(indio_dev->buffer);
	clk_disable_unprepare(st->ref_clk);

	return 0;
}

static const struct spi_device_id ad9957_id[] = {
	{"ad9957", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9957_id);

static struct spi_driver ad9957_driver = {
	.driver = {
		.name	= "ad9957",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9957_probe,
	.remove		= ad9957_remove,
	.id_table	= ad9957_id,
};
module_spi_driver(ad9957_driver);

MODULE_AUTHOR("Cyril Zwahlen <zwahlen@precisionwave.com>");
MODULE_DESCRIPTION("Analog Devices AD9957 DAC");
MODULE_LICENSE("GPL v2");
