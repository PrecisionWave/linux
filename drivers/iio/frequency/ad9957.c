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
 #include <linux/dma-mapping.h>

 #include <linux/iio/iio.h>
 #include <linux/iio/sysfs.h>
 #include <linux/iio/buffer-dma.h>
 #include <linux/iio/buffer-dmaengine.h>


#define NUM_TX_CH       4

struct ad9957_state {
	struct device		*dev;
	struct spi_device	*spi;
	struct mutex 		lock;
	struct clk			*clk;
	u64					dac_clk;
	u64					center_frequency;
	u64					sampling_freq;
	u32					num_ch;
};

struct ad9957_tx_state {
	char name[20];
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

#define DECLARE_AD9957_CHANNELS(name)	\
static struct iio_chan_spec name[] = {	\
		AD9957_CHAN(0), \
		AD9957_CHAN(1), \
}

DECLARE_AD9957_CHANNELS(ad9957_channels);


static int ad9957_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	struct ad9957_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
// 	case IIO_CHAN_INFO_SCALE:
// 		ret = regulator_get_voltage(st->vref);
// 		if (ret < 0)
// 			return ret;
//
// 		*val = 2 * (ret / 1000);
// 		*val2 = chan->scan_type.realbits;
// 		return IIO_VAL_FRACTIONAL_LOG2;
// 	case IIO_CHAN_INFO_SAMP_FREQ:
// 		*val = st->sampling_freq;
// 		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad9957_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	struct ad9957_state *st = iio_priv(indio_dev);

	switch (mask) {
// 	case IIO_CHAN_INFO_SAMP_FREQ:
// 		return ad7768_samp_freq_config(st, val);
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad9957_info = {
	.read_raw = &ad9957_read_raw,
	.write_raw = &ad9957_write_raw,
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

static void ad9957_init(struct ad9957_state *st)
{
	struct spi_transfer xfer = {};
	int ret;
	u8 buffer5[5];
	u8 buffer9[9];

	// Control Function Register 2 CFR2 (0x01)
	buffer5[0] = 0x01;
	buffer5[1] = 0x00;
	buffer5[2] = 0x40;
	buffer5[3] = 0x28;
	buffer5[4] = 0x20;

	mutex_lock(&st->lock);

	xfer.len = sizeof(buffer5);
	xfer.tx_buf = &buffer5;

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret) {
		dev_err(&st->spi->dev, "CFR2 setup failed, status=%d", ret);
		goto error_ret;
	}

	// Control Function Register 3 CFR3 (0x02)
	buffer5[0] = 0x02;
	buffer5[1] = 0x1e;
	buffer5[2] = 0x3f;
	buffer5[3] = 0xc0;
	buffer5[4] = 0x00;

	xfer.len = sizeof(buffer5);
	xfer.tx_buf = &buffer5;

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret) {
		dev_err(&st->spi->dev, "CFR3 setup failed, status=%d", ret);
		goto error_ret;
	}

	// Auxiliary DAC Control Register (0x03)
	buffer5[0] = 0x03;
	buffer5[1] = 0x00;
	buffer5[2] = 0x00;
	buffer5[3] = 0xff;
	buffer5[4] = 0x7f;

	xfer.len = sizeof(buffer5);
	xfer.tx_buf = &buffer5;

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret) {
		dev_err(&st->spi->dev, "AuxDac setup failed, status=%d", ret);
		goto error_ret;
	}

	// Profile 0 Register - QDUC (0x0E)
	buffer9[0] = 0x0e;
	buffer9[1] = 0x0c;
	buffer9[2] = 0xb0;
	buffer9[3] = 0x00;
	buffer9[4] = 0x00;
	buffer9[5] = 0x35;	// 0x35 55 55 55 = Frequency tuning word for fc=204.8MHz when fdac=983.04 MHz
	buffer9[6] = 0x55;
	buffer9[7] = 0x55;
	buffer9[8] = 0x55;

	xfer.len = sizeof(buffer9);
	xfer.tx_buf = &buffer9;

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret) {
		dev_err(&st->spi->dev, "Profile 0 - QDUC setup failed, status=%d", ret);
		goto error_ret;
	}

	// Control Function Register 1 CFR1 (0x00)
	buffer5[0] = 0x00;
	buffer5[1] = 0x00;
	buffer5[2] = 0x00;
	buffer5[3] = 0x00;
	buffer5[4] = 0x00;

	xfer.len = sizeof(buffer5);
	xfer.tx_buf = &buffer5;

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret) {
		dev_err(&st->spi->dev, "CFR1 setup failed, status=%d", ret);
		goto error_ret;
	}

error_ret:
	mutex_unlock(&st->lock);
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

	mutex_init(&st->lock);

	st->clk = devm_clk_get(&spi->dev, "clk");
	if (IS_ERR(st->clk)) {
		dev_err(&spi->dev, "devm_clk_get failed");
		return PTR_ERR(st->clk);
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
	ret = clk_prepare_enable(st->clk);
	if (ret < 0) {
		dev_err(&spi->dev, "clk_prepare_enable failed");
		goto error_disable_clk;
	}

/*
	spi->max_speed_hz = 2000000;
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_3;
	spi_setup(spi);
*/
	ad9957_init(st);

	return 0;


error_disable_clk:
	clk_disable_unprepare(st->clk);
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
	clk_disable_unprepare(st->clk);

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
