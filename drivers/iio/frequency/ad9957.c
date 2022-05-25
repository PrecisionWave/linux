/*
 * AD9122 SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

 #include <linux/clk.h>
 #include <linux/module.h>
 #include <linux/spi/spi.h>
 #include <linux/dma-mapping.h>
 #include <linux/dmaengine.h>

 #include <linux/iio/iio.h>
 #include <linux/iio/sysfs.h>
 #include <linux/iio/buffer_impl.h>
 #include <linux/iio/buffer-dma.h>
 #include <linux/iio/buffer-dmaengine.h>

struct ad9957_state {
	struct spi_device	*spi;
	struct mutex 		lock;
	struct clk			*clk;
	u64					dac_clk;
	u64					center_frequency;
	u64					sampling_freq;
};

#define AD9957_CHAN(index)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.address = index,					\
		.indexed = 1,						\
		.channel = index,					\
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


// static int ad9957_read_raw(struct iio_dev *indio_dev,
// 			   const struct iio_chan_spec *chan,
// 			   int *val, int *val2, long info)
// {
// 	struct ad9957_state *st = iio_priv(indio_dev);
// 	int ret;
//
// 	switch (info) {
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
// 	default:
// 		return -EINVAL;
// 	}
// }
//
// static int ad9957_write_raw(struct iio_dev *indio_dev,
// 			    struct iio_chan_spec const *chan,
// 			    int val, int val2, long mask)
// {
// 	struct ad9957_state *st = iio_priv(indio_dev);
//
// 	switch (mask) {
// 	case IIO_CHAN_INFO_SAMP_FREQ:
// 		return ad7768_samp_freq_config(st, val);
// 	default:
// 		return -EINVAL;
// 	}
// }
//
// static const struct iio_info ad9957_info = {
// 	.read_raw = &aad9957_read_raw,
// 	.write_raw = &aad9957_write_raw,
// };


// static int dds_buffer_submit_block(struct iio_dma_buffer_queue *queue,
// 	struct iio_dma_buffer_block *block)
// {
// 	struct cf_axi_dds_state *st = iio_priv(queue->driver_data);
//
// 	if (block->block.bytes_used) {
// 		bool enable_fifo = false;
//
// 		if (cf_axi_dds_dma_fifo_en(st) &&
// 			(block->block.flags & IIO_BUFFER_BLOCK_FLAG_CYCLIC)) {
// 			block->block.flags &= ~IIO_BUFFER_BLOCK_FLAG_CYCLIC;
// 			enable_fifo = true;
// 		}
//
// 		cf_axi_dds_pl_ddr_fifo_ctrl(st, enable_fifo);
// 	}
//
// 	return iio_dmaengine_buffer_submit_block(queue, block, DMA_TO_DEVICE);
// }

static int hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	block->block.bytes_used = block->block.size;

	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static const struct iio_dma_buffer_ops dma_buffer_ops = {
	.submit = hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static int ad9957_probe(struct spi_device *spi)
{
	struct ad9957_state *st;
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));		// alloc iio device (contain the spi device and enough space for the private data)
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);										// link pointer to private data

	st->clk = devm_clk_get(&spi->dev, "clk");
	if (IS_ERR(st->clk))
		return PTR_ERR(st->clk);

	spi_set_drvdata(spi, indio_dev);								// do the following:   spi->dev->driver_data = indio_dev
	st->spi = spi;

	mutex_init(&st->lock);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->channels = ad9957_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad9957_channels);
	// indio_dev->info = &ad9957_info;

	buffer = iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "tx",
					    &dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);


	ret = clk_prepare_enable(st->clk);
	if (ret < 0)
		goto error_disable_reg;



	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret < 0)
		goto error_disable_clk;

	return 0;

error_disable_clk:
	clk_disable_unprepare(st->clk);
error_disable_reg:
	iio_dmaengine_buffer_free(indio_dev->buffer);

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
