/*
 * Variable Device Driver
 *
 * Copyright 2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
 #include <linux/kernel.h>
 #include <linux/slab.h>
 #include <linux/module.h>
 #include <linux/string.h>

 #include <linux/iio/iio.h>
 #include <linux/iio/sysfs.h>
 #include <linux/of_device.h>
 #include <linux/of_platform.h>


#define DRIVER_NAME		"xband_controller"

enum attributes{
	XB_ATTR_MODE_FMCWN_TRANSCEIVER,
	XB_ATTR_LO2_FREQUENCY_HZ,
	XB_ATTR_LOMODE_LOWSPUR_LOWPNN,
	XB_ATTR_VCXO_SEL_0_AUTO_1_100M_2_125M,
	XB_ATTR_RXDECIMATION,
	XB_ATTR_RXDSPFREQUENCYSHIFT_HZ,
	XB_ATTR_TXRATE_HZ,
	XB_ATTR_RXRATE_HZ,
	XB_ATTR_CHIRPSTARTFREQUENCY_HZ,
	XB_ATTR_CHIRPSTOPFREQUENCY_HZ,
	XB_ATTR_CHIRPOFFSETFREQUENCY_HZ,
	XB_ATTR_CHIRPLENSAMPLES,
	XB_ATTR_TXINTERPOLATION,
	XB_ATTR_TXDSPFREQUENCYSHIFT_HZ,
	XB_ATTR_RXGAIN_DB,
	XB_ATTR_TXGAIN_DB,
	XB_ATTR_RXSAMPLESPERFRAME,
	XB_ATTR_RXGUARDLENSAMPLES,
	XB_ATTR_RXTOTXOFFSETSAMPLES,
	XB_ATTR_INTEGERTOVOLTSCALAR_TX1,
	XB_ATTR_INTEGERTOVOLTSCALAR_TX2,
	XB_ATTR_VOLTTOINTEGERSCALAR_RX1,
	XB_ATTR_VOLTTOINTEGERSCALAR_RX2
};

struct xband_controller_state{
	struct spi_device *spi;

	u32 mode_FMCWn_transceiver;
	u64 lo2_frequency_hz;
	u32 lomode_lowspur_lowpnn;
	u32 vcxo_sel_0_auto_1_100M_2_125M;
	u32 rxdecimation;
	u32 rxdspfrequencyshift_hz;
	u32 txrate_hz;
	u32 rxrate_hz;
	u64 chirpstartfrequency_hz;
	u64 chirpstopfrequency_hz;
	u64 chirpoffsetfrequency_hz;
	u32 chirplensamples;
	u32 txinterpolation;
	u32 txdspfrequencyshift_hz;
	u32 rxgain_db;
	u32 txgain_db;
	u32 rxsamplesperframe;
	u32 rxguardlensamples;
	u32 rxtotxoffsetsamples;
	u32 integertovoltscalar_tx1;
	u32 integertovoltscalar_tx2;
	u32 volttointegerscalar_rx1;
	u32 volttointegerscalar_rx2;
};

static ssize_t xband_controller_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct xband_controller_state *st = iio_priv(indio_dev);
	long val;
	unsigned long long val64;
	int ret = 0;

	if((u32)this_attr->address == XB_ATTR_LO2_FREQUENCY_HZ ||
			(u32)this_attr->address == XB_ATTR_CHIRPSTARTFREQUENCY_HZ ||
			(u32)this_attr->address == XB_ATTR_CHIRPSTOPFREQUENCY_HZ ||
			(u32)this_attr->address == XB_ATTR_CHIRPOFFSETFREQUENCY_HZ
		)
		ret = kstrtoull(buf, 10, &val64);
	else
		ret = kstrtol(buf, 0, &val);
	if(ret)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address){
	case XB_ATTR_MODE_FMCWN_TRANSCEIVER:
		st->mode_FMCWn_transceiver = val;
		break;
	case XB_ATTR_LO2_FREQUENCY_HZ:
		st->lo2_frequency_hz = val64;
		break;
	case XB_ATTR_LOMODE_LOWSPUR_LOWPNN:
		st->lomode_lowspur_lowpnn = val;
		break;
	case XB_ATTR_VCXO_SEL_0_AUTO_1_100M_2_125M:
		st->vcxo_sel_0_auto_1_100M_2_125M = val;
		break;
	case XB_ATTR_RXDECIMATION:
		st->rxdecimation = val;
		break;
	case XB_ATTR_RXDSPFREQUENCYSHIFT_HZ:
		st->rxdspfrequencyshift_hz = val;
		break;
	case XB_ATTR_TXRATE_HZ:
		st->txrate_hz = val;
		break;
	case XB_ATTR_RXRATE_HZ:
		st->rxrate_hz = val;
		break;
	case XB_ATTR_CHIRPSTARTFREQUENCY_HZ:
		st->chirpstartfrequency_hz = val64;
		break;
	case XB_ATTR_CHIRPSTOPFREQUENCY_HZ:
		st->chirpstopfrequency_hz = val64;
		break;
	case XB_ATTR_CHIRPOFFSETFREQUENCY_HZ:
		st->chirpoffsetfrequency_hz = val64;
		break;
	case XB_ATTR_CHIRPLENSAMPLES:
		st->chirplensamples = val;
		break;
	case XB_ATTR_TXINTERPOLATION:
		st->txinterpolation = val;
		break;
	case XB_ATTR_TXDSPFREQUENCYSHIFT_HZ:
		st->txdspfrequencyshift_hz = val;
		break;
	case XB_ATTR_RXGAIN_DB:
		st->rxgain_db = val;
		break;
	case XB_ATTR_TXGAIN_DB:
		st->txgain_db = val;
		break;
	case XB_ATTR_RXSAMPLESPERFRAME:
		st->rxsamplesperframe = val;
		break;
	case XB_ATTR_RXGUARDLENSAMPLES:
		st->rxguardlensamples = val;
		break;
	case XB_ATTR_RXTOTXOFFSETSAMPLES:
		st->rxtotxoffsetsamples = val;
		break;
	case XB_ATTR_INTEGERTOVOLTSCALAR_TX1:
		st->integertovoltscalar_tx1 = val;
		break;
	case XB_ATTR_INTEGERTOVOLTSCALAR_TX2:
		st->integertovoltscalar_tx2 = val;
		break;
	case XB_ATTR_VOLTTOINTEGERSCALAR_RX1:
		st->volttointegerscalar_rx1 = val;
		break;
	case XB_ATTR_VOLTTOINTEGERSCALAR_RX2:
		st->volttointegerscalar_rx2 = val;
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t xband_controller_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct xband_controller_state *st = iio_priv(indio_dev);
	u32 val = 0;
	u64 val64 = 0;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address){
	case XB_ATTR_MODE_FMCWN_TRANSCEIVER:
		val = st->mode_FMCWn_transceiver;
		break;
	case XB_ATTR_LO2_FREQUENCY_HZ:
		val64 = st->lo2_frequency_hz;
		break;
	case XB_ATTR_LOMODE_LOWSPUR_LOWPNN:
		val = st->lomode_lowspur_lowpnn;
		break;
	case XB_ATTR_VCXO_SEL_0_AUTO_1_100M_2_125M:
		val = st->vcxo_sel_0_auto_1_100M_2_125M;
		break;
	case XB_ATTR_RXDECIMATION:
		val = st->rxdecimation;
		break;
	case XB_ATTR_RXDSPFREQUENCYSHIFT_HZ:
		val = st->rxdspfrequencyshift_hz;
		break;
	case XB_ATTR_TXRATE_HZ:
		val = st->txrate_hz;
		break;
	case XB_ATTR_RXRATE_HZ:
		val = st->rxrate_hz;
		break;
	case XB_ATTR_CHIRPSTARTFREQUENCY_HZ:
		val64 = st->chirpstartfrequency_hz;
		break;
	case XB_ATTR_CHIRPSTOPFREQUENCY_HZ:
		val64 = st->chirpstopfrequency_hz;
		break;
	case XB_ATTR_CHIRPOFFSETFREQUENCY_HZ:
		val64 = st->chirpoffsetfrequency_hz;
		break;
	case XB_ATTR_CHIRPLENSAMPLES:
		val = st->chirplensamples;
		break;
	case XB_ATTR_TXINTERPOLATION:
		val = st->txinterpolation;
		break;
	case XB_ATTR_TXDSPFREQUENCYSHIFT_HZ:
		val = st->txdspfrequencyshift_hz;
		break;
	case XB_ATTR_RXGAIN_DB:
		val = st->rxgain_db;
		break;
	case XB_ATTR_TXGAIN_DB:
		val = st->txgain_db;
		break;
	case XB_ATTR_RXSAMPLESPERFRAME:
		val = st->rxsamplesperframe;
		break;
	case XB_ATTR_RXGUARDLENSAMPLES:
		val = st->rxguardlensamples;
		break;
	case XB_ATTR_RXTOTXOFFSETSAMPLES:
		val = st->rxtotxoffsetsamples;
		break;
	case XB_ATTR_INTEGERTOVOLTSCALAR_TX1:
		val = st->integertovoltscalar_tx1;
		break;
	case XB_ATTR_INTEGERTOVOLTSCALAR_TX2:
		val = st->integertovoltscalar_tx2;
		break;
	case XB_ATTR_VOLTTOINTEGERSCALAR_RX1:
		val = st->volttointegerscalar_rx1;
		break;
	case XB_ATTR_VOLTTOINTEGERSCALAR_RX2:
		val = st->volttointegerscalar_rx2;
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0){
		if((u32)this_attr->address == XB_ATTR_LO2_FREQUENCY_HZ ||
				(u32)this_attr->address == XB_ATTR_CHIRPSTARTFREQUENCY_HZ ||
				(u32)this_attr->address == XB_ATTR_CHIRPSTOPFREQUENCY_HZ ||
				(u32)this_attr->address == XB_ATTR_CHIRPOFFSETFREQUENCY_HZ
			)
			ret = sprintf(buf, "%llu\n", val64);
		else
			ret = sprintf(buf, "%d\n", val);
	}
	return ret;
}

static IIO_DEVICE_ATTR(Mode_FMCWn_Transceiver, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_MODE_FMCWN_TRANSCEIVER);

static IIO_DEVICE_ATTR(LO2_Frequency_Hz, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_LO2_FREQUENCY_HZ);

static IIO_DEVICE_ATTR(LOMode_LowSpur_LowPNn, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_LOMODE_LOWSPUR_LOWPNN);

static IIO_DEVICE_ATTR(VcxoSel_0_Auto_1_100M_2_125M, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_VCXO_SEL_0_AUTO_1_100M_2_125M);

static IIO_DEVICE_ATTR(RxDecimation, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_RXDECIMATION);

static IIO_DEVICE_ATTR(RxDSPfrequencyShift_Hz, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_RXDSPFREQUENCYSHIFT_HZ);

static IIO_DEVICE_ATTR(TxRate_Hz, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_TXRATE_HZ);

static IIO_DEVICE_ATTR(RxRate_Hz, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_RXRATE_HZ);

static IIO_DEVICE_ATTR(ChirpStartFrequency_Hz, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_CHIRPSTARTFREQUENCY_HZ);

static IIO_DEVICE_ATTR(ChirpStopFrequency_Hz, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_CHIRPSTOPFREQUENCY_HZ);

static IIO_DEVICE_ATTR(ChirpOffsetFrequency_Hz, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_CHIRPOFFSETFREQUENCY_HZ);

static IIO_DEVICE_ATTR(ChirpLenSamples, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_CHIRPLENSAMPLES);

static IIO_DEVICE_ATTR(TxInterpolation, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_TXINTERPOLATION);

static IIO_DEVICE_ATTR(TxDSPfrequencyShift_Hz, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_TXDSPFREQUENCYSHIFT_HZ);

static IIO_DEVICE_ATTR(RxGain_dB, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_RXGAIN_DB);

static IIO_DEVICE_ATTR(TxGain_dB, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_TXGAIN_DB);

static IIO_DEVICE_ATTR(RxSamplesPerFrame, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_RXSAMPLESPERFRAME);

static IIO_DEVICE_ATTR(RxGuardLenSamples, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_RXGUARDLENSAMPLES);

static IIO_DEVICE_ATTR(RxToTxOffsetSamples, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_RXTOTXOFFSETSAMPLES);

static IIO_DEVICE_ATTR(IntegerToVoltScalar_TX1, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_INTEGERTOVOLTSCALAR_TX1);

static IIO_DEVICE_ATTR(IntegerToVoltScalar_TX2, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_INTEGERTOVOLTSCALAR_TX2);

static IIO_DEVICE_ATTR(VoltToIntegerScalar_RX1, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_VOLTTOINTEGERSCALAR_RX1);

static IIO_DEVICE_ATTR(VoltToIntegerScalar_RX2, S_IRUGO | S_IWUSR,
			xband_controller_show,
			xband_controller_store,
			XB_ATTR_VOLTTOINTEGERSCALAR_RX2);

static struct attribute *xband_controller_attributes[] = {
	&iio_dev_attr_Mode_FMCWn_Transceiver.dev_attr.attr,
	&iio_dev_attr_LO2_Frequency_Hz.dev_attr.attr,
	&iio_dev_attr_LOMode_LowSpur_LowPNn.dev_attr.attr,
	&iio_dev_attr_VcxoSel_0_Auto_1_100M_2_125M.dev_attr.attr,
	&iio_dev_attr_RxDecimation.dev_attr.attr,
	&iio_dev_attr_RxDSPfrequencyShift_Hz.dev_attr.attr,
	&iio_dev_attr_TxRate_Hz.dev_attr.attr,
	&iio_dev_attr_RxRate_Hz.dev_attr.attr,
	&iio_dev_attr_ChirpStartFrequency_Hz.dev_attr.attr,
	&iio_dev_attr_ChirpStopFrequency_Hz.dev_attr.attr,
	&iio_dev_attr_ChirpOffsetFrequency_Hz.dev_attr.attr,
	&iio_dev_attr_ChirpLenSamples.dev_attr.attr,
	&iio_dev_attr_TxInterpolation.dev_attr.attr,
	&iio_dev_attr_TxDSPfrequencyShift_Hz.dev_attr.attr,
	&iio_dev_attr_RxGain_dB.dev_attr.attr,
	&iio_dev_attr_TxGain_dB.dev_attr.attr,
	&iio_dev_attr_RxSamplesPerFrame.dev_attr.attr,
	&iio_dev_attr_RxGuardLenSamples.dev_attr.attr,
	&iio_dev_attr_RxToTxOffsetSamples.dev_attr.attr,
	&iio_dev_attr_IntegerToVoltScalar_TX1.dev_attr.attr,
	&iio_dev_attr_IntegerToVoltScalar_TX2.dev_attr.attr,
	&iio_dev_attr_VoltToIntegerScalar_RX1.dev_attr.attr,
	&iio_dev_attr_VoltToIntegerScalar_RX2.dev_attr.attr,
	NULL
};

static const struct attribute_group xband_controller_attribute_group = {
	.attrs = xband_controller_attributes,
};

static const struct iio_info xband_controller_info = {
	// .driver_module = THIS_MODULE,
	.attrs = &xband_controller_attribute_group,
};

static const struct of_device_id xband_controller_of_match[] = {
	{ .compatible = DRIVER_NAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, xband_controller_of_match);

static int xband_controller_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;	// for devicetree parsing
	struct iio_dev *indio_dev;
	struct xband_controller_state *st;
	const struct of_device_id *id;
	int ret;

	printk("probing x-band controller..\n");
	id = of_match_device(xband_controller_of_match, &pdev->dev);
	if (!id){
		printk("\nx-band controller: -ENODEV\n");
		return -ENODEV;
	}

	indio_dev = iio_device_alloc(sizeof(*st));
	if (!indio_dev){
		printk("\nx-band controller: -ENOMEM\n");
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &xband_controller_info;

	ret = iio_device_register(indio_dev);
	if(ret)
		goto error_free_device;
	platform_set_drvdata(pdev, indio_dev);

	printk("probing x-band controller DONE!\n");
	return 0;

error_free_device:
			printk("\nx-band controller: probe() failed!\n");
	iio_device_free(indio_dev);
	return ret;
}

static int xband_controller_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static struct platform_driver xband_controller_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = xband_controller_of_match,
	},
	.probe = xband_controller_probe,
	.remove = xband_controller_remove,
};
module_platform_driver(xband_controller_driver);

MODULE_AUTHOR("Cyril Zwahlen <zwahlen@precisionwave.com>");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_LICENSE("GPL v2");
