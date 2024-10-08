// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Zebu header manipulation access
 *
 * Copyright 2024 PrecisionWave AG
 *
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

enum attributes {
	// Stream generator
	UDP_DST_PORT_BASE,
	UDP_DST_PORT_STREAMS,
	MAX_PAYLOAD_BYTES,
	ENABLE_TESTMODE,
	ENABLE_PPS_EVENT,

	// UDP packet generator
	ETH_DST_MAC,
	ETH_SRC_MAC,
	IP_TTL,
	IP_SRC_IPv4,
	IP_DST_IPv4,
	UDP_SRC_PORT,
};

// Stream generator
#define REG_STRGEN_UDP_DST_PORT_BASE 0
#define REG_STRGEN_UDP_DST_PORT_STREAMS 4
#define REG_STRGEN_MAX_PAYLOAD_BYTES 8
#define REG_STRGEN_ENABLE_TESTMODE 12
#define REG_STRGEN_ENABLE_PPS_EVENT 16

// UDP packet generator
#define REG_PKTGEN_ETH_DST_MAC 0
#define REG_PKTGEN_ETH_SRC_MAC 6
#define REG_PKTGEN_IP_TTL (14 + 8)
#define REG_PKTGEN_IP_SRC_IPv4 (14 + 12)
#define REG_PKTGEN_IP_DST_IPv4 (14 + 16)
#define REG_PKTGEN_UDP_SRC_PORT (14 + 20 + 0)

struct csr {
	void __iomem *base;
	resource_size_t length;
	struct resource *res;
};

struct zebu_udp_state {
	struct csr pktgen_csr;
	struct csr strgen_csr;
};

static void pktgen_csr_write(struct zebu_udp_state *st, unsigned reg,
			     uint32_t val)
{
	iowrite32(val, st->pktgen_csr.base + reg);
}

static void pktgen_csr_write8(struct zebu_udp_state *st, unsigned reg,
			      uint32_t val)
{
	iowrite8(val, st->pktgen_csr.base + reg);
}

static uint32_t pktgen_csr_read(struct zebu_udp_state *st, unsigned reg)
{
	return ioread32(st->pktgen_csr.base + reg);
}

static uint32_t pktgen_csr_read8(struct zebu_udp_state *st, unsigned reg)
{
	return ioread8(st->pktgen_csr.base + reg);
}

static void strgen_csr_write(struct zebu_udp_state *st, unsigned reg,
			     uint32_t val)
{
	iowrite32(val, st->strgen_csr.base + reg);
}

static uint32_t strgen_csr_read(struct zebu_udp_state *st, unsigned reg)
{
	return ioread32(st->strgen_csr.base + reg);
}

static int pktgen_reg_access(struct iio_dev *indio_dev, u32 reg, u32 writeval,
			     u32 *readval)
{
	struct zebu_udp_state *st = iio_priv(indio_dev);

	if (reg > st->pktgen_csr.length)
		return -EINVAL;

	/* Unaligned access is disallowed */
	if ((reg & 0x3) != 0)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);

	if (!readval) {
		pktgen_csr_write(st, reg, writeval);
	} else {
		*readval = pktgen_csr_read(st, reg);
	}

	mutex_unlock(&indio_dev->mlock);
	return 0;
}

static int strgen_reg_access(struct iio_dev *indio_dev, u32 reg, u32 writeval,
			     u32 *readval)
{
	struct zebu_udp_state *st = iio_priv(indio_dev);

	if (reg > st->strgen_csr.length)
		return -EINVAL;

	/* Unaligned access is disallowed */
	if ((reg & 0x3) != 0)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);

	if (!readval) {
		strgen_csr_write(st, reg, writeval);
	} else {
		*readval = strgen_csr_read(st, reg);
	}

	mutex_unlock(&indio_dev->mlock);
	return 0;
}

static int zebu_udp_reg_access(struct iio_dev *indio_dev, u32 reg, u32 writeval,
			       u32 *readval)
{
	if (reg & 0x8000)
		return pktgen_reg_access(indio_dev, reg & ~0x8000, writeval,
					 readval);
	else
		return strgen_reg_access(indio_dev, reg & ~0x8000, writeval,
					 readval);
}

static ssize_t zebu_udp_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct zebu_udp_state *st = iio_priv(indio_dev);
	unsigned uv[6];
	long val;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	// Stream generator
	case UDP_DST_PORT_BASE:
		ret = kstrtol(buf, 0, &val);
		if (ret)
			goto error_unlock;
		strgen_csr_write(st, REG_STRGEN_UDP_DST_PORT_BASE, val);
		break;

	case UDP_DST_PORT_STREAMS:
		ret = kstrtol(buf, 0, &val);
		if (ret)
			goto error_unlock;
		strgen_csr_write(st, REG_STRGEN_UDP_DST_PORT_STREAMS, val);
		break;

	case MAX_PAYLOAD_BYTES:
		ret = kstrtol(buf, 0, &val);
		if (ret)
			goto error_unlock;
		strgen_csr_write(st, REG_STRGEN_MAX_PAYLOAD_BYTES, val);
		break;

	case ENABLE_TESTMODE:
		ret = kstrtol(buf, 0, &val);
		if (ret)
			goto error_unlock;
		strgen_csr_write(st, REG_STRGEN_ENABLE_TESTMODE, val);
		break;

	case ENABLE_PPS_EVENT:
		ret = kstrtol(buf, 0, &val);
		if (ret)
			goto error_unlock;
		strgen_csr_write(st, REG_STRGEN_ENABLE_PPS_EVENT, val);
		break;

	// UDP packet generator
	case ETH_DST_MAC:
		ret = sscanf(buf, "%x:%x:%x:%x:%x:%x", &uv[0], &uv[1], &uv[2],
			     &uv[3], &uv[4], &uv[5]);
		if (ret != 6) {
			ret = -EINVAL;
			goto error_unlock;
		}
		ret = 0;
		pktgen_csr_write8(st, REG_PKTGEN_ETH_DST_MAC + 0, uv[0]);
		pktgen_csr_write8(st, REG_PKTGEN_ETH_DST_MAC + 1, uv[1]);
		pktgen_csr_write8(st, REG_PKTGEN_ETH_DST_MAC + 2, uv[2]);
		pktgen_csr_write8(st, REG_PKTGEN_ETH_DST_MAC + 3, uv[3]);
		pktgen_csr_write8(st, REG_PKTGEN_ETH_DST_MAC + 4, uv[4]);
		pktgen_csr_write8(st, REG_PKTGEN_ETH_DST_MAC + 5, uv[5]);
		break;

	case ETH_SRC_MAC:
		ret = sscanf(buf, "%x:%x:%x:%x:%x:%x", &uv[0], &uv[1], &uv[2],
			     &uv[3], &uv[4], &uv[5]);
		if (ret != 6) {
			ret = -EINVAL;
			goto error_unlock;
		}
		ret = 0;
		pktgen_csr_write8(st, REG_PKTGEN_ETH_SRC_MAC + 0, uv[0]);
		pktgen_csr_write8(st, REG_PKTGEN_ETH_SRC_MAC + 1, uv[1]);
		pktgen_csr_write8(st, REG_PKTGEN_ETH_SRC_MAC + 2, uv[2]);
		pktgen_csr_write8(st, REG_PKTGEN_ETH_SRC_MAC + 3, uv[3]);
		pktgen_csr_write8(st, REG_PKTGEN_ETH_SRC_MAC + 4, uv[4]);
		pktgen_csr_write8(st, REG_PKTGEN_ETH_SRC_MAC + 5, uv[5]);
		break;

	case IP_TTL:
		ret = kstrtol(buf, 0, &val);
		if (ret)
			goto error_unlock;
		if (val > 0xff || val < 0) {
			ret = -EINVAL;
			goto error_unlock;
		}
		pktgen_csr_write8(st, REG_PKTGEN_IP_TTL, val);
		break;

	case IP_SRC_IPv4:
		ret = sscanf(buf, "%d.%d.%d.%d", &uv[0], &uv[1], &uv[2],
			     &uv[3]);
		if (ret != 4) {
			ret = -EINVAL;
			goto error_unlock;
		}
		ret = 0;
		pktgen_csr_write8(st, REG_PKTGEN_IP_SRC_IPv4 + 0, uv[0]);
		pktgen_csr_write8(st, REG_PKTGEN_IP_SRC_IPv4 + 1, uv[1]);
		pktgen_csr_write8(st, REG_PKTGEN_IP_SRC_IPv4 + 2, uv[2]);
		pktgen_csr_write8(st, REG_PKTGEN_IP_SRC_IPv4 + 3, uv[3]);
		break;

	case IP_DST_IPv4:
		ret = sscanf(buf, "%d.%d.%d.%d", &uv[0], &uv[1], &uv[2],
			     &uv[3]);
		if (ret != 4) {
			ret = -EINVAL;
			goto error_unlock;
		}
		ret = 0;
		pktgen_csr_write8(st, REG_PKTGEN_IP_DST_IPv4 + 0, uv[0]);
		pktgen_csr_write8(st, REG_PKTGEN_IP_DST_IPv4 + 1, uv[1]);
		pktgen_csr_write8(st, REG_PKTGEN_IP_DST_IPv4 + 2, uv[2]);
		pktgen_csr_write8(st, REG_PKTGEN_IP_DST_IPv4 + 3, uv[3]);
		break;

	case UDP_SRC_PORT:
		ret = kstrtol(buf, 0, &val);
		if (ret)
			goto error_unlock;
		pktgen_csr_write8(st, REG_PKTGEN_UDP_SRC_PORT + 0, val >> 8);
		pktgen_csr_write8(st, REG_PKTGEN_UDP_SRC_PORT + 1, val);
		break;

	default:
		ret = -ENODEV;
	}
error_unlock:
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t zebu_udp_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct zebu_udp_state *st = iio_priv(indio_dev);
	int ret = -ENODEV;
	u32 val32;
	unsigned uv[6];

	mutex_lock(&indio_dev->mlock);

	switch (this_attr->address) {
	// Stream generator
	case UDP_DST_PORT_BASE:
		val32 = strgen_csr_read(st, REG_STRGEN_UDP_DST_PORT_BASE);
		ret = sprintf(buf, "0x%04x\n", val32 & 0xffff);
		break;

	case UDP_DST_PORT_STREAMS:
		val32 = strgen_csr_read(st, REG_STRGEN_UDP_DST_PORT_STREAMS);
		ret = sprintf(buf, "%u\n", val32);
		break;

	case MAX_PAYLOAD_BYTES:
		val32 = strgen_csr_read(st, REG_STRGEN_MAX_PAYLOAD_BYTES);
		ret = sprintf(buf, "%u\n", val32);
		break;

	case ENABLE_TESTMODE:
		val32 = strgen_csr_read(st, REG_STRGEN_ENABLE_TESTMODE);
		ret = sprintf(buf, "%u\n", val32);
		break;

	case ENABLE_PPS_EVENT:
		val32 = strgen_csr_read(st, REG_STRGEN_ENABLE_PPS_EVENT);
		ret = sprintf(buf, "%u\n", val32);
		break;

	// UDP packet generator
	case ETH_DST_MAC:
		uv[0] = pktgen_csr_read8(st, REG_PKTGEN_ETH_DST_MAC + 0);
		uv[1] = pktgen_csr_read8(st, REG_PKTGEN_ETH_DST_MAC + 1);
		uv[2] = pktgen_csr_read8(st, REG_PKTGEN_ETH_DST_MAC + 2);
		uv[3] = pktgen_csr_read8(st, REG_PKTGEN_ETH_DST_MAC + 3);
		uv[4] = pktgen_csr_read8(st, REG_PKTGEN_ETH_DST_MAC + 4);
		uv[5] = pktgen_csr_read8(st, REG_PKTGEN_ETH_DST_MAC + 5);
		ret = sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n", uv[0],
			      uv[1], uv[2], uv[3], uv[4], uv[5]);
		break;

	case ETH_SRC_MAC:
		uv[0] = pktgen_csr_read8(st, REG_PKTGEN_ETH_SRC_MAC + 0);
		uv[1] = pktgen_csr_read8(st, REG_PKTGEN_ETH_SRC_MAC + 1);
		uv[2] = pktgen_csr_read8(st, REG_PKTGEN_ETH_SRC_MAC + 2);
		uv[3] = pktgen_csr_read8(st, REG_PKTGEN_ETH_SRC_MAC + 3);
		uv[4] = pktgen_csr_read8(st, REG_PKTGEN_ETH_SRC_MAC + 4);
		uv[5] = pktgen_csr_read8(st, REG_PKTGEN_ETH_SRC_MAC + 5);
		ret = sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n", uv[0],
			      uv[1], uv[2], uv[3], uv[4], uv[5]);
		break;

	case IP_TTL:
		uv[0] = pktgen_csr_read8(st, REG_PKTGEN_IP_TTL);
		ret = sprintf(buf, "%u\n", uv[0]);
		break;

	case IP_SRC_IPv4:
		uv[0] = pktgen_csr_read8(st, REG_PKTGEN_IP_SRC_IPv4 + 0);
		uv[1] = pktgen_csr_read8(st, REG_PKTGEN_IP_SRC_IPv4 + 1);
		uv[2] = pktgen_csr_read8(st, REG_PKTGEN_IP_SRC_IPv4 + 2);
		uv[3] = pktgen_csr_read8(st, REG_PKTGEN_IP_SRC_IPv4 + 3);
		ret = sprintf(buf, "%u.%u.%u.%u\n", uv[0], uv[1], uv[2], uv[3]);
		break;

	case IP_DST_IPv4:
		uv[0] = pktgen_csr_read8(st, REG_PKTGEN_IP_DST_IPv4 + 0);
		uv[1] = pktgen_csr_read8(st, REG_PKTGEN_IP_DST_IPv4 + 1);
		uv[2] = pktgen_csr_read8(st, REG_PKTGEN_IP_DST_IPv4 + 2);
		uv[3] = pktgen_csr_read8(st, REG_PKTGEN_IP_DST_IPv4 + 3);
		ret = sprintf(buf, "%u.%u.%u.%u\n", uv[0], uv[1], uv[2], uv[3]);
		break;

	case UDP_SRC_PORT:
		val32 = pktgen_csr_read8(st, REG_PKTGEN_UDP_SRC_PORT + 0);
		val32 <<= 8;
		val32 |= pktgen_csr_read8(st, REG_PKTGEN_UDP_SRC_PORT + 1);
		ret = sprintf(buf, "0x%04x\n", val32);
		break;

	default:
		ret = -ENODEV;
		break;
	}

	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static IIO_DEVICE_ATTR(udp_dst_port_base, S_IRUGO | S_IWUSR, zebu_udp_show,
		       zebu_udp_store, UDP_DST_PORT_BASE);

static IIO_DEVICE_ATTR(udp_dst_port_streams, S_IRUGO | S_IWUSR, zebu_udp_show,
		       zebu_udp_store, UDP_DST_PORT_STREAMS);

static IIO_DEVICE_ATTR(max_payload_bytes, S_IRUGO | S_IWUSR, zebu_udp_show,
		       zebu_udp_store, MAX_PAYLOAD_BYTES);

static IIO_DEVICE_ATTR(enable_testmode, S_IRUGO | S_IWUSR, zebu_udp_show,
		       zebu_udp_store, ENABLE_TESTMODE);

static IIO_DEVICE_ATTR(enable_pps_event, S_IRUGO | S_IWUSR, zebu_udp_show,
		       zebu_udp_store, ENABLE_PPS_EVENT);

static IIO_DEVICE_ATTR(eth_dst_mac, S_IRUGO | S_IWUSR, zebu_udp_show,
		       zebu_udp_store, ETH_DST_MAC);

static IIO_DEVICE_ATTR(eth_src_mac, S_IRUGO | S_IWUSR, zebu_udp_show,
		       zebu_udp_store, ETH_SRC_MAC);

static IIO_DEVICE_ATTR(ip_ttl, S_IRUGO | S_IWUSR, zebu_udp_show, zebu_udp_store,
		       IP_TTL);

static IIO_DEVICE_ATTR(ip_src_ipv4, S_IRUGO | S_IWUSR, zebu_udp_show,
		       zebu_udp_store, IP_SRC_IPv4);

static IIO_DEVICE_ATTR(ip_dst_ipv4, S_IRUGO | S_IWUSR, zebu_udp_show,
		       zebu_udp_store, IP_DST_IPv4);

static IIO_DEVICE_ATTR(udp_src_port, S_IRUGO | S_IWUSR, zebu_udp_show,
		       zebu_udp_store, UDP_SRC_PORT);

static struct attribute *zebu_udp_attributes[] = {
	&iio_dev_attr_udp_dst_port_base.dev_attr.attr,
	&iio_dev_attr_udp_dst_port_streams.dev_attr.attr,
	&iio_dev_attr_max_payload_bytes.dev_attr.attr,
	&iio_dev_attr_enable_testmode.dev_attr.attr,
	&iio_dev_attr_enable_pps_event.dev_attr.attr,
	&iio_dev_attr_eth_dst_mac.dev_attr.attr,
	&iio_dev_attr_eth_src_mac.dev_attr.attr,
	&iio_dev_attr_ip_ttl.dev_attr.attr,
	&iio_dev_attr_ip_src_ipv4.dev_attr.attr,
	&iio_dev_attr_ip_dst_ipv4.dev_attr.attr,
	&iio_dev_attr_udp_src_port.dev_attr.attr,
	NULL
};

static const struct attribute_group zebu_udp_attribute_group = {
	.attrs = zebu_udp_attributes,
};

static const struct iio_info zebu_udp_info = {
	.attrs = &zebu_udp_attribute_group,
	.debugfs_reg_access = &zebu_udp_reg_access,
};

/* Match table for of_platform binding */
static const struct of_device_id zebu_udp_of_match[] = {
	{
		.compatible = "pcw,zebu-udp",
	},
	{}
};
MODULE_DEVICE_TABLE(of, zebu_udp_of_match);

static int zebu_udp_probe(struct platform_device *pdev)
{
	struct zebu_udp_state *st;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->strgen_csr.res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->strgen_csr.base =
		devm_ioremap_resource(&pdev->dev, st->strgen_csr.res);
	if (IS_ERR(st->strgen_csr.base))
		return PTR_ERR(st->strgen_csr.base);
	st->strgen_csr.length =
		st->strgen_csr.res->end - st->strgen_csr.res->start;

	st->pktgen_csr.res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	st->pktgen_csr.base =
		devm_ioremap_resource(&pdev->dev, st->pktgen_csr.res);
	if (IS_ERR(st->pktgen_csr.base))
		return PTR_ERR(st->pktgen_csr.base);

	st->pktgen_csr.length =
		st->pktgen_csr.res->end - st->pktgen_csr.res->start;

	indio_dev->dev.parent = &pdev->dev;
	if (pdev->dev.of_node)
		indio_dev->name = pdev->dev.of_node->name;
	else
		indio_dev->name = "zebu-udp";
	indio_dev->info = &zebu_udp_info;

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static struct platform_driver zebu_udp_driver = {
	.driver = {
		.name = "zebu_udp",
		.of_match_table = zebu_udp_of_match,
	},
	.probe = zebu_udp_probe,
};

module_platform_driver(zebu_udp_driver);

MODULE_AUTHOR("Philipp Diethelm <philipp.diethelm@precisionwave.com>");
MODULE_DESCRIPTION("PCW Zebu UDP generator access driver");
MODULE_LICENSE("Dual BSD/GPL");
