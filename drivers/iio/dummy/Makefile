# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the IIO Dummy Driver
#

obj-$(CONFIG_IIO_SIMPLE_DUMMY) += iio_dummy.o
iio_dummy-y := iio_simple_dummy.o
iio_dummy-$(CONFIG_IIO_SIMPLE_DUMMY_EVENTS) += iio_simple_dummy_events.o
iio_dummy-$(CONFIG_IIO_SIMPLE_DUMMY_BUFFER) += iio_simple_dummy_buffer.o

obj-$(CONFIG_IIO_DUMMY_EVGEN) += iio_dummy_evgen.o
obj-$(CONFIG_XBAND_CONTROLLER) += xband_controller.o
obj-$(CONFIG_IIO_AXI_FPGA_REG) += iio_axi_fpga_reg.o
obj-$(CONFIG_PCW_AXSS_VER) += pcw_axss_ver.o
