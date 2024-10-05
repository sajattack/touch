#
# Makefile for SiW touch test driver
#

KERNEL_DIR = /tmp/linux-sdm845-build
BASE_DIR = $(shell pwd)

INSTALL_DIR = ./mod

CC=aarch64-linux-gnu-gcc
#CC=arm-linux-gnueabihf-gcc

ccflags-y += -D__KERNEL__ -DLINUX

# Mobile
#ENTRY_NAME = lg4894
#ENTRY_NAME = lg4895
#ENTRY_NAME = lg4946
#ENTRY_NAME = lg4951
#ENTRY_NAME = sw46104
#ENTRY_NAME = sw49106
#ENTRY_NAME = sw49107
#ENTRY_NAME = sw49407
#ENTRY_NAME = sw49408
ENTRY_NAME = sw49410
#ENTRY_NAME = sw49501
#ENTRY_NAME = sw42000a
#ENTRY_NAME = sw82905
# Large
#ENTRY_NAME = sw42101
# Auto
#ENTRY_NAME = sw1828
#ENTRY_NAME = sw42103
#ENTRY_NAME = sw17700

SIWMON=y
ccflags-$(SIWMON) += -DCONFIG_TOUCHSCREEN_SIWMON

MODULE_NAME = s$(ENTRY_NAME)
CHIP_NAME = $(shell echo $(ENTRY_NAME) | tr a-z A-Z)

ccflags-y += -DCONFIG_TOUCHSCREEN_SIW_$(CHIP_NAME)

CONFIG_TOUCHSCREEN_SIW_$(CHIP_NAME)=y


LIBS=-lm

obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-objs := siw_touch.o
$(MODULE_NAME)-objs += siw_touch_hal.o siw_touch_hal_sysfs.o
$(MODULE_NAME)-objs += siw_touch_hal_abt.o
$(MODULE_NAME)-objs += siw_touch_hal_prd.o
$(MODULE_NAME)-objs += siw_touch_hal_watch.o
$(MODULE_NAME)-objs += siw_touch_bus.o
$(MODULE_NAME)-objs += siw_touch_bus_i2c.o siw_touch_bus_spi.o
$(MODULE_NAME)-objs += siw_touch_of.o
$(MODULE_NAME)-objs += siw_touch_irq.o siw_touch_gpio.o
$(MODULE_NAME)-objs += siw_touch_event.o siw_touch_notify.o
$(MODULE_NAME)-objs += siw_touch_sys.o siw_touch_sysfs.o
$(MODULE_NAME)-objs += siw_touch_misc.o
$(MODULE_NAME)-objs += touch_$(ENTRY_NAME).o

module:
	$(MAKE) -C $(KERNEL_DIR) M=$(BASE_DIR) modules ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-

clean:
	rm -rf .tmp_versions
	rm -f *.ko
	rm -f *.mod.*
	rm -f .*.cmd
	rm -f *.o
	rm -f *.order
	rm -f *.symvers
	rm -f modules.builtin
	rm -f $(MODULE_NAME)


