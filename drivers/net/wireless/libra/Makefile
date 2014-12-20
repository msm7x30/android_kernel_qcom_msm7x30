
# Makefile for wlan sdio if driver

librasdioif-objs += libra_sdioif.o

ifdef CONFIG_ARCH_MSM8X60
	librasdioif-objs += qcomwlan_pwrif.o
endif

ifdef CONFIG_ARCH_MSM7X27A
	librasdioif-objs += qcomwlan7x27a_pwrif.o
endif

obj-$(CONFIG_LIBRA_CORE) += libra_wlan.o
obj-$(CONFIG_LIBRA_SDIOIF) += librasdioif.o
obj-$(CONFIG_LIBRA_MEM_PRE_ALLOC) += libra_prealloc.o
