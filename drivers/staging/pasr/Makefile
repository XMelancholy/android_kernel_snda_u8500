pasr-objs := helper.o init.o core.o service.o
pasr_dbg-objs := debugfs.o

obj-$(CONFIG_PASR) += pasr.o
obj-$(CONFIG_UX500_PASR) += ux500.o
obj-$(CONFIG_PASR_SYSFS) += sysfs.o
obj-$(CONFIG_PASR_DEBUGFS) += pasr_dbg.o

ccflags-$(CONFIG_PASR_DEBUG) := -DDEBUG
