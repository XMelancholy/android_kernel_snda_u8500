#
# Makefile for the linux kernel, UX500 machine.
#

obj-y				:= clock.o cpu.o devices.o dcache.o \
				   devices-common.o id.o pins.o \
				   usb.o reboot_reasons.o timer.o \
				   uart-db8500.o clock-debug.o
obj-y				+= pm/ test/
obj-$(CONFIG_CACHE_L2X0)	+= cache-l2x0.o

ifeq ($(CONFIG_UX500_SOC_DB8500), y)

obj-$(CONFIG_UX500_SOC_DBX500)	+= cpu-db8500.o devices-db8500.o \
				   clock-db8500.o dma-db8500.o
board-mcde-objs			+= board-mop500-mcde.o
endif


obj-$(CONFIG_MACH_HREFV60)	+= board-pins-sleep-force.o \
				board-mop500.o \
				board-mop500-sdi.o \
				board-common-sdi.o \
				board-mop500-regulators.o \
				board-mop500-pins.o \
				board-mop500-msp.o \
				board-mop500-bm.o \
				board-mop500-wlan.o \
				board-mop500-rmi4.o

obj-$(CONFIG_MACH_CCU9540)	+=  board-ccu9540.o \
				board-ccux540-ver.o \
				board-ccu9540-regulators.o \
				board-ccu9540-pins.o \
				board-ccu9540-sdi.o \
				board-ccu9540-modem.o

obj-$(CONFIG_MACH_PALLADIUM9540) 	+= board-palladium9540.o \
					board-ccu9540-pins.o \
					board-palladiumx540-sdi.o

obj-$(CONFIG_MACH_CCU8540)	+= board-ccu8540.o \
				board-ccux540-ver.o \
				board-ccu8540-regulators.o \
				board-ccu8540-pins.o \
				board-ccu8540-sdi.o

obj-$(CONFIG_MACH_PALLADIUM8540) 	+= board-palladium8540.o \
					board-ccu8540-pins.o \
					board-palladiumx540-sdi.o

obj-$(CONFIG_UIB) 		+= board-common-uib.o \
				board-mop500-stuib.o \
				board-mop500-u8500uib.o \
				board-mop500-cyttsp.o \
				board-mop500-u9540uib.o \
				board-mop500-u8540uib.o

obj-$(CONFIG_ANDROID_STE_TIMED_VIBRA)	+= board-mop500-vibra.o
obj-$(CONFIG_MCDE)		+= board-mcde.o
obj-$(CONFIG_U8500_MMIO)	+= board-mop500-mmio.o board-mop500-mmio_raw.o board-mop500-mmio_yuv.o
obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)	+= hotplug.o
obj-$(CONFIG_TEE_UX500)        	+= product.o
obj-$(CONFIG_DB8500_MLOADER) 	+= mloader-db8500.o open_modem_shared_memory.o
obj-$(CONFIG_U8500_REGULATOR_DEBUG)	+= virt-regulator-u8500.o
obj-$(CONFIG_UX500_DEBUG_HWREG) 	+= hwreg.o
obj-$(CONFIG_HWMEM)			+= hwmem-int.o
obj-$(CONFIG_UX500_L2X0_PREFETCH_CTRL)	+= l2x0-prefetch.o
obj-$(CONFIG_UX500_ROMCODE_SHARED_MUTEX) 	+= romcode-shared_mutex.o
obj-$(CONFIG_DBX500_PRCMU_DEBUG)	+= prcmu-debug.o
obj-$(CONFIG_UX500_DB_DUMP)		+= dbx500_dump.o
obj-$(CONFIG_RTC_HCTOHC)		+= hctohc.o
obj-$(CONFIG_UX500_HW_OBSERVER)		+= hw-observer-debug.o
ifeq ($(CONFIG_UX500_SOC_DB8500), y)
obj-$(CONFIG_STM_TRACE)			+= board-mop500-stm.o
endif
obj-$(CONFIG_SWITCH)			+= android_switch.o
obj-$(CONFIG_INPUT_AB8505_MICRO_USB_DETECT)	+= ssg_micro_usb_iddet.o abx_micro_usb_iddet.o
