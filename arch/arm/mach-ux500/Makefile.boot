   zreladdr-y	:= 0x00008000
params_phys-y	:= 0x00000100
initrd_phys-y	:= 0x00800000

dtb-$(CONFIG_MACH_SNOWBALL) += snowball.dtb
dtb-$(CONFIG_MACH_U9540) += u9540.dtb
