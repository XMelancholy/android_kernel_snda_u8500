mcde-objs		+= mcde_mod.o
mcde-objs		+= mcde_hw.o
mcde-objs		+= mcde_dss.o
mcde-objs		+= mcde_display.o
mcde-objs		+= mcde_bus.o
ifdef CONFIG_FB_MCDE
mcde-objs		+= mcde_fb.o
endif
mcde-objs		+= mcde_debugfs.o
mcde-objs		+= dsilink.o
mcde-objs		+= dsilink_v2.o
mcde-objs		+= dsilink_v3.o
mcde-objs		+= dsilink_debugfs.o
obj-$(CONFIG_MCDE)	+= mcde.o

obj-$(CONFIG_MCDE_DISPLAY_SAMSUNG_S6D16D0)	+= display-samsung_s6d16d0.o
obj-$(CONFIG_MCDE_DISPLAY_SONY_ACX424AKP_DSI)   += display-sony_acx424akp_dsi.o
obj-$(CONFIG_MCDE_DISPLAY_TOSHIBA_DSI2LVDS)	+= display-toshiba_dsi2lvds.o
obj-$(CONFIG_MCDE_DISPLAY_HIMAX_HX8392)		+= display-himax_hx8392.o
obj-$(CONFIG_MCDE_DISPLAY_VUIB500_DPI)		+= display-vuib500-dpi.o
obj-$(CONFIG_MCDE_DISPLAY_AV8100)		+= display-av8100.o
obj-$(CONFIG_DISPLAY_FICTIVE)			+= display-fictive.o
obj-$(CONFIG_MCDE_DISPLAY_BAMBOOK)			+= display-bambook-dsi.o

ifdef CONFIG_FB_MCDE_DEBUG
EXTRA_CFLAGS += -DDEBUG
endif
ifdef CONFIG_FB_MCDE_VDEBUG
EXTRA_CFLAGS += -DVERBOSE_DEBUG
endif
