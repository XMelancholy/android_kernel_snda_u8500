/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/compdev.h>
#include <linux/clk.h>
#include <mach/devices.h>
#include <plat/pincfg.h>
#include <video/av8100.h>
#include <video/mcde_display.h>
#include <video/mcde_display-av8100.h>
#include <video/mcde_fb.h>
#include <video/mcde_dss.h>

#include "board-ccu9540.h"
#include "pins.h"
#include <mach/id.h>

#define DSI_UNIT_INTERVAL_2	0x5

#define DSITV_PLL_FREQ_HZ	840320000
/* Based on PLL DDR Freq at 1065,6 MHz */
#define HDMI_FREQ_HZ		33340000

#define TV_FREQ_HZ		38400000

/* Based on PLL SOC0 Freq at 800 MHz */
#define DSILCD_FREQ_HZ	33280000
#define UIB9540_S_V1_DSILCD_PLL_FREQ_HZ	420160000
#define UIB9540_T_V1_DSILCD_PLL_FREQ_HZ	570000000
#define UIB9540_S_V2_DSILCD_PLL_FREQ_HZ	580000000

#ifdef CONFIG_U8500_TV_OUTPUT_AV8100
/*
 * The initialization of hdmi disp driver must be delayed in order to
 * ensure that inputclk will be available (needed by hdmi hw)
 */
static struct delayed_work work_dispreg_hdmi;
#define DISPREG_HDMI_DELAY 6000
#endif

enum {
	PRIMARY_DISPLAY_ID,
	SECONDARY_DISPLAY_ID,
	FICTIVE_DISPLAY_ID,
	AV8100_DISPLAY_ID,
	MCDE_NR_OF_DISPLAYS
};

static int display_initialized_during_boot;

static int __init startup_graphics_setup(char *str)
{

	if (get_option(&str, &display_initialized_during_boot) != 1)
		display_initialized_during_boot = 0;

	switch (display_initialized_during_boot) {
	case 1:
		pr_info("Startup graphics support\n");
		break;
	case 0:
	default:
		pr_info("No startup graphics supported\n");
		break;
	};

	return 1;
}
__setup("startup_graphics=", startup_graphics_setup);

#if defined(CONFIG_U8500_TV_OUTPUT_AV8100)
static struct mcde_col_transform rgb_2_yCbCr_transform = {
	.matrix = {
		{0x0042, 0x0081, 0x0019},
		{0xffda, 0xffb6, 0x0070},
		{0x0070, 0xffa2, 0xffee},
	},
	.offset = {0x10, 0x80, 0x80},
};
#endif


static struct mcde_port sony_port0 = {
	.sync_src = MCDE_SYNCSRC_TE0,
	.frame_trig = MCDE_TRIG_HW,
};

static struct mcde_display_dsi_platform_data sony_acx424akp_display_pdata = {
	.link = 0,
	.num_data_lanes = 2,
};

static struct mcde_display_device sony_acx424akp_display = {
	.name = "mcde_disp_sony_acx424akp",
	.id = PRIMARY_DISPLAY_ID,
	.port = &sony_port0,
	.chnl_id = MCDE_CHNL_A,
	.fifo = MCDE_FIFO_A,
	.orientation = MCDE_DISPLAY_ROT_0,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGBA8888,
	.has_backlight = true,
	.dev = {
		.platform_data = &sony_acx424akp_display_pdata,
	},
};

static struct mcde_port himax_hx8392_port = {
	.sync_src = MCDE_SYNCSRC_TE0,
	.frame_trig = MCDE_TRIG_HW,
};

static struct mcde_display_dsi_platform_data
			himax_hx8392_display_pdata = {
	.link = 1,
	.num_data_lanes = 3,
};

static struct mcde_display_device himax_hx8392_display = {
	.name = "himax_hx8392",
	.id = PRIMARY_DISPLAY_ID,
	.port = &himax_hx8392_port,
	.chnl_id = MCDE_CHNL_A,
	.fifo = MCDE_FIFO_A,
	.orientation = MCDE_DISPLAY_ROT_0,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGBA8888,
	.dev = {
		.platform_data = &himax_hx8392_display_pdata,
	},
};

static struct mcde_port toshiba_dsi2lvds_port = {
	.sync_src = MCDE_SYNCSRC_OFF,
	.frame_trig = MCDE_TRIG_HW,
};

static struct mcde_display_dsi_platform_data toshiba_dsi2lvds_pdata = {
	.link = 1,
	.num_data_lanes = 3,
};

static struct mcde_display_device toshiba_dsi2lvds_display = {
	.name = "toshiba_dsi2lvds",
	.id = PRIMARY_DISPLAY_ID,
	.port = &toshiba_dsi2lvds_port,
	.chnl_id = MCDE_CHNL_A,
	.fifo = MCDE_FIFO_A,
	.orientation = MCDE_DISPLAY_ROT_0,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGBA8888,
	.dev = {
		.platform_data = &toshiba_dsi2lvds_pdata,
	},
};

#ifdef CONFIG_U8500_TV_OUTPUT_AV8100

#if defined(CONFIG_AV8100_HWTRIG_INT)
	#define AV8100_SYNC_SRC MCDE_SYNCSRC_TE0
#elif defined(CONFIG_AV8100_HWTRIG_I2SDAT3)
	#define AV8100_SYNC_SRC MCDE_SYNCSRC_TE1
#elif defined(CONFIG_AV8100_HWTRIG_DSI_TE)
	#define AV8100_SYNC_SRC MCDE_SYNCSRC_TE_POLLING
#else
	#define AV8100_SYNC_SRC MCDE_SYNCSRC_OFF
#endif
static struct mcde_port av8100_port2 = {
	.type = MCDE_PORTTYPE_DSI,
	.mode = MCDE_PORTMODE_CMD,
	.pixel_format = MCDE_PORTPIXFMT_DSI_24BPP,
	.link = 2,
	.sync_src = AV8100_SYNC_SRC,
	.update_auto_trig = true,
	.phy = {
		.dsi = {
			.num_data_lanes = 2,
			.ui = DSI_UNIT_INTERVAL_2,
		},
	},
	.hdmi_sdtv_switch = HDMI_SWITCH,
};

static struct mcde_display_hdmi_platform_data av8100_hdmi_pdata = {
	.rgb_2_yCbCr_transform = &rgb_2_yCbCr_transform,
};

static struct mcde_display_device av8100_hdmi = {
	.name = "av8100_hdmi",
	.id = AV8100_DISPLAY_ID,
	.port = &av8100_port2,
	.chnl_id = MCDE_CHNL_B,
	.fifo = MCDE_FIFO_B,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGBA8888,
	.native_x_res = 1280,
	.native_y_res = 720,
	.dev = {
		.platform_data = &av8100_hdmi_pdata,
	},
};

static void delayed_work_dispreg_hdmi(struct work_struct *ptr)
{
	if (mcde_display_device_register(&av8100_hdmi))
		pr_warning("Failed to register av8100_hdmi\n");
}
#endif /* CONFIG_U8500_TV_OUTPUT_AV8100 */

#ifdef CONFIG_FB_MCDE

/*
* This function will create the framebuffer for the display that is registered.
*/
static int display_postregistered_callback(struct notifier_block *nb,
	unsigned long event, void *dev)
{
	struct mcde_display_device *ddev = dev;
	u16 width, height;
	u16 virtual_height;
	struct fb_info *fbi;
#if defined(CONFIG_COMPDEV)
	struct mcde_fb *mfb;
#endif

	if (event != MCDE_DSS_EVENT_DISPLAY_REGISTERED)
		return 0;

	if (ddev->id < 0 || ddev->id >= MCDE_NR_OF_DISPLAYS)
		return 0;

	mcde_dss_get_native_resolution(ddev, &width, &height);
	virtual_height = height * 4;

#ifndef CONFIG_MCDE_DISPLAY_HDMI_FB_AUTO_CREATE
	if (ddev->id == AV8100_DISPLAY_ID)
		goto out;
#endif

	/* Create frame buffer */
	fbi = mcde_fb_create(ddev, width, height, width, virtual_height,
				ddev->default_pixel_format, FB_ROTATE_UR);
	if (IS_ERR(fbi)) {
		dev_warn(&ddev->dev,
			"Failed to create fb for display %s\n", ddev->name);
		goto display_postregistered_callback_err;
	} else {
		dev_info(&ddev->dev, "Framebuffer created (%s)\n", ddev->name);
	}

#ifdef CONFIG_COMPDEV
	/* Only create compdev for the main display */
	if (ddev->id == PRIMARY_DISPLAY_ID) {
		bool mcde_rotation = false;

		mfb = to_mcde_fb(fbi);
		/* Create a compdev overlay for this display */
		if (compdev_create(ddev, mfb->ovlys[0], mcde_rotation,
					NULL) < 0) {
			dev_warn(&ddev->dev,
				"Failed to create compdev for display %s\n",
						ddev->name);
			goto display_postregistered_callback_err;
		} else {
			dev_info(&ddev->dev, "compdev created for (%s)\n",
						ddev->name);
		}
	}
#endif
#ifndef CONFIG_MCDE_DISPLAY_HDMI_FB_AUTO_CREATE
out:
#endif
	return 0;

display_postregistered_callback_err:
	return -1;
}

static struct notifier_block display_nb = {
	.notifier_call = display_postregistered_callback,
};
#endif /* CONFIG_FB_MCDE */

static int __init handle_display_devices(void)
{
	struct mcde_platform_data *pdata = ux500_mcde_device.dev.platform_data;
	pr_debug("%s\n", __func__);

#ifdef CONFIG_FB_MCDE
	(void)mcde_dss_register_notifier(&display_nb);
#endif

	/* Initialize all needed clocks*/
	if (!display_initialized_during_boot) {
		struct clk *clk_hdmi;
		struct clk *clk_tv;
		struct clk *clk_dsiclk;
		struct clk *clk_dsitv_pll;
		struct clk *clk_dsilcd_pll;
		long round_rate;

		/*
		 * The TV CLK is used as parent for the
		 * DSI LP clock.
		 */
		clk_tv = clk_get(&ux500_mcde_device.dev, "tv");
		round_rate = clk_round_rate(clk_tv, TV_FREQ_HZ);
		if (TV_FREQ_HZ != round_rate)
			dev_warn(&ux500_mcde_device.dev,
				"%s: TV_CLK freq differs %ld\n", __func__,
								round_rate);
		clk_set_rate(clk_tv, TV_FREQ_HZ);
		clk_put(clk_tv);

		/*
		 * The HDMI CLK is used as parent for the
		 * DSI HS clock when DSITV_PLL is used.
		 */
		clk_hdmi = clk_get(&ux500_mcde_device.dev, "hdmi");
		round_rate = clk_round_rate(clk_hdmi, HDMI_FREQ_HZ);
		if (HDMI_FREQ_HZ != round_rate)
			dev_warn(&ux500_mcde_device.dev,
				"%s: HDMI freq differs %ld\n", __func__,
								round_rate);
		clk_set_rate(clk_hdmi, HDMI_FREQ_HZ);
		clk_put(clk_hdmi);

		/*
		 * The DSI PLL CLK is used as DSI PLL for direct freq for
		 * link 2.
		 */
		clk_dsitv_pll = clk_get(&ux500_mcde_device.dev, "dsihs2");
		round_rate = clk_round_rate(clk_dsitv_pll,
							DSITV_PLL_FREQ_HZ);
		if (DSITV_PLL_FREQ_HZ != round_rate)
			dev_warn(&ux500_mcde_device.dev,
				"%s: DSITV_PLL freq differs %ld\n", __func__,
								round_rate);
		clk_set_rate(clk_dsitv_pll, DSITV_PLL_FREQ_HZ);
		clk_put(clk_dsitv_pll);

		/*
		 * The SPARE1CLK is used as parent for the
		 * DSI HS clock when DSILCD_PLL is used.
		 */
		clk_dsiclk = clk_get(&ux500_mcde_device.dev, "dsilcd");
		round_rate = clk_round_rate(clk_dsiclk, DSILCD_FREQ_HZ);
		if (DSILCD_FREQ_HZ != round_rate)
			dev_warn(&ux500_mcde_device.dev,
				"%s: DSICLK freq differs %ld\n", __func__,
								round_rate);
		clk_set_rate(clk_dsiclk, DSILCD_FREQ_HZ);
		clk_put(clk_dsiclk);

		if (uib_is_u9540uibs_v1()) {
			clk_dsilcd_pll = clk_get(&ux500_mcde_device.dev,
								"dsilcd_pll");
			round_rate = clk_round_rate(clk_dsilcd_pll,
					UIB9540_S_V1_DSILCD_PLL_FREQ_HZ);
			if (UIB9540_S_V1_DSILCD_PLL_FREQ_HZ != round_rate)
				dev_warn(&ux500_mcde_device.dev,
					"%s: DSILCD_PLL freq differs %ld\n",
							__func__, round_rate);
			clk_set_rate(clk_dsilcd_pll,
					UIB9540_S_V1_DSILCD_PLL_FREQ_HZ);
			clk_put(clk_dsilcd_pll);
		} else if (uib_is_u9540uibs_v2() || uib_is_u9540uibs_v3()) {
			clk_dsilcd_pll = clk_get(&ux500_mcde_device.dev,
								"dsilcd_pll");
			round_rate = clk_round_rate(clk_dsilcd_pll,
					UIB9540_S_V2_DSILCD_PLL_FREQ_HZ);
			if (UIB9540_S_V2_DSILCD_PLL_FREQ_HZ != round_rate)
				dev_warn(&ux500_mcde_device.dev,
					"%s: DSILCD_PLL freq differs %ld\n",
							__func__, round_rate);
			clk_set_rate(clk_dsilcd_pll,
					UIB9540_S_V2_DSILCD_PLL_FREQ_HZ);
			clk_put(clk_dsilcd_pll);
		} else if (uib_is_u9540uibt()) {
			clk_dsilcd_pll = clk_get(&ux500_mcde_device.dev,
								"dsilcd_pll");
			round_rate = clk_round_rate(clk_dsilcd_pll,
					UIB9540_T_V1_DSILCD_PLL_FREQ_HZ);
			if (UIB9540_T_V1_DSILCD_PLL_FREQ_HZ != round_rate)
				dev_warn(&ux500_mcde_device.dev,
					"%s: DSILCD_PLL freq differs %ld\n",
							__func__, round_rate);
			clk_set_rate(clk_dsilcd_pll,
					UIB9540_T_V1_DSILCD_PLL_FREQ_HZ);
			clk_put(clk_dsilcd_pll);
		}
	}

	if (uib_is_u9540uibs_v1()) {
		/*
		 * MCDE pixelfetchwtrmrk levels per overlay
		 * Currently only 3 overlays can be active at the same time
		 * 0, 1 and 2 or in the video use case and cloning
		 * 0, 1 and 3.
		 */
		pdata->pixelfetchwtrmrk[0] = 64;	/* LCD 32 bpp */
		pdata->pixelfetchwtrmrk[1] = 96;	/* LCD 16 bpp */
		pdata->pixelfetchwtrmrk[2] = 192;	/* HDMI 32 bpp */
		pdata->pixelfetchwtrmrk[3] = 256;	/* HDMI 16 bpp */
		/* Set powermode to STANDBY if startup graphics is executed */
		if (display_initialized_during_boot)
			sony_acx424akp_display.power_mode = MCDE_DISPLAY_PM_STANDBY;

		sony_acx424akp_display_pdata.reset_gpio =
							UIB_9540_DISP1_RST_GPIO;
		(void)mcde_display_device_register(&sony_acx424akp_display);
	} else if (uib_is_u9540uibs_v2() || uib_is_u9540uibs_v3()) {
		pdata->pixelfetchwtrmrk[0] = 96;	/* LCD 32 bpp */
		pdata->pixelfetchwtrmrk[1] = 192;	/* LCD 16 bpp */
		pdata->pixelfetchwtrmrk[2] = 96;	/* HDMI 32 bpp */
		pdata->pixelfetchwtrmrk[3] = 192;	/* HDMI 16 bpp */
		/* Set powermode to STANDBY if startup graphics is executed */
		if (display_initialized_during_boot)
			himax_hx8392_display.power_mode = MCDE_DISPLAY_PM_STANDBY;
		himax_hx8392_display_pdata.reset_gpio = UIB_9540_DISP1_RST_GPIO;

		(void)mcde_display_device_register(&himax_hx8392_display);
	} else if (uib_is_u9540uibt()) {
		pdata->pixelfetchwtrmrk[0] = 96;	/* LCD 32 bpp */
		pdata->pixelfetchwtrmrk[1] = 192;	/* LCD 16 bpp */
		pdata->pixelfetchwtrmrk[2] = 96;	/* HDMI 32 bpp */
		pdata->pixelfetchwtrmrk[3] = 192;	/* HDMI 16 bpp */
		/* Set powermode to STANDBY if startup graphics is executed */
		if (display_initialized_during_boot)
			toshiba_dsi2lvds_display.power_mode =
							MCDE_DISPLAY_PM_STANDBY;
		toshiba_dsi2lvds_pdata.reset_gpio = UIB_9540_DISP1_RST_GPIO;
		(void)mcde_display_device_register(&toshiba_dsi2lvds_display);
	}

#if defined(CONFIG_U8500_TV_OUTPUT_AV8100)
	INIT_DELAYED_WORK_DEFERRABLE(&work_dispreg_hdmi,
			delayed_work_dispreg_hdmi);
	schedule_delayed_work(&work_dispreg_hdmi,
			msecs_to_jiffies(DISPREG_HDMI_DELAY));
#endif
	return 0;
}

static int __init init_display_devices(void)
{
	if (cpu_is_u9540())
		return handle_display_devices();
	else
		return 0;
}
module_init(init_display_devices);
