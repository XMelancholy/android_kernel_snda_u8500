/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * ST-Ericsson MCDE Sony acx424akp DCS display driver
 *
 * Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/stat.h>

#include <linux/regulator/consumer.h>

#include <video/mcde_display.h>

#define RESET_DELAY_MS		11
#define RESET_LOW_DELAY_US	20
#define SLEEP_OUT_DELAY_MS	140
#define SLEEP_IN_DELAY_MS	85	/* Assume 60 Hz 5 frames */

#define DSI_HS_FREQ_HZ		420160000
#define DSI_LP_FREQ_HZ		19200000

#define LCD_AVDD_EN_PIN     149
#define LCD_BL_GPIO         68

struct device_info {
	int reset_gpio;
	int power_gpio;
	int bl_gpio;
	struct mcde_port port;
};

static inline struct device_info *get_drvdata(struct mcde_display_device *ddev)
{
	return (struct device_info *)dev_get_drvdata(&ddev->dev);
}

static int sharp_panel_initialize(struct mcde_display_device *ddev)
{
	int ret = 0;
	u8 buf[60];
	u16 te_scanline = 300;

	//send the gamma values
	//manufacture command set enable and page 1
	buf[0] = 0x55;
	buf[1] = 0xAA;
	buf[2] = 0x52;
	buf[3] = 0x08;
	buf[4] = 0x01;
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xF0, buf, 5);

	//red positive node
	//D1
	buf[0]=0x00;
	buf[1]=0x03;
	buf[2]=0x00;
	buf[3]=0x0E;
	buf[4]=0x00;
	buf[5]=0x22;
	buf[6]=0x00;
	buf[7]=0x36;
	buf[8]=0x00;
	buf[9]=0x48;
	buf[10]=0x00;
	buf[11]=0x67;
	buf[12]=0x00;
	buf[13]=0x81;
	buf[14]=0x00;
	buf[15]=0xAC;
	//D2
	buf[16]=0x00;
	buf[17]=0xCD;
	buf[18]=0x00;
	buf[19]=0xFD;
	buf[20]=0x01;
	buf[21]=0x23;
	buf[22]=0x01;
	buf[23]=0x58;
	buf[24]=0x01;
	buf[25]=0x7D;
	buf[26]=0x01;
	buf[27]=0x7E;
	buf[28]=0x01;
	buf[29]=0x9E;
	buf[30]=0x01;
	buf[31]=0xBA;
	//D3
	buf[32]=0x01;
	buf[33]=0xCA;
	buf[34]=0x01;
	buf[35]=0xDE;
	buf[36]=0x01;
	buf[37]=0xEC;
	buf[38]=0x02;
	buf[39]=0x04;
	buf[40]=0x02;
	buf[41]=0x18;
	buf[42]=0x02;
	buf[43]=0x3D;
	buf[44]=0x02;
	buf[45]=0x6A;
	buf[46]=0x02;
	buf[47]=0x8C;
	//D4
	buf[48]=0x02;
	buf[49]=0x98;
	buf[50]=0x02;
	buf[51]=0x99;
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xD1, buf, 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xD2, &(buf[16]), 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xD3, &(buf[32]), 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xD4, &(buf[48]), 4);

	//green positive node
	//D5
	buf[0]=0x00;
	buf[1]=0x5E;
	buf[2]=0x00;
	buf[3]=0x66;
	buf[4]=0x00;
	buf[5]=0x75;
	buf[6]=0x00;
	buf[7]=0x83;
	buf[8]=0x00;
	buf[9]=0x90;
	buf[10]=0x00;
	buf[11]=0xA7;
	buf[12]=0x00;
	buf[13]=0xBB;
	buf[14]=0x00;
	buf[15]=0xD9;
	//D6
	buf[16]=0x00;
	buf[17]=0xF1;
	buf[18]=0x01;
	buf[19]=0x1A;
	buf[20]=0x01;
	buf[21]=0x39;
	buf[22]=0x01;
	buf[23]=0x65;
	buf[24]=0x01;
	buf[25]=0x86;
	buf[26]=0x01;
	buf[27]=0x87;
	buf[28]=0x01;
	buf[29]=0xA4;
	buf[30]=0x01;
	buf[31]=0xBE;
	//D7
	buf[32]=0x01;
	buf[33]=0xCD;
	buf[34]=0x01;
	buf[35]=0xE1;
	buf[36]=0x01;
	buf[37]=0xEE;
	buf[38]=0x02;
	buf[39]=0x06;
	buf[40]=0x02;
	buf[41]=0x1A;
	buf[42]=0x02;
	buf[43]=0x3F;
	buf[44]=0x02;
	buf[45]=0x59;
	buf[46]=0x02;
	buf[47]=0x82;
	//D8
	buf[48]=0x02;
	buf[49]=0x98;
	buf[50]=0x02;
	buf[51]=0x99;
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xD5, buf, 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xD6, &(buf[16]), 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xD7, &(buf[32]), 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xD8, &(buf[48]), 4);
	//blue positive node
	//D9
	buf[0]=0x00;
	buf[1]=0x84;
	buf[2]=0x00;
	buf[3]=0x8C;
	buf[4]=0x00;
	buf[5]=0x9B;
	buf[6]=0x00;
	buf[7]=0xA8;
	buf[8]=0x00;
	buf[9]=0xB4;
	buf[10]=0x00;
	buf[11]=0xC9;
	buf[12]=0x00;
	buf[13]=0xDA;
	buf[14]=0x00;
	buf[15]=0xF5;
	//DD
	buf[16]=0x01;
	buf[17]=0x0C;
	buf[18]=0x01;
	buf[19]=0x30;
	buf[20]=0x01;
	buf[21]=0x4A;
	buf[22]=0x01;
	buf[23]=0x72;
	buf[24]=0x01;
	buf[25]=0x90;
	buf[26]=0x01;
	buf[27]=0x91;
	buf[28]=0x01;
	buf[29]=0xAA;
	buf[30]=0x01;
	buf[31]=0xC3;
	//DE
	buf[32]=0x01;
	buf[33]=0xD2;
	buf[34]=0x01;
	buf[35]=0xE5;
	buf[36]=0x01;
	buf[37]=0xF3;
	buf[38]=0x02;
	buf[39]=0x0B;
	buf[40]=0x02;
	buf[41]=0x1A;
	buf[42]=0x02;
	buf[43]=0x95;
	buf[44]=0x02;
	buf[45]=0x98;
	buf[46]=0x02;
	buf[47]=0x99;
	//DF
	buf[48]=0x02;
	buf[49]=0x99;
	buf[50]=0x02;
	buf[51]=0x99;
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xD9, buf, 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xDD, &(buf[16]), 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xDE, &(buf[32]), 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xDF, &(buf[48]), 4);

	//red negative node
	//E0
	buf[0]=0x00;
	buf[1]=0x01;
	buf[2]=0x00;
	buf[3]=0x0F;
	buf[4]=0x00;
	buf[5]=0x2A;
	buf[6]=0x00;
	buf[7]=0x43;
	buf[8]=0x00;
	buf[9]=0x5B;
	buf[10]=0x00;
	buf[11]=0x84;
	buf[12]=0x00;
	buf[13]=0xA8;
	buf[14]=0x00;
	buf[15]=0xE1;
	//E1
	buf[16]=0x01;
	buf[17]=0x0F;
	buf[18]=0x01;
	buf[19]=0x53;
	buf[20]=0x01;
	buf[21]=0x8A;
	buf[22]=0x01;
	buf[23]=0xDB;
	buf[24]=0x02;
	buf[25]=0x16;
	buf[26]=0x02;
	buf[27]=0x18;
	buf[28]=0x02;
	buf[29]=0x50;
	buf[30]=0x02;
	buf[31]=0x88;
	//E2
	buf[32]=0x02;
	buf[33]=0xAA;
	buf[34]=0x02;
	buf[35]=0xD8;
	buf[36]=0x02;
	buf[37]=0xFA;
	buf[38]=0x03;
	buf[39]=0x2E;
	buf[40]=0x03;
	buf[41]=0x4E;
	buf[42]=0x03;
	buf[43]=0x7D;
	buf[44]=0x03;
	buf[45]=0xB0;
	buf[46]=0x03;
	buf[47]=0xDB;
	//D4
	buf[48]=0x03;
	buf[49]=0xEC;
	buf[50]=0x03;
	buf[51]=0xEE;
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xE0, buf, 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xE1, &(buf[16]), 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xE2, &(buf[32]), 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xE3, &(buf[48]), 4);

	//green negative node
	//E4
	buf[0]=0x00;
	buf[1]=0x78;
	buf[2]=0x00;
	buf[3]=0x83;
	buf[4]=0x00;
	buf[5]=0x97;
	buf[6]=0x00;
	buf[7]=0xAA;
	buf[8]=0x00;
	buf[9]=0xBB;
	buf[10]=0x00;
	buf[11]=0xDB;
	buf[12]=0x00;
	buf[13]=0xF6;
	buf[14]=0x01;
	buf[15]=0x20;
	//E5
	buf[16]=0x01;
	buf[17]=0x42;
	buf[18]=0x01;
	buf[19]=0x7D;
	buf[20]=0x01;
	buf[21]=0xAB;
	buf[22]=0x01;
	buf[23]=0xEF;
	buf[24]=0x02;
	buf[25]=0x26;
	buf[26]=0x02;
	buf[27]=0x27;
	buf[28]=0x02;
	buf[29]=0x5B;
	buf[30]=0x02;
	buf[31]=0x90;
	//E6
	buf[32]=0x02;
	buf[33]=0xB1;
	buf[34]=0x02;
	buf[35]=0xDF;
	buf[36]=0x03;
	buf[37]=0x02;
	buf[38]=0x03;
	buf[39]=0x32;
	buf[40]=0x03;
	buf[41]=0x51;
	buf[42]=0x03;
	buf[43]=0x7F;
	buf[44]=0x03;
	buf[45]=0x9E;
	buf[46]=0x03;
	buf[47]=0xCD;
	//E7
	buf[48]=0x03;
	buf[49]=0xEC;
	buf[50]=0x03;
	buf[51]=0xEE;
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xE4, buf, 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xE5, &(buf[16]), 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xE6, &(buf[32]), 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xE7, &(buf[48]), 4);

	//blue negative node
	//E8
	buf[0]=0x00;
	buf[1]=0xAC;
	buf[2]=0x00;
	buf[3]=0xB6;
	buf[4]=0x00;
	buf[5]=0xCB;
	buf[6]=0x00;
	buf[7]=0xDD;
	buf[8]=0x00;
	buf[9]=0xEE;
	buf[10]=0x01;
	buf[11]=0x0A;
	buf[12]=0x01;
	buf[13]=0x21;
	buf[14]=0x01;
	buf[15]=0x48;
	//E9
	buf[16]=0x01;
	buf[17]=0x69;
	buf[18]=0x01;
	buf[19]=0x9E;
	buf[20]=0x01;
	buf[21]=0xC5;
	buf[22]=0x02;
	buf[23]=0x03;
	buf[24]=0x02;
	buf[25]=0x37;
	buf[26]=0x02;
	buf[27]=0x39;
	buf[28]=0x02;
	buf[29]=0x69;
	buf[30]=0x02;
	buf[31]=0x9C;
	//EA
	buf[32]=0x02;
	buf[33]=0xBD;
	buf[34]=0x02;
	buf[35]=0xE9;
	buf[36]=0x03;
	buf[37]=0x0D;
	buf[38]=0x03;
	buf[39]=0x3A;
	buf[40]=0x03;
	buf[41]=0x51;
	buf[42]=0x03;
	buf[43]=0xE8;
	buf[44]=0x03;
	buf[45]=0xEB;
	buf[46]=0x03;
	buf[47]=0xED;
	//EB
	buf[48]=0x03;
	buf[49]=0xED;
	buf[50]=0x03;
	buf[51]=0xEE;
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xE8, buf, 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xE9, &(buf[16]), 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xEA, &(buf[32]), 16);
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xEB, &(buf[48]), 4);

	//manufacture command set disable and page 0
	buf[0]=0x55;
	buf[1]=0xAA;
	buf[2]=0x52;
	buf[3]=0x00;
	buf[4]=0x00;
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0xF0, buf, 5);   

	buf[0]=0x02;
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0x36, buf, 1);

	buf[0]=te_scanline >> 8;
	buf[1]=te_scanline & 0xff;
	ret = mcde_dsi_dcs_write(ddev->chnl_state, 0x44, buf, 2);

	return ret;
}


static int power_on(struct mcde_display_device *dev)
{
	struct device_info *di = get_drvdata(dev);

	dev_dbg(&dev->dev, "%s: Reset & power on sharp display\n", __func__);

	//supply the voltage of lcd avdd by setting gpio LCD_AVDD_EN_PIN as high
	gpio_direction_output(di->power_gpio, 1);

	gpio_set_value_cansleep(di->reset_gpio, 1);
	msleep(RESET_DELAY_MS);
	gpio_set_value_cansleep(di->reset_gpio, 0);
	udelay(RESET_LOW_DELAY_US);
	gpio_set_value_cansleep(di->reset_gpio, 1);
	msleep(RESET_DELAY_MS);

	return 0;
}

static int power_off(struct mcde_display_device *dev)
{
	struct device_info *di = get_drvdata(dev);

	dev_dbg(&dev->dev, "%s:Reset & power off sharp display\n", __func__);

	gpio_set_value_cansleep(di->reset_gpio, 0);
	msleep(RESET_DELAY_MS);

	//cut off the voltage of lcd avdd by setting gpio LCD_AVDD_EN_PIN as low
	gpio_direction_output(di->power_gpio, 0);

	return 0;
}

static int display_on(struct mcde_display_device *ddev)
{
	int ret;
	u8 val = 0;
	struct device_info *di = get_drvdata(ddev);

	sharp_panel_initialize(ddev);

	ret = mcde_dsi_dcs_write(ddev->chnl_state,
						DCS_CMD_SET_TEAR_ON, &val, 1);
	if (ret)
		dev_warn(&ddev->dev,
			"%s:Failed to enable synchronized update\n", __func__);

	ret = mcde_dsi_dcs_write(ddev->chnl_state, DCS_CMD_EXIT_SLEEP_MODE,
								NULL, 0);
	if (ret)
		dev_warn(&ddev->dev, "%s: Failed to exit sleep\n", __func__);

	msleep(SLEEP_OUT_DELAY_MS);
    
	ret = mcde_dsi_dcs_write(ddev->chnl_state, DCS_CMD_SET_DISPLAY_ON, NULL, 0);
	if (ret)
		dev_warn(&ddev->dev, "%s: Failed to set display on\n", __func__);

	gpio_direction_output(di->bl_gpio, 1);
	return ret;
}

static int display_off(struct mcde_display_device *ddev)
{
	int ret;
	struct device_info *di = get_drvdata(ddev);

	dev_dbg(&ddev->dev, "Display off sharp display\n");

	gpio_direction_output(di->bl_gpio, 0);

	ret = mcde_dsi_dcs_write(ddev->chnl_state, DCS_CMD_SET_DISPLAY_OFF,
								NULL, 0);
	if (ret)
		return ret;

	ret = mcde_dsi_dcs_write(ddev->chnl_state, DCS_CMD_ENTER_SLEEP_MODE,
								NULL, 0);
	/* Wait for 4 frames or more */
	msleep(SLEEP_IN_DELAY_MS);

	return ret;
}

static int sharp_lq043t1_set_power_mode(struct mcde_display_device *ddev,
	enum mcde_display_power_mode power_mode)
{
	int ret = 0;

	dev_dbg(&ddev->dev, "%s:Set Power mode\n", __func__);

	/* OFF -> STANDBY */
	if (ddev->power_mode == MCDE_DISPLAY_PM_OFF &&
					power_mode != MCDE_DISPLAY_PM_OFF) {
		ret = power_on(ddev);
		if (ret)
			return ret;
		ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
	}

	/* STANDBY -> ON */
	if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
					power_mode == MCDE_DISPLAY_PM_ON) {

		ret = display_on(ddev);
		if (ret)
			return ret;
		ddev->power_mode = MCDE_DISPLAY_PM_ON;
	}
	/* ON -> STANDBY */
	else if (ddev->power_mode == MCDE_DISPLAY_PM_ON &&
					power_mode <= MCDE_DISPLAY_PM_STANDBY) {

		ret = display_off(ddev);
		if (ret)
			return ret;
		ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
	}

	/* STANDBY -> OFF */
	if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
					power_mode == MCDE_DISPLAY_PM_OFF) {
		ret = power_off(ddev);
		if (ret)
			return ret;
		ddev->power_mode = MCDE_DISPLAY_PM_OFF;
	}

	return mcde_chnl_set_power_mode(ddev->chnl_state, ddev->power_mode);
}

static ssize_t store_power_mode(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mcde_display_device *ddev =
		container_of(dev, struct mcde_display_device, dev);
	long val;
	int ret;

	ret = strict_strtol(buf, 0, &val);
	if (ret)
		return ret;

	if (val >=  MCDE_DISPLAY_PM_OFF && val <= MCDE_DISPLAY_PM_ON)
		sharp_lq043t1_set_power_mode(ddev, val);

	return count;
}

static ssize_t show_power_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mcde_display_device *ddev =
		container_of(dev, struct mcde_display_device, dev);

	return sprintf(buf, "%d\n", ddev->power_mode);
}

static DEVICE_ATTR(power_mode, S_IRUGO | S_IWUGO, show_power_mode, store_power_mode);

static struct attribute *display_sysfs_entries[] = {
	&dev_attr_power_mode.attr,
	NULL,
};

static struct attribute_group display_attr_group = {
	.attrs	= display_sysfs_entries,
};

static int __devinit sharp_lq043t1_probe(struct mcde_display_device *dev)
{
	int ret = 0;
	struct device_info *di;
	struct mcde_port *port;
	struct mcde_display_dsi_platform_data *pdata =
		dev->dev.platform_data;

	if (pdata == NULL || !pdata->reset_gpio) {
		dev_err(&dev->dev, "Invalid platform data\n");
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	port = dev->port;
	di->reset_gpio = pdata->reset_gpio;
	di->port.type = MCDE_PORTTYPE_DSI;
	di->port.mode = MCDE_PORTMODE_CMD;
	di->port.pixel_format = MCDE_PORTPIXFMT_DSI_24BPP;
	di->port.sync_src = dev->port->sync_src;
	if (dev->port->sync_src == MCDE_SYNCSRC_TE0 ||
				dev->port->sync_src == MCDE_SYNCSRC_TE1) {
		di->port.vsync_polarity = VSYNC_ACTIVE_HIGH;
		di->port.vsync_clock_div = 0;
		di->port.vsync_min_duration = 0;
		di->port.vsync_max_duration = 0;
	}
	di->port.frame_trig = dev->port->frame_trig;
	di->port.phy.dsi.num_data_lanes = 2;
	di->port.link = port->link;
	di->port.phy.dsi.host_eot_gen = true;
	/* TODO: Move UI to mcde_hw.c when clk_get_rate(dsi) is done */
	di->port.phy.dsi.ui = 9;
	di->port.phy.dsi.hs_freq = DSI_HS_FREQ_HZ;
	di->port.phy.dsi.lp_freq = DSI_LP_FREQ_HZ;

	ret = gpio_request(di->reset_gpio, NULL);
	if (WARN_ON(ret))
		goto reset_gpio_request_failed;

	di->power_gpio = LCD_AVDD_EN_PIN;
	ret = gpio_request(di->power_gpio, NULL);
	if (WARN_ON(ret))
		goto power_gpio_request_failed;

	di->bl_gpio = LCD_BL_GPIO;
	ret = gpio_request(di->bl_gpio, NULL);
	if (WARN_ON(ret))
		goto bl_gpio_request_failed;

	gpio_direction_output(di->reset_gpio, 1);

	dev->set_power_mode = sharp_lq043t1_set_power_mode;

	dev->port = &di->port;
	dev->native_x_res = 540;
	dev->native_y_res = 960;
	dev_set_drvdata(&dev->dev, di);

	/*
	* When u-boot has display a startup screen.
	* U-boot has turned on display power however the
	* regulator framework does not know about that
	* This is the case here, the display driver has to
	* enable the regulator for the display.
	*/
	if (dev->power_mode != MCDE_DISPLAY_PM_OFF) {
		//supply the voltage of lcd avdd by setting gpio LCD_AVDD_EN_PIN as high
		gpio_direction_output(di->power_gpio, 1);
	} else {
		power_on(dev);
		dev->power_mode = MCDE_DISPLAY_PM_STANDBY;
	}

	ret = sysfs_create_group(&dev->dev.kobj, &display_attr_group);
	if (ret)
		dev_err(&dev->dev, "error creating sysfs entries\n");

	return 0;

bl_gpio_request_failed:
	gpio_free(di->power_gpio);
power_gpio_request_failed:
	gpio_free(di->reset_gpio);
reset_gpio_request_failed:
	kfree(di);
	return ret;
}

static int __devexit sharp_lq043t1_remove(struct mcde_display_device *dev)
{
	struct device_info *di = get_drvdata(dev);

	dev->set_power_mode(dev, MCDE_DISPLAY_PM_OFF);

	gpio_direction_output(di->power_gpio, 0);
	gpio_direction_input(di->power_gpio);
	gpio_direction_input(di->reset_gpio);
	gpio_direction_output(di->bl_gpio, 0);
	gpio_direction_input(di->bl_gpio);
	gpio_free(di->reset_gpio);
	gpio_free(di->power_gpio);
	gpio_free(di->bl_gpio);

	kfree(di);

	return 0;
}

static struct mcde_display_driver sharp_lq043t1_driver = {
	.probe	= sharp_lq043t1_probe,
	.remove = sharp_lq043t1_remove,
	.driver = {
		.name	= "mcde_disp_sharp_lq043t1",
	},
};

/* Module init */
static int __init mcde_display_sharp_lq043t1_init(void)
{
	pr_info("%s\n", __func__);

	return mcde_display_driver_register(&sharp_lq043t1_driver);
}
module_init(mcde_display_sharp_lq043t1_init);

static void __exit mcde_display_sharp_lq043t1_exit(void)
{
	pr_info("%s\n", __func__);

	mcde_display_driver_unregister(&sharp_lq043t1_driver);
}
module_exit(mcde_display_sharp_lq043t1_exit);
