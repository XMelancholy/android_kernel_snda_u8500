/*
 * Copyright (C) 2010 ST-Ericsson SA
 *
 * License terms:GNU General Public License (GPL) version 2
 */

#include <linux/i2c.h>
#include <linux/i2c-gpio.h>

#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/rmi.h>
#include <linux/platform_device.h>

#include "board-mop500.h"

static int reset_gpio = HREFV60_TOUCH_RST_GPIO;

static int rmi_gpio_config(void *gpio_data, bool configure)
{
	int *gpio = gpio_data;
	printk("\n %s gpio:%d configure:%d\n",__func__,*gpio,configure);
	if (configure == true) {
		gpio_request(*gpio,"rmi-rest");
		gpio_direction_output(*gpio,1);
		gpio_free(*gpio);
	}
	return 0;
}

static int rmi_gpio_reset(void *gpio_data)
{
	int *gpio = gpio_data;
	printk("\n%s: enter reset--\n",__func__);
	gpio_request(*gpio,"rmi-rest");

	gpio_direction_output(*gpio,1);
	msleep(10);
	gpio_direction_output(*gpio,0);
	msleep(10);
	gpio_direction_output(*gpio,1);
	msleep(100);
	gpio_free(*gpio);

	return 0;
}

/* Descriptor structure.
 * Describes the number of i2c devices on the bus that speak RMI4.
 */
static struct rmi_device_platform_data rmi4_ts_platformdata = {
	.driver_name = "rmi_generic",
	.sensor_name = "tpk43",
	.attn_gpio = 6,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.gpio_data = &reset_gpio,
	.gpio_config = rmi_gpio_config,
	.gpio_reset = rmi_gpio_reset,
	.axis_align = {
		.flip_x = false,
 		.flip_y = false,
		.virtual_Y_high = 1899,	// max x:y 1068:2054
	},
};

static struct i2c_board_info __initdata u8500_i2c3_devices_rmi4_touch[] = {
	{
		/* Synaptics RMI4 Touschscreen */
		.type			= "rmi_i2c",
		.addr			= 0x20,
		.platform_data		= &rmi4_ts_platformdata,
	},
};


static struct i2c_board_info __initdata u8500_i2c4_devices_rmi4_touch[]  = {         
    {                                                                      
        I2C_BOARD_INFO("rmi_i2c", 0x20),                                    
        .platform_data  = &rmi4_ts_platformdata,                                            
    },                                                                     
    {                                                                      
    },                                               
};

static struct i2c_gpio_platform_data u8500_i2c4__gpio_data = { 
    .sda_pin    = 229,                                           
    .scl_pin    = 230,                                           
    .udelay     = 50,                                                      
    .timeout    = 1000,                                                    
};                                                   

static struct platform_device u8500_i2c4_gpio_device = {
    .name   = "i2c-gpio",
    .id = 4,                                         
    .dev    = {
        .platform_data = &u8500_i2c4__gpio_data, 
    },
};                     

extern int platform_device_register(struct platform_device *pdev);
void __init mop500_rmi4_touch_init(void)
{

//#if defined(CONFIG_I2C_GPIO)                                                            
    //platform_device_register(&u8500_i2c4_gpio_device);
    //i2c_register_board_info(4, u8500_i2c4_devices_rmi4_touch, ARRAY_SIZE(u8500_i2c4_devices_rmi4_touch)); 
//#endif                                                                                  
	i2c_register_board_info(3, u8500_i2c3_devices_rmi4_touch,
				ARRAY_SIZE(u8500_i2c3_devices_rmi4_touch));
}
