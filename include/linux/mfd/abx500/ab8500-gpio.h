/*
 * Copyright ST-Ericsson 2010.
 *
 * Author: Bibek Basu <bibek.basu@stericsson.com>
 * Licensed under GPLv2.
 */

#ifndef _AB8500_GPIO_H
#define _AB8500_GPIO_H

#include <mach/gpio.h>

/*
 * Platform data to register a block: only the initial gpio/irq number.
 * Array sizes are large enough to contain all AB8500 and AB9540 GPIO
 * registers.
 */

struct ab8500_gpio_platform_data {
	int gpio_base;
	u32 irq_base;
	u8  config_reg[8];
	u8  config_direction[7];
	u8  config_pullups[7];
	u8  config_vinsel;
	u8  config_pullupdown;
};

enum ab8540_gpio_pull_updown {
	AB8540_GPIO_PULL_DOWN = 0x0,
	AB8540_GPIO_PULL_NONE = 0x1,
	AB8540_GPIO_PULL_UP = 0x3,
};

enum ab8540_gpio_vinsel {
	AB8540_GPIO_VINSEL_VBAT = 0x0,
	AB8540_GPIO_VINSEL_VIN_1V8 = 0x1,
	AB8540_GPIO_VINSEL_VDD_BIF = 0x2,
};

int ab8500_config_pulldown(struct device *dev,
				int gpio, bool enable);

/* Selects pull up/pull down for GPIOx_VBAT, x=1 to 4 */
int ab8540_config_pull_updown(struct device *dev,
				int gpio, enum ab8540_gpio_pull_updown val);

/* Selects voltage for GPIOx_VBAT, x=1 to 4 */
int ab8540_gpio_config_vinsel(struct device *dev,
				int gpio, enum ab8540_gpio_vinsel val);

int ab8500_gpio_config_select(struct device *dev,
				int gpio, bool gpio_select);

int ab8500_gpio_config_get_select(struct device *dev,
				int gpio, bool *gpio_select);

#endif /* _AB8500_GPIO_H */
