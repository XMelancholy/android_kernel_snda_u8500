/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Maxime Coquelin <maxime.coquelin@stericsson.com> for ST Ericsson.
 * License terms: GNU General Public License (GPL) version 2
 */
#ifndef __AB8500_MODCTRL_H
#define __AB8500_MODCTRL_H

enum modpwr_states {
	MODPWR_RSTN_CLR,
	MODPWR_RSTN_SET,
};

struct modctrl_reg_pwr {
	int reg_pwr_bank;
	int reg_pwr_addr;
	int reg_pwr_mask;
};

struct abx540_modctrl_platform_data {
	int gpio_power;
	struct modctrl_reg_pwr *reg_power;

};

#define AB9540_MODEM_CTRL2_REG			0x23
#define AB9540_MODEM_CTRL2_SWDBBRSTN_BIT	BIT(2)
#define AB8540_MODEM_CTRL2_ONSWN_BIT	BIT(0)

#ifdef CONFIG_ABX540_MODCTRL

int abx540_get_modem_power(void);
int abx540_set_modem_power(enum modpwr_states state);

#else
static inline int abx540_get_modem_power(void)
{
	return 0;
}
static inline int abx540_set_modem_power(enum modpwr_states state)
{
	return 0;
}

#endif /* CONFIG_ABX540_MODCTRL */

#endif /* __AB8500_MODCTRL_H */
