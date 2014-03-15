#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H

/*
 * 288 (#267 is the highest one actually hooked up) onchip GPIOs, plus enough
 * room for a couple of GPIO expanders.
 */

#if CONFIG_ARCH_NR_GPIO > 0
#define ARCH_NR_GPIOS CONFIG_ARCH_NR_GPIO
#else

/*
 * Warning: keep ARCH_NR_GPIOS greater than the following sum:
 * sum = NOMADIK_NR_GPIO + MOP500_EGPIO_END +
 *	max(AB8500_NUM_GPIO, AB9540_NUM_GPIO, AB8505_NUM_GPIO, AB8540_NUM_GPIO)
 *
 * Look at drivers/gpio/gpio-ab8500.c for AB8500 gpios definition.
 */
#define ARCH_NR_GPIOS	512
#endif

#define NOMADIK_NR_GPIO	288

#include <asm-generic/gpio.h>

/* Invoke gpiolibs gpio_chip abstraction */
#define gpio_get_value  __gpio_get_value
#define gpio_set_value  __gpio_set_value
#define gpio_cansleep   __gpio_cansleep
#define gpio_to_irq     __gpio_to_irq

#define MOP500_EGPIO(x)		(NOMADIK_NR_GPIO + (x))
#define MOP500_EGPIO_END	MOP500_EGPIO(8+16+18)
#define AB8500_GPIO_BASE	MOP500_EGPIO_END

/* define AB8500_PIN_GPIO(x).(x-1) as AB8500_PIN_GPIO(1) = AB8500_GPIO_BASE */
#define AB8500_PIN_GPIO(x)	(AB8500_GPIO_BASE + (x - 1))

#endif /* __ASM_ARCH_GPIO_H */
