/* arch/arm/mach-msm/board-vision-keypad.c
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <mach/board_htc.h>

#include "board-spade.h"
#include "proc_comm.h"

#include <linux/mfd/pmic8058.h>
#include <linux/input/matrix_keypad.h>

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_spade."
module_param_named(keycaps, keycaps, charp, 0);

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[KEY] %s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}


static struct gpio_event_direct_entry spade_keypad_input_map[] = {
	{
		.gpio = SPADE_GPIO_KEYPAD_POWER_KEY,
		.code = KEY_POWER,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(SPADE_VOL_UP),
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(SPADE_VOL_DN),
		.code = KEY_VOLUMEDOWN,
	},
};

static void spade_setup_input_gpio(void)
{
	uint32_t inputs_gpio_table[] = {
		PCOM_GPIO_CFG(SPADE_GPIO_KEYPAD_POWER_KEY, 0, GPIO_INPUT,	GPIO_PULL_UP, GPIO_4MA),
	};

	config_gpio_table(inputs_gpio_table, ARRAY_SIZE(inputs_gpio_table));
}

static struct gpio_event_input_info spade_keypad_input_info = {
	.info.func = gpio_event_input_func,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv64 = 5 * NSEC_PER_MSEC,
	.keymap = spade_keypad_input_map,
	.keymap_size = ARRAY_SIZE(spade_keypad_input_map),
	.setup_input_gpio = spade_setup_input_gpio,
};

static struct gpio_event_info *spade_keypad_info[] = {
	&spade_keypad_input_info.info,
};

static struct gpio_event_platform_data spade_keypad_data = {
	.names = {
		"spade-keypad",
		NULL,
	},
	.info = spade_keypad_info,
	.info_count = ARRAY_SIZE(spade_keypad_info),
};

static struct platform_device spade_keypad_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &spade_keypad_data,
	},
};

/*
static int spade_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};
*/

static struct keyreset_platform_data spade_reset_keys_pdata = {
  //	.keys_up = spade_reset_keys_up,
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		BTN_MOUSE,
		0
	},
};

struct platform_device spade_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &spade_reset_keys_pdata,
};

int __init spade_init_keypad(void)
{
	printk(KERN_DEBUG "%s\n",	__func__);

	if (platform_device_register(&spade_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&spade_keypad_input_device);
}

