/*
 * Copyright (C) 2010 HTC, Inc.
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

#ifndef __ASM_ARCH_MSM_MHL_H
#define __ASM_ARCH_MSM_MHL_H


#define MHL_SII9234_I2C_NAME   "sii9234"
#define MHL_SII9232_I2C_NAME   "sii9232"

#define MHL_SEND_DEBUG(x...) pr_info(x)

struct mhl_platform_data
{
	int i2c_addr_tpi;
	int i2c_addr_cbus;
	int gpio_intr;
	int gpio_reset;
	void (*power_switch)(int);
};

struct mhl_info
{
	struct i2c_client *i2c_client;
	void (*reset_chip)(void);
	int (*get_int_status)(void);
	int i2c_addr_tpi;
	int i2c_addr_cbus;
	uint8_t *edid_buf;
	int (*read_edid)(void);
	void (*change_hdmi_state)(int state);
	void (*change_hdcp_state)(int state);
	void (*video_switch)(int on_off);
	void (*device_suspend)(void);
	void (*device_wakeup)(void);
	void (*send_keyevent)(uint32_t key, uint32_t type);
	uint32_t (*check_hdmi_sink)(void);
	void (*cable_status_update)(bool);
};

extern void mhl_device_wakeup(void);

#endif/*__ASM_ARCH_MSM_MHL_H*/

