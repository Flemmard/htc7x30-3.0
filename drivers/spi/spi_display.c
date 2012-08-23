/* linux/driver/spi/spi_display.c
 *
 *
 * Copyright (C) 2009 HTC Corporation.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/wakelock.h>
#include <linux/rtc.h>

static struct spi_device *display_dev;

int spi_display_send_16bit(unsigned char id, unsigned data)
{
	unsigned char buffer[2];
	int tmp;
	tmp = (id<<13 | data)<<16;

	buffer[0] = tmp >> 24;
	buffer[1] = (tmp & 0x00FF0000) >> 16;
	spi_write(display_dev,buffer, 2);
	return 0;
}

int spi_display_send_9bit(struct spi_msg *msg)
{
	int tmp = 0;
	if(display_dev == NULL)
		return -1;	
	else
		display_dev->bits_per_word = 9;
	tmp = (0x0 <<8 | msg->cmd)<<23;
	msg->buffer[0] = tmp >> 24;
	msg->buffer[1] = (tmp & 0x00FF0000) >> 16;

	if(msg->len != 0) {
		int i = 0, j;
		for(j = 2; i < msg->len; i++, j+=2){
			tmp &= 0x00000000;
			tmp = (0x1<<8 | *(msg->data+i))<<23;
			msg->buffer[j] = tmp >> 24;
			msg->buffer[j+1] = (tmp & 0x00FF0000) >> 16;
		}
	}

	spi_read_write_lock(display_dev, msg, NULL, 2, 1);
	return 0;
}

static int spi_display_probe(struct spi_device *display)
{
	display_dev = display;
	return 0;
}

static int spi_display_remove(struct spi_device *display)
{
	display_dev = NULL;
	return 0;
}

static struct spi_driver spi_display = {
	 .driver = {
		.name = "spi_display",
		.owner = THIS_MODULE,
	 },
	 .probe          = spi_display_probe,
	 .remove = spi_display_remove,
};

static int __init spi_display_init(void)
{
	int rc;
	rc = spi_register_driver(&spi_display);
	return rc;
}
module_init(spi_display_init);

static void __exit spi_display_exit(void)
{
	spi_unregister_driver(&spi_display);
}
module_exit(spi_display_exit);

MODULE_LICENSE("GPL");
