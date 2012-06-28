/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#ifdef CONFIG_SPI_QSD
#include <linux/spi/spi.h>
#endif
#include <linux/mfd/pmic8058.h>
#include <linux/leds.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/lightsensor.h>
#include <linux/input.h>
#include <linux/atmel_qt602240.h>
#include <linux/cy8c_tma_ts.h>
#include <linux/elan_ktf2k.h>
#include <mach/atmega_microp.h>
#include <linux/akm8975.h>
#include <linux/bma150.h>
#include <linux/isl29028.h>
#include <linux/power_supply.h>
#include <linux/leds-pm8058.h>
#include <linux/msm_adc.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h>
#include <linux/htc_flashlight.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/panel_id.h>

#include <mach/system.h>
#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/camera-7x30.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#ifdef CONFIG_USB_MSM_OTG_72K
#include <mach/msm_hsusb.h>
#else
#include <linux/usb/msm_hsusb.h>
#endif
#include <mach/rpc_hsusb.h>
#include <mach/msm_spi.h>
#include <mach/qdsp5v2_2x/msm_lpa.h>
#include <mach/dma.h>
#include <linux/android_pmem.h>
#include <linux/input/msm_ts.h>
//#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
#include <mach/qdsp5v2_2x/aux_pcm.h>
#include <mach/qdsp5v2_2x/mi2s.h>
#include <mach/qdsp5v2_2x/audio_dev_ctl.h>
#include <mach/htc_battery.h>
#include <mach/tpa2051d3.h>
#include <linux/ds2746_battery.h>
#include <linux/tps65200.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>
#include <mach/socinfo.h>
#include <mach/msm_memtypes.h>
#include <asm/mach/mmc.h>
#include <asm/mach/flash.h>
#include <mach/vreg.h>
#include <linux/platform_data/qcom_crypto_device.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_pmic.h>
#include <mach/htc_headset_microp.h>

#include "devices.h"
#include "timer.h"
#ifdef CONFIG_USB_G_ANDROID
#include <mach/htc_usb.h>
#include <linux/usb/android_composite.h>
#include <linux/usb/android.h>
#include <mach/usbdiag.h>
#endif
#include "proc_comm.h"
#include "pm.h"
#include "pm-boot.h"
#include "spm.h"
#include "acpuclock.h"
#include <mach/dal_axi.h>
#include <mach/msm_serial_hs.h>
#ifdef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
#include <mach/bcm_bt_lpm.h>
#endif
#include <mach/qdsp5v2_2x/mi2s.h>
#include <mach/qdsp5v2_2x/audio_dev_ctl.h>
#include <mach/sdio_al.h>
#include "smd_private.h"
#include "board-vivo.h"
#include "board-msm7x30-regulator.h"
#include <mach/board_htc.h>
#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif
#ifdef CONFIG_BT
#include <mach/htc_bdaddress.h>
#endif
#ifdef CONFIG_USB_ACCESSORY_DETECT_BY_ADC
#include <mach/cable_detect.h>
#endif

#define GPIO_2MA	0
#define GPIO_4MA	1
#define GPIO_6MA	2
#define GPIO_8MA	3
#define GPIO_10MA	4
#define GPIO_12MA	5
#define GPIO_14MA	6
#define GPIO_16MA	7

#define GPIO_INPUT      0
#define GPIO_OUTPUT     1

#define GPIO_NO_PULL    0
#define GPIO_PULL_DOWN  1
#define GPIO_PULL_UP    3

#define PCOM_GPIO_CFG(gpio, func, dir, pull, drvstr) \
		((((gpio) & 0x3FF) << 4)        | \
		((func) & 0xf)                  | \
		(((dir) & 0x1) << 14)           | \
		(((pull) & 0x3) << 15)          | \
		(((drvstr) & 0xF) << 17))

#define XC 2
#define XD 3

struct pm8xxx_gpio_init_info {
	unsigned			gpio;
	struct pm_gpio			config;
};

static unsigned int engineerid;
extern unsigned long msm_fb_base;

static int pm8xxx_gpio_cfg(int gpio, int dir, int outb, int outv, int pull, int vsel, int out_strength, int function, int inv_int_pol)
{
  static struct pm_gpio tmp;
  tmp.direction          = dir;
  tmp.output_buffer      = outb;
  tmp.output_value       = outv;
  tmp.pull               = pull;
  tmp.vin_sel            = vsel;
  tmp.out_strength       = out_strength;
  tmp.function           = function;
  tmp.inv_int_pol        = inv_int_pol;
  return pm8xxx_gpio_config(gpio, &tmp);
}

#include <linux/workqueue.h>
static void my_work_handler(struct work_struct *w);

static struct workqueue_struct *wq = 0;
static DECLARE_DELAYED_WORK(my_work, my_work_handler);
static unsigned long delay;

static void
my_work_handler(struct work_struct *w)
{
  uint32_t restart_reason = 0x6f656d99;
  msm_proc_comm(PCOM_RESET_CHIP_IMM, &restart_reason, 0);
}

unsigned int vivo_get_engineerid(void)
{
	return engineerid;
}

#if defined(CONFIG_MSM7KV2_1X_AUDIO) || defined(CONFIG_MSM7KV2_AUDIO)
static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_mi2s_resources[] = {
	{
		.name = "hdmi",
		.start = 0xac900000,
		.end = 0xac900038,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_rx",
		.start = 0xac940040,
		.end = 0xac940078,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_tx",
		.start = 0xac980080,
		.end = 0xac9800B8,
		.flags = IORESOURCE_MEM,
	}

};

static struct msm_lpa_platform_data lpa_pdata = {
	.obuf_hlb_size = 0x2BFF8,
	.dsp_proc_id = 0,
	.app_proc_id = 2,
	.nosb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x3ff8,
		.sb_min_addr = 0,
		.sb_max_addr = 0,
	},
	.sb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x37f8,
		.sb_min_addr = 0x3800,
		.sb_max_addr = 0x3ff8,
	}
};

static struct resource msm_lpa_resources[] = {
	{
		.name = "lpa",
		.start = 0xa5000000,
		.end = 0xa50000a0,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name = "aux_codec_reg_addr",
		.start = 0xac9c00c0,
		.end = 0xac9c00c8,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 138,
		.end    = 138,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 139,
		.end    = 139,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 140,
		.end    = 140,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 141,
		.end    = 141,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};

static struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

static struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

static struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev		= {
		.platform_data = &lpa_pdata,
	},
};
#endif

static int vivo_ts_atmel_power(int on)
{
	pr_info("%s: power %d\n", __func__, on);

	if (on == 1) {
		gpio_set_value(VIVO_GPIO_TP_EN, 1);
		msleep(5);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIVO_TP_RSTz), 1);
	} else if (on == 2) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIVO_TP_RSTz), 0);
		msleep(5);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIVO_TP_RSTz), 1);
		msleep(40);
	}

	return 0;
}

struct atmel_i2c_platform_data vivo_ts_atmel_data[] = {
	{
		.version = 0x0020,
		.source = 1,
		.abs_x_min = 1,
		.abs_x_max = 1023,
		.abs_y_min = 2,
		.abs_y_max = 966,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_TP_INT_N),
		.power = vivo_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {8, 0, 5, 5, 0, 0, 5, 40, 4, 170},
		.config_T9 = {139, 0, 0, 18, 12, 0, 16, 45, 3, 1, 10, 10, 5, 15, 4, 10, 20, 0, 0, 0, 0, 0, 6, 0, 20, 52, 150, 48, 146, 82, 20, 12},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 52, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T22 = {15, 0, 0, 0, 0, 0, 0, 0, 20, 0, 0, 0, 7, 18, 255, 255, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 200, 50, 64, 31, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 2, 8, 28, 60},
		.object_crc = {0x6D, 0xDB, 0x95},
		.cable_config = {45, 16, 8, 16},
		.GCAF_level = {20, 24, 28, 40, 63},
	},
	{
		.version = 0x0020,
		.source = 0,
		.abs_x_min = 1,
		.abs_x_max = 1023,
		.abs_y_min = 2,
		.abs_y_max = 966,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_TP_INT_N),
		.power = vivo_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {8, 0, 5, 5, 0, 0, 5, 40, 4, 170},
		.config_T9 = {139, 0, 0, 18, 12, 0, 16, 45, 3, 5, 10, 10, 5, 15, 4, 10, 20, 0, 0, 0, 0, 0, 0, 0, 0, 30, 152, 48, 146, 82, 20, 12},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 52, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T22 = {15, 0, 0, 0, 0, 0, 0, 0, 20, 0, 0, 0, 7, 18, 255, 255, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 200, 50, 64, 31, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 2, 8, 28, 60},
		.object_crc = {0xA8, 0x85, 0x00},
		.cable_config = {45, 16, 8, 16},
		.GCAF_level = {20, 24, 28, 40, 63},
	},
	{
		.version = 0x0016,
		.source = 1,
		.abs_x_min = 1,
		.abs_x_max = 1023,
		.abs_y_min = 2,
		.abs_y_max = 966,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_TP_INT_N),
		.power = vivo_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {8, 0, 10, 10, 0, 0, 10, 15},
		.config_T9 = {139, 0, 0, 18, 12, 0, 16, 30, 3, 1, 10, 10, 5, 15, 4, 10, 20, 0, 0, 0, 0, 0, 6, 0, 20, 52, 150, 48, 146, 82, 40},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {7, 0, 0, 0, 0, 0, 0, 40, 20, 4, 15, 0},
		.config_T22 = {15, 0, 0, 0, 0, 0, 0, 0, 16, 0, 1, 0, 7, 18, 255, 255, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 200, 50, 64, 31, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 2, 4, 8, 60},
		.object_crc = {0x75, 0x0F, 0xF7},
		.cable_config = {30, 30, 8, 16},
		.GCAF_level = {20, 24, 28, 40, 63},
	},
	{
		.version = 0x016,
		.source = 0,
		.abs_x_min = 1,
		.abs_x_max = 1023,
		.abs_y_min = 2,
		.abs_y_max = 966,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_TP_INT_N),
		.power = vivo_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {8, 0, 10, 10, 0, 0, 10, 15},
		.config_T9 = {139, 0, 0, 18, 12, 0, 16, 30, 3, 5, 10, 10, 5, 15, 4, 10, 20, 0, 0, 0, 0, 0, 6, 0, 20, 52, 150, 48, 146, 82, 40},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {7, 0, 0, 0, 0, 0, 0, 40, 20, 4, 15, 0},
		.config_T22 = {15, 0, 0, 0, 0, 0, 0, 0, 16, 0, 1, 0, 7, 18, 255, 255, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 200, 50, 64, 31, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 2, 4, 8, 60},
		.cable_config = {30, 30, 8, 16},
		.GCAF_level = {20, 24, 28, 40, 63},
	},
	{
		.version = 0x0015,
		.source = 1,
		.abs_x_min = 1,
		.abs_x_max = 1023,
		.abs_y_min = 2,
		.abs_y_max = 966,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_TP_INT_N),
		.power = vivo_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {10, 0, 20, 10, 0, 0, 5, 0},
		.config_T9 = {139, 0, 0, 18, 12, 0, 16, 45, 2, 1, 0, 5, 2, 15, 2, 10, 25, 5, 0, 0, 0, 0, 0, 0, 0, 0, 159, 47, 149, 81},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {7, 0, 0, 0, 0, 0, 0, 40, 20, 4, 15, 0},
		.config_T22 = {7, 0, 0, 0, 0, 0, 0, 0, 20, 0, 1, 10, 15, 20, 25, 30, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 200, 50, 64, 31, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 2, 4, 8, 60},
	},
	{
		.version = 0x015,
		.source = 0,
		.abs_x_min = 1,
		.abs_x_max = 1023,
		.abs_y_min = 2,
		.abs_y_max = 966,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_TP_INT_N),
		.power = vivo_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {10, 0, 20, 10, 0, 0, 5, 0},
		.config_T9 = {139, 0, 0, 18, 12, 0, 16, 45, 2, 5, 0, 5, 2, 15, 2, 10, 25, 5, 0, 0, 0, 0, 0, 0, 0, 0, 159, 47, 149, 81},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {7, 0, 0, 0, 0, 0, 0, 40, 20, 4, 15, 0},
		.config_T22 = {7, 0, 0, 0, 0, 0, 0, 0, 20, 0, 1, 10, 15, 20, 25, 30, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 200, 50, 64, 31, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 2, 4, 8, 60},
	},
};

static int vivo_elan_ktf2k_ts_power(int on)
{
	pr_info("%s: power %d\n", __func__, on);

	if (on) {
		gpio_set_value(VIVO_GPIO_TP_EN, 1);
		udelay(300);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIVO_TP_RSTz), 1);
		msleep(300);
	} else {
		gpio_set_value(VIVO_GPIO_TP_EN, 0);
		udelay(11);
	}

	return 0;
}

static int vivo_elan_ktf2k_ts_reset(void)
{
	pr_info("%s: gpio reset\n", __func__);
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIVO_TP_RSTz), 0);
	udelay(100);
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIVO_TP_RSTz), 1);
	msleep(300);

	return 0;
}

struct elan_ktf2k_i2c_platform_data vivo_ts_elan_ktf2k_data[] = {
	{
		.version = 0x0021,
		.abs_x_min = 0,
		.abs_x_max = 640,
		.abs_y_min = 0,
		.abs_y_max = 1088,
		.intr_gpio = PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_TP_INT_N),
		.power = vivo_elan_ktf2k_ts_power,
		.reset = vivo_elan_ktf2k_ts_reset,
	},
};

static int vivo_ts_cy8c_power(int on)
{
	printk(KERN_INFO "%s():\n", __func__);
	if (on) {
		gpio_set_value(VIVO_GPIO_TP_EN, 1);
		msleep(5);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIVO_TP_RSTz), 1);
	}
	return 0;
}

struct cy8c_i2c_platform_data vivo_ts_cy8c_data[] = {
	{
		.version = 0x03,
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 932,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 255,
		.power = vivo_ts_cy8c_power,
		.gpio_irq = PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_TP_INT_N),
	},
	{
		.version = 0x00,
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 932,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 255,
		.power = vivo_ts_cy8c_power,
		.gpio_irq = PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_TP_INT_N),
		.orient = 1,
	},
};

static ssize_t vivo_virtual_keys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_HOME)	":65:845:80:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)	":180:845:83:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)	":310:845:88:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":445:845:91:50"
		"\n");
}

static struct kobj_attribute vivo_elan_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.elan-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &vivo_virtual_keys_show,
};

static struct kobj_attribute vivo_cy8c_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.cy8c-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &vivo_virtual_keys_show,
};

static struct kobj_attribute vivo_atmel_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.atmel-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &vivo_virtual_keys_show,
};

static struct attribute *vivo_properties_attrs[] = {
	&vivo_elan_virtual_keys_attr.attr,
	&vivo_cy8c_virtual_keys_attr.attr,
	&vivo_atmel_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group vivo_properties_attr_group = {
	.attrs = vivo_properties_attrs,
};


/* HTC_HEADSET_GPIO Driver */
static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio		= VIVO_GPIO_35MM_HEADSET_DET,
	.key_enable_gpio	= 0,
	.mic_select_gpio	= 0,
};

static struct platform_device htc_headset_gpio = {
	.name	= "HTC_HEADSET_GPIO",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_gpio_data,
	},
};

/* HTC_HEADSET_PMIC Driver */
static struct htc_headset_pmic_platform_data htc_headset_pmic_data = {
	.driver_flag	= DRIVER_HS_PMIC_RPC_KEY |
			  DRIVER_HS_PMIC_DYNAMIC_THRESHOLD,
	.hpin_gpio	= 0,
	.hpin_irq	= 0,
	.adc_mic	= 14894,
	.adc_remote	= {0, 2502, 2860, 6822, 9086, 13614},
	.hs_controller	= HS_PMIC_CONTROLLER_2,
	.hs_switch	= HS_PMIC_SC_SWITCH_TYPE,
};

static struct platform_device htc_headset_pmic = {
	.name	= "HTC_HEADSET_PMIC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_pmic_data,
	},
};

/* HTC_HEADSET_MICROP Driver */
static struct htc_headset_microp_platform_data htc_headset_microp_data = {
	.remote_int		= 1 << 5,
	.remote_irq		= MSM_uP_TO_INT(5),
	.remote_enable_pin	= 1 << 4,
	.adc_channel		= 0x01,
	.adc_remote		= {0, 33, 38, 82, 95, 167},
};

static struct platform_device htc_headset_microp = {
	.name	= "HTC_HEADSET_MICROP",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_microp_data,
	},
};


/* HTC_HEADSET_MGR Driver */
static struct platform_device *headset_devices[] = {
        &htc_headset_microp,
	&htc_headset_gpio,
	&htc_headset_pmic,
	/* Please put the headset detection driver on the last */
};

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
	.headset_devices_num	= ARRAY_SIZE(headset_devices),
	.headset_devices	= headset_devices,
};

/* HEADSET DRIVER END */


static struct microp_function_config microp_functions[] = {
	{
		.name   = "microp_intrrupt",
		.category = MICROP_FUNCTION_INTR,
	},
	{
		.name   = "reset-int",
		.category = MICROP_FUNCTION_RESET_INT,
		.int_pin = 1 << 8,
	},
};

static struct microp_led_config up_led_config[] = {
	{
		.name = "amber",
		.type = LED_RGB,
	},
	{
		.name = "green",
		.type = LED_RGB,
	},
	{
		.name = "button-backlight",
		.type = LED_PWM,
		.led_pin = 1 << 0,
		.init_value = 30,
		.fade_time = 5,
	},
};

static struct microp_led_platform_data microp_leds_data = {
	.num_leds	= ARRAY_SIZE(up_led_config),
	.led_config	= up_led_config,
};

static struct bma150_platform_data bravo_g_sensor_pdata = {
	.microp_new_cmd = 1,
	.chip_layout = 0,
};

static struct platform_device microp_devices[] = {
	{
		.name = "leds-microp",
		.id = -1,
		.dev = {
			.platform_data = &microp_leds_data,
		},
	},
	{
		.name = BMA150_G_SENSOR_NAME,
		.dev = {
			.platform_data = &bravo_g_sensor_pdata,
		},
	},
	{
		.name	= "HTC_HEADSET_MGR",
		.id	= -1,
		.dev	= {
			.platform_data	= &htc_headset_mgr_data,
		},
	},
};

static struct pm8058_led_config pm_led_config[] = {
  {
    .name = "keyboard-backlight",
    .type = PM8058_LED_CURRENT,
    .bank = 3,
    .out_current = 100,
  },
};

static struct pm8058_led_platform_data pm8058_leds_data = {
  .led_config = pm_led_config,
  .num_leds = ARRAY_SIZE(pm_led_config),
};

static struct platform_device pm8058_leds = {
  .name   = "leds-pm8058",
  .id     = -1,
  .dev    = {
    .platform_data  = &pm8058_leds_data,
  },
};

static struct akm8975_platform_data compass_platform_data = {
	.layouts = VIVO_LAYOUTS,
};

static struct bma150_platform_data gsensor_platform_data = {
	.intr = VIVO_GPIO_GSENSOR_INT,
	.chip_layout = 1,
};

static struct tpa2051d3_platform_data tpa2051d3_platform_data = {
	//.gpio_tpa2051_spk_en = VIVO_AUD_SPK_ENO,
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(ATMEL_QT602240_NAME, 0x94 >> 1),
		.platform_data = &vivo_ts_atmel_data,
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_TP_INT_N))
	},
	{
		I2C_BOARD_INFO(CYPRESS_TMA_NAME, 0x67),
		.platform_data = &vivo_ts_cy8c_data,
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_TP_INT_N))
	},
	{
		I2C_BOARD_INFO(ELAN_KTF2K_NAME, 0x15),
		.platform_data = &vivo_ts_elan_ktf2k_data,
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_TP_INT_N))
	},
	{
		I2C_BOARD_INFO(TPA2051D3_I2C_NAME, 0xE0 >> 1),
		.platform_data = &tpa2051d3_platform_data,
	},
	{
		I2C_BOARD_INFO("tps65200", 0xD4 >> 1),
	},
};

static struct i2c_board_info i2c_sensors_devices[] = {
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x1A >> 1),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_COMPASS_INT_N)),
	},
	{
		I2C_BOARD_INFO(BMA150_I2C_NAME, 0x70 >> 1),
		.platform_data = &gsensor_platform_data,
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_GSENSOR_INT)),
	},
};

static int pm8058_gpios_init(void)
{
	/* touch panel */
	pm8xxx_gpio_cfg(VIVO_GPIO_TP_INT_N, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_UP_31P5,
		PM8058_GPIO_VIN_L5, 0, PM_GPIO_FUNC_NORMAL, 0);
	pm8xxx_gpio_cfg(VIVO_TP_RSTz, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 1, PM_GPIO_PULL_NO,
		PM8058_GPIO_VIN_L5, PM_GPIO_STRENGTH_HIGH, PM_GPIO_FUNC_NORMAL, 0);

	/* led */
	pm8xxx_gpio_cfg(VIVO_GREEN_LED, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 1, PM_GPIO_PULL_NO,
		PM8058_GPIO_VIN_L5, PM_GPIO_STRENGTH_HIGH, PM_GPIO_FUNC_2, 0);
	pm8xxx_gpio_cfg(VIVO_AMBER_LED, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 1, PM_GPIO_PULL_NO,
		PM8058_GPIO_VIN_L5, PM_GPIO_STRENGTH_HIGH, PM_GPIO_FUNC_2, 0);
	//pm8xxx_gpio_cfg(VIVO_KEYPAD_LED, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 0, PM_GPIO_PULL_NO,
	//	PM8058_GPIO_VIN_S3, PM_GPIO_STRENGTH_HIGH, PM_GPIO_FUNC_2, 0);

	/* direct key */
	pm8xxx_gpio_cfg(VIVO_VOL_UP, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_UP_31P5, PM8058_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);
	pm8xxx_gpio_cfg(VIVO_VOL_DN, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_UP_31P5, PM8058_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);

	/* sd detect */
	pm8xxx_gpio_cfg(VIVO_GPIO_SDMC_CD_N, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_UP_31P5, PM8058_GPIO_VIN_L5, 0, PM_GPIO_FUNC_NORMAL, 0);


	/* audio */
	//pm8xxx_gpio_cfg(VIVO_AUD_SPK_ENO, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 0, PM_GPIO_PULL_NO,
	//	PM8058_GPIO_VIN_L5, PM_GPIO_STRENGTH_HIGH, PM_GPIO_FUNC_NORMAL, 0);
	//pm8xxx_gpio_cfg(VIVO_AUD_HANDSET_ENO, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 0, PM_GPIO_PULL_NO,
	//	PM8058_GPIO_VIN_L5, PM_GPIO_STRENGTH_HIGH, PM_GPIO_FUNC_NORMAL, 0);

	/* P-sensor and light sensor */
	pm8xxx_gpio_cfg(VIVO_GPIO_PS_EN, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 0, PM_GPIO_PULL_NO,
		PM8058_GPIO_VIN_L5, PM_GPIO_STRENGTH_HIGH, PM_GPIO_FUNC_NORMAL, 0);
	pm8xxx_gpio_cfg(VIVO_GPIO_LS_EN, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 0, PM_GPIO_PULL_NO,
		PM8058_GPIO_VIN_L5, PM_GPIO_STRENGTH_HIGH, PM_GPIO_FUNC_NORMAL, 0);
	if (system_rev == 0)
		pm8xxx_gpio_cfg(VIVO_GPIO_PS_INT_N, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_DN, PM8058_GPIO_VIN_L5, 0, PM_GPIO_FUNC_NORMAL, 0);
	else
		pm8xxx_gpio_cfg(VIVO_GPIO_PS_INT_N, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_UP_31P5, PM8058_GPIO_VIN_L5, 0, PM_GPIO_FUNC_NORMAL, 0);

	if (system_rev >= 1) {
		/* G Sensor INT*/
		pm8xxx_gpio_cfg(VIVO_GPIO_GSENSOR_INT, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_NO, PM8058_GPIO_VIN_L5, 0, PM_GPIO_FUNC_NORMAL, 0);
		/* E-Compass INT */
		pm8xxx_gpio_cfg(VIVO_GPIO_COMPASS_INT_N, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_NO, PM8058_GPIO_VIN_L5, 0, PM_GPIO_FUNC_NORMAL, 0);
	}

	return 0;
}

/* Regulator API support */

#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
static struct platform_device msm_proccomm_regulator_dev = {
	.name = PROCCOMM_REGULATOR_DEV_NAME,
	.id   = -1,
	.dev  = {
		.platform_data = &msm7x30_proccomm_regulator_data
	}
};
#endif

static int pm8058_pwm_config(struct pwm_device *pwm, int ch, int on)
{
	struct pm_gpio pwm_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM8058_GPIO_VIN_S3,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};
	int	rc = -EINVAL;
	int	id, mode, max_mA;

	id = mode = max_mA = 0;
	switch (ch) {
	case 0:
	case 1:
	case 2:
		if (on) {
			id = 24 + ch;
			rc = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(id - 1),
							&pwm_gpio_config);
			if (rc)
				pr_err("%s: pm8xxx_gpio_config(%d): rc=%d\n",
				       __func__, id, rc);
		}
		break;

	case 3:
		id = PM_PWM_LED_KPD;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	case 4:
		id = PM_PWM_LED_0;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 40;
		break;

	case 5:
		id = PM_PWM_LED_2;
		mode = PM_PWM_CONF_PWM2;
		max_mA = 40;
		break;

	case 6:
		id = PM_PWM_LED_FLASH;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	default:
		break;
	}

	if (ch >= 3 && ch <= 6) {
		if (!on) {
			mode = PM_PWM_CONF_NONE;
			max_mA = 0;
		}
		rc = pm8058_pwm_config_led(pwm, id, mode, max_mA);
		if (rc)
			pr_err("%s: pm8058_pwm_config_led(ch=%d): rc=%d\n",
			       __func__, ch, rc);
	}

	return rc;
}

static int pm8058_pwm_enable(struct pwm_device *pwm, int ch, int on)
{
	int	rc;

	switch (ch) {
	case 7:
		rc = pm8058_pwm_set_dtest(pwm, on);
		if (rc)
			pr_err("%s: pwm_set_dtest(%d): rc=%d\n",
			       __func__, on, rc);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static struct pm8xxx_vibrator_platform_data pm8058_vib_pdata = {
       .initial_vibrate_ms  = 0,
       .level_mV = 3000,
       .max_timeout_ms = 15000,
};
static struct pm8058_pwm_pdata pm8058_pwm_data = {
	.config         = pm8058_pwm_config,
	.enable         = pm8058_pwm_enable,
};

static struct pm8xxx_irq_platform_data pm8xxx_irq_pdata = {
	.irq_base		= PMIC8058_IRQ_BASE,
	.devirq			= MSM_GPIO_TO_INT(PMIC_GPIO_INT),
	.irq_trigger_flag       = IRQF_TRIGGER_LOW,
};

static struct pm8xxx_gpio_platform_data pm8xxx_gpio_pdata = {
	.gpio_base		= PM8058_GPIO_PM_TO_SYS(0),
};

static struct pm8xxx_mpp_platform_data pm8xxx_mpp_pdata = {
	.mpp_base	= PM8058_MPP_PM_TO_SYS(0),
};

static struct pm8058_platform_data pm8058_7x30_data = {
	.irq_pdata		= &pm8xxx_irq_pdata,
	.gpio_pdata		= &pm8xxx_gpio_pdata,
	.mpp_pdata		= &pm8xxx_mpp_pdata,
        .pwm_pdata		= &pm8058_pwm_data,
        .vibrator_pdata		= &pm8058_vib_pdata,
};

#ifdef CONFIG_MSM_SSBI
static struct msm_ssbi_platform_data msm7x30_ssbi_pm8058_pdata __devinitdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
	.slave	= {
		.name			= "pm8058-core",
		.irq = MSM_GPIO_TO_INT(PMIC_GPIO_INT),
		.platform_data		= &pm8058_7x30_data,
	},
	.rspinlock_name	= "D:PMIC_SSBI",
};
#endif

static int __init buses_init(void)
{
  if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 1, GPIO_CFG_INPUT,
                                GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
    pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
           __func__, PMIC_GPIO_INT);
  
  return 0;
}

#define TIMPANI_RESET_GPIO	1

struct bahama_config_register{
	u8 reg;
	u8 value;
	u8 mask;
};

enum version{
	VER_1_0,
	VER_2_0,
	VER_UNSUPPORTED = 0xFF
};


static struct vreg *vreg_marimba_1;
static struct vreg *vreg_marimba_2;

static unsigned int msm_marimba_setup_power(void)
{
	int rc;

	rc = vreg_enable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}
	rc = vreg_enable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}

out:
	return rc;
};

static void msm_marimba_shutdown_power(void)
{
	int rc;

	rc = vreg_disable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
	}
	rc = vreg_disable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
};

/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
//#define MARIMBA_SLAVE_ID_FM_ADDR	0x2A
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66

static const char *tsadc_id = "MADC";
static const char *vregs_tsadc_name[] = {
	"gp12",
	"s2",
};
static struct vreg *vregs_tsadc[ARRAY_SIZE(vregs_tsadc_name)];

static int marimba_tsadc_power(int vreg_on)
{
	int i, rc = 0;

        for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
          if (!vregs_tsadc[i]) {
            pr_err("%s: vreg_get %s failed (%d)\n",
                   __func__, vregs_tsadc_name[i], rc);
            goto vreg_fail;
          }
          
          rc = vreg_on ? vreg_enable(vregs_tsadc[i]) :
            vreg_disable(vregs_tsadc[i]);
          if (rc < 0) {
            pr_err("%s: vreg %s %s failed (%d)\n",
                   __func__, vregs_tsadc_name[i],
                   vreg_on ? "enable" : "disable", rc);
            goto vreg_fail;
          }
        }
        /* If marimba vote for DO buffer */
        rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_DO,
                              vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
        if (rc)	{
          pr_err("%s: unable to %svote for d0 clk\n",
                 __func__, vreg_on ? "" : "de-");
          goto do_vote_fail;
        }
        msleep(5); /* ensure power is stable */
        
        return 0;
        
do_vote_fail:
vreg_fail:
	while (i) {
          if (vreg_on) {
            vreg_disable(vregs_tsadc[--i]);
          } else {
            vreg_enable(vregs_tsadc[--i]);
          }
	}

	return rc;
}

static int marimba_tsadc_vote(int vote_on)
{
	int rc = 0;

        int level = vote_on ? 1300 : 0;
        rc = pmapp_vreg_level_vote(tsadc_id, PMAPP_VREG_S2, level);
        if (rc < 0)
          pr_err("%s: vreg level %s failed (%d)\n",
                 __func__, vote_on ? "on" : "off", rc);
        
        return rc;
}

static int marimba_tsadc_init(void)
{
	int i, rc = 0;

        for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
          vregs_tsadc[i] = vreg_get(NULL, vregs_tsadc_name[i]);
          if (IS_ERR(vregs_tsadc[i])) {
            pr_err("%s: vreg get %s failed (%ld)\n",
                   __func__, vregs_tsadc_name[i],
                   PTR_ERR(vregs_tsadc[i]));
            rc = PTR_ERR(vregs_tsadc[i]);
            goto vreg_get_fail;
          }
        }

	return 0;

vreg_get_fail:
	while (i) {
          vreg_put(vregs_tsadc[--i]);
	}
	return rc;
}

static int marimba_tsadc_exit(void)
{
	int i, rc = 0;

        for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
          if (vregs_tsadc[i])
            vreg_put(vregs_tsadc[i]);
        }
        rc = pmapp_vreg_level_vote(tsadc_id, PMAPP_VREG_S2, 0);
        if (rc < 0)
          pr_err("%s: vreg level off failed (%d)\n",
                 __func__, rc);

	return rc;
}


static struct msm_ts_platform_data msm_ts_data = {
	.min_x          = 0,
	.max_x          = 4096,
	.min_y          = 0,
	.max_y          = 4096,
	.min_press      = 0,
	.max_press      = 255,
	.inv_x          = 4096,
	.inv_y          = 4096,
	.can_wakeup	= false,
};

static struct marimba_tsadc_platform_data marimba_tsadc_pdata = {
	.marimba_tsadc_power =  marimba_tsadc_power,
	.init		     =  marimba_tsadc_init,
	.exit		     =  marimba_tsadc_exit,
	.level_vote	     =  marimba_tsadc_vote,
	.tsadc_prechg_en = true,
	.can_wakeup	= false,
	.setup = {
		.pen_irq_en	=	true,
		.tsadc_en	=	true,
	},
	.params2 = {
		.input_clk_khz		=	2400,
		.sample_prd		=	TSADC_CLK_3,
	},
	.params3 = {
		.prechg_time_nsecs	=	6400,
		.stable_time_nsecs	=	6400,
		.tsadc_test_mode	=	0,
	},
	.tssc_data = &msm_ts_data,
};

static struct vreg *vreg_codec_s4;
static int msm_marimba_codec_power(int vreg_on)
{
	int rc = 0;

	if (!vreg_codec_s4) {

		vreg_codec_s4 = vreg_get(NULL, "s4");

		if (IS_ERR(vreg_codec_s4)) {
			printk(KERN_ERR "%s: vreg_get() failed (%ld)\n",
				__func__, PTR_ERR(vreg_codec_s4));
			rc = PTR_ERR(vreg_codec_s4);
			goto  vreg_codec_s4_fail;
		}
	}

	if (vreg_on) {
		rc = vreg_enable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	} else {
		rc = vreg_disable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	}

vreg_codec_s4_fail:
	return rc;
}

static struct marimba_codec_platform_data mariba_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
};

static struct marimba_platform_data marimba_pdata = {
  //.slave_id[MARIMBA_SLAVE_ID_FM]       = MARIMBA_SLAVE_ID_FM_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_CDC]	     = MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_marimba_setup_power,
	.marimba_shutdown = msm_marimba_shutdown_power,
        //	.marimba_gpio_config = msm_marimba_gpio_config_svlte,
        //	.fm = &marimba_fm_pdata,
        .tsadc = &marimba_tsadc_pdata,
	.codec = &mariba_codec_pdata,
        .tsadc_ssbi_adap = MARIMBA_SSBI_ADAP,
};

static void __init vivo_init_marimba(void)
{
	vreg_marimba_1 = vreg_get(NULL, "s2");
	if (IS_ERR(vreg_marimba_1)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_1));
		return;
	}
	vreg_marimba_2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_marimba_2)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_2));
		return;
	}
}

#if defined(CONFIG_MSM7KV2_1X_AUDIO) || defined(CONFIG_MSM7KV2_AUDIO)

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	0,
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_MODE_LP)|
	(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 1 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	 /* Concurrency 2 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 3 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 4 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 5 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 6 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

#define DEC_INSTANCE(max_instance_same, max_instance_diff) { \
	.max_instances_same_dec = max_instance_same, \
	.max_instances_diff_dec = max_instance_diff}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
};

static struct dec_instance_table dec_instance_list[][MSM_MAX_DEC_CNT] = {
	/* Non Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 2), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 2), /* WMA */
		DEC_INSTANCE(3, 2), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(1, 1), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 1), /* WMAPRO */
	},
	/* Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 3), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 3), /* WMA */
		DEC_INSTANCE(4, 3), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(2, 3), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 2), /* WMAPRO */
	},
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
	.dec_instance_list = &dec_instance_list[0][0],
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static unsigned aux_pcm_gpio_off[] = {
	PCOM_GPIO_CFG(138, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* PCM_DOUT */
	PCOM_GPIO_CFG(139, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* PCM_DIN  */
	PCOM_GPIO_CFG(140, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* PCM_SYNC */
	PCOM_GPIO_CFG(141, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* PCM_CLK  */
};

static void __init aux_pcm_gpio_init(void)
{
	config_gpio_table(aux_pcm_gpio_off,
		ARRAY_SIZE(aux_pcm_gpio_off));
}

#endif /* CONFIG_MSM7KV2_AUDIO */

#ifdef CONFIG_USB_G_ANDROID
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0BB4,
	.product_id	= 0x0c91,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.fserial_init_string = "tty:modem,tty:autobot,tty:serial,tty:autobot",
	.nluns = 1,
	.usb_id_pin_gpio = VIVO_GPIO_USB_ID_PIN,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

static struct i2c_board_info msm_marimba_board_info[] = {
	{
		I2C_BOARD_INFO("marimba", 0xc),
		.platform_data = &marimba_pdata,
	}
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},
	[MSM_PM_SLEEP_MODE_APPS_SLEEP] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},
/*
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 4594,
		.residency = 23740,
	},
*/
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE] = {
#ifdef CONFIG_MSM_STANDALONE_POWER_COLLAPSE
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 0,
#else /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
		.idle_supported = 0,
		.suspend_supported = 0,
		.idle_enabled = 0,
		.suspend_enabled = 0,
#endif /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
		.latency = 500,
		.residency = 6000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 0,
		.suspend_enabled = 1,
		.latency = 443,
		.residency = 1098,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};

#ifdef CONFIG_SPI_QSD
static struct spi_board_info msm_spi_board_info[] __initdata = {
	{
		.modalias	= "spi_qsd",
		.mode		= SPI_MODE_3,
		.bus_num	= 0,
		.chip_select	= 2,
		.max_speed_hz	= 10000000,
	},
	{
		.modalias	= "spi_aic3254",
		.mode           = SPI_MODE_1,
		.bus_num        = 0,
		.chip_select    = 3,
		.max_speed_hz   = 9963243,
	}
};
#endif


static uint32_t qsd_spi_gpio_on_table[] = {
	PCOM_GPIO_CFG(45, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
	PCOM_GPIO_CFG(47, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),
	PCOM_GPIO_CFG(48, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
	PCOM_GPIO_CFG(87, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
};

static uint32_t qsd_spi_gpio_off_table[] = {
	PCOM_GPIO_CFG(45, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
	PCOM_GPIO_CFG(47, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
	PCOM_GPIO_CFG(48, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
	PCOM_GPIO_CFG(87, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
};

static int msm_qsd_spi_gpio_config(void)
{
#if 0
	config_gpio_table(qsd_spi_gpio_on_table,
		ARRAY_SIZE(qsd_spi_gpio_on_table));
#endif
	return 0;
}

static void msm_qsd_spi_gpio_release(void)
{
#if 0
	config_gpio_table(qsd_spi_gpio_off_table,
		ARRAY_SIZE(qsd_spi_gpio_off_table));
#endif
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 26331429,
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	int rc;
	static int vbus_is_on;
	struct pm8xxx_gpio_init_info usb_vbus = {
		PM8058_GPIO_PM_TO_SYS(36),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 1,
			.vin_sel        = 2,
			.out_strength   = PM_GPIO_STRENGTH_MED,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};

	/* If VBUS is already on (or off), do nothing. */
	if (unlikely(on == vbus_is_on))
		return;

	if (on) {
          rc = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(usb_vbus.gpio), &usb_vbus.config);
		if (rc) {
			pr_err("%s PMIC GPIO 36 write failed\n", __func__);
			return;
		}
	} else {
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(36), 0);
	}

	vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
		.phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
		.vbus_power = msm_hsusb_vbus_power,
		.power_budget   = 180,
};
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	uint32_t version = 0;
	int def_vol = 3400;

	version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) >= 2 &&
			SOCINFO_VERSION_MINOR(version) >= 1) {
		def_vol = 3075;
		pr_debug("%s: default voltage:%d\n", __func__, def_vol);
	}

	if (init) {
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		vreg_set_level(vreg_3p3, def_vol);
	} else
		vreg_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return vreg_enable(vreg_3p3);

	return vreg_disable(vreg_3p3);
}

static int msm_hsusb_ldo_set_voltage(int mV)
{
	static int cur_voltage = 3400;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (cur_voltage == mV)
		return 0;

	cur_voltage = mV;

	pr_debug("%s: (%d)\n", __func__, mV);

	return vreg_set_level(vreg_3p3, mV);
}
#endif

static int phy_init_seq[] = { 0x06, 0x36, 0x0C, 0x31, 0x31, 0x32, 0x1, 0x0D, 0x1, 0x10, -1 };
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.phy_init_seq		= phy_init_seq,
	.is_phy_status_timer_on = 1,
};

static struct msm_otg_platform_data msm_otg_pdata = {
#ifdef CONFIG_USB_EHCI_MSM_72K
	.vbus_power = msm_hsusb_vbus_power,
#endif
	.pemp_level		= PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		= CDR_AUTO_RESET_DISABLE,
	.drv_ampl		= HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		= SE1_GATING_DISABLE,
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static struct platform_device msm_migrate_pages_device = {
	.name   = "msm_migrate_pages",
	.id     = -1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
       .name = "pmem_adsp",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
       .name = "pmem_audio",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_adsp_device = {
       .name = "android_pmem",
       .id = 2,
       .dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_audio_device = {
       .name = "android_pmem",
       .id = 4,
       .dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.guage_driver = GUAGE_MODEM,
	.charger = SWITCH_CHARGER,
	.m2a_cable_detect = 1,
        //	.int_data.chg_int = MSM_GPIO_TO_INT(VIVO_GPIO_CHG_INT),
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id   = -1,
	.dev  = {
		.platform_data = &htc_battery_pdev_data,
	},
};

#ifdef CONFIG_BATTERY_DS2746
static int get_thermal_id(void)
{
	return (system_rev == XD? THERMAL_1000_100_4360: THERMAL_300_100_4360);
}

static int get_battery_id(void)
{
	return BATTERY_ID_SAMSUNG_1230MAH;
}

/* battery parameters */
UINT32 m_parameter_unknown_1280mah[] = {
	10000, 4100, 5500, 3839, 2400, 3759,
	400, 3667, 0, 3397,
};
UINT32 m_parameter_samsung_1230mah[] = {
	/* capacity (in 0.01%) -> voltage (in mV)*/
	10000, 4135, 7500, 3960, 4700, 3800, 1700, 3727,
	900, 3674, 300, 3640, 0, 3420,
};

static UINT32* m_param_tbl[] = {
	m_parameter_unknown_1280mah,
	m_parameter_samsung_1230mah,
};

static UINT32 fl_25[] = {1280, 1230};
static UINT32 pd_m_coef[] = {24, 30};
static UINT32 pd_m_resl[] = {100, 100};
static UINT32 pd_t_coef[] = {140, 250};
static INT32 padc[] = {200, 200};
static INT32 pw[] = {5, 5};

static UINT32* pd_m_coef_tbl[] = {pd_m_coef,};
static UINT32* pd_m_resl_tbl[] = {pd_m_resl,};
static UINT32 capacity_deduction_tbl_01p[] = {0,};

static struct battery_parameter vivo_battery_parameter = {
	.fl_25 = fl_25,
	.pd_m_coef_tbl = pd_m_coef_tbl,
	.pd_m_coef_tbl_boot = pd_m_coef_tbl,
	.pd_m_resl_tbl = pd_m_resl_tbl,
	.pd_m_resl_tbl_boot = pd_m_resl_tbl,
	.pd_t_coef = pd_t_coef,
	.padc = padc,
	.pw = pw,
	.capacity_deduction_tbl_01p = capacity_deduction_tbl_01p,
	.id_tbl = NULL,
	.temp_index_tbl = NULL,
	.m_param_tbl = m_param_tbl,
	.m_param_tbl_size = sizeof(m_param_tbl)/sizeof(UINT32*),
};

static ds2746_platform_data ds2746_pdev_data = {
	.func_get_thermal_id = get_thermal_id,
	.func_get_battery_id = get_battery_id,
	.func_poweralg_config_init = NULL,	/* by default */
	.func_update_charging_protect_flag = NULL,	/* by default */
	.r2_kohm = 0,	/* use get_battery_id, doesn't need this */
	.batt_param = &vivo_battery_parameter,
};

static struct platform_device ds2746_battery_pdev = {
	.name = "ds2746-battery",
	.id = -1,
	.dev = {
		.platform_data = &ds2746_pdev_data,
	},
};
#endif

static int isl29028_power(int pwr_device, uint8_t enable);

#ifdef CONFIG_SUPPORT_DQ_BATTERY
static int __init check_dq_setup(char *str)
{
	if (!strcmp(str, "PASS"))
		tps65200_data.dq_result = 1;
	else
		tps65200_data.dq_result = 0;

	return 1;
}
__setup("androidboot.dq=", check_dq_setup);
#endif

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup = 0,
	.cpu_lock_supported = 1,
#ifdef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
        .exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
#endif
	/* for bcm BT */
	.bt_wakeup_pin_supported = 1,
	.bt_wakeup_pin = VIVO_GPIO_BT_CHIP_WAKE,
	.host_wakeup_pin = VIVO_GPIO_BT_HOST_WAKE,
};

#ifdef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
  .gpio_wake = VIVO_GPIO_BT_CHIP_WAKE,
  .gpio_host_wake = VIVO_GPIO_BT_HOST_WAKE,
  .request_clock_off_locked = msm_hs_request_clock_off_locked,
  .request_clock_on_locked = msm_hs_request_clock_on_locked,
};

struct platform_device vivo_bcm_bt_lpm_device = {
  .name = "bcm_bt_lpm",
  .id = 0,
  .dev = {
    .platform_data = &bcm_bt_lpm_pdata,
  },
};
#endif
#endif

#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0xA3A00000,
		.end    = 0xA3A00000 + 0x0150 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#ifdef CONFIG_MSM_VPE
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0xAD200000,
		.end	= 0xAD200000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
       .name = "msm_vpe",
       .id   = 0,
       .num_resources = ARRAY_SIZE(msm_vpe_resources),
       .resource = msm_vpe_resources,
};
#endif

#ifdef CONFIG_MSM_CAMERA
static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
#ifdef CONFIG_OV8810
	{
		I2C_BOARD_INFO("ov8810", 0x6C >> 1),
	},
#endif
#ifdef CONFIG_S5K3H1GX
	{
		I2C_BOARD_INFO("s5k3h1gx", 0x20 >> 1),
	},
#endif
};

static uint32_t camera_off_gpio_table[] = {
  PCOM_GPIO_CFG(CAM2_RST, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
  PCOM_GPIO_CFG(CAM1_PWD, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
  PCOM_GPIO_CFG(CAM2_PWD, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
  PCOM_GPIO_CFG(4, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DAT2 */
  PCOM_GPIO_CFG(5, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DAT3 */
  PCOM_GPIO_CFG(6, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DAT4 */
  PCOM_GPIO_CFG(7, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DAT5 */
  PCOM_GPIO_CFG(8, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DAT6 */
  PCOM_GPIO_CFG(9, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DAT7 */
  PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DAT8 */
  PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DAT9 */
  PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_8MA), /* PCLK */
  PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_8MA), /* HSYNC_IN */
  PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_8MA), /* VSYNC_IN */
  PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_14MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
#if 0
  PCOM_GPIO_CFG(CAM2_RST, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
  PCOM_GPIO_CFG(CAM1_PWD, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
  PCOM_GPIO_CFG(CAM2_PWD, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
#endif
  PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
  PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
  PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
  PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
  PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
  PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
  PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
  PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
  PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
  PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
  PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
  PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* MCLK */
};

static int sensor_power_enable(char *power, unsigned volt)
{
	struct vreg *vreg_gp;
	int rc;

	if (power == NULL)
		return EIO;

	vreg_gp = vreg_get(NULL, power);
	if (IS_ERR(vreg_gp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
			__func__, power, PTR_ERR(vreg_gp));
		return EIO;
	}

	rc = vreg_set_level(vreg_gp, volt);
	if (rc) {
		pr_err("%s: vreg wlan set %s level failed (%d)\n",
			__func__, power, rc);
		return EIO;
	}

	rc = vreg_enable(vreg_gp);
	if (rc) {
		pr_err("%s: vreg enable %s failed (%d)\n",
			__func__, power, rc);
		return EIO;
	}
	return rc;
}

static int sensor_power_disable(char *power)
{
	struct vreg *vreg_gp;
	int rc;
	vreg_gp = vreg_get(NULL, power);
	if (IS_ERR(vreg_gp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
			__func__, power, PTR_ERR(vreg_gp));
		return EIO;
	}

	rc = vreg_disable(vreg_gp);
	if (rc) {
		pr_err("%s: vreg disable %s failed (%d)\n",
			__func__, power, rc);
		return EIO;
	}
	return rc;
}

static int sensor_vreg_on(void)
{
  int rc;
  pr_info("%s camera vreg on\n", __func__);

  /* main camera VCM power */
  rc = sensor_power_enable("gp4", 2850);
  pr_info("sensor_power_enable(\"gp4\", 2850) == %d\n", rc);

  /* main / 2nd camera analog power */
  rc = sensor_power_enable("gp6", 2850);
  pr_info("sensor_power_enable(\"gp6\", 2850) == %d\n", rc);
  msleep(5);

  /* main / 2nd camera digital power */
  rc = sensor_power_enable("lvsw1", 1800);
  pr_info("sensor_power_enable(\"lvsw1\", 1800) == %d\n", rc);
  msleep(5);

  /* 2nd camera IO power */
  rc = sensor_power_enable("wlan2", 1800);
  pr_info("sensor_power_enable(\"wlan2\", 1800) == %d\n", rc);

  return rc;
}



static int sensor_vreg_off(void)
{
  int rc;
  pr_info("%s camera vreg off\n", __func__);

  /* 2nd camera IO power */
  rc = sensor_power_disable("wlan2");
  pr_info("sensor_power_disable(\"wlan2\") == %d\n", rc);

  /* main / 2nd camera digital power */
  rc = sensor_power_disable("lvsw1");
  pr_info("sensor_power_disable(\"lvsw1\") == %d\n", rc);

  /* main / 2nd camera analog power */
  rc = sensor_power_disable("gp6");
  pr_info("sensor_power_disable(\"gp6\") == %d\n", rc);

  /* main camera VCM power */
  rc = sensor_power_disable("gp4");
  pr_info("sensor_power_disable(\"gp4\") == %d\n", rc);

  return rc;
}

static void config_vivo_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_vivo_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] = {
	{
		.start	= 0xA6000000,
		.end	= 0xA6000000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

#ifdef CONFIG_ARCH_MSM_FLASHLIGHT
static int flashlight_control(int mode)
{
	return aat1271_flashlight_control(mode);
}
#endif

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_vivo_camera_on_gpios,
	.camera_gpio_off = config_vivo_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400
};

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.camera_flash		= flashlight_control,
	.num_flash_levels	= FLASHLIGHT_NUM,
	.low_temp_limit		= 5,
	.low_cap_limit		= 10,
};

#ifdef CONFIG_OV8810
static struct msm_camera_sensor_info msm_camera_sensor_ov8810_data = {
	.sensor_name = "ov8810",
	.sensor_reset = VIVO_CAM_RST,
	.sensor_pwd = VIVO_CAM_PWD,
	.camera_power_on = vivo_sensor_vreg_on,
	.camera_power_off = vivo_sensor_vreg_off,
	.camera_main_get_probe = vivo_camera_main_get_probe,
	.camera_main_set_probe = vivo_camera_main_set_probe,
	.pdata = &msm_camera_device_data,
	.flash_type = MSM_CAMERA_FLASH_LED,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.flash_cfg = &msm_camera_sensor_flash_cfg,
};

static struct platform_device msm_camera_sensor_ov8810 = {
	.name      = "msm_camera_ov8810",
	.dev       = {
		.platform_data = &msm_camera_sensor_ov8810_data,
	},
};
#endif
#ifdef CONFIG_S5K3H1GX
static struct msm_camera_sensor_info msm_camera_sensor_s5k3h1gx_data = {
	.sensor_name = "s5k3h1gx",
	.vcm_pwd = VIVO_CAM_RST,
	.sensor_pwd = VIVO_CAM_PWD,
	.camera_power_on = vivo_sensor_vreg_on,
	.camera_power_off = vivo_sensor_vreg_off,
	.camera_main_get_probe = vivo_camera_main_get_probe,
	.camera_main_set_probe = vivo_camera_main_set_probe,
	.pdata = &msm_camera_device_data,
	.flash_type = MSM_CAMERA_FLASH_LED,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.flash_cfg = &msm_camera_sensor_flash_cfg,
	.csi_if = 0,
};

static struct platform_device msm_camera_sensor_s5k3h1gx = {
  .name = "msm_camera_s5k3h1gx",
  .dev = {
    .platform_data = &msm_camera_sensor_s5k3h1gx_data,
  },
};
#endif
#endif /*CONFIG_MSM_CAMERA*/

#ifdef CONFIG_BT
static struct platform_device vivo_rfkill = {
	.name = "vivo_rfkill",
	.id = -1,
};
#endif

#ifdef CONFIG_MSM_SDIO_AL
static struct msm_gpio mdm2ap_status = {
	GPIO_CFG(77, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	"mdm2ap_status"
};

static int configure_mdm2ap_status(int on)
{
	if (on)
		return msm_gpios_request_enable(&mdm2ap_status, 1);
	else {
		msm_gpios_disable_free(&mdm2ap_status, 1);
		return 0;
	}
}

static int get_mdm2ap_status(void)
{
	return gpio_get_value(GPIO_PIN(mdm2ap_status.gpio_cfg));
}

static struct sdio_al_platform_data sdio_al_pdata = {
	.config_mdm2ap_status = configure_mdm2ap_status,
	.get_mdm2ap_status = get_mdm2ap_status,
	.allow_sdioc_version_major_2 = 1,
	.peer_sdioc_version_minor = 0x0001,
	.peer_sdioc_version_major = 0x0003,
	.peer_sdioc_boot_version_minor = 0x0001,
	.peer_sdioc_boot_version_major = 0x0003,
};

struct platform_device msm_device_sdio_al = {
	.name = "msm_sdio_al",
	.id = -1,
	.dev		= {
		.platform_data	= &sdio_al_pdata,
	},
};
#endif /* CONFIG_MSM_SDIO_AL */

static struct resource ram_console_resources[] = {
	{
		.start  = MSM_RAM_CONSOLE_BASE,
		.end    = MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ram_console_resources),
	.resource       = ram_console_resources,
};

#ifdef CONFIG_ARCH_MSM_FLASHLIGHT
static void config_vivo_flashlight_gpios(void)
{
	static uint32_t flashlight_gpio_table[] = {
		PCOM_GPIO_CFG(VIVO_GPIO_TORCH_EN, 0,
					GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
		PCOM_GPIO_CFG(VIVO_GPIO_FLASH_EN, 0,
					GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),

	};

	config_gpio_table(flashlight_gpio_table,
		ARRAY_SIZE(flashlight_gpio_table));
}

static struct flashlight_platform_data vivo_flashlight_data = {
	.gpio_init  = config_vivo_flashlight_gpios,
	.torch = VIVO_GPIO_TORCH_EN,
	.flash = VIVO_GPIO_FLASH_EN,
	.flash_duration_ms = 600,
	.led_count = 1,
	.chip_model = 0,
};

static struct platform_device vivo_flashlight_device = {
	.name = FLASHLIGHT_NAME,
	.dev		= {
		.platform_data	= &vivo_flashlight_data,
	},
};
#endif

#define PM8058ADC_16BIT(adc) ((adc * 2200) / 65535) /* vref=2.2v, 16-bits resolution */

int64_t vivo_get_usbid_adc(void)
{
	uint32_t adc_value = 0xffffffff;
	htc_get_usb_accessory_adc_level(&adc_value);
	adc_value = PM8058ADC_16BIT(adc_value);
	return adc_value;
}

static uint32_t usb_ID_PIN_input_table[] = {
	GPIO_CFG(VIVO_GPIO_USB_ID_PIN, 0, GPIO_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
	GPIO_CFG(VIVO_GPIO_USB_ID1_PIN, 0, GPIO_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

static uint32_t usb_ID_PIN_ouput_table[] = {
	GPIO_CFG(VIVO_GPIO_USB_ID_PIN, 0, GPIO_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

void config_vivo_usb_id_gpios(bool output)
{
	if (output) {
		config_gpio_table(usb_ID_PIN_ouput_table, ARRAY_SIZE(usb_ID_PIN_ouput_table));
		gpio_set_value(VIVO_GPIO_USB_ID_PIN, 1);
		printk(KERN_INFO "%s %d output high\n",  __func__, VIVO_GPIO_USB_ID_PIN);
	} else {
		config_gpio_table(usb_ID_PIN_input_table, ARRAY_SIZE(usb_ID_PIN_input_table));
		printk(KERN_INFO "%s %d input none pull\n",  __func__, VIVO_GPIO_USB_ID_PIN);
	}
}

static void vivo_disable_usb_charger(void)
{
	printk(KERN_INFO "%s\n", __func__);
        htc_battery_charger_disable();
}

static struct cable_detect_platform_data cable_detect_pdata = {
	.detect_type 		= CABLE_TYPE_PMIC_ADC,
	.usb_id_pin_gpio 	= VIVO_GPIO_USB_ID_PIN,
	.config_usb_id_gpios 	= config_vivo_usb_id_gpios,
	.get_adc_cb		= vivo_get_usbid_adc,
};

static struct platform_device cable_detect_device = {
	.name	= "cable_detect",
	.id	= -1,
	.dev	= {
		.platform_data = &cable_detect_pdata,
	},
};

static struct msm_gpio msm_i2c_gpios_hw[] = {
	{ GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl" },
	{ GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda" },
};

static struct msm_gpio msm_i2c_gpios_io[] = {
	{ GPIO_CFG(70, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl" },
	{ GPIO_CFG(71, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda" },
};

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(16, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(17, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};
static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(16, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(17, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};

static void
msm_i2c_gpio_config(int adap_id, int config_type)
{
	struct msm_gpio *msm_i2c_table;

	/* Each adapter gets 2 lines from the table */
	if (adap_id > 0)
		return;
	if (config_type)
		msm_i2c_table = &msm_i2c_gpios_hw[adap_id*2];
	else
		msm_i2c_table = &msm_i2c_gpios_io[adap_id*2];
	msm_gpios_enable(msm_i2c_table, 2);
}
/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
static struct vreg *qup_vreg;
#endif
static void
qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
	/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	if (qup_vreg) {
		int rc = vreg_set_level(qup_vreg, 1800);
		if (rc) {
			pr_err("%s: vreg LVS1 set level failed (%d)\n",
			__func__, rc);
		}
		rc = vreg_enable(qup_vreg);
		if (rc) {
			pr_err("%s: vreg_enable() = %d \n",
			__func__, rc);
		}
	}
#endif
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 400000,
	.pri_clk = 70,
	.pri_dat = 71,
	.rmutex  = 1,
        //	.rsl_id = "D:I2C02000021",
        .rsl_id = "S:6",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (msm_gpios_request(msm_i2c_gpios_hw, ARRAY_SIZE(msm_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_i2c_platform_data msm_i2c_2_pdata = {
	.clk_freq = 100000,
	.rmutex  = 1,
        //	.rsl_id = "D:I2C02000022",
	.rsl_id = "S:7",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_2_init(void)
{
	msm_device_i2c_2.dev.platform_data = &msm_i2c_2_pdata;
}

static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 384000,
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};

static void __init qup_device_i2c_init(void)
{
	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
	/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	qup_vreg = vreg_get(NULL, "lvsw1");
	if (IS_ERR(qup_vreg)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(qup_vreg));
	}
#endif
}

#ifdef CONFIG_I2C_SSBI
static struct msm_i2c_ssbi_platform_data msm_i2c_ssbi6_pdata = {
	.rsl_id = "D:PMIC_SSBI",
};

static struct msm_i2c_ssbi_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI",
};
#endif

static struct vreg *vreg_s3;
static struct vreg *vreg_mmc;

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};
#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT)
static struct msm_gpio sdc1_lvlshft_cfg_data[] = {
	{GPIO_CFG(35, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), "sdc1_lvlshft"},
};
#endif
static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(38, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc1_clk"},
	{GPIO_CFG(39, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_cmd"},
	{GPIO_CFG(40, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_3"},
	{GPIO_CFG(41, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_2"},
	{GPIO_CFG(42, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_1"},
	{GPIO_CFG(43, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_0"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(64, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc2_clk"},
	{GPIO_CFG(65, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc2_cmd"},
	{GPIO_CFG(66, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc2_dat_3"},
	{GPIO_CFG(67, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc2_dat_2"},
	{GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc2_dat_1"},
	{GPIO_CFG(69, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc2_dat_0"},

#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	{GPIO_CFG(115, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc2_dat_4"},
	{GPIO_CFG(114, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc2_dat_5"},
	{GPIO_CFG(113, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc2_dat_6"},
	{GPIO_CFG(112, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc2_dat_7"},
#endif
};

static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc3_clk"},
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},
	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_3"},
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_2"},
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_1"},
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_0"},
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), "sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), "sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), "sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), "sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), "sdc4_dat_0"},
};

static struct msm_gpio sdc3_sleep_cfg_data[] = {
	{GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_clk"},
	{GPIO_CFG(111, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_cmd"},
	{GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_3"},
	{GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_2"},
	{GPIO_CFG(118, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_1"},
	{GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_0"},
};

static struct msm_gpio sdc4_sleep_cfg_data[] = {
	{GPIO_CFG(58, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc4_clk"},
	{GPIO_CFG(59, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			"sdc4_cmd"},
	{GPIO_CFG(60, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			"sdc4_dat_3"},
	{GPIO_CFG(61, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			"sdc4_dat_2"},
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			"sdc4_dat_1"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			"sdc4_dat_0"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
		.sleep_cfg_data = sdc3_sleep_cfg_data,
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
		.sleep_cfg_data = sdc4_sleep_cfg_data,
	},
};

struct sdcc_vreg {
	struct vreg *vreg_data;
	unsigned level;
};

static struct sdcc_vreg sdcc_vreg_data[4];

static unsigned long vreg_sts, gpio_sts;

static uint32_t msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
		} else {
			msm_gpios_disable_free(curr->cfg_data, curr->size);
		}
	}

	return rc;
}

static uint32_t msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_vreg *curr;
	static int enabled_once[] = {0, 0, 0, 0};

	curr = &sdcc_vreg_data[dev_id - 1];

	if (!(test_bit(dev_id, &vreg_sts)^enable))
		return rc;

	if (dev_id != 4) {
		if (!enable || enabled_once[dev_id - 1])
			return 0;
	}

	if (enable) {
		if (dev_id == 4) {
			printk(KERN_INFO "%s: Enabling SD slot power\n", __func__);
			mdelay(5);
		}
		set_bit(dev_id, &vreg_sts);
		rc = vreg_set_level(curr->vreg_data, curr->level);
		if (rc) {
			printk(KERN_ERR "%s: vreg_set_level() = %d \n",
					__func__, rc);
		}
		rc = vreg_enable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		}
		enabled_once[dev_id - 1] = 1;
	} else {
		if (dev_id == 4) {
			printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
			mdelay(5);
		}
		clear_bit(dev_id, &vreg_sts);
		rc = vreg_disable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		}
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_gpio(pdev->id, (vdd ? 1 : 0));
	if (rc)
		goto out;

	if (pdev->id == 4) /* S3 is always ON and cannot be disabled */
		rc = msm_sdcc_setup_vreg(pdev->id, (vdd ? 1 : 0));
out:
	return rc;
}

#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int msm7x30_sdcc_slot_status(struct device *dev)
{
	return (unsigned int)
		!gpio_get_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(VIVO_GPIO_SDMC_CD_N));
}
#endif
#endif

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT)
static struct mmc_platform_data msm7x30_sdc1_data = {
	.ocr_mask	= MMC_VDD_165_195,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static unsigned int vivo_sdc2_slot_type = MMC_TYPE_MMC;
static struct mmc_platform_data msm7x30_sdc2_data = {
	.ocr_mask       = MMC_VDD_165_195 | MMC_VDD_27_28,
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	//	.msmsdcc_fmax	= 49152000,
	.msmsdcc_fmax	= 50000000,
	.slot_type		= &vivo_sdc2_slot_type,
	.nonremovable	= 1,
	.emmc_dma_ch	= 7,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
/* HTC_WIFI_START */
/*
static unsigned int vivo_sdc3_slot_type = MMC_TYPE_SDIO_WIFI;
static struct mmc_platform_data msm7x30_sdc3_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	.sdiowakeup_irq = MSM_GPIO_TO_INT(118),
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.slot_type		= &vivo_sdc3_slot_type,
	.nonremovable	= 0,
};
*/
/* HTC_WIFI_END */
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static unsigned int vivo_sdc4_slot_type = MMC_TYPE_SD;
static struct mmc_platform_data msm7x30_sdc4_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x30_sdcc_slot_status,
	.status_irq  = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, VIVO_GPIO_SDMC_CD_N),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif

#ifdef CONFIG_MMC_MSM_SDC4_WP_SUPPORT
	.wpswitch    = msm_sdcc_get_wpswitch,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	//	.msmsdcc_fmax	= 49152000,
	.msmsdcc_fmax	= 50000000,
	.nonremovable	= 0,
	.slot_type     = &vivo_sdc4_slot_type,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static void msm_sdc1_lvlshft_enable(void)
{
	int rc;

	/* Enable LDO5, an input to the FET that powers slot 1 */
	rc = vreg_set_level(vreg_mmc, 2850);
	if (rc)
		printk(KERN_ERR "%s: vreg_set_level() = %d \n",	__func__, rc);

	rc = vreg_enable(vreg_mmc);
	if (rc)
		printk(KERN_ERR "%s: vreg_enable() = %d \n", __func__, rc);

	/* Enable GPIO 35, to turn on the FET that powers slot 1 */
	rc = msm_gpios_request_enable(sdc1_lvlshft_cfg_data,
				ARRAY_SIZE(sdc1_lvlshft_cfg_data));
	if (rc)
		printk(KERN_ERR "%s: Failed to enable GPIO 35\n", __func__);

	rc = gpio_direction_output(GPIO_PIN(sdc1_lvlshft_cfg_data[0].gpio_cfg),
				1);
	if (rc)
		printk(KERN_ERR "%s: Failed to turn on GPIO 35\n", __func__);
}
#endif

static void __init msm7x30_init_mmc(void)
{
	vreg_s3 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_s3)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_s3));
		return;
	}

	vreg_mmc = vreg_get(NULL, "gp10");
	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_mmc));
		return;
	}

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (machine_is_msm7x30_fluid()) {
		msm7x30_sdc1_data.ocr_mask =  MMC_VDD_27_28 | MMC_VDD_28_29;
		msm_sdc1_lvlshft_enable();
	}
	sdcc_vreg_data[0].vreg_data = vreg_s3;
	sdcc_vreg_data[0].level = 1800;
	msm_add_sdcc(1, &msm7x30_sdc1_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (machine_is_msm8x55_svlte_surf())
		msm7x30_sdc2_data.msmsdcc_fmax =  24576000;
	sdcc_vreg_data[1].vreg_data = vreg_s3;
	sdcc_vreg_data[1].level = 1800;
	msm7x30_sdc2_data.swfi_latency =
		msm_pm_data[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency;
	msm_add_sdcc(2, &msm7x30_sdc2_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	sdcc_vreg_data[2].vreg_data = vreg_s3;
	sdcc_vreg_data[2].level = 1800;
/* HTC_WIFI_START */
	/*
	msm_sdcc_setup_gpio(3, 1);
	msm_add_sdcc(3, &msm7x30_sdc3_data);
	*/
	vivo_init_mmc(system_rev);
/* HTC_WIFI_END*/
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	sdcc_vreg_data[3].vreg_data = vreg_mmc;
	sdcc_vreg_data[3].level = 2850;
	msm_add_sdcc(4, &msm7x30_sdc4_data);
#endif

}

static struct msm_spm_platform_data msm_spm_data __initdata = {
	.reg_base_addr = MSM_SAW_BASE,

	.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

#ifdef CONFIG_PERFLOCK
static unsigned vivo_perf_acpu_table[] = {
	245000000,
	768000000,
	1024000000,
};

static struct perflock_platform_data vivo_perflock_data = {
	.perf_acpu_table = vivo_perf_acpu_table,
	.table_size = ARRAY_SIZE(vivo_perf_acpu_table),
};
#endif

static void vivo_reset(void)
{
	gpio_set_value(VIVO_GPIO_PS_HOLD, 0);
}

void vivo_add_usb_devices(void)
{
	printk(KERN_INFO "%s rev: %d\n", __func__, system_rev);
	android_usb_pdata.products[0].product_id =
			android_usb_pdata.product_id;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#if defined(CONFIG_USB_OTG)
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	platform_device_register(&msm_device_otg);
#endif
	platform_device_register(&msm_device_gadget_peripheral);
	platform_device_register(&android_usb_device);
}

static int __init board_serialno_setup(char *serialno)
{
	android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);

#ifdef CONFIG_MDP4_HW_VSYNC
static void vivo_te_gpio_config(void)
{
	uint32_t te_gpio_table[] = {
	PCOM_GPIO_CFG(30, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	};
	config_gpio_table(te_gpio_table, ARRAY_SIZE(te_gpio_table));
}
#endif

static struct platform_device *devices[] __initdata = {
        &ram_console_device,
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
        &msm_device_uart2,
#endif
#ifdef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
        &vivo_bcm_bt_lpm_device,
#endif
#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
        &msm_proccomm_regulator_dev,
#endif
        &asoc_msm_pcm,
        &asoc_msm_dai0,
        &asoc_msm_dai1,
#if defined(CONFIG_SND_MSM_MVS_DAI_SOC)
        &asoc_msm_mvs,
        &asoc_mvs_dai0,
        &asoc_mvs_dai1,
#endif
        &msm_device_smd,
        &msm_device_dmov,
        &qsd_device_spi,
#ifdef CONFIG_MSM_SSBI
        &msm_device_ssbi_pmic1,
#endif
#ifdef CONFIG_I2C_SSBI
        &msm_device_ssbi6,
        &msm_device_ssbi7,
#endif
        &android_pmem_device,
        &msm_migrate_pages_device,
#ifdef CONFIG_MSM_ROTATOR
        &msm_rotator_device,
#endif
        &android_pmem_adsp_device,
        &android_pmem_audio_device,
        &msm_device_i2c,
        &msm_device_i2c_2,
#if defined(CONFIG_MSM7KV2_1X_AUDIO) || defined(CONFIG_MSM7KV2_AUDIO)
        &msm_aictl_device,
        &msm_mi2s_device,
        &msm_lpa_device,
        &msm_aux_pcm_device,
#endif
#ifdef CONFIG_OV8810
        &msm_camera_sensor_ov8810,
#endif
#ifdef CONFIG_S5K3H1GX
        &msm_camera_sensor_s5k3h1gx,
#endif

        &msm_device_adspdec,
        &qup_device_i2c,
#if defined(CONFIG_MARIMBA_CORE) && \
   (defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
        /*&msm_bt_power_device,*/
#endif
        &msm_kgsl_3d0,
        &msm_kgsl_2d0,
        &msm_device_vidc_720p,
#ifdef CONFIG_MSM_GEMINI
        &msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
        &msm_vpe_device,
#endif
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
        &msm_device_tsif,
#endif
#ifdef CONFIG_MSM_SDIO_AL
        /* &msm_device_sdio_al, */
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
                defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
        &qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
                defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
        &qcedev_device,
#endif

        &htc_battery_pdev,
	&ds2746_battery_pdev,
        &msm_ebi0_thermal,
        &msm_ebi1_thermal,
#ifdef CONFIG_SERIAL_MSM_HS
        &msm_device_uart_dm1,
#endif
#ifdef CONFIG_BT
        &vivo_rfkill,
#endif
#ifdef CONFIG_ARCH_MSM_FLASHLIGHT
        &vivo_flashlight_device,
#endif
        &pm8058_leds,
        &cable_detect_device,
};

static void __init vivo_init(void)
{
	int rc = 0, i = 0;
	struct kobject *properties_kobj;
	unsigned smem_size;
	uint32_t soc_version = 0;
	struct proc_dir_entry *entry = NULL;
	char *device_mid;

	soc_version = socinfo_get_version();

	/* Must set msm_hw_reset_hook before first proc comm */
	msm_hw_reset_hook = vivo_reset;

	msm_clock_init(&msm7x30_clock_init_data);
	msm_spm_init(&msm_spm_data, 1);
	acpuclk_init(&acpuclk_7x30_soc_data);

#ifdef CONFIG_PERFLOCK
	perflock_init(&vivo_perflock_data);
#endif

#ifdef CONFIG_BT
	bt_export_bd_address();
#endif

#ifdef CONFIG_SERIAL_MSM_HS
#ifdef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
        msm_uart_dm1_pdata.rx_wakeup_irq = -1;
#else
        msm_uart_dm1_pdata.rx_wakeup_irq = gpio_to_irq(VIVO_GPIO_BT_HOST_WAKE);
        msm_device_uart_dm1.name = "msm_serial_hs_brcm";
#endif
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
	if (SOCINFO_VERSION_MAJOR(soc_version) >= 2 &&
			SOCINFO_VERSION_MINOR(soc_version) >= 1) {
		pr_debug("%s: SOC Version:2.(1 or more)\n", __func__);
		msm_otg_pdata.ldo_set_voltage = 0;
	}

#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
	msm_pm_data
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif

#ifdef CONFIG_MSM_SSBI
	msm_device_ssbi_pmic1.dev.platform_data =
				&msm7x30_ssbi_pm8058_pdata;
#endif
	buses_init();
	platform_add_devices(devices, ARRAY_SIZE(devices));

	vivo_add_usb_devices();

#ifdef CONFIG_USB_EHCI_MSM_72K
	msm_add_host(0, &msm_usb_host_pdata);
#endif
	msm7x30_init_mmc();
	msm_qsd_spi_init();

	spi_register_board_info(msm_spi_board_info, ARRAY_SIZE(msm_spi_board_info));

	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	BUG_ON(msm_pm_boot_init(MSM_PM_BOOT_CONFIG_RESET_VECTOR, ioremap(0x0, PAGE_SIZE)));

	msm_device_i2c_init();
	msm_device_i2c_2_init();

	qup_device_i2c_init();
	vivo_init_marimba();
#if defined(CONFIG_MSM7KV2_1X_AUDIO) || defined(CONFIG_MSM7KV2_AUDIO)
	aux_pcm_gpio_init();
	msm_snddev_init();
#endif

	i2c_register_board_info(2, msm_marimba_board_info,
			ARRAY_SIZE(msm_marimba_board_info));

	i2c_register_board_info(0, i2c_sensors_devices,
				ARRAY_SIZE(i2c_sensors_devices));

	i2c_register_board_info(4 /* QUP ID */, msm_camera_boardinfo,
				ARRAY_SIZE(msm_camera_boardinfo));

#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi6.dev.platform_data = &msm_i2c_ssbi6_pdata;
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif
	pm8058_gpios_init();

#if defined(CONFIG_MSM_RMT_STORAGE_CLIENT)
	rmt_storage_add_ramfs();
#endif

	entry = create_proc_read_entry("emmc", 0, NULL, emmc_partition_read_proc, NULL);
	if (!entry)
		printk(KERN_ERR"Create /proc/emmc FAILED!\n");

	entry = create_proc_read_entry("dying_processes", 0, NULL, dying_processors_read_proc, NULL);
	if (!entry)
		printk(KERN_ERR"Create /proc/dying_processes FAILED!\n");

	boot_reason = *(unsigned int *)
		(smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &smem_size));
	printk(KERN_NOTICE "Boot Reason = 0x%02x\n", boot_reason);

#ifdef CONFIG_MSM_CAMERA
          config_gpio_table(camera_on_gpio_table, ARRAY_SIZE(camera_on_gpio_table));
#endif
	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
				&vivo_properties_attr_group);
	if (!properties_kobj || rc)
		pr_err("failed to create board_properties\n");

	i2c_register_board_info(0, i2c_devices,	ARRAY_SIZE(i2c_devices));
       	vivo_init_keypad();
#ifdef CONFIG_MDP4_HW_VSYNC
	vivo_te_gpio_config();
#endif
	vivow_init_panel(system_rev);
	vivo_audio_init();
        vivo_wifi_init();
	msm_init_pmic_vibrator(3000);
        delay = msecs_to_jiffies(5000);
        wq = create_singlethread_workqueue("my");
        queue_delayed_work(wq, &my_work, delay);
}

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static struct memtype_reserve msm7x30_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_device(struct android_pmem_platform_data *pdata, unsigned long start, unsigned long size)
{
	pdata->start = start;
	pdata->size = size;
	if (pdata->start)
		pr_info("%s: pmem %s requests %lu bytes at 0x%p (0x%lx physical).\n",
			__func__, pdata->name, size, __va(start), start);
	else
		pr_info("%s: pmem %s requests %lu bytes dynamically.\n",
			__func__, pdata->name, size);
}

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	size_pmem_device(&android_pmem_adsp_pdata, 0, pmem_adsp_size);
	size_pmem_device(&android_pmem_audio_pdata, 0, pmem_audio_size);
	size_pmem_device(&android_pmem_pdata, 0, pmem_sf_size);
	msm7x30_reserve_table[MEMTYPE_EBI1].size += PMEM_KERNEL_EBI1_SIZE;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	if (p->start == 0) {
		pr_info("%s: reserve %lu bytes from memory %d for %s.\n", __func__, p->size, p->memory_type, p->name);
		msm7x30_reserve_table[p->memory_type].size += p->size;
	}
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	reserve_memory_for(&android_pmem_pdata);
#endif
}

static void __init msm7x30_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x30_paddr_to_memtype(unsigned int paddr)
{
	if (paddr < 0x40000000)
		return MEMTYPE_EBI1;
	if (paddr >= 0x40000000 && paddr < 0x80000000)
		return MEMTYPE_EBI1;
	return MEMTYPE_NONE;
}

static struct reserve_info msm7x30_reserve_info __initdata = {
	.memtype_reserve_table = msm7x30_reserve_table,
	.calculate_reserve_sizes = msm7x30_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x30_paddr_to_memtype,
};

static void __init vivo_reserve(void)
{
	reserve_info = &msm7x30_reserve_info;
	msm_reserve();
}

static void __init vivo_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_base = msm_fb_resources[0].start;
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	printk("allocating %lu bytes at %p (%lx physical) for fb\n",
			size, addr, __pa(addr));
}

static void __init vivo_map_io(void)
{
	msm_shared_ram_phys = 0x00400000;
	msm_map_msm7x30_io();
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
}

static void __init vivo_init_early(void)
{
	vivo_allocate_memory_regions();
}

static void __init vivo_fixup(struct machine_desc *desc, struct tag *tags,
								char **cmdline, struct meminfo *mi)
{
	int mem = parse_tag_memsize((const struct tag *)tags);
	engineerid = parse_tag_engineerid(tags);

	mi->nr_banks = 2;
	mi->bank[0].start = MSM_LINUX_BASE1;
	mi->bank[0].size = MSM_LINUX_SIZE1;
	mi->bank[1].start = MSM_LINUX_BASE2;
	mi->bank[1].size = MSM_LINUX_SIZE2;
	if (mem == 768)
		mi->bank[0].size += MSM_MEM_256MB_OFFSET;
}

MACHINE_START(VIVO, "vivo")
	.fixup = vivo_fixup,
	.map_io = vivo_map_io,
	.reserve = vivo_reserve,
	.init_irq = msm_init_irq,
	.init_machine = vivo_init,
	.timer = &msm_timer,
	.init_early = vivo_init_early,
MACHINE_END

