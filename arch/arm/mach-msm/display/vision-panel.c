#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_fb-7x30.h>
#include <mach/msm_iomap-7x30.h>
#include <mach/msm_panel.h>
#include <mach/vreg.h>
#include <mach/panel_id.h>

#include "../board-vision.h"
#include "../devices.h"
#include "../proc_comm.h"

struct vreg *vreg_ldo19, *vreg_ldo20;
struct vreg *vreg_ldo12;

#define LCM_GPIO_CFG(gpio, func)                                        \
  PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)
static uint32_t display_on_gpio_table[] = {
  LCM_GPIO_CFG(VISION_LCD_PCLK, 1),
  LCM_GPIO_CFG(VISION_LCD_DE, 1),
  LCM_GPIO_CFG(VISION_LCD_VSYNC, 1),
  LCM_GPIO_CFG(VISION_LCD_HSYNC, 1),
  LCM_GPIO_CFG(VISION_LCD_G2, 1),
  LCM_GPIO_CFG(VISION_LCD_G3, 1),
  LCM_GPIO_CFG(VISION_LCD_G4, 1),
  LCM_GPIO_CFG(VISION_LCD_G5, 1),
  LCM_GPIO_CFG(VISION_LCD_G6, 1),
  LCM_GPIO_CFG(VISION_LCD_G7, 1),
  LCM_GPIO_CFG(VISION_LCD_B3, 1),
  LCM_GPIO_CFG(VISION_LCD_B4, 1),
  LCM_GPIO_CFG(VISION_LCD_B5, 1),
  LCM_GPIO_CFG(VISION_LCD_B6, 1),
  LCM_GPIO_CFG(VISION_LCD_B7, 1),
  LCM_GPIO_CFG(VISION_LCD_R3, 1),
  LCM_GPIO_CFG(VISION_LCD_R4, 1),
  LCM_GPIO_CFG(VISION_LCD_R5, 1),
  LCM_GPIO_CFG(VISION_LCD_R6, 1),
  LCM_GPIO_CFG(VISION_LCD_R7, 1),
};

static uint32_t display_off_gpio_table[] = {
  LCM_GPIO_CFG(VISION_LCD_PCLK, 0),
  LCM_GPIO_CFG(VISION_LCD_DE, 0),
  LCM_GPIO_CFG(VISION_LCD_VSYNC, 0),
  LCM_GPIO_CFG(VISION_LCD_HSYNC, 0),
  LCM_GPIO_CFG(VISION_LCD_G2, 0),
  LCM_GPIO_CFG(VISION_LCD_G3, 0),
  LCM_GPIO_CFG(VISION_LCD_G4, 0),
  LCM_GPIO_CFG(VISION_LCD_G5, 0),
  LCM_GPIO_CFG(VISION_LCD_G6, 0),
  LCM_GPIO_CFG(VISION_LCD_G7, 0),
  LCM_GPIO_CFG(VISION_LCD_B0, 0),
  LCM_GPIO_CFG(VISION_LCD_B3, 0),
  LCM_GPIO_CFG(VISION_LCD_B4, 0),
  LCM_GPIO_CFG(VISION_LCD_B5, 0),
  LCM_GPIO_CFG(VISION_LCD_B6, 0),
  LCM_GPIO_CFG(VISION_LCD_B7, 0),
  LCM_GPIO_CFG(VISION_LCD_R0, 0),
  LCM_GPIO_CFG(VISION_LCD_R3, 0),
  LCM_GPIO_CFG(VISION_LCD_R4, 0),
  LCM_GPIO_CFG(VISION_LCD_R5, 0),
  LCM_GPIO_CFG(VISION_LCD_R6, 0),
  LCM_GPIO_CFG(VISION_LCD_R7, 0),
};

static uint32_t display_gpio_table[] = {
  VISION_LCD_PCLK,
  VISION_LCD_DE,
  VISION_LCD_VSYNC,
  VISION_LCD_HSYNC,
  VISION_LCD_G2,
  VISION_LCD_G3,
  VISION_LCD_G4,
  VISION_LCD_G5,
  VISION_LCD_G6,
  VISION_LCD_G7,
  VISION_LCD_B0,
  VISION_LCD_B3,
  VISION_LCD_B4,
  VISION_LCD_B5,
  VISION_LCD_B6,
  VISION_LCD_B7,
  VISION_LCD_R0,
  VISION_LCD_R3,
  VISION_LCD_R4,
  VISION_LCD_R5,
  VISION_LCD_R6,
  VISION_LCD_R7,
};

extern unsigned long msm_fb_base;

inline int is_samsung_panel(void)
{
  return (panel_type == SAMSUNG_PANEL || panel_type == SAMSUNG_PANELII)? 1 : 0;
}

static inline int is_sony_panel(void)
{
  return (panel_type == SONY_PANEL_SPI)? 1 : 0;
}

int panel_power_on(void)
{
  int rc;

  /* turn on L19 for OJ. Note: must before L12 */
  rc = vreg_enable(vreg_ldo19);
  if (rc) {
    pr_err("%s: LDO19 vreg enable failed (%d)\n",
           __func__, rc);
    return -1;
  }
  hr_msleep(5);
  rc = vreg_enable(vreg_ldo12);
  if (rc) {
    pr_err("%s: LDO12 vreg enable failed (%d)\n",
           __func__, rc);
    return -1;
  }
  hr_msleep(5);
  rc = vreg_enable(vreg_ldo20);
  if (rc) {
    pr_err("%s: LDO20 vreg enable failed (%d)\n",
           __func__, rc);
    return -1;
  }
  hr_msleep(5);

  if (is_samsung_panel())
    {
      hr_msleep(5);
      gpio_set_value(VISION_LCD_RSTz, 1);
      hr_msleep(25);
      gpio_set_value(VISION_LCD_RSTz, 0);
      hr_msleep(10);
      gpio_set_value(VISION_LCD_RSTz, 1);
      hr_msleep(20);
      /* XA, XB board has HW panel issue, need to set EL_EN pin */
      if(system_rev <= 1)
        gpio_set_value(VISION_EL_EN, 1);
    }
  else
    {
      hr_msleep(10);
      gpio_set_value(VISION_LCD_RSTz, 1);
      hr_msleep(10);
      gpio_set_value(VISION_LCD_RSTz, 0);
      udelay(500);
      gpio_set_value(VISION_LCD_RSTz, 1);
      hr_msleep(10);
    }

  return 0;
}

int panel_power_off(void)
{
  int rc;

  if (is_samsung_panel())
    {
      hr_msleep(5);
      if(system_rev <= 1)
        gpio_set_value(VISION_EL_EN, 0);
      gpio_set_value(VISION_LCD_RSTz, 0);
    }
  else
    {
      hr_msleep(10);
      gpio_set_value(VISION_LCD_RSTz, 0);
      hr_msleep(120);
    }

  rc = vreg_disable(vreg_ldo12);
  if (rc)
    {
      pr_err("%s: LDO12, 19, 20 vreg disable failed (%d)\n",
             __func__, rc);
      return -1;
    }
  hr_msleep(5);
  rc = vreg_disable(vreg_ldo19);
  if (rc)
    {
      pr_err("%s: LDO12, 19, 20 vreg disable failed (%d)\n",
             __func__, rc);
      return -1;
    }
  hr_msleep(5);
  rc = vreg_disable(vreg_ldo20);
  if (rc)
    {
      pr_err("%s: LDO12, 19, 20 vreg disable failed (%d)\n",
             __func__, rc);
      return -1;
    }
  hr_msleep(5);
  return 0;
}

int panel_power(int on)
{
  int rc;

  if (on)
    rc = panel_power_on();
  if (!on)
    rc = panel_power_off();

  printk(KERN_ERR "%s: Panel power %s=%d\n", __func__, (on == 1 ? "ON" : "OFF"), rc);

  return rc;
}

static int panel_gpio_switch(int on)
{
  uint32_t pin, id;

  config_gpio_table(
                    !!on ? display_on_gpio_table : display_off_gpio_table,
                    ARRAY_SIZE(display_on_gpio_table));

  if (!on) {
    for (pin = 0; pin < ARRAY_SIZE(display_gpio_table); pin++) {
      gpio_set_value(display_gpio_table[pin], 0);
    }
    id = PCOM_GPIO_CFG(VISION_LCD_R6, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA);
    msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
    id = PCOM_GPIO_CFG(VISION_LCD_R7, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA);
    msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
    gpio_set_value(VISION_LCD_R6, 0);
    gpio_set_value(VISION_LCD_R7, 0);
  }
  return 0;
}

static struct resource resources_msm_fb[] = {
  {
    .start = MSM_FB_BASE,
    .end = MSM_FB_BASE + MSM_FB_SIZE - 1,
    .flags = IORESOURCE_MEM,
  },
};

static struct panel_platform_data amoled_data = {
  .fb_res = &resources_msm_fb[0],
  .power = panel_power,
  .gpio_switch = panel_gpio_switch,
};

static struct platform_device amoled_panel[] = {
  {
    .name = "panel-tl2796a",
    .id = -1,
    .dev = { .platform_data = &amoled_data,     }
  },
  {
    .name = "panel-s6e63m0",
    .id = -1,
    .dev = { .platform_data = &amoled_data,     }
  },
};

static struct panel_platform_data sonywvga_data = {
  .fb_res = &resources_msm_fb[0],
  .power = panel_power,
  .gpio_switch = panel_gpio_switch,
};

static struct platform_device sonywvga_panel = {
  .name = "panel-sonywvga-s6d16a0x21-7x30",
  .id = -1,
  .dev = {
    .platform_data = &sonywvga_data,
  },
};

int vision_init_panel(void)
{
  int ret = 0;

  printk(KERN_ERR "%s: Sony=%d Samsung=%d Other=%d\n", __func__, is_sony_panel(), panel_type == SAMSUNG_PANEL, panel_type != SAMSUNG_PANEL && !is_sony_panel());

  vreg_ldo12 = vreg_get(NULL, "gp9");
  if (IS_ERR(vreg_ldo12)) {
    pr_err("%s: gp9 vreg get failed (%ld)\n",
           __func__, PTR_ERR(vreg_ldo12));
    return -1;
  }
  ret = vreg_set_level(vreg_ldo12, 2850);
  if (ret) {
    pr_err("%s: vreg LDO12(gp9) set level failed (%d)\n",
           __func__, ret);
    return -1;
  }

  vreg_ldo19 = vreg_get(NULL, "wlan2");
  if (IS_ERR(vreg_ldo19)) {
    pr_err("%s: wlan2 vreg get failed (%ld)\n",
           __func__, PTR_ERR(vreg_ldo19));
    return -1;
  }
  vreg_ldo20 = vreg_get(NULL, "gp13");

  if (IS_ERR(vreg_ldo20)) {
    pr_err("%s: gp13 vreg get failed (%ld)\n",
           __func__, PTR_ERR(vreg_ldo20));
    return -1;
  }

  ret = vreg_set_level(vreg_ldo19, 1800);
  if (ret) {
    pr_err("%s: vreg LDO19 set level failed (%d)\n",
           __func__, ret);
    return -1;
  }

  /*
  resources_msm_fb[0].start = msm_fb_base;
  resources_msm_fb[0].end = msm_fb_base + MSM_FB_SIZE - 1;
  */
  if(is_samsung_panel())
    ret = vreg_set_level(vreg_ldo20, 2850);
  else
    ret = vreg_set_level(vreg_ldo20, 2600);
  if (ret) {
    pr_err("%s: vreg LDO20 set level failed (%d)\n",
           __func__, ret);
    return -1;
  }

  if (is_sony_panel()) {
    ret = platform_device_register(&sonywvga_panel);
    printk(KERN_ERR "%s: registered sony panel: %d\n", __func__, ret);
  } else if (panel_type == SAMSUNG_PANEL) {
    ret = platform_device_register(&amoled_panel[0]);
    printk(KERN_ERR "%s: registered amoled panel[0]: %d\n", __func__, ret);
  } else {
    ret = platform_device_register(&amoled_panel[1]);
    printk(KERN_ERR "%s: registered amoled panel[1]: %d\n", __func__, ret);
  }
  return ret;
}
