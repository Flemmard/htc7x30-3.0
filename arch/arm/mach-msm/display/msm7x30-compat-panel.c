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

#include "../devices.h"
#include "../proc_comm.h"

#define SPI_CONFIG              (0x00000000)
#define SPI_IO_CONTROL          (0x00000004)
#define SPI_OPERATIONAL         (0x00000030)
#define SPI_ERROR_FLAGS_EN      (0x00000038)
#define SPI_ERROR_FLAGS         (0x00000038)
#define SPI_OUTPUT_FIFO         (0x00000100)

static void __iomem *spi_base;
static struct clk *spi_clk ;
static bool hackInited = false;
int panel_init_spi_hack(void)
{
  int ret;

  if (hackInited)
    return 0;
  spi_base = ioremap(MSM_SPI_PHYS, MSM_SPI_SIZE);
  if (!spi_base)
    return -1;

  spi_clk = clk_get(&qsd_device_spi.dev, "core_clk");
  if (IS_ERR(spi_clk)) {
    pr_err("%s: unable to get spi_clk\n", __func__);
    ret = PTR_ERR(spi_clk);
    goto err_clk_get;
  }

  clk_enable(spi_clk);

  //  if(&qsd_device_spi == NULL)
    clk_set_rate(spi_clk, 26331429);
    //  else
    //    clk_set_rate(spi_clk, qsd_device_spi.max_clock_speed);

  printk("spi: SPI_CONFIG=%x\n", readl(spi_base + SPI_CONFIG));
  printk("spi: SPI_IO_CONTROL=%x\n", readl(spi_base + SPI_IO_CONTROL));
  printk("spi: SPI_OPERATIONAL=%x\n", readl(spi_base + SPI_OPERATIONAL));
  printk("spi: SPI_ERROR_FLAGS_EN=%x\n",
	 readl(spi_base + SPI_ERROR_FLAGS_EN));
  printk("spi: SPI_ERROR_FLAGS=%x\n", readl(spi_base + SPI_ERROR_FLAGS));
  printk("-%s()\n", __FUNCTION__);
  clk_disable(spi_clk);
  hackInited = true;
  return 0;

 err_clk_get:
  hackInited = false;
  iounmap(spi_base);
  return ret;
}


int qspi_send(uint32_t id, uint8_t data)
{
  uint32_t err;

  clk_enable(spi_clk);
  /* bit-5: OUTPUT_FIFO_NOT_EMPTY */
  while (readl(spi_base + SPI_OPERATIONAL) & (1<<5)) {
    if ((err = readl(spi_base + SPI_ERROR_FLAGS))) {
      pr_err("%s: ERROR: SPI_ERROR_FLAGS=0x%08x\n", __func__,
	     err);
      return -EIO;
    }
  }
  writel((0x7000 | (id << 9) | data) << 16, spi_base + SPI_OUTPUT_FIFO);
  udelay(100);
  clk_disable(spi_clk);

  return 0;
}
int qspi_send_16bit(unsigned char id, unsigned data)
{
        unsigned err ;

	clk_enable(spi_clk);
        /* bit-5: OUTPUT_FIFO_NOT_EMPTY */
        while( readl(spi_base+SPI_OPERATIONAL) & (1<<5) )
          {
            if( (err=readl(spi_base+SPI_ERROR_FLAGS)) )
              {
                printk("\rERROR:  SPI_ERROR_FLAGS=%d\r", err);
                return -1;
              }
          }
        
	writel( (id<<13 | data)<<16, spi_base+SPI_OUTPUT_FIFO );/*AUO*/
        udelay(1000);
	clk_disable(spi_clk);

	return 0;
}

int qspi_send_9bit(uint32_t id, uint8_t data)
{
  uint32_t err;

  clk_enable(spi_clk);
  while (readl(spi_base + SPI_OPERATIONAL) & (1<<5)) {
    err = readl(spi_base + SPI_ERROR_FLAGS);
    if (err) {
      pr_err("%s: ERROR: SPI_ERROR_FLAGS=0x%08x\n", __func__,
	     err);
      return -EIO;
    }
  }
  writel(((id << 8) | data) << 23, spi_base + SPI_OUTPUT_FIFO);
  udelay(100);
  clk_disable(spi_clk);

  return 0;
}

