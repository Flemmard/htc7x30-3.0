/*
 * Definitions for SAR Sensor CY8C20X34 chip.
 */

#ifndef SAR_SENSOR_CY8C20X34_H
#define SAR_SENSOR_CY8C20X34_H

#include <linux/ioctl.h>

#define SAR_LOGD(fmt, args...) pr_debug("[SAR] "fmt, ##args)
#define SAR_LOGI(fmt, args...) pr_info("[SAR] "fmt, ##args)
#define SAR_LOGE(fmt, args...) pr_err("[SAR ERR] "fmt, ##args)

#ifdef CONFIG_SENSORS_CAPSENSE
#define CAPSENSE_NAME "CapSense"

/* SAR Sensor register address */
#define BTN_STATE		(uint8_t) 0x00
#define FW_VERSION		(uint8_t) 0x01
#define IDAC_SAR_BTN		(uint8_t) 0x02
#define CS_DEBOUNCE		(uint8_t) 0x03
#define CS_SLEEP_MODE		(uint8_t) 0x04
#define THD_SAR_BTN_H		(uint8_t) 0x05
#define THD_SAR_BTN_L		(uint8_t) 0x06
#define CS_SLEEP_DELAY		(uint8_t) 0x07
#define SCAN_TIME_SAR_BTN	(uint8_t) 0x08
#define CS_INTERRUPT_CNT	(uint8_t) 0x09
#define CS_BTN_SELECT		(uint8_t) 0x0A
#define CS_BTN_BASELINE_H	(uint8_t) 0x0B
#define CS_BTN_BASELINE_L	(uint8_t) 0x0C
#define CS_BTN_RAW_COUNT_H	(uint8_t) 0x0D
#define CS_BTN_RAW_COUNT_L	(uint8_t) 0x0E
#define CS_BTN_DIFF_H		(uint8_t) 0x0F
#define CS_BTN_DIFF_L		(uint8_t) 0x10
#define CS_RESERVED		(uint8_t) 0x11
#define CS_HYSTERESIS		(uint8_t) 0x12
#define CS_DESC_FLT1		(uint8_t) 0x13
#define CS_DESC_FLT2		(uint8_t) 0x14
#define CS_RESET_VALUE		(uint8_t) 0x15
#define CS_REG_CNT		CS_RESET_VALUE
#endif

#ifdef CONFIG_SENSORS_CAPSENSE_EXPRESS
#define CS_EXPRESS_NAME		"CapSense_Express"
#define CS_EXPRESS_DSA		0x00	/* default slave address */
#define CS_EXPRESS_ID		0x10	/* Device ID */

/* CapSense Express register address */
#define CSE_INPUT_PORT0		(uint8_t) 0x00
#define CSE_INPUT_PORT2		(uint8_t) 0x01

#define CSE_ENABLE0		(uint8_t) 0x06
#define CSE_ENABLE1		(uint8_t) 0x07

#define CSE_FINGER_TH_2		(uint8_t) 0x63

#define CSE_DEVICE_ID		(uint8_t) 0x7A
#define CSE_DEVICE_STATUS	(uint8_t) 0x7B
#define CSE_READ_BTN		(uint8_t) 0x81
#define CSE_READ_BL_H		(uint8_t) 0x82
#define CSE_READ_BL_L		(uint8_t) 0x83
#define CSE_READ_DIFF_H		(uint8_t) 0x84
#define CSE_READ_DIFF_L		(uint8_t) 0x85
#define CSE_READ_RAW_H		(uint8_t) 0x86
#define CSE_READ_RAW_L		(uint8_t) 0x87
#endif

extern unsigned int csa_kvalue1;
extern unsigned int csa_kvalue2;
extern unsigned int csa_kvalue3;

struct capsense_platform_data {
	void (*gpio_init) (void);
	int reset;
	int intr;
	int active_high; /* 0: low active */
};
#endif

