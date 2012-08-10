/* drivers/input/touchscreen/ntrig.c - NTRIG G3.5 Touch driver
 *
 * Copyright (C) 2010-2011 HTC Corporation.
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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <mach/vreg.h>
#include <asm/mach-types.h>
#include <linux/ntrig.h>
#include <linux/ntrig_fw.h>
#include <linux/spi/spi.h>
#include <linux/stat.h>
#include <linux/earlysuspend.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/sched.h>

#if 0
#define SPIBUF(buf) do { \
	int q; \
	for (q = 0; q < 8; q++) \
		pr_info("[ts]%.2x %.2x %.2x %.2x\\%.2x %.2x %.2x %.2x\\%.2x %.2x %.2x %.2x\\%.2x %.2x %.2x %.2x\\%.2x",\
			buf[0+17*q],  buf[1+17*q], buf[2+17*q],  buf[3+17*q],  buf[4+17*q],  buf[5+17*q],  buf[6+17*q], \
			buf[7+17*q],  buf[8+17*q], buf[9+17*q], buf[10+17*q], buf[11+17*q], buf[12+17*q], buf[13+17*q], \
			buf[14+17*q], buf[15+17*q], buf[16+17*q]); } while (0)
#endif
#define SPIBUF(buf) {}

#define ESDSEQ	3000
#define POWERONSEQ 320
#define ESDEXECUTE 25000
#define USBDISABLE_DELAY 20000
#define ABS_MT_TRACKING_ID_MAX 10

static bool aLive, fUsboff, fPalmRej;
static bool fTSLogFlag = false;
static short CalState;

char spiAnalyRx[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];

static const char NTRIGNAME[] = "Ntrig";
static short anpidx;
static int TSLog, count_isr_from_resume, cResetESD, gFingerMap[ABS_MT_TRACKING_ID_MAX];
static int event_google_enable = 1;
u32 g_lastcht = 0, g_missRep = 0;
static uint32_t ntg_fwver;

static DEFINE_MUTEX(ntg_spi_transfer_lock);

#ifdef COUNTER_DEBUG
unsigned char byte_muti_tp[MTM_REPORT_SIZE+1+4] = {0};
#else
unsigned char byte_muti_tp[MTM_REPORT_SIZE+1] = {0};
#endif

struct ntg_tsData {
	struct input_dev 	*input_dev;
	struct input_dev	*input_devP;
	struct spi_device 	*spiDev;
	struct workqueue_struct *ntg_wq;
	struct work_struct 	ntg_work;
	struct ntg_touch_event *touchEvnt;
	uint16_t  abs_x_min;
	uint16_t  abs_x_max;
	uint16_t  abs_y_min;
	uint16_t  abs_y_max;
	uint16_t  fwtwozero;
	uint8_t orientate;
	uint8_t abs_pressure_min;
	uint8_t abs_pressure_max;
	uint8_t abs_width_min;
	uint8_t abs_width_max;
	int spi_enable;
	int irq_gpio;
	int	(*power)(int on);
	bool esdFlag;
	wait_queue_head_t get_data_wq;
	wait_queue_head_t send_data_wq;
	int get_data;
	int send_lib;
	atomic_t data_ok_agent;
	atomic_t info_send_agent;
	atomic_t start_agent;
	struct early_suspend early_suspend;
	struct workqueue_struct *ntg_wq_esd;
	struct delayed_work ntg_work_esd;
	struct workqueue_struct *ntg_wq_usb_disable;
	struct delayed_work ntg_work_usb_disable;
	struct workqueue_struct *ntg_wq_resume;
	struct delayed_work ntg_work_resume;
};
static struct ntg_tsData *private_ts;

struct ntg_AnalyData {
	unsigned short fn;
	unsigned short han;
	unsigned short van;
	unsigned short fnId;
	unsigned short phyfrq;
	unsigned char *map;
	struct ntg_AnaPen pen;
};
static struct ntg_AnalyData *mapData;

static int ntg_spi_transfer(struct ntg_tsData *, char *);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ntrig_ts_early_suspend(struct early_suspend *h);
static void ntrig_ts_late_resume(struct early_suspend *h);
#endif
extern int board_mfg_mode(void);
static char data_agent[DEBUG_ANGENT_ALL_REPORT_SIZE];
struct ntg_debug_agent_uart g_data_uart;
static DECLARE_WAIT_QUEUE_HEAD(data_ready_agent_wq);
static DECLARE_WAIT_QUEUE_HEAD(data_send_agent_wq);

static ssize_t spiDuplex_Lock(struct spi_device *dev, char *txbuf,
			      char *rxbuf, size_t len)
{
	struct spi_msg msg;
	int err = 0;
	if (!dev)
		return 0;
	msg.data = (u8 *)txbuf;

	if (TSLog & Dbg_L9) {
		pr_info("[ts]Txbuf\n");
		SPIBUF(txbuf);
	}
	err = spi_read_write_lock(dev, &msg, rxbuf, len, 2);
        if (err < 0)
          printk(KERN_ERR "%s: failed (error %d)\n", __func__, err);
	return err;
}

static int ntg_checksumCal(unsigned char *PacketData, u16 PackLen)
{
	int sum = 0, i;
	for (i = 0; i < PackLen; i++)
		sum += PacketData[i];

	sum = 0 - sum;
	return sum;
}

static void ntg_usb_powoff(struct ntg_tsData *ntg)
{
	short i;
	char spiUsbTx[NTRIG_PACKET_SIZE];

	for (i = 0; i < 4 && fUsboff == false; i++) {
		pr_info("[tp]usb_powoff (%d)\n", i+1);
		memset(spiUsbTx, 0xAA, NTRIG_PACKET_SIZE);
		memcpy(spiUsbTx, ntgFW_usbDis, sizeof(ntgFW_usbDis));
		ntg_spi_transfer(ntg, spiUsbTx);
		msleep(150);
	}
}

static void ntg_palmRejOff(struct ntg_tsData *ntg)
{
	short i;
	char spiPalmTx[NTRIG_PACKET_SIZE];

	memset(spiPalmTx, 0xAA, NTRIG_PACKET_SIZE);

	if ((uint16_t)ntg_fwver >= ntg->fwtwozero) {
		memcpy(spiPalmTx, ntgFW_palmrejoff2, sizeof(ntgFW_palmrejoff2));
	} else {
		memcpy(spiPalmTx, ntgFW_palmrejoff, sizeof(ntgFW_palmrejoff));
	}
	for (i = 0; i < 4 && fPalmRej == false; i++) {
		pr_info("[tp]palm_rejoff (%d)\n", i+1);
		ntg_spi_transfer(ntg, spiPalmTx);
		msleep(150);
	}
}

static void ntg_usb_disable_mechanism(struct work_struct *work)
{
	struct ntg_tsData *ntg;
	ntg = container_of(work, struct ntg_tsData, ntg_work_usb_disable.work);
	pr_info("[tp]%s: enter\n", __func__);
	ntg_usb_powoff(ntg);
	ntg_palmRejOff(ntg);
}

static void ntg_alive_mechanism(struct work_struct *work)
{
	short i;
	struct ntg_tsData *ntg;
	char spiTxEbuf[NTRIG_PACKET_SIZE];
	ntg = container_of(work, struct ntg_tsData, ntg_work_esd.work);
	memset(spiTxEbuf, 0xAA, NTRIG_PACKET_SIZE);
	for (i = 0; i < 3; i++) {
		memcpy(spiTxEbuf, &ntgFW_getfw, sizeof(ntgFW_getfw));
		ntg_spi_transfer(ntg, spiTxEbuf);

		msleep(50*(i+1));
		if (TSLog & (Dbg_L9 | Dbg_L2))
			pr_err("[E-ts(%d)] AlIVE=%d\n", i, aLive);

		if (aLive) {
			cResetESD = 0;
			if (!fTSLogFlag)
				TSLog = 0;
			break;
		}
	}

	if (!aLive) {
		cResetESD++;
		if ((!fTSLogFlag) && (cResetESD == 3))
                  TSLog = 0;

		if ((cResetESD == 13) && (!fTSLogFlag))
			TSLog = 0;

		fUsboff = false;
		fPalmRej = false;
		pr_err("[ts]ESD reset\n");
		disable_irq(ntg->spiDev->irq);
		ntg->power(0);
		msleep(15);
		ntg->power(1);
		msleep(350);
		gpio_set_value(ntg->spi_enable, 1);
		enable_irq(ntg->spiDev->irq);
		ntg_usb_powoff(ntg);
		ntg_palmRejOff(ntg);
	}
	aLive = false;
	queue_delayed_work(ntg->ntg_wq_esd, &ntg->ntg_work_esd, msecs_to_jiffies(ESDSEQ));
}

static bool analy_init(void)
{
	struct ntg_tsData *ts;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];
	ts = private_ts;

	pr_info("[ts]%s\n", __func__);
	if (mapData == NULL || (mapData->fn == 0))
		mapData = kzalloc(sizeof(struct ntg_AnalyData), GFP_KERNEL);
	else
		pr_info("[ts]Already allocate mapData!!\n");

	memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
	memset(spiRxbuf, 0x00, NTRIG_PACKET_SIZE);

	ntgFW_map[9] = 0x17;
	ntgFW_map[17] = 0x0F;
	ntgFW_map[21] = ANA_SEN_CONF;
	ntgFW_map[sizeof(ntgFW_map)-3] = ntg_checksumCal(&ntgFW_map[14], 14);
	memcpy(spiTxbuf, ntgFW_map, sizeof(ntgFW_map)-2);

	spiDuplex_Lock(ts->spiDev, spiTxbuf, spiRxbuf, NTRIG_PACKET_SIZE);

	msleep(500);

	if (mapData->map == NULL)
		mapData->map = kzalloc(mapData->han*mapData->van*sizeof(unsigned char *), GFP_KERNEL);

	pr_info("[ts_anaConf]Fn=%d, HAN=%d, VAN=%d\n", mapData->fn,  mapData->han, mapData->van);
	return true;
}

static bool analy_opfreq(void)
{
	int err;
	struct ntg_tsData *ts;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];
	ts = private_ts;

	memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
	memset(spiRxbuf, 0x00, NTRIG_PACKET_SIZE);

	if (mapData == NULL)
		mapData = kzalloc(sizeof(struct ntg_AnalyData), GFP_KERNEL);
	else if (mapData != NULL)
		pr_info("[ts_ana]Already allocat mapData!!\n");

	pr_info("[ts]%s\n", __func__);

	ntgFW_setting[21] = CMD_OPERATE_FREQ;
	ntgFW_setting[sizeof(ntgFW_setting)-1] = ntg_checksumCal(&ntgFW_setting[14], 14);
	memcpy(spiTxbuf, ntgFW_setting, sizeof(ntgFW_setting));

	mapData->fnId = 0;
	mapData->phyfrq = 0;

	err = spiDuplex_Lock(ts->spiDev , spiTxbuf, spiRxbuf,
			     sizeof(char)*NTRIG_PACKET_SIZE);
	msleep(500);
	pr_info("[ts_ana]FreqID=%d, Freq=%d\n", mapData->fnId, mapData->phyfrq*10);
	return true;
}

static bool analy_base(short map)
{
	short i;
	struct ntg_tsData *ts;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];
	ts = private_ts;

	if (mapData == NULL) {
		pr_info("[ts_ana]Execute cat anaConf First!!\n");
		return false;
	}
	anpidx = 0;
	memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
	memset(spiRxbuf, 0x00, NTRIG_PACKET_SIZE);
	ntgFW_map[9] = 0x1F;
	ntgFW_map[17] = 0x11;
	ntgFW_map[21] = ANA_GET_BASEMAP;

	switch (map) {
	case 1:
		ntgFW_map[28] = 1;
		memset((char *)mapData->map, mapData->han * mapData->van, 0);
		break;
	case 2:
		ntgFW_map[28] = 2;
		memset((char *)mapData->map, mapData->han * mapData->van, 0);
		break;
	case 3:
		ntgFW_map[28] = 3;
		memset((char *)mapData->map, mapData->han * mapData->van, 0);
		break;
	case 4:
		ntgFW_map[28] = 4;
		memset((char *)mapData->map, mapData->han * mapData->van, 0);
		break;
	}
	for (i = 1; i <= mapData->han; i++) {
		ntgFW_map[29] = i;
		ntgFW_map[sizeof(ntgFW_map)-1] = ntg_checksumCal(&ntgFW_map[14], 16);

		memcpy(spiTxbuf, ntgFW_map , sizeof(ntgFW_map));
		spiDuplex_Lock(ts->spiDev, spiTxbuf, spiRxbuf, NTRIG_PACKET_SIZE);
		msleep(30);
	}

	if (anpidx != mapData->han) {
		 pr_info("[ts_ana]Base map(%d) Reply channel Error!!\n", map);
		 return false;
	} else
		pr_info("[ts_ana]Base map(%d) OK\n", map);
	return true;
}

static bool analy_active(void)
{
	short i;

	struct ntg_tsData *ts;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];
	ts = private_ts;
	memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
	memset(spiRxbuf, 0x00, NTRIG_PACKET_SIZE);

	if (mapData == NULL) {
		pr_info("[ts_ana]Execute cat anaConf First!!\n");
		return false;
	}
	pr_info("[ts]%s\n", __func__);
	anpidx = 0;
	ntgFW_map[9] = 0x1E;
	ntgFW_map[17] = 0x10;
	ntgFW_map[21] = ANA_GET_ACTMAP;
	memset((char *)mapData->map, mapData->han * mapData->van, 0);

	for (i = 1; i <= mapData->han; i++) {
		ntgFW_map[28] = i;
		ntgFW_map[sizeof(ntgFW_map)-2] = ntg_checksumCal(&ntgFW_map[14], 15);

		memcpy(spiTxbuf, ntgFW_map , sizeof(ntgFW_map)-1);
		spiDuplex_Lock(ts->spiDev, spiTxbuf, spiRxbuf, NTRIG_PACKET_SIZE);
		msleep(30);
	}
	if (anpidx != mapData->han) {
		 pr_info("[ts_ana]Reply channel Error!!\n");
		 return false;
	} else
		pr_info("[ts_ana]Actmap OK\n");

	return true;
}

static bool analy_finger(void)
{
	short i;

	struct ntg_tsData *ts;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];
	ts = private_ts;
	memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
	memset(spiRxbuf, 0x00, NTRIG_PACKET_SIZE);

	if (mapData == NULL) {
		pr_info("[ts_ana]Execute cat anaConf First!!\n");
		return false;
	}
	pr_info("[ts]%s\n", __func__);
	anpidx = 0;
	ntgFW_map[9] = 0x1E;
	ntgFW_map[17] = 0x10;
	ntgFW_map[21] = ANA_GET_FINMAP;
	memset((char *)mapData->map, mapData->han * mapData->van, 0);

	for (i = 1; i <= mapData->han; i++) {
		ntgFW_map[28] = i;
		ntgFW_map[sizeof(ntgFW_map)-2] = ntg_checksumCal(&ntgFW_map[14], 15);

		memcpy(spiTxbuf, ntgFW_map , sizeof(ntgFW_map)-1);
		spiDuplex_Lock(ts->spiDev, spiTxbuf, spiRxbuf, NTRIG_PACKET_SIZE);
		msleep(30);
	}
	if (anpidx != mapData->han) {
		pr_info("[ts_ana]Reply channel Error!!\n");
		return false;
	} else
		pr_info("[ts_ana]Finger Map OK\n");
	return true;
}

static bool analy_pen(void)
{
	struct ntg_tsData *ts;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];
	ts = private_ts;

	memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
	memset(spiRxbuf, 0x00, NTRIG_PACKET_SIZE);

	if (mapData == NULL) {
		pr_info("[ts_ana]Execute cat anaConf First!!\n");
		return false;
	}
	pr_info("[ts]%s\n", __func__);

	ntgFW_map[9] = 0x1D;
	ntgFW_map[17] = 0x0F;
	ntgFW_map[21] = ANA_GET_PEN;
	ntgFW_map[sizeof(ntgFW_map)-3] = ntg_checksumCal(&ntgFW_map[14], 14);
	memcpy(spiTxbuf, ntgFW_map, sizeof(ntgFW_map)-2);

	spiDuplex_Lock(ts->spiDev, spiTxbuf, spiRxbuf, NTRIG_PACKET_SIZE);
	memset(&mapData->pen, 0, sizeof(struct ntg_AnaPen));
	msleep(500);

	pr_info("[ts_ana]PenState:%d,Th:%d,Mag:%d,Pressure:%d,X:%d,Y:%d,Btn:%d\n",
		mapData->pen.pState, mapData->pen.pTh, mapData->pen.mag,
		mapData->pen.pPress, mapData->pen.posX, mapData->pen.posY,
		mapData->pen.pBtn);
	return true;
}

static long ntrig_analy_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	char buf[100];
	int err = 0;
        long ret = 0;
	unsigned long size = 0;
	void __user *argp = (void __user *)arg;
	if (_IOC_TYPE(cmd) != ANALY_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > ANALY_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;
	memset(buf, 0, 100);

	switch (cmd) {
	case ANALY_INIT:
		if (analy_init()) {
			sprintf(buf, "Fn=%d, HAN=%d, VAN=%d\n",
				     mapData->fn,  mapData->han, mapData->van);
			size = copy_to_user(argp, &buf, sizeof(buf));
		}
		break;
	case ANALY_BASE1:
		if (analy_base(1))
			size = copy_to_user(argp, mapData->map, mapData->han*mapData->van);
		break;
	case ANALY_BASE2:
		if (analy_base(2))
			size = copy_to_user(argp, mapData->map, mapData->han*mapData->van);
		break;
	case ANALY_BASE3:
		if (analy_base(3))
			size = copy_to_user(argp, mapData->map, mapData->han*mapData->van);
		break;
	case ANALY_BASE4:
		if (analy_base(4))
			size = copy_to_user(argp, mapData->map, mapData->han*mapData->van);
		break;
	case ANALY_OPFREQ:
		if (analy_opfreq()) {
			sprintf(buf, "FreqID=%d, Freq=%d\n", mapData->fnId, mapData->phyfrq*10);
			size = copy_to_user(argp, &buf, sizeof(buf));
		}
		break;
	case ANALY_ACTIVE:
		if (analy_active())
			size = copy_to_user(argp, mapData->map, mapData->han*mapData->van);
		break;
	case ANALY_FINGER:
		if (analy_finger())
			size = copy_to_user(argp, mapData->map, mapData->han*mapData->van);
		break;
	case ANALY_PEN:
		if (analy_pen()) {
			sprintf(buf + ret, "PenState:%d,Th:%d,Mag:%d,Pressure:%d,X:%d,Y:%d,Btn:%d\n",
					   mapData->pen.pState, mapData->pen.pTh, mapData->pen.mag,
					   mapData->pen.pPress, mapData->pen.posX, mapData->pen.posY,
					   mapData->pen.pBtn);
			size = copy_to_user(argp, &buf, sizeof(buf));
		}
		break;
	}
	if (size)
		pr_info("[ts]analysis ioctl copy_to_user Error (%ld)", size);

	return 0;
}


static int ntrig_analy_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}
static int ntrig_analy_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations ntrig_analy_fops = {
	.owner = THIS_MODULE,
	.open = ntrig_analy_open,
	.release = ntrig_analy_release,
	.unlocked_ioctl = ntrig_analy_ioctl,
};

static struct miscdevice ntrig_map_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ntrig_analysis",
	.fops = &ntrig_analy_fops,
};

static ssize_t stop_esdwq(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct ntg_tsData *ts;

	ts = private_ts;
	if (!cancel_delayed_work_sync(&ts->ntg_work_esd)) {
		cancel_delayed_work(&ts->ntg_work_esd);
	}
	sprintf(buf, "Destory ESD delayworkqueue\n");
	ret = strlen(buf) + 1;
	return ret;
}
DEVICE_ATTR(stopesd, 0444, stop_esdwq, NULL);

static void ntg_powSeq_machanism(struct work_struct *work)
{
	struct ntg_tsData *ts;
	ts = container_of(work, struct ntg_tsData, ntg_work_resume.work);
	pr_info("[tp]%s\n", __func__);
	gpio_set_value(ts->spi_enable, 1);
	enable_irq(ts->spiDev->irq);
	ntg_usb_powoff(ts);
	ntg_palmRejOff(ts);
}

static ssize_t touch_vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	short err = 0;
	struct ntg_tsData *ts;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];

	if (ntg_fwver) {
		sprintf(buf, "%s_%#x\n", NTRIGNAME, ntg_fwver);
		ntg_fwver = 0x00;
		ret = strlen(buf) + 1;
	} else {
		ts = private_ts;
		pr_info("[ts]%s\n", __func__);
		memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
		memset(spiRxbuf, 0x00, NTRIG_PACKET_SIZE);

		memcpy(spiTxbuf, ntgFW_getfw, sizeof(ntgFW_getfw));

		err = spiDuplex_Lock(ts->spiDev, spiTxbuf, spiRxbuf,
					sizeof(char)*NTRIG_PACKET_SIZE);
		msleep(1500);
		sprintf(buf, "%s_%#x\n", NTRIGNAME, ntg_fwver);
		ret = strlen(buf) + 1;
	}
	return ret;
}
DEVICE_ATTR(vendor, 0444, touch_vendor_show, NULL);

static ssize_t ts_setting(struct device  *dev, struct device_attribute *attr, char *buf)
{
	int ret, err;
	struct ntg_tsData *ts;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];
	ts = private_ts;
	memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
	memset(spiRxbuf, 0x00, NTRIG_PACKET_SIZE);

	ntgFW_setting[21] = 0x08;
	ntgFW_setting[sizeof(ntgFW_setting)-1] = ntg_checksumCal(&ntgFW_setting[14], 14);
	memcpy(spiTxbuf, ntgFW_setting, sizeof(ntgFW_setting));

	err = spiDuplex_Lock(ts->spiDev , spiTxbuf, spiRxbuf,
				sizeof(char)*NTRIG_PACKET_SIZE);

	sprintf(buf, "0x00:DPSM OFF\n0x01:single\n0x02:multi\
		     \n\r0x03:Palm Rejection On\n0x04:Palm Rejection Off\
		     \n\r0x05:3D Active\n0x06:3D inactive\n0x20:Pen only\
		     \n\r0x30:NO pen, No touch\n");

	ret	= strlen(buf) + 1;
	return ret;
}

static ssize_t ts_switch(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	int ret = 0, mode;
	struct ntg_tsData *ts;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];
	ts = private_ts;

	mode  = simple_strtoul(buf, NULL, 10);

	memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
	memset(spiRxbuf, 0x00, NTRIG_PACKET_SIZE);

	switch (mode) {
	case 0x00:/*DPSM off*/
		ntgFW_mode[28] = 0x02;
		ntgFW_mode[29] = 0x00;
		break;
	case 0x01:/*single*/
		ntgFW_mode[28] = 0x01;
		ntgFW_mode[29] = 0x0A;
		break;
	case 0x02:/*multi*/
		ntgFW_mode[28] = 0x02;
		ntgFW_mode[29] = 0x0A;
		break;
	case 0x03:/*Plam Rejection On*/
		ntgFW_mode[28] = 0x02;
		ntgFW_mode[29] = 0x0A;
		ntgFW_mode[31] = 0x01;
		break;
	case 0x04:/*Plam Rejection Off*/
		ntgFW_mode[28] = 0x02;
		ntgFW_mode[29] = 0x0A;
		ntgFW_mode[31] = 0x00;
		break;
	case 0x05:/*3D Active*/
		ntgFW_mode[28] = 0x02;
		ntgFW_mode[29] = 0x0A;
		ntgFW_mode[32] = 0x01;
		break;
	case 0x06:/*3D inactive*/
		ntgFW_mode[28] = 0x02;
		ntgFW_mode[29] = 0x0A;
		ntgFW_mode[32] = 0x00;
		break;
	case 0x30:/*No pen ,NO touch*/
		ntgFW_mode[28] = 0x30;
		ntgFW_mode[29] = 0x0A;
		break;
	case 0x20:/*Pen only*/
		ntgFW_mode[28] = 0x20;
		ntgFW_mode[29] = 0x0A;
		break;
	}

	if ((uint16_t)ntg_fwver >= ts->fwtwozero) {
		ntgFW_mode[sizeof(ntgFW_mode)-1] = ntg_checksumCal(&ntgFW_mode[14], 21);
		memcpy(spiTxbuf, ntgFW_mode, sizeof(ntgFW_mode));

	} else {
		ntgFW_mode[9] = 0x1A;
		ntgFW_mode[17] = 0x14;
		ntgFW_mode[sizeof(ntgFW_mode)-3] = ntg_checksumCal(&ntgFW_mode[14], 19);
		memcpy(spiTxbuf, ntgFW_mode, (sizeof(ntgFW_mode)-2));
	}

	ret = spiDuplex_Lock(ts->spiDev , spiTxbuf, spiRxbuf,
				sizeof(char)*NTRIG_PACKET_SIZE);

	ret	= strlen(buf);
	return count;
}
DEVICE_ATTR(mode, 0644, ts_setting, ts_switch);

static ssize_t ts_algget(struct device  *dev, struct device_attribute *attr, char *buf)
{
	int ret, err;
	struct ntg_tsData *ts;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];
	ts = private_ts;
	memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
	memset(spiRxbuf, 0x00, NTRIG_PACKET_SIZE);

	ntgFW_setting[21] = 0x0F;
	ntgFW_setting[sizeof(ntgFW_setting)-1] = ntg_checksumCal(&ntgFW_setting[14], 14);
	memcpy(spiTxbuf, ntgFW_setting, sizeof(ntgFW_setting));

	err = spiDuplex_Lock(ts->spiDev , spiTxbuf, spiRxbuf,
				sizeof(char)*NTRIG_PACKET_SIZE);
	sprintf(buf, "[ts]Algo Control get\
			\n\r0x00:OTFC Disalbe\n\r0x01:OTFC Enable\
			\r0x10:FH auto\n\r0x11:FH mode 1\n\
			\r0x12:FH mode 2\n\r0x13:FH mode 3\n\
			\r0x14:FH mode 4\n");
	ret = strlen(buf);
	return ret;
}

static ssize_t ts_otfc(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int ret = 0, mode;
	struct ntg_tsData *ts;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];
	ts = private_ts;

/*        mode = simple_strtoul(buf, NULL, 10);*/
	mode = strict_strtoul(buf, 10, NULL);

	memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
	memset(spiRxbuf, 0x00, NTRIG_PACKET_SIZE);

	switch (mode) {
	case 0x01:/*OTFC Enable*/
		ntgFW_otfcmode[28] = 0x01;
		ntgFW_otfcmode[29] = 0x00;
		break;
	case 0x00:/*OTFC Disable*/
		ntgFW_otfcmode[28] = 0x00;
		ntgFW_otfcmode[29] = 0x00;
		break;
	case 0x10:/*FH auto*/
		ntgFW_otfcmode[28] = 0x01;
		ntgFW_otfcmode[29] = 0x00;
		break;
	case 0x11:/*FH 1*/
		ntgFW_otfcmode[28] = 0x01;
		ntgFW_otfcmode[29] = 0x01;
		break;
	case 0x12:/*FH 2*/
		ntgFW_otfcmode[28] = 0x01;
		ntgFW_otfcmode[29] = 0x02;
		break;
	case 0x13:/*FH 3*/
		ntgFW_otfcmode[28] = 0x01;
		ntgFW_otfcmode[29] = 0x03;
		break;
	case 0x14:/*FH 4*/
		ntgFW_otfcmode[28] = 0x01;
		ntgFW_otfcmode[29] = 0x04;
		break;
	}
	ntgFW_otfcmode[sizeof(ntgFW_otfcmode)-1] = ntg_checksumCal(&ntgFW_otfcmode[14], 24);
	memcpy(spiTxbuf, ntgFW_otfcmode, sizeof(ntgFW_otfcmode));

	ret = spiDuplex_Lock(ts->spiDev , spiTxbuf, spiRxbuf,
				sizeof(char)*NTRIG_PACKET_SIZE);
	ret = strlen(buf);
	return count;
}
DEVICE_ATTR(OTFC, 0644, ts_algget, ts_otfc);

static ssize_t ts_logen(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	int buglevel;
	buglevel = simple_strtoul(buf, NULL, 10);
        //	if (buglevel) {
        //		TSLog = buglevel;
        //		fTSLogFlag = true;
        //	} else {
		TSLog = 0x00;
		fTSLogFlag = false;
                //	}
	pr_info("[ts]log enable  = %.2x\n", TSLog);
	return count;
}

static ssize_t show_ts_logen(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = sprintf(buf, "log enable  = %d\n0x00:Flag for ESD log\n0x01:Rx after state machine\
			\n\r0x02:Rx Befor state machine\n0x04:For deamon partial log\
			\n\r0x08:For DA & x-y cooridate\n0x10:For all deamon log\n0x20:For ESD pulling log\
			\n\r0x40:For Deamon Raw data\n0x80:For Pen state\n0x100:For Tx buf\n", TSLog);
	ret += 1;
	return ret;
}
DEVICE_ATTR(debug_level, 0644, show_ts_logen, ts_logen);

static ssize_t set_event_google(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	int buglevel;
	buglevel = simple_strtoul(buf, NULL, 10);
	event_google_enable = buglevel;

	pr_info("[ts]event google enable = %d\n", event_google_enable);
	return count;
}

static ssize_t show_event_google(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "event google enable = %d\n", event_google_enable);
}
DEVICE_ATTR(event_google, 0664, show_event_google, set_event_google);

static ssize_t ntg_SetCal(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret, err;
	struct ntg_tsData *ts_data;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];
	ts_data = private_ts;

	if (ts_data->esdFlag)
		cancel_delayed_work(&ts_data->ntg_work_esd);
	CalState = NTG_CALSTART;
	memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);

	memcpy(spiTxbuf, &ntgFW_setCal, sizeof(ntgFW_setCal));
	memset(spiRxbuf, 0, sizeof(char)*NTRIG_PACKET_SIZE);

	pr_info("[ts-SetCal]SetCal test send\n");
	err = spiDuplex_Lock(ts_data->spiDev , spiTxbuf, spiRxbuf,
								sizeof(char)*NTRIG_PACKET_SIZE);
	memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
	mdelay(1500);
	while (!CalState) {
		memcpy(spiTxbuf, &ntgFW_getCal, sizeof(ntgFW_getCal));
		memset(spiRxbuf, 0, sizeof(char)*NTRIG_PACKET_SIZE);
		pr_info("[ts-GetCal]Calibrating!!\n");
		err = spiDuplex_Lock(ts_data->spiDev, spiTxbuf, spiRxbuf,
							NTRIG_PACKET_SIZE);
		mdelay(2500);
	}
	if (CalState == NTG_CALPASS)
		ret = sprintf(buf, "Calibration PASS!!\n");
	else if (CalState == NTG_CALDEFEAT)
		ret = sprintf(buf, "Calibration FAILURE!!\n");
	ret = strlen(buf)+1;

	if (ts_data->esdFlag)
		queue_delayed_work(ts_data->ntg_wq_esd, &ts_data->ntg_work_esd, msecs_to_jiffies(ESDSEQ));

	return ret;
}
DEVICE_ATTR(ts_SetCal, 0444, ntg_SetCal, NULL);

static ssize_t ts_gpio(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct ntg_tsData *ts;
	ts  = private_ts;

	ret  = gpio_get_value(ts->irq_gpio);

	printk(KERN_DEBUG "GPIO_TP_INT_N=%d\n", ret);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}
DEVICE_ATTR(gpio, 0444, ts_gpio, NULL);

static ssize_t ts_touch(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	int ret = 0, mode;
	struct ntg_tsData *ts;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];
	ts = private_ts;

	mode  = simple_strtoul(buf, NULL, 10);

	memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
	memset(spiRxbuf, 0x00, NTRIG_PACKET_SIZE);

	switch (mode) {
	case 0x01:/*Touch Enable*/
		ntgFW_ctltouch[16] = 0x0F;
		break;
	case 0x00:/*Touch Disable*/
		ntgFW_ctltouch[16] = 0x14;
		break;
	}
	memcpy(spiTxbuf, ntgFW_ctltouch, sizeof(ntgFW_ctltouch));

	ret = spiDuplex_Lock(ts->spiDev , spiTxbuf, spiRxbuf,
			     sizeof(char)*NTRIG_PACKET_SIZE);
	ret = strlen(buf);
	return count;
}
DEVICE_ATTR(touch, 0644, NULL, ts_touch);

static ssize_t opfreq(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (analy_opfreq())
		ret = sprintf(buf, "FreqID=%d, Freq=%d\n", mapData->fnId, mapData->phyfrq*10);
	else
		ret = sprintf(buf, "Fetch Frequency Data FAILURE\n");
	return ret;
}
DEVICE_ATTR(opfreq, 0444, opfreq, NULL);

static struct kobject *android_touch_kobj;

static int ntrig_touch_sysfs_init(void)
{
	int ret;
	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		pr_err("[tp_ntg_err]subsystem_register_failed\n");
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_ts_SetCal.attr);
	if (ret) {
		pr_err("[tp_ntg_err]sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		pr_err("[tp_ntg_err]sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		pr_err("[tp_ntg_err]sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_mode.attr);
	if (ret) {
		pr_err("[tp_ntg_err]sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_event_google.attr);
	if (ret) {
		pr_err("[tp_ntg_err]sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		pr_err("[tp_ntg_err]sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_stopesd.attr);
	if (ret) {
		pr_err("[tp_ntg_err]sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_OTFC.attr);
	if (ret) {
		pr_err("[tp_ntg_err]sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_touch.attr);
	if (ret) {
		pr_err("[tp_ntg_err]sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_opfreq.attr);
	if (ret) {
		pr_err("[tp_ntg_err]sysfs_create_file failed\n");
		return ret;
	}
	return 0;
}

static void ntrig_touch_sysfs_remove(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_ts_SetCal.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_mode.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_event_google.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_stopesd.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_OTFC.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_touch.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_opfreq.attr);

	kobject_del(android_touch_kobj);
}

static u32 CalcCheck_agentsum(unsigned char *PacketData, u16 PackLen)
{
	int i;
	u32  sum = 0;
	for (i = 0 ; i < PackLen; i++)
		sum += PacketData[i];
	return sum;
}

static int ntrig_debug_agent_reset(struct ntg_tsData *ntg)
{
	memset(&g_data_uart, 0x00, sizeof(struct ntg_debug_agent_uart));
	return 0;
}
static void ntrig_debug_agent_fragment(struct ntg_tsData *ntg)
{
	char preamble[] = {0xA5, 0x5A, 0xE7, 0x7E};
	u32 c_sum, i;

	memset(&data_agent[0], 0xff, 4);
	memcpy(&data_agent[4], &preamble[0], 4);
	data_agent[8] = g_data_uart.header.type;
	data_agent[9] = (g_data_uart.header.length & 0xff);
	data_agent[10] = (g_data_uart.header.length & 0xff00) >> 8;
	data_agent[11] = g_data_uart.header.flag;
	data_agent[12] = g_data_uart.header.channel;
	data_agent[13] = g_data_uart.header.function;
	memcpy(&data_agent[14], &g_data_uart.data, g_data_uart.header.length - 6);
	c_sum = CalcCheck_agentsum((unsigned char *)&data_agent[8], g_data_uart.header.length);
	data_agent[g_data_uart.header.length + 8] = (c_sum & 0xff);
	data_agent[g_data_uart.header.length + 9] = (c_sum & 0xff00) >> 8;
	data_agent[g_data_uart.header.length + 10] = (c_sum & 0xff0000) >> 16;
	data_agent[g_data_uart.header.length + 11] = (c_sum & 0xff000000) >> 24;
	if (TSLog & Dbg_L4) {
		pr_info("ntrig agent [from agent] c_sum = 0x%.4x, length 0x%.2x\n",
			c_sum, g_data_uart.header.length);

		pr_info("ntrig agent [from agent] 0x%.2x, 0x%.2x, 0x%.2x, 0x%.2x, 0x%.2x, 0x%.2x, 0x%.2x, 0x%.2x\n",
			data_agent[0], data_agent[1], data_agent[2], data_agent[3],
			data_agent[4], data_agent[5], data_agent[6], data_agent[7]);

		pr_info("ntrig agent [from agent] 0x%.2x, 0x%.2x, 0x%.2x, 0x%.2x, 0x%.2x, 0x%.2x\n",
			data_agent[8], data_agent[9], data_agent[10], data_agent[11],
			data_agent[12], data_agent[13]);

		for (i = 0; i < (g_data_uart.header.length - 6 + 4)/8; i++) {
			pr_info("ntrig agent %d[from agent] %.2x %.2x %.2x %.2x\\%.2x %.2x %.2x %.2x\n",
				i+1, data_agent[14+0+i*8], data_agent[14+1+i*8],
				data_agent[14+2+i*8], data_agent[14+3+i*8], data_agent[14+4+i*8], data_agent[14+5+i*8],
				data_agent[14+6+i*8], data_agent[14+7+i*8]);
		}
		for (i = 0; i < ((g_data_uart.header.length - 6 + 4)%8); i++) {
			pr_info("ntrig agent [from agent]the last %d byte => 0x%.2x,\n",
				14+1+i+8 + (((g_data_uart.header.length  - 6 + 4)/8) - 1)*8,
				data_agent[14+i+8+(((g_data_uart.header.length - 6 + 4)/8) - 1)*8]);
		}
	}
	atomic_set(&ntg->data_ok_agent, 1);
	wake_up_interruptible(&data_ready_agent_wq);
	if (TSLog & Dbg_L4)
		pr_info("ntrig [ts]%s: wait_event  data_send_agent_wq\n", __func__);
	wait_event_interruptible(data_send_agent_wq, atomic_read(&ntg->info_send_agent));
	atomic_set(&ntg->info_send_agent, 0);
	if (TSLog & Dbg_L4)
		pr_info("ntrig [ts]%s: wait_event  ok\n", __func__);
	memset(&data_agent, 0x00, DEBUG_ANGENT_ALL_REPORT_SIZE);
	ntrig_debug_agent_reset(ntg);
	/*atomic_set(&ntg->start_agent, 0);*/
}

static int ntrig_debug_agent_copy(struct ntg_tsData *ntg, char *buf, int index)
{
	int i;

	g_data_uart.header = ntg->touchEvnt->header;
	memcpy(&g_data_uart.data[index],
				&buf[10],
				ntg->touchEvnt->header.length - 6);
	if (TSLog & Dbg_L4) {
		pr_info("ntrig agent [spi]type 0x%x, length 0x%x, flag 0x%x, channel 0x%x, function 0x%x\n",
			g_data_uart.header.type, g_data_uart.header.length,
			g_data_uart.header.flag, g_data_uart.header.channel,
			g_data_uart.header.function);

		for (i = 0; i < ((ntg->touchEvnt->header.length - 6)/8); i++) {
			pr_info("ntrig agent [spi] %.2x %.2x %.2x %.2x\\%.2x %.2x %.2x %.2x\n",
				g_data_uart.data[index+0+i*8], g_data_uart.data[index+1+i*8],
				g_data_uart.data[index+2+i*8], g_data_uart.data[index+3+i*8],
				g_data_uart.data[index+4+i*8], g_data_uart.data[index+5+i*8],
				g_data_uart.data[index+6+i*8], g_data_uart.data[index+7+i*8]);
		}
		for (i = 0; i < ((ntg->touchEvnt->header.length - 6)%8); i++) {
			pr_info("ntrig agent [spi]the last %d byte => 0x%.2x,\n",
				1 + i + 8 + (((ntg->touchEvnt->header.length - 6)/8) - 1)*8,
				g_data_uart.data[index + 1 + i + 7 + (((ntg->touchEvnt->header.length - 6)/8) - 1)*8]);
		}
	}
	return 0;
}

static int ntrig_debug_agent_parser(struct ntg_tsData *ntg, char *buf)
{
	static int pre_flag, data_uart_length;
	struct ntg_touch_header  temp_header;

	if (atomic_read(&ntg->start_agent) == 0) {
		return 0;
	}

	temp_header = ntg->touchEvnt->header;
	if (temp_header.flag > 0xf || temp_header.flag < 0x0) {
		pr_info("ntrig [ts]%s: err flag 0x%x\n", __func__, temp_header.flag);
		ntrig_debug_agent_reset(ntg);
		pre_flag = 0;
		data_uart_length = 0;
		return -1;
	}

	if (temp_header.channel == 0x30) {
		if (temp_header.flag == 0 && pre_flag == 0) {/*only one packet*/
			ntrig_debug_agent_copy(ntg, buf, 0);
			ntrig_debug_agent_fragment(ntg);
			pre_flag = 0;
			data_uart_length = 0;
		} else {
			if (pre_flag == 0 || (temp_header.flag >= 0 &&/*check flag*/
			    (pre_flag == (temp_header.flag+1)) &&
			    (temp_header.function ==/*check function*/
			    g_data_uart.header.function))) {
				ntrig_debug_agent_copy(ntg, buf, data_uart_length);
				data_uart_length = data_uart_length + (temp_header.length - 6);
				pre_flag = temp_header.flag;

				if (temp_header.flag == 0) {
					g_data_uart.header.length = data_uart_length + 6; /*add type ~func*/
					ntrig_debug_agent_fragment(ntg);
					pre_flag = 0;
					data_uart_length = 0;
				}
			} else {
				pr_info("ntrig [ts]%s: err flag 0x%x, pre_flag 0x%x, function 0x%x, pre function 0x%x\n",
					__func__, temp_header.flag, pre_flag, temp_header.function ,
					g_data_uart.header.function);
				ntrig_debug_agent_reset(ntg);
				pre_flag = 0;
				data_uart_length = 0;
				return -2;
			}
		}
	}
	return 0;
}

static int ntg_parser_agent_usbcmd(struct ntg_tsData *ntg, char* usbRxbuf)
{
	short DataLen, nIdx_pre = 0;
	static int status, index_next;
	static char tempusbTx[NTRIG_PACKET_SIZE];
	int ret = 0, loop_i = 0;
	char usbAnalyTx[NTRIG_PACKET_SIZE], tempRxbuf[NTRIG_PACKET_SIZE];

	if (status == CHK5A) {
		for (loop_i = 0; loop_i < NTRIG_PACKET_SIZE; loop_i++) {
			if (usbRxbuf[loop_i] == 0xA5) {
				nIdx_pre = loop_i;
				if (TSLog & Dbg_L4)
					pr_info("[ts]CHK5A-for-loop, loop_i = %d\n", loop_i);
				break;
			}
			if (loop_i == NTRIG_PACKET_SIZE - 1) {
				if (TSLog & Dbg_L4)
					pr_info("[ts]CHK5A-for-loop,no pre, loop_i = %d\n", loop_i);
				return 0;
			}
		}
	}

	memcpy((char *)&DataLen, (usbRxbuf + nIdx_pre + 4 + 1), 2);
	memcpy(tempusbTx, (usbRxbuf + nIdx_pre), DataLen+4);

	memset(usbAnalyTx, 0xFF, sizeof(char)*NTRIG_PACKET_SIZE);
	memcpy(&usbAnalyTx[4], tempusbTx, DataLen+4);
	if (TSLog & Dbg_L4) {
		pr_info("[ntrig agent form usb]%s, data length 0x%x\n", __func__, DataLen);
		SPIBUF(usbAnalyTx);
	}
	memset(tempRxbuf, 0x00, sizeof(char)*NTRIG_PACKET_SIZE);
	atomic_set(&ntg->start_agent, 1);
	ntg_spi_transfer(ntg, usbAnalyTx);
	memset(tempusbTx, 0x00, sizeof(char)*NTRIG_PACKET_SIZE);
	status = CHK5A;
	index_next = 0;

	return ret;
}

static ssize_t ntrig_agent_read(struct file *file,
			  char __user *buf, size_t count, loff_t *pos)
{
	unsigned long size;
	struct ntg_tsData *ntg;
	size_t length_report;
	static size_t remain_report_len;
	int start_report;

	ntg  = private_ts;

	if (remain_report_len == 0) {
		wait_event_interruptible(data_ready_agent_wq, atomic_read(&ntg->data_ok_agent));

		if ((g_data_uart.header.length + 8 + 4) > DEBUG_ANGENT_REPORT_SIZE) {
			length_report = DEBUG_ANGENT_REPORT_SIZE;
			remain_report_len = (g_data_uart.header.length + 8 + 4) - DEBUG_ANGENT_REPORT_SIZE;
		} else {
			length_report = (g_data_uart.header.length + 8 + 4);
			remain_report_len = 0;
		}
		start_report = 0;
	} else {
		length_report = remain_report_len;
		start_report = DEBUG_ANGENT_REPORT_SIZE;
		remain_report_len = 0;
	}
	if (TSLog & Dbg_L4)
		pr_info(" %s start,report length %#x, all length %#x\n",
			__func__, length_report, (g_data_uart.header.length + 8 + 4));
	size = copy_to_user(buf, &data_agent[start_report], length_report);
	if (size) {
		pr_info(" %s copy_to_user err\n", __func__);
	}

	atomic_set(&ntg->data_ok_agent, 0);

	if (remain_report_len == 0) {
		atomic_set(&ntg->info_send_agent, 1);
		wake_up_interruptible(&data_send_agent_wq);
	}
	if (TSLog & Dbg_L4)
		pr_info(" %s end\n", __func__);

	return length_report;
}

static ssize_t ntrig_agent_write(struct file *file,
			   const char __user *buf, size_t count, loff_t *pos)
{
	char data_form_usb[NTRIG_PACKET_SIZE];
	int i;
	unsigned long size;
	struct ntg_tsData *ts_data;

	ts_data  = private_ts;

	if (count > NTRIG_PACKET_SIZE) {
		printk(KERN_ERR "%s: Err :command length %d > %d or < 5\n",
			__func__, count, NTRIG_PACKET_SIZE);
		return count;
	}

	memset(data_form_usb, 0xFF, sizeof(char)*NTRIG_PACKET_SIZE);
	if (TSLog & Dbg_L4)
		printk(KERN_INFO "%s: send agent command length, count %d\n", __func__, count);

	size = copy_from_user(data_form_usb, buf, count);
	if (TSLog & Dbg_L4)
		for (i = 0; i < NTRIG_PACKET_SIZE/8; i++) {
			pr_info("ntrig agent %d[cmd from usb] %.2x %.2x %.2x %.2x\\%.2x %.2x %.2x %.2x\n",
				i+1, data_form_usb[0+i*8], data_form_usb[1+i*8], data_form_usb[2+i*8],
				data_form_usb[3+i*8], data_form_usb[4+i*8], data_form_usb[5+i*8],
				data_form_usb[6+i*8], data_form_usb[7+i*8]);
		}
	ntg_parser_agent_usbcmd(ts_data, data_form_usb);

	return count;
}

static int ntrig_agent_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int ntrig_agent_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations ntrig_agent_fops = {
	.read		= ntrig_agent_read,
	.write		= ntrig_agent_write,
	.open		= ntrig_agent_open,
	.release	= ntrig_agent_release,
};

static struct miscdevice ntrigi_miscdev = {
	.name	= "ttyntrig-debug",
	.fops	= &ntrig_agent_fops,
	.minor	= MISC_DYNAMIC_MINOR,
};

static void ntg_orientate(short orien, unsigned short *posX, unsigned short *posY)
{
	u16 switchTmp = 0;
	struct ntg_tsData *ts_data;
	ts_data = private_ts;

	if (orien & 0x01) {
		switchTmp = *posX;
		*posX = *posY;
		*posY = switchTmp;
	}
	if (orien & 0x02) {
		(*posX) = ts_data->abs_x_max - (*posX);
	}
	if (orien & 0x04) {
		(*posY) = ts_data->abs_y_max - (*posY);
	}
}

static void compatible_tp_report(struct input_dev *idev, void *event, int index, bool finger, bool ontip)
{
	if (!ontip) {
		input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0x00);
                input_report_abs(idev, ABS_MT_PRESSURE, 0x00);
	} else {
		if (finger) {
			NtrTrackedFinger *touch = (NtrTrackedFinger *)event;
                        input_report_abs(idev, ABS_MT_TRACKING_ID, index);
			input_report_abs(idev, ABS_MT_PRESSURE, 0xFF);
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0x28);
			input_report_abs(idev, ABS_MT_WIDTH_MAJOR,
					 (touch->posDx + touch->posDy)/140);
			input_report_abs(idev, ABS_MT_POSITION_X, touch->posX);
			input_report_abs(idev, ABS_MT_POSITION_Y, touch->posY);
		} else {
			struct ntg_touch_Stouch *touch = (struct ntg_touch_Stouch *)event;
                        input_report_abs(idev, ABS_MT_TRACKING_ID, index);
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, touch->bit);
			input_report_abs(idev, ABS_MT_PRESSURE, touch->bit);
			input_report_abs(idev, ABS_MT_WIDTH_MAJOR, touch->posDx);
			input_report_abs(idev, ABS_MT_POSITION_X, touch->posX);
			input_report_abs(idev, ABS_MT_POSITION_Y, touch->posY);
		}
	}
	input_mt_sync(idev);
}

static int ntrig_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}
static int ntrig_release(struct inode *inode, struct file *file)
{
	return 0;
}

int get_finger_index(int id)
{
	static int first = 1;
	int i;

	if (first) {
		first = 0;
		for (i = 0; i < ABS_MT_TRACKING_ID_MAX; i++)
			gFingerMap[i] = -1;
	}

	/* search for existing finger */
	for (i = 0; i < ABS_MT_TRACKING_ID_MAX; i++) {
		if (gFingerMap[i] == id)
			return i;
	}
	/* search for place for new finger */
	for (i = 0; i < ABS_MT_TRACKING_ID_MAX; i++) {
		if (gFingerMap[i] < 0) {
			/* found */
			gFingerMap[i] = id;
			return i;
		}
	}
	/* new finger, and all places are in use (should not happen) */
	return -1;
}

void ntrig_send_multi_touch_to_android(struct input_dev *input,
		NtrTrackedFingersReport *multi_touch, short orien)
{
	int i = 0, index;
	int fingers_num = multi_touch->FingerCount;
	NtrTrackedFinger *fingers = multi_touch->Fingers;

	if (!input)
		return;

	for (i = 0; i < fingers_num; i++) {
		index = get_finger_index(fingers[i].TrackID);
		if (index < 0)
			continue;

		if (fingers[i].IsRemoved) {
                  {
			input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0x00);
                        input_report_abs(input, ABS_MT_PRESSURE, 0x00);
                  }
		} else {
			if (TSLog & Dbg_L5) {
				pr_info("NTRIG #finger%d,[Id:%d][X:Y][%#x,%#x][dX:dY][%d,%d][Palm:%d][Removed:%d]htc:dzdw=%#x, posi=%x\n",
					i+1, fingers[i].TrackID, fingers[i].posX, (int)fingers[i].posY,
					(int)fingers[i].posDx, fingers[i].posDy,
					(int)fingers[i].IsPalm, (int)fingers[i].IsRemoved,
					(0x28 << 16) | ((fingers[i].posDx + fingers[i].posDy)/60),
					(fingers[i].posX << 16 | fingers[i].posY));
			}
			ntg_orientate(orien, &fingers[i].posX, &fingers[i].posY);
			if (TSLog & Dbg_L4) {
				pr_info("[ts] Finger=%d, X=%d, Y=%d\n", i+1, fingers[i].posX, fingers[i].posY);
			}

                        compatible_tp_report(input, (void *)&fingers[i], index, true, true);
		}
	}
        input_sync(input);
        if (((1 == fingers_num) && (fingers[0].IsRemoved)) || (0 == fingers_num)) {
          input_report_abs(input, ABS_MT_PRESSURE, 0x00);
          input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0x00);
          input_sync(input);
          if (TSLog & Dbg_L4)
            pr_info("[ts]Finger Leave\n");
        }
	for (i = 0; i < fingers_num; i++) {
		if (0 != fingers[i].IsRemoved) {
			index = get_finger_index(fingers[i].TrackID);
			if (index >= 0)
				gFingerMap[index] = -1;
		}
	}
}

static long ntrig_ioctl(struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	long ret = 0;
	NtrTrackedFingersReport outReport;
	struct ntg_tsData *ntg;

	ntg  = private_ts;
	switch (cmd) {
	case NTRIG_IOCTL_READ_DATA:
		ret = wait_event_interruptible(ntg->get_data_wq, ntg->get_data);
		if (ret) {
			pr_err("[ts] get_data = %x\n", ntg->get_data);
			return -EFAULT;
		}
		ntg->get_data = 0;
		if (copy_to_user(argp, &byte_muti_tp, sizeof(byte_muti_tp))) {
			ntg->send_lib = 1;
			wake_up(&ntg->send_data_wq);
			pr_err("%s: copy_to_user error\n", __func__);
			return -EFAULT;
		}
		if (count_isr_from_resume == 2) {/*fisrt report FW ver then mt report after resume*/
			count_isr_from_resume++;
			pr_info("[ntrig MT] IOCTL_READ_DATA, after resume send data to lib: repID = 0x%x, Contact count(1 byte) = 0x%x",
				byte_muti_tp[0], byte_muti_tp[MTM_REPORT_SIZE-1]);
		}
		if (TSLog & Dbg_L3)
			pr_info("[ntrig MT] NTRIG_IOCTL_READ_DATA: repID = 0x%x, Contact count(1 byte) = 0x%x",
				byte_muti_tp[0], byte_muti_tp[MTM_REPORT_SIZE-1]);

		memset(byte_muti_tp, 0x00, MTM_REPORT_SIZE+1);
		ntg->send_lib = 1;
		wake_up(&ntg->send_data_wq);
		break;

	case NTRIG_IOCTL_SEND_DATA:
		if (copy_from_user(&outReport, argp, sizeof(outReport))) {
			pr_err("%s: copy_from_user error\n", __func__);
			return -EFAULT;
		}
		if (TSLog & Dbg_L3)
			pr_info("[NTRIG] FingerCount = %d\n", outReport.FingerCount);

		ntrig_send_multi_touch_to_android(ntg->input_dev, &outReport, ntg->orientate);
		break;
	default:
		break;
	}

	return 0;
}

static struct file_operations ntrig_fops = {
	.owner = THIS_MODULE,
	.open = ntrig_open,
	.release = ntrig_release,
	.unlocked_ioctl = ntrig_ioctl,
};

static struct miscdevice ntrig_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ntrig_touch",
	.fops = &ntrig_fops,
};

static void touchEventReport(struct ntg_tsData *ntg)
{
  if (ntg->touchEvnt->header.type == 0x01
      && ntg->touchEvnt->header.channel == 0x02) {
    compatible_tp_report(ntg->input_dev, &ntg->touchEvnt->stouch, 0x00,
                         true, (ntg->touchEvnt->stouch.bit & 0x02));
    input_sync(ntg->input_dev);
    
  } else if (ntg->touchEvnt->header.type == 0x01
             && ntg->touchEvnt->header.channel == 0x03) {
    if (TSLog & Dbg_L8)
      pr_info("[ts_Pen] %d, %d, bit = %d\n", ntg->touchEvnt->ptouch.posX,
              ntg->touchEvnt->ptouch.posY, ntg->touchEvnt->ptouch.bit);
    
    compatible_tp_report(ntg->input_devP, (struct ntg_touch_Stouch *)&ntg->touchEvnt->ptouch, 0x00, false,
                         (ntg->touchEvnt->ptouch.bit & 0x02) || (ntg->touchEvnt->ptouch.bit & 0x4) ||
                         (ntg->touchEvnt->ptouch.bit & 0x08) || (ntg->touchEvnt->ptouch.bit & 0x1));
    input_sync(ntg->input_devP);
  }
  
  memset(ntg->touchEvnt, 0x00, sizeof(struct ntg_touch_event));
}

static bool ntg_CheckSum(char *buf, struct ntg_tsData *ntg)
{
	short i;
	u16 sum = 0, oriSum, length = 0;

	if (atomic_read(&ntg->start_agent))
		return true;

	memcpy((void *)&length, (void *)&buf[1], 2);

	for (i = 0; i < length; i++)
		sum += buf[i];

	memcpy((void *)&oriSum, (void *)&buf[length+4], 2);

	if (sum == oriSum) {
		return true;
	} else {
		pr_info("[ts]c_sum = %.2x, o_Sum = %.2x", sum, oriSum);
		SPIBUF(buf);
		return false;
	}
}

static int ntrig_frame_parser(struct ntg_tsData *ntg, char *buf)
{
	bool fCheckSum = true;
	short anIdx = 0, nIndex = 0, region = 0;
	int i, contact_count, DTime = 0;
	struct ntg_touch_Mtouch *mtouch;
#ifdef COUNTER_DEBUG
	u32 cnt = 0, cnt_mt = 0;
#endif
	i = 0;
	if (buf[i] == 0xA5 && buf[i+1] == 0x5A &&
	    buf[i+2] == 0xE7 && buf[i+3] == 0x7E) {
		nIndex = i+4;
		if (TSLog & Dbg_L1) {
			pr_info("[ts-Rx]%s\n", __func__);
			SPIBUF(buf);
		}
		memcpy((void *)&ntg->touchEvnt->header, &buf[nIndex], sizeof(struct ntg_touch_header));

#ifdef COUNTER_DEBUG
		if (ntg->touchEvnt->header.channel == 0x01)	{
			memcpy(&cnt, &buf[104], 4);
			if (cnt != 0x00) {
				if ((g_lastcht+1) != cnt) {
					if (cnt != 0xFFFFFFFF)
						g_lastcht = cnt;

					g_missRep++;
					pr_info("ts cnt = %d] counter not match, miss cnt = 0x%.4x\n", cnt, g_missRep);
					SPIBUF(buf);
				}
				if (cnt % 100 == 0) {
					pr_info("[ts cnt = %d ] missRep = %.4x\n", cnt, g_missRep);
				}
			}
			g_lastcht = cnt;
		} else {
			memcpy(&cnt, &buf[104], 4);
			pr_info("[ts cnt = %d] counter , channel = 0x%.4x, type = 0x%.4x\n", cnt, ntg->touchEvnt->header.channel,
					 ntg->touchEvnt->header.type);
		}
#endif
		if (ntg->touchEvnt->header.channel == 0x30) {
			 ntrig_debug_agent_parser(ntg, buf);
		} else if (ntg->touchEvnt->header.type == 0x02 || ntg->touchEvnt->header.type == 0x01) {
			switch (ntg->touchEvnt->header.channel) {
			case 0x01:
#ifdef COUNTER_DEBUG
				memcpy(byte_muti_tp, &buf[nIndex+6], MTM_REPORT_SIZE+4);
				memcpy(&cnt_mt, &byte_muti_tp[MTM_REPORT_SIZE], 4);
				byte_muti_tp[MTM_REPORT_SIZE+4] = TSLog;
				pr_info("[ts_MT] cnt_mt %d for multi tp\n", cnt_mt);
				if (cnt_mt != cnt)
					pr_info("[ts_MT] miss cnt_mt %d for multi tp, cnt %d\n", cnt_mt, cnt);
#else
				memcpy(byte_muti_tp, &buf[nIndex+6], MTM_REPORT_SIZE);
				byte_muti_tp[MTM_REPORT_SIZE] = TSLog;/*for daemon to enable log*/
#endif
				fCheckSum = ntg_CheckSum(&buf[nIndex], ntg);
				if (fCheckSum) {
					contact_count = byte_muti_tp[MTM_REPORT_SIZE - 1];
					for (i = 0; i < contact_count; i++) {
						mtouch = (struct ntg_touch_Mtouch *)&(byte_muti_tp[3+sizeof(struct ntg_touch_Mtouch)*i]);
						if (mtouch->bit & 0x10) {
							mtouch->posX = 0;
							mtouch->posY = 0;
							mtouch->posDx = 250;
							mtouch->posDy = 150;
							mtouch->vdefine = 0x0D000B00;
							pr_info("[ts_inPalmRegion] bit = %d, i= %d\n", mtouch->bit, i+1);
						}
					}

					if (TSLog & Dbg_L5) {
						pr_info("[ts_MT]repID = %#x, Report Count= %#x, %#x\n",
							byte_muti_tp[0], byte_muti_tp[1], byte_muti_tp[2]);
						for (i = 0; i < 6; i++) {
							pr_info("[ts_MT]State= %#x,Fgr-idx= %#x,%#x;x= %#x,%#x;y=%#x,%#x;dx=%#x,%#x;dy=%#x,%#x,VDef= %#x,%#x,%#x,%#x\n",
								byte_muti_tp[3+(i*15)], byte_muti_tp[4+(i*15)], byte_muti_tp[5+(i*15)], byte_muti_tp[6+(i*15)],
								byte_muti_tp[7+(i*15)], byte_muti_tp[8+(i*15)], byte_muti_tp[9+(i*15)], byte_muti_tp[10+(i*15)],
								byte_muti_tp[11+(i*15)], byte_muti_tp[12+(i*15)], byte_muti_tp[13+(i*15)], byte_muti_tp[14+(i*15)],
								byte_muti_tp[15+(i*15)], byte_muti_tp[16+(i*15)], byte_muti_tp[17+(i*15)]);
						}
						pr_info("[ts_MT] contact cnt = %d\n", byte_muti_tp[93]);
						i = 0;
					}
					if (TSLog & Dbg_L7) {
						pr_info("[ts_DM]%#x,%#x,%#x\n", byte_muti_tp[0], byte_muti_tp[1], byte_muti_tp[2]);
						for (i = 0; i < 3; i++) {
							pr_info("[ts_DM]%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x\n",
								byte_muti_tp[3+(i*15)], byte_muti_tp[4+(i*15)], byte_muti_tp[5+(i*15)], byte_muti_tp[6+(i*15)],
								byte_muti_tp[7+(i*15)], byte_muti_tp[8+(i*15)], byte_muti_tp[9+(i*15)], byte_muti_tp[10+(i*15)],
								byte_muti_tp[11+(i*15)], byte_muti_tp[12+(i*15)], byte_muti_tp[13+(i*15)], byte_muti_tp[14+(i*15)],
								byte_muti_tp[15+(i*15)], byte_muti_tp[16+(i*15)], byte_muti_tp[17+(i*15)]);
						}
						pr_info("[ts_DM]%#x\n", byte_muti_tp[93]);
						i = 0;
					}
					if (count_isr_from_resume == 2)/*fisrt report FW ver then mt report after resume*/
						pr_info("[ts_MT] after resume mt report from FW:Contact count(1 byte) = %d\n", contact_count);
					if (TSLog & Dbg_L3)
						pr_info("[ts_MT] :Contact count(1 byte) = %d\n", contact_count);

					ntg->get_data = 1;
					wake_up(&ntg->get_data_wq);
					wait_event_interruptible_timeout(ntg->send_data_wq, ntg->send_lib, msecs_to_jiffies(5000));
					ntg->send_lib = 0;
				} else
					pr_err("[ts]Mulit-Touch Check sum Error\n");
				break;
			case 0x02:
				memcpy((void *)&ntg->touchEvnt->stouch, &buf[nIndex+6], sizeof(struct ntg_touch_Stouch));
				ntg_orientate(ntg->orientate, &ntg->touchEvnt->stouch.posX, &ntg->touchEvnt->stouch.posY);
				break;
			case 0x03:
				fCheckSum = ntg_CheckSum(&buf[nIndex], ntg);
				if (fCheckSum) {
					memcpy((void *)&ntg->touchEvnt->ptouch, &buf[nIndex+6], sizeof(struct ntg_touch_Ptouch));
					ntg_orientate(ntg->orientate, &ntg->touchEvnt->ptouch.posX, &ntg->touchEvnt->ptouch.posY);
				} else
					pr_err("[ts]Pen Check sum Error\n");
				break;
			case 0x10:
				ntg->touchEvnt->version.repId  = buf[nIndex+6];
				switch (ntg->touchEvnt->version.repId) {
				case 0x14:
					pr_info("[ts] touch Disable\n");
					break;
				case 0x0F:
					pr_info("[ts] touch Enable\n");
					break;
				case 0x0C:
					ntg->touchEvnt->version.reserved_0 = buf[nIndex + 7];
					memcpy(&ntg->touchEvnt->version.value, &buf[nIndex + 8], 4);
					memcpy(&ntg_fwver, &ntg->touchEvnt->version.value, 4);
					aLive = true;
					if (TSLog & Dbg_L6)
						pr_info("[ts]ESD=repID:%x,value:%04x\n", ntg->touchEvnt->version.repId, ntg->touchEvnt->version.value);
					break;
				case 0x0B:
					memcpy(&ntg->touchEvnt->version.value, &buf[nIndex + 7], 3);
					printk(KERN_INFO "[ts_parser](Start-Cal)repID:%x,value:%03x\n",
							ntg->touchEvnt->version.repId, (ntg->touchEvnt->version.value & NTG_MASK));
					break;
				case 0x11:
					memcpy(&ntg->touchEvnt->version.value, &buf[nIndex + 7], 3);
					printk(KERN_INFO "[ts_parser](GetCal-ing)repID:%x,value:%#x\n",
							ntg->touchEvnt->version.repId, (ntg->touchEvnt->version.value & NTG_MASK));
					if ((ntg->touchEvnt->version.value & NTG_MASK) == NTG_CAL_FAIL) {
						pr_err("[ts] Calibration FAILURE!!\n");
						CalState = NTG_CALDEFEAT;
					} else if ((ntg->touchEvnt->version.value & NTG_MASK) == NTG_CAL_STATE) {
						pr_info("[ts] Calibration PASS!!\n");
						CalState = NTG_CALPASS;
					}
					break;
				}
				break;
			case 0x20:
				switch (ntg->touchEvnt->header.function) {
				case 0x61:
					if ((buf[nIndex+6] == 0x7E && buf[nIndex+11] == 0x81) ||
					    (buf[nIndex+6] == 0x7E && buf[nIndex+11] == 0xC1)) {
						memcpy((void *)&ntg->touchEvnt->DfuStCmd.dfuHeader, &buf[nIndex+6], sizeof(ntg_CmdDfuStart));
						if (ntg->touchEvnt->DfuStCmd.dfuHeader.msgGroup == CMDGroup) {
							switch (ntg->touchEvnt->DfuStCmd.dfuHeader.msgCode) {
							case BOOTLOADER_VERSION:
								pr_info("[ts_BootLoader_Ver]RetCode=%.4lx; BL=%.2x%.2x%.2x%.2x\\%.2x%.2x%.2x%.2x\\%.2x\n", ntg->touchEvnt->DfuStCmd.dfuHeader.returnCode,
									buf[nIndex+26], buf[nIndex+27], buf[nIndex+28], buf[nIndex+29],
									buf[nIndex+30], buf[nIndex+31], buf[nIndex+32], buf[nIndex+33],
									buf[nIndex+34]);
								break;
							case CMD_MSG_CODE_TURNOFF_USB:
								ntg->touchEvnt->DfuStCmd.dfuHeader.returnCode == 0 ?
									pr_info("[ts_USB_OFF]OK!\n") : pr_info("[ts_USB]Still on\n");
								ntg->touchEvnt->DfuStCmd.dfuHeader.returnCode == 0 ?
									(fUsboff = true) : (fUsboff = false);
								break;
							case CMD_CONFIG_FETCH_SETTING:
								if (ntg->touchEvnt->DfuStCmd.dfuHeader.returnCode == 0) {
									memcpy((char *)&DTime, &buf[nIndex+21], 2);
									buf[nIndex+20] == 0x01 ? pr_info("[ts] Single Touch : DPSM time = %d\n", DTime) :
												 pr_info("[ts] Multi Touch : DPSM time = %d\n", DTime);
									buf[nIndex+23] == 0x01 ? pr_info("[ts] Palm Rej Sens. ON\n") :
												 pr_info("[ts] Palm Rej Sens. OFF\n");
									buf[nIndex+24] == 0x01 ? pr_info("[ts] 3D mode ON\n") :
												 pr_info("[ts] 3D mode OFF\n");
									if ((uint16_t)ntg_fwver >= ntg->fwtwozero) {
										memcpy((char *)&region, &buf[nIndex+25], 2);
										pr_info("[ts] Palm Region =%d\n", region);
									}
								} else
									pr_info("[ts] Fetch setting ERROR!!\n");
								break;
							case CMD_CONFIG_SETTING:
								ntg->touchEvnt->DfuStCmd.dfuHeader.returnCode == 0 ?
									pr_info("[tp_Setting]OK!\n") : pr_info("[tp_Setting]Failure!\n");
								ntg->touchEvnt->DfuStCmd.dfuHeader.returnCode == 0 ?
									(fPalmRej = true) : (fPalmRej = false);
								break;
							case CMD_CONFIG_ALGO_FETCH:
								if (ntg->touchEvnt->DfuStCmd.dfuHeader.returnCode == 0) {
									switch (buf[nIndex+20]) {
									case OTFCDisable:
										pr_info("[ts]OTFC = Disable\n");
										break;
									case OTFCEnable:
										pr_info("[ts]OTFC = Enable\n");
										break;
									case OTFCRevert:
										pr_info("[ts]OTFC = Revert\n");
										break;
									};
									if ((uint16_t)ntg_fwver >= ntg->fwtwozero) {
										switch (buf[nIndex+21]) {
										case FHauto:
											pr_info("[ts]FHmode = AUTO\n");
											break;
										case FHmanu:
											pr_info("[ts]FHmode = MANU\n");
											break;
										}
									}
								} else {
									pr_info("[ts] return code = %#lx\n", ntg->touchEvnt->DfuStCmd.dfuHeader.returnCode);
								}
								break;
							case CMD_CONFIG_ALGO_SETTING:
								ntg->touchEvnt->DfuStCmd.dfuHeader.returnCode == 0 ?
									pr_info("[ts_OTFC_set]OK!\n") : pr_info("[ts_OTFC_set]Failure!\n");
								break;
							case CMD_OPERATE_FREQ:
								ntg->touchEvnt->DfuStCmd.dfuHeader.returnCode == 0 ?
									pr_info("[ts_Freq]OK!\n") : pr_info("[ts_Freq]Failure!\n");
								mapData->fnId =  buf[nIndex+6+sizeof(ntg_CmdDfuHeader)+2];
								memcpy((char *)&mapData->phyfrq, &buf[nIndex+6+sizeof(ntg_CmdDfuHeader)+3], 2);
								break;
							}
						} else if (ntg->touchEvnt->DfuStCmd.dfuHeader.msgGroup == AnaGroup) {
							if (ntg->touchEvnt->DfuStCmd.dfuHeader.returnCode == 0) {
								switch (ntg->touchEvnt->DfuStCmd.dfuHeader.msgCode) {
								case ANA_GET_SN:
									break;
								case ANA_SEN_CONF:
									mapData->fn = (ntg->touchEvnt->DfuStCmd.startAddress & 0xFF);
									mapData->han = ((ntg->touchEvnt->DfuStCmd.startAddress & 0xFF00) >> 8);
									mapData->van = ((ntg->touchEvnt->DfuStCmd.startAddress & 0xFF0000) >> 16);
									pr_info("[ts_anaConf]Fn=%d, HAN=%d, VAN=%d\n",
										mapData->fn,  mapData->han, mapData->van);
									break;
								case ANA_GET_BASEMAP:
									mapData->fnId = buf[nIndex+6+sizeof(ntg_CmdDfuHeader)+2];
									memcpy((char *)&mapData->phyfrq, &buf[nIndex+6+sizeof(ntg_CmdDfuHeader)+3], 2);
									anIdx = buf[nIndex+6+sizeof(ntg_CmdDfuHeader)+5];
									memcpy((char *)(mapData->map+(anIdx-1)*mapData->van),
									       &buf[nIndex+6+sizeof(ntg_CmdDfuHeader)+6], mapData->van);
									anpidx++ ;
									pr_info("[ts_BaseMap(%d)]%02d\n", mapData->fnId, anIdx);
									break;
								case ANA_GET_ACTMAP:
									anIdx = buf[nIndex+6+sizeof(ntg_CmdDfuHeader)+2];
									memcpy((char *)(mapData->map+(anIdx-1)*mapData->van),
									       &buf[nIndex+6+sizeof(ntg_CmdDfuHeader)+3], mapData->van);
									anpidx++;
									pr_info("[ts_ActiveMap]%02d\n", anIdx);
									break;
								case ANA_GET_FINMAP:
									anIdx = buf[nIndex+6+sizeof(ntg_CmdDfuHeader)+2];
									memcpy((char *)(mapData->map+(anIdx-1)*mapData->van),
									       &buf[nIndex+6+sizeof(ntg_CmdDfuHeader)+3], mapData->van);
									anpidx++;
									pr_info("[ts_FingerMap]%02d\n", anIdx);
									break;
								case ANA_GET_PEN:
									memcpy((char *)&mapData->pen, &buf[nIndex+6+sizeof(ntg_CmdDfuHeader)+2],
										sizeof(struct ntg_AnaPen));
									pr_info("[ts_anaPen]\n");
									break;
								}
							} else {
								pr_info("[ts_ana] unexpected failure!!return code=%#4lx\n",
									ntg->touchEvnt->DfuStCmd.dfuHeader.returnCode);
							}
						}
					} else {
						printk(KERN_INFO "[ts_Dfu] protocol error,%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x\n",
								buf[nIndex+6], buf[nIndex+7], buf[nIndex+8], buf[nIndex+9],
								buf[nIndex+10], buf[nIndex+11], buf[nIndex+12], buf[nIndex+13]);
					}
					break;
				}
				break;
			}
		}
	} else {
		return -1;
	}
	memset(buf, 0x00, NTRIG_PACKET_SIZE);

	if (ntg->touchEvnt->header.channel == 0x02 || ntg->touchEvnt->header.channel == 0x03) {
		if (fCheckSum)
			touchEventReport(ntg);
	}
	else if (ntg->touchEvnt->version.repId != 0x0C)
		memset(ntg->touchEvnt, 0x00, sizeof(struct ntg_touch_event));

	return 0;
}

static int ntg_spi_transfer(struct ntg_tsData *ntg, char *spiTxbuf)
{
	short DataLen = 0;
	int ret = 0, status, loop_i = 0, subStatus, nIdx = 0;
	int lastCnt = 0, aIdx = 0;

	mutex_lock(&ntg_spi_transfer_lock);
	memset(spiRxbuf, 0x0, sizeof(char)*NTRIG_PACKET_SIZE);

	status = CHK5A, subStatus = st_START;

	while (status != FINISH) {
		nIdx = 0;
		spiDuplex_Lock(ntg->spiDev, spiTxbuf, spiRxbuf, NTRIG_PACKET_SIZE);

		if (TSLog & Dbg_L2) {
			pr_info("[ts-Rx]%s\n", __func__);
			SPIBUF(spiRxbuf);
		}
	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE);
check5A:
		if (status == CHK5A) {
			for (loop_i = nIdx; loop_i < NTRIG_PACKET_SIZE; loop_i++) {
				if (spiRxbuf[loop_i] == 0xA5) {
					status = CHKPMB;
					subStatus = st_START;
					nIdx = loop_i;
					break;
				}
			}
			if (loop_i == NTRIG_PACKET_SIZE) {
				nIdx = loop_i;
				status = FINISH;
			}
		}

		if (status == CHKPMB) {
			if (subStatus == st_START) {
				if (loop_i > (NTRIG_PACKET_SIZE - 4)) {
					lastCnt = (NTRIG_PACKET_SIZE - nIdx);
					memcpy(spiAnalyRx, (spiRxbuf + nIdx), lastCnt);
					aIdx += lastCnt;
					subStatus = st_CONTI;
				} else {
					memcpy(spiAnalyRx, (spiRxbuf + nIdx), 4);
					nIdx += 4;
					aIdx += 4;
					subStatus = st_FINISH;
				}
			} else if (subStatus == st_CONTI) {
				nIdx = (4-lastCnt);
				memcpy((spiAnalyRx + aIdx), spiRxbuf, nIdx);
				aIdx += nIdx;
				subStatus = st_FINISH;
			}
			if (subStatus == st_FINISH) {
				if (spiAnalyRx[0] == 0xA5 && spiAnalyRx[1] == 0x5A &&
					spiAnalyRx[2] == 0xE7 && spiAnalyRx[3] == 0x7E) {
					status = CHKLEN;
					subStatus = st_START;
				} else {
					status = CHK5A;
					goto check5A;
				}
			}
		}

		if (status == CHKLEN) {
			if (subStatus == st_START) {
				if (nIdx >= (NTRIG_PACKET_SIZE - 3)) {
					lastCnt = (NTRIG_PACKET_SIZE - nIdx);
					memcpy((spiAnalyRx + aIdx), (spiRxbuf + nIdx), lastCnt);
					aIdx += lastCnt;
					subStatus = st_CONTI;
				} else {
					memcpy((spiAnalyRx + aIdx), (spiRxbuf + nIdx), 3);
					nIdx += 3;
					aIdx += 3;
					subStatus = st_FINISH;
				}
			} else if (subStatus == st_CONTI) {
				nIdx = (3 - lastCnt);
				memcpy((spiAnalyRx + aIdx), spiRxbuf, nIdx);
				aIdx += nIdx;
				subStatus = st_FINISH;
			}
			if (subStatus == st_FINISH) {
				memcpy((char *)&DataLen, (spiAnalyRx + 5), 2);
				DataLen -= 3;
				if (atomic_read(&ntg->start_agent) == 0)
					DataLen += CK_CNT;
				status = CHKDATA;
				subStatus = st_START;
			}
		}

		if (status == CHKDATA) {
			if (subStatus == st_START) {
				if (DataLen > (NTRIG_PACKET_SIZE - nIdx)) {
					lastCnt = (NTRIG_PACKET_SIZE - nIdx);
					memcpy((spiAnalyRx + aIdx), (spiRxbuf + nIdx), lastCnt);
					DataLen = DataLen - lastCnt;
					aIdx += lastCnt;
					subStatus = st_CONTI;
				} else {
					memcpy((spiAnalyRx + aIdx), (spiRxbuf + nIdx), DataLen);
					nIdx += DataLen;
					aIdx += DataLen;
					subStatus = st_FINISH;
				}
			} else if (subStatus == st_CONTI) {
				if (DataLen > NTRIG_PACKET_SIZE) {
					memcpy((spiAnalyRx + aIdx), spiRxbuf , NTRIG_PACKET_SIZE);
					DataLen = DataLen - NTRIG_PACKET_SIZE;
					aIdx += NTRIG_PACKET_SIZE;
					subStatus = st_CONTI;
				} else {
					memcpy((spiAnalyRx + aIdx), (spiRxbuf + nIdx), DataLen);
					nIdx += DataLen;
					aIdx += DataLen;
					subStatus = st_FINISH;
				}
			}
			if (subStatus == st_FINISH) {
				status = FINISH;
				subStatus = st_START;
			}
		}

		if (status == FINISH) {
			ret = ntrig_frame_parser(ntg, spiAnalyRx);
			aIdx = 0;
			if ((NTRIG_PACKET_SIZE-nIdx-1) > 0) {
				status = CHK5A;
				goto check5A;
			}
		}
	}
	mutex_unlock(&ntg_spi_transfer_lock);
	return ret;
}

static void ntrig_ts_work(struct work_struct *work)
{
	int ret = 0;
	struct ntg_tsData *ntg_ts;
	char spiTxbuf[NTRIG_PACKET_SIZE];

	if (TSLog & Dbg_L6)
		pr_info("[ts]%s", __func__);

	ntg_ts = container_of(work, struct ntg_tsData, ntg_work);
	memset(spiTxbuf, 0xAA, sizeof(char)*NTRIG_PACKET_SIZE);
	ret = ntg_spi_transfer(ntg_ts, spiTxbuf);

	enable_irq(ntg_ts->spiDev->irq);
}

static irqreturn_t ntrig_ts_irq_handler(int irq, void *dev_id)
{
	struct ntg_tsData *ts;
	ts = dev_id;

	if (count_isr_from_resume < 2) {
		count_isr_from_resume++;
	}
	disable_irq_nosync(ts->spiDev->irq);
	queue_work(ts->ntg_wq, &ts->ntg_work);

	return IRQ_HANDLED;
}

static int ntrig_ts_probe(struct spi_device *dev)
{
	int ret, err, intFlag, i;
	struct ntg_tsData *ntg_ts;
	struct ntrig_spi_platform_data *pdata;
	char spiTxbuf[NTRIG_PACKET_SIZE], spiRxbuf[NTRIG_PACKET_SIZE];

	if (board_mfg_mode() == 5) {
		pr_info("[tp] Recover mode charging!!Touch driver wouldn't probe\n");
		return 0;
	}
	pdata = dev->dev.platform_data;

	ret = 0;
	gpio_set_value(pdata->spi_enable, 0);
        udelay(20);
	gpio_set_value(pdata->spi_enable, 1);
	msleep(10);

	for (i = 0; i < 12; i++) {
		intFlag = gpio_get_value(pdata->irq_gpio);
		if (intFlag) {
			printk(KERN_DEBUG "[tp]touch(%d) irq HIGH\n", i);
			memset(spiTxbuf, 0xAA, NTRIG_PACKET_SIZE);
			memset(spiRxbuf, 0x00, NTRIG_PACKET_SIZE);
			spiDuplex_Lock(dev, spiTxbuf, spiRxbuf, NTRIG_PACKET_SIZE);
			msleep(50);
			if (spiRxbuf[8] == 0x02) {
				memcpy((char *)&ntg_fwver, (char *)&spiRxbuf[16], 4);
				pr_info("[tp_FwVersion]ver = %#x", ntg_fwver);
				intFlag = 0;
				break;
			}
		} else
			break;
	}
	if (intFlag) {
		pr_err("[tp_ntg_err][ts_interrupts] Always High!!\n");
		return 0;
	}

	ntg_ts = kzalloc(sizeof(struct ntg_tsData), GFP_KERNEL);
	if (ntg_ts == NULL) {
		pr_err("[tp_ntg_err]allocate ntg_tsdata failed \n");
		ret = -ENODEV;
		goto err_alloc_data_failed;
	}
	if (pdata)
		ntg_ts->power = pdata->power;

	ntg_ts->ntg_wq = create_singlethread_workqueue("ntrig_wq");
	if (!ntg_ts->ntg_wq) {
		pr_err("[tp_ntg_err]allocate ntg_tsdata failed \n");
		ret = -ENOMEM;
		goto err_create_wq_failed;
	}
	INIT_WORK(&ntg_ts->ntg_work, ntrig_ts_work);

	ntg_ts->spiDev = dev;
	if (!ntg_ts->spiDev)
		pr_err("[tp_ntg_err]dev pointer give failure!\n");

	dev_set_drvdata(&dev->dev, ntg_ts);
	ntg_ts->touchEvnt = kzalloc(sizeof(struct ntg_touch_event), GFP_KERNEL);
	if (ntg_ts->touchEvnt == NULL) {
		pr_err("[tp_ntg_err]allocate ntg_tsdata failed \n");
		ret = -ENODEV;
		goto err_create_wq_failed;
	}
	ntg_ts->abs_x_max = pdata->abs_x_max;
	ntg_ts->abs_x_min = pdata->abs_x_min;
	ntg_ts->abs_y_max = pdata->abs_y_max;
	ntg_ts->abs_y_min = pdata->abs_y_min;
	ntg_ts->orientate = pdata->orientate;
	ntg_ts->abs_pressure_max = pdata->abs_pressure_max;
	ntg_ts->abs_pressure_min = pdata->abs_pressure_min;
	ntg_ts->abs_width_max = pdata->abs_width_max;
	ntg_ts->abs_width_min = pdata->abs_width_min;
	ntg_ts->spi_enable = pdata->spi_enable;
	ntg_ts->esdFlag = pdata->esdFlag;
	ntg_ts->irq_gpio = pdata->irq_gpio;
	ntg_ts->fwtwozero = pdata->fwtwozero;

	ntg_ts->input_dev = input_allocate_device();
	if (ntg_ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&ntg_ts->spiDev->dev,
			"Failed to allocate input device\n");
		pr_err("[tp_ntg_err]input_allocate_device failed [Finger]\n");
		goto err_input_dev_alloc_failed;
	}
	ntg_ts->input_dev->name = "Ntrig-touchscreen";

	ntg_ts->input_devP = input_allocate_device();
	if (ntg_ts->input_devP == NULL) {
		ret = -ENOMEM;
		dev_err(&ntg_ts->spiDev->dev,
			"Failed to allocate input device\n");
		pr_err("[tp_ntg_err]input_allocate_device failed [Pen]\n");
		goto err_input_dev_alloc_failed;
	}
	ntg_ts->input_devP->name = "Ntrig-Pen-touchscreen";

	set_bit(EV_SYN, ntg_ts->input_dev->evbit);
	set_bit(EV_KEY, ntg_ts->input_dev->evbit);
        //	set_bit(BTN_TOUCH, ntg_ts->input_dev->keybit);
        //	set_bit(BTN_2, ntg_ts->input_dev->keybit);
	set_bit(EV_ABS, ntg_ts->input_dev->evbit);
	set_bit(KEY_BACK, ntg_ts->input_dev->keybit);
	input_set_abs_params(ntg_ts->input_dev, ABS_MT_POSITION_X,
			     ntg_ts->abs_x_min, ntg_ts->abs_x_max, 0, 0);
	input_set_abs_params(ntg_ts->input_dev, ABS_MT_POSITION_Y,
			     ntg_ts->abs_y_min, ntg_ts->abs_y_max, 0, 0);
        input_set_abs_params(ntg_ts->input_dev, ABS_MT_PRESSURE,
                             ntg_ts->abs_pressure_min, ntg_ts->abs_pressure_max, 0, 0);
	input_set_abs_params(ntg_ts->input_dev, ABS_MT_TOUCH_MAJOR,
			     ntg_ts->abs_pressure_min, ntg_ts->abs_pressure_max, 0, 0);
	input_set_abs_params(ntg_ts->input_dev, ABS_MT_WIDTH_MAJOR,
			     ntg_ts->abs_width_min, ntg_ts->abs_width_max, 0, 0);
        //	input_set_abs_params(ntg_ts->input_dev, ABS_MT_TRACKING_ID, 0,
        //			     ABS_MT_TRACKING_ID_MAX, 0, 0);

	set_bit(EV_SYN, ntg_ts->input_devP->evbit);
	set_bit(EV_KEY, ntg_ts->input_devP->evbit);
        //	set_bit(BTN_TOUCH, ntg_ts->input_devP->keybit);
	set_bit(BTN_2, ntg_ts->input_devP->keybit);
	set_bit(EV_ABS, ntg_ts->input_devP->evbit);
	set_bit(KEY_BACK, ntg_ts->input_devP->keybit);
	input_set_abs_params(ntg_ts->input_devP, ABS_MT_POSITION_X,
			     ntg_ts->abs_x_min, ntg_ts->abs_x_max, 0, 0);
	input_set_abs_params(ntg_ts->input_devP, ABS_MT_POSITION_Y,
			     ntg_ts->abs_y_min, ntg_ts->abs_y_max, 0, 0);
	input_set_abs_params(ntg_ts->input_devP, ABS_MT_TOUCH_MAJOR,
			     ntg_ts->abs_pressure_min, ntg_ts->abs_pressure_max, 0, 0);
	input_set_abs_params(ntg_ts->input_devP, ABS_MT_PRESSURE,
			     ntg_ts->abs_pressure_min, ntg_ts->abs_pressure_max, 0, 0);
	input_set_abs_params(ntg_ts->input_devP, ABS_MT_WIDTH_MAJOR,
			     ntg_ts->abs_width_min, ntg_ts->abs_width_max, 0, 0);

/*#ifndef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT*/
	input_set_abs_params(ntg_ts->input_dev, ABS_MT_AMPLITUDE,
			     0, ((255 << 16) | 30), 0, 0);
	input_set_abs_params(ntg_ts->input_dev, ABS_MT_POSITION,
			     0, ((1 << 31) | (ntg_ts->abs_x_max << 16) | ntg_ts->abs_y_max), 0, 0);

	input_set_abs_params(ntg_ts->input_devP, ABS_MT_AMPLITUDE,
			     0, ((255 << 16) | 32), 0, 0);
	input_set_abs_params(ntg_ts->input_devP, ABS_MT_POSITION,
			     0, ((ntg_ts->abs_x_max << 16) | ntg_ts->abs_y_max), 0, 0);
/*#endif*/

	ret = input_register_device(ntg_ts->input_dev);
	if (ret) {
		dev_err(&ntg_ts->spiDev->dev,
			"ntrig_ts_probe: Unable to register [Finger] %s input device\n",
			ntg_ts->input_dev->name);
		pr_err("[tp_ntg_err]input_register_device [Finger] failed\n");
		goto err_input_register_device_failed;
	}

	ret = input_register_device(ntg_ts->input_devP);
	if (ret) {
		dev_err(&ntg_ts->spiDev->dev,
			"ntrig_ts_probe: Unable to register [Pen] %s input device\n",
			ntg_ts->input_devP->name);
		pr_err("[tp_ntg_err]input_register_device [Pen] failed\n");
		goto err_input_register_device_failed;
	}

	init_waitqueue_head(&ntg_ts->get_data_wq);
	init_waitqueue_head(&ntg_ts->send_data_wq);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ntg_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 3;
	ntg_ts->early_suspend.suspend = ntrig_ts_early_suspend;
	ntg_ts->early_suspend.resume = ntrig_ts_late_resume;
	register_early_suspend(&ntg_ts->early_suspend);
#endif

	ntrig_touch_sysfs_init();

	init_waitqueue_head(&data_ready_agent_wq);
	init_waitqueue_head(&data_send_agent_wq);
	atomic_set(&ntg_ts->data_ok_agent, 0);
	atomic_set(&ntg_ts->info_send_agent, 0);
	atomic_set(&ntg_ts->start_agent, 0);
	if (misc_register(&ntrigi_miscdev) < 0) {
		pr_err("[tp_ntg_err]Can't register ntrigi_miscdev device with minor.");
		ret = -EIO;
		goto err_input_register_device_failed;
	}
	err = misc_register(&ntrig_device);
	if (err) {
		pr_err("[tp_ntg_err]ntrig_device: device register failed\n");
		ret = -EIO;
		goto err_input_register_device_failed;
	}
	err = misc_register(&ntrig_map_device);
	if (err) {
		pr_err("[tp_ntg_err]ntrig_map_device: device register failed\n");
		ret = -EIO;
		goto err_input_register_device_failed;
	}

	if (ntg_ts->esdFlag) {
		ntg_ts->ntg_wq_esd = create_singlethread_workqueue("ntrig_esd");
		if (!ntg_ts->ntg_wq_esd) {
			pr_err("[tp_ntg_err]allocate ntg_tsdata failed \n");
			ret = -ENOMEM;
			goto err_create_wq_failed;
		}
		INIT_DELAYED_WORK(&ntg_ts->ntg_work_esd, ntg_alive_mechanism);
		queue_delayed_work(ntg_ts->ntg_wq_esd, &ntg_ts->ntg_work_esd, msecs_to_jiffies(ESDEXECUTE));
	}

	ntg_ts->ntg_wq_usb_disable = create_singlethread_workqueue("ntrig_usb_disable");
		if (!ntg_ts->ntg_wq_usb_disable) {
			pr_err("[tp_ntg_err]allocate ntg_tsdata failed \n");
			ret = -ENOMEM;
			goto err_create_wq_failed;
		}
	INIT_DELAYED_WORK(&ntg_ts->ntg_work_usb_disable, ntg_usb_disable_mechanism);
	queue_delayed_work(ntg_ts->ntg_wq_usb_disable, &ntg_ts->ntg_work_usb_disable, msecs_to_jiffies(USBDISABLE_DELAY));

	ntg_ts->ntg_wq_resume = create_singlethread_workqueue("ntrig_resume");
	if (!ntg_ts->ntg_wq_resume) {
		pr_err("[tp_ntg_err]allocate ntg_tsdata failed \n");
		ret = -ENOMEM;
		goto err_create_wq_failed;
	}
	INIT_DELAYED_WORK(&ntg_ts->ntg_work_resume, ntg_powSeq_machanism);

	private_ts = ntg_ts;

	ret = request_irq(dev->irq, ntrig_ts_irq_handler, IRQF_TRIGGER_HIGH,
			  "ntrig_irq", ntg_ts);
	if (ret < 0) {
		dev_err(&ntg_ts->spiDev->dev, "request_irq_failed");
		pr_info("[tp_ntg_err]request_irq failed, %d\n", ret);
	}

	return 0;

err_input_register_device_failed:
	input_free_device(ntg_ts->input_dev);

err_input_dev_alloc_failed:
	destroy_workqueue(ntg_ts->ntg_wq);
	destroy_workqueue(ntg_ts->ntg_wq_esd);
err_create_wq_failed:
	kfree(ntg_ts);

err_alloc_data_failed:
	return ret;
}

static int __exit ntrig_ts_remove(struct spi_device *dev)
{
	struct ntg_tsData *ts;
	ts = private_ts;
	destroy_workqueue(ts->ntg_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	if (mapData != NULL) {
		if (mapData->map != NULL)
			kfree(mapData->map);
		kfree(mapData);
	}
	ntrig_touch_sysfs_remove();

	return 0;
}

static int ntrig_ts_suspend(struct spi_device *dev, pm_message_t mesg)
{
	int ret = 0;
	struct ntg_tsData *ts;
	ts = private_ts;

	disable_irq(ts->spiDev->irq);
	ret = cancel_work_sync(&ts->ntg_work);
	if (ret)
		enable_irq(ts->spiDev->irq);

	ts->power(0);
	count_isr_from_resume = 0;
	atomic_set(&ts->start_agent, 0);
	fUsboff = false;
	fPalmRej = false;
	if (ts->esdFlag) {
		ret = cancel_delayed_work_sync(&ts->ntg_work_esd);
		if (!ret)
			cancel_delayed_work(&ts->ntg_work_esd);
	}
	return ret;
}

static int ntrig_ts_resume(struct spi_device *dev)
{
	struct ntg_tsData *ts ;
	int ret = 0;
	ts = private_ts;

	ts->power(1);
	queue_delayed_work(ts->ntg_wq_resume, &ts->ntg_work_resume, msecs_to_jiffies(POWERONSEQ));

	if (ts->esdFlag)
		queue_delayed_work(ts->ntg_wq_esd, &ts->ntg_work_esd, msecs_to_jiffies(ESDSEQ));
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ntrig_ts_early_suspend(struct early_suspend *h)
{
	int ret;
	struct ntg_tsData *ts;
	ts = container_of(h, struct ntg_tsData, early_suspend);

	ret = cancel_delayed_work_sync(&ts->ntg_work_resume);
	if (ret)
		enable_irq(ts->spiDev->irq);

	if (event_google_enable == 1) {
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0x00);
		input_sync(ts->input_dev);
	} else {
		input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0x00);
		input_report_abs(ts->input_dev, ABS_MT_POSITION,  1<<31);
	}
	ntrig_ts_suspend(ts->spiDev, PMSG_SUSPEND);
}

static void ntrig_ts_late_resume(struct early_suspend *h)
{
	struct ntg_tsData *ts;
	ts = container_of(h, struct ntg_tsData, early_suspend);
	ntrig_ts_resume(ts->spiDev);
}
#endif

static const struct spi_device_id ntrig_ts_spi_id[] = {
	{NTRIG_NAME, 0},
	{}
};

static struct spi_driver ntrig_ts_driver = {
	.id_table = ntrig_ts_spi_id,
	.probe = ntrig_ts_probe,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = ntrig_ts_suspend,
	.resume = ntrig_ts_resume,
#endif
	.remove = __exit_p(ntrig_ts_remove),
	.driver = {
		.name = NTRIG_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ntrig_ts_init(void)
{
	int ret = 0;
	pr_info("[tp] %s(HW:%d)\n", __func__, system_rev);
	ret = spi_register_driver(&ntrig_ts_driver);
	if (ret < 0)
		pr_err("[tp] Ntrig touch driver Register FAILUE(%d)\n", ret);
	else
		pr_info("[tp] Ntrig touch driver Rigister OK!\n");

	return ret;
}

static void __exit ntrig_ts_exit(void)
{
	spi_unregister_driver(&ntrig_ts_driver);
}

module_init(ntrig_ts_init);
module_exit(ntrig_ts_exit);

MODULE_DESCRIPTION("N-trig SPI TOUCHSCREEN");
MODULE_LICENSE("GPL");

