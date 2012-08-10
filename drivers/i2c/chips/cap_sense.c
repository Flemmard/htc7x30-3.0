/* drivers/i2c/chips/cap_sense.c
 *
 * Copyright (C) 2010 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <mach/msm_iomap.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/cap_sense.h>
#include <linux/irq.h>
#include <linux/wakelock.h>

#include "../../../arch/arm/mach-msm/proc_comm.h"

#define I2C_RETRY_COUNT 10
#define WAKEUP_DELAY 1*HZ

static struct workqueue_struct *capsense_wq;
static int capsense_thd;

enum mode {
	KEEP_AWAKE = 0,
	DEEP_SLEEP,
	/*AUTO_SLEEP,*/
};

struct capsense_data {
	struct early_suspend	early_suspend;
	struct class *capsense_class;
	struct device *capsense_dev;
	int intr_gpio;
	int intr_irq;
	struct work_struct work;
	struct delayed_work sleep_work;
	int sleep_mode;
	int radio_state;
	int pm_state;
	uint8_t is_actived;
	uint8_t polarity;
	spinlock_t spin_lock;
	unsigned long spinlock_flags;
	struct wake_lock sleep_wake_lock;
	struct wake_lock intr_wake_lock;
};

static struct capsense_data *cs_data;
static struct i2c_client *this_client;

static int I2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		 },
		{
		 .addr = this_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		SAR_LOGE("%s retry over %d\n",
				__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(char *txData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		SAR_LOGE("%s retry over %d\n",
				__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int csa_set_threshold(uint32_t data)
{
	uint8_t reg[2];
	int err = 0;

	reg[0] = THD_SAR_BTN_H;
	reg[1] = (data >> 8) & 0xFF;
	err = I2C_TxData(reg, 2);
	if (err) {
		SAR_LOGE("%s: I2C write fail. reg %d, err %d\n",
					__func__, THD_SAR_BTN_H, err);
		return err;
	}
	reg[0] = THD_SAR_BTN_L;
	reg[1] = data & 0xFF;
	err = I2C_TxData(reg, 2);
	if (err) {
		SAR_LOGE("%s: I2C write fail. reg %d, err %d\n",
					__func__, THD_SAR_BTN_L, err);
		return err;
	}

	capsense_thd = data;
	return err;
}

static int csa_get_threshold(uint32_t *data)
{
	uint8_t reg = 0;
	int thd = 0;
	int err = 0;

	reg = THD_SAR_BTN_H;
	I2C_RxData(&reg, sizeof(reg));
	if (err) {
		SAR_LOGE("%s: I2C read fail. reg %d, err %d\n",
					__func__, THD_SAR_BTN_H, err);
		return 0;
	}
	thd = reg;

	reg = THD_SAR_BTN_L;
	I2C_RxData(&reg, sizeof(reg));
	if (err) {
		SAR_LOGE("%s: I2C read fail. reg %d, err %d\n",
					__func__, THD_SAR_BTN_L, err);
		return 0;
	}
	*data = (thd << 8) + reg;
	SAR_LOGD("%s thresold = 0x%x\n", __func__, *data);

	return err;
}

#define CSA_SIGNATURE 0x534152 /*"SAR" in ASCII*/
static int csa_set_kadc(void)
{
	uint32_t thd = 0;
	int err = 0;

	SAR_LOGI("csa_kvalue = (0x%x, 0x%x, 0x%x)\n",
		csa_kvalue1, csa_kvalue2, csa_kvalue3);

	if ((csa_kvalue3 & 0xFFFFFF) != CSA_SIGNATURE)
		return -EINVAL;

	thd = csa_kvalue2 & 0xFFFF;
	err = csa_set_threshold(thd);

	return err;
}

static int csa_get_fw_ver(uint8_t *data)
{
	int err = 0;

	*data = FW_VERSION;
	err = I2C_RxData(data, 1);
	return err;
}

static void capsense_early_suspend(struct early_suspend *handler)
{
}

static void capsense_late_resume(struct early_suspend *handler)
{
}

static irqreturn_t cap_sense_irq_handler(int irq, void *data)
{
	struct capsense_data *cd = (struct capsense_data *)data;
	SAR_LOGD("%s in\n", __func__);

	disable_irq_nosync(irq);
	queue_work(capsense_wq, &cd->work);
	return IRQ_HANDLED;
}

static void capsense_work_func(struct work_struct *work)
{
	struct capsense_data *cs = container_of(
			work, struct capsense_data, work);
	int state = 0, active = 0;
	int err = 0;

	wake_lock(&cs->intr_wake_lock);
	state = gpio_get_value(cs->intr_gpio);
	active = (state ^ !cs->polarity) ? 3 : 0;
	SAR_LOGI("%s: active=%d, is_actived=%d\n",
				__func__, active, cs->is_actived);
	irq_set_irq_type(cs->intr_irq,
		state ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);

	if (active != cs->is_actived) {
		err = msm_proc_comm(PCOM_CUSTOMER_CMD1, &active, NULL);
		if (err)
			SAR_LOGE("%s: sent proc_comm fail, err=%d\n",
							__func__, err);
		else
			cs->is_actived = active;
	}

	enable_irq(cs->intr_irq);
	wake_unlock(&cs->intr_wake_lock);
}

static int csa_update_mode(int radio, int pm)
{
	SAR_LOGD("radio=%d, pm=%d\n", radio, pm);
	if (radio == KEEP_AWAKE && pm == KEEP_AWAKE)
		return KEEP_AWAKE;
	else
		return DEEP_SLEEP;
}

static void capsense_sleep_func(struct work_struct *work)
{
	struct capsense_data *cs = container_of(
			work, struct capsense_data, sleep_work.work);
	uint8_t data[2] = {CS_SLEEP_MODE, 0x0};
	int mode = 0;
	int err = 0;
	int active = 0;

	wake_lock(&cs->sleep_wake_lock);
	mode = csa_update_mode(cs->radio_state, cs->pm_state);
	if (mode == cs->sleep_mode) {
		SAR_LOGI("sleep mode no change.\n");
		wake_unlock(&cs->sleep_wake_lock);
		return;
	}

	switch (mode) {
	case KEEP_AWAKE:
		gpio_direction_output(cs->intr_gpio, 0);
		msleep(1);
		gpio_direction_input(cs->intr_gpio);
		queue_work(capsense_wq, &cs->work);
		break;

	case DEEP_SLEEP:
		disable_irq(cs->intr_irq);
		if (cancel_work_sync(&cs->work))
			enable_irq(cs->intr_irq);
		gpio_direction_output(cs->intr_gpio, 1);

		data[1] = 0x2;
		err = I2C_TxData(data, 2);
		if (err) {
			SAR_LOGE("%s: I2C write fail. reg %d, err %d\n",
						__func__, CS_SLEEP_MODE, err);
			wake_unlock(&cs->sleep_wake_lock);
			return;
		}
		err = msm_proc_comm(PCOM_CUSTOMER_CMD1, &active, NULL);
		if (err)
			SAR_LOGE("%s: sent proc_comm fail, err=%d\n",
							__func__, err);
		break;
/*
	case AUTO_SLEEP:data[1] = 0x2;
		data[1] = 0x1;
		err = I2C_TxData(data, 2);
		if (err) {
			SAR_LOGE("%s: I2C write fail. reg %d, err %d\n",
						__func__, CS_SLEEP_MODE, err);
			wake_unlock(&cs->sleep_wake_lock);
			return;
		}
		break;
*/
	}

	cs->sleep_mode = mode;
	SAR_LOGI("Set SAR sleep mode = %d\n", cs->sleep_mode);

	wake_unlock(&cs->sleep_wake_lock);
}

/* dev_attr_reg */
static ssize_t csa_reg_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	uint32_t data[2];
	uint8_t reg[2];
	int err = 0;

	sscanf(buf, "0x%x 0x%x", &data[0], &data[1]);
	SAR_LOGI("%s: reg=0x%x, value=0x%x\n", __func__, data[0], data[1]);
	reg[0] = (uint8_t)data[0];
	reg[1] = (uint8_t)data[1];
	err = I2C_TxData(reg, 2);
	if (err)
		SAR_LOGE("%s: I2C write fail. err %d\n",
						__func__, err);

	return count;
}
static ssize_t csa_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	char log[20*CS_REG_CNT];
	int i, p = 0;
	uint8_t data = 0;
	int err = 0;

	p = sprintf(log+p, "(");
	for (i = 0; i <= CS_REG_CNT ; i++) {
		data = i;
		err = I2C_RxData(&data, sizeof(data));
		if (err) {
			SAR_LOGE("%s: I2C read fail. reg %d, err %d\n",
							__func__, i, err);
			return 0;
		}
		p += sprintf(log+p, "0x%x", data);
		if (i == CS_REG_CNT)
			p += sprintf(log+p, ")");
		else
			p += sprintf(log+p, ",");
	}
	return sprintf(buf, "%s\n", log);
}
static DEVICE_ATTR(reg, 0664, csa_reg_show, csa_reg_store);

/* dev_attr_kadc */
static ssize_t csa_kadc_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	uint32_t data;
	int err = 0;

	sscanf(buf, "0x%x", &data);
	SAR_LOGI("%s: set kadc=0x%x\n", __func__, data);

	err = csa_set_threshold(data);
	if (err) {
		SAR_LOGE("Fail to set threshold\n");
		return 0;
	}
	return count;
}
static ssize_t csa_kadc_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint32_t thd = 0;
	int err = 0;

	err = csa_get_threshold(&thd);
	if (err) {
		SAR_LOGE("Fail to get threshold\n");
		return 0;
	}
	if (capsense_thd != thd)
		capsense_thd = thd;

	SAR_LOGI("%s: capsense_thd = 0x%x\n", __func__, capsense_thd);
	return sprintf(buf, "0x%x\n", capsense_thd);
}
static DEVICE_ATTR(kadc, 0664, csa_kadc_show, csa_kadc_store);

/* dev_attr_sleep */
static ssize_t csa_sleep_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	uint32_t data;
	sscanf(buf, "%d", &data);

	if (data != KEEP_AWAKE && data != DEEP_SLEEP)
		return count;

	cancel_delayed_work_sync(&cs_data->sleep_work);

	spin_lock_irqsave(&cs_data->spin_lock, cs_data->spinlock_flags);
	SAR_LOGI("%s: current mode = %d, new mode = %d\n",
				__func__, cs_data->sleep_mode, data);

	cs_data->radio_state = data;
	if (cs_data->radio_state == KEEP_AWAKE)
		queue_delayed_work(capsense_wq, &cs_data->sleep_work, WAKEUP_DELAY);
	else
		queue_delayed_work(capsense_wq, &cs_data->sleep_work, 0);
	spin_unlock_irqrestore(&cs_data->spin_lock, cs_data->spinlock_flags);
	return count;
}
static ssize_t csa_sleep_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	SAR_LOGD("%s: sleep_mode = %d\n", __func__, cs_data->sleep_mode);
	return sprintf(buf, "sleep_mode = %d\n", cs_data->sleep_mode);
}
static DEVICE_ATTR(sleep, 0664, csa_sleep_show, csa_sleep_store);

static int capsense_sensor_init(struct capsense_platform_data *pdata)
{
	uint8_t ver = 0;
	int err = 0;

	if (pdata->intr) {
		gpio_request(pdata->intr, "csa_wakeup");
		gpio_direction_output(pdata->intr, 1);
		msleep(1);
		gpio_direction_output(pdata->intr, 0);
		msleep(1);
		gpio_direction_input(pdata->intr);
		gpio_free(pdata->intr);
	}

	err = csa_get_fw_ver(&ver);
	if (err) {
		SAR_LOGE("check version fail, err %d\n", err);
		return err;
	} else
		SAR_LOGI("%s: firmware version = 0x%x\n", __func__, ver);

	if (csa_set_kadc() != 0)
		SAR_LOGI("%s: device has no proper kadc\n", __func__);

	return err;
}
static int capsense_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct capsense_data *cs;
	struct capsense_platform_data *pdata;
	int err = 0;

	SAR_LOGD("%s: in\n", __func__);
	pdata = client->dev.platform_data;
	if (!pdata) {
		SAR_LOGE("Assign platform_data error!!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SAR_LOGE("check i2c fail !!!\n");
		err = -ENODEV;
		goto check_functionality_failed;
	}

	cs = kzalloc(sizeof(struct capsense_data), GFP_KERNEL);
	if (!cs) {
		SAR_LOGE("kzalloc fail !!!\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, cs);
	this_client = client;

	err = capsense_sensor_init(pdata);
	if (err)
		goto check_version_failed;

	cs->sleep_mode = cs->radio_state = cs->pm_state = KEEP_AWAKE;
	cs->is_actived = 0;
	cs->polarity = pdata->active_high;

	wake_lock_init(&cs->sleep_wake_lock, WAKE_LOCK_SUSPEND, "capsense_sleep");
	wake_lock_init(&cs->intr_wake_lock, WAKE_LOCK_SUSPEND, "capsense_intr");
	spin_lock_init(&cs->spin_lock);

	capsense_wq = create_singlethread_workqueue("capsense_wq");
	INIT_WORK(&cs->work, capsense_work_func);
	INIT_DELAYED_WORK(&cs->sleep_work, capsense_sleep_func);

	if (pdata->gpio_init)
		pdata->gpio_init();
	if (pdata->intr) {
		err = gpio_request(pdata->intr, "csa_intr");
		if (err < 0) {
			SAR_LOGE("gpio_request(intr) fail,"
				"err %d\n", err);
			goto request_gpio_failed;
		}
		cs->intr_gpio = pdata->intr;
		cs->intr_irq = gpio_to_irq(cs->intr_gpio);
		set_irq_flags(cs->intr_irq, IRQF_VALID | IRQF_NOAUTOEN);
		err = request_irq(cs->intr_irq, cap_sense_irq_handler,
			cs->polarity ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW,
			"cap_sense", cs);
		if (err) {
			SAR_LOGE("request_irq failed for gpio %d,"
			" irq %d\n", cs->intr_gpio, cs->intr_irq);
			goto request_irq_failed;
		}
	}
	queue_work(capsense_wq, &cs->work);

#ifdef CONFIG_HAS_EARLYSUSPEND
	cs->early_suspend.suspend = capsense_early_suspend;
	cs->early_suspend.resume  = capsense_late_resume;
	register_early_suspend(&cs->early_suspend);
#endif

	cs->capsense_class = class_create(THIS_MODULE, "cap_sense");
	if (IS_ERR(cs->capsense_class)) {
		err = PTR_ERR(cs->capsense_class);
		cs->capsense_class = NULL;
		SAR_LOGE("%s:fail to create class, err=%d\n", __func__, err);
	}

	cs->capsense_dev = device_create(cs->capsense_class,
				NULL, 0, "%s", "sar");
	if (unlikely(IS_ERR(cs->capsense_dev))) {
		err = PTR_ERR(cs->capsense_dev);
		cs->capsense_dev = NULL;
		SAR_LOGE("%s:fail to create device, err=%d\n", __func__, err);
	}
	err = device_create_file(cs->capsense_dev, &dev_attr_reg);
	if (err)
		SAR_LOGE("%s:fail to create attr_reg, err=%d\n", __func__, err);
	err = device_create_file(cs->capsense_dev, &dev_attr_kadc);
	if (err)
		SAR_LOGE("%s:fail to create attr_kadc, err=%d\n", __func__, err);
	err = device_create_file(cs->capsense_dev, &dev_attr_sleep);
	if (err)
		SAR_LOGE("%s:fail to create attr_sleep, err=%d\n", __func__, err);

	cs_data = cs;
	return 0;

request_irq_failed:
	gpio_free(cs->intr_gpio);
request_gpio_failed:
	destroy_workqueue(capsense_wq);
	wake_lock_destroy(&cs->sleep_wake_lock);
	wake_lock_destroy(&cs->intr_wake_lock);
check_version_failed:
	kfree(cs);
check_functionality_failed:
	return err;
}

static int capsense_remove(struct i2c_client *client)
{
	struct capsense_data *cs = i2c_get_clientdata(client);

	device_remove_file(cs->capsense_dev, &dev_attr_reg);
	device_remove_file(cs->capsense_dev, &dev_attr_kadc);
	device_remove_file(cs->capsense_dev, &dev_attr_sleep);

	wake_lock_destroy(&cs->sleep_wake_lock);
	wake_lock_destroy(&cs->intr_wake_lock);
	kfree(cs);

	destroy_workqueue(capsense_wq);

	SAR_LOGI("%s:\n", __func__);
	return 0;
}

static int capsense_suspend(struct i2c_client *client, pm_message_t mesg)
{
	SAR_LOGI("capsense_suspend in\n");
	cancel_delayed_work_sync(&cs_data->sleep_work);

	cs_data->pm_state = DEEP_SLEEP;
	queue_delayed_work(capsense_wq, &cs_data->sleep_work, 0);
	return 0;
}

static int capsense_resume(struct i2c_client *client)
{
	SAR_LOGI("capsense_resume in\n");
	cancel_delayed_work_sync(&cs_data->sleep_work);

	cs_data->pm_state = KEEP_AWAKE;
	queue_delayed_work(capsense_wq, &cs_data->sleep_work, WAKEUP_DELAY*2);
	return 0;
}

static const struct i2c_device_id capsense_id[] = {
	{ CAPSENSE_NAME, 0 },
};

static struct i2c_driver capsense_driver = {
	.probe		= capsense_probe,
	.remove		= capsense_remove,
	.id_table	= capsense_id,
	.suspend	= capsense_suspend,
	.resume		= capsense_resume,
	.driver		= {
		.name = CAPSENSE_NAME,
	},
};

static int __init capsense_init(void)
{
	SAR_LOGI("CSA driver: init\n");
	return i2c_add_driver(&capsense_driver);
}

static void __exit capsense_exit(void)
{
	i2c_del_driver(&capsense_driver);
}

module_init(capsense_init);
module_exit(capsense_exit);

MODULE_DESCRIPTION("capsense driver");
MODULE_LICENSE("GPL");
