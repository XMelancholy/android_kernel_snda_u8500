/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name		: ftk.c
* Authors		: Sensor & MicroActuators BU - Application Team
*			      : Hui HU  (hui.hu@st.com)
* Version		: V 2.0
* Date			: 13/12/2011
* Description	: Capacitive touch screen controller (FingertipK)
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
********************************************************************************
* REVISON HISTORY
*
*VERSION | DATE       | AUTHORS        | DESCRIPTION
* 1.0    | 29/06/2011 | Bela Somaiah   | First Release(link to ftk_patch.h)
* 2.0    | 13/12/2011 | Hui HU         | the default setting is cut1.3 for
*        |            |                | 480*800 phone
* 3.0    | 03/04/2012 | Hui HU         | load patch and config from userspaece
*******************************************************************************/
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/firmware.h>
#include <linux/earlysuspend.h>
#include <linux/input/st-ftk.h>
#include <linux/regulator/consumer.h>

/*
 * Definitions & global arrays.
 */
#define FTK_DRIVER_DESC		"FingerTipK MultiTouch I2C Driver"
#define FTK_DRIVER_NAME		"st-ftk"

#define WRITE_CHUNK_SIZE	64
#define P70_PATCH_ADDR_START	0x00420000

static struct workqueue_struct *ftk_touch_wq;
static int cor_xyz[10][3];
static unsigned char id_indexes[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

struct ftk_ts {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ftk_platform_data *pdata;
	struct hrtimer timer;
	struct work_struct work;
	int irq;
	struct regulator *regulator;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ftk_early_suspend(struct early_suspend *h);
static void ftk_late_resume(struct early_suspend *h);
#endif

static int ftk_write_reg(struct ftk_ts *ftk, u8 *reg, u16 num_com)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = ftk->client->addr;
	xfer_msg[0].len = num_com;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = reg;

	return i2c_transfer(ftk->client->adapter, xfer_msg, 1);
}

static int ftk_read_reg(struct ftk_ts *ftk, u8 *reg, int cnum, u8 *buf,
			int num)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = ftk->client->addr;
	xfer_msg[0].len = cnum;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = reg;

	xfer_msg[1].addr = ftk->client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags = I2C_M_RD;
	xfer_msg[1].buf = buf;

	return i2c_transfer(ftk->client->adapter, xfer_msg, 2);
}

static u8 load_config(struct ftk_ts *ftk, const u8 *data, size_t size)
{
	u16 one_group_length = 0;
	u16 patch_length = 0;
	u16 config_length = 0;
	u16 i = 0;
	u8 *config_data;
	u8 reg_add[8];
	u8 val[8];

	patch_length = (data[0] << 8) + data[1];
	config_length =
	    (data[patch_length + 2] << 8) +
	    data[patch_length + 3];
	config_data = (u8 *) &data[patch_length + 4];

	while (i < config_length) {
		one_group_length = (config_data[i] << 8) + config_data[i + 1];

		if ((config_data[i + 2] == 0xFF) &&
		    (config_data[i + 3] == 0xFF)) {
			usleep_range(1000*config_data[i + 4],
					1000*config_data[i + 4]);
		} else {
			ftk_write_reg(ftk, &(config_data[i + 2]),
				      one_group_length);
		}

		i = i + 2;
		i = i + one_group_length;
	}

	/* Clear ISR */
	reg_add[0] = 0xB0;
	reg_add[1] = 0x07;
	ftk_read_reg(ftk, &reg_add[0], 2, &val[0], 1);
	usleep_range(5000, 5000);

	/* Clear the buffer */
	reg_add[0] = 0x85;
	ftk_read_reg(ftk, &reg_add[0], 1, &val[0], 8);
	usleep_range(20000, 20000);

	/* Clear the buffer */
	reg_add[0] = 0x85;
	ftk_read_reg(ftk, &reg_add[0], 1, &val[0], 8);
	usleep_range(20000, 20000);

	/* Read version */
	reg_add[0] = 0xB0;
	reg_add[1] = 0x03;
	ftk_read_reg(ftk, reg_add, 2, val, 1);
	usleep_range(5000, 5000);

	/* Touch screen orientation: portrait mode */
	if (ftk->pdata->portrait) {
		dev_info(&ftk->client->dev, "set portrait mode\n");
		reg_add[0] = 0x8F;
		reg_add[1] = 0x20;
		ftk_write_reg(ftk, &reg_add[0], 2);
		usleep_range(5000, 5000);
	}

	/* TS Sense on */
	reg_add[0] = 0x83;
	ftk_write_reg(ftk, &reg_add[0], 1);
	usleep_range(5000, 5000);


	/* TS Sleep out */
	reg_add[0] = 0x81;
	ftk_write_reg(ftk, &reg_add[0], 1);
	usleep_range(5000, 5000);


	/* Enable interrupt */
	reg_add[0] = 0xB0;
	reg_add[1] = 0x06;
	reg_add[2] = 0x40;
	ftk_write_reg(ftk, &reg_add[0], 3);
	usleep_range(5000, 5000);

	dev_info(&ftk->client->dev, "config loaded version=%X\n", val[0]);
	return 0;

}

static u8 load_patch(struct ftk_ts *ftk, const u8 *data, size_t size)
{
	u32 write_addr, j = 0, i = 0;
	u16 patch_length = 0;
	u8 byte_work1[256 + 3] = { 0 };
	u8 reg_add[3] = { 0 };

	patch_length = (data[0] << 8) + data[1];

	while (j < patch_length) {
		write_addr = P70_PATCH_ADDR_START + j;

		reg_add[0] = 0xB3;
		reg_add[1] = (write_addr >> 24) & 0xFF;
		reg_add[2] = (write_addr >> 16) & 0xFF;
		ftk_write_reg(ftk, &reg_add[0], 3);

		byte_work1[0] = 0xB1;
		byte_work1[1] = (write_addr >> 8) & 0xFF;
		byte_work1[2] = write_addr & 0xFF;

		i = 0;
		while ((j < size) && (i < 256)) {
			byte_work1[i + 3] = data[j + 2];
			i++;
			j++;
		}
		ftk_write_reg(ftk, &byte_work1[0], 256 + 3);
	}

	dev_info(&ftk->client->dev, "patch loaded\n");

	return 0;
}

static int verify_firmware(struct ftk_ts *ftk, const u8 *data, size_t size)
{
	u16 firmware_length;
	u16 patch_length;
	u16 config_length;

	firmware_length = size;
	patch_length = (data[0] << 8) + data[1];
	config_length =
	    (data[patch_length + 2] << 8) +
	    data[patch_length + 3];

	if (firmware_length != patch_length + config_length + 4) {
		dev_info(&ftk->client->dev, "firmware file corrupted (%d: %d %d)\n",
			firmware_length, patch_length, config_length);
		return -EINVAL;
	}
	dev_info(&ftk->client->dev, "firmware file okay\n");
	return 0;
}

static int init_ftk(struct ftk_ts *ftk)
{
	const struct firmware *firmware;
	u8 reg_add[7];
	u8 val[8];
	int ret;

	/* TS Get Chip ID */
	reg_add[0] = 0xB0;
	reg_add[1] = 0x00;
	ret = ftk_read_reg(ftk, reg_add, 2, val, 3);
	if (ret < 0) {
		dev_err(&ftk->client->dev, "get chip id failed\n");
		return ret;
	}
	dev_info(&ftk->client->dev, "chip id = %x %x %x\n",
			val[0], val[1], val[2]);
	usleep_range(1000, 1000);


	/* TS Soft Reset */
	reg_add[0] = 0x9E;
	ret = ftk_write_reg(ftk, &reg_add[0], 1);
	if (ret < 0) {
		dev_err(&ftk->client->dev, "soft reset failed\n");
		return ret;
	}
	usleep_range(1000, 1000);

	/* Load firmware from file */
	dev_info(&ftk->client->dev, "loading firmware from file: %s\n",
		ftk->pdata->patch_file);
	ret = request_firmware(&firmware, ftk->pdata->patch_file,
			&ftk->client->dev);
	if (ret < 0) {
		dev_err(&ftk->client->dev,
				"request patch firmware failed (%d)\n",
				ret);
	} else {
		ret = verify_firmware(ftk, firmware->data,
			firmware->size);
		if (ret == 0)
			ret = load_patch(ftk, firmware->data,
				firmware->size);
		if (ret == 0)
			load_config(ftk, firmware->data,
				firmware->size);
	}
	release_firmware(firmware);

	if (ret < 0) {
		dev_err(&ftk->client->dev, "not initialised\n");
		return ret;
	}

	dev_info(&ftk->client->dev, "initialised\n");
	return 0;
}

static enum hrtimer_restart ftk_timer_func(struct hrtimer *timer)
{
	struct ftk_ts *ftk = container_of(timer, struct ftk_ts, timer);
	queue_work(ftk_touch_wq, &ftk->work);
	return HRTIMER_NORESTART;
}

static irqreturn_t ftk_interrupt(int irq, void *handle)
{
	struct ftk_ts *ftk = handle;
	disable_irq_nosync(ftk->client->irq);
	queue_work(ftk_touch_wq, &ftk->work);
	return IRQ_HANDLED;
}

static u8 ftk_decode_data_packet(struct ftk_ts *ftk,
					unsigned char data[],
					unsigned char left_event)
{
	u8 event_num = 0;
	u8 max_touches = 0;
	u8 touch_id, event_id;
	u8 last_left_event = 0;
	u8 reload_flag = 0;
	int i, num_touch;

	for (event_num = 0; event_num <= left_event; event_num++) {
		last_left_event = data[7 + event_num * 8] & 0x0F;
		max_touches = (data[1 + event_num * 8] & 0xF0) >> 4;
		touch_id = data[1 + event_num * 8] & 0x0F;
		event_id = data[event_num * 8] & 0xFF;
		if ((event_id == 0x03) ||
			(event_id == 0x04) ||
			(event_id == 0x05)) {
			id_indexes[touch_id] = 1;
			cor_xyz[touch_id][0] =
				((data[4 + event_num * 8] & 0xF0) >> 4) |
				((data[2 + event_num * 8]) << 4);
			cor_xyz[touch_id][1] =
				((data[4 + event_num * 8] & 0x0F) |
				((data[3 + event_num * 8]) << 4));
			cor_xyz[touch_id][2] = data[5 + event_num * 8];
		}
		if (event_id == 0x10) {
			for (i = 0; i < 10; i++)
				id_indexes[i] = 0;
			input_mt_sync(ftk->input_dev);
			input_sync(ftk->input_dev);
			init_ftk(ftk);
			return 0;
		}
	}

	num_touch = 0;
	for (i = 0; i < 10; i++) {
		if (id_indexes[i]) {
			num_touch++;
			input_report_abs(ftk->input_dev,
				ABS_MT_TRACKING_ID, i);
			input_report_abs(ftk->input_dev,
				ABS_MT_POSITION_X, cor_xyz[i][0]);
			input_report_abs(ftk->input_dev,
				ABS_MT_POSITION_Y, cor_xyz[i][1]);
			input_report_abs(ftk->input_dev,
				ABS_MT_TOUCH_MAJOR, cor_xyz[i][2]);
			input_report_key(ftk->input_dev,
				BTN_TOUCH, 1);
			input_mt_sync(ftk->input_dev);
		}
	}

	if (num_touch == 0) {
		input_report_key(ftk->input_dev, BTN_TOUCH, 0);
		input_mt_sync(ftk->input_dev);
	}

	input_sync(ftk->input_dev);
	reload_flag = 0;
	if (num_touch != 0) {
		for (event_num = 0; event_num <= left_event; event_num++) {
			last_left_event = data[7 + event_num * 8] & 0x0F;
			max_touches = (data[1 + event_num * 8] & 0xF0) >> 4;
			touch_id = data[1 + event_num * 8] & 0x0F;
			event_id = data[event_num * 8] & 0xFF;
			if (event_id == 0x04) {
				id_indexes[touch_id] = 0;
				reload_flag = 1;
			}
		}
		if (reload_flag == 1) {
			num_touch = 0;
			for (i = 0; i < 10; i++) {
				if (id_indexes[i]) {
					num_touch++;
					input_report_abs(ftk->input_dev,
						ABS_MT_TRACKING_ID, i);
					input_report_abs(ftk->input_dev,
						ABS_MT_POSITION_X,
						cor_xyz[i][0]);
					input_report_abs(ftk->input_dev,
						ABS_MT_POSITION_Y,
						cor_xyz[i][1]);
					input_report_abs(ftk->input_dev,
						ABS_MT_TOUCH_MAJOR,
						cor_xyz[i][2]);
					input_report_key(ftk->input_dev,
						BTN_TOUCH, 1);
					input_mt_sync(ftk->input_dev);
				}
			}
			if (num_touch == 0) {
				input_report_key(ftk->input_dev, BTN_TOUCH, 0);
				input_mt_sync(ftk->input_dev);
			}
			input_sync(ftk->input_dev);
		}
	}

	return last_left_event;
}

/* FTK Touch data reading and processing */
static void ftk_tasklet_proc(struct work_struct *work)
{
	struct ftk_ts *ftk = container_of(work, struct ftk_ts, work);

	unsigned char data[256+1];
	u8 temp_data[2];
	u8 status;
	u8 reg_add;
	u8 first_left_event = 0;
	u8 repeat_flag = 0;

	data[0] = 0xB0;
	data[1] = 0x07;
	ftk_read_reg(ftk, &data[0], 2, &status, 1);

	if (status & 0x40) {
		do {
			data[0] = 0xB1;
			data[1] = 0xFC;
			data[2] = 0x21;
			ftk_read_reg(ftk, &data[0], 3, &temp_data[0], 2);
			first_left_event = temp_data[1];

			if (first_left_event > 0) {
				memset(data, 0x0, 257);
				reg_add = 0x86;
				data[0] = 0;
				ftk_read_reg(ftk, &reg_add, 1, data,
					(first_left_event<<1));
				ftk_decode_data_packet(ftk, data,
					(first_left_event>>2));
			}

			data[0] = 0xB0;
			data[1] = 0x07;
			ftk_read_reg(ftk, &data[0], 2, &status, 1);

			if (status & 0x40)
				repeat_flag = 1;
			else
				repeat_flag = 0;

		} while (repeat_flag);
	}

	usleep_range(1000, 1000);

	if (!ftk->irq) {
		hrtimer_start(&ftk->timer, ktime_set(0, 10000000),
					  HRTIMER_MODE_REL);
	} else {
		enable_irq(ftk->client->irq);
	}
}

static int ftk_reset(int gpio)
{
	int ret = 0;
	/* touch reset pin is low at boot, take it high now */
	ret = gpio_request(gpio, "ftk reset");
	if (ret < 0)
		goto ftk_reset_out;
	ret = gpio_direction_output(gpio, 1);
	if (ret < 0)
		goto ftk_reset_out;
	usleep_range(1000, 1000);
	gpio_set_value(gpio, 0);
	usleep_range(9000, 9000);
	gpio_set_value(gpio, 1);
	usleep_range(5000, 5000);
	pr_info("st-ftk: reset ok\n");
ftk_reset_out:
	gpio_free(gpio);
	return ret;
}

static int ftk_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	struct ftk_ts *ftk = NULL;
	struct ftk_platform_data *pdata = NULL;
	int err = 0;

	dev_info(&client->dev, "probe\n");

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "probe err = EINVAL!\n");
		err = -EINVAL;
		goto fail;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "probe err = EIO!\n");
		err = -EIO;
		goto fail;
	}

	ftk = kzalloc(sizeof(struct ftk_ts), GFP_KERNEL);
	if (!ftk) {
		dev_err(&client->dev, "probe err = ENOMEM!\n");
		err = -ENOMEM;
		goto fail;
	}

	pdata = client->dev.platform_data;
	ftk->pdata = pdata;

	ftk->regulator = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(ftk->regulator)) {
		dev_warn(&client->dev, "failed to get regulator\n");
		ftk->regulator = NULL;
	}

	if (ftk->regulator) {
		err = regulator_enable(ftk->regulator);
		if (err) {
			dev_err(&client->dev, "failed to enable regulator\n");
			goto fail;
		}
	}

	INIT_WORK(&ftk->work, ftk_tasklet_proc);

	ftk->client = client;
	i2c_set_clientdata(client, ftk);

	if (gpio_is_valid(pdata->gpio_rst))
		if (ftk_reset(pdata->gpio_rst) < 0) {
			dev_err(&client->dev, "reset not performed!\n");
			err = -EIO;
			goto fail;
		}

	ftk->input_dev = input_allocate_device();
	ftk->input_dev->dev.parent = &client->dev;
	if (!ftk->input_dev) {
		dev_err(&client->dev, "err = ENOMEM!\n");
		err = -ENOMEM;
		goto fail;
	}
	ftk->input_dev->name = FTK_DRIVER_NAME;
	ftk->input_dev->phys = "ftk/input0";
	ftk->input_dev->id.bustype = BUS_I2C;
	ftk->input_dev->id.vendor = 0x0001;
	ftk->input_dev->id.product = 0x0002;
	ftk->input_dev->id.version = 0x0100;

	ftk->input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ftk->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	set_bit(EV_SYN, ftk->input_dev->evbit);
	set_bit(EV_KEY, ftk->input_dev->evbit);
	set_bit(BTN_TOUCH, ftk->input_dev->keybit);
	set_bit(BTN_2, ftk->input_dev->keybit);
	set_bit(EV_ABS, ftk->input_dev->evbit);

	input_set_abs_params(ftk->input_dev,
			ABS_X, pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(ftk->input_dev,
			ABS_Y, pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(ftk->input_dev,
			ABS_PRESSURE, pdata->p_min, pdata->p_max, 0, 0);
	input_set_abs_params(ftk->input_dev,
			ABS_MT_TRACKING_ID, 0, 10, 0, 0);
	input_set_abs_params(ftk->input_dev,
			ABS_MT_TOUCH_MAJOR, pdata->p_min, pdata->p_max, 0, 0);
	input_set_abs_params(ftk->input_dev,
			ABS_MT_WIDTH_MAJOR, pdata->p_min, pdata->p_max, 0, 0);
	input_set_abs_params(ftk->input_dev,
			ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(ftk->input_dev,
			ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);

	err = input_register_device(ftk->input_dev);
	if (err) {
		dev_err(&client->dev, "input_register_device fail!\n");
		goto fail;
	}

	err = init_ftk(ftk);
	if (err) {
		dev_err(&client->dev, "init_ftk fail!\n");
		goto fail;
	}

	ftk->irq = client->irq;

	if (!ftk->irq) {
		hrtimer_init(&ftk->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ftk->timer.function = ftk_timer_func;
		hrtimer_start(&ftk->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	} else {
		if (request_irq(ftk->irq, ftk_interrupt, IRQF_TRIGGER_FALLING,
						client->name, ftk)) {
			dev_err(&client->dev, "request_irq fail!\n");
			err = -EBUSY;
			goto fail;
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ftk->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ftk->early_suspend.suspend = ftk_early_suspend;
	ftk->early_suspend.resume = ftk_late_resume;
	register_early_suspend(&ftk->early_suspend);
#endif

	return 0;

fail:
	if (ftk) {
		if (ftk->regulator)
			regulator_disable(ftk->regulator);
		if (ftk->input_dev)
			input_free_device(ftk->input_dev);
		kfree(ftk);
	}
	dev_err(&client->dev, "probe err=%d\n", err);
	return err;
}

static int ftk_remove(struct i2c_client *client)
{
	struct ftk_ts *ftk = i2c_get_clientdata(client);
	int err;

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ftk->early_suspend);
#endif

	if (ftk->irq)
		free_irq(client->irq, ftk);
	else
		hrtimer_cancel(&ftk->timer);

	input_unregister_device(ftk->input_dev);

	if (ftk->regulator)
		if (regulator_disable(ftk->regulator))
			dev_err(&client->dev, "failed to disabel regulator\n");

	kfree(ftk);
	return 0;
}

static int ftk_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ftk_ts *ftk = i2c_get_clientdata(client);
	int ret, i;
	u8 reg_add[3];

	dev_info(&client->dev, "suspend\n");

	if (ftk->irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ftk->timer);

	ret = cancel_work_sync(&ftk->work);
	if (ret)
		goto fail;

	/* TS Sleep in */
	reg_add[0] = 0x80;
	ret = ftk_write_reg(ftk, &reg_add[0], 1);
	if (ret)
		goto fail;
	usleep_range(5000, 5000);

	/* flush buffer */
	reg_add[0] = 0x88;
	ret = ftk_write_reg(ftk, &reg_add[0], 1);
	if (ret)
		goto fail;
	usleep_range(5000, 5000);

	/* Disable interrupt */
	reg_add[0] = 0xB0;
	reg_add[1] = 0x06;
	reg_add[2] = 0x00;
	ret = ftk_write_reg(ftk, &reg_add[0], 3);
	if (ret)
		goto fail;
	usleep_range(5000, 5000);

	/* Clear the data buffer of x,y,z */
	for (i = 0; i < 10; i++)
		id_indexes[i] = 0;
	input_mt_sync(ftk->input_dev);
	input_sync(ftk->input_dev);


	if (ftk->regulator) {
		ret = regulator_disable(ftk->regulator);
		if (ret)
			goto fail;
	}

	dev_dbg(&client->dev, "suspend done\n");

	return 0;
fail:
	dev_err(&client->dev, "suspend failed (%d)\n", ret);

	return ret;
}

static int ftk_resume(struct i2c_client *client)
{
	struct ftk_ts *ftk = i2c_get_clientdata(client);
	int ret;
	u8 reg_add[3];

	dev_info(&client->dev, "resume\n");

	if (ftk->regulator) {
		ret = regulator_enable(ftk->regulator);
		if (ret) {
			dev_err(&client->dev, "failed to resume power for touch panel\n");
			return ret;
		}

		/*
		 * Touch panel need to reinit after suspend,
		 * since the power supply will be off in sleep
		 */
		if (gpio_is_valid(ftk->pdata->gpio_rst)) {
			if (ftk_reset(ftk->pdata->gpio_rst) < 0) {
				dev_err(&client->dev, "resume: reset not performed!\n");
				return -EIO;
			}
		}

		ret = init_ftk(ftk);
		if (ret) {
			dev_err(&client->dev, "resume:init_ftk fail!\n");
			return ret;
		}
	}

	/* TS Sleep out */
	reg_add[0] = 0x81;
	ret = ftk_write_reg(ftk, &reg_add[0], 1);
	usleep_range(5000, 5000);

	/* Enable interrupt */
	reg_add[0] = 0xB0;
	reg_add[1] = 0x06;
	reg_add[2] = 0x40;
	ret = ftk_write_reg(ftk, &reg_add[0], 3);
	usleep_range(5000, 5000);

	if (ftk->irq)
		enable_irq(client->irq);
	else
		hrtimer_start(&ftk->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	dev_dbg(&client->dev, "resume done\n");

	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ftk_early_suspend(struct early_suspend *h)
{
	struct ftk_ts *ftk = container_of(h, struct ftk_ts, early_suspend);
	ftk_suspend(ftk->client, PMSG_SUSPEND);
}

static void ftk_late_resume(struct early_suspend *h)
{
	struct ftk_ts *ftk = container_of(h, struct ftk_ts, early_suspend);
	ftk_resume(ftk->client);
}
#endif

static const struct i2c_device_id ftk_id[] = {
	{FTK_DRIVER_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ftk_id);

static struct i2c_driver ftk_driver = {
	.driver = {
		.name = FTK_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = ftk_probe,
	.remove = ftk_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = ftk_suspend,
	.resume = ftk_resume,
#endif
	.id_table = ftk_id,
};

static int __init ftk_init(void)
{
	ftk_touch_wq = create_singlethread_workqueue("ftk_touch_wq");
	if (!ftk_touch_wq)
		return -ENOMEM;

	return i2c_add_driver(&ftk_driver);
}

static void __exit ftk_exit(void)
{
	i2c_del_driver(&ftk_driver);
	if (ftk_touch_wq)
		destroy_workqueue(ftk_touch_wq);
}

module_init(ftk_init);
module_exit(ftk_exit);

MODULE_DESCRIPTION(FTK_DRIVER_DESC);
MODULE_AUTHOR("Hui HU");
MODULE_LICENSE("GPL");
