/*
 * Copyright (C) ST-Ericsson AB 2012
 *
 * Battery Management Deep debug support
 *
 * Note: Deep debug features are needed to perform the
 * HW validation of the platform
 *
 * License Terms: GNU General Public License v2
 * Author: Cedric Madianga <cedric.madianga@stericsson.com>
 */

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/kobject.h>
#include <linux/mfd/abx500/ab8500.h>
#include <linux/mfd/abx500.h>
#include <linux/mfd/abx500/ab8500-bm.h>
#include <linux/mfd/abx500/ab8500-gpadc.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/completion.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/ux500_deepdebug.h>
#include <linux/uaccess.h>
#include <linux/irq.h>

#include "ab8500_fg.h"

/* Exposure to the debugfs interface for test purpose only */

/**
 * ab8500_fg_test_algorithm_en() - enable or disable gas gauge test mode
 * @di:		pointer to the ab8500_fg structure
 * @enable:	enable/disable gas gaude test mode
 *
 * Return 0 or error code
 * Only used for test purpose
 */
static int ab8500_fg_test_algorithm_en(struct ab8500_fg *di, bool enable)
{
	int ret = 0;

	if (enable) {
		/* Set coulomb counter in test mode. */
		dev_dbg(di->dev, "Try to put gas gauge in test mode\n");
		cancel_delayed_work_sync(&di->fg_periodic_work);
		if (di->flags.fg_enabled) {
			ret = ab8500_fg_coulomb_counter(di, false);
			if (ret)
				return ret;
		}
		di->test.enable = true;
		dev_dbg(di->dev, "Gas gauge in test mode\n");
	} else {
		/* Set coulomb counter in normal mode. */
		dev_dbg(di->dev, "Try to put gas gauge in normal mode\n");
		if (di->flags.fg_enabled) {
			ret = ab8500_fg_coulomb_counter(di, false);
			if (ret)
				return ret;
		}

		di->init_capacity = true;
		ab8500_fg_charge_state_to(di, AB8500_FG_CHARGE_INIT);
		ab8500_fg_discharge_state_to(di, AB8500_FG_DISCHARGE_INIT);

		di->flags.batt_unknown = true;
		di->flags.batt_id_received = false;

		di->test.enable = false;
		ab8500_fg_coulomb_counter(di, true);

		di->flags.calibrate = true;
		di->calib_state = AB8500_FG_CALIB_INIT;
		dev_dbg(di->dev, "Gas gauge in normal mode\n");
	}

	return ret;
}

/**
 * ab8500_fg_is_test_is_algorithm_en() -
 * Return 1 if fg algorithm is enable 0 else
 * @di:		pointer to the ab8500_fg structure
 *
 * Only used for test purpose
 */
static bool ab8500_fg_test_is_algorithm_en(struct ab8500_fg *di)
{
	return di->test.enable;
}

/**
 * ab8500_fg_test_en() - enable coulomb counter
 * @di:		pointer to the ab8500_fg structure
 * @enable:	enable/disable
 *
 * Return 0 or error code on failure
 * Only used for test purpose
 */

static int ab8500_fg_test_en(struct ab8500_fg *di, bool enable)
{
	return ab8500_fg_coulomb_counter(di, enable);
}

/**
 * ab8500_fg_test_is_en() - Return 1 if fg is enabled 0 else
 * @di:		pointer to the ab8500_fg structure
 *
 *  Only used for test purpose
 */
static bool ab8500_fg_test_is_en(struct ab8500_fg *di)
{
	return di->flags.fg_enabled;
}

/**
 * ab8500_fg_test_set_cc_int_n_avg() - set number of conversion to average for
 * internal calibration
 * @di:		pointer to the ab8500_fg structure
 * @val:	number of conversion to average
 *
 * Return 0 or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_set_cc_int_n_avg(struct ab8500_fg *di, u8 val)
{
	int ret;
	u8 cc_int_n_avg = 0;

	switch (val) {
	case 4:
		cc_int_n_avg = CC_INT_CAL_SAMPLES_4;
		break;
	case 8:
		cc_int_n_avg = CC_INT_CAL_SAMPLES_8;
		break;
	case 16:
		cc_int_n_avg = CC_INT_CAL_SAMPLES_16;
		break;
	default:
		dev_err(di->dev,
				"incorrect sample values\n"
				"correct sample values should be 4, 8 or 16\n");
	}

	ret = abx500_mask_and_set_register_interruptible(di->dev,
			AB8500_GAS_GAUGE, AB8500_GASG_CC_CTRL_REG,
			CC_INT_CAL_N_AVG_MASK, cc_int_n_avg);
	if (ret < 0)
		dev_err(di->dev,
				"set number of conversion to average failed\n");

	return ret;
}

/**
 * ab8500_fg_test_get_cc_int_n_avg() - get number of conversion to average for
 * internal calibration
 * @di:		pointer to the ab8500_fg structure
 *
 * Return number of conversion to average or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_get_cc_int_n_avg(struct ab8500_fg *di)
{
	int ret;
	u8 val = 0;

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_CTRL_REG,  &val);
	if (ret < 0) {
		dev_err(di->dev,
			"get number of conversion to average failed\n");
		return ret;
	}

	switch (val & CC_INT_CAL_N_AVG_MASK) {
	case CC_INT_CAL_SAMPLES_4:
		ret = 4;
		break;
	case CC_INT_CAL_SAMPLES_8:
		ret = 8;
		break;
	case CC_INT_CAL_SAMPLES_16:
		ret = 16;
		break;
	case CC_INT_CAL_N_AVG_MASK:
		ret = 16;
		break;
	default:
		dev_err(di->dev,
			"incorrect val read in AB8500_GASG_CC_CTRL_REG");
		ret = -EINVAL;
		break;
	}

	return ret;
}

/**
 * ab8500_fg_test_int_calib() - launch internal calibration
 * @di:		pointer to the ab8500_fg structure
 *
 * Return result of calibration or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_int_calib(struct ab8500_fg *di)
{
	int ret;
	u8 val;

	mutex_lock(&di->test.lock);
	dev_dbg(di->dev, "Internal calibration ongoing...\n");

	ret = abx500_mask_and_set_register_interruptible(di->dev,
		AB8500_GAS_GAUGE, AB8500_GASG_CC_CTRL_REG,
		CC_INTAVGOFFSET_ENA, CC_INTAVGOFFSET_ENA);
	if (ret < 0) {
		dev_err(di->dev,
			"enabling offset average computation failed\n");
		goto err;
	}

	/* wait for completion of calibration */
	if (!wait_for_completion_timeout(&di->test.cc_int_calib_complete,
				5*HZ)) {
		dev_err(di->dev,
			"timeout: didn't receive CCIntCalib interrupt\n");
		ret = -EINVAL;
		goto err;
	}

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_CNTR_AVGOFF_REG,  &val);
	if (ret < 0)
		goto err;

	di->test.cc_int_offset = val;
	dev_dbg(di->dev, "Internal Calibration done...\n");
	mutex_unlock(&di->test.lock);

	return di->test.cc_int_offset;

err:
	mutex_unlock(&di->test.lock);
	dev_err(di->dev, "Internal calibration failure\n");
	return ret;
}

/**
 * ab8500_fg_test_soft_calib() - launch software calibration
 * @di:		pointer to the ab8500_fg structure
 *
 * Return result of calibration or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_soft_calib(struct ab8500_fg *di)
{
	int ret;
	u8 low_data, high_data;
	struct irq_desc *desc = irq_to_desc(di->irq);
	struct irq_data *data = irq_desc_get_irq_data(desc);

	if (irqd_irq_disabled(data)) {
		dev_dbg(di->dev, "irq is disabled --- enabling IRQ\n");
		enable_irq(di->irq);
	}

	mutex_lock(&di->test.lock);
	dev_dbg(di->dev, "Software calibration ongoing...\n");

	/* Set ADconverter in calibration mode */
	ret = abx500_mask_and_set_register_interruptible(di->dev,
		AB8500_GAS_GAUGE, AB8500_GASG_CC_CTRL_REG,
		CC_CALIB, CC_CALIB);
	if (ret < 0) {
		dev_err(di->dev,
			"set ADconverter in calibration mode failed\n");
		goto err;
	}

	/* wait for completion of calibration */
	if (!wait_for_completion_timeout(&di->test.cceoc_complete, 4*HZ)) {
		dev_err(di->dev,
			"timeout: didn't receive CCEOC interrupt\n");
		ret = -EINVAL;
		goto err;
	}

	/* Don't set ADConverter in calibration mode */
	ret = abx500_mask_and_set_register_interruptible(di->dev,
		AB8500_GAS_GAUGE, AB8500_GASG_CC_CTRL_REG,
		CC_CALIB, 0x00);
	if (ret < 0) {
		dev_err(di->dev, "stopping calibration mode failed\n");
		goto err;
	}

	/* Transfer sample and accumulator values */
	ret = abx500_mask_and_set_register_interruptible(di->dev,
		AB8500_GAS_GAUGE, AB8500_GASG_CC_CTRL_REG,
		READ_REQ, READ_REQ);
	if (ret < 0) {
		dev_err(di->dev, "transfer accumulator data failed\n");
		goto err;
	}

	/* Retrieve sample conversion */
	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_SMPL_CNVL_REG, &low_data);
	if (ret < 0) {
		dev_err(di->dev, "read low byte sample conversion failed\n");
		goto err;
	}

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_SMPL_CNVH_REG, &high_data);
	if (ret < 0) {
		dev_err(di->dev, "read high byte sample conversion failed\n");
		goto err;
	}

	di->test.cc_soft_offset = (high_data << 8) | low_data;
	dev_dbg(di->dev, "Software Calibration done...\n");
	mutex_unlock(&di->test.lock);

	if (!irqd_irq_disabled(data)) {
		dev_dbg(di->dev, "irq is ENABLED --- Disabling IRQ for CCEOC\n");
		disable_irq(di->irq);
	}
	return di->test.cc_soft_offset;

err:
	mutex_unlock(&di->test.lock);
	if (!irqd_irq_disabled(data)) {
		dev_dbg(di->dev, "irq is ENABLED --- Disabling IRQ\n");
		disable_irq(di->irq);
	}
	dev_err(di->dev, "Software calibration failure\n");
	return ret;
}

/**
 * ab8500_fg_test_set_cc_soft_offset() - set software offset into register
 * @di:		pointer to the ab8500_fg structure
 * @enable:	manual offset to be stored
 *
 * Return 0 or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_set_cc_soft_offset(struct ab8500_fg *di, u8 val)
{
	int ret;

	ret = abx500_set_register_interruptible(di->dev, AB8500_GAS_GAUGE,
			AB8500_GASG_CC_OFFSET_REG, val);
	if (ret < 0)
		dev_err(di->dev,
				"set software offset failed\n");
	else
		di->test.cc_soft_offset = val;
	return ret;
}

/**
 * ab8500_fg_test_get_cc_soft_offset() - get software offset into register
 * @di:		pointer to the ab8500_fg structure
 * @enable:	manual offset to be stored
 *
 * Return software offset or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_get_cc_soft_offset(struct ab8500_fg *di, u8 *val)
{
	int ret;

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
			AB8500_GASG_CC_OFFSET_REG,  val);
	if (ret < 0)
		dev_err(di->dev,
				"get software offset failed\n");
	else
		di->test.cc_soft_offset = *val;

	return ret;
}

/**
 * ab8500_fg_test_set_rst_accu_sample_counter() - set reset accumulator
 * sample counter bit
 * @di:		pointer to the ab8500_fg structure
 * @enable:	enable/disable reset acc
 *
 * Return 0 or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_set_rst_accu_sample_counter(struct ab8500_fg *di,
		bool enable)
{
	int ret;
	u8 val = 0;

	if (enable)
		val = RESET_ACCU;
	ret = abx500_mask_and_set_register_interruptible(di->dev,
			AB8500_GAS_GAUGE, AB8500_GASG_CC_CTRL_REG,
			RESET_ACCU, val);
	if (ret < 0)
		dev_err(di->dev,
			"set accumulator sample counter reset bit failed\n");


	return ret;
}

/**
 * ab8500_fg_test_get_rst_accu_sample_counter() - get reset accumulator
 * sample counter bit
 * @di:		pointer to the ab8500_fg structure
 *
 * Return reset accumulator sample counter bit or error code
 * Only used for test purpose
 */
static int ab8500_fg_test_get_rst_accu_sample_counter(struct ab8500_fg *di)
{
	u8 val = 0;
	int ret;

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_CTRL_REG,  &val);
	if (ret < 0) {
		dev_err(di->dev,
			"get accumulator sample counter reset bit failed\n");
		return ret;
	}

	if (val & RESET_ACCU)
		ret = 1;
	else
		ret = 0;
	return ret;
}

/**
 * ab8500_fg_test_set_cc_mux_offset() - set coumlomb counter offset
 * @di:		pointer to the ab8500_fg structure
 * @enable:	enable/disable offset
 *
 * Return 0 or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_set_cc_mux_offset(struct ab8500_fg *di, bool enable)
{
	int ret;
	u8 val = 0;

	if (enable)
		val = CC_MUXOFFSET;
	ret = abx500_mask_and_set_register_interruptible(di->dev,
		AB8500_GAS_GAUGE, AB8500_GASG_CC_CTRL_REG,
		CC_MUXOFFSET, val);
	if (ret < 0)
		dev_err(di->dev,
			"set mux offset failed\n");

	return ret;
}

/**
 * ab8500_fg_test_get_cc_mux_offset() - get coulomb counter mux offset
 * @di:		pointer to the ab8500_fg structure
 *
 * Get mux offset or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_get_cc_mux_offset(struct ab8500_fg *di)
{
	u8 val = 0;
	int ret;

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_CTRL_REG,  &val);
	if (ret < 0) {
		dev_err(di->dev,
			"get mux offset failed\n");
		return ret;
	}

	if (val & CC_MUXOFFSET)
		ret = 1;
	else
		ret = 0;
	return ret;
}

/**
 * ab8500_fg_test_read_sample() - read one sample
 * @di:		pointer to the ab8500_fg structure
 *
 * Return sample or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_read_sample(struct ab8500_fg *di)
{
	int ret;
	u8 low_data, high_data;
	struct irq_desc *desc = irq_to_desc(di->irq);
	struct irq_data *data = irq_desc_get_irq_data(desc);

	mutex_lock(&di->test.lock);
	dev_dbg(di->dev, "Sample reading ongoing...\n");

	if (irqd_irq_disabled(data)) {
		dev_dbg(di->dev, "irq is disabled --- enabling IRQ\n");
		enable_irq(di->irq);
	}

	/* wait for completion of calibration */
	if (!wait_for_completion_timeout(&di->test.cceoc_complete, 4*HZ)) {
		dev_err(di->dev,
			"timeout: didn't receive CCEOC interrupt\n");
		ret = -EINVAL;
		goto err;
	}

	/* Transfer sample and accumulator values */
	ret = abx500_mask_and_set_register_interruptible(di->dev,
		AB8500_GAS_GAUGE, AB8500_GASG_CC_CTRL_REG,
		READ_REQ, READ_REQ);
	if (ret < 0) {
		dev_err(di->dev, "transfer accumulator data failed\n");
		goto err;
	}

	/* Retrieve sample conversion */
	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_SMPL_CNVL_REG, &low_data);
	if (ret < 0) {
		dev_err(di->dev, "read low byte sample conversion failed\n");
		goto err;
	}

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_SMPL_CNVH_REG, &high_data);
	if (ret < 0) {
		dev_err(di->dev, "read high byte sample conversion failed\n");
		goto err;
	}

	di->test.cc_sample_conv = (high_data << 8) | low_data;

	dev_dbg(di->dev, "Sample reading done...\n");
	mutex_unlock(&di->test.lock);

	if (!irqd_irq_disabled(data)) {
		dev_dbg(di->dev, "irq is ENABLED --- Disabling IRQ\n");
		disable_irq(di->irq);
	}
	return di->test.cc_sample_conv;

err:
	mutex_unlock(&di->test.lock);
	if (!irqd_irq_disabled(data)) {
		dev_dbg(di->dev, "irq is ENABLED --- Disabling IRQ\n");
		disable_irq(di->irq);
	}
	dev_err(di->dev, "Sample reading failure\n");
	return ret;
}

/**
 * ab8500_fg_test_sample_calibrate() - compute sample calibrated data
 * @di:		pointer to the ab8500_fg structure
 * @val:	raw sample
 *
 * Return sample calibrated value
 * Only used for test purpose
 */
static int ab8500_fg_test_sample_calibrate(struct ab8500_fg *di, int val)
{
	int ret;

	ret = ab8500_fg_test_get_cc_mux_offset(di);
	if (ret < 0)
		return ret;

	if (ret)
		return val - di->test.cc_int_offset;
	else
		return val - di->test.cc_soft_offset;
}

/**
 * ab8500_fg_test_sample_calibrate_to_uA() -  convert sample calibrated data
 * to nuAH
 * @di:		pointer to the ab8500_fg structure
 * @val:	calibrate sample
 *
 * Return sample calibrated value
 * Only used for test purpose
 */
static int ab8500_fg_test_sample_calibrate_to_uA(struct ab8500_fg *di, int val)
{
	di->test.cc_sample_conv_calib_uA = val * QLSB_NANO_AMP_HOURS_X10;
	return di->test.cc_sample_conv_calib_uA;
}

/**
 * ab8500_fg_test_get_nconv_accu() - get number of conversion accumulated
 * @di:		pointer to the ab8500_fg structure
 *
 * Return umber of conversion accumulated or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_get_nconv_accu(struct ab8500_fg *di, u8 *val)
{
	int ret;

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_NCOV_ACCU, val);
	if (ret < 0)
		dev_err(di->dev,
			"get nb samples to be accumulated failed\n");

	return ret;
}

/**
 * ab8500_fg_test_get_nconv_accu_to_uA() - get number of conversion accumulated
 * in uA
 * @di:		pointer to the ab8500_fg structure
 *
 * Return umber of conversion accumulated or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_get_nconv_accu_to_uA(struct ab8500_fg *di, int val)
{
	return val * di->test.cc_sample_conv_calib_uA;
}

/**
 * ab8500_fg_test_set_rst_nconv_accu() -  allows to reset the 21bits accumulator data
 * @di:		pointer to the ab8500_fg structure
 * @enable:	enable/disable to reset the 21bits accumulator data
 *
 * Return 0 or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_set_rst_nconv_accu(struct ab8500_fg *di,
		bool enable)
{
	int ret;
	u8 val = 0;

	if (enable)
		val = RESET_ACCU;
	ret = abx500_mask_and_set_register_interruptible(di->dev,
		AB8500_GAS_GAUGE, AB8500_GASG_CC_NCOV_ACCU_CTRL,
		RESET_ACCU, val);
	if (ret < 0)
		dev_err(di->dev,
			"set accumulator reset bit failed\n");

	return ret;
}

/**
 * ab8500_fg_test_get_rst_nconv_accu() - get staus of ResetNconvAccu bit
 * @di:		pointer to the ab8500_fg structure
 *
 * Return accumulator reset bit or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_get_rst_nconv_accu(struct ab8500_fg *di)
{
	u8 val = 0;
	int ret;

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_NCOV_ACCU_CTRL,  &val);
	if (ret < 0) {
		dev_err(di->dev,
			"get accumulator reset bit failedd\n");
		goto out;
	}

	if (val & RESET_ACCU)
		ret = 1;
	else
		ret = 0;
out:
	return ret;
}

/**
 * ab8500_fg_test_set_nconv_accu_nb_sample() - set number of sample conversion
 * to be accumulated in 21bits accumulator
 * @di:		pointer to the ab8500_fg structure
 * @nb_sample:	number of samples
 *
 * Return 0 or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_set_nconv_accu_nb_sample(struct ab8500_fg *di, u8 val)
{
	int ret;

	ret = abx500_set_register_interruptible(di->dev,
		AB8500_GAS_GAUGE, AB8500_GASG_CC_NCOV_ACCU, val);
	if (ret < 0)
		dev_err(di->dev,
			"set number of samples to accumulated failed\n");

	return ret;
}

/**
 * ab8500_fg_test_get_nconv_accu_nb_sample() - get number of sample conversion
 * to be accumulated in 21bits accumulator
 * @di:		pointer to the ab8500_fg structure
 *
 * Return number of samples to be accumulated or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_get_nconv_accu_nb_sample(struct ab8500_fg *di,
		u8 *val)
{
	int ret;

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_NCOV_ACCU,  val);
	if (ret < 0)
		dev_err(di->dev,
			"get number of samples to accumulated failed\n");

	return ret;
}

/**
 * ab8500_fg_test_read_nconv_accu_sample() - read of accumulator after N samples
 * @di:		pointer to the ab8500_fg structure
 *
 * Return sample or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_read_nconv_accu_sample(struct ab8500_fg *di)
{
	int ret = 0;
	u8 nb_sample;
	u8 low_data, med_data, high_data;
	struct irq_desc *desc = irq_to_desc(di->irq);
	struct irq_data *data = irq_desc_get_irq_data(desc);

	if (irqd_irq_disabled(data)) {
		dev_dbg(di->dev, "irq is disabled --- enabling IRQ for NCONV\n");
		enable_irq(di->irq);
	}

	/* Get nb sample to average */
	ret = ab8500_fg_test_get_nconv_accu_nb_sample(di, &nb_sample);
	if (ret < 0)
		goto out;

	mutex_lock(&di->test.lock);
	dev_dbg(di->dev, "N Samples reading ongoing...\n");

	/* Launch measure */
	ret = abx500_mask_and_set_register_interruptible(di->dev,
		AB8500_GAS_GAUGE, AB8500_GASG_CC_NCOV_ACCU_CTRL,
		RD_NCONV_ACCU_REQ, RD_NCONV_ACCU_REQ);
	if (ret < 0) {
		dev_err(di->dev,
			"launch measure failed\n");
		goto err;
	}

	/* wait for completion of measure */
	if (!wait_for_completion_timeout(&di->test.nconv_accu_complete,
				nb_sample*(HZ/2))) {
		dev_err(di->dev,
			"timeout: didn't receive NCONV_ACCU interrupt\n");
		ret = -EINVAL;
		goto err;
	}

	/* Retrieve samples */
	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_NCOV_ACCU_LOW,  &low_data);
	if (ret < 0) {
		dev_err(di->dev,
			"read low data failed\n");
		goto err;
	}

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_NCOV_ACCU_MED,  &med_data);
	if (ret < 0) {
		dev_err(di->dev,
			"read med data failed\n");
		goto err;
	}

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_NCOV_ACCU_HIGH, &high_data);
	if (ret < 0) {
		dev_err(di->dev,
			"read high data failed\n");
		goto err;
	}

	dev_dbg(di->dev, "N Samples reading done...\n");
	mutex_unlock(&di->test.lock);

	if (!irqd_irq_disabled(data)) {
		dev_dbg(di->dev, "irq is ENABLED --- Disabling IRQ for NCONV\n");
		disable_irq(di->irq);
	}

	return (high_data << 16) | (med_data << 8) | low_data;

err:
	mutex_unlock(&di->test.lock);
	if (!irqd_irq_disabled(data)) {
		dev_dbg(di->dev, "irq is ENABLED --- Disabling IRQ\n");
		disable_irq(di->irq);
	}
	dev_err(di->dev, "Sample reading failure\n");
out:
	return ret;

}

/**
 * ab8500_fg_test_read_nconv_accu_sample_to_uA - convert accu read in uA
 * @di:		pointer to the ab8500_fg structure
 * @val:	accu read
 *
 * Return sample or error code on failure
 * Only used for test purpose
 */
static int ab8500_fg_test_read_nconv_accu_sample_to_uA(struct ab8500_fg *di,
		int val)
{
	return val * QLSB_NANO_AMP_HOURS_X10;
}

/*
 * DebugFS interface
 */

#define AB8500_FG_NAME_STRING "fg"

static int ab8500_fg_test_algorithm_en_print(struct seq_file *s, void *p)
{
	struct ab8500_fg *di;
	bool status;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	status = ab8500_fg_test_is_algorithm_en(di);

	return seq_printf(s, "%d\n", status);
}

static int ab8500_fg_test_algorithm_en_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, ab8500_fg_test_algorithm_en_print,
			inode->i_private);
}

static ssize_t ab8500_fg_test_algorithm_en_write(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct device *dev;
	struct ab8500_fg *di;
	unsigned long user_enable;
	bool enable;
	int err;

	dev = ((struct seq_file *)(file->private_data))->private;
	if (!dev)
		return -ENOMEM;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	/* Get userspace string and assure termination */
	err = kstrtoul_from_user(user_buf, count, 0, &user_enable);
	if (err)
		return -EINVAL;

	if ((user_enable == 0) || (user_enable == 1)) {
		enable = (bool) user_enable;
		err = ab8500_fg_test_algorithm_en(di, enable);
		if (err)
			return err;
	} else {
		dev_err(di->dev, "Wrong input\n"
				"Enter 0 => Disable Gas Gauge test mode\n"
				"Enter 1 => Enable Gas Gauge test mode\n");
		return -EINVAL;
	}

	return count;
}

static const struct file_operations ab8500_fg_test_algorithm_en_fops = {
	.open = ab8500_fg_test_algorithm_en_open,
	.read = seq_read,
	.write = ab8500_fg_test_algorithm_en_write,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int ab8500_fg_test_en_print(struct seq_file *s, void *p)
{
	struct ab8500_fg *di;
	bool status;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	status = ab8500_fg_test_is_en(di);

	return seq_printf(s, "%d\n", status);
}

static int ab8500_fg_test_en_open(struct inode *inode, struct file *file)
{
	return single_open(file, ab8500_fg_test_en_print,
			inode->i_private);
}

static ssize_t ab8500_fg_test_en_write(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct device *dev;
	struct ab8500_fg *di;
	unsigned long user_enable;
	bool enable;
	int err;

	dev = ((struct seq_file *)(file->private_data))->private;
	if (!dev)
		return -ENOMEM;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	/* Get userspace string and assure termination */
	err = kstrtoul_from_user(user_buf, count, 0, &user_enable);
	if (err)
		return -EINVAL;

	if ((user_enable == 0) || (user_enable == 1)) {
		enable = (bool) user_enable;
		err = ab8500_fg_test_en(di, enable);
		if (err)
			return err;
	} else {
		dev_err(di->dev, "Wrong input\n"
				"Enter 0 => Disable Gas Gauge\n"
				"Enter 1 => Enable Gas Gauge\n");
		return -EINVAL;
	}

	return count;
}

static const struct file_operations ab8500_fg_test_en_fops = {
	.open = ab8500_fg_test_en_open,
	.read = seq_read,
	.write = ab8500_fg_test_en_write,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int ab8500_fg_test_cc_int_n_avg_print(struct seq_file *s, void *p)
{
	struct ab8500_fg *di;
	int result;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	result = ab8500_fg_test_get_cc_int_n_avg(di);
	if (result < 0)
		return result;

	return seq_printf(s, "%d\n", result);
}

static int ab8500_fg_test_cc_int_n_avg_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, ab8500_fg_test_cc_int_n_avg_print,
			inode->i_private);
}

static ssize_t ab8500_fg_test_cc_int_n_avg_write(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct ab8500_fg *di;
	char buf[32];
	int buf_size;
	unsigned long user_cc_int_n_avg;
	u8 cc_int_n_avg;
	int err;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	/* Get userspace string and assure termination */
	buf_size = min(count, (sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	err = strict_strtoul(buf, 0, &user_cc_int_n_avg);
	if (err)
		return -EINVAL;

	cc_int_n_avg = (u8) user_cc_int_n_avg;
	err = ab8500_fg_test_set_cc_int_n_avg(di, cc_int_n_avg);
	if (err)
		return err;

	return buf_size;
}

static const struct file_operations ab8500_fg_test_cc_int_n_avg_fops = {
	.open = ab8500_fg_test_cc_int_n_avg_open,
	.read = seq_read,
	.write = ab8500_fg_test_cc_int_n_avg_write,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int ab8500_fg_test_int_calib_print(struct seq_file *s, void *p)
{
	struct ab8500_fg *di;
	int result;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	result = ab8500_fg_test_int_calib(di);
	if (result < 0)
		return result;

	return seq_printf(s, "%d\n", result);
}

static int ab8500_fg_test_int_calib_open(struct inode *inode, struct file *file)
{
	return single_open(file, ab8500_fg_test_int_calib_print,
			inode->i_private);
}

static const struct file_operations ab8500_fg_test_int_calib_fops = {
	.open = ab8500_fg_test_int_calib_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int ab8500_fg_test_soft_calib_print(struct seq_file *s, void *p)
{
	struct ab8500_fg *di;
	int result;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	result = ab8500_fg_test_soft_calib(di);
	if (result < 0)
		return result;

	return seq_printf(s, "%d\n", result);
}

static int ab8500_fg_test_soft_calib_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, ab8500_fg_test_soft_calib_print,
			inode->i_private);
}

static const struct file_operations ab8500_fg_test_soft_calib_fops = {
	.open = ab8500_fg_test_soft_calib_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int ab8500_fg_test_soft_offset_print(struct seq_file *s, void *p)
{
	struct ab8500_fg *di;
	int ret;
	u8 result;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	ret = ab8500_fg_test_get_cc_soft_offset(di, &result);
	if (ret < 0)
		return ret;

	return seq_printf(s, "%d\n", result);
}

static int ab8500_fg_test_soft_offset_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, ab8500_fg_test_soft_offset_print,
			inode->i_private);
}

static ssize_t ab8500_fg_test_soft_offset_write(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct ab8500_fg *di;
	char buf[32];
	int buf_size;
	unsigned long user_val;
	u8 val;
	int err;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	/* Get userspace string and assure termination */
	buf_size = min(count, (sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	err = strict_strtoul(buf, 0, &user_val);
	if (err)
		return -EINVAL;

	val = (u8) user_val;
	err = ab8500_fg_test_set_cc_soft_offset(di, val);
	if (err)
		return err;

	return buf_size;
}

static const struct file_operations ab8500_fg_test_soft_offset_fops = {
	.open = ab8500_fg_test_soft_offset_open,
	.read = seq_read,
	.write = ab8500_fg_test_soft_offset_write,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};


static int ab8500_fg_test_rst_accu_sample_counter_print(struct seq_file *s,
	void *p)
{
	struct ab8500_fg *di;
	int result;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	result = ab8500_fg_test_get_rst_accu_sample_counter(di);

	return seq_printf(s, "%d\n", result);
}

static int ab8500_fg_test_rst_accu_sample_counter_open(struct inode *inode,
	struct file *file)
{
	return single_open(file, ab8500_fg_test_rst_accu_sample_counter_print,
			inode->i_private);
}

static ssize_t ab8500_fg_test_rst_accu_sample_counter_write(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct device *dev;
	struct ab8500_fg *di;
	char buf[32];
	int buf_size;
	unsigned long user_enable;
	bool enable;
	int err;

	dev = ((struct seq_file *)(file->private_data))->private;
	if (!dev)
		return -ENOMEM;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	/* Get userspace string and assure termination */
	buf_size = min(count, (sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	err = strict_strtoul(buf, 0, &user_enable);
	if (err)
		return -EINVAL;
	if ((user_enable == 0) || (user_enable == 1)) {
		enable = (bool) user_enable;
		err = ab8500_fg_test_set_rst_accu_sample_counter(di, enable);
		if (err)
			return err;
	} else {
		dev_err(di->dev, "Wrong input\n"
				"Enter 0. => Disable Reset Acc\n"
				"Enter 1. => Enable Reset Acc\n");
		return -EINVAL;
	}
	return buf_size;
}

static const struct file_operations ab8500_fg_test_rst_accu_sample_fops = {
	.open = ab8500_fg_test_rst_accu_sample_counter_open,
	.read = seq_read,
	.write = ab8500_fg_test_rst_accu_sample_counter_write,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int ab8500_fg_test_cc_mux_offset_print(struct seq_file *s,
	void *p)
{
	struct ab8500_fg *di;
	int result;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	result = ab8500_fg_test_get_cc_mux_offset(di);
	if (result < 0)
		return result;

	return seq_printf(s, "%d\n", result);
}

static int ab8500_fg_test_cc_mux_offset_open(struct inode *inode,
	struct file *file)
{
	return single_open(file, ab8500_fg_test_cc_mux_offset_print,
			inode->i_private);
}

static ssize_t ab8500_fg_test_cc_mux_offset_write(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct device *dev;
	struct ab8500_fg *di;
	char buf[32];
	int buf_size;
	unsigned long user_enable;
	bool enable;
	int err;

	dev = ((struct seq_file *)(file->private_data))->private;
	if (!dev)
		return -ENOMEM;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	buf_size = min(count, (sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	err = strict_strtoul(buf, 0, &user_enable);
	if (err)
		return -EINVAL;
	if ((user_enable == 0) || (user_enable == 1)) {
		enable = (bool) user_enable;
		err = ab8500_fg_test_set_cc_mux_offset(di, enable);
		if (err)
			return err;
	} else {
		dev_err(di->dev, "Wrong input\n"
				"Enter 0. => Manual offset\n"
				"Enter 1. => Internal offset\n");
		return -EINVAL;
	}
	return buf_size;
}

static const struct file_operations ab8500_fg_test_cc_mux_offset_fops = {
	.open = ab8500_fg_test_cc_mux_offset_open,
	.read = seq_read,
	.write = ab8500_fg_test_cc_mux_offset_write,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int ab8500_fg_test_read_sample_print(struct seq_file *s, void *p)
{
	struct ab8500_fg *di;
	int result, cc_sample_calibrate, cc_sample_calibrate_uA;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	result = ab8500_fg_test_read_sample(di);
	if (result < 0)
		return result;

	cc_sample_calibrate = ab8500_fg_test_sample_calibrate(di, result);
	if (cc_sample_calibrate < 0)
		return cc_sample_calibrate;

	cc_sample_calibrate_uA = ab8500_fg_test_sample_calibrate_to_uA(di,
			cc_sample_calibrate);

	return seq_printf(s, "0x%X,%d,%d\n", result, cc_sample_calibrate,
			cc_sample_calibrate_uA);
}

static int ab8500_fg_test_read_sample_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, ab8500_fg_test_read_sample_print,
			inode->i_private);
}

static const struct file_operations ab8500_fg_test_read_sample_fops = {
	.open = ab8500_fg_test_read_sample_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int ab8500_fg_test_get_nconv_accu_print(struct seq_file *s, void *p)
{
	struct ab8500_fg *di;
	int ret, nconv_accu_uA;
	u8 val;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	ret = ab8500_fg_test_get_nconv_accu(di, &val);
	if (ret < 0)
		return ret;

	nconv_accu_uA = ab8500_fg_test_get_nconv_accu_to_uA(di, (int)val);

	return seq_printf(s, "%d,%d\n", val, nconv_accu_uA);
}

static int ab8500_fg_test_get_nconv_accu_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, ab8500_fg_test_get_nconv_accu_print,
			inode->i_private);
}

static const struct file_operations ab8500_fg_test_get_nconv_accu_fops = {
	.open = ab8500_fg_test_get_nconv_accu_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int ab8500_fg_test_rst_nconv_accu_print(struct seq_file *s,
	void *p)
{
	struct ab8500_fg *di;
	int result;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	result = ab8500_fg_test_get_rst_nconv_accu(di);
	if (result < 0)
		return result;

	return seq_printf(s, "%d\n", result);
}

static int ab8500_fg_test_rst_nconv_accu_open(struct inode *inode,
	struct file *file)
{
	return single_open(file, ab8500_fg_test_rst_nconv_accu_print,
			inode->i_private);
}

static ssize_t ab8500_fg_test_rst_nconv_accu_write(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct device *dev;
	struct ab8500_fg *di;
	char buf[32];
	int buf_size;
	unsigned long user_enable;
	bool enable;
	int err;

	dev = ((struct seq_file *)(file->private_data))->private;
	if (!dev)
		return -ENOMEM;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	/* Get userspace string and assure termination */
	buf_size = min(count, (sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	err = strict_strtoul(buf, 0, &user_enable);
	if (err)
		return -EINVAL;
	if ((user_enable == 0) || (user_enable == 1)) {
		enable = (bool) user_enable;
		err = ab8500_fg_test_set_rst_nconv_accu(di, enable);
		if (err)
			return err;
	} else {
		dev_err(di->dev, "Wrong input\n"
				"Enter 0. => Disable Reset Acc\n"
				"Enter 1. => Enable Reset Acc\n");
		return -EINVAL;
	}
	return buf_size;
}

static const struct file_operations ab8500_fg_test_rst_nconv_accu_fops = {
	.open = ab8500_fg_test_rst_nconv_accu_open,
	.read = seq_read,
	.write = ab8500_fg_test_rst_nconv_accu_write,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int ab8500_fg_test_nconv_accu_nb_sample_print(struct seq_file *s,
	void *p)
{
	struct ab8500_fg *di;
	u8 result;
	int ret;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	ret = ab8500_fg_test_get_nconv_accu_nb_sample(di, &result);
	if (ret < 0)
		return ret;

	return seq_printf(s, "%d\n", result);
}

static int ab8500_fg_test_nconv_accu_nb_sample_open(struct inode *inode,
	struct file *file)
{
	return single_open(file, ab8500_fg_test_nconv_accu_nb_sample_print,
			inode->i_private);
}

static ssize_t ab8500_fg_test_nconv_accu_nb_sample_write(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct ab8500_fg *di;
	char buf[32];
	int buf_size;
	unsigned long user_nb_sample;
	u8 nb_sample;
	int err;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	/* Get userspace string and assure termination */
	buf_size = min(count, (sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	err = strict_strtoul(buf, 0, &user_nb_sample);
	if (err)
		return -EINVAL;

	nb_sample = (u8) user_nb_sample;
	err = ab8500_fg_test_set_nconv_accu_nb_sample(di, nb_sample);
	if (err)
		return err;

	return buf_size;
}

static const struct file_operations ab8500_fg_test_nconv_accu_nb_sample_fops = {
	.open = ab8500_fg_test_nconv_accu_nb_sample_open,
	.read = seq_read,
	.write = ab8500_fg_test_nconv_accu_nb_sample_write,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int ab8500_fg_test_nconv_accu_sample_print(struct seq_file *s, void *p)
{
	struct ab8500_fg *di;
	int result, nconv_accu_sample_uA;

	di = ab8500_fg_get();
	if (!di)
		return -ENOMEM;

	result = ab8500_fg_test_read_nconv_accu_sample(di);
	if (result < 0)
		return result;

	nconv_accu_sample_uA = ab8500_fg_test_read_nconv_accu_sample_to_uA(di,
			result);

	return seq_printf(s, "0x%X,%d\n", result, nconv_accu_sample_uA);
}

static int ab8500_fg_test_nconv_accu_sample_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, ab8500_fg_test_nconv_accu_sample_print,
			inode->i_private);
}

static const struct file_operations ab8500_fg_test_nconv_accu_sample_fops = {
	.open = ab8500_fg_test_nconv_accu_sample_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static struct dentry *ab8500_fg_dir;

int ab8500_bm_deepdebug_probe(struct ddbg_service *data,
		struct dentry *parent)
{
	struct dentry *file;
	struct dentry *dir;
	int ret = -ENOMEM;

	ab8500_fg_dir = debugfs_create_dir(AB8500_FG_NAME_STRING,
		parent);
	if (!ab8500_fg_dir)
		goto err;

	file = debugfs_create_file("fg_algo_enable", (S_IRUGO | S_IWUGO),
		ab8500_fg_dir, data, &ab8500_fg_test_algorithm_en_fops);
	if (!file)
		goto err;

	file = debugfs_create_file("fg_enable", (S_IRUGO | S_IWUGO),
		ab8500_fg_dir, data, &ab8500_fg_test_en_fops);
	if (!file)
		goto err;

	dir = debugfs_create_dir("internal_calibration", ab8500_fg_dir);
	if (!dir)
		goto err;

	file = debugfs_create_file("cc_int_n_avg", (S_IRUGO | S_IWUGO),
		dir, data, &ab8500_fg_test_cc_int_n_avg_fops);
	if (!file)
		goto err;

	file = debugfs_create_file("cc_int_offset", (S_IRUGO | S_IWUGO),
		dir, data, &ab8500_fg_test_int_calib_fops);
	if (!file)
		goto err;

	dir = debugfs_create_dir("software_calibration", ab8500_fg_dir);
	if (!dir)
		goto err;

	file = debugfs_create_file("cc_sample_conv", (S_IRUGO | S_IWUGO),
		dir, data, &ab8500_fg_test_soft_calib_fops);
	if (!file)
		goto err;

	file = debugfs_create_file("cc_soft_offset", (S_IRUGO | S_IWUGO),
		dir, data, &ab8500_fg_test_soft_offset_fops);
	if (!file)
		goto err;

	dir = debugfs_create_dir("cc_one_sample", ab8500_fg_dir);
	if (!dir)
		goto err;

	file = debugfs_create_file("cc_rst_accu_sample", (S_IRUGO | S_IWUGO),
		dir, data, &ab8500_fg_test_rst_accu_sample_fops);
	if (!file)
		goto err;

	file = debugfs_create_file("cc_mux_offset", (S_IRUGO | S_IWUGO),
		dir, data, &ab8500_fg_test_cc_mux_offset_fops);
	if (!file)
		goto err;

	file = debugfs_create_file("cc_one_sample", (S_IRUGO | S_IWUGO),
		dir, data, &ab8500_fg_test_read_sample_fops);
	if (!file)
		goto err;
	file = debugfs_create_file("cc_nconv_accu", (S_IRUGO | S_IWUGO),
		dir, data, &ab8500_fg_test_get_nconv_accu_fops);
	if (!file)
		goto err;

	dir = debugfs_create_dir("read_n_samples", ab8500_fg_dir);
	if (!dir)
		goto err;

	file = debugfs_create_file("cc_rst_nconv_accu", (S_IRUGO | S_IWUGO),
		dir, data, &ab8500_fg_test_rst_nconv_accu_fops);
	if (!file)
		goto err;

	file = debugfs_create_file("cc_mux_offset", (S_IRUGO | S_IWUGO),
		dir, data, &ab8500_fg_test_cc_mux_offset_fops);
	if (!file)
		goto err;

	file = debugfs_create_file("cc_nb_samples_to_average",
		(S_IRUGO | S_IWUGO),
		dir, data, &ab8500_fg_test_nconv_accu_nb_sample_fops);
	if (!file)
		goto err;

	file = debugfs_create_file("cc_retrieve_samples", (S_IRUGO | S_IWUGO),
	    dir, data, &ab8500_fg_test_nconv_accu_sample_fops);
	if (!file)
		goto err;

	return 0;

err:
	if (ab8500_fg_dir)
		debugfs_remove_recursive(ab8500_fg_dir);
	pr_err("failed to create debugfs entries.\n");

	return ret;
}

static struct ddbg_service ab8500_fg_ddbg_services = {
	.name = AB8500_FG_NAME_STRING,
	.probe = ab8500_bm_deepdebug_probe,
};

/*
 * Initialization
 */

void __devinit ab8500_fg_test_init(struct ab8500_fg *di)
{
	/* Initialize objects need for test purpose. */
	di->test.enable = false;
	di->test.cc_int_offset = 0;
	di->test.cc_soft_offset = 0;
	di->test.cc_sample_conv = 0;
	di->test.cc_sample_conv_calib_uA = 0;
	init_completion(&di->test.cceoc_complete);
	init_completion(&di->test.nconv_accu_complete);
	init_completion(&di->test.cc_int_calib_complete);
	mutex_init(&di->test.lock);

	deep_debug_service_access_register(&ab8500_fg_ddbg_services);
}

