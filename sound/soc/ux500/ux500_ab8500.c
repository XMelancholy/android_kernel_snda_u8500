/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Mikko J. Lehto <mikko.lehto@symbio.com>,
 *         Mikko Sarmanne <mikko.sarmanne@symbio.com>,
 *         Jarmo K. Kuronen <jarmo.kuronen@symbio.com>.
 *         Ola Lilja <ola.o.lilja@stericsson.com>,
 *         Kristoffer Karlsson <kristoffer.karlsson@stericsson.com>
 *         for ST-Ericsson.
 *
 * License terms:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/abx500/ab8500-gpadc.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/ux500_ab8500.h>
#include <mach/hardware.h>
#include "ux500_pcm.h"
#include "ux500_msp_dai.h"
#include "../codecs/ab8500_audio.h"

#define TX_SLOT_MONO	0x0001
#define TX_SLOT_STEREO	0x0003
#define RX_SLOT_MONO	0x0001
#define RX_SLOT_STEREO	0x0003
#define TX_SLOT_8CH	0x00FF
#define RX_SLOT_8CH	0x00FF

#define DEF_TX_SLOTS	TX_SLOT_STEREO
#define DEF_RX_SLOTS	RX_SLOT_MONO

#define BURST_SLOTS (8)
#define BURST_TX_SLOTS (0xFF)
#define BURST_SLOT_WIDTH (20)

#define DRIVERMODE_NORMAL	0
#define DRIVERMODE_CODEC_ONLY	1

static struct snd_soc_jack jack;
static bool vibra_on;

static int wakeup_irq;
static bool wakeup_enabled;
static void *wakeup_private;

/* Power-control */
static DEFINE_MUTEX(power_lock);
static int ab850x_power_count;

/* ADCM-control */
static DEFINE_MUTEX(adcm_lock);
#define GPADC_MIN_DELTA_DELAY	500
#define GPADC_MAX_DELTA_DELAY	1000
#define GPADC_MAX_VOLT_DIFF	20
#define GPADC_MAX_ITERATIONS	4

/* Clocks */
/* audioclk -> intclk -> sysclk/ulpclk */
static int master_clock_sel;
static int msp4clk_status;

static struct clk *clk_ptr_audioclk;
static struct clk *clk_ptr_intclk;
static struct clk *clk_ptr_sysclk;
static struct clk *clk_ptr_ulpclk;
static struct clk *clk_ptr_gpio1;
static struct clk *clk_ptr_msp4clk;
static struct clk *clk_ptr_sysclk12buf1;

static struct snd_soc_codec *ab850x_codec;

static const char * const enum_mclk[] = {"SYSCLK", "ULPCLK"};
static SOC_ENUM_SINGLE_EXT_DECL(soc_enum_mclk, enum_mclk);

static const char * const enum_dis_ena[] = {"Disabled", "Enabled"};
static SOC_ENUM_SINGLE_EXT_DECL(soc_enum_msp4_clk, enum_dis_ena);

/* ANC States */
static const char * const enum_anc_state[] = {
	"Unconfigured",
	"Apply FIR and IIR",
	"FIR and IIR are configured",
	"Apply FIR",
	"FIR is configured",
	"Apply IIR",
	"IIR is configured",
	"Error"
};
static SOC_ENUM_SINGLE_EXT_DECL(soc_enum_ancstate, enum_anc_state);
enum anc_state {
	ANC_UNCONFIGURED = 0,
	ANC_APPLY_FIR_IIR = 1,
	ANC_FIR_IIR_CONFIGURED = 2,
	ANC_APPLY_FIR = 3,
	ANC_FIR_CONFIGURED = 4,
	ANC_APPLY_IIR = 5,
	ANC_IIR_CONFIGURED = 6,
	ANC_ERROR = 7
};
static enum anc_state anc_status = ANC_UNCONFIGURED;

/* Regulators */
enum regulator_idx {
	REGULATOR_AUDIO,
	REGULATOR_DMIC,
	REGULATOR_AMIC1,
	REGULATOR_AMIC2,
	REGULATOR_VINTCORE
};
static struct regulator_bulk_data reg_info[5] = {
	{	.consumer = NULL, .supply = "v-audio"	},
	{	.consumer = NULL, .supply = "v-dmic"	},
	{	.consumer = NULL, .supply = "v-amic1"	},
	{	.consumer = NULL, .supply = "v-amic2"	},
	{	.consumer = NULL, .supply = "v-intcore"	}
};
static bool reg_enabled[5] =  {
	false,
	false,
	false,
	false,
	false
};
static int reg_claim[4];
enum amic_idx { AMIC_1A, AMIC_1B, AMIC_2 };
struct amic_conf {
	enum regulator_idx reg_id;
	bool enabled;
	char *name;
};
static struct amic_conf amic_info[3] = {
	{ REGULATOR_AMIC1, false, "amic1a" },
	{ REGULATOR_AMIC1, false, "amic1b" },
	{ REGULATOR_AMIC2, false, "amic2" }
};
static DEFINE_MUTEX(amic_conf_lock);

static const char *enum_amic_reg_conf[2] = { "v-amic1", "v-amic2" };
static SOC_ENUM_SINGLE_EXT_DECL(soc_enum_amicconf, enum_amic_reg_conf);

/* Slot configuration */
static unsigned int tx_slots = DEF_TX_SLOTS;
static unsigned int rx_slots = DEF_RX_SLOTS;

static irqreturn_t wakeup_handler(int irq, void *p)
{
	struct snd_pcm_substream *substream = p;
	ux500_pcm_wakeup(substream);
	return IRQ_HANDLED;
}

/* Regulators */

static int enable_regulator(enum regulator_idx idx)
{
	int ret;

	if (reg_info[idx].consumer == NULL) {
		pr_err("%s: Failure to enable regulator '%s'\n",
			__func__, reg_info[idx].supply);
		return -EIO;
	}

	if (reg_enabled[idx])
		return 0;

	ret = regulator_enable(reg_info[idx].consumer);
	if (ret != 0) {
		pr_err("%s: Failure to enable regulator '%s' (ret = %d)\n",
			__func__, reg_info[idx].supply, ret);
		return -EIO;
	};

	reg_enabled[idx] = true;
	pr_debug("%s: Enabled regulator '%s', status: %d, %d, %d, %d\n",
		__func__,
		reg_info[idx].supply,
		(int)reg_enabled[0],
		(int)reg_enabled[1],
		(int)reg_enabled[2],
		(int)reg_enabled[3]);
	return 0;
}

static void disable_regulator(enum regulator_idx idx)
{
	if (reg_info[idx].consumer == NULL) {
		pr_err("%s: Failure to disable regulator '%s'\n",
			__func__, reg_info[idx].supply);
		return;
	}

	if (!reg_enabled[idx])
		return;

	regulator_disable(reg_info[idx].consumer);

	reg_enabled[idx] = false;
	pr_debug("%s: Disabled regulator '%s', status: %d, %d, %d, %d\n",
		__func__,
		reg_info[idx].supply,
		(int)reg_enabled[0],
		(int)reg_enabled[1],
		(int)reg_enabled[2],
		(int)reg_enabled[3]);
}

static int create_regulators(struct snd_soc_codec *codec)
{
	int i, status = 0;
	enum ab850x_audio_chipid chipid = ab850x_audio_get_chipid(codec->dev);
	pr_debug("%s: Enter.\n", __func__);

	for (i = 0; i < ARRAY_SIZE(reg_info); ++i)
		reg_info[i].consumer = NULL;

	for (i = 0; i < ARRAY_SIZE(reg_info); ++i) {
		/* On AB8505 dmic exist on chip but are not drawn to a pin */
		switch (chipid) {
			case AB850X_AUDIO_AB8505_V1:
			case AB850X_AUDIO_AB8505_V2:
			case AB850X_AUDIO_AB8505_V3:
				if (i == REGULATOR_DMIC)
					continue;
			default:
			break;
		}
		reg_info[i].consumer = regulator_get(NULL, reg_info[i].supply);
		if (IS_ERR(reg_info[i].consumer)) {
			status = PTR_ERR(reg_info[i].consumer);
			pr_err("%s: ERROR: Failed to get regulator '%s' (ret = %d)!\n",
				__func__, reg_info[i].supply, status);
			reg_info[i].consumer = NULL;
			goto err_get;
		}
	}

	return 0;

err_get:
	for (i = 0; i < ARRAY_SIZE(reg_info); ++i) {
		if (reg_info[i].consumer) {
			regulator_put(reg_info[i].consumer);
			reg_info[i].consumer = NULL;
		}
	}

	return status;
}

static int claim_amic_regulator(enum amic_idx amic_id)
{
	enum regulator_idx reg_id = amic_info[amic_id].reg_id;
	int ret = 0;

	reg_claim[reg_id]++;
	if (reg_claim[reg_id] > 1)
		goto cleanup;

	ret = enable_regulator(reg_id);
	if (ret < 0) {
		pr_err("%s: Failed to claim %s for %s (ret = %d)!",
			__func__, reg_info[reg_id].supply,
			amic_info[amic_id].name, ret);
		reg_claim[reg_id]--;
	}

cleanup:
	amic_info[amic_id].enabled = (ret == 0);

	return ret;
}

static void release_amic_regulator(enum amic_idx amic_id)
{
	enum regulator_idx reg_id = amic_info[amic_id].reg_id;

	reg_claim[reg_id]--;
	if (reg_claim[reg_id] <= 0) {
		disable_regulator(reg_id);
		reg_claim[reg_id] = 0;
	}

	amic_info[amic_id].enabled = false;
}

/* Power/clock control */

static int ux500_ab850x_power_control_inc(struct snd_soc_codec *codec)
{
	int ret = 0;
	enum ab850x_audio_chipid chipid ;

	chipid = ab850x_audio_get_chipid(codec->dev);

	pr_debug("%s: Enter.\n", __func__);

	mutex_lock(&power_lock);

	ab850x_power_count++;
	pr_debug("%s: ab850x_power_count changed from %d to %d",
		__func__,
		ab850x_power_count-1,
		ab850x_power_count);

	if (ab850x_power_count == 1) {
		/* Turn on audio-regulator */
		ret = enable_regulator(REGULATOR_AUDIO);
		if (ret < 0)
			goto out;

		if (chipid == AB850X_AUDIO_AB9540_V1) {
			/* Turn on VINTCORE regulator bug cut 1*/
			ret = enable_regulator(REGULATOR_VINTCORE);
			if (ret < 0) {
				pr_err("%s: ERROR to Enable VINTCORE Regul", __func__);
				goto reg_err;
			}
		}
		/* Enable audio-clock */
		ret = clk_set_parent(clk_ptr_intclk,
			(master_clock_sel == 0) ?
			clk_ptr_sysclk : clk_ptr_ulpclk);
		if (ret != 0) {
			pr_err("%s: ERROR: Setting master-clock to %s failed (ret = %d)!",
				__func__,
				(master_clock_sel == 0) ? "SYSCLK" : "ULPCLK",
				ret);
			ret = -EIO;
			goto clk_err;
		}
		pr_debug("%s: Enabling master-clock (%s).",
			__func__,
			(master_clock_sel == 0) ? "SYSCLK" : "ULPCLK");
		ret = clk_enable(clk_ptr_audioclk);
		if (ret != 0) {
			pr_err("%s: ERROR: clk_enable failed (ret = %d)!",
				__func__, ret);
			ret = -EIO;
			ab850x_power_count = 0;
			goto clk_err;
		}

		/* Power on audio-parts of AB850X */
		ret = ab850x_audio_power_control(codec, true);
	}

	goto out;

clk_err:
	if (chipid == AB850X_AUDIO_AB9540_V1)
		disable_regulator(REGULATOR_VINTCORE);
reg_err:
	disable_regulator(REGULATOR_AUDIO);

out:
	mutex_unlock(&power_lock);

	pr_debug("%s: Exit.\n", __func__);

	return ret;
}

static void ux500_ab850x_power_control_dec(struct snd_soc_codec *codec)
{
	enum ab850x_audio_chipid chipid;

	pr_debug("%s: Enter.\n", __func__);

	mutex_lock(&power_lock);
	chipid = ab850x_audio_get_chipid(codec->dev);

	ab850x_power_count--;

	pr_debug("%s: ab850x_power_count changed from %d to %d",
		__func__,
		ab850x_power_count+1,
		ab850x_power_count);

	if (ab850x_power_count == 0) {
		/* Power off audio-parts of AB850X */
		ab850x_audio_power_control(codec, false);

		/* Disable audio-clock */
		pr_debug("%s: Disabling master-clock (%s).",
			__func__,
			(master_clock_sel == 0) ? "SYSCLK" : "ULPCLK");
		clk_disable(clk_ptr_audioclk);

		/* Turn off audio-regulator */
		disable_regulator(REGULATOR_AUDIO);

		if (chipid == AB850X_AUDIO_AB9540_V1)
			disable_regulator(REGULATOR_VINTCORE);

	}

	mutex_unlock(&power_lock);

	pr_debug("%s: Exit.\n", __func__);
}

/* Controls - Non-DAPM Non-ASoC */

static int mclk_input_control_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = master_clock_sel;

	return 0;
}

static int mclk_input_control_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	unsigned int val;
	int ret = 0;
	val = (ucontrol->value.enumerated.item[0] != 0);

	if (master_clock_sel == val)
		return ret;

	mutex_lock(&power_lock);
	master_clock_sel = val;
	if (ab850x_power_count > 0) {
		ret = clk_set_parent(clk_ptr_intclk,
			(master_clock_sel == 0) ?
			clk_ptr_sysclk : clk_ptr_ulpclk);
		if (ret != 0) {
			pr_err("%s: ERROR: Setting master-clock to %s failed (ret = %d)!",
				__func__,
				(master_clock_sel == 0) ? "SYSCLK" : "ULPCLK",
				ret);
			ret = -EIO;
			goto clk_err;
		}
	}
	ret = 1;

clk_err:
	mutex_unlock(&power_lock);

	return ret;
}

static int msp4_clk_control_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = msp4clk_status;

	return 0;
}

static int msp4_clk_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	unsigned int val;

	val = ucontrol->value.enumerated.item[0];
	if (val == msp4clk_status)
		return 0;

	if ((clk_ptr_sysclk12buf1 == NULL) || (clk_ptr_msp4clk == NULL)
			|| (clk_ptr_sysclk == NULL)) {
		pr_err("%s: ERROR: clock not available (%p/%p/%p)", __func__,
				clk_ptr_sysclk12buf1, clk_ptr_msp4clk,
				clk_ptr_sysclk);
		return -ENODEV;
	}

	pr_debug("%s: Enter. %s sysclk12buf1\n", __func__, enum_dis_ena[val]);

	if (val == 1) {
		if (clk_enable(clk_ptr_sysclk)) {
			pr_err("%s: ERROR: clk_enable(sysclk) FAILED\n",
					__func__);
			return -EIO;
		}
		if (clk_enable(clk_ptr_sysclk12buf1)) {
			pr_err("%s: ERROR: clk_enable(sysclk12buf1) FAILED\n",
					__func__);
			clk_disable(clk_ptr_sysclk);
			return -EIO;
		}
		if (clk_enable(clk_ptr_msp4clk)) {
			pr_err("%s: ERROR: clk_enable(msp4clk) FAILED\n",
					__func__);
			clk_disable(clk_ptr_sysclk);
			clk_disable(clk_ptr_sysclk12buf1);
			return -EIO;
		}
	} else {
		pr_debug("%s: Enter. Disable sysclk12buf1\n", __func__);
		clk_disable(clk_ptr_msp4clk);
		clk_disable(clk_ptr_sysclk12buf1);
		clk_disable(clk_ptr_sysclk);
	}

	msp4clk_status = val;

	return 1;
}

static int anc_status_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	mutex_lock(&codec->mutex);
	ucontrol->value.integer.value[0] = anc_status;
	mutex_unlock(&codec->mutex);

	return 0;
}

static int anc_status_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec;
	bool apply_fir, apply_iir;
	int req, ret;

	pr_debug("%s: Enter.\n", __func__);

	req = ucontrol->value.integer.value[0];
	if (req != ANC_APPLY_FIR_IIR && req != ANC_APPLY_FIR &&
		req != ANC_APPLY_IIR) {
		pr_err("%s: ERROR: Unsupported status to set '%s'!\n",
			__func__, enum_anc_state[req]);
		return -EINVAL;
	}
	apply_fir = req == ANC_APPLY_FIR || req == ANC_APPLY_FIR_IIR;
	apply_iir = req == ANC_APPLY_IIR || req == ANC_APPLY_FIR_IIR;

	codec = snd_kcontrol_chip(kcontrol);

	ret = ux500_ab850x_power_control_inc(codec);
	if (ret < 0) {
		pr_err("%s: ERROR: Failed to enable power (ret = %d)!\n",
			__func__, ret);
		goto cleanup;
	}

	mutex_lock(&codec->mutex);

	ab850x_audio_anc_configure(codec, apply_fir, apply_iir);

	if (apply_fir) {
		if (anc_status == ANC_IIR_CONFIGURED)
			anc_status = ANC_FIR_IIR_CONFIGURED;
		else if (anc_status != ANC_FIR_IIR_CONFIGURED)
			anc_status =  ANC_FIR_CONFIGURED;
	}
	if (apply_iir) {
		if (anc_status == ANC_FIR_CONFIGURED)
			anc_status = ANC_FIR_IIR_CONFIGURED;
		else if (anc_status != ANC_FIR_IIR_CONFIGURED)
			anc_status =  ANC_IIR_CONFIGURED;
	}

	mutex_unlock(&codec->mutex);

	ux500_ab850x_power_control_dec(codec);

cleanup:
	if (ret < 0)
		pr_err("%s: Unable to configure ANC! (ret = %d)\n",
			__func__, ret);

	pr_debug("%s: Exit.\n", __func__);

	return (ret < 0) ? ret : 1;
}

static int amic_reg_control_get(struct snd_ctl_elem_value *ucontrol,
		enum amic_idx amic_id)
{
	ucontrol->value.integer.value[0] =
		(amic_info[amic_id].reg_id == REGULATOR_AMIC2);

	return 0;
}

static int amic_reg_control_put(struct snd_ctl_elem_value *ucontrol,
		enum amic_idx amic_id)
{
	enum regulator_idx old_reg_id, new_reg_id;
	int ret = 0;

	if (ucontrol->value.integer.value[0] == 0)
		new_reg_id = REGULATOR_AMIC1;
	else
		new_reg_id = REGULATOR_AMIC2;

	mutex_lock(&amic_conf_lock);

	old_reg_id = amic_info[amic_id].reg_id;
	if (old_reg_id == new_reg_id)
		goto cleanup;

	if (!amic_info[amic_id].enabled) {
		amic_info[amic_id].reg_id = new_reg_id;
		goto cleanup;
	}

	release_amic_regulator(amic_id);
	amic_info[amic_id].reg_id = new_reg_id;
	ret = claim_amic_regulator(amic_id);

cleanup:
	mutex_unlock(&amic_conf_lock);

	return (ret < 0) ? 0 : 1;
}

static int amic1a_reg_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return amic_reg_control_get(ucontrol, AMIC_1A);
}

static int amic1a_reg_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return amic_reg_control_put(ucontrol, AMIC_1A);
}

static int amic1b_reg_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return amic_reg_control_get(ucontrol, AMIC_1B);
}

static int amic1b_reg_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return amic_reg_control_put(ucontrol, AMIC_1B);
}

static int amic2_reg_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return amic_reg_control_get(ucontrol, AMIC_2);
}

static int amic2_reg_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return amic_reg_control_put(ucontrol, AMIC_2);
}

/* DAPM-events */

static int dapm_audioreg_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	struct snd_soc_codec *codec = w->codec;

	if (SND_SOC_DAPM_EVENT_ON(event))
		ux500_ab850x_power_control_inc(codec);
	else
		ux500_ab850x_power_control_dec(codec);

	return 0;
}

static int dapm_amicreg_event(enum amic_idx amic_id, int event)
{
	int ret = 0;

	mutex_lock(&amic_conf_lock);

	if (SND_SOC_DAPM_EVENT_ON(event))
		ret = claim_amic_regulator(amic_id);
	else if (amic_info[amic_id].enabled)
		release_amic_regulator(amic_id);

	mutex_unlock(&amic_conf_lock);

	return ret;
}

static int dapm_amic1areg_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	return dapm_amicreg_event(AMIC_1A, event);
}

static int dapm_amic1breg_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	return dapm_amicreg_event(AMIC_1B, event);
}

static int dapm_amic2reg_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	return dapm_amicreg_event(AMIC_2, event);
}

static int dapm_dmicreg_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	int ret = 0;

	if (SND_SOC_DAPM_EVENT_ON(event))
		ret = enable_regulator(REGULATOR_DMIC);
	else
		disable_regulator(REGULATOR_DMIC);

	return ret;
}

/* DAPM-widgets */

static const struct snd_soc_dapm_widget ux500_ab850x_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("AUDIO Regulator",
			SND_SOC_NOPM, 0, 0, dapm_audioreg_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("AMIC1A Regulator",
			SND_SOC_NOPM, 0, 0, dapm_amic1areg_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("AMIC1B Regulator",
			SND_SOC_NOPM, 0, 0, dapm_amic1breg_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("AMIC2 Regulator",
			SND_SOC_NOPM, 0, 0, dapm_amic2reg_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("DMIC Regulator",
			SND_SOC_NOPM, 0, 0, dapm_dmicreg_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

/* DAPM-routes */

static const struct snd_soc_dapm_route ux500_ab850x_dapm_intercon[] = {

	/* Power AB850X audio-block when AD/DA is active */
	{"DAC", NULL, "AUDIO Regulator"},
	{"ADC", NULL, "AUDIO Regulator"},

	/* Power AB850X audio-block when LineIn is active */
	{"LINL Enable", NULL, "AUDIO Regulator"},
	{"LINR Enable", NULL, "AUDIO Regulator"},

	/* Power configured regulator when an analog mic is enabled */
	{"MIC1A Input", NULL, "AMIC1A Regulator"},
	{"MIC1B Input", NULL, "AMIC1B Regulator"},
	{"MIC2 Input", NULL, "AMIC2 Regulator"},

	/* Power DMIC-regulator when any digital mic is enabled */
	{"DMic 1", NULL, "DMIC Regulator"},
	{"DMic 2", NULL, "DMIC Regulator"},
	{"DMic 3", NULL, "DMIC Regulator"},
	{"DMic 4", NULL, "DMIC Regulator"},
	{"DMic 5", NULL, "DMIC Regulator"},
	{"DMic 6", NULL, "DMIC Regulator"},
};

static const struct snd_soc_dapm_route ux500_ab8500_ab9540_dapm_routes[] = {

	/* Power DMIC-regulator when any digital mic is enabled */
	{"DMic 1", NULL, "DMIC Regulator"},
	{"DMic 2", NULL, "DMIC Regulator"},
	{"DMic 3", NULL, "DMIC Regulator"},
	{"DMic 4", NULL, "DMIC Regulator"},
	{"DMic 5", NULL, "DMIC Regulator"},
	{"DMic 6", NULL, "DMIC Regulator"},
};

/* Key controls */

static struct snd_kcontrol_new ab850x_snd_controls[] = {
	SOC_ENUM_EXT("Master Clock Select", soc_enum_mclk,
			mclk_input_control_get, mclk_input_control_put),
	SOC_ENUM_EXT("ANC Status", soc_enum_ancstate,
			anc_status_control_get, anc_status_control_put),
	SOC_ENUM_EXT("Mic 1A Regulator", soc_enum_amicconf,
			amic1a_reg_control_get, amic1a_reg_control_put),
	SOC_ENUM_EXT("Mic 1B Regulator", soc_enum_amicconf,
			amic1b_reg_control_get, amic1b_reg_control_put),
	SOC_ENUM_EXT("Mic 2 Regulator", soc_enum_amicconf,
			amic2_reg_control_get, amic2_reg_control_put),
};

static struct snd_kcontrol_new ab9540_vx_snd_controls[] = {
	SOC_ENUM_EXT("MSP4 Clock Enable", soc_enum_msp4_clk,
			msp4_clk_control_get, msp4_clk_control_put),
};

static int add_widgets(struct snd_soc_codec *codec)
{
	int ret;
	enum ab850x_audio_chipid chipid;

	ret = snd_soc_dapm_new_controls(&codec->dapm,
			ux500_ab850x_dapm_widgets,
			ARRAY_SIZE(ux500_ab850x_dapm_widgets));
	if (ret < 0) {
		pr_err("%s: Failed to create DAPM controls (%d).\n",
			__func__, ret);
		return ret;
	}

	ret = snd_soc_dapm_add_routes(&codec->dapm,
				ux500_ab850x_dapm_intercon,
				ARRAY_SIZE(ux500_ab850x_dapm_intercon));
	if (ret < 0) {
		pr_err("%s: Failed to add DAPM routes (%d).\n",
			__func__, ret);
		return ret;
	}

	/* On AB8505 dmic exist on chip but are not drawn to a pin */
	chipid = ab850x_audio_get_chipid(codec->dev);
	switch (chipid) {
	case AB850X_AUDIO_AB8505_V1:
	case AB850X_AUDIO_AB8505_V2:
	case AB850X_AUDIO_AB8505_V3:
		break;
	default:
		ret = snd_soc_dapm_add_routes(&codec->dapm,
				ux500_ab8500_ab9540_dapm_routes,
				ARRAY_SIZE(ux500_ab8500_ab9540_dapm_routes));
		if (ret < 0) {
			pr_err("%s: Failed to add DMIC DAPM routes (%d).\n",
					__func__, ret);
			return ret;
		}
	break;
	}

	return 0;
}

/* ASoC */
int ux500_ab850x_probe(struct snd_soc_card *card)
{
	struct ux500_card_data *data = snd_soc_card_get_drvdata(card);

	wakeup_irq = data->wakeup_irq;

	return 0;
}

int ux500_ab850x_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;

	pr_debug("%s: Enter\n", __func__);

	/* Enable gpio.1-clock (needed by DSP in burst mode) */
	ret = clk_enable(clk_ptr_gpio1);
	if (ret) {
		pr_err("%s: ERROR: clk_enable(gpio.1) failed (ret = %d)!",
			__func__, ret);
		return ret;
	}

	return 0;
}

void ux500_ab850x_shutdown(struct snd_pcm_substream *substream)
{
	pr_debug("%s: Enter\n", __func__);

	/* Reset slots configuration to default(s) */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		tx_slots = DEF_TX_SLOTS;
	else
		rx_slots = DEF_RX_SLOTS;

	clk_disable(clk_ptr_gpio1);

	if (wakeup_enabled) {
		free_irq(wakeup_irq, wakeup_private);
		wakeup_enabled = false;
	}
}

int ux500_ab850x_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int fmt, fmt_if1;
	int channels, ret = 0, slots, driver_mode, err = 0;
	unsigned int codec_slot_width, cpu_slot_width;
	bool streamIsPlayback =
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK);
	int burst_mode;

	pr_debug("%s: Enter\n", __func__);

	pr_debug("%s: substream->pcm->name = %s\n"
		"substream->pcm->id = %s.\n"
		"substream->name = %s.\n"
		"substream->number = %d.\n",
		__func__,
		substream->pcm->name,
		substream->pcm->id,
		substream->name,
		substream->number);

	channels = params_channels(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S32_LE:
		cpu_slot_width = 32;
		break;

	default:
	case SNDRV_PCM_FORMAT_S16_LE:
		cpu_slot_width = 16;
		break;
	}

	burst_mode = snd_soc_test_bits(codec, REG_DIGIFCONF3,
		1 << REG_DIGIFCONF3_IF0BFIFOEN, 0);

	if (burst_mode) {
		if (cpu_slot_width != 32 || channels != 2)
			return -EINVAL;

		if (streamIsPlayback) {
			if (!wakeup_enabled) {
				err = request_irq(
					wakeup_irq,
					wakeup_handler,
					IRQF_TRIGGER_RISING,
					"ab8500-wakeup",
					substream);
				pr_debug("%s: request_irq wakeup (%d)\n",
					__func__,
					err);
				if (err) {
					pr_err("%s: request_irq wakeup failed(%d)\n",
						__func__,
						err);
					return err;
				}
				wakeup_enabled = true;
				wakeup_private = substream;
			}
		} else {
			return -EAGAIN;
		}
	} else {
		if (wakeup_enabled) {
			free_irq(wakeup_irq, wakeup_private);
			wakeup_enabled = false;
		}
	}

	/* Setup codec depending on driver-mode */
	driver_mode = (channels == 8) ?
		DRIVERMODE_CODEC_ONLY : DRIVERMODE_NORMAL;
	pr_debug("%s: Driver-mode: %s.\n",
		__func__,
		(driver_mode == DRIVERMODE_NORMAL) ? "NORMAL" : "CODEC_ONLY");

	ab850x_audio_set_bit_delay(codec_dai, 1);
	ux500_msp_dai_set_data_delay(cpu_dai, MSP_DELAY_1);

	if (driver_mode == DRIVERMODE_NORMAL) {
		codec_slot_width = cpu_slot_width;
		ab850x_audio_set_word_length(codec_dai, codec_slot_width);
		fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_CBM_CFM |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CONT;
	} else {
		codec_slot_width = 20;
		ab850x_audio_set_word_length(codec_dai, codec_slot_width);
		fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_CBM_CFM |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_GATED;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("%s: ERROR: snd_soc_dai_set_fmt failed for codec_dai (ret = %d)!\n",
			__func__,
			ret);
		return ret;
	}

	if (burst_mode)
		fmt |= SND_SOC_DAIFMT_GATED;

	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		pr_err("%s: ERROR: snd_soc_dai_set_fmt for cpu_dai (ret = %d)!\n",
			__func__,
			ret);
		return ret;
	}

	/* Setup TDM-slots */
	if (burst_mode) {
		tx_slots = BURST_TX_SLOTS;
		rx_slots = 0;
		slots = BURST_SLOTS;
		cpu_slot_width = BURST_SLOT_WIDTH;
	} else {
		switch (channels) {
		case 1:
			slots = 8;
			tx_slots = (streamIsPlayback) ? TX_SLOT_MONO : 0;
			rx_slots = (streamIsPlayback) ? 0 : RX_SLOT_MONO;
			break;
		case 2:
			slots = 8;
			tx_slots = (streamIsPlayback) ? TX_SLOT_STEREO : 0;
			rx_slots = (streamIsPlayback) ? 0 : RX_SLOT_STEREO;
			break;
		case 8:
			slots = 8;
			tx_slots = (streamIsPlayback) ? TX_SLOT_8CH : 0;
			rx_slots = (streamIsPlayback) ? 0 : RX_SLOT_8CH;
			break;
		default:
			return -EINVAL;
		}
	}

	pr_debug("%s: CPU-DAI TDM: TX=0x%04X RX=0x%04x\n",
		__func__, tx_slots, rx_slots);

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, tx_slots, rx_slots, slots,
		cpu_slot_width);

	if (ret)
		return ret;

	pr_debug("%s: CODEC-DAI TDM: TX=0x%04X RX=0x%04x\n",
		__func__, tx_slots, rx_slots);
	ret = snd_soc_dai_set_tdm_slot(codec_dai, tx_slots, rx_slots, slots,
		codec_slot_width);
	if (ret)
		return ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pr_debug("%s: Setup IF1 for FM-radio.\n", __func__);
		fmt_if1 = SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_I2S;
		ret = ab850x_audio_setup_if1(codec_dai->codec, fmt_if1, 16, 1);
		if (ret)
			return ret;
	}

	return 0;
}

struct snd_soc_ops ux500_ab850x_ops[] = {
	{
	.hw_params = ux500_ab850x_hw_params,
	.startup = ux500_ab850x_startup,
	.shutdown = ux500_ab850x_shutdown,
	}
};

int ux500_ab850x_machine_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	enum ab850x_audio_chipid chipid;
	struct snd_soc_codec *codec = rtd->codec;
	int ret;

	pr_debug("%s Enter.\n", __func__);

	ret = create_regulators(codec);
	if (ret < 0) {
		pr_err("%s: ERROR: Failed to instantiate regulators (ret = %d)!\n",
				__func__, ret);
		return ret;
	}


	ret = snd_soc_jack_new(codec,
			"AB850X Hs Status",
			SND_JACK_HEADPHONE     |
			SND_JACK_MICROPHONE    |
			SND_JACK_HEADSET       |
			SND_JACK_LINEOUT       |
			SND_JACK_MECHANICAL    |
			SND_JACK_VIDEOOUT,
			&jack);

	chipid = ab850x_audio_get_chipid(codec->dev);

	/* Add controls */
	ret = snd_soc_add_codec_controls(codec, ab850x_snd_controls,
		ARRAY_SIZE(ab850x_snd_controls));
	if (ret < 0)
		goto controls_done;

	switch (chipid) {
	case AB850X_AUDIO_AB8500:
	case AB850X_AUDIO_AB8505_V1:
	case AB850X_AUDIO_AB8505_V2:
	case AB850X_AUDIO_AB8505_V3:
		break;
	case AB850X_AUDIO_AB9540_V1:
	case AB850X_AUDIO_AB9540_V2:
		ret = snd_soc_add_codec_controls(codec, ab9540_vx_snd_controls,
			ARRAY_SIZE(ab9540_vx_snd_controls));
		break;
	default:
		pr_err("ERROR: controls not mapped for chipid %d! (ret = %d)!",
				chipid, -EFAULT);
		return -EFAULT;
	}

controls_done:
	if (ret < 0) {
		pr_err("ERROR: %s: Failed to add controls (%d).\n",
			__func__, ret);
		return ret;
	}

	/* Get references to clock-nodes */
	clk_ptr_sysclk = NULL;
	clk_ptr_ulpclk = NULL;
	clk_ptr_intclk = NULL;
	clk_ptr_audioclk = NULL;
	clk_ptr_gpio1 = NULL;
	clk_ptr_sysclk12buf1 = NULL;
	clk_ptr_msp4clk = NULL;

	clk_ptr_ulpclk = clk_get(codec->dev, "ulpclk");
	if (IS_ERR(clk_ptr_ulpclk)) {
		pr_err("ERROR: clk_get(ulp) failed (ret = %d)!", -EFAULT);
		return -EFAULT;
	}
	clk_ptr_intclk = clk_get(codec->dev, "intclk");
	if (IS_ERR(clk_ptr_intclk)) {
		pr_err("ERROR: clk_get(int) failed (ret = %d)!", -EFAULT);
		return -EFAULT;
	}
	clk_ptr_audioclk = clk_get(codec->dev, "audioclk");
	if (IS_ERR(clk_ptr_audioclk)) {
		pr_err("ERROR: clk_get(audio) failed (ret = %d)!", -EFAULT);
		return -EFAULT;
	}
	clk_ptr_gpio1 = clk_get_sys("gpio.1", NULL);
	if (IS_ERR(clk_ptr_gpio1)) {
		pr_err("ERROR: clk_get_sys(gpio.1) failed (ret = %d)!",
			-EFAULT);
		return -EFAULT;
	}

	switch (chipid) {
	case AB850X_AUDIO_AB8500:
	case AB850X_AUDIO_AB8505_V1:
	case AB850X_AUDIO_AB8505_V2:
	case AB850X_AUDIO_AB8505_V3:
		clk_ptr_sysclk = clk_get(codec->dev, "sysclk");
		if (IS_ERR(clk_ptr_sysclk)) {
			pr_err("ERROR: clk_get(sys) failed (ret = %d)!",
					-EFAULT);
			return -EFAULT;
		}
		break;
	case AB850X_AUDIO_AB9540_V1:
	case AB850X_AUDIO_AB9540_V2:
		clk_ptr_sysclk = clk_get(codec->dev, "ab9540-sysclk");
		if (IS_ERR(clk_ptr_sysclk)) {
			pr_err("ERROR: clk_get(sys) failed (ret = %d)!",
					-EFAULT);
			return -EFAULT;
		}
		clk_ptr_sysclk12buf1 = clk_get(codec->dev,
				"ab9540-sysclk12Buf1");
		if (IS_ERR(clk_ptr_sysclk12buf1)) {
			pr_err("ERROR: clk_get(sclk12Buf1) failed (ret = %d)!",
					-EFAULT);
			return -EFAULT;
		}
		clk_ptr_msp4clk = clk_get(codec->dev, "msp4");
		if (IS_ERR(clk_ptr_msp4clk)) {
			pr_err("ERROR: clk_get(msp4) failed (ret = %d)!",
					-EFAULT);
			return -EFAULT;
		}
		break;
	default:
		pr_err("ERROR: clocks not mapped for chipid %d! (ret = %d)!",
				chipid, -EFAULT);
		return -EFAULT;
	}

	/* Set intclk default parent to ulpclk */
	ret = clk_set_parent(clk_ptr_intclk, clk_ptr_ulpclk);
	if (ret) {
		pr_err("%s: Setting intclk parent to ulpclk failed (ret = %d)!",
			__func__, ret);
		return -EFAULT;
	}

	master_clock_sel = 1;	/* default is ulp */
	msp4clk_status = 0;	/* default is msp4 not requested */
	ab850x_power_count = 0;

	reg_claim[REGULATOR_AMIC1] = 0;
	reg_claim[REGULATOR_AMIC2] = 0;

	/* Add DAPM-widgets */
	ret = add_widgets(codec);
	if (ret < 0) {
		pr_err("%s: ERROR: Failed add widgets (%d).\n", __func__, ret);
		return ret;
	}

	ab850x_codec = codec;

	return 0;
}

int ux500_ab850x_soc_machine_drv_init(void)
{
	pr_debug("%s: Enter.\n", __func__);

	vibra_on = false;

	return 0;
}

void ux500_ab850x_soc_machine_drv_cleanup(void)
{
	pr_debug("%s: Enter.\n", __func__);

	regulator_bulk_free(ARRAY_SIZE(reg_info), reg_info);

	if (clk_ptr_sysclk != NULL)
		clk_put(clk_ptr_sysclk);
	if (clk_ptr_ulpclk != NULL)
		clk_put(clk_ptr_ulpclk);
	if (clk_ptr_intclk != NULL)
		clk_put(clk_ptr_intclk);
	if (clk_ptr_audioclk != NULL)
		clk_put(clk_ptr_audioclk);
	if (clk_ptr_gpio1 != NULL)
		clk_put(clk_ptr_gpio1);
	if (clk_ptr_msp4clk != NULL)
		clk_put(clk_ptr_msp4clk);
	if (clk_ptr_sysclk12buf1 != NULL)
		clk_put(clk_ptr_sysclk12buf1);

	clk_ptr_sysclk = NULL;
	clk_ptr_ulpclk = NULL;
	clk_ptr_intclk = NULL;
	clk_ptr_audioclk = NULL;
	clk_ptr_gpio1 = NULL;
	clk_ptr_sysclk12buf1 = NULL;
	clk_ptr_msp4clk = NULL;
}

/*
 * Measures a relative stable voltage from spec. input on spec channel
 */
static int gpadc_convert_stable(struct ab8500_gpadc *gpadc,
			u8 channel, int *value)
{
	int i = GPADC_MAX_ITERATIONS;
	int mv1, mv2, dmv;

	mv1 = ab8500_gpadc_sw_hw_convert(gpadc, channel, SAMPLE_16,
			0, 0, ADC_SW);
	do {
		i--;
		usleep_range(GPADC_MIN_DELTA_DELAY, GPADC_MAX_DELTA_DELAY);
		mv2 = ab8500_gpadc_sw_hw_convert(gpadc, channel, SAMPLE_16,
				0, 0, ADC_SW);
		dmv = abs(mv2 - mv1);
		mv1 = mv2;
	} while (i > 0 && dmv > GPADC_MAX_VOLT_DIFF);

	if (mv1 < 0 || dmv > GPADC_MAX_VOLT_DIFF)
		return -EIO;

	*value = mv1;

	return 0;
}

/* Extended interface */

int ux500_ab850x_audio_pwm_vibra(unsigned char pdc1, unsigned char ndc1,
		unsigned char pdc2, unsigned char ndc2)
{
	enum ab850x_audio_chipid chipid;
	bool vibra_on_new;
	int ret;

	if (ab850x_codec == NULL) {
		pr_err("%s: ERROR: ab850x ASoC-driver not yet probed!\n",
			__func__);
		return -EFAULT;
	}

	pr_debug("%s: Enter.\n", __func__);

	chipid = ab850x_audio_get_chipid(ab850x_codec->dev);
	if (chipid == AB850X_AUDIO_UNKNOWN) {
		pr_err("%s: ab850x chipset not yet known!\n", __func__);
		return -EIO;
	}

	vibra_on_new = pdc1 | ndc1;
	if (chipid == AB850X_AUDIO_AB8500)
		vibra_on_new |= pdc2 | ndc2;

	if ((!vibra_on_new) && (vibra_on)) {
		pr_debug("%s: PWM-vibra off.\n", __func__);
		vibra_on = false;
		ux500_ab850x_power_control_dec(ab850x_codec);
	}
	if ((vibra_on_new) && (!vibra_on)) {
		pr_debug("%s: PWM-vibra on.\n", __func__);
		vibra_on = true;
		ux500_ab850x_power_control_inc(ab850x_codec);
	}

	ret = ab850x_audio_pwm_vibra(ab850x_codec, pdc1, ndc1, pdc2, ndc2);

	pr_debug("%s: Exit.\n", __func__);

	return ret;
}

int ux500_ab850x_audio_gpadc_measure(struct ab8500_gpadc *gpadc,
		u8 channel, bool mode, int *value)
{
	int ret = 0;
	int adcm = (mode) ?
		AB850X_AUDIO_ADCM_FORCE_UP :
		AB850X_AUDIO_ADCM_FORCE_DOWN;

	if (ab850x_codec == NULL) {
		pr_err("%s: ERROR: ab850x ASoC-driver not yet probed!\n",
			__func__);
		return -EFAULT;
	}

	pr_debug("%s: Enter.\n", __func__);

	mutex_lock(&adcm_lock);

	ret = ux500_ab850x_power_control_inc(ab850x_codec);
	if (ret < 0) {
		pr_err("%s: ERROR: Failed to enable power (ret = %d)!\n",
			__func__, ret);
		goto power_failure;
	}

	ret = ab850x_audio_set_adcm(ab850x_codec, adcm);
	if (ret < 0) {
		pr_err("%s: ERROR: Failed to force adcm %s (ret = %d)!\n",
			__func__, (mode) ? "UP" : "DOWN", ret);
		goto adcm_failure;
	}

	ret = gpadc_convert_stable(gpadc, channel, value);
	ret |= ab850x_audio_set_adcm(ab850x_codec, AB850X_AUDIO_ADCM_NORMAL);

adcm_failure:
	ux500_ab850x_power_control_dec(ab850x_codec);

power_failure:
	mutex_unlock(&adcm_lock);

	pr_debug("%s: Exit.\n", __func__);

	return ret;
}

void ux500_ab850x_jack_report(int value)
{
	if (jack.jack)
		snd_soc_jack_report(&jack, value, 0xFF);
}
EXPORT_SYMBOL_GPL(ux500_ab850x_jack_report);
