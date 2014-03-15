/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * Author: Mikko J. Lehto <mikko.lehto@symbio.com>,
 *         Mikko Sarmanne <mikko.sarmanne@symbio.com>,
 *         Jarmo K. Kuronen <jarmo.kuronen@symbio.com>,
 *         Ola Lilja <ola.o.lilja@stericsson.com>,
 *         Joris Gignoux <joris.gignoux@st.com>,
 *         Arach Mohammed Brahim <arach.mohammed.brahim@st.com>
 *         for ST-Ericsson.
 *
 * License terms:
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/mfd/abx500.h>
#include <linux/mfd/abx500/ux500_sysctrl.h>
#include <linux/mfd/dbx500-prcmu.h>
#include "ab8540_audio.h"

/* To convert register definition shifts to masks */
#define BMASK(bsft)	(1 << (bsft))

/* Macrocell value definitions */
#define CLK_32K_OUT2_DISABLE			0x01
#define INACTIVE_RESET_AUDIO			0x02
#define ENABLE_AUDIO_CLK_TO_AUDIO_BLK		0x10
#define ENABLE_VINTCORE12_SUPPLY		0x04
#define GPIO27_DIR_OUTPUT			0x04
#define GPIO29_DIR_OUTPUT			0x10
#define GPIO31_DIR_OUTPUT			0x40

#define GPIO7_sel 0x20
#define GPIO2_sel 0x40
#define GPIO3_sel 0x10

#define AB8540_GPIO_SEL7_REG 0x06
#define AB8540_GPIO_DIR7_REG 0x16
#define AB8540_GPIO_OUT7_REG 0x26
#define AB8540_GPIO_PUD7_REG 0x36

#define AB8540_GPIO_SEL2_REG 0x01
#define AB8540_GPIO_DIR2_REG 0x11
#define AB8540_GPIO_OUT2_REG 0x21
#define AB8540_GPIO_PUD2_REG 0x31

/* Macrocell register definitions */
#define AB8500_CTRL3_REG			0x0200
#define AB8500_GPIO_DIR4_REG			0x1013

/* Nr of FIR/IIR-coeff banks in ANC-block */
#define AB8540_NR_OF_ANC_COEFF_BANKS		2

/*
 * Minimum duration to keep ANC IIR Init bit high or
 * low before proceeding with the configuration sequence
 */
#define AB8540_ANC_SM_DELAY		2000

/*
 * AB8540 register cache & default register settings
 */
static const u8 ab8540_reg_cache[AB8540_CACHEREGNUM] = {
	0x00, /* REG_POWERUP		(0x00) */
	0x00, /* REG_AUDSWRESET		(0x01) */
	0x00, /* REG_ADPATHENA		(0x02) */
	0x00, /* REG_DAPATHENA		(0x03) */
	0x00, /* REG_ANACONF1		(0x04) */
	0x0F, /* REG_ANACONF2		(0x05) */
	0x00, /* REG_DIGMICCONF		(0x06) */
	0x00, /* REG_ANACONF3		(0x07) */
	0x00, /* REG_ANACONF4		(0x08) */
	0x00, /* REG_DAPATHCONF		(0x09) */
	0x40, /* REG_MUTECONF		(0x0A) */
	0x00, /* REG_SHORTCIRCONF	(0x0B) */
	0x01, /* REG_ANACONF5		(0x0C) */
	0x00, /* REG_ENVCPCONF		(0x0D) */
	0x00, /* REG_SIGENVCONF		(0x0E) */
	0x00, /* NOT USED REG_PWMGENCONF1	(0x0F) */
	0x00, /* NOT USED REG_PWMGENCONF2	(0x10) */
	0x00, /* NOT USED REG_PWMGENCONF3	(0x11) */
	0x00, /* NOT USED REG_PWMGENCONF4	(0x12) */
	0x00, /* NOT USED REG_PWMGENCONF5	(0x13) */
	0x0F, /* REG_ANAGAIN1		(0x14) */
	0x0F, /* REG_ANAGAIN2		(0x15) */
	0x22, /* REG_ANAGAIN3		(0x16) */
	0x55, /* REG_ANAGAIN4		(0x17) */
	0x13, /* REG_DIGLINHSLGAIN	(0x18) */
	0x13, /* REG_DIGLINHSRGAIN	(0x19) */
	0x00, /* REG_ADFILTCONF		(0x1A) */
	0x00, /* REG_DIGIFCONF1		(0x1B) */
	0x02, /* REG_DIGIFCONF2		(0x1C) */
	0x00, /* REG_DIGIFCONF3		(0x1D) */
	0x02, /* REG_DIGIFCONF4		(0x1E) */
	0xCC, /* REG_ADSLOTSEL1		(0x1F) */
	0xCC, /* REG_ADSLOTSEL2		(0x20) */
	0xCC, /* REG_ADSLOTSEL3		(0x21) */
	0xCC, /* REG_ADSLOTSEL4		(0x22) */
	0xCC, /* REG_ADSLOTSEL5		(0x23) */
	0xCC, /* REG_ADSLOTSEL6		(0x24) */
	0xCC, /* REG_ADSLOTSEL7		(0x25) */
	0xCC, /* REG_ADSLOTSEL8		(0x26) */
	0xCC, /* REG_ADSLOTSEL9		(0x27) */
	0xCC, /* REG_ADSLOTSEL10	(0x28) */
	0xCC, /* REG_ADSLOTSEL11	(0x29) */
	0xCC, /* REG_ADSLOTSEL12	(0x2A) */
	0xCC, /* REG_ADSLOTSEL13	(0x2B) */
	0xCC, /* REG_ADSLOTSEL14	(0x2C) */
	0xCC, /* REG_ADSLOTSEL15	(0x2D) */
	0xCC, /* REG_ADSLOTSEL16	(0x2E) */
	0x00, /* REG_ADSLOTHIZCTRL1	(0x2F) */
	0x00, /* REG_ADSLOTHIZCTRL2	(0x30) */
	0x00, /* REG_ADSLOTHIZCTRL3	(0x31) */
	0x00, /* REG_ADSLOTHIZCTRL4	(0x32) */
	0x08, /* REG_DASLOTCONF1	(0x33) */
	0x08, /* REG_DASLOTCONF2	(0x34) */
	0x08, /* REG_DASLOTCONF3	(0x35) */
	0x08, /* REG_DASLOTCONF4	(0x36) */
	0x08, /* REG_DASLOTCONF5	(0x37) */
	0x08, /* REG_DASLOTCONF6	(0x38) */
	0x08, /* REG_DASLOTCONF7	(0x39) */
	0x08, /* REG_DASLOTCONF8	(0x3A) */
	0x00, /* REG_CLASSDCONF1	(0x3B) */
	0x00, /* REG_CLASSDCONF2	(0x3C) */
	0x84, /* REG_CLASSDCONF3	(0x3D) */
	0x00, /* REG_DMICFILTCONF	(0x3E) */
	0xFE, /* REG_DIGMULTCONF1	(0x3F) */
	0xC0, /* REG_DIGMULTCONF2	(0x40) */
	0x3F, /* REG_ADDIGGAIN1		(0x41) */
	0x3F, /* REG_ADDIGGAIN2		(0x42) */
	0x1F, /* REG_ADDIGGAIN3		(0x43) */
	0x1F, /* REG_ADDIGGAIN4		(0x44) */
	0x3F, /* REG_ADDIGGAIN5		(0x45) */
	0x3F, /* REG_ADDIGGAIN6		(0x46) */
	0x1F, /* REG_DADIGGAIN1		(0x47) */
	0x1F, /* REG_DADIGGAIN2		(0x48) */
	0x3F, /* REG_DADIGGAIN3		(0x49) */
	0x3F, /* REG_DADIGGAIN4		(0x4A) */
	0x3F, /* REG_DADIGGAIN5		(0x4B) */
	0x3F, /* REG_DADIGGAIN6		(0x4C) */
	0x3F, /* REG_ADDIGLOOPGAIN1	(0x4D) */
	0x3F, /* REG_ADDIGLOOPGAIN2	(0x4E) */
	0x00, /* REG_HSLEARDIGGAIN	(0x4F) */
	0x00, /* REG_HSRDIGGAIN		(0x50) */
	0x1F, /* REG_SIDFIRGAIN1	(0x51) */
	0x1F, /* REG_SIDFIRGAIN2	(0x52) */
	0x00, /* REG_ANCCONF1		(0x53) */
	0x00, /* REG_ANCCONF2		(0x54) */
	0x00, /* REG_ANCCONF3		(0x55) */
	0x00, /* REG_ANCCONF4		(0x56) */
	0x00, /* REG_ANCCONF5		(0x57) */
	0x00, /* REG_ANCCONF6		(0x58) */
	0x00, /* REG_ANCCONF7		(0x59) */
	0x00, /* REG_ANCCONF8		(0x5A) */
	0x00, /* REG_ANCCONF9		(0x5B) */
	0x00, /* REG_ANCCONF10		(0x5C) */
	0x00, /* REG_ANCCONF11		(0x5D) - read only */
	0x00, /* REG_ANCCONF12		(0x5E) - read only */
	0x00, /* REG_ANCCONF13		(0x5F) - read only */
	0x00, /* REG_ANCCONF14		(0x60) - read only */
	0x00, /* REG_SIDFIRADR		(0x61) */
	0x00, /* REG_SIDFIRCOEF1	(0x62) */
	0x00, /* REG_SIDFIRCOEF2	(0x63) */
	0x00, /* REG_SIDFIRCONF		(0x64) */
	0x00, /* REG_AUDINTMASK1	(0x65) */
	0x00, /* REG_AUDINTSOURCE1	(0x66) - read only */
	0x00, /* REG_AUDINTMASK2	(0x67) */
	0x00, /* REG_AUDINTSOURCE2	(0x68) - read only */
	0x00, /* REG_FIFOCONF1		(0x69) */
	0x00, /* REG_FIFOCONF2		(0x6A) */
	0x00, /* REG_FIFOCONF3		(0x6B) */
	0x00, /* REG_FIFOCONF4		(0x6C) */
	0x00, /* REG_FIFOCONF5		(0x6D) */
	0x00, /* REG_FIFOCONF6		(0x6E) */
	0x02, /* REG_AUDREV		(0x6F) - read only */
	0x00,  /* REG_EPWM1CONF      (0x70) */
	0x00,  /* REG_EPWM2CONF      (0x71) */
	0x00,  /* REG_DMICFREQ       (0x72) */
	0x00,  /* NOT USED REG_USBHSGAIN      (0x73) */
	0x00,  /* REG_USBDRVCTRL     (0x74) */
	0x00,  /* REG_EARGAINMICSEL  (0x75) */
	0x00,  /* REG_DIGMULTCONF3   (0x76) */
	0x00,  /* REG_PWMEMICTRL     (0x77) */
	0x00,  /* NOT USED REG_HFGAINCTRL     (0x78) */
	0x00,  /* NOT USED REG_VIBGAINCTRL    (0x79) */
	0x32,  /* REG_EPWMACCONF1    (0x7A) */
	0x32,  /* REG_EPWMACCONF2    (0x7B) */
	0x00,  /* REG_EPWMACCONF3    (0x7C) */
	0x32,  /* REG_EPWMACCONF4    (0x7D) */
	0x32,  /* REG_EPWMACCONF5    (0x7E) */
	0x00,  /* REG_EPWMACCONF6    (0x7F) */
	0x1F,  /* REG_HFGAIN         (0x80) */
	0x1F,  /* REG_VIBGAIN        (0x81) */
	0x00,  /* RESERVED           (0x82) */
	0x00,  /* REG_DA78MIX        (0x83) */
	0x00,  /* REG_DIGIFCONF5     (0x84) */
	0x00,  /* REG_DIGIFCONF6     (0x85) */
	0xCC,  /* REG_TXSLOTSEL1     (0x86) */
	0xCC,  /* REG_TXSLOTSEL2     (0x87) */
	0xCC,  /* REG_TXSLOTSEL3     (0x88) */
	0xCC,  /* REG_TXSLOTSEL4     (0x89) */
	0xCC,  /* REG_TXSLOTSEL5     (0x8A) */
	0xCC,  /* REG_TXSLOTSEL6     (0x8B) */
	0xCC,  /* REG_TXSLOTSEL7     (0x8C) */
	0xCC,  /* REG_TXSLOTSEL8     (0x8D) */
	0x00,  /* REG_ADSLOTHIZCTRL5 (0x8E) */
	0x00,  /* REG_ADSLOTHIZCTRL6 (0x8F) */
	0x00,  /* RESERVED           (0x90) */
	0x00,  /* REG_RXSLOTCONF     (0x91) */
	0x00,  /* REG_TXPATHENA      (0x92) */
	0x00,  /* REG_DIGMULTCONF4   (0x93) */
	0x00,  /* REG_AUDINTMASK3    (0x94) */
	0x00,  /* REG_AUDINTSOURCE3  (0x95) */
	0x00,  /* REG_TXRXFILTCONF   (0x96) */
	0x00,  /* REG_TXRXSIDETONE1  (0x97) */
	0x00,  /* REG_TXRXSIDETONE2  (0x98) */
	0x00,  /* REG_TXDGAIN1       (0x99) */
	0x00,  /* REG_TXDGAIN2       (0x9A) */
	0x00,  /* REG_TXDGAIN3       (0x9B) */
	0x00,  /* REG_TXDGAIN4       (0x9C) */
	0x00,  /* REG_RXDGAIN        (0x9D) */
	0x00,  /* REG_PNCPCONF       (0x9E) */
	0x00,  /* REG_MIC3CONFIG     (0x9F) */
};

static struct snd_soc_codec *ab8540_codec;

/* ADCM */
static const u8 ADCM_ANACONF5_MASK = BMASK(REG_ANACONF5_ENCPHS);
static const u8 ADCM_MUTECONF_MASK = BMASK(REG_MUTECONF_MUTHSL) |
		BMASK(REG_MUTECONF_MUTHSR);
static const u8 ADCM_ANACONF4_MASK = BMASK(REG_ANACONF4_ENHSL) |
		BMASK(REG_ANACONF4_ENHSR);
static unsigned int adcm_anaconf5, adcm_muteconf, adcm_anaconf4;
static int adcm = AB8540_AUDIO_ADCM_NORMAL;

/* Signed multi register array controls. */
struct soc_smra_control {
	unsigned int *reg;
	const unsigned int rcount, count, invert;
	long min, max;
	const char **texts;
	long *values;
};

/* Sidetone FIR-coeff cache */
static long sid_fir_cache[REG_SID_FIR_COEFFS];

/* ANC FIR- & IIR-coeff caches */
static long anc_fir_cache[REG_ANC_FIR_COEFFS];
static long anc_iir_cache[REG_ANC_IIR_COEFFS];

/* Reads an arbitrary register from the ab8540 chip.*/
static int ab8540_codec_read_reg(struct snd_soc_codec *codec, unsigned int bank,
		unsigned int reg)
{
	u8 value;
	int status = abx500_get_register_interruptible(
		codec->dev, bank, reg, &value);

	if (status < 0) {
		pr_err("%s: Register (%02x:%02x) read failed (%d).\n",
			__func__, (u8)bank, (u8)reg, status);
	} else {
		pr_debug("Read 0x%02x from register %02x:%02x\n",
			(u8)value, (u8)bank, (u8)reg);
		status = value;
	}

	return status;
}

/* Writes an arbitrary register to the ab8540 chip.*/
static int ab8540_codec_write_reg(struct snd_soc_codec *codec,
		unsigned int bank,
		unsigned int reg, unsigned int value)
{
	int status = abx500_set_register_interruptible(
		codec->dev, bank, reg, value);

	if (status < 0) {
		pr_err("%s: Register (%02x:%02x) write failed (%d).\n",
			__func__, (u8)bank, (u8)reg, status);
	} else {
		pr_debug("Wrote 0x%02x into register %02x:%02x\n",
			(u8)value, (u8)bank, (u8)reg);
	}

	return status;
}

/* Reads an audio register from the cache.*/
static unsigned int ab8540_codec_read_reg_audio(struct snd_soc_codec *codec,
		unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg == REG_SIDFIRCONF)
		return ab8540_codec_read_reg(codec, AB8500_AUDIO, reg);

	return cache[reg];
}

/* Writes an audio register to the hardware and cache.*/
static int ab8540_codec_write_reg_audio(struct snd_soc_codec *codec,
		unsigned int reg, unsigned int value)
{
	u8 *cache = codec->reg_cache;
	int status = ab8540_codec_write_reg(codec, AB8500_AUDIO, reg, value);

	if (status >= 0)
		cache[reg] = value;

	return status;

}

/*
 * Updates an audio register.
 *
 * Returns 1 for change, 0 for no change, or negative error code.
 */
static inline int ab8540_codec_update_reg_audio(struct snd_soc_codec *codec,
		unsigned int reg, unsigned int clr, unsigned int ins)
{
	unsigned int new, old;
	int ret;

	old = ab8540_codec_read_reg_audio(codec, reg);
	new = (old & ~clr) | ins;
	if (old == new)
		return 0;

	ret = ab8540_codec_write_reg_audio(codec, reg, new);

	return (ret < 0) ? ret : 1;
}

/* Generic soc info for signed register controls. */
static int snd_soc_info_s(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_smra_control *smra =
		(struct soc_smra_control *)kcontrol->private_value;
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = smra->count;
	uinfo->value.integer.min = smra->min;
	uinfo->value.integer.max = smra->max;

	return 0;
}

/* Generic soc get for signed multi register controls. */
static int snd_soc_get_smr(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_smra_control *smra =
		(struct soc_smra_control *)kcontrol->private_value;
	unsigned int *reg = smra->reg;
	unsigned int rcount = smra->rcount;
	long min = smra->min;
	long max = smra->max;
	unsigned int invert = smra->invert;
	unsigned long mask = abs(min) | abs(max);
	long value = 0;
	int i, rvalue;

	for (i = 0; i < rcount; i++) {
		rvalue = snd_soc_read(codec, reg[i]) & REG_MASK_ALL;
		value |= rvalue << (8 * (rcount - i - 1));
	}
	value &= mask;
	if (min < 0 && value > max)
		value |= ~mask;
	if (invert)
		value = ~value;
	ucontrol->value.integer.value[0] = value;

	return 0;
}

/* Generic soc put for signed multi register controls. */
static int snd_soc_put_smr(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_smra_control *smra =
		(struct soc_smra_control *)kcontrol->private_value;
	unsigned int *reg = smra->reg;
	unsigned int rcount = smra->rcount;
	long min = smra->min;
	long max = smra->max;
	unsigned int invert = smra->invert;
	unsigned long mask = abs(min) | abs(max);
	long value = ucontrol->value.integer.value[0];
	int i, rvalue, err;

	if (invert)
		value = ~value;
	if (value > max)
		value = max;
	else if (value < min)
		value = min;
	value &= mask;
	for (i = 0; i < rcount; i++) {
		rvalue = (value >> (8 * (rcount - i - 1))) & REG_MASK_ALL;
		err = snd_soc_write(codec, reg[i], rvalue);
		if (err < 0)
			return 0;
	}

	return 1;
}

/* Generic soc get for signed array controls. */
static int snd_soc_get_sa(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_smra_control *smra =
		(struct soc_smra_control *)kcontrol->private_value;
	long *values = smra->values;
	unsigned int count = smra->count;
	unsigned int idx;

	for (idx = 0; idx < count; idx++)
		ucontrol->value.integer.value[idx] = values[idx];

	return 0;
}

/* Generic soc put for signed array controls. */
static int snd_soc_put_sa(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_smra_control *smra =
			(struct soc_smra_control *) kcontrol->private_value;
	long *values = smra->values;
	unsigned int count = smra->count;
	long min = smra->min;
	long max = smra->max;
	unsigned int idx;
	long value;

	for (idx = 0; idx < count; idx++) {
		value = ucontrol->value.integer.value[idx];
		if (value > max)
			value = max;
		else if (value < min)
			value = min;
		values[idx] = value;
	}

	return 0;
}

static const char * const enum_ena_dis[] = {"Enabled", "Disabled"};
static const char * const enum_dis_ena[] = {"Disabled", "Enabled"};
static const char * const enum_rdy_apl[] = {"Ready", "Apply"};

/* Sidetone FIR-coefficients configuration sequence */
static int sid_apply_control_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec;
	unsigned int param, sidconf;
	int ret = 0;

	pr_debug("%s: Enter\n", __func__);

	if (ucontrol->value.integer.value[0] != 1) {
		pr_err("%s: ERROR: This control supports 'Apply' only!\n",
			__func__);
		return ret;
	}

	codec = snd_kcontrol_chip(kcontrol);

	mutex_lock(&codec->mutex);

	sidconf = snd_soc_read(codec, REG_SIDFIRCONF);
	if (((sidconf & BMASK(REG_SIDFIRCONF_FIRSIDBUSY)) != 0)) {
		if ((sidconf & BMASK(REG_SIDFIRCONF_ENFIRSIDS)) == 0) {
			pr_err("%s: Sidetone busy while off. Resetting...\n",
				__func__);
			snd_soc_update_bits(codec, REG_SIDFIRADR,
				REG_MASK_NONE, BMASK(REG_SIDFIRADR_FIRSIDSET));
			snd_soc_update_bits(codec, REG_SIDFIRADR,
				BMASK(REG_SIDFIRADR_FIRSIDSET), REG_MASK_NONE);
		}
		ret = -EBUSY;
		goto out;
	}

	snd_soc_write(codec, REG_SIDFIRADR, REG_MASK_NONE);

	for (param = 0; param < REG_SID_FIR_COEFFS; param++) {
		snd_soc_write(codec, REG_SIDFIRCOEF1,
			sid_fir_cache[param] >> 8 & REG_MASK_ALL);
		snd_soc_write(codec, REG_SIDFIRCOEF2,
			sid_fir_cache[param] & REG_MASK_ALL);
	}

	snd_soc_update_bits(codec, REG_SIDFIRADR,
		REG_MASK_NONE, BMASK(REG_SIDFIRADR_FIRSIDSET));
	snd_soc_update_bits(codec, REG_SIDFIRADR,
		BMASK(REG_SIDFIRADR_FIRSIDSET), REG_MASK_NONE);

	ret = 1;
out:
	mutex_unlock(&codec->mutex);

	pr_debug("%s: Exit\n", __func__);

	return ret;
}

static int digital_mute_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int reg = ab8540_codec_read_reg_audio(ab8540_codec, REG_DAPATHENA);

	ucontrol->value.enumerated.item[0] =
			(reg & (BMASK(REG_DAPATHENA_ENDA1) |
				BMASK(REG_DAPATHENA_ENDA2) |
				BMASK(REG_DAPATHENA_ENDA3) |
				BMASK(REG_DAPATHENA_ENDA4))) > 0;

	return 0;
}

static int digital_mute_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	unsigned int set_mask_hs	= 0;
	unsigned int clear_mask_hs	= 0;

	if (ucontrol->value.enumerated.item[0] != 0) {
		set_mask_hs =   BMASK(REG_DAPATHENA_ENDA1) |
				BMASK(REG_DAPATHENA_ENDA2) |
				BMASK(REG_DAPATHENA_ENDA3) |
				BMASK(REG_DAPATHENA_ENDA4);
	} else {
		clear_mask_hs = BMASK(REG_DAPATHENA_ENDA1) |
				BMASK(REG_DAPATHENA_ENDA2) |
				BMASK(REG_DAPATHENA_ENDA3) |
				BMASK(REG_DAPATHENA_ENDA4);
	}

	ret = ab8540_codec_update_reg_audio(ab8540_codec,
		REG_DAPATHENA,
		clear_mask_hs,
		set_mask_hs);

	if (ret < 0) {
		pr_err("%s: ERROR: Failed to change digital mute (%d)!\n",
		__func__, ucontrol->value.enumerated.item[0]);
		return 0;
	}

	pr_debug("%s: Digital mute set to %d\n",
		__func__, ucontrol->value.enumerated.item[0]);

	return 1;
}

static const char * const enum_chipid[] = {
	"Unknown",
	"AB8500",
	"AB9540_V1", "AB9540_V2", "AB9540_V3",
	"AB8505_V1", "AB8505_V2", "AB8505_V3",
	"AB8540_V1"
};

static int chipid_control_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	ucontrol->value.enumerated.item[0] = 8;

	return 0;
}

static SOC_ENUM_SINGLE_EXT_DECL(soc_enum_chipid, enum_chipid);

static struct snd_kcontrol_new chipid_control = \
	SOC_ENUM_EXT("ChipId", soc_enum_chipid, chipid_control_get, NULL);

/* Controls - DAPM */

/* Inverted order - Ascending/Descending */
enum control_inversion {
	NORMAL = 0,
	INVERT = 1
};

/* Headset */

/* Headset Left - Enable/Disable */
static const struct soc_enum enum_headset_left = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_headset_left_mux =
				SOC_DAPM_ENUM_VIRT("Headset Left",
						enum_headset_left);

/* Headset Right - Enable/Disable */
static const struct soc_enum enum_headset_right = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_headset_right_mux =
				SOC_DAPM_ENUM_VIRT("Headset Right",
						enum_headset_right);


/* CKLOL switch control */
static const struct snd_kcontrol_new dapm_ckl_dm[] = {
	SOC_DAPM_SINGLE("CKL switch", REG_USBCKLOCTRL,
			REG_USBCKLOCTRL_ENCKLOLDM, 1, NORMAL),
};

/* CKLOR switch control */
static const struct snd_kcontrol_new dapm_ckr_dp[] = {
	SOC_DAPM_SINGLE("CKR switch", REG_USBCKLOCTRL,
			REG_USBCKLOCTRL_ENCKLORDP, 1, NORMAL),
};

/* Earpiece */

/* Earpiece - Mute */
static const struct soc_enum enum_ear = SOC_ENUM_SINGLE(0, 0, 2, enum_dis_ena);
static const struct snd_kcontrol_new dapm_ear_mux =
				SOC_DAPM_ENUM_VIRT("Earpiece", enum_ear);

/* Earpiece source selector */
static const char * const enum_ear_lineout_source[] = {"Headset Left",
		"IHF"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_ear_lineout_source, REG_DMICFILTCONF,
			REG_DMICFILTCONF_DA3TOEAR, enum_ear_lineout_source);
static const struct snd_kcontrol_new dapm_ear_lineout_source[] = {
	SOC_DAPM_ENUM("Earpiece or LineOut Mono Source",
			dapm_enum_ear_lineout_source),
};

/* LineOut */

/* LineOut source selector */
static const char * const enum_lineout_source[] = {"Mono Path", "Stereo Path"};
static SOC_ENUM_DOUBLE_DECL(dapm_enum_lineout_source, REG_ANACONF5,
			REG_ANACONF5_HSLDACTOLOL, REG_ANACONF5_HSRDACTOLOR,
			enum_lineout_source);
static const struct snd_kcontrol_new dapm_lineout_source[] = {
	SOC_DAPM_ENUM("LineOut Source", dapm_enum_lineout_source),
};

/* LineOut */

/* LineOut Left - Enable/Disable */
static const struct soc_enum enum_lineout_left = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_lineout_left_mux =
				SOC_DAPM_ENUM_VIRT("LineOut Left",
						enum_lineout_left);

/* LineOut Right - Enable/Disable */
static const struct soc_enum enum_lineout_right = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_lineout_right_mux =
				SOC_DAPM_ENUM_VIRT("LineOut Right",
						enum_lineout_right);

/* LineOut/IHF - Select */
static const char * const enum_ihf_or_lineout_select_sel[] = {"IHF", "LineOut"};
static const struct soc_enum enum_ihf_or_lineout_select =
		SOC_ENUM_SINGLE(0, 0, 2, enum_ihf_or_lineout_select_sel);
static const struct snd_kcontrol_new dapm_ihf_or_lineout_select_mux =
				SOC_DAPM_ENUM_VIRT("IHF or LineOut Select",
						enum_ihf_or_lineout_select);

/* LineOut/VIB - Select */
static const char * const enum_vib_or_lineout_select_sel[] = {
		"Vibra", "LineOut"};
static const struct soc_enum enum_vib_or_lineout_select =
		SOC_ENUM_SINGLE(0, 0, 2, enum_vib_or_lineout_select_sel);
static const struct snd_kcontrol_new dapm_vib_or_lineout_select_mux =
				SOC_DAPM_ENUM_VIRT("Vibra or LineOut Select",
						enum_vib_or_lineout_select);

/* IHF */

/* IHF - Enable/Disable */
static const struct soc_enum enum_ihf = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_ihf_mux =
				SOC_DAPM_ENUM_VIRT("HandsFree", enum_ihf);

/* IHF - ANC selector */
static const char * const enum_ihf_sel[] = {"Audio Path", "ANC"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_ihf_sel, REG_DIGMULTCONF2,
			REG_DIGMULTCONF2_HFSEL, enum_ihf_sel);
static const struct snd_kcontrol_new dapm_ihf_select[] = {
	SOC_DAPM_ENUM("IHF Source", dapm_enum_ihf_sel),
};

/* DA3 or DA5 to HFl selector control */
static const char * const enum_da_hf_sel[] = {"DA3", "DA5"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_da_hf_sel, REG_DIGMULTCONF3,
		REG_DIGMULTCONF3_DA5TOHF, enum_da_hf_sel);
static const struct snd_kcontrol_new dapm_da_hf_select[] = {
	SOC_DAPM_ENUM("DA input for IHF", dapm_enum_da_hf_sel),
};

static const char * const enum_epwm2tohf[] = {"PWM-HF", "EPWM2"};

static SOC_ENUM_SINGLE_DECL(dapm_enum_epwm2tohf, REG_EPWM2CONF,
		REG_EPWM2CONF_EPWM2TOHF, enum_epwm2tohf);

static const struct snd_kcontrol_new dapm_epwm2tohf[] = {
	SOC_DAPM_ENUM("IHF Controller", dapm_enum_epwm2tohf),
};

/* PDM */

/* PDM - Enable/Disable */
static const struct soc_enum enum_pdm1 = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_pdm1_mux =
				SOC_DAPM_ENUM_VIRT("PDM 1", enum_pdm1);

static const struct soc_enum enum_pdm2 = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_pdm2_mux =
				SOC_DAPM_ENUM_VIRT("PDM 2", enum_pdm2);

/* Vibra */

/* Vibra 1 - Enable/Disable */
static const struct soc_enum enum_ihf_right = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_ihf_right_mux =
				SOC_DAPM_ENUM_VIRT("Vibra", enum_ihf_right);


/* VIBRA - ANC selector */
static SOC_ENUM_SINGLE_DECL(dapm_enum_vibra_sel, REG_DIGMULTCONF2,
			REG_DIGMULTCONF2_VIBSEL, enum_ihf_sel);
static const struct snd_kcontrol_new dapm_vibra_select[] = {
	SOC_DAPM_ENUM("Vibra Source", dapm_enum_vibra_sel),
};

/* DA1 or DA4/DA6 to Vibra selector control */
static const char * const enum_da_vib1_sel[] = {"DA4/DA6", "DA1"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_da_vib1_sel, REG_DIGMULTCONF3,
		REG_DIGMULTCONF3_DA1TOVIB, enum_da_vib1_sel);
static const struct snd_kcontrol_new dapm_da_vib1_select[] = {
	SOC_DAPM_ENUM("DA first input selection for VIBRA",
			dapm_enum_da_vib1_sel),
};

/* DA6 or DA4 to Vibra selector control */
static const char * const enum_da_vib2_sel[] = {"DA4", "DA6"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_da_vib2_sel, REG_DIGMULTCONF3,
		REG_DIGMULTCONF3_DA6TOVIB, enum_da_vib2_sel);
static const struct snd_kcontrol_new dapm_da_vib2_select[] = {
	SOC_DAPM_ENUM("DA second input selection for VIBRA",
			dapm_enum_da_vib2_sel),
};

static const char * const enum_epwm1tovib[] = {"PWM-VIB", "EPWM1"};

static SOC_ENUM_SINGLE_DECL(dapm_enum_epwm1tovib, REG_EPWM1CONF,
		REG_EPWM1CONF_EPWM1TOVIB, enum_epwm1tovib);

static const struct snd_kcontrol_new dapm_epwm1tovib[] = {
	SOC_DAPM_ENUM("Vibra Controller", dapm_enum_epwm1tovib),
};

/* FM Mixing */

static const struct snd_kcontrol_new dapm_da7_to_da1_switch[] = {
	SOC_DAPM_SINGLE("Playback Switch"
		, REG_DA78MIX, REG_DA78MIX_DA7ADDDA1, 1, NORMAL),
};
static const struct snd_kcontrol_new dapm_da8_to_da2_switch[] = {
	SOC_DAPM_SINGLE("Playback Switch"
		, REG_DA78MIX, REG_DA78MIX_DA8ADDDA2, 1, NORMAL),
};
static const struct snd_kcontrol_new dapm_da7_to_da3_switch[] = {
	SOC_DAPM_SINGLE("Playback Switch"
		, REG_DA78MIX, REG_DA78MIX_DA7ADDDA3, 1, NORMAL),
};
static const struct snd_kcontrol_new dapm_da8_to_da4_switch[] = {
	SOC_DAPM_SINGLE("Playback Switch"
		, REG_DA78MIX, REG_DA78MIX_DA8ADDDA4, 1, NORMAL),
};

/* Mic 1 */

/* Mic 1 - Mute */
static const struct soc_enum enum_mic1 = SOC_ENUM_SINGLE(0, 0, 2, enum_dis_ena);
static const struct snd_kcontrol_new dapm_mic1_mux =
				SOC_DAPM_ENUM_VIRT("Mic 1", enum_mic1);

/* Mic 1 - Mic 1A or 1B selector */
static const char * const enum_mic1ab_sel[] = {"Mic 1A", "Mic 1B"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_mic1ab_sel, REG_ANACONF3,
			REG_ANACONF3_MIC1SEL, enum_mic1ab_sel);
static const struct snd_kcontrol_new dapm_mic1ab_select[] = {
	SOC_DAPM_ENUM("Mic 1A or 1B Select", dapm_enum_mic1ab_sel),
};

/* Mic 1 - AD3 - Mic 1 or DMic 3 selector */
static const char * const enum_ad3_sel[] = {"Mic 1", "DMic 3"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_ad3_sel, REG_DIGMULTCONF1,
			REG_DIGMULTCONF1_AD3SEL, enum_ad3_sel);
static const struct snd_kcontrol_new dapm_ad3_select[] = {
	SOC_DAPM_ENUM("AD 3 Select", dapm_enum_ad3_sel),
};

/* Mic 1 - AD6 - Mic 1 or DMic 6 selector */
static const char * const enum_ad6_sel[] = {"Mic 1", "DMic 6"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_ad6_sel, REG_DIGMULTCONF1,
			REG_DIGMULTCONF1_AD6SEL, enum_ad6_sel);
static const struct snd_kcontrol_new dapm_ad6_select[] = {
	SOC_DAPM_ENUM("AD 6 Select", dapm_enum_ad6_sel),
};

/* Mic 2 */

/* Mic 2 - Mute */
static const struct soc_enum enum_mic2 = SOC_ENUM_SINGLE(0, 0, 2, enum_dis_ena);
static const struct snd_kcontrol_new dapm_mic2_mux =
				SOC_DAPM_ENUM_VIRT("Mic 2", enum_mic2);

/* Mic 2 - AD5 - Mic 2 or DMic 5 selector */
static const char * const enum_ad5_sel[] = {"Mic 2", "DMic 5"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_ad5_sel, REG_DIGMULTCONF1,
			REG_DIGMULTCONF1_AD5SEL, enum_ad5_sel);
static const struct snd_kcontrol_new dapm_ad5_select[] = {
	SOC_DAPM_ENUM("AD 5 Select", dapm_enum_ad5_sel),
};

/* Mic 3 */

/* Mic 3 - Mute */
static const struct soc_enum enum_mic3 = SOC_ENUM_SINGLE(0, 0, 2, enum_dis_ena);
static const struct snd_kcontrol_new dapm_mic3_mux =
				SOC_DAPM_ENUM_VIRT("Mic 3", enum_mic3);

/* LineIn */

/* LineIn left - Mute */
static const struct soc_enum enum_linl = SOC_ENUM_SINGLE(0, 0, 2, enum_dis_ena);
static const struct snd_kcontrol_new dapm_linl_mux =
				SOC_DAPM_ENUM_VIRT("LineIn Left", enum_linl);

static const struct snd_kcontrol_new dapm_linl_mute[] = {
	SOC_DAPM_SINGLE("Capture Switch", REG_ANACONF2,
			REG_ANACONF2_MUTLINL, 1, INVERT),
};

/* LineIn left - AD1 - LineIn Left or DMic 1 selector */
static const char * const enum_ad1_sel[] = {"LineIn Left", "DMic 1"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_ad1_sel, REG_DIGMULTCONF1,
			REG_DIGMULTCONF1_AD1SEL, enum_ad1_sel);
static const struct snd_kcontrol_new dapm_ad1_select[] = {
	SOC_DAPM_ENUM("AD 1 Select", dapm_enum_ad1_sel),
};

/* AD1 - RX1 Voice or AD1 Select output selector */
static const char * const enum_rx1_to_ad1_sel[] = {"AD1", "RX1"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_rx1_to_ad1_sel, REG_DIGMULTCONF4,
			REG_DIGMULTCONF4_RX1_TO_AD1_SEL, enum_rx1_to_ad1_sel);
static const struct snd_kcontrol_new dapm_rx1_to_ad1_select[] = {
	SOC_DAPM_ENUM("AD 1 Voice RX1 Select", dapm_enum_rx1_to_ad1_sel),
};

/* LineIn right - Mute */
static const struct soc_enum enum_linr = SOC_ENUM_SINGLE(0, 0, 2, enum_dis_ena);
static const struct snd_kcontrol_new dapm_linr_mux =
				SOC_DAPM_ENUM_VIRT("LineIn Right", enum_linr);

static const struct snd_kcontrol_new dapm_linr_mute[] = {
	SOC_DAPM_SINGLE("Capture Switch", REG_ANACONF2,
			REG_ANACONF2_MUTLINR, 1, INVERT),
};

/* LineIn - Mic 2/3 or LineIn Right/Left selector */
static const char * const enum_miclin_sel[] = {"Mic", "LineIn"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_miclin_sel, REG_ANACONF3,
			REG_ANACONF3_LINSEL, enum_miclin_sel);
static const struct snd_kcontrol_new dapm_miclin_select[] = {
	SOC_DAPM_ENUM("Mic or LIN Select", dapm_enum_miclin_sel),
};

/* LineIn right - AD2 - LineIn Right or DMic2 selector */
static const char * const enum_ad2_sel[] = {"LineIn Right", "DMic 2"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_ad2_sel, REG_DIGMULTCONF1,
			REG_DIGMULTCONF1_AD2SEL, enum_ad2_sel);
static const struct snd_kcontrol_new dapm_ad2_select[] = {
	SOC_DAPM_ENUM("AD 2 Select", dapm_enum_ad2_sel),
};

/* AD4 - RX1 Voice or AD4 Select output selector */
static const char * const enum_rx1_to_ad4_sel[] = {"AD4", "RX1"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_rx1_to_ad4_sel, REG_DIGMULTCONF4,
			REG_DIGMULTCONF4_RX1_TO_AD4_SEL, enum_rx1_to_ad4_sel);
static const struct snd_kcontrol_new dapm_rx1_to_ad4_select[] = {
	SOC_DAPM_ENUM("AD 4 Voice RX1 Select", dapm_enum_rx1_to_ad4_sel),
};

/* AD1 to DA3 (IHF) Switch */
static const struct soc_enum enum_ad1_to_ihf = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_ad1_to_ihf_mux =
	SOC_DAPM_ENUM_VIRT("AD1 to IHF", enum_ad1_to_ihf);

/* AD2 to DA4 (Vib) Switch */
static const struct soc_enum enum_ad2_to_vib = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_ad2_to_vib_mux =
	SOC_DAPM_ENUM_VIRT("AD2 to Vibra", enum_ad2_to_vib);

/* LineIn Left to Headset Left switch */
static const struct soc_enum enum_linl_to_hs_left = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_linl_to_hs_left_mux =
	SOC_DAPM_ENUM_VIRT("LineIn Left to Headset Left", enum_linl_to_hs_left);

/* LineIn Right to Headset Right switch */
static const struct soc_enum enum_linr_to_hs_right = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_linr_to_hs_right_mux =
	SOC_DAPM_ENUM_VIRT("LineIn Right to Headset Right",
			enum_linr_to_hs_right);

/* DMic */

/* DMic 1 - Mute */
static const struct soc_enum enum_dmic1 = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_dmic1_mux =
				SOC_DAPM_ENUM_VIRT("DMic 1", enum_dmic1);

/* DMic 2 - Mute */
static const struct soc_enum enum_dmic2 = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_dmic2_mux =
				SOC_DAPM_ENUM_VIRT("DMic 2", enum_dmic2);

/* DMic 3 - Mute */
static const struct soc_enum enum_dmic3 = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_dmic3_mux =
				SOC_DAPM_ENUM_VIRT("DMic 3", enum_dmic3);

/* DMic 4 - Mute */
static const struct soc_enum enum_dmic4 = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_dmic4_mux =
				SOC_DAPM_ENUM_VIRT("DMic 4", enum_dmic4);

/* DMic 5 - Mute */
static const struct soc_enum enum_dmic5 = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_dmic5_mux =
				SOC_DAPM_ENUM_VIRT("DMic 5", enum_dmic5);

/* DMic 6 - Mute */
static const struct soc_enum enum_dmic6 = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_dmic6_mux =
				SOC_DAPM_ENUM_VIRT("DMic 6", enum_dmic6);

/* ANC */

static const char * const enum_anc_in_sel[] = {"Mic 1 / DMic 6",
		"Mic 2 / DMic 5"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_anc_in_sel, REG_DMICFILTCONF,
			REG_DMICFILTCONF_ANCINSEL, enum_anc_in_sel);
static const struct snd_kcontrol_new dapm_anc_in_select[] = {
	SOC_DAPM_ENUM("ANC Source", dapm_enum_anc_in_sel),
};

/* ANC - Enable/Disable */
static SOC_ENUM_SINGLE_DECL(dapm_enum_anc_enable, REG_ANCCONF1,
			REG_ANCCONF1_ENANC, enum_dis_ena);
static const struct snd_kcontrol_new dapm_anc_enable[] = {
	SOC_DAPM_ENUM("ANC", dapm_enum_anc_enable),
};

/* ANC to Earpiece - Mute */
static const struct snd_kcontrol_new dapm_anc_ear_mute[] = {
	SOC_DAPM_SINGLE("Playback Switch", REG_DIGMULTCONF1,
			REG_DIGMULTCONF1_ANCSEL, 1, NORMAL),
};

/* Sidetone left */

/* Sidetone left - Input selector */
static const char * const enum_stfir1_in_sel[] = {
	"LineIn Left", "LineIn Right", "Mic 1", "Headset Left"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_stfir1_in_sel, REG_DIGMULTCONF2,
			REG_DIGMULTCONF2_FIRSID1SEL, enum_stfir1_in_sel);
static const struct snd_kcontrol_new dapm_stfir1_in_select[] = {
	SOC_DAPM_ENUM("Sidetone Left Source", dapm_enum_stfir1_in_sel),
};

/* Sidetone left - Enable/Disable */
static const struct soc_enum enum_stfir1_ena = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_stfir1_ena =
				SOC_DAPM_ENUM_VIRT("Sidetone Left",
						enum_stfir1_ena);

/* Sidetone right path */

/* Sidetone right - Input selector */
static const char * const enum_stfir2_in_sel[] = {
	"LineIn Right", "Mic 1", "DMic 4", "Headset Right"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_stfir2_in_sel, REG_DIGMULTCONF2,
			REG_DIGMULTCONF2_FIRSID2SEL, enum_stfir2_in_sel);
static const struct snd_kcontrol_new dapm_stfir2_in_select[] = {
	SOC_DAPM_ENUM("Sidetone Right Source", dapm_enum_stfir2_in_sel),
};

/* Sidetone right - Enable/Disable */
static const struct soc_enum enum_stfir2_ena = SOC_ENUM_SINGLE(0, 0, 2,
		enum_dis_ena);
static const struct snd_kcontrol_new dapm_stfir2_ena =
			SOC_DAPM_ENUM_VIRT("Sidetone Right", enum_stfir2_ena);

/* Voice Interface */

/* Voice Interface - Echo reference source */
static const char * const enum_voice_echoref_source[] = {
	"Headset", "IHF"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_voice_echoref_source, REG_DIGMULTCONF4,
		REG_DIGMULTCONF4_DA34_TO_TX34, enum_voice_echoref_source);
static const struct snd_kcontrol_new dapm_voice_echoref_source[] = {
	SOC_DAPM_ENUM("Voice Interface Echo Reference Source"
		, dapm_enum_voice_echoref_source),
};

/* Voice Interface - Rx and DAx mixing */
static const struct snd_kcontrol_new dapm_rx_to_hs_switch[] = {
	SOC_DAPM_SINGLE("Playback Switch"
		, REG_DIGMULTCONF4, REG_DIGMULTCONF4_RX1_TO_HS, 1, NORMAL),
};

static const struct snd_kcontrol_new dapm_rx_to_ihf_switch[] = {
	SOC_DAPM_SINGLE("Playback Switch"
		, REG_DIGMULTCONF4, REG_DIGMULTCONF4_RX1_TO_HF, 1, NORMAL),
};

/* Voice Interface - Voice Sidetone */
static const char * const enum_voice_sidetone_source[] = {
	"TX1", "TX2"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_voice_sidetone_source, REG_TXRXSIDETONE1,
			REG_TXRXSIDETONE1_ST_TX2, enum_voice_sidetone_source);
static const struct snd_kcontrol_new dapm_voice_sidetone_source[] = {
	SOC_DAPM_ENUM("Voice Interface Sidetone Source"
		, dapm_enum_voice_sidetone_source),
};

static const struct snd_kcontrol_new dapm_voice_direct_sidetone_switch[] = {
	SOC_DAPM_SINGLE("Switch"
		, REG_TXRXSIDETONE1, REG_TXRXSIDETONE1_ST_INT_ACT, 1, NORMAL),
};
/* Voice Interface - Voice Sidetone through STFIRS */
static const char * const enum_voice_sidetone_stfir[] = {"Disabled", "Enabled"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_voice_sidetone_stfir, REG_TXRXSIDETONE1,
		REG_TXRXSIDETONE1_ST_EXT_ACT, enum_voice_sidetone_stfir);
static const struct snd_kcontrol_new dapm_voice_sidetone_stfir[] = {
	SOC_DAPM_ENUM("Voice Interface Sidetone through STFIRS"
		, dapm_enum_voice_sidetone_stfir),
};

/* PWM */

static const char * const enum_pwmtomod[] = {"Audio Path", "EPWM Generator"};

static SOC_ENUM_SINGLE_DECL(dapm_enum_pwm1tomod1, REG_EPWMACCONF1,
		REG_EPWMACCONF1_EPWM1ACSEL, enum_pwmtomod);

static const struct snd_kcontrol_new dapm_pwm1tomod1[] = {
	SOC_DAPM_ENUM("EPWM1 Source", dapm_enum_pwm1tomod1),
};

static SOC_ENUM_SINGLE_DECL(dapm_enum_pwm2tomod2, REG_EPWMACCONF4,
		REG_EPWMACCONF4_EPWM2ACSEL, enum_pwmtomod);

static const struct snd_kcontrol_new dapm_pwm2tomod2[] = {
	SOC_DAPM_ENUM("EPWM2 Source", dapm_enum_pwm2tomod2),
};

/* PWM 1 - Enable/Disable */
static const struct soc_enum enum_pwm1 = SOC_ENUM_SINGLE(0, 0, 2, enum_dis_ena);
static const struct snd_kcontrol_new dapm_pwm1_mux =
				SOC_DAPM_ENUM_VIRT("PWM 1", enum_pwm1);
/* PWM 2 - Enable/Disable */
static const struct soc_enum enum_pwm2 = SOC_ENUM_SINGLE(0, 0, 2, enum_dis_ena);
static const struct snd_kcontrol_new dapm_pwm2_mux =
				SOC_DAPM_ENUM_VIRT("PWM 2", enum_pwm2);

/* Event-handlers - DAPM */

static int stfir_enable;
static int stfir_enable_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (!stfir_enable)
			snd_soc_update_bits(codec, REG_SIDFIRCONF,
					0, BMASK(REG_SIDFIRCONF_ENFIRSIDS));
		stfir_enable++;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		stfir_enable--;
		if (!stfir_enable)
			snd_soc_update_bits(codec, REG_SIDFIRCONF,
					BMASK(REG_SIDFIRCONF_ENFIRSIDS), 0);
		break;
	}
	return 0;
}

static int linein_enable_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
	case SND_SOC_DAPM_POST_PMD:
		msleep(LINEIN_RAMP_DELAY);
		break;
	}
	return 0;
}


static const struct snd_soc_dapm_widget ab8540_dapm_widgets[] = {

	/* DA/AD */

	SND_SOC_DAPM_INPUT("ADC Input"),
	SND_SOC_DAPM_ADC("ADC", "ab8540_0c", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC voice", "ab8540_voice_0c", SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_DAC("DAC", "ab8540_0p", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC voice", "ab8540_voice_0p", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("DAC Output"),

	SND_SOC_DAPM_AIF_IN("DA_IN1", "ab8540_0p", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DA_IN2", "ab8540_0p", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DA_IN3", "ab8540_0p", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DA_IN4", "ab8540_0p", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DA_IN5", "ab8540_0p", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DA_IN6", "ab8540_0p", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DA_IN7", "ab8540_0p", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DA_IN8", "ab8540_0p", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AD_OUT1", "ab8540_0c", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AD_OUT2", "ab8540_0c", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AD_OUT3", "ab8540_0c", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AD_OUT4", "ab8540_0c", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AD_OUT57", "ab8540_0c", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AD_OUT68", "ab8540_0c", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_IN("RX_IN1", "ab8540_voice_0p", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TX_OUT1", "ab8540_voice_0c", 0,
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TX_OUT2", "ab8540_voice_0c", 0,
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TX_OUT3", "ab8540_voice_0c", 0,
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TX_OUT4", "ab8540_voice_0c", 0,
			SND_SOC_NOPM, 0, 0),

	/* Headset & USHLR & CKL path */

	SND_SOC_DAPM_SUPPLY("Charge Pump", REG_ANACONF5, REG_ANACONF5_ENCPHS,
			NORMAL, NULL, 0),

	SND_SOC_DAPM_DAC("DA1 Enable", "ab8540_0p",
			REG_DAPATHENA, REG_DAPATHENA_ENDA1, 0),
	SND_SOC_DAPM_DAC("DA2 Enable", "ab8540_0p",
			REG_DAPATHENA, REG_DAPATHENA_ENDA2, 0),

	SND_SOC_DAPM_MUX("Headset Left",
			SND_SOC_NOPM, 0, 0, &dapm_headset_left_mux),
	SND_SOC_DAPM_MUX("Headset Right",
			SND_SOC_NOPM, 0, 0, &dapm_headset_right_mux),


	SND_SOC_DAPM_PGA("HSL Digital Gain", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HSR Digital Gain", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_DAC("HSL DAC", "ab8540_0p",
			REG_DAPATHCONF, REG_DAPATHCONF_ENDACHSL, 0),
	SND_SOC_DAPM_DAC("HSR DAC", "ab8540_0p",
			REG_DAPATHCONF, REG_DAPATHCONF_ENDACHSR, 0),
	SND_SOC_DAPM_MIXER("HSL DAC Mute", REG_MUTECONF, REG_MUTECONF_MUTDACHSL,
			INVERT, NULL, 0),
	SND_SOC_DAPM_MIXER("HSR DAC Mute", REG_MUTECONF, REG_MUTECONF_MUTDACHSR,
			INVERT, NULL, 0),
	SND_SOC_DAPM_DAC("HSL DAC Driver", "ab8540_0p",
			REG_ANACONF3, REG_ANACONF3_ENDRVHSL, 0),
	SND_SOC_DAPM_DAC("HSR DAC Driver", "ab8540_0p",
			REG_ANACONF3, REG_ANACONF3_ENDRVHSR, 0),

	SND_SOC_DAPM_MIXER("HSL Mute", REG_MUTECONF, REG_MUTECONF_MUTHSL,
			INVERT, NULL, 0),
	SND_SOC_DAPM_MIXER("HSR Mute", REG_MUTECONF, REG_MUTECONF_MUTHSR,
			INVERT, NULL, 0),

	SND_SOC_DAPM_MIXER("HSL Enable", REG_ANACONF4, REG_ANACONF4_ENHSL,
			NORMAL, NULL, 0),
	SND_SOC_DAPM_MIXER("HSR Enable", REG_ANACONF4, REG_ANACONF4_ENHSR,
			NORMAL, NULL, 0),

	SND_SOC_DAPM_SWITCH("CKL to DM", SND_SOC_NOPM,
		    0, 0, dapm_ckl_dm),
	SND_SOC_DAPM_SWITCH("CKR to DP", SND_SOC_NOPM,
		    0, 0, dapm_ckr_dp),

	SND_SOC_DAPM_MIXER("CKLoL Enable", REG_USBCKLOCTRL,
			REG_USBCKLOCTRL_ENCKLOL, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("CKLoR Enable", REG_USBCKLOCTRL,
			REG_USBCKLOCTRL_ENCKLOR, 0, NULL, 0),

	SND_SOC_DAPM_PGA("HSL Gain", SND_SOC_NOPM, 0,
			0, NULL, 0),
	SND_SOC_DAPM_PGA("HSR Gain", SND_SOC_NOPM, 0,
			0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("HSL"),
	SND_SOC_DAPM_OUTPUT("HSR"),

	SND_SOC_DAPM_OUTPUT("CKLoL"),
	SND_SOC_DAPM_OUTPUT("CKLoR"),

	/* LineOut path */

	SND_SOC_DAPM_MUX("LineOut Source Playback Route",
			SND_SOC_NOPM, 0, 0, dapm_lineout_source),

	SND_SOC_DAPM_MIXER("LOL Enable", REG_ANACONF5,
			REG_ANACONF5_ENLOL, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("LOR Enable", REG_ANACONF5,
			REG_ANACONF5_ENLOR, 0, NULL, 0),

	SND_SOC_DAPM_MUX("LineOut Left",
			SND_SOC_NOPM, 0, 0, &dapm_lineout_left_mux),

	SND_SOC_DAPM_MUX("LineOut Right",
			SND_SOC_NOPM, 0, 0, &dapm_lineout_right_mux),

	/* Earpiece path */

	SND_SOC_DAPM_MUX("Earpiece or LineOut Mono Source",
			SND_SOC_NOPM, 0, 0, dapm_ear_lineout_source),

	SND_SOC_DAPM_MIXER("EAR DAC", REG_DAPATHCONF,
			REG_DAPATHCONF_ENDACEAR, 0, NULL, 0),

	SND_SOC_DAPM_MUX("Earpiece", SND_SOC_NOPM, 0, 0, &dapm_ear_mux),

	SND_SOC_DAPM_MIXER("EAR Mute", REG_MUTECONF,
			REG_MUTECONF_MUTEAR, 1, NULL, 0),

	SND_SOC_DAPM_MIXER("EAR Enable", REG_ANACONF4,
			REG_ANACONF4_ENEAR, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("EAR"),

	/* Handsfree path */

	SND_SOC_DAPM_DAC("DA3 Enable", "ab8540_0p",
			REG_DAPATHENA, REG_DAPATHENA_ENDA3, 0),

	SND_SOC_DAPM_MUX("IHF Source Playback Route",
			SND_SOC_NOPM, 0, 0, dapm_ihf_select),


	SND_SOC_DAPM_MUX("HandsFree", SND_SOC_NOPM, 0, 0, &dapm_ihf_mux),

	SND_SOC_DAPM_PGA("HandsFree Digital Gain", SND_SOC_NOPM, 0, 0, NULL, 0),


	SND_SOC_DAPM_MUX("IHF or LineOut Select", SND_SOC_NOPM,
			0, 0, &dapm_ihf_or_lineout_select_mux),

	SND_SOC_DAPM_MUX("DA input selection for IHF",
			SND_SOC_NOPM, 0, 0, dapm_da_hf_select),

	SND_SOC_DAPM_MIXER("IHF DAC", REG_DAPATHCONF,
			REG_DAPATHCONF_ENDACHF, 0, NULL, 0),

	SND_SOC_DAPM_MUX("IHF Controller Playback Route",
			SND_SOC_NOPM, 0, 0, dapm_epwm2tohf),

	SND_SOC_DAPM_MIXER("DA3 or ANC path to Hf", REG_DIGMULTCONF2,
			REG_DIGMULTCONF2_DATOHFEN, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("IHF Enable", REG_ANACONF4,
			REG_ANACONF4_ENHF, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("IHF"),


	/* PDM path */

	SND_SOC_DAPM_MUX("PDM 1",
			SND_SOC_NOPM, 0, 0, &dapm_pdm1_mux),
	SND_SOC_DAPM_MUX("PDM 2",
			SND_SOC_NOPM, 0, 0, &dapm_pdm2_mux),

	SND_SOC_DAPM_MIXER("PDM1 Enable", REG_DIGMULTCONF3,
			REG_DIGMULTCONF3_ENPDM1, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("PDM2 Enable", REG_DIGMULTCONF3,
			REG_DIGMULTCONF3_ENPDM2, 0, NULL, 0),
	SND_SOC_DAPM_OUTPUT("PDM"),

	/* Vibrator path */


	SND_SOC_DAPM_DAC("DA4 Enable", "ab8540_0p",
			REG_DAPATHENA, REG_DAPATHENA_ENDA4, 0),

	SND_SOC_DAPM_MUX("Vibra Source Playback Route",
			SND_SOC_NOPM, 0, 0, dapm_vibra_select),

	SND_SOC_DAPM_MUX("DA first input selection for VIBRA",
			SND_SOC_NOPM, 0, 0, dapm_da_vib1_select),

	SND_SOC_DAPM_MUX("DA second input selection for VIBRA",
			SND_SOC_NOPM, 0, 0, dapm_da_vib2_select),

	SND_SOC_DAPM_MUX("Vibra or LineOut Select", SND_SOC_NOPM,
			0, 0, &dapm_vib_or_lineout_select_mux),

	SND_SOC_DAPM_MUX("Vibra", SND_SOC_NOPM, 0, 0, &dapm_ihf_right_mux),

	SND_SOC_DAPM_PGA("Vibra Digital Gain", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("Vibra DAC", REG_DAPATHCONF,
			REG_DAPATHCONF_ENDACVIB1, 0, NULL, 0),

	SND_SOC_DAPM_INPUT("EPWMGEN1"),
	SND_SOC_DAPM_INPUT("EPWMGEN2"),

	SND_SOC_DAPM_MUX("EPWM1 Source",
			SND_SOC_NOPM, 0, 0, dapm_pwm1tomod1),
	SND_SOC_DAPM_MUX("EPWM2 Source",
			SND_SOC_NOPM, 0, 0, dapm_pwm2tomod2),

	SND_SOC_DAPM_MUX("Vibra Controller Playback Route",
			SND_SOC_NOPM, 0, 0, dapm_epwm1tovib),

	SND_SOC_DAPM_MIXER("DA4 or ANC path to Vib", REG_DIGMULTCONF2,
			REG_DIGMULTCONF2_DATOVIBEN, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("Vibra Enable", REG_ANACONF4,
			REG_ANACONF4_ENVIB1, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("VIB"),

	/* PWM1 & PWM2 path */

	SND_SOC_DAPM_MUX("PWM 1", SND_SOC_NOPM, 0, 0, &dapm_pwm1_mux),
	SND_SOC_DAPM_MUX("PWM 2", SND_SOC_NOPM, 0, 0, &dapm_pwm2_mux),

	SND_SOC_DAPM_MIXER("DA5 Channel Gain", REG_DAPATHENA,
			REG_DAPATHENA_ENDA5, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("DA6 Channel Gain", REG_DAPATHENA,
			REG_DAPATHENA_ENDA6, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("EPWM1 Enable", REG_EPWM1CONF,
			REG_EPWM1CONF_EN, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("EPWM2 Enable", REG_EPWM2CONF,
			REG_EPWM2CONF_EN, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("EPWM1"),
	SND_SOC_DAPM_OUTPUT("EPWM2"),

	/* FM paths (DA7 &DA8) */

	SND_SOC_DAPM_SWITCH("DA 7 to DA 1", SND_SOC_NOPM, 0, 0,
			dapm_da7_to_da1_switch),
	SND_SOC_DAPM_SWITCH("DA 8 to DA 2", SND_SOC_NOPM, 0, 0,
			dapm_da8_to_da2_switch),
	SND_SOC_DAPM_SWITCH("DA 7 to DA 3", SND_SOC_NOPM, 0, 0,
			dapm_da7_to_da3_switch),
	SND_SOC_DAPM_SWITCH("DA 8 to DA 4", SND_SOC_NOPM, 0, 0,
			dapm_da8_to_da4_switch),


	/* LineIn & Microphone 2 and 3 path */

	SND_SOC_DAPM_INPUT("LINL"),
	SND_SOC_DAPM_INPUT("LINR"),
	SND_SOC_DAPM_INPUT("MIC2 Input"),
	SND_SOC_DAPM_INPUT("MIC3 Input"),

	SND_SOC_DAPM_MUX("LineIn Left", SND_SOC_NOPM, 0, 0,
			&dapm_linl_mux),
	SND_SOC_DAPM_MIXER("LINL Mute", REG_ANACONF2,
			REG_ANACONF2_MUTLINL, INVERT, NULL, 0),

	SND_SOC_DAPM_MUX("LineIn Right", SND_SOC_NOPM, 0, 0,
			&dapm_linr_mux),
	SND_SOC_DAPM_MIXER("LINR Mute", REG_ANACONF2,
			REG_ANACONF2_MUTLINR, INVERT, NULL, 0),

	SND_SOC_DAPM_MUX("Mic 2", SND_SOC_NOPM, 0, 0,
			&dapm_mic2_mux),
	SND_SOC_DAPM_MIXER("MIC2 Mute", REG_ANACONF2,
			REG_ANACONF2_MUTMIC2, INVERT, NULL, 0),

	SND_SOC_DAPM_MUX("Mic 3", SND_SOC_NOPM, 0, 0,
					&dapm_mic3_mux),
	SND_SOC_DAPM_MIXER("MIC3 Mute", REG_MIC3CONFIG,
			REG_MIC3CONFIG_MUTEMIC3, INVERT, NULL, 0),

	SND_SOC_DAPM_MIXER_E("LINL Enable", REG_ANACONF2,
			REG_ANACONF2_ENLINL, 0, NULL, 0,
			linein_enable_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("LINR Enable", REG_ANACONF2,
			REG_ANACONF2_ENLINR, 0, NULL, 0,
			linein_enable_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER("MIC2 Enable", REG_ANACONF2,
			REG_ANACONF2_ENMIC2, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("MIC3 Enable", REG_MIC3CONFIG,
			REG_MIC3CONFIG_ENMIC3, 0, NULL, 0),

	SND_SOC_DAPM_MUX("Mic or LIN Select Capture Route",
			SND_SOC_NOPM, 0, 0, dapm_miclin_select),

	SND_SOC_DAPM_MIXER("LINL ADC", REG_ANACONF3,
			REG_ANACONF3_ENADCLINL, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("LINR ADC", REG_ANACONF3,
			REG_ANACONF3_ENADCLINR, 0, NULL, 0),

	SND_SOC_DAPM_MUX("AD 1 Select Capture Route",
			SND_SOC_NOPM, 0, 0, dapm_ad1_select),
	SND_SOC_DAPM_MUX("AD 2 Select Capture Route",
			SND_SOC_NOPM, 0, 0, dapm_ad2_select),

	SND_SOC_DAPM_MUX("AD 1 Voice RX1 Select Capture Route",
			SND_SOC_NOPM, 0, 0, dapm_rx1_to_ad1_select),
	SND_SOC_DAPM_MUX("AD 4 Voice RX1 Select Capture Route",
			SND_SOC_NOPM, 0, 0, dapm_rx1_to_ad4_select),

	SND_SOC_DAPM_MIXER("AD1 Channel Gain", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("AD2 Channel Gain", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("AD12 Enable", REG_ADPATHENA,
			REG_ADPATHENA_ENAD12, 0, NULL, 0),

	/* Microphone 1 path */

	SND_SOC_DAPM_INPUT("MIC1A Input"),
	SND_SOC_DAPM_INPUT("MIC1B Input"),

	SND_SOC_DAPM_MUX("Mic 1A or 1B Select Capture Route",
			SND_SOC_NOPM, 0, 0, dapm_mic1ab_select),

	SND_SOC_DAPM_MUX("Mic 1", SND_SOC_NOPM, 0, 0, &dapm_mic1_mux),

	SND_SOC_DAPM_MIXER("MIC1 Mute", REG_ANACONF2,
			REG_ANACONF2_MUTMIC1, INVERT, NULL, 0),

	SND_SOC_DAPM_MIXER("MIC1 Enable", REG_ANACONF2,
			REG_ANACONF2_ENMIC1, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("MIC1 ADC", REG_ANACONF3,
			REG_ANACONF3_ENADCMIC, 0, NULL, 0),

	SND_SOC_DAPM_MUX("AD 3 Select Capture Route",
			SND_SOC_NOPM, 0, 0, dapm_ad3_select),

	SND_SOC_DAPM_MIXER("AD3 Channel Gain", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("AD3 Enable", REG_ADPATHENA,
			REG_ADPATHENA_ENAD34, 0, NULL, 0),

	/* HD Capture path */

	SND_SOC_DAPM_MUX("AD 5 Select Capture Route",
			SND_SOC_NOPM, 0, 0, dapm_ad5_select),
	SND_SOC_DAPM_MUX("AD 6 Select Capture Route",
			SND_SOC_NOPM, 0, 0, dapm_ad6_select),

	SND_SOC_DAPM_MIXER("AD5 Channel Gain", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("AD6 Channel Gain", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("AD57 Enable", REG_ADPATHENA,
			REG_ADPATHENA_ENAD5768, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("AD68 Enable", REG_ADPATHENA,
			REG_ADPATHENA_ENAD5768, 0, NULL, 0),

	/* Digital Microphone path */

	SND_SOC_DAPM_INPUT("DMIC Input"),

	SND_SOC_DAPM_MUX("DMic 1", SND_SOC_NOPM, 0, 0, &dapm_dmic1_mux),
	SND_SOC_DAPM_MUX("DMic 2", SND_SOC_NOPM, 0, 0, &dapm_dmic2_mux),
	SND_SOC_DAPM_MUX("DMic 3", SND_SOC_NOPM, 0, 0, &dapm_dmic3_mux),
	SND_SOC_DAPM_MUX("DMic 4", SND_SOC_NOPM, 0, 0, &dapm_dmic4_mux),
	SND_SOC_DAPM_MUX("DMic 5", SND_SOC_NOPM, 0, 0, &dapm_dmic5_mux),
	SND_SOC_DAPM_MUX("DMic 6", SND_SOC_NOPM, 0, 0, &dapm_dmic6_mux),

	SND_SOC_DAPM_MIXER("DMIC1 Mute", REG_DIGMICCONF,
			REG_DIGMICCONF_ENDMIC1, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("DMIC2 Mute", REG_DIGMICCONF,
			REG_DIGMICCONF_ENDMIC2, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("DMIC3 Mute", REG_DIGMICCONF,
			REG_DIGMICCONF_ENDMIC3, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("DMIC4 Mute", REG_DIGMICCONF,
			REG_DIGMICCONF_ENDMIC4, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("DMIC5 Mute", REG_DIGMICCONF,
			REG_DIGMICCONF_ENDMIC5, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("DMIC6 Mute", REG_DIGMICCONF,
			REG_DIGMICCONF_ENDMIC6, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("AD4 Channel Gain", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("AD4 Enable", REG_ADPATHENA,
			REG_ADPATHENA_ENAD34, 0, NULL, 0),

	/* LineIn to Bypass path */

	SND_SOC_DAPM_MIXER("LINL to HSL Gain", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("LINR to HSR Gain", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("LineIn Left to Headset Left", SND_SOC_NOPM, 0, 0,
			&dapm_linl_to_hs_left_mux),
	SND_SOC_DAPM_MUX("LineIn Right to Headset Right", SND_SOC_NOPM, 0, 0,
			&dapm_linr_to_hs_right_mux),

	/* LineIn to Speaker */

	SND_SOC_DAPM_MUX("AD1 to IHF", SND_SOC_NOPM, 0, 0,
			&dapm_ad1_to_ihf_mux),
	SND_SOC_DAPM_MUX("AD2 to Vibra", SND_SOC_NOPM, 0, 0,
			&dapm_ad2_to_vib_mux),

	/* Acoustical Noise Cancellation path */

	SND_SOC_DAPM_MUX("ANC Source Playback Route",
			SND_SOC_NOPM, 0, 0, dapm_anc_in_select),

	SND_SOC_DAPM_MUX("ANC Playback Switch",
			SND_SOC_NOPM, 0, 0, dapm_anc_enable),

	SND_SOC_DAPM_SWITCH("ANC to Earpiece",
			SND_SOC_NOPM, 0, 0, dapm_anc_ear_mute),

	/* Sidetone Filter path */

	SND_SOC_DAPM_MUX("Sidetone Left Source Playback Route",
			SND_SOC_NOPM, 0, 0, dapm_stfir1_in_select),
	SND_SOC_DAPM_MUX("Sidetone Right Source Playback Route",
			SND_SOC_NOPM, 0, 0, dapm_stfir2_in_select),

	SND_SOC_DAPM_MUX("Sidetone Left",
			SND_SOC_NOPM, 0, 0, &dapm_stfir1_ena),
	SND_SOC_DAPM_MUX("Sidetone Right",
			SND_SOC_NOPM, 0, 0, &dapm_stfir2_ena),

	SND_SOC_DAPM_MIXER_E("STFIR1 Control", SND_SOC_NOPM, 0, 0, NULL, 0,
		stfir_enable_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER_E("STFIR2 Control", SND_SOC_NOPM, 0, 0, NULL, 0,
		stfir_enable_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_MIXER("STFIR1 Gain", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("STFIR2 Gain", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* Voice Interface */
	SND_SOC_DAPM_MIXER("RX1 Hi-Pass Filter",
			REG_TXPATHENA, REG_TXPATHENA_ENRX1, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("TX1 Hi-Pass Filter",
			REG_TXPATHENA, REG_TXPATHENA_ENTX1, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("TX2 Hi-Pass Filter",
			REG_TXPATHENA, REG_TXPATHENA_ENTX2, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("TX3 Hi-Pass Filter",
			REG_TXPATHENA, REG_TXPATHENA_ENTX3, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("TX4 Hi-Pass Filter",
			REG_TXPATHENA, REG_TXPATHENA_ENTX4, 0, NULL, 0),

	SND_SOC_DAPM_MUX("Voice Interface Echo Reference Source1",
			SND_SOC_NOPM, 0, 0, dapm_voice_echoref_source),

	SND_SOC_DAPM_MUX("Voice Interface Echo Reference Source2",
			SND_SOC_NOPM, 0, 0, dapm_voice_echoref_source),

	SND_SOC_DAPM_MUX("Voice Sidetone through STFIR1 InputMux",
			SND_SOC_NOPM, 0, 0, dapm_voice_sidetone_stfir),

	SND_SOC_DAPM_MUX("Voice Sidetone through STFIR1 OutputMuxA",
			SND_SOC_NOPM, 0, 0, dapm_voice_sidetone_stfir),

	SND_SOC_DAPM_MUX("Voice Sidetone through STFIR1 OutputMuxB",
			SND_SOC_NOPM, 0, 0, dapm_voice_sidetone_stfir),

	SND_SOC_DAPM_MUX("Voice Sidetone through STFIR2 OutputMux",
			SND_SOC_NOPM, 0, 0, dapm_voice_sidetone_stfir),

	SND_SOC_DAPM_SWITCH("Voice Interface RX1 to HS", SND_SOC_NOPM, 0, 0,
		dapm_rx_to_hs_switch),

	SND_SOC_DAPM_SWITCH("Voice Interface RX1 to IHF", SND_SOC_NOPM, 0, 0,
		dapm_rx_to_ihf_switch),

	SND_SOC_DAPM_MUX("Voice Interface Sidetone Source",
			SND_SOC_NOPM, 0, 0, dapm_voice_sidetone_source),

	SND_SOC_DAPM_SWITCH("Voice Interface Direct Sidetone", SND_SOC_NOPM,
			0, 0, dapm_voice_direct_sidetone_switch),
};

/* DAPM-routes */

static const struct snd_soc_dapm_route dapm_routes[] = {
	/* AD/DA */
	{"ADC", NULL, "ADC Input"},
	{"DAC Output", NULL, "DAC"},

	/* AD/DA voice */
	{"ADC voice", NULL, "ADC Input"},
	{"DAC Output", NULL, "DAC voice"},

	/* Powerup charge pump if DA1/2 is in use */
	{"HSL Mute", NULL, "Charge Pump"},
	{"HSR Mute", NULL, "Charge Pump"},

	/* Headset path */

	{"DA1 Enable", NULL, "DA_IN1"},
	{"DA2 Enable", NULL, "DA_IN2"},

	{"HSL Digital Gain", NULL, "DA1 Enable"},
	{"HSR Digital Gain", NULL, "DA2 Enable"},

	{"HSL DAC", NULL, "HSL Digital Gain"},
	{"HSR DAC", NULL, "HSR Digital Gain"},

	{"HSL DAC Mute", NULL, "HSL DAC"},
	{"HSR DAC Mute", NULL, "HSR DAC"},

	{"HSL DAC Driver", NULL, "HSL DAC Mute"},
	{"HSR DAC Driver", NULL, "HSR DAC Mute"},

	{"HSL Mute", NULL, "HSL DAC Driver"},
	{"HSR Mute", NULL, "HSR DAC Driver"},

	{"CKLoL Enable", NULL, "HSL DAC Driver"},
	{"CKLoR Enable", NULL, "HSR DAC Driver"},

	{"Headset Left", "Enabled", "HSL Mute"},
	{"Headset Right", "Enabled", "HSR Mute"},

	{"CKL to DM", "CKL switch", "CKLoL Enable"},
	{"CKR to DP", "CKR switch", "CKLoR Enable"},

	{"HSL Enable", NULL, "Headset Left"},
	{"HSR Enable", NULL, "Headset Right"},

	{"HSL Gain", NULL, "HSL Enable"},
	{"HSR Gain", NULL, "HSR Enable"},

	{"HSL", NULL, "HSL Gain"},
	{"HSR", NULL, "HSR Gain"},

	{"CKLoL", NULL, "CKL to DM"},
	{"CKLoR", NULL, "CKR to DP"},

	/* IHF, Vibra or LineOut path */

	{"DA3 Enable", NULL, "DA_IN3"},
	{"DA4 Enable", NULL, "DA_IN4"},

	{"HandsFree Digital Gain", NULL, "DA3 Enable"},
	{"Vibra Digital Gain", NULL, "DA4 Enable"},

	{"IHF Source Playback Route", "Audio Path", "HandsFree Digital Gain"},
	{"Vibra Source Playback Route", "Audio Path", "Vibra Digital Gain"},

	{"DA3 or ANC path to Hf", NULL, "IHF Source Playback Route"},
	{"DA4 or ANC path to Vib", NULL, "Vibra Source Playback Route"},

	/* IHF path */

	{"HandsFree", "Enabled", "DA3 or ANC path to Hf"},

	{"DA input selection for IHF", "DA3", "HandsFree"},
	{"DA input selection for IHF", "DA5", "DA5 Channel Gain"},

	{"IHF DAC", NULL, "DA input selection for IHF"},

	{"IHF Controller Playback Route", "PWM-HF", "IHF DAC"},
	{"IHF Controller Playback Route", "EPWM2", "EPWM2 Enable"},

	{"IHF Enable", NULL, "IHF Controller Playback Route"},

	{"IHF or LineOut Select", "IHF", "IHF Enable"},


	/* Earpiece path */

	{"Earpiece or LineOut Mono Source", "Headset Left", "HSL Digital Gain"},
	{"Earpiece or LineOut Mono Source", "IHF",
			"DA3 or ANC path to Hf"},

	{"EAR DAC", NULL, "Earpiece or LineOut Mono Source"},

	{"Earpiece", "Enabled", "EAR DAC"},

	{"EAR Mute", NULL, "Earpiece"},

	{"EAR Enable", NULL, "EAR Mute"},

	{"EAR", NULL, "EAR Enable"},

	/* LineOut path stereo */

	{"LineOut Source Playback Route", "Stereo Path", "HSL DAC Driver"},
	{"LineOut Source Playback Route", "Stereo Path", "HSR DAC Driver"},

	/* LineOut path mono */

	{"LineOut Source Playback Route", "Mono Path", "EAR DAC"},

	/* LineOut path */

	{"LineOut Left", "Enabled", "LineOut Source Playback Route"},
	{"LineOut Right", "Enabled", "LineOut Source Playback Route"},

	{"LOL Enable", NULL, "LineOut Left"},
	{"LOR Enable", NULL, "LineOut Right"},

	{"IHF or LineOut Select", "LineOut", "LOL Enable"},
	{"Vibra or LineOut Select", "LineOut", "LOR Enable"},

	/* IHF path */

	{"IHF", NULL, "IHF or LineOut Select"},
	{"VIB", NULL, "Vibra or LineOut Select"},

    /* PDM path*/
	{"PDM 1", "Enabled", "HandsFree"},
	{"PDM 2", "Enabled", "Vibra"},

	{"PDM1 Enable", NULL, "PDM 1"},
	{"PDM2 Enable", NULL, "PDM 2"},

	{"PDM", NULL, "PDM1 Enable"},
	{"PDM", NULL, "PDM2 Enable"},

	/* Vibrator path */

	{"Vibra", "Enabled", "DA4 or ANC path to Vib"},

	{"DA second input selection for VIBRA", "DA4", "Vibra"},

	{"DA first input selection for VIBRA", "DA4/DA6",
		"DA second input selection for VIBRA"},
	{"DA first input selection for VIBRA", "DA1", "DA1 Enable"},

	{"Vibra DAC", NULL, "DA first input selection for VIBRA"},

	{"Vibra Controller Playback Route", "PWM-VIB", "Vibra DAC"},
	{"Vibra Controller Playback Route", "EPWM1", "EPWM1 Enable"},

	{"Vibra Enable", NULL, "Vibra Controller Playback Route"},

	{"Vibra or LineOut Select", "Vibra", "Vibra Enable"},

	/* PWM path */

	{"DA5 Channel Gain", NULL, "DA_IN5"},
	{"DA6 Channel Gain", NULL, "DA_IN6"},

	{"EPWM1 Source", "Audio Path", "DA6 Channel Gain"},
	{"EPWM1 Source", "EPWM Generator", "EPWMGEN1"},

	{"EPWM2 Source", "Audio Path", "DA5 Channel Gain"},
	{"EPWM2 Source", "EPWM Generator", "EPWMGEN2"},

	{"PWM 2", "Enabled", "EPWM2 Source"},
	{"PWM 1", "Enabled", "EPWM1 Source"},

	{"EPWM2 Enable", NULL, "PWM 2"},
	{"EPWM1 Enable", NULL, "PWM 1"},


	{"EPWM2", NULL, "EPWM2 Enable"},
	{"EPWM1", NULL, "EPWM1 Enable"},

	/* FM path */
	{"DA 7 to DA 1", "Playback Switch", "DA_IN7"},
	{"DA 7 to DA 3", "Playback Switch", "DA_IN7"},
	{"DA 8 to DA 2", "Playback Switch", "DA_IN8"},
	{"DA 8 to DA 4", "Playback Switch", "DA_IN8"},

	{"DA1 Enable", NULL, "DA 7 to DA 1"},
	{"DA2 Enable", NULL, "DA 8 to DA 2"},
	{"DA3 Enable", NULL, "DA 7 to DA 3"},
	{"DA4 Enable", NULL, "DA 8 to DA 4"},

	/* Voice interface RX path */
	{"RX1 Hi-Pass Filter", NULL, "RX_IN1"},

	{"AD 1 Voice RX1 Select Capture Route", "RX1", "RX1 Hi-Pass Filter"},
	{"AD 4 Voice RX1 Select Capture Route", "RX1", "RX1 Hi-Pass Filter"},

	{"Voice Interface RX1 to HS", "Playback Switch", "RX1 Hi-Pass Filter"},
	{"Voice Interface RX1 to IHF", "Playback Switch", "RX1 Hi-Pass Filter"},

	{"HSL Digital Gain", NULL, "Voice Interface RX1 to HS"},
	{"HSR Digital Gain", NULL, "Voice Interface RX1 to HS"},

	{"HandsFree", NULL, "Voice Interface RX1 to IHF"},
	{"Vibra"    , NULL, "Voice Interface RX1 to IHF"},

	/* Voice interface TX path */
	{"TX1 Hi-Pass Filter", NULL, "AD 3 Select Capture Route"},
	{"TX2 Hi-Pass Filter", NULL, "AD 2 Select Capture Route"},

	{"TX_OUT1", NULL, "TX1 Hi-Pass Filter"},
	{"TX_OUT2", NULL, "TX2 Hi-Pass Filter"},

	{"Voice Interface Echo Reference Source1",
		"Headset", "HSL Digital Gain"},
	{"Voice Interface Echo Reference Source2",
		"Headset", "HSR Digital Gain"},

	{"Voice Interface Echo Reference Source1", "IHF", "HandsFree"},
	{"Voice Interface Echo Reference Source2", "IHF", "Vibra"},

	{"TX3 Hi-Pass Filter", NULL, "Voice Interface Echo Reference Source1"},
	{"TX4 Hi-Pass Filter", NULL, "Voice Interface Echo Reference Source2"},

	{"TX_OUT3", NULL, "TX3 Hi-Pass Filter"},
	{"TX_OUT4", NULL, "TX4 Hi-Pass Filter"},

	/* LineIn & Microphone 2 & 3 path */

	{"LineIn Left", "Enabled", "LINL"},
	{"LineIn Right", "Enabled", "LINR"},

	{"LINL Mute", NULL, "LineIn Left"},
	{"LINR Mute", NULL, "LineIn Right"},

	{"Mic 2", "Enabled", "MIC2 Input"},
	{"Mic 3", "Enabled", "MIC3 Input"},

	{"MIC2 Mute", NULL, "Mic 2"},
	{"MIC3 Mute", NULL, "Mic 3"},

	{"LINL Enable", NULL, "LINL Mute"},
	{"LINR Enable", NULL, "LINR Mute"},
	{"MIC2 Enable", NULL, "MIC2 Mute"},
	{"MIC3 Enable", NULL, "MIC3 Mute"},

	{"Mic or LIN Select Capture Route", "LineIn", "LINR Enable"},
	{"Mic or LIN Select Capture Route", "Mic", "MIC2 Enable"},
	{"Mic or LIN Select Capture Route", "LineIn", "LINL Enable"},
	{"Mic or LIN Select Capture Route", "Mic", "MIC3 Enable"},

	{"LINL ADC", NULL, "Mic or LIN Select Capture Route"},
	{"LINR ADC", NULL, "Mic or LIN Select Capture Route"},

	{"AD 1 Select Capture Route", "LineIn Left", "LINL ADC"},
	{"AD 2 Select Capture Route", "LineIn Right", "LINR ADC"},

	{"AD 1 Voice RX1 Select Capture Route",
		"AD1", "AD 1 Select Capture Route"},

	{"AD1 Channel Gain", NULL, "AD 1 Voice RX1 Select Capture Route"},
	{"AD2 Channel Gain", NULL, "AD 2 Select Capture Route"},

	{"AD12 Enable", NULL, "AD1 Channel Gain"},
	{"AD12 Enable", NULL, "AD2 Channel Gain"},

	{"AD_OUT1", NULL, "AD12 Enable"},
	{"AD_OUT2", NULL, "AD12 Enable"},

	/* Microphone 1 path */

	{"Mic 1A or 1B Select Capture Route", "Mic 1A", "MIC1A Input"},
	{"Mic 1A or 1B Select Capture Route", "Mic 1B", "MIC1B Input"},

	{"Mic 1", "Enabled", "Mic 1A or 1B Select Capture Route"},

	{"MIC1 Mute", NULL, "Mic 1"},

	{"MIC1 Enable", NULL, "MIC1 Mute"},

	{"MIC1 ADC", NULL, "MIC1 Enable"},

	{"AD 3 Select Capture Route", "Mic 1", "MIC1 ADC"},

	{"AD3 Channel Gain", NULL, "AD 3 Select Capture Route"},

	{"AD3 Enable", NULL, "AD3 Channel Gain"},

	{"AD_OUT3", NULL, "AD3 Enable"},

	/* HD Capture path */

	{"AD 5 Select Capture Route", "Mic 2", "LINR ADC"},
	{"AD 6 Select Capture Route", "Mic 1", "MIC1 ADC"},

	{"AD5 Channel Gain", NULL, "AD 5 Select Capture Route"},
	{"AD6 Channel Gain", NULL, "AD 6 Select Capture Route"},

	{"AD57 Enable", NULL, "AD5 Channel Gain"},
	{"AD68 Enable", NULL, "AD6 Channel Gain"},

	{"AD_OUT57", NULL, "AD57 Enable"},
	{"AD_OUT68", NULL, "AD68 Enable"},

	/* Digital Microphone path */

	{"DMic 1", "Enabled", "DMIC Input"},
	{"DMic 2", "Enabled", "DMIC Input"},
	{"DMic 3", "Enabled", "DMIC Input"},
	{"DMic 4", "Enabled", "DMIC Input"},
	{"DMic 5", "Enabled", "DMIC Input"},
	{"DMic 6", "Enabled", "DMIC Input"},

	{"DMIC1 Mute", NULL, "DMic 1"},
	{"DMIC2 Mute", NULL, "DMic 2"},
	{"DMIC3 Mute", NULL, "DMic 3"},
	{"DMIC4 Mute", NULL, "DMic 4"},
	{"DMIC5 Mute", NULL, "DMic 5"},
	{"DMIC6 Mute", NULL, "DMic 6"},

	{"AD 1 Select Capture Route", "DMic 1", "DMIC1 Mute"},
	{"AD 2 Select Capture Route", "DMic 2", "DMIC2 Mute"},
	{"AD 3 Select Capture Route", "DMic 3", "DMIC3 Mute"},
	{"AD 5 Select Capture Route", "DMic 5", "DMIC5 Mute"},
	{"AD 6 Select Capture Route", "DMic 6", "DMIC6 Mute"},

	{"AD 4 Voice RX1 Select Capture Route", "AD4", "DMIC4 Mute"},
	{"AD4 Channel Gain", NULL, "AD 4 Voice RX1 Select Capture Route"},

	{"AD4 Enable", NULL, "AD4 Channel Gain"},

	{"AD_OUT4", NULL, "AD4 Enable"},

	/* LineIn to Headset Bypass path */

	{"LINL to HSL Gain", NULL, "LINL Enable"},
	{"LINR to HSR Gain", NULL, "LINR Enable"},

	{"LineIn Left to Headset Left", "Enabled", "LINL to HSL Gain"},
	{"LineIn Right to Headset Right", "Enabled", "LINR to HSR Gain"},

	{"HSL DAC Driver", NULL, "LineIn Left to Headset Left"},
	{"HSR DAC Driver", NULL, "LineIn Right to Headset Right"},

	/* LineIn to Speaker path */

	{"AD1 to IHF", "Enabled", "AD12 Enable"},
	{"AD2 to Vibra", "Enabled", "AD12 Enable"},

	{"DA3 Enable", NULL, "AD1 to IHF"},
	{"DA4 Enable", NULL, "AD2 to Vibra"},

	/* Acoustical Noise Cancellation path */

	{"ANC Source Playback Route", "Mic 2 / DMic 5", "AD5 Channel Gain"},
	{"ANC Source Playback Route", "Mic 1 / DMic 6", "AD6 Channel Gain"},

	{"ANC Playback Switch", "Enabled", "ANC Source Playback Route"},

	{"IHF Source Playback Route", "ANC", "ANC Playback Switch"},
	{"Vibra Source Playback Route", "ANC", "ANC Playback Switch"},
	{"ANC to Earpiece", "Playback Switch", "ANC Playback Switch"},

	{"HSL Digital Gain", NULL, "ANC to Earpiece"},

	/* Sidetone Filter path */

	{"Voice Interface Sidetone Source", "TX1", "TX1 Hi-Pass Filter"},
	{"Voice Interface Sidetone Source", "TX2", "TX2 Hi-Pass Filter"},

	{"Voice Interface Direct Sidetone",
		"Switch", "Voice Interface Sidetone Source"},
	{"RX1 Hi-Pass Filter", NULL, "Voice Interface Direct Sidetone"},

	{"Sidetone Left Source Playback Route", "LineIn Left", "AD12 Enable"},
	{"Sidetone Left Source Playback Route", "LineIn Right", "AD12 Enable"},
	{"Sidetone Left Source Playback Route", "Mic 1", "AD3 Enable"},
	{"Sidetone Left Source Playback Route", "Headset Left", "DA_IN1"},
	{"Sidetone Right Source Playback Route", "LineIn Right", "AD12 Enable"},
	{"Sidetone Right Source Playback Route", "Mic 1", "AD3 Enable"},
	{"Sidetone Right Source Playback Route", "DMic 4", "AD4 Enable"},
	{"Sidetone Right Source Playback Route", "Headset Right", "DA_IN2"},

	{"Voice Sidetone through STFIR1 InputMux",
		"Disabled", "Sidetone Left Source Playback Route"},
	{"Voice Sidetone through STFIR1 InputMux",
		"Enabled", "Voice Interface Sidetone Source"},

	{"Sidetone Left", "Enabled", "Voice Sidetone through STFIR1 InputMux"},
	{"Sidetone Right", "Enabled", "Sidetone Right Source Playback Route"},

	{"STFIR1 Control", NULL, "Sidetone Left"},
	{"STFIR2 Control", NULL, "Sidetone Right"},

	{"Voice Sidetone through STFIR1 OutputMuxA",
		"Disabled", "STFIR1 Control"},
	{"Voice Sidetone through STFIR1 OutputMuxB",
		"Enabled", "STFIR1 Control"},

	{"Voice Sidetone through STFIR2 OutputMux",
		"Disabled", "STFIR2 Control"},

	{"STFIR1 Gain", NULL, "Voice Sidetone through STFIR1 OutputMuxA"},
	{"RX1 Hi-Pass Filter",
		NULL , "Voice Sidetone through STFIR1 OutputMuxB"},

	{"STFIR2 Gain", NULL, "Voice Sidetone through STFIR2 OutputMux"},

	{"DA1 Enable", NULL, "STFIR1 Gain"},
	{"DA2 Enable", NULL, "STFIR2 Gain"},
};

/* Controls - Non-DAPM ASoC */

/* from -31 to 31 dB in 1 dB steps (mute instead of -32 dB) */
static DECLARE_TLV_DB_SCALE(adx_dig_gain_tlv, -3200, 100, 1);

/* from -62 to 0 dB in 1 dB steps (mute instead of -63 dB) */
static DECLARE_TLV_DB_SCALE(dax_dig_gain_tlv, -6300, 100, 1);

/* from 0 to 8 dB in 1 dB steps (mute instead of -1 dB) */
static DECLARE_TLV_DB_SCALE(hs_ear_dig_gain_tlv, -100, 100, 1);

/* from -30 to 0 dB in 1 dB steps (mute instead of -31 dB) */
static DECLARE_TLV_DB_SCALE(stfir_dig_gain_tlv, -3100, 100, 1);

/* from -3 to 20 dB in 1 dB steps  */
static DECLARE_TLV_DB_SCALE(hf_dig_gain_tlv, -300, 100, 0);

/* from -3 to 20 dB in 1 dB steps  */
static DECLARE_TLV_DB_SCALE(vib_dig_gain_tlv, -300, 100, 0);

/* from -32 to -20 dB in 4 dB steps / from -18 to 2 dB in 2 dB steps */
static const unsigned int hs_gain_tlv[] = {
	TLV_DB_RANGE_HEAD(2),
	0, 3, TLV_DB_SCALE_ITEM(-3200, 400, 0),
	4, 15, TLV_DB_SCALE_ITEM(-1800, 200, 0),
};

/* from 0 to 31 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(mic_gain_tlv, 0, 100, 0);

/* from -10 to 20 dB in 2 dB steps */
static DECLARE_TLV_DB_SCALE(lin_gain_tlv, -1000, 200, 0);

/* from -36 to 0 dB in 2 dB steps (mute instead of -38 dB) */
static DECLARE_TLV_DB_SCALE(lin2hs_gain_tlv, -3800, 200, 1);

/* from -8 to 8 dB in 2 dB steps */
static DECLARE_TLV_DB_SCALE(ear_gain_tlv, -800, 200, 0);

static SOC_ENUM_SINGLE_DECL(soc_enum_hslowpow,
	REG_ANACONF1, REG_ANACONF1_HSLOWPOW, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_daclowpow1,
	REG_ANACONF1, REG_ANACONF1_DACLOWPOW1, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_daclowpow0,
	REG_ANACONF1, REG_ANACONF1_DACLOWPOW0, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_eardaclowpow,
	REG_ANACONF1, REG_ANACONF1_EARDACLOWPOW, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_eardrvlowpow,
	REG_ANACONF1, REG_ANACONF1_EARDRVLOWPOW, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_enhpear,
	REG_DAPATHENA, REG_DAPATHENA_ENHPEAR, enum_dis_ena);

static const char * const enum_earselcm[] = {"0.95V", "1.10V",
		"1.27V", "1.58V"};
static SOC_ENUM_SINGLE_DECL(soc_enum_earselcm,
	REG_ANACONF1, REG_ANACONF1_EARSELCM, enum_earselcm);

static const char * const enum_hsfadspeed[] = {"2ms", "0.5ms",
		"10.6ms", "5ms"};
static SOC_ENUM_SINGLE_DECL(soc_enum_hsfadspeed,
	REG_DIGMICCONF, REG_DIGMICCONF_HSFADSPEED, enum_hsfadspeed);

static const char * const enum_envdetthre[] = {
	"250mV", "300mV", "350mV", "400mV",
	"450mV", "500mV", "550mV", "600mV",
	"650mV", "700mV", "750mV", "800mV",
	"850mV", "900mV", "950mV", "1.00V" };
static SOC_ENUM_SINGLE_DECL(soc_enum_envdetcpen,
	REG_SIGENVCONF, REG_SIGENVCONF_ENVDETCPEN, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_envdeththre,
	REG_ENVCPCONF, REG_ENVCPCONF_ENVDETHTHRE, enum_envdetthre);
static SOC_ENUM_SINGLE_DECL(soc_enum_envdetlthre,
	REG_ENVCPCONF, REG_ENVCPCONF_ENVDETLTHRE, enum_envdetthre);

static const char * const enum_envdettime[] = {
	"26.6us", "53.2us", "106us",  "213us",
	"426us",  "851us",  "1.70ms", "3.40ms",
	"6.81ms", "13.6ms", "27.2ms", "54.5ms",
	"109ms",  "218ms",  "436ms",  "872ms" };
static SOC_ENUM_SINGLE_DECL(soc_enum_envdettime,
	REG_SIGENVCONF, REG_SIGENVCONF_ENVDETTIME, enum_envdettime);

static const char * const enum_ensemicx[] = {"Differential", "Single Ended"};
static SOC_ENUM_SINGLE_DECL(soc_enum_ensemic1,
	REG_ANAGAIN1, REG_ANAGAINX_ENSEMICX, enum_ensemicx);
static SOC_ENUM_SINGLE_DECL(soc_enum_ensemic2,
	REG_ANAGAIN2, REG_ANAGAINX_ENSEMICX, enum_ensemicx);
static SOC_ENUM_SINGLE_DECL(soc_enum_ensemic3,
		REG_MIC3CONFIG, REG_MIC3CONFIG_ENSEMIC3, enum_ensemicx);
static SOC_ENUM_SINGLE_DECL(soc_enum_lowpowmic1,
	REG_ANAGAIN1, REG_ANAGAINX_LOWPOWMICX, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_lowpowmic2,
	REG_ANAGAIN2, REG_ANAGAINX_LOWPOWMICX, enum_dis_ena);

static const char * const enum_usbmic_sel[] = {"None",
		"USBSWCAP", "USB DP", "USB ID"};
static SOC_ENUM_SINGLE_DECL(soc_enum_usbmic, REG_EARGAINMICSEL,
		REG_EARGAINMICSEL_USBMICSEL, enum_usbmic_sel);

static SOC_ENUM_DOUBLE_DECL(soc_enum_ad12nh, REG_ADFILTCONF,
	REG_ADFILTCONF_AD1NH, REG_ADFILTCONF_AD2NH, enum_ena_dis);
static SOC_ENUM_DOUBLE_DECL(soc_enum_ad34nh, REG_ADFILTCONF,
	REG_ADFILTCONF_AD3NH, REG_ADFILTCONF_AD4NH, enum_ena_dis);

static const char * const enum_av_mode[] = {"Audio", "Voice"};
static SOC_ENUM_DOUBLE_DECL(soc_enum_ad12voice, REG_ADFILTCONF,
	REG_ADFILTCONF_AD1VOICE, REG_ADFILTCONF_AD2VOICE, enum_av_mode);
static SOC_ENUM_DOUBLE_DECL(soc_enum_ad34voice, REG_ADFILTCONF,
	REG_ADFILTCONF_AD3VOICE, REG_ADFILTCONF_AD4VOICE, enum_av_mode);

static SOC_ENUM_SINGLE_DECL(soc_enum_da12voice,
	REG_DASLOTCONF1, REG_DASLOTCONF1_DA12VOICE, enum_av_mode);
static SOC_ENUM_SINGLE_DECL(soc_enum_da34voice,
	REG_DASLOTCONF3, REG_DASLOTCONF3_DA34VOICE, enum_av_mode);
static SOC_ENUM_SINGLE_DECL(soc_enum_da56voice,
	REG_DASLOTCONF5, REG_DASLOTCONF5_DA56VOICE, enum_av_mode);

static SOC_ENUM_SINGLE_DECL(soc_enum_swapda12_34,
	REG_DASLOTCONF1, REG_DASLOTCONF1_SWAPDA12_34, enum_dis_ena);


static const char * const enum_sinc53[] = {"Sinc 5", "Sinc 3"};
static SOC_ENUM_DOUBLE_DECL(soc_enum_dmic12sinc, REG_DMICFILTCONF,
	REG_DMICFILTCONF_DMIC1SINC3, REG_DMICFILTCONF_DMIC2SINC3, enum_sinc53);
static SOC_ENUM_DOUBLE_DECL(soc_enum_dmic34sinc, REG_DMICFILTCONF,
	REG_DMICFILTCONF_DMIC3SINC3, REG_DMICFILTCONF_DMIC4SINC3, enum_sinc53);
static SOC_ENUM_DOUBLE_DECL(soc_enum_dmic56sinc, REG_DMICFILTCONF,
	REG_DMICFILTCONF_DMIC5SINC3, REG_DMICFILTCONF_DMIC6SINC3, enum_sinc53);

static const char * const enum_da2hslr[] = {"Sidetone", "Audio Path"};
static SOC_ENUM_DOUBLE_DECL(soc_enum_da2hslr, REG_DIGMULTCONF1,
	REG_DIGMULTCONF1_DATOHSLEN, REG_DIGMULTCONF1_DATOHSREN,	enum_da2hslr);

static const char * const enum_sinc31[] = {"Sinc 3", "Sinc 1"};
static SOC_ENUM_SINGLE_DECL(soc_enum_hsesinc,
		REG_HSLEARDIGGAIN, REG_HSLEARDIGGAIN_HSLOWLAT, enum_sinc31);

static const char *const enum_fadespeed[] = {"1ms", "4ms", "8ms", "16ms"};
static SOC_ENUM_SINGLE_DECL(soc_enum_fadespeed,
	REG_HSRDIGGAIN, REG_HSRDIGGAIN_FADESPEED, enum_fadespeed);

/* Digital interface - Clocks */
static SOC_ENUM_SINGLE_DECL(soc_enum_mastgen,
	REG_DIGIFCONF1, REG_DIGIFCONF1_ENMASTGEN, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_fsbitclk0,
	REG_DIGIFCONF1, REG_DIGIFCONF1_ENFSBITCLK0, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_fsbitclk1,
	REG_DIGIFCONF1, REG_DIGIFCONF1_ENFSBITCLK1, enum_dis_ena);

/* Digital interface - DA from slot mapping */
static const char * const enum_da_from_slot_map[] = {"SLOT0",
					"SLOT1",
					"SLOT2",
					"SLOT3",
					"SLOT4",
					"SLOT5",
					"SLOT6",
					"SLOT7",
					"SLOT8",
					"SLOT9",
					"SLOT10",
					"SLOT11",
					"SLOT12",
					"SLOT13",
					"SLOT14",
					"SLOT15",
					"SLOT16",
					"SLOT17",
					"SLOT18",
					"SLOT19",
					"SLOT20",
					"SLOT21",
					"SLOT22",
					"SLOT23",
					"SLOT24",
					"SLOT25",
					"SLOT26",
					"SLOT27",
					"SLOT28",
					"SLOT29",
					"SLOT30",
					"SLOT31"};
static SOC_ENUM_SINGLE_DECL(soc_enum_da1slotmap,
	REG_DASLOTCONF1, REG_DASLOTCONFX_SLTODAX_SHIFT, enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da2slotmap,
	REG_DASLOTCONF2, REG_DASLOTCONFX_SLTODAX_SHIFT, enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da3slotmap,
	REG_DASLOTCONF3, REG_DASLOTCONFX_SLTODAX_SHIFT, enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da4slotmap,
	REG_DASLOTCONF4, REG_DASLOTCONFX_SLTODAX_SHIFT, enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da5slotmap,
	REG_DASLOTCONF5, REG_DASLOTCONFX_SLTODAX_SHIFT, enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da6slotmap,
	REG_DASLOTCONF6, REG_DASLOTCONFX_SLTODAX_SHIFT, enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da7slotmap,
	REG_DASLOTCONF7, REG_DASLOTCONFX_SLTODAX_SHIFT, enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da8slotmap,
	REG_DASLOTCONF8, REG_DASLOTCONFX_SLTODAX_SHIFT, enum_da_from_slot_map);

/* Digital interface - AD to slot mapping */
static const char * const enum_ad_to_slot_map[] = {"AD_OUT1",
					"AD_OUT2",
					"AD_OUT3",
					"AD_OUT4",
					"AD_OUT5",
					"AD_OUT6",
					"AD_OUT7",
					"AD_OUT8",
					"zeroes",
					"tristate"};
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot0map,
	REG_ADSLOTSEL1, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot1map,
	REG_ADSLOTSEL1, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot2map,
	REG_ADSLOTSEL2, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot3map,
	REG_ADSLOTSEL2, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot4map,
	REG_ADSLOTSEL3, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot5map,
	REG_ADSLOTSEL3, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot6map,
	REG_ADSLOTSEL4, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot7map,
	REG_ADSLOTSEL4, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot8map,
	REG_ADSLOTSEL5, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot9map,
	REG_ADSLOTSEL5, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot10map,
	REG_ADSLOTSEL6, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot11map,
	REG_ADSLOTSEL6, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot12map,
	REG_ADSLOTSEL7, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot13map,
	REG_ADSLOTSEL7, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot14map,
	REG_ADSLOTSEL8, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot15map,
	REG_ADSLOTSEL8, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot16map,
	REG_ADSLOTSEL9, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot17map,
	REG_ADSLOTSEL9, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot18map,
	REG_ADSLOTSEL10, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot19map,
	REG_ADSLOTSEL10, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot20map,
	REG_ADSLOTSEL11, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot21map,
	REG_ADSLOTSEL11, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot22map,
	REG_ADSLOTSEL12, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot23map,
	REG_ADSLOTSEL12, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot24map,
	REG_ADSLOTSEL13, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot25map,
	REG_ADSLOTSEL13, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot26map,
	REG_ADSLOTSEL14, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot27map,
	REG_ADSLOTSEL14, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot28map,
	REG_ADSLOTSEL15, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot29map,
	REG_ADSLOTSEL15, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot30map,
	REG_ADSLOTSEL16, REG_ADSLOTSELX_EVEN_SHIFT, enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot31map,
	REG_ADSLOTSEL16, REG_ADSLOTSELX_ODD_SHIFT, enum_ad_to_slot_map);

/* Digital interface - Digital loopback */
static SOC_ENUM_SINGLE_DECL(soc_enum_ad1loop,
	REG_DASLOTCONF1, REG_DASLOTCONF1_DAI7TOADO1, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_ad2loop,
	REG_DASLOTCONF2, REG_DASLOTCONF2_DAI8TOADO2, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_ad3loop,
	REG_DASLOTCONF3, REG_DASLOTCONF3_DAI7TOADO3, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_ad4loop,
	REG_DASLOTCONF4, REG_DASLOTCONF4_DAI8TOADO4, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_ad5loop,
	REG_DASLOTCONF5, REG_DASLOTCONF5_DAI7TOADO5, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_ad6loop,
	REG_DASLOTCONF6, REG_DASLOTCONF6_DAI8TOADO6, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_ad7loop,
	REG_DASLOTCONF7, REG_DASLOTCONF7_DAI8TOADO7, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_ad8loop,
	REG_DASLOTCONF8, REG_DASLOTCONF8_DAI7TOADO8, enum_dis_ena);

/* Digital interface - Burst mode */
static SOC_ENUM_SINGLE_DECL(soc_enum_if0fifoen,
	REG_DIGIFCONF3, REG_DIGIFCONF3_IF0BFIFOEN, enum_dis_ena);
static const char * const enum_mask[] = {"Unmasked", "Masked"};
static SOC_ENUM_SINGLE_DECL(soc_enum_bfifomask,
	REG_FIFOCONF1, REG_FIFOCONF1_BFIFOMASK, enum_mask);
static const char * const enum_bitclk0[] = {"19_2_MHz", "38_4_MHz"};
static SOC_ENUM_SINGLE_DECL(soc_enum_bfifo19m2,
	REG_FIFOCONF1, REG_FIFOCONF1_BFIFO19M2, enum_bitclk0);
static const char *const enum_slavemaster[] = {"Slave", "Master"};
static SOC_ENUM_SINGLE_DECL(soc_enum_bfifomast,
	REG_FIFOCONF3, REG_FIFOCONF3_BFIFOMAST_SHIFT, enum_slavemaster);
static SOC_ENUM_SINGLE_DECL(soc_enum_bfifoint,
	REG_FIFOCONF3, REG_FIFOCONF3_BFIFORUN_SHIFT, enum_dis_ena);
static SOC_ENUM_SINGLE_EXT_DECL(soc_enum_digimute, enum_dis_ena);

static const char * const enum_monostereo_mode[] = {"Stereo", "Mono"};
static SOC_ENUM_SINGLE_DECL(soc_enum_ad12mode,
	REG_DMICFREQ, REG_DMICFREQ_AD12LBMONO, enum_monostereo_mode);

static SOC_ENUM_SINGLE_DECL(soc_enum_da78mode,
		REG_DA78MIX, REG_DA78MIX_MONO, enum_monostereo_mode);

static const char * const enum_dmicfreq[] = {"2.4MHz", "3.84MHz", "4.8MHz"};
static SOC_ENUM_SINGLE_DECL(soc_enum_dmic12freq,
		REG_DMICFREQ, REG_DMICFREQ_MIC12FREQ, enum_dmicfreq);
static SOC_ENUM_SINGLE_DECL(soc_enum_dmic34freq,
		REG_DMICFREQ, REG_DMICFREQ_MIC34FREQ, enum_dmicfreq);
static SOC_ENUM_SINGLE_DECL(soc_enum_dmic56freq,
		REG_DMICFREQ, REG_DMICFREQ_MIC56FREQ, enum_dmicfreq);


/* Voice Interface - RX from slot mapping */
static const char * const enum_voice_RX_from_slot_map[] = {"SLOT32",
					"SLOT33",
					"SLOT34",
					"SLOT35",
					"SLOT36",
					"SLOT37",
					"SLOT38",
					"SLOT39",
					"SLOT40",
					"SLOT41",
					"SLOT42",
					"SLOT43",
					"SLOT44",
					"SLOT45",
					"SLOT46",
					"SLOT47"};

static SOC_ENUM_SINGLE_DECL(soc_enum_RXslotmap,
	REG_RXSLOTCONF, REG_RXSLOTCONF_SLTORX1, enum_voice_RX_from_slot_map);

static const char * const enum_voice_TX_to_slot_map[] = {"TX1",
					"TX2",
					"TX3",
					"TX4",
					"zeroes",
					"zeroes",
					"zeroes",
					"zeroes",
					"zeroes",
					"zeroes",
					"zeroes",
					"zeroes",
					"tristate"};
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot32map,
	REG_TXSLOTSEL1, REG_TXSLOTSELX_EVEN_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot33map,
	REG_TXSLOTSEL1, REG_TXSLOTSELX_ODD_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot34map,
	REG_TXSLOTSEL2, REG_TXSLOTSELX_EVEN_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot35map,
	REG_TXSLOTSEL2, REG_TXSLOTSELX_ODD_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot36map,
	REG_TXSLOTSEL3, REG_TXSLOTSELX_EVEN_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot37map,
	REG_TXSLOTSEL3, REG_TXSLOTSELX_ODD_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot38map,
	REG_TXSLOTSEL4, REG_TXSLOTSELX_EVEN_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot39map,
	REG_TXSLOTSEL4, REG_TXSLOTSELX_ODD_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot40map,
	REG_TXSLOTSEL5, REG_TXSLOTSELX_EVEN_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot41map,
	REG_TXSLOTSEL5, REG_TXSLOTSELX_ODD_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot42map,
	REG_TXSLOTSEL6, REG_TXSLOTSELX_EVEN_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot43map,
	REG_TXSLOTSEL6, REG_TXSLOTSELX_ODD_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot44map,
	REG_TXSLOTSEL7, REG_TXSLOTSELX_EVEN_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot45map,
	REG_TXSLOTSEL7, REG_TXSLOTSELX_ODD_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot46map,
	REG_TXSLOTSEL8, REG_TXSLOTSELX_EVEN_SHIFT, enum_voice_TX_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_TXslot47map,
	REG_TXSLOTSEL8, REG_TXSLOTSELX_ODD_SHIFT, enum_voice_TX_to_slot_map);

/*
static SOC_ENUM_SINGLE_DECL(soc_enum_rx1_hp_ena,
	REG_TXPATHENA, REG_TXPATHENA_ENRX1, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_tx1_hp_ena,
	REG_TXPATHENA, REG_TXPATHENA_ENTX1, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_tx2_hp_ena,
	REG_TXPATHENA, REG_TXPATHENA_ENTX2, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_tx3_hp_ena,
	REG_TXPATHENA, REG_TXPATHENA_ENTX3, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_tx4_hp_ena,
	REG_TXPATHENA, REG_TXPATHENA_ENTX4, enum_dis_ena);
*/

/* Voice interface - Gains */
/* TX Gain - from -32 to 31 dB in 1 dB steps (mute instead of -32 dB) */
static DECLARE_TLV_DB_SCALE(voice_tx_gain_tlv, -3200, 100, 1);
/* RX Gain - from -63 to 0 dB in 1 dB steps (mute instead of -63 dB) */
static DECLARE_TLV_DB_SCALE(voice_rx_gain_tlv, -6300, 100, 1);

/* Voice interface - Clocks */
static SOC_ENUM_SINGLE_DECL(soc_enum_mastgen_if3_ena,
	REG_DIGIFCONF5, REG_DIGIFCONF5_ENMASTGEN_IF3, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_fsbitclk3_ena,
	REG_DIGIFCONF5, REG_DIGIFCONF5_ENFSBITCLK3, enum_dis_ena);

/* Voice interface - Filters */

static const char * const enum_voice_filter_selection[] = {
	"Disabled", "1stOrder40Hz", "3rdOrder300Hz"
};
static SOC_ENUM_SINGLE_DECL(soc_enum_rx1_filter_selection, REG_TXRXFILTCONF,
		REG_TXRXFILTCONF_RX1HPF, enum_voice_filter_selection);
static SOC_ENUM_SINGLE_DECL(soc_enum_tx12_filter_selection, REG_TXRXFILTCONF,
		REG_TXRXFILTCONF_TX12HPF, enum_voice_filter_selection);
static SOC_ENUM_SINGLE_DECL(soc_enum_tx34_filter_selection, REG_TXRXFILTCONF,
		REG_TXRXFILTCONF_TX34HPF, enum_voice_filter_selection);

/* Voice Interface - Sidetone Gain */
/* from -32 to 6 dB in 1 dB steps (mute instead of -32 dB) */
static DECLARE_TLV_DB_SCALE(voice_sidetone_gain_tlv, -3200, 100, 1);

/* TODO: move to DAPM */
static SOC_ENUM_SINGLE_DECL(soc_enum_enfirsids,
	REG_SIDFIRCONF, REG_SIDFIRCONF_ENFIRSIDS, enum_dis_ena);
static SOC_ENUM_SINGLE_DECL(soc_enum_parlhf,
	REG_CLASSDCONF1, REG_CLASSDCONF1_PARLHF, enum_dis_ena);


/* PWM */

static const char * const enum_epwmfreq[] = {
	"20Khz", "24Khz", "30Khz",  "48Khz",
	"60Khz",  "64Khz",  "80Khz", "100Khz",
	"120Khz", "150Khz", "192Khz", "256Khz",
	"480Khz",  "600Khz",  "768Khz",  "960Khz" };
static SOC_ENUM_SINGLE_DECL(soc_enum_epwm1freq,
		REG_EPWM1CONF, REG_EPWM1CONF_FREQ, enum_epwmfreq);
static SOC_ENUM_SINGLE_DECL(soc_enum_epwm2freq,
		REG_EPWM2CONF, REG_EPWM2CONF_FREQ, enum_epwmfreq);

static const char * const enum_epwmedge[] = {"Sawtooth mod", "Triangular mod"};
static SOC_ENUM_SINGLE_DECL(soc_enum_epwm1edge,
		REG_EPWM1CONF, REG_EPWM1CONF_EDGE, enum_epwmedge);
static SOC_ENUM_SINGLE_DECL(soc_enum_epwm2edge,
		REG_EPWM2CONF, REG_EPWM2CONF_EDGE, enum_epwmedge);

static const char * const enum_epwm2gpiosel[] = {"PdmClk signal",
		"PWM2 signal"};
static SOC_ENUM_SINGLE_DECL(soc_enum_epwm2gpiosel,
		REG_EPWM2CONF, REG_EPWM2CONF_GPIOSEL, enum_epwm2gpiosel);

static SOC_ENUM_SINGLE_EXT_DECL(soc_enum_applysidetone, enum_rdy_apl);

static struct snd_kcontrol_new ab8540_snd_controls[] = {
	SOC_ENUM("Headset Low Power Playback Switch", soc_enum_hslowpow),
	SOC_ENUM("Headset DAC Low Power Playback Switch", soc_enum_daclowpow1),
	SOC_ENUM("Headset DAC Drv Low Power Playback Switch",
		soc_enum_daclowpow0),
	SOC_ENUM("Earpiece High Pass Playback Switch",
		soc_enum_enhpear),
	SOC_ENUM("Earpiece DAC Low Power Playback Switch",
		soc_enum_eardaclowpow),
	SOC_ENUM("Earpiece DAC Drv Low Power Playback Switch",
		soc_enum_eardrvlowpow),
	SOC_ENUM("Earpiece Common Mode Playback Switch", soc_enum_earselcm),

	SOC_ENUM("Headset Fade Speed Playback Switch", soc_enum_hsfadspeed),

	SOC_ENUM("Charge Pump High Threshold For Low Voltage",
		soc_enum_envdeththre),
	SOC_ENUM("Charge Pump Low Threshold For Low Voltage",
		soc_enum_envdetlthre),
	SOC_ENUM("Charge Pump Envelope Detection", soc_enum_envdetcpen),
	SOC_ENUM("Charge Pump Envelope Detection Decay Time",
		soc_enum_envdettime),

	SOC_ENUM("Mic 1 Type Capture Switch", soc_enum_ensemic1),
	SOC_ENUM("Mic 2 Type Capture Switch", soc_enum_ensemic2),
	SOC_ENUM("Mic 3 Type Capture Switch", soc_enum_ensemic3),
	SOC_ENUM("Mic 1 Low Power Capture Switch", soc_enum_lowpowmic1),
	SOC_ENUM("Mic 2 Low Power Capture Switch", soc_enum_lowpowmic2),
	SOC_ENUM("Usb Mic Selection", soc_enum_usbmic),

	SOC_ENUM("LineIn High Pass Capture Switch", soc_enum_ad12nh),
	SOC_ENUM("Mic High Pass Capture Switch", soc_enum_ad34nh),
	SOC_ENUM("LineIn Mode Capture Switch", soc_enum_ad12voice),
	SOC_ENUM("Mic Mode Capture Switch", soc_enum_ad34voice),

	SOC_ENUM("Headset Mode Playback Switch", soc_enum_da12voice),
	SOC_ENUM("IHF Mode Playback Switch", soc_enum_da34voice),
	SOC_ENUM("Vibra Mode Playback Switch", soc_enum_da56voice),

	SOC_ENUM("IHF and Headset Swap Playback Switch", soc_enum_swapda12_34),

	SOC_ENUM("EPWM1 frequency", soc_enum_epwm1freq),
	SOC_ENUM("EPWM2 frequency", soc_enum_epwm2freq),
	SOC_ENUM("EPWM1 Type Modulation Switch", soc_enum_epwm1edge),
	SOC_ENUM("EPWM2 Type Modulation Switch", soc_enum_epwm2edge),
	SOC_ENUM("Signal selected for GPIO output", soc_enum_epwm2gpiosel),

	/* TODO: Cannot be changed on the fly with digital channel enabled. */

	SOC_ENUM("LineIn Filter Capture Switch", soc_enum_dmic12sinc),
	SOC_ENUM("Mic Filter Capture Switch", soc_enum_dmic34sinc),
	SOC_ENUM("HD Mic Filter Capture Switch", soc_enum_dmic56sinc),

	SOC_ENUM("Headset Source Playback Route", soc_enum_da2hslr),

	/* TODO: Cannot be changed on the fly with digital channel enabled. */
	SOC_ENUM("Headset Filter Playback Switch", soc_enum_hsesinc),

	SOC_ENUM("Digital Gain Fade Speed Switch", soc_enum_fadespeed),

	SOC_SINGLE("Vibra PWM Duty Cycle P Playback Volume",
		REG_PWMGENCONF3,
		REG_PWMGENCONFX_PWMVIBXDUTCYC,
		REG_PWMGENCONFX_PWMVIBXDUTCYC_MAX, NORMAL),
	SOC_SINGLE("Vibra PWM Duty Cycle N Playback Volume",
		REG_PWMGENCONF2,
		REG_PWMGENCONFX_PWMVIBXDUTCYC,
		REG_PWMGENCONFX_PWMVIBXDUTCYC_MAX, NORMAL),

	/* TODO: move to DAPM */
	SOC_ENUM("Sidetone Playback Switch", soc_enum_enfirsids),
	SOC_ENUM("IHF L and R Bridge Playback Route", soc_enum_parlhf),

	/* Digital gains for AD side */

	SOC_DOUBLE_R_TLV("LineIn Master Gain Capture Volume",
		REG_ADDIGGAIN1, REG_ADDIGGAIN2,
		0, REG_ADDIGGAINX_ADXGAIN_MAX, INVERT, adx_dig_gain_tlv),
	SOC_DOUBLE_R_TLV("Mic Master Gain Capture Volume",
		REG_ADDIGGAIN3, REG_ADDIGGAIN4,
		0, REG_ADDIGGAINX_ADXGAIN_MAX, INVERT, adx_dig_gain_tlv),
	SOC_DOUBLE_R_TLV("HD Mic Master Gain Capture Volume",
		REG_ADDIGGAIN5, REG_ADDIGGAIN6,
		0, REG_ADDIGGAINX_ADXGAIN_MAX, INVERT, adx_dig_gain_tlv),

	/* Digital gains for DA side */

	SOC_DOUBLE_R_TLV("Headset Master Gain Playback Volume",
		REG_DADIGGAIN1, REG_DADIGGAIN2,
		0, REG_DADIGGAINX_DAXGAIN_MAX, INVERT, dax_dig_gain_tlv),
	SOC_SINGLE_TLV("IHF Master Gain Playback Volume",
		REG_DADIGGAIN3,
		0, REG_DADIGGAINX_DAXGAIN_MAX, INVERT, dax_dig_gain_tlv),
	SOC_SINGLE_TLV("Vibra Master Gain Playback Volume",
		REG_DADIGGAIN4,
		0, REG_DADIGGAINX_DAXGAIN_MAX, INVERT, dax_dig_gain_tlv),
	SOC_DOUBLE_R_TLV("EPWM Master Gain Playback Volume",
		REG_DADIGGAIN5, REG_DADIGGAIN6,
		0, REG_DADIGGAINX_DAXGAIN_MAX, INVERT, dax_dig_gain_tlv),
	SOC_DOUBLE_R_TLV("Analog Loopback Gain Playback Volume",
		REG_ADDIGLOOPGAIN1, REG_ADDIGLOOPGAIN2,
		0, REG_ADDIGLOOPGAINX_ADXLBGAIN_MAX, INVERT, dax_dig_gain_tlv),
	SOC_DOUBLE_R_TLV("Headset Digital Gain Playback Volume",
		REG_HSLEARDIGGAIN, REG_HSRDIGGAIN,
		0, REG_HSLEARDIGGAIN_HSLDGAIN_MAX, INVERT, hs_ear_dig_gain_tlv),
	SOC_DOUBLE_R_TLV("Sidetone Digital Gain Playback Volume",
		REG_SIDFIRGAIN1, REG_SIDFIRGAIN2,
		0, REG_SIDFIRGAINX_FIRSIDXGAIN_MAX, INVERT, stfir_dig_gain_tlv),
	SOC_SINGLE_TLV("Handsfree Digital Gain Playback Volume",
		REG_HFGAIN,
		REG_HFGAIN_HF,
		REG_HFGAIN_HF_MAX, INVERT, hf_dig_gain_tlv),

	SOC_SINGLE_TLV("Vibra Digital Gain Playback Volume",
		REG_VIBGAIN,
		REG_VIBGAIN_GAIN,
		REG_VIBGAIN_GAIN_MAX, INVERT, vib_dig_gain_tlv),
	/* Analog gains */

	SOC_DOUBLE_TLV("Headset Gain Playback Volume",
		REG_ANAGAIN3,
		REG_ANAGAIN3_HSLGAIN, REG_ANAGAIN3_HSRGAIN,
		REG_ANAGAIN3_HSXGAIN_MAX, INVERT, hs_gain_tlv),

	SOC_SINGLE_TLV("Mic 1 Capture Volume",
		REG_ANAGAIN1,
		REG_ANAGAINX_MICXGAIN,
		REG_ANAGAINX_MICXGAIN_MAX, NORMAL, mic_gain_tlv),
	SOC_SINGLE_TLV("Mic 2 Capture Volume",
		REG_ANAGAIN2,
		REG_ANAGAINX_MICXGAIN,
		REG_ANAGAINX_MICXGAIN_MAX, NORMAL, mic_gain_tlv),
	SOC_DOUBLE_TLV("LineIn Capture Volume",
		REG_ANAGAIN4,
		REG_ANAGAIN4_LINLGAIN, REG_ANAGAIN4_LINRGAIN,
		REG_ANAGAIN4_LINXGAIN_MAX, NORMAL, lin_gain_tlv),
	SOC_DOUBLE_R_TLV("LineIn to Headset Bypass Playback Volume",
		REG_DIGLINHSLGAIN, REG_DIGLINHSRGAIN,
		REG_DIGLINHSXGAIN_LINTOHSXGAIN,
		REG_DIGLINHSXGAIN_LINTOHSXGAIN_MAX, INVERT, lin2hs_gain_tlv),

	SOC_SINGLE_TLV("Ear Analog Gain Playback Volume",
	    REG_EARGAINMICSEL,
	    REG_EARGAINMICSEL_GAIN,
	    REG_EARGAINMICSEL_GAIN_MAX, INVERT, ear_gain_tlv),
	/* Digital interface - Clocks */
	SOC_ENUM("Digital Interface Master Generator Switch", soc_enum_mastgen),
	SOC_ENUM("Digital Interface 0 Bit-clock Switch", soc_enum_fsbitclk0),
	SOC_ENUM("Digital Interface 1 Bit-clock Switch", soc_enum_fsbitclk1),

	/* Digital interface - DA from slot mapping */
	SOC_ENUM("Digital Interface DA 1 From Slot Map", soc_enum_da1slotmap),
	SOC_ENUM("Digital Interface DA 2 From Slot Map", soc_enum_da2slotmap),
	SOC_ENUM("Digital Interface DA 3 From Slot Map", soc_enum_da3slotmap),
	SOC_ENUM("Digital Interface DA 4 From Slot Map", soc_enum_da4slotmap),
	SOC_ENUM("Digital Interface DA 5 From Slot Map", soc_enum_da5slotmap),
	SOC_ENUM("Digital Interface DA 6 From Slot Map", soc_enum_da6slotmap),
	SOC_ENUM("Digital Interface DA 7 From Slot Map", soc_enum_da7slotmap),
	SOC_ENUM("Digital Interface DA 8 From Slot Map", soc_enum_da8slotmap),

	/* Digital interface - AD to slot mapping */
	SOC_ENUM("Digital Interface AD To Slot 0 Map", soc_enum_adslot0map),
	SOC_ENUM("Digital Interface AD To Slot 1 Map", soc_enum_adslot1map),
	SOC_ENUM("Digital Interface AD To Slot 2 Map", soc_enum_adslot2map),
	SOC_ENUM("Digital Interface AD To Slot 3 Map", soc_enum_adslot3map),
	SOC_ENUM("Digital Interface AD To Slot 4 Map", soc_enum_adslot4map),
	SOC_ENUM("Digital Interface AD To Slot 5 Map", soc_enum_adslot5map),
	SOC_ENUM("Digital Interface AD To Slot 6 Map", soc_enum_adslot6map),
	SOC_ENUM("Digital Interface AD To Slot 7 Map", soc_enum_adslot7map),
	SOC_ENUM("Digital Interface AD To Slot 8 Map", soc_enum_adslot8map),
	SOC_ENUM("Digital Interface AD To Slot 9 Map", soc_enum_adslot9map),
	SOC_ENUM("Digital Interface AD To Slot 10 Map", soc_enum_adslot10map),
	SOC_ENUM("Digital Interface AD To Slot 11 Map", soc_enum_adslot11map),
	SOC_ENUM("Digital Interface AD To Slot 12 Map", soc_enum_adslot12map),
	SOC_ENUM("Digital Interface AD To Slot 13 Map", soc_enum_adslot13map),
	SOC_ENUM("Digital Interface AD To Slot 14 Map", soc_enum_adslot14map),
	SOC_ENUM("Digital Interface AD To Slot 15 Map", soc_enum_adslot15map),
	SOC_ENUM("Digital Interface AD To Slot 16 Map", soc_enum_adslot16map),
	SOC_ENUM("Digital Interface AD To Slot 17 Map", soc_enum_adslot17map),
	SOC_ENUM("Digital Interface AD To Slot 18 Map", soc_enum_adslot18map),
	SOC_ENUM("Digital Interface AD To Slot 19 Map", soc_enum_adslot19map),
	SOC_ENUM("Digital Interface AD To Slot 20 Map", soc_enum_adslot20map),
	SOC_ENUM("Digital Interface AD To Slot 21 Map", soc_enum_adslot21map),
	SOC_ENUM("Digital Interface AD To Slot 22 Map", soc_enum_adslot22map),
	SOC_ENUM("Digital Interface AD To Slot 23 Map", soc_enum_adslot23map),
	SOC_ENUM("Digital Interface AD To Slot 24 Map", soc_enum_adslot24map),
	SOC_ENUM("Digital Interface AD To Slot 25 Map", soc_enum_adslot25map),
	SOC_ENUM("Digital Interface AD To Slot 26 Map", soc_enum_adslot26map),
	SOC_ENUM("Digital Interface AD To Slot 27 Map", soc_enum_adslot27map),
	SOC_ENUM("Digital Interface AD To Slot 28 Map", soc_enum_adslot28map),
	SOC_ENUM("Digital Interface AD To Slot 29 Map", soc_enum_adslot29map),
	SOC_ENUM("Digital Interface AD To Slot 30 Map", soc_enum_adslot30map),
	SOC_ENUM("Digital Interface AD To Slot 31 Map", soc_enum_adslot31map),

	/* Digital interface - Loopback */
	SOC_ENUM("Digital Interface AD 1 Loopback Switch", soc_enum_ad1loop),
	SOC_ENUM("Digital Interface AD 2 Loopback Switch", soc_enum_ad2loop),
	SOC_ENUM("Digital Interface AD 3 Loopback Switch", soc_enum_ad3loop),
	SOC_ENUM("Digital Interface AD 4 Loopback Switch", soc_enum_ad4loop),
	SOC_ENUM("Digital Interface AD 5 Loopback Switch", soc_enum_ad5loop),
	SOC_ENUM("Digital Interface AD 6 Loopback Switch", soc_enum_ad6loop),
	SOC_ENUM("Digital Interface AD 7 Loopback Switch", soc_enum_ad7loop),
	SOC_ENUM("Digital Interface AD 8 Loopback Switch", soc_enum_ad8loop),

	/* Digital interface - Burst FIFO */
	SOC_ENUM("Digital Interface 0 FIFO Enable Switch", soc_enum_if0fifoen),
	SOC_ENUM("Burst FIFO Mask", soc_enum_bfifomask),
	SOC_ENUM("Burst FIFO Bit-clock Frequency", soc_enum_bfifo19m2),
	SOC_SINGLE("Burst FIFO Threshold",
		REG_FIFOCONF1,
		REG_FIFOCONF1_BFIFOINT_SHIFT,
		REG_FIFOCONF1_BFIFOINT_MAX,
		NORMAL),
	SOC_SINGLE("Burst FIFO Length",
		REG_FIFOCONF2,
		REG_FIFOCONF2_BFIFOTX_SHIFT,
		REG_FIFOCONF2_BFIFOTX_MAX,
		NORMAL),
	SOC_SINGLE("Burst FIFO EOS Extra Slots",
		REG_FIFOCONF3,
		REG_FIFOCONF3_BFIFOEXSL_SHIFT,
		REG_FIFOCONF3_BFIFOEXSL_MAX,
		NORMAL),
	SOC_SINGLE("Burst FIFO FS Extra Bit-clocks",
		REG_FIFOCONF3,
		REG_FIFOCONF3_PREBITCLK0_SHIFT,
		REG_FIFOCONF3_PREBITCLK0_MAX,
		NORMAL),
	SOC_ENUM("Burst FIFO Interface Mode", soc_enum_bfifomast),
	SOC_ENUM("Burst FIFO Interface Switch", soc_enum_bfifoint),
	SOC_SINGLE("Burst FIFO Switch Frame Number",
		REG_FIFOCONF4,
		REG_FIFOCONF4_BFIFOFRAMSW_SHIFT,
		REG_FIFOCONF4_BFIFOFRAMSW_MAX,
		NORMAL),
	SOC_SINGLE("Burst FIFO Wake Up Delay",
		REG_FIFOCONF5,
		REG_FIFOCONF5_BFIFOWAKEUP_SHIFT,
		REG_FIFOCONF5_BFIFOWAKEUP_MAX,
		NORMAL),
	SOC_SINGLE("Burst FIFO Samples In FIFO",
		REG_FIFOCONF6,
		REG_FIFOCONF6_BFIFOSAMPLE_SHIFT,
		REG_FIFOCONF6_BFIFOSAMPLE_MAX,
		NORMAL),

	/* ANC */
	SOC_SINGLE_S1R("ANC Warp Delay Shift",
		REG_ANCCONF2,
		REG_ANCCONF2_VALUE_MIN,
		REG_ANCCONF2_VALUE_MAX,
		0),
	SOC_SINGLE_S1R("ANC FIR Output Shift",
		REG_ANCCONF3,
		REG_ANCCONF3_VALUE_MIN,
		REG_ANCCONF3_VALUE_MAX,
		0),
	SOC_SINGLE_S1R("ANC IIR Output Shift",
		REG_ANCCONF4,
		REG_ANCCONF4_VALUE_MIN,
		REG_ANCCONF4_VALUE_MAX,
		0),
	SOC_SINGLE_S2R("ANC Warp Delay",
		REG_ANCCONF9, REG_ANCCONF10,
		REG_ANC_WARP_DELAY_MIN,
		REG_ANC_WARP_DELAY_MAX,
		0),
	SOC_MULTIPLE_SA("ANC FIR Coefficients",
		anc_fir_cache,
		REG_ANC_FIR_COEFF_MIN,
		REG_ANC_FIR_COEFF_MAX,
		0),
	SOC_MULTIPLE_SA("ANC IIR Coefficients",
		anc_iir_cache,
		REG_ANC_IIR_COEFF_MIN,
		REG_ANC_IIR_COEFF_MAX,
		0),

	/* Sidetone */
	SOC_MULTIPLE_SA("Sidetone FIR Coefficients",
		sid_fir_cache,
		REG_SID_FIR_COEFF_MIN,
		REG_SID_FIR_COEFF_MAX,
		0),
	SOC_ENUM_EXT("Sidetone FIR Apply Coefficients",
		soc_enum_applysidetone,
		snd_soc_get_enum_double,
		sid_apply_control_put),

	/* Digital Mute */
	SOC_ENUM_EXT("Digital Interface Mute", soc_enum_digimute,
		digital_mute_control_get, digital_mute_control_put),

	/* DMIC frequeny */
	SOC_ENUM("DMic 1 and 2 Frequency", soc_enum_dmic12freq),
	SOC_ENUM("DMic 3 and 4 Frequency", soc_enum_dmic34freq),
	SOC_ENUM("DMic 5 and 6 Frequency", soc_enum_dmic56freq),

	/* AD12 to DA34 Stereo-Mono loop */

	SOC_ENUM("AD12 to DA34 loop mode", soc_enum_ad12mode),

	/* DA78 Mode */

	SOC_ENUM("DA78 mode", soc_enum_da78mode),

	/* Voice Interface */
	SOC_ENUM("Voice Interface RX1 From Slot Map", soc_enum_RXslotmap),
	SOC_ENUM("Voice Interface TX To Slot 32 Map", soc_enum_TXslot32map),
	SOC_ENUM("Voice Interface TX To Slot 33 Map", soc_enum_TXslot33map),
	SOC_ENUM("Voice Interface TX To Slot 34 Map", soc_enum_TXslot34map),
	SOC_ENUM("Voice Interface TX To Slot 35 Map", soc_enum_TXslot35map),
	SOC_ENUM("Voice Interface TX To Slot 36 Map", soc_enum_TXslot36map),
	SOC_ENUM("Voice Interface TX To Slot 37 Map", soc_enum_TXslot37map),
	SOC_ENUM("Voice Interface TX To Slot 38 Map", soc_enum_TXslot38map),
	SOC_ENUM("Voice Interface TX To Slot 39 Map", soc_enum_TXslot39map),
	SOC_ENUM("Voice Interface TX To Slot 40 Map", soc_enum_TXslot40map),
	SOC_ENUM("Voice Interface TX To Slot 41 Map", soc_enum_TXslot41map),
	SOC_ENUM("Voice Interface TX To Slot 42 Map", soc_enum_TXslot42map),
	SOC_ENUM("Voice Interface TX To Slot 43 Map", soc_enum_TXslot43map),
	SOC_ENUM("Voice Interface TX To Slot 44 Map", soc_enum_TXslot44map),
	SOC_ENUM("Voice Interface TX To Slot 45 Map", soc_enum_TXslot45map),
	SOC_ENUM("Voice Interface TX To Slot 46 Map", soc_enum_TXslot46map),
	SOC_ENUM("Voice Interface TX To Slot 47 Map", soc_enum_TXslot47map),
/*
	SOC_ENUM("Voice Interface RX1 Hi-Pass Playback Switch",
	soc_enum_rx1_hp_ena),
	SOC_ENUM("Voice Interface TX1 Hi-Pass Capture Switch",
	soc_enum_tx1_hp_ena),
	SOC_ENUM("Voice Interface TX2 Hi-Pass Capture Switch",
	soc_enum_tx2_hp_ena),
	SOC_ENUM("Voice Interface TX3 Hi-Pass Capture Switch",
	soc_enum_tx3_hp_ena),
	SOC_ENUM("Voice Interface TX4 Hi-Pass Capture Switch",
	soc_enum_tx4_hp_ena),
*/

	SOC_SINGLE_TLV("Voice Interface TX1 Gain Capture Volume",
		REG_TXDGAIN1, REG_TXDGAINX_TXXGAIN,
		REG_TXDGAINX_TXXGAIN_MAX, INVERT, voice_tx_gain_tlv),
	SOC_SINGLE_TLV("Voice Interface TX2 Gain Capture Volume",
		REG_TXDGAIN2, REG_TXDGAINX_TXXGAIN,
		REG_TXDGAINX_TXXGAIN_MAX, INVERT, voice_tx_gain_tlv),
	SOC_SINGLE_TLV("Voice Interface TX3 Gain Capture Volume",
		REG_TXDGAIN3, REG_TXDGAINX_TXXGAIN,
		REG_TXDGAINX_TXXGAIN_MAX, INVERT, voice_tx_gain_tlv),
	SOC_SINGLE_TLV("Voice Interface TX4 Gain Capture Volume",
		REG_TXDGAIN4, REG_TXDGAINX_TXXGAIN,
		REG_TXDGAINX_TXXGAIN_MAX, INVERT, voice_tx_gain_tlv),

	SOC_SINGLE_TLV("Voice Interface RX1 Gain Playback Volume",
		REG_RXDGAIN, REG_RXDGAIN_RX1GAIN,
		REG_RXDGAIN_RX1GAIN_MAX, INVERT, voice_rx_gain_tlv),

	SOC_ENUM("Voice Interface Master Generator Switch",
		soc_enum_mastgen_if3_ena),
	SOC_ENUM("Voice Interface Bit-clock Switch",
		soc_enum_fsbitclk3_ena),

	SOC_ENUM("Voice Interface RX1 Hi-Pass Filter Select",
		soc_enum_rx1_filter_selection),
	SOC_ENUM("Voice Interface TX12 Hi-Pass Filter Select",
		soc_enum_tx12_filter_selection),
	SOC_ENUM("Voice Interface TX34 Hi-Pass Filter Select",
		soc_enum_tx34_filter_selection),

	SOC_SINGLE_TLV("Voice Interface Sidetone Gain Volume",
		REG_TXRXSIDETONE2, REG_TXRXSIDETONE2_ST_PGA,
		REG_TXRXSIDETONE2_ST_PGA_MAX, INVERT, voice_sidetone_gain_tlv),

	/* EPWM1/EPWM2 configuration */
	SOC_SINGLE("EPWM1 DC A value", REG_EPWMACCONF1,
			REG_EPWMACCONF1_DCA, REG_EPWMACCONF1_DCA_MAX, 0),

	SOC_SINGLE("EPWM1 DC B value", REG_EPWMACCONF2,
			REG_EPWMACCONF2_DCB, REG_EPWMACCONF2_DCB_MAX, 0),

	SOC_SINGLE("EPWM1 AC Frequency", REG_EPWMACCONF3,
			REG_EPWMACCONF3_FREQ, REG_EPWMACCONF3_FREQ_MAX, 0),

	SOC_SINGLE("EPWM2 DC A value", REG_EPWMACCONF4,
			REG_EPWMACCONF4_DCA, REG_EPWMACCONF4_DCA_MAX, 0),

	SOC_SINGLE("EPWM2 DC B value", REG_EPWMACCONF5,
			REG_EPWMACCONF5_DCB, REG_EPWMACCONF5_DCB_MAX, 0),

	SOC_SINGLE("EPWM2 AC Frequency", REG_EPWMACCONF6,
			REG_EPWMACCONF6_FREQ, REG_EPWMACCONF6_FREQ_MAX, 0),

};

static int ab8540_codec_set_format_if2(struct snd_soc_codec *codec,
		unsigned int fmt)
{
	unsigned int clear_mask, set_mask;

	/* Master or slave */

	clear_mask = BMASK(REG_DIGIFCONF3_IF1MASTER);
	set_mask = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM: /* codec clk & FRM master */
		pr_debug("%s: IF2 Master-mode: AB8540 master.\n", __func__);
		set_mask |= BMASK(REG_DIGIFCONF3_IF1MASTER);
		break;
	case SND_SOC_DAIFMT_CBS_CFS: /* codec clk & FRM slave */
		pr_debug("%s: IF2 Master-mode: AB8540 slave.\n", __func__);
		break;
	case SND_SOC_DAIFMT_CBS_CFM: /* codec clk slave & FRM master */
	case SND_SOC_DAIFMT_CBM_CFS: /* codec clk master & frame slave */
		pr_err("%s: ERROR: The device is either a master or a slave.\n",
			__func__);
	default:
		pr_err("%s: ERROR: Unsupporter master mask 0x%x\n",
			__func__,
			fmt & SND_SOC_DAIFMT_MASTER_MASK);
		return -EINVAL;
	}

	ab8540_codec_update_reg_audio(codec,
				REG_DIGIFCONF3,
				BMASK(REG_DIGIFCONF3_IF1MASTER),
				BMASK(REG_DIGIFCONF3_IF1MASTER));

	/* I2S or TDM */

	clear_mask = BMASK(REG_DIGIFCONF4_FSYNC1P) |
			BMASK(REG_DIGIFCONF4_BITCLK1P) |
			BMASK(REG_DIGIFCONF4_IF1FORMAT1) |
			BMASK(REG_DIGIFCONF4_IF1FORMAT0);
	set_mask = 0;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S: /* I2S mode */
		pr_debug("%s: IF2 Protocol: I2S\n", __func__);
		set_mask |= BMASK(REG_DIGIFCONF4_IF1FORMAT1);
		break;
	case SND_SOC_DAIFMT_DSP_B: /* L data MSB during FRM LRC */
		pr_debug("%s: IF2 Protocol: DSP B (TDM)\n", __func__);
		set_mask |= BMASK(REG_DIGIFCONF4_IF1FORMAT0);
		break;
	default:
		pr_err("%s: ERROR: Unsupported format (0x%x)!\n",
			__func__,
			fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	ab8540_codec_update_reg_audio(codec, REG_DIGIFCONF4,
			clear_mask, set_mask);

	return 0;
}

/* Configures audio macrocell into the AB8540 Chip */
static void ab8540_codec_configure_audio_macrocell(struct snd_soc_codec *codec)
{
	int data, ret;

	ret = ab8500_sysctrl_write(AB8500_STW4500CTRL3,
		AB8500_STW4500CTRL3_CLK32KOUT2DIS |
		AB8500_STW4500CTRL3_RESETAUDN,
		AB8500_STW4500CTRL3_RESETAUDN);
	if (ret < 0)
		pr_err("%s: WARN: Unable to set reg STW4500CTRL3!\n", __func__);

	data = ab8540_codec_read_reg(codec, AB8500_MISC, AB8500_GPIO_DIR4_REG);
	data |= GPIO27_DIR_OUTPUT | GPIO29_DIR_OUTPUT | GPIO31_DIR_OUTPUT;
	ab8540_codec_write_reg(codec, AB8500_MISC, AB8500_GPIO_DIR4_REG, data);
}

/* Extended interface for codec-driver */

int ab8540_audio_power_control(bool power_on)
{
int pwr_mask = BMASK(REG_POWERUP_POWERUP) | BMASK(REG_POWERUP_ENANA);
	int data;

	if (ab8540_codec == NULL) {
		pr_err("%s: ERROR: AB8540 ASoC-driver not yet probed!\n",
				__func__);
		return -EIO;
	}

	pr_debug("%s AB8500.", (power_on) ? "Enabling" : "Disabling");


		if (power_on) { /* NAHJ_CTRL*/
	data = ab8540_codec_read_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_SEL7_REG);
	data |= GPIO7_sel;
	ab8540_codec_write_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_SEL7_REG, data);
	pr_debug("%s: AB8540_GPIO_SEL7_REG: %d .\n", __func__, data);

	data = ab8540_codec_read_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_DIR7_REG);
	data |= GPIO7_sel;
	ab8540_codec_write_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_DIR7_REG, data);
	pr_debug("%s: AB8540_GPIO_DIR7_REG: %d .\n", __func__, data);

	data = ab8540_codec_read_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_OUT7_REG);
	data &= ~GPIO7_sel;
	ab8540_codec_write_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_OUT7_REG, data);
	pr_debug("%s: AB8540_GPIO_OUT7_REG: %d .\n", __func__, data);

	data = ab8540_codec_read_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_PUD7_REG);
	data &= ~GPIO7_sel;
	ab8540_codec_write_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_PUD7_REG, data);
	pr_debug("%s: AB8540_GPIO_PUD7_REG: %d .\n", __func__, data);


	/* MIC_CTRL*/
	data = ab8540_codec_read_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_SEL2_REG);
	data |= GPIO2_sel;
	ab8540_codec_write_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_SEL2_REG, data);
	pr_debug("%s: AB8540_GPIO_SEL2_REG: %d .\n", __func__, data);

	data = ab8540_codec_read_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_DIR2_REG);
	data |= GPIO2_sel;
	ab8540_codec_write_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_DIR2_REG, data);
	pr_debug("%s: AB8540_GPIO_DIR2_REG: %d .\n", __func__, data);

	data = ab8540_codec_read_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_OUT2_REG);
	data |= GPIO2_sel;
	ab8540_codec_write_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_OUT2_REG, data);
	pr_debug("%s: AB8540_GPIO_OUT2_REG: %d .\n", __func__, data);

	data = ab8540_codec_read_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_PUD2_REG);
	data &= ~GPIO2_sel;
	ab8540_codec_write_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_PUD2_REG, data);
	pr_debug("%s: AB8540_GPIO_PUD2_REG: %d .\n", __func__, data);


	/* VIDEO_CTRL*/
	data = ab8540_codec_read_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_SEL2_REG);
	data |= GPIO3_sel;
	ab8540_codec_write_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_SEL2_REG, data);
	pr_debug("%s: AB8540_GPIO_SEL2_REG: %d .\n", __func__, data);

	data = ab8540_codec_read_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_DIR2_REG);
	data |= GPIO3_sel;
	ab8540_codec_write_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_DIR2_REG, data);
	pr_debug("%s: AB8540_GPIO_DIR2_REG: %d .\n", __func__, data);

	data = ab8540_codec_read_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_OUT2_REG);
	data &= ~GPIO3_sel;
	ab8540_codec_write_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_OUT2_REG, data);
	pr_debug("%s: AB8540_GPIO_OUT2_REG: %d .\n", __func__, data);

	data = ab8540_codec_read_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_PUD2_REG);
	data &= ~GPIO3_sel;
	ab8540_codec_write_reg(ab8540_codec, AB8500_MISC,
			AB8540_GPIO_PUD2_REG, data);
	pr_debug("%s: AB8540_GPIO_PUD2_REG: %d .\n", __func__, data);
		}

	return ab8540_codec_update_reg_audio(ab8540_codec, REG_POWERUP,
		pwr_mask, (power_on) ? pwr_mask : REG_MASK_NONE);
}

void ab8540_audio_pwm_vibra(unsigned char speed_pos,
			unsigned char speed_neg)
{
	unsigned int clear_mask, set_mask;
	bool vibra_on;

	if (ab8540_codec == NULL) {
		pr_err("%s: ERROR: AB8540 ASoC-driver not yet probed!\n",
				__func__);
		return;
	}

	vibra_on = speed_pos | speed_neg ;
	if (!vibra_on) {
		speed_pos = 0;
		speed_neg = 0;
	}

	pr_debug("%s: PWM-vibra (%d, %d).\n",
			__func__,
			speed_pos,
			speed_neg);

	set_mask = BMASK(REG_PWMGENCONF1_PWMTOVIB1) |
		BMASK(REG_PWMGENCONF1_PWM1CTRL) |
		BMASK(REG_PWMGENCONF1_PWM1NCTRL) |
		BMASK(REG_PWMGENCONF1_PWM1PCTRL);
	ab8540_codec_update_reg_audio(ab8540_codec, REG_PWMGENCONF1,
			0x00, set_mask);

	if (speed_pos > REG_PWMGENCONFX_PWMVIBXDUTCYC_MAX)
		speed_pos = REG_PWMGENCONFX_PWMVIBXDUTCYC_MAX;
	ab8540_codec_update_reg_audio(ab8540_codec, REG_PWMGENCONF3,
			REG_MASK_ALL, speed_pos);

	if (speed_neg > REG_PWMGENCONFX_PWMVIBXDUTCYC_MAX)
		speed_neg = REG_PWMGENCONFX_PWMVIBXDUTCYC_MAX;
	ab8540_codec_update_reg_audio(ab8540_codec, REG_PWMGENCONF2,
			REG_MASK_ALL, speed_neg);

	if (vibra_on) {
		clear_mask = 0;
		set_mask = BMASK(REG_ANACONF4_ENVIB1);
	} else {
		clear_mask = BMASK(REG_ANACONF4_ENVIB1);
		set_mask = 0;
	};
	ab8540_codec_update_reg_audio(ab8540_codec, REG_ANACONF4,
			clear_mask, set_mask);
}

void ab8540_audio_pwm_output(unsigned int pwm1_modulation,
		unsigned int pwm1_freq,
		unsigned int pwm2_modulation,
		unsigned int pwm2_freq)
{


	if (ab8540_codec == NULL) {
			pr_err("%s: ERROR: AB8540 ASoC-driver not yet probed!\n",
					__func__);
			return;
							}

	ab8540_codec_update_reg_audio(ab8540_codec, REG_EPWM1CONF,
			0x00, pwm1_modulation);

	ab8540_codec_update_reg_audio(ab8540_codec, REG_EPWM1CONF,
			0x0F, pwm1_freq);


	ab8540_codec_update_reg_audio(ab8540_codec, REG_EPWM2CONF,
			0x00, pwm2_modulation);

	ab8540_codec_update_reg_audio(ab8540_codec, REG_EPWM2CONF,
			0x0F, pwm2_freq);


}
int ab8540_audio_set_word_length(struct snd_soc_codec *codec,
		int if_id, unsigned int wl)
{
	unsigned int clear_mask, set_mask;
	int reg_id, bit0_id, bit1_id;

		switch (if_id) {
		case AB8540_IF1_DAI_ID:
			reg_id = REG_DIGIFCONF2;
			bit0_id = REG_DIGIFCONF2_IF0WL0;
			bit1_id = REG_DIGIFCONF2_IF0WL1;
			break;
		case AB8540_IF2_DAI_ID:
			reg_id = REG_DIGIFCONF4;
			bit0_id = REG_DIGIFCONF4_IF1WL0;
			bit1_id = REG_DIGIFCONF4_IF1WL1;
			break;
		case AB8540_IF3_DAI_ID:
			reg_id = REG_DIGIFCONF6;
			bit0_id = REG_DIGIFCONF6_IF3WL0;
			bit1_id = REG_DIGIFCONF6_IF3WL1;
			break;
		default:
			pr_err("%s: ERROR: Unknown interface id:%d\n",
				__func__, if_id);
			return -EINVAL;
	}

	clear_mask = BMASK(bit0_id)|BMASK(bit1_id);
	set_mask = 0;

	switch (wl) {
	case 16:
		break;
	case 20:
		set_mask |= BMASK(bit0_id);
		break;
	case 24:
		set_mask |= BMASK(bit1_id);
		break;
	case 32:
		set_mask |= BMASK(bit1_id) | BMASK(bit0_id);
		break;
	default:
		pr_err("%s: Unsupported word-length 0x%x\n",
				__func__, wl);
		return -EINVAL;
	}

	pr_debug("%s: IF%d Word-length: %d bits.\n",
			__func__, if_id, wl);
	ab8540_codec_update_reg_audio(codec, reg_id,
			clear_mask, set_mask);

	return 0;
}

int ab8540_audio_set_bit_delay(struct snd_soc_codec *codec,
		int if_id, unsigned int delay)
{
	unsigned int clear_mask, set_mask;
	int reg_id, bit_id;

		switch (if_id) {
		case AB8540_IF1_DAI_ID:
			reg_id = REG_DIGIFCONF2;
			bit_id = REG_DIGIFCONF2_IF0DEL;
			break;
		case AB8540_IF2_DAI_ID:
			reg_id = REG_DIGIFCONF4;
			bit_id = REG_DIGIFCONF4_IF1DEL;
			break;
		case AB8540_IF3_DAI_ID:
			reg_id = REG_DIGIFCONF6;
			bit_id = REG_DIGIFCONF6_IF3DEL;
			break;
		default:
			pr_err("%s: ERROR: Unknown interface id:%d\n",
				__func__, if_id);
			return -EINVAL;
	}

	clear_mask = BMASK(bit_id);
	set_mask = 0;

	switch (delay) {
	case 0:
		break;
	case 1:
		set_mask |= BMASK(bit_id);
		break;
	default:
		pr_err("%s: ERROR: Unsupported bit-delay (0x%x)!\n",
				__func__, delay);
		return -EINVAL;
	}

	pr_debug("%s: IF%d Bit-delay: %d bits.\n",
			__func__, if_id, delay);
	ab8540_codec_update_reg_audio(codec, reg_id,
			clear_mask, set_mask);

	return 0;
}

int ab8540_audio_setup_if2(struct snd_soc_dai *dai,
			unsigned int fmt,
			unsigned int wl,
			unsigned int delay)
{
	int ret;

	pr_debug("%s: Enter.\n", __func__);

	ret = ab8540_codec_set_format_if2(dai->codec, fmt);
	if (ret)
		return -1;

	ret = ab8540_audio_set_bit_delay(dai->codec, AB8540_IF2_DAI_ID, delay);
	if (ret)
		return -1;


	ret = ab8540_audio_set_word_length(dai->codec, AB8540_IF2_DAI_ID, wl);
	if (ret)
		return -1;

	return 0;
}

/* ANC FIR-coefficients configuration sequence */
static void ab8540_audio_anc_fir(struct snd_soc_codec *codec,
		unsigned int bank, unsigned int param)
{
	if (param == 0 && bank == 0)
		snd_soc_update_bits(codec, REG_ANCCONF1,
			REG_MASK_NONE, BMASK(REG_ANCCONF1_ANCFIRUPDATE));

	snd_soc_write(codec, REG_ANCCONF5,
		anc_fir_cache[param] >> 8 & REG_MASK_ALL);
	snd_soc_write(codec, REG_ANCCONF6,
		anc_fir_cache[param] & REG_MASK_ALL);

	if (param == REG_ANC_FIR_COEFFS - 1 && bank == 1)
		snd_soc_update_bits(codec, REG_ANCCONF1,
			BMASK(REG_ANCCONF1_ANCFIRUPDATE), REG_MASK_NONE);
}

/* ANC IIR-coefficients configuration sequence */
static void ab8540_audio_anc_iir(struct snd_soc_codec *codec,
		unsigned int bank, unsigned int param)
{
	if (param == 0) {
		if (bank == 0) {
			snd_soc_update_bits(codec, REG_ANCCONF1,
				REG_MASK_NONE, BMASK(REG_ANCCONF1_ANCIIRINIT));
			usleep_range(AB8540_ANC_SM_DELAY, AB8540_ANC_SM_DELAY);
			snd_soc_update_bits(codec, REG_ANCCONF1,
				BMASK(REG_ANCCONF1_ANCIIRINIT), REG_MASK_NONE);
			usleep_range(AB8540_ANC_SM_DELAY, AB8540_ANC_SM_DELAY);
		} else {
			snd_soc_update_bits(codec, REG_ANCCONF1,
				REG_MASK_NONE,
				BMASK(REG_ANCCONF1_ANCIIRUPDATE));
		}
	} else if (param > 3) {
		snd_soc_write(codec, REG_ANCCONF7, REG_MASK_NONE);
		snd_soc_write(codec, REG_ANCCONF8,
			anc_iir_cache[param] >> 16 & REG_MASK_ALL);
	}

	snd_soc_write(codec, REG_ANCCONF7,
		anc_iir_cache[param] >> 8 & REG_MASK_ALL);
	snd_soc_write(codec, REG_ANCCONF8,
		anc_iir_cache[param] & REG_MASK_ALL);

	if (param == REG_ANC_IIR_COEFFS - 1 && bank == 1)
		snd_soc_update_bits(codec, REG_ANCCONF1,
			BMASK(REG_ANCCONF1_ANCIIRUPDATE), REG_MASK_NONE);
}

/* ANC IIR-/FIR-coefficients configuration sequence */
void ab8540_audio_anc_configure(struct snd_soc_codec *codec,
		bool apply_fir, bool apply_iir)
{
	unsigned int bank, param;

pr_debug("%s: Enter.\n", __func__);

	if (apply_fir)
		snd_soc_update_bits(codec, REG_ANCCONF1,
			BMASK(REG_ANCCONF1_ENANC), REG_MASK_NONE);

	snd_soc_update_bits(codec, REG_ANCCONF1,
		REG_MASK_NONE, BMASK(REG_ANCCONF1_ENANC));

	if (apply_fir)
		for (bank = 0; bank < AB8540_NR_OF_ANC_COEFF_BANKS; bank++)
			for (param = 0; param < REG_ANC_FIR_COEFFS; param++)
				ab8540_audio_anc_fir(codec, bank, param);

	if (apply_iir)
		for (bank = 0; bank < AB8540_NR_OF_ANC_COEFF_BANKS; bank++)
			for (param = 0; param < REG_ANC_IIR_COEFFS; param++)
				ab8540_audio_anc_iir(codec, bank, param);

	pr_debug("%s: Exit.\n", __func__);
}

int ab8540_audio_set_adcm(enum ab8540_audio_adcm req_adcm)
{

	if (ab8540_codec == NULL) {
		pr_err("%s: ERROR: AB8540 ASoC-driver not yet probed!\n",
				__func__);
		return -EIO;
	}

	if (adcm == req_adcm)
		return 0;

	pr_debug("%s: Enter.\n", __func__);
	if (AB8540_AUDIO_ADCM_FORCE_UP == req_adcm ||
			AB8540_AUDIO_ADCM_FORCE_DOWN == req_adcm) {

		mutex_lock(&ab8540_codec->mutex);

		adcm_anaconf5 = snd_soc_read(ab8540_codec,
				REG_ANACONF5);
		adcm_muteconf = snd_soc_read(ab8540_codec,
				REG_MUTECONF);
		adcm_anaconf4 = snd_soc_read(ab8540_codec,
				REG_ANACONF4);

		if (AB8540_AUDIO_ADCM_FORCE_UP == req_adcm) {
			snd_soc_update_bits(ab8540_codec,
					REG_ANACONF5, REG_MASK_NONE,
					ADCM_ANACONF5_MASK);
			snd_soc_update_bits(ab8540_codec,
					REG_MUTECONF, REG_MASK_NONE,
					ADCM_MUTECONF_MASK);
			snd_soc_update_bits(ab8540_codec,
					REG_ANACONF4, REG_MASK_NONE,
					ADCM_ANACONF4_MASK);

		} else {
			snd_soc_update_bits(ab8540_codec,
					REG_ANACONF5, ADCM_ANACONF5_MASK,
					REG_MASK_NONE);
		}
	} else if (AB8540_AUDIO_ADCM_NORMAL == req_adcm) {

		if (AB8540_AUDIO_ADCM_FORCE_UP == adcm) {
			snd_soc_update_bits(ab8540_codec,
					REG_ANACONF5,
					~adcm_anaconf5 & ADCM_ANACONF5_MASK,
					adcm_anaconf5 & ADCM_ANACONF5_MASK);
			snd_soc_update_bits(ab8540_codec,
					REG_MUTECONF,
					~adcm_muteconf & ADCM_MUTECONF_MASK,
					adcm_muteconf & ADCM_MUTECONF_MASK);
			snd_soc_update_bits(ab8540_codec,
					REG_ANACONF4,
					~adcm_anaconf4 & ADCM_ANACONF4_MASK,
					adcm_anaconf4 & ADCM_ANACONF4_MASK);
		} else {
			snd_soc_update_bits(ab8540_codec,
					REG_ANACONF5,
					~adcm_anaconf5 & ADCM_ANACONF5_MASK,
					adcm_anaconf5 & ADCM_ANACONF5_MASK);
		}
	}

	adcm = req_adcm;


	if (AB8540_AUDIO_ADCM_NORMAL == adcm)
		mutex_unlock(&ab8540_codec->mutex);

	pr_debug("%s: Exit.\n", __func__);

	return 0;
}

static int ab8540_codec_add_widgets(struct snd_soc_codec *codec)
{
	int ret;

	ret = snd_soc_dapm_new_controls(&codec->dapm, ab8540_dapm_widgets,
			ARRAY_SIZE(ab8540_dapm_widgets));
	if (ret < 0) {
		pr_err("%s: Failed to create DAPM controls (%d).\n",
			__func__, ret);
		return ret;
	}

	ret = snd_soc_dapm_add_routes(&codec->dapm, dapm_routes,
			ARRAY_SIZE(dapm_routes));
	if (ret < 0) {
		pr_err("%s: Failed to add DAPM routes (%d).\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int ab8540_codec_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params, struct snd_soc_dai *dai)
{
	pr_debug("%s Enter.\n", __func__);

	return 0;
}

static int ab8540_codec_pcm_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	pr_debug("%s Enter.\n", __func__);

	return 0;
}

static int ab8540_codec_pcm_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	pr_debug("%s Enter.\n", __func__);

	return 0;
}

static void ab8540_codec_pcm_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	pr_debug("%s Enter.\n", __func__);
}

static int ab8540_codec_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	pr_err("%s Enter.\n", __func__);

	return 0;
}

/* Gates clocking according format mask */
static int ab8540_codec_set_dai_clock_gate(struct snd_soc_codec *codec,
		unsigned int fmt)
{
	unsigned int clear_mask;
	unsigned int set_mask;

	clear_mask = BMASK(REG_DIGIFCONF1_ENMASTGEN) |
			BMASK(REG_DIGIFCONF1_ENFSBITCLK0);

	set_mask = BMASK(REG_DIGIFCONF1_ENMASTGEN);

	switch (fmt & SND_SOC_DAIFMT_CLOCK_MASK) {
	case SND_SOC_DAIFMT_CONT: /* continuous clock */
		pr_debug("%s: IF1 Clock is continuous.\n", __func__);
		set_mask |= BMASK(REG_DIGIFCONF1_ENFSBITCLK0);
		break;
	case SND_SOC_DAIFMT_GATED: /* clock is gated */
		pr_debug("%s: IF1 Clock is gated.\n", __func__);
		break;
	default:
		pr_err("%s: ERROR: Unsupported clock mask (0x%x)!\n",
			__func__,
			fmt & SND_SOC_DAIFMT_CLOCK_MASK);
		return -EINVAL;
	}

	ab8540_codec_update_reg_audio(codec, REG_DIGIFCONF1,
			clear_mask, set_mask);

	return 0;
}

static int ab8540_codec_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	unsigned int clear_mask;
	unsigned int set_mask;
	struct snd_soc_codec *codec = dai->codec;
	int err;

	pr_debug("%s: Enter (fmt = 0x%x)\n", __func__, fmt);

	clear_mask = BMASK(REG_DIGIFCONF3_IF1DATOIF0AD) |
			BMASK(REG_DIGIFCONF3_IF1CLKTOIF0CLK) |
			BMASK(REG_DIGIFCONF3_IF0BFIFOEN) |
			BMASK(REG_DIGIFCONF3_IF0MASTER);
	set_mask = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM: /* codec clk & FRM master */
		pr_debug("%s: IF1 Master-mode: AB8540 master.\n", __func__);
		set_mask |= BMASK(REG_DIGIFCONF3_IF0MASTER);
		break;
	case SND_SOC_DAIFMT_CBS_CFS: /* codec clk & FRM slave */
		pr_debug("%s: IF1 Master-mode: AB8540 slave.\n", __func__);
		break;
	case SND_SOC_DAIFMT_CBS_CFM: /* codec clk slave & FRM master */
	case SND_SOC_DAIFMT_CBM_CFS: /* codec clk master & frame slave */
		pr_err("%s: ERROR: The device is either a master or a slave.\n",
				__func__);
	default:
		pr_err("%s: ERROR: Unsupporter master mask 0x%x\n",
				__func__,
				(fmt & SND_SOC_DAIFMT_MASTER_MASK));
		return -EINVAL;
		break;
	}

	ab8540_codec_update_reg_audio(codec, REG_DIGIFCONF3,
			clear_mask, set_mask);

	/* Set clock gating */
	err = ab8540_codec_set_dai_clock_gate(codec, fmt);
	if (err) {
		pr_err("%s: ERRROR: Failed to set clock gate (%d).\n",
				__func__, err);
		return err;
	}

	/* Setting data transfer format */

	clear_mask = BMASK(REG_DIGIFCONF2_IF0FORMAT0) |
		BMASK(REG_DIGIFCONF2_IF0FORMAT1) |
		BMASK(REG_DIGIFCONF2_FSYNC0P) |
		BMASK(REG_DIGIFCONF2_BITCLK0P);
	set_mask = 0;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S: /* I2S mode */
		pr_debug("%s: IF1 Protocol: I2S\n", __func__);
		set_mask |= BMASK(REG_DIGIFCONF2_IF0FORMAT1);

		/* 32 bit, 0 delay */
		ab8540_audio_set_word_length(dai->codec, AB8540_IF1_DAI_ID, 32);
		ab8540_audio_set_bit_delay(dai->codec, AB8540_IF1_DAI_ID, 0);

		break;
	case SND_SOC_DAIFMT_DSP_A: /* L data MSB after FRM LRC */
		pr_debug("%s: IF1 Protocol: DSP A (TDM)\n", __func__);
		set_mask |= BMASK(REG_DIGIFCONF2_IF0FORMAT0);
		break;
	case SND_SOC_DAIFMT_DSP_B: /* L data MSB during FRM LRC */
		pr_debug("%s: IF1 Protocol: DSP B (TDM)\n", __func__);
		set_mask |= BMASK(REG_DIGIFCONF2_IF0FORMAT0);
		break;
	default:
		pr_err("%s: ERROR: Unsupported format (0x%x)!\n",
				__func__,
				fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF: /* normal bit clock + frame */
		pr_debug("%s: IF1: Normal bit clock, normal frame\n",
				__func__);
		break;
	case SND_SOC_DAIFMT_NB_IF: /* normal BCLK + inv FRM */
		pr_debug("%s: IF1: Normal bit clock, inverted frame\n",
				__func__);
		set_mask |= BMASK(REG_DIGIFCONF2_FSYNC0P);
		break;
	case SND_SOC_DAIFMT_IB_NF: /* invert BCLK + nor FRM */
		pr_debug("%s: IF1: Inverted bit clock, normal frame\n",
				__func__);
		set_mask |= BMASK(REG_DIGIFCONF2_BITCLK0P);
		break;
	case SND_SOC_DAIFMT_IB_IF: /* invert BCLK + FRM */
		pr_debug("%s: IF1: Inverted bit clock, inverted frame\n",
				__func__);
		set_mask |= BMASK(REG_DIGIFCONF2_FSYNC0P);
		set_mask |= BMASK(REG_DIGIFCONF2_BITCLK0P);
		break;
	default:
		pr_err("%s: ERROR: Unsupported INV mask 0x%x\n",
				__func__,
				(fmt & SND_SOC_DAIFMT_INV_MASK));
		return -EINVAL;
		break;
	}

	ab8540_codec_update_reg_audio(codec, REG_DIGIFCONF2,
			clear_mask, set_mask);

	return 0;
}

static int ab8540_codec_set_dai_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int set_mask, clear_mask, slots_active, i;

	if (!(slot_width == 16 || slot_width == 32 || slot_width == 20)) {
		pr_err("%s: ERROR: Unsupported slot_width %d.\n",
			__func__, slot_width);
		return -EINVAL;
	}

	/* Setup TDM bitclock */
	pr_debug("%s: Slots, total: (%d) slot_width: (%d)\n",
		__func__,
		slots,
		slot_width);

	clear_mask = BMASK(REG_DIGIFCONF1_IF0BITCLKOS0) |
			BMASK(REG_DIGIFCONF1_IF0BITCLKOS1);

	i = slots * slot_width;

	if (i > 128)
		set_mask = BMASK(REG_DIGIFCONF1_IF0BITCLKOS0) |
				BMASK(REG_DIGIFCONF1_IF0BITCLKOS1);
	else if (i > 64)
		set_mask = BMASK(REG_DIGIFCONF1_IF0BITCLKOS1);
	else if (i > 32)
		set_mask = BMASK(REG_DIGIFCONF1_IF0BITCLKOS0);
	else
		set_mask = REG_MASK_NONE;

	ab8540_codec_update_reg_audio(codec, REG_DIGIFCONF1,
			clear_mask, set_mask);

	clear_mask = REG_DASLOTCONFX_SLTODAX_MASK;
	slots_active = hweight32(tx_mask);
	pr_debug("%s: Slots: (%d), TX: (%d)\n", __func__,
			slots_active, tx_mask);
	switch (slots_active) {
	case 0:
		break;
	case 1:
		i = find_first_bit((long unsigned int*)&tx_mask,
				sizeof(tx_mask));
		pr_debug("%s: slot %d", __func__, (slots+i));
		ab8540_codec_update_reg_audio(codec, REG_DASLOTCONF1,
				clear_mask, slots+i);
		ab8540_codec_update_reg_audio(codec, REG_DASLOTCONF2,
				clear_mask, slots+i);
		ab8540_codec_update_reg_audio(codec, REG_DASLOTCONF3,
				clear_mask, slots+i);
		ab8540_codec_update_reg_audio(codec, REG_DASLOTCONF4,
				clear_mask, slots+i);
		break;
	case 2:
		i = find_first_bit((long unsigned int*)&tx_mask,
				sizeof(tx_mask));
		pr_debug("%s: slot %d", __func__, (slots+i));
		ab8540_codec_update_reg_audio(codec, REG_DASLOTCONF1,
				clear_mask, slots+i);
		ab8540_codec_update_reg_audio(codec, REG_DASLOTCONF3,
				clear_mask, slots+i);
		i = find_next_bit((long unsigned int*)&tx_mask,
				sizeof(tx_mask), i+1);
		ab8540_codec_update_reg_audio(codec, REG_DASLOTCONF2,
				clear_mask, slots+i);
		ab8540_codec_update_reg_audio(codec, REG_DASLOTCONF4,
				clear_mask, slots+i);
		break;
	case 8:
		pr_debug("%s: In 8-channel mode DA-from-slot mapping"
				"is set manually.", __func__);
		break;
	default:
		pr_err("%s: Unsupported number of active TX-slots (%d)!\n",
				__func__, slots_active);
		return -EINVAL;
	}

	/* Setup TDM AD according to active RX-slots */
	slots_active = hweight32(rx_mask);
	pr_debug("%s: Slots, active, RX: %d\n", __func__, slots_active);
	switch (slots_active) {
	case 0:
		break;
	case 1:
		/* AD_OUT3 -> slot 0 & 1 */
		ab8540_codec_update_reg_audio(codec, REG_ADSLOTSEL1,
			REG_MASK_ALL,
			REG_ADSLOTSELX_AD_OUT3_TO_SLOT_EVEN |
			REG_ADSLOTSELX_AD_OUT3_TO_SLOT_ODD);
		break;
	case 2:
		/* AD_OUT3 -> slot 0, AD_OUT2 -> slot 1 */
		ab8540_codec_update_reg_audio(codec, REG_ADSLOTSEL1,
			REG_MASK_ALL,
			REG_ADSLOTSELX_AD_OUT3_TO_SLOT_EVEN |
			REG_ADSLOTSELX_AD_OUT2_TO_SLOT_ODD);
		break;
	case 8:
		pr_debug("%s: In 8-channel mode AD-to-slot mapping is set manually.",
				__func__);
		break;
	default:
		pr_err("%s: Unsupported number of active RX-slots (%d)!\n",
				__func__, slots_active);
		return -EINVAL;
	}

	return 0;
}

/* Voice Interface DAI configuration functions */
static int ab8540_codec_voice_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int clear_mask;
	unsigned int set_mask;

	pr_err("%s Enter.\n", __func__);

	clear_mask = BMASK(REG_DIGIFCONF5_FSYNC3_FREQ1) |
			BMASK(REG_DIGIFCONF5_FSYNC3_FREQ0);

	switch (freq) {
	case 8000:
		set_mask = REG_MASK_NONE;
		break;
	case 16000:
		set_mask = BMASK(REG_DIGIFCONF5_FSYNC3_FREQ0);
		break;
	case 44100:
		set_mask = BMASK(REG_DIGIFCONF5_FSYNC3_FREQ1);
		break;
	case 48000:
		set_mask = BMASK(REG_DIGIFCONF5_FSYNC3_FREQ1) |
			BMASK(REG_DIGIFCONF5_FSYNC3_FREQ0);
		break;
	default:
		pr_err("%s: ERROR: Unsupported sample rate (%d)!\n",
				__func__, freq);
		return -EINVAL;
	}
	ab8540_codec_update_reg_audio(codec, REG_DIGIFCONF5,
			clear_mask, set_mask);
	return 0;
}

static int ab8540_codec_voice_set_dai_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int set_mask, clear_mask;
	unsigned int nbBitsPerFrame;

	/* Only 16 bit slot width is supported at the moment in TDM mode */
	if (!(slot_width == 16 || slot_width == 32)) {
		pr_err("%s: ERROR: Unsupported slot_width %d.\n",
			__func__, slot_width);
		return -EINVAL;
	}

	/* Only 2, 4 or 8 slots  is supported at the moment in TDM mode */
	if (!(slots == 2 || slots == 4 || slots == 8)) {
		pr_err("%s: ERROR: Unsupported number of slots %d.\n",
			__func__, slots);
		return -EINVAL;
	}

	/* Compute number of Bits per Frame */
	nbBitsPerFrame = slot_width * slots;

	/* Setup TDM clocking according to slot count */
	pr_debug("%s: Slots:%d slot_width:%d\n", __func__, slots, slot_width);
	pr_debug("%s: nbBitsPerFrame:%d\n", __func__, nbBitsPerFrame);
	clear_mask = BMASK(REG_DIGIFCONF5_IF3_BITCLK_OSR0) |
			BMASK(REG_DIGIFCONF5_IF3_BITCLK_OSR1);
	switch (nbBitsPerFrame) {
	case 32:
		set_mask = REG_MASK_NONE;
		break;
	case 64:
		set_mask = BMASK(REG_DIGIFCONF5_IF3_BITCLK_OSR0);
		break;
	case 128:
		set_mask = BMASK(REG_DIGIFCONF5_IF3_BITCLK_OSR1);
		break;
	case 256:
		set_mask = BMASK(REG_DIGIFCONF5_IF3_BITCLK_OSR0) |
				BMASK(REG_DIGIFCONF5_IF3_BITCLK_OSR1);
		break;
	default:
		pr_err("%s: ERROR: Unsupported number of bits per Frame (%d)!\n",
				__func__, nbBitsPerFrame);
		return -EINVAL;
	}
	ab8540_codec_update_reg_audio(codec, REG_DIGIFCONF5,
			clear_mask, set_mask);
	return 0;
}

static int ab8540_codec_voice_set_dai_fmt(struct snd_soc_dai *dai,
		unsigned int fmt)
{
	unsigned int clear_mask;
	unsigned int set_mask;
	struct snd_soc_codec *codec = dai->codec;

	pr_debug("%s: Enter (fmt = 0x%x)\n", __func__, fmt);

	/* REG_DIGIFCONF5 configuration */
	clear_mask = BMASK(REG_DIGIFCONF5_ENMASTGEN_IF3) |
			BMASK(REG_DIGIFCONF5_IF3MASTER) |
			BMASK(REG_DIGIFCONF5_ENFSBITCLK3);
	set_mask = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM: /* codec clk & FRM master */
		pr_debug("%s: IF%d Master-mode: AB8540 master.\n",
				__func__, dai->id);
		set_mask |= BMASK(REG_DIGIFCONF5_ENMASTGEN_IF3);
		set_mask |= BMASK(REG_DIGIFCONF5_IF3MASTER);
		break;
	case SND_SOC_DAIFMT_CBS_CFS: /* codec clk & FRM slave */
		pr_debug("%s: IF%d Master-mode: AB8540 slave.\n",
				__func__, dai->id);
		break;
	case SND_SOC_DAIFMT_CBS_CFM: /* codec clk slave & FRM master */
	case SND_SOC_DAIFMT_CBM_CFS: /* codec clk master & frame slave */
		pr_err("%s: ERROR: The device is either a master or a slave.\n",
				__func__);
	default:
		pr_err("%s: ERROR: Unsupporter master mask 0x%x\n",
				__func__,
				(fmt & SND_SOC_DAIFMT_MASTER_MASK));
		return -EINVAL;
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_CLOCK_MASK) {
	case SND_SOC_DAIFMT_CONT: /* continuous clock */
		pr_debug("%s: IF%d Clock is continuous.\n", __func__, dai->id);
		set_mask |= BMASK(REG_DIGIFCONF5_ENFSBITCLK3);
		break;
	case SND_SOC_DAIFMT_GATED: /* clock is gated */
		pr_debug("%s: IF%d Clock is gated.\n", __func__, dai->id);
		break;
	default:
		pr_err("%s: ERROR: Unsupported clock mask (0x%x)!\n",
			__func__,
			fmt & SND_SOC_DAIFMT_CLOCK_MASK);
		return -EINVAL;
	}

	ab8540_codec_update_reg_audio(codec, REG_DIGIFCONF5,
			clear_mask, set_mask);

	/* REG_DIGIFCONF6 configuration */
	clear_mask = BMASK(REG_DIGIFCONF6_IF3FORMAT0) |
		BMASK(REG_DIGIFCONF6_IF3FORMAT1) |
		BMASK(REG_DIGIFCONF6_FSYNC3P) |
		BMASK(REG_DIGIFCONF6_BITCLK3P);
	set_mask = 0;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S: /* I2S mode */
		pr_debug("%s: IF%d Protocol: I2S\n", __func__, dai->id);
		set_mask |= BMASK(REG_DIGIFCONF6_IF3FORMAT1);

		/* 32 bit, 0 delay */
		ab8540_audio_set_word_length(dai->codec, AB8540_IF3_DAI_ID, 32);
		ab8540_audio_set_bit_delay(dai->codec, AB8540_IF3_DAI_ID, 0);

		break;
	case SND_SOC_DAIFMT_DSP_A: /* L data MSB after FRM LRC */
		pr_debug("%s: IF%d Protocol: DSP A (TDM)\n",
				__func__ , dai->id);
		set_mask |= BMASK(REG_DIGIFCONF6_IF3FORMAT0);
		break;
	case SND_SOC_DAIFMT_DSP_B: /* L data MSB during FRM LRC */
		pr_debug("%s: IF%d Protocol: DSP B (TDM)\n", __func__, dai->id);
		set_mask |= BMASK(REG_DIGIFCONF6_IF3FORMAT0);
		break;
	default:
		pr_err("%s: ERROR: Unsupported format (0x%x)!\n",
				__func__,
				fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF: /* normal bit clock + frame */
		pr_debug("%s: IF%d Normal bit clock, normal frame\n",
				__func__, dai->id);
		break;
	case SND_SOC_DAIFMT_NB_IF: /* normal BCLK + inv FRM */
		pr_debug("%s: IF%d Normal bit clock, inverted frame\n",
				__func__, dai->id);
		set_mask |= BMASK(REG_DIGIFCONF6_FSYNC3P);
		break;
	case SND_SOC_DAIFMT_IB_NF: /* invert BCLK + nor FRM */
		pr_debug("%s: IF%d Inverted bit clock, normal frame\n",
				__func__, dai->id);
		set_mask |= BMASK(REG_DIGIFCONF6_BITCLK3P);
		break;
	case SND_SOC_DAIFMT_IB_IF: /* invert BCLK + FRM */
		pr_debug("%s: IF%d Inverted bit clock, inverted frame\n",
				__func__, dai->id);
		set_mask |= BMASK(REG_DIGIFCONF6_FSYNC3P);
		set_mask |= BMASK(REG_DIGIFCONF6_BITCLK3P);
		break;
	default:
		pr_err("%s: ERROR: Unsupported INV mask 0x%x\n",
				__func__,
				(fmt & SND_SOC_DAIFMT_INV_MASK));
		return -EINVAL;
		break;
	}
	ab8540_codec_update_reg_audio(codec, REG_DIGIFCONF6,
			clear_mask, set_mask);

	return 0;
}

struct snd_soc_dai_driver ab8540_codec_dai[] = {
	{
		.name = "ab8540-codec-dai.0",
		.id = AB8540_IF1_DAI_ID,
		.playback = {
			.stream_name = "ab8540_0p",
			.channels_min = 1,
			.channels_max = 8,
			.rates = AB8540_SUPPORTED_RATE,
			.formats = AB8540_SUPPORTED_FMT,
		},
		.ops = (struct snd_soc_dai_ops[]) {
			{
				.startup = ab8540_codec_pcm_startup,
				.prepare = ab8540_codec_pcm_prepare,
				.hw_params = ab8540_codec_pcm_hw_params,
				.shutdown = ab8540_codec_pcm_shutdown,
				.set_sysclk = ab8540_codec_set_dai_sysclk,
				.set_tdm_slot = ab8540_codec_set_dai_tdm_slot,
				.set_fmt = ab8540_codec_set_dai_fmt,
			}
		},
		.symmetric_rates = 1
	},
	{
		.name = "ab8540-codec-dai.1",
		.id = AB8540_IF2_DAI_ID,
		.capture = {
			.stream_name = "ab8540_0c",
			.channels_min = 1,
			.channels_max = 8,
			.rates = AB8540_SUPPORTED_RATE,
			.formats = AB8540_SUPPORTED_FMT,
		},
		.ops = (struct snd_soc_dai_ops[]) {
			{
				.startup = ab8540_codec_pcm_startup,
				.prepare = ab8540_codec_pcm_prepare,
				.hw_params = ab8540_codec_pcm_hw_params,
				.shutdown = ab8540_codec_pcm_shutdown,
				.set_sysclk = ab8540_codec_set_dai_sysclk,
				.set_tdm_slot = ab8540_codec_set_dai_tdm_slot,
				.set_fmt = ab8540_codec_set_dai_fmt,
			}
		},
		.symmetric_rates = 1
	},
	{
		.name = "ab8540-codec-dai.2",
		.id = AB8540_IF3_DAI_ID,
		.playback = {
			.stream_name = "ab8540_voice_0p",
			.channels_min = 1,
			.channels_max = 1,
			.rates = AB8540_VOICE_SUPPORTED_RATE,
			.formats = AB8540_VOICE_SUPPORTED_FMT,
		},
		.capture = {
			.stream_name = "ab8540_voice_0c",
			.channels_min = 1,
			.channels_max = 4,
			.rates = AB8540_VOICE_SUPPORTED_RATE,
			.formats = AB8540_VOICE_SUPPORTED_FMT,
		},
		.ops = (struct snd_soc_dai_ops[]) {
			{
				.startup = ab8540_codec_pcm_startup,
				.prepare = ab8540_codec_pcm_prepare,
				.hw_params = ab8540_codec_pcm_hw_params,
				.shutdown = ab8540_codec_pcm_shutdown,
				.set_sysclk = ab8540_codec_voice_set_dai_sysclk,
				.set_tdm_slot =
					ab8540_codec_voice_set_dai_tdm_slot,
				.set_fmt = ab8540_codec_voice_set_dai_fmt,
			}
		},
		.symmetric_rates = 1
	},
};

static int ab8540_codec_probe(struct snd_soc_codec *codec)
{
	int i, ret;
	u8 *cache = codec->reg_cache;

	pr_debug("%s: Enter.\n", __func__);

	ab8540_codec_configure_audio_macrocell(codec);

	for (i = AB8540_LAST_REG; i >= AB8540_FIRST_REG; i--)
		ab8540_codec_write_reg_audio(codec, i, cache[i]);

	/* Add controls */
	ret = snd_ctl_add(codec->card->snd_card,
			snd_ctl_new1(&chipid_control, codec));
	if (ret < 0) {
		pr_err("%s: failed to add soc controls (%d).\n",
				__func__, ret);
		return ret;
	}
	ret = snd_soc_add_codec_controls(codec, ab8540_snd_controls,
			ARRAY_SIZE(ab8540_snd_controls));
	if (ret < 0) {
		pr_err("%s: failed to add soc controls (%d).\n",
				__func__, ret);
		return ret;
	}

	/* Add DAPM-widgets */
	ret = ab8540_codec_add_widgets(codec);
	if (ret < 0) {
		pr_err("%s: Failed add widgets (%d).\n", __func__, ret);
		return ret;
	}

	ab8540_codec = codec;

	return ret;
}

static int ab8540_codec_remove(struct snd_soc_codec *codec)
{
	snd_soc_dapm_free(&codec->dapm);
	ab8540_codec = NULL;

	return 0;
}

static int ab8540_codec_suspend(struct snd_soc_codec *codec)
{
	pr_debug("%s Enter.\n", __func__);

	return 0;
}

static int ab8540_codec_resume(struct snd_soc_codec *codec)
{
	pr_debug("%s Enter.\n", __func__);

	return 0;
}

struct snd_soc_codec_driver ab8540_codec_driver = {
	.probe =		ab8540_codec_probe,
	.remove =		ab8540_codec_remove,
	.suspend =		ab8540_codec_suspend,
	.resume =		ab8540_codec_resume,
	.read =			ab8540_codec_read_reg_audio,
	.write =		ab8540_codec_write_reg_audio,
	.reg_cache_size =	ARRAY_SIZE(ab8540_reg_cache),
	.reg_word_size =	sizeof(u8),
	.reg_cache_default =	ab8540_reg_cache,
};

static int __devinit ab8540_codec_driver_probe(struct platform_device *pdev)
{
	int err;

	pr_debug("%s: Enter.\n", __func__);

	pr_info("%s: Register codec.\n", __func__);
	err = snd_soc_register_codec(&pdev->dev,
				&ab8540_codec_driver,
				ab8540_codec_dai,
				ARRAY_SIZE(ab8540_codec_dai));

	if (err < 0) {
		pr_err("%s: Error: Failed to register codec (%d).\n",
			__func__, err);
	}

	return err;
}

static int __devexit ab8540_codec_driver_remove(struct platform_device *pdev)
{
	pr_info("%s Enter.\n", __func__);

	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static int ab8540_codec_driver_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	pr_debug("%s Enter.\n", __func__);

	return 0;
}

static int ab8540_codec_driver_resume(struct platform_device *pdev)
{
	pr_debug("%s Enter.\n", __func__);

	return 0;
}

static struct platform_driver ab8540_codec_platform_driver = {
	.driver	= {
		.name	= "ab8540-codec",
		.owner	= THIS_MODULE,
	},
	.probe		= ab8540_codec_driver_probe,
	.remove		= __devexit_p(ab8540_codec_driver_remove),
	.suspend	= ab8540_codec_driver_suspend,
	.resume		= ab8540_codec_driver_resume,
};

static int __devinit ab8540_codec_platform_driver_init(void)
{
	int ret;

	pr_info("%s: Enter.\n", __func__);

	ret = platform_driver_register(&ab8540_codec_platform_driver);
	if (ret != 0) {
		pr_err("%s: Failed to register AB8540 platform driver (%d)!\n",
			__func__, ret);
	}

	return ret;
}

static void __exit ab8540_codec_platform_driver_exit(void)
{
	pr_info("%s: Enter.\n", __func__);

	platform_driver_unregister(&ab8540_codec_platform_driver);
}

module_init(ab8540_codec_platform_driver_init);
module_exit(ab8540_codec_platform_driver_exit);

MODULE_DESCRIPTION("AB8540 Codec driver");
MODULE_ALIAS("platform:ab8540-codec");
MODULE_AUTHOR("ST-Ericsson");
MODULE_LICENSE("GPL v2");
