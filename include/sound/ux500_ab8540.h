/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * Author: Jarmo K. Kuronen <jarmo.kuronen@symbio.com>
 *         for ST-Ericsson.
 *
 * License terms:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#ifndef UX500_AB8540_H
#define UX500_AB8540_H

extern struct snd_soc_ops ux500_ab8540_ops[];
extern struct snd_soc_ops ux500_ab8540_voice_ops[];

struct snd_soc_pcm_runtime;

int ux500_ab8540_startup(struct snd_pcm_substream *substream);

void ux500_ab8540_shutdown(struct snd_pcm_substream *substream);

int ux500_ab8540_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params);

int ux500_ab8540_soc_machine_drv_init(void);

void ux500_ab8540_soc_machine_drv_cleanup(void);

int ux500_ab8540_machine_codec_init(struct snd_soc_pcm_runtime *runtime);

extern void ux500_ab8540_jack_report(int);

#endif
