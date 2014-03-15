/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * ST-Ericsson MCDE base driver
 *
 * Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

/*
 * =====================================
 * Frame dump + vsync mode functionality
 * =====================================
 *
 * All functions accessible through debugfs/mcde.
 *
 * - MCDE global state dump
 * - Channel state dump
 * - Overlay state dump
 * - Overlay frame dump
 * - Vsync trigger method
 * - Vsync mode
 * - Overlay pixel format dump trigger
 *
 * Within each channel and overlay, as well as within <debugfs>/mcde,
 * there are files named "get_dump" that the user may write to when a
 * dump is required. Example:
 *
 * $ cd <debugfs>/mcde/chnl<id>
 * $ echo <mode> > get_dump
 *
 * Where <mode> is a (bitwise-OR) combination of the following values:
 *
 * 1 - perform dump of channel/overlay structures only
 * 2 - perform dump of overlay frame(s) only
 *
 * New per-overlay files are:
 * - frame_snapshot: frame dump (binary)
 * - frame_stats: human-readable information regarding the frame dump
 * - state_snapshot: human-readable dump of the overlay structure
 * - format_filter: pixel format filter for frame dump triggering
 * - available_format_filters: list of human-readable overlay pixel
 * formats that can be used as a trigger
 *
 * New per-channel files are:
 * - state_snapshot: human-readable dump of the channel structure
 * - trig_mode: vsync trigger method
 * - vsync_mode: vsync mode
 * - available_trig_modes: list of human-readable vsync trigger methods
 * - available_vsync_modes: list of human-readable vsync modes
 *
 * Example 1: take a state and frame dump of channel 0, and all overlays
 * within channel 0.
 *
 * $ cd <debugfs>/mcde/chnl0
 * $ echo 3 > get_dump
 *
 * Example 2: take a state dump of channel 1, and all overlays within
 * channel 1.
 *
 * $ cd <debugfs>/mcde/chnl1
 * $ echo 1 > get_dump
 *
 * Example 3: take a frame dump of channel 0, overlay 1.
 *
 * $ cd <debugfs>/mcde/chnl0/ovly0
 * $ echo 2 > get_dump
 *
 * Example 4: take a state and frame dump of all channels and overlays:
 *
 * $ cd <debugfs>/mcde
 * $ echo 3 > state_get_dump
 *
 * Format filters are also available, to trigger a dump when the next
 * overlay that matches any item in a list of pixel formats arrives in
 * MCDE. To list available pixel formats, use the command (per-overlay):
 *
 * $ cd <debugfs>/mcde/chnl0/ovly0
 * $ cat available_format_filters
 *
 * To add pixel format(s) to the trigger list:
 *
 * $ echo RGBA4444 > format_filter
 * $ echo RGB565 > format_filter
 *
 * To remove a filter from the trigger list, prefix with '-':
 *
 * $ echo -RGBA4444 > format_filter
 *
 * All format filters are case-insensitive, and must match one of the
 * filters listed in 'available_filter_formats'.
 *
 * After a dump is triggered, the format filter list will be cleared.
 *
 * Changing the method of vsync trigger and sync mode is also available,
 * per-channel. To view the current trigger method (currently HW or SW):
 *
 * $ cd <debugfs>/mcde/chnl0
 * $ cat trig_mode
 *
 * To view the current vsync mode:
 *
 * $ cat vsync_mode
 *
 * Changes to the current trigger method/mode are performed in a similar
 * manner as adding format filters, e.g.
 *
 * $ echo TE_POLLING > trig_mode
 * $ echo HW > vsync_mode
 *
 * It is also possible to set these parameters to "auto", which instructs MCDE
 * to handle settings itself. Otherwise, these parameters will override
 * whatever changes MCDE attempts to make.
 *
 * Whenever the value is "auto", the setting that MCDE is currently using
 * will be displayed in brackets, e.g.
 *
 * $ cat trig_mode
 * AUTO (SW)
 * $ cat vsync_mode
 * AUTO (BTA)
 *
 */

/*
 * ==================
 * LUT2 functionality
 * ==================
 *
 * Functions accessible through debugfs/mcde:
 *
 * - LUT2 segment modification
 * - LUT2 enable/disable
 *
 * C interface (mcde.h):
 *
 * int mcde_chnl_set_oled_palette(
 * struct mcde_chnl_state *chnl,
 * struct mcde_oled_palette_table *palette);
 *
 * Usage (debugfs):
 *
 * $ cd <debugfs>/mcde/chnl<id>
 * $ echo <RGB index> <lut entry> <segment min> <segment max> > lut2
 * $ echo <flag> > lut2_enable
 *
 * Where <RGB index> is in the range [0, 2], and corresponds to which
 * colour channel of LUT2 is to be modified.
 *
 * <lut entry> is in the range [0, 31], and corresponds to which of the
 * 32 segments of LUT2 is to be modified.
 *
 * <segment min> and <segment max> correspond to the min/max values
 * for the particular segment, which are 12-bit values in the range
 * [0, 4095].
 *
 * <flag> corresponds to a boolean that is mapped to false for zero,
 * and true for non-zero.
 *
 * Example 1:
 *
 * To set a linear mapping for the blue channel, in which the output
 * has 1/2 the magnitude of the input, use the following:
 *
 * $ echo 2 0 0 64 > lut2
 * $ echo 2 1 64 128 > lut2
 * $ echo 2 2 128 192 > lut2
 * $ echo 2 3 192 256 > lut2
 * ...
 * $ echo 2 29 1856 1920 > lut2
 * $ echo 2 30 1920 1984 > lut2
 * $ echo 2 31 1984 2048 > lut2
 * $ echo 1 > lut2_enable
 *
 * Example 2:
 *
 * To set a flat mapping for the green channel, in which the output
 * is mapped to 16-colours (truncation), use the following
 *
 * $ echo 1 0 0 0 > lut2
 * $ echo 1 1 0 0 > lut2
 * $ echo 1 2 256 256 > lut2
 * $ echo 1 3 256 256 > lut2
 * ...
 * $ echo 1 29 3584 3584 > lut2
 * $ echo 1 30 3840 3840 > lut2
 * $ echo 1 31 3840 3840 > lut2
 * $ echo 1 > lut2_enable
 *
 * To display the current LUT2 entries for a channel, use the command:
 *
 * $ cat lut2
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/stat.h>
#include <linux/time.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <asm/page.h>
#include <linux/io.h>

#include <video/mcde.h>
#include <video/nova_dsilink.h>
#include "mcde_regs.h"

#include "mcde_struct.h"
#include "mcde_hw.h"
#include "mcde_debugfs.h"

#define DUMP_STATE_BIT		(0x01)
#define DUMP_FRAME_BIT		(0x02)
#define DUMP_MASK		(DUMP_STATE_BIT | DUMP_FRAME_BIT)

#define MAX_NUM_OVERLAYS 2
#define MAX_NUM_CHANNELS 4
#define DEFAULT_DMESG_FPS_LOG_INTERVAL 100

enum mcde_debugfs_sync_src {
	MCDE_DEBUGFS_SYNCSRC_AUTO = 0, /* Automatic */
	MCDE_DEBUGFS_SYNCSRC_OFF,
	MCDE_DEBUGFS_SYNCSRC_TE0,
	MCDE_DEBUGFS_SYNCSRC_TE1,
	MCDE_DEBUGFS_SYNCSRC_BTA,
	MCDE_DEBUGFS_SYNCSRC_TE_POLLING,
};

enum mcde_debugfs_trig_method {
	MCDE_DEBUGFS_TRIG_AUTO = 0, /* Automatic */
	MCDE_DEBUGFS_TRIG_HW,
	MCDE_DEBUGFS_TRIG_SW,
};

struct fps_info {
	u32 enable_dmesg;
	u32 interval_ms;
	struct timespec timestamp_last;
	u32 frame_counter_last;
	u32 frame_counter;
	u32 fpks;
};

struct overlay_frame {
	u32 width;
	u32 height;
	enum mcde_ovly_pix_fmt pix_fmt;
	u32 frame_size_bytes;
	u8 *frame_data;
	struct debugfs_blob_wrapper *frame_blob;
};

struct overlay_info {
	u8 id;
	struct dentry *dentry;
	struct fps_info fps;
	struct mcde_ovly_state *ovly;
	struct mcde_ovly_state ovly_snapshot;
	struct overlay_frame frame;
	u8 dump_flags;
	u32 format_filter;
	/*
	 * bitmask: 0x01 = dump overlay struct,
	 * 0x02 = dump overlay frame
	 */
};

struct channel_info {
	u8 id;
	struct dentry *dentry;
	struct mcde_chnl_state *chnl;
	struct fps_info fps;
	struct overlay_info overlays[MAX_NUM_OVERLAYS];
	struct mcde_chnl_state channel_snapshot;
	u8 dump_flags;
	/*
	 * bitmask: 0x01 = dump overlay/channel structs,
	 * 0x02 = dump overlay frame
	 * (propogates to all overlays on channel)
	 */
	enum mcde_debugfs_sync_src sync_mode;
	enum mcde_debugfs_trig_method trig_mode;
	u32 lut2_enable;
	u32 lut2[32 * 3]; /* (2 x packed 12-bit entries) x 32 x 3 channels*/
};

static struct mcde_info {
	struct device *dev;
	struct dentry *dentry;
	struct channel_info channels[MAX_NUM_CHANNELS];
	u8 dump_flags;
	/*
	 * bitmask: 0x01 = dump overlay/channel structs,
	 * 0x02 = dump overlay frames
	 * 0x04 = dump MCDE registers (currently unimplemented)
	 * (propogates to all channels & overlays)
	 */
} mcde;

struct mcde_format_filter {
	const char *name;
	enum mcde_ovly_pix_fmt fmt;
};

#define ENTRY_EXPAND_P1(fmt)	{#fmt, MCDE_OVLYPIXFMT_ ## fmt}
static const struct mcde_format_filter mcde_format_filter_list[] = {
	ENTRY_EXPAND_P1(RGB565),
	ENTRY_EXPAND_P1(RGBA5551),
	ENTRY_EXPAND_P1(RGBA4444),
	ENTRY_EXPAND_P1(YCbCr422),
	ENTRY_EXPAND_P1(RGB888),
	ENTRY_EXPAND_P1(RGBX8888),
	ENTRY_EXPAND_P1(RGBA8888),
};
#undef ENTRY_EXPAND_P1

struct mcde_sync_map {
	const char *name;
	enum mcde_debugfs_sync_src debugfs_sync;
	enum mcde_sync_src mcde_sync;
};

#define ENTRY_EXPAND_P1A(src)	{#src, MCDE_DEBUGFS_SYNCSRC_ ## src, -1}
#define ENTRY_EXPAND_P1(src)	{#src, MCDE_DEBUGFS_SYNCSRC_ ## src,\
	MCDE_SYNCSRC_ ## src}
static const struct mcde_sync_map mcde_sync_map_list[] = {
	ENTRY_EXPAND_P1A(AUTO),
	ENTRY_EXPAND_P1(OFF),
	ENTRY_EXPAND_P1(TE0),
	ENTRY_EXPAND_P1(TE1),
	ENTRY_EXPAND_P1(BTA),
	ENTRY_EXPAND_P1(TE_POLLING),
};
#undef ENTRY_EXPAND_P1
#undef ENTRY_EXPAND_P1A

struct mcde_trig_map {
	const char *name;
	enum mcde_debugfs_trig_method debugfs_trig;
	enum mcde_trig_method mcde_trig;
};

#define ENTRY_EXPAND_P1A(src)	{#src, MCDE_DEBUGFS_TRIG_ ## src, -1}
#define ENTRY_EXPAND_P1(src)	{#src, MCDE_DEBUGFS_TRIG_ ## src,\
	MCDE_TRIG_ ## src}
static const struct mcde_trig_map mcde_trig_map_list[] = {
	ENTRY_EXPAND_P1A(AUTO),
	ENTRY_EXPAND_P1(HW),
	ENTRY_EXPAND_P1(SW),
};
#undef ENTRY_EXPAND_P1
#undef ENTRY_EXPAND_P1A

static const s8 mcde_debugfs_get_format_index_from_string(char *fmt)
{
	unsigned int i;

	for (i = 0; i < sizeof(mcde_format_filter_list) /
		sizeof(struct mcde_format_filter); i++)
		if (!strcasecmp(fmt, mcde_format_filter_list[i].name))
			return i;

	return -EINVAL;
}

static const s8 mcde_debugfs_get_format_index_from_enum(
	enum mcde_ovly_pix_fmt fmt)
{
	unsigned int i;

	for (i = 0; i < sizeof(mcde_format_filter_list) /
		sizeof(struct mcde_format_filter); i++)
		if (fmt == mcde_format_filter_list[i].fmt)
			return i;

	return -EINVAL;
}
static const s8 mcde_debugfs_get_syncsrc_index_from_string(char *syncsrc)
{
	unsigned int i;

	for (i = 0; i < sizeof(mcde_sync_map_list) /
		sizeof(struct mcde_sync_map); i++)
		if (!strcasecmp(syncsrc, mcde_sync_map_list[i].name))
			return i;

	return -EINVAL;
}

static enum mcde_sync_src mcde_debugfs_debugfs2mcde_syncsrc_enum(
	enum mcde_debugfs_sync_src syncsrc)
{
	unsigned int i;

	for (i = 0; i < sizeof(mcde_sync_map_list) /
		sizeof(struct mcde_sync_map); i++)
		if (syncsrc == mcde_sync_map_list[i].debugfs_sync)
			return mcde_sync_map_list[i].mcde_sync;

	/* should never happen */
	BUG_ON(true);
	return mcde_sync_map_list[0].mcde_sync;
}

static const s8 mcde_debugfs_get_trigmethod_index_from_string(char *trig)
{
	unsigned int i;

	for (i = 0; i < sizeof(mcde_trig_map_list) /
		sizeof(struct mcde_trig_map); i++)
		if (!strcasecmp(trig, mcde_trig_map_list[i].name))
			return i;

	return -EINVAL;
}

static enum mcde_trig_method mcde_debugfs_debugfs2mcde_trigmethod_enum(
	enum mcde_debugfs_trig_method trig)
{
	unsigned int i;

	for (i = 0; i < sizeof(mcde_trig_map_list) /
		sizeof(struct mcde_trig_map); i++)
		if (trig == mcde_trig_map_list[i].debugfs_trig)
			return mcde_trig_map_list[i].mcde_trig;

	/* should never happen */
	BUG_ON(true);
	return mcde_trig_map_list[0].mcde_trig;
}

static bool mcde_debugfs_is_filter_enabled(
	u32 filter_bitfield,
	u32 filter_index)
{
	return filter_bitfield & (1 << filter_index) ? true : false;
}

static const char *debugfs_mcde_dsipacking_to_string(u8 dsipacking)
{
	switch (dsipacking) {
	case MCDE_DSIVID0CONF0_PACKING_RGB565:
		return "MCDE_DSIVID0CONF0_PACKING_RGB565";
	case MCDE_DSIVID0CONF0_PACKING_RGB666:
		return "MCDE_DSIVID0CONF0_PACKING_RGB666";
	case MCDE_DSIVID0CONF0_PACKING_RGB888:
		return "MCDE_DSIVID0CONF0_PACKING_RGB888";
	case MCDE_DSIVID0CONF0_PACKING_BGR888:
		return "MCDE_DSIVID0CONF0_PACKING_BGR888";
	case MCDE_DSIVID0CONF0_PACKING_HDTV:
		return "MCDE_DSIVID0CONF0_PACKING_HDTV";
	default:
		return "<UNKNOWN DSIPACKING>";
	}
}

static const char *debugfs_mcde_rotdir_to_string(u8 rotdir)
{
	switch (rotdir) {
	case MCDE_ROTACONF_ROTDIR_CCW:
		return "MCDE_ROTACONF_ROTDIR_CCW";
	case MCDE_ROTACONF_ROTDIR_CW:
		return "MCDE_ROTACONF_ROTDIR_CW";
	default:
		return "<UNKNOWN ROTDIR>";
	}
}

static const char *debugfs_mcde_fifo_id_to_string(enum mcde_fifo fifo)
{
	switch (fifo) {
	case MCDE_FIFO_A:
		return "MCDE_FIFO_A";
	case MCDE_FIFO_B:
		return "MCDE_FIFO_B";
	case MCDE_FIFO_C0:
		return "MCDE_FIFO_C0";
	case MCDE_FIFO_C1:
		return "MCDE_FIFO_C1";
	default:
		return "<UNKNOWN FIFO>";
	}
}

static const char *debugfs_mcde_channel_id_to_string(enum mcde_chnl chnl)
{
	switch (chnl) {
	case MCDE_CHNL_A:
		return "MCDE_CHNL_A";
	case MCDE_CHNL_B:
		return "MCDE_CHNL_B";
	case MCDE_CHNL_C0:
		return "MCDE_CHNL_C0";
	case MCDE_CHNL_C1:
		return "MCDE_CHNL_C1";
	default:
		return "<UNKNOWN CHANNEL>";
	}
}

static u32 debugfs_mcde_fmt_to_bytes_per_pixel(enum mcde_ovly_pix_fmt pix_fmt)
{
	switch (pix_fmt) {
	case MCDE_OVLYPIXFMT_RGB565:
	case MCDE_OVLYPIXFMT_RGBA5551:
	case MCDE_OVLYPIXFMT_RGBA4444:
	case MCDE_OVLYPIXFMT_YCbCr422:
		return 2;
	case MCDE_OVLYPIXFMT_RGB888:
		return 3;
	case MCDE_OVLYPIXFMT_RGBX8888:
	case MCDE_OVLYPIXFMT_RGBA8888:
		return 4;
	default:
		return 0;
	}
}

static const char *debugfs_mcde_pix_fmt_to_string(
		enum mcde_ovly_pix_fmt pix_fmt)
{
	switch (pix_fmt) {
	case MCDE_OVLYPIXFMT_RGB565:
		return "RGB565";
	case MCDE_OVLYPIXFMT_RGBA5551:
		return "RGBA5551";
	case MCDE_OVLYPIXFMT_RGBA4444:
		return "RGBA4444";
	case MCDE_OVLYPIXFMT_RGB888:
		return "RGB888";
	case MCDE_OVLYPIXFMT_RGBX8888:
		return "RGBX8888";
	case MCDE_OVLYPIXFMT_RGBA8888:
		return "RGBA8888";
	case MCDE_OVLYPIXFMT_YCbCr422:
		return "YCbCr422";
	default:
		return "<UNKNOWN PIX FORMAT>";
	}
}

static const char *debugfs_mcde_port_type_to_string(
		enum mcde_port_type type)
{
	switch (type) {
	case MCDE_PORTTYPE_DSI:
		return "DSI";
	case MCDE_PORTTYPE_DPI:
		return "DPI";
	default:
		return "<UNKNOWN PORT TYPE>";
	}
}

static const char *debugfs_mcde_port_mode_to_string(
		enum mcde_port_mode mode)
{
	switch (mode) {
	case MCDE_PORTMODE_CMD:
		return "CMD";
	case MCDE_PORTMODE_VID:
		return "VID";
	default:
		return "<UNKNOWN PORT TYPE>";
	}
}

static const char *debugfs_mcde_port_pix_fmt_to_string(
		enum mcde_port_pix_fmt pix_fmt)
{
	switch (pix_fmt) {
	case MCDE_PORTPIXFMT_DPI_16BPP_C1:
		return "DPI_16BPP_C1";
	case MCDE_PORTPIXFMT_DPI_16BPP_C2:
		return "DPI_16BPP_C2";
	case MCDE_PORTPIXFMT_DPI_16BPP_C3:
		return "DPI_16BPP_C3";
	case MCDE_PORTPIXFMT_DPI_18BPP_C1:
		return "DPI_18BPP_C1";
	case MCDE_PORTPIXFMT_DPI_18BPP_C2:
		return "DPI_18BPP_C2";
	case MCDE_PORTPIXFMT_DPI_24BPP:
		return "DPI_24BPP";
	case MCDE_PORTPIXFMT_DSI_16BPP:
		return "DSI_16BPP";
	case MCDE_PORTPIXFMT_DSI_18BPP:
		return "DSI_18BPP";
	case MCDE_PORTPIXFMT_DSI_18BPP_PACKED:
		return "DSI_18BPP_PACKED";
	case MCDE_PORTPIXFMT_DSI_24BPP:
		return "DSI_24BPP";
	case MCDE_PORTPIXFMT_DSI_YCBCR422:
		return "DSI_YCBCR422";
	default:
		return "<UNKNOWN PIX FORMAT>";
	}
}

static const char *debugfs_mcde_sync_src_to_string(
		enum mcde_sync_src mode)
{
	switch (mode) {
	case MCDE_SYNCSRC_OFF:
		return "OFF";
	case MCDE_SYNCSRC_TE0:
		return "TE0";
	case MCDE_SYNCSRC_TE1:
		return "TE1";
	case MCDE_SYNCSRC_BTA:
		return "BTA";
	case MCDE_SYNCSRC_TE_POLLING:
		return "TE_POLLING";
	default:
		return "<UNKNOWN SYNC MODE>";
	}
}
static const char *debugfs_mcde_trig_method_to_string(
		enum mcde_trig_method method)
{
	switch (method) {
	case MCDE_TRIG_HW:
		return "HW";
	case MCDE_TRIG_SW:
		return "SW";
	default:
		return "<UNKNOWN TRIG METHOD>";
	}
}
static const char *debugfs_mcde_vsync_polarity_to_string(
		enum mcde_vsync_polarity polarity)
{
	switch (polarity) {
	case VSYNC_ACTIVE_HIGH:
		return "HIGH";
	case VSYNC_ACTIVE_LOW:
		return "LOW";
	default:
		return "<UNKNOWN VSYNC POLARITY>";
	}
}
static const char *debugfs_mcde_hdmi_sdtv_switch_to_string(
		enum mcde_hdmi_sdtv_switch mode)
{
	switch (mode) {
	case HDMI_SWITCH:
		return "HDMI";
	case SDTV_SWITCH:
		return "SDTV";
	case DVI_SWITCH:
		return "DVI";
	default:
		return "<UNKNOWN HDMI SWITCH>";
	}
}

/*
 * Similar behaviour to vsnprintf.
 *
 * parameters:
 * buf - base address of string
 * cur_size - current offset into string buffer
 * max_size - size of string buffer from the base address
 *				(e.g. "char string[16]" -> max_size = 16)
 * format - format string
 *
 * IFF buf == 0, then the return value is the number of characters
 * (excluding the trailing '\0') that would have been written were
 * the buffer large enough.
 *
 * Otherwise, the return value is the number of characters written to the
 * buffer, with the following caveats:
 *     output truncated: number of characters written INCLUDING trailing '\0'
 *     otherwise: number of characters written EXCLUDING trailing '\0'
 *
 */
static size_t mcde_snprintf(
	char *buf,
	size_t cur_size,
	size_t max_size,
	char *format,
	...)
{
	va_list args;
	va_start(args, format);

	if (buf) {
		size_t ret;

		buf += cur_size;

		if (max_size < cur_size)
			max_size = 0;
		else
			max_size -= cur_size;

		ret = vsnprintf(buf, max_size, format, args);

		return ret > max_size ? max_size : ret;
	} else {
		return vsnprintf(0, 0, format, args);
	}
}

static size_t debugfs_mcde_snprint_oled_regs(
	char *buf,
	size_t dev_size,
	size_t size,
	const char *var_name,
	struct oled_regs *regs)
{
	size_t base_offset = dev_size;

	dev_size += mcde_snprintf(buf, dev_size, size, "%s.dirty: %u\n",
			var_name, regs->dirty);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.alfa_red: %u\n",
			var_name, regs->alfa_red);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.alfa_green: %u\n",
			var_name, regs->alfa_green);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.alfa_blue: %u\n",
			var_name, regs->alfa_blue);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.beta_red: %u\n",
			var_name, regs->beta_red);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.beta_green: %u\n",
			var_name, regs->beta_green);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.beta_blue: %u\n",
			var_name, regs->beta_blue);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.gamma_red: %u\n",
			var_name, regs->gamma_red);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.gamma_green: %u\n",
			var_name, regs->gamma_green);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.gamma_blue: %u\n",
			var_name, regs->gamma_blue);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.off_red: %u\n",
			var_name, regs->off_red);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.off_green: %u\n",
			var_name, regs->off_green);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.off_blue: %u\n",
			var_name, regs->off_blue);

	return dev_size - base_offset;
}

static size_t debugfs_mcde_snprint_tv_regs(
	char *buf,
	size_t dev_size,
	size_t size,
	const char *var_name,
	struct tv_regs *regs)
{
	size_t base_offset = dev_size;

	dev_size += mcde_snprintf(buf, dev_size, size, "%s.dirty: %u\n",
			var_name, regs->dirty);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.dho: %u\n",
			var_name, regs->dho);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.alw: %u\n",
			var_name, regs->alw);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.hsw: %u\n",
			var_name, regs->hsw);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.dvo: %u\n",
			var_name, regs->dvo);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.bsl: %u\n",
			var_name, regs->bsl);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.bel1: %u\n",
			var_name, regs->bel1);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.fsl1: %u\n",
			var_name, regs->fsl1);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.bel2: %u\n",
			var_name, regs->bel2);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.fsl2: %u\n",
			var_name, regs->fsl2);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.tv_mode: %u\n",
			var_name, regs->tv_mode);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.sel_mode_tv: %u\n",
			var_name, regs->sel_mode_tv);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.inv_clk: %u\n",
			var_name, regs->inv_clk);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.interlaced_en: %u\n",
			var_name, regs->interlaced_en);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.lcdtim1: %u\n",
			var_name, regs->lcdtim1);

	return dev_size - base_offset;
}

static size_t debugfs_mcde_snprint_col_regs(
	char *buf,
	size_t dev_size,
	size_t size,
	const char *var_name,
	struct col_regs *regs)
{
	size_t base_offset = dev_size;

	dev_size += mcde_snprintf(buf, dev_size, size, "%s.dirty: %u\n",
			var_name, regs->dirty);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.y_red: %u\n",
			var_name, regs->y_red);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.y_green: %u\n",
			var_name, regs->y_green);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.y_blue: %u\n",
			var_name, regs->y_blue);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.cb_red: %u\n",
			var_name, regs->cb_red);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.cb_green: %u\n",
			var_name, regs->cb_green);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.cb_blue: %u\n",
			var_name, regs->cb_blue);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.cr_red: %u\n",
			var_name, regs->cr_red);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.cr_green: %u\n",
			var_name, regs->cr_green);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.cr_blue: %u\n",
			var_name, regs->cr_blue);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.off_y: %u\n",
			var_name, regs->off_y);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.off_cb: %u\n",
			var_name, regs->off_cb);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.off_cr: %u\n",
			var_name, regs->off_cr);

	return dev_size - base_offset;
}

static size_t debugfs_mcde_snprint_chnl_regs(
	char *buf,
	size_t dev_size,
	size_t size,
	const char *var_name,
	struct chnl_regs *regs)
{
	size_t base_offset = dev_size;

	dev_size += mcde_snprintf(buf, dev_size, size, "%s.dirty: %u\n",
			var_name, regs->dirty);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.floen: %u\n",
			var_name, regs->floen);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.x: %u\n",
			var_name, regs->x);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.y: %u\n",
			var_name, regs->y);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.ppl: %u\n",
			var_name, regs->ppl);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.lpf: %u\n",
			var_name, regs->lpf);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.bpp: %u\n",
			var_name, regs->bpp);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.internal_clk: %u\n",
			var_name, regs->internal_clk);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.pcd: %u\n",
			var_name, regs->pcd);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.clksel: %u\n",
			var_name, regs->clksel);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.cdwin: %u\n",
			var_name, regs->cdwin);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.map_r: 0x%08X\n",
			var_name, (u32)regs->map_r);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.map_g: 0x%08X\n",
			var_name, (u32)regs->map_g);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.map_b: 0x%08X\n",
			var_name, (u32)regs->map_b);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.oled_map_r: 0x%08X\n",
			var_name, (u32)regs->oled_map_r);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.oled_map_g: 0x%08X\n",
			var_name, (u32)regs->oled_map_g);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.oled_map_b: 0x%08X\n",
			var_name, (u32)regs->oled_map_b);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.oled_palette_context: 0x%08X\n",
			var_name, (u32)regs->oled_palette_context);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.palette_enable: %u\n",
			var_name, regs->palette_enable);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.oled_palette_enable: %u\n",
			var_name, regs->oled_palette_enable);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.oled_enable: %u\n",
			var_name, regs->oled_enable);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.background_yuv: %u\n",
			var_name, regs->background_yuv);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.bcd: %u\n",
			var_name, regs->bcd);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.roten: %u\n",
			var_name, regs->roten);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.rotdir: %u (%s)\n",
			var_name, regs->rotdir,
			debugfs_mcde_rotdir_to_string(regs->rotdir));
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.rotbuf1: 0x%08X\n",
			var_name, regs->rotbuf1);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.rotbuf2: 0x%08X\n",
			var_name, regs->rotbuf2);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.rotbufsize: %u\n",
			var_name, regs->rotbufsize);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.blend_ctrl: %u\n",
			var_name, regs->blend_ctrl);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.blend_en: %u\n",
			var_name, regs->blend_en);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.alpha_blend: %u\n",
			var_name, regs->alpha_blend);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.dsipacking: %u (%s)\n",
			var_name, regs->dsipacking,
			debugfs_mcde_dsipacking_to_string(regs->dsipacking));

	return dev_size - base_offset;
}

static size_t debugfs_mcde_snprint_col_transform(
	char *buf,
	size_t dev_size,
	size_t size,
	const char *var_name,
	struct mcde_col_transform *transform)
{
	size_t base_offset = dev_size;

	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.matrix[0]: [%6i] [%6i] [%6i]\n",
			var_name,
			(s16)transform->matrix[0][0],
			(s16)transform->matrix[0][1],
			(s16)transform->matrix[0][2]);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.matrix[1]: [%6i] [%6i] [%6i]\n",
			var_name,
			(s16)transform->matrix[1][0],
			(s16)transform->matrix[1][1],
			(s16)transform->matrix[1][2]);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.matrix[2]: [%6i] [%6i] [%6i]\n",
			var_name,
			(s16)transform->matrix[2][0],
			(s16)transform->matrix[2][1],
			(s16)transform->matrix[2][2]);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.offset:    [%6i] [%6i] [%6i]\n",
			var_name,
			(s16)transform->offset[0],
			(s16)transform->offset[1],
			(s16)transform->offset[2]);

	return dev_size - base_offset;
}

static size_t debugfs_mcde_snprint_oled_transform(
	char *buf,
	size_t dev_size,
	size_t size,
	const char *var_name,
	struct mcde_oled_transform *transform)
{
	size_t base_offset = dev_size;

	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.matrix[0]: [%6i] [%6i] [%6i]\n",
			var_name,
			(s16)transform->matrix[0][0],
			(s16)transform->matrix[0][1],
			(s16)transform->matrix[0][2]);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.matrix[1]: [%6i] [%6i] [%6i]\n",
			var_name,
			(s16)transform->matrix[1][0],
			(s16)transform->matrix[1][1],
			(s16)transform->matrix[1][2]);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.matrix[2]: [%6i] [%6i] [%6i]\n",
			var_name,
			(s16)transform->matrix[2][0],
			(s16)transform->matrix[2][1],
			(s16)transform->matrix[2][2]);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.offset:    [%6i] [%6i] [%6i]\n",
			var_name,
			(s16)transform->offset[0],
			(s16)transform->offset[1],
			(s16)transform->offset[2]);

	return dev_size - base_offset;
}

static size_t debugfs_mcde_snprint_port(
	char *buf,
	size_t dev_size,
	size_t size,
	const char *var_name,
	struct mcde_port *port)
{
	size_t base_offset = dev_size;

	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.type: %s", var_name,
		debugfs_mcde_port_type_to_string(port->type));
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.mode: %s", var_name,
		debugfs_mcde_port_mode_to_string(port->mode));
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.pixel_format: %s",
		var_name,
		debugfs_mcde_port_pix_fmt_to_string(port->pixel_format));
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.ifc: %u", var_name,
		port->ifc);
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.link: %u", var_name,
		port->link);
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.sync_src: %s",
		var_name,
		debugfs_mcde_sync_src_to_string(port->sync_src));
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.frame_trig: %s",
		var_name,
		debugfs_mcde_trig_method_to_string(port->frame_trig));
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.requested_sync_src: %s",
		var_name,
		debugfs_mcde_sync_src_to_string(port->requested_sync_src));
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.requested_frame_trig: %s",
		var_name,
		debugfs_mcde_trig_method_to_string(port->requested_frame_trig));
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.vsync_polarity: %s",
		var_name,
		debugfs_mcde_vsync_polarity_to_string(port->vsync_polarity));
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.vsync_clock_div: %u",
		var_name,
		port->vsync_clock_div);
	/* duration is expressed as number of (STBCLK / VSPDIV) clock period */
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.vsync_min_duration: %u",
		var_name,
		port->vsync_min_duration);
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.vsync_max_duration: %u",
		var_name,
		port->vsync_max_duration);
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.update_auto_trig: %s",
		var_name,
		port->update_auto_trig == true ? "true" : false);
	dev_size += mcde_snprintf(buf, dev_size, size,
		"%s.hdmi_sdtv_switch: %s",
		var_name,
		debugfs_mcde_hdmi_sdtv_switch_to_string(
			port->hdmi_sdtv_switch));

	return dev_size - base_offset;
}

static size_t debugfs_mcde_snprint_blob(
	char *buf,
	size_t dev_size,
	size_t size,
	const char *var_name,
	size_t blob_size,
	const void *blob)
{
	u8 *blob_p = (u8 *)blob;
	size_t indent_size = 0;
	size_t i;
	size_t base_offset = dev_size;

	indent_size = mcde_snprintf(buf, dev_size, size, "%s: ", var_name);
	dev_size += indent_size;

	/*
	 * prints data in format:
	 * var_name: XXXXXXXX XXXXXXXX
	 *           XXXXXXXX XXXXXXXX
	 *           XXXXXXXX XXXXXXXX
	 */
	for (i = 0; i < blob_size; i++) {
		if (i != 0) {
			if (!(i & 7)) {
				size_t j;

				dev_size += mcde_snprintf(buf,
						dev_size, size, "\n");

				for (j = 0; j < indent_size; j++)
					dev_size += mcde_snprintf(buf,
							dev_size, size, " ");
			} else if (!(i & 3)) {
				dev_size += mcde_snprintf(buf,
						dev_size, size, " ");
			}
		}
		dev_size += mcde_snprintf(buf, dev_size, size,
				"%02X", blob_p[i]);
	}

	dev_size += mcde_snprintf(buf, dev_size, size, "\n");

	return dev_size - base_offset;
}

static size_t debugfs_mcde_print_channel_struct(
	char *buf,
	size_t size,
	struct mcde_chnl_state *state)
{
	size_t dev_size = 0;

	dev_size += mcde_snprintf(buf, dev_size, size, "enabled: %i\n",
			state->enabled);
	dev_size += mcde_snprintf(buf, dev_size, size, "reserved: %i\n",
			state->reserved);
	dev_size += mcde_snprintf(buf, dev_size, size, "id: %i (%s)\n",
			state->id,
			debugfs_mcde_channel_id_to_string(state->id));
	dev_size += mcde_snprintf(buf, dev_size, size, "fifo: %i (%s)\n",
			state->fifo,
			debugfs_mcde_fifo_id_to_string(state->fifo));

	dev_size += debugfs_mcde_snprint_port(buf, dev_size, size,
			"port",
			&state->port);

	dev_size += mcde_snprintf(buf, dev_size, size, "ovly0: 0x%08X\n",
			(u32)state->ovly0);
	dev_size += mcde_snprintf(buf, dev_size, size, "ovly1: 0x%08X\n",
			(u32)state->ovly1);
	dev_size += mcde_snprintf(buf, dev_size, size, "state: %i (%s)\n",
			state->state,
			mcde_channel_state_to_string(state->state));
	dev_size += debugfs_mcde_snprint_blob(buf, dev_size, size,
			"state_waitq",
			sizeof(state->state_waitq), &state->state_waitq);
	dev_size += debugfs_mcde_snprint_blob(buf, dev_size, size, "vcmp_waitq",
			sizeof(state->vcmp_waitq), &state->vcmp_waitq);
	dev_size += debugfs_mcde_snprint_blob(buf, dev_size, size, "vcmp_cnt",
			sizeof(state->vcmp_cnt), &state->vcmp_cnt);
	dev_size += mcde_snprintf(buf, dev_size, size, "vcmp_cnt_wait: %i\n",
			state->vcmp_cnt_wait);
	dev_size += mcde_snprintf(buf, dev_size, size, "vsync_cnt_wait: %i\n",
			state->vsync_cnt_wait);

	dev_size += debugfs_mcde_snprint_blob(buf, dev_size, size,
			"n_vsync_capture_listeners",
			sizeof(state->n_vsync_capture_listeners),
			&state->n_vsync_capture_listeners);

	dev_size += mcde_snprintf(buf, dev_size, size,
			"is_bta_te_listening: %i\n",
			state->is_bta_te_listening);

	dev_size += mcde_snprintf(buf, dev_size, size,
			"oled_color_conversion: %i\n",
			state->oled_color_conversion);

	dev_size += mcde_snprintf(buf, dev_size, size, "clk_dpi: 0x%08X\n",
			(u32)state->clk_dpi);
	dev_size += mcde_snprintf(buf, dev_size, size, "dsilink: 0x%08X\n",
			(u32)state->dsilink);

	dev_size += mcde_snprintf(buf, dev_size, size, "power_mode: %i\n",
			state->power_mode);

	dev_size += mcde_snprintf(buf, dev_size, size, "map_r: 0x%08X\n",
			(u32)state->map_r);
	dev_size += mcde_snprintf(buf, dev_size, size, "map_g: 0x%08X\n",
			(u32)state->map_g);
	dev_size += mcde_snprintf(buf, dev_size, size, "map_b: 0x%08X\n",
			(u32)state->map_b);
	dev_size += mcde_snprintf(buf, dev_size, size, "oled_map_r: 0x%08X\n",
			(u32)state->oled_map_r);
	dev_size += mcde_snprintf(buf, dev_size, size, "oled_map_g: 0x%08X\n",
			(u32)state->oled_map_g);
	dev_size += mcde_snprintf(buf, dev_size, size, "oled_map_b: 0x%08X\n",
			(u32)state->oled_map_b);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"oled_palette_context: 0x%08X\n",
			(u32)state->oled_palette_context);
	dev_size += mcde_snprintf(buf, dev_size, size, "palette_enable: %i\n",
			state->palette_enable);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"oled_palette_enable: %i\n",
			state->oled_palette_enable);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"update_oled_palette: %i\n",
			state->update_oled_palette);

	dev_size += mcde_snprintf(buf, dev_size, size, "vmode.xres: %u\n",
			state->vmode.xres);
	dev_size += mcde_snprintf(buf, dev_size, size, "vmode.yres: %u\n",
			state->vmode.yres);
	dev_size += mcde_snprintf(buf, dev_size, size, "vmode.pixclock: %u\n",
			state->vmode.pixclock);
	dev_size += mcde_snprintf(buf, dev_size, size, "vmode.hbp: %u\n",
			state->vmode.hbp);
	dev_size += mcde_snprintf(buf, dev_size, size, "vmode.hfp: %u\n",
			state->vmode.hfp);
	dev_size += mcde_snprintf(buf, dev_size, size, "vmode.hsw: %u\n",
			state->vmode.hsw);
	dev_size += mcde_snprintf(buf, dev_size, size, "vmode.vbp: %u\n",
			state->vmode.vbp);
	dev_size += mcde_snprintf(buf, dev_size, size, "vmode.vfp: %u\n",
			state->vmode.vfp);
	dev_size += mcde_snprintf(buf, dev_size, size, "vmode.vsw: %u\n",
			state->vmode.vsw);
	dev_size += mcde_snprintf(buf, dev_size, size, "vmode.interlaced: %i\n",
			state->vmode.interlaced);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"vmode.force_update: %i\n",
			state->vmode.force_update);

	dev_size += mcde_snprintf(buf, dev_size, size, "hw_rot: %i\n",
			state->hw_rot);
	dev_size += mcde_snprintf(buf, dev_size, size, "rotbuf1: 0x%08X\n",
			state->rotbuf1);
	dev_size += mcde_snprintf(buf, dev_size, size, "rotbuf2: 0x%08X\n",
			state->rotbuf2);
	dev_size += mcde_snprintf(buf, dev_size, size, "rotbufsize: %i\n",
			state->rotbufsize);

	dev_size += debugfs_mcde_snprint_col_transform(buf, dev_size, size,
			"rgb_2_ycbcr", &state->rgb_2_ycbcr);
	dev_size += debugfs_mcde_snprint_col_transform(buf, dev_size, size,
			"ycbcr_2_rgb", &state->ycbcr_2_rgb);

	if (state->transform != 0)
		dev_size += debugfs_mcde_snprint_col_transform(buf, dev_size,
				size,
				"transform", state->transform);
	else
		dev_size += mcde_snprintf(buf, dev_size, size,
				"transform: 0x%08X\n",
				(u32)state->transform);

	if (state->oled_transform != 0)
		dev_size += debugfs_mcde_snprint_oled_transform(buf, dev_size,
				size,
				"oled_transform", state->oled_transform);
	else
		dev_size += mcde_snprintf(buf, dev_size, size,
				"oled_transform: 0x%08X\n",
				(u32)state->oled_transform);

	dev_size += mcde_snprintf(buf, dev_size, size, "blend_ctrl: %i\n",
			state->blend_ctrl);
	dev_size += mcde_snprintf(buf, dev_size, size, "blend_en: %i\n",
			state->blend_en);
	dev_size += mcde_snprintf(buf, dev_size, size, "alpha_blend: %i\n",
			state->alpha_blend);

	dev_size += debugfs_mcde_snprint_chnl_regs(buf, dev_size, size,
			"regs",
			&state->regs);
	dev_size += debugfs_mcde_snprint_col_regs(buf, dev_size, size,
			"col_regs",
			&state->col_regs);
	dev_size += debugfs_mcde_snprint_tv_regs(buf, dev_size, size,
			"tv_regs",
			&state->tv_regs);
	dev_size += debugfs_mcde_snprint_oled_regs(buf, dev_size, size,
			"oled_regs",
			&state->oled_regs);

	dev_size += mcde_snprintf(buf, dev_size, size, "vcmp_per_field: %i\n",
			state->vcmp_per_field);
	dev_size += mcde_snprintf(buf, dev_size, size, "even_vcmp: %i\n",
			state->even_vcmp);

	dev_size += mcde_snprintf(buf, dev_size, size,
			"formatter_updated: %i\n",
			state->formatter_updated);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"esram_is_enabled: %i\n",
			state->esram_is_enabled);

	dev_size += mcde_snprintf(buf, dev_size, size,
			"first_frame_vsync_fix: %i\n",
			state->first_frame_vsync_fix);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"force_disable: %i\n",
			state->force_disable);

	return dev_size;
}

static size_t debugfs_mcde_snprint_ovly_regs(
	char *buf,
	size_t dev_size,
	size_t size,
	const char *var_name,
	struct ovly_regs *regs)
{
	size_t base_offset = dev_size;

	dev_size += mcde_snprintf(buf, dev_size, size, "%s.enabled: %u\n",
			var_name, regs->enabled);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.dirty: %u\n",
			var_name, regs->dirty);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.dirty_buf: %u\n",
			var_name, regs->dirty_buf);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.ch_id: %u\n",
			var_name, regs->ch_id);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.baseaddress0: 0x%08X\n",
			var_name, regs->baseaddress0);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.baseaddress1: 0x%08X\n",
			var_name, regs->baseaddress1);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"%s.bits_per_pixel: %u\n",
			var_name, regs->bits_per_pixel);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.bpp: %u\n",
			var_name, regs->bpp);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.bgr: %u\n",
			var_name, regs->bgr);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.bebo: %u\n",
			var_name, regs->bebo);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.opq: %u\n",
			var_name, regs->opq);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.col_conv: %u\n",
			var_name, regs->col_conv);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.alpha_source: %u\n",
			var_name, regs->alpha_source);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.alpha_value: %u\n",
			var_name, regs->alpha_value);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.pixoff: %u\n",
			var_name, regs->pixoff);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.ppl: %u\n",
			var_name, regs->ppl);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.lpf: %u\n",
			var_name, regs->lpf);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.cropx: %u\n",
			var_name, regs->cropx);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.cropy: %u\n",
			var_name, regs->cropy);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.xpos: %u\n",
			var_name, regs->xpos);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.ypos: %u\n",
			var_name, regs->ypos);
	dev_size += mcde_snprintf(buf, dev_size, size, "%s.z: %u\n",
			var_name, regs->z);

	return dev_size - base_offset;
}

static size_t debugfs_mcde_print_overlay_struct(
	char *buf,
	size_t size,
	struct mcde_ovly_state *state)
{
	size_t dev_size = 0;

	dev_size += mcde_snprintf(buf, dev_size, size, "inuse: %u\n",
			state->inuse);
	dev_size += mcde_snprintf(buf, dev_size, size, "idx: %u\n",
			state->idx);
	dev_size += mcde_snprintf(buf, dev_size, size, "chnl: 0x%08X\n",
			(u32)state->chnl);
	dev_size += mcde_snprintf(buf, dev_size, size, "dirty: %u\n",
			state->dirty);
	dev_size += mcde_snprintf(buf, dev_size, size, "dirty_buf: %u\n",
			state->dirty_buf);

	dev_size += mcde_snprintf(buf, dev_size, size, "paddr: 0x%08X\n",
			state->paddr);
	dev_size += mcde_snprintf(buf, dev_size, size, "stride: %u\n",
			state->stride);
	dev_size += mcde_snprintf(buf, dev_size, size, "pix_fmt: %u (%s)\n",
			state->pix_fmt,
			debugfs_mcde_pix_fmt_to_string(state->pix_fmt));

	dev_size += mcde_snprintf(buf, dev_size, size, "src_x: %u\n",
			state->src_x);
	dev_size += mcde_snprintf(buf, dev_size, size, "src_y: %u\n",
			state->src_y);
	dev_size += mcde_snprintf(buf, dev_size, size, "dst_x: %u\n",
			state->dst_x);
	dev_size += mcde_snprintf(buf, dev_size, size, "dst_y: %u\n",
			state->dst_y);
	dev_size += mcde_snprintf(buf, dev_size, size, "dst_z: %u\n",
			state->dst_z);
	dev_size += mcde_snprintf(buf, dev_size, size, "w: %u\n", state->w);
	dev_size += mcde_snprintf(buf, dev_size, size, "h: %u\n", state->h);

	dev_size += mcde_snprintf(buf, dev_size, size, "alpha_source: %u\n",
			state->alpha_source);
	dev_size += mcde_snprintf(buf, dev_size, size, "alpha_value: %u\n",
			state->alpha_value);

	dev_size += debugfs_mcde_snprint_ovly_regs(buf, dev_size, size, "regs",
			&state->regs);

	return dev_size;
}

static size_t debugfs_mcde_print_mcde_static_state(char *buf, size_t size)
{
	size_t dev_size = 0;

	dev_size += mcde_snprintf(buf, dev_size, size,
			"---[ MCDE global state start ]---\n");
	dev_size += mcde_snprintf(buf, dev_size, size,
			"mcdeio: 0x%08X\n", (u32)mcdeio);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"num_channels: %u\n", num_channels);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"num_overlays: %u\n", num_overlays);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"mcde_irq: %u\n", mcde_irq);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"input_fifo_size: %u\n", input_fifo_size);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"output_fifo_ab_size: %u\n", output_fifo_ab_size);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"output_fifo_c0c1_size: %u\n", output_fifo_c0c1_size);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"mcde_is_enabled: %u\n", mcde_is_enabled);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"dsi_pll_is_enabled: %u\n", dsi_pll_is_enabled);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"dsi_ifc_is_supported: %u\n", dsi_ifc_is_supported);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"dsi_use_clk_framework: %u\n", dsi_use_clk_framework);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"mcde_clk_rate: %u\n", mcde_clk_rate);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"hw_alignment: %u\n", hw_alignment);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"mcde_dynamic_power_management: %u\n",
			mcde_dynamic_power_management);
	dev_size += mcde_snprintf(buf, dev_size, size,
			"---[ MCDE global state end ]---\n");

	return dev_size;
}

static ssize_t debugfs_mcde_dummy_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *f_pos)
{
	size_t dev_size = 0;
	int ret = 0;
	char *local_buf = kmalloc(sizeof(char) * 4096, GFP_KERNEL);

	if (local_buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	dev_size += sprintf(local_buf + dev_size, "Dummy read function\n");

	if (*f_pos > dev_size)
		goto out;

	if (*f_pos + count > dev_size)
		count = dev_size - *f_pos;

	if (copy_to_user(buf, local_buf + *f_pos, count))
		ret = -EINVAL;
	*f_pos += count;
	ret = count;

out:
	if (local_buf != NULL)
		kfree(local_buf);

	return ret;
}

static int debugfs_mcde_dummy_write(struct file *filp, const char __user *buf,
				  size_t count, loff_t *f_pos)
{
	char tmpbuf[64];
	int ret = 0;

	if (count >= sizeof(tmpbuf))
		count = sizeof(tmpbuf) - 1;
	if (copy_from_user(tmpbuf, buf, count))
		return -EINVAL;
	tmpbuf[count] = 0;

	*f_pos += count;
	ret = count;

	return ret;
}

static const struct file_operations debugfs_mcde_dummy_fops = {
	.owner = THIS_MODULE,
	.read  = debugfs_mcde_dummy_read,
	.write = debugfs_mcde_dummy_write,
};

static ssize_t debugfs_mcde_format_filter_read(
	struct file *filp,
	char __user *buf,
	size_t count, loff_t *f_pos)
{
	size_t dev_size = 0;
	int ret = 0;
	char *local_buf = kmalloc(sizeof(char) * 4096, GFP_KERNEL);
	unsigned int i;
	struct overlay_info *state = filp->f_dentry->d_inode->i_private;

	if (local_buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < sizeof(mcde_format_filter_list) /
		sizeof(struct mcde_format_filter); i++)
		if (mcde_debugfs_is_filter_enabled(state->format_filter, i))
			dev_size += sprintf(local_buf + dev_size,
					"%s\n",
					mcde_format_filter_list[i].name);

	if (*f_pos > dev_size)
		goto out;

	if (*f_pos + count > dev_size)
		count = dev_size - *f_pos;

	if (copy_to_user(buf, local_buf + *f_pos, count))
		ret = -EINVAL;
	*f_pos += count;
	ret = count;

out:
	kfree(local_buf);
	return ret;
}

static int debugfs_mcde_format_filter_write(struct file *filp,
	const char __user *buf,
	size_t count, loff_t *f_pos)
{
	char tmpbuf[64];
	char *tmpbuf_p = tmpbuf;
	struct overlay_info *state = filp->f_dentry->d_inode->i_private;
	s8 format_index;
	int remove_entry = 0;

	if (count == 0)
		goto invalid_input;
	if (count >= sizeof(tmpbuf))
		count = sizeof(tmpbuf) - 1;
	if (copy_from_user(tmpbuf, buf, count))
		return -EINVAL;

	tmpbuf[count] = 0;

	tmpbuf_p = strpbrk(tmpbuf, " \r\n");

	if (tmpbuf_p)
		*tmpbuf_p = '\0';

	remove_entry = (tmpbuf[0] == '-') ? 1 : 0;

	format_index = mcde_debugfs_get_format_index_from_string(
		&tmpbuf[remove_entry]);

	if (format_index >= 0) {
		if (remove_entry)
			state->format_filter &= ~(1 << format_index);
		else
			state->format_filter |= 1 << format_index;
	}

invalid_input:

	*f_pos += count;
	return count;
}

static const struct file_operations debugfs_mcde_format_filter_fops = {
	.owner = THIS_MODULE,
	.read  = debugfs_mcde_format_filter_read,
	.write = debugfs_mcde_format_filter_write,
};

static ssize_t debugfs_mcde_available_format_filter_read(struct file *filp,
	char __user *buf,
	size_t count, loff_t *f_pos)
{
	size_t dev_size = 0;
	int ret = 0;
	char *local_buf = kmalloc(sizeof(char) * 4096, GFP_KERNEL);
	unsigned int i;

	if (local_buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < sizeof(mcde_format_filter_list) /
		sizeof(struct mcde_format_filter); i++)
		dev_size += sprintf(local_buf + dev_size, "%s\n",
			mcde_format_filter_list[i].name);

	if (*f_pos > dev_size)
		goto out;

	if (*f_pos + count > dev_size)
		count = dev_size - *f_pos;

	if (copy_to_user(buf, local_buf + *f_pos, count))
		ret = -EINVAL;
	*f_pos += count;
	ret = count;

out:
	kfree(local_buf);
	return ret;
}

static const struct file_operations
	debugfs_mcde_available_format_filter_fops = {
	.owner = THIS_MODULE,
	.read  = debugfs_mcde_available_format_filter_read,
};

static ssize_t debugfs_mcde_channel_snapshot_read(struct file *filp,
	char __user *buf,
	size_t count, loff_t *f_pos)
{
	size_t dev_size = 0;
	int ret = 0;
	size_t req_size = 0;
	char *local_buf;
	struct mcde_chnl_state *state = filp->f_dentry->d_inode->i_private;

	req_size = debugfs_mcde_print_channel_struct(0, 0, state);
	req_size += debugfs_mcde_print_mcde_static_state(0, 0) + 1;

	local_buf = kmalloc(sizeof(char) * req_size, GFP_KERNEL);

	if (local_buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	dev_size = debugfs_mcde_print_channel_struct(local_buf,
			req_size, state);
	dev_size += debugfs_mcde_print_mcde_static_state(local_buf + dev_size,
			req_size - dev_size);

	if (*f_pos > dev_size)
		goto out;

	if (*f_pos + count > dev_size)
		count = dev_size - *f_pos;

	if (copy_to_user(buf, local_buf + *f_pos, count))
		ret = -EINVAL;
	*f_pos += count;
	ret = count;

out:
	kfree(local_buf);
	return ret;
}

static const struct file_operations debugfs_mcde_channel_snapshot_fops = {
	.owner = THIS_MODULE,
	.read  = debugfs_mcde_channel_snapshot_read,
};

static ssize_t debugfs_mcde_overlay_snapshot_read(struct file *filp,
	char __user *buf,
	size_t count, loff_t *f_pos)
{
	size_t dev_size = 0;
	int ret = 0;
	size_t req_size = 0;
	char *local_buf;
	struct mcde_ovly_state *state = filp->f_dentry->d_inode->i_private;

	req_size = debugfs_mcde_print_overlay_struct(0, 0, state) + 1;

	if (!state->regs.enabled)
		req_size += snprintf(0, 0,
			"WARNING: Overlay is disabled (state values may be invalid).\n\n");

	local_buf = kmalloc(sizeof(char) * req_size, GFP_KERNEL);

	if (local_buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	if (!state->regs.enabled)
		dev_size = snprintf(local_buf, req_size,
			"WARNING: Overlay is disabled (state values may be invalid).\n\n");

	dev_size += debugfs_mcde_print_overlay_struct(local_buf + dev_size,
			req_size - dev_size, state);

	if (*f_pos > dev_size)
		goto out;

	if (*f_pos + count > dev_size)
		count = dev_size - *f_pos;

	if (copy_to_user(buf, local_buf + *f_pos, count))
		ret = -EINVAL;
	*f_pos += count;
	ret = count;

out:
	kfree(local_buf);
	return ret;
}

static const struct file_operations debugfs_mcde_overlay_snapshot_fops = {
	.owner = THIS_MODULE,
	.read  = debugfs_mcde_overlay_snapshot_read,
};

static int debugfs_mcde_overlay_dump_write(struct file *filp,
	const char __user *buf,
	size_t count, loff_t *f_pos)
{
	char tmpbuf[64];
	unsigned long bitmask;
	struct overlay_info *state = filp->f_dentry->d_inode->i_private;

	if (count >= sizeof(tmpbuf))
		count = sizeof(tmpbuf) - 1;
	if (copy_from_user(tmpbuf, buf, count))
		return -EINVAL;

	if (sscanf(tmpbuf, "%8lX", (unsigned long *) &bitmask) != 1)
		return -EINVAL;

	state->dump_flags = bitmask & DUMP_MASK;

	*f_pos += count;
	return count;
}

static const struct file_operations debugfs_mcde_overlay_dump_fops = {
	.owner = THIS_MODULE,
	.write = debugfs_mcde_overlay_dump_write,
};

static int debugfs_mcde_channel_write(struct file *filp,
	const char __user *buf,
	size_t count, loff_t *f_pos)
{
	char tmpbuf[64];
	int ret = 0;
	unsigned long bitmask;
	unsigned int i;
	struct channel_info *state = filp->f_dentry->d_inode->i_private;

	if (count >= sizeof(tmpbuf))
		count = sizeof(tmpbuf) - 1;
	if (copy_from_user(tmpbuf, buf, count))
		return -EINVAL;

	if (sscanf(tmpbuf, "%8lX", (unsigned long *) &bitmask) != 1)
		return -EINVAL;

	state->dump_flags = bitmask & DUMP_MASK;

	for (i = 0; i < MAX_NUM_OVERLAYS; i++)
		state->overlays[i].dump_flags = state->dump_flags;

	*f_pos += count;
	ret = count;

	return ret;
}

static const struct file_operations debugfs_mcde_channel_dump_fops = {
	.owner = THIS_MODULE,
	.write = debugfs_mcde_channel_write,
};

static int debugfs_mcde_dump_write(struct file *filp,
	const char __user *buf,
	size_t count, loff_t *f_pos)
{
	char tmpbuf[64];
	unsigned long bitmask;
	unsigned int i, j;
	struct mcde_info *state = filp->f_dentry->d_inode->i_private;

	if (count >= sizeof(tmpbuf))
		count = sizeof(tmpbuf) - 1;
	if (copy_from_user(tmpbuf, buf, count))
		return -EINVAL;

	if (sscanf(tmpbuf, "%8lX", (unsigned long *) &bitmask) != 1)
		return -EINVAL;

	state->dump_flags = bitmask & DUMP_MASK;

	for (i = 0; i < MAX_NUM_CHANNELS; i++) {
		state->channels[i].dump_flags = state->dump_flags;

		for (j = 0; j < MAX_NUM_OVERLAYS; j++)
			state->channels[i].overlays[j].dump_flags =
					state->dump_flags;
	}

	*f_pos += count;
	return count;
}

static const struct file_operations debugfs_mcde_dump_fops = {
	.owner = THIS_MODULE,
	.write = debugfs_mcde_dump_write,
};

static ssize_t debugfs_mcde_frame_stats_read(struct file *filp,
	char __user *buf,
	size_t count, loff_t *f_pos)
{
	size_t dev_size = 0;
	int ret = 0;
	char *local_buf;
	struct overlay_info *state = filp->f_dentry->d_inode->i_private;

	local_buf = kmalloc(sizeof(char) * 4096, GFP_KERNEL);

	if (local_buf == NULL)
		return -ENOMEM;

	if (!state->ovly_snapshot.regs.enabled)
		dev_size += sprintf(local_buf + dev_size,
			"WARNING: Overlay is disabled (state values may be invalid).\n\n");

	dev_size += sprintf(local_buf + dev_size, "Width:       %u\n",
			state->frame.width);
	dev_size += sprintf(local_buf + dev_size, "Height:      %u\n",
			state->frame.height);
	dev_size += sprintf(local_buf + dev_size, "Format:      0x%08X (%s)\n",
			state->frame.pix_fmt,
			debugfs_mcde_pix_fmt_to_string(state->frame.pix_fmt));
	dev_size += sprintf(local_buf + dev_size, "Buffer Size: %u\n",
			state->frame.frame_size_bytes);

	if (*f_pos > dev_size)
		goto out;

	if (*f_pos + count > dev_size)
		count = dev_size - *f_pos;

	if (copy_to_user(buf, local_buf + *f_pos, count))
		ret = -EINVAL;

	*f_pos += count;
	ret = count;

out:
	kfree(local_buf);
	return ret;
}

static const struct file_operations debugfs_mcde_frame_stats_fops = {
	.owner = THIS_MODULE,
	.read  = debugfs_mcde_frame_stats_read,
};

static ssize_t debugfs_mcde_trig_mode_read(
	struct file *filp,
	char __user *buf,
	size_t count, loff_t *f_pos)
{
	size_t dev_size = 0;
	int ret = 0;
	char *local_buf = kmalloc(sizeof(char) * 4096, GFP_KERNEL);
	unsigned int i;
	struct channel_info *state = filp->f_dentry->d_inode->i_private;

	if (local_buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < sizeof(mcde_trig_map_list) /
		sizeof(struct mcde_trig_map); i++)
		if (state->trig_mode == mcde_trig_map_list[i].debugfs_trig) {
			dev_size += sprintf(local_buf + dev_size,
					"%s",
					mcde_trig_map_list[i].name);
			break;
		}

	if ((state->trig_mode == MCDE_DEBUGFS_TRIG_AUTO) && state->chnl) {
		dev_size += sprintf(local_buf + dev_size, " (%s)",
			debugfs_mcde_trig_method_to_string(
				state->chnl->port.frame_trig));

	}

	dev_size += sprintf(local_buf + dev_size, "\n");

	if (*f_pos > dev_size)
		goto out;

	if (*f_pos + count > dev_size)
		count = dev_size - *f_pos;

	if (copy_to_user(buf, local_buf + *f_pos, count))
		ret = -EINVAL;
	*f_pos += count;
	ret = count;

out:
	kfree(local_buf);
	return ret;
}

static int debugfs_mcde_trig_mode_write(struct file *filp,
	const char __user *buf,
	size_t count,
	loff_t *f_pos)
{
	char tmpbuf[64];
	char *tmpbuf_p = tmpbuf;
	struct channel_info *state = filp->f_dentry->d_inode->i_private;
	s8 index;

	if (count == 0)
		goto invalid_input;
	if (count >= sizeof(tmpbuf))
		count = sizeof(tmpbuf) - 1;
	if (copy_from_user(tmpbuf, buf, count))
		return -EINVAL;

	tmpbuf[count] = 0;

	tmpbuf_p = strpbrk(tmpbuf, " \r\n");

	if (tmpbuf_p)
		*tmpbuf_p = '\0';


	index = mcde_debugfs_get_trigmethod_index_from_string(tmpbuf);

	if (index >= 0)
		state->trig_mode = mcde_trig_map_list[index].debugfs_trig;

invalid_input:

	*f_pos += count;
	return count;
}

static const struct file_operations debugfs_mcde_trig_mode_fops = {
	.owner = THIS_MODULE,
	.read  = debugfs_mcde_trig_mode_read,
	.write = debugfs_mcde_trig_mode_write,
};

static ssize_t debugfs_mcde_available_trig_modes_read(
	struct file *filp,
	char __user *buf,
	size_t count,
	loff_t *f_pos)
{
	size_t dev_size = 0;
	int ret = 0;
	char *local_buf = kmalloc(sizeof(char) * 4096, GFP_KERNEL);
	unsigned int i;

	if (local_buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < sizeof(mcde_trig_map_list) /
		sizeof(struct mcde_trig_map); i++)
		dev_size += sprintf(local_buf + dev_size, "%s\n",
			mcde_trig_map_list[i].name);

	if (*f_pos > dev_size)
		goto out;

	if (*f_pos + count > dev_size)
		count = dev_size - *f_pos;

	if (copy_to_user(buf, local_buf + *f_pos, count))
		ret = -EINVAL;
	*f_pos += count;
	ret = count;

out:
	kfree(local_buf);
	return ret;
}

static const struct file_operations debugfs_mcde_available_trig_modes_fops = {
	.owner = THIS_MODULE,
	.read  = debugfs_mcde_available_trig_modes_read,
};

static ssize_t debugfs_mcde_vsync_mode_read(
	struct file *filp,
	char __user *buf,
	size_t count, loff_t *f_pos)
{
	size_t dev_size = 0;
	int ret = 0;
	char *local_buf = kmalloc(sizeof(char) * 4096, GFP_KERNEL);
	unsigned int i;
	struct channel_info *state = filp->f_dentry->d_inode->i_private;

	if (local_buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < sizeof(mcde_sync_map_list) /
		sizeof(struct mcde_sync_map); i++)
		if (state->sync_mode == mcde_sync_map_list[i].debugfs_sync) {
			dev_size += sprintf(local_buf + dev_size,
					"%s",
					mcde_sync_map_list[i].name);
			break;
		}

	if ((state->sync_mode == MCDE_DEBUGFS_SYNCSRC_AUTO) && state->chnl) {
		dev_size += sprintf(local_buf + dev_size, " (%s)",
			debugfs_mcde_sync_src_to_string(
				state->chnl->port.sync_src));

	}

	dev_size += sprintf(local_buf + dev_size, "\n");

	if (*f_pos > dev_size)
		goto out;

	if (*f_pos + count > dev_size)
		count = dev_size - *f_pos;

	if (copy_to_user(buf, local_buf + *f_pos, count))
		ret = -EINVAL;
	*f_pos += count;
	ret = count;

out:
	kfree(local_buf);
	return ret;
}

static int debugfs_mcde_vsync_mode_write(
	struct file *filp,
	const char __user *buf,
	size_t count,
	loff_t *f_pos)
{
	char tmpbuf[64];
	char *tmpbuf_p;
	struct channel_info *state = filp->f_dentry->d_inode->i_private;
	s8 index;

	if (count == 0)
		goto invalid_input;
	if (count >= sizeof(tmpbuf))
		count = sizeof(tmpbuf) - 1;
	if (copy_from_user(tmpbuf, buf, count))
		return -EINVAL;

	tmpbuf[count] = 0;

	tmpbuf_p = strpbrk(tmpbuf, " \r\n");

	if (tmpbuf_p)
		*tmpbuf_p = '\0';

	index = mcde_debugfs_get_syncsrc_index_from_string(tmpbuf);

	if (index >= 0)
		state->sync_mode = mcde_sync_map_list[index].debugfs_sync;

invalid_input:

	*f_pos += count;
	return count;
}

static const struct file_operations debugfs_mcde_vsync_mode_fops = {
	.owner = THIS_MODULE,
	.read  = debugfs_mcde_vsync_mode_read,
	.write = debugfs_mcde_vsync_mode_write,
};

static ssize_t debugfs_mcde_available_vsync_modes_read(
	struct file *filp,
	char __user *buf,
	size_t count,
	loff_t *f_pos)
{
	size_t dev_size = 0;
	int ret = 0;
	char *local_buf = kmalloc(sizeof(char) * 4096, GFP_KERNEL);
	unsigned int i;

	if (local_buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < sizeof(mcde_sync_map_list) /
		sizeof(struct mcde_sync_map); i++)
		dev_size += sprintf(local_buf + dev_size, "%s\n",
			mcde_sync_map_list[i].name);

	if (*f_pos > dev_size)
		goto out;

	if (*f_pos + count > dev_size)
		count = dev_size - *f_pos;

	if (copy_to_user(buf, local_buf + *f_pos, count))
		ret = -EINVAL;
	*f_pos += count;
	ret = count;

out:
	kfree(local_buf);
	return ret;
}

static const struct file_operations debugfs_mcde_available_vsync_modes_fops = {
	.owner = THIS_MODULE,
	.read  = debugfs_mcde_available_vsync_modes_read,
};

/* Requires: lhs > rhs */
static inline u32 timespec_ms_diff(struct timespec lhs, struct timespec rhs)
{
	struct timespec tmp_ts = timespec_sub(lhs, rhs);
	u64 tmp_ns = (u64)timespec_to_ns(&tmp_ts);
	do_div(tmp_ns, NSEC_PER_MSEC);
	return (u32)tmp_ns;
}

/* Returns "frames per 1000 secs", divide by 1000 to get fps with 3 decimals */
static u32 update_fps(struct fps_info *fps)
{
	struct timespec now;
	u32 fpks = 0, ms_since_last, num_frames;

	getrawmonotonic(&now);
	fps->frame_counter++;

	ms_since_last = timespec_ms_diff(now, fps->timestamp_last);
	num_frames = fps->frame_counter - fps->frame_counter_last;
	if (num_frames > 1 && ms_since_last >= fps->interval_ms) {
		fpks = (num_frames * 1000000) / ms_since_last;
		fps->timestamp_last = now;
		fps->frame_counter_last = fps->frame_counter;
		fps->fpks = fpks;
	}

	return fpks;
}

static ssize_t debugfs_mcde_lut2_read(struct file *filp,
	char __user *buf,
	size_t count, loff_t *f_pos)
{
	size_t dev_size = 0;
	int ret = 0;
	unsigned int i, k;
	char *local_buf = kmalloc(sizeof(char) * 4096, GFP_KERNEL);
	struct channel_info *ci = filp->f_dentry->d_inode->i_private;

	if (local_buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	for (k = 0; k < 3; k++) {
		for (i = 0; i < 32; i++)
			dev_size += sprintf(local_buf + dev_size,
					"[%1i, %2i]: %5u, %5u\n",
					k, i,
					(ci->lut2[i + k * 32] >> 12) & 0x0FFF,
					ci->lut2[i + k * 32] & 0x0FFF);

		dev_size += sprintf(local_buf + dev_size, "\n");
	}
	if (*f_pos > dev_size)
		goto out;

	if (*f_pos + count > dev_size)
		count = dev_size - *f_pos;

	if (copy_to_user(buf, local_buf + *f_pos, count))
		ret = -EINVAL;
	*f_pos += count;
	ret = count;

out:
	kfree(local_buf);

	return ret;
}

u16 mcde_debugfs_oled_pal_generic(u8 col_channel,
	u8 dir,
	u8 index,
	void *context)
{
	struct channel_info *ci = context;

	if (dir == 0)
		return (ci->lut2[index + col_channel * 32] >> 12) & 0x0FFF;
	else
		return ci->lut2[index + col_channel * 32] & 0x0FFF;
}

u16 mcde_debugfs_oled_pal_ch0(u8 dir, u8 index, void *context)
{
	return mcde_debugfs_oled_pal_generic(0, dir, index, context);
}
u16 mcde_debugfs_oled_pal_ch1(u8 dir, u8 index, void *context)
{
	return mcde_debugfs_oled_pal_generic(1, dir, index, context);
}
u16 mcde_debugfs_oled_pal_ch2(u8 dir, u8 index, void *context)
{
	return mcde_debugfs_oled_pal_generic(2, dir, index, context);
}

static int debugfs_mcde_lut2_write(struct file *filp,
	const char __user *buf,
	size_t count, loff_t *f_pos)
{
	char tmpbuf[128];
	int ret = 0;
	struct channel_info *ci = filp->f_dentry->d_inode->i_private;
	struct mcde_oled_palette_table oled_palette;
	u32 index, left, right, channel;

	if (count >= sizeof(tmpbuf))
		count = sizeof(tmpbuf) - 1;
	if (copy_from_user(tmpbuf, buf, count))
		return -EINVAL;

	if (sscanf(tmpbuf, "%u %u %u %u", &channel, &index, &left, &right) != 4)
		return -EINVAL;

	if (index >= 32)
		return -EINVAL;
	if (channel >= 3)
		return -EINVAL;

	ci->lut2[index + channel * 32] =
		((left & 0x0FFF) << 12) | (right & 0x0FFF);

	oled_palette.context = ci;
	oled_palette.map_col_ch0 = mcde_debugfs_oled_pal_ch0;
	oled_palette.map_col_ch1 = mcde_debugfs_oled_pal_ch1;
	oled_palette.map_col_ch2 = mcde_debugfs_oled_pal_ch2;

	if (ci->lut2_enable)
		mcde_chnl_set_oled_palette(ci->chnl, &oled_palette);

	*f_pos += count;
	ret = count;

	return ret;
}

static const struct file_operations debugfs_mcde_lut2_fops = {
	.owner = THIS_MODULE,
	.read  = debugfs_mcde_lut2_read,
	.write = debugfs_mcde_lut2_write,
};

static ssize_t debugfs_mcde_lut2_enable_read(struct file *filp,
	char __user *buf,
	size_t count, loff_t *f_pos)
{
	size_t dev_size = 0;
	int ret = 0;
	char *local_buf = kmalloc(sizeof(char) * 64, GFP_KERNEL);
	struct channel_info *ci = filp->f_dentry->d_inode->i_private;

	if (local_buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	dev_size += sprintf(local_buf + dev_size,
				"%i\n",
				ci->lut2_enable);

	if (*f_pos > dev_size)
		goto out;

	if (*f_pos + count > dev_size)
		count = dev_size - *f_pos;

	if (copy_to_user(buf, local_buf + *f_pos, count))
		ret = -EINVAL;
	*f_pos += count;
	ret = count;

out:
	kfree(local_buf);

	return ret;
}

static int debugfs_mcde_lut2_enable_write(struct file *filp,
	const char __user *buf,
	size_t count, loff_t *f_pos)
{
	char tmpbuf[64];
	int ret = 0;
	struct channel_info *ci = filp->f_dentry->d_inode->i_private;
	struct mcde_oled_palette_table oled_palette;
	u32 flag;

	if (count >= sizeof(tmpbuf))
		count = sizeof(tmpbuf) - 1;
	if (copy_from_user(tmpbuf, buf, count))
		return -EINVAL;

	if (sscanf(tmpbuf, "%u", &flag) != 1)
		return -EINVAL;

	ci->lut2_enable = !!flag;

	oled_palette.context = ci;
	oled_palette.map_col_ch0 = mcde_debugfs_oled_pal_ch0;
	oled_palette.map_col_ch1 = mcde_debugfs_oled_pal_ch1;
	oled_palette.map_col_ch2 = mcde_debugfs_oled_pal_ch2;

	if (ci->lut2_enable)
		mcde_chnl_set_oled_palette(ci->chnl, &oled_palette);
	else
		mcde_chnl_set_oled_palette(ci->chnl, NULL);

	*f_pos += count;
	ret = count;

	return ret;
}

static const struct file_operations debugfs_mcde_lut2_enable_fops = {
	.owner = THIS_MODULE,
	.read  = debugfs_mcde_lut2_enable_read,
	.write = debugfs_mcde_lut2_enable_write,
};

static void update_chnl_fps(struct channel_info *ci)
{
	u32 fpks = update_fps(&ci->fps);
	if (fpks && ci->fps.enable_dmesg)
		dev_info(mcde.dev, "FPS: chnl=%d fps=%d.%.3d\n", ci->id,
						fpks / 1000, fpks % 1000);
}

static void update_ovly_fps(struct channel_info *ci, struct overlay_info *oi)
{
	u32 fpks = update_fps(&oi->fps);
	if (fpks && oi->fps.enable_dmesg)
		dev_info(mcde.dev, "FPS: ovly=%d.%d fps=%d.%.3d\n", ci->id,
					oi->id, fpks / 1000, fpks % 1000);
}

static void create_mcde_files(struct dentry *dentry)
{
	debugfs_create_file("get_dump", S_IWUGO, dentry,
			/* context */ &mcde, &debugfs_mcde_dump_fops);
}

static struct channel_info *find_chnl(u8 chnl_id)
{
	if (chnl_id > MAX_NUM_CHANNELS)
		return NULL;
	return &mcde.channels[chnl_id];
}

static struct overlay_info *find_ovly(struct channel_info *ci, u8 ovly_id)
{
	if (!ci || ovly_id >= MAX_NUM_OVERLAYS)
		return NULL;
	return &ci->overlays[ovly_id];
}

static void create_fps_files(struct dentry *dentry, struct fps_info *fps)
{
	debugfs_create_u32("frame_counter", S_IRUGO, dentry,
							&fps->frame_counter);
	debugfs_create_u32("frames_per_ksecs", S_IRUGO, dentry, &fps->fpks);
	debugfs_create_u32("interval_ms", S_IRUGO|S_IWUGO, dentry,
							&fps->interval_ms);
	debugfs_create_u32("dmesg", S_IRUGO|S_IWUGO, dentry,
							&fps->enable_dmesg);
}

static void create_channel_files(
	struct dentry *dentry,
	struct channel_info *ci)
{
	int i;

	debugfs_create_file("state_snapshot", S_IRUGO, dentry,
			/* context */&ci->channel_snapshot,
			&debugfs_mcde_channel_snapshot_fops);
	debugfs_create_file("get_dump", S_IWUGO, dentry,
			/* context */ci, &debugfs_mcde_channel_dump_fops);

	debugfs_create_file("available_trig_modes", S_IRUGO, dentry,
			/* context */0,
			&debugfs_mcde_available_trig_modes_fops);
	debugfs_create_file("available_vsync_modes", S_IRUGO, dentry,
			/* context */0,
			&debugfs_mcde_available_vsync_modes_fops);

	debugfs_create_file("trig_mode", S_IRUGO, dentry,
			/* context */ci,
			&debugfs_mcde_trig_mode_fops);
	debugfs_create_file("vsync_mode", S_IRUGO, dentry,
			/* context */ci,
			&debugfs_mcde_vsync_mode_fops);

	debugfs_create_file("lut2", S_IWUGO | S_IRUGO, dentry,
			/* context */ci,
			&debugfs_mcde_lut2_fops);
	debugfs_create_file("lut2_enable", S_IWUGO | S_IRUGO, dentry,
			/* context */ci,
			&debugfs_mcde_lut2_enable_fops);

	for (i = 0; i < 32; i++) {
		/* default palette: flat colour, 16 steps */
		u16 left = ((i >> 1) << 8) + 128;
		u16 right = ((i >> 1) << 8) + 128;

		ci->lut2[i + 0] = ((left & 0x0FFF) << 12) | (right & 0x0FFF);
		ci->lut2[i + 32] = ((left & 0x0FFF) << 12) | (right & 0x0FFF);
		ci->lut2[i + 64] = ((left & 0x0FFF) << 12) | (right & 0x0FFF);
	}
}

static int create_overlay_files(
	struct dentry *dentry,
	struct overlay_info *oi)
{
	oi->frame.frame_blob = kmalloc(sizeof(struct debugfs_blob_wrapper),
			GFP_KERNEL);

	if (!oi->frame.frame_blob)
		return -ENOMEM;

	oi->frame.frame_blob->data = 0;
	oi->frame.frame_blob->size = 0;
	oi->frame.frame_size_bytes = 0;
	oi->frame.frame_data = 0;

	debugfs_create_file("state_snapshot", S_IRUGO, dentry,
			/* context */&oi->ovly_snapshot,
			&debugfs_mcde_overlay_snapshot_fops);
	debugfs_create_file("get_dump", S_IWUGO, dentry,
			/* context */oi, &debugfs_mcde_overlay_dump_fops);
	debugfs_create_file("frame_stats", S_IRUGO, dentry,
			/* context */oi, &debugfs_mcde_frame_stats_fops);
	debugfs_create_blob("frame_snapshot", S_IRUGO, dentry,
			oi->frame.frame_blob);
	debugfs_create_file("format_filter", S_IWUGO | S_IRUGO, dentry,
			/* context */oi, &debugfs_mcde_format_filter_fops);
	debugfs_create_file("available_format_filters", S_IRUGO, dentry,
			/* context */oi,
			&debugfs_mcde_available_format_filter_fops);

	return 0;
}

static void mcde_debugfs_overlay_frame_dump(struct overlay_info *oi)
{
	unsigned long last_size = oi->frame.frame_blob->size;

	oi->frame.frame_blob->size = 0;
	oi->frame.frame_blob->data = 0;

	oi->frame.width = oi->ovly->w;
	oi->frame.height = oi->ovly->h;
	oi->frame.pix_fmt = oi->ovly->pix_fmt;

	oi->frame.frame_size_bytes = oi->frame.width * oi->frame.height *
			debugfs_mcde_fmt_to_bytes_per_pixel(oi->frame.pix_fmt);

	if (oi->frame.frame_size_bytes > last_size) {
		if (oi->frame.frame_data)
			vfree(oi->frame.frame_data);
		oi->frame.frame_data = vmalloc(oi->frame.frame_size_bytes);
	}

	/*
	 * In compdev, we'll either have a buffer that's derived
	 * from HWMEM, in which case we can do a straight memcpy,
	 * or we'll have a physical address, in which case we
	 * need to do an ioremap. The physical address is
	 * actually always derived from HWMEM, but as the rest of
	 * the HWMEM structure has been discarded, we assume
	 * a generic physical address, just to be on the safe
	 * side.
	 *
	 * Average time to perform either of the methods is ~10ms.
	 */

	if (oi->ovly->kaddr) {
		memcpy(oi->frame.frame_data, oi->ovly->kaddr,
			oi->frame.frame_size_bytes);
	} else {
		u8 *vaddr;
		vaddr = ioremap(oi->ovly->paddr,
			oi->frame.frame_size_bytes);
		memcpy(oi->frame.frame_data, vaddr,
			oi->frame.frame_size_bytes);
		iounmap(vaddr);
	}

	oi->frame.frame_blob->data = oi->frame.frame_data;
	oi->frame.frame_blob->size = oi->frame.frame_size_bytes;
}

int mcde_debugfs_create(struct device *dev)
{
	if (mcde.dev)
		return -EBUSY;

	mcde.dentry = debugfs_create_dir("mcde", NULL);
	if (!mcde.dentry)
		return -ENOMEM;
	mcde.dev = dev;

	create_mcde_files(mcde.dentry);

	return 0;
}

int mcde_debugfs_channel_create(u8 chnl_id, struct mcde_chnl_state *chnl)
{
	struct channel_info *ci = find_chnl(chnl_id);
	char name[10];

	if (!chnl || !ci)
		return -EINVAL;
	if (ci->chnl)
		return -EBUSY;

	snprintf(name, sizeof(name), "chnl%d", chnl_id);
	ci->dentry = debugfs_create_dir(name, mcde.dentry);
	if (!ci->dentry)
		return -ENOMEM;

	create_fps_files(ci->dentry, &ci->fps);
	create_channel_files(ci->dentry, ci);

	ci->fps.interval_ms = DEFAULT_DMESG_FPS_LOG_INTERVAL;
	ci->id = chnl_id;
	ci->chnl = chnl;
	ci->channel_snapshot = *ci->chnl;
	ci->sync_mode = MCDE_DEBUGFS_SYNCSRC_AUTO;
	ci->trig_mode = MCDE_DEBUGFS_TRIG_AUTO;

	return 0;
}

int mcde_debugfs_overlay_create(u8 chnl_id, u8 ovly_id,
	struct mcde_ovly_state *ovly)
{
	struct channel_info *ci = find_chnl(chnl_id);
	struct overlay_info *oi = find_ovly(ci, ovly_id);
	char name[10];

	if (!oi || !ci || ovly_id >= MAX_NUM_OVERLAYS)
		return -EINVAL;
	if (oi->dentry)
		return -EBUSY;

	snprintf(name, sizeof(name), "ovly%d", ovly_id);
	oi->dentry = debugfs_create_dir(name, ci->dentry);
	if (!oi->dentry)
		return -ENOMEM;

	create_fps_files(oi->dentry, &oi->fps);

	if (create_overlay_files(oi->dentry, oi))
		return -ENOMEM;

	oi->fps.interval_ms = DEFAULT_DMESG_FPS_LOG_INTERVAL;
	oi->id = ovly_id;
	oi->ovly = ovly;
	oi->ovly_snapshot = *oi->ovly;
	oi->format_filter = 0;

	return 0;
}

void mcde_debugfs_channel_update(u8 chnl_id)
{
	struct channel_info *ci = find_chnl(chnl_id);

	if (!ci || !ci->chnl)
		return;
	if (ci->dump_flags & DUMP_STATE_BIT) {
		dev_info(mcde.dev, "Dumping channel id %i state.\n", chnl_id);
		ci->dump_flags &= ~DUMP_STATE_BIT;
		ci->channel_snapshot = *ci->chnl;
	}

	update_chnl_fps(ci);

	if ((ci->sync_mode != MCDE_DEBUGFS_SYNCSRC_AUTO) &&
		(ci->chnl->port.sync_src != ci->sync_mode))
		ci->chnl->port.sync_src = ci->sync_mode;

	if ((ci->trig_mode != MCDE_DEBUGFS_TRIG_AUTO) &&
		(ci->chnl->port.frame_trig != ci->trig_mode))
		ci->chnl->port.frame_trig = ci->trig_mode;
}

void mcde_debugfs_overlay_update(u8 chnl_id, u8 ovly_id)
{
	struct channel_info *ci = find_chnl(chnl_id);
	struct overlay_info *oi = find_ovly(ci, ovly_id);

	if (!oi || !oi->dentry)
		return;

	if (oi->ovly) {

		s8 format_index =
			mcde_debugfs_get_format_index_from_enum(
				oi->ovly->pix_fmt);

		if (format_index >= 0)
			if (mcde_debugfs_is_filter_enabled(oi->format_filter,
					format_index)) {
				/* clear format filter */
				oi->format_filter = 0;
				/* enable all bits */
				oi->dump_flags |= DUMP_MASK;
				dev_info(mcde.dev,
					"Overlay id %i dump triggered by format '%s'.\n",
					ovly_id,
					mcde_format_filter_list[format_index].name);
			}
		if (oi->dump_flags & DUMP_STATE_BIT) {
			dev_info(mcde.dev, "Dumping overlay id %i state.\n",
					ovly_id);
			oi->dump_flags &= ~DUMP_STATE_BIT;
			oi->ovly_snapshot = *oi->ovly;
		}
		if (oi->dump_flags & DUMP_FRAME_BIT) {
			dev_info(mcde.dev, "Dumping overlay id %i frame.\n",
					ovly_id);
			oi->dump_flags &= ~DUMP_FRAME_BIT;
			mcde_debugfs_overlay_frame_dump(oi);
		}
	}

	update_ovly_fps(ci, oi);
}

/* callback when MCDE HW is disabled */
void mcde_debugfs_hw_disabled(void)
{
	/* currently unused */
}

/* callback when MCDE HW is enabled */
void mcde_debugfs_hw_enabled(void)
{
	unsigned int i;

	for (i = 0; i < MAX_NUM_CHANNELS; i++) {

		if (mcde.channels[i].sync_mode != MCDE_DEBUGFS_SYNCSRC_AUTO)
			mcde.channels[i].chnl->port.sync_src =
				mcde_debugfs_debugfs2mcde_syncsrc_enum(
					mcde.channels[i].sync_mode);

		if (mcde.channels[i].trig_mode != MCDE_DEBUGFS_TRIG_AUTO)
			mcde.channels[i].chnl->port.frame_trig =
				mcde_debugfs_debugfs2mcde_trigmethod_enum(
					mcde.channels[i].trig_mode);
	}
}

/* external API for enabling/disabling specific format filters */

int mcde_debugfs_enable_format_filter(
	u8 chnl_id,
	u8 ovly_id,
	enum mcde_ovly_pix_fmt fmt)
{
	struct channel_info *ci = find_chnl(chnl_id);
	struct overlay_info *oi = find_ovly(ci, ovly_id);
	s8 format_index;

	if (!oi || !oi->dentry)
		return -EINVAL;

	format_index = mcde_debugfs_get_format_index_from_enum(fmt);

	if (format_index < 0)
		return -EINVAL;

	oi->format_filter |= 1 << format_index;

	return 0;
}
int mcde_debugfs_disable_format_filter(
	u8 chnl_id,
	u8 ovly_id,
	enum mcde_ovly_pix_fmt fmt)
{
	struct channel_info *ci = find_chnl(chnl_id);
	struct overlay_info *oi = find_ovly(ci, ovly_id);
	s8 format_index;

	if (!oi || !oi->dentry)
		return -EINVAL;

	format_index = mcde_debugfs_get_format_index_from_enum(fmt);

	if (format_index < 0)
		return -EINVAL;

	oi->format_filter &= ~(1 << format_index);

	return 0;
}
int mcde_debugfs_disable_all_format_filters(u8 chnl_id, u8 ovly_id)
{
	struct channel_info *ci = find_chnl(chnl_id);
	struct overlay_info *oi = find_ovly(ci, ovly_id);

	if (!oi || !oi->dentry)
		return -EINVAL;

	oi->format_filter = 0;

	return 0;
}
int mcde_debugfs_enable_all_format_filters(u8 chnl_id, u8 ovly_id)
{
	struct channel_info *ci = find_chnl(chnl_id);
	struct overlay_info *oi = find_ovly(ci, ovly_id);

	if (!oi || !oi->dentry)
		return -EINVAL;

	oi->format_filter = 0xFFFFFFFF;

	return 0;
}
