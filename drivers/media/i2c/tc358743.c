// SPDX-License-Identifier: GPL-2.0-only
/*
 * tc358743 - Toshiba HDMI to CSI-2 bridge
 *
 * Copyright 2015 Cisco Systems, Inc. and/or its affiliates. All rights
 * reserved.
 */

/*
 * References (c = chapter, p = page):
 * REF_01 - Toshiba, TC358743XBG (H2C), Functional Specification, Rev 0.60
 * REF_02 - Toshiba, TC358743XBG_HDMI-CSI_Tv11p_nm.xls
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/of_graph.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/hdmi.h>
#include <media/cec.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/i2c/tc358743.h>

#include "tc358743_regs.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

MODULE_DESCRIPTION("Toshiba TC358743 HDMI to CSI-2 bridge driver");
MODULE_AUTHOR("Ramakrishnan Muthukrishnan <ram@rkrishnan.org>");
MODULE_AUTHOR("Mikhail Khelik <mkhelik@cisco.com>");
MODULE_AUTHOR("Mats Randgaard <matrandg@cisco.com>");
MODULE_LICENSE("GPL");

#define EDID_NUM_BLOCKS_MAX 8
#define EDID_BLOCK_SIZE 128

#define I2C_MAX_XFER_SIZE  (EDID_BLOCK_SIZE + 2)

#define POLL_INTERVAL_CEC_MS	10
#define POLL_INTERVAL_MS	1000

static const struct v4l2_dv_timings_cap tc358743_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },
	/* Pixel clock from REF_01 p. 20. Min/max height/width are unknown */
	V4L2_INIT_BT_TIMINGS(640, 1920, 350, 1200, 13000000, 165000000,
			V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT,
			V4L2_DV_BT_CAP_PROGRESSIVE |
			V4L2_DV_BT_CAP_REDUCED_BLANKING |
			V4L2_DV_BT_CAP_CUSTOM)
};

struct tc358743_state {
	struct tc358743_platform_data pdata;
	struct v4l2_mbus_config_mipi_csi2 bus;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler hdl;
	struct i2c_client *i2c_client;
	/* CONFCTL is modified in ops and tc358743_hdmi_sys_int_handler */
	struct mutex confctl_mutex;

	/* controls */
	struct v4l2_ctrl *detect_tx_5v_ctrl;
	struct v4l2_ctrl *audio_sampling_rate_ctrl;
	struct v4l2_ctrl *audio_present_ctrl;

	struct delayed_work delayed_work_enable_hotplug;

	struct timer_list timer;
	struct work_struct work_i2c_poll;

	/* edid  */
	u8 edid_blocks_written;

	struct v4l2_dv_timings timings;
	u32 mbus_fmt_code;
	u8 csi_lanes_in_use;

	struct gpio_desc *reset_gpio;

	struct cec_adapter *cec_adap;
};

static void tc358743_enable_interrupts(struct v4l2_subdev *sd,
		bool cable_connected);
static int tc358743_s_ctrl_detect_tx_5v(struct v4l2_subdev *sd);

static inline struct tc358743_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tc358743_state, sd);
}

/* --------------- I2C --------------- */

static int i2c_rd(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358743_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	u8 buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = n,
			.buf = values,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		v4l2_err(sd, "%s: reading register 0x%x from 0x%x failed: %d\n",
				__func__, reg, client->addr, err);
	}
	return err != ARRAY_SIZE(msgs);
}

static void i2c_wr(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358743_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err, i;
	struct i2c_msg msg;
	u8 data[I2C_MAX_XFER_SIZE];

	if ((2 + n) > I2C_MAX_XFER_SIZE) {
		n = I2C_MAX_XFER_SIZE - 2;
		v4l2_warn(sd, "i2c wr reg=%04x: len=%d is too big!\n",
			  reg, 2 + n);
	}

	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2 + n;
	msg.flags = 0;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;

	for (i = 0; i < n; i++)
		data[2 + i] = values[i];

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		v4l2_err(sd, "%s: writing register 0x%x from 0x%x failed: %d\n",
				__func__, reg, client->addr, err);
		return;
	}

	if (debug < 3)
		return;

	switch (n) {
	case 1:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x",
				reg, data[2]);
		break;
	case 2:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x%02x",
				reg, data[3], data[2]);
		break;
	case 4:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x%02x%02x%02x",
				reg, data[5], data[4], data[3], data[2]);
		break;
	default:
		v4l2_info(sd, "I2C write %d bytes from address 0x%04x\n",
				n, reg);
	}
}

static noinline u32 i2c_rdreg_err(struct v4l2_subdev *sd, u16 reg, u32 n,
				  int *err)
{
	int error;
	__le32 val = 0;

	error = i2c_rd(sd, reg, (u8 __force *)&val, n);
	if (err)
		*err = error;

	return le32_to_cpu(val);
}

static inline u32 i2c_rdreg(struct v4l2_subdev *sd, u16 reg, u32 n)
{
	return i2c_rdreg_err(sd, reg, n, NULL);
}

static noinline void i2c_wrreg(struct v4l2_subdev *sd, u16 reg, u32 val, u32 n)
{
	__le32 raw = cpu_to_le32(val);

	i2c_wr(sd, reg, (u8 __force *)&raw, n);
}

static u8 i2c_rd8(struct v4l2_subdev *sd, u16 reg)
{
	return i2c_rdreg(sd, reg, 1);
}

static void i2c_wr8(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	i2c_wrreg(sd, reg, val, 1);
}

static void i2c_wr8_and_or(struct v4l2_subdev *sd, u16 reg,
		u8 mask, u8 val)
{
	i2c_wrreg(sd, reg, (i2c_rdreg(sd, reg, 1) & mask) | val, 1);
}

static u16 i2c_rd16(struct v4l2_subdev *sd, u16 reg)
{
	return i2c_rdreg(sd, reg, 2);
}

static int i2c_rd16_err(struct v4l2_subdev *sd, u16 reg, u16 *value)
{
	int err;
	*value = i2c_rdreg_err(sd, reg, 2, &err);
	return err;
}

static void i2c_wr16(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	i2c_wrreg(sd, reg, val, 2);
}

static void i2c_wr16_and_or(struct v4l2_subdev *sd, u16 reg, u16 mask, u16 val)
{
	i2c_wrreg(sd, reg, (i2c_rdreg(sd, reg, 2) & mask) | val, 2);
}

static u32 i2c_rd32(struct v4l2_subdev *sd, u16 reg)
{
	return i2c_rdreg(sd, reg, 4);
}

static void i2c_wr32(struct v4l2_subdev *sd, u16 reg, u32 val)
{
	i2c_wrreg(sd, reg, val, 4);
}

/* --------------- STATUS --------------- */

static inline bool is_hdmi(struct v4l2_subdev *sd)
{
	return i2c_rd8(sd, SYS_STATUS) & MASK_S_HDMI;
}

static inline bool tx_5v_power_present(struct v4l2_subdev *sd)
{
	return i2c_rd8(sd, SYS_STATUS) & MASK_S_DDC5V;
}

static inline bool no_signal(struct v4l2_subdev *sd)
{
	return !(i2c_rd8(sd, SYS_STATUS) & MASK_S_TMDS);
}

static inline bool no_sync(struct v4l2_subdev *sd)
{
	return !(i2c_rd8(sd, SYS_STATUS) & MASK_S_SYNC);
}

static inline bool audio_present(struct v4l2_subdev *sd)
{
	return i2c_rd8(sd, AU_STATUS0) & MASK_S_A_SAMPLE;
}

static int get_audio_sampling_rate(struct v4l2_subdev *sd)
{
	static const int code_to_rate[] = {
		44100, 0, 48000, 32000, 22050, 384000, 24000, 352800,
		88200, 768000, 96000, 705600, 176400, 0, 192000, 0
	};

	/* Register FS_SET is not cleared when the cable is disconnected */
	if (no_signal(sd))
		return 0;

	return code_to_rate[i2c_rd8(sd, FS_SET) & MASK_FS];
}

/* --------------- TIMINGS --------------- */

static inline unsigned fps(const struct v4l2_bt_timings *t)
{
	if (!V4L2_DV_BT_FRAME_HEIGHT(t) || !V4L2_DV_BT_FRAME_WIDTH(t))
		return 0;

	return DIV_ROUND_CLOSEST((unsigned)t->pixelclock,
			V4L2_DV_BT_FRAME_HEIGHT(t) * V4L2_DV_BT_FRAME_WIDTH(t));
}

static int tc358743_get_detected_timings(struct v4l2_subdev *sd,
				     struct v4l2_dv_timings *timings)
{
	struct v4l2_bt_timings *bt = &timings->bt;
	unsigned width, height, frame_width, frame_height, frame_interval, fps;

	memset(timings, 0, sizeof(struct v4l2_dv_timings));

	if (no_signal(sd)) {
		v4l2_dbg(1, debug, sd, "%s: no valid signal\n", __func__);
		return -ENOLINK;
	}
	if (no_sync(sd)) {
		v4l2_dbg(1, debug, sd, "%s: no sync on signal\n", __func__);
		return -ENOLCK;
	}

	timings->type = V4L2_DV_BT_656_1120;
	bt->interlaced = i2c_rd8(sd, VI_STATUS1) & MASK_S_V_INTERLACE ?
		V4L2_DV_INTERLACED : V4L2_DV_PROGRESSIVE;

	width = ((i2c_rd8(sd, DE_WIDTH_H_HI) & 0x1f) << 8) +
		i2c_rd8(sd, DE_WIDTH_H_LO);
	height = ((i2c_rd8(sd, DE_WIDTH_V_HI) & 0x1f) << 8) +
		i2c_rd8(sd, DE_WIDTH_V_LO);
	frame_width = ((i2c_rd8(sd, H_SIZE_HI) & 0x1f) << 8) +
		i2c_rd8(sd, H_SIZE_LO);
	frame_height = (((i2c_rd8(sd, V_SIZE_HI) & 0x3f) << 8) +
		i2c_rd8(sd, V_SIZE_LO)) / 2;
	/* frame interval in milliseconds * 10
	 * Require SYS_FREQ0 and SYS_FREQ1 are precisely set */
	frame_interval = ((i2c_rd8(sd, FV_CNT_HI) & 0x3) << 8) +
		i2c_rd8(sd, FV_CNT_LO);
	fps = (frame_interval > 0) ?
		DIV_ROUND_CLOSEST(10000, frame_interval) : 0;

	bt->width = width;
	bt->height = height;
	bt->vsync = frame_height - height;
	bt->hsync = frame_width - width;
	bt->pixelclock = frame_width * frame_height * fps;
	if (bt->interlaced == V4L2_DV_INTERLACED) {
		bt->height *= 2;
		bt->il_vsync = bt->vsync + 1;
		bt->pixelclock /= 2;
	}

	return 0;
}

/* --------------- HOTPLUG / HDCP / EDID --------------- */

static void tc358743_delayed_work_enable_hotplug(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct tc358743_state *state = container_of(dwork,
			struct tc358743_state, delayed_work_enable_hotplug);
	struct v4l2_subdev *sd = &state->sd;

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_OUT0, MASK_HPD_OUT0);
}

static void tc358743_set_hdmi_hdcp(struct v4l2_subdev *sd, bool enable)
{
	v4l2_dbg(2, debug, sd, "%s: %s\n", __func__, enable ?
				"enable" : "disable");

	if (enable) {
		i2c_wr8_and_or(sd, HDCP_REG3, ~KEY_RD_CMD, KEY_RD_CMD);

		i2c_wr8_and_or(sd, HDCP_MODE, ~MASK_MANUAL_AUTHENTICATION, 0);

		i2c_wr8_and_or(sd, HDCP_REG1, 0xff,
				MASK_AUTH_UNAUTH_SEL_16_FRAMES |
				MASK_AUTH_UNAUTH_AUTO);

		i2c_wr8_and_or(sd, HDCP_REG2, ~MASK_AUTO_P3_RESET,
				SET_AUTO_P3_RESET_FRAMES(0x0f));
	} else {
		i2c_wr8_and_or(sd, HDCP_MODE, ~MASK_MANUAL_AUTHENTICATION,
				MASK_MANUAL_AUTHENTICATION);
	}
}

static void tc358743_disable_edid(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	cancel_delayed_work_sync(&state->delayed_work_enable_hotplug);

	/* DDC access to EDID is also disabled when hotplug is disabled. See
	 * register DDC_CTL */
	i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_OUT0, 0x0);
}

static void tc358743_enable_edid(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	if (state->edid_blocks_written == 0) {
		v4l2_dbg(2, debug, sd, "%s: no EDID -> no hotplug\n", __func__);
		tc358743_s_ctrl_detect_tx_5v(sd);
		return;
	}

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	/* Enable hotplug after 100 ms. DDC access to EDID is also enabled when
	 * hotplug is enabled. See register DDC_CTL */
	schedule_delayed_work(&state->delayed_work_enable_hotplug, HZ / 10);

	tc358743_enable_interrupts(sd, true);
	tc358743_s_ctrl_detect_tx_5v(sd);
}

static void tc358743_erase_bksv(struct v4l2_subdev *sd)
{
	int i;

	for (i = 0; i < 5; i++)
		i2c_wr8(sd, BKSV + i, 0);
}

/* --------------- AVI infoframe --------------- */

static void print_avi_infoframe(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	union hdmi_infoframe frame;
	u8 buffer[HDMI_INFOFRAME_SIZE(AVI)];

	if (!is_hdmi(sd)) {
		v4l2_info(sd, "DVI-D signal - AVI infoframe not supported\n");
		return;
	}

	i2c_rd(sd, PK_AVI_0HEAD, buffer, HDMI_INFOFRAME_SIZE(AVI));

	if (hdmi_infoframe_unpack(&frame, buffer, sizeof(buffer)) < 0) {
		v4l2_err(sd, "%s: unpack of AVI infoframe failed\n", __func__);
		return;
	}

	hdmi_infoframe_log(KERN_INFO, dev, &frame);
}

/* --------------- CTRLS --------------- */

static int tc358743_s_ctrl_detect_tx_5v(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	return v4l2_ctrl_s_ctrl(state->detect_tx_5v_ctrl,
			tx_5v_power_present(sd));
}

static int tc358743_s_ctrl_audio_sampling_rate(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	return v4l2_ctrl_s_ctrl(state->audio_sampling_rate_ctrl,
			get_audio_sampling_rate(sd));
}

static int tc358743_s_ctrl_audio_present(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	return v4l2_ctrl_s_ctrl(state->audio_present_ctrl,
			audio_present(sd));
}

static int tc358743_update_controls(struct v4l2_subdev *sd)
{
	int ret = 0;

	ret |= tc358743_s_ctrl_detect_tx_5v(sd);
	ret |= tc358743_s_ctrl_audio_sampling_rate(sd);
	ret |= tc358743_s_ctrl_audio_present(sd);

	return ret;
}

/* --------------- INIT --------------- */

static void tc358743_reset_phy(struct v4l2_subdev *sd)
{
	v4l2_dbg(1, debug, sd, "%s:\n", __func__);

	i2c_wr8_and_or(sd, PHY_RST, ~MASK_RESET_CTRL, 0);
	i2c_wr8_and_or(sd, PHY_RST, ~MASK_RESET_CTRL, MASK_RESET_CTRL);
}

static void tc358743_reset(struct v4l2_subdev *sd, uint16_t mask)
{
	u16 sysctl = i2c_rd16(sd, SYSCTL);

	i2c_wr16(sd, SYSCTL, sysctl | mask);
	i2c_wr16(sd, SYSCTL, sysctl & ~mask);
}

static inline void tc358743_sleep_mode(struct v4l2_subdev *sd, bool enable)
{
	i2c_wr16_and_or(sd, SYSCTL, ~MASK_SLEEP,
			enable ? MASK_SLEEP : 0);
}

static inline void enable_stream(struct v4l2_subdev *sd, bool enable)
{
	struct tc358743_state *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s: %sable\n",
			__func__, enable ? "en" : "dis");

	if (enable) {
		/* It is critical for CSI receiver to see lane transition
		 * LP11->HS. Set to non-continuous mode to enable clock lane
		 * LP11 state. */
		i2c_wr32(sd, TXOPTIONCNTRL, 0);
		/* Set to continuous mode to trigger LP11->HS transition */
		i2c_wr32(sd, TXOPTIONCNTRL, MASK_CONTCLKMODE);
		/* Unmute video */
		i2c_wr8(sd, VI_MUTE, MASK_AUTO_MUTE);
	} else {
		/* Mute video so that all data lanes go to LSP11 state.
		 * No data is output to CSI Tx block. */
		i2c_wr8(sd, VI_MUTE, MASK_AUTO_MUTE | MASK_VI_MUTE);
	}

	mutex_lock(&state->confctl_mutex);
	i2c_wr16_and_or(sd, CONFCTL, ~(MASK_VBUFEN | MASK_ABUFEN),
			enable ? (MASK_VBUFEN | MASK_ABUFEN) : 0x0);
	mutex_unlock(&state->confctl_mutex);
}

static void tc358743_set_pll(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct tc358743_platform_data *pdata = &state->pdata;
	u16 pllctl0 = i2c_rd16(sd, PLLCTL0);
	u16 pllctl1 = i2c_rd16(sd, PLLCTL1);
	u16 pllctl0_new = SET_PLL_PRD(pdata->pll_prd) |
		SET_PLL_FBD(pdata->pll_fbd);
	u32 hsck = (pdata->refclk_hz / pdata->pll_prd) * pdata->pll_fbd;

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	/* Only rewrite when needed (new value or disabled), since rewriting
	 * triggers another format change event. */
	if ((pllctl0 != pllctl0_new) || ((pllctl1 & MASK_PLL_EN) == 0)) {
		u16 pll_frs;

		if (hsck > 500000000)
			pll_frs = 0x0;
		else if (hsck > 250000000)
			pll_frs = 0x1;
		else if (hsck > 125000000)
			pll_frs = 0x2;
		else
			pll_frs = 0x3;

		v4l2_dbg(1, debug, sd, "%s: updating PLL clock\n", __func__);
		tc358743_sleep_mode(sd, true);
		i2c_wr16(sd, PLLCTL0, pllctl0_new);
		i2c_wr16_and_or(sd, PLLCTL1,
				~(MASK_PLL_FRS | MASK_RESETB | MASK_PLL_EN),
				(SET_PLL_FRS(pll_frs) | MASK_RESETB |
				 MASK_PLL_EN));
		udelay(10); /* REF_02, Sheet "Source HDMI" */
		i2c_wr16_and_or(sd, PLLCTL1, ~MASK_CKEN, MASK_CKEN);
		tc358743_sleep_mode(sd, false);
	}
}

static void tc358743_set_ref_clk(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct tc358743_platform_data *pdata = &state->pdata;
	u32 sys_freq;
	u32 lockdet_ref;
	u32 cec_freq;
	u16 fh_min;
	u16 fh_max;

	BUG_ON(!(pdata->refclk_hz == 26000000 ||
		 pdata->refclk_hz == 27000000 ||
		 pdata->refclk_hz == 42000000));

	sys_freq = pdata->refclk_hz / 10000;
	i2c_wr8(sd, SYS_FREQ0, sys_freq & 0x00ff);
	i2c_wr8(sd, SYS_FREQ1, (sys_freq & 0xff00) >> 8);

	i2c_wr8_and_or(sd, PHY_CTL0, ~MASK_PHY_SYSCLK_IND,
			(pdata->refclk_hz == 42000000) ?
			MASK_PHY_SYSCLK_IND : 0x0);

	fh_min = pdata->refclk_hz / 100000;
	i2c_wr8(sd, FH_MIN0, fh_min & 0x00ff);
	i2c_wr8(sd, FH_MIN1, (fh_min & 0xff00) >> 8);

	fh_max = (fh_min * 66) / 10;
	i2c_wr8(sd, FH_MAX0, fh_max & 0x00ff);
	i2c_wr8(sd, FH_MAX1, (fh_max & 0xff00) >> 8);

	lockdet_ref = pdata->refclk_hz / 100;
	i2c_wr8(sd, LOCKDET_REF0, lockdet_ref & 0x0000ff);
	i2c_wr8(sd, LOCKDET_REF1, (lockdet_ref & 0x00ff00) >> 8);
	i2c_wr8(sd, LOCKDET_REF2, (lockdet_ref & 0x0f0000) >> 16);

	i2c_wr8_and_or(sd, NCO_F0_MOD, ~MASK_NCO_F0_MOD,
			(pdata->refclk_hz == 27000000) ?
			MASK_NCO_F0_MOD_27MHZ : 0x0);

	/*
	 * Trial and error suggests that the default register value
	 * of 656 is for a 42 MHz reference clock. Use that to derive
	 * a new value based on the actual reference clock.
	 */
	cec_freq = (656 * sys_freq) / 4200;
	i2c_wr16(sd, CECHCLK, cec_freq);
	i2c_wr16(sd, CECLCLK, cec_freq);
}

static void tc358743_set_csi_color_space(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	switch (state->mbus_fmt_code) {
	case MEDIA_BUS_FMT_UYVY8_1X16:
		v4l2_dbg(2, debug, sd, "%s: YCbCr 422 16-bit\n", __func__);
		i2c_wr8_and_or(sd, VOUT_SET2,
				~(MASK_SEL422 | MASK_VOUT_422FIL_100) & 0xff,
				MASK_SEL422 | MASK_VOUT_422FIL_100);
		i2c_wr8_and_or(sd, VI_REP, ~MASK_VOUT_COLOR_SEL & 0xff,
				MASK_VOUT_COLOR_601_YCBCR_LIMITED);
		mutex_lock(&state->confctl_mutex);
		i2c_wr16_and_or(sd, CONFCTL, ~MASK_YCBCRFMT,
				MASK_YCBCRFMT_422_8_BIT);
		mutex_unlock(&state->confctl_mutex);
		break;
	case MEDIA_BUS_FMT_RGB888_1X24:
		v4l2_dbg(2, debug, sd, "%s: RGB 888 24-bit\n", __func__);
		i2c_wr8_and_or(sd, VOUT_SET2,
				~(MASK_SEL422 | MASK_VOUT_422FIL_100) & 0xff,
				0x00);
		i2c_wr8_and_or(sd, VI_REP, ~MASK_VOUT_COLOR_SEL & 0xff,
				MASK_VOUT_COLOR_RGB_FULL);
		mutex_lock(&state->confctl_mutex);
		i2c_wr16_and_or(sd, CONFCTL, ~MASK_YCBCRFMT, 0);
		mutex_unlock(&state->confctl_mutex);
		break;
	default:
		v4l2_dbg(2, debug, sd, "%s: Unsupported format code 0x%x\n",
				__func__, state->mbus_fmt_code);
	}
}

static unsigned tc358743_num_csi_lanes_needed(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct v4l2_bt_timings *bt = &state->timings.bt;
	struct tc358743_platform_data *pdata = &state->pdata;
	u32 bits_pr_pixel =
		(state->mbus_fmt_code == MEDIA_BUS_FMT_UYVY8_1X16) ?  16 : 24;
	u32 bps = bt->width * bt->height * fps(bt) * bits_pr_pixel;
	u32 bps_pr_lane = (pdata->refclk_hz / pdata->pll_prd) * pdata->pll_fbd;

	return DIV_ROUND_UP(bps, bps_pr_lane);
}

static void tc358743_set_csi(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct tc358743_platform_data *pdata = &state->pdata;
	unsigned lanes = tc358743_num_csi_lanes_needed(sd);

	v4l2_dbg(3, debug, sd, "%s:\n", __func__);

	state->csi_lanes_in_use = lanes;

	tc358743_reset(sd, MASK_CTXRST);

	if (lanes < 1)
		i2c_wr32(sd, CLW_CNTRL, MASK_CLW_LANEDISABLE);
	if (lanes < 1)
		i2c_wr32(sd, D0W_CNTRL, MASK_D0W_LANEDISABLE);
	if (lanes < 2)
		i2c_wr32(sd, D1W_CNTRL, MASK_D1W_LANEDISABLE);
	if (lanes < 3)
		i2c_wr32(sd, D2W_CNTRL, MASK_D2W_LANEDISABLE);
	if (lanes < 4)
		i2c_wr32(sd, D3W_CNTRL, MASK_D3W_LANEDISABLE);

	i2c_wr32(sd, LINEINITCNT, pdata->lineinitcnt);
	i2c_wr32(sd, LPTXTIMECNT, pdata->lptxtimecnt);
	i2c_wr32(sd, TCLK_HEADERCNT, pdata->tclk_headercnt);
	i2c_wr32(sd, TCLK_TRAILCNT, pdata->tclk_trailcnt);
	i2c_wr32(sd, THS_HEADERCNT, pdata->ths_headercnt);
	i2c_wr32(sd, TWAKEUP, pdata->twakeup);
	i2c_wr32(sd, TCLK_POSTCNT, pdata->tclk_postcnt);
	i2c_wr32(sd, THS_TRAILCNT, pdata->ths_trailcnt);
	i2c_wr32(sd, HSTXVREGCNT, pdata->hstxvregcnt);

	i2c_wr32(sd, HSTXVREGEN,
			((lanes > 0) ? MASK_CLM_HSTXVREGEN : 0x0) |
			((lanes > 0) ? MASK_D0M_HSTXVREGEN : 0x0) |
			((lanes > 1) ? MASK_D1M_HSTXVREGEN : 0x0) |
			((lanes > 2) ? MASK_D2M_HSTXVREGEN : 0x0) |
			((lanes > 3) ? MASK_D3M_HSTXVREGEN : 0x0));

	i2c_wr32(sd, TXOPTIONCNTRL, (state->bus.flags &
		 V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK) ? 0 : MASK_CONTCLKMODE);
	i2c_wr32(sd, STARTCNTRL, MASK_START);
	i2c_wr32(sd, CSI_START, MASK_STRT);

	i2c_wr32(sd, CSI_CONFW, MASK_MODE_SET |
			MASK_ADDRESS_CSI_CONTROL |
			MASK_CSI_MODE |
			MASK_TXHSMD |
			((lanes == 4) ? MASK_NOL_4 :
			 (lanes == 3) ? MASK_NOL_3 :
			 (lanes == 2) ? MASK_NOL_2 : MASK_NOL_1));

	i2c_wr32(sd, CSI_CONFW, MASK_MODE_SET |
			MASK_ADDRESS_CSI_ERR_INTENA | MASK_TXBRK | MASK_QUNK |
			MASK_WCER | MASK_INER);

	i2c_wr32(sd, CSI_CONFW, MASK_MODE_CLEAR |
			MASK_ADDRESS_CSI_ERR_HALT | MASK_TXBRK | MASK_QUNK);

	i2c_wr32(sd, CSI_CONFW, MASK_MODE_SET |
			MASK_ADDRESS_CSI_INT_ENA | MASK_INTER);
}

static void tc358743_set_hdmi_phy(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct tc358743_platform_data *pdata = &state->pdata;

	/* Default settings from REF_02, sheet "Source HDMI"
	 * and custom settings as platform data */
	i2c_wr8_and_or(sd, PHY_EN, ~MASK_ENABLE_PHY, 0x0);
	i2c_wr8(sd, PHY_CTL1, SET_PHY_AUTO_RST1_US(1600) |
			SET_FREQ_RANGE_MODE_CYCLES(1));
	i2c_wr8_and_or(sd, PHY_CTL2, ~MASK_PHY_AUTO_RSTn,
			(pdata->hdmi_phy_auto_reset_tmds_detected ?
			 MASK_PHY_AUTO_RST2 : 0) |
			(pdata->hdmi_phy_auto_reset_tmds_in_range ?
			 MASK_PHY_AUTO_RST3 : 0) |
			(pdata->hdmi_phy_auto_reset_tmds_valid ?
			 MASK_PHY_AUTO_RST4 : 0));
	i2c_wr8(sd, PHY_BIAS, 0x40);
	i2c_wr8(sd, PHY_CSQ, SET_CSQ_CNT_LEVEL(0x0a));
	i2c_wr8(sd, AVM_CTL, 45);
	i2c_wr8_and_or(sd, HDMI_DET, ~MASK_HDMI_DET_V,
			pdata->hdmi_detection_delay << 4);
	i2c_wr8_and_or(sd, HV_RST, ~(MASK_H_PI_RST | MASK_V_PI_RST),
			(pdata->hdmi_phy_auto_reset_hsync_out_of_range ?
			 MASK_H_PI_RST : 0) |
			(pdata->hdmi_phy_auto_reset_vsync_out_of_range ?
			 MASK_V_PI_RST : 0));
	i2c_wr8_and_or(sd, PHY_EN, ~MASK_ENABLE_PHY, MASK_ENABLE_PHY);
}

static void tc358743_set_hdmi_audio(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	/* Default settings from REF_02, sheet "Source HDMI" */
	i2c_wr8(sd, FORCE_MUTE, 0x00);
	i2c_wr8(sd, AUTO_CMD0, MASK_AUTO_MUTE7 | MASK_AUTO_MUTE6 |
			MASK_AUTO_MUTE5 | MASK_AUTO_MUTE4 |
			MASK_AUTO_MUTE1 | MASK_AUTO_MUTE0);
	i2c_wr8(sd, AUTO_CMD1, MASK_AUTO_MUTE9);
	i2c_wr8(sd, AUTO_CMD2, MASK_AUTO_PLAY3 | MASK_AUTO_PLAY2);
	i2c_wr8(sd, BUFINIT_START, SET_BUFINIT_START_MS(500));
	i2c_wr8(sd, FS_MUTE, 0x00);
	i2c_wr8(sd, FS_IMODE, MASK_NLPCM_SMODE | MASK_FS_SMODE);
	i2c_wr8(sd, ACR_MODE, MASK_CTS_MODE);
	i2c_wr8(sd, ACR_MDF0, MASK_ACR_L2MDF_1976_PPM | MASK_ACR_L1MDF_976_PPM);
	i2c_wr8(sd, ACR_MDF1, MASK_ACR_L3MDF_3906_PPM);
	i2c_wr8(sd, SDO_MODE1, MASK_SDO_FMT_I2S);
	i2c_wr8(sd, DIV_MODE, SET_DIV_DLY_MS(100));

	mutex_lock(&state->confctl_mutex);
	i2c_wr16_and_or(sd, CONFCTL, 0xffff, MASK_AUDCHNUM_2 |
			MASK_AUDOUTSEL_I2S | MASK_AUTOINDEX);
	mutex_unlock(&state->confctl_mutex);
}

static void tc358743_set_hdmi_info_frame_mode(struct v4l2_subdev *sd)
{
	/* Default settings from REF_02, sheet "Source HDMI" */
	i2c_wr8(sd, PK_INT_MODE, MASK_ISRC2_INT_MODE | MASK_ISRC_INT_MODE |
			MASK_ACP_INT_MODE | MASK_VS_INT_MODE |
			MASK_SPD_INT_MODE | MASK_MS_INT_MODE |
			MASK_AUD_INT_MODE | MASK_AVI_INT_MODE);
	i2c_wr8(sd, NO_PKT_LIMIT, 0x2c);
	i2c_wr8(sd, NO_PKT_CLR, 0x53);
	i2c_wr8(sd, ERR_PK_LIMIT, 0x01);
	i2c_wr8(sd, NO_PKT_LIMIT2, 0x30);
	i2c_wr8(sd, NO_GDB_LIMIT, 0x10);
}

static void tc358743_initial_setup(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct tc358743_platform_data *pdata = &state->pdata;

	/*
	 * IR is not supported by this driver.
	 * CEC is only enabled if needed.
	 */
	i2c_wr16_and_or(sd, SYSCTL, ~(MASK_IRRST | MASK_CECRST),
				     (MASK_IRRST | MASK_CECRST));

	tc358743_reset(sd, MASK_CTXRST | MASK_HDMIRST);
#ifdef CONFIG_VIDEO_TC358743_CEC
	tc358743_reset(sd, MASK_CECRST);
#endif
	tc358743_sleep_mode(sd, false);

	i2c_wr16(sd, FIFOCTL, pdata->fifo_level);

	tc358743_set_ref_clk(sd);

	i2c_wr8_and_or(sd, DDC_CTL, ~MASK_DDC5V_MODE,
			pdata->ddc5v_delay & MASK_DDC5V_MODE);
	i2c_wr8_and_or(sd, EDID_MODE, ~MASK_EDID_MODE, MASK_EDID_MODE_E_DDC);

	tc358743_set_hdmi_phy(sd);
	tc358743_set_hdmi_hdcp(sd, pdata->enable_hdcp);
	tc358743_set_hdmi_audio(sd);
	tc358743_set_hdmi_info_frame_mode(sd);

	/* All CE and IT formats are detected as RGB full range in DVI mode */
	i2c_wr8_and_or(sd, VI_MODE, ~MASK_RGB_DVI, 0);

	i2c_wr8_and_or(sd, VOUT_SET2, ~MASK_VOUTCOLORMODE,
			MASK_VOUTCOLORMODE_AUTO);
	i2c_wr8(sd, VOUT_SET3, MASK_VOUT_EXTCNT);
}

/* --------------- CEC --------------- */

#ifdef CONFIG_VIDEO_TC358743_CEC
static int tc358743_cec_adap_enable(struct cec_adapter *adap, bool enable)
{
	struct tc358743_state *state = adap->priv;
	struct v4l2_subdev *sd = &state->sd;

	i2c_wr32(sd, CECIMSK, enable ? MASK_CECTIM | MASK_CECRIM : 0);
	i2c_wr32(sd, CECICLR, MASK_CECTICLR | MASK_CECRICLR);
	i2c_wr32(sd, CECEN, enable);
	if (enable)
		i2c_wr32(sd, CECREN, MASK_CECREN);
	return 0;
}

static int tc358743_cec_adap_monitor_all_enable(struct cec_adapter *adap,
						bool enable)
{
	struct tc358743_state *state = adap->priv;
	struct v4l2_subdev *sd = &state->sd;
	u32 reg;

	reg = i2c_rd32(sd, CECRCTL1);
	if (enable)
		reg |= MASK_CECOTH;
	else
		reg &= ~MASK_CECOTH;
	i2c_wr32(sd, CECRCTL1, reg);
	return 0;
}

static int tc358743_cec_adap_log_addr(struct cec_adapter *adap, u8 log_addr)
{
	struct tc358743_state *state = adap->priv;
	struct v4l2_subdev *sd = &state->sd;
	unsigned int la = 0;

	if (log_addr != CEC_LOG_ADDR_INVALID) {
		la = i2c_rd32(sd, CECADD);
		la |= 1 << log_addr;
	}
	i2c_wr32(sd, CECADD, la);
	return 0;
}

static int tc358743_cec_adap_transmit(struct cec_adapter *adap, u8 attempts,
				   u32 signal_free_time, struct cec_msg *msg)
{
	struct tc358743_state *state = adap->priv;
	struct v4l2_subdev *sd = &state->sd;
	unsigned int i;

	i2c_wr32(sd, CECTCTL,
		 (cec_msg_is_broadcast(msg) ? MASK_CECBRD : 0) |
		 (signal_free_time - 1));
	for (i = 0; i < msg->len; i++)
		i2c_wr32(sd, CECTBUF1 + i * 4,
			msg->msg[i] | ((i == msg->len - 1) ? MASK_CECTEOM : 0));
	i2c_wr32(sd, CECTEN, MASK_CECTEN);
	return 0;
}

static const struct cec_adap_ops tc358743_cec_adap_ops = {
	.adap_enable = tc358743_cec_adap_enable,
	.adap_log_addr = tc358743_cec_adap_log_addr,
	.adap_transmit = tc358743_cec_adap_transmit,
	.adap_monitor_all_enable = tc358743_cec_adap_monitor_all_enable,
};

static void tc358743_cec_handler(struct v4l2_subdev *sd, u16 intstatus,
				 bool *handled)
{
	struct tc358743_state *state = to_state(sd);
	unsigned int cec_rxint, cec_txint;
	unsigned int clr = 0;

	cec_rxint = i2c_rd32(sd, CECRSTAT);
	cec_txint = i2c_rd32(sd, CECTSTAT);

	if (intstatus & MASK_CEC_RINT)
		clr |= MASK_CECRICLR;
	if (intstatus & MASK_CEC_TINT)
		clr |= MASK_CECTICLR;
	i2c_wr32(sd, CECICLR, clr);

	if ((intstatus & MASK_CEC_TINT) && cec_txint) {
		if (cec_txint & MASK_CECTIEND)
			cec_transmit_attempt_done(state->cec_adap,
						  CEC_TX_STATUS_OK);
		else if (cec_txint & MASK_CECTIAL)
			cec_transmit_attempt_done(state->cec_adap,
						  CEC_TX_STATUS_ARB_LOST);
		else if (cec_txint & MASK_CECTIACK)
			cec_transmit_attempt_done(state->cec_adap,
						  CEC_TX_STATUS_NACK);
		else if (cec_txint & MASK_CECTIUR) {
			/*
			 * Not sure when this bit is set. Treat
			 * it as an error for now.
			 */
			cec_transmit_attempt_done(state->cec_adap,
						  CEC_TX_STATUS_ERROR);
		}
		if (handled)
			*handled = true;
	}
	if ((intstatus & MASK_CEC_RINT) &&
	    (cec_rxint & MASK_CECRIEND)) {
		struct cec_msg msg = {};
		unsigned int i;
		unsigned int v;

		v = i2c_rd32(sd, CECRCTR);
		msg.len = v & 0x1f;
		if (msg.len > CEC_MAX_MSG_SIZE)
			msg.len = CEC_MAX_MSG_SIZE;
		for (i = 0; i < msg.len; i++) {
			v = i2c_rd32(sd, CECRBUF1 + i * 4);
			msg.msg[i] = v & 0xff;
		}
		cec_received_msg(state->cec_adap, &msg);
		if (handled)
			*handled = true;
	}
	i2c_wr16(sd, INTSTATUS,
		 intstatus & (MASK_CEC_RINT | MASK_CEC_TINT));
}

#endif

/* --------------- IRQ --------------- */

static void tc358743_format_change(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct v4l2_dv_timings timings;
	const struct v4l2_event tc358743_ev_fmt = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	if (tc358743_get_detected_timings(sd, &timings)) {
		enable_stream(sd, false);

		v4l2_dbg(1, debug, sd, "%s: No signal\n",
				__func__);
	} else {
		if (!v4l2_match_dv_timings(&state->timings, &timings, 0, false))
			enable_stream(sd, false);

		if (debug)
			v4l2_print_dv_timings(sd->name,
					"tc358743_format_change: New format: ",
					&timings, false);
	}

	if (sd->devnode)
		v4l2_subdev_notify_event(sd, &tc358743_ev_fmt);
}

static void tc358743_init_interrupts(struct v4l2_subdev *sd)
{
	u16 i;

	/* clear interrupt status registers */
	for (i = SYS_INT; i <= KEY_INT; i++)
		i2c_wr8(sd, i, 0xff);

	i2c_wr16(sd, INTSTATUS, 0xffff);
}

static void tc358743_enable_interrupts(struct v4l2_subdev *sd,
		bool cable_connected)
{
	v4l2_dbg(2, debug, sd, "%s: cable connected = %d\n", __func__,
			cable_connected);

	if (cable_connected) {
		i2c_wr8(sd, SYS_INTM, ~(MASK_M_DDC | MASK_M_DVI_DET |
					MASK_M_HDMI_DET) & 0xff);
		i2c_wr8(sd, CLK_INTM, ~MASK_M_IN_DE_CHG);
		i2c_wr8(sd, CBIT_INTM, ~(MASK_M_CBIT_FS | MASK_M_AF_LOCK |
					MASK_M_AF_UNLOCK) & 0xff);
		i2c_wr8(sd, AUDIO_INTM, ~MASK_M_BUFINIT_END);
		i2c_wr8(sd, MISC_INTM, ~MASK_M_SYNC_CHG);
	} else {
		i2c_wr8(sd, SYS_INTM, ~MASK_M_DDC & 0xff);
		i2c_wr8(sd, CLK_INTM, 0xff);
		i2c_wr8(sd, CBIT_INTM, 0xff);
		i2c_wr8(sd, AUDIO_INTM, 0xff);
		i2c_wr8(sd, MISC_INTM, 0xff);
	}
}

static void tc358743_hdmi_audio_int_handler(struct v4l2_subdev *sd,
		bool *handled)
{
	u8 audio_int_mask = i2c_rd8(sd, AUDIO_INTM);
	u8 audio_int = i2c_rd8(sd, AUDIO_INT) & ~audio_int_mask;

	i2c_wr8(sd, AUDIO_INT, audio_int);

	v4l2_dbg(3, debug, sd, "%s: AUDIO_INT = 0x%02x\n", __func__, audio_int);

	tc358743_s_ctrl_audio_sampling_rate(sd);
	tc358743_s_ctrl_audio_present(sd);
}

static void tc358743_csi_err_int_handler(struct v4l2_subdev *sd, bool *handled)
{
	v4l2_err(sd, "%s: CSI_ERR = 0x%x\n", __func__, i2c_rd32(sd, CSI_ERR));

	i2c_wr32(sd, CSI_INT_CLR, MASK_ICRER);
}

static void tc358743_hdmi_misc_int_handler(struct v4l2_subdev *sd,
		bool *handled)
{
	u8 misc_int_mask = i2c_rd8(sd, MISC_INTM);
	u8 misc_int = i2c_rd8(sd, MISC_INT) & ~misc_int_mask;

	i2c_wr8(sd, MISC_INT, misc_int);

	v4l2_dbg(3, debug, sd, "%s: MISC_INT = 0x%02x\n", __func__, misc_int);

	if (misc_int & MASK_I_SYNC_CHG) {
		/* Reset the HDMI PHY to try to trigger proper lock on the
		 * incoming video format. Erase BKSV to prevent that old keys
		 * are used when a new source is connected. */
		if (no_sync(sd) || no_signal(sd)) {
			tc358743_reset_phy(sd);
			tc358743_erase_bksv(sd);
		}

		tc358743_format_change(sd);

		misc_int &= ~MASK_I_SYNC_CHG;
		if (handled)
			*handled = true;
	}

	if (misc_int) {
		v4l2_err(sd, "%s: Unhandled MISC_INT interrupts: 0x%02x\n",
				__func__, misc_int);
	}
}

static void tc358743_hdmi_cbit_int_handler(struct v4l2_subdev *sd,
		bool *handled)
{
	u8 cbit_int_mask = i2c_rd8(sd, CBIT_INTM);
	u8 cbit_int = i2c_rd8(sd, CBIT_INT) & ~cbit_int_mask;

	i2c_wr8(sd, CBIT_INT, cbit_int);

	v4l2_dbg(3, debug, sd, "%s: CBIT_INT = 0x%02x\n", __func__, cbit_int);

	if (cbit_int & MASK_I_CBIT_FS) {

		v4l2_dbg(1, debug, sd, "%s: Audio sample rate changed\n",
				__func__);
		tc358743_s_ctrl_audio_sampling_rate(sd);

		cbit_int &= ~MASK_I_CBIT_FS;
		if (handled)
			*handled = true;
	}

	if (cbit_int & (MASK_I_AF_LOCK | MASK_I_AF_UNLOCK)) {

		v4l2_dbg(1, debug, sd, "%s: Audio present changed\n",
				__func__);
		tc358743_s_ctrl_audio_present(sd);

		cbit_int &= ~(MASK_I_AF_LOCK | MASK_I_AF_UNLOCK);
		if (handled)
			*handled = true;
	}

	if (cbit_int) {
		v4l2_err(sd, "%s: Unhandled CBIT_INT interrupts: 0x%02x\n",
				__func__, cbit_int);
	}
}

static void tc358743_hdmi_clk_int_handler(struct v4l2_subdev *sd, bool *handled)
{
	u8 clk_int_mask = i2c_rd8(sd, CLK_INTM);
	u8 clk_int = i2c_rd8(sd, CLK_INT) & ~clk_int_mask;

	/* Bit 7 and bit 6 are set even when they are masked */
	i2c_wr8(sd, CLK_INT, clk_int | 0x80 | MASK_I_OUT_H_CHG);

	v4l2_dbg(3, debug, sd, "%s: CLK_INT = 0x%02x\n", __func__, clk_int);

	if (clk_int & (MASK_I_IN_DE_CHG)) {

		v4l2_dbg(1, debug, sd, "%s: DE size or position has changed\n",
				__func__);

		/* If the source switch to a new resolution with the same pixel
		 * frequency as the existing (e.g. 1080p25 -> 720p50), the
		 * I_SYNC_CHG interrupt is not always triggered, while the
		 * I_IN_DE_CHG interrupt seems to work fine. Format change
		 * notifications are only sent when the signal is stable to
		 * reduce the number of notifications. */
		if (!no_signal(sd) && !no_sync(sd))
			tc358743_format_change(sd);

		clk_int &= ~(MASK_I_IN_DE_CHG);
		if (handled)
			*handled = true;
	}

	if (clk_int) {
		v4l2_err(sd, "%s: Unhandled CLK_INT interrupts: 0x%02x\n",
				__func__, clk_int);
	}
}

static void tc358743_hdmi_sys_int_handler(struct v4l2_subdev *sd, bool *handled)
{
	struct tc358743_state *state = to_state(sd);
	u8 sys_int_mask = i2c_rd8(sd, SYS_INTM);
	u8 sys_int = i2c_rd8(sd, SYS_INT) & ~sys_int_mask;

	i2c_wr8(sd, SYS_INT, sys_int);

	v4l2_dbg(3, debug, sd, "%s: SYS_INT = 0x%02x\n", __func__, sys_int);

	if (sys_int & MASK_I_DDC) {
		bool tx_5v = tx_5v_power_present(sd);

		v4l2_dbg(1, debug, sd, "%s: Tx 5V power present: %s\n",
				__func__, tx_5v ?  "yes" : "no");

		if (tx_5v) {
			tc358743_enable_edid(sd);
		} else {
			tc358743_enable_interrupts(sd, false);
			tc358743_disable_edid(sd);
			memset(&state->timings, 0, sizeof(state->timings));
			tc358743_erase_bksv(sd);
			tc358743_update_controls(sd);
		}

		sys_int &= ~MASK_I_DDC;
		if (handled)
			*handled = true;
	}

	if (sys_int & MASK_I_DVI) {
		v4l2_dbg(1, debug, sd, "%s: HDMI->DVI change detected\n",
				__func__);

		/* Reset the HDMI PHY to try to trigger proper lock on the
		 * incoming video format. Erase BKSV to prevent that old keys
		 * are used when a new source is connected. */
		if (no_sync(sd) || no_signal(sd)) {
			tc358743_reset_phy(sd);
			tc358743_erase_bksv(sd);
		}

		sys_int &= ~MASK_I_DVI;
		if (handled)
			*handled = true;
	}

	if (sys_int & MASK_I_HDMI) {
		v4l2_dbg(1, debug, sd, "%s: DVI->HDMI change detected\n",
				__func__);

		/* Register is reset in DVI mode (REF_01, c. 6.6.41) */
		i2c_wr8(sd, ANA_CTL, MASK_APPL_PCSX_NORMAL | MASK_ANALOG_ON);

		sys_int &= ~MASK_I_HDMI;
		if (handled)
			*handled = true;
	}

	if (sys_int) {
		v4l2_err(sd, "%s: Unhandled SYS_INT interrupts: 0x%02x\n",
				__func__, sys_int);
	}
}

/* --------------- CORE OPS --------------- */

static int tc358743_log_status(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct v4l2_dv_timings timings;
	uint8_t hdmi_sys_status =  i2c_rd8(sd, SYS_STATUS);
	uint16_t sysctl = i2c_rd16(sd, SYSCTL);
	u8 vi_status3 =  i2c_rd8(sd, VI_STATUS3);
	const int deep_color_mode[4] = { 8, 10, 12, 16 };
	static const char * const input_color_space[] = {
		"RGB", "YCbCr 601", "opRGB", "YCbCr 709", "NA (4)",
		"xvYCC 601", "NA(6)", "xvYCC 709", "NA(8)", "sYCC601",
		"NA(10)", "NA(11)", "NA(12)", "opYCC 601"};

	v4l2_info(sd, "-----Chip status-----\n");
	v4l2_info(sd, "Chip ID: 0x%02x\n",
			(i2c_rd16(sd, CHIPID) & MASK_CHIPID) >> 8);
	v4l2_info(sd, "Chip revision: 0x%02x\n",
			i2c_rd16(sd, CHIPID) & MASK_REVID);
	v4l2_info(sd, "Reset: IR: %d, CEC: %d, CSI TX: %d, HDMI: %d\n",
			!!(sysctl & MASK_IRRST),
			!!(sysctl & MASK_CECRST),
			!!(sysctl & MASK_CTXRST),
			!!(sysctl & MASK_HDMIRST));
	v4l2_info(sd, "Sleep mode: %s\n", sysctl & MASK_SLEEP ? "on" : "off");
	v4l2_info(sd, "Cable detected (+5V power): %s\n",
			hdmi_sys_status & MASK_S_DDC5V ? "yes" : "no");
	v4l2_info(sd, "DDC lines enabled: %s\n",
			(i2c_rd8(sd, EDID_MODE) & MASK_EDID_MODE_E_DDC) ?
			"yes" : "no");
	v4l2_info(sd, "Hotplug enabled: %s\n",
			(i2c_rd8(sd, HPD_CTL) & MASK_HPD_OUT0) ?
			"yes" : "no");
	v4l2_info(sd, "CEC enabled: %s\n",
			(i2c_rd16(sd, CECEN) & MASK_CECEN) ?  "yes" : "no");
	v4l2_info(sd, "-----Signal status-----\n");
	v4l2_info(sd, "TMDS signal detected: %s\n",
			hdmi_sys_status & MASK_S_TMDS ? "yes" : "no");
	v4l2_info(sd, "Stable sync signal: %s\n",
			hdmi_sys_status & MASK_S_SYNC ? "yes" : "no");
	v4l2_info(sd, "PHY PLL locked: %s\n",
			hdmi_sys_status & MASK_S_PHY_PLL ? "yes" : "no");
	v4l2_info(sd, "PHY DE detected: %s\n",
			hdmi_sys_status & MASK_S_PHY_SCDT ? "yes" : "no");

	if (tc358743_get_detected_timings(sd, &timings)) {
		v4l2_info(sd, "No video detected\n");
	} else {
		v4l2_print_dv_timings(sd->name, "Detected format: ", &timings,
				true);
	}
	v4l2_print_dv_timings(sd->name, "Configured format: ", &state->timings,
			true);

	v4l2_info(sd, "-----CSI-TX status-----\n");
	v4l2_info(sd, "Lanes needed: %d\n",
			tc358743_num_csi_lanes_needed(sd));
	v4l2_info(sd, "Lanes in use: %d\n",
			state->csi_lanes_in_use);
	v4l2_info(sd, "Waiting for particular sync signal: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & MASK_S_WSYNC) ?
			"yes" : "no");
	v4l2_info(sd, "Transmit mode: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & MASK_S_TXACT) ?
			"yes" : "no");
	v4l2_info(sd, "Receive mode: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & MASK_S_RXACT) ?
			"yes" : "no");
	v4l2_info(sd, "Stopped: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & MASK_S_HLT) ?
			"yes" : "no");
	v4l2_info(sd, "Color space: %s\n",
			state->mbus_fmt_code == MEDIA_BUS_FMT_UYVY8_1X16 ?
			"YCbCr 422 16-bit" :
			state->mbus_fmt_code == MEDIA_BUS_FMT_RGB888_1X24 ?
			"RGB 888 24-bit" : "Unsupported");

	v4l2_info(sd, "-----%s status-----\n", is_hdmi(sd) ? "HDMI" : "DVI-D");
	v4l2_info(sd, "HDCP encrypted content: %s\n",
			hdmi_sys_status & MASK_S_HDCP ? "yes" : "no");
	v4l2_info(sd, "Input color space: %s %s range\n",
			input_color_space[(vi_status3 & MASK_S_V_COLOR) >> 1],
			(vi_status3 & MASK_LIMITED) ? "limited" : "full");
	if (!is_hdmi(sd))
		return 0;
	v4l2_info(sd, "AV Mute: %s\n", hdmi_sys_status & MASK_S_AVMUTE ? "on" :
			"off");
	v4l2_info(sd, "Deep color mode: %d-bits per channel\n",
			deep_color_mode[(i2c_rd8(sd, VI_STATUS1) &
				MASK_S_DEEPCOLOR) >> 2]);
	print_avi_infoframe(sd);

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static void tc358743_print_register_map(struct v4l2_subdev *sd)
{
	v4l2_info(sd, "0x0000-0x00FF: Global Control Register\n");
	v4l2_info(sd, "0x0100-0x01FF: CSI2-TX PHY Register\n");
	v4l2_info(sd, "0x0200-0x03FF: CSI2-TX PPI Register\n");
	v4l2_info(sd, "0x0400-0x05FF: Reserved\n");
	v4l2_info(sd, "0x0600-0x06FF: CEC Register\n");
	v4l2_info(sd, "0x0700-0x84FF: Reserved\n");
	v4l2_info(sd, "0x8500-0x85FF: HDMIRX System Control Register\n");
	v4l2_info(sd, "0x8600-0x86FF: HDMIRX Audio Control Register\n");
	v4l2_info(sd, "0x8700-0x87FF: HDMIRX InfoFrame packet data Register\n");
	v4l2_info(sd, "0x8800-0x88FF: HDMIRX HDCP Port Register\n");
	v4l2_info(sd, "0x8900-0x89FF: HDMIRX Video Output Port & 3D Register\n");
	v4l2_info(sd, "0x8A00-0x8BFF: Reserved\n");
	v4l2_info(sd, "0x8C00-0x8FFF: HDMIRX EDID-RAM (1024bytes)\n");
	v4l2_info(sd, "0x9000-0x90FF: HDMIRX GBD Extraction Control\n");
	v4l2_info(sd, "0x9100-0x92FF: HDMIRX GBD RAM read\n");
	v4l2_info(sd, "0x9300-      : Reserved\n");
}

static int tc358743_get_reg_size(u16 address)
{
	/* REF_01 p. 66-72 */
	if (address <= 0x00ff)
		return 2;
	else if ((address >= 0x0100) && (address <= 0x06FF))
		return 4;
	else if ((address >= 0x0700) && (address <= 0x84ff))
		return 2;
	else
		return 1;
}

static int tc358743_g_register(struct v4l2_subdev *sd,
			       struct v4l2_dbg_register *reg)
{
	if (reg->reg > 0xffff) {
		tc358743_print_register_map(sd);
		return -EINVAL;
	}

	reg->size = tc358743_get_reg_size(reg->reg);

	reg->val = i2c_rdreg(sd, reg->reg, reg->size);

	return 0;
}

static int tc358743_s_register(struct v4l2_subdev *sd,
			       const struct v4l2_dbg_register *reg)
{
	if (reg->reg > 0xffff) {
		tc358743_print_register_map(sd);
		return -EINVAL;
	}

	/* It should not be possible for the user to enable HDCP with a simple
	 * v4l2-dbg command.
	 *
	 * DO NOT REMOVE THIS unless all other issues with HDCP have been
	 * resolved.
	 */
	if (reg->reg == HDCP_MODE ||
	    reg->reg == HDCP_REG1 ||
	    reg->reg == HDCP_REG2 ||
	    reg->reg == HDCP_REG3 ||
	    reg->reg == BCAPS)
		return 0;

	i2c_wrreg(sd, (u16)reg->reg, reg->val,
			tc358743_get_reg_size(reg->reg));

	return 0;
}
#endif

static int tc358743_isr(struct v4l2_subdev *sd, u32 status, bool *handled)
{
	u16 intstatus = i2c_rd16(sd, INTSTATUS);

	//v4l2_dbg(1, debug, sd, "%s: IntStatus = 0x%04x\n", __func__, intstatus);

	if (intstatus & MASK_HDMI_INT) {
		u8 hdmi_int0 = i2c_rd8(sd, HDMI_INT0);
		u8 hdmi_int1 = i2c_rd8(sd, HDMI_INT1);

		if (hdmi_int0 & MASK_I_MISC)
			tc358743_hdmi_misc_int_handler(sd, handled);
		if (hdmi_int1 & MASK_I_CBIT)
			tc358743_hdmi_cbit_int_handler(sd, handled);
		if (hdmi_int1 & MASK_I_CLK)
			tc358743_hdmi_clk_int_handler(sd, handled);
		if (hdmi_int1 & MASK_I_SYS)
			tc358743_hdmi_sys_int_handler(sd, handled);
		if (hdmi_int1 & MASK_I_AUD)
			tc358743_hdmi_audio_int_handler(sd, handled);

		i2c_wr16(sd, INTSTATUS, MASK_HDMI_INT);
		intstatus &= ~MASK_HDMI_INT;
	}

#ifdef CONFIG_VIDEO_TC358743_CEC
	if (intstatus & (MASK_CEC_RINT | MASK_CEC_TINT)) {
		tc358743_cec_handler(sd, intstatus, handled);
		i2c_wr16(sd, INTSTATUS,
			 intstatus & (MASK_CEC_RINT | MASK_CEC_TINT));
		intstatus &= ~(MASK_CEC_RINT | MASK_CEC_TINT);
	}
#endif

	if (intstatus & MASK_CSI_INT) {
		u32 csi_int = i2c_rd32(sd, CSI_INT);

		if (csi_int & MASK_INTER)
			tc358743_csi_err_int_handler(sd, handled);

		i2c_wr16(sd, INTSTATUS, MASK_CSI_INT);
	}

	intstatus = i2c_rd16(sd, INTSTATUS);
	if (intstatus) {
		v4l2_dbg(1, debug, sd,
				"%s: Unhandled IntStatus interrupts: 0x%02x\n",
				__func__, intstatus);
	}

	return 0;
}

static irqreturn_t tc358743_irq_handler(int irq, void *dev_id)
{
	struct tc358743_state *state = dev_id;
	bool handled = false;

	tc358743_isr(&state->sd, 0, &handled);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static void tc358743_irq_poll_timer(struct timer_list *t)
{
	struct tc358743_state *state = from_timer(state, t, timer);
	unsigned int msecs;

	schedule_work(&state->work_i2c_poll);
	/*
	 * If CEC is present, then we need to poll more frequently,
	 * otherwise we will miss CEC messages.
	 */
	msecs = state->cec_adap ? POLL_INTERVAL_CEC_MS : POLL_INTERVAL_MS;
	mod_timer(&state->timer, jiffies + msecs_to_jiffies(msecs));
}

static void tc358743_work_i2c_poll(struct work_struct *work)
{
	struct tc358743_state *state = container_of(work,
			struct tc358743_state, work_i2c_poll);
	bool handled;

	tc358743_isr(&state->sd, 0, &handled);
}

static int tc358743_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				    struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

/* --------------- VIDEO OPS --------------- */

static int tc358743_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	*status = 0;
	*status |= no_signal(sd) ? V4L2_IN_ST_NO_SIGNAL : 0;
	*status |= no_sync(sd) ? V4L2_IN_ST_NO_SYNC : 0;

	v4l2_dbg(1, debug, sd, "%s: status = 0x%x\n", __func__, *status);

	return 0;
}

static int tc358743_s_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct tc358743_state *state = to_state(sd);

	if (!timings)
		return -EINVAL;

	if (debug)
		v4l2_print_dv_timings(sd->name, "tc358743_s_dv_timings: ",
				timings, false);

	if (v4l2_match_dv_timings(&state->timings, timings, 0, false)) {
		v4l2_dbg(1, debug, sd, "%s: no change\n", __func__);
		return 0;
	}

	if (!v4l2_valid_dv_timings(timings,
				&tc358743_timings_cap, NULL, NULL)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	state->timings = *timings;

	enable_stream(sd, false);
	tc358743_set_pll(sd);
	tc358743_set_csi(sd);

	return 0;
}

static int tc358743_g_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct tc358743_state *state = to_state(sd);

	*timings = state->timings;

	return 0;
}

static int tc358743_enum_dv_timings(struct v4l2_subdev *sd,
				    struct v4l2_enum_dv_timings *timings)
{
	if (timings->pad != 0)
		return -EINVAL;

	return v4l2_enum_dv_timings_cap(timings,
			&tc358743_timings_cap, NULL, NULL);
}

static int tc358743_query_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	int ret;

	ret = tc358743_get_detected_timings(sd, timings);
	if (ret)
		return ret;

	if (debug)
		v4l2_print_dv_timings(sd->name, "tc358743_query_dv_timings: ",
				timings, false);

	if (!v4l2_valid_dv_timings(timings,
				&tc358743_timings_cap, NULL, NULL)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	return 0;
}

static int tc358743_dv_timings_cap(struct v4l2_subdev *sd,
		struct v4l2_dv_timings_cap *cap)
{
	if (cap->pad != 0)
		return -EINVAL;

	*cap = tc358743_timings_cap;

	return 0;
}

static int tc358743_get_mbus_config(struct v4l2_subdev *sd,
				    unsigned int pad,
				    struct v4l2_mbus_config *cfg)
{
	struct tc358743_state *state = to_state(sd);

	cfg->type = V4L2_MBUS_CSI2_DPHY;

	/* Support for non-continuous CSI-2 clock is missing in the driver */
	cfg->bus.mipi_csi2.flags = 0;
	cfg->bus.mipi_csi2.num_data_lanes = state->csi_lanes_in_use;

	return 0;
}

static int tc358743_s_stream(struct v4l2_subdev *sd, int enable)
{
	enable_stream(sd, enable);
	if (!enable) {
		/* Put all lanes in LP-11 state (STOPSTATE) */
		tc358743_set_csi(sd);
	}

	return 0;
}

/* --------------- PAD OPS --------------- */

static int tc358743_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_mbus_code_enum *code)
{
	switch (code->index) {
	case 0:
		code->code = MEDIA_BUS_FMT_RGB888_1X24;
		break;
	case 1:
		code->code = MEDIA_BUS_FMT_UYVY8_1X16;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static u32 tc358743_g_colorspace(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_RGB888_1X24:
		return V4L2_COLORSPACE_SRGB;
	case MEDIA_BUS_FMT_UYVY8_1X16:
		return V4L2_COLORSPACE_SMPTE170M;
	default:
		return 0;
	}
}

static int tc358743_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_format *format)
{
	struct tc358743_state *state = to_state(sd);

	if (format->pad != 0)
		return -EINVAL;

	format->format.code = state->mbus_fmt_code;
	format->format.width = state->timings.bt.width;
	format->format.height = state->timings.bt.height;
	format->format.field = V4L2_FIELD_NONE;

	format->format.colorspace = tc358743_g_colorspace(format->format.code);

	return 0;
}

static int tc358743_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_format *format)
{
	struct tc358743_state *state = to_state(sd);

	u32 code = format->format.code; /* is overwritten by get_fmt */
	int ret = tc358743_get_fmt(sd, sd_state, format);

	if (code == MEDIA_BUS_FMT_RGB888_1X24 ||
	    code == MEDIA_BUS_FMT_UYVY8_1X16)
		format->format.code = code;
	format->format.colorspace = tc358743_g_colorspace(format->format.code);

	if (ret)
		return ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	state->mbus_fmt_code = format->format.code;

	enable_stream(sd, false);
	tc358743_set_pll(sd);
	tc358743_set_csi(sd);
	tc358743_set_csi_color_space(sd);

	return 0;
}

static int tc358743_g_edid(struct v4l2_subdev *sd,
		struct v4l2_subdev_edid *edid)
{
	struct tc358743_state *state = to_state(sd);

	memset(edid->reserved, 0, sizeof(edid->reserved));

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->start_block == 0 && edid->blocks == 0) {
		edid->blocks = state->edid_blocks_written;
		return 0;
	}

	if (state->edid_blocks_written == 0)
		return -ENODATA;

	if (edid->start_block >= state->edid_blocks_written ||
			edid->blocks == 0)
		return -EINVAL;

	if (edid->start_block + edid->blocks > state->edid_blocks_written)
		edid->blocks = state->edid_blocks_written - edid->start_block;

	i2c_rd(sd, EDID_RAM + (edid->start_block * EDID_BLOCK_SIZE), edid->edid,
			edid->blocks * EDID_BLOCK_SIZE);

	return 0;
}

static int tc358743_s_edid(struct v4l2_subdev *sd,
				struct v4l2_subdev_edid *edid)
{
	struct tc358743_state *state = to_state(sd);
	u16 edid_len = edid->blocks * EDID_BLOCK_SIZE;
	u16 pa;
	int err;
	int i;

	v4l2_dbg(2, debug, sd, "%s, pad %d, start block %d, blocks %d\n",
		 __func__, edid->pad, edid->start_block, edid->blocks);

	memset(edid->reserved, 0, sizeof(edid->reserved));

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->start_block != 0)
		return -EINVAL;

	if (edid->blocks > EDID_NUM_BLOCKS_MAX) {
		edid->blocks = EDID_NUM_BLOCKS_MAX;
		return -E2BIG;
	}
	pa = cec_get_edid_phys_addr(edid->edid, edid->blocks * 128, NULL);
	err = v4l2_phys_addr_validate(pa, &pa, NULL);
	if (err)
		return err;

	cec_phys_addr_invalidate(state->cec_adap);

	tc358743_disable_edid(sd);

	i2c_wr8(sd, EDID_LEN1, edid_len & 0xff);
	i2c_wr8(sd, EDID_LEN2, edid_len >> 8);

	if (edid->blocks == 0) {
		state->edid_blocks_written = 0;
		return 0;
	}

	for (i = 0; i < edid_len; i += EDID_BLOCK_SIZE)
		i2c_wr(sd, EDID_RAM + i, edid->edid + i, EDID_BLOCK_SIZE);

	state->edid_blocks_written = edid->blocks;

	cec_s_phys_addr(state->cec_adap, pa, false);

	if (tx_5v_power_present(sd))
		tc358743_enable_edid(sd);

	return 0;
}

/* -------------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops tc358743_core_ops = {
	.log_status = tc358743_log_status,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = tc358743_g_register,
	.s_register = tc358743_s_register,
#endif
	.interrupt_service_routine = tc358743_isr,
	.subscribe_event = tc358743_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops tc358743_video_ops = {
	.g_input_status = tc358743_g_input_status,
	.s_dv_timings = tc358743_s_dv_timings,
	.g_dv_timings = tc358743_g_dv_timings,
	.query_dv_timings = tc358743_query_dv_timings,
	.s_stream = tc358743_s_stream,
};

static const struct v4l2_subdev_pad_ops tc358743_pad_ops = {
	.enum_mbus_code = tc358743_enum_mbus_code,
	.set_fmt = tc358743_set_fmt,
	.get_fmt = tc358743_get_fmt,
	.get_edid = tc358743_g_edid,
	.set_edid = tc358743_s_edid,
	.enum_dv_timings = tc358743_enum_dv_timings,
	.dv_timings_cap = tc358743_dv_timings_cap,
	.get_mbus_config = tc358743_get_mbus_config,
};

static const struct v4l2_subdev_ops tc358743_ops = {
	.core = &tc358743_core_ops,
	.video = &tc358743_video_ops,
	.pad = &tc358743_pad_ops,
};

/* --------------- CUSTOM CTRLS --------------- */

static const struct v4l2_ctrl_config tc358743_ctrl_audio_sampling_rate = {
	.id = TC358743_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio sampling rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 768000,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config tc358743_ctrl_audio_present = {
	.id = TC358743_CID_AUDIO_PRESENT,
	.name = "Audio present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

/* --------------- PROBE / REMOVE --------------- */

#ifdef CONFIG_OF
static void tc358743_gpio_reset(struct tc358743_state *state)
{
	usleep_range(5000, 10000);
	gpiod_set_value(state->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value(state->reset_gpio, 0);
	msleep(20);
}

static int tc358743_probe_of(struct tc358743_state *state)
{
	struct device *dev = &state->i2c_client->dev;
	struct v4l2_fwnode_endpoint endpoint = { .bus_type = 0 };
	struct device_node *ep;
	struct clk *refclk;
	u32 bps_pr_lane;
	int ret;

	refclk = devm_clk_get(dev, "refclk");
	if (IS_ERR(refclk))
		return dev_err_probe(dev, PTR_ERR(refclk),
				     "failed to get refclk\n");

	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(ep), &endpoint);
	if (ret) {
		dev_err(dev, "failed to parse endpoint\n");
		goto put_node;
	}

	if (endpoint.bus_type != V4L2_MBUS_CSI2_DPHY ||
	    endpoint.bus.mipi_csi2.num_data_lanes == 0 ||
	    endpoint.nr_of_link_frequencies == 0) {
		dev_err(dev, "missing CSI-2 properties in endpoint\n");
		ret = -EINVAL;
		goto free_endpoint;
	}

	if (endpoint.bus.mipi_csi2.num_data_lanes > 4) {
		dev_err(dev, "invalid number of lanes\n");
		ret = -EINVAL;
		goto free_endpoint;
	}

	state->bus = endpoint.bus.mipi_csi2;

	ret = clk_prepare_enable(refclk);
	if (ret) {
		dev_err(dev, "Failed! to enable clock\n");
		goto free_endpoint;
	}

	state->pdata.refclk_hz = clk_get_rate(refclk);
	state->pdata.ddc5v_delay = DDC5V_DELAY_100_MS;
	state->pdata.enable_hdcp = false;
	/* A FIFO level of 16 should be enough for 2-lane 720p60 at 594 MHz. */
	state->pdata.fifo_level = 374;
	/*
	 * The PLL input clock is obtained by dividing refclk by pll_prd.
	 * It must be between 6 MHz and 40 MHz, lower frequency is better.
	 */
	switch (state->pdata.refclk_hz) {
	case 26000000:
	case 27000000:
	case 42000000:
		state->pdata.pll_prd = state->pdata.refclk_hz / 6000000;
		break;
	default:
		dev_err(dev, "unsupported refclk rate: %u Hz\n",
			state->pdata.refclk_hz);
		goto disable_clk;
	}

	/*
	 * The CSI bps per lane must be between 62.5 Mbps and 1 Gbps.
	 * The default is 594 Mbps for 4-lane 1080p60 or 2-lane 720p60.
	 * 972 Mbps allows 1080P50 UYVY over 2-lane.
	 */
	bps_pr_lane = 2 * endpoint.link_frequencies[0];
	if (bps_pr_lane < 62500000U || bps_pr_lane > 1000000000U) {
		dev_err(dev, "unsupported bps per lane: %u bps\n", bps_pr_lane);
		ret = -EINVAL;
		goto disable_clk;
	}

	/* The CSI speed per lane is refclk / pll_prd * pll_fbd */
	state->pdata.pll_fbd = bps_pr_lane /
			       state->pdata.refclk_hz * state->pdata.pll_prd;

	/*
	 * FIXME: These timings are from REF_02 for 594 or 972 Mbps per lane
	 * (297 MHz or 486 MHz link frequency).
	 * In principle it should be possible to calculate
	 * them based on link frequency and resolution.
	 */
	switch (bps_pr_lane) {
	default:
		dev_warn(dev, "untested bps per lane: %u bps\n", bps_pr_lane);
		fallthrough;
	case 594000000U:
		state->pdata.lineinitcnt = 0xe80;
		state->pdata.lptxtimecnt = 0x003;
		/* tclk-preparecnt: 3, tclk-zerocnt: 20 */
		state->pdata.tclk_headercnt = 0x1403;
		state->pdata.tclk_trailcnt = 0x00;
		/* ths-preparecnt: 3, ths-zerocnt: 1 */
		state->pdata.ths_headercnt = 0x0103;
		state->pdata.twakeup = 0x4882;
		state->pdata.tclk_postcnt = 0x008;
		state->pdata.ths_trailcnt = 0x2;
		state->pdata.hstxvregcnt = 0;
		break;
	case 972000000U:
		state->pdata.lineinitcnt = 0x1b58;
		state->pdata.lptxtimecnt = 0x007;
		/* tclk-preparecnt: 6, tclk-zerocnt: 40 */
		state->pdata.tclk_headercnt = 0x2806;
		state->pdata.tclk_trailcnt = 0x00;
		/* ths-preparecnt: 6, ths-zerocnt: 8 */
		state->pdata.ths_headercnt = 0x0806;
		state->pdata.twakeup = 0x4268;
		state->pdata.tclk_postcnt = 0x008;
		state->pdata.ths_trailcnt = 0x5;
		state->pdata.hstxvregcnt = 0;
		break;
	}

	state->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						    GPIOD_OUT_LOW);
	if (IS_ERR(state->reset_gpio)) {
		dev_err(dev, "failed to get reset gpio\n");
		ret = PTR_ERR(state->reset_gpio);
		goto disable_clk;
	}

	if (state->reset_gpio)
		tc358743_gpio_reset(state);

	ret = 0;
	goto free_endpoint;

disable_clk:
	clk_disable_unprepare(refclk);
free_endpoint:
	v4l2_fwnode_endpoint_free(&endpoint);
put_node:
	of_node_put(ep);
	return ret;
}
#else
static inline int tc358743_probe_of(struct tc358743_state *state)
{
	return -ENODEV;
}
#endif
#pragma GCC diagnostic warning "-Wdate-time"
static int tc358743_probe(struct i2c_client *client)
{
	static struct v4l2_dv_timings default_timing =
		V4L2_DV_BT_CEA_640X480P59_94;
	struct tc358743_state *state;
	struct tc358743_platform_data *pdata = client->dev.platform_data;
	struct v4l2_subdev *sd;
	u16 irq_mask = MASK_HDMI_MSK | MASK_CSI_MSK;
	u16 chipid;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;
	v4l_dbg(1, debug, client, "chip found @ 0x%x (%s) "__DATE__"	"__TIME__"\n",
		client->addr << 1, client->adapter->name);

	state = devm_kzalloc(&client->dev, sizeof(struct tc358743_state),
			GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->i2c_client = client;

	/* platform data */
	if (pdata) {
		state->pdata = *pdata;
		state->bus.flags = 0;
	} else {
		err = tc358743_probe_of(state);
		if (err == -ENODEV)
			v4l_err(client, "No platform data!\n");
		if (err)
			return err;
	}

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &tc358743_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	/* i2c access */
	if (i2c_rd16_err(sd, CHIPID, &chipid) ||
	    (chipid & MASK_CHIPID) != 0) {
		v4l2_info(sd, "not a TC358743 on address 0x%x\n",
			  client->addr << 1);
		return -ENODEV;
	}

	/* control handlers */
	v4l2_ctrl_handler_init(&state->hdl, 3);

	state->detect_tx_5v_ctrl = v4l2_ctrl_new_std(&state->hdl, NULL,
			V4L2_CID_DV_RX_POWER_PRESENT, 0, 1, 0, 0);

	/* custom controls */
	state->audio_sampling_rate_ctrl = v4l2_ctrl_new_custom(&state->hdl,
			&tc358743_ctrl_audio_sampling_rate, NULL);

	state->audio_present_ctrl = v4l2_ctrl_new_custom(&state->hdl,
			&tc358743_ctrl_audio_present, NULL);

	sd->ctrl_handler = &state->hdl;
	if (state->hdl.error) {
		err = state->hdl.error;
		goto err_hdl;
	}

	if (tc358743_update_controls(sd)) {
		err = -ENODEV;
		goto err_hdl;
	}

	state->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	err = media_entity_pads_init(&sd->entity, 1, &state->pad);
	if (err < 0)
		goto err_hdl;

	state->mbus_fmt_code = MEDIA_BUS_FMT_RGB888_1X24;

	sd->dev = &client->dev;

	mutex_init(&state->confctl_mutex);

	INIT_DELAYED_WORK(&state->delayed_work_enable_hotplug,
			tc358743_delayed_work_enable_hotplug);

#ifdef CONFIG_VIDEO_TC358743_CEC
	state->cec_adap = cec_allocate_adapter(&tc358743_cec_adap_ops,
		state, dev_name(&client->dev),
		CEC_CAP_DEFAULTS | CEC_CAP_MONITOR_ALL, CEC_MAX_LOG_ADDRS);
	if (IS_ERR(state->cec_adap)) {
		err = PTR_ERR(state->cec_adap);
		goto err_hdl;
	}
	irq_mask |= MASK_CEC_RMSK | MASK_CEC_TMSK;
#endif

	tc358743_initial_setup(sd);

	tc358743_s_dv_timings(sd, &default_timing);

	tc358743_set_csi_color_space(sd);

	tc358743_init_interrupts(sd);

	if (state->i2c_client->irq) {
		err = devm_request_threaded_irq(&client->dev,
						state->i2c_client->irq,
						NULL, tc358743_irq_handler,
						IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
						"tc358743", state);
		if (err)
			goto err_work_queues;
	} else {
		INIT_WORK(&state->work_i2c_poll,
			  tc358743_work_i2c_poll);
		timer_setup(&state->timer, tc358743_irq_poll_timer, 0);
		state->timer.expires = jiffies +
				       msecs_to_jiffies(POLL_INTERVAL_MS);
		add_timer(&state->timer);
	}

	err = cec_register_adapter(state->cec_adap, &client->dev);
	if (err < 0) {
		pr_err("%s: failed to register the cec device\n", __func__);
		cec_delete_adapter(state->cec_adap);
		state->cec_adap = NULL;
		goto err_work_queues;
	}

	tc358743_enable_interrupts(sd, tx_5v_power_present(sd));
	i2c_wr16(sd, INTMASK, ~irq_mask);

	err = v4l2_ctrl_handler_setup(sd->ctrl_handler);
	if (err)
		goto err_work_queues;

	err = v4l2_async_register_subdev(sd);
	if (err < 0)
		goto err_work_queues;

	v4l2_info(sd, "%s found @ 0x%x (%s) %s %s\n", client->name,
		  client->addr << 1, client->adapter->name, __DATE__, __TIME__);

	return 0;

err_work_queues:
	cec_unregister_adapter(state->cec_adap);
	if (!state->i2c_client->irq)
		flush_work(&state->work_i2c_poll);
	cancel_delayed_work(&state->delayed_work_enable_hotplug);
	mutex_destroy(&state->confctl_mutex);
err_hdl:
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&state->hdl);
	return err;
}

static void tc358743_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tc358743_state *state = to_state(sd);

	if (!state->i2c_client->irq) {
		del_timer_sync(&state->timer);
		flush_work(&state->work_i2c_poll);
	}
	cancel_delayed_work_sync(&state->delayed_work_enable_hotplug);
	cec_unregister_adapter(state->cec_adap);
	v4l2_async_unregister_subdev(sd);
	v4l2_device_unregister_subdev(sd);
	mutex_destroy(&state->confctl_mutex);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&state->hdl);
}

static const struct i2c_device_id tc358743_id[] = {
	{"tc358743", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tc358743_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id tc358743_of_match[] = {
	{ .compatible = "toshiba,tc358743" },
	{},
};
MODULE_DEVICE_TABLE(of, tc358743_of_match);
#endif

static struct i2c_driver tc358743_driver = {
	.driver = {
		.name = "tc358743",
		.of_match_table = of_match_ptr(tc358743_of_match),
	},
	.probe = tc358743_probe,
	.remove = tc358743_remove,
	.id_table = tc358743_id,
};

module_i2c_driver(tc358743_driver);

//========================================================================
//========================================================================
//========================================================================

// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX708 cameras.
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * Based on Sony imx477 camera driver
 * Copyright (C) 2020 Raspberry Pi Ltd
 */
#include <asm/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

/*
 * Parameter to adjust Quad Bayer re-mosaic broken line correction
 * strength, used in full-resolution mode only. Set zero to disable.
 */
static int qbc_adjust = 2;
module_param(qbc_adjust, int, 0644);
MODULE_PARM_DESC(qbc_adjust, "Quad Bayer broken line correction strength [0,2-5]");

#define IMX911_REG_VALUE_08BIT		1
#define IMX911_REG_VALUE_16BIT		2

/* Chip ID */
#define IMX911_REG_CHIP_ID		0x0016
#define IMX911_CHIP_ID			0x0708

#define IMX911_REG_MODE_SELECT		0x0100
#define IMX911_MODE_STANDBY		0x00
#define IMX911_MODE_STREAMING		0x01

#define IMX911_REG_ORIENTATION		0x101

#define IMX911_INCLK_FREQ		24000000

/* Default initial pixel rate, will get updated for each mode. */
#define IMX911_INITIAL_PIXEL_RATE	590000000

/* V_TIMING internal */
#define IMX911_REG_FRAME_LENGTH		0x0340
#define IMX911_FRAME_LENGTH_MAX		0xffff

/* Long exposure multiplier */
#define IMX911_LONG_EXP_SHIFT_MAX	7
#define IMX911_LONG_EXP_SHIFT_REG	0x3100

/* Exposure control */
#define IMX911_REG_EXPOSURE		0x0202
#define IMX911_EXPOSURE_OFFSET		48
#define IMX911_EXPOSURE_DEFAULT		0x640
#define IMX911_EXPOSURE_STEP		1
#define IMX911_EXPOSURE_MIN		1
#define IMX911_EXPOSURE_MAX		(IMX911_FRAME_LENGTH_MAX - \
					 IMX911_EXPOSURE_OFFSET)

/* Analog gain control */
#define IMX911_REG_ANALOG_GAIN		0x0204
#define IMX911_ANA_GAIN_MIN		112
#define IMX911_ANA_GAIN_MAX		960
#define IMX911_ANA_GAIN_STEP		1
#define IMX911_ANA_GAIN_DEFAULT	   IMX911_ANA_GAIN_MIN

/* Digital gain control */
#define IMX911_REG_DIGITAL_GAIN		0x020e
#define IMX911_DGTL_GAIN_MIN		0x0100
#define IMX911_DGTL_GAIN_MAX		0xffff
#define IMX911_DGTL_GAIN_DEFAULT	0x0100
#define IMX911_DGTL_GAIN_STEP		1

/* Colour balance controls */
#define IMX911_REG_COLOUR_BALANCE_RED   0x0b90
#define IMX911_REG_COLOUR_BALANCE_BLUE	0x0b92
#define IMX911_COLOUR_BALANCE_MIN	0x01
#define IMX911_COLOUR_BALANCE_MAX	0xffff
#define IMX911_COLOUR_BALANCE_STEP	0x01
#define IMX911_COLOUR_BALANCE_DEFAULT	0x100

/* Test Pattern Control */
#define IMX911_REG_TEST_PATTERN		0x0600
#define IMX911_TEST_PATTERN_DISABLE	0
#define IMX911_TEST_PATTERN_SOLID_COLOR	1
#define IMX911_TEST_PATTERN_COLOR_BARS	2
#define IMX911_TEST_PATTERN_GREY_COLOR	3
#define IMX911_TEST_PATTERN_PN9		4

/* Test pattern colour components */
#define IMX911_REG_TEST_PATTERN_R	0x0602
#define IMX911_REG_TEST_PATTERN_GR	0x0604
#define IMX911_REG_TEST_PATTERN_B	0x0606
#define IMX911_REG_TEST_PATTERN_GB	0x0608
#define IMX911_TEST_PATTERN_COLOUR_MIN	0
#define IMX911_TEST_PATTERN_COLOUR_MAX	0x0fff
#define IMX911_TEST_PATTERN_COLOUR_STEP	1

#define IMX911_REG_BASE_SPC_GAINS_L	0x7b10
#define IMX911_REG_BASE_SPC_GAINS_R	0x7c00

/* HDR exposure ratio (long:med == med:short) */
#define IMX911_HDR_EXPOSURE_RATIO       4
#define IMX911_REG_MID_EXPOSURE		0x3116
#define IMX911_REG_SHT_EXPOSURE		0x0224
#define IMX911_REG_MID_ANALOG_GAIN	0x3118
#define IMX911_REG_SHT_ANALOG_GAIN	0x0216

/* QBC Re-mosaic broken line correction registers */
#define IMX911_LPF_INTENSITY_EN		0xC428
#define IMX911_LPF_INTENSITY_ENABLED	0x00
#define IMX911_LPF_INTENSITY_DISABLED	0x01
#define IMX911_LPF_INTENSITY		0xC429

/*
 * Metadata buffer holds a variety of data, all sent with the same VC/DT (0x12).
 * It comprises two scanlines (of up to 5760 bytes each, for 4608 pixels)
 * of embedded data, one line of PDAF data, and two lines of AE-HIST data
 * (AE histograms are valid for HDR mode and empty in non-HDR modes).
 */
#define IMX911_EMBEDDED_LINE_WIDTH (5 * 5760)
#define IMX911_NUM_EMBEDDED_LINES 1

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

/* IMX708 native and active pixel array size. */
#define IMX911_NATIVE_WIDTH		4640U
#define IMX911_NATIVE_HEIGHT		2658U
#define IMX911_PIXEL_ARRAY_LEFT		16U
#define IMX911_PIXEL_ARRAY_TOP		24U
#define IMX911_PIXEL_ARRAY_WIDTH	4608U
#define IMX911_PIXEL_ARRAY_HEIGHT	2592U

struct imx911_reg {
	u16 address;
	u8 val;
};

struct imx911_reg_list {
	unsigned int num_of_regs;
	const struct imx911_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx911_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* H-timing in pixels */
	unsigned int line_length_pix;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Highest possible framerate. */
	unsigned int vblank_min;

	/* Default framerate. */
	unsigned int vblank_default;

	/* Default register values */
	struct imx911_reg_list reg_list;

	/* Not all modes have the same pixel rate. */
	u64 pixel_rate;

	/* Not all modes have the same minimum exposure. */
	u32 exposure_lines_min;

	/* Not all modes have the same exposure lines step. */
	u32 exposure_lines_step;

	/* HDR flag, used for checking if the current mode is HDR */
	bool hdr;

	/* Quad Bayer Re-mosaic flag */
	bool remosaic;
};

/* Default PDAF pixel correction gains */
static const u8 pdaf_gains[2][9] = {
	{ 0x4c, 0x4c, 0x4c, 0x46, 0x3e, 0x38, 0x35, 0x35, 0x35 },
	{ 0x35, 0x35, 0x35, 0x38, 0x3e, 0x46, 0x4c, 0x4c, 0x4c }
};

/* Link frequency setup */
enum {
	IMX911_LINK_FREQ_450MHZ,
	IMX911_LINK_FREQ_447MHZ,
	IMX911_LINK_FREQ_453MHZ,
};

static const s64 link_freqs[] = {
	[IMX911_LINK_FREQ_450MHZ] = 450000000,
	[IMX911_LINK_FREQ_447MHZ] = 447000000,
	[IMX911_LINK_FREQ_453MHZ] = 453000000,
};

/* 450MHz is the nominal "default" link frequency */
static const struct imx911_reg link_450Mhz_regs[] = {
	{0x030E, 0x01},
	{0x030F, 0x2c},
};

static const struct imx911_reg link_447Mhz_regs[] = {
	{0x030E, 0x01},
	{0x030F, 0x2a},
};

static const struct imx911_reg link_453Mhz_regs[] = {
	{0x030E, 0x01},
	{0x030F, 0x2e},
};

static const struct imx911_reg_list link_freq_regs[] = {
	[IMX911_LINK_FREQ_450MHZ] = {
		.regs = link_450Mhz_regs,
		.num_of_regs = ARRAY_SIZE(link_450Mhz_regs)
	},
	[IMX911_LINK_FREQ_447MHZ] = {
		.regs = link_447Mhz_regs,
		.num_of_regs = ARRAY_SIZE(link_447Mhz_regs)
	},
	[IMX911_LINK_FREQ_453MHZ] = {
		.regs = link_453Mhz_regs,
		.num_of_regs = ARRAY_SIZE(link_453Mhz_regs)
	},
};

static const struct imx911_reg mode_common_regs[] = {
	{0x0100, 0x00},
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x33F0, 0x02},
	{0x33F1, 0x05},
	{0x3062, 0x00},
	{0x3063, 0x12},
	{0x3068, 0x00},
	{0x3069, 0x12},
	{0x306A, 0x00},
	{0x306B, 0x30},
	{0x3076, 0x00},
	{0x3077, 0x30},
	{0x3078, 0x00},
	{0x3079, 0x30},
	{0x5E54, 0x0C},
	{0x6E44, 0x00},
	{0xB0B6, 0x01},
	{0xE829, 0x00},
	{0xF001, 0x08},
	{0xF003, 0x08},
	{0xF00D, 0x10},
	{0xF00F, 0x10},
	{0xF031, 0x08},
	{0xF033, 0x08},
	{0xF03D, 0x10},
	{0xF03F, 0x10},
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x01},
	{0x0B8E, 0x01},
	{0x0B8F, 0x00},
	{0x0B94, 0x01},
	{0x0B95, 0x00},
	{0x3400, 0x01},
	{0x3478, 0x01},
	{0x3479, 0x1c},
	{0x3091, 0x01},
	{0x3092, 0x00},
	{0x3419, 0x00},
	{0xBCF1, 0x02},
	{0x3094, 0x01},
	{0x3095, 0x01},
	{0x3362, 0x00},
	{0x3363, 0x00},
	{0x3364, 0x00},
	{0x3365, 0x00},
	{0x0138, 0x01},
};

/* 10-bit. */
static const struct imx911_reg mode_4608x2592_regs[] = {
	{0x0342, 0x3D},
	{0x0343, 0x20},
	{0x0340, 0x0A},
	{0x0341, 0x59},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x11},
	{0x0349, 0xFF},
	{0x034A, 0X0A},
	{0x034B, 0x1F},
	{0x0220, 0x62},
	{0x0222, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x0A},
	{0x3200, 0x01},
	{0x3201, 0x01},
	{0x32D5, 0x01},
	{0x32D6, 0x00},
	{0x32DB, 0x01},
	{0x32DF, 0x00},
	{0x350C, 0x00},
	{0x350D, 0x00},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x12},
	{0x040D, 0x00},
	{0x040E, 0x0A},
	{0x040F, 0x20},
	{0x034C, 0x12},
	{0x034D, 0x00},
	{0x034E, 0x0A},
	{0x034F, 0x20},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0x7C},
	{0x030B, 0x02},
	{0x030D, 0x04},
	{0x0310, 0x01},
	{0x3CA0, 0x00},
	{0x3CA1, 0x64},
	{0x3CA4, 0x00},
	{0x3CA5, 0x00},
	{0x3CA6, 0x00},
	{0x3CA7, 0x00},
	{0x3CAA, 0x00},
	{0x3CAB, 0x00},
	{0x3CB8, 0x00},
	{0x3CB9, 0x08},
	{0x3CBA, 0x00},
	{0x3CBB, 0x00},
	{0x3CBC, 0x00},
	{0x3CBD, 0x3C},
	{0x3CBE, 0x00},
	{0x3CBF, 0x00},
	{0x0202, 0x0A},
	{0x0203, 0x29},
	{0x0224, 0x01},
	{0x0225, 0xF4},
	{0x3116, 0x01},
	{0x3117, 0xF4},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x0216, 0x00},
	{0x0217, 0x00},
	{0x0218, 0x01},
	{0x0219, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x3118, 0x00},
	{0x3119, 0x00},
	{0x311A, 0x01},
	{0x311B, 0x00},
	{0x341a, 0x00},
	{0x341b, 0x00},
	{0x341c, 0x00},
	{0x341d, 0x00},
	{0x341e, 0x01},
	{0x341f, 0x20},
	{0x3420, 0x00},
	{0x3421, 0xd8},
	{0x3366, 0x00},
	{0x3367, 0x00},
	{0x3368, 0x00},
	{0x3369, 0x00},
};

static const struct imx911_reg mode_2x2binned_regs[] = {
	{0x0342, 0x1E},
	{0x0343, 0x90},
	{0x0340, 0x05},
	{0x0341, 0x38},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x11},
	{0x0349, 0xFF},
	{0x034A, 0X0A},
	{0x034B, 0x1F},
	{0x0220, 0x62},
	{0x0222, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x08},
	{0x3200, 0x41},
	{0x3201, 0x41},
	{0x32D5, 0x00},
	{0x32D6, 0x00},
	{0x32DB, 0x01},
	{0x32DF, 0x00},
	{0x350C, 0x00},
	{0x350D, 0x00},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x09},
	{0x040D, 0x00},
	{0x040E, 0x05},
	{0x040F, 0x10},
	{0x034C, 0x09},
	{0x034D, 0x00},
	{0x034E, 0x05},
	{0x034F, 0x10},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0x7A},
	{0x030B, 0x02},
	{0x030D, 0x04},
	{0x0310, 0x01},
	{0x3CA0, 0x00},
	{0x3CA1, 0x3C},
	{0x3CA4, 0x00},
	{0x3CA5, 0x3C},
	{0x3CA6, 0x00},
	{0x3CA7, 0x00},
	{0x3CAA, 0x00},
	{0x3CAB, 0x00},
	{0x3CB8, 0x00},
	{0x3CB9, 0x1C},
	{0x3CBA, 0x00},
	{0x3CBB, 0x08},
	{0x3CBC, 0x00},
	{0x3CBD, 0x1E},
	{0x3CBE, 0x00},
	{0x3CBF, 0x0A},
	{0x0202, 0x05},
	{0x0203, 0x08},
	{0x0224, 0x01},
	{0x0225, 0xF4},
	{0x3116, 0x01},
	{0x3117, 0xF4},
	{0x0204, 0x00},
	{0x0205, 0x70},
	{0x0216, 0x00},
	{0x0217, 0x70},
	{0x0218, 0x01},
	{0x0219, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x3118, 0x00},
	{0x3119, 0x70},
	{0x311A, 0x01},
	{0x311B, 0x00},
	{0x341a, 0x00},
	{0x341b, 0x00},
	{0x341c, 0x00},
	{0x341d, 0x00},
	{0x341e, 0x00},
	{0x341f, 0x90},
	{0x3420, 0x00},
	{0x3421, 0x6c},
	{0x3366, 0x00},
	{0x3367, 0x00},
	{0x3368, 0x00},
	{0x3369, 0x00},
};

static const struct imx911_reg mode_2x2binned_720p_regs[] = {
	{0x0342, 0x14},
	{0x0343, 0x60},
	{0x0340, 0x04},
	{0x0341, 0xB6},
	{0x0344, 0x03},
	{0x0345, 0x00},
	{0x0346, 0x01},
	{0x0347, 0xB0},
	{0x0348, 0x0E},
	{0x0349, 0xFF},
	{0x034A, 0x08},
	{0x034B, 0x6F},
	{0x0220, 0x62},
	{0x0222, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x08},
	{0x3200, 0x41},
	{0x3201, 0x41},
	{0x32D5, 0x00},
	{0x32D6, 0x00},
	{0x32DB, 0x01},
	{0x32DF, 0x01},
	{0x350C, 0x00},
	{0x350D, 0x00},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x06},
	{0x040D, 0x00},
	{0x040E, 0x03},
	{0x040F, 0x60},
	{0x034C, 0x06},
	{0x034D, 0x00},
	{0x034E, 0x03},
	{0x034F, 0x60},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0x76},
	{0x030B, 0x02},
	{0x030D, 0x04},
	{0x0310, 0x01},
	{0x3CA0, 0x00},
	{0x3CA1, 0x3C},
	{0x3CA4, 0x01},
	{0x3CA5, 0x5E},
	{0x3CA6, 0x00},
	{0x3CA7, 0x00},
	{0x3CAA, 0x00},
	{0x3CAB, 0x00},
	{0x3CB8, 0x00},
	{0x3CB9, 0x0C},
	{0x3CBA, 0x00},
	{0x3CBB, 0x04},
	{0x3CBC, 0x00},
	{0x3CBD, 0x1E},
	{0x3CBE, 0x00},
	{0x3CBF, 0x05},
	{0x0202, 0x04},
	{0x0203, 0x86},
	{0x0224, 0x01},
	{0x0225, 0xF4},
	{0x3116, 0x01},
	{0x3117, 0xF4},
	{0x0204, 0x00},
	{0x0205, 0x70},
	{0x0216, 0x00},
	{0x0217, 0x70},
	{0x0218, 0x01},
	{0x0219, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x3118, 0x00},
	{0x3119, 0x70},
	{0x311A, 0x01},
	{0x311B, 0x00},
	{0x341a, 0x00},
	{0x341b, 0x00},
	{0x341c, 0x00},
	{0x341d, 0x00},
	{0x341e, 0x00},
	{0x341f, 0x60},
	{0x3420, 0x00},
	{0x3421, 0x48},
	{0x3366, 0x00},
	{0x3367, 0x00},
	{0x3368, 0x00},
	{0x3369, 0x00},
};

static const struct imx911_reg mode_hdr_regs[] = {
	{0x0342, 0x14},
	{0x0343, 0x60},
	{0x0340, 0x0A},
	{0x0341, 0x5B},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x11},
	{0x0349, 0xFF},
	{0x034A, 0X0A},
	{0x034B, 0x1F},
	{0x0220, 0x01},
	{0x0222, IMX911_HDR_EXPOSURE_RATIO},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x0A},
	{0x3200, 0x01},
	{0x3201, 0x01},
	{0x32D5, 0x00},
	{0x32D6, 0x00},
	{0x32DB, 0x01},
	{0x32DF, 0x00},
	{0x350C, 0x00},
	{0x350D, 0x00},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x09},
	{0x040D, 0x00},
	{0x040E, 0x05},
	{0x040F, 0x10},
	{0x034C, 0x09},
	{0x034D, 0x00},
	{0x034E, 0x05},
	{0x034F, 0x10},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xA2},
	{0x030B, 0x02},
	{0x030D, 0x04},
	{0x0310, 0x01},
	{0x3CA0, 0x00},
	{0x3CA1, 0x00},
	{0x3CA4, 0x00},
	{0x3CA5, 0x00},
	{0x3CA6, 0x00},
	{0x3CA7, 0x28},
	{0x3CAA, 0x00},
	{0x3CAB, 0x00},
	{0x3CB8, 0x00},
	{0x3CB9, 0x30},
	{0x3CBA, 0x00},
	{0x3CBB, 0x00},
	{0x3CBC, 0x00},
	{0x3CBD, 0x32},
	{0x3CBE, 0x00},
	{0x3CBF, 0x00},
	{0x0202, 0x0A},
	{0x0203, 0x2B},
	{0x0224, 0x0A},
	{0x0225, 0x2B},
	{0x3116, 0x0A},
	{0x3117, 0x2B},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x0216, 0x00},
	{0x0217, 0x00},
	{0x0218, 0x01},
	{0x0219, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x3118, 0x00},
	{0x3119, 0x00},
	{0x311A, 0x01},
	{0x311B, 0x00},
	{0x341a, 0x00},
	{0x341b, 0x00},
	{0x341c, 0x00},
	{0x341d, 0x00},
	{0x341e, 0x00},
	{0x341f, 0x90},
	{0x3420, 0x00},
	{0x3421, 0x6c},
	{0x3360, 0x01},
	{0x3361, 0x01},
	{0x3366, 0x09},
	{0x3367, 0x00},
	{0x3368, 0x05},
	{0x3369, 0x10},
};

/* Mode configs. Keep separate lists for when HDR is enabled or not. */
static const struct imx911_mode supported_modes_10bit_no_hdr[] = {
	{
		/* Full resolution. */
		.width = 4608,
		.height = 2592,
		.line_length_pix = 0x3d20,
		.crop = {
			.left = IMX911_PIXEL_ARRAY_LEFT,
			.top = IMX911_PIXEL_ARRAY_TOP,
			.width = 4608,
			.height = 2592,
		},
		.vblank_min = 58,
		.vblank_default = 58,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4608x2592_regs),
			.regs = mode_4608x2592_regs,
		},
		.pixel_rate = 595200000,
		.exposure_lines_min = 8,
		.exposure_lines_step = 1,
		.hdr = false,
		.remosaic = true
	},
	{
		/* regular 2x2 binned. */
		.width = 2304,
		.height = 1296,
		.line_length_pix = 0x1e90,
		.crop = {
			.left = IMX911_PIXEL_ARRAY_LEFT,
			.top = IMX911_PIXEL_ARRAY_TOP,
			.width = 4608,
			.height = 2592,
		},
		.vblank_min = 40,
		.vblank_default = 1198,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2x2binned_regs),
			.regs = mode_2x2binned_regs,
		},
		.pixel_rate = 585600000,
		.exposure_lines_min = 4,
		.exposure_lines_step = 2,
		.hdr = false,
		.remosaic = false
	},
	{
		/* 2x2 binned and cropped for 720p. */
		.width = 1536,
		.height = 864,
		.line_length_pix = 0x1460,
		.crop = {
			.left = IMX911_PIXEL_ARRAY_LEFT + 768,
			.top = IMX911_PIXEL_ARRAY_TOP + 432,
			.width = 3072,
			.height = 1728,
		},
		.vblank_min = 40,
		.vblank_default = 2755,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2x2binned_720p_regs),
			.regs = mode_2x2binned_720p_regs,
		},
		.pixel_rate = 566400000,
		.exposure_lines_min = 4,
		.exposure_lines_step = 2,
		.hdr = false,
		.remosaic = false
	},
};

static const struct imx911_mode supported_modes_10bit_hdr[] = {
	{
		/* There's only one HDR mode, which is 2x2 downscaled */
		.width = 2304,
		.height = 1296,
		.line_length_pix = 0x1460,
		.crop = {
			.left = IMX911_PIXEL_ARRAY_LEFT,
			.top = IMX911_PIXEL_ARRAY_TOP,
			.width = 4608,
			.height = 2592,
		},
		.vblank_min = 3673,
		.vblank_default = 3673,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_hdr_regs),
			.regs = mode_hdr_regs,
		},
		.pixel_rate = 777600000,
		.exposure_lines_min = 8 * IMX911_HDR_EXPOSURE_RATIO * IMX911_HDR_EXPOSURE_RATIO,
		.exposure_lines_step = 2 * IMX911_HDR_EXPOSURE_RATIO * IMX911_HDR_EXPOSURE_RATIO,
		.hdr = true,
		.remosaic = false
	}
};

/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	/* 10-bit modes. */
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const char * const imx911_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

static const int imx911_test_pattern_val[] = {
	IMX911_TEST_PATTERN_DISABLE,
	IMX911_TEST_PATTERN_COLOR_BARS,
	IMX911_TEST_PATTERN_SOLID_COLOR,
	IMX911_TEST_PATTERN_GREY_COLOR,
	IMX911_TEST_PATTERN_PN9,
};

/* regulator supplies */
static const char * const imx911_supply_name[] = {
	/* Supplies can be enabled in any order */
	"vana1",  /* Analog1 (2.8V) supply */
	"vana2",  /* Analog2 (1.8V) supply */
	"vdig",  /* Digital Core (1.1V) supply */
	"vddl",  /* IF (1.8V) supply */
};

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define IMX911_XCLR_MIN_DELAY_US	8000
#define IMX911_XCLR_DELAY_RANGE_US	1000

struct imx911 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	struct v4l2_mbus_framefmt fmt;

	struct clk *inclk;
	u32 inclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx911_supply_name)];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *hdr_mode;
	struct v4l2_ctrl *link_freq;
	struct {
		struct v4l2_ctrl *hflip;
		struct v4l2_ctrl *vflip;
	};

	/* Current mode */
	const struct imx911_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;

	/* Current long exposure factor in use. Set through V4L2_CID_VBLANK */
	unsigned int long_exp_shift;

	unsigned int link_freq_idx;
};

static inline struct imx911 *to_imx911(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx911, sd);
}

static inline void get_mode_table(unsigned int code,
				  const struct imx911_mode **mode_list,
				  unsigned int *num_modes,
				  bool hdr_enable)
{
	switch (code) {
	/* 10-bit */
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		if (hdr_enable) {
			*mode_list = supported_modes_10bit_hdr;
			*num_modes = ARRAY_SIZE(supported_modes_10bit_hdr);
		} else {
			*mode_list = supported_modes_10bit_no_hdr;
			*num_modes = ARRAY_SIZE(supported_modes_10bit_no_hdr);
		}
		break;
	default:
		*mode_list = NULL;
		*num_modes = 0;
	}
}

/* Read registers up to 2 at a time */
static int imx911_read_reg(struct imx911 *imx911, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx911->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

static uint8_t fakedevice[0x1000];

/* Write registers up to 2 at a time */
static int imx911_write_reg(struct imx911 *imx911, u16 reg, u32 len, u32 val)
{
#if 0
	struct i2c_client *client = v4l2_get_subdevdata(&imx911->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;
#else
    if (len) fakedevice[reg] = val & 0xFF;
    if (len > 1) fakedevice[reg+1] = (val >> 8) & 0xFF ;
#endif
	return 0;
}

/* Write a list of registers */
static int imx911_write_regs(struct imx911 *imx911,
			     const struct imx911_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx911->sd);
	unsigned int i;

	for (i = 0; i < len; i++) {
		int ret;

		ret = imx911_write_reg(imx911, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

/* Get bayer order based on flip setting. */
static u32 imx911_get_format_code(struct imx911 *imx911)
{
	unsigned int i;

	lockdep_assert_held(&imx911->mutex);

	i = (imx911->vflip->val ? 2 : 0) |
	    (imx911->hflip->val ? 1 : 0);

	return codes[i];
}

static void imx911_set_default_format(struct imx911 *imx911)
{
	struct v4l2_mbus_framefmt *fmt = &imx911->fmt;

	/* Set default mode to max resolution */
	imx911->mode = &supported_modes_10bit_no_hdr[0];

	/* fmt->code not set as it will always be computed based on flips */
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = imx911->mode->width;
	fmt->height = imx911->mode->height;
	fmt->field = V4L2_FIELD_NONE;
}

static int imx911_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx911 *imx911 = to_imx911(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&imx911->mutex);

	/* Initialize try_fmt for the image pad */
	if (imx911->hdr_mode->val) {
		try_fmt_img->width = supported_modes_10bit_hdr[0].width;
		try_fmt_img->height = supported_modes_10bit_hdr[0].height;
	} else {
		try_fmt_img->width = supported_modes_10bit_no_hdr[0].width;
		try_fmt_img->height = supported_modes_10bit_no_hdr[0].height;
	}
	try_fmt_img->code = imx911_get_format_code(imx911);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = IMX911_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = IMX911_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, IMAGE_PAD);
	try_crop->left = IMX911_PIXEL_ARRAY_LEFT;
	try_crop->top = IMX911_PIXEL_ARRAY_TOP;
	try_crop->width = IMX911_PIXEL_ARRAY_WIDTH;
	try_crop->height = IMX911_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx911->mutex);

	return 0;
}

static int imx911_set_exposure(struct imx911 *imx911, unsigned int val)
{
	val = max(val, imx911->mode->exposure_lines_min);
	val -= val % imx911->mode->exposure_lines_step;

	/*
	 * In HDR mode this will set the longest exposure. The sensor
	 * will automatically divide the medium and short ones by 4,16.
	 */
	return imx911_write_reg(imx911, IMX911_REG_EXPOSURE,
				IMX911_REG_VALUE_16BIT,
				val >> imx911->long_exp_shift);
}

static void imx911_adjust_exposure_range(struct imx911 *imx911,
					 struct v4l2_ctrl *ctrl)
{
	int exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure. */
	exposure_max = imx911->mode->height + imx911->vblank->val -
		IMX911_EXPOSURE_OFFSET;
	exposure_def = min(exposure_max, imx911->exposure->val);
	__v4l2_ctrl_modify_range(imx911->exposure, imx911->exposure->minimum,
				 exposure_max, imx911->exposure->step,
				 exposure_def);
}

static int imx911_set_analogue_gain(struct imx911 *imx911, unsigned int val)
{
	int ret;

	/*
	 * In HDR mode this will set the gain for the longest exposure,
	 * and by default the sensor uses the same gain for all of them.
	 */
	ret = imx911_write_reg(imx911, IMX911_REG_ANALOG_GAIN,
			       IMX911_REG_VALUE_16BIT, val);

	return ret;
}

static int imx911_set_frame_length(struct imx911 *imx911, unsigned int val)
{
	int ret;

	imx911->long_exp_shift = 0;

	while (val > IMX911_FRAME_LENGTH_MAX) {
		imx911->long_exp_shift++;
		val >>= 1;
	}

	ret = imx911_write_reg(imx911, IMX911_REG_FRAME_LENGTH,
			       IMX911_REG_VALUE_16BIT, val);
	if (ret)
		return ret;

	return imx911_write_reg(imx911, IMX911_LONG_EXP_SHIFT_REG,
				IMX911_REG_VALUE_08BIT, imx911->long_exp_shift);
}

static void imx911_set_framing_limits(struct imx911 *imx911)
{
	const struct imx911_mode *mode = imx911->mode;
	unsigned int hblank;

	__v4l2_ctrl_modify_range(imx911->pixel_rate,
				 mode->pixel_rate, mode->pixel_rate,
				 1, mode->pixel_rate);

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(imx911->vblank, mode->vblank_min,
				 ((1 << IMX911_LONG_EXP_SHIFT_MAX) *
					IMX911_FRAME_LENGTH_MAX) - mode->height,
				 1, mode->vblank_default);

	/*
	 * Currently PPL is fixed to the mode specified value, so hblank
	 * depends on mode->width only, and is not changeable in any
	 * way other than changing the mode.
	 */
	hblank = mode->line_length_pix - mode->width;
	__v4l2_ctrl_modify_range(imx911->hblank, hblank, hblank, 1, hblank);
}

static int imx911_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx911 *imx911 =
		container_of(ctrl->handler, struct imx911, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx911->sd);
	const struct imx911_mode *mode_list;
	unsigned int code, num_modes;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/*
		 * The VBLANK control may change the limits of usable exposure,
		 * so check and adjust if necessary.
		 */
		imx911_adjust_exposure_range(imx911, ctrl);
		break;

	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		/*
		 * The WIDE_DYNAMIC_RANGE control can also be applied immediately
		 * as it doesn't set any registers. Don't do anything if the mode
		 * already matches.
		 */
		if (imx911->mode && imx911->mode->hdr != ctrl->val) {
			code = imx911_get_format_code(imx911);
			get_mode_table(code, &mode_list, &num_modes, ctrl->val);
			imx911->mode = v4l2_find_nearest_size(mode_list,
							      num_modes,
							      width, height,
							      imx911->mode->width,
							      imx911->mode->height);
			imx911_set_framing_limits(imx911);
		}
		break;
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		imx911_set_analogue_gain(imx911, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx911_set_exposure(imx911, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx911_write_reg(imx911, IMX911_REG_DIGITAL_GAIN,
				       IMX911_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx911_write_reg(imx911, IMX911_REG_TEST_PATTERN,
				       IMX911_REG_VALUE_16BIT,
				       imx911_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = imx911_write_reg(imx911, IMX911_REG_TEST_PATTERN_R,
				       IMX911_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = imx911_write_reg(imx911, IMX911_REG_TEST_PATTERN_GR,
				       IMX911_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = imx911_write_reg(imx911, IMX911_REG_TEST_PATTERN_B,
				       IMX911_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = imx911_write_reg(imx911, IMX911_REG_TEST_PATTERN_GB,
				       IMX911_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx911_write_reg(imx911, IMX911_REG_ORIENTATION, 1,
				       imx911->hflip->val |
				       imx911->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK:
		ret = imx911_set_frame_length(imx911,
					      imx911->mode->height + ctrl->val);
		break;
	case V4L2_CID_NOTIFY_GAINS:
		ret = imx911_write_reg(imx911, IMX911_REG_COLOUR_BALANCE_BLUE,
				       IMX911_REG_VALUE_16BIT,
				       ctrl->p_new.p_u32[0]);
		if (ret)
			break;
		ret = imx911_write_reg(imx911, IMX911_REG_COLOUR_BALANCE_RED,
				       IMX911_REG_VALUE_16BIT,
				       ctrl->p_new.p_u32[3]);
		break;
	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		/* Already handled above. */
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx911_ctrl_ops = {
	.s_ctrl = imx911_set_ctrl,
};

static int imx911_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx911 *imx911 = to_imx911(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index >= (ARRAY_SIZE(codes) / 4))
			return -EINVAL;

		code->code = imx911_get_format_code(imx911);
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int imx911_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx911 *imx911 = to_imx911(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		const struct imx911_mode *mode_list;
		unsigned int num_modes;

		get_mode_table(fse->code, &mode_list, &num_modes,
			       imx911->hdr_mode->val);

		if (fse->index >= num_modes)
			return -EINVAL;

		if (fse->code != imx911_get_format_code(imx911))
			return -EINVAL;

		fse->min_width = mode_list[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = mode_list[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = IMX911_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = IMX911_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void imx911_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx911_update_image_pad_format(struct imx911 *imx911,
					   const struct imx911_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx911_reset_colorspace(&fmt->format);
}

static void imx911_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = IMX911_EMBEDDED_LINE_WIDTH;
	fmt->format.height = IMX911_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int imx911_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx911 *imx911 = to_imx911(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx911->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&imx911->sd, sd_state,
						   fmt->pad);
		/* update the code which could change due to vflip or hflip */
		try_fmt->code = fmt->pad == IMAGE_PAD ?
				imx911_get_format_code(imx911) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			imx911_update_image_pad_format(imx911, imx911->mode,
						       fmt);
			fmt->format.code = imx911_get_format_code(imx911);
		} else {
			imx911_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx911->mutex);
	return 0;
}

static int imx911_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx911_mode *mode;
	struct imx911 *imx911 = to_imx911(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx911->mutex);

	if (fmt->pad == IMAGE_PAD) {
		const struct imx911_mode *mode_list;
		unsigned int num_modes;

		/* Bayer order varies with flips */
		fmt->format.code = imx911_get_format_code(imx911);

		get_mode_table(fmt->format.code, &mode_list, &num_modes,
			       imx911->hdr_mode->val);

		mode = v4l2_find_nearest_size(mode_list,
					      num_modes,
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		imx911_update_image_pad_format(imx911, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			imx911->mode = mode;
			imx911_set_framing_limits(imx911);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			imx911_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx911->mutex);

	return 0;
}

static const struct v4l2_rect *
__imx911_get_pad_crop(struct imx911 *imx911, struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx911->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx911->mode->crop;
	}

	return NULL;
}

static int imx911_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx911 *imx911 = to_imx911(sd);

		mutex_lock(&imx911->mutex);
		sel->r = *__imx911_get_pad_crop(imx911, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&imx911->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX911_NATIVE_WIDTH;
		sel->r.height = IMX911_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX911_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX911_PIXEL_ARRAY_TOP;
		sel->r.width = IMX911_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX911_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

/* Start streaming */
static int imx911_start_streaming(struct imx911 *imx911)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx911->sd);
	const struct imx911_reg_list *reg_list, *freq_regs;
	int i, ret;
	u32 val;

	if (!imx911->common_regs_written) {
		ret = imx911_write_regs(imx911, mode_common_regs,
					ARRAY_SIZE(mode_common_regs));
		if (ret) {
			dev_err(&client->dev, "%s failed to set common settings\n",
				__func__);
			return ret;
		}

		ret = imx911_read_reg(imx911, IMX911_REG_BASE_SPC_GAINS_L,
				      IMX911_REG_VALUE_08BIT, &val);
		if (ret == 0 && val == 0x40) {
			for (i = 0; i < 54 && ret == 0; i++) {
				ret = imx911_write_reg(imx911,
						       IMX911_REG_BASE_SPC_GAINS_L + i,
						       IMX911_REG_VALUE_08BIT,
						       pdaf_gains[0][i % 9]);
			}
			for (i = 0; i < 54 && ret == 0; i++) {
				ret = imx911_write_reg(imx911,
						       IMX911_REG_BASE_SPC_GAINS_R + i,
						       IMX911_REG_VALUE_08BIT,
						       pdaf_gains[1][i % 9]);
			}
		}
		if (ret) {
			dev_err(&client->dev, "%s failed to set PDAF gains\n",
				__func__);
			return ret;
		}

		imx911->common_regs_written = true;
	}

	/* Apply default values of current mode */
	reg_list = &imx911->mode->reg_list;
	ret = imx911_write_regs(imx911, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Update the link frequency registers */
	freq_regs = &link_freq_regs[imx911->link_freq_idx];
	ret = imx911_write_regs(imx911, freq_regs->regs,
				freq_regs->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set link frequency registers\n",
			__func__);
		return ret;
	}

	/* Quad Bayer re-mosaic adjustments (for full-resolution mode only) */
	if (imx911->mode->remosaic && qbc_adjust > 0) {
		imx911_write_reg(imx911, IMX911_LPF_INTENSITY,
				 IMX911_REG_VALUE_08BIT, qbc_adjust);
		imx911_write_reg(imx911,
				 IMX911_LPF_INTENSITY_EN,
				 IMX911_REG_VALUE_08BIT,
				 IMX911_LPF_INTENSITY_ENABLED);
	} else {
		imx911_write_reg(imx911,
				 IMX911_LPF_INTENSITY_EN,
				 IMX911_REG_VALUE_08BIT,
				 IMX911_LPF_INTENSITY_DISABLED);
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx911->sd.ctrl_handler);
	if (ret)
		return ret;

	/* set stream on register */
	return imx911_write_reg(imx911, IMX911_REG_MODE_SELECT,
				IMX911_REG_VALUE_08BIT, IMX911_MODE_STREAMING);
}

/* Stop streaming */
static void imx911_stop_streaming(struct imx911 *imx911)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx911->sd);
	int ret;

	/* set stream off register */
	ret = imx911_write_reg(imx911, IMX911_REG_MODE_SELECT,
			       IMX911_REG_VALUE_08BIT, IMX911_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);
}

static int imx911_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx911 *imx911 = to_imx911(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx911->mutex);
	if (imx911->streaming == enable) {
		mutex_unlock(&imx911->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx911_start_streaming(imx911);
		if (ret)
			goto err_rpm_put;
	} else {
		imx911_stop_streaming(imx911);
		pm_runtime_put(&client->dev);
	}

	imx911->streaming = enable;

	/* vflip/hflip and hdr mode cannot change during streaming */
	__v4l2_ctrl_grab(imx911->vflip, enable);
	__v4l2_ctrl_grab(imx911->hflip, enable);
	__v4l2_ctrl_grab(imx911->hdr_mode, enable);

	mutex_unlock(&imx911->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx911->mutex);

	return ret;
}

/* Power/clock management functions */
static int imx911_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx911 *imx911 = to_imx911(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(imx911_supply_name),
				    imx911->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx911->inclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx911->reset_gpio, 1);
	usleep_range(IMX911_XCLR_MIN_DELAY_US,
		     IMX911_XCLR_MIN_DELAY_US + IMX911_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(ARRAY_SIZE(imx911_supply_name),
			       imx911->supplies);
	return ret;
}

static int imx911_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx911 *imx911 = to_imx911(sd);

	gpiod_set_value_cansleep(imx911->reset_gpio, 0);
	regulator_bulk_disable(ARRAY_SIZE(imx911_supply_name),
			       imx911->supplies);
	clk_disable_unprepare(imx911->inclk);

	/* Force reprogramming of the common registers when powered up again. */
	imx911->common_regs_written = false;

	return 0;
}

static int __maybe_unused imx911_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx911 *imx911 = to_imx911(sd);

	if (imx911->streaming)
		imx911_stop_streaming(imx911);

	return 0;
}

static int __maybe_unused imx911_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx911 *imx911 = to_imx911(sd);
	int ret;

	if (imx911->streaming) {
		ret = imx911_start_streaming(imx911);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx911_stop_streaming(imx911);
	imx911->streaming = 0;
	return ret;
}

static int imx911_get_regulators(struct imx911 *imx911)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx911->sd);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx911_supply_name); i++)
		imx911->supplies[i].supply = imx911_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       ARRAY_SIZE(imx911_supply_name),
				       imx911->supplies);
}

/* Verify chip ID */
static int imx911_identify_module(struct imx911 *imx911)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx911->sd);
	int ret;
	u32 val;

	ret = imx911_read_reg(imx911, IMX911_REG_CHIP_ID,
			      IMX911_REG_VALUE_16BIT, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x, with error %d\n",
			IMX911_CHIP_ID, ret);
		return ret;
	}

	if (val != IMX911_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			IMX911_CHIP_ID, val);
		return -EIO;
	}

	ret = imx911_read_reg(imx911, 0x0000, IMX911_REG_VALUE_16BIT, &val);
	if (!ret) {
		dev_info(&client->dev, "camera module ID 0x%04x\n", val);
		snprintf(imx911->sd.name, sizeof(imx911->sd.name), "imx911%s%s",
			 val & 0x02 ? "_wide" : "",
			 val & 0x80 ? "_noir" : "");
	}

	return 0;
}

static const struct v4l2_subdev_core_ops imx911_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx911_video_ops = {
	.s_stream = imx911_set_stream,
};

static const struct v4l2_subdev_pad_ops imx911_pad_ops = {
	.enum_mbus_code = imx911_enum_mbus_code,
	.get_fmt = imx911_get_pad_format,
	.set_fmt = imx911_set_pad_format,
	.get_selection = imx911_get_selection,
	.enum_frame_size = imx911_enum_frame_size,
};

static const struct v4l2_subdev_ops imx911_subdev_ops = {
	.core = &imx911_core_ops,
	.video = &imx911_video_ops,
	.pad = &imx911_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx911_internal_ops = {
	.open = imx911_open,
};

static const struct v4l2_ctrl_config imx911_notify_gains_ctrl = {
	.ops = &imx911_ctrl_ops,
	.id = V4L2_CID_NOTIFY_GAINS,
	.type = V4L2_CTRL_TYPE_U32,
	.min = IMX911_COLOUR_BALANCE_MIN,
	.max = IMX911_COLOUR_BALANCE_MAX,
	.step = IMX911_COLOUR_BALANCE_STEP,
	.def = IMX911_COLOUR_BALANCE_DEFAULT,
	.dims = { 4 },
	.elem_size = sizeof(u32),
};

/* Initialize control handlers */
static int imx911_init_controls(struct imx911 *imx911)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx911->sd);
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	unsigned int i;
	int ret;

	ctrl_hdlr = &imx911->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&imx911->mutex);
	ctrl_hdlr->lock = &imx911->mutex;

	/* By default, PIXEL_RATE is read only */
	imx911->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx911_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX911_INITIAL_PIXEL_RATE,
					       IMX911_INITIAL_PIXEL_RATE, 1,
					       IMX911_INITIAL_PIXEL_RATE);

	ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx911_ctrl_ops,
				      V4L2_CID_LINK_FREQ, 0, 0,
				      &link_freqs[imx911->link_freq_idx]);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx911_set_framing_limits() call below.
	 */
	imx911->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx911_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xffff, 1, 0);
	imx911->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx911_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	imx911->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx911_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX911_EXPOSURE_MIN,
					     IMX911_EXPOSURE_MAX,
					     IMX911_EXPOSURE_STEP,
					     IMX911_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx911_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX911_ANA_GAIN_MIN, IMX911_ANA_GAIN_MAX,
			  IMX911_ANA_GAIN_STEP, IMX911_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx911_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX911_DGTL_GAIN_MIN, IMX911_DGTL_GAIN_MAX,
			  IMX911_DGTL_GAIN_STEP, IMX911_DGTL_GAIN_DEFAULT);

	imx911->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx911_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);

	imx911->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx911_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_cluster(2, &imx911->hflip);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx911_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx911_test_pattern_menu) - 1,
				     0, 0, imx911_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &imx911_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  IMX911_TEST_PATTERN_COLOUR_MIN,
				  IMX911_TEST_PATTERN_COLOUR_MAX,
				  IMX911_TEST_PATTERN_COLOUR_STEP,
				  IMX911_TEST_PATTERN_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}

	v4l2_ctrl_new_custom(ctrl_hdlr, &imx911_notify_gains_ctrl, NULL);

	imx911->hdr_mode = v4l2_ctrl_new_std(ctrl_hdlr, &imx911_ctrl_ops,
					     V4L2_CID_WIDE_DYNAMIC_RANGE,
					     0, 1, 1, 0);

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx911_ctrl_ops, &props);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	imx911->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	imx911->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	imx911->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	imx911->hdr_mode->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx911->sd.ctrl_handler = ctrl_hdlr;

	/* Setup exposure and frame/line length limits. */
	imx911_set_framing_limits(imx911);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx911->mutex);

	return ret;
}

static void imx911_free_controls(struct imx911 *imx911)
{
	v4l2_ctrl_handler_free(imx911->sd.ctrl_handler);
	mutex_destroy(&imx911->mutex);
}

static int imx911_check_hwcfg(struct device *dev, struct imx911 *imx911)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;
	int i;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	for (i = 0; i < ARRAY_SIZE(link_freqs); i++) {
		if (link_freqs[i] == ep_cfg.link_frequencies[0]) {
			imx911->link_freq_idx = i;
			break;
		}
	}

	if (i == ARRAY_SIZE(link_freqs)) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
			ret = -EINVAL;
			goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int imx911_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx911 *imx911;
	int ret;

	imx911 = devm_kzalloc(&client->dev, sizeof(*imx911), GFP_KERNEL);
	if (!imx911)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx911->sd, client, &imx911_subdev_ops);

#if 0  // no powermgt no clock stuff
	/* Check the hardware configuration in device tree */
	if (imx911_check_hwcfg(dev, imx911))
		return -EINVAL;

	/* Get system clock (inclk) */
	imx911->inclk = devm_clk_get(dev, "inclk");
	if (IS_ERR(imx911->inclk))
		return dev_err_probe(dev, PTR_ERR(imx911->inclk),
				     "failed to get inclk\n");

	imx911->inclk_freq = clk_get_rate(imx911->inclk);
	if (imx911->inclk_freq != IMX911_INCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "inclk frequency not supported: %d Hz\n",
				     imx911->inclk_freq);

	ret = imx911_get_regulators(imx911);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	/* Request optional enable pin */
	imx911->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for imx911_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx911_power_on(dev);
	if (ret)
		return ret;

	ret = imx911_identify_module(imx911);
	if (ret)
		goto error_power_off;
#endif

	/* Initialize default format */
	imx911_set_default_format(imx911);

#if 0 // no pm no clocks
	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
#endif

	/* This needs the pm runtime to be registered. */
	ret = imx911_init_controls(imx911);
	if (ret)
		goto error_pm_runtime;

	/* Initialize subdev */
	imx911->sd.internal_ops = &imx911_internal_ops;
	imx911->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	imx911->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx911->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	imx911->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx911->sd.entity, NUM_PADS, imx911->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx911->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&imx911->sd.entity);

error_handler_free:
	imx911_free_controls(imx911);

error_pm_runtime:
	///pm_runtime_disable(&client->dev);
	///pm_runtime_set_suspended(&client->dev);

error_power_off:
	///imx911_power_off(&client->dev);

	return ret;
}

static void imx911_remove(struct i2c_client *client)
{
#if 0
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx911 *imx911 = to_imx911(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx911_free_controls(imx911);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx911_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
#endif
}

//static const struct of_device_id imx911_dt_ids[] = {
//	{ .compatible = "sony,imx911" },
//	{ /* sentinel */ }
//};

//MODULE_DEVICE_TABLE(of, imx911_dt_ids);

//static const struct dev_pm_ops imx911_pm_ops = {
//	SET_SYSTEM_SLEEP_PM_OPS(imx911_suspend, imx911_resume)
//	SET_RUNTIME_PM_OPS(imx911_power_off, imx911_power_on, NULL)
//};

//static struct i2c_driver imx911_i2c_driver = {
//	.driver = {
//		.name = "imx911",
//		.of_match_table	= imx911_dt_ids,
//		.pm = &imx911_pm_ops,
//	},
//	.probe = imx911_probe,
//	.remove = imx911_remove,
//};

///module_i2c_driver(imx911_i2c_driver);

///MODULE_AUTHOR("David Plowman <david.plowman@raspberrypi.com>");
///MODULE_DESCRIPTION("Sony IMX708 sensor driver");
///MODULE_LICENSE("GPL v2");
