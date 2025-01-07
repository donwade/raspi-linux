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

/* Default initial pixel rate, will get updated for each mode. */
#define IMX708_INITIAL_PIXEL_RATE	590000000

/* V_TIMING internal */
#define IMX708_FRAME_LENGTH_MAX		0xffff

/* Long exposure multiplier */
#define IMX708_LONG_EXP_SHIFT_MAX	7

/* Exposure control */
#define IMX708_EXPOSURE_OFFSET		48
#define IMX708_EXPOSURE_DEFAULT		0x640
#define IMX708_EXPOSURE_STEP		1
#define IMX708_EXPOSURE_MIN		1
#define IMX708_EXPOSURE_MAX		(IMX708_FRAME_LENGTH_MAX - \
					 IMX708_EXPOSURE_OFFSET)

/* Analog gain control */
#define IMX708_ANA_GAIN_MIN		112
#define IMX708_ANA_GAIN_MAX		960
#define IMX708_ANA_GAIN_STEP		1
#define IMX708_ANA_GAIN_DEFAULT	   IMX708_ANA_GAIN_MIN

/* Digital gain control */
#define IMX708_DGTL_GAIN_MIN		0x0100
#define IMX708_DGTL_GAIN_MAX		0xffff
#define IMX708_DGTL_GAIN_DEFAULT	0x0100
#define IMX708_DGTL_GAIN_STEP		1

/* Colour balance controls */
#define IMX708_COLOUR_BALANCE_MIN	0x01
#define IMX708_COLOUR_BALANCE_MAX	0xffff
#define IMX708_COLOUR_BALANCE_STEP	0x01
#define IMX708_COLOUR_BALANCE_DEFAULT	0x100

/* Test pattern colour components */
#define IMX708_TEST_PATTERN_COLOUR_MIN	0
#define IMX708_TEST_PATTERN_COLOUR_MAX	0x0fff
#define IMX708_TEST_PATTERN_COLOUR_STEP	1

/* HDR exposure ratio (long:med == med:short) */
#define IMX708_HDR_EXPOSURE_RATIO       4
#
/* QBC Re-mosaic broken line correction registers */
#define IMX708_LPF_INTENSITY_EN		0xC428

/*
 * Metadata buffer holds a variety of data, all sent with the same VC/DT (0x12).
 * It comprises two scanlines (of up to 5760 bytes each, for 4608 pixels)
 * of embedded data, one line of PDAF data, and two lines of AE-HIST data
 * (AE histograms are valid for HDR mode and empty in non-HDR modes).
 */
#define IMX708_EMBEDDED_LINE_WIDTH (5 * 5760)
#define IMX708_NUM_EMBEDDED_LINES 1

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

/* IMX708 native and active pixel array size. */
#define IMX708_NATIVE_WIDTH		    4640U
#define IMX708_NATIVE_HEIGHT		2658U
#define IMX708_PIXEL_ARRAY_LEFT		16U
#define IMX708_PIXEL_ARRAY_TOP		24U
#define IMX708_PIXEL_ARRAY_WIDTH	4608U
#define IMX708_PIXEL_ARRAY_HEIGHT	2592U

struct imx708_reg {
	u16 address;
	u8 val;
};

struct imx708_reg_list {
	unsigned int num_of_regs;
	const struct imx708_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx708_mode {
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

/* 10-bit. */
static const struct imx708_reg mode_4608x2592_regs[] = {
	{0x0342, 0x3D},
};

static const struct imx708_reg mode_2x2binned_regs[] = {
	{0x0342, 0x1E},
};

static const struct imx708_reg mode_2x2binned_720p_regs[] = {
	{0x0342, 0x14},
};

static const struct imx708_reg mode_hdr_regs[] = {
	{0x0342, 0x14},
};

/* Mode configs. Keep separate lists for when HDR is enabled or not. */
static const struct imx708_mode supported_modes_10bit_no_hdr[] = {
	{
		/* Full resolution. */
		.width = 4608,
		.height = 2592,
		.line_length_pix = 0x3d20,
		.crop = {
			.left = IMX708_PIXEL_ARRAY_LEFT,
			.top = IMX708_PIXEL_ARRAY_TOP,
			.width = 4608,
			.height = 2592,
		},
		.vblank_min = 58,
		.vblank_default = 58,
		.pixel_rate = 595200000,
		.exposure_lines_min = 8,
		.exposure_lines_step = 1,
		.hdr = false,
	},
	{
		/* regular 2x2 binned. */
		.width = 2304,
		.height = 1296,
		.line_length_pix = 0x1e90,
		.crop = {
			.left = IMX708_PIXEL_ARRAY_LEFT,
			.top = IMX708_PIXEL_ARRAY_TOP,
			.width = 4608,
			.height = 2592,
		},
		.vblank_min = 40,
		.vblank_default = 1198,
		.pixel_rate = 585600000,
		.exposure_lines_min = 4,
		.exposure_lines_step = 2,
		.hdr = false,
	},
	{
		/* 2x2 binned and cropped for 720p. */
		.width = 1536,
		.height = 864,
		.line_length_pix = 0x1460,
		.crop = {
			.left = IMX708_PIXEL_ARRAY_LEFT + 768,
			.top = IMX708_PIXEL_ARRAY_TOP + 432,
			.width = 3072,
			.height = 1728,
		},
		.vblank_min = 40,
		.vblank_default = 2755,
		.pixel_rate = 566400000,
		.exposure_lines_min = 4,
		.exposure_lines_step = 2,
		.hdr = false,
	},
};

static const struct imx708_mode supported_modes_10bit_hdr[] = {
	{
		/* There's only one HDR mode, which is 2x2 downscaled */
		.width = 2304,
		.height = 1296,
		.line_length_pix = 0x1460,
		.crop = {
			.left = IMX708_PIXEL_ARRAY_LEFT,
			.top = IMX708_PIXEL_ARRAY_TOP,
			.width = 4608,
			.height = 2592,
		},
		.vblank_min = 3673,
		.vblank_default = 3673,
		.pixel_rate = 777600000,
		.exposure_lines_min = 8 * IMX708_HDR_EXPOSURE_RATIO * IMX708_HDR_EXPOSURE_RATIO,
		.exposure_lines_step = 2 * IMX708_HDR_EXPOSURE_RATIO * IMX708_HDR_EXPOSURE_RATIO,
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

struct imx708 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	struct v4l2_mbus_framefmt fmt;

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
	const struct imx708_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

};

static inline struct imx708 *to_imx708(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx708, sd);
}

static inline void get_mode_table(unsigned int code,
				  const struct imx708_mode **mode_list,
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

/* Get bayer order based on flip setting. */
static u32 imx708_get_format_code(struct imx708 *imx708)
{
	unsigned int i;

	lockdep_assert_held(&imx708->mutex);

	i = (imx708->vflip->val ? 2 : 0) |
	    (imx708->hflip->val ? 1 : 0);

	return codes[i];
}

static void imx708_set_default_format(struct imx708 *imx708)
{
	struct v4l2_mbus_framefmt *fmt = &imx708->fmt;

	/* Set default mode to max resolution */
	imx708->mode = &supported_modes_10bit_no_hdr[0];

	/* fmt->code not set as it will always be computed based on flips */
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = imx708->mode->width;
	fmt->height = imx708->mode->height;
	fmt->field = V4L2_FIELD_NONE;
}

static int imx708_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx708 *imx708 = to_imx708(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&imx708->mutex);

	/* Initialize try_fmt for the image pad */
	if (imx708->hdr_mode->val) {
		try_fmt_img->width = supported_modes_10bit_hdr[0].width;
		try_fmt_img->height = supported_modes_10bit_hdr[0].height;
	} else {
		try_fmt_img->width = supported_modes_10bit_no_hdr[0].width;
		try_fmt_img->height = supported_modes_10bit_no_hdr[0].height;
	}
	try_fmt_img->code = imx708_get_format_code(imx708);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = IMX708_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = IMX708_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, IMAGE_PAD);
	try_crop->left = IMX708_PIXEL_ARRAY_LEFT;
	try_crop->top = IMX708_PIXEL_ARRAY_TOP;
	try_crop->width = IMX708_PIXEL_ARRAY_WIDTH;
	try_crop->height = IMX708_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx708->mutex);

	return 0;
}

static int imx708_set_exposure(struct imx708 *imx708, unsigned int val)
{
	val = max(val, imx708->mode->exposure_lines_min);
	val -= val % imx708->mode->exposure_lines_step;
    return 0;
}

static void imx708_adjust_exposure_range(struct imx708 *imx708,
					 struct v4l2_ctrl *ctrl)
{
	int exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure. */
	exposure_max = imx708->mode->height + imx708->vblank->val -
		IMX708_EXPOSURE_OFFSET;
	exposure_def = min(exposure_max, imx708->exposure->val);
	__v4l2_ctrl_modify_range(imx708->exposure, imx708->exposure->minimum,
				 exposure_max, imx708->exposure->step,
				 exposure_def);
}

static int imx708_set_analogue_gain(struct imx708 *imx708, unsigned int val)
{
	return 0;
}

static int imx708_set_frame_length(struct imx708 *imx708, unsigned int val)
{
    return 0;
}

static void imx708_set_framing_limits(struct imx708 *imx708)
{
	const struct imx708_mode *mode = imx708->mode;
	unsigned int hblank;

	__v4l2_ctrl_modify_range(imx708->pixel_rate,
				 mode->pixel_rate, mode->pixel_rate,
				 1, mode->pixel_rate);

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(imx708->vblank, mode->vblank_min,
				 ((1 << IMX708_LONG_EXP_SHIFT_MAX) *
					IMX708_FRAME_LENGTH_MAX) - mode->height,
				 1, mode->vblank_default);

	/*
	 * Currently PPL is fixed to the mode specified value, so hblank
	 * depends on mode->width only, and is not changeable in any
	 * way other than changing the mode.
	 */
	hblank = mode->line_length_pix - mode->width;
	__v4l2_ctrl_modify_range(imx708->hblank, hblank, hblank, 1, hblank);
}

static int imx708_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx708 *imx708 =
		container_of(ctrl->handler, struct imx708, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
	const struct imx708_mode *mode_list;
	unsigned int code, num_modes;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/*
		 * The VBLANK control may change the limits of usable exposure,
		 * so check and adjust if necessary.
		 */
		imx708_adjust_exposure_range(imx708, ctrl);
		break;

	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		/*
		 * The WIDE_DYNAMIC_RANGE control can also be applied immediately
		 * as it doesn't set any registers. Don't do anything if the mode
		 * already matches.
		 */
		if (imx708->mode && imx708->mode->hdr != ctrl->val) {
			code = imx708_get_format_code(imx708);
			get_mode_table(code, &mode_list, &num_modes, ctrl->val);
			imx708->mode = v4l2_find_nearest_size(mode_list,
							      num_modes,
							      width, height,
							      imx708->mode->width,
							      imx708->mode->height);
			imx708_set_framing_limits(imx708);
		}
		break;
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
    ret = 0;
	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		dev_info(&client->dev,
			 "ctrl V4L2_CID_DIGITAL_GAIN (id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
        break;
	case V4L2_CID_EXPOSURE:
		dev_info(&client->dev,
			 "ctrl V4L2_CID_DIGITAL_GAIN (id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		dev_info(&client->dev,
			 "ctrl V4L2_CID_DIGITAL_GAIN (id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		dev_info(&client->dev,
			 "ctrl V4L2_CID_TEST_PATTERN (id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		dev_info(&client->dev,
			 "ctrl V4L2_CID_TEST_PATTERN_RED (id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		dev_info(&client->dev,
			 "ctrl V4L2_CID_TEST_PATTERN_BLUE (id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		dev_info(&client->dev,
			 "ctrl V4L2_CID_TEST_PATTERN_GREENB (id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		dev_info(&client->dev,
			 "ctrl V4L2_CID_H/VFLIP (id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_info(&client->dev,
			 "ctrl V4L2_CID_VBLANK (id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	case V4L2_CID_NOTIFY_GAINS:
		dev_info(&client->dev,
			 "ctrl V4L2_CID_NOTIFY_GAINS (id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		dev_info(&client->dev,
			 "ctrl V4L2_CID_WIDE_DYNAMIC_RANGE (id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops imx708_ctrl_ops = {
	.s_ctrl = imx708_set_ctrl,
};

static int imx708_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx708 *imx708 = to_imx708(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index >= (ARRAY_SIZE(codes) / 4))
			return -EINVAL;

		code->code = imx708_get_format_code(imx708);
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int imx708_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx708 *imx708 = to_imx708(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		const struct imx708_mode *mode_list;
		unsigned int num_modes;

		get_mode_table(fse->code, &mode_list, &num_modes,
			       imx708->hdr_mode->val);

		if (fse->index >= num_modes)
			return -EINVAL;

		if (fse->code != imx708_get_format_code(imx708))
			return -EINVAL;

		fse->min_width = mode_list[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = mode_list[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = IMX708_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = IMX708_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void imx708_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx708_update_image_pad_format(struct imx708 *imx708,
					   const struct imx708_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx708_reset_colorspace(&fmt->format);
}

static void imx708_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = IMX708_EMBEDDED_LINE_WIDTH;
	fmt->format.height = IMX708_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int imx708_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx708 *imx708 = to_imx708(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx708->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&imx708->sd, sd_state,
						   fmt->pad);
		/* update the code which could change due to vflip or hflip */
		try_fmt->code = fmt->pad == IMAGE_PAD ?
				imx708_get_format_code(imx708) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			imx708_update_image_pad_format(imx708, imx708->mode,
						       fmt);
			fmt->format.code = imx708_get_format_code(imx708);
		} else {
			imx708_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx708->mutex);
	return 0;
}

static int imx708_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx708_mode *mode;
	struct imx708 *imx708 = to_imx708(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx708->mutex);

	if (fmt->pad == IMAGE_PAD) {
		const struct imx708_mode *mode_list;
		unsigned int num_modes;

		/* Bayer order varies with flips */
		fmt->format.code = imx708_get_format_code(imx708);

		get_mode_table(fmt->format.code, &mode_list, &num_modes,
			       imx708->hdr_mode->val);

		mode = v4l2_find_nearest_size(mode_list,
					      num_modes,
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		imx708_update_image_pad_format(imx708, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			imx708->mode = mode;
			imx708_set_framing_limits(imx708);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			imx708_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx708->mutex);

	return 0;
}

static const struct v4l2_rect *
__imx708_get_pad_crop(struct imx708 *imx708, struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx708->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx708->mode->crop;
	}

	return NULL;
}

static int imx708_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx708 *imx708 = to_imx708(sd);

		mutex_lock(&imx708->mutex);
		sel->r = *__imx708_get_pad_crop(imx708, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&imx708->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX708_NATIVE_WIDTH;
		sel->r.height = IMX708_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX708_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX708_PIXEL_ARRAY_TOP;
		sel->r.width = IMX708_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX708_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

/* Start streaming */
static int imx708_start_streaming(struct imx708 *imx708)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
	const struct imx708_reg_list *reg_list;
	int ret;

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx708->sd.ctrl_handler);
    return ret;
}

/* Stop streaming */
static void imx708_stop_streaming(struct imx708 *imx708)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    dev_err(&client->dev, "%s failed to set stream\n", __func__);
}

static int imx708_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx708 *imx708 = to_imx708(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx708->mutex);
	if (imx708->streaming == enable) {
		mutex_unlock(&imx708->mutex);
		return 0;
	}

	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx708_start_streaming(imx708);
	} else {
		imx708_stop_streaming(imx708);
	}

	imx708->streaming = enable;

	/* vflip/hflip and hdr mode cannot change during streaming */
	__v4l2_ctrl_grab(imx708->vflip, enable);
	__v4l2_ctrl_grab(imx708->hflip, enable);
	__v4l2_ctrl_grab(imx708->hdr_mode, enable);

	mutex_unlock(&imx708->mutex);

	return ret;

}

/* Power/clock management functions */
static int imx708_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx708 *imx708 = to_imx708(sd);
	int ret;

	//usleep_range(IMX708_XCLR_MIN_DELAY_US, IMX708_XCLR_MIN_DELAY_US + IMX708_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	return ret;
}

static int imx708_get_regulators(struct imx708 *imx708)
{
	//struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    return 0;
}

static const struct v4l2_subdev_core_ops imx708_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx708_video_ops = {
	.s_stream = imx708_set_stream,
};

static const struct v4l2_subdev_pad_ops imx708_pad_ops = {
	.enum_mbus_code = imx708_enum_mbus_code,
	.get_fmt = imx708_get_pad_format,
	.set_fmt = imx708_set_pad_format,
	.get_selection = imx708_get_selection,
	.enum_frame_size = imx708_enum_frame_size,
};

static const struct v4l2_subdev_ops imx708_subdev_ops = {
	.core = &imx708_core_ops,
	.video = &imx708_video_ops,
	.pad = &imx708_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx708_internal_ops = {
	.open = imx708_open,
};

static const struct v4l2_ctrl_config imx708_notify_gains_ctrl = {
	.ops = &imx708_ctrl_ops,
	.id = V4L2_CID_NOTIFY_GAINS,
	.type = V4L2_CTRL_TYPE_U32,
	.min = IMX708_COLOUR_BALANCE_MIN,
	.max = IMX708_COLOUR_BALANCE_MAX,
	.step = IMX708_COLOUR_BALANCE_STEP,
	.def = IMX708_COLOUR_BALANCE_DEFAULT,
	.dims = { 4 },
	.elem_size = sizeof(u32),
};

/* Initialize control handlers */
static int imx708_init_controls(struct imx708 *imx708)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
	struct v4l2_fwnode_device_properties props;
	unsigned int i;
	int ret;

	ctrl_hdlr = &imx708->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 14);
	if (ret)
		return ret;

	mutex_init(&imx708->mutex);
	ctrl_hdlr->lock = &imx708->mutex;

	/* By default, PIXEL_RATE is read only */
	imx708->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX708_INITIAL_PIXEL_RATE,
					       IMX708_INITIAL_PIXEL_RATE, 1,
					       IMX708_INITIAL_PIXEL_RATE);
	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx708_set_framing_limits() call below.
	 */
	imx708->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xffff, 1, 0);
	imx708->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	imx708->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX708_EXPOSURE_MIN,
					     IMX708_EXPOSURE_MAX,
					     IMX708_EXPOSURE_STEP,
					     IMX708_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX708_ANA_GAIN_MIN, IMX708_ANA_GAIN_MAX,
			  IMX708_ANA_GAIN_STEP, IMX708_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX708_DGTL_GAIN_MIN, IMX708_DGTL_GAIN_MAX,
			  IMX708_DGTL_GAIN_STEP, IMX708_DGTL_GAIN_DEFAULT);

	imx708->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);

	imx708->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_cluster(2, &imx708->hflip);

	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  IMX708_TEST_PATTERN_COLOUR_MIN,
				  IMX708_TEST_PATTERN_COLOUR_MAX,
				  IMX708_TEST_PATTERN_COLOUR_STEP,
				  IMX708_TEST_PATTERN_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}

	v4l2_ctrl_new_custom(ctrl_hdlr, &imx708_notify_gains_ctrl, NULL);

	imx708->hdr_mode = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					     V4L2_CID_WIDE_DYNAMIC_RANGE,
					     0, 1, 1, 0);

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx708_ctrl_ops, &props);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	imx708->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	imx708->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	imx708->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	imx708->hdr_mode->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx708->sd.ctrl_handler = ctrl_hdlr;

	/* Setup exposure and frame/line length limits. */
	imx708_set_framing_limits(imx708);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx708->mutex);

	return ret;
}

static void imx708_free_controls(struct imx708 *imx708)
{
	v4l2_ctrl_handler_free(imx708->sd.ctrl_handler);
	mutex_destroy(&imx708->mutex);
}

static int imx708_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx708 *imx708;
	int ret;

	imx708 = devm_kzalloc(&client->dev, sizeof(*imx708), GFP_KERNEL);
	if (!imx708)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx708->sd, client, &imx708_subdev_ops);

	ret = imx708_get_regulators(imx708);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	/*
	 * The sensor must be powered for imx708_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx708_power_on(dev);
	if (ret)
		return ret;

	/* Initialize default format */
	imx708_set_default_format(imx708);

	imx708_init_controls(imx708);

	/* Initialize subdev */
	imx708->sd.internal_ops = &imx708_internal_ops;
	imx708->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx708->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx708->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	imx708->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx708->sd.entity, NUM_PADS, imx708->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx708->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&imx708->sd.entity);

error_handler_free:
	imx708_free_controls(imx708);

	return ret;
}

static void imx708_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx708 *imx708 = to_imx708(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx708_free_controls(imx708);

}

static const struct of_device_id imx708_dt_ids[] = {
	{ .compatible = "sony,imx708" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx708_dt_ids);

static struct i2c_driver imx708_i2c_driver = {
	.driver = {
		.name = "imx708",
		.of_match_table	= imx708_dt_ids,
	},
	.probe = imx708_probe,
	.remove = imx708_remove,
};

module_i2c_driver(imx708_i2c_driver);

MODULE_AUTHOR("David Plowman <david.plowman@raspberrypi.com>");
MODULE_DESCRIPTION("Sony IMX708 sensor driver");
MODULE_LICENSE("GPL v2");
