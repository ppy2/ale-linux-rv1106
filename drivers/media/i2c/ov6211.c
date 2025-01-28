// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 subdevice driver for OmniVision OV6211 Camera Sensor
 *
 * Copyright (C) 2024 Huy Duong <huy.duong@ologn.tech>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define OV6211_DEFAULT_READ_I2C_ADDR 0x21

/* System Control */
#define OV6211_SC_MODE_SELECT 0x0100
#define OV6211_SC_SOFTWARE_RESET 0x0103
#define OV6211_SC_CHIP_ID_HIGH 0x300a
#define OV6211_SC_CHIP_ID_LOW 0x300b
#define OV6211_SC_REG0C 0x300c

/* AEC/AGC */
#define OV6211_AEC_EXPO1 0x3500
#define OV6211_AEC_EXPO2 0x3501
#define OV6211_AEC_EXPO3 0x3502
#define OV6211_AEC_MANUAL 0x3503

/* Timing Conrol Registers */
#define OV6211_TVTS_HI 0x380e
#define OV6211_TVTS_LO 0x380f

/* Strobe Frame Span Registers */
#define OV6211_STROBE_SPAN1 0x3b8d
#define OV6211_STROBE_SPAN2 0x3b8e
#define OV6211_STROBE_SPAN3 0x3b8f

#define OV6211_LAST_REG 0x5e08

#define DEF_LINK_FREQ 38400000LL

enum ov6211_mode_id {
	OV6211_MODE_Y8_400_200 = 0,
	OV6211_MODE_Y8_400_400,
	OV6211_NUM_MODES,
};

struct ov6211_pixfmt {
	u32 code;
	u32 colorspace;
};

static const struct ov6211_pixfmt ov6211_formats[] = {
	{
		MEDIA_BUS_FMT_Y8_1X8,
		V4L2_COLORSPACE_RAW,
	},
	{
		MEDIA_BUS_FMT_Y8_1X8,
		V4L2_COLORSPACE_RAW,
	},
};

enum ov6211_framerate_ids {
	OV6211_10_FPS = 0,
	OV6211_15_FPS,
	OV6211_30_FPS,
	OV6211_45_FPS,
	OV6211_60_FPS,
	OV6211_NUM_FRAMERATES,
};

static const int ov6211_framerates[] = {
	[OV6211_10_FPS] = 10, [OV6211_15_FPS] = 15, [OV6211_30_FPS] = 30,
	[OV6211_45_FPS] = 45, [OV6211_60_FPS] = 60,
};

/* regulator supplies */
static const char *const ov6211_supply_name[] = {
	"dovdd",
	"avdd",
};

static const struct regmap_config ov6211_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = OV6211_LAST_REG,
	.cache_type = REGCACHE_NONE,
};

struct reg_value {
	u16 reg_addr;
	u8 val;
	u8 mask;
	u32 delay_ms;
};

struct ov6211_mode_info {
	enum ov6211_mode_id id;
	u32 width;
	u32 height;
	const struct reg_value *reg_data;
	u32 reg_data_size;
	u32 pixel_clock;
};

struct ov6211_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
	};
	struct {
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct v4l2_ctrl *link_freq;
};

struct ov6211_dev {
	struct i2c_client *i2c_client;
	struct i2c_client *i2c_client_read;
	struct regmap *regmap;
	struct regmap *regmap_read;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */

	struct regulator_bulk_data supplies[ARRAY_SIZE(ov6211_supply_name)];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;

	struct mutex lock;

	struct v4l2_mbus_framefmt fmt;

	const struct ov6211_mode_info *cur_mode;
	enum ov6211_framerate_ids cur_fr_id;
	struct v4l2_fract frame_interval;

	struct ov6211_ctrls ctrls;

	u32 exposure;
	bool pending_mode_change;
	bool pending_fi_change;
	bool streaming;
};

static inline struct ov6211_dev *to_ov6211_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov6211_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov6211_dev, ctrls.handler)
			->sd;
}

static const struct reg_value ov6211_init_y8_400_400[] = {
	{ 0x0103, 0x01, 0, 0 }, { 0x0100, 0x00, 0, 0 }, { 0x3005, 0x08, 0, 0 },
	{ 0x3013, 0x12, 0, 0 }, { 0x3014, 0x04, 0, 0 }, { 0x3016, 0x10, 0, 0 },
	{ 0x3017, 0x00, 0, 0 }, { 0x3018, 0x00, 0, 0 }, { 0x301a, 0x00, 0, 0 },
	{ 0x301b, 0x00, 0, 0 }, { 0x301c, 0x00, 0, 0 }, { 0x3037, 0xf0, 0, 0 },
	{ 0x3080, 0x01, 0, 0 }, { 0x3081, 0x00, 0, 0 }, { 0x3082, 0x01, 0, 0 },
	{ 0x3098, 0x04, 0, 0 }, { 0x3099, 0x28, 0, 0 }, { 0x309a, 0x06, 0, 0 },
	{ 0x309b, 0x04, 0, 0 }, { 0x309c, 0x00, 0, 0 }, { 0x309d, 0x00, 0, 0 },
	{ 0x309e, 0x01, 0, 0 }, { 0x309f, 0x00, 0, 0 }, { 0x30b0, 0x08, 0, 0 },
	{ 0x30b1, 0x02, 0, 0 }, { 0x30b2, 0x00, 0, 0 }, { 0x30b3, 0x28, 0, 0 },
	{ 0x30b4, 0x02, 0, 0 }, { 0x30b5, 0x00, 0, 0 }, { 0x3106, 0xd9, 0, 0 },
	{ 0x3500, 0x00, 0, 0 }, { 0x3501, 0x1b, 0, 0 }, { 0x3502, 0x20, 0, 0 },
	{ 0x3503, 0x07, 0, 0 }, { 0x3509, 0x10, 0, 0 }, { 0x350b, 0x10, 0, 0 },
	{ 0x3600, 0xfc, 0, 0 }, { 0x3620, 0xb7, 0, 0 }, { 0x3621, 0x05, 0, 0 },
	{ 0x3626, 0x31, 0, 0 }, { 0x3627, 0x40, 0, 0 }, { 0x3632, 0xa3, 0, 0 },
	{ 0x3633, 0x34, 0, 0 }, { 0x3634, 0x40, 0, 0 }, { 0x3636, 0x00, 0, 0 },
	{ 0x3660, 0x80, 0, 0 }, { 0x3662, 0x03, 0, 0 }, { 0x3664, 0xf0, 0, 0 },
	{ 0x366a, 0x10, 0, 0 }, { 0x366b, 0x06, 0, 0 }, { 0x3680, 0xf4, 0, 0 },
	{ 0x3681, 0x50, 0, 0 }, { 0x3682, 0x00, 0, 0 }, { 0x3708, 0x20, 0, 0 },
	{ 0x3709, 0x40, 0, 0 }, { 0x370d, 0x03, 0, 0 }, { 0x373b, 0x02, 0, 0 },
	{ 0x373c, 0x08, 0, 0 }, { 0x3742, 0x00, 0, 0 }, { 0x3744, 0x16, 0, 0 },
	{ 0x3745, 0x08, 0, 0 }, { 0x3781, 0xfc, 0, 0 }, { 0x3788, 0x00, 0, 0 },
	{ 0x3800, 0x00, 0, 0 }, { 0x3801, 0x04, 0, 0 }, { 0x3802, 0x00, 0, 0 },
	{ 0x3803, 0x04, 0, 0 }, { 0x3804, 0x01, 0, 0 }, { 0x3805, 0x9b, 0, 0 },
	{ 0x3806, 0x01, 0, 0 }, { 0x3807, 0x9b, 0, 0 }, { 0x3808, 0x01, 0, 0 },
	{ 0x3809, 0x90, 0, 0 }, { 0x380a, 0x01, 0, 0 }, { 0x380b, 0x90, 0, 0 },
	{ 0x380c, 0x05, 0, 0 }, { 0x380d, 0xf2, 0, 0 }, { 0x380e, 0x03, 0, 0 },
	{ 0x380f, 0x6c, 0, 0 }, { 0x3810, 0x00, 0, 0 }, { 0x3811, 0x04, 0, 0 },
	{ 0x3812, 0x00, 0, 0 }, { 0x3813, 0x04, 0, 0 }, { 0x3814, 0x11, 0, 0 },
	{ 0x3815, 0x11, 0, 0 }, { 0x3820, 0x00, 0, 0 }, { 0x3821, 0x00, 0, 0 },
	{ 0x382b, 0xfa, 0, 0 }, { 0x382f, 0x04, 0, 0 }, { 0x3832, 0x00, 0, 0 },
	{ 0x3833, 0x05, 0, 0 }, { 0x3834, 0x00, 0, 0 }, { 0x3835, 0x05, 0, 0 },
	{ 0x3882, 0x04, 0, 0 }, { 0x3883, 0x00, 0, 0 }, { 0x38a4, 0x10, 0, 0 },
	{ 0x38a5, 0x00, 0, 0 }, { 0x38b1, 0x03, 0, 0 }, { 0x3b80, 0x00, 0, 0 },
	{ 0x3b81, 0xff, 0, 0 }, { 0x3b82, 0x10, 0, 0 }, { 0x3b83, 0x00, 0, 0 },
	{ 0x3b84, 0x08, 0, 0 }, { 0x3b85, 0x00, 0, 0 }, { 0x3b86, 0x01, 0, 0 },
	{ 0x3b87, 0x00, 0, 0 }, { 0x3b88, 0x00, 0, 0 }, { 0x3b89, 0x00, 0, 0 },
	{ 0x3b8a, 0x00, 0, 0 }, { 0x3b8b, 0x05, 0, 0 }, { 0x3b8c, 0x00, 0, 0 },
	{ 0x3b8d, 0x00, 0, 0 }, { 0x3b8e, 0x01, 0, 0 }, { 0x3b8f, 0xb2, 0, 0 },
	{ 0x3b94, 0x05, 0, 0 }, { 0x3b95, 0xf2, 0, 0 }, { 0x3b96, 0xc0, 0, 0 },
	{ 0x4004, 0x04, 0, 0 }, { 0x404e, 0x01, 0, 0 }, { 0x4801, 0x0f, 0, 0 },
	{ 0x4806, 0x0f, 0, 0 }, { 0x4837, 0x43, 0, 0 }, { 0x5a08, 0x00, 0, 0 },
	{ 0x5a01, 0x00, 0, 0 }, { 0x5a03, 0x00, 0, 0 }, { 0x5a04, 0x10, 0, 0 },
	{ 0x5a05, 0xa0, 0, 0 }, { 0x5a06, 0x0c, 0, 0 }, { 0x5a07, 0x78, 0, 0 },
};

static const struct reg_value ov6211_init_y8_400_200[] = {
	{ 0x0103, 0x01, 0, 0 }, { 0x0100, 0x00, 0, 0 }, { 0x3005, 0x08, 0, 0 },
	{ 0x3013, 0x12, 0, 0 }, { 0x3014, 0x04, 0, 0 }, { 0x3016, 0x10, 0, 0 },
	{ 0x3017, 0x00, 0, 0 }, { 0x3018, 0x00, 0, 0 }, { 0x301a, 0x00, 0, 0 },
	{ 0x301b, 0x00, 0, 0 }, { 0x301c, 0x00, 0, 0 }, { 0x3037, 0xf0, 0, 0 },
	{ 0x3080, 0x01, 0, 0 }, { 0x3081, 0x00, 0, 0 }, { 0x3082, 0x01, 0, 0 },
	{ 0x3098, 0x04, 0, 0 }, { 0x3099, 0x28, 0, 0 }, { 0x309a, 0x06, 0, 0 },
	{ 0x309b, 0x04, 0, 0 }, { 0x309c, 0x00, 0, 0 }, { 0x309d, 0x00, 0, 0 },
	{ 0x309e, 0x01, 0, 0 }, { 0x309f, 0x00, 0, 0 }, { 0x30b0, 0x08, 0, 0 },
	{ 0x30b1, 0x02, 0, 0 }, { 0x30b2, 0x00, 0, 0 }, { 0x30b3, 0x28, 0, 0 },
	{ 0x30b4, 0x02, 0, 0 }, { 0x30b5, 0x00, 0, 0 }, { 0x3106, 0xd9, 0, 0 },
	{ 0x3500, 0x00, 0, 0 }, { 0x3501, 0x1b, 0, 0 }, { 0x3502, 0x20, 0, 0 },
	{ 0x3503, 0x07, 0, 0 }, { 0x3509, 0x10, 0, 0 }, { 0x350b, 0x10, 0, 0 },
	{ 0x3600, 0xfc, 0, 0 }, { 0x3620, 0xb7, 0, 0 }, { 0x3621, 0x05, 0, 0 },
	{ 0x3626, 0x31, 0, 0 }, { 0x3627, 0x40, 0, 0 }, { 0x3632, 0xa3, 0, 0 },
	{ 0x3633, 0x34, 0, 0 }, { 0x3634, 0x40, 0, 0 }, { 0x3636, 0x00, 0, 0 },
	{ 0x3660, 0x80, 0, 0 }, { 0x3662, 0x03, 0, 0 }, { 0x3664, 0xf0, 0, 0 },
	{ 0x366a, 0x10, 0, 0 }, { 0x366b, 0x06, 0, 0 }, { 0x3680, 0xf4, 0, 0 },
	{ 0x3681, 0x50, 0, 0 }, { 0x3682, 0x00, 0, 0 }, { 0x3708, 0x20, 0, 0 },
	{ 0x3709, 0x40, 0, 0 }, { 0x370d, 0x03, 0, 0 }, { 0x373b, 0x02, 0, 0 },
	{ 0x373c, 0x08, 0, 0 }, { 0x3742, 0x00, 0, 0 }, { 0x3744, 0x16, 0, 0 },
	{ 0x3745, 0x08, 0, 0 }, { 0x3781, 0xfc, 0, 0 }, { 0x3788, 0x00, 0, 0 },
	{ 0x3800, 0x00, 0, 0 }, { 0x3801, 0x04, 0, 0 }, { 0x3802, 0x00, 0, 0 },
	{ 0x3803, 0x04, 0, 0 }, { 0x3804, 0x01, 0, 0 }, { 0x3805, 0x9b, 0, 0 },
	{ 0x3806, 0x01, 0, 0 }, { 0x3807, 0x9b, 0, 0 }, { 0x3808, 0x01, 0, 0 },
	{ 0x3809, 0x90, 0, 0 }, { 0x380a, 0x00, 0, 0 }, { 0x380b, 0xc8, 0, 0 },
	{ 0x380c, 0x05, 0, 0 }, { 0x380d, 0xf2, 0, 0 }, { 0x380e, 0x0d, 0, 0 },
	{ 0x380f, 0xb0, 0, 0 }, { 0x3810, 0x00, 0, 0 }, { 0x3811, 0x04, 0, 0 },
	{ 0x3812, 0x00, 0, 0 }, { 0x3813, 0x9a, 0, 0 }, { 0x3814, 0x11, 0, 0 },
	{ 0x3815, 0x11, 0, 0 }, { 0x3820, 0x00, 0, 0 }, { 0x3821, 0x00, 0, 0 },
	{ 0x382b, 0xfa, 0, 0 }, { 0x382f, 0x04, 0, 0 }, { 0x3832, 0x00, 0, 0 },
	{ 0x3833, 0x05, 0, 0 }, { 0x3834, 0x00, 0, 0 }, { 0x3835, 0x05, 0, 0 },
	{ 0x3882, 0x04, 0, 0 }, { 0x3883, 0x00, 0, 0 }, { 0x38a4, 0x10, 0, 0 },
	{ 0x38a5, 0x00, 0, 0 }, { 0x38b1, 0x03, 0, 0 }, { 0x3b80, 0x00, 0, 0 },
	{ 0x3b81, 0xff, 0, 0 }, { 0x3b82, 0x10, 0, 0 }, { 0x3b83, 0x00, 0, 0 },
	{ 0x3b84, 0x08, 0, 0 }, { 0x3b85, 0x00, 0, 0 }, { 0x3b86, 0x01, 0, 0 },
	{ 0x3b87, 0x00, 0, 0 }, { 0x3b88, 0x00, 0, 0 }, { 0x3b89, 0x00, 0, 0 },
	{ 0x3b8a, 0x00, 0, 0 }, { 0x3b8b, 0x05, 0, 0 }, { 0x3b8c, 0x00, 0, 0 },
	{ 0x3b8d, 0x00, 0, 0 }, { 0x3b8e, 0x01, 0, 0 }, { 0x3b8f, 0xb2, 0, 0 },
	{ 0x3b94, 0x05, 0, 0 }, { 0x3b95, 0xf2, 0, 0 }, { 0x3b96, 0xc0, 0, 0 },
	{ 0x4004, 0x04, 0, 0 }, { 0x404e, 0x01, 0, 0 }, { 0x4801, 0x0f, 0, 0 },
	{ 0x4806, 0x0f, 0, 0 }, { 0x4837, 0x43, 0, 0 }, { 0x5a08, 0x00, 0, 0 },
	{ 0x5a01, 0x00, 0, 0 }, { 0x5a03, 0x00, 0, 0 }, { 0x5a04, 0x10, 0, 0 },
	{ 0x5a05, 0xa0, 0, 0 }, { 0x5a06, 0x0c, 0, 0 }, { 0x5a07, 0x78, 0, 0 },
};

static struct ov6211_mode_info ov6211_mode_data[OV6211_NUM_MODES] = {
	{
		OV6211_MODE_Y8_400_200,
		400,
		200,
		ov6211_init_y8_400_200,
		ARRAY_SIZE(ov6211_init_y8_400_200),
		400 * 400 * 60 * 2,
	},
	{
		OV6211_MODE_Y8_400_400,
		400,
		400,
		ov6211_init_y8_400_400,
		ARRAY_SIZE(ov6211_init_y8_400_400),
		400 * 400 * 60 * 2,
	},
};

static const s64 link_freq_menu_items[] = {
	DEF_LINK_FREQ,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov6211_get_register(struct v4l2_subdev *sd,
			       struct v4l2_dbg_register *reg)
{
	struct ov6211_dev *sensor = to_ov6211_dev(sd);
	struct regmap *regmap = sensor->regmap_read;
	unsigned int val = 0;
	int ret;

	ret = regmap_read(regmap, reg->reg, &val);
	reg->val = val;
	reg->size = 1;

	return ret;
}

static int ov6211_set_register(struct v4l2_subdev *sd,
			       const struct v4l2_dbg_register *reg)
{
	struct ov6211_dev *sensor = to_ov6211_dev(sd);
	struct regmap *regmap = sensor->regmap;

	return regmap_write(regmap, reg->reg, reg->val & 0xff);
}
#endif

static int ov6211_write_reg(struct ov6211_dev *sensor, u16 reg, u8 val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct regmap *regmap = sensor->regmap;
	int ret;

	ret = regmap_write(regmap, reg, val);
	if (ret < 0)
		dev_err(&client->dev, "error writing reg %u\n", reg);

	return ret;
}

static int ov6211_read_reg(struct ov6211_dev *sensor, u16 reg, u8 *val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct regmap *regmap = sensor->regmap_read;
	unsigned int r;
	int ret;

	ret = regmap_read(regmap, reg, &r);
	if (ret < 0)
		dev_err(&client->dev, "error reading reg %u\n", reg);
	*val = r & 0xff;

	return ret;
}

static int ov6211_mod_reg(struct ov6211_dev *sensor, u16 reg, u8 mask, u8 val)
{
	u8 readval;
	int ret;

	ret = ov6211_read_reg(sensor, reg, &readval);
	if (ret)
		return ret;

	readval &= ~mask;
	val &= mask;
	val |= readval;

	return ov6211_write_reg(sensor, reg, val);
}

static int ov6211_load_regs(struct ov6211_dev *sensor,
			    const struct ov6211_mode_info *mode)
{
	const struct reg_value *regs = mode->reg_data;
	unsigned int i;
	u32 delay_ms;
	u16 reg_addr;
	u8 mask, val;
	int ret = 0;

	for (i = 0; i < mode->reg_data_size; ++i, ++regs) {
		delay_ms = regs->delay_ms;
		reg_addr = regs->reg_addr;
		val = regs->val;
		mask = regs->mask;

		if (mask)
			ret = ov6211_mod_reg(sensor, reg_addr, mask, val);
		else
			ret = ov6211_write_reg(sensor, reg_addr, val);

		if (ret)
			break;

		if (delay_ms)
			usleep_range(1000 * delay_ms, 1000 * delay_ms + 100);
	}

	return ret;
}

static void ov6211_soft_reset(struct ov6211_dev *sensor)
{
	ov6211_write_reg(sensor, OV6211_SC_SOFTWARE_RESET, 0x01);
	usleep_range(5000, 9000);
	ov6211_write_reg(sensor, OV6211_SC_SOFTWARE_RESET, 0x00);
}

static int ov6211_set_exposure(struct ov6211_dev *sensor, u32 exposure)
{
	u32 ce;
	u8 v;

	ov6211_read_reg(sensor, OV6211_TVTS_HI, &v);
	ce = v << 8;
	ov6211_read_reg(sensor, OV6211_TVTS_LO, &v);
	ce |= v;
	ce -= 4;

	if (ce < exposure)
		exposure = ce;

	ov6211_mod_reg(sensor, OV6211_AEC_MANUAL, 1, 1);

	ov6211_write_reg(sensor, OV6211_AEC_EXPO1, (exposure >> 12) & 0x0f);
	ov6211_write_reg(sensor, OV6211_AEC_EXPO2, (exposure >> 4) & 0xff);
	ov6211_write_reg(sensor, OV6211_AEC_EXPO3, (exposure << 4) & 0xf0);

	/* set strobe width equal to exposure time */
	ov6211_write_reg(sensor, OV6211_STROBE_SPAN1, (exposure >> 16) & 0xff);
	ov6211_write_reg(sensor, OV6211_STROBE_SPAN2, (exposure >> 8) & 0xff);
	ov6211_write_reg(sensor, OV6211_STROBE_SPAN3, (exposure) & 0xff);

	return 0;
}

static int internal_set_stream(struct ov6211_dev *sensor, bool on)
{
	u8 hi, lo;

	if (sensor->pending_fi_change == false)
		goto stream;

	switch (sensor->cur_fr_id) {
	case OV6211_10_FPS:
		hi = 0x14;
		lo = 0x88;
		break;
	case OV6211_15_FPS:
		hi = 0x0d;
		lo = 0xb0;
		break;
	case OV6211_30_FPS:
		hi = 0x06;
		lo = 0xd8;
		break;
	case OV6211_45_FPS:
		hi = 0x04;
		lo = 0x90;
		break;
	case OV6211_60_FPS:
		hi = 0x03;
		lo = 0x6c;
		break;
	case OV6211_NUM_FRAMERATES:
		return -EINVAL;
	}

	sensor->pending_fi_change = false;
	ov6211_write_reg(sensor, OV6211_TVTS_HI, hi);
	ov6211_write_reg(sensor, OV6211_TVTS_LO, lo);
stream:
	ov6211_write_reg(sensor, OV6211_SC_MODE_SELECT, 0);
	if (on) {
		usleep_range(4000, 5000);
		if (sensor->exposure)
			ov6211_set_exposure(sensor, sensor->exposure);
		ov6211_write_reg(sensor, OV6211_SC_MODE_SELECT, 1);
		sensor->streaming = true;
	} else {
		sensor->streaming = false;
	}

	return 0;
}

static struct ov6211_mode_info *ov6211_find_mode(int w, int h)
{
	return v4l2_find_nearest_size(ov6211_mode_data,
				      ARRAY_SIZE(ov6211_mode_data), width,
				      height, w, h);
}

static int ov6211_set_mode(struct ov6211_dev *sensor)
{
	const struct ov6211_mode_info *mode = sensor->cur_mode;

	ov6211_soft_reset(sensor);
	ov6211_load_regs(sensor, mode);

	return 0;
}

/* --------------- Subdev Operations --------------- */

static int ov6211_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct ov6211_dev *sensor = to_ov6211_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg, format->pad);
		format->format = *fmt;
	} else {
		/* these are hardcoded as we don't support anything else */
		format->format.colorspace = V4L2_COLORSPACE_RAW;
		format->format.field = V4L2_FIELD_NONE;
		format->format.code = MEDIA_BUS_FMT_Y8_1X8;
		format->format.width = sensor->cur_mode->width;
		format->format.height = sensor->cur_mode->height;
	}

	mutex_unlock(&sensor->lock);

	return 0;
}

static int ov6211_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct ov6211_dev *sensor = to_ov6211_dev(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct ov6211_mode_info *mode;

	mutex_lock(&sensor->lock);

	if (sensor->streaming)
		return -EBUSY;

	/* these are hardcoded as we don't support anything else */
	format->format.colorspace = V4L2_COLORSPACE_RAW;
	format->format.field = V4L2_FIELD_NONE;
	format->format.code = MEDIA_BUS_FMT_Y8_1X8;
	mode = ov6211_find_mode(format->format.width, format->format.height);
	format->format.width = mode->width;
	format->format.height = mode->height;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg, format->pad);
		*fmt = format->format;
		goto out;
	}
	sensor->cur_mode = mode;
out:
	mutex_unlock(&sensor->lock);

	return 0;
}

/*
 * Sensor Controls.
 */

static int ov6211_set_ctrl_exposure(struct ov6211_dev *sensor,
				    enum v4l2_exposure_auto_type auto_exposure)
{
	struct ov6211_ctrls *ctrls = &sensor->ctrls;

	if (auto_exposure == V4L2_EXPOSURE_AUTO) {
		sensor->exposure = 0;
	} else {
		sensor->exposure = ctrls->exposure->val;
		ov6211_set_exposure(sensor, sensor->exposure);
	}

	return 0;
}

static int ov6211_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct ov6211_dev *sensor = to_ov6211_dev(sd);
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_AUTO:
		ret = ov6211_set_ctrl_exposure(sensor, ctrl->val);
		break;
	case V4L2_CID_LINK_FREQ:
		return 0;
	case V4L2_CID_PIXEL_RATE:
		return 0;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ov6211_ctrl_ops = {
	.s_ctrl = ov6211_s_ctrl,
};

static int ov6211_init_controls(struct ov6211_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &ov6211_ctrl_ops;
	struct ov6211_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int ret;

	v4l2_ctrl_handler_init(hdl, 16);

	hdl->lock = &sensor->lock;

	ctrls->auto_exp =
		v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_EXPOSURE_AUTO,
				       V4L2_EXPOSURE_MANUAL, 0,
				       V4L2_EXPOSURE_AUTO);
	ctrls->exposure =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE, 0, 65535, 1, 0);
	v4l2_ctrl_auto_cluster(3, &ctrls->auto_exp, 0, false);
	ctrls->link_freq = v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_LINK_FREQ,
						  0, 0, link_freq_menu_items);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	ctrls->exposure->flags |= V4L2_CTRL_FLAG_VOLATILE;
	sensor->sd.ctrl_handler = hdl;

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int ov6211_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov6211_dev *sensor = to_ov6211_dev(sd);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int internal_set_frame_interval(struct ov6211_dev *sensor,
				       struct v4l2_subdev_frame_interval *fi)
{
	u32 fr_rate;
	int i, ret = -EINVAL;

	if (fi->interval.numerator == 0)
		goto out;

	fr_rate = fi->interval.denominator / fi->interval.numerator;

	for (i = 0; i < ARRAY_SIZE(ov6211_framerates); i++) {
		if (ov6211_framerates[i] == fr_rate)
			break;
	}

	if (i == ARRAY_SIZE(ov6211_framerates))
		goto out;

	sensor->cur_fr_id = i;
	sensor->frame_interval = fi->interval;
	ret = 0;
out:
	return ret;
}

static int ov6211_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov6211_dev *sensor = to_ov6211_dev(sd);
	int ret;

	mutex_lock(&sensor->lock);
	ret = internal_set_frame_interval(sensor, fi);
	sensor->pending_fi_change = true;
	mutex_unlock(&sensor->lock);

	return ret;
}

static int ov6211_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(ov6211_formats))
		return -EINVAL;

	code->code = ov6211_formats[code->index].code;

	return 0;
}

static int ov6211_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov6211_dev *sensor = to_ov6211_dev(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);

	if (enable)
		ret = ov6211_set_mode(sensor);
	internal_set_stream(sensor, enable);

	mutex_unlock(&sensor->lock);

	return ret;
}

static const struct v4l2_subdev_core_ops ov6211_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov6211_get_register,
	.s_register = ov6211_set_register,
#endif
};

static const struct v4l2_subdev_video_ops ov6211_video_ops = {
	.g_frame_interval = ov6211_g_frame_interval,
	.s_frame_interval = ov6211_s_frame_interval,
	.s_stream = ov6211_s_stream,
};

static const struct v4l2_subdev_pad_ops ov6211_pad_ops = {
	.enum_mbus_code = ov6211_enum_mbus_code,
	.get_fmt = ov6211_get_fmt,
	.set_fmt = ov6211_set_fmt,
};

static const struct v4l2_subdev_ops ov6211_subdev_ops = {
	.core = &ov6211_core_ops,
	.video = &ov6211_video_ops,
	.pad = &ov6211_pad_ops,
};

static int ov6211_get_regulators(struct ov6211_dev *sensor)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ov6211_supply_name); i++)
		sensor->supplies[i].supply = ov6211_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       ARRAY_SIZE(ov6211_supply_name),
				       sensor->supplies);
}

static int ov6211_check_chip_id(struct ov6211_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	struct regmap *regmap = sensor->regmap_read;
	unsigned int cid;
	int ret = 0;

	ret = regmap_read(regmap, OV6211_SC_CHIP_ID_HIGH, &cid);
	if (ret || cid != 0x67) {
		ret = ENXIO;
		goto err;
	}

	ret = regmap_read(regmap, OV6211_SC_CHIP_ID_LOW, &cid);
	if (ret || cid != 0x10) {
		ret = -ENXIO;
		goto err;
	}

	ret = regmap_read(regmap, OV6211_SC_REG0C, &cid);
	if (ret)
		goto err;

	dev_info(&client->dev, "found OV6211, sub revision: 0x%02X\n", cid);
	return 0;
err:
	dev_err(&client->dev, "failed to detect OV6211\n");
	return ret;
}

static int ov6211_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct ov6211_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

	fmt = &sensor->fmt;
	fmt->code = MEDIA_BUS_FMT_Y8_1X8;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->field = V4L2_FIELD_NONE;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = 400;
	fmt->height = 200;

	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = ov6211_framerates[OV6211_45_FPS];
	sensor->cur_fr_id = OV6211_45_FPS;
	sensor->cur_mode = &ov6211_mode_data[OV6211_MODE_Y8_400_200];

	sensor->ep.bus_type = V4L2_MBUS_CSI2_DPHY;
	endpoint =
		fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}

	/* request optional power down pin */
	sensor->pwdn_gpio =
		devm_gpiod_get_optional(dev, "powerdown", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->pwdn_gpio))
		return PTR_ERR(sensor->pwdn_gpio);

	/* request optional reset pin */
	sensor->reset_gpio =
		devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return PTR_ERR(sensor->reset_gpio);

	sensor->regmap = devm_regmap_init_i2c(client, &ov6211_regmap_config);
	if (IS_ERR(sensor->regmap)) {
		dev_err(dev, "regmap init failed\n");
		return PTR_ERR(sensor->regmap);
	}

	sensor->i2c_client_read = i2c_new_ancillary_device(
		sensor->i2c_client, "read", OV6211_DEFAULT_READ_I2C_ADDR);
	if (IS_ERR(sensor->i2c_client_read))
		return PTR_ERR(sensor->i2c_client_read);

	i2c_set_clientdata(sensor->i2c_client_read, sensor);

	sensor->regmap_read = devm_regmap_init_i2c(sensor->i2c_client_read,
						   &ov6211_regmap_config);
	if (IS_ERR(sensor->regmap_read)) {
		dev_err(dev, "regmap read init failed\n");
		return PTR_ERR(sensor->regmap_read);
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &ov6211_subdev_ops);

	mutex_init(&sensor->lock);

	sensor->sd.flags |=
		V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto entity_cleanup;

	ret = ov6211_get_regulators(sensor);
	if (ret)
		goto entity_cleanup;

	ret = ov6211_check_chip_id(sensor);
	if (ret)
		goto entity_cleanup;

	ret = ov6211_init_controls(sensor);
	if (ret)
		goto entity_cleanup;

	ov6211_load_regs(sensor, sensor->cur_mode);

	ret = v4l2_async_register_subdev_sensor_common(&sensor->sd);
	if (ret)
		goto free_ctrls;

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);

	return ret;
}

static int ov6211_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov6211_dev *sensor = to_ov6211_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	mutex_destroy(&sensor->lock);

	return 0;
}

static const struct i2c_device_id ov6211_id[] = {
	{ "ov6211", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ov6211_id);

static const struct of_device_id ov6211_dt_ids[] = {
	{ .compatible = "ovti,ov6211" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ov6211_dt_ids);

static struct i2c_driver ov6211_i2c_driver = {
	.driver = {
		.name  = "ov6211",
		.of_match_table	= ov6211_dt_ids,
	},
	.probe_new = ov6211_probe,
	.remove   = ov6211_remove,
};

module_i2c_driver(ov6211_i2c_driver);

MODULE_AUTHOR("Huy Duong <huy.duong@ologn.tech>");
MODULE_DESCRIPTION("V4L2 subdevice driver for OmniVision OV6211 Camera Sensor");
MODULE_LICENSE("GPL");
