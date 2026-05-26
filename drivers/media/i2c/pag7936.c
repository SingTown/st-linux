// SPDX-License-Identifier: GPL-2.0-only
/*
 * PixArt PAG7936 Camera Sensor Driver
 *
 * Copyright (C) 2021 Intel Corporation
 */
#include <asm/unaligned.h>

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Streaming Mode */
#define PAG7936_REG_MODE_SELECT		0x0030
#define PAG7936_REG_OPMODE		0x0008
#define PAG7936_MODE_STANDBY		0x00
#define PAG7936_MODE_STREAMING		0x01

/* Chip ID */
#define PAG7936_REG_ID			0x0000
#define PAG7936_ID			0x7936

/* Input clock rate */
#define PAG7936_INCLK_RATE		24000000

/* CSI2 HW configuration */
#define PAG7936_LINK_FREQ		400000000
#define PAG7936_PIX_FREQ		160000000

/* PAG7936 native and active pixel array size. */
#define PAG7936_NATIVE_WIDTH		1280U
#define PAG7936_NATIVE_HEIGHT		800U

/* Frame time / AE registers */
#define FRAME_TIME_7_0			0x004C
#define AE_EXPO_MANUAL			0x1400
#define AE_EXPO_MANUAL_AE_MANUAL_EN	0x10
#define AE_EXPO_MANUAL_AE_ENH		0x01
#define AE_MAXEXPO_7_0			0x1412
#define AE_GAIN_MANUAL_7_0		0x1423
#define AE_EXPO_MANUAL_7_0		0x1425

#define SENSOR_UPDATE			0x00EB
#define SENSOR_UPDATE_FLAG		0x80
#define SENSOR_OPMODE_RUN		0x83
#define SENSOR_OPMODE_SUSPEND		0x85

#define ISP_TEST_MODE			0x0801
#define ISP_TEST_MODE_RAMP		0x04
#define ISP_WOI_EN			0x0E10

#define TG_MONO_SENSOR			0x01BF
#define TG_FLIP				0x01CE
#define TG_FLIP_HFLIP			BIT(3)
#define TG_FLIP_VFLIP			BIT(2)

#define AVERAGE_MODE			0x01C0
#define ROW_AVERAGE_MODE		0x0166
#define COL_AVERAGE_MODE		0x016F
#define HSIZE_L				0x01C6
#define HSIZE_H				0x01C7
#define VSIZE_L				0x01C8
#define VSIZE_H				0x01C9
#define WOI_HSIZE_L			0x0221
#define WOI_HSIZE_H			0x0222
#define WOI_VSIZE_L			0x0223
#define WOI_VSIZE_H			0x0224
#define WOI_HSTART_L			0x0225
#define WOI_HSTART_H			0x0226
#define WOI_VSTART_L			0x0227
#define WOI_VSTART_H			0x0228

#define FT_CLK				1000000
#define FPS_MIN				1
#define FPS_MAX				120
#define FPS_HW_MAX			470

#define PAG7936_EXP_OFFSET		80
#define PAG7936_EXP_MIN			80
#define PAG7936_EXP_DIV			8
#define PAG7936_GAIN_MIN		23
#define PAG7936_GAIN_MAX		256


/**
 * struct pag7936_reg - pag7936 sensor register
 * @address: Register address
 * @val: Register value
 */
struct pag7936_reg {
	u16 address;
	u8 val;
};

static const char * const pag7936_supply_name[] = {
	"avdd", /* Analog (2.9V) supply */
	"ovdd", /* Digital I/O (1.8V) supply */
	"dvdd", /* Digital Core (1.2V) supply */
};

/**
 * struct pag7936 - pag7936 sensor device structure
 * @dev: Pointer to generic device
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @reset_gpio: Sensor reset gpio
 * @powerdown_gpio: Sensor powerdown gpio
 * @supplies: Regulator supplies to handle power control
 * @inclk: Sensor input clock
 * @ctrl_handler: V4L2 control handler
 * @link_freq_ctrl: Pointer to link frequency control
 * @auto_exp: Pointer to V4L2_CID_EXPOSURE_AUTO control
 * @exposure: Pointer to V4L2_CID_EXPOSURE control
 * @auto_gain: Pointer to V4L2_CID_AUTOGAIN control
 * @gain: Pointer to V4L2_CID_ANALOGUE_GAIN control
 * @hflip: Pointer to V4L2_CID_HFLIP control
 * @vflip: Pointer to V4L2_CID_VFLIP control
 * @pclk_ctrl: Pointer to pixel clock control
 * @test_pattern: Pointer to V4L2_CID_TEST_PATTERN control
 * @width: Current frame width
 * @height: Current frame height
 * @frame_interval: Current frame interval (after clamping)
 * @mutex: Mutex for serializing sensor state and i2c access
 * @streaming: Flag indicating streaming state
 * @lanes_nb: Number of CSI lanes to be used
 */
struct pag7936 {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *powerdown_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(pag7936_supply_name)];
	struct clk *inclk;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq_ctrl;
	struct v4l2_ctrl *auto_exp;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *auto_gain;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *pclk_ctrl;
	struct v4l2_ctrl *test_pattern;
	u32 width;
	u32 height;
	struct v4l2_fract frame_interval;
	struct mutex mutex;
	bool streaming;
	u32 lanes_nb;
};

static const s64 link_freq[] = {
	PAG7936_LINK_FREQ,
};

/* Sensor mode registers */
static const struct pag7936_reg pag7936_1280x800_init[] = {
	{0x004C, 0x8D},
	{0x004D, 0x20},
	{0x004E, 0x00},
	{0x004F, 0x00},
	{0x110A, 0x00},
	{0x0905, 0x60},
	{0x0978, 0x01},
	{0x0979, 0x66},
	{0x097C, 0x30},
	{0x097D, 0x02},
	{0x0985, 0x06},
	{0x0989, 0x10},
	{0x098B, 0x91},
	{0x098E, 0x06},
	{0x0850, 0x00},
	{0x094A, 0x43},
	{0x09A0, 0x08},
	{0x0032, 0x00},
	{0x0033, 0x00},
	{0x0764, 0x00},
	{0x0304, 0x30},
	{0x0305, 0x03},
	{0x0307, 0x10},
	{0x0308, 0x05},
	{0x0306, 0x04},
	{0x0311, 0x00},
	{0x030F, 0x01},
	{0x0168, 0x6D},
	{0x0730, 0x78},
	{0x0724, 0x20},
	{0x0188, 0x14},
	{0x02A5, 0xEF},
	{0x0186, 0x40},
	{0x0A1A, 0x20},
	{0x0A32, 0x2A},
	{0x0A33, 0x2A},
	{0x0A34, 0x0D},
	{0x0A35, 0x0D},
	{0x000B, 0x01},
	{0x0810, 0x01},
	{0x0814, 0xB3}, //R_center_rx[10:0]=691
	{0x0815, 0x02}, //R_center_rx[10:0]=691
	{0x0816, 0xBB}, //R_center_ry[9:0]=443
	{0x0817, 0x01}, //R_center_ry[9:0]=443
	{0x0818, 0xA9}, //R_center_gx[10:0]=681
	{0x0819, 0x02}, //R_center_gx[10:0]=681
	{0x081A, 0xBB}, //R_center_gy[9:0]=443
	{0x081B, 0x01}, //R_center_gy[9:0]=443
	{0x081C, 0xB0}, //R_center_bx[10:0]=688
	{0x081D, 0x02}, //R_center_bx[10:0]=688
	{0x081E, 0xBD}, //R_center_by[9:0]=445
	{0x081F, 0x01}, //R_center_by[9:0]=445
	{0x0820, 0x61}, //R_LSC_RS[7:0]=97
	{0x0821, 0x5B}, //R_LSC_GS[7:0]=91
	{0x0822, 0x5F}, //R_LSC_BS[7:0]=95
	{0x0823, 0x41}, //R_LSC_RQ[7:0]=65
	{0x0824, 0x50}, //R_LSC_GQ[7:0]=80
	{0x0825, 0x7F}, //R_LSC_BQ[7:0]=127
	{0x0826, 0x07}, //R_LSC_SftRS[3:0]=7
	{0x0827, 0x07}, //R_LSC_SftGS[3:0]=7
	{0x0828, 0x07}, //R_LSC_SftBS[3:0]=7
	{0x0829, 0x09}, //R_LSC_SftRQ[3:0]=9
	{0x082A, 0x09}, //R_LSC_SftGQ[3:0]=9
	{0x082B, 0x09}, //R_LSC_SftBQ[3:0]=9
	{0x082E, 0xD8}, //R_LSC_LMaxR2R[15:0]=39640
	{0x082F, 0x9A}, //R_LSC_LMaxR2R[15:0]=39640
	{0x0830, 0xD8}, //R_LSC_LMaxR2G[15:0]=39640
	{0x0831, 0x9A}, //R_LSC_LMaxR2G[15:0]=39640
	{0x0832, 0xD8}, //R_LSC_LMaxR2B[15:0]=39640
	{0x0833, 0x9A}, //R_LSC_LMaxR2B[15:0]=39640
	{0x0E10, 0x01},
	{0x0E11, 0x00},
	{0x0E12, 0x05},
	{0x0E13, 0x20},
	{0x0E14, 0x03},
	{0x0E15, 0x10},
	{0x0E16, 0x00},
	{0x0E17, 0x10},
	{0x0E18, 0x00},
	{SENSOR_UPDATE, SENSOR_UPDATE_FLAG},
};

static const struct pag7936_reg pag7936_1280x800[] = {
	{ TG_MONO_SENSOR,	0x00 },
	{ AVERAGE_MODE,		0x00 },
	{ ROW_AVERAGE_MODE,	0x00 },
	{ COL_AVERAGE_MODE,	0x00 },
	{ ISP_WOI_EN,		0x00 },
	{ HSIZE_L,		0x10 },
	{ HSIZE_H,		0x05 },
	{ VSIZE_L,		0x30 },
	{ VSIZE_H,		0x03 },
	{ WOI_HSIZE_L,		0x00 },
	{ WOI_HSIZE_H,		0x05 },
	{ WOI_VSIZE_L,		0x20 },
	{ WOI_VSIZE_H,		0x03 },
	{ WOI_HSTART_L,		0x08 },
	{ WOI_HSTART_H,		0x00 },
	{ WOI_VSTART_L,		0x08 },
	{ WOI_VSTART_H,		0x00 },
};

static const struct pag7936_reg pag7936_640x400[] = {
	{ TG_MONO_SENSOR,	0x00 },
	{ AVERAGE_MODE,		0x01 },
	{ ROW_AVERAGE_MODE,	0x00 },
	{ COL_AVERAGE_MODE,	0x00 },
	{ ISP_WOI_EN,		0x00 },
	{ HSIZE_L,		0x88 },
	{ HSIZE_H,		0x02 },
	{ VSIZE_L,		0x98 },
	{ VSIZE_H,		0x01 },
	{ WOI_HSIZE_L,		0x80 },
	{ WOI_HSIZE_H,		0x02 },
	{ WOI_VSIZE_L,		0x90 },
	{ WOI_VSIZE_H,		0x01 },
	{ WOI_HSTART_L,		0x04 },
	{ WOI_HSTART_H,		0x00 },
	{ WOI_VSTART_L,		0x04 },
	{ WOI_VSTART_H,		0x00 },
};

static const struct pag7936_reg pag7936_320x200[] = {
	{ TG_MONO_SENSOR,	0x00 },
	{ AVERAGE_MODE,		0x02 },
	{ ROW_AVERAGE_MODE,	0x00 },
	{ COL_AVERAGE_MODE,	0x00 },
	{ ISP_WOI_EN,		0x00 },
	{ HSIZE_L,		0x44 },
	{ HSIZE_H,		0x01 },
	{ VSIZE_L,		0xCC },
	{ VSIZE_H,		0x00 },
	{ WOI_HSIZE_L,		0x40 },
	{ WOI_HSIZE_H,		0x01 },
	{ WOI_VSIZE_L,		0xC8 },
	{ WOI_VSIZE_H,		0x00 },
	{ WOI_HSTART_L,		0x02 },
	{ WOI_HSTART_H,		0x00 },
	{ WOI_VSTART_L,		0x02 },
	{ WOI_VSTART_H,		0x00 },
};

struct pag7936_mode {
	u32 width;
	u32 height;
	const struct pag7936_reg *regs;
	u32 num_regs;
};

static const struct pag7936_mode pag7936_modes[] = {
	{ 1280, 800, pag7936_1280x800, ARRAY_SIZE(pag7936_1280x800) },
	{  640, 400, pag7936_640x400,  ARRAY_SIZE(pag7936_640x400)  },
	{  320, 200, pag7936_320x200,  ARRAY_SIZE(pag7936_320x200)  },
};

static const struct pag7936_mode *pag7936_find_mode(u32 w, u32 h)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(pag7936_modes); i++)
		if (pag7936_modes[i].width == w && pag7936_modes[i].height == h)
			return &pag7936_modes[i];
	return NULL;
}

/**
 * to_pag7936() - pag7936 V4L2 sub-device to pag7936 device.
 * @subdev: pointer to pag7936 V4L2 sub-device
 *
 * Return: pointer to pag7936 device
 */
static inline struct pag7936 *to_pag7936(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct pag7936, sd);
}

/**
 * pag7936_read_reg() - Read registers.
 * @pag7936: pointer to pag7936 device
 * @reg: register address
 * @len: length of bytes to read. Max supported bytes is 4
 * @val: pointer to register value to be filled.
 *
 * Big endian register addresses with little endian values.
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_read_reg(struct pag7936 *pag7936, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&pag7936->sd);
	struct i2c_msg msgs[2] = {0};
	u8 addr_buf[2] = {0};
	u8 data_buf[4] = {0};
	int ret;

	if (WARN_ON(len == 0 || len > 4))
		return -EINVAL;

	put_unaligned_be16(reg, addr_buf);

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data_buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;

	*val = get_unaligned_le32(data_buf);
	if (len < 4)
		*val &= (1U << (len * 8)) - 1;

	return 0;
}

/**
 * pag7936_write_reg() - Write register
 * @pag7936: pointer to pag7936 device
 * @reg: register address
 * @len: length of bytes. Max supported bytes is 4
 * @val: register value
 *
 * Big endian register addresses with little endian values.
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_write_reg(struct pag7936 *pag7936, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&pag7936->sd);
	u8 buf[6] = {0};
	int ret;

	if (WARN_ON(len == 0 || len > 4))
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	switch (len) {
	case 1:
		buf[2] = val & 0xff;
		break;
	case 2:
		put_unaligned_le16(val & 0xffff, buf + 2);
		break;
	case 3:
		buf[2] = val & 0xff;
		buf[3] = (val >> 8) & 0xff;
		buf[4] = (val >> 16) & 0xff;
		break;
	case 4:
		put_unaligned_le32(val, buf + 2);
		break;
	}

	ret = i2c_master_send(client, buf, len + 2);
	if (ret != (int)(len + 2)) {
		dev_err(pag7936->dev,
			"i2c write reg 0x%04x len %u failed: %d\n",
			reg, len, ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

/**
 * pag7936_write_regs() - Write a list of registers
 * @pag7936: pointer to pag7936 device
 * @regs: list of registers to be written
 * @len: length of registers array
 *
 * Return: 0 if successful. error code otherwise.
 */
static int pag7936_write_regs(struct pag7936 *pag7936,
			     const struct pag7936_reg *regs, u32 len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = pag7936_write_reg(pag7936, regs[i].address, 1,
					regs[i].val);
		if (ret) {
			dev_err(pag7936->dev,
				"failed to write reg[%u] 0x%04x=0x%02x: %d\n",
				i, regs[i].address, regs[i].val, ret);
			return ret;
		}
	}

	return 0;
}

static int pag7936_set_auto_gain_exp(struct pag7936 *pag7936, bool auto_en)
{
	u32 reg = auto_en ? AE_EXPO_MANUAL_AE_ENH : AE_EXPO_MANUAL_AE_MANUAL_EN;
	int ret;

	ret = pag7936_write_reg(pag7936, AE_EXPO_MANUAL, 1, reg);
	if (ret)
		return ret;
	return pag7936_write_reg(pag7936, SENSOR_UPDATE, 1, SENSOR_UPDATE_FLAG);
}

static int pag7936_get_auto_gain_exp(struct pag7936 *pag7936, bool *flag)
{
	u32 val = 0;
	int ret;

	ret = pag7936_read_reg(pag7936, AE_EXPO_MANUAL, 1, &val);
	if (ret)
		return ret;
	*flag = !!(val & AE_EXPO_MANUAL_AE_ENH);

	return 0;
}

/**
 * pag7936_set_ctrl() - Set subdevice control
 * @ctrl: pointer to v4l2_ctrl structure
 *
 * Called by the V4L2 control framework with ctrl_hdlr->lock held;
 * that lock aliases @pag7936->mutex (see pag7936_init_controls()).
 * This callback must therefore NOT take @pag7936->mutex itself, and
 * must not call any helper that does (the underlying mutex is not
 * recursive). All i2c helpers used here are lock-free.
 *
 * Supported controls:
 * - cluster controls:
 *   - V4L2_CID_ANALOGUE_GAIN
 *   - V4L2_CID_EXPOSURE
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct pag7936 *pag7936 =
		container_of(ctrl->handler, struct pag7936, ctrl_handler);
	u32 reg, exposure_us, frame_time;
	bool auto_en;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		auto_en = ctrl->val;
		/* AE/AGC share one HW bit; mirror state on the other ctrl. */
		pag7936->auto_exp->cur.val = auto_en ? V4L2_EXPOSURE_AUTO
						     : V4L2_EXPOSURE_MANUAL;
		return pag7936_set_auto_gain_exp(pag7936, auto_en);

	case V4L2_CID_EXPOSURE_AUTO:
		auto_en = (ctrl->val == V4L2_EXPOSURE_AUTO);
		pag7936->auto_gain->cur.val = auto_en;
		return pag7936_set_auto_gain_exp(pag7936, auto_en);

	case V4L2_CID_ANALOGUE_GAIN:
		ret = pag7936_set_auto_gain_exp(pag7936, false);
		if (ret)
			return ret;
		pag7936->auto_gain->cur.val = 0;
		pag7936->auto_exp->cur.val = V4L2_EXPOSURE_MANUAL;

		ret = pag7936_write_reg(pag7936, AE_GAIN_MANUAL_7_0, 2,
					ctrl->val);
		if (ret)
			return ret;
		return pag7936_write_reg(pag7936, SENSOR_UPDATE, 1,
					 SENSOR_UPDATE_FLAG);

	case V4L2_CID_EXPOSURE:
		ret = pag7936_set_auto_gain_exp(pag7936, false);
		if (ret)
			return ret;
		pag7936->auto_gain->cur.val = 0;
		pag7936->auto_exp->cur.val = V4L2_EXPOSURE_MANUAL;

		frame_time = 0;
		ret = pag7936_read_reg(pag7936, FRAME_TIME_7_0, 3, &frame_time);
		if (ret)
			return ret;

		exposure_us = ctrl->val;
		if (frame_time > PAG7936_EXP_OFFSET) {
			u32 max = (frame_time - PAG7936_EXP_OFFSET) /
				  PAG7936_EXP_DIV;

			if (exposure_us > max)
				exposure_us = max;
		}
		exposure_us = max_t(u32, exposure_us, PAG7936_EXP_MIN);
		/* Reflect clamped value back so framework updates cur.val. */
		ctrl->val = exposure_us;

		ret = pag7936_write_reg(pag7936, AE_EXPO_MANUAL_7_0, 3,
					exposure_us);
		if (ret)
			return ret;
		return pag7936_write_reg(pag7936, SENSOR_UPDATE, 1,
					 SENSOR_UPDATE_FLAG);

	case V4L2_CID_VFLIP:
		reg = pag7936->hflip->cur.val ? TG_FLIP_HFLIP : 0;
		if (ctrl->val)
			reg |= TG_FLIP_VFLIP;
		ret = pag7936_write_reg(pag7936, TG_FLIP, 1, reg);
		if (ret)
			return ret;
		return pag7936_write_reg(pag7936, SENSOR_UPDATE, 1,
					 SENSOR_UPDATE_FLAG);

	case V4L2_CID_HFLIP:
		reg = pag7936->vflip->cur.val ? TG_FLIP_VFLIP : 0;
		if (ctrl->val)
			reg |= TG_FLIP_HFLIP;
		ret = pag7936_write_reg(pag7936, TG_FLIP, 1, reg);
		if (ret)
			return ret;
		return pag7936_write_reg(pag7936, SENSOR_UPDATE, 1,
					 SENSOR_UPDATE_FLAG);

	case V4L2_CID_TEST_PATTERN:
		ret = pag7936_read_reg(pag7936, ISP_TEST_MODE, 1, &reg);
		if (ret)
			return ret;
		reg = ctrl->val ? (reg | ISP_TEST_MODE_RAMP)
				: (reg & ~ISP_TEST_MODE_RAMP);
		ret = pag7936_write_reg(pag7936, ISP_TEST_MODE, 1, reg);
		if (ret)
			return ret;
		return pag7936_write_reg(pag7936, SENSOR_UPDATE, 1,
					 SENSOR_UPDATE_FLAG);
	}

	return 0;
}

/* V4l2 subdevice control ops*/
static const struct v4l2_ctrl_ops pag7936_ctrl_ops = {
	.s_ctrl = pag7936_set_ctrl,
};

/**
 * pag7936_enum_mbus_code() - Enumerate V4L2 sub-device mbus codes
 * @sd: pointer to pag7936 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @code: V4L2 sub-device code enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	return 0;
}

/**
 * pag7936_enum_frame_size() - Enumerate V4L2 sub-device frame sizes
 * @sd: pointer to pag7936 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fsize)
{
	if (fsize->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;
	if (fsize->index >= ARRAY_SIZE(pag7936_modes))
		return -EINVAL;

	fsize->min_width = pag7936_modes[fsize->index].width;
	fsize->max_width = pag7936_modes[fsize->index].width;
	fsize->min_height = pag7936_modes[fsize->index].height;
	fsize->max_height = pag7936_modes[fsize->index].height;

	return 0;
}

/**
 * pag7936_fill_pad_format() - Fill subdevice pad format
 *                            from selected sensor mode
 * @pag7936: pointer to pag7936 device
 * @fmt: V4L2 sub-device format need to be filled
 */
static void pag7936_fill_pad_format(struct pag7936 *pag7936,
				   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = pag7936->width;
	fmt->format.height = pag7936->height;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->format.quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
}

/**
 * pag7936_get_pad_format() - Get subdevice pad format
 * @sd: pointer to pag7936 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct pag7936 *pag7936 = to_pag7936(sd);

	/*
	 * TRY format lives in sd_state, whose lock the V4L2 framework
	 * already holds for us; reading it does not need the driver
	 * mutex and taking both would risk an ABBA against any path
	 * that holds the driver mutex first.
	 */
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
		fmt->format = *framefmt;
		return 0;
	}

	mutex_lock(&pag7936->mutex);
	pag7936_fill_pad_format(pag7936, fmt);
	mutex_unlock(&pag7936->mutex);

	return 0;
}

/**
 * pag7936_set_pad_format() - Set subdevice pad format
 * @sd: pointer to pag7936 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct pag7936 *pag7936 = to_pag7936(sd);
	const struct pag7936_mode *mode;
	int ret = 0;

	mode = pag7936_find_mode(fmt->format.width, fmt->format.height);
	if (!mode)
		mode = &pag7936_modes[0];

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->format.quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;

	/*
	 * Same locking rationale as get_pad_format(): TRY only touches
	 * sd_state (lock held by the V4L2 framework), so do not take the
	 * driver mutex here to avoid ABBA against ACTIVE callers.
	 */
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt =
			v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);

		*framefmt = fmt->format;
		return 0;
	}

	mutex_lock(&pag7936->mutex);

	if (pag7936->streaming) {
		ret = -EBUSY;
		goto unlock;
	}

	if (pag7936->width == mode->width && pag7936->height == mode->height)
		goto unlock;

	ret = pag7936_write_regs(pag7936, mode->regs, mode->num_regs);
	if (ret)
		goto unlock;
	ret = pag7936_write_reg(pag7936, SENSOR_UPDATE, 1, SENSOR_UPDATE_FLAG);
	if (ret)
		goto unlock;

	pag7936->width = mode->width;
	pag7936->height = mode->height;

unlock:
	mutex_unlock(&pag7936->mutex);
	return ret;
}

/**
 * pag7936_init_pad_cfg() - Initialize sub-device pad configuration
 * @sd: pointer to pag7936 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_init_pad_cfg(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state)
{
	struct pag7936 *pag7936 = to_pag7936(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};

	if (!sd_state)
		return 0;

	pag7936_fill_pad_format(pag7936, &fmt);
	return pag7936_set_pad_format(sd, sd_state, &fmt);
}

/**
 * pag7936_get_selection() - Selection API
 * @sd: pointer to pag7936 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @sel: V4L2 selection info
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = PAG7936_NATIVE_WIDTH;
		sel->r.height = PAG7936_NATIVE_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

/**
 * pag7936_start_streaming() - Start sensor stream
 * @pag7936: pointer to pag7936 device
 *
 * Caller must hold @pag7936->mutex. The control handler's lock
 * (ctrl_hdlr->lock) is set to the same mutex in pag7936_init_controls(),
 * so __v4l2_ctrl_handler_setup() — which requires the handler lock to
 * be held but does not take it — is called legitimately from here.
 * As a consequence, pag7936_set_ctrl() (invoked via that helper) must
 * NEVER re-acquire @pag7936->mutex; doing so would deadlock.
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_start_streaming(struct pag7936 *pag7936)
{
	int ret;

	lockdep_assert_held(&pag7936->mutex);

	/* Setup handler will write actual exposure and gain */
	ret = __v4l2_ctrl_handler_setup(pag7936->sd.ctrl_handler);
	if (ret) {
		dev_err(pag7936->dev, "fail to setup handler\n");
		return ret;
	}

	ret = pag7936_write_reg(pag7936, PAG7936_REG_MODE_SELECT, 1,
				PAG7936_MODE_STREAMING);
	if (ret) {
		dev_err(pag7936->dev, "fail to set mode\n");
		return ret;
	}
	ret = pag7936_write_reg(pag7936, PAG7936_REG_OPMODE, 1,
				SENSOR_OPMODE_RUN);
	if (ret) {
		dev_err(pag7936->dev, "fail to start streaming\n");
		return ret;
	}

	/* Initial regulator stabilization period */
	usleep_range(18000, 20000);

	return 0;
}

/**
 * pag7936_stop_streaming() - Stop sensor stream
 * @pag7936: pointer to pag7936 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_stop_streaming(struct pag7936 *pag7936)
{
	int ret;

	ret = pag7936_write_reg(pag7936, PAG7936_REG_MODE_SELECT, 1,
				PAG7936_MODE_STANDBY);
	if (ret)
		return ret;
	return pag7936_write_reg(pag7936, PAG7936_REG_OPMODE, 1,
				 SENSOR_OPMODE_SUSPEND);
}

static int pag7936_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct pag7936 *pag7936 = to_pag7936(sd);

	mutex_lock(&pag7936->mutex);
	fi->interval = pag7936->frame_interval;
	mutex_unlock(&pag7936->mutex);

	return 0;
}

static int pag7936_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct pag7936 *pag7936 = to_pag7936(sd);
	u32 exposure_us = 0;
	u32 frame_time;
	bool auto_gain_exp = false;
	u16 exp_reg;
	int fps_max;
	int fps;
	int ret;

	if (fi->interval.numerator == 0 || fi->interval.denominator == 0)
		return -EINVAL;

	fps = fi->interval.denominator / fi->interval.numerator;
	if (fps < FPS_MIN)
		return -EINVAL;

	mutex_lock(&pag7936->mutex);

	fps_max = min_t(int, FPS_MAX * 1280 / pag7936->width, FPS_HW_MAX);
	if (fps_max < FPS_MIN) {
		ret = -EINVAL;
		goto unlock;
	}
	if (fps > fps_max)
		fps = fps_max;

	pag7936->frame_interval.numerator = 1;
	pag7936->frame_interval.denominator = fps;

	frame_time = FT_CLK / fps;
	ret = pag7936_write_reg(pag7936, FRAME_TIME_7_0, 3, frame_time);
	if (ret)
		goto unlock;

	ret = pag7936_get_auto_gain_exp(pag7936, &auto_gain_exp);
	if (ret)
		goto unlock;

	exp_reg = auto_gain_exp ? AE_MAXEXPO_7_0 : AE_EXPO_MANUAL_7_0;
	ret = pag7936_read_reg(pag7936, exp_reg, 3, &exposure_us);
	if (ret)
		goto unlock;

	if (frame_time > PAG7936_EXP_OFFSET) {
		u32 max = (frame_time - PAG7936_EXP_OFFSET) / PAG7936_EXP_DIV;

		if (exposure_us > max)
			exposure_us = max;
	}
	exposure_us = max_t(u32, exposure_us, PAG7936_EXP_MIN);

	ret = pag7936_write_reg(pag7936, exp_reg, 3, exposure_us);
	if (ret)
		goto unlock;
	ret = pag7936_write_reg(pag7936, SENSOR_UPDATE, 1, SENSOR_UPDATE_FLAG);

	fi->interval = pag7936->frame_interval;
unlock:
	mutex_unlock(&pag7936->mutex);
	return ret;
}

/**
 * pag7936_set_stream() - Enable sensor streaming
 * @sd: pointer to pag7936 subdevice
 * @enable: set to enable sensor streaming
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct pag7936 *pag7936 = to_pag7936(sd);
	int ret = 0;

	mutex_lock(&pag7936->mutex);

	if (pag7936->streaming == !!enable)
		goto unlock;

	if (enable)
		ret = pag7936_start_streaming(pag7936);
	else
		ret = pag7936_stop_streaming(pag7936);

	if (!ret)
		pag7936->streaming = !!enable;

unlock:
	mutex_unlock(&pag7936->mutex);
	return ret;
}

/**
 * pag7936_detect() - Detect pag7936 sensor
 * @pag7936: pointer to pag7936 device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int pag7936_detect(struct pag7936 *pag7936)
{
	int ret;
	u32 val;

	ret = pag7936_read_reg(pag7936, PAG7936_REG_ID, 2, &val);
	if (ret) {
		dev_err(pag7936->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != PAG7936_ID) {
		dev_err(pag7936->dev,
			"chip id mismatch: expected 0x%x, got 0x%x\n",
			PAG7936_ID, val);
		return -ENXIO;
	}

	return 0;
}

/**
 * pag7936_parse_hw_config() - Parse HW configuration and check if supported
 * @pag7936: pointer to pag7936 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_parse_hw_config(struct pag7936 *pag7936)
{
	struct fwnode_handle *fwnode = dev_fwnode(pag7936->dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *ep;
	unsigned long rate;
	unsigned int i;
	int ret;

	if (!fwnode)
		return -ENXIO;

	/* Request optional reset pin */
	pag7936->reset_gpio = devm_gpiod_get_optional(pag7936->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(pag7936->reset_gpio)) {
		dev_err(pag7936->dev, "failed to get reset gpio %ld\n",
			PTR_ERR(pag7936->reset_gpio));
		return PTR_ERR(pag7936->reset_gpio);
	}

	/* Request optional powerdown pin */
	pag7936->powerdown_gpio = devm_gpiod_get_optional(pag7936->dev, "powerdown",
							 GPIOD_OUT_HIGH);
	if (IS_ERR(pag7936->powerdown_gpio)) {
		dev_err(pag7936->dev, "failed to get powerdown gpio %ld\n",
			PTR_ERR(pag7936->powerdown_gpio));
		return PTR_ERR(pag7936->powerdown_gpio);
	}

	for (i = 0; i < ARRAY_SIZE(pag7936_supply_name); i++)
		pag7936->supplies[i].supply = pag7936_supply_name[i];

	ret = devm_regulator_bulk_get(pag7936->dev,
				      ARRAY_SIZE(pag7936_supply_name),
				      pag7936->supplies);
	if (ret) {
		dev_err(pag7936->dev, "Failed to get regulators\n");
		return ret;
	}

	/* Get sensor input clock */
	pag7936->inclk = devm_clk_get(pag7936->dev, NULL);
	if (IS_ERR(pag7936->inclk)) {
		dev_err(pag7936->dev, "could not get inclk\n");
		return PTR_ERR(pag7936->inclk);
	}

	rate = clk_get_rate(pag7936->inclk);
	if (rate != PAG7936_INCLK_RATE) {
		dev_err(pag7936->dev, "inclk frequency mismatch\n");
		return -EINVAL;
	}

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep) {
		dev_err(pag7936->dev, "Failed to get next endpoint\n");
		return -ENXIO;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != 2 &&
	    bus_cfg.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(pag7936->dev,
			"number of CSI2 data lanes %d is not supported\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto done_endpoint_free;
	}
	pag7936->lanes_nb = bus_cfg.bus.mipi_csi2.num_data_lanes;

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(pag7936->dev, "no link frequencies defined\n");
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	ret = -EINVAL;
	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
		if (bus_cfg.link_frequencies[i] == PAG7936_LINK_FREQ) {
			ret = 0;
			break;
		}
	}
	if (ret)
		dev_err(pag7936->dev, "no compatible link frequencies found\n");

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/* V4l2 subdevice ops */
static const struct v4l2_subdev_video_ops pag7936_video_ops = {
	.g_frame_interval = pag7936_g_frame_interval,
	.s_frame_interval = pag7936_s_frame_interval,
	.s_stream = pag7936_set_stream,
};

static const struct v4l2_subdev_pad_ops pag7936_pad_ops = {
	.init_cfg = pag7936_init_pad_cfg,
	.enum_mbus_code = pag7936_enum_mbus_code,
	.enum_frame_size = pag7936_enum_frame_size,
	.get_selection = pag7936_get_selection,
	.get_fmt = pag7936_get_pad_format,
	.set_fmt = pag7936_set_pad_format,
};

static const struct v4l2_subdev_ops pag7936_subdev_ops = {
	.video = &pag7936_video_ops,
	.pad = &pag7936_pad_ops,
};

/**
 * pag7936_power_on() - Sensor power on sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct pag7936 *pag7936 = to_pag7936(sd);
	int ret;

	ret = clk_prepare_enable(pag7936->inclk);
	if (ret) {
		dev_err(pag7936->dev, "fail to enable inclk\n");
		return ret;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(pag7936_supply_name),
				    pag7936->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators\n");
		goto error_clk;
	}

	gpiod_set_value_cansleep(pag7936->reset_gpio, 0);
	gpiod_set_value_cansleep(pag7936->powerdown_gpio, 1);
	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(pag7936->powerdown_gpio, 0);
	usleep_range(3000, 5000);

	gpiod_set_value_cansleep(pag7936->reset_gpio, 1);

	usleep_range(20000, 21000);

	return 0;

error_clk:
	clk_disable_unprepare(pag7936->inclk);
	return ret;
}

static int pag7936_suspend(struct device *dev);

/**
 * pag7936_resume() - Sensor power on sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_resume(struct device *dev)
{
	struct pag7936 *pag7936 = to_pag7936(dev_get_drvdata(dev));
	int ret;

	mutex_lock(&pag7936->mutex);

	ret = pag7936_power_on(dev);
	if (ret) {
		dev_err(dev, "failed to power on sensor\n");
		goto unlock;
	}

	ret = pag7936_write_regs(pag7936, pag7936_1280x800_init,
				 ARRAY_SIZE(pag7936_1280x800_init));
	if (ret) {
		dev_err(pag7936->dev, "fail to write initial registers\n");
		mutex_unlock(&pag7936->mutex);
		pag7936_suspend(dev);
		return ret;
	}

unlock:
	mutex_unlock(&pag7936->mutex);
	return ret;
}

/**
 * pag7936_suspend() - Sensor power off sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_suspend(struct device *dev)
{
	struct pag7936 *pag7936 = to_pag7936(dev_get_drvdata(dev));
	int ret;

	mutex_lock(&pag7936->mutex);

	if (pag7936->streaming) {
		ret = pag7936_stop_streaming(pag7936);
		if (ret)
			dev_warn(dev, "stop streaming on suspend failed: %d\n",
				 ret);
		pag7936->streaming = false;
	}

	gpiod_set_value_cansleep(pag7936->reset_gpio, 0);
	gpiod_set_value_cansleep(pag7936->powerdown_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(pag7936_supply_name), pag7936->supplies);
	clk_disable_unprepare(pag7936->inclk);

	mutex_unlock(&pag7936->mutex);

	return 0;
}

static const char * const test_pattern_menu[] = {
	"Disabled",
	"Ramp",
};

/**
 * pag7936_init_controls() - Initialize sensor subdevice controls
 * @pag7936: pointer to pag7936 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_init_controls(struct pag7936 *pag7936)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &pag7936->ctrl_handler;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 9);
	if (ret)
		return ret;

	/* Serialize controls with sensor device */
	ctrl_hdlr->lock = &pag7936->mutex;

	/* Initialize exposure and gain */

	pag7936->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &pag7936_ctrl_ops,
					   V4L2_CID_HFLIP, 0, 1, 1, 0);
	pag7936->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &pag7936_ctrl_ops,
					   V4L2_CID_VFLIP, 0, 1, 1, 0);

	/* Auto/manual exposure */
	pag7936->auto_exp = v4l2_ctrl_new_std_menu(ctrl_hdlr, &pag7936_ctrl_ops,
						   V4L2_CID_EXPOSURE_AUTO,
						   V4L2_EXPOSURE_MANUAL, 0,
						   V4L2_EXPOSURE_AUTO);
	pag7936->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &pag7936_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      PAG7936_EXP_MIN, 25000, 1,
					      PAG7936_EXP_MIN);

	/* Auto/manual gain */
	pag7936->auto_gain = v4l2_ctrl_new_std(ctrl_hdlr, &pag7936_ctrl_ops,
					       V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	pag7936->gain = v4l2_ctrl_new_std(ctrl_hdlr, &pag7936_ctrl_ops,
					  V4L2_CID_ANALOGUE_GAIN,
					  PAG7936_GAIN_MIN, PAG7936_GAIN_MAX, 1,
					  PAG7936_GAIN_MIN);

	/* Read only controls */
	pag7936->pclk_ctrl = v4l2_ctrl_new_std(ctrl_hdlr, &pag7936_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       PAG7936_PIX_FREQ,
					       PAG7936_PIX_FREQ, 1,
					       PAG7936_PIX_FREQ);
	if (pag7936->pclk_ctrl)
		pag7936->pclk_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pag7936->link_freq_ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr,
					&pag7936_ctrl_ops,
					V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(link_freq) - 1,
					0, link_freq);
	if (pag7936->link_freq_ctrl)
		pag7936->link_freq_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pag7936->test_pattern = v4l2_ctrl_new_std_menu_items(ctrl_hdlr,
					&pag7936_ctrl_ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(test_pattern_menu) - 1,
					0, 0, test_pattern_menu);

	if (ctrl_hdlr->error) {
		dev_err(pag7936->dev, "control init failed: %d\n",
			ctrl_hdlr->error);
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ctrl_hdlr->error;
	}

	pag7936->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

/**
 * pag7936_probe() - I2C client device binding
 * @client: pointer to i2c client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int pag7936_probe(struct i2c_client *client)
{
	struct pag7936 *pag7936;
	int ret;

	pag7936 = devm_kzalloc(&client->dev, sizeof(*pag7936), GFP_KERNEL);
	if (!pag7936)
		return -ENOMEM;

	pag7936->dev = &client->dev;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&pag7936->sd, client, &pag7936_subdev_ops);

	ret = pag7936_parse_hw_config(pag7936);
	if (ret) {
		dev_err(pag7936->dev, "HW configuration is not supported\n");
		return ret;
	}

	mutex_init(&pag7936->mutex);

	ret = pag7936_power_on(pag7936->dev);
	if (ret) {
		dev_err(pag7936->dev, "failed to power-on the sensor\n");
		goto error_mutex_destroy;
	}

	/* Check module identity */
	ret = pag7936_detect(pag7936);
	if (ret) {
		dev_err(pag7936->dev, "failed to find sensor: %d\n", ret);
		goto error_power_off;
	}

	ret = pag7936_write_regs(pag7936, pag7936_1280x800_init,
				 ARRAY_SIZE(pag7936_1280x800_init));
	if (ret) {
		dev_err(pag7936->dev, "fail to write initial registers\n");
		goto error_power_off;
	}

	/* Set default mode to max resolution */
	pag7936->width = 1280;
	pag7936->height = 800;
	pag7936->frame_interval.numerator = 1;
	pag7936->frame_interval.denominator = 120;

	ret = pag7936_init_controls(pag7936);
	if (ret) {
		dev_err(pag7936->dev, "failed to init controls: %d\n", ret);
		goto error_power_off;
	}

	/* Initialize subdev */
	pag7936->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	pag7936->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	pag7936->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&pag7936->sd.entity, 1, &pag7936->pad);
	if (ret) {
		dev_err(pag7936->dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&pag7936->sd);
	if (ret < 0) {
		dev_err(pag7936->dev,
			"failed to register async subdev: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&pag7936->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(&pag7936->ctrl_handler);
error_power_off:
	pag7936_suspend(pag7936->dev);
error_mutex_destroy:
	mutex_destroy(&pag7936->mutex);

	return ret;
}

/**
 * pag7936_remove() - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static void pag7936_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct pag7936 *pag7936 = to_pag7936(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&pag7936->ctrl_handler);
	pag7936_suspend(&client->dev);
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&pag7936->mutex);
}

static const struct dev_pm_ops pag7936_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pag7936_suspend, pag7936_resume)
};

static const struct of_device_id pag7936_of_match[] = {
	{ .compatible = "pag7936" },
	{ }
};

MODULE_DEVICE_TABLE(of, pag7936_of_match);

static struct i2c_driver pag7936_driver = {
	.probe = pag7936_probe,
	.remove = pag7936_remove,
	.driver = {
		.name = "pag7936",
		.pm = &pag7936_pm_ops,
		.of_match_table = pag7936_of_match,
	},
};

module_i2c_driver(pag7936_driver);

MODULE_AUTHOR("PixArt Imaging");
MODULE_DESCRIPTION("Pag7936 sensor driver");
MODULE_LICENSE("GPL");
