// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for VD55G0 global shutter sensor family driver
 *
 * Copyright (C) 2022 STMicroelectronics SA
 */

#include <linux/version.h>

#include <asm-generic/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Backward compatibility */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 18, 0)
#define MIPI_CSI2_DT_RAW8 	0x2a
#define MIPI_CSI2_DT_RAW10	0x2b
#define MIPI_CSI2_DT_RAW12	0x2c
#define MIPI_CSI2_DT_RAW14	0x2d
#define MIPI_CSI2_DT_RAW16	0x2e
#else
#include <media/mipi-csi2.h>
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 15, 0)
#define HZ_PER_MHZ		1000000UL
#else
#include <linux/units.h>
#endif

#define VD55G0_REG_8BIT(n)				((1 << 16) | (n))
#define VD55G0_REG_16BIT(n)				((2 << 16) | (n))
#define VD55G0_REG_32BIT(n)				((4 << 16) | (n))
#define VD55G0_REG_SIZE_SHIFT				16
#define VD55G0_REG_ADDR_MASK				0xffff

#define VD55G0_REG_MODEL_ID				VD55G0_REG_16BIT(0x0000)
#define VD55G0_MODEL_ID					0x4730
#define VD55G0_REG_FWPATCH_REVISION			VD55G0_REG_16BIT(0x0022)
#define VD55G0_REG_SYSTEM_FSM				VD55G0_REG_8BIT(0x002c)
#define VD55G0_SYSTEM_FSM_READY_TO_BOOT			0x01
#define VD55G0_SYSTEM_FSM_SW_STBY			0x02
#define VD55G0_SYSTEM_FSM_STREAMING			0x03
#define VD55G0_REG_TEMPERATURE				VD55G0_REG_16BIT(0x004c)
#define VD55G0_REG_BOOT					VD55G0_REG_8BIT(0x0200)
#define VD55G0_BOOT_BOOT				1
#define VD55G0_BOOT_PATCH_SETUP				2
#define VD55G0_REG_SW_STBY				VD55G0_REG_8BIT(0x0201)
#define VD55G0_SW_STBY_START_STREAM			1
#define VD55G0_SW_STBY_THSENS_READ			4
#define VD55G0_REG_STREAMING				VD55G0_REG_8BIT(0x0202)
#define VD55G0_STREAMING_STOP_STREAM			1
#define VD55G0_REG_EXT_CLOCK				VD55G0_REG_32BIT(0x0220)
#define VD55G0_REG_LINE_LENGTH				VD55G0_REG_16BIT(0x0300)
#define VD55G0_REG_ORIENTATION				VD55G0_REG_8BIT(0x0302)
#define VD55G0_REG_FORMAT_CTRL				VD55G0_REG_8BIT(0x030a)
#define VD55G0_REG_OIF_CTRL				VD55G0_REG_16BIT(0x030c)
#define VD55G0_REG_OIF_IMG_CTRL				VD55G0_REG_8BIT(0x030f)
#define VD55G0_REG_OIF_ISL_CTRL				VD55G0_REG_8BIT(0x0310)
#define VD55G0_REG_CLK_PLL_MIPI				VD55G0_REG_32BIT(0x0224)
#define VD55G0_REG_ISL_ENABLE				VD55G0_REG_8BIT(0x0329)
#define VD55G0_REG_PATGEN_CTRL				VD55G0_REG_16BIT(0x0400)
#define VD55G0_PATGEN_TYPE_SHIFT			4
#define VD55G0_PATGEN_ENABLE				BIT(0)
#define VD55G0_REG_MANUAL_ANALOG_GAIN			VD55G0_REG_8BIT(0x044d)
#define VD55G0_REG_MANUAL_COARSE_EXPOSURE		VD55G0_REG_16BIT(0x044e)
#define VD55G0_REG_MANUAL_DIGITAL_GAIN			VD55G0_REG_16BIT(0x0450)
#define VD55G0_REG_EXP_MODE				VD55G0_REG_8BIT(0x044c)
#define VD55G0_EXP_MODE_AUTO				0
#define VD55G0_EXP_MODE_FREEZE				1
#define VD55G0_EXP_MODE_MANUAL				2
#define VD55G0_REG_FRAME_LENGTH				VD55G0_REG_16BIT(0x0458)
#define VD55G0_REG_ROI_X_START				VD55G0_REG_16BIT(0x045e)
#define VD55G0_REG_ROI_X_END				VD55G0_REG_16BIT(0x0460)
#define VD55G0_REG_ROI_Y_START				VD55G0_REG_16BIT(0x0462)
#define VD55G0_REG_ROI_Y_END				VD55G0_REG_16BIT(0x0464)
#define VD55G0_REG_Y_START				VD55G0_REG_16BIT(0x045a)
#define VD55G0_REG_Y_END				VD55G0_REG_16BIT(0x045c)
#define VD55G0_REG_AE_ROI_START_H			VD55G0_REG_16BIT(0x0436)
#define VD55G0_REG_AE_ROI_START_V			VD55G0_REG_16BIT(0x0438)
#define VD55G0_REG_AE_ROI_END_H				VD55G0_REG_16BIT(0x043a)
#define VD55G0_REG_AE_ROI_END_V				VD55G0_REG_16BIT(0x043c)
#define VD55G0_REG_GPIO_0_CTRL				VD55G0_REG_8BIT(0x0467)
#define VD55G0_REG_GPIO_1_CTRL				VD55G0_REG_8BIT(0x0468)
#define VD55G0_REG_GPIO_2_CTRL				VD55G0_REG_8BIT(0x0469)
#define VD55G0_REG_GPIO_3_CTRL				VD55G0_REG_8BIT(0x046a)
#define VD55G0_REG_READOUT_CTRL				VD55G0_REG_8BIT(0x047a)

#define VD55G0_WIDTH					644
#define VD55G0_HEIGHT					604
#define VD55G0_DEFAULT_MODE				0
#define VD55G0_WRITE_MULTIPLE_CHUNK_MAX			16
#define VD55G0_MIN_FRAME_LENGTH				(605 + 76)
#define VD55G0_FRAME_LENGTH_DEF				1980 /* 60 fps */
#define VD55G0_TIMEOUT_MS				500
#define VD55G0_MEDIA_BUS_FMT_DEF			MEDIA_BUS_FMT_Y8_1X8

#define V4L2_CID_GPIO0_MODE			(V4L2_CID_USER_BASE | 0x1010)
#define V4L2_CID_GPIO1_MODE			(V4L2_CID_USER_BASE | 0x1011)
#define V4L2_CID_GPIO2_MODE			(V4L2_CID_USER_BASE | 0x1012)
#define V4L2_CID_GPIO3_MODE			(V4L2_CID_USER_BASE | 0x1013)
#define V4L2_CID_TEMPERATURE			(V4L2_CID_USER_BASE | 0x1020)

#include "st-vd55g0_patch.c"

static const char * const vd55g0_test_pattern_menu[] = {
	"Disabled",
	"Solid",
	"Colorbar",
	"Gradbar",
	"Hgrey",
	"Vgrey",
	"Dgrey",
	"PN28",
};

static const char * const vd55g0_gpios_modes[] = {
	"disabled",
	"strobe envelope positive",
	"strobe envelope negative",
};

static const char * const vd55g0_supply_name[] = {
	"VCORE",
	"VDDIO",
	"VANA",
};

static const s64 link_freq[] = {
	/*
	 * MIPI output freq is 804Mhz / 2, as it uses both rising edge and
	 * falling edges to send data
	 */
	402000000ULL
};

enum vd55g0_bin_mode {
	VD55G0_BIN_MODE_NORMAL,
	VD55G0_BIN_MODE_DIGITAL_X2,
	VD55G0_BIN_MODE_DIGITAL_X4,
};

struct vd55g0_mode_info {
	u32 width;
	u32 height;
	enum vd55g0_bin_mode bin_mode;
	struct v4l2_rect crop;
	int is_isl;
};

struct vd55g0_fmt_desc {
	u32 code;
	u8 bpp;
	u8 data_type;
};

static const struct vd55g0_fmt_desc vd55g0_supported_codes[] = {
	{
		.code = MEDIA_BUS_FMT_Y8_1X8,
		.bpp = 8,
		.data_type = MIPI_CSI2_DT_RAW8,
	},
	{
		.code = MEDIA_BUS_FMT_Y10_1X10,
		.bpp = 10,
		.data_type = MIPI_CSI2_DT_RAW10,
	},
	/*
	 * The sensor is monochrome, but on some platforms such as the db410c
	 * the media pipeline does not support Y* formats. Trick it by sending
	 * it a bayer format instead.
	 */
	{
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.bpp = 8,
		.data_type = MIPI_CSI2_DT_RAW8,
	},
	{
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.bpp = 10,
		.data_type = MIPI_CSI2_DT_RAW10,
	},
};

//TODO ISL ?
static const struct vd55g0_mode_info vd55g0_mode_data[] = {
/* Uncomment once frame alignement bug is sorted out */
#if 0
	{
		.width = VD55G0_WIDTH,
		.height = VD55G0_HEIGHT,
		.bin_mode = VD55G0_BIN_MODE_NORMAL,
		.crop = {
			.left = 0,
			.top = 0,
			.width = VD55G0_WIDTH,
			.height = VD55G0_HEIGHT,
		},
	},
	{
		.width = VD55G0_WIDTH,
		.height = 600,
		.bin_mode = VD55G0_BIN_MODE_NORMAL,
		.crop = {
			.left = 0,
			.top = 2,
			.width = VD55G0_WIDTH,
			.height = 600,
		},
	},
#endif
	{
		.width = 640,
		.height = 480,
		.bin_mode = VD55G0_BIN_MODE_NORMAL,
		.crop = {
			.left = 2,
			.top = 62,
			.width = 640,
			.height = 480,
		},
	},
	{
		.width = 320,
		.height = 240,
		.bin_mode = VD55G0_BIN_MODE_DIGITAL_X2,
		.crop = {
			.left = 2,
			.top = 62,
			.width = 640,
			.height = 480,
		},
	},
};

enum vd55g0_expo_state {
	VD55G0_EXP_AUTO,
	VD55G0_EXP_FREEZE,
	VD55G0_EXP_MANUAL
};


struct vd55g0_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regulator_bulk_data supplies[ARRAY_SIZE(vd55g0_supply_name)];
	struct gpio_desc *reset_gpio;
	struct clk *xclk;
	u32 clk_freq;
	u16 oif_ctrl;
	int nb_of_lane;
	int data_rate_in_mbps;
	int pclk;
	u16 line_length;
	/* Lock to protect all members below */
	struct mutex lock;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct v4l2_ctrl *vflip_ctrl;
	struct v4l2_ctrl *hflip_ctrl;
	bool streaming;
	struct v4l2_mbus_framefmt fmt;
	const struct vd55g0_mode_info *current_mode;
	struct v4l2_fract frame_interval;
	bool hflip;
	bool vflip;
	int manual_expo_ms;
	enum vd55g0_expo_state expo_state;
	u16 digital_gain;
	u8 analog_gain;
	u16 vblank;
	u16 vblank_min;
	u16 frame_length;
	bool ae_frozen;
	u32 pattern;
};

static inline struct vd55g0_dev *to_vd55g0_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct vd55g0_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct vd55g0_dev,
		ctrl_handler)->sd;
}

static u8 get_bpp_by_code(__u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vd55g0_supported_codes); i++) {
		if (vd55g0_supported_codes[i].code == code)
			return vd55g0_supported_codes[i].bpp;
	}
	/* Should never happen */
	WARN(1, "Unsupported code %d. default to 8 bpp", code);
	return 8;
}

static u8 get_datatype_by_code(__u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vd55g0_supported_codes); i++) {
		if (vd55g0_supported_codes[i].code == code)
			return vd55g0_supported_codes[i].data_type;
	}
	/* Should never happen */
	WARN(1, "Unsupported code %d. default to MIPI_CSI2_DT_RAW8 data type",
	     code);
	return MIPI_CSI2_DT_RAW8;
}

static s32 get_pixel_rate(struct vd55g0_dev *sensor)
{
	return div64_u64((u64)sensor->data_rate_in_mbps * sensor->nb_of_lane,
			 get_bpp_by_code(sensor->fmt.code));
}

static int get_chunk_size(struct vd55g0_dev *sensor)
{
	int max_write_len = VD55G0_WRITE_MULTIPLE_CHUNK_MAX;
	struct i2c_adapter *adapter = sensor->i2c_client->adapter;

	if (adapter->quirks && adapter->quirks->max_write_len)
		max_write_len = adapter->quirks->max_write_len - 2;

	max_write_len = min(max_write_len, VD55G0_WRITE_MULTIPLE_CHUNK_MAX);

	return max(max_write_len, 1);
}

static int vd55g0_read_multiple(struct vd55g0_dev *sensor, u32 reg,
				unsigned int len)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg[2];
	u8 buf[2];
	u8 val[sizeof(u32)] = {0};
	int ret;

	if (len > sizeof(u32))
		return -EINVAL;
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = val;
	msg[1].len = len;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_dbg(&client->dev, "%s: %x i2c_transfer, reg: %x => %d\n",
			__func__, client->addr, reg, ret);
		return ret;
	}

	return get_unaligned_le32(val);
}

static inline int vd55g0_read_reg(struct vd55g0_dev *sensor, u32 reg)
{
	return vd55g0_read_multiple(sensor, reg & VD55G0_REG_ADDR_MASK,
				     (reg >> VD55G0_REG_SIZE_SHIFT) & 7);
}

static int vd55g0_write_multiple(struct vd55g0_dev *sensor, u32 reg,
				 const u8 *data, unsigned int len, int *err)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	u8 buf[VD55G0_WRITE_MULTIPLE_CHUNK_MAX + 2];
	unsigned int i;
	int ret;

	if (err && *err)
		return *err;

	if (len > VD55G0_WRITE_MULTIPLE_CHUNK_MAX)
		return -EINVAL;
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	for (i = 0; i < len; i++)
		buf[i + 2] = data[i];

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = len + 2;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_dbg(&client->dev, "%s: i2c_transfer, reg: %x => %d\n",
			__func__, reg, ret);
		if (err)
			*err = ret;
		return ret;
	}

	return 0;
}

static int vd55g0_write_array(struct vd55g0_dev *sensor, u32 reg,
			      unsigned int nb, const u8 *array)
{
	const unsigned int chunk_size = get_chunk_size(sensor);
	int ret;
	unsigned int sz;

	while (nb) {
		sz = min(nb, chunk_size);
		ret = vd55g0_write_multiple(sensor, reg, array, sz, NULL);
		if (ret < 0)
			return ret;
		nb -= sz;
		reg += sz;
		array += sz;
	}

	return 0;
}

static inline int vd55g0_write_reg(struct vd55g0_dev *sensor, u32 reg, u32 val,
				   int *err)
{
	return vd55g0_write_multiple(sensor, reg & VD55G0_REG_ADDR_MASK,
				     (u8 *)&val,
				     (reg >> VD55G0_REG_SIZE_SHIFT) & 7, err);
}

static int vd55g0_poll_reg(struct vd55g0_dev *sensor, u32 reg, u8 poll_val,
			   unsigned int timeout_ms)
{
	const unsigned int loop_delay_ms = 10;
	int ret;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 7, 0)
	int loop_nb = timeout_ms / loop_delay_ms;

	while (--loop_nb) {
		ret = vd55g0_read_reg(sensor, reg);
		if (ret < 0)
			return ret;
		if (ret == poll_val)
			return 0;
		msleep(loop_delay_ms);
	}
	return -ETIMEDOUT;
#else
	return read_poll_timeout(vd55g0_read_reg, ret,
				 ((ret < 0) || (ret == poll_val)),
				 loop_delay_ms * 1000, timeout_ms * 1000,
				 false, sensor, reg);
#endif
}

static int vd55g0_wait_state(struct vd55g0_dev *sensor, int state,
			     unsigned int timeout_ms)
{
	return vd55g0_poll_reg(sensor, VD55G0_REG_SYSTEM_FSM, state,
			       timeout_ms);
}

static int vd55g0_apply_exposure_auto(struct vd55g0_dev *sensor) {
	enum vd55g0_expo_state exp = sensor->expo_state;

	if (sensor->ae_frozen && sensor->expo_state == VD55G0_EXP_AUTO)
		exp = VD55G0_EXP_FREEZE;

	return vd55g0_write_reg(sensor, VD55G0_REG_EXP_MODE, exp, NULL);
}

static int vd55g0_get_regulators(struct vd55g0_dev *sensor)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vd55g0_supply_name); i++)
		sensor->supplies[i].supply = vd55g0_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       ARRAY_SIZE(vd55g0_supply_name),
				       sensor->supplies);
}

static int apply_exposure(struct vd55g0_dev *sensor)
{
	return vd55g0_write_reg(sensor, VD55G0_REG_MANUAL_COARSE_EXPOSURE,
				 sensor->manual_expo_ms, NULL);
}

static int vd55g0_apply_patgen(struct vd55g0_dev *sensor)
{
	static const u8 index2val[] = {
		0x0, 0x0, 0x8, 0x10, 0x20, 0x21, 0x22, 0x28
	};
	u32 pattern = index2val[sensor->pattern];
	u32 reg = pattern << VD55G0_PATGEN_TYPE_SHIFT;

	if (sensor->pattern != 0)
		reg |= VD55G0_PATGEN_ENABLE;
	return vd55g0_write_reg(sensor, VD55G0_REG_PATGEN_CTRL, reg, NULL);
}

static int vd55g0_update_patgen(struct vd55g0_dev *sensor, u32 pattern)
{
	sensor->pattern = pattern;

	if (sensor->streaming)
		return vd55g0_apply_patgen(sensor);
	return 0;
}

static int vd55g0_update_exposure_auto(struct vd55g0_dev *sensor, u32 index)
{
	int ret;

	switch (index) {
	case V4L2_EXPOSURE_AUTO:
		sensor->expo_state = VD55G0_EXP_AUTO;
		break;
	case V4L2_EXPOSURE_MANUAL:
		sensor->expo_state = VD55G0_EXP_MANUAL;
		break;
	default:
		/* Should never happen */
		ret = -EINVAL;
	}

	if (sensor->streaming)
		return vd55g0_apply_exposure_auto(sensor);
	return 0;
}

static int vd55g0_lock_exposure(struct vd55g0_dev *sensor, struct v4l2_ctrl *ctrl)
{
	/* Only exposure lock is supported */
	bool ae_lock = ctrl->val & V4L2_LOCK_EXPOSURE;
	int ret;

	sensor->ae_frozen = ae_lock;

	/* Cap control value to reflect the hardware state */
	ctrl->val = ae_lock;

	ret = vd55g0_apply_exposure_auto(sensor);
	if (ret)
		return ret;
	return 0;
}


static int vd55g0_update_gpiox_strobe_mode(struct vd55g0_dev *sensor, u32 mode,
					   int idx)
{
	u8 regs[ARRAY_SIZE(vd55g0_gpios_modes)] = {0x01, 0x02, 0x22};

	return vd55g0_write_reg(sensor, VD55G0_REG_GPIO_0_CTRL + idx,
				regs[mode], NULL);
}

static int vd55g0_get_temp_stream_enable(struct vd55g0_dev *sensor, int *temp)
{
	int temperature;

	temperature = vd55g0_read_reg(sensor, VD55G0_REG_TEMPERATURE);
	if (temperature < 0)
		return temperature;

	/* temperature is signed 10 bits value. extend sign */
	temperature = (temperature << 6) >> 6;
	*temp = temperature;

	return 0;
}

static int vd55g0_apply_framelength(struct vd55g0_dev *sensor)
{
	return vd55g0_write_reg(sensor, VD55G0_REG_FRAME_LENGTH,
				sensor->frame_length, NULL);
}

static int vd55g0_update_vblank(struct vd55g0_dev *sensor, u16 vblank)
{
	sensor->vblank_min = VD55G0_MIN_FRAME_LENGTH -
			     sensor->current_mode->crop.height;
	sensor->vblank = vblank;
	sensor->frame_length = sensor->current_mode->crop.height +
			       sensor->vblank;

	if (sensor->streaming)
		return vd55g0_apply_framelength(sensor);
	return 0;
}

static int vd55g0_get_temp_stream_disable(struct vd55g0_dev *sensor, int *temp)
{
	int ret;

	/* request temperature read */
	ret = vd55g0_write_reg(sensor, VD55G0_REG_SW_STBY,
			       VD55G0_SW_STBY_THSENS_READ, NULL);
	if (ret)
		return ret;
	ret = vd55g0_poll_reg(sensor, VD55G0_REG_SW_STBY, 0, VD55G0_TIMEOUT_MS);
	if (ret)
		return ret;

	return vd55g0_get_temp_stream_enable(sensor, temp);
}

static int vd55g0_get_temp(struct vd55g0_dev *sensor, int *temp)
{
	*temp = 0;
	if (sensor->streaming)
		return vd55g0_get_temp_stream_enable(sensor, temp);
	else
		return vd55g0_get_temp_stream_disable(sensor, temp);
}

static int vd55g0_update_analog_gain(struct vd55g0_dev *sensor, u32 target)
{
	sensor->analog_gain = target;

	if (sensor->streaming)
		return vd55g0_write_reg(sensor, VD55G0_REG_MANUAL_ANALOG_GAIN,
					target, NULL);
	return 0;
}

static int vd55g0_update_digital_gain(struct vd55g0_dev *sensor, u32 target)
{
	sensor->digital_gain = target;

	if (sensor->streaming)
		return vd55g0_write_reg(sensor, VD55G0_REG_MANUAL_DIGITAL_GAIN,
					target, NULL);
	return 0;
}

static int vd55g0_set_exposure(struct vd55g0_dev *sensor, int expo_ms)
{
	sensor->manual_expo_ms = expo_ms;
	if (sensor->streaming)
		return apply_exposure(sensor);

	return 0;
}

static int vd55g0_apply_reset(struct vd55g0_dev *sensor)
{
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(5000, 10000);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	usleep_range(5000, 10000);
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(5000, 10000);
	return vd55g0_wait_state(sensor, VD55G0_SYSTEM_FSM_READY_TO_BOOT,
				 VD55G0_TIMEOUT_MS);
}

static void vd55g0_fill_framefmt(struct vd55g0_dev *sensor,
				 const struct vd55g0_mode_info *mode,
				 struct v4l2_mbus_framefmt *fmt, u32 code)
{
	fmt->code = code;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->field = V4L2_FIELD_NONE;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int vd55g0_try_fmt_internal(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   const struct vd55g0_mode_info **new_mode)
{
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	const struct vd55g0_mode_info *mode = vd55g0_mode_data;
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(vd55g0_supported_codes); index++) {
		if (vd55g0_supported_codes[index].code == fmt->code)
			break;
	}
	if (index == ARRAY_SIZE(vd55g0_supported_codes))
		index = 0;

	mode = v4l2_find_nearest_size(vd55g0_mode_data,
				      ARRAY_SIZE(vd55g0_mode_data), width, height,
				      fmt->width, fmt->height);
	if (new_mode)
		*new_mode = mode;

	vd55g0_fill_framefmt(sensor, mode, fmt,
			     vd55g0_supported_codes[index].code);

	return 0;
}


static int vd55g0_apply_settings(struct vd55g0_dev *sensor)
{
	int ret;

	ret = vd55g0_apply_framelength(sensor);
	if (ret)
		return ret;

	ret = vd55g0_apply_exposure_auto(sensor);
	if (ret)
		return ret;

	ret = apply_exposure(sensor);
	if (ret)
		return ret;

	ret = vd55g0_write_reg(sensor, VD55G0_REG_MANUAL_ANALOG_GAIN,
			       sensor->analog_gain, NULL);
	if (ret)
		return ret;
	ret = vd55g0_write_reg(sensor, VD55G0_REG_MANUAL_DIGITAL_GAIN,
			       sensor->digital_gain, NULL);
	if (ret)
		return ret;

	ret = vd55g0_write_reg(sensor, VD55G0_REG_ORIENTATION,
			       sensor->hflip | (sensor->vflip << 1), NULL);
	if (ret)
		return ret;

	ret = vd55g0_apply_patgen(sensor);
	if (ret)
		return ret;

	return 0;
}

static int vd55g0_apply_frame_format(struct vd55g0_dev *sensor)
{
	const struct v4l2_rect *crop = &sensor->current_mode->crop;
	int ret = 0;

	vd55g0_write_reg(sensor, VD55G0_REG_READOUT_CTRL,
			 sensor->current_mode->bin_mode, &ret);
	vd55g0_write_reg(sensor, VD55G0_REG_ROI_X_START, crop->left, &ret);
	vd55g0_write_reg(sensor, VD55G0_REG_ROI_X_END,
			 crop->left + crop->width - 1, &ret);
	vd55g0_write_reg(sensor, VD55G0_REG_ROI_Y_START, crop->top, &ret);
	vd55g0_write_reg(sensor, VD55G0_REG_ROI_Y_END,
			 crop->top + crop->height - 1, &ret);

	/*
	 * Use the same auto exposure crop as the image crop, performing auto
	 * exposure computation only on image boundaries.
	 */
	vd55g0_write_reg(sensor, VD55G0_REG_AE_ROI_START_H, crop->left, &ret);
	vd55g0_write_reg(sensor, VD55G0_REG_AE_ROI_END_H,
			 crop->left + crop->width - 1, &ret);
	vd55g0_write_reg(sensor, VD55G0_REG_AE_ROI_START_V, crop->top, &ret);
	vd55g0_write_reg(sensor, VD55G0_REG_AE_ROI_END_V,
			 crop->top + crop->height - 1, &ret);

	/*
	 * Only aquire lines required for this image format, optimizing power
	 * usage
	 */
#if 0
	vd55g0_write_reg(sensor, VD55G0_REG_Y_START, crop->top - 50, &ret);
	vd55g0_write_reg(sensor, VD55G0_REG_Y_END,
			 crop->top + crop->height - 1, &ret);
#endif

	return ret;
}

static int vd55g0_stream_enable(struct vd55g0_dev *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	int is_isl = sensor->current_mode->is_isl;
	int ret;

	ret = pm_runtime_get_sync(&client->dev);
	if (ret < 0) {
		pm_runtime_put_autosuspend(&client->dev);
		return ret;
	}

	/* pm_runtime_get_sync() can return 1 as a valid return code */
	ret = 0;

	vd55g0_write_reg(sensor, VD55G0_REG_FORMAT_CTRL,
			 get_bpp_by_code(sensor->fmt.code), &ret);
	vd55g0_write_reg(sensor, VD55G0_REG_OIF_IMG_CTRL,
			 get_datatype_by_code(sensor->fmt.code), &ret);
	if (ret)
		goto err_rpm_put;

	ret = vd55g0_write_reg(sensor, VD55G0_REG_OIF_ISL_CTRL, is_isl ?
			       get_datatype_by_code(sensor->fmt.code) :
			       0x12, NULL);
	ret = vd55g0_write_reg(sensor, VD55G0_REG_ISL_ENABLE, is_isl, NULL);
	if (ret)
		goto err_rpm_put;

	ret = vd55g0_apply_frame_format(sensor);
	if (ret)
		goto err_rpm_put;

	ret = vd55g0_apply_settings(sensor);
	if (ret)
		goto err_rpm_put;

	ret = vd55g0_write_reg(sensor, VD55G0_REG_SW_STBY,
			       VD55G0_SW_STBY_START_STREAM, NULL);
	if (ret)
		goto err_rpm_put;

	ret = vd55g0_poll_reg(sensor, VD55G0_REG_SW_STBY, 0, VD55G0_TIMEOUT_MS);
	if (ret)
		goto err_rpm_put;

	ret = vd55g0_wait_state(sensor, VD55G0_SYSTEM_FSM_STREAMING,
				VD55G0_TIMEOUT_MS);
	if (ret)
		goto err_rpm_put;

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static int vd55g0_stream_disable(struct vd55g0_dev *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	int ret;

	ret = vd55g0_write_reg(sensor, VD55G0_REG_STREAMING,
			       VD55G0_STREAMING_STOP_STREAM, NULL);
	if (ret)
		goto err_str_dis;

	ret = vd55g0_poll_reg(sensor, VD55G0_REG_STREAMING, 0, 2000);
	if (ret)
		goto err_str_dis;

	ret = vd55g0_wait_state(sensor, VD55G0_SYSTEM_FSM_SW_STBY,
				VD55G0_TIMEOUT_MS);

err_str_dis:
	if (ret)
		WARN(1, "Can't disable stream");
	pm_runtime_put(&client->dev);

	return ret;
}

#if KERNEL_VERSION(4, 20, 0) > LINUX_VERSION_CODE
static int vd55g0_rx_from_ep(struct vd55g0_dev *sensor,
			     struct fwnode_handle *endpoint)
{
	struct i2c_client *client = sensor->i2c_client;
	struct v4l2_fwnode_endpoint *ep;
	u32 log2phy[3] = {~0, ~0, ~0};
	u32 phy2log[3] = {~0, ~0, ~0};
	int polarities[3] = {0, 0, 0};
	int l_nb;
	int p, l;
	int i;

	ep = v4l2_fwnode_endpoint_alloc_parse(endpoint);
	if (IS_ERR(ep))
		goto error_alloc;

	l_nb = ep->bus.mipi_csi2.num_data_lanes;
	if (l_nb != 1 && l_nb != 2) {
		dev_err(&client->dev, "invalid data lane number %d\n", l_nb);
		goto error_ep;
	}

	/* build  log2phy, phy2log and polarities from ep info */
	log2phy[0] = ep->bus.mipi_csi2.clock_lane;
	phy2log[log2phy[0]] = 0;
	for (l = 1; l < l_nb + 1; l++) {
		log2phy[l] = ep->bus.mipi_csi2.data_lanes[l - 1];
		phy2log[log2phy[l]] = l;
	}
	/*
	 * then fill remaining slots for every physical slot have something
	 * valid for hardware stuff.
	 */
	for (p = 0; p < 3; p++) {
		if (phy2log[p] != ~0)
			continue;
		phy2log[p] = l;
		log2phy[l] = p;
		l++;
	}
	for (l = 0; l < l_nb + 1; l++)
		polarities[l] = ep->bus.mipi_csi2.lane_polarities[l];

	if (log2phy[0] != 0) {
		dev_err(&client->dev, "clk lane must be map to physical lane 0\n");
		goto error_ep;
	}
	sensor->oif_ctrl = l_nb |
			   (polarities[0] << 3) |
			   ((phy2log[1] - 1) << 4) |
			   (polarities[1] << 6) |
			   ((phy2log[2] - 1) << 7) |
			   (polarities[2] << 9);
	sensor->nb_of_lane = l_nb;

	dev_dbg(&client->dev, "rx use %d lanes", l_nb);
	for (i = 0; i < 3; i++) {
		dev_dbg(&client->dev, "log2phy[%d] = %d", i, log2phy[i]);
		dev_dbg(&client->dev, "phy2log[%d] = %d", i, phy2log[i]);
		dev_dbg(&client->dev, "polarity[%d] = %d", i, polarities[i]);
	}
	dev_dbg(&client->dev, "oif_ctrl = 0x%04x\n", sensor->oif_ctrl);

	v4l2_fwnode_endpoint_free(ep);

	return 0;

error_ep:
	v4l2_fwnode_endpoint_free(ep);
error_alloc:

	return -EINVAL;
}
#else
static int vd55g0_rx_from_ep(struct vd55g0_dev *sensor,
			     struct fwnode_handle *endpoint)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct i2c_client *client = sensor->i2c_client;
	u32 log2phy[3] = {~0, ~0, ~0};
	u32 phy2log[3] = {~0, ~0, ~0};
	int polarities[3] = {0, 0, 0};
	int l_nb;
	int p, l;
	int ret;
	int i;

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	if (ret)
		goto error_alloc;

	l_nb = ep.bus.mipi_csi2.num_data_lanes;
	if (l_nb != 1 && l_nb != 2) {
		dev_err(&client->dev, "invalid data lane number %d\n", l_nb);
		goto error_ep;
	}

	/* build  log2phy, phy2log and polarities from ep info */
	log2phy[0] = ep.bus.mipi_csi2.clock_lane;
	phy2log[log2phy[0]] = 0;
	for (l = 1; l < l_nb + 1; l++) {
		log2phy[l] = ep.bus.mipi_csi2.data_lanes[l - 1];
		phy2log[log2phy[l]] = l;
	}
	/*
	 * then fill remaining slots for every physical slot have something
	 * valid for hardware stuff.
	 */
	for (p = 0; p < 3; p++) {
		if (phy2log[p] != ~0)
			continue;
		phy2log[p] = l;
		log2phy[l] = p;
		l++;
	}
	for (l = 0; l < l_nb + 1; l++)
		polarities[l] = ep.bus.mipi_csi2.lane_polarities[l];

	if (log2phy[0] != 0) {
		dev_err(&client->dev, "clk lane must be map to physical lane 0\n");
		goto error_ep;
	}
	sensor->oif_ctrl = l_nb |
			   (polarities[0] << 3) |
			   ((phy2log[1] - 1) << 4) |
			   (polarities[1] << 6) |
			   ((phy2log[2] - 1) << 7) |
			   (polarities[2] << 9);
	sensor->nb_of_lane = l_nb;

	dev_dbg(&client->dev, "rx use %d lanes", l_nb);
	for (i = 0; i < 3; i++) {
		dev_dbg(&client->dev, "log2phy[%d] = %d", i, log2phy[i]);
		dev_dbg(&client->dev, "phy2log[%d] = %d", i, phy2log[i]);
		dev_dbg(&client->dev, "polarity[%d] = %d", i, polarities[i]);
	}
	dev_dbg(&client->dev, "oif_ctrl = 0x%04x\n", sensor->oif_ctrl);

	v4l2_fwnode_endpoint_free(&ep);

	return 0;

error_ep:
	v4l2_fwnode_endpoint_free(&ep);
error_alloc:

	return -EINVAL;
}
#endif

static int vd55g0_patch(struct vd55g0_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	u16 patch;
	int ret;

	ret = vd55g0_write_array(sensor, 0x2000, sizeof(array_0x2000),
				 array_0x2000);
	if (ret)
		return ret;

	ret = vd55g0_write_reg(sensor, VD55G0_REG_BOOT,
			       VD55G0_BOOT_PATCH_SETUP, NULL);
	if (ret)
		return ret;

	ret = vd55g0_poll_reg(sensor, VD55G0_REG_BOOT, 0, VD55G0_TIMEOUT_MS);
	if (ret)
		return ret;

	patch = vd55g0_read_reg(sensor, VD55G0_REG_FWPATCH_REVISION);
	if (patch < 0)
		return patch;

	if (patch != (VD55G0_FWPATCH_REVISION_MAJOR << 8) +
	    VD55G0_FWPATCH_REVISION_MINOR) {
		dev_err(&client->dev, "bad patch version expected %d.%d got %d.%d",
			VD55G0_FWPATCH_REVISION_MAJOR,
			VD55G0_FWPATCH_REVISION_MINOR,
			patch >> 8, patch & 0xff);
		return -ENODEV;
	}
	dev_info(&client->dev, "patch %d.%d applied", patch >> 8, patch & 0xff);

	return 0;
}

static int vd55g0_boot(struct vd55g0_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	ret = vd55g0_write_reg(sensor, VD55G0_REG_BOOT, VD55G0_BOOT_BOOT, NULL);
	if (ret)
		return ret;

	ret = vd55g0_poll_reg(sensor, VD55G0_REG_BOOT, 0, VD55G0_TIMEOUT_MS);
	if (ret)
		return ret;

	ret = vd55g0_wait_state(sensor, VD55G0_SYSTEM_FSM_SW_STBY,
				VD55G0_TIMEOUT_MS);
	if (ret)
		return ret;

	dev_info(&client->dev, "sensor boot successfully");

	return 0;
}

static int vd55g0_configure(struct vd55g0_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	u32 mipi_bps = link_freq[0] * 2;
	unsigned int i;
	int line_length;
	int ret;

	line_length = vd55g0_read_reg(sensor, VD55G0_REG_LINE_LENGTH);
	if (line_length < 0)
		return line_length;
	sensor->line_length = line_length;
	/* configure clocks */
	ret = vd55g0_write_reg(sensor, VD55G0_REG_EXT_CLOCK,
				 sensor->clk_freq, NULL);
	/* Contrary to the fox, PLL_PREDIV and PLL_MULT are not accessible. We
	 * rely on the firmware to set the correct multiplier for the clock.
	 * Hence we don't do anything more here.
	 */
	/* configure interface */
	ret = vd55g0_write_reg(sensor, VD55G0_REG_OIF_CTRL, sensor->oif_ctrl, NULL);
	if (ret)
		return ret;
	ret = vd55g0_write_reg(sensor, VD55G0_REG_CLK_PLL_MIPI, mipi_bps, NULL);
	if (ret)
		return ret;
	ret = vd55g0_write_reg(sensor, VD55G0_REG_ISL_ENABLE, 0, NULL);
	if (ret)
		return ret;
	/* use auto expo by default */
	ret = vd55g0_write_reg(sensor, VD55G0_REG_EXP_MODE,
			       VD55G0_EXP_MODE_AUTO, NULL);
	if (ret)
		return ret;
	/* gpios in input (disabled) by default */
	for (i = 0; i < 8; i++) {
		ret = vd55g0_write_reg(sensor, VD55G0_REG_GPIO_0_CTRL + i,
				       0x01, NULL);
		if (ret)
			return ret;
	}

	sensor->data_rate_in_mbps = mipi_bps;
	sensor->pclk = (sensor->data_rate_in_mbps * 2) / 10;
	dev_info(&client->dev, "data rate = %d mbps",
		 sensor->data_rate_in_mbps);

	return 0;
}

static int vd55g0_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);

	ret = enable ? vd55g0_stream_enable(sensor) :
		       vd55g0_stream_disable(sensor);
	if (!ret)
		sensor->streaming = enable;

	mutex_unlock(&sensor->lock);

	if (!ret) {
		/* vflip and hflip cannot change during streaming */
		v4l2_ctrl_grab(sensor->vflip_ctrl, enable);
		v4l2_ctrl_grab(sensor->hflip_ctrl, enable);
	}

	return ret;
}

static int vd55g0_get_selection(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 15, 0)
				 struct v4l2_subdev_pad_config *cfg,
#else
				struct v4l2_subdev_state *sd_state,
#endif
				struct v4l2_subdev_selection *sel)
{
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = sensor->current_mode->crop;
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = VD55G0_WIDTH;
		sel->r.height = VD55G0_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int vd55g0_enum_mbus_code(struct v4l2_subdev *sd,
#if KERNEL_VERSION(5, 15, 0) > LINUX_VERSION_CODE
				 struct v4l2_subdev_pad_config *cfg,
#else
				 struct v4l2_subdev_state *sd_state,
#endif
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(vd55g0_supported_codes))
		return -EINVAL;

	code->code = vd55g0_supported_codes[code->index].code;

	return 0;
}

static int vd55g0_get_fmt(struct v4l2_subdev *sd,
#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
			  struct v4l2_subdev_pad_config *cfg,
#else
			  struct v4l2_subdev_state *sd_state,
#endif
			  struct v4l2_subdev_format *format)
{
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
#if KERNEL_VERSION(5, 15, 0) > LINUX_VERSION_CODE
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg,
						 format->pad);
#else
		fmt = v4l2_subdev_get_try_format(&sensor->sd, sd_state,
						 format->pad);
#endif
	else
		fmt = &sensor->fmt;

	format->format = *fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

static int vd55g0_set_fmt(struct v4l2_subdev *sd,
#if KERNEL_VERSION(5, 15, 0) > LINUX_VERSION_CODE
			  struct v4l2_subdev_pad_config *cfg,
#else
			  struct v4l2_subdev_state *sd_state,
#endif
			  struct v4l2_subdev_format *format)
{
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	const struct vd55g0_mode_info *new_mode;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	ret = vd55g0_try_fmt_internal(sd, &format->format, &new_mode);
	if (ret)
		goto out;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 15, 0)
		fmt = v4l2_subdev_get_try_format(sd, cfg, 0);
#else
		fmt = v4l2_subdev_get_try_format(sd, sd_state, 0);
#endif
		*fmt = format->format;
	} else if (sensor->current_mode != new_mode ||
		   sensor->fmt.code != format->format.code) {
		fmt = &sensor->fmt;
		*fmt = format->format;

		sensor->current_mode = new_mode;

		/* Reset vblank and framelength to default */
		ret = vd55g0_update_vblank(sensor,
					   VD55G0_FRAME_LENGTH_DEF -
					   new_mode->crop.height);

		/* Update controls to reflect new mode */
		__v4l2_ctrl_s_ctrl_int64(sensor->pixel_rate_ctrl,
					 get_pixel_rate(sensor));
		__v4l2_ctrl_modify_range(sensor->vblank_ctrl,
					 sensor->vblank_min,
					 0xffff - new_mode->crop.height,
					 1, sensor->vblank);
		__v4l2_ctrl_s_ctrl(sensor->vblank_ctrl, sensor->vblank);
	}

out:
	mutex_unlock(&sensor->lock);

	return ret;
}

static int vd55g0_init_cfg(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 15, 0)
			   struct v4l2_subdev_pad_config *cfg
#else
			   struct v4l2_subdev_state *sd_state
#endif
			   )
{
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	struct v4l2_subdev_format fmt = { 0 };

	vd55g0_fill_framefmt(sensor, sensor->current_mode, &fmt.format,
			     VD55G0_MEDIA_BUS_FMT_DEF);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 15, 0)
	return vd55g0_set_fmt(sd, cfg, &fmt);
#else
	return vd55g0_set_fmt(sd, sd_state, &fmt);
#endif
}


static int vd55g0_enum_frame_size(struct v4l2_subdev *sd,
#if KERNEL_VERSION(5, 15, 0) > LINUX_VERSION_CODE
				  struct v4l2_subdev_pad_config *cfg,
#else
				  struct v4l2_subdev_state *sd_state,
#endif
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(vd55g0_mode_data))
		return -EINVAL;

	fse->min_width = vd55g0_mode_data[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = vd55g0_mode_data[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static const struct v4l2_subdev_core_ops vd55g0_core_ops = {
};

static const struct v4l2_subdev_video_ops vd55g0_video_ops = {
	.s_stream = vd55g0_s_stream,
};

static const struct v4l2_subdev_pad_ops vd55g0_pad_ops = {
	.init_cfg = vd55g0_init_cfg,
	.enum_mbus_code = vd55g0_enum_mbus_code,
	.get_fmt = vd55g0_get_fmt,
	.set_fmt = vd55g0_set_fmt,
	.get_selection = vd55g0_get_selection,
	.enum_frame_size = vd55g0_enum_frame_size,
};

static const struct v4l2_subdev_ops vd55g0_subdev_ops = {
	.core = &vd55g0_core_ops,
	.video = &vd55g0_video_ops,
	.pad = &vd55g0_pad_ops,
};

static const struct media_entity_operations vd55g0_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int vd55g0_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	int temperature;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_PIXEL_RATE:
		ret = __v4l2_ctrl_s_ctrl_int64(ctrl, get_pixel_rate(sensor));
		break;
	case V4L2_CID_TEMPERATURE:
		ret = vd55g0_get_temp(sensor, &temperature);
		if (ret)
			break;
		ret = __v4l2_ctrl_s_ctrl(ctrl, temperature);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int vd55g0_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		if (sensor->streaming) {
			ret = -EBUSY;
			break;
		}
		if (ctrl->id == V4L2_CID_VFLIP)
			sensor->vflip = ctrl->val;
		if (ctrl->id == V4L2_CID_HFLIP)
			sensor->hflip = ctrl->val;
		ret = 0;
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = vd55g0_update_patgen(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = vd55g0_update_exposure_auto(sensor, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = vd55g0_update_analog_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = vd55g0_update_digital_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = vd55g0_set_exposure(sensor, ctrl->val);
		break;
	case V4L2_CID_3A_LOCK:
		ret = vd55g0_lock_exposure(sensor, ctrl);
		break;
	case V4L2_CID_GPIO0_MODE:
	case V4L2_CID_GPIO1_MODE:
	case V4L2_CID_GPIO2_MODE:
	case V4L2_CID_GPIO3_MODE:
		ret = vd55g0_update_gpiox_strobe_mode(sensor, ctrl->val,
			ctrl->id - V4L2_CID_GPIO0_MODE);
		break;
	case V4L2_CID_VBLANK:
		ret = vd55g0_update_vblank(sensor, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops vd55g0_ctrl_ops = {
	.g_volatile_ctrl = vd55g0_g_volatile_ctrl,
	.s_ctrl = vd55g0_s_ctrl,
};

static const struct v4l2_ctrl_config vd55g0_gpio0_ctrl = {
	.ops		= &vd55g0_ctrl_ops,
	.id		= V4L2_CID_GPIO0_MODE,
	.name		= "Gpio0 mode",
	.type		= V4L2_CTRL_TYPE_MENU,
	.max		= ARRAY_SIZE(vd55g0_gpios_modes) - 1,
	.qmenu		= vd55g0_gpios_modes,
};

static const struct v4l2_ctrl_config vd55g0_gpio1_ctrl = {
	.ops		= &vd55g0_ctrl_ops,
	.id		= V4L2_CID_GPIO1_MODE,
	.name		= "Gpio1 mode",
	.type		= V4L2_CTRL_TYPE_MENU,
	.max		= ARRAY_SIZE(vd55g0_gpios_modes) - 1,
	.qmenu		= vd55g0_gpios_modes,
};

static const struct v4l2_ctrl_config vd55g0_gpio2_ctrl = {
	.ops		= &vd55g0_ctrl_ops,
	.id		= V4L2_CID_GPIO2_MODE,
	.name		= "Gpio2 mode",
	.type		= V4L2_CTRL_TYPE_MENU,
	.max		= ARRAY_SIZE(vd55g0_gpios_modes) - 1,
	.qmenu		= vd55g0_gpios_modes,
};

static const struct v4l2_ctrl_config vd55g0_gpio3_ctrl = {
	.ops		= &vd55g0_ctrl_ops,
	.id		= V4L2_CID_GPIO3_MODE,
	.name		= "Gpio3 mode",
	.type		= V4L2_CTRL_TYPE_MENU,
	.max		= ARRAY_SIZE(vd55g0_gpios_modes) - 1,
	.qmenu		= vd55g0_gpios_modes,
};
static const struct v4l2_ctrl_config vd55g0_temp_ctrl = {
	.ops		= &vd55g0_ctrl_ops,
	.id		= V4L2_CID_TEMPERATURE,
	.name		= "Temperature in celsius",
	.type		= V4L2_CTRL_TYPE_INTEGER,
	.min		= -1024,
	.max		= 1023,
	.step		= 1,
};

static int vd55g0_init_controls(struct vd55g0_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &vd55g0_ctrl_ops;
	struct v4l2_ctrl_handler *hdl = &sensor->ctrl_handler;
	const struct vd55g0_mode_info *cur_mode = sensor->current_mode;
	struct v4l2_ctrl *ctrl;
	int ret;

	v4l2_ctrl_handler_init(hdl, 16);
	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;
	v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(vd55g0_test_pattern_menu) - 1,
				     0, 0, vd55g0_test_pattern_menu);
	ctrl = v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(link_freq) - 1, 0, link_freq);
	ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_EXPOSURE_AUTO, 1, ~0x3,
			       V4L2_EXPOSURE_AUTO);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN, 0, 24, 1,
			  sensor->analog_gain);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_DIGITAL_GAIN, 0, 0xfff, 1,
			  sensor->digital_gain); //TODO better bounds
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE, 0, 0xffff, 1, 10);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_3A_LOCK, 0, 1, 0, 0);
	v4l2_ctrl_new_custom(hdl, &vd55g0_gpio0_ctrl, NULL);
	v4l2_ctrl_new_custom(hdl, &vd55g0_gpio1_ctrl, NULL);
	v4l2_ctrl_new_custom(hdl, &vd55g0_gpio2_ctrl, NULL);
	v4l2_ctrl_new_custom(hdl, &vd55g0_gpio3_ctrl, NULL);
	ctrl = v4l2_ctrl_new_custom(hdl, &vd55g0_temp_ctrl, NULL);
	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY;
	ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK, 0,
				 sensor->line_length, 1,
				 sensor->line_length - cur_mode->width);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Keep a pointer to these controls as we need to update them when
	 * setting the format
	 */
	sensor->pixel_rate_ctrl = v4l2_ctrl_new_std(hdl, ops,
						    V4L2_CID_PIXEL_RATE, 1,
						    INT_MAX, 1,
						    get_pixel_rate(sensor));
	if (sensor->pixel_rate_ctrl)
		sensor->pixel_rate_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->vblank_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
						sensor->vblank_min,
						0xffff - cur_mode->crop.height,
						1, sensor->vblank);
	sensor->vflip_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP,
					       0, 1, 1, sensor->vflip);
	sensor->hflip_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP,
					       0, 1, 1, sensor->hflip);
	v4l2_ctrl_grab(sensor->vflip_ctrl, false);
	v4l2_ctrl_grab(sensor->hflip_ctrl, false);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int vd55g0_detect(struct vd55g0_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int id = 0;

	id = vd55g0_read_reg(sensor, VD55G0_REG_MODEL_ID);
	if (id < 0)
		return id;

	if (id != VD55G0_MODEL_ID) {
		dev_warn(&client->dev, "Unsupported sensor id %x", id);
		return -ENODEV;
	}

	return 0;
}

/* Power/clock management functions */
static int vd55g0_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(vd55g0_supply_name),
				    sensor->supplies);
	if (ret) {
		dev_err(&client->dev, "failed to enable regulators %d", ret);
		return ret;
	}

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "failed to enable clock %d", ret);
		goto disable_bulk;
	}

	if (sensor->reset_gpio) {
		ret = vd55g0_apply_reset(sensor);
		if (ret) {
			dev_err(&client->dev, "sensor reset failed %d\n", ret);
			goto disable_clock;
		}
	}

	ret = vd55g0_detect(sensor);
	if (ret) {
		dev_err(&client->dev, "sensor detect failed %d", ret);
		goto disable_clock;
	}

	ret = vd55g0_patch(sensor);
	if (ret) {
		dev_err(&client->dev, "sensor patch failed %d", ret);
		goto disable_clock;
	}

	ret = vd55g0_boot(sensor);
	if (ret) {
		dev_err(&client->dev, "sensor boot failed %d", ret);
		goto disable_clock;
	}

	ret = vd55g0_configure(sensor);
	if (ret) {
		dev_err(&client->dev, "sensor configuration failed %d", ret);
		goto disable_clock;
	}

	return 0;

disable_clock:
	clk_disable_unprepare(sensor->xclk);
disable_bulk:
	regulator_bulk_disable(ARRAY_SIZE(vd55g0_supply_name),
			       sensor->supplies);

	return ret;
}

static int vd55g0_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);

	clk_disable_unprepare(sensor->xclk);
	regulator_bulk_disable(ARRAY_SIZE(vd55g0_supply_name),
			       sensor->supplies);
	return 0;
}

static int vd55g0_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct vd55g0_dev *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->analog_gain = 0;
	sensor->digital_gain = 256;

	sensor->i2c_client = client;
	sensor->streaming = false;
	sensor->fmt.code = MEDIA_BUS_FMT_SGBRG8_1X8;
	sensor->fmt.field = V4L2_FIELD_NONE;
	sensor->fmt.colorspace = V4L2_COLORSPACE_SRGB;
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = 15;
	sensor->manual_expo_ms = 10;
	sensor->vflip = false;
	sensor->hflip = false;

	sensor->current_mode = &vd55g0_mode_data[VD55G0_DEFAULT_MODE];

	endpoint = fwnode_graph_get_next_endpoint(
		of_fwnode_handle(dev->of_node), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = vd55g0_rx_from_ep(sensor, endpoint);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Failed to parse endpoint %d\n", ret);
		return ret;
	}

	sensor->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(sensor->xclk);
	}
	sensor->clk_freq = clk_get_rate(sensor->xclk);
	if (sensor->clk_freq < 6 * HZ_PER_MHZ ||
	    sensor->clk_freq > 27 * HZ_PER_MHZ) {
		dev_err(dev, "Only 6Mhz-27Mhz clock range supported. provide %lu MHz\n",
			sensor->clk_freq / HZ_PER_MHZ);
		return -EINVAL;
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &vd55g0_subdev_ops);
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.ops = &vd55g0_subdev_entity_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	ret = vd55g0_get_regulators(sensor);
	if (ret) {
		dev_err(&client->dev, "failed to get regulators %d", ret);
		return ret;
	}

	ret = vd55g0_power_on(dev);
	if (ret)
		return ret;

	vd55g0_fill_framefmt(sensor, sensor->current_mode, &sensor->fmt,
			     VD55G0_MEDIA_BUS_FMT_DEF);

	mutex_init(&sensor->lock);

	ret = vd55g0_update_vblank(sensor, VD55G0_FRAME_LENGTH_DEF -
				   sensor->current_mode->crop.height);
	if (ret)
		goto error_power_off;

	ret = vd55g0_init_controls(sensor);
	if (ret) {
		dev_err(&client->dev, "controls initialization failed %d", ret);
		goto error_power_off;
	}

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret) {
		dev_err(&client->dev, "pads init failed %d", ret);
		goto error_handler_free;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret) {
		dev_err(&client->dev, "async subdev register failed %d", ret);
		goto error_pm_runtime;
	}

	pm_runtime_set_autosuspend_delay(&client->dev, 1000);
	pm_runtime_use_autosuspend(&client->dev);

	dev_info(&client->dev, "vd55g0 probe successfully");

	return 0;

error_pm_runtime:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	media_entity_cleanup(&sensor->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(sensor->sd.ctrl_handler);
error_power_off:
	mutex_destroy(&sensor->lock);
	vd55g0_power_off(dev);

	return ret;
}

static int vd55g0_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		vd55g0_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct of_device_id vd55g0_dt_ids[] = {
	{ .compatible = "st,st-vd55g0" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vd55g0_dt_ids);

static const struct dev_pm_ops vd55g0_pm_ops = {
	SET_RUNTIME_PM_OPS(vd55g0_power_off, vd55g0_power_on, NULL)
};

static struct i2c_driver vd55g0_i2c_driver = {
	.driver = {
		.name  = "st-vd55g0",
		.of_match_table = vd55g0_dt_ids,
		.pm = &vd55g0_pm_ops,
	},
	.probe_new = vd55g0_probe,
	.remove = vd55g0_remove,
};

module_i2c_driver(vd55g0_i2c_driver);

MODULE_AUTHOR("Benjamin Mugnier <benjamin.mugnier@st.com>");
MODULE_AUTHOR("Mickael Guene <mickael.guene@st.com>");
MODULE_DESCRIPTION("VD55G0 camera subdev driver");
MODULE_LICENSE("GPL v2");
