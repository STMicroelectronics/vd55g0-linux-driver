// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for VD55G0 global shutter sensor
 *
 * Copyright (C) STMicroelectronics SA 2019
 * Authors: Mickael Guene <mickael.guene@st.com>
 *          for STMicroelectronics.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include <linux/version.h>

#define WRITE_MULTIPLE_CHUNK_MAX			16

#define DEVICE_MODEL_ID_REG				0x0000
#define VD55G0_MODEL_ID					0x4730
#define DEVICE_FWPATCH_REVISION				0x0022
#define DEVICE_SYSTEM_FSM				0x002c
#define SENSOR_READY_TO_BOOT				0x01
#define SENSOR_SW_STBY					0x02
#define SENSOR_STREAMING				0x03
#define DEVICE_TEMPERATURE				0x004c
#define DEVICE_BOOT					0x0200
#define CMD_BOOT					1
#define CMD_PATCH_SETUP					2
#define DEVICE_SW_STBY					0x0201
#define CMD_START_STREAM				1
#define CMD_THSENS_READ					4
#define DEVICE_STREAMING				0x0202
#define CMD_STOP_STREAM					1
#define DEVICE_EXT_CLOCK				0x0220
#define DEVICE_LINE_LENGTH				0x0300
#define DEVICE_ORIENTATION				0x0302
#define DEVICE_FORMAT_CTRL				0x030a
#define DEVICE_OIF_CTRL					0x030c
#define DEVICE_OIF_IMG_CTRL				0x030f
#define DEVICE_OIF_ISL_CTRL				0x0310
#define DEVICE_OIF_CSI_BITRATE				0x0224  //Changed from Statics to sensor settings 
#define DEVICE_ISL_ENABLE				0x0329
#define DEVICE_PATGEN_CTRL				0x0400
#define DEVICE_EXP_MODE					0x044c
#define DEVICE_MANUAL_ANALOG_GAIN			0x044d
#define DEVICE_MANUAL_COARSE_EXPOSURE			0x044e
#define DEVICE_MANUAL_DIGITAL_GAIN			0x0450
#define EXP_MODE_AUTO					0
#define EXP_MODE_FREEZE					1
#define EXP_MODE_MANUAL					2
#define DEVICE_FRAME_LENGTH				0x0458
#define DEVICE_ROI_X_START				0x045e
#define DEVICE_ROI_X_END				0x0460
#define DEVICE_ROI_Y_START				0x0462
#define DEVICE_ROI_Y_END				0x0464
#define DEVICE_GPIO_0_CTRL				0x0467
#define DEVICE_GPIO_1_CTRL				0x0468
#define DEVICE_GPIO_2_CTRL				0x0469
#define DEVICE_GPIO_3_CTRL				0x046a
#define DEVICE_READOUT_CTRL				0x047a

#define SENSOR_WIDTH					640
#define SENSOR_HEIGHT					600

#define V4L2_CID_GPIO0_MODE			(V4L2_CID_USER_BASE | 0x1010)
#define V4L2_CID_GPIO1_MODE			(V4L2_CID_USER_BASE | 0x1011)
#define V4L2_CID_GPIO2_MODE			(V4L2_CID_USER_BASE | 0x1012)
#define V4L2_CID_GPIO3_MODE			(V4L2_CID_USER_BASE | 0x1013)

#define V4L2_CID_TEMPERATURE			(V4L2_CID_USER_BASE | 0x1020)

#include "st-vd55g0_patch.c"

static const char * const vd55g0_test_pattern_menu[] = {
	"Disabled", "Solid", "Colorbar", "Gradbar",
	"Hgrey", "Vgrey", "Dgrey", "PN28"
};

static const char * const vd55g0_gpios_modes[] = {
	"disabled",
	"strobe envelope positive",
	"strobe envelope negative",
};

/* regulator supplies */
static const char * const vd55g0_supply_name[] = {
	"VCORE",
	"VDDIO",
	"VANA",
};

#define VD55G0_NUM_SUPPLIES		ARRAY_SIZE(vd55g0_supply_name)

static const s64 link_freq[] = {
	402000000ULL
};

/* macro to convert index to 8.8 fixed point gain */
#define I2FP(i)				(u32)(8192.0 / (32 - (i)))
/* array of possibles analog gains in 8.8 fixed point */
static const u16 analog_gains[29] = {
	I2FP(0), I2FP(1), I2FP(2), I2FP(3),
	I2FP(4), I2FP(5), I2FP(6), I2FP(7),
	I2FP(8), I2FP(9), I2FP(10), I2FP(11),
	I2FP(12), I2FP(13), I2FP(14), I2FP(15),
	I2FP(16), I2FP(17), I2FP(18), I2FP(19),
	I2FP(20), I2FP(21), I2FP(22), I2FP(23),
	I2FP(24), I2FP(25), I2FP(26), I2FP(27),
	I2FP(28)
};

struct vd55g0_mode_info {
	u32 width;
	u32 height;
	int bin_mode;
	int is_isl;
};

static const u32 vd55g0_supported_codes[] = {
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGBRG10_1X10
};

const int vd55g0_sensor_frame_rates[] = { 90, 60, 50, 30, 25, 15, 10, 5, 1 };

static const struct vd55g0_mode_info vd55g0_mode_data[] = {
	{ 640,  600, 0, 0},{ 640,  482, 0, 1},
	{ 640,  480, 0, 0},{ 480,  642, 1, 1},
	{ 480,  640, 1, 0},{ 320,  242, 1, 1},
	{ 320,  240, 1, 0},{ 240,  320, 2, 0},
};

enum vd55g0_expo_state {
	VD55G0_EXPO_AUTO,
	VD55G0_EXPO_AUTO_FREEZE,
	VD55G0_EXPO_MANUAL
};

struct vd55g0_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regulator_bulk_data supplies[VD55G0_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwr_reset;
	struct clk *xclk;
	u32 clk_freq;
	u16 oif_ctrl;
	int nb_of_lane;
	int data_rate_in_mbps;
	int pclk;
	u16 line_length;
	/* lock to protect all members below */
	struct mutex lock;
	struct v4l2_ctrl_handler ctrl_handler;
	bool streaming;
	struct v4l2_mbus_framefmt fmt;
	const struct vd55g0_mode_info *current_mode;
	struct v4l2_fract frame_interval;
	bool hflip;
	bool vflip;
	int manual_expo_ms;
	enum vd55g0_expo_state expo_state;
};

/* helpers */
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
	switch (code) {
	case MEDIA_BUS_FMT_SGBRG8_1X8:
		return 8;
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		return 10;
	}

	WARN(1, "Unsupported code %d. default to 8 bpp", code);

	return 8;
}

static u8 get_datatype_by_code(__u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SGBRG8_1X8:
		return 0x2a;
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		return 0x2b;
	}

	WARN(1, "Unsupported code %d. default to 0x2a data type", code);

	return 0x2a;
}

static void compute_pll_parameters_by_freq(u32 freq, unsigned int *prediv,
					   unsigned int *mult)
{
	const unsigned int predivs[] = {1, 2, 4};
	int i;

	/*
	 * freq range is [6Mhz-27Mhz] already checked.
	 * output of divider should be in [6Mhz-12Mhz[.
	 */
	for (i = 0; i < ARRAY_SIZE(predivs); i++) {
		*prediv = predivs[i];
		if (freq / *prediv < 12000000)
			break;
	}
	BUG_ON(i == ARRAY_SIZE(predivs));

	/*
	 * target freq is 804Mhz. Don't change this as it will impact image
	 * quality.
	 */
	*mult = (804000000U * (*prediv) + freq / 2) / freq;
}

static s32 get_pixel_rate(struct vd55g0_dev *sensor)
{
	return div64_u64((u64)sensor->data_rate_in_mbps * sensor->nb_of_lane,
			 get_bpp_by_code(sensor->fmt.code));
}

static int get_chunk_size(struct vd55g0_dev *sensor)
{
	int max_write_len = WRITE_MULTIPLE_CHUNK_MAX;
	struct i2c_adapter *adapter = sensor->i2c_client->adapter;

	if (adapter->quirks && adapter->quirks->max_write_len)
		max_write_len = adapter->quirks->max_write_len - 2;

	max_write_len = min(max_write_len, WRITE_MULTIPLE_CHUNK_MAX);

	return max(max_write_len, 1);
}

static int vd55g0_read_reg(struct vd55g0_dev *sensor, u16 reg, u8 *val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = val;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_dbg(&client->dev, "%s: %x i2c_transfer, reg: %x => %d\n",
			__func__, client->addr, reg, ret);
		return ret;
	}

	return 0;
}

static int vd55g0_read_reg16(struct vd55g0_dev *sensor, u16 reg, u16 *val)
{
	u8 hi, lo;
	int ret;

	ret = vd55g0_read_reg(sensor, reg, &lo);
	if (ret)
		return ret;
	ret = vd55g0_read_reg(sensor, reg + 1, &hi);
	if (ret)
		return ret;
	*val = ((u16)hi << 8) | (u16)lo;

	return 0;
}

static int vd55g0_write_reg(struct vd55g0_dev *sensor, u16 reg, u8 val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_dbg(&client->dev, "%s: i2c_transfer, reg: %x => %d\n",
			__func__, reg, ret);
		return ret;
	}

	return 0;
}

static int vd55g0_write_reg16(struct vd55g0_dev *sensor, u16 reg, u16 val)
{
	int ret;

	ret = vd55g0_write_reg(sensor, reg, val & 0xff);
	if (ret)
		return ret;

	return vd55g0_write_reg(sensor, reg + 1, val >> 8);
}

static int vd55g0_write_reg32(struct vd55g0_dev *sensor, u16 reg, u32 val)
{
	int ret;

	ret = vd55g0_write_reg16(sensor, reg, val & 0xffff);
	if (ret)
		return ret;

	return vd55g0_write_reg16(sensor, reg + 2, val >> 16);
}

static int vd55g0_write_multiple(struct vd55g0_dev *sensor, u16 reg,
				 const u8 *data, int len)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	u8 buf[WRITE_MULTIPLE_CHUNK_MAX + 2];
	int i;
	int ret;

	if (len > WRITE_MULTIPLE_CHUNK_MAX)
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
		return ret;
	}

	return 0;
}

static int vd55g0_write_array(struct vd55g0_dev *sensor, u16 reg, int nb,
			      const u8 *array)
{
	const int chunk_size = get_chunk_size(sensor);
	int ret;
	int sz;

	while (nb) {
		sz = min(nb, chunk_size);
		ret = vd55g0_write_multiple(sensor, reg, array, sz);
		if (ret < 0)
			return ret;
		nb -= sz;
		reg += sz;
		array += sz;
	}

	return 0;
}

static int vd55g0_poll_reg(struct vd55g0_dev *sensor, u16 reg, u8 poll_val)
{
	struct i2c_client *client = sensor->i2c_client;
	const int loop_delay_ms = 10;
	const int timeout_ms = 500;
	int loop_nb = timeout_ms / loop_delay_ms;
	u8 val;
	int ret;

	while (--loop_nb) {
		ret = vd55g0_read_reg(sensor, reg, &val);
		if (ret)
			return ret;
		dev_dbg(&client->dev,  "%s: got %d / waiting %d => ret = %d",
			__func__, val, poll_val, ret);
		if (val == poll_val)
			break;
		msleep(loop_delay_ms);
	}
	if (!loop_nb)
		return -ETIMEDOUT;

	return ret;
}

static int vd55g0_wait_state(struct vd55g0_dev *sensor, int state)
{
	return vd55g0_poll_reg(sensor, DEVICE_SYSTEM_FSM, state);
}

static int vd55g0_get_regulators(struct vd55g0_dev *sensor)
{
	int i;

	for (i = 0; i < VD55G0_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = vd55g0_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       VD55G0_NUM_SUPPLIES,
				       sensor->supplies);
}

static bool is_expo_valid(struct vd55g0_dev *sensor, int frame_length,
			  int line_n, int *expo_ms)
{
	/* FIXME : formulae need to be updated */
	if (line_n < frame_length - 100)
		return true;

	*expo_ms = *expo_ms - 1;

	return false;
}

static int apply_exposure(struct vd55g0_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	u16 frame_length;
	int line_duration_ns;
	int expo_line_nb;
	int ret;
	int expo_ms = sensor->manual_expo_ms;

	dev_dbg(&client->dev, "%s request expo %d ms", __func__, expo_ms);
	ret = vd55g0_read_reg16(sensor, DEVICE_FRAME_LENGTH, &frame_length);
	if (ret)
		return ret;
	line_duration_ns = div64_u64((u64)sensor->line_length * 1000000000,
				     sensor->pclk);

	do {
		expo_line_nb = (expo_ms * 1000000 + line_duration_ns / 2) /
				    line_duration_ns;
		expo_line_nb = max(1, expo_line_nb);
	} while (!is_expo_valid(sensor, frame_length, expo_line_nb, &expo_ms));

	ret = vd55g0_write_reg16(sensor, DEVICE_MANUAL_COARSE_EXPOSURE,
				 expo_line_nb);
	if (ret)
		return ret;

	dev_dbg(&client->dev, "%s applied expo %d ms", __func__, expo_ms);
	sensor->manual_expo_ms = expo_ms;

	return 0;
}

static int vd55g0_update_patgen(struct vd55g0_dev *sensor, u32 index)
{
	u32 pattern = index <= 3 ? index : index + 12;
	u16 reg;

	reg = pattern << 4;
	if (index)
		reg |= 1;

	return vd55g0_write_reg16(sensor, DEVICE_PATGEN_CTRL, reg);
}

static int vd55g0_update_exposure_auto(struct vd55g0_dev *sensor, u32 index)
{
	int ret;

	/* VD55G0_EXPO_AUTO_FREEZE => VD55G0_EXPO_MANUAL is invalid */
	if (sensor->expo_state == VD55G0_EXPO_AUTO_FREEZE &&
	    index == V4L2_EXPOSURE_MANUAL)
		return -EINVAL;

	switch (index) {
	case V4L2_EXPOSURE_AUTO:
		ret = vd55g0_write_reg(sensor, DEVICE_EXP_MODE, EXP_MODE_AUTO);
		sensor->expo_state = VD55G0_EXPO_AUTO;
		break;
	case V4L2_EXPOSURE_MANUAL:
		ret = vd55g0_write_reg(sensor, DEVICE_EXP_MODE,
				       EXP_MODE_MANUAL);
		sensor->expo_state = VD55G0_EXPO_MANUAL;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int vd55g0_lock_exposure(struct vd55g0_dev *sensor, u32 is_lock)
{
	/* only exposure lock is supported */
	if ((is_lock & 1) != is_lock)
		return -EINVAL;

	/* we can't lock / unlock if we are in manual mode */
	if (sensor->expo_state == VD55G0_EXPO_MANUAL)
		return -EINVAL;

	return vd55g0_write_reg(sensor, DEVICE_EXP_MODE,
				is_lock ? EXP_MODE_FREEZE : EXP_MODE_AUTO);
}

static int vd55g0_update_gpiox_strobe_mode(struct vd55g0_dev *sensor, u32 mode,
					   int idx)
{
	u8 regs[ARRAY_SIZE(vd55g0_gpios_modes)] = {0x01, 0x02, 0x22};

	return vd55g0_write_reg(sensor, DEVICE_GPIO_0_CTRL + idx, regs[mode]);
}

static int vd55g0_get_temp_stream_enable(struct vd55g0_dev *sensor, int *temp)
{
	int ret;
	int16_t temperature;

	ret = vd55g0_read_reg16(sensor, DEVICE_TEMPERATURE,
				(u16 *) &temperature);
	if (ret)
		return ret;
	/* temperature is signed 10 bits value. extend sign */
	temperature = (temperature << 6) >> 6;
	*temp = temperature;

	return 0;
}

static int vd55g0_get_temp_stream_disable(struct vd55g0_dev *sensor, int *temp)
{
	int ret;

	/* request temperature read */
	ret = vd55g0_write_reg(sensor, DEVICE_SW_STBY, CMD_THSENS_READ);
	if (ret)
		return ret;
	ret = vd55g0_poll_reg(sensor, DEVICE_SW_STBY, 0);
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

static int vd55g0_update_gains(struct vd55g0_dev *sensor, u32 target)
{
	struct i2c_client *client = sensor->i2c_client;
	unsigned int idx;
	u16 digital_gain;
	int ret;

	/* find smallest analog gains which is above or equal to target gain */
	for (idx = 0; idx < ARRAY_SIZE(analog_gains); idx++) {
		if (analog_gains[idx] >= target)
			break;
	}
	if (idx == ARRAY_SIZE(analog_gains))
		idx--;

	/* adjust gigital gain to match target gain */
	digital_gain = (target * 256 + analog_gains[idx] / 2) /
		       analog_gains[idx];

	/* applied gains */
	ret = vd55g0_write_reg(sensor, DEVICE_MANUAL_ANALOG_GAIN, idx);
	if (ret)
		return ret;
	ret = vd55g0_write_reg16(sensor, DEVICE_MANUAL_DIGITAL_GAIN,
				 digital_gain);
	if (ret)
		return ret;

	dev_dbg(&client->dev, "Target gain  is 0x%04x", target);
	dev_dbg(&client->dev, "      analog is 0x%04x", analog_gains[idx]);
	dev_dbg(&client->dev, "     digital is 0x%04x", digital_gain);
	dev_dbg(&client->dev, "Applied gain is 0x%04x",
		(analog_gains[idx] * digital_gain) / 256);

	return 0;
}

static int vd55g0_set_exposure(struct vd55g0_dev *sensor, int expo_ms)
{
	sensor->manual_expo_ms = expo_ms;
	if (sensor->streaming)
		return apply_exposure(sensor);

	return 0;
}

static void vd55g0_apply_reset(struct vd55g0_dev *sensor)
{
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(5000, 10000);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	usleep_range(5000, 10000);
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(5000, 10000);
	gpiod_set_value_cansleep(sensor->pwr_reset, 1);
}

static int vd55g0_detect(struct vd55g0_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	u16 id = 0;
	int ret;

	ret = vd55g0_wait_state(sensor, SENSOR_READY_TO_BOOT);
	if (ret)
		return ret;

	ret = vd55g0_read_reg16(sensor, DEVICE_MODEL_ID_REG, &id);
	if (ret)
		return ret;

	if (id != VD55G0_MODEL_ID) {
		dev_warn(&client->dev, "Unsupported sensor id %x", id);
		return -ENODEV;
	}

	return 0;
}

static int vd55g0_try_fmt_internal(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   const struct vd55g0_mode_info **new_mode)
{
	const struct vd55g0_mode_info *mode = vd55g0_mode_data;
	unsigned int index;
	unsigned int i;

	/* select code */
	for (index = 0; index < ARRAY_SIZE(vd55g0_supported_codes); index++) {
		if (vd55g0_supported_codes[index] == fmt->code)
			break;
	}
	if (index == ARRAY_SIZE(vd55g0_supported_codes))
		index = 0;

	/* select size */
	for (i = 0; i < ARRAY_SIZE(vd55g0_mode_data); i++) {
		if (mode->width <= fmt->width && mode->height <= fmt->height)
			break;
		mode++;
	}
	if (i == ARRAY_SIZE(vd55g0_mode_data))
		mode--;

	*new_mode = mode;
	fmt->code = vd55g0_supported_codes[index];
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->field = V4L2_FIELD_NONE;

	return 0;
}

static int set_frame_rate(struct vd55g0_dev *sensor)
{
	u16 frame_length;

	frame_length = sensor->pclk /
		(sensor->line_length * sensor->frame_interval.denominator);

	return vd55g0_write_reg16(sensor, DEVICE_FRAME_LENGTH, frame_length);
}

static int vd55g0_stream_enable(struct vd55g0_dev *sensor)
{
	int center_x = SENSOR_WIDTH / 2;
	int center_y = SENSOR_HEIGHT / 2;
	int is_isl = sensor->current_mode->is_isl;
	int scale = 1 << sensor->current_mode->bin_mode;
	int width = sensor->current_mode->width * scale;
	int height = sensor->current_mode->height * scale;
	int ret;

	if (is_isl)
		height -= 2 *scale;

	/* configure output mode */
	ret = vd55g0_write_reg(sensor, DEVICE_FORMAT_CTRL,
			       get_bpp_by_code(sensor->fmt.code));
	if (ret)
		return ret;
	ret = vd55g0_write_reg(sensor, DEVICE_OIF_IMG_CTRL,
			       get_datatype_by_code(sensor->fmt.code));
	if (ret)
		return ret;
	ret = vd55g0_write_reg(sensor, DEVICE_OIF_ISL_CTRL, is_isl ?
			       get_datatype_by_code(sensor->fmt.code) :
			       0x12);
	ret = vd55g0_write_reg(sensor, DEVICE_ISL_ENABLE, is_isl);
	if (ret)
		return ret;

	/* configure size and bin mode */
	ret = vd55g0_write_reg(sensor, DEVICE_READOUT_CTRL,
			       sensor->current_mode->bin_mode);
	if (ret)
		return ret;
	ret = vd55g0_write_reg16(sensor, DEVICE_ROI_X_START,
				 center_x - width / 2);
	if (ret)
		return ret;
	ret = vd55g0_write_reg16(sensor, DEVICE_ROI_X_END,
				 center_x + width / 2 - 1);
	if (ret)
		return ret;
	ret = vd55g0_write_reg16(sensor, DEVICE_ROI_Y_START,
				 center_y - height / 2);
	if (ret)
		return ret;
	ret = vd55g0_write_reg16(sensor, DEVICE_ROI_Y_END,
				 center_y + height / 2 - 1);
	if (ret)
		return ret;
#if 1
	dev_info(&client->dev, "setting gpio PWM");
	//TODO check it does not interfere
	ret = vd55g0_write_reg(sensor, DEVICE_GPIO_0_CTRL, 0x03);
    // To get 100 microsecond on time
    // Clock_divider [0x:472] : 59999 (0xea5f)
    // Duty_cycle[0x470] : 12 (25%)
	reg = 0x472;
	val = 0xea5f;
	ret = vd55g0_write_reg16(sensor, reg, val);
	reg = 0x470;
	val = 0x0C;
	ret = vd55g0_write_reg16(sensor, reg, val);
#endif

	/* configure frame rate */
	ret = set_frame_rate(sensor);
	if (ret)
		return ret;

	/* apply exposure */
	ret = apply_exposure(sensor);
	if (ret)
		return ret;

	/* start streaming */
	ret = vd55g0_write_reg(sensor, DEVICE_SW_STBY, CMD_START_STREAM);
	if (ret)
		return ret;

	ret = vd55g0_poll_reg(sensor, DEVICE_SW_STBY, 0);
	if (ret)
		return ret;

	ret = vd55g0_wait_state(sensor, SENSOR_STREAMING);
	if (ret)
		return ret;

	return 0;
}

static int vd55g0_stream_disable(struct vd55g0_dev *sensor)
{
	int ret;

	ret = vd55g0_write_reg(sensor, DEVICE_STREAMING, CMD_STOP_STREAM);
	if (ret)
		return ret;

	ret = vd55g0_poll_reg(sensor, DEVICE_STREAMING, 0);
	if (ret)
		return ret;

	ret = vd55g0_wait_state(sensor, SENSOR_SW_STBY);
	if (ret)
		return ret;

	return 0;
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

	ret = vd55g0_write_reg(sensor, DEVICE_BOOT, CMD_PATCH_SETUP);
	if (ret)
		return ret;

	ret = vd55g0_poll_reg(sensor, DEVICE_BOOT, 0);
	if (ret)
		return ret;

	ret = vd55g0_read_reg16(sensor, DEVICE_FWPATCH_REVISION, &patch);
	if (ret)
		return ret;

	if (patch != (DEVICE_FWPATCH_REVISION_MAJOR << 8) +
	    DEVICE_FWPATCH_REVISION_MINOR) {
		dev_err(&client->dev, "bad patch version expected %d.%d got %d.%d",
			DEVICE_FWPATCH_REVISION_MAJOR,
			DEVICE_FWPATCH_REVISION_MINOR,
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

	ret = vd55g0_write_reg(sensor, DEVICE_BOOT, CMD_BOOT);
	if (ret)
		return ret;

	ret = vd55g0_poll_reg(sensor, DEVICE_BOOT, 0);
	if (ret)
		return ret;

	ret = vd55g0_wait_state(sensor, SENSOR_SW_STBY);
	if (ret)
		return ret;

	dev_info(&client->dev, "sensor boot successfully");

	return 0;
}

static int vd55g0_configure(struct vd55g0_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	unsigned int prediv;
	unsigned int mult;
	unsigned int i;
	int ret;
	u16 oif_ctrl = 0;

	compute_pll_parameters_by_freq(sensor->clk_freq, &prediv, &mult);
	/* cache line_length value */
	ret = vd55g0_read_reg16(sensor, DEVICE_LINE_LENGTH,
				&sensor->line_length);
	if (ret)
		return ret;
	/* configure clocks */
	ret = vd55g0_write_reg32(sensor, DEVICE_EXT_CLOCK, sensor->clk_freq);
#if 0
	if (ret)
		return ret;
	ret = vd55g0_write_reg(sensor, DEVICE_CLK_PLL_PREDIV, prediv);
	if (ret)
		return ret;
	ret = vd55g0_write_reg(sensor, DEVICE_CLK_SYS_PLL_MULT, mult);
	if (ret)
		return ret;
	/* configure interface */
	ret = vd55g0_write_reg16(sensor, DEVICE_OIF_CTRL, sensor->oif_ctrl);
#endif
#if 0
	oif_ctrl |= 1 << 3;
	oif_ctrl |= 1 << 6;
	ret = vd55g0_write_reg16(sensor, DEVICE_OIF_CTRL, oif_ctrl);
#endif
	/* configure interface */
	ret = vd55g0_write_reg16(sensor, DEVICE_OIF_CTRL, sensor->oif_ctrl);
	if (ret)
		return ret;
	dev_dbg(&client->dev, "oif_ctrl_updatedv = 0x%x prev = 0x%x", oif_ctrl, sensor->oif_ctrl);
	ret = vd55g0_write_reg16(sensor, DEVICE_OIF_CSI_BITRATE, 804);
	if (ret)
		return ret;
	/* use auto expo by default */
	ret = vd55g0_write_reg(sensor, DEVICE_EXP_MODE, EXP_MODE_AUTO);
	if (ret)
		return ret;
	/* gpios in input (disabled) by default */
	for (i = 0; i < 8; i++) {
		ret = vd55g0_write_reg(sensor, DEVICE_GPIO_0_CTRL + i, 0x01);
		if (ret)
			return ret;
	}

	sensor->data_rate_in_mbps = (mult * sensor->clk_freq) / prediv;
	sensor->pclk = (sensor->data_rate_in_mbps * 2) / 10;
	dev_info(&client->dev, "data rate = %d mbps",
		 sensor->data_rate_in_mbps);

	return 0;
}

/* implement v4l2_subdev_video_ops */
static int vd55g0_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;

	mutex_lock(&sensor->lock);
	dev_dbg(&client->dev, "%s : requested %d / current = %d", __func__,
		enable, sensor->streaming);
	if (sensor->streaming == enable)
		goto out;

	ret = enable ? vd55g0_stream_enable(sensor) :
		       vd55g0_stream_disable(sensor);
	if (!ret)
		sensor->streaming = enable;

out:
	dev_dbg(&client->dev, "%s current now = %d / %d", __func__,
		sensor->streaming, ret);
	mutex_unlock(&sensor->lock);

	return ret;
}

static int vd55g0_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int vd55g0_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	u64 req_int, err, min_err = ~0ULL;
	u64 test_int;
	unsigned int i;
	int ret;

	if (fi->pad != 0)
		return -EINVAL;

	if (fi->interval.denominator == 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	dev_dbg(&client->dev, "%s request %d/%d\n", __func__,
		fi->interval.numerator, fi->interval.denominator);
	/* find nearest period */
	req_int = div64_u64((u64)(fi->interval.numerator * 10000),
			    fi->interval.denominator);
	for (i = 0; i < ARRAY_SIZE(vd55g0_sensor_frame_rates); i++) {
		test_int = div64_u64((u64)10000, vd55g0_sensor_frame_rates[i]);
		err = abs(test_int - req_int);
		if (err < min_err) {
			fi->interval.numerator = 1;
			fi->interval.denominator = vd55g0_sensor_frame_rates[i];
			min_err = err;
		}
	}
	sensor->frame_interval = fi->interval;
	dev_dbg(&client->dev, "%s set     %d/%d\n", __func__,
		fi->interval.numerator, fi->interval.denominator);

	ret = 0;
out:
	mutex_unlock(&sensor->lock);

	return ret;
}

/* implement v4l2_subdev_pad_ops */
static int vd55g0_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

	dev_dbg(&client->dev, "%s probe index %d", __func__, code->index);
	if (code->index >= ARRAY_SIZE(vd55g0_supported_codes))
		return -EINVAL;

	code->code = vd55g0_supported_codes[code->index];

	return 0;
}

static int vd55g0_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	struct v4l2_mbus_framefmt *fmt;

	dev_dbg(&client->dev, "%s probe %d", __func__, format->pad);
	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg,
						 format->pad);
	else
		fmt = &sensor->fmt;

	format->format = *fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

static int vd55g0_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	const struct vd55g0_mode_info *new_mode;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	if (format->pad != 0)
		return -EINVAL;

	dev_dbg(&client->dev, "%s %dx%d", __func__, format->format.width,
		format->format.height);

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	/* find best format */
	ret = vd55g0_try_fmt_internal(sd, &format->format, &new_mode);
	if (ret)
		goto out;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt = v4l2_subdev_get_try_format(sd, cfg, 0);
	} else {
		fmt = &sensor->fmt;
		sensor->current_mode = new_mode;
	}
	*fmt = format->format;

out:
	mutex_unlock(&sensor->lock);

	return ret;
}

static int vd55g0_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

	dev_dbg(&client->dev, "%s for index %d", __func__, fse->index);
	if (fse->pad != 0)
		return -EINVAL;
	if (fse->index >= ARRAY_SIZE(vd55g0_mode_data))
		return -EINVAL;

	fse->min_width = vd55g0_mode_data[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = vd55g0_mode_data[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int vd55g0_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum
				      *fie)
{
	const struct vd55g0_mode_info *mode = vd55g0_mode_data;
	unsigned int i;

	if (fie->pad != 0)
		return -EINVAL;
	if (fie->index >= ARRAY_SIZE(vd55g0_sensor_frame_rates))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(vd55g0_mode_data); i++) {
		if (mode->width <= fie->width && mode->height <= fie->height)
			break;
		mode++;
	}
	if (i == ARRAY_SIZE(vd55g0_mode_data))
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = vd55g0_sensor_frame_rates[fie->index];

	return 0;
}

static const struct v4l2_subdev_core_ops vd55g0_core_ops = {
};

static const struct v4l2_subdev_video_ops vd55g0_video_ops = {
	.s_stream = vd55g0_s_stream,
	.g_frame_interval = vd55g0_g_frame_interval,
	.s_frame_interval = vd55g0_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops vd55g0_pad_ops = {
	.enum_mbus_code = vd55g0_enum_mbus_code,
	.get_fmt = vd55g0_get_fmt,
	.set_fmt = vd55g0_set_fmt,
	.enum_frame_size = vd55g0_enum_frame_size,
	.enum_frame_interval = vd55g0_enum_frame_interval,
};

static const struct v4l2_subdev_ops vd55g0_subdev_ops = {
	.core = &vd55g0_core_ops,
	.video = &vd55g0_video_ops,
	.pad = &vd55g0_pad_ops,
};

static const struct media_entity_operations vd55g0_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* controls */
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
		ret = vd55g0_write_reg(sensor, DEVICE_ORIENTATION,
				       sensor->hflip | (sensor->vflip << 1));
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = vd55g0_update_patgen(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = vd55g0_update_exposure_auto(sensor, ctrl->val);
		break;
	case V4L2_CID_GAIN:
		ret = vd55g0_update_gains(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = vd55g0_set_exposure(sensor, ctrl->val);
		ctrl->val = sensor->manual_expo_ms;
		break;
	case V4L2_CID_3A_LOCK:
		ret = vd55g0_lock_exposure(sensor, ctrl->val);
		break;
	case V4L2_CID_GPIO0_MODE:
	case V4L2_CID_GPIO1_MODE:
	case V4L2_CID_GPIO2_MODE:
	case V4L2_CID_GPIO3_MODE:
		ret = vd55g0_update_gpiox_strobe_mode(sensor, ctrl->val,
			ctrl->id - V4L2_CID_GPIO0_MODE);
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

/* FIXME: better add a macro here ? */
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
	struct v4l2_ctrl *ctrl;
	int ret;

	v4l2_ctrl_handler_init(hdl, 16);
	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;
	/* add flipping */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	/* add pattern generator */
	v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(vd55g0_test_pattern_menu) - 1,
				     0, 0, vd55g0_test_pattern_menu);
	/* add V4L2_CID_PIXEL_RATE */
	ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE, 1, INT_MAX, 1,
				 get_pixel_rate(sensor));
	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY;
	ctrl = v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(link_freq) - 1, 0, link_freq);
	ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	/* add V4L2_CID_EXPOSURE_AUTO */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_EXPOSURE_AUTO, 1, ~0x3,
			       V4L2_EXPOSURE_AUTO);
	/* V4L2_CID_GAIN. This is 8.8 fixed point value */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN, 0, 0x3fff, 1, 0x100);
	/* V4L2_CID_EXPOSURE */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE, 1, 500, 1, 10);
	/* V4L2_CID_3A_LOCK */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_3A_LOCK, 0, 7, 0, 0);
	/* gpios stuff */
	v4l2_ctrl_new_custom(hdl, &vd55g0_gpio0_ctrl, NULL);
	v4l2_ctrl_new_custom(hdl, &vd55g0_gpio1_ctrl, NULL);
	v4l2_ctrl_new_custom(hdl, &vd55g0_gpio2_ctrl, NULL);
	v4l2_ctrl_new_custom(hdl, &vd55g0_gpio3_ctrl, NULL);
	/* temperature */
	ctrl = v4l2_ctrl_new_custom(hdl, &vd55g0_temp_ctrl, NULL);
	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY;

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

static int vd55g0_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct vd55g0_dev *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;
	sensor->streaming = false;
	sensor->fmt.code = MEDIA_BUS_FMT_SGBRG8_1X8;
	sensor->fmt.field = V4L2_FIELD_NONE;
	sensor->fmt.colorspace = V4L2_COLORSPACE_SRGB;
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = 15;
	sensor->manual_expo_ms = 10;
	sensor->expo_state = VD55G0_EXPO_AUTO;

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

	sensor->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(sensor->xclk);
	}
	sensor->clk_freq = clk_get_rate(sensor->xclk);
	if (sensor->clk_freq < 6000000 || sensor->clk_freq > 27000000) {
		dev_err(dev, "Only 6Mhz-27Mhz clock range supported. provide %d Hz\n",
			sensor->clk_freq);
		return -EINVAL;
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &vd55g0_subdev_ops);
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.ops = &vd55g0_subdev_entity_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret) {
		dev_err(&client->dev, "pads init failed %d", ret);
		return ret;
	}

	/* request optional reset pin */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	sensor->pwr_reset = devm_gpiod_get_optional(dev, "power",
						     GPIOD_OUT_HIGH);
	ret = vd55g0_get_regulators(sensor);
	if (ret) {
		dev_err(&client->dev, "failed to get regulators %d", ret);
		goto entity_cleanup;
	}

	ret = regulator_bulk_enable(VD55G0_NUM_SUPPLIES, sensor->supplies);
	if (ret) {
		dev_err(&client->dev, "failed to enable regulators %d", ret);
		goto entity_cleanup;
	}

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "failed to enable clock %d", ret);
		goto disable_bulk;
	}

	mutex_init(&sensor->lock);

	/* apply reset sequence */
	if (sensor->reset_gpio)
		vd55g0_apply_reset(sensor);

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

	ret = vd55g0_init_controls(sensor);
	if (ret) {
		dev_err(&client->dev, "controls initialization failed %d", ret);
		goto disable_clock;
	}

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret) {
		dev_err(&client->dev, "async subdev register failed %d", ret);
		goto disable_clock;
	}

	dev_info(&client->dev, "vd55g0 probe successfully");

	return 0;

disable_clock:
	clk_disable_unprepare(sensor->xclk);
disable_bulk:
	regulator_bulk_disable(VD55G0_NUM_SUPPLIES, sensor->supplies);
entity_cleanup:
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

static int vd55g0_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vd55g0_dev *sensor = to_vd55g0_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	clk_disable_unprepare(sensor->xclk);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	regulator_bulk_disable(VD55G0_NUM_SUPPLIES, sensor->supplies);

	return 0;
}

static const struct of_device_id vd55g0_dt_ids[] = {
	{ .compatible = "st,st-vd55g0" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vd55g0_dt_ids);

static struct i2c_driver vd55g0_i2c_driver = {
	.driver = {
		.name  = "st-vd55g0",
		.of_match_table = vd55g0_dt_ids,
	},
	.probe_new = vd55g0_probe,
	.remove = vd55g0_remove,
};

module_i2c_driver(vd55g0_i2c_driver);

MODULE_AUTHOR("Mickael Guene <mickael.guene@st.com>");
MODULE_AUTHOR("Benjamin Mugnier <benjamin.mugnier@st.com>");
MODULE_DESCRIPTION("vd55g0 Camera Subdev Driver");
MODULE_LICENSE("GPL v2");
