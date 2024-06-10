/*
 * Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_lp5521

/**
 * @file
 * @brief LP5521 LED driver
 *
 * The LP5521 is a 3-channel LED driver that communicates over I2C. The three
 * channels are expected to be connected to a red, green, blue LED.
 *
 * 1. The brightness of each LED can be configured directly by setting a
 * register that drives the PWM of the connected LED.
 *
 * 2. A program can be transferred to the driver and run by one of the three
 * available execution LED engines. Up to 16 commands can be defined in each
 * program. Possible commands are:
 *   - Set the brightness.
 *   - Fade the brightness over time.
 *   - Loop parts of the program or the whole program.
 *   - Add delays.
 *   - Synchronize between the engines.
 *
 * After the program has been transferred, it can run infinitely without
 * communication between the host MCU and the driver.
 */
#define DT_DRV_COMPAT ti_lp5521

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define LOG_LEVEL CONFIG_LED_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lp5521);

#include "led_context.h"

/* Registers */
#define LP5521_ENABLE                  0x00
#define LP5521_OP_MODE                 0x01
#define LP5521_R_PWM                   0x02
#define LP5521_G_PWM                   0x03
#define LP5521_B_PWM                   0x04
#define LP5521_R_CURRENT               0x05
#define LP5521_G_CURRENT               0x06
#define LP5521_B_CURRENT               0x07
#define LP5521_CONFIG                  0x08
#define LP5521_ENG1_PC                 0x09
#define LP5521_ENG2_PC                 0x0A
#define LP5521_ENG3_PC                 0x0B
#define LP5521_STATUS                  0x0C
#define LP5521_RESET                   0x0D
#define LP5521_GPO                     0x0E
#define LP5521_PROG_MEM_RED_ENG_BASE   0x10
#define LP5521_PROG_MEM_GREEN_ENG_BASE 0x30
#define LP5521_PROG_MEM_BLUE_ENG_BASE  0x50

/*
 * The wait command has six bits for the number of steps (max 63) with up to
 * 15.6ms per step if the prescaler is set to 1. We round the step length
 * however to 16ms for easier handling, so the maximum blinking period is
 * therefore (16 * 63) = 1008ms. We round it down to 1000ms to be on the safe
 * side.
 */
#define LP5521_MAX_BLINK_PERIOD 1000
/*
 * The minimum waiting period is 0.49ms with the prescaler set to 0 and one
 * step. We round up to a full millisecond.
 */
#define LP5521_MIN_BLINK_PERIOD 1

/* Brightness limits in percent */
#define LP5521_MIN_BRIGHTNESS 0
#define LP5521_MAX_BRIGHTNESS 100

/* Output current limits in 0.1 mA */
#define LP5521_MIN_CURRENT_SETTING 0
#define LP5521_MAX_CURRENT_SETTING 255

/* Values for ENABLE register. */
#define LP5521_ENABLE_CHIP_EN (1 << 6)
#define LP5521_ENABLE_LOG_EN  (1 << 7)

/* Values for execution engine programs. */
#define LP5521_PROG_COMMAND_SET_PWM                           (1 << 6)
#define LP5521_PROG_COMMAND_RAMP_TIME(prescale, step_time)    (((prescale) << 6) | (step_time))
#define LP5521_PROG_COMMAND_STEP_COUNT(fade_direction, count) (((fade_direction) << 7) | (count))

/* Helper definitions. */
#define LP5521_PROG_MAX_COMMANDS     16
#define LP5521_MASK                  0x03
#define LP5521_CHANNEL_MASK(channel) ((LP5521_MASK) << (channel << 1))

/*
 * Available channels. There are three LED channels usable with the LP5521. While
 * they can be mapped to LEDs of any color, the driver's typical application is
 * with a red, a green and a blue  LED. Since the data sheet's
 * nomenclature uses RGB, we keep it that way.
 */
enum lp5521_led_channels {
	LP5521_CHANNEL_R,
	LP5521_CHANNEL_G,
	LP5521_CHANNEL_B,

	LP5521_CHANNEL_COUNT,
};

/* Operational modes of the execution engines. */
enum lp5521_led_op_modes {
	LP5521_OP_MODE_DISABLED = 0x00,
	LP5521_OP_MODE_LOAD = 0x01,
	LP5521_OP_MODE_RUN = 0x02,
	LP5521_OP_MODE_DIRECT_CTRL = 0x03,
};

/* Execution state of the engines. */
enum lp5521_engine_exec_states {
	LP5521_ENGINE_MODE_HOLD = 0x00,
	LP5521_ENGINE_MODE_STEP = 0x01,
	LP5521_ENGINE_MODE_RUN = 0x02,
	LP5521_ENGINE_MODE_EXEC = 0x03,
};

/* Fading directions for programs executed by the engines. */
enum lp5521_engine_fade_dirs {
	LP5521_FADE_UP = 0x00,
	LP5521_FADE_DOWN = 0x01,
};

struct lp5521_config {
	struct i2c_dt_spec bus;
	uint8_t r_current;
	uint8_t g_current;
	uint8_t b_current;
	const struct gpio_dt_spec en_pin;
};

struct lp5521_data {
	struct led_data dev_data;
};

/*
 * @brief Get the register for the given LED channel used to directly write a
 *	brightness value instead of using the execution engines.
 *
 * @param led     target led.
 * @param reg     Pointer to the register address.
 *
 * @retval 0       On success.
 * @retval -EINVAL If an invalid channel is given.
 */
static int lp5521_get_pwm_reg(enum lp5521_led_channels led, uint8_t *reg)
{
	*reg = (LP5521_R_PWM + led);
	return 0;
}

/*
 * @brief Get the base address for programs of the given execution engine.
 *
 * @param led       target led
 * @param base_addr Pointer to the base address.
 *
 * @retval 0       On success.
 * @retval -EINVAL If a source is given that is not a valid engine.
 */
static int lp5521_get_led_ram_base_addr(enum lp5521_led_channels led, uint8_t *base_addr)
{
	*base_addr = LP5521_PROG_MEM_RED_ENG_BASE + (led * 0x20);
	return 0;
}

/*
 * @brief Helper to get the register bit shift for the execution engines.
 *
 * The engine with the highest index is placed on the lowest two bits in the
 * OP_MODE and ENABLE registers.
 *
 * @param led    target led
 * @param shift  Pointer to the shift value.
 *
 * @retval 0       On success.
 * @retval -EINVAL If a source is given that is not a valid engine.
 */
static int lp5521_get_led_reg_shift(enum lp5521_led_channels led, uint8_t *shift)
{
	*shift = 4 - (2 * led);
	return 0;
}

/*
 * @brief Convert a time in milliseconds to a combination of prescale and
 *	step_time for the execution engine programs.
 *
 * This function expects the given time in milliseconds to be in the allowed
 * range the device can handle (0ms to 1000ms).
 *
 * @param data      Capabilities of the driver.
 * @param ms        Time to be converted in milliseconds [0..1000].
 * @param prescale  Pointer to the prescale value.
 * @param step_time Pointer to the step_time value.
 */
static void lp5521_ms_to_prescale_and_step(struct led_data *data, uint32_t ms, uint8_t *prescale,
					   uint8_t *step_time)
{
	/*
	 * One step with the prescaler set to 0 takes 0.49ms. The max value for
	 * step_time is 63, so we just double the millisecond value. That way
	 * the step_time value never goes above the allowed 63.
	 */
	if (ms < 31) {
		*prescale = 0U;
		*step_time = ms << 1;
	} else {
		/*
		 * With a prescaler value set to 1 one step takes 15.6ms. So by dividing
		 * through 16 we get a decent enough result with low effort.
		 */
		*prescale = 1U;
		*step_time = ms >> 4;
	}
}

/*
 * @brief Get the assigned source of the given LED channel.
 *
 * @param dev     LP5521 device.
 * @param led     target led
 * @param source  Pointer to the source of the channel.
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static int lp5521_get_led_op_mode(const struct device *dev, enum lp5521_led_channels led,
				  enum lp5521_led_op_modes *source)
{
	const struct lp5521_config *config = dev->config;
	uint8_t led_map;

	if (i2c_reg_read_byte_dt(&config->bus, LP5521_OP_MODE, &led_map)) {
		return -EIO;
	}

	*source = (led_map >> (4 - (led * 2))) & LP5521_MASK;

	return 0;
}

/*
 * @brief Request whether an LED engine is currently running.
 *
 * @param dev    LP5521 device.
 * @param led    target led
 *
 * @return Indication of the engine execution state.
 *
 * @retval true  If the LED engine is currently running.
 * @retval false If the LED engine is not running or an error occurred.
 */
static bool lp5521_is_led_executing_cmds(const struct device *dev, enum lp5521_led_channels led)
{
	const struct lp5521_config *config = dev->config;
	uint8_t enabled, shift;
	int ret;

	ret = lp5521_get_led_reg_shift(led, &shift);
	if (ret) {
		return false;
	}

	if (i2c_reg_read_byte_dt(&config->bus, LP5521_ENABLE, &enabled)) {
		LOG_ERR("Failed to read ENABLE register.");
		return false;
	}

	enabled = (enabled >> shift) & LP5521_MASK;

	if (enabled == LP5521_ENGINE_MODE_RUN) {
		return true;
	}

	return false;
}

/*
 * @brief Set an register shifted for the given execution LED engine.
 *
 * @param dev    LP5521 device.
 * @param led    target led
 * @param reg    Register address to set.
 * @param val    Value to set.
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static int lp5521_set_led_reg(const struct device *dev, enum lp5521_led_channels led, uint8_t reg,
			      uint8_t val)
{
	const struct lp5521_config *config = dev->config;
	uint8_t shift;
	int ret;

	ret = lp5521_get_led_reg_shift(led, &shift);
	if (ret) {
		return ret;
	}

	if (i2c_reg_update_byte_dt(&config->bus, reg, LP5521_MASK << shift, val << shift)) {
		return -EIO;
	}

	return 0;
}

/*
 * @brief Set the operational mode of the given LED engine.
 *
 * @param dev    LP5521 device.
 * @param led    target led
 * @param mode   Mode to set.
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static inline int lp5521_set_led_op_mode(const struct device *dev, enum lp5521_led_channels led,
					 enum lp5521_led_op_modes mode)
{
	int ret = lp5521_set_led_reg(dev, led, LP5521_OP_MODE, mode);

	k_msleep(1); /* Must wait for next i2c write on LP5521_OP_MODE reg */
	return ret;
}

/*
 * @brief Set the execution state of the given LED engine.
 *
 * @param dev    LP5521 device.
 * @param led    target led
 * @param state  State to set.
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static inline int lp5521_set_led_exec_state(const struct device *dev, enum lp5521_led_channels led,
					    enum lp5521_engine_exec_states state)
{
	int ret;

	ret = lp5521_set_led_reg(dev, led, LP5521_ENABLE, state);

	/*
	 * Delay between consecutive I2C writes to
	 * ENABLE register (00h) need to be longer than 488μs (typ.).
	 */
	k_msleep(1);

	return ret;
}

/*
 * @brief Start the execution of the program of the given LED engine.
 *
 * @param dev    LP5521 device.
 * @param led    target led
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static inline int lp5521_start_program_exec(const struct device *dev, enum lp5521_led_channels led)
{
	if (lp5521_set_led_op_mode(dev, led, LP5521_OP_MODE_RUN)) {
		return -EIO;
	}

	return lp5521_set_led_exec_state(dev, led, LP5521_ENGINE_MODE_RUN);
}

/*
 * @brief Stop the execution of the program of the given LED engine.
 *
 * @param dev    LP5521 device.
 * @param led    target led
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static inline int lp5521_stop_program_exec(const struct device *dev, enum lp5521_led_channels led)
{
	if (lp5521_set_led_op_mode(dev, led, LP5521_OP_MODE_DISABLED)) {
		return -EIO;
	}

	return lp5521_set_led_exec_state(dev, led, LP5521_ENGINE_MODE_HOLD);
}

/*
 * @brief Program a command to the memory of the given execution LED engine.
 *
 * @param dev           LP5521 device.
 * @param led        target led
 * @param command_index Index of the command that is programmed.
 * @param command_msb   Most significant byte of the command.
 * @param command_lsb   Least significant byte of the command.
 *
 * @retval 0       On success.
 * @retval -EINVAL If the given command index is out of range or an invalid
 *		   engine is passed.
 * @retval -EIO    If the underlying I2C call fails.
 */
static int lp5521_program_command(const struct device *dev, enum lp5521_led_channels led,
				  uint8_t command_index, uint8_t command_msb, uint8_t command_lsb)
{
	const struct lp5521_config *config = dev->config;
	uint8_t prog_base_addr;
	int ret;

	if (command_index >= LP5521_PROG_MAX_COMMANDS || led >= LP5521_CHANNEL_COUNT) {
		return -EINVAL;
	}

	ret = lp5521_get_led_ram_base_addr(led, &prog_base_addr);
	if (ret) {
		LOG_ERR("Failed to get base RAM address.");
		return ret;
	}

	if (i2c_reg_write_byte_dt(&config->bus, prog_base_addr + (command_index << 1),
				  command_msb)) {
		LOG_ERR("Failed to update LED.");
		return -EIO;
	}

	if (i2c_reg_write_byte_dt(&config->bus, prog_base_addr + (command_index << 1) + 1,
				  command_lsb)) {
		LOG_ERR("Failed to update LED.");
		return -EIO;
	}

	return 0;
}

/*
 * @brief Program a command to set a fixed brightness to the given led engine.
 *
 * @param dev           LP5521 device.
 * @param led        target led.
 * @param command_index Index of the command in the program sequence.
 * @param brightness    Brightness to be set for the LED in percent.
 *
 * @retval 0       On success.
 * @retval -EINVAL If the passed arguments are invalid or out of range.
 * @retval -EIO    If the underlying I2C call fails.
 */
static int lp5521_program_set_brightness(const struct device *dev, enum lp5521_led_channels led,
					 uint8_t command_index, uint8_t brightness)
{
	struct lp5521_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	uint8_t val;

	if ((brightness < dev_data->min_brightness) || (brightness > dev_data->max_brightness)) {
		return -EINVAL;
	}

	val = (brightness * 0xFF) / dev_data->max_brightness;

	return lp5521_program_command(dev, led, command_index, LP5521_PROG_COMMAND_SET_PWM, val);
}

/*
 * @brief Program a command to ramp the brightness over time.
 *
 * In each step the PWM value is increased or decreased by 1/255th until the
 * maximum or minimum value is reached or step_count steps have been done.
 *
 * @param dev           LP5521 device.
 * @param led           target led.
 * @param command_index Index of the command in the program sequence.
 * @param time_per_step Time each step takes in milliseconds.
 * @param step_count    Number of steps to perform.
 * @param fade_dir      Direction of the ramp (in-/decrease brightness).
 *
 * @retval 0       On success.
 * @retval -EINVAL If the passed arguments are invalid or out of range.
 * @retval -EIO    If the underlying I2C call fails.
 */
static int lp5521_program_ramp(const struct device *dev, enum lp5521_led_channels led,
			       uint8_t command_index, uint32_t time_per_step, uint8_t step_count,
			       enum lp5521_engine_fade_dirs fade_dir)
{
	struct lp5521_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	uint8_t prescale, step_time;

	if ((time_per_step < dev_data->min_period) || (time_per_step > dev_data->max_period)) {
		return -EINVAL;
	}

	lp5521_ms_to_prescale_and_step(dev_data, time_per_step, &prescale, &step_time);

	return lp5521_program_command(dev, led, command_index,
				      LP5521_PROG_COMMAND_RAMP_TIME(prescale, step_time),
				      LP5521_PROG_COMMAND_STEP_COUNT(fade_dir, step_count));
}

/*
 * @brief Program a command to do nothing for the given time.
 *
 * @param dev           LP5521 device.
 * @param led           target led.
 * @param command_index Index of the command in the program sequence.
 * @param time          Time to do nothing in milliseconds.
 *
 * @retval 0       On success.
 * @retval -EINVAL If the passed arguments are invalid or out of range.
 * @retval -EIO    If the underlying I2C call fails.
 */
static inline int lp5521_program_wait(const struct device *dev, enum lp5521_led_channels led,
				      uint8_t command_index, uint32_t time)
{
	/*
	 * A wait command is a ramp with the step_count set to 0. The fading
	 * direction does not matter in this case.
	 */
	return lp5521_program_ramp(dev, led, command_index, time, 0, LP5521_FADE_UP);
}

/*
 * @brief Program a command to go back to the beginning of the program.
 *
 * Can be used at the end of a program to loop it infinitely.
 *
 * @param dev           LP5521 device.
 * @param led           target led
 * @param command_index Index of the command in the program sequence.
 *
 * @retval 0       On success.
 * @retval -EINVAL If the given command index is out of range or an invalid
 *		   engine is passed.
 * @retval -EIO    If the underlying I2C call fails.
 */
static inline int lp5521_program_go_to_start(const struct device *dev, enum lp5521_led_channels led,
					     uint8_t command_index)
{
	return lp5521_program_command(dev, led, command_index, 0x00, 0x00);
}

/*
 * @brief Change the brightness of a running blink program.
 *
 * We know that the current program executes a blinking pattern
 * consisting of following commands:
 *
 * - set_brightness high
 * - wait on_delay
 * - set_brightness low
 * - wait off_delay
 * - return to start
 *
 * In order to change the brightness during blinking, we overwrite only
 * the first command and start execution again.
 *
 * @param dev           LP5521 device.
 * @param led           target led.
 * @param brightness_on New brightness value.
 *
 * @retval 0       On Success.
 * @retval -EINVAL If the engine ID or brightness is out of range.
 * @retval -EIO    If the underlying I2C call fails.
 */
static int lp5521_update_blinking_brightness(const struct device *dev, enum lp5521_led_channels led,
					     uint8_t brightness_on)
{
	int ret;

	ret = lp5521_stop_program_exec(dev, led);
	if (ret) {
		return ret;
	}

	ret = lp5521_set_led_op_mode(dev, led, LP5521_OP_MODE_LOAD);
	if (ret) {
		return ret;
	}

	ret = lp5521_program_set_brightness(dev, led, 0, brightness_on);
	if (ret) {
		return ret;
	}

	ret = lp5521_start_program_exec(dev, led);
	if (ret) {
		LOG_ERR("Failed to execute program.");
		return ret;
	}

	return 0;
}

static int lp5521_led_blink(const struct device *dev, uint32_t led, uint32_t delay_on,
			    uint32_t delay_off)
{
	struct lp5521_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	int ret;
	uint8_t command_index = 0U;

	if (led >= LP5521_CHANNEL_COUNT) {
		return -EINVAL;
	}

	ret = lp5521_set_led_op_mode(dev, led, LP5521_OP_MODE_LOAD);
	if (ret) {
		return ret;
	}

	ret = lp5521_program_set_brightness(dev, led, command_index, dev_data->max_brightness);
	if (ret) {
		return ret;
	}

	ret = lp5521_program_wait(dev, led, ++command_index, delay_on);
	if (ret) {
		return ret;
	}

	ret = lp5521_program_set_brightness(dev, led, ++command_index, dev_data->min_brightness);
	if (ret) {
		return ret;
	}

	ret = lp5521_program_wait(dev, led, ++command_index, delay_off);
	if (ret) {
		return ret;
	}

	ret = lp5521_program_go_to_start(dev, led, ++command_index);
	if (ret) {
		return ret;
	}

	ret = lp5521_start_program_exec(dev, led);
	if (ret) {
		LOG_ERR("Failed to execute program.");
		return ret;
	}
	return 0;
}

static int lp5521_led_set_brightness(const struct device *dev, uint32_t led, uint8_t value)
{
	const struct lp5521_config *config = dev->config;
	struct lp5521_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	int ret;
	uint8_t val, reg;
	enum lp5521_led_op_modes current_mode;

	if ((value < dev_data->min_brightness) || (value > dev_data->max_brightness) ||
	    (led >= LP5521_CHANNEL_COUNT)) {
		return -EINVAL;
	}

	ret = lp5521_get_led_op_mode(dev, led, &current_mode);
	if (ret) {
		return ret;
	}
	if ((current_mode != LP5521_OP_MODE_DIRECT_CTRL) &&
	    (current_mode != LP5521_OP_MODE_DISABLED)) {
		if (lp5521_is_led_executing_cmds(dev, led)) {
			/*
			 * LED is blinking currently. Restart the blinking with
			 * the passed brightness.
			 */
			return lp5521_update_blinking_brightness(dev, led, value);
		}

		ret = lp5521_set_led_op_mode(dev, led, LP5521_OP_MODE_DIRECT_CTRL);
		if (ret) {
			return ret;
		}
	}

	if (current_mode == LP5521_OP_MODE_DISABLED) {
		ret = lp5521_set_led_op_mode(dev, led, LP5521_OP_MODE_DIRECT_CTRL);
		if (ret) {
			return ret;
		}
	}
	val = (value * 0xFF) / dev_data->max_brightness;

	ret = lp5521_get_pwm_reg(led, &reg);
	if (ret) {
		return ret;
	}

	if (i2c_reg_write_byte_dt(&config->bus, reg, val)) {
		return -EIO;
	}

	return 0;
}

static inline int lp5521_led_on(const struct device *dev, uint32_t led)
{
	struct lp5521_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	int ret;

	if (led >= LP5521_CHANNEL_COUNT) {
		return -EINVAL;
	}

	ret = lp5521_set_led_op_mode(dev, led, LP5521_OP_MODE_DIRECT_CTRL);
	if (ret) {
		return ret;
	}

	return lp5521_led_set_brightness(dev, led, dev_data->max_brightness);
}

static inline int lp5521_led_off(const struct device *dev, uint32_t led)
{
	struct lp5521_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	int ret;
	enum lp5521_led_op_modes curr_op_mode;

	if (led >= LP5521_CHANNEL_COUNT) {
		return -EINVAL;
	}

	ret = lp5521_get_led_op_mode(dev, led, &curr_op_mode);
	if (ret) {
		return ret;
	}

	if ((curr_op_mode != LP5521_OP_MODE_DIRECT_CTRL) &&
	    (curr_op_mode != LP5521_OP_MODE_DISABLED)) {
		ret = lp5521_stop_program_exec(dev, curr_op_mode);
		if (ret) {
			return ret;
		}
	}

	return lp5521_led_set_brightness(dev, led, dev_data->min_brightness);
}

static int lp5521_led_update_current(const struct device *dev)
{
	const struct lp5521_config *config = dev->config;
	int ret;
	uint8_t tx_buf[4] = {LP5521_R_CURRENT, config->r_current, config->g_current,
			     config->b_current};

	ret = i2c_write_dt(&config->bus, tx_buf, sizeof(tx_buf));

	return ret;
}

static int lp5521_led_init(const struct device *dev)
{
	const struct lp5521_config *config = dev->config;
	struct lp5521_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	const struct gpio_dt_spec *en_pin = &config->en_pin;

	int ret;

	if (en_pin != 0) {
		if (!device_is_ready(en_pin->port)) {
			return -ENODEV;
		}
		if (gpio_pin_configure_dt(en_pin, GPIO_OUTPUT_ACTIVE)) {
			return -ENODEV;
		}
		if (gpio_pin_set_dt(en_pin, true)) {
			return -ENODEV;
		}
		k_msleep(5);
	}

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	/* Hardware specific limits */
	dev_data->min_period = LP5521_MIN_BLINK_PERIOD;
	dev_data->max_period = LP5521_MAX_BLINK_PERIOD;
	dev_data->min_brightness = LP5521_MIN_BRIGHTNESS;
	dev_data->max_brightness = LP5521_MAX_BRIGHTNESS;

	ret = lp5521_led_update_current(dev);
	if (ret) {
		LOG_ERR("Setting current setting LP5521 LED chip failed.");
		return ret;
	}

	if (i2c_reg_write_byte_dt(&config->bus, LP5521_ENABLE, LP5521_ENABLE_CHIP_EN)) {
		LOG_ERR("Enabling LP5521 LED chip failed.");
		return -EIO;
	}

	if (i2c_reg_write_byte_dt(&config->bus, LP5521_CONFIG, 0x11)) {
		LOG_ERR("Configuring LP5521 LED chip failed.");
		return -EIO;
	}

	if (i2c_reg_write_byte_dt(&config->bus, LP5521_OP_MODE, 0x3F)) {
		LOG_ERR("Disabling all engines failed.");
		return -EIO;
	}

	return 0;
}

static const struct led_driver_api lp5521_led_api = {
	.blink = lp5521_led_blink,
	.set_brightness = lp5521_led_set_brightness,
	.on = lp5521_led_on,
	.off = lp5521_led_off,
};

#define LP5521_DEFINE(id)                                                                          \
	BUILD_ASSERT(DT_INST_PROP(id, red_output_current) <= LP5521_MAX_CURRENT_SETTING,           \
		     "Red channel current must be between 0 and 25.5 mA.");                        \
	BUILD_ASSERT(DT_INST_PROP(id, green_output_current) <= LP5521_MAX_CURRENT_SETTING,         \
		     "Green channel current must be between 0 and 25.5 mA.");                      \
	BUILD_ASSERT(DT_INST_PROP(id, blue_output_current) <= LP5521_MAX_CURRENT_SETTING,          \
		     "Blue channel current must be between 0 and 25.5 mA.");                       \
                                                                                                   \
	static const struct lp5521_config lp5521_config_##id = {                                   \
		.bus = I2C_DT_SPEC_INST_GET(id),                                                   \
		.r_current = DT_INST_PROP(id, red_output_current),                                 \
		.g_current = DT_INST_PROP(id, green_output_current),                               \
		.b_current = DT_INST_PROP(id, blue_output_current),                                \
		.en_pin = GPIO_DT_SPEC_INST_GET_OR(id, en_gpios, {0}),                             \
	};                                                                                         \
                                                                                                   \
	struct lp5521_data lp5521_data_##id;                                                       \
	DEVICE_DT_INST_DEFINE(id, &lp5521_led_init, NULL, &lp5521_data_##id, &lp5521_config_##id,  \
			      POST_KERNEL, CONFIG_LED_INIT_PRIORITY, &lp5521_led_api);

DT_INST_FOREACH_STATUS_OKAY(LP5521_DEFINE)
