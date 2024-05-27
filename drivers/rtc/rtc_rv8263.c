/* Copyright (c) 2024 Daniel Kampert
 * Author: Daniel Kampert <DanielKampert@kampis-Elektroecke.de>
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include "rtc_utils.h"

#define RV8263C8_REGISTER_CONTROL_1     0x00
#define RV8263C8_REGISTER_CONTROL_2     0x01
#define RV8263C8_REGISTER_OFFSET        0x02
#define RV8263C8_REGISTER_RAM           0x03
#define RV8263C8_REGISTER_SECONDS       0x04
#define RV8263C8_REGISTER_MINUTES       0x05
#define RV8263C8_REGISTER_HOURS         0x06
#define RV8263C8_REGISTER_DATE          0x07
#define RV8263C8_REGISTER_WEEKDAY       0x08
#define RV8263C8_REGISTER_MONTH         0x09
#define RV8263C8_REGISTER_YEAR          0x0A
#define RV8263C8_REGISTER_SECONDS_ALARM 0x0B
#define RV8263C8_REGISTER_MINUTES_ALARM 0x0C
#define RV8263C8_REGISTER_HOURS_ALARM   0x0D
#define RV8263C8_REGISTER_DATE_ALARM    0x0E
#define RV8263C8_REGISTER_WEEKDAY_ALARM 0x0F
#define RV8263C8_REGISTER_TIMER_VALUE   0x10
#define RV8263C8_REGISTER_TIMER_MODE    0x11

#define RV8263C8_BM_24H_MODE_ENABLE         (0x00 << 1)
#define RV8263C8_BM_24H_MODE_DISABLE        (0x00 << 1)
#define RV8263C8_BM_CLOCK_ENABLE            (0x00 << 5)
#define RV8263C8_BM_CLOCK_DISABLE           (0x01 << 5)
#define RV8263C8_BM_ALARM_INT_ENABLE        (0x01 << 7)
#define RV8263C8_BM_ALARM_INT_DISABLE       (0x00 << 7)
#define RV8263C8_BM_MINUTE_INT_ENABLE       (0x01 << 5)
#define RV8263C8_BM_MINUTE_INT_DISABLE      (0x00 << 5)
#define RV8263C8_BM_HALF_MINUTE_INT_ENABLE  (0x01 << 4)
#define RV8263C8_BM_HALF_MINUTE_INT_DISABLE (0x00 << 4)
#define RV8263C8_BM_ALARM_ENABLE            (0x00 << 7)
#define RV8263C8_BM_ALARM_DISABLE           (0x01 << 7)
#define RV8263C8_BM_AF                      (0x01 << 6)
#define RV8263C8_BM_TF                      (0x01 << 3)
#define RV8263_BM_MODE                      (0x01 << 7)
#define RV8263_BM_TD_1HZ                    (0x02 << 3)
#define RV8263_BM_TE_ENABLE                 (0x01 << 2)
#define RV8263_BM_TIE_ENABLE                (0x01 << 1)
#define RV8263_BM_TI_TP_PULSE               (0x01 << 0)
#define RV8263C8_BM_SOFTWARE_RESET          (0x58)

#define SECONDS_BITS  GENMASK(6, 0)
#define MINUTES_BITS  GENMASK(7, 0)
#define HOURS_BITS    GENMASK(5, 0)
#define DATE_BITS     GENMASK(5, 0)
#define MONTHS_BITS   GENMASK(4, 0)
#define WEEKDAY_BITS  GENMASK(2, 0)
#define YEAR_BITS     GENMASK(7, 0)
#define VALIDATE_24HR BIT(6)

#define MIN_SEC       0
#define MAX_SEC       59
#define MIN_MIN       0
#define MAX_MIN       59
#define MIN_HOUR      0
#define MAX_HOUR      23
#define MAX_WDAY      7
#define MIN_WDAY      1
#define MAX_MDAY      31
#define MIN_MDAY      1
#define MAX_MON       12
#define MIN_MON       1
#define MIN_YEAR_DIFF 0
#define MAX_YEAR_DIFF 99

#define DT_DRV_COMPAT microcrystal_rv_8263_c8

LOG_MODULE_REGISTER(microcrystal_rv8263c8, CONFIG_RTC_LOG_LEVEL);

struct rv8263c8_config {
	struct i2c_dt_spec i2c_bus;
	uint8_t clkout;

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	struct gpio_dt_spec int_gpio;
#endif
};

struct rv8263c8_data {
	struct k_sem lock;

#if (CONFIG_RTC_ALARM || CONFIG_RTC_UPDATE) && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	const struct device *dev;
#endif

#if (CONFIG_RTC_ALARM || CONFIG_RTC_UPDATE) && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	struct gpio_callback gpio_cb;
#endif

#if CONFIG_RTC_ALARM && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	rtc_alarm_callback alarm_cb;
	void *alarm_cb_data;
	struct k_work alarm_work;
#endif

#if CONFIG_RTC_UPDATE && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	rtc_update_callback update_cb;
	void *update_cb_data;
	struct k_work update_work;
#endif
};

#if (CONFIG_RTC_ALARM || CONFIG_RTC_UPDATE) && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
static void rv8263c8_gpio_callback_handler(const struct device *p_port, struct gpio_callback *p_cb,
					   gpio_port_pins_t pins)
{
	ARG_UNUSED(pins);
	ARG_UNUSED(p_port);

	struct rv8263c8_data *data = CONTAINER_OF(p_cb, struct rv8263c8_data, gpio_cb);

#if CONFIG_RTC_ALARM && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	k_work_submit(&data->alarm_work);
#endif

#if CONFIG_RTC_UPDATE && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	k_work_submit(&data->update_work);
#endif
}
#endif

#if CONFIG_RTC_ALARM && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
static void rv8263c8_alarm_worker(struct k_work *p_work)
{
	struct rv8263c8_data *data = CONTAINER_OF(p_work, struct rv8263c8_data, alarm_work);
	const struct rv8263c8_config *config = data->dev->config;

	LOG_DBG("Process alarm interrupt from worker");

	if (data->alarm_cb != NULL) {
		uint8_t reg;

		i2c_reg_read_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_CONTROL_2, &reg);

		if (reg & RV8263C8_BM_AF) {
			reg &= ~RV8263C8_BM_AF;

			LOG_DBG("Calling alarm callback");
			data->alarm_cb(data->dev, 0, data->alarm_cb_data);

			i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_CONTROL_2, reg);
		}
	}
}
#endif

#if CONFIG_RTC_UPDATE && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
static int rv8263c8_update_enable_timer(const struct device *dev)
{
	int err;
	uint8_t regs;
	const struct rv8263c8_config *config = dev->config;

	regs = 0;
	err = i2c_burst_write_dt(&config->i2c_bus, RV8263C8_REGISTER_TIMER_VALUE, &regs,
				 sizeof(regs));
	if (err < 0) {
		return err;
	}

	regs = RV8263_BM_TD_1HZ | RV8263_BM_TE_ENABLE | RV8263_BM_TIE_ENABLE |
	       RV8263_BM_TI_TP_PULSE;
	err = i2c_burst_write_dt(&config->i2c_bus, RV8263C8_REGISTER_TIMER_MODE, &regs,
				 sizeof(regs));

	return 0;
}

static void rv8263c8_update_worker(struct k_work *p_work)
{
	uint8_t reg;
	struct rv8263c8_data *data = CONTAINER_OF(p_work, struct rv8263c8_data, update_work);
	const struct rv8263c8_config *config = data->dev->config;

	LOG_DBG("Process update interrupt from worker");

	if (data->update_cb != NULL) {
		i2c_reg_read_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_CONTROL_2, &reg);

		if (reg & RV8263C8_BM_TF) {
			LOG_DBG("Calling update callback");
			data->update_cb(data->dev, data->update_cb_data);
		}
	}

	rv8263c8_update_enable_timer(data->dev);
	i2c_reg_update_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_CONTROL_2, RV8263C8_BM_TF,
			       RV8263C8_BM_TF);
}
#endif

static int rv8263c8_time_set(const struct device *dev, const struct rtc_time *timeptr)
{
	uint8_t regs[7];
	const struct rv8263c8_config *config = dev->config;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	LOG_DBG("Set time: year = %u, mon = %u, mday = %u, wday = %u, hour = %u, min = %u, sec = "
		"%u",
		timeptr->tm_year, timeptr->tm_mon, timeptr->tm_mday, timeptr->tm_wday,
		timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);

	regs[0] = bin2bcd(timeptr->tm_sec) & SECONDS_BITS;
	regs[1] = bin2bcd(timeptr->tm_min);
	regs[2] = bin2bcd(timeptr->tm_hour);
	regs[3] = bin2bcd(timeptr->tm_wday);
	regs[4] = bin2bcd(timeptr->tm_mday);
	regs[5] = bin2bcd(timeptr->tm_mon);
	regs[6] = bin2bcd((timeptr->tm_year % 100));

	return i2c_burst_write_dt(&config->i2c_bus, RV8263C8_REGISTER_SECONDS, regs, sizeof(regs));
}

static int rv8263c8_time_get(const struct device *dev, struct rtc_time *timeptr)
{
	int err;
	uint8_t regs[7];
	const struct rv8263c8_config *config = dev->config;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	err = i2c_burst_read_dt(&config->i2c_bus, RV8263C8_REGISTER_SECONDS, regs, sizeof(regs));
	if (err < 0) {
		return err;
	}

	timeptr->tm_sec = bcd2bin(regs[0] & SECONDS_BITS);
	timeptr->tm_min = bcd2bin(regs[1] & MINUTES_BITS);
	timeptr->tm_hour = bcd2bin(regs[2] & HOURS_BITS);
	timeptr->tm_wday = bcd2bin(regs[3] & WEEKDAY_BITS);
	timeptr->tm_mday = bcd2bin(regs[4] & DATE_BITS);
	timeptr->tm_mon = bcd2bin(regs[5] & MONTHS_BITS);
	timeptr->tm_year = bcd2bin(regs[6] & YEAR_BITS);
	timeptr->tm_year = timeptr->tm_year + 100;

	/* Unused */
	timeptr->tm_nsec = 0;
	timeptr->tm_isdst = -1;
	timeptr->tm_yday = -1;

	/* Validate the chip in 24hr mode */
	if (regs[2] & VALIDATE_24HR) {
		return -ENODATA;
	}

	LOG_DBG("get time: year = %u, mon = %u, mday = %u, wday = %u, hour = %u, min = %u, sec = "
		"%u",
		timeptr->tm_year, timeptr->tm_mon, timeptr->tm_mday, timeptr->tm_wday,
		timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);

	return 0;
}

static int rv8263c8_init(const struct device *dev)
{
	int err;
	int temp;
	struct rv8263c8_data *data = dev->data;
	const struct rv8263c8_config *config = dev->config;

#if (CONFIG_RTC_ALARM || CONFIG_RTC_UPDATE) && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	if (config->int_gpio.port != NULL) {
		return -EINVAL;
	}
#endif

	if (!i2c_is_ready_dt(&config->i2c_bus)) {
		LOG_ERR("I2C bus not ready!");
		return -ENODEV;
	}

	LOG_DBG("Configure RV-8263-C8:");
	LOG_DBG("	ClkOut: %u", config->clkout);

	k_sem_init(&data->lock, 1, 1);

	err = i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_CONTROL_1,
				    RV8263C8_BM_24H_MODE_DISABLE | RV8263C8_BM_CLOCK_ENABLE);
	if (err < 0) {
		LOG_ERR("Error while writing CONTROL_1! Error: %i", err);
		return err;
	}

	temp = RV8263C8_BM_MINUTE_INT_DISABLE | RV8263C8_BM_HALF_MINUTE_INT_DISABLE |
	       (config->clkout << 0x00);

#if CONFIG_RTC_ALARM && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	temp |= RV8263C8_BM_ALARM_INT_ENABLE;
#endif
	err = i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_CONTROL_2, temp);
	if (err < 0) {
		LOG_ERR("Error while writing CONTROL_2! Error: %i", err);
		return err;
	}

#if CONFIG_RTC_UPDATE && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	uint8_t regs = 0;

	err = i2c_burst_write_dt(&config->i2c_bus, RV8263C8_REGISTER_TIMER_MODE, &regs,
				 sizeof(regs));
	if (err < 0) {
		LOG_ERR("Error while writing CONTROL2! Error: %i", err);
		return err;
	}
#endif

#if (CONFIG_RTC_ALARM || CONFIG_RTC_UPDATE) && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("GPIO not ready!");
		return err;
	}

	err = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	if (err < 0) {
		LOG_ERR("Failed to configure GPIO! Error: %u", err);
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_INACTIVE);
	if (err < 0) {
		LOG_ERR("Failed to configure interrupt! Error: %u", err);
		return err;
	}

	gpio_init_callback(&data->gpio_cb, rv8263c8_gpio_callback_handler,
			   BIT(config->int_gpio.pin));

	err = gpio_add_callback_dt(&config->int_gpio, &data->gpio_cb);
	if (err < 0) {
		LOG_ERR("Failed to add GPIO callback! Error: %u", err);
		return err;
	}
#endif

	(void)k_sem_take(&data->lock, K_FOREVER);
#if CONFIG_RTC_ALARM && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	data->alarm_work.handler = rv8263c8_alarm_worker;
#endif

#if CONFIG_RTC_UPDATE && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	data->update_work.handler = rv8263c8_update_worker;
#endif

#if (CONFIG_RTC_ALARM || CONFIG_RTC_UPDATE) && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	data->dev = dev;
#endif
	k_sem_give(&data->lock);

	return 0;
}

#if CONFIG_RTC_ALARM
static int rv8263c8_alarm_get_supported_fields(const struct device *dev, uint16_t id,
					       uint16_t *p_mask)
{
	ARG_UNUSED(dev);

	if (id != 0) {
		return -EINVAL;
	}

	(*p_mask) = (RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE |
		     RTC_ALARM_TIME_MASK_HOUR | RTC_ALARM_TIME_MASK_MONTHDAY |
		     RTC_ALARM_TIME_MASK_WEEKDAY);

	return 0;
}

static int rv8263c8_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				   const struct rtc_time *timeptr)
{
	int err;
	const struct rv8263c8_config *config = dev->config;

	if ((timeptr == NULL) || (id != 0) || ((mask > 0) && (timeptr == NULL))) {
		return -EINVAL;
	}

	if (!rtc_utils_validate_rtc_time(timeptr, mask)) {
		return -EINVAL;
	}

	if (mask & RTC_ALARM_TIME_MASK_SECOND) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_SECONDS_ALARM,
					    RV8263C8_BM_ALARM_ENABLE | timeptr->tm_sec);
	} else {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_SECONDS_ALARM,
					    RV8263C8_BM_ALARM_DISABLE);
	}

	if (err < 0) {
		LOG_ERR("Error while writing SECONDS alarm! Error: %i", err);
		return err;
	}

	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_MINUTES_ALARM,
					    RV8263C8_BM_ALARM_ENABLE | timeptr->tm_min);
	} else {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_MINUTES_ALARM,
					    RV8263C8_BM_ALARM_DISABLE);
	}

	if (err < 0) {
		LOG_ERR("Error while writing MINUTE alarm! Error: %i", err);
		return err;
	}

	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_HOURS_ALARM,
					    RV8263C8_BM_ALARM_ENABLE | timeptr->tm_hour);
	} else {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_HOURS_ALARM,
					    RV8263C8_BM_ALARM_DISABLE);
	}

	if (err < 0) {
		LOG_ERR("Error while writing HOUR alarm! Error: %i", err);
		return err;
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_DATE_ALARM,
					    RV8263C8_BM_ALARM_ENABLE | timeptr->tm_mday);
	} else {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_DATE_ALARM,
					    RV8263C8_BM_ALARM_DISABLE);
	}

	if (err < 0) {
		LOG_ERR("Error while writing MONTHDAY alarm! Error: %i", err);
		return err;
	}

	if (mask & RTC_ALARM_TIME_MASK_WEEKDAY) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_WEEKDAY_ALARM,
					    RV8263C8_BM_ALARM_ENABLE | timeptr->tm_wday);
	} else {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_WEEKDAY_ALARM,
					    RV8263C8_BM_ALARM_DISABLE);
	}

	if (err < 0) {
		LOG_ERR("Error while writing WEEKDAY alarm! Error: %i", err);
		return err;
	}

	return 0;
}

static int rv8263c8_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *p_mask,
				   struct rtc_time *timeptr)
{
	int err;
	const struct rv8263c8_config *config = dev->config;
	uint8_t value[5];

	if ((timeptr == NULL) || (id != 0)) {
		return -EINVAL;
	}

	(*p_mask) = 0;

	err = i2c_burst_read_dt(&config->i2c_bus, RV8263C8_REGISTER_SECONDS_ALARM, value,
				sizeof(value));
	if (err < 0) {
		LOG_ERR("Error while reading alarm! Error: %i", err);
		return err;
	}

	if (value[0] <= MAX_SEC) {
		timeptr->tm_sec = value[0];
		(*p_mask) |= RTC_ALARM_TIME_MASK_SECOND;
	}

	if (value[1] <= MAX_MIN) {
		timeptr->tm_min = value[1];
		(*p_mask) |= RTC_ALARM_TIME_MASK_MINUTE;
	}

	if (value[2] <= MAX_HOUR) {
		timeptr->tm_hour = value[2];
		(*p_mask) |= RTC_ALARM_TIME_MASK_HOUR;
	}

	if (value[3] <= MAX_MDAY) {
		timeptr->tm_mday = value[3];
		(*p_mask) |= RTC_ALARM_TIME_MASK_MONTHDAY;
	}

	if (value[4] <= MAX_WDAY) {
		timeptr->tm_hour = value[4];
		(*p_mask) |= RTC_ALARM_TIME_MASK_WEEKDAY;
	}

	return 0;
}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
static int rv8263c8_alarm_set_callback(const struct device *dev, uint16_t id,
				       rtc_alarm_callback callback, void *p_user_data)
{
	int err;
	struct rv8263c8_data *const data = dev->data;
	const struct rv8263c8_config *config = dev->config;

	if (id != 0) {
		return -EINVAL;
	}

	(void)k_sem_take(&data->lock, K_FOREVER);
	data->alarm_cb = callback;
	data->alarm_cb_data = p_user_data;
	k_sem_give(&data->lock);

	if (callback != NULL) {
		err = i2c_reg_update_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_CONTROL_2,
					     RV8263C8_BM_ALARM_INT_ENABLE,
					     RV8263C8_BM_ALARM_INT_ENABLE);
	} else {
		err = i2c_reg_update_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_CONTROL_2,
					     RV8263C8_BM_ALARM_INT_ENABLE,
					     RV8263C8_BM_ALARM_INT_DISABLE);
	}

	if (err < 0) {
		LOG_ERR("Error while writing CONTROL2! Error: %i", err);
		return err;
	}

	return 0;
}
#endif

static int rv8263c8_alarm_is_pending(const struct device *dev, uint16_t id)
{
	int err;
	uint8_t reg;
	const struct rv8263c8_config *config = dev->config;

	if (id != 0) {
		return -EINVAL;
	}

	err = i2c_reg_read_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_CONTROL_2, &reg);
	if (err) {
		return err;
	}

	if (reg & RV8263C8_BM_AF) {
		reg &= ~RV8263C8_BM_AF;
		err = i2c_reg_write_byte_dt(&config->i2c_bus, RV8263C8_REGISTER_CONTROL_2, reg);
		if (err) {
			return err;
		}

		return 1;
	}

	return 0;
}
#endif

#if CONFIG_RTC_UPDATE && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
int rv8263_update_callback(const struct device *dev, rtc_update_callback callback,
			   void *p_user_data)
{
	struct rv8263c8_data *const data = dev->data;

	(void)k_sem_take(&data->lock, K_FOREVER);
	data->update_cb = callback;
	data->update_cb_data = p_user_data;
	k_sem_give(&data->lock);

	return rv8263c8_update_enable_timer(dev);
}
#endif

#ifdef CONFIG_RTC_CALIBRATION
int rv8263c8_calibration_set(const struct device *dev, int32_t calibration)
{
	int err;
	int8_t reg;
	struct rv8263c8_data *const data = dev->data;
	const struct rv8263c8_config *config = dev->config;

	/* NOTE: The RTC API is using a PPB (Parts Per Billion) value. The RTC is using PPM. */

	return 0;
}

int rv8263c8_calibration_get(const struct device *dev, int32_t *calibration)
{
	int err;
	uint8_t value;
	struct rv8263c8_data *const data = dev->data;
	const struct rv8263c8_config *config = dev->config;

	if (calibration == NULL) {
		return -EINVAL;
	}

	return 0;
}
#endif

static const struct rtc_driver_api rv8263c8_driver_api = {
	.set_time = rv8263c8_time_set,
	.get_time = rv8263c8_time_get,
#if CONFIG_RTC_ALARM
	.alarm_get_supported_fields = rv8263c8_alarm_get_supported_fields,
	.alarm_set_time = rv8263c8_alarm_set_time,
	.alarm_get_time = rv8263c8_alarm_get_time,
	.alarm_is_pending = rv8263c8_alarm_is_pending,
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	.alarm_set_callback = rv8263c8_alarm_set_callback,
#endif
#endif
#if CONFIG_RTC_UPDATE && DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	.update_set_callback = rv8263_update_callback,
#endif
#ifdef CONFIG_RTC_CALIBRATION
	.set_calibration = rv8263c8_calibration_set,
	.get_calibration = rv8263c8_calibration_get,
#endif
};

#define RV8263_DEFINE(inst)                                                                        \
	static struct rv8263c8_data rv8263c8_data_##inst;                                          \
	static const struct rv8263c8_config rv8263c8_config_##inst = {                             \
		.i2c_bus = I2C_DT_SPEC_INST_GET(inst),                                             \
		.clkout = DT_INST_PROP_OR(inst, clkout, 0),                                        \
		IF_ENABLED(DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios),                            \
			   (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),))};       \
	DEVICE_DT_INST_DEFINE(inst, &rv8263c8_init, NULL, &rv8263c8_data_##inst,                   \
			      &rv8263c8_config_##inst, POST_KERNEL, CONFIG_RTC_INIT_PRIORITY,      \
			      &rv8263c8_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RV8263_DEFINE)
