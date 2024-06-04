/*
 * Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT worldsemi_ws2812_uart

#include <zephyr/drivers/led_strip.h>

#include <string.h>

#define LOG_LEVEL CONFIG_LED_STRIP_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ws2812_uart);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/led/led.h>

struct ws2812_uart_cfg {
	const struct device *uart_dev; /* UART device */
	uint8_t *px_buf;
	size_t length;
	uint8_t num_colors;
	const uint8_t *color_mapping;
	const uint8_t *uart_lookup_table;
	uint8_t bits_per_rgb_bit;
	uint8_t data_bits;
	uint16_t reset_delay;
};

/*
 * Serialize a 24-bit color channel value into an equivalent sequence
 * of 8 bytes of UART frames,.
 */
static inline int ws2812_uart_ser(const struct device *dev, uint8_t *buf, uint8_t color[3])
{
	/* Explaination of serialisation for WS2812 ic:
	 * 3 pulses/bits of UART are used to represent one bit of RGB data.
	 * During the development of this driver, I was using an ESP32C3 MCU
	 * with its UART baud rate set to 2.5Mbits per sec.Hence,1 bit equals 400ns.
	 * For WS2812 timing details, please refer to its datasheet.
	 * Link: https://cdn-shop.adafruit.com/datasheets/WS2812.pdf
	 * We can represent logic 1 with 0b110 (first two bits high,
	 * remaining one bit low),
	 * and logic 0 with 0b100 (first bit high, remaining two bits low).
	 * We also need to invert the polarity of UART pins because in default,
	 * the idle TX line is high, but WS2812 or similar LEDs need the idle
	 * line to be low, otherwise, it will misinterpret.
	 * When we make the TX line inverted, the start bit will be low to high
	 * and the stop bit will be high to low.
	 * Hence, the nine bits we are sending in one go will always have 1 at
	 * the start and 0 at the end, we just need to calculate middle 7 bits.
	 * Also, keep in mind that UART always sends the MSB first.
	 * Keeping all this data in mind, I created a lookup table to ease the
	 * calculations.
	 * For example, to drive WS2812 led this function is just taking three bits at a time from
	 * the 24-bit color data, i.e., color[3] array (MSB first), and passing those three bits
	 * into the lookup table ws2812_uart_lut[] to retrieve the 7-bit data and fills out the
	 * serialized output buffer. This driver can be used with other ic of different timing
	 * requirements by simply changing baudrate, data-bits, bits per rgb bit and lookup table.
	 * lookup_table[0] = 0x5B->1011011(7-data-bits)-> 1+1011011+0(start/stop bits added) ->
	 * 110 110 110(3 UART bits per RGB bit)-> 1 1 1(RGB bits).
	 * lookup_table[1] = 0x1B->0011011(7-data-bits)-> 1+0011011+0(start/stop bits added) ->
	 * 100 110 110(3 UART bits per RGB bit)-> 0 1 1(RGB bits)
	 */

	int i, j;
	int buff_indx = 0;
	uint8_t serialized_val = 0;
	const struct ws2812_uart_cfg *cfg = dev->config;

	/*Total RGB bits send per transaction*/
	int rgb_bits_send_per_trans = (cfg->data_bits + 2) / cfg->bits_per_rgb_bit;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 8; j++) {
			serialized_val = (serialized_val << 1) | ((color[i] >> (7 - j)) & 0x01);
			if (((i * 8) + j) % rgb_bits_send_per_trans ==
			    (rgb_bits_send_per_trans - 1)) {
				buf[buff_indx++] = cfg->uart_lookup_table[serialized_val];
				serialized_val = 0;
			}
		}
	}
	return buff_indx;
}

static void ws2812_uart_tx(const struct ws2812_uart_cfg *cfg, uint8_t *tx, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(cfg->uart_dev, tx[i]);
	}
}

static int ws2812_strip_update_rgb(const struct device *dev, struct led_rgb *pixels,
				   size_t num_pixels)
{
	const struct ws2812_uart_cfg *cfg = dev->config;

	uint8_t *px_buf = cfg->px_buf;
	size_t px_buf_len = cfg->length * cfg->num_colors * 8;
	size_t uart_data_len = 0;
	size_t i;
	uint8_t color[3];

	/*
	 * Convert pixel data into UART frames. Each frame has pixel data
	 * in color mapping on-wire format (e.g. GRB, GRBW, RGB, etc).
	 */

	for (i = 0; i < num_pixels; i++) {
		uint8_t j;

		for (j = 0; j < cfg->num_colors; j++) {
			switch (cfg->color_mapping[j]) {
			/* White channel is not supported by LED strip API. */
			case LED_COLOR_ID_WHITE:
				color[j] = 0;
				break;
			case LED_COLOR_ID_RED:
				color[j] = pixels[i].r;
				break;
			case LED_COLOR_ID_GREEN:
				color[j] = pixels[i].g;
				break;
			case LED_COLOR_ID_BLUE:
				color[j] = pixels[i].b;
				break;
			default:
				return -EINVAL;
			}
		}
		uart_data_len += ws2812_uart_ser(dev, px_buf, &color[0]);
		if (uart_data_len <= px_buf_len) {
			px_buf = cfg->px_buf + uart_data_len;
		} else {
			return -ENOMEM;
		}
	}
	ws2812_uart_tx(cfg, cfg->px_buf, uart_data_len);
	k_usleep(cfg->reset_delay);

	return 0;
}

static int ws2812_uart_init(const struct device *dev)
{
	const struct ws2812_uart_cfg *cfg = dev->config;
	uint8_t i;

	if (!device_is_ready(cfg->uart_dev)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	for (i = 0; i < cfg->num_colors; i++) {
		switch (cfg->color_mapping[i]) {
		case LED_COLOR_ID_WHITE:
		case LED_COLOR_ID_RED:
		case LED_COLOR_ID_GREEN:
		case LED_COLOR_ID_BLUE:
			break;
		default:
			LOG_ERR("%s: invalid channel to color mapping."
				"Check the color-mapping DT property",
				dev->name);
			return -EINVAL;
		}
	}

	return 0;
}

static size_t ws2812_strip_length(const struct device *dev)
{
	const struct ws2812_uart_cfg *cfg = dev->config;

	return cfg->length;
}

static const struct led_strip_driver_api ws2812_uart_api = {
	.update_rgb = ws2812_strip_update_rgb,
	.length = ws2812_strip_length,
};

#define WS2812_DEFAULT_RESET_DELAY_MS (50)
#define WS2812_UART_NUM_PIXELS(idx)   (DT_INST_PROP(idx, chain_length))
#define WS2812_UART_BUFSZ(idx)        (WS2812_NUM_COLORS(idx) * 8 * WS2812_UART_NUM_PIXELS(idx))

/*
 * Retrieve the channel to color mapping (e.g. RGB, BGR, GRB, ...) from the
 * "color-mapping" DT property.
 */
#define WS2812_COLOR_MAPPING(idx)                                                                  \
	static const uint8_t ws2812_uart_##idx##_color_mapping[] = DT_INST_PROP(idx, color_mapping)

#define WS2812_UART_LUT(idx)                                                                       \
	static const uint8_t ws2812_uart_##idx##_lut[] = DT_INST_PROP(idx, lookup_table)

#define WS2812_NUM_COLORS(idx) (DT_INST_PROP_LEN(idx, color_mapping))

#define WS2812_UART_LUT_LEN(idx) (DT_INST_PROP_LEN(idx, lookup_table))

/* Get the latch/reset delay from the "reset-delay" DT property. */
#define WS2812_RESET_DELAY(idx) DT_INST_PROP_OR(idx, reset_delay, WS2812_DEFAULT_RESET_DELAY_MS)

#define UART_NODE(idx) DT_PARENT(DT_INST(idx, worldsemi_ws2812_uart))

#define BITS_PER_RGB_BIT(idx) DT_INST_PROP(idx, bits_per_rgb_bit)

#define UART_DATA_BITS(idx) DT_PROP(UART_NODE(idx), data_bits)

#define WS2812_UART_DEVICE(idx)                                                                    \
                                                                                                   \
	BUILD_ASSERT(DT_PROP(UART_NODE(idx), tx_invert),                                           \
		     "This driver depends on tx-invert UART property and "                         \
		     "is not set. Please check your device-tree settings");                        \
                                                                                                   \
	BUILD_ASSERT(0 == (((UART_DATA_BITS(idx) + 2) % BITS_PER_RGB_BIT(idx))),                   \
		     "UART data bits+2 should be multiple of bits_per_rgb_bit"                     \
		     "property.Please check your device-tree settings");                           \
                                                                                                   \
	BUILD_ASSERT(WS2812_UART_LUT_LEN(idx) ==                                                   \
			     1 << ((UART_DATA_BITS(idx) + 2) / BITS_PER_RGB_BIT(idx)),             \
		     "UART lookup_table property's length != 2^((data-bits+2)/bits-per-rgb-bit)"   \
		     ".Please check your device-tree settings");                                   \
                                                                                                   \
	BUILD_ASSERT(DT_PROP(UART_NODE(idx), data_bits), "data-bits property missing");            \
                                                                                                   \
	static uint8_t ws2812_uart_##idx##_px_buf[WS2812_UART_BUFSZ(idx)];                         \
                                                                                                   \
	WS2812_COLOR_MAPPING(idx);                                                                 \
	WS2812_UART_LUT(idx);                                                                      \
                                                                                                   \
	static const struct ws2812_uart_cfg ws2812_uart_##idx##_cfg = {                            \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(idx)),                                       \
		.px_buf = ws2812_uart_##idx##_px_buf,                                              \
		.num_colors = WS2812_NUM_COLORS(idx),                                              \
		.color_mapping = ws2812_uart_##idx##_color_mapping,                                \
		.reset_delay = MAX(WS2812_RESET_DELAY(idx), 8),                                    \
		.length = WS2812_UART_NUM_PIXELS(idx),                                             \
		.uart_lookup_table = ws2812_uart_##idx##_lut,                                      \
		.data_bits = UART_DATA_BITS(idx),                                                  \
		.bits_per_rgb_bit = BITS_PER_RGB_BIT(idx),                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, ws2812_uart_init, NULL, NULL, &ws2812_uart_##idx##_cfg,         \
			      POST_KERNEL, CONFIG_LED_STRIP_INIT_PRIORITY, &ws2812_uart_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_UART_DEVICE)
