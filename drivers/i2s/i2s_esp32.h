#pragma once

#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma.h>

#include "esp_types.h"
#include "esp_err.h"
#include "soc/i2s_periph.h"
#include "soc/rtc_periph.h"
/*#include "soc/soc_caps.h"*/
#include "hal/i2s_hal.h"
#include "esp_intr_alloc.h"

struct queue_item {
	void *mem_block;
	size_t size;
};

/* Minimal ring buffer implementation */
struct ring_buf {
	struct queue_item *buf;
	uint16_t len;
	uint16_t head;
	uint16_t tail;
};

struct stream {
	int32_t state;
	struct k_sem sem;

	const struct device *dev_dma;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	uint8_t priority;
	uint8_t fifo_threshold;

	struct i2s_config i2s_cfg;
	struct ring_buf mem_block_queue;
	void *mem_block;
	bool last_block;
	bool master;
	int (*stream_start)(struct stream *stream, const struct device *dev);
	void (*stream_stop)(struct stream *stream, const struct device *dev);
	void (*queue_drop)(struct stream *stream);

	void *i2s_blk_addr;
};

/* Device constant configuration parameters */
struct i2s_esp32_cfg {
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config)(const struct device *dev);
	const int i2s_num;
};

/* Device run time data */
struct i2s_esp32_data {
	i2s_hal_context_t hal_cxt;
	i2s_hal_clock_info_t clk_info;
	struct stream rx_stream;
	struct stream tx_stream;
};

#ifdef __cplusplus
}
#endif
