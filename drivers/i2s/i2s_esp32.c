#define DT_DRV_COMPAT espressif_esp32_i2s

#include <string.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma/dma_esp32.h>
#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include "esp_check.h"
#include "i2s_esp32.h"
LOG_MODULE_REGISTER(i2s_esp32);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#error "I2S series coder is not defined in DTS"
#endif

#include "esp_clk_tree.h"
#include "clk_ctrl_os.h"

#if SOC_I2S_SUPPORTS_APLL
#define IS2_ESP32_CLK_SRC  I2S_CLK_SRC_APLL
#else
#define IS2_ESP32_CLK_SRC  I2S_CLK_SRC_DEFAULT
#endif

#if SOC_I2S_HW_VERSION_2
#if IS2_ESP32_CLK_SRC == I2S_CLK_SRC_EXTERNAL
#define CONFIG_I2S_CLK_SRC_EXTERNAL_FREQ 25000000
#endif /* IS2_ESP32_CLK_SRC == I2S_CLK_SRC_EXTERNAL */
#endif /* SOC_I2S_HW_VERSION_2 */

#define I2S_ESP32_RX_BLOCK_COUNT 1
#define I2S_ESP32_TX_BLOCK_COUNT 1

#define I2S_ESP32_MODULO_INC(val, max) { val = (++val < max) ? val : 0; }

#define ESP32_DMA_NUM_CHANNELS 8
static const struct device *i2s_esp32_active_dma_rx_channel[ESP32_DMA_NUM_CHANNELS];
static const struct device *i2s_esp32_active_dma_tx_channel[ESP32_DMA_NUM_CHANNELS];

static int i2s_esp32_enable_clock(const struct device *dev)
{
	const struct i2s_esp32_cfg *dev_cfg = dev->config;
	const struct device *clk_dev = dev_cfg->clock_dev;
	int ret;

	if (!device_is_ready(clk_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(clk_dev, dev_cfg->clock_subsys);
	if (ret != 0) {
		LOG_ERR("Could not enable I2S clock");
		return -EIO;
	}

	return 0;
}

#if SOC_I2S_SUPPORTS_APLL

static uint32_t i2s_esp32_set_get_apll_freq(uint32_t mclk_freq_hz)
{
	/* Calculate the expected APLL  */
	int mclk_div = (int)((SOC_APLL_MIN_HZ / mclk_freq_hz) + 1);

	/* apll_freq = mclk * div
	 * when div = 1, hardware will still divide 2
	 * when div = 0, the final mclk will be unpredictable
	 * So the div here should be at least 2
	 */
	mclk_div = mclk_div < 2 ? 2 : mclk_div;

	uint32_t expt_freq = mclk_freq_hz * mclk_div;

	if (expt_freq > SOC_APLL_MIN_HZ) {
		LOG_ERR("The required APLL frequency exceed its maximum value");
		return 0;
	}

	uint32_t real_freq = 0;
	esp_err_t ret = periph_rtc_apll_freq_set(expt_freq, &real_freq);

	if (ret == ESP_ERR_INVALID_ARG) {
		LOG_ERR("Set APLL freq failed due to invalid argument");
		return 0;
	}

	if (ret == ESP_ERR_INVALID_STATE) {
		LOG_WRN("APLL is occupied already, it is working at %"PRIu32" Hz while "
			"the expected  frequency is %"PRIu32" Hz", real_freq, expt_freq);
		LOG_WRN("Trying to work at %"PRIu32" Hz...", real_freq);
	}

	LOG_DBG("APLL expected frequency is %"PRIu32" Hz, real frequency is %"PRIu32" Hz",
		expt_freq, real_freq);

	return real_freq;
}
#endif

uint32_t i2s_esp32_get_source_clk_freq(i2s_clock_src_t clk_src, uint32_t mclk_freq_hz)
{
	uint32_t clk_freq = 0;
#if SOC_I2S_SUPPORTS_APLL
	if (clk_src == I2S_CLK_SRC_APLL) {
		return i2s_esp32_set_get_apll_freq(mclk_freq_hz);
	}
#endif
	esp_clk_tree_src_get_freq_hz(clk_src, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &clk_freq);
	return clk_freq;
}

#if SOC_I2S_SUPPORTS_ADC
#if CONFIG_I2S_ESP32_MODE_ADC_BUILT_IN || CONFIG_I2S_ESP32_MODE_DAC_BUILT_IN
static esp_err_t i2s_calculate_adc_dac_clock(const struct i2s_config *i2s_cfg,
					     i2s_hal_clock_info_t *i2s_hal_clock_info)
{
	return ESP_OK;
}
#endif /* CONFIG_I2S_ESP32_MODE_ADC_BUILT_IN || CONFIG_I2S_ESP32_MODE_DAC_BUILT_IN */
#endif /* SOC_I2S_SUPPORTS_ADC */

#if SOC_I2S_SUPPORTS_PDM_TX && CONFIG_I2S_ESP32_MODE_PDM_TX
static esp_err_t i2s_calculate_pdm_tx_clock(const struct i2s_config *i2s_cfg,
					    i2s_hal_clock_info_t *i2s_hal_clock_info)
{
	return ESP_OK;
}
#endif /* SOC_I2S_SUPPORTS_PDM_TX && CONFIG_I2S_ESP32_MODE_PDM_TX */

#if SOC_I2S_SUPPORTS_PDM_RX && CONFIG_I2S_ESP32_MODE_PDM_RX
static esp_err_t i2s_calculate_pdm_rx_clock(const struct i2s_config *i2s_cfg,
					    i2s_hal_clock_info_t *i2s_hal_clock_info)
{
	return ESP_OK;
}
#endif /* SOC_I2S_SUPPORTS_PDM_RX && CONFIG_I2S_ESP32_MODE_PDM_RX */

static esp_err_t i2s_esp32_calculate_common_clock(const struct i2s_config *i2s_cfg,
						  i2s_hal_clock_info_t *i2s_hal_clock_info)
{
	if (i2s_cfg == NULL) {
		LOG_ERR("Input i2s_cfg is NULL");
		return ESP_ERR_INVALID_ARG;
	}

	if (i2s_hal_clock_info == NULL) {
		LOG_ERR("Input hal_clock_info is NULL");
		return ESP_ERR_INVALID_ARG;
	}

	/* For words greater than 16-bit the channel length is considered 32-bit */
	const uint8_t channel_length = i2s_cfg->word_size > 16U ? 32U : 16U;

	uint16_t mclk_multiple = 256;

	if (i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE ||
	    i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) {
		i2s_hal_clock_info->bclk_div = 8;
		i2s_hal_clock_info->bclk = i2s_cfg->frame_clk_freq * i2s_cfg->channels
					   * channel_length;
		i2s_hal_clock_info->mclk = i2s_cfg->frame_clk_freq
					   * i2s_hal_clock_info->bclk_div;
	} else {
		i2s_hal_clock_info->bclk = i2s_cfg->frame_clk_freq * i2s_cfg->channels
					   * channel_length;
		i2s_hal_clock_info->mclk = i2s_cfg->frame_clk_freq * mclk_multiple;
		i2s_hal_clock_info->bclk_div = i2s_hal_clock_info->mclk
					       / i2s_hal_clock_info->bclk;
	}

#if SOC_I2S_HW_VERSION_2

#if IS2_ESP32_CLK_SRC == I2S_CLK_SRC_EXTERNAL
	i2s_hal_clock_info->sclk = CONFIG_I2S_CLK_SRC_EXTERNAL_FREQ;
#else
	i2s_hal_clock_info->sclk =
	       i2s_esp32_get_source_clk_freq(IS2_ESP32_CLK_SRC, i2s_hal_clock_info->mclk);
#endif /* IS2_ESP32_CLK_SRC == I2S_CLK_SRC_EXTERNAL */

#else
	i2s_hal_clock_info->sclk =
	       i2s_esp32_get_source_clk_freq(IS2_ESP32_CLK_SRC, i2s_hal_clock_info->mclk);
#endif /*SOC_I2S_HW_VERSION_2 */

	i2s_hal_clock_info->mclk_div = i2s_hal_clock_info->sclk /
				       i2s_hal_clock_info->mclk;
	if (i2s_hal_clock_info->mclk_div == 0) {
		LOG_ERR("Sample rate is too large for the current clock source");
		return ESP_ERR_INVALID_ARG;
	}

	return ESP_OK;
}

static esp_err_t i2s_esp32_calculate_clock(const struct i2s_config *i2s_cfg,
					   i2s_hal_clock_info_t *i2s_hal_clock_info)
{
	esp_err_t ret;

#if SOC_I2S_SUPPORTS_ADC && CONFIG_I2S_ESP32_MODE_ADC_BUILT_IN
	ret = i2s_calculate_adc_dac_clock(i2s_cfg, i2s_hal_clock_info);
	if (ret != ESP_OK) {
		LOG_ERR("ADC clock calculate failed");
	}
	return ret;
#endif /* SOC_I2S_SUPPORTS_ADC && CONFIG_I2S_ESP32_MODE_ADC_BUILT_IN */

#if SOC_I2S_SUPPORTS_DAC && CONFIG_I2S_ESP32_MODE_DAC_BUILT_IN
	ret = i2s_calculate_adc_dac_clock(i2s_cfg, i2s_hal_clock_info);
	if (ret != ESP_OK) {
		LOG_ERR("DAC clock calculate failed");
	}
	return ret;
#endif /* SOC_I2S_SUPPORTS_DAC && CONFIG_I2S_ESP32_MODE_DAC_BUILT_IN */

#if SOC_I2S_SUPPORTS_PDM_TX && CONFIG_I2S_ESP32_MODE_PDM_TX
	ret = i2s_calculate_pdm_tx_clock(i2s_cfg, i2s_hal_clock_info);
	if (ret != ESP_OK) {
		LOG_ERR("PDM TX clock calculate failed");
	}
	return ret;
#endif /* SOC_I2S_SUPPORTS_PDM_TX && CONFIG_I2S_ESP32_MODE_PDM_TX */

#if SOC_I2S_SUPPORTS_PDM_RX && CONFIG_I2S_ESP32_MODE_PDM_RX
	ret = i2s_calculate_pdm_rx_clock(i2s_cfg, i2s_hal_clock_info);
	if (ret != ESP_OK) {
		LOG_ERR("PDM RX clock calculate failed");
	}
	return ret;
#endif /* SOC_I2S_SUPPORTS_PDM_RX && CONFIG_I2S_ESP32_MODE_PDM_RX */

	ret = i2s_esp32_calculate_common_clock(i2s_cfg, i2s_hal_clock_info);
	if (ret != ESP_OK) {
		LOG_ERR("Common clock calculate failed");
	}
	return ret;
}

static void i2s_esp32_isr(const struct device *dev)
{
	const struct i2s_esp32_cfg *dev_cfg = dev->config;
	struct i2s_esp32_data *const dev_data = dev->data;
	struct stream *stream = &dev_data->rx_stream;

	stream->state = I2S_STATE_ERROR;
}
/*
static int i2s_esp32_queue_get(struct ring_buf *rb, void **mem_block, size_t *size)
{
	unsigned int key;

	key = irq_lock();

	if (rb->tail == rb->head) {
		irq_unlock(key);
		return -ENOMEM;
	}

	*mem_block = rb->buf[rb->tail].mem_block;
	*size = rb->buf[rb->tail].size;
	I2S_ESP32_MODULO_INC(rb->tail, rb->len);

	irq_unlock(key);

	return 0;
}

static int i2s_esp32_queue_put(struct ring_buf *rb, void *mem_block, size_t size)
{
	uint16_t head_next;
	unsigned int key;

	key = irq_lock();

	head_next = rb->head;
	I2S_ESP32_MODULO_INC(head_next, rb->len);

	if (head_next == rb->tail) {
		irq_unlock(key);
		return -ENOMEM;
	}

	rb->buf[rb->head].mem_block = mem_block;
	rb->buf[rb->head].size = size;
	rb->head = head_next;

	irq_unlock(key);

	return 0;
}

static void i2s_esp32_rx_queue_drop(struct stream *stream)
{
	size_t size;
	void *mem_block;

	while (i2s_esp32_queue_get(&stream->mem_block_queue, &mem_block, &size) == 0) {
		k_mem_slab_free(stream->i2s_cfg.mem_slab, &mem_block);
	}

	k_sem_reset(&stream->sem);
}

static void i2s_esp32_tx_queue_drop(struct stream *stream)
{
	size_t size;
	void *mem_block;
	unsigned int n = 0U;

	while (i2s_esp32_queue_get(&stream->mem_block_queue, &mem_block, &size) == 0) {
		k_mem_slab_free(stream->i2s_cfg.mem_slab, &mem_block);
		n++;
	}

	for (; n > 0; n--) {
		k_sem_give(&stream->sem);
	}
}

static int i2s_esp32_reload_dma(const struct device *dev_dma, uint32_t channel,
				struct dma_config *dcfg, void *src, void *dst,
				uint32_t blk_size)
{
	int ret;

	ret = dma_reload(dev_dma, channel, (uint32_t)src, (uint32_t)dst, blk_size);
	if (ret < 0) {
		return ret;
	}

	ret = dma_start(dev_dma, channel);

	return ret;
}

static int i2s_esp32_start_dma(const struct device *dev_dma, uint32_t channel,
			       struct dma_config *dcfg, void *src, void *dst,
			       uint8_t fifo_threshold, uint32_t blk_size)
{
	struct dma_block_config blk_cfg;
	int ret;

	memset(&blk_cfg, 0, sizeof(blk_cfg));
	blk_cfg.block_size = blk_size;
	blk_cfg.source_address = (uint32_t)src;
	blk_cfg.dest_address = (uint32_t)dst;

	blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	blk_cfg.fifo_mode_control = fifo_threshold;

	dcfg->head_block = &blk_cfg;

	ret = dma_config(dev_dma, channel, dcfg);
	if (ret < 0) {
		return ret;
	}

	ret = dma_start(dev_dma, channel);

	return ret;
}

static int i2s_esp32_rx_stream_start(struct stream *stream, const struct device *dev)
{
	struct i2s_esp32_data *const dev_data = dev->data;
	i2s_hal_context_t *hal_cxt = &dev_data->hal_cxt;
	int ret;

	ret = k_mem_slab_alloc(stream->i2s_cfg.mem_slab, &stream->mem_block, K_NO_WAIT);
	if (ret < 0) {
		return ret;
	}

	i2s_hal_rx_stop(hal_cxt);
	i2s_hal_rx_reset(hal_cxt);
	i2s_hal_rx_reset_fifo(hal_cxt);

	i2s_esp32_active_dma_rx_channel[stream->dma_channel] = dev;

	ret = i2s_esp32_start_dma(stream->dev_dma, stream->dma_channel, &stream->dma_cfg,
				  stream->i2s_blk_addr, stream->mem_block, stream->fifo_threshold,
				  stream->i2s_cfg.block_size);
	if (ret < 0) {
		LOG_ERR("Failed to start RX DMA transfer: %d", ret);
		return ret;
	}

	i2s_hal_rx_start(hal_cxt);

	return 0;
}

static int i2s_esp32_tx_stream_start(struct stream *stream, const struct device *dev)
{
	struct i2s_esp32_data *const dev_data = dev->data;
	i2s_hal_context_t *hal_cxt = &dev_data->hal_cxt;
	size_t mem_block_size;
	int ret;

	ret = i2s_esp32_queue_get(&stream->mem_block_queue, &stream->mem_block, &mem_block_size);
	if (ret < 0) {
		return ret;
	}
	k_sem_give(&stream->sem);

	i2s_hal_tx_stop(hal_cxt);
	i2s_hal_tx_reset(hal_cxt);
	i2s_hal_tx_reset_fifo(hal_cxt);

	i2s_esp32_active_dma_tx_channel[stream->dma_channel] = dev;

	ret = i2s_esp32_start_dma(stream->dev_dma, stream->dma_channel, &stream->dma_cfg,
				  stream->mem_block, stream->i2s_blk_addr,
				  stream->fifo_threshold, stream->i2s_cfg.block_size);
	if (ret < 0) {
		LOG_ERR("Failed to start TX DMA transfer: %d", ret);
		return ret;
	}

	i2s_hal_tx_start(hal_cxt);

	return 0;
}

static void i2s_esp32_rx_stream_stop(struct stream *stream, const struct device *dev)
{
	struct i2s_esp32_data *const dev_data = dev->data;
	i2s_hal_context_t *hal_cxt = &dev_data->hal_cxt;

	dma_stop(stream->dev_dma, stream->dma_channel);
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->i2s_cfg.mem_slab, &stream->mem_block);
		stream->mem_block = NULL;
	}

	i2s_hal_rx_stop(hal_cxt);

	i2s_esp32_active_dma_rx_channel[stream->dma_channel] = NULL;
}

static void i2s_esp32_tx_stream_stop(struct stream *stream, const struct device *dev)
{
	struct i2s_esp32_data *const dev_data = dev->data;
	i2s_hal_context_t *hal_cxt = &dev_data->hal_cxt;

	dma_stop(stream->dev_dma, stream->dma_channel);
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->i2s_cfg.mem_slab, &stream->mem_block);
		stream->mem_block = NULL;
	}

	i2s_hal_rx_stop(hal_cxt);

	i2s_esp32_active_dma_tx_channel[stream->dma_channel] = NULL;
}

static const struct device *i2s_esp32_get_dev_from_rx_dma_channel(uint32_t dma_channel)
{
	return i2s_esp32_active_dma_rx_channel[dma_channel];
}

static const struct device *i2s_esp32_get_dev_from_tx_dma_channel(uint32_t dma_channel)
{
	return i2s_esp32_active_dma_tx_channel[dma_channel];
}

static void i2s_esp32_dma_rx_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				      int status)
{
	const struct device *dev = i2s_esp32_get_dev_from_rx_dma_channel(channel);
	const struct i2s_esp32_cfg *dev_cfg = dev->config;
	struct i2s_esp32_data *const dev_data = dev->data;
	struct stream *stream = &dev_data->rx_stream;
	void *mblk_tmp;
	int ret;

	if (status < 0) {
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}

	__ASSERT_NO_MSG(stream->mem_block != NULL);

	if (stream->state == I2S_STATE_ERROR) {
		goto rx_disable;
	}

	mblk_tmp = stream->mem_block;

	ret = k_mem_slab_alloc(stream->i2s_cfg.mem_slab, &stream->mem_block, K_NO_WAIT);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}

	ret = i2s_esp32_reload_dma(stream->dev_dma, stream->dma_channel, &stream->dma_cfg,
				   stream->i2s_blk_addr, stream->mem_block,
				   stream->i2s_cfg.block_size);
	if (ret < 0) {
		goto rx_disable;
	}

	ret = i2s_esp32_queue_put(&stream->mem_block_queue, mblk_tmp, stream->i2s_cfg.block_size);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}
	k_sem_give(&stream->sem);

	if (stream->state == I2S_STATE_STOPPING) {
		stream->state = I2S_STATE_READY;
		goto rx_disable;
	}

	return;

rx_disable:
	i2s_esp32_rx_stream_stop(stream, dev);
}

static void i2s_esp32_dma_tx_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				      int status)
{
	const struct device *dev = i2s_esp32_get_dev_from_tx_dma_channel(channel);
	const struct i2s_esp32_cfg *dev_cfg = dev->config;
	struct i2s_esp32_data *const dev_data = dev->data;
	struct stream *stream = &dev_data->rx_stream;
	size_t mem_block_size;
	int ret;

	if (status < 0) {
		ret = -EIO;
		stream->state = I2S_STATE_ERROR;
		goto tx_disable;
	}

	__ASSERT_NO_MSG(stream->mem_block != NULL);

	k_mem_slab_free(stream->i2s_cfg.mem_slab, &stream->mem_block);
	stream->mem_block = NULL;

	if (stream->state == I2S_STATE_ERROR) {
		LOG_ERR("TX error detected");
		goto tx_disable;
	}

	if (stream->last_block) {
		stream->state = I2S_STATE_READY;
		goto tx_disable;
	}

	ret = i2s_esp32_queue_get(&stream->mem_block_queue, &stream->mem_block, &mem_block_size);
	if (ret < 0) {
		if (stream->state == I2S_STATE_STOPPING) {
			stream->state = I2S_STATE_READY;
		} else {
			stream->state = I2S_STATE_ERROR;
		}
		goto tx_disable;
	}
	k_sem_give(&stream->sem);

	ret = i2s_esp32_reload_dma(stream->dev_dma, stream->dma_channel, &stream->dma_cfg,
				   stream->mem_block, stream->i2s_blk_addr,
				   stream->i2s_cfg.block_size);
	if (ret < 0) {
		goto tx_disable;
	}

	return;

tx_disable:
	i2s_esp32_tx_stream_stop(stream, dev);
}
*/
static int i2s_esp32_configure(const struct device *dev, enum i2s_dir dir,
			       const struct i2s_config *i2s_cfg)
{
	const struct i2s_esp32_cfg *const dev_cfg = dev->config;
	struct i2s_esp32_data *const dev_data = dev->data;
	struct stream *stream;

	switch (dir) {
	case I2S_DIR_RX:
		stream = &dev_data->rx_stream;
		break;
	case I2S_DIR_TX:
		stream = &dev_data->tx_stream;
		break;
	case I2S_DIR_BOTH:
		LOG_ERR("I2S_DIR_BOTH is not supported");
		return -ENOSYS;
	default:
		LOG_ERR("Either RX or TX direction must be selected");
		return -EINVAL;
	}

	if (stream->state != I2S_STATE_NOT_READY &&
	    stream->state != I2S_STATE_READY) {
		LOG_ERR("invalid state");
		return -EINVAL;
	}

	stream->master = true;
	if (i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE ||
	    i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) {
		stream->master = false;
	}

	if (i2s_cfg->frame_clk_freq == 0U) {
		/*
		stream->queue_drop(stream);
		*/
		memset(&stream->i2s_cfg, 0, sizeof(struct i2s_config));
		stream->state = I2S_STATE_NOT_READY;
		return 0;
	}

	memcpy(&stream->i2s_cfg, i2s_cfg, sizeof(struct i2s_config));

	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		break;
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		break;
	default:
		LOG_ERR("I2S format not supported");
		return -ENOSYS;
	}

	int ret;
	uint32_t bit_clk_freq;
	i2s_hal_clock_info_t i2s_hal_clock_info;

	ret = i2s_esp32_calculate_clock(i2s_cfg, &i2s_hal_clock_info);
	if (ret != ESP_OK) {
		return -EINVAL;
	}

	i2s_hal_init(&dev_data->hal_cxt, dev_cfg->i2s_num);
	i2s_ll_enable_clock(dev_data->hal_cxt.dev);

	bool is_slave;

	if (stream->master) {
		is_slave = false;
	} else {
		is_slave = true;
	}

	if (dir == I2S_DIR_RX) {
		i2s_hal_set_rx_clock(&dev_data->hal_cxt, &i2s_hal_clock_info, IS2_ESP32_CLK_SRC);
	} else if (dir == I2S_DIR_TX) {
		i2s_hal_set_tx_clock(&dev_data->hal_cxt, &i2s_hal_clock_info, IS2_ESP32_CLK_SRC);
	}

	stream->state = I2S_STATE_READY;

	return 0;
}

static int i2s_esp32_read(const struct device *dev, void **mem_block, size_t *size)
{
/*
	struct i2s_esp32_data *const dev_data = dev->data;
	int ret;

	if (dev_data->rx_stream.state == I2S_STATE_NOT_READY) {
		return -EIO;
	}

	if (dev_data->rx_stream.state != I2S_STATE_ERROR) {
		ret = k_sem_take(&dev_data->rx_stream.sem,
				 SYS_TIMEOUT_MS(dev_data->rx_stream.i2s_cfg.timeout));
		if (ret < 0) {
			return ret;
		}
	}

	ret = i2s_esp32_queue_get(&dev_data->rx_stream.mem_block_queue, mem_block, size);
	if (ret < 0) {
		return -EIO;
	}
*/
	return 0;
}

static int i2s_esp32_write(const struct device *dev, void *mem_block, size_t size)
{
/*
	struct i2s_esp32_data *const dev_data = dev->data;
	int ret;

	if (dev_data->tx_stream.state != I2S_STATE_RUNNING &&
	    dev_data->tx_stream.state != I2S_STATE_READY) {
		return -EIO;
	}

	ret = k_sem_take(&dev_data->tx_stream.sem,
			 SYS_TIMEOUT_MS(dev_data->tx_stream.i2s_cfg.timeout));
	if (ret < 0) {
		return ret;
	}

	i2s_esp32_queue_put(&dev_data->tx_stream.mem_block_queue, mem_block, size);
*/
	return 0;
}

static int i2s_esp32_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	struct i2s_esp32_data *const dev_data = dev->data;
	struct stream *stream;
	unsigned int key;
	int ret;

	switch (dir) {
	case I2S_DIR_RX:
		stream = &dev_data->rx_stream;
		break;
	case I2S_DIR_TX:
		stream = &dev_data->tx_stream;
		break;
	case I2S_DIR_BOTH:
		return -ENOSYS;
	default:
		LOG_ERR("Either RX or TX direction must be selected");
		return -EINVAL;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d", stream->state);
			return -EIO;
		}
		/*
		__ASSERT_NO_MSG(stream->mem_block == NULL);

		ret = stream->stream_start(stream, dev);
		if (ret < 0) {
			LOG_ERR("START trigger failed %d", ret);
			return ret;
		}
		*/
		stream->state = I2S_STATE_RUNNING;
		stream->last_block = false;
		break;

	case I2S_TRIGGER_STOP:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("STOP trigger: invalid state");
			return -EIO;
		}
		irq_unlock(key);
		/*
		stream->stream_stop(stream, dev);
		stream->queue_drop(stream);
		*/
		stream->state = I2S_STATE_READY;
		stream->last_block = true;
		break;

	case I2S_TRIGGER_DRAIN:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("DRAIN trigger: invalid state");
			return -EIO;
		}
		/*
		stream->stream_stop(stream, dev);
		stream->queue_drop(stream);
		*/
		stream->state = I2S_STATE_READY;
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DROP:
		if (stream->state == I2S_STATE_NOT_READY) {
			LOG_ERR("DROP trigger: invalid state");
			return -EIO;
		}
		/*
		stream->stream_stop(stream, dev);
		stream->queue_drop(stream);
		*/
		stream->state = I2S_STATE_READY;
		break;

	case I2S_TRIGGER_PREPARE:
		if (stream->state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE trigger: invalid state");
			return -EIO;
		}
		/*
		stream->queue_drop(stream);
		*/
		stream->state = I2S_STATE_READY;
		break;

	default:
		LOG_ERR("Unsupported trigger command");
		return -EINVAL;
	}

	return 0;
}

static int i2s_esp32_initialize(const struct device *dev)
{
	const struct i2s_esp32_cfg *dev_cfg = dev->config;
	struct i2s_esp32_data *const dev_data = dev->data;
	int ret, i;

	ret = i2s_esp32_enable_clock(dev);
	if (ret < 0) {
		LOG_ERR("%s: clock enabling failed: %d", __func__, ret);
		return -EIO;
	}

	ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("I2S pinctrl setup failed (%d)", ret);
		return ret;
	}

	dev_cfg->irq_config(dev);

	ret = k_sem_init(&dev_data->rx_stream.sem, 0, I2S_ESP32_RX_BLOCK_COUNT);
	if (ret != 0) {
		return ret;
	}

	ret = k_sem_init(&dev_data->tx_stream.sem, I2S_ESP32_TX_BLOCK_COUNT,
			 I2S_ESP32_TX_BLOCK_COUNT);
	if (ret != 0) {
		return ret;
	}

	for (i = 0; i < ESP32_DMA_NUM_CHANNELS; i++) { // (?) <---
		i2s_esp32_active_dma_rx_channel[i] = NULL;
		i2s_esp32_active_dma_tx_channel[i] = NULL;
	}

	if (!device_is_ready(dev_data->tx_stream.dev_dma)) {
		LOG_ERR("%s device not ready", dev_data->tx_stream.dev_dma->name);
		return -ENODEV;
	}

	if (!device_is_ready(dev_data->rx_stream.dev_dma)) {
		LOG_ERR("%s device not ready", dev_data->rx_stream.dev_dma->name);
		return -ENODEV;
	}

	LOG_INF("%s inited", dev->name);

	return 0;
}

static const struct i2s_driver_api i2s_esp32_driver_api = {
	.configure = i2s_esp32_configure,
	.read = i2s_esp32_read,
	.write = i2s_esp32_write,
	.trigger = i2s_esp32_trigger,
};

#define I2S(idx) DT_NODELABEL(i2s##idx)

#define I2S_ESP32_DMA_CHANNEL_INIT(index, dir, dir_cap, src_dev, dest_dev)                         \
	.dir##_stream = {                                                                          \
		.dev_dma = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, dir)),                   \
		.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),                     \
		.dma_cfg = {                                                                       \
				.block_count = 2,                                                  \
				.dma_slot = ESP_GDMA_TRIG_PERIPH_I2S##index,                       \
				.channel_direction = src_dev##_TO_##dest_dev,                      \
				.source_data_size = 2,                                             \
				.dest_data_size = 2,                                               \
				.source_burst_length = 1,                                          \
				.dest_burst_length = 1,                                            \
				.channel_priority = 1,                                             \
				.dma_callback = i2s_esp32_dma_##dir##_callback,                    \
		},                                                                                 \
		.fifo_threshold = 1,                                                               \
		.stream_start = i2s_esp32_##dir##_stream_start,                                    \
		.stream_stop = i2s_esp32_##dir##_stream_stop,                                      \
		.queue_drop = i2s_esp32_##dir##_queue_drop,                                        \
		.mem_block_queue.buf = i2s_esp32_##dir##_##index##_ring_buf,                       \
		.mem_block_queue.len = ARRAY_SIZE(i2s_esp32_##dir##_##index##_ring_buf),           \
		.i2s_blk_addr = (void *)DT_REG_ADDR(DT_NODELABEL(i2s##index))                      \
	}

#define I2S_ESP32_INIT(index)                                                                      \
	static void i2s_esp32_irq_config_func_##index(const struct device *dev)                    \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority),                     \
			    i2s_esp32_isr, DEVICE_DT_INST_GET(index), 0);                          \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}                                                                                          \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct i2s_esp32_cfg i2s_esp32_config_##index = {                             \
		.i2s_num = index,                                                                  \
		.irq_config = i2s_esp32_irq_config_func_##index,                                   \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(I2S(index))),                            \
		.clock_subsys = (clock_control_subsys_t)DT_CLOCKS_CELL(I2S(index), offset),        \
	};                                                                                         \
                                                                                                   \
	struct queue_item i2s_esp32_rx_##index##_ring_buf[I2S_ESP32_RX_BLOCK_COUNT + 1];           \
	struct queue_item i2s_esp32_tx_##index##_ring_buf[I2S_ESP32_TX_BLOCK_COUNT + 1];           \
                                                                                                   \
	static struct i2s_esp32_data i2s_esp32_data_##index = {                                    \
		.hal_cxt =                                                                         \
			{                                                                          \
				.dev = (i2s_dev_t *)DT_REG_ADDR(I2S(index)),                       \
			},                                                                         \
		/*UTIL_AND(DT_INST_DMAS_HAS_NAME(index, rx),*/                                         \
		/*	 I2S_ESP32_DMA_CHANNEL_INIT(index, rx, RX, PERIPHERAL, MEMORY)),*/           \
		/*UTIL_AND(DT_INST_DMAS_HAS_NAME(index, tx),*/                                         \
		/*	 I2S_ESP32_DMA_CHANNEL_INIT(index, tx, TX, MEMORY, PERIPHERAL)),*/           \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &i2s_esp32_initialize, NULL, &i2s_esp32_data_##index,         \
			      &i2s_esp32_config_##index, POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,    \
			      &i2s_esp32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_ESP32_INIT)
