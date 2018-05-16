/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef LOG_OUTPUT_H
#define LOG_OUTPUT_H

#include <logging/log_msg.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Flag forcing ANSI escape code colors, red (errors), yellow (warnings).
 */
#define LOG_OUTPUT_FLAG_COLORS                  (1 << 0)

/** @brief Flag forcing timestamp formatting. */
#define LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP        (1 << 1)

/**
 * @brief Prototype of the function processing output data.
 *
 * @param p_data Data.
 * @param length Data length.
 * @param p_ctx  User context.
 *
 * @return Number of bytes processed.
 */
typedef int (*log_output_func_t)(u8_t *p_data, size_t length, void *p_ctx);

struct log_output_ctx {
	log_output_func_t func;
	u8_t *p_data;
	size_t length;
	size_t offset;
	void *p_ctx;
};

/** @brief Function for processing log messages to readable strings.
 *
 * Function is using provided context with the buffer and output function to
 * process formatted string and output the data.
 *
 * @param p_msg Log message.
 * @param p_ctx Context.
 * @param flags Optional flags.
 */
void log_output_msg_process(struct log_msg *p_msg,
			    struct log_output_ctx *p_ctx,
			    u32_t flags);

/** @brief Function for setting timestamp frequency.
 *
 * @param freq Frequency in Hz.
 */
void log_output_timestamp_freq_set(u32_t freq);

#ifdef __cplusplus
}
#endif

#endif /* LOG_OUTPUT_H */
