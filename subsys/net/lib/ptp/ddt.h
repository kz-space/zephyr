/*
 * Copyright (c) 2024 BayLibre SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file ddt.h
 * @brief Derived data types.
 *
 * @note Based on IEEE 1588:2019 section 5.3 - Derived data types
 */

#ifndef ZEPHYR_INCLUDE_PTP_DDT_H_
#define ZEPHYR_INCLUDE_PTP_DDT_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PTP time interval in nanoseconds.
 * @note 5.3.2 - time interval expressed in nanoseconds multiplied by 2^16
 */
typedef int64_t ptp_timeinterval;

/**
 * @brief Structure for storing PTP timestamp used in PTP Protocol.
 * @note 5.3.3 - timestamp with respect to epoch
 */
struct ptp_timestamp {
	/* Seconds encoded on 48 bits. */
	uint16_t seconds_high;
	uint32_t seconds_low;
	/* Nanoseconds. */
	uint32_t nanoseconds;
} __packed;

/**
 * @brief PTP Clock Identity.
 * @note 5.3.4 - identifies unique entities within a PTP network.
 */
typedef struct {
	uint8_t id[8];
} ptp_clk_id;

/**
 * @brief PTP Port Identity.
 * @note 5.3.5 - identifies a PTP Port or a Link port.
 */
struct ptp_port_id {
	ptp_clk_id clk_id;
	uint16_t   port_number;
} __packed;

/**
 * @brief Structure represeniting address of a PTP Port.
 * @note 5.3.6 - represents the protocol address of a PTP port
 */
struct ptp_port_addr {
	uint16_t protocol;
	uint16_t addr_len; /* range from 1-16 */
	uint8_t  address[];
} __packed;

/**
 * @brief Structure for PTP Clock quality metrices.
 * @note 5.3.7 - quality of a clock
 */
struct ptp_clk_quality {
	uint8_t  class;
	uint8_t  accuracy;
	uint16_t offset_scaled_log_variance;
} __packed;

/**
 * @brief
 * @note 5.3.8 - TLV (type, length, value) extension fields
 */
struct ptp_tlv {
	uint16_t type;
	uint16_t length;
	uint8_t  value[];
} __packed;

/**
 * @brief Generic datatype for storing text in PTP messages.
 * @note 5.3.9 - holds textual content in PTP messages
 */
struct ptp_text {
	uint8_t length; /* might be larger than number of symbols due to UTF-8 encoding */
	uint8_t text[]; /* encoded as UTF-8, single symbol can be 1-4 bytes long */
} __packed;

/**
 * @brief Structure for capturing fault logs.
 * @note 5.3.10 - fault log datatype
 */
struct ptp_fault_record {
	uint16_t	    length;
	struct net_ptp_time time;
	uint8_t		    code;
	struct ptp_text	    name;
	struct ptp_text	    value;
	struct ptp_text	    desc;
} __packed;

/**
 * @brief Type holding difference between two numeric value
 * @note 5.3.11 - relative difference between two numeric values.
 * It's a dimensionless fraction and multiplied by 2^62.
 */
typedef int64_t ptp_relative_diff;

/**
 * @brief Type holding data for element of alternate timeset offset mechanism,
 * @note 16.3.4.2 - element of ptp_alt_timescale_offset_ds list
 */
struct ptp_alt_timescale {
	uint8_t	        key;
	bool	        enable;
	int32_t	        current_offset;
	int32_t	        jump_seconds;
	uint64_t        time_of_next_jump:48;
	struct ptp_text disp_name;
} __packed;

struct ptp_port;

struct ptp_clock;

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_PTP_PDT_H_ */
