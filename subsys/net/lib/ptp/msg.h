/*
 * Copyright (c) 2024 BayLibre SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file msg.h
 * @brief PTP messages definition.
 *
 * References are to version 2019 of IEEE 1588, ("PTP")
 */

#ifndef ZEPHYR_INCLUDE_PTP_MSG_H_
#define ZEPHYR_INCLUDE_PTP_MSG_H_

#include <zephyr/net/ptp_time.h>

#include "ddt.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PTP message type.
 */
enum ptp_msg_type {
	/* PTP event message types */
	PTP_MSG_SYNC = 0,
	PTP_MSG_DELAY_REQ,
	PTP_MSG_PDELAY_REQ,
	PTP_MSG_PDELAY_RESP,
	/* General PTP message types */
	PTP_MSG_FOLLOW_UP = 8,
	PTP_MSG_DELAY_RESP,
	PTP_MSG_PDELAY_RESP_FOLLOW_UP,
	PTP_MSG_ANNOUNCE,
	PTP_MSG_SIGNALING,
	PTP_MSG_MANAGEMENT,
};

/**
 * @brief Common PTP message header.
 */
struct ptp_header {
	uint8_t		   type_major_sdo_id;
	uint8_t		   version;
	uint16_t	   msg_length;
	uint8_t		   domain_number;
	uint8_t		   minor_sdo_id;
	uint8_t		   flags[2];
	int64_t		   correction;
	uint32_t	   reserved;
	struct ptp_port_id src_port_id;
	uint16_t	   sequence_id;
	uint8_t		   control;
	int8_t		   log_msg_interval;
} __packed;

/**
 * @brief PTP Announce message header.
 */
struct ptp_announce_msg {
	struct ptp_header      hdr;
	struct ptp_timestamp   origin_timestamp;
	uint16_t	       current_utc_offset;
	uint8_t		       reserved;
	uint8_t		       gm_priority1;
	struct ptp_clk_quality gm_clk_quality;
	uint8_t		       gm_priority2;
	ptp_clk_id	       gm_id;
	uint16_t	       steps_rm;
	uint8_t		       time_src;
	uint8_t		       suffix[];
} __packed;

/**
 * @brief PTP Sync message header.
 */
struct ptp_sync_msg {
	struct ptp_header    hdr;
	struct ptp_timestamp origin_timestamp;
	uint8_t		     suffix[];
} __packed;

/**
 * @brief PTP Delay_Req message header.
 */
struct ptp_delay_req_msg {
	struct ptp_header    hdr;
	struct ptp_timestamp origin_timestamp;
	uint8_t		     suffix[];
} __packed;

/**
 * @brief PTP Follow_Up message header.
 */
struct ptp_follow_up_msg {
	struct ptp_header    hdr;
	struct ptp_timestamp precise_origin_timestamp;
	uint8_t		     suffix[];
} __packed;

/**
 * @brief PTP Delay_Resp message header.
 */
struct ptp_delay_resp_msg {
	struct ptp_header    hdr;
	struct ptp_timestamp receive_timestamp;
	struct ptp_port_id   req_port_id;
	uint8_t		     suffix[];
} __packed;

/**
 * @brief PTP Pdelay_Req message header.
 */
struct ptp_pdelay_req_msg {
	struct ptp_header    hdr;
	struct ptp_timestamp origin_timestamp;
	struct ptp_port_id   reserved; /* make it the same length as ptp_pdelay_resp */
	uint8_t		     suffix[];
} __packed;

/**
 * @brief PTP Pdelay_Resp message header.
 */
struct ptp_pdelay_resp_msg {
	struct ptp_header    hdr;
	struct ptp_timestamp req_receipt_timestamp;
	struct ptp_port_id   req_port_id;
	uint8_t		     suffix[];
} __packed;

/**
 * @brief PTP Pdelay_Resp_Follow_Up message header.
 */
struct ptp_pdelay_resp_follow_up_msg {
	struct ptp_header    hdr;
	struct ptp_timestamp resp_origin_timestamp;
	struct ptp_port_id   req_port_id;
	uint8_t		     suffix[];
} __packed;

/**
 * @brief PTP Signaling message header.
 */
struct ptp_signaling_msg {
	struct ptp_header  hdr;
	struct ptp_port_id target_port_id;
	uint8_t		   suffix[];
} __packed;

/**
 * @brief PTP Management message header.
 */
struct ptp_management_msg {
	struct ptp_header  hdr;
	struct ptp_port_id target_port_id;
	uint8_t		   starting_boundary_hops;
	uint8_t		   boundary_hops;
	uint8_t		   action:5;
	uint8_t		   reserved;
	uint8_t		   suffix[];
} __packed;

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_PTP_MSG_H_ */
