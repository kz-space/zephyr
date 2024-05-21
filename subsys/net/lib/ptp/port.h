/*
 * Copyright (c) 2024 BayLibre SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file port.h
 * @brief PTP port data structure and interface to operate on PTP Ports.
 *
 * References are to version 2019 of IEEE 1588, ("PTP")
 */

#ifndef ZEPHYR_INCLUDE_PTP_PORT_H_
#define ZEPHYR_INCLUDE_PTP_PORT_H_

#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>

#include "ds.h"
#include "state_machine.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PTP_PORT_TIMER_ANNOUNCE_TO	(0)
#define PTP_PORT_TIMER_DELAY_TO		(1)
#define PTP_PORT_TIMER_SYNC_TO		(2)
#define PTP_PORT_TIMER_QUALIFICATION_TO (3)

/**
 * @brief Structure describing PTP Port.
 */
struct ptp_port {
	sys_snode_t			node; /* object list */
	struct ptp_port_ds		port_ds;
	struct net_if			*iface;
	int				socket[2];
	struct {
		struct k_timer		announce;
		struct k_timer		delay;
		struct k_timer		sync;
		struct k_timer		qualification;
	} timers;
	atomic_t			timeouts;
	struct {
		uint16_t		announce;
		uint16_t		delay;
		uint16_t		signaling;
		uint16_t		sync;
	} seq_id;
	enum ptp_port_state		(*state_machine)(enum ptp_port_state state,
							 enum ptp_port_event event,
							 bool master_diff);
	struct ptp_foreign_master_clock *best;
	sys_slist_t			foreign_list;
	sys_slist_t			delay_req_list;
	struct ptp_msg			*last_sync_fup;
	struct net_if_timestamp_cb	delay_req_ts_cb;
	struct net_if_timestamp_cb	sync_ts_cb;
};

/**
 * @brief Function initializing PTP Port.
 *
 * @param[in] iface     Pointer to current network interface.
 * @param[in] user_data Unused argument needed to comply with @ref net_if_cb_t type.
 */
void ptp_port_init(struct net_if *iface, void *user_data);

/**
 * @brief Function returning PTP Port's state.
 *
 * @param[in] port Pointer to the PTP Port structure.
 *
 * @return PTP Port's current state.
 */
enum ptp_port_state ptp_port_state(struct ptp_port *port);

/**
 * @brief Function generating PTP Port events based on PTP Port's timers.
 *
 * @param[in] port  Pointer to the PTP Port structure.
 * @param[in] timer Pointer to the PTP Port's timer to be checked.
 *
 * @return PTP Port event.
 */
enum ptp_port_event ptp_port_timer_event_gen(struct ptp_port *port, struct k_timer *timer);
/**
 * @brief Function generating PTP Port events based on messages
 * that has been recived on a PTP Port's socket.
 *
 * @param[in] port  Pointer to the PTP Port structure.
 * @param[in] idx   Index of the PTP Port's socket to be checked.
 *
 * @return PTP Port event.
 */
enum ptp_port_event ptp_port_event_gen(struct ptp_port *port, int idx);

/**
 * @brief Function handling PTP Port event.
 *
 * @param[in] port	  Pointer to the PTP Port structure.
 * @param[in] event	  PTP Port Event to be processed.
 * @param[in] master_diff Flag indicating whether PTP Master has changed.
 */
void ptp_port_event_handle(struct ptp_port *port, enum ptp_port_event event, bool master_diff);

/**
 * @brief Function checking if two port identities are equal.
 *
 * @param[in] p1 Pointer to the port identity structure.
 * @param[in] p2 Pointer to the port identity structure.
 *
 * @return True if port identities are equal, False otherwise.
 */
bool ptp_port_id_eq(const struct ptp_port_id *p1, const struct ptp_port_id *p2);

/**
 * @brief Function for getting a common dataset for the port's best foreign master clock.
 *
 * @param[in] port Pointer to the PTP Port structure.
 *
 * @return NULL if the port doesn't have best foreign master clock of pointer to the ptp_dataset
 * of the best foreign master clock.
 */
struct ptp_dataset *ptp_port_best_foreign_ds(struct ptp_port *port);

/**
 * @brief Compute PTP Port's best Foreign Master Clock.
 *
 * @param[in] port Pointer to the PTP Port.
 *
 * @return Pointer to the PTP Port's best Foreign Master.
 */
struct ptp_foreign_master_clock *ptp_port_best_foreign(struct ptp_port *port);

/**
 * @brief Function adding foreign Master Clock for the PTP Port based on specified message.
 *
 * @param[in] port Pointer to the PTP Port.
 * @param[in] msg  Pointer to the announce message containg PTP Master data.
 *
 * @return Non-zero if the announce message is different than the last.
 */
int ptp_port_add_foreign_master(struct ptp_port *port, struct ptp_msg *msg);

/**
 * @brief Function freeing memory used by foreign masters assigned to given PTP Port.
 *
 * @param[in] port Pointer to the PTP Port.
 */
void ptp_port_free_foreign_masters(struct ptp_port *port);

/**
 * @brief Function updating current PTP Master Clock of the PTP Port based on specified message.
 *
 * @param[in] port Pointer to the PTP Port.
 * @param[in] msg  Pointer to the announce message containg PTP Master data.
 *
 * @return Non-zero if the announce message is different than the last.
 */
int ptp_port_update_current_master(struct ptp_port *port, struct ptp_msg *msg);

/**
 * @brief Function processing received PTP Management message on the PTP Port's level.
 *
 * @param[in] port    PTP Port to process the received message.
 * @param[in] ingress Original PTP Port that received the message.
 * @param[in] msg     PTP Management message to be processed.
 * @param[in] tlv     Management TLV attached to the message.
 *
 * @return 0 on positive message processing, negative otherwise
 */
int ptp_port_management_msg_process(struct ptp_port *port,
				    struct ptp_port *ingress,
				    struct ptp_msg *msg,
				    struct ptp_tlv_mgmt *tlv);

/**
 * @brief Function for sending Management Error resposnse message.
 *
 * @param[in] port PTP Port that is going to send Management Error response.
 * @param[in] msg  PTP Management message that caused Management Error.
 * @param[in] err  Management Error code.
 *
 * @return 0 if message sent successfully, negative otherwise.
 */
int ptp_port_management_error(struct ptp_port *port, struct ptp_msg *msg, enum ptp_mgmt_err err);

/**
 * @brief Function for sending response on specific PTP PORT to received PTP Management message.
 *
 * @param[in] port PTP Port that will send response message.
 * @param[in] req  Received PTP Management message.
 * @param[in] tlv  Management TLV attached to the message.
 *
 * @return 0 if message sent successfully, negative otherwise.
 */
int ptp_port_management_resp(struct ptp_port *port, struct ptp_msg *req, struct ptp_tlv_mgmt *tlv);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_PTP_PORT_H_ */
