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

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_PTP_PORT_H_ */
