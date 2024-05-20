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
};

/**
 * @brief Function initializing PTP Port.
 *
 * @param[in] iface     Pointer to current network interface.
 * @param[in] user_data Unused argument needed to comply with @ref net_if_cb_t type.
 */
void ptp_port_init(struct net_if *iface, void *user_data);

/**
 * @brief Function checking if two port identities are equal.
 *
 * @param[in] p1 Pointer to the port identity structure.
 * @param[in] p2 Pointer to the port identity structure.
 *
 * @return True if port identities are equal, False otherwise.
 */
bool ptp_port_id_eq(const struct ptp_port_id *p1, const struct ptp_port_id *p2);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_PTP_PORT_H_ */
