/*
 * Copyright (c) 2017-2021 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SUBSYS_BLUETOOTH_HOST_SCAN_H_
#define SUBSYS_BLUETOOTH_HOST_SCAN_H_

#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/bluetooth.h>

enum scan_enabled_reason {
	/** The application explicitly instructed the stack to scan for advertisers
	 * using the API @ref bt_le_scan_start().
	 */
	SCAN_ENABLED_REASON_EXPLICIT_SCAN,
	SCAN_ENABLED_REASON_SYNC_SYNCING,
	SCAN_ENABLED_REASON_SCAN_BEFORE_INITIATE,

	SCAN_UPDATE_JUST_CHECK,
	SCAN_ENABLED_REASON_NUM_FLAGS,
};

/* TODO: this can be defined private */
struct scanner_state {
	ATOMIC_DEFINE(scan_flags, SCAN_ENABLED_REASON_NUM_FLAGS);
	struct bt_le_scan_param explicit_scan_param;
	struct bt_le_scan_param used_scan_param;
};

void bt_scan_reset(void);

bool bt_id_scan_random_addr_check(void);
bool bt_le_scan_active_scanner_running(void);

int bt_le_scan_set_enable(uint8_t enable);

struct bt_le_per_adv_sync *bt_hci_get_per_adv_sync(uint16_t handle);

void bt_periodic_sync_disable(void);

#endif /* defined SUBSYS_BLUETOOTH_HOST_SCAN_H_ */
