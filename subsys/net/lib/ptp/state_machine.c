/*
 * Copyright (c) 2024 BayLibre SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include "state_machine.h"

enum ptp_port_state ptp_state_machine(enum ptp_port_state state,
				      enum ptp_port_event event,
				      bool master_diff)
{
	enum ptp_port_state new_state = state;

	if (event == PTP_EVT_INITIALIZE || event == PTP_EVT_POWERUP) {
		/* initialize port data sets, HW and communication facilities */
		return PTP_PS_INITIALIZING;
	}

	switch (state) {
	case PTP_PS_INITIALIZING:
		switch (event) {
		case PTP_EVT_FAULT_DETECTED:
			new_state = PTP_PS_FAULTY;
			break;
		case PTP_EVT_INIT_COMPLETE:
			new_state = PTP_PS_LISTENING;
			break;
		default:
			break;
		}
		break;
	case PTP_PS_FAULTY:
		switch (event) {
		case PTP_EVT_DESIGNATED_DISABLED:
			new_state = PTP_PS_DISABLED;
			break;
		case PTP_EVT_FAULT_CLEARED:
			new_state = PTP_PS_INITIALIZING;
			break;
		default:
			break;
		}
		break;
	case PTP_PS_DISABLED:
		if (event == PTP_EVT_DESIGNATED_ENABLED) {
			new_state = PTP_PS_INITIALIZING;
		}
		break;
	case PTP_PS_LISTENING:
		switch (event) {
		case PTP_EVT_DESIGNATED_DISABLED:
			new_state = PTP_PS_DISABLED;
			break;
		case PTP_EVT_FAULT_DETECTED:
			new_state = PTP_PS_FAULTY;
			break;
		case PTP_EVT_ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES:
			new_state = PTP_PS_MASTER;
			break;
		case PTP_EVT_RS_MASTER:
			new_state = PTP_PS_PRE_MASTER;
			break;
		case PTP_EVT_RS_GRAND_MASTER:
			new_state = PTP_PS_GRAND_MASTER;
			break;
		case PTP_EVT_RS_PASSIVE:
			new_state = PTP_PS_PASSIVE;
			break;
		case PTP_EVT_RS_SLAVE:
			new_state = PTP_PS_UNCALIBRATED;
			break;
		default:
			break;
		}
		break;
	case PTP_PS_PRE_MASTER:
		switch (event) {
		case PTP_EVT_DESIGNATED_DISABLED:
			new_state = PTP_PS_DISABLED;
			break;
		case PTP_EVT_FAULT_DETECTED:
			new_state = PTP_PS_FAULTY;
			break;
		case PTP_EVT_QUALIFICATION_TIMEOUT_EXPIRES:
			new_state = PTP_PS_MASTER;
			break;
		case PTP_EVT_RS_PASSIVE:
			new_state = PTP_PS_PASSIVE;
			break;
		case PTP_EVT_RS_SLAVE:
			new_state = PTP_PS_UNCALIBRATED;
			break;
		default:
			break;
		}
		break;
	case PTP_PS_MASTER:
		switch (event) {
		case PTP_EVT_DESIGNATED_DISABLED:
			new_state = PTP_PS_DISABLED;
			break;
		case PTP_EVT_FAULT_DETECTED:
			new_state = PTP_PS_FAULTY;
			break;
		case PTP_EVT_RS_PASSIVE:
			new_state = PTP_PS_PASSIVE;
			break;
		case PTP_EVT_RS_SLAVE:
			new_state = PTP_PS_UNCALIBRATED;
			break;
		default:
			break;
		}
		break;
	case PTP_PS_PASSIVE:
		switch (event) {
		case PTP_EVT_DESIGNATED_DISABLED:
			new_state = PTP_PS_DISABLED;
			break;
		case PTP_EVT_FAULT_DETECTED:
			new_state = PTP_PS_FAULTY;
			break;
		case PTP_EVT_RS_MASTER:
			new_state = PTP_PS_PRE_MASTER;
			break;
		case PTP_EVT_RS_GRAND_MASTER:
			new_state = PTP_PS_GRAND_MASTER;
			break;
		case PTP_EVT_RS_SLAVE:
			new_state = PTP_PS_UNCALIBRATED;
			break;
		default:
			break;
		}
		break;
	case PTP_PS_UNCALIBRATED:
		switch (event) {
		case PTP_EVT_DESIGNATED_DISABLED:
			new_state = PTP_PS_DISABLED;
			break;
		case PTP_EVT_FAULT_DETECTED:
			new_state = PTP_PS_FAULTY;
			break;
		case PTP_EVT_ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES:
			new_state = PTP_PS_MASTER;
			break;
		case PTP_EVT_MASTER_CLOCK_SELECTED:
			new_state = PTP_PS_SLAVE;
			break;
		case PTP_EVT_RS_MASTER:
			new_state = PTP_PS_PRE_MASTER;
			break;
		case PTP_EVT_RS_GRAND_MASTER:
			new_state = PTP_PS_GRAND_MASTER;
			break;
		case PTP_EVT_RS_PASSIVE:
			new_state = PTP_PS_PASSIVE;
			break;
		default:
			break;
		}
		break;
	case PTP_PS_SLAVE:
		switch (event) {
		case PTP_EVT_DESIGNATED_DISABLED:
			new_state = PTP_PS_DISABLED;
			break;
		case PTP_EVT_FAULT_DETECTED:
			new_state = PTP_PS_FAULTY;
			break;
		case PTP_EVT_SYNCHRONIZATION_FAULT:
			new_state = PTP_PS_UNCALIBRATED;
			break;
		case PTP_EVT_ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES:
			new_state = PTP_PS_MASTER;
			break;
		case PTP_EVT_RS_MASTER:
			new_state = PTP_PS_PRE_MASTER;
			break;
		case PTP_EVT_RS_GRAND_MASTER:
			new_state = PTP_PS_GRAND_MASTER;
			break;
		case PTP_EVT_RS_PASSIVE:
			new_state = PTP_PS_PASSIVE;
			break;
		case PTP_EVT_RS_SLAVE:
			if (master_diff) {
				new_state = PTP_PS_UNCALIBRATED;
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return new_state;
}

enum ptp_port_state ptp_so_state_machine(enum ptp_port_state state,
					 enum ptp_port_event event,
					 bool master_diff)
{
	enum ptp_port_state new_state = state;

	if (event == PTP_EVT_INITIALIZE || event == PTP_EVT_POWERUP) {
		/* initialize port data sets, HW and communication facilities */
		return PTP_PS_INITIALIZING;
	}

	switch (state) {
	case PTP_PS_INITIALIZING:
		switch (event) {
		case PTP_EVT_FAULT_DETECTED:
			new_state = PTP_PS_FAULTY;
			break;
		case PTP_EVT_INIT_COMPLETE:
			new_state = PTP_PS_LISTENING;
			break;
		default:
			break;
		}
		break;
	case PTP_PS_FAULTY:
		switch (event) {
		case PTP_EVT_DESIGNATED_DISABLED:
			new_state = PTP_PS_DISABLED;
			break;
		case PTP_EVT_FAULT_CLEARED:
			new_state = PTP_PS_INITIALIZING;
			break;
		default:
			break;
		}
		break;
	case PTP_PS_DISABLED:
		if (event == PTP_EVT_DESIGNATED_ENABLED) {
			new_state = PTP_PS_INITIALIZING;
		}
		break;
	case PTP_PS_LISTENING:
		switch (event) {
		case PTP_EVT_DESIGNATED_DISABLED:
			new_state = PTP_PS_DISABLED;
			break;
		case PTP_EVT_FAULT_DETECTED:
			new_state = PTP_PS_FAULTY;
			break;
		case PTP_EVT_RS_SLAVE:
			new_state = PTP_PS_UNCALIBRATED;
			break;
		default:
			break;
		}
		break;
	case PTP_PS_UNCALIBRATED:
		switch (event) {
		case PTP_EVT_DESIGNATED_DISABLED:
			new_state = PTP_PS_DISABLED;
			break;
		case PTP_EVT_FAULT_DETECTED:
			new_state = PTP_PS_FAULTY;
			break;
		case PTP_EVT_ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES:
		case PTP_EVT_RS_MASTER:
		case PTP_EVT_RS_GRAND_MASTER:
		case PTP_EVT_RS_PASSIVE:
			new_state = PTP_PS_LISTENING;
			break;
		case PTP_EVT_MASTER_CLOCK_SELECTED:
			new_state = PTP_PS_SLAVE;
		default:
			break;
		}
		break;
	case PTP_PS_SLAVE:
		switch (event) {
		case PTP_EVT_DESIGNATED_DISABLED:
			new_state = PTP_PS_DISABLED;
			break;
		case PTP_EVT_FAULT_DETECTED:
			new_state = PTP_PS_FAULTY;
			break;
		case PTP_EVT_SYNCHRONIZATION_FAULT:
			new_state = PTP_PS_UNCALIBRATED;
			break;
		case PTP_EVT_ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES:
		case PTP_EVT_RS_MASTER:
		case PTP_EVT_RS_GRAND_MASTER:
		case PTP_EVT_RS_PASSIVE:
			new_state = PTP_PS_LISTENING;
			break;
		case PTP_EVT_RS_SLAVE:
			if (master_diff) {
				new_state = PTP_PS_UNCALIBRATED;
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return new_state;
}
