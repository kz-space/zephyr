/*
 * Copyright 2023 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/usb_c/tcpci_priv.h>
#include <zephyr/usb_c/usbc.h>
#include <zephyr/usb_c/tcpci.h>
#include <zephyr/shell/shell.h>

/** Macro used to call the dump_std_reg function from USB-C connector node */
#define TCPC_DUMP_NODE(node) ret |= tcpc_dump_std_reg(DEVICE_DT_GET(DT_PROP(node, tcpc)));

/** Macro used to call the vbus_measure function from USB-C connector node */
#define TCPC_VBUS_NODE(node)                                                                       \
	{                                                                                          \
		int val;                                                                           \
		ret |= usbc_vbus_measure(DEVICE_DT_GET(DT_PROP(node, vbus)), &val);                \
		shell_print(sh, "%s vbus: %d mV", DEVICE_DT_GET(node)->name, val);                 \
	}

/** Macro used to call the get_chip function from USB-C connector node */
#define TCPC_GET_CHIP_NODE(node)                                                                   \
	{                                                                                          \
		ret |= tcpc_get_chip_info(DEVICE_DT_GET(DT_PROP(node, tcpc)), &chip_info);         \
		shell_print(sh, "Chip: %s", DEVICE_DT_GET(node)->name);                            \
		shell_print(sh, "\tVendor:   %04x", chip_info.vendor_id);                          \
		shell_print(sh, "\tProduct:  %04x", chip_info.product_id);                         \
		shell_print(sh, "\tDevice:   %04x", chip_info.device_id);                          \
		shell_print(sh, "\tFirmware: %llx", chip_info.fw_version_number);                  \
	}

/**
 * @brief Shell command that dumps standard registers of TCPCs for all available USB-C ports
 *
 * @param sh Shell structure
 * @param argc Unused arguments count
 * @param argv Unused arguments
 * @return int ORed return values of all the functions executed, 0 in case of success
 */
static int cmd_tcpc_dump(const struct shell *sh, size_t argc, char **argv)
{
	int ret = 0;

	DT_FOREACH_STATUS_OKAY(usb_c_connector, TCPC_DUMP_NODE);

	return ret;
}

/**
 * @brief Shell command that prints the vbus measures for all available USB-C ports
 *
 * @param sh Shell structure
 * @param argc Unused arguments count
 * @param argv Unused arguments
 * @return int ORed return values of all the functions executed, 0 in case of success
 */
static int cmd_tcpc_vbus(const struct shell *sh, size_t argc, char **argv)
{
	int ret = 0;

	DT_FOREACH_STATUS_OKAY(usb_c_connector, TCPC_VBUS_NODE);

	return ret;
}

/**
 * @brief Shell command that prints the TCPCs chips information for all available USB-C ports
 *
 * @param sh Shell structure
 * @param argc Unused arguments count
 * @param argv Unused arguments
 * @return int ORed return values of all the functions executed, 0 in case of success
 */
static int cmd_tcpc_chip_info(const struct shell *sh, size_t argc, char **argv)
{
	struct tcpc_chip_info chip_info;
	int ret = 0;

	DT_FOREACH_STATUS_OKAY(usb_c_connector, TCPC_GET_CHIP_NODE);

	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tcpc_cmds,
			       SHELL_CMD(dump, NULL, "Dump TCPC registers", cmd_tcpc_dump),
			       SHELL_CMD(vbus, NULL, "Display all VBUS voltages", cmd_tcpc_vbus),
			       SHELL_CMD(chip, NULL, "Display chips information",
					 cmd_tcpc_chip_info),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(tcpc, &sub_tcpc_cmds, "TCPC (USB-C PD) diagnostics", NULL);
