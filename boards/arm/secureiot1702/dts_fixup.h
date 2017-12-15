/*
 * Copyright (c) 2017 Crypta Labs Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* This file is a temporary workaround for mapping of the generated information
 * to the current driver definitions.  This will be removed when the drivers
 * are modified to handle the generated information, or the mapping of
 * generated data matches the driver definitions.
 */

#define DT_NUM_IRQ_PRIO_BITS			DT_ARM_V7M_NVIC_E000E100_ARM_NUM_IRQ_PRIORITY_BITS

#define CONFIG_NS16550_REG_SHIFT		DT_NS16550_400F2400_REG_SHIFT
#define DT_UART_NS16550_PORT_0_BASE_ADDR	DT_NS16550_400F2400_BASE_ADDRESS
#define DT_UART_NS16550_PORT_0_CLK_FREQ		DT_NS16550_400F2400_CLOCK_FREQUENCY
#define CONFIG_UART_NS16550_PORT_0_NAME		DT_NS16550_400F2400_LABEL
#define CONFIG_UART_NS16550_PORT_0_BAUD_RATE	DT_NS16550_400F2400_CURRENT_SPEED

#define DT_UART_NS16550_PORT_1_BASE_ADDR	DT_NS16550_400F2800_BASE_ADDRESS
#define DT_UART_NS16550_PORT_1_CLK_FREQ		DT_NS16550_400F2800_CLOCK_FREQUENCY
#define CONFIG_UART_NS16550_PORT_1_NAME		DT_NS16550_400F2800_LABEL
#define CONFIG_UART_NS16550_PORT_1_BAUD_RATE	DT_NS16550_400F2800_CURRENT_SPEED
