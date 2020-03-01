/*
 * Copyright (c) 2018, Intel Corporation
 * Copyright (c) 2011-2015, Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for the Apollo Lake SoC
 *
 * This module provides routines to initialize and support soc-level hardware
 * for the Apollo Lake SoC.
 */

#include <kernel.h>
#include "soc.h"
#include <drivers/uart.h>
#include <device.h>
#include <init.h>

#ifdef CONFIG_X86_MMU
/* loapic */
MMU_BOOT_REGION(CONFIG_LOAPIC_BASE_ADDRESS, 4 * 1024, MMU_ENTRY_WRITE);

/* ioapic */
MMU_BOOT_REGION(DT_IOAPIC_BASE_ADDRESS, 1024 * 1024, MMU_ENTRY_WRITE);

#ifdef CONFIG_HPET_TIMER
MMU_BOOT_REGION(DT_INST_0_INTEL_HPET_BASE_ADDRESS, KB(4), MMU_ENTRY_WRITE);
#endif  /* CONFIG_HPET_TIMER */

/* for UARTs */
#ifdef CONFIG_UART_NS16550

#ifdef CONFIG_UART_NS16550_PORT_0
MMU_BOOT_REGION(DT_UART_NS16550_PORT_0_BASE_ADDR, 0x1000,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
#endif

#ifdef CONFIG_UART_NS16550_PORT_1
MMU_BOOT_REGION(DT_UART_NS16550_PORT_1_BASE_ADDR, 0x1000,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
#endif

#ifdef CONFIG_UART_NS16550_PORT_2
MMU_BOOT_REGION(DT_UART_NS16550_PORT_2_BASE_ADDR, 0x1000,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
#endif

#ifdef CONFIG_UART_NS16550_PORT_3
MMU_BOOT_REGION(DT_UART_NS16550_PORT_3_BASE_ADDR, 0x1000,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
#endif

#endif  /* CONFIG_UART_NS16550 */

/* for I2C controllers */
#ifdef CONFIG_I2C

#ifdef CONFIG_I2C_0
MMU_BOOT_REGION(DT_I2C_0_BASE_ADDR, 0x1000,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
#endif

#ifdef CONFIG_I2C_1
MMU_BOOT_REGION(DT_I2C_1_BASE_ADDR, 0x1000,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
#endif

#ifdef CONFIG_I2C_2
MMU_BOOT_REGION(DT_I2C_2_BASE_ADDR, 0x1000,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
#endif

#ifdef CONFIG_I2C_3
MMU_BOOT_REGION(DT_I2C_3_BASE_ADDR, 0x1000,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
#endif

#ifdef CONFIG_I2C_4
MMU_BOOT_REGION(DT_I2C_4_BASE_ADDR, 0x1000,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
#endif

#ifdef CONFIG_I2C_5
MMU_BOOT_REGION(DT_I2C_5_BASE_ADDR, 0x1000,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
#endif

#ifdef CONFIG_I2C_6
MMU_BOOT_REGION(DT_I2C_6_BASE_ADDR, 0x1000,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
#endif

#ifdef CONFIG_I2C_7
MMU_BOOT_REGION(DT_I2C_7_BASE_ADDR, 0x1000,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
#endif

#endif  /* CONFIG_I2C */

/* for GPIO controller */
#ifdef CONFIG_GPIO_INTEL_APL
MMU_BOOT_REGION(DT_APL_GPIO_BASE_ADDRESS_N,
		DT_APL_GPIO_MEM_SIZE_N,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
MMU_BOOT_REGION(DT_APL_GPIO_BASE_ADDRESS_NW,
		DT_APL_GPIO_MEM_SIZE_NW,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
MMU_BOOT_REGION(DT_APL_GPIO_BASE_ADDRESS_W,
		DT_APL_GPIO_MEM_SIZE_W,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
MMU_BOOT_REGION(DT_APL_GPIO_BASE_ADDRESS_SW,
		DT_APL_GPIO_MEM_SIZE_SW,
		(MMU_ENTRY_READ | MMU_ENTRY_WRITE));
#endif

#endif  /* CONFIG_X86_MMU */
