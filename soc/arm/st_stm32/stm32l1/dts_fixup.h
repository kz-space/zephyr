/*
 * Copyright (c) 2019 Linaro Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* SoC level DTS fixup file */

#define DT_NUM_IRQ_PRIO_BITS	        DT_ARM_V7M_NVIC_E000E100_ARM_NUM_IRQ_PRIORITY_BITS

#define DT_UART_STM32_USART_1_BASE_ADDRESS	DT_ST_STM32_USART_40013800_BASE_ADDRESS
#define DT_UART_STM32_USART_1_BAUD_RATE	DT_ST_STM32_USART_40013800_CURRENT_SPEED
#define DT_UART_STM32_USART_1_IRQ_PRI	DT_ST_STM32_USART_40013800_IRQ_0_PRIORITY
#define DT_UART_STM32_USART_1_NAME		DT_ST_STM32_USART_40013800_LABEL
#define DT_USART_1_IRQ				DT_ST_STM32_USART_40013800_IRQ_0
#define DT_UART_STM32_USART_1_CLOCK_BITS	DT_ST_STM32_USART_40013800_CLOCK_BITS
#define DT_UART_STM32_USART_1_CLOCK_BUS	DT_ST_STM32_USART_40013800_CLOCK_BUS
#define DT_UART_STM32_USART_1_HW_FLOW_CONTROL DT_ST_STM32_USART_40013800_HW_FLOW_CONTROL

#define DT_UART_STM32_USART_2_BASE_ADDRESS	DT_ST_STM32_USART_40004400_BASE_ADDRESS
#define DT_UART_STM32_USART_2_BAUD_RATE	DT_ST_STM32_USART_40004400_CURRENT_SPEED
#define DT_UART_STM32_USART_2_IRQ_PRI	DT_ST_STM32_USART_40004400_IRQ_0_PRIORITY
#define DT_UART_STM32_USART_2_NAME		DT_ST_STM32_USART_40004400_LABEL
#define DT_USART_2_IRQ				DT_ST_STM32_USART_40004400_IRQ_0
#define DT_UART_STM32_USART_2_CLOCK_BITS	DT_ST_STM32_USART_40004400_CLOCK_BITS
#define DT_UART_STM32_USART_2_CLOCK_BUS	DT_ST_STM32_USART_40004400_CLOCK_BUS
#define DT_UART_STM32_USART_2_HW_FLOW_CONTROL DT_ST_STM32_USART_40004400_HW_FLOW_CONTROL

#define DT_UART_STM32_USART_3_BASE_ADDRESS	DT_ST_STM32_USART_40004800_BASE_ADDRESS
#define DT_UART_STM32_USART_3_BAUD_RATE	DT_ST_STM32_USART_40004800_CURRENT_SPEED
#define DT_UART_STM32_USART_3_IRQ_PRI	DT_ST_STM32_USART_40004800_IRQ_0_PRIORITY
#define DT_UART_STM32_USART_3_NAME		DT_ST_STM32_USART_40004800_LABEL
#define DT_USART_3_IRQ				DT_ST_STM32_USART_40004800_IRQ_0
#define DT_UART_STM32_USART_3_CLOCK_BITS	DT_ST_STM32_USART_40004800_CLOCK_BITS
#define DT_UART_STM32_USART_3_CLOCK_BUS	DT_ST_STM32_USART_40004800_CLOCK_BUS
#define DT_UART_STM32_USART_3_HW_FLOW_CONTROL DT_ST_STM32_USART_40004800_HW_FLOW_CONTROL

/* End of SoC Level DTS fixup file */
