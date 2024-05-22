/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include "dmm.h"

#define NODE_SPIM_SLOW DT_ALIAS(spi_slow_dut)
#define NODE_SPIM_FAST DT_ALIAS(spi_fast_dut)

static void dmm_showcase(void *memory_region)
{
	uint8_t data[32];
	void * buf;
	int retval;

	printf("Memory region: %p. User buffer address: %p\n", memory_region, data);

	retval = dmm_buffer_out_prepare(memory_region, data, sizeof(data), &buf);
	__ASSERT(retval == 0, "Output buffer allocation failed.");
	printf("Output buffer=%p\n", buf);
	dmm_buffer_out_release(memory_region, buf);

	retval = dmm_buffer_in_prepare(memory_region, data, sizeof(data), &buf);
	__ASSERT(retval == 0, "Input buffer allocation failed.");
	printf("Input buffer=%p\n", buf);
	dmm_buffer_in_release(memory_region, data, sizeof(data), buf);
}

int main(void)
{

	printf("DMM sample start\n");

	dmm_showcase(DMM_GET_MEM(NODE_SPIM_SLOW));
	dmm_showcase(DMM_GET_MEM(NODE_SPIM_FAST));

	return 0;
}
