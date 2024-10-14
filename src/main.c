/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/init.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>

#include "ble.h"


#include <debug/cpu_load.h>

#include <max86141.h>

LOG_MODULE_REGISTER(MAXBLE, LOG_LEVEL_INF);

#define SPIOP	SPI_WORD_SET(8) | SPI_TRANSFER_MSB







// K_THREAD_DEFINE(computationally_exp, 4096, computation_task, NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
	
	cpu_load_init();

	
	return 0;
}