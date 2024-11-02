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
#include <arm_math.h>
#include <max86141.h>

LOG_MODULE_REGISTER(MAXBLE, LOG_LEVEL_INF);

#define SPIOP	SPI_WORD_SET(8) | SPI_TRANSFER_MSB




#define BLOCK_SIZE 32
#define NUM_TAPS 51

// Example FIR low-pass filter coefficients




// K_THREAD_DEFINE(computationally_exp, 4096, computation_task, NULL, NULL, NULL, 7, 0, 0);
void print_thread_info(const struct k_thread *thread, void *user_data)
{
    const char *thread_name = k_thread_name_get((struct k_thread *)thread);
    
    printk("Thread ID: %p, Name: %s, Priority: %d\n",
           thread, 
           thread_name ? thread_name : "Unnamed",
           k_thread_priority_get((struct k_thread *)thread));
}

void main(void)
{
    printk("Listing all threads:\n");
    k_thread_foreach(print_thread_info, NULL);
    cpu_load_init();
}