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
const float32_t fir_coeffs[NUM_TAPS] = {
    // Paste the coefficients from your Python/SciPy design here
    // Example coefficients (replace with your actual values)
    -0.0016, -0.0024, -0.0025, -0.0014, 0.0009,
     0.0051, 0.0100, 0.0153, 0.0209, 0.0253,
     0.0273, 0.0265, 0.0224, 0.0148, 0.0039,
    -0.0107, -0.0263, -0.0433, -0.0608, -0.0771,
    -0.0901, -0.0989, -0.1027, -0.1007, -0.0923,
    -0.0771, -0.0559, -0.0293, 0.0014, 0.0341,
     0.0676, 0.0990, 0.1264, 0.1476, 0.1600,
     0.1610, 0.1488, 0.1217, 0.0776, 0.0146,
    -0.0600, -0.1527, -0.2638, -0.3922, -0.5371,
    -0.6963, -0.8651, -1.0371, -1.2066, -1.3685
};

// Instance structure for FIR filter
static arm_fir_instance_f32 S;
static float32_t state_fir_f32[BLOCK_SIZE + NUM_TAPS - 1];


// K_THREAD_DEFINE(computationally_exp, 4096, computation_task, NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
	const float32_t input_f32[3] = {1.0, 2.0, 3.0};
	
	cpu_load_init();

	
	return 0;
}