// // File: src/sensor_data_manager.c

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "afe4960.h"
#include "ble_sensor_control.h"
// #include "disk.h"
// #include "stream.h"

#define PRIORITY -1
const struct device *afe4960_dev = DEVICE_DT_GET(DT_NODELABEL(afe4960));
LOG_MODULE_REGISTER(AFE4960_DATA_MANAGER, LOG_LEVEL_INF);
// Main data thread

static int process_raw_data(uint8_t *data_buffer, int32_t *ecg_buffer, int32_t *biozi_buffer, int32_t *biozq_buffer)
{
    int ecg_index = 0;
    int bioz_i_index = 0;
    int bioz_q_index = 0;

    // Process 900 bytes of data_buffer
    for (int i = 0; i < 384; i += 3) {
        // Combine three bytes into a 24-bit unsigned integer (big-endian)
        uint32_t raw_data = (uint32_t)(data_buffer[i] << 16 | data_buffer[i + 1] << 8 | data_buffer[i + 2]);

        // Extract the tag bits (bits [1:0])
        uint8_t tag = raw_data & 0x03;

        // Extract the 22-bit data (bits [23:2])
        int32_t data_value = (raw_data >> 2) & 0x003FFFFF;

        // Sign-extend the 22-bit data to a 32-bit signed integer
        if (data_value & 0x00200000) {
            data_value |= 0xFFC00000; // Set bits [31:22] to 1 for negative numbers
        }

        // Switch based on the tag to assign data to the correct buffer
        switch (tag) {
            case 0x01: // ECG sample
                if (ecg_index < 100) {
                    ecg_buffer[ecg_index++] = data_value;
                }
                break;
            case 0x02: // BioZ-I sample
                LOG_ERR("TAG 10? You shouldn't be here");
                break;
            case 0x03: // BioZ-Q sample
                LOG_ERR("TAG 11? You shouldn't be here");
                break;
            default:
                // Ignore unknown tags
                break;
        }
    }

    return 0;
}

void afe4960_data_thread(void)
{
    LOG_INF("data_thread started. Thread ID: %p", k_current_get());
    k_thread_name_set(k_current_get(), "DataThread");

    uint8_t data_buffer[384] = {0};
    int32_t ecg_buffer[128] = {0};
    int32_t biozi_buffer[128] = {0};
    int32_t biozq_buffer[128] = {0};
    int32_t ecg_chunk[4] = {0};

    int err;

    if (!device_is_ready(afe4960_dev)) {
        LOG_ERR("AFE4960 device not ready");
        return;
    }

    while (1) {
        // Read data from AFE4960 (controlled by semaphore)
        err = afe4960_read_data(afe4960_dev, data_buffer);
        if (err < 0) {
            LOG_ERR("Failed to read data from AFE4960: %d", err);
            continue;
        }

        // Process raw data into separate buffers
        err = process_raw_data(data_buffer, ecg_buffer, biozi_buffer, biozq_buffer);
        if (err < 0) {
            LOG_ERR("Failed to process raw data: %d", err);
            continue;
        }

        // Split ecg_buffer into chunks of 5 samples each
        for (int i = 0; i < 128; i += 4) {
            memcpy(ecg_chunk, &ecg_buffer[i], sizeof(ecg_chunk));

            // Stream the chunk
            err = stream_sensor_data(0, ecg_chunk, sizeof(ecg_chunk));
            if (err < 0) {
                LOG_ERR("Failed to stream ECG data: %d", err);
            }
        }

        k_yield();
    }
}

// K_THREAD_DEFINE(afe4960_data_thread_id, 4096 * 4, afe4960_data_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
