#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "ads1299.h"
#include "ble.h"

#define PRIORITY 1

#define ADS1299_NUM_CHANNELS          8
#define ADS1299_BITS_PER_CHANNEL      24
#define ADS1299_BYTES_PER_CHANNEL     (ADS1299_BITS_PER_CHANNEL / 8) // 3 bytes
#define ADS1299_STATUS_BITS           24
#define ADS1299_STATUS_BYTES          (ADS1299_STATUS_BITS / 8)      // 3 bytes
#define ADS1299_DATA_SIZE             (ADS1299_STATUS_BYTES + (ADS1299_NUM_CHANNELS * ADS1299_BYTES_PER_CHANNEL)) // 27 bytes
#define DATA_SIZE ADS1299_DATA_SIZE  // 27 bytes

#define BUFFER_SIZE 9  // Number of samples to buffer before sending

void process_data(uint8_t *data_buffer, int32_t *channel_data)
{
    uint32_t status = 0;

    /* Combine the 3 status bytes into a 24-bit status word */
    status = (data_buffer[0] << 16) | (data_buffer[1] << 8) | data_buffer[2];

    for (int i = 0; i < ADS1299_NUM_CHANNELS; i++) {
        /* Combine three bytes into a 24-bit signed integer (two's complement) */
        channel_data[i] = (data_buffer[3 + (i * 3)] << 16) |
                          (data_buffer[4 + (i * 3)] << 8) |
                          (data_buffer[5 + (i * 3)]);

        /* Sign extension for negative values */
        if (channel_data[i] & 0x800000) {
            channel_data[i] |= 0xFF000000;
        }

        // Uncomment for debugging individual channel data
        // printk("Channel %d: %d\n", i + 1, channel_data[i]);
    }

    // Further processing can be done here if needed
}

void data_thread(void)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ads1299));
    uint8_t data_buffer[DATA_SIZE];
    int64_t prev_time = 0;
    int64_t current_time = 0; // Variable to store the current timestamp
    int32_t delta_time = 0;   // Time difference between reads
    float frequency = 0.0;    // Variable to store the calculated frequency

    int err;

    // Buffers for storing 9 samples
    int32_t channel1_buffer[BUFFER_SIZE];
    int32_t channel2_buffer[BUFFER_SIZE];
    int32_t channel3_buffer[BUFFER_SIZE];
    int32_t channel4_buffer[BUFFER_SIZE];
    int32_t channel5_buffer[BUFFER_SIZE];
    int32_t channel6_buffer[BUFFER_SIZE];
    int32_t channel7_buffer[BUFFER_SIZE];
    int32_t channel8_buffer[BUFFER_SIZE];


    int buffer_index = 0;  // Current index in the buffer

    if (!device_is_ready(dev)) {
        printk("ADS1299 device not ready\n");
        return;
    }

    prev_time = k_uptime_get();

    while (1) {
        err = ads1299_read_data(dev, data_buffer, DATA_SIZE); // Capture return value
        if (err == 0) {
            // Get the current timestamp
            current_time = k_uptime_get();

            // Calculate time difference (in milliseconds)
            delta_time = current_time - prev_time;

            // Calculate the frequency (in Hz) if delta_time > 0 to avoid division by zero
            if (delta_time > 0) {
                frequency = 1000.0 / delta_time;
                // Uncomment for debugging frequency
                // printk("Data read frequency: %.2f Hz\n", frequency);
            }

            // Update the previous timestamp
            prev_time = current_time;

            // Process your data here
            int32_t channel_data[ADS1299_NUM_CHANNELS];
            process_data(data_buffer, channel_data);

            // Accumulate data in buffers
            channel1_buffer[buffer_index] = channel_data[0];
            channel2_buffer[buffer_index] = channel_data[1];
            channel3_buffer[buffer_index] = channel_data[2];
            channel4_buffer[buffer_index] = channel_data[3];
            channel5_buffer[buffer_index] = channel_data[4];
            channel6_buffer[buffer_index] = channel_data[5];
            channel7_buffer[buffer_index] = channel_data[6];
            channel8_buffer[buffer_index] = channel_data[7];
            
            buffer_index++;

            // If buffer is full, send the data
            if (buffer_index >= BUFFER_SIZE) {
                // Send channel1_buffer
                int ret1 = stream_sensor_data(CHANNEL1, channel1_buffer, sizeof(channel1_buffer));
                if (ret1 < 0) {
                    printk("Failed to stream CHANNEL1 data: %d\n", ret1);
                }

                // Send channel2_buffer
                int ret2 = stream_sensor_data(CHANNEL2, channel2_buffer, sizeof(channel2_buffer));
                if (ret2 < 0) {
                    printk("Failed to stream CHANNEL2 data: %d\n", ret2);
                }

                // Send channel3_buffer
                int ret3 = stream_sensor_data(CHANNEL3, channel3_buffer, sizeof(channel3_buffer));
                if (ret3 < 0) {
                    printk("Failed to stream CHANNEL3 data: %d\n", ret3);
                }

                // Reset buffer index
                buffer_index = 0;

                // Optional: Log that data was sent
                // printk("Buffered 9 samples sent via BLE.\n");
            }

        } else {
            printk("Error reading data: %d\n", err); // Enhanced error logging
        }

        // Optional: Yield to allow other threads to run
        k_yield();
    }
}

K_THREAD_DEFINE(data_thread_id, 4096, data_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
