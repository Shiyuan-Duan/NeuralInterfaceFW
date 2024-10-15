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

int count = 0;
void process_data(uint8_t *data_buffer)
{
    uint32_t status = 0;
    int32_t channel_data[ADS1299_NUM_CHANNELS];

    /* Combine the 3 status bytes into a 24-bit status word */
    status = (data_buffer[0] << 16) | (data_buffer[1] << 8) | data_buffer[2];

    printk("Status: 0x%06x\n", status);

    for (int i = 0; i < ADS1299_NUM_CHANNELS; i++) {
        /* Combine three bytes into a 24-bit signed integer (two's complement) */
        channel_data[i] = (data_buffer[3 + (i * 3)] << 16) |
                          (data_buffer[4 + (i * 3)] << 8) |
                          (data_buffer[5 + (i * 3)]);

        /* Sign extension for negative values */
        if (channel_data[i] & 0x800000) {
            channel_data[i] |= 0xFF000000;
        }

        // printk("Channel %d: %d\n", i + 1, channel_data[i]);
        


    }

    // Only stream the first 3 channels.
    for (int i = 0; i < 3; i++) {
        stream_sensor_data(i, &channel_data[i], sizeof(int32_t));
    }


    // Further processing can be done here
}




void data_thread(void)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ads1299));
    uint8_t data_buffer[DATA_SIZE];
    int err;

    if (!device_is_ready(dev)) {
        printk("ADS1299 device not ready\n");
        return;
    }

    while (1) {
        err = ads1299_read_data(dev, data_buffer, DATA_SIZE); // Capture return value
        if (err == 0) {
            // Process your data here
            process_data(data_buffer);

        } else {
            printk("Error reading data: %d\n", err); // Enhanced error logging
        }



        // Optional: Yield to allow other threads to run
        k_yield();
    }
}

K_THREAD_DEFINE(data_thread_id, 2048, data_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
