// File: src/stream.c

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "stream.h"
#include "ble.h" // Include BLE functions if necessary

LOG_MODULE_REGISTER(STREAM, LOG_LEVEL_INF);

#define BLE_STREAM_THREAD_STACK_SIZE 10240
#define BLE_STREAM_THREAD_PRIORITY -5

#define BLE_STREAM_DATA_COUNT 250 // Adjust as needed

#define ADS1299_NUM_CHANNELS 8   // Define the number of channels
#define BLE_BUFFER_SIZE 40       // Number of data points to accumulate

extern struct k_fifo ble_stream_fifo; // Declare the FIFO defined in stream.c
extern struct k_mem_slab ble_stream_slab; // Declare the memory slab


// Define the memory slab
K_MEM_SLAB_DEFINE(ble_stream_slab, sizeof(struct ble_stream_data), BLE_STREAM_DATA_COUNT, 4);

// Define the FIFO queue
K_FIFO_DEFINE(ble_stream_fifo);

void ble_stream_thread(void)
{
    LOG_INF("BLE stream thread started");

    // Buffers to accumulate data
    int32_t channel_buffers[ADS1299_NUM_CHANNELS][BLE_BUFFER_SIZE];
    int buffer_index = 0;
    int counter = 0;
    while (1)
    {
        // Wait indefinitely for data
        struct ble_stream_data *stream_data = k_fifo_get(&ble_stream_fifo, K_FOREVER);

        if (stream_data)
        {

            // Accumulate data into buffers
            for (int ch = 0; ch < ADS1299_NUM_CHANNELS; ch++)
            {
                channel_buffers[ch][buffer_index] = stream_data->channel_data[ch];

            }

            buffer_index++;

            // Free the allocated memory back to the slab


            k_mem_slab_free(&ble_stream_slab, (void *)stream_data);
            // printk("Data: %d\n", stream_data->channel_data[0]); 
            uint32_t data = stream_data->channel_data[0];






            // When buffer_index reaches BLE_BUFFER_SIZE, send data over BLE
            if (buffer_index >= BLE_BUFFER_SIZE)
            {
                // Stream data over BLE for each channel
                for (int ch = 0; ch < ADS1299_NUM_CHANNELS; ch++)
                {
                    int ret = stream_sensor_data(ch, channel_buffers[ch], BLE_BUFFER_SIZE * sizeof(int32_t));
                    if (ret < 0)
                    {
                        LOG_ERR("Failed to stream CHANNEL%d data: %d", ch + 1, ret);
                    }
                }

                // Reset buffer_index
                buffer_index = 0;

            }

        

        }
    }
}

K_THREAD_DEFINE(ble_stream_thread_id, BLE_STREAM_THREAD_STACK_SIZE, ble_stream_thread, NULL, NULL, NULL, BLE_STREAM_THREAD_PRIORITY, 0, 0);
