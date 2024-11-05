// File: src/sensor_data_manager.c

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "ads1299.h"
#include "ble.h"
#include "disk.h"

#define PRIORITY 2

#define ADS1299_SAMPLE_RATE CONFIG_ADS1299_SAMPLE_RATE
#define BLE_STREAM_INTERVAL (ADS1299_SAMPLE_RATE / 250)

#define FLASH_BUFFER_BATCH_SIZE 75
#define BLE_BUFFER_SIZE 60
#define ADS1299_NUM_CHANNELS 8
#define ADS1299_DATA_SIZE 27

const struct device *ads1299_dev = DEVICE_DT_GET(DT_NODELABEL(ads1299));

LOG_MODULE_REGISTER(DATA_MANAGER, LOG_LEVEL_INF);

// Global variables
static bool recording_active = false;
int ble_stream_counter = 0;

// Flash buffer
static uint8_t flash_data_buffer[2048] = {0};
static uint8_t flash_buffer_batch_index = 0;

// Read buffer
static uint8_t read_buffer[2048];

// Forward declarations
static void process_data(uint8_t *data_buffer, int32_t *channel_data);
static void save_data_to_flash(uint8_t *data_buffer);
static void read_data_cb(const uint8_t *data, size_t length);
static void download_data(void);
void end_recording(void);

// Work items for offloading heavy processing
static void toggle_sensor_work_handler(struct k_work *work);
static void sensor_download_work_handler(struct k_work *work);

K_WORK_DEFINE(toggle_sensor_work, toggle_sensor_work_handler);
K_WORK_DEFINE(sensor_download_work, sensor_download_work_handler);

static uint8_t toggle_sensor_value;
static uint8_t sensor_download_value;

// Function to process data from flash (used in read_data_cb)
static void process_flash_data(uint8_t *data, size_t length);

// Work handler for toggle_sensor
void toggle_sensor_work_handler(struct k_work *work)
{
    if (toggle_sensor_value) {
        // Wake up sensor
        ads1299_wakeup(ads1299_dev);
        LOG_INF("Sensor awakened");

        // Start a new recording with default name
        DiskStatus status = disk_start_recording("NI_recording");
        if (status != DISK_OK) {
            LOG_ERR("Failed to start recording: %d", status);
        } else {
            recording_active = true;
            LOG_INF("Recording started");
        }
    } else {
        // Put sensor in standby
        ads1299_standby(ads1299_dev);
        LOG_INF("Sensor put in standby");

        // End recording
        end_recording();
    }
}

// Work handler for sensor_download_data
void sensor_download_work_handler(struct k_work *work)
{
    // Read data from flash
    DiskStatus status = disk_read_data(read_data_cb);
    if (status != DISK_OK) {
        LOG_ERR("Failed to read data from flash: %d", status);
    }
}

// BLE callback functions
static int toggle_sensor(uint8_t val)
{
    LOG_INF("Scheduling toggle_sensor to %d", val);
    toggle_sensor_value = val;
    k_work_submit(&toggle_sensor_work);
    return 0;
}

void read_data_cb(const uint8_t *data, size_t length)
{
    // Process data read from flash
    process_flash_data((uint8_t *)data, length);
}

static int sensor_download_data(uint8_t val)
{

    LOG_INF("Scheduling sensor data download: %d", val);
    sensor_download_value = val;
    k_work_submit(&sensor_download_work);
    return 0;
}

// BLE callback structure
struct ble_cb cb = {
    .sensor_switch_cb = toggle_sensor,
    .sensor_data_download_cb = sensor_download_data,
};

// Process raw data from ADS1299
static void process_data(uint8_t *data_buffer, int32_t *channel_data)
{
    uint32_t status = 0;

    // Combine the 3 status bytes into a 24-bit status word
    status = (data_buffer[0] << 16) | (data_buffer[1] << 8) | data_buffer[2];

    for (int i = 0; i < ADS1299_NUM_CHANNELS; i++) {
        // Combine three bytes into a 24-bit signed integer (two's complement)
        channel_data[i] = (data_buffer[3 + (i * 3)] << 16) |
                          (data_buffer[4 + (i * 3)] << 8) |
                          (data_buffer[5 + (i * 3)]);

        // Sign extension for negative values
        if (channel_data[i] & 0x800000) {
            channel_data[i] |= 0xFF000000;
        }
    }

    // Further processing can be done here if needed
}

// Process data read from flash
static void process_flash_data(uint8_t *data, size_t length)
{
    uint8_t data_buffer[ADS1299_DATA_SIZE];
    int32_t channel_data[ADS1299_NUM_CHANNELS];
    int ble_buffer_index = 0;

    size_t data_points = length / ADS1299_DATA_SIZE;
    int32_t ble_channel_buffers[ADS1299_NUM_CHANNELS][BLE_BUFFER_SIZE];
    for (size_t i = 0; i < data_points; i++)
    {
        // Copy one data point from the data buffer
        memcpy(data_buffer, &data[i * ADS1299_DATA_SIZE], ADS1299_DATA_SIZE);

        // Process the raw data to get channel data
        process_data(data_buffer, channel_data);

        // Accumulate the channel data into BLE buffers
        for (int ch = 0; ch < ADS1299_NUM_CHANNELS; ch++)
        {
            ble_channel_buffers[ch][ble_buffer_index] = channel_data[ch];
        }

        ble_buffer_index++;

        // If the BLE buffer is full, send the data over BLE
        if (ble_buffer_index >= BLE_BUFFER_SIZE)
        {
            // Stream data over BLE for each channel
            for (int ch = 0; ch < ADS1299_NUM_CHANNELS; ch++)
            {
                int ret = stream_sensor_data(ch, ble_channel_buffers[ch], ble_buffer_index * sizeof(int32_t));
                if (ret < 0)
                {
                    LOG_ERR("Failed to stream CHANNEL%d data: %d", ch + 1, ret);
                }
            }

            // Reset BLE buffer index
            ble_buffer_index = 0;
        }
    }

    // After processing all data, send any remaining data
    if (ble_buffer_index > 0)
    {
        // Stream the remaining data over BLE
        for (int ch = 0; ch < ADS1299_NUM_CHANNELS; ch++)
        {
            int ret = stream_sensor_data(ch, ble_channel_buffers[ch], ble_buffer_index * sizeof(int32_t));
            if (ret < 0)
            {
                LOG_ERR("Failed to stream CHANNEL%d data: %d", ch + 1, ret);
            }
        }
    }
}


static void save_data_to_flash(uint8_t *data_buffer)
{
    // Copy data to flash buffer
    memcpy(&flash_data_buffer[flash_buffer_batch_index * ADS1299_DATA_SIZE], data_buffer, ADS1299_DATA_SIZE);
    flash_buffer_batch_index++;
    if (flash_buffer_batch_index >= FLASH_BUFFER_BATCH_SIZE) {
        // Write data to flash
        DiskStatus status = disk_write_data(flash_data_buffer, sizeof(flash_data_buffer));
        flash_buffer_batch_index = 0;
        if (status != DISK_OK) {
            LOG_ERR("Failed to write data to flash: %d", status);
        }
    }
}

void end_recording(void)
{
    // Write any remaining data to flash
    if (flash_buffer_batch_index > 0) {
        size_t remaining_size = flash_buffer_batch_index * ADS1299_DATA_SIZE;
        DiskStatus status = disk_write_data(flash_data_buffer, remaining_size);
        if (status != DISK_OK) {
            LOG_ERR("Failed to write remaining data to flash: %d", status);
        }
        flash_buffer_batch_index = 0;
    }

    // Stop recording
    recording_active = false;
}

// Main data thread
void data_thread(void)
{
    LOG_INF("data_thread started. Thread ID: %p", k_current_get());
    k_thread_name_set(k_current_get(), "DataThread");
    register_ble_cb(&cb);

    uint8_t data_buffer[ADS1299_DATA_SIZE];
    int32_t channel_data[ADS1299_NUM_CHANNELS];
    int err;

    if (!device_is_ready(ads1299_dev)) {
        LOG_ERR("ADS1299 device not ready");
        return;
    }

    // Initialize disk system
    DiskStatus disk_status = disk_init();
    if (disk_status != DISK_OK) {
        LOG_ERR("Disk initialization failed: %d", disk_status);
        return;
    }

    int32_t ble_channel_buffers[ADS1299_NUM_CHANNELS][BLE_BUFFER_SIZE];
    int ble_buffer_index = 0;

    while (1) {
        // Read data from ADS1299 (controlled by semaphore)
        err = ads1299_read_data(ads1299_dev, data_buffer, ADS1299_DATA_SIZE);

        if (err == 0) {
            // Process raw data
            process_data(data_buffer, channel_data);

            // Save data to flash if recording is active
            if (recording_active) {
                save_data_to_flash(data_buffer);
            }

            // Stream data over BLE
            if (ble_stream_counter++ == BLE_STREAM_INTERVAL) {
                ble_stream_counter = 0;

                for (int ch = 0; ch < ADS1299_NUM_CHANNELS; ch++) {
                    ble_channel_buffers[ch][ble_buffer_index] = channel_data[ch];
                }
                ble_buffer_index++;

                // If buffers are full, send data over BLE
                if (ble_buffer_index >= BLE_BUFFER_SIZE) {
                    // Stream data over BLE for each channel
                    for (int ch = 0; ch < ADS1299_NUM_CHANNELS; ch++) {
                        int ret = stream_sensor_data(ch, ble_channel_buffers[ch], sizeof(ble_channel_buffers[ch]));
                        if (ret < 0) {
                            LOG_ERR("Failed to stream CHANNEL%d data: %d", ch + 1, ret);
                        }
                    }

                    // Reset BLE buffer index
                    ble_buffer_index = 0;
                }
            }
            k_yield();
        }
    }
}

K_THREAD_DEFINE(data_thread_id, 4096 * 4, data_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
