#include <stdio.h>
#include <string.h> // Include for memcpy
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "ads1299.h"
#include "ble.h"
#include "disk.h" // Include the disk API

#define PRIORITY 2

// ADS1299 configuration
#define ADS1299_NUM_CHANNELS          8
#define ADS1299_BITS_PER_CHANNEL      24
#define ADS1299_BYTES_PER_CHANNEL     (ADS1299_BITS_PER_CHANNEL / 8) // 3 bytes
#define ADS1299_STATUS_BITS           24
#define ADS1299_STATUS_BYTES          (ADS1299_STATUS_BITS / 8)      // 3 bytes
#define ADS1299_DATA_SIZE             (ADS1299_STATUS_BYTES + (ADS1299_NUM_CHANNELS * ADS1299_BYTES_PER_CHANNEL)) // 27 bytes

// Sample rate configuration (from Kconfig)
#define SAMPLE_RATE CONFIG_ADS1299_SAMPLE_RATE // e.g., 250, 500, ..., 16000

// Flash buffer configuration
#define FLASH_BUFFER_DURATION_SEC 1 // Buffer duration in seconds
#define FLASH_BUFFER_SIZE (SAMPLE_RATE * ADS1299_DATA_SIZE * FLASH_BUFFER_DURATION_SEC)

// BLE streaming configuration
#define BLE_STREAM_RATE 250 // Hz
#define DOWNSAMPLE_FACTOR (SAMPLE_RATE / BLE_STREAM_RATE)

// Ensure the downsample factor is at least 1
#if DOWNSAMPLE_FACTOR < 1
    #undef DOWNSAMPLE_FACTOR
    #define DOWNSAMPLE_FACTOR 1
#endif

#define BLE_BUFFER_SIZE 60 // Number of samples to buffer per channel before sending

const struct device *ads1299_dev = DEVICE_DT_GET(DT_NODELABEL(ads1299));

LOG_MODULE_REGISTER(DATA_MANAGER, LOG_LEVEL_INF);

// Global variables
static bool recording_active = false; // Flag indicating if recording is active

// Forward declarations
static void process_data(uint8_t *data_buffer, int32_t *channel_data);
static void save_data_to_flash(uint8_t *data_buffer, size_t data_size);
static void download_data(void);

// BLE callback functions
static int toggle_sensor(uint8_t val)
{
    LOG_INF("Toggling sensor to %d", val);

    if (val) {
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

        // Stop recording
        recording_active = false;
    }

    return 0;
}

static int sensor_download_data(uint8_t val)
{
    if (val) {
        LOG_INF("Initiating data download from flash");
        download_data();
    }
    return 0;
}

// BLE callback structure
struct ble_cb cb = {
    .sensor_switch_cb = toggle_sensor,
    .sensor_data_download_cb = sensor_download_data, // Implement if needed
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

// Save data to flash buffer
static void save_data_to_flash(uint8_t *data_buffer, size_t data_size)
{
    static uint8_t flash_buffer[FLASH_BUFFER_SIZE];
    static size_t flash_buffer_index = 0;

    // Copy data to flash buffer
    memcpy(&flash_buffer[flash_buffer_index], data_buffer, data_size);
    flash_buffer_index += data_size;

    // If flash buffer is full, write to flash
    if (flash_buffer_index >= FLASH_BUFFER_SIZE) {
        if (recording_active) {
            DiskStatus status = disk_write_data(flash_buffer, flash_buffer_index);
            if (status != DISK_OK) {
                LOG_ERR("Failed to write data to flash: %d", status);
            } else {
                LOG_INF("Data written to flash: %u bytes", (unsigned int)flash_buffer_index);
            }
        }
        // Reset flash buffer index
        flash_buffer_index = 0;
    }
}

// Callback function for disk_read_data()
static void flash_data_callback(const uint8_t *data, size_t length)
{
    int32_t channel_data[ADS1299_NUM_CHANNELS];
    for(int i = 0; i < length; i += ADS1299_DATA_SIZE) {
        printk("Data: %x\n", data[i]);
    }
    process_data((uint8_t *)data, channel_data);

    // Stream uncompressed data over BLE
    for (int ch = 0; ch < ADS1299_NUM_CHANNELS; ch++) {
        int ret = stream_sensor_data(ch, &channel_data[ch], sizeof(int32_t));
        if (ret < 0) {
            LOG_ERR("Failed to stream CHANNEL%d data: %d", ch + 1, ret);
        }
    }
}

// Download data from flash, process, and stream over BLE
static void download_data(void)
{
    // Read data from flash using disk_read_data()
    DiskStatus status = disk_read_data(flash_data_callback);

    if (status != DISK_OK) {
        LOG_ERR("Failed to read data from flash: %d", status);
    } else {
        LOG_INF("Data download complete");
    }
}

// Main data thread
void data_thread(void)
{
    LOG_INF("data_thread started. Thread ID: %p", k_current_get());
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

    // Variables for downsampling
    uint32_t sample_counter = 0;

    // BLE buffers for each channel
    int32_t ble_channel_buffers[ADS1299_NUM_CHANNELS][BLE_BUFFER_SIZE];
    int ble_buffer_index = 0; // Common buffer index for all channels

    while (1) {
        // Read data from ADS1299 (controlled by semaphore)
        err = ads1299_read_data(ads1299_dev, data_buffer, ADS1299_DATA_SIZE);

        if (err == 0) {
            // Process raw data
            process_data(data_buffer, channel_data);

            // Save data to flash buffer
            save_data_to_flash(data_buffer, ADS1299_DATA_SIZE);

            // Handle BLE streaming
            if (sample_counter % DOWNSAMPLE_FACTOR == 0) {
                // Buffer downsampled data for BLE streaming
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

            sample_counter++;

        } else {
            LOG_ERR("Error reading data: %d", err);
        }

        // Optional: Yield to allow other threads to run
        k_yield();
    }
}

// K_THREAD_DEFINE(data_thread_id, 4096*30, data_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
