// ads1299_data_handler.c

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "ads1299.h"
#include "ble_sensor_control.h"
#include "disk.h"
#include "w25m02gw.h"

LOG_MODULE_REGISTER(ads1299_data_handler, LOG_LEVEL_DBG);

// Constants


////////////////////////////////////////////////
#define DEBUG_MODE 0 // Do NOT FORGET THIS!!!!!!
////////////////////////////////////////////////
#if DEBUG_MODE
#include <math.h>

#define NUM_CHANNELS 8
#define M_PI 3.14159
#define DEG_TO_RAD(angle) ((angle) * M_PI / 180.0f)

static int generate_sine_wave(uint8_t *buffer, size_t size, size_t sample_rate, float frequency, float amplitude)
{
    static size_t sample_index = 0;
    int32_t *int_buffer = (int32_t *)buffer;

    // Phase shift in radians for each channel (10 degrees per channel)
    float phase_shift[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
        phase_shift[i] = DEG_TO_RAD(10 * i);
    }

    size_t total_samples = size / sizeof(int32_t);

    for (size_t i = 0; i < total_samples; i++) {
        // Calculate the current channel (wraps around NUM_CHANNELS)
        int channel = i % NUM_CHANNELS;

        // Generate the sine wave for the current channel
        float angle = 2.0f * M_PI * frequency * (sample_index / (float)sample_rate) + phase_shift[channel];
        int_buffer[i] = (int32_t)(amplitude * sin(angle));

        // Increment sample_index for the first channel only
        if (channel == NUM_CHANNELS - 1) {
            sample_index++;
        }
    }

    return 0;
}
#endif


#define ADS1299_DATA_BUFFER_SIZE 27   // 27 bytes from ADS1299
#define ADS1299_NUM_CHANNELS 8

#define FLASH_PAGE_SIZE 2048          // 2048 bytes per page
#define FLASH_PAGES_PER_BLOCK 64
#define FLASH_BLOCK_SIZE (FLASH_PAGE_SIZE * FLASH_PAGES_PER_BLOCK)
#define FLASH_TOTAL_BLOCKS 1024

#define FLASH_BUFFER_SIZE 512          // Number of int32_t samples to buffer before writing to flash

#define BLE_PACKAGE_SIZE 128           // BLE package size in bytes
#define BLE_SAMPLES_PER_PACKAGE (BLE_PACKAGE_SIZE / (ADS1299_NUM_CHANNELS * sizeof(int32_t))) // 4 samples per channel

#define PRIORITY 1

// Device instance
const struct device *ads1299_dev = DEVICE_DT_GET(DT_NODELABEL(ads1299));
const struct device *w25m02gw_dev = DEVICE_DT_GET(DT_NODELABEL(w25m02gw));

// Data buffers
static uint8_t data_buffer[ADS1299_DATA_BUFFER_SIZE] = {0};
static int32_t flash_buffer[FLASH_BUFFER_SIZE] = {0};
static int32_t read_buffer[FLASH_BUFFER_SIZE] = {0};

// Indices and counters
static uint16_t flash_buffer_index = 0;
static uint16_t write_block_addr = 0;
static uint16_t write_page_addr = 0;
static uint16_t read_block_addr = 0;
static uint16_t read_page_addr = 0;
static size_t read_offset = 0;

static uint8_t recording_active = 1;  // Set to 1 to enable recording
static uint32_t data_counter = 0;

// BLE streaming buffer
static int32_t ble_stream_buffer[ADS1299_NUM_CHANNELS * BLE_SAMPLES_PER_PACKAGE] = {0};
static uint16_t ble_sample_index = 0;

// BLE callback struct


// Function prototypes
static void process_data(uint8_t *data_buffer);
static void write_flash_buffer(void);
static int download_data(uint8_t *output_buffer, size_t buffer_size);
static int sensor_read_data_cb(uint8_t *data);



// Process ADS1299 data and store in flash buffer
static size_t num_byte_ready(void)
{
    size_t write_offset = (write_block_addr * FLASH_BLOCK_SIZE) + (write_page_addr * FLASH_PAGE_SIZE);
    size_t read_offset_total = (read_block_addr * FLASH_BLOCK_SIZE) + (read_page_addr * FLASH_PAGE_SIZE) + read_offset;

    if (write_offset >= read_offset_total) {
        return write_offset - read_offset_total; // Normal case
    } else {
        return (FLASH_TOTAL_BLOCKS * FLASH_BLOCK_SIZE) - (read_offset_total - write_offset); // Wrap-around case
    }
}


static void preload_dummy_data(void)
{
#if DEBUG_MODE

    write_block_addr = 40;
    write_page_addr = 0;

#endif
}


static void process_data(uint8_t *data_buffer)
{
    uint32_t status = 0;

    // Combine the 3 status bytes into a 24-bit status word
    status = (data_buffer[0] << 16) | (data_buffer[1] << 8) | data_buffer[2];

    for (int i = 0; i < ADS1299_NUM_CHANNELS; i++) {
        // Combine three bytes into a 24-bit signed integer (two's complement)
        int32_t raw_value = (int32_t)((data_buffer[3 + (i * 3)] << 16) |
                                      (data_buffer[4 + (i * 3)] << 8) |
                                      (data_buffer[5 + (i * 3)]));

        // Sign extension for negative values
        if (raw_value & 0x800000) {
            raw_value |= 0xFF000000;
        }

        // Store the converted data into the flash buffer
        flash_buffer[flash_buffer_index++] = raw_value;

        // Store data into BLE stream buffer
        // ble_stream_buffer[ble_sample_index++] = raw_value;

        // Check if BLE stream buffer is full
        // if (ble_sample_index >= (ADS1299_NUM_CHANNELS * BLE_SAMPLES_PER_PACKAGE)) {
        //     send_ble_package();
        //     ble_sample_index = 0;
        // }
    }

    // Check if flash buffer is full and write to flash
    if (flash_buffer_index >= FLASH_BUFFER_SIZE) {
        write_flash_buffer();
        flash_buffer_index = 0;
    }
}

// Write flash buffer to flash memory
static void write_flash_buffer(void)
{
    int err;

    // Write the flash_buffer to flash memory
    err = w25m02gw_write(w25m02gw_dev, write_block_addr, write_page_addr, (uint8_t *)flash_buffer, sizeof(flash_buffer));
    if (err < 0) {
        LOG_ERR("Failed to write data to flash");
        return;
    }

    // Update page and block addresses
    write_page_addr++;
    if (write_page_addr >= FLASH_PAGES_PER_BLOCK) {
        write_page_addr = 0;
        write_block_addr = (write_block_addr + 1) % FLASH_TOTAL_BLOCKS;
        // Erase the new block before writing
        err = w25m02gw_erase_block(w25m02gw_dev, write_block_addr);
        if (err < 0) {
            LOG_ERR("Failed to erase flash block");
            return;
        }
    }

    // Prevent overwriting unread data
    if ((write_block_addr == read_block_addr) && (write_page_addr == read_page_addr)) {
        LOG_ERR("Flash buffer overflow: overwriting unread data");
        // Handle overflow (e.g., stop recording, overwrite old data, etc.)
        recording_active = 0;  // Example action: stop recording
    }
}

// Download data from flash memory for BLE transmission
// static int download_data(uint8_t *output_buffer, size_t buffer_size)
// {
//     int err;

//     if (read_offset == 0) {
//         // Read a page from flash into the read_buffer

//         err = w25m02gw_read(w25m02gw_dev, read_block_addr, read_page_addr, 0, (uint8_t *)read_buffer, sizeof(read_buffer));

//         if (err < 0) {
//             LOG_ERR("Failed to read data from flash");
//             return -1;
//         }

//         printk("Read buffer (int32_t):\n");
//         for (size_t i = 0; i < FLASH_BUFFER_SIZE; i++) {
//             printk("[%d]: %d\n", i, read_buffer[i]);
//         }
//     }
//     printk("Read offset: %d\n", read_offset);

//     // Calculate how much data we can copy
//     size_t bytes_available = sizeof(read_buffer) - read_offset;
//     size_t bytes_to_copy = MIN(buffer_size, bytes_available);

//     // Copy data from read_buffer to output_buffer
//     printk("Bytes to copy: %d\n", bytes_to_copy);
//     memcpy(output_buffer, ((uint8_t *)read_buffer) + read_offset, bytes_to_copy);
//     read_offset += bytes_to_copy;

//     // If we've read the entire page, move to the next one
//     if (read_offset >= sizeof(read_buffer)) {
//         read_offset = 0;
//         read_page_addr++;
//         if (read_page_addr >= FLASH_PAGES_PER_BLOCK) {
//             read_page_addr = 0;
//             read_block_addr = (read_block_addr + 1) % FLASH_TOTAL_BLOCKS;
//         }
//     }

//     return bytes_to_copy;
// }


static int download_data(uint8_t *output_buffer, size_t buffer_size)
{
    int err;

    if (read_offset == 0) {
#if DEBUG_MODE
        // Generate a sine wave instead of reading from flash
        err = generate_sine_wave((uint8_t *)read_buffer, sizeof(read_buffer), 1000, 10.0f, 1000000.0f); // 1 kHz sample rate, 10 Hz sine wave, amplitude 1M
        if (err < 0) {
            LOG_ERR("Failed to generate sine wave data");
            return -1;
        }
#else
        // Read a page from flash into the read_buffer
        err = w25m02gw_read(w25m02gw_dev, read_block_addr, read_page_addr, 0, (uint8_t *)read_buffer, sizeof(read_buffer));
        if (err < 0) {
            LOG_ERR("Failed to read data from flash");
            return -1;
        }
#endif

        // Debugging: Print the content of the read_buffer as int32_t
        printk("Read buffer (int32_t):\n");
        for (size_t i = 0; i < FLASH_BUFFER_SIZE; i++) {
            printk("[%d]: %d\n", i, read_buffer[i]);
        }
    }

    // Calculate how much data we can copy
    size_t bytes_available = sizeof(read_buffer) - read_offset;
    size_t bytes_to_copy = MIN(buffer_size, bytes_available);

    // Copy data from read_buffer to output_buffer
    printk("Bytes to copy: %d\n", bytes_to_copy);
    memcpy(output_buffer, ((uint8_t *)read_buffer) + read_offset, bytes_to_copy);
    read_offset += bytes_to_copy;

    // If we've read the entire page, move to the next one
    if (read_offset >= sizeof(read_buffer)) {
        read_offset = 0;
        read_page_addr++;
        if (read_page_addr >= FLASH_PAGES_PER_BLOCK) {
            read_page_addr = 0;
            read_block_addr = (read_block_addr + 1) % FLASH_TOTAL_BLOCKS;
        }
    }

    return bytes_to_copy;
}


// BLE callback function to read data
static int sensor_read_data_cb(uint8_t *data)
{
    printk("Reading data cb found\n");
    // Define the size of data to read (128 bytes)
    size_t data_size = BLE_PACKAGE_SIZE;
    int bytes_read = download_data(data, data_size);
    if (bytes_read < 0) {
        // Handle error
        return -1;
    }
    return bytes_read;
}


static int toggle_sensor(uint8_t value)
{
    printk("This is triggered\n");
    if (value) {
        // Wake up sensor
        ads1299_wakeup(ads1299_dev);
    } else {
        // Put sensor in standby
        ads1299_standby(ads1299_dev);

    }
    return 0;
}
// Initialize BLE callbacks
struct ble_sensor_ctrl_cb control_cb = {
    .sensor_read_data_cb = sensor_read_data_cb,
    .sensor_switch_cb = toggle_sensor,
    .sensor_read_fifo_size_cb = num_byte_ready
    // Initialize other callbacks if necessary
};

// Data acquisition thread
void data_thread(void)
{
    int err;
    printk("Is this even executed????\n");
    // Register BLE callbacks
    register_ble_cb(&control_cb);

    #if DEBUG_MODE
    preload_dummy_data();
    #endif

    if (!device_is_ready(ads1299_dev)) {
        LOG_ERR("ADS1299 device not ready");
        return;
    }

    while (1) {
        // Read data from ADS1299
        err = ads1299_read_data(ads1299_dev, data_buffer, ADS1299_DATA_BUFFER_SIZE);
        if (err == 0) {
            // Process raw data
            data_counter++;
            process_data(data_buffer);

            // Save data to flash if recording is active
            if (recording_active) {
                // Data is already saved in process_data
            }

            k_yield();
        } else {
            LOG_ERR("Failed to read data from ADS1299");
            // Handle read error if necessary
        }
    }
}

// Define the data_thread as a Zephyr thread
K_THREAD_DEFINE(data_thread_id, 4096, data_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
