#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include "w25m02gw.h"

#define STACK_SIZE (2048 * 10)
#define PRIORITY 5

#define PAGES_PER_BLOCK 64
#define PAGE_SIZE 2048    // Data area size per page
#define TOTAL_DATA_TO_WRITE (2 * 1024 * 1024) // 2MB
#define INT_SIZE 4        // Size of an integer in bytes

void test_flash_thread(void)
{
    /* Obtain the flash device using the node label from your device tree */
    const struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(w25m02gw));

    if (!device_is_ready(flash_dev)) {
        printk("Flash device is not ready\n");
        return;
    }

    uint8_t buffer[PAGE_SIZE];
    uint8_t read_buffer[PAGE_SIZE];
    uint32_t total_data_written = 0;
    uint32_t total_ints_written = 0;
    uint32_t total_data_read = 0;
    uint32_t total_ints_read = 0;

    printk("Starting flash test...\n");

    /* First, erase the necessary blocks */
    uint32_t total_pages = TOTAL_DATA_TO_WRITE / PAGE_SIZE;
    uint32_t total_blocks = (total_pages + PAGES_PER_BLOCK - 1) / PAGES_PER_BLOCK;

    for (uint16_t block_addr = 0; block_addr < total_blocks; block_addr++) {
        int err = w25m02gw_erase_block(flash_dev, block_addr);
        if (err < 0) {
            printk("Failed to erase block %u\n", block_addr);
            return;
        }
        printk("Erased block %u\n", block_addr);
    }

    /* Write data to flash */
    while (total_data_written < TOTAL_DATA_TO_WRITE) {
        /* Fill buffer with incrementing integers */
        for (int i = 0; i < PAGE_SIZE / INT_SIZE; i++) {
            uint32_t value = total_ints_written + i;
            memcpy(&buffer[i * INT_SIZE], &value, INT_SIZE);
        }

        /* Compute block and page numbers */
        uint32_t current_page = total_data_written / PAGE_SIZE;
        uint16_t block_addr = current_page / PAGES_PER_BLOCK;
        uint8_t page_addr = current_page % PAGES_PER_BLOCK;

        /* Write buffer to flash */
        int err = w25m02gw_write(flash_dev, block_addr, page_addr, buffer, PAGE_SIZE);
        if (err < 0) {
            printk("Failed to write to flash at block %u, page %u\n", block_addr, page_addr);
            return;
        }
        printk("Wrote to block %u, page %u\n", block_addr, page_addr);

        /* Update counters */
        total_data_written += PAGE_SIZE;
        total_ints_written += PAGE_SIZE / INT_SIZE;
    }

    printk("Write operations completed.\n");

    /* Read back and verify data */
    total_data_read = 0;
    total_ints_read = 0;

    while (total_data_read < TOTAL_DATA_TO_WRITE) {
        /* Compute block and page numbers */
        uint32_t current_page = total_data_read / PAGE_SIZE;
        uint16_t block_addr = current_page / PAGES_PER_BLOCK;
        uint8_t page_addr = current_page % PAGES_PER_BLOCK;

        /* Read data from flash */
        int err = w25m02gw_read(flash_dev, block_addr, page_addr,0, read_buffer, PAGE_SIZE);
        if (err < 0) {
            printk("Failed to read from flash at block %u, page %u\n", block_addr, page_addr);
            return;
        }

        /* Verify data */
        for (int i = 0; i < PAGE_SIZE / INT_SIZE; i++) {
            uint32_t expected_value = total_ints_read + i;
            uint32_t read_value;
            memcpy(&read_value, &read_buffer[i * INT_SIZE], INT_SIZE);
            if (read_value != expected_value) {
                printk("Data mismatch at int %u: expected %u, got %u\n", total_ints_read + i, expected_value, read_value);
                return;
            }else{
                printk("Data match at int %u: expected %u, got %u\n", total_ints_read + i, expected_value, read_value);
            }
        }
        printk("Verified data at block %u, page %u\n", block_addr, page_addr);

        /* Update counters */
        total_data_read += PAGE_SIZE;
        total_ints_read += PAGE_SIZE / INT_SIZE;
    }

    printk("Flash test completed successfully.\n");
}

// K_THREAD_DEFINE(flash_test_tid, STACK_SIZE, test_flash_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
