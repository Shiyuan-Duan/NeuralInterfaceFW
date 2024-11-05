//disk.c
#include "disk.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include "w25m02gw.h"

// Device instance


// Flash memory specifications
#define PAGE_SIZE 2048               // Size of a page in bytes
#define PAGES_PER_BLOCK 64           // Number of pages per block
#define TOTAL_BLOCKS 1024            // Total number of blocks
#define TOTAL_SIZE (PAGE_SIZE * PAGES_PER_BLOCK * TOTAL_BLOCKS)

// Block and page constraints
#define METADATA_BLOCK 0             // Block used for metadata
#define METADATA_PAGE 0              // Page used for metadata
#define START_BLOCK 1                // Start block for recording data
#define START_PAGE 0                // Skip first 13 pages in each block
#define USABLE_PAGES_PER_BLOCK (PAGES_PER_BLOCK - START_PAGE)

LOG_MODULE_REGISTER(DISK, LOG_LEVEL_INF);

#define MIN(a, b) ((a) < (b) ? (a) : (b))


#define METADATA_SIGNATURE 0xDEADBEEF


typedef struct {
    uint32_t signature;                    // Unique identifier
    char name[RECORDING_NAME_MAX_LENGTH];  // Recording name
    uint32_t size;                         // Size in bytes
    uint16_t current_block;                // Current block being written to
    uint8_t current_page;                  // Current page in the block
    uint16_t event_count;                  // Number of events
    EventMark events[MAX_EVENTS];          // Array of event markings
} DiskMetadata;

static DiskMetadata metadata;


static DiskStatus load_metadata(void);
static DiskStatus save_metadata(void);
static DiskStatus erase_data_blocks(void);

static const struct device *nand_dev = DEVICE_DT_GET_ANY(dsy_w25m02gw);


DiskStatus disk_init(void) {
    if (!nand_dev) {
        printk("Failed to get NAND device\n");
        return DISK_ERR_DEVICE_NOT_READY;
    }

    // Initialize the device (if required)
    if (!device_is_ready(nand_dev)) {
        printk("NAND device not ready\n");
        return DISK_ERR_DEVICE_NOT_READY;
    }

    // Load existing metadata
    return load_metadata();
}


DiskStatus disk_start_recording(const char *name) {
    // Initialize metadata
    LOG_INF("Starting new recording: %s", name);
    memset(&metadata, 0, sizeof(DiskMetadata));
    LOG_INF("1");
    metadata.signature = METADATA_SIGNATURE;
    LOG_INF("2");
    strncpy(metadata.name, name, RECORDING_NAME_MAX_LENGTH - 1);
    LOG_INF("3");
    metadata.size = 0;
    LOG_INF("4");
    metadata.current_block = START_BLOCK;
    LOG_INF("5");
    metadata.current_page = START_PAGE;
    metadata.event_count = 0;
    LOG_INF("Initialized new metadata");
    // Save metadata
    DiskStatus status = save_metadata();
    if (status != DISK_OK) {
        return status;
    }
    LOG_INF("Saved metadata");

    return DISK_OK;
}




DiskStatus disk_write_data(const uint8_t *data, size_t length) {
    int err;

    // Check if we need to erase the current block
    static uint16_t last_erased_block = 0xFFFF; // Initialize to an invalid block number
    if (metadata.current_block != last_erased_block) {
        // Erase the block before writing to it
        err = w25m02gw_erase_block(nand_dev, metadata.current_block);
        if (err < 0) {
            printk("Failed to erase block %d\n", metadata.current_block);
            return DISK_ERR_UNKNOWN;
        }
        last_erased_block = metadata.current_block;
    }

    // Write data to the current page
    size_t write_size = MIN(length, PAGE_SIZE);
    err = w25m02gw_write(nand_dev, metadata.current_block, metadata.current_page, (uint8_t *)data, write_size);
    if (err < 0) {
        printk("Failed to write data to flash\n");
        return DISK_ERR_UNKNOWN;
    }

    // Update metadata
    metadata.size += write_size;

    // Move to the next page
    metadata.current_page++;
    if (metadata.current_page >= PAGES_PER_BLOCK) {
        // Move to the next block
        metadata.current_page = START_PAGE;
        metadata.current_block++;
        if (metadata.current_block >= TOTAL_BLOCKS) {
            printk("No more space on disk\n");
            return DISK_ERR_NO_SPACE;
        }
    }

    // Save metadata
    DiskStatus status = save_metadata();
    if (status != DISK_OK) {
        return status;
    }

    return DISK_OK;
}




DiskStatus disk_read_data(DataCallback callback) {
    static uint8_t buffer[PAGE_SIZE];
    uint16_t block = START_BLOCK;
    uint8_t page = START_PAGE;
    uint32_t total_read = 0;
    int err;

    while (total_read < metadata.size) {
        size_t read_size = MIN(PAGE_SIZE, metadata.size - total_read);

        err = w25m02gw_read(nand_dev, block, page, 0, buffer, read_size);
        if (err < 0) {
            printk("Failed to read data from flash\n");
            return DISK_ERR_UNKNOWN;
        }

        // Call the callback function
        callback(buffer, read_size);

        total_read += read_size;

        // Move to the next page
        page++;
        if (page >= PAGES_PER_BLOCK) {
            // Move to the next block
            page = START_PAGE;
            block++;
            if (block >= TOTAL_BLOCKS) {
                printk("Unexpected end of disk\n");
                return DISK_ERR_UNKNOWN;
            }
        }
    }

    return DISK_OK;
}



DiskStatus disk_add_event(uint32_t time, uint16_t event_number) {
    if (metadata.event_count >= MAX_EVENTS) {
        printk("Event list is full\n");
        return DISK_ERR_METADATA_INVALID;
    }
    metadata.events[metadata.event_count].time = time;
    metadata.events[metadata.event_count].event_number = event_number;
    metadata.event_count++;

    // Save metadata
    DiskStatus status = save_metadata();
    if (status != DISK_OK) {
        return status;
    }

    return DISK_OK;
}

// Might not need this function
// DiskStatus disk_delete_recording(void) {

//     return 0;
// }


DiskStatus disk_get_recording_size(uint32_t *size) {
    if (!size) {
        return DISK_ERR_UNKNOWN;
    }
    *size = metadata.size;
    return DISK_OK;
}

DiskStatus disk_get_recording_name(char *name_buffer, size_t buffer_length) {
    if (!name_buffer || buffer_length == 0) {
        return DISK_ERR_UNKNOWN;
    }
    strncpy(name_buffer, metadata.name, buffer_length - 1);
    name_buffer[buffer_length - 1] = '\0'; // Ensure null-termination
    return DISK_OK;
}


DiskStatus disk_get_events(EventMark *events_buffer, uint16_t *event_count) {
    if (!events_buffer || !event_count) {
        return DISK_ERR_UNKNOWN;
    }
    if (*event_count < metadata.event_count) {
        // Not enough space in events_buffer
        return DISK_ERR_UNKNOWN;
    }
    memcpy(events_buffer, metadata.events, metadata.event_count * sizeof(EventMark));
    *event_count = metadata.event_count;
    return DISK_OK;
}


static DiskStatus load_metadata(void) {
    int err;
    uint8_t buffer[sizeof(DiskMetadata)];

    // Read the metadata from METADATA_BLOCK and METADATA_PAGE
    err = w25m02gw_read(nand_dev, METADATA_BLOCK, METADATA_PAGE, 0, buffer, sizeof(DiskMetadata));
    if (err < 0) {
        printk("Failed to read metadata\n");
        return DISK_ERR_UNKNOWN;
    }

    // Copy data into metadata structure
    memcpy(&metadata, buffer, sizeof(DiskMetadata));

    // Check if signature matches
    if (metadata.signature != METADATA_SIGNATURE) {
        // No valid metadata, initialize metadata
        memset(&metadata, 0, sizeof(DiskMetadata));
        metadata.signature = METADATA_SIGNATURE;
        LOG_INF("Initialized new metadata \n");
        return DISK_OK;
    }

    return DISK_OK;
}



static DiskStatus save_metadata(void) {
    int err;

    static uint8_t buffer[sizeof(DiskMetadata)];

    // Copy metadata into buffer
    memcpy(buffer, &metadata, sizeof(DiskMetadata));

    // Erase the metadata block before writing
    err = w25m02gw_erase_block(nand_dev, METADATA_BLOCK);
    if (err < 0) {
        printk("Failed to erase metadata block\n");
        return DISK_ERR_UNKNOWN;
    }


    // Write metadata to METADATA_BLOCK and METADATA_PAGE

    err = w25m02gw_write(nand_dev, METADATA_BLOCK, METADATA_PAGE, buffer, sizeof(DiskMetadata));
    if (err < 0) {
        printk("Failed to write metadata\n");
        return DISK_ERR_UNKNOWN;
    }


    return DISK_OK;
}



static DiskStatus erase_data_blocks(void) {
    int err;
    for (uint16_t block = START_BLOCK; block < TOTAL_BLOCKS; block++) {
        err = w25m02gw_erase_block(nand_dev, block);
        if (err < 0) {
            printk("Failed to erase block %d\n", block);
            return DISK_ERR_UNKNOWN;
        }
    }
    return DISK_OK;
}

