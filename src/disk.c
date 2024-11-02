#include "disk.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include "w25m02gw.h"

// Device instance
static const struct device *nand_dev = NULL;

// Flash memory specifications
#define PAGE_SIZE 2048               // Size of a page in bytes
#define PAGES_PER_BLOCK 64           // Number of pages per block
#define TOTAL_BLOCKS 1024            // Total number of blocks
#define TOTAL_SIZE (PAGE_SIZE * PAGES_PER_BLOCK * TOTAL_BLOCKS)

// Block and page constraints
#define METADATA_BLOCK 0             // Block used for metadata
#define METADATA_PAGE 13              // Page used for metadata
#define START_BLOCK 1                // Start block for recording data
#define START_PAGE 13                // Skip first 13 pages in each block
#define USABLE_PAGES_PER_BLOCK (PAGES_PER_BLOCK - START_PAGE)
#define CHUNK_SIZE 128 // Define the chunk size to 256 bytes

// Utility macros
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// Unique identifier for metadata validation
#define METADATA_SIGNATURE 0xDEADBEEF

// Metadata structure
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

// Internal functions
static DiskStatus load_metadata(void);
static DiskStatus save_metadata(void);
static DiskStatus erase_data_blocks(void);

// Initialize the disk system
DiskStatus disk_init(void) {
    // nand_dev = DEVICE_DT_GET_ANY(dsy_w25m02gw);
    // if (!device_is_ready(nand_dev)) {
    //     printk("NAND device not ready\n");
    //     return DISK_ERR_DEVICE_NOT_READY;
    // }

    // // Load metadata from flash
    // DiskStatus status = load_metadata();
    // if (status != DISK_OK) {
    //     // If metadata is invalid, initialize it
    //     memset(&metadata, 0, sizeof(DiskMetadata));
    //     metadata.signature = METADATA_SIGNATURE;
    //     strcpy(metadata.name, "NI_recording");
    //     metadata.current_block = START_BLOCK;
    //     metadata.current_page = START_PAGE;
    //     save_metadata();
    // }
    return DISK_OK;
}

// Start a new recording
DiskStatus disk_start_recording(const char *name) {
    // // Erase existing data blocks
    // DiskStatus status = erase_data_blocks();
    // if (status != DISK_OK) {
    //     return status;
    // }

    // // Initialize metadata
    // memset(&metadata, 0, sizeof(DiskMetadata));
    // metadata.signature = METADATA_SIGNATURE;
    // if (name && strlen(name) < RECORDING_NAME_MAX_LENGTH) {
    //     strcpy(metadata.name, name);
    // } else {
    //     strcpy(metadata.name, "NI_recording");
    // }
    // metadata.current_block = START_BLOCK;
    // metadata.current_page = START_PAGE;

    // Save metadata to flash
    return save_metadata();

}

// Write data to the recording
DiskStatus disk_write_data(const uint8_t *data, size_t length) {
    // if (!data || length == 0) {
    //     return DISK_ERR_INVALID_OPERATION;
    // }

    // size_t bytes_written = 0;
    // uint8_t buffer[PAGE_SIZE] = {0}; // Buffer for writing a full page
    // while (bytes_written < length) {
    //     size_t to_write = MIN(length - bytes_written, PAGE_SIZE); // Write up to a page size (2048 bytes)
    //     memcpy(buffer, data + bytes_written, to_write);

    //     // Write data in chunks of 256 bytes
    //     size_t chunk_offset = 0;
    //     while (chunk_offset < to_write) {
    //         size_t chunk_size = MIN(to_write - chunk_offset, CHUNK_SIZE); // Write up to 256 bytes at a time

    //         // Write a chunk to flash
    //         printk("Writing to block %d, page %d, offset %d, size %d\n", metadata.current_block, metadata.current_page, chunk_offset, chunk_size);
    //         w25m02gw_write(nand_dev, metadata.current_block, metadata.current_page, chunk_offset, buffer + chunk_offset, chunk_size);
    //         printk("Data written: %x\n", buffer[0]);
    //         chunk_offset += chunk_size;
    //     }

    //     // Update metadata
    //     metadata.size += to_write;
    //     bytes_written += to_write;
    //     metadata.current_page++;

    //     if (metadata.current_page >= PAGES_PER_BLOCK) {
    //         // Move to the next block
    //         metadata.current_block++;
    //         if (metadata.current_block >= TOTAL_BLOCKS) {
    //             return DISK_ERR_NO_SPACE;
    //         }
    //         metadata.current_page = START_PAGE; // Skip the first few pages in the block
    //         w25m02gw_erase(nand_dev, metadata.current_block); // Erase the new block
    //     }
    // }

    // Save updated metadata
    return save_metadata();
}
// Read data from the recording


DiskStatus disk_read_data(DataCallback callback) {
    // if (!callback) {
    //     return DISK_ERR_INVALID_OPERATION;
    // }

    // uint32_t bytes_remaining = metadata.size;
    // uint16_t block_addr = START_BLOCK;
    // uint8_t page_addr = START_PAGE;

    // while (bytes_remaining > 0) {
    //     uint32_t page_bytes_remaining = MIN(bytes_remaining, PAGE_SIZE);
    //     uint32_t page_offset = 0;

    //     // Read data in chunks within the current page
    //     while (page_bytes_remaining > 0) {
    //         uint8_t buffer[CHUNK_SIZE] = {0}; // Allocate buffer on the stack for each chunk
    //         size_t chunk_size = MIN(page_bytes_remaining, CHUNK_SIZE);
    //         printk("Reading from block %d, page %d, offset %d, size %d\n", block_addr, page_addr, page_offset, chunk_size);
    //         // Read a chunk from flash
    //         w25m02gw_read(nand_dev, block_addr, page_addr, page_offset, buffer, chunk_size);
    //         printk("Data read: %x\n", buffer[0]);

    //         // Call the callback with the chunk
    //         callback(buffer, chunk_size);

    //         bytes_remaining -= chunk_size;
    //         page_bytes_remaining -= chunk_size;
    //         page_offset += chunk_size;

    //         // If no more bytes remaining, break out
    //         if (bytes_remaining == 0) {
    //             break;
    //         }
    //     }

    //     page_addr++;
    //     if (page_addr >= PAGES_PER_BLOCK) {
    //         block_addr++;
    //         if (block_addr >= TOTAL_BLOCKS) {
    //             return DISK_ERR_UNKNOWN;
    //         }
    //         page_addr = START_PAGE;
    //     }
    // }
    return DISK_OK;
}


// Add an event marking to the recording
DiskStatus disk_add_event(uint32_t time, uint16_t event_number) {
    // if (metadata.event_count >= MAX_EVENTS) {
    //     return DISK_ERR_NO_SPACE;
    // }

    // metadata.events[metadata.event_count].time = time;
    // metadata.events[metadata.event_count].event_number = event_number;
    // metadata.event_count++;

    // Save updated metadata
    return save_metadata();
}

// Delete the recording
DiskStatus disk_delete_recording(void) {
    // Erase data blocks
    // DiskStatus status = erase_data_blocks();
    // if (status != DISK_OK) {
    //     return status;
    // }

    // // Reset metadata
    // memset(&metadata, 0, sizeof(DiskMetadata));
    // metadata.signature = METADATA_SIGNATURE;
    // strcpy(metadata.name, "NI_recording");
    // metadata.current_block = START_BLOCK;
    // metadata.current_page = START_PAGE;

    // Save metadata
    return save_metadata();
}

// Get the size of the recording
DiskStatus disk_get_recording_size(uint32_t *size) {
    // if (!size) {
    //     return DISK_ERR_INVALID_OPERATION;
    // }
    // *size = metadata.size;
    return DISK_OK;
}

// Get the name of the recording
DiskStatus disk_get_recording_name(char *name_buffer, size_t buffer_length) {
    // if (!name_buffer || buffer_length < RECORDING_NAME_MAX_LENGTH) {
    //     return DISK_ERR_INVALID_OPERATION;
    // }
    // strncpy(name_buffer, metadata.name, RECORDING_NAME_MAX_LENGTH);
    return DISK_OK;
}

// Get the list of event markings
DiskStatus disk_get_events(EventMark *events_buffer, uint16_t *event_count) {
    // if (!events_buffer || !event_count) {
    //     return DISK_ERR_INVALID_OPERATION;
    // }
    // memcpy(events_buffer, metadata.events, metadata.event_count * sizeof(EventMark));
    // *event_count = metadata.event_count;
    return DISK_OK;
}

// Internal function to load metadata from flash
static DiskStatus load_metadata(void) {
    // uint8_t buffer[PAGE_SIZE] = {0};
    // w25m02gw_read(nand_dev, METADATA_BLOCK, METADATA_PAGE, 0, buffer, sizeof(DiskMetadata));
    // memcpy(&metadata, buffer, sizeof(DiskMetadata));

    // // Validate the signature
    // if (metadata.signature != METADATA_SIGNATURE) {
    //     return DISK_ERR_METADATA_INVALID;
    // }

    // // Basic validation of current block and page
    // if (metadata.current_block < START_BLOCK || metadata.current_block >= TOTAL_BLOCKS) {
    //     return DISK_ERR_METADATA_INVALID;
    // }
    // if (metadata.current_page < START_PAGE || metadata.current_page >= PAGES_PER_BLOCK) {
    //     return DISK_ERR_METADATA_INVALID;
    // }

    return DISK_OK;
}

// Internal function to save metadata to flash
static DiskStatus save_metadata(void) {
    // static uint8_t buffer[PAGE_SIZE] = {0};
    // memcpy(buffer, &metadata, sizeof(DiskMetadata));

    // // Erase metadata block
    // w25m02gw_erase(nand_dev, METADATA_BLOCK);

    // // Write metadata to flash in chunks of 256 bytes
    // size_t chunk_offset = 0;
    // while (chunk_offset < PAGE_SIZE) {
    //     size_t chunk_size = MIN(PAGE_SIZE - chunk_offset, CHUNK_SIZE);
    //     w25m02gw_write(nand_dev, METADATA_BLOCK, METADATA_PAGE, chunk_offset, buffer + chunk_offset, chunk_size);
    //     chunk_offset += chunk_size;
    // }

    return DISK_OK;
}

// Internal function to erase data blocks
static DiskStatus erase_data_blocks(void) {
    // for (uint16_t block = START_BLOCK; block < TOTAL_BLOCKS; block++) {
    //     w25m02gw_erase(nand_dev, block);
    // }
    return DISK_OK;
}
