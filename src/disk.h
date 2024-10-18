#ifndef DISK_H
#define DISK_H

#include <stdint.h>
#include <stddef.h>

#define RECORDING_NAME_MAX_LENGTH 32
#define MAX_EVENTS 200

typedef struct {
    uint32_t time;          // Time of the event
    uint16_t event_number;  // Event identifier
} EventMark;

typedef void (*DataCallback)(const uint8_t *data, size_t length);

// Error codes
typedef enum {
    DISK_OK = 0,
    DISK_ERR_DEVICE_NOT_READY,
    DISK_ERR_NO_SPACE,
    DISK_ERR_INVALID_OPERATION,
    DISK_ERR_METADATA_INVALID,
    DISK_ERR_UNKNOWN,
} DiskStatus;

// Initialize the disk system
DiskStatus disk_init(void);

// Start a new recording (overwrites any existing recording)
DiskStatus disk_start_recording(const char *name);

// Write data to the recording
DiskStatus disk_write_data(const uint8_t *data, size_t length);

// Read data from the recording
DiskStatus disk_read_data(DataCallback callback);

// Add an event marking to the recording
DiskStatus disk_add_event(uint32_t time, uint16_t event_number);

// Delete the recording
DiskStatus disk_delete_recording(void);

// Get the size of the recording in bytes
DiskStatus disk_get_recording_size(uint32_t *size);

// Get the name of the recording
DiskStatus disk_get_recording_name(char *name_buffer, size_t buffer_length);

// Get the list of event markings
DiskStatus disk_get_events(EventMark *events_buffer, uint16_t *event_count);

#endif // DISK_H
