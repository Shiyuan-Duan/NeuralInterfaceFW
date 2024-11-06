// File: src/stream.h

#ifndef STREAM_H
#define STREAM_H

#include <zephyr/kernel.h>

#define ADS1299_NUM_CHANNELS 8

extern struct k_fifo ble_stream_fifo;
extern struct k_mem_slab ble_stream_slab;

// Ensure that the structure is fully defined here
struct ble_stream_data {
    void *fifo_reserved;
    int32_t channel_data[ADS1299_NUM_CHANNELS];
};

#endif /* STREAM_H */
