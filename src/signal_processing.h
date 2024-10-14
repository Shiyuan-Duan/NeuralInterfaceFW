#ifndef SIGNAL_PROCESSING_H_
#define SIGNAL_PROCESSING_H_

#include <stdint.h>
#include <string.h>
// Function prototypes
void calculate_pulse_rate(const int32_t *samples, uint32_t sample_rate, float *hr);
void accumulate_samples(const int32_t *new_samples, size_t size);

#endif /* SIGNAL_PROCESSING_H_ */
