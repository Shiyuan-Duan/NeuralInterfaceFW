#include "signal_processing.h"
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>
#include "ble.h"

#define SAMPLE_BUFFER_SIZE 64
#define MIN_PEAK_DISTANCE 1  // Minimum distance between peaks in samples
#define THRESHOLD 0        // Threshold for peak detection (adjust based on signal characteristics)
#define SAMPLE_RATE 32
static int32_t sample_buffer[SAMPLE_BUFFER_SIZE];
static size_t sample_count = 0;
// Function to preprocess the signal (e.g., moving average filter)
void moving_mean(int* input, int* output, int length, int window_size) {
    if (window_size > length || window_size <= 0) {
        printf("Invalid window size\n");
        return;
    }

    // Temporary variable to store the sum of elements in the current window
    int window_sum = 0;
    int i;

    // Initialize the sum of the first window
    for (i = 0; i < window_size; i++) {
        window_sum += input[i];
    }

    // Calculate the mean for the first window
    output[0] = window_sum / window_size;

    // Calculate the mean for the rest of the windows where a full window can be formed
    for (i = 1; i <= length - window_size; i++) {
        // Update the window sum by removing the element that is sliding out and adding the new element
        window_sum = window_sum - input[i - 1] + input[i + window_size - 1];
        output[i] = window_sum / window_size;
    }

    // Pad the rest of the output array with the last computed mean
    for (int j = i; j < length; j++) {
        output[j] = output[i - 1];
    }
}

// Function to detect peaks in the PPG signal
void find_peaks(int* signal, int length, int* peaks, int* peak_count) {
    int count = 0;
    for (int i = 1; i < length - 1; i++) {
        if (signal[i] > signal[i-1] && signal[i] > signal[i+1]) {
            peaks[count++] = i;  // Store the index of the peak
        }
    }

    
    *peak_count = count;
}

double calculate_heart_rate(int* raw_signal, int length, int sampling_freq, int window_size) {
    int smoothed_signal[length];
    moving_mean(raw_signal, smoothed_signal, length, window_size);
    
    int peaks[length];  // Large enough to hold potential peaks
    int peak_count;
    find_peaks(smoothed_signal, length, peaks, &peak_count);

    if (peak_count < 2) {
        return 0.0;  // Not enough peaks to calculate heart rate
    }

    double average_interval = 0.0;
    for (int i = 1; i < peak_count; i++) {
        average_interval += (peaks[i] - peaks[i - 1]);
        // printk("interval: %d\n", peaks[i] - peaks[i - 1]);
        // printk("Average interval: %d.%03d\n", (int)average_interval, (int)((average_interval - (int)average_interval) * 1000));
    }
    average_interval /= (peak_count - 1);
    average_interval = average_interval / sampling_freq;  // Convert interval to seconds

    double heart_rate = 60.0 / average_interval;  // Calculate heart rate in beats per minute
    return heart_rate;
}


void accumulate_samples(const int32_t *new_samples, size_t size) {

    if (sample_count + size <= SAMPLE_BUFFER_SIZE) {
        memcpy(&sample_buffer[sample_count], new_samples, size * sizeof(int32_t));
        sample_count += size;
    }else {
        int hr;
        hr = (int)calculate_heart_rate(sample_buffer, sample_count, SAMPLE_RATE, 10);
        printk("hr: %d\n", hr);
        stream_sensor_data(HR, &hr, sizeof(float));
        
        sample_count = 0;
    }
}


