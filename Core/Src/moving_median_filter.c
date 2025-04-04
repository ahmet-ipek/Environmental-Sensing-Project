/*
 * moving_median_filter.c
 *
 *  Created on: Mar 26, 2025
 *      Author: ahmet
 */


#include "moving_median_filter.h"
#include <math.h>
#include <string.h>

float filter_sensor_value(median_filter_t *filter, float raw_value) {
    if (filter->window_size > MAX_WINDOW_SIZE) {
        filter->window_size = MAX_WINDOW_SIZE;
    }

    filter->window[filter->current_index] = raw_value; // Add new value to circular window

    memcpy(filter->sorted_window, filter->window, sizeof(float) * filter->window_size); // Create working copy for sorting

    // Bubble sort for sorted_window
    for (uint8_t i = 0; i < filter->window_size - 1; i++) {
        for (uint8_t j = 0; j < filter->window_size - i - 1; j++) {
            if (filter->sorted_window[j] > filter->sorted_window[j + 1]) {
                float temp = filter->sorted_window[j];
                filter->sorted_window[j] = filter->sorted_window[j + 1];
                filter->sorted_window[j + 1] = temp;
            }
        }
    }

    // Calculate Median
    float median;
    if (filter->window_size % 2 == 0) {
        median = (filter->sorted_window[filter->window_size/2 - 1] +
                  filter->sorted_window[filter->window_size/2]) / 2.0f;
    } else {
        median = filter->sorted_window[filter->window_size/2];
    }

    // Update window index
    filter->current_index = (filter->current_index + 1) % filter->window_size;
    if (filter->current_size < filter->window_size) {
        filter->current_size++;
    }

    return median;
}

void calculate_sensor_stats(float *data, uint8_t count, sensor_stats_t *stats) {
    if (count == 0) {
        memset(stats, 0, sizeof(sensor_stats_t));
        return;
    }

    float max_val = data[0], min_val = data[0], sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        if (data[i] > max_val) max_val = data[i];
        if (data[i] < min_val) min_val = data[i];
        sum += data[i];
    }

    float mean = sum / count;
    float variance_sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        variance_sum += powf(data[i] - mean, 2);
    }

    stats->standard_deviation = sqrtf(variance_sum / count);
    stats->max_value = max_val;
    stats->min_value = min_val;

    // Sort data for median calculation
    float sorted_data[MAX_WINDOW_SIZE];
    memcpy(sorted_data, data, sizeof(float) * count);

    for (uint8_t i = 0; i < count - 1; i++) {
        for (uint8_t j = 0; j < count - i - 1; j++) {
            if (sorted_data[j] > sorted_data[j + 1]) {
                float temp = sorted_data[j];
                sorted_data[j] = sorted_data[j + 1];
                sorted_data[j + 1] = temp;
            }
        }
    }

    if (count % 2 == 0) {
        stats->median_value = (sorted_data[count/2 - 1] + sorted_data[count/2]) / 2.0f;
    } else {
        stats->median_value = sorted_data[count/2];
    }
}
