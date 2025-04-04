/*
 * moving_median_filter.h
 *
 *  Created on: Mar 26, 2025
 *      Author: ahmet
 */

#ifndef INC_MOVING_MEDIAN_FILTER_H_
#define INC_MOVING_MEDIAN_FILTER_H_


#include <stdint.h>

#define MAX_WINDOW_SIZE 10

typedef struct {
    float window[MAX_WINDOW_SIZE];
    float sorted_window[MAX_WINDOW_SIZE];
    uint8_t current_index;
    uint8_t current_size;
    uint8_t window_size;
} median_filter_t;

typedef struct {
    float standard_deviation;
    float max_value;
    float min_value;
    float median_value;
} sensor_stats_t;

float filter_sensor_value(median_filter_t *filter, float raw_value);
void calculate_sensor_stats(float *data, uint8_t count, sensor_stats_t *stats);




#endif /* INC_MOVING_MEDIAN_FILTER_H_ */
