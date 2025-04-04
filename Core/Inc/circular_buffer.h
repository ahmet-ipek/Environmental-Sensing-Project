/*
 * circular_buffer.h
 *
 *  Created on: Mar 26, 2025
 *      Author: ahmet
 */

#ifndef INC_CIRCULAR_BUFFER_H_
#define INC_CIRCULAR_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_BUFFER_SIZE 10

typedef struct {
    float data[MAX_BUFFER_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} circular_buffer_t;

void buffer_init(circular_buffer_t *buffer);
int buffer_add_value(circular_buffer_t *buffer, float value);
int buffer_get_all_values(circular_buffer_t *buffer, float *data, uint8_t *count);
void buffer_clear(circular_buffer_t *buffer);


#endif /* INC_CIRCULAR_BUFFER_H_ */
