/*
 * circular_buffer.c
 *
 *  Created on: Mar 26, 2025
 *      Author: ahmet
 */


#include "circular_buffer.h"

// Initialize buffer to empty state
void buffer_init(circular_buffer_t *buffer) {
    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
}

// Add value to buffer. Overwrites oldest data when full.
int buffer_add_value(circular_buffer_t *buffer, float value) {
	// Overwrite oldest data if buffer full
    if (buffer->count >= MAX_BUFFER_SIZE) {
    	// Move head forward and wrap around
        buffer->head = (buffer->head + 1) % MAX_BUFFER_SIZE;
    }

    buffer->data[buffer->tail] = value; // Store new value at tail position
    buffer->tail = (buffer->tail + 1) % MAX_BUFFER_SIZE;  // Update tail index with wrap-around

    // Update count unless already at maximum
    if (buffer->count < MAX_BUFFER_SIZE) {
        buffer->count++;
    }

    return 0;
}

// Retrieve all current buffer values
int buffer_get_all_values(circular_buffer_t *buffer, float *data, uint8_t *count) {
    if (buffer->count == 0) {
        return -1; // Buffer empty
    }

    *count = buffer->count;
    uint8_t index = buffer->head;

    for (uint8_t i = 0; i < buffer->count; i++) {
        data[i] = buffer->data[index];
        index = (index + 1) % MAX_BUFFER_SIZE; // Move to next index with wrap-around
    }

    return 0;
}

// Reset buffer to empty state
void buffer_clear(circular_buffer_t *buffer) {
    buffer_init(buffer);
}
