/**
 * @file CircularBuffer.c
 * @brief Implementation of a generic circular buffer.
 */

#include "CircularBuffer.h"
#include <stdlib.h>
#include <string.h>

/**
 * @brief Create a circular buffer.
 *
 * @param buffer Pointer to buffer structure.
 */
void CB_Init(CircularBuffer *buffer) {
    buffer->data = malloc(buffer->size * buffer->item_size);
    buffer->head = 0;
    buffer->tail = 0;
}

/**
 * @brief Free the memory allocated for the circular buffer.
 *
 * @param buffer Pointer to the buffer.
 */
void CB_Free(CircularBuffer *buffer) {
    free(buffer->data);
    free(buffer);
}

/**
 * @brief Push an item to the circular buffer.
 *
 * @param buffer Pointer to the buffer.
 * @param item Pointer to the item to add.
 */
void CB_Push(CircularBuffer *buffer, const void *item) {
    memcpy((char*)buffer->data + (buffer->tail * buffer->item_size), item, buffer->item_size);
    buffer->tail = (buffer->tail + 1) % buffer->size;
    if(buffer->tail == buffer->head) {
        buffer->head = (buffer->head + 1) % buffer->size;
    }
}

/**
 * @brief Pop an item from the circular buffer.
 *
 * @param buffer Pointer to the buffer.
 * @param item Pointer to store the popped item.
 * @return int 1 if successful, 0 if buffer is empty.
 */
uint8_t CB_Pop(CircularBuffer *buffer, void *item) {
    if (buffer->head == buffer->tail) {
        return 0; // Buffer is empty
    }
    memcpy(item, (char*)buffer->data + (buffer->head * buffer->item_size), buffer->item_size);
    buffer->head = (buffer->head + 1) % buffer->size;
    return 1;
}

/**
 * @brief Calculate the maximum difference between consecutive elements.
 *
 * @param buffer Pointer to the buffer.
 * @param compare Function pointer for comparison logic.
 * @return uint32_t Maximum difference found.
 */
uint32_t CB_Diff(CircularBuffer *buffer, uint32_t (*compare)(const void*, const void*)) {
    uint32_t maxDiff = 0;
    if(buffer->head != buffer->tail) {
        uint8_t i = buffer->head;
        uint8_t j = (buffer->head + 1) % buffer->size;
        do {
            uint32_t diff = compare((char*)buffer->data + (i * buffer->item_size),
                                    (char*)buffer->data + (j * buffer->item_size));
            if(diff > maxDiff) maxDiff = diff;
            i = (i + 1) % buffer->size;
            j = (j + 1) % buffer->size;
        } while(i != buffer->tail);
    }
    return maxDiff;
}

/**
 * @brief Calculate the average value of all elements in the buffer.
 *
 * @param buffer Pointer to the buffer.
 * @param sum Function pointer to sum two elements.
 * @param divide Function pointer to divide the sum by the count.
 * @return uint32_t Average value of the elements.
 */
uint32_t CB_Average(CircularBuffer *buffer, uint32_t (*sum)(const void*, const void*), uint32_t (*divide)(const void*, uint32_t)) {
    if(buffer->head == buffer->tail) return 0; // No elements

    uint32_t total = 0;
    uint32_t count = 0;
    uint8_t i = buffer->head;

    do {
        total = sum(&total, (char*)buffer->data + (i * buffer->item_size));
        count++;
        i = (i + 1) % buffer->size;
    } while(i != buffer->tail);

    return divide(&total, count);
}
