#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define FIFO_SIZE 64

typedef struct
{
    uint8_t fifo_write_idx;
    uint8_t fifo_read_idx;
    uint8_t count;
    uint8_t fifo[FIFO_SIZE];
} FIFO;

void fifo_init(FIFO *f)
{
    f->fifo_write_idx = 0;
    f->fifo_read_idx = 0;
    f->count = 0;
}

void fifo_flush(FIFO *f)
{
    f->fifo_write_idx = 0;
    f->fifo_read_idx = 0;
    f->count = 0;
    memset(f->fifo, 0, FIFO_SIZE);
}

bool fifo_is_empty(FIFO *f)
{
    return f->count == 0;
}

bool fifo_is_full(FIFO *f)
{
    return f->count == FIFO_SIZE;
}

bool fifo_write(FIFO *f, uint8_t data)
{
    if (fifo_is_full(f))
    {
        return false; // Buffer is full
    }
    f->fifo[f->fifo_write_idx] = data;
    f->fifo_write_idx = (f->fifo_write_idx + 1) % FIFO_SIZE;
    f->count++;
    return true;
}

bool fifo_write_many(FIFO *f, const uint8_t *source, uint8_t count)
{
    if (f->count + count > FIFO_SIZE)
    {
        return false; // Not enough space
    }
    for (uint8_t i = 0; i < count; i++)
    {
        f->fifo[f->fifo_write_idx] = source[i];
        f->fifo_write_idx = (f->fifo_write_idx + 1) % FIFO_SIZE;
    }
    f->count += count;
    return true;
}

bool fifo_read(FIFO *f, uint8_t *data)
{
    if (fifo_is_empty(f))
    {
        return false; // Buffer is empty
    }
    *data = f->fifo[f->fifo_read_idx];
    f->fifo_read_idx = (f->fifo_read_idx + 1) % FIFO_SIZE;
    f->count--;
    return true;
}

bool fifo_peek(FIFO *f, uint8_t offset, uint8_t *data)
{
    if (fifo_is_empty(f))
    {
        return false; // Buffer is empty
    }
    uint8_t idx = (f->fifo_read_idx + offset) % FIFO_SIZE;
    *data = f->fifo[idx];
    return true;
}

void fifo_print(FIFO *f)
{
    for (uint8_t i = 0; i < f->count; i++)
    {
        uint8_t idx = (f->fifo_read_idx + i) % FIFO_SIZE;
        printf("%02X ", f->fifo[idx]);
    }
}
