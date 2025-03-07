#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// crc routines implemented by LLM
uint16_t crc_init()
{
    uint16_t crc = 0x6363; // ISO 14443-A initial value
    return crc;
}
uint16_t crc_update(uint16_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t j = 0; j < 8; j++)
    {
        if (crc & 0x0001)
        {
            crc = (crc >> 1) ^ 0x8408; // Polynomial 0x1021, bit-reversed
        }
        else
        {
            crc >>= 1;
        }
    }
    return crc;
}

uint16_t iso14443a_crc(uint8_t *data, size_t length)
{
    uint16_t crc = 0x6363; // ISO 14443-A initial value
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0x8408; // Polynomial 0x1021, bit-reversed
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc; // No final XOR for ISO 14443-3 CRC
}