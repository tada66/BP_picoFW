#ifndef DS18B20_H
#define DS18B20_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

static void onewire_write_bit(bool bit);
static bool onewire_read_bit(void);
static void onewire_write_byte(uint8_t data);
static uint8_t onewire_read_byte(void);
static bool onewire_reset(void);
float ds18b20_read_temp(void);

#endif // DS18B20_H