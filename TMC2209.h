#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "PIN_ASSIGNMENTS.h"
#include "DEBUGPRINT.h"

// TMC2209 UART Configuration
#define TMC2209_UART uart1
#define TMC2209_BAUD 9600
#define TMC2209_TX_PIN 4  // Single GPIO connected to all TMC2209 UART RX pins

// TMC2209 Register addresses
#define TMC2209_REG_GCONF    0x00
#define TMC2209_REG_CHOPCONF 0x6C
#define TMC2209_REG_IHOLD_IRUN 0x10

// TMC2209 microstepping settings
#define TMC2209_MSTEP_1    0   // 1x microstepping (fastest)
#define TMC2209_MSTEP_2    1   // 2x microstepping
#define TMC2209_MSTEP_4    2   // 4x microstepping
#define TMC2209_MSTEP_8    3   // 8x microstepping
#define TMC2209_MSTEP_16   4   // 16x microstepping (default)
#define TMC2209_MSTEP_32   5   // 32x microstepping
#define TMC2209_MSTEP_64   6   // 64x microstepping
#define TMC2209_MSTEP_128  7   // 128x microstepping
#define TMC2209_MSTEP_256  8   // 256x microstepping (smoothest)

// TMC2209 slave addresses (broadcast address 0x00 affects all drivers)
#define TMC2209_BROADCAST_ADDR 0x00  // Affects all drivers

// Function declarations
void tmc2209_init(void);
void tmc2209_set_microstepping_all(uint8_t mstep);
void tmc2209_set_current_all(uint8_t irun, uint8_t ihold);
bool tmc2209_write_register(uint8_t slave_addr, uint8_t reg_addr, uint32_t data);
uint8_t tmc2209_calculate_crc(uint8_t *data, uint8_t len);
uint8_t tmc2209_get_current_microstepping(void);

#endif // TMC2209_H