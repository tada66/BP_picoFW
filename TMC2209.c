#include "TMC2209.h"

// Static variables
static bool tmc2209_uart_initialized = false;
static uint8_t current_microstepping = TMC2209_MSTEP_16;  // Global microstepping for all drivers

void tmc2209_init(void) {
    if (tmc2209_uart_initialized) return;
    
    // Initialize UART for TMC2209 communication (TX only)
    uart_init(TMC2209_UART, TMC2209_BAUD);
    gpio_set_function(TMC2209_TX_PIN, GPIO_FUNC_UART);
    
    // Configure UART parameters
    uart_set_hw_flow(TMC2209_UART, false, false);
    uart_set_format(TMC2209_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(TMC2209_UART, false);
    
    tmc2209_uart_initialized = true;
    
    sleep_ms(100);  // Allow drivers to initialize
    
    // Initialize all TMC2209 drivers using broadcast address
    DEBUG_PRINT("Initializing all TMC2209 drivers via broadcast...\n");
    
    // Set reasonable current for all drivers (adjust these values for your motors)
    tmc2209_set_current_all(20, 10);  // IRUN=20, IHOLD=10
    sleep_ms(10);
    
    // Set initial microstepping to 16x for all drivers
    tmc2209_set_microstepping_all(TMC2209_MSTEP_16);
    sleep_ms(10);
    
    DEBUG_PRINT("TMC2209 UART initialized - all drivers configured\n");
}

uint8_t tmc2209_calculate_crc(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t currentByte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (currentByte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            currentByte = currentByte >> 1;
        }
    }
    return crc;
}

bool tmc2209_write_register(uint8_t slave_addr, uint8_t reg_addr, uint32_t data) {
    if (!tmc2209_uart_initialized) return false;
    
    uint8_t packet[8];
    
    // Construct write packet
    packet[0] = 0x05;  // Sync byte
    packet[1] = slave_addr;  // Slave address (0x00 = broadcast to all)
    packet[2] = reg_addr | 0x80;  // Register address with write bit
    packet[3] = (data >> 24) & 0xFF;  // Data bytes (MSB first)
    packet[4] = (data >> 16) & 0xFF;
    packet[5] = (data >> 8) & 0xFF;
    packet[6] = data & 0xFF;
    packet[7] = tmc2209_calculate_crc(packet, 7);  // CRC
    
    // Send packet
    uart_write_blocking(TMC2209_UART, packet, 8);
    
    // Small delay to allow processing
    sleep_ms(2);
    
    return true;
}

void tmc2209_set_microstepping_all(uint8_t mstep) {
    if (!tmc2209_uart_initialized) return;
    
    // Read current CHOPCONF register value (using default/typical value)
    // Since we can't read from all drivers, we'll use a safe default configuration
    uint32_t chopconf = 0x10000053;  // Default CHOPCONF value for TMC2209
    
    // Clear microstepping bits (bits 27:24) and set new value
    chopconf &= ~(0xF << 24);
    chopconf |= (mstep & 0xF) << 24;
    
    // Write to all drivers using broadcast address
    tmc2209_write_register(TMC2209_BROADCAST_ADDR, TMC2209_REG_CHOPCONF, chopconf);
    
    current_microstepping = mstep;
    
    DEBUG_PRINT("TMC2209 All drivers: Set microstepping to %dx\n", 1 << mstep);
}

void tmc2209_set_current_all(uint8_t irun, uint8_t ihold) {
    if (!tmc2209_uart_initialized) return;
    
    uint32_t ihold_irun = (irun << 8) | ihold | (2 << 16);  // IHOLDDELAY = 2
    tmc2209_write_register(TMC2209_BROADCAST_ADDR, TMC2209_REG_IHOLD_IRUN, ihold_irun);
    
    DEBUG_PRINT("TMC2209 All drivers: Set current IRUN=%d, IHOLD=%d\n", irun, ihold);
}

uint8_t tmc2209_get_current_microstepping(void) {
    return current_microstepping;
}