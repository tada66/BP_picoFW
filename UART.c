#include "UART.h"

uint8_t calculate_crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0xFF; // Initial value
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
            crc &= 0xFF;  // Keep only the lower 8 bits
        }
    }
    
    return crc;
}

// Helper function to send responses with CRC8
void send_uart_response(uint8_t cmd_type, const uint8_t *data, size_t data_length) {
    // Command format: START(0xAA) + CMD_TYPE + LEN + DATA + CRC8
    uint8_t header[3] = {0xAA, cmd_type, data_length};
    
    // Create a temporary buffer for combined header + data for CRC calculation
    uint8_t temp_buffer[256];
    
    // Copy header to temp buffer
    memcpy(temp_buffer, header, 3);
    
    // Copy data to temp buffer (if any)
    if (data_length > 0 && data != NULL) {
        memcpy(temp_buffer + 3, data, data_length);
    }
    
    // Calculate CRC8 over the entire message (header + data)
    uint8_t crc = calculate_crc8(temp_buffer, 3 + data_length);
    
    // Send packet
    uart_write_blocking(UART_ID, header, 3);
    if (data_length > 0 && data != NULL) {
        uart_write_blocking(UART_ID, data, data_length);
    }
    uart_putc(UART_ID, crc);
    
    // Debug output to USB
    printf("Sent response: CMD=0x%02X, LEN=%d, CRC=0x%02X\n", cmd_type, data_length, crc);
}