#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "DS18B20.h"
#include "PIN_ASSIGNMENTS.h"
#include "UART.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

#define CMD_BUFFER_SIZE 128
char cmd_buffer[CMD_BUFFER_SIZE];
int cmd_buffer_index = 0;
enum Commands {
    CMD_PING = 0x01,
    CMD_READY = 0x02,
    CMD_MOVE_ABS = 0x10,
    CMD_TRACK = 0x11,
    CMD_PAUSE = 0x12,
    CMD_RESUME = 0x13,
    CMD_GETPOS = 0x14,
    CMD_POSITION = 0x15,
    CMD_STATUS = 0x20,
    CMD_ESTOPTRIG = 0x21
};
bool is_paused = false;

void on_uart_rx()
{
    // Process incoming data
    // First byte is START byte 0xAA
    // Second byte is command type (defined in enum Commands)
    // Third byte is data length
    // DATA bytes follow
    // Last byte is checksum
    while (uart_is_readable(UART_ID)) {
        uint8_t c = uart_getc(UART_ID);
        if (cmd_buffer_index >= CMD_BUFFER_SIZE - 1) {
            fprintf(stderr, "ERROR: Command buffer overflow!\n");
            cmd_buffer_index = 0;
            continue;
        }
        if (cmd_buffer_index == 0 && c != (char)0xAA) {
            continue;
        }
        cmd_buffer[cmd_buffer_index++] = c;
        if (cmd_buffer_index >= 3) {
            int data_length = (unsigned char)cmd_buffer[2];
            if (cmd_buffer_index == data_length + 4) { // DATA + START + CMD + LEN + CHKSUM (crc8)
                // Full command received
                // Verify checksum
                uint8_t received_crc = (uint8_t)cmd_buffer[cmd_buffer_index - 1];
                uint8_t calculated_crc = calculate_crc8((uint8_t *)cmd_buffer, cmd_buffer_index - 1);
                if (received_crc != calculated_crc) {
                    fprintf(stderr, "ERROR: CRC8 mismatch! Received: 0x%02X, Calculated: 0x%02X\n", 
                           received_crc, calculated_crc);
                    // Try to resync by looking for next 0xAA
                    int i = 1;
                    while (i < cmd_buffer_index && cmd_buffer[i] != 0xAA) i++;
                    if (i < cmd_buffer_index) {
                        memmove(cmd_buffer, &cmd_buffer[i], cmd_buffer_index - i);
                        cmd_buffer_index -= i;
                    } else {
                        cmd_buffer_index = 0;
                    }
                    continue;
                }

                printf("Command received: 0x%02X, Length: %d\n", cmd_buffer[1], data_length);
                // Process fast commands here, movements will be handled in main loop to avoid blocking
                switch (cmd_buffer[1]) {
                    case CMD_PING:
                        send_uart_response(CMD_PING, NULL, 0);
                        break;
                }
                cmd_buffer_index = 0; // Reset buffer index for next command
            }
        }

    }
}

void fan_set_speed(float duty_percent) {
    if (duty_percent < 0) duty_percent = 0;
    if (duty_percent > 100) duty_percent = 100;

    uint slice = pwm_gpio_to_slice_num(FAN_PWM_PIN);
    uint chan  = pwm_gpio_to_channel(FAN_PWM_PIN);
    uint16_t level = (uint16_t)(duty_percent / 100.0 * 65535);
    pwm_set_chan_level(slice, chan, level);
}

int main()
{
    stdio_init_all();
    
    // Configure stdio to use USB only, not UART
    stdio_usb_init();
    // Disable stdio on UART - this ensures printf doesn't go to UART
    stdio_uart_init_full(uart1, 115200, -1, -1);
    
    // GPIO setup
    gpio_init(Y_STEP_PIN); gpio_set_dir(Y_STEP_PIN, GPIO_OUT);
    gpio_init(Y_DIR_PIN); gpio_set_dir(Y_DIR_PIN, GPIO_OUT);
    gpio_init(X_STEP_PIN); gpio_set_dir(X_STEP_PIN, GPIO_OUT);
    gpio_init(X_DIR_PIN); gpio_set_dir(X_DIR_PIN, GPIO_OUT);
    gpio_init(X_DIR_PIN_INV); gpio_set_dir(X_DIR_PIN_INV, GPIO_OUT);
    gpio_init(Z_STEP_PIN); gpio_set_dir(Z_STEP_PIN, GPIO_OUT);
    gpio_init(Z_DIR_PIN); gpio_set_dir(Z_DIR_PIN, GPIO_OUT);
    gpio_init(EN_SENSE_PIN); gpio_set_dir(EN_SENSE_PIN, GPIO_IN);
    gpio_init(EN_PIN); gpio_set_dir(EN_PIN, GPIO_OUT);
    gpio_init(TEMP_SENSE_PIN); gpio_set_dir(TEMP_SENSE_PIN, GPIO_OUT);
    gpio_put(TEMP_SENSE_PIN, 1);
    gpio_init(FAN_PWM_PIN); gpio_set_dir(FAN_PWM_PIN, GPIO_OUT);
    gpio_init(ONBOARD_LED_PIN); gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);

    // Setup PWM on FAN pin
    gpio_set_function(FAN_PWM_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(FAN_PWM_PIN);
    pwm_set_wrap(slice, 65535); // 16-bit resolution
    pwm_set_clkdiv(slice, 76.3f); // divisor for ~25 kHz, lower frequencies cause the fan to emit audible high pitched noise
    pwm_set_enabled(slice, true);

    // UART setup
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);
    printf("Initialization complete!\n");

    gpio_put(ONBOARD_LED_PIN, 1); // Turn on onboard LED to indicate ready
    gpio_put(EN_PIN, 0);

    int counter = 0;
    char uart_buffer[64];

    while (1) {
        float t = ds18b20_read_temp();
        // Send telemetry in text format for easy debugging
        sprintf(uart_buffer, "%d,%.2f\r\n", counter, t);
        uart_puts(UART_ID, uart_buffer);
        
        // Or send it in binary format with CRC8
        // Format: START + CMD_POSITION + LEN(8) + float temp + int counter + CRC8
        uint8_t telemetry[9];
        memcpy(&telemetry[0], &t, sizeof(float));
        memcpy(&telemetry[4], &counter, sizeof(int));
        send_uart_response(CMD_STATUS, telemetry, 8);
        
        printf("Temperature: %.2f °C\n", t);
        counter++;
        sleep_ms(1000);
    }
}
