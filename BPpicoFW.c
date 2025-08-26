#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200
#define UART_TX_PIN 1
#define UART_RX_PIN 2

// Stepper driver pins
#define Y_STEP_PIN 16
#define Y_DIR_PIN 17
#define X_STEP_PIN 18
#define X_DIR_PIN 19
#define X_DIR_PIN_INV 20
#define Z_STEP_PIN 21
#define Z_DIR_PIN 22

// Stepper driver enable
#define EN_SENSE_PIN 12
#define EN_PIN 13   // a4988 drivers (and compatibles) use LOW to enable

#define TEMP_SENSE_PIN 15
#define FAN_PWM_PIN 14
#define ONBOARD_LED_PIN 25

#define CMD_BUFFER_SIZE 128
char cmd_buffer[CMD_BUFFER_SIZE];
int cmd_buffer_index = 0;
enum Commands {
    CMD_OK = 0x01,
    CMD_PAUSE = 0x02,
    CMD_RESUME = 0x03,
    CMD_STOP = 0x04
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
            if (cmd_buffer_index == data_length + 4) { // DATA + START + CMD + LEN + CHK
                // Full command received
                // Verify checksum
                char checksum = 0;
                for (int i = 0; i < cmd_buffer_index - 1; i++) {
                    checksum ^= cmd_buffer[i];
                }
                if (checksum != cmd_buffer[cmd_buffer_index - 1]) {
                    fprintf(stderr, "ERROR: Checksum mismatch!\n");
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
                // Process fast commands here, movements will be handled in main loop to avoid blocking
                switch (cmd_buffer[1]) {
                    case CMD_OK:
                        printf("Received CMD_OK\n");
                        break;
                    case CMD_PAUSE:
                        printf("Received CMD_PAUSE\n");
                        is_paused = true;
                        break;
                    case CMD_RESUME:
                        printf("Received CMD_RESUME\n");
                        is_paused = false;
                        break;
                    case CMD_STOP:
                        printf("Received CMD_STOP\n");
                        gpio_put(EN_PIN, 1); // Disable steppers
                        break;
                    default:
                        fprintf(stderr, "ERROR: Unknown command!\n");
                        break;
                }
                cmd_buffer_index = 0; // Reset buffer index for next command
            }
        }

    }
}

int main()
{
    stdio_init_all();

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
    gpio_init(TEMP_SENSE_PIN); gpio_set_dir(TEMP_SENSE_PIN, GPIO_IN);
    gpio_init(FAN_PWM_PIN); gpio_set_dir(FAN_PWM_PIN, GPIO_OUT);
    gpio_init(ONBOARD_LED_PIN); gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);

    // UART setup
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    gpio_put(ONBOARD_LED_PIN, 1); // Turn on onboard LED to indicate ready
    gpio_put(EN_PIN, 0);

    while (!is_paused) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
