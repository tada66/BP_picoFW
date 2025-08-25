#include <stdio.h>
#include <string.h>
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
#define EN_PIN 13

#define TEMP_SENSE_PIN 15
#define FAN_PWM_PIN 14

#define CMD_BUFFER_SIZE 128
char cmd_buffer[CMD_BUFFER_SIZE];
int cmd_buffer_index = 0;

void on_uart_rx()
{
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);
        if (c == '\n' || c == '\r') {
            cmd_buffer[cmd_buffer_index] = '\0';
            printf("Received command: %s\n", cmd_buffer);
            cmd_buffer_index = 0;
        } 
        else {
            if (cmd_buffer_index < CMD_BUFFER_SIZE - 1) {
                cmd_buffer[cmd_buffer_index++] = c;
            }
            else {
                fprintf(stderr, "ERROR: Command buffer overflow!\n");
                cmd_buffer_index = 0;
            }
        }
    }
}

int main()
{
    stdio_init_all();

    // UART setup
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

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

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
