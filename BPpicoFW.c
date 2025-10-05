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
bool is_paused = false;


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
    // Disable stdio on UART
    // This unsures that printf only use the USB for output and doesn't make the uart output garbage
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
        
        // Or send it in binary format with CRC8
        // Format: START + CMD_POSITION + LEN(8) + float temp + int counter + CRC8
        uint8_t telemetry[9];
        memcpy(&telemetry[0], &t, sizeof(float));
        memcpy(&telemetry[4], &counter, sizeof(int));
        send_uart_command(CMD_STATUS, telemetry, 8);
        uart_background_task();
        //printf("Temperature: %.2f °C\n", t);
        counter++;
        sleep_ms(20000);
    }
}
