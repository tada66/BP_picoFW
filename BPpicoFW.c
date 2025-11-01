#include "BPpicoFW.h"
#include "hardware/timer.h"

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
    // This unsures that DEBUG_PRINT only use the USB for output and doesn't make the uart output garbage
    stdio_uart_init_full(uart1, 9600, -1, -1);
    
    // GPIO setup
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
    fan_set_speed(100);

    sleep_ms(5000); // Sleep for me to allow time to connect to USB serial console
    // Initialize stepper motor GPIOs and launch process in a separate core
    stepper_init();

    // UART setup
    uart_init_protocol();
    DEBUG_PRINT("Initialization complete!\n");

    gpio_put(ONBOARD_LED_PIN, 1); // Turn on onboard LED to indicate ready
    sleep_ms(4000);
    gpio_put(ONBOARD_LED_PIN, 0);

    int counter = 0;
    char uart_buffer[64];
    uint32_t current_time = time_us_32();

    while (1) {
        // send telemetry every 10 seconds
        if (time_us_32() - current_time >= 10000000) {
            float t = ds18b20_read_temp();
            uint8_t telemetry[9];
            memcpy(&telemetry[0], &t, sizeof(float));
            memcpy(&telemetry[4], &counter, sizeof(int));
            if(counter % 50 == 0)
                queue_response(CMD_STATUS, telemetry, 8);
            counter++;
            current_time = time_us_32();
        }
        uart_background_task();
    }
}
