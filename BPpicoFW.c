#include "BPpicoFW.h"

bool is_paused = false;
// Track current fan speed (%) for telemetry
static uint8_t g_fan_speed_percent = 0;

void fan_set_speed(float duty_percent) {
    if (duty_percent < 0) duty_percent = 0;
    if (duty_percent > 100) duty_percent = 100;

    // Save for telemetry
    g_fan_speed_percent = (uint8_t)(duty_percent + 0.5f);

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
    // This ensures that DEBUG_PRINT only use the USB for output and doesn't make the uart output garbage
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
    fan_set_speed(100); // the fans are pretty slow, just let them be at full speed

    sleep_ms(5000); // The stepper drivers do like to power up with some delay
                    // haven't had any issues with this removed, however, some people on the internet reported issues 
                    // and since this happens only once at startup, well leave it here for safety

    // Initialize stepper motor GPIOs and launch process in a separate core
    stepper_init();

    // UART setup
    uart_init_protocol();
    DEBUG_PRINT("Initialization complete!\n");

    gpio_put(ONBOARD_LED_PIN, 1); // Turn on onboard LED to indicate ready

    uint32_t last_telemetry_time = time_us_32();
    const uint32_t TELEMETRY_INTERVAL_US = 2000000; // 2 seconds

    while (1) {
        uart_background_task();
        uint32_t current_time = time_us_32();
        
        //time will wrap around every 71 minutes or so, handle that
        uint32_t time_diff;
        if (current_time >= last_telemetry_time) {
            time_diff = current_time - last_telemetry_time;
        } else {
            time_diff = (UINT32_MAX - last_telemetry_time) + current_time + 1;
        }

        // Send telemetry every 2 seconds
        if (time_diff >= TELEMETRY_INTERVAL_US) {
            float t = ds18b20_read_temp();

            // Telemetry: temp (float) + X,Y,Z (int32) + enabled(u8) + paused(u8) + fan_pct(u8) = 19 bytes
            uint8_t telemetry[19];
            int32_t x = stepper_get_position_arcsec(AXIS_X);
            int32_t y = stepper_get_position_arcsec(AXIS_Y);
            int32_t z = stepper_get_position_arcsec(AXIS_Z);
            uint8_t enabled = stepper_is_enabled() ? 1 : 0;
            uint8_t paused  = stepper_is_paused() ? 1 : 0;

            memcpy(&telemetry[0],  &t, sizeof(float));
            memcpy(&telemetry[4],  &x, sizeof(int32_t));
            memcpy(&telemetry[8],  &y, sizeof(int32_t));
            memcpy(&telemetry[12], &z, sizeof(int32_t));
            telemetry[16] = enabled;
            telemetry[17] = paused;
            telemetry[18] = g_fan_speed_percent;

            queue_response(CMD_STATUS, telemetry, 19);
            DEBUG_PRINT("Telemetry: T=%.2fC X=%d Y=%d Z=%d en=%d pa=%d fan=%u%%\n",
                        t, x, y, z, enabled, paused, g_fan_speed_percent);

            last_telemetry_time = current_time;
        }
    }
}
