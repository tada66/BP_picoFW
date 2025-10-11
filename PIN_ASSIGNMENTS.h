// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart0
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