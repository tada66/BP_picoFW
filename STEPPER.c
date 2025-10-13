#include "STEPPER.h"
#include "hardware/timer.h"

// Global state variables
bool stepper_enabled = false;
volatile bool stepper_paused = false;

// Positional tracking
volatile int32_t x_position_steps = 0;
volatile int32_t y_position_steps = 0;
volatile int32_t z_position_steps = 0;

// Command structures
volatile stepper_command_t current_command = {STATIC_MOVE, false, 0, 0};
volatile tracking_state_t tracking_state = {false, {0.0f, 0.0f, 0.0f}, {0, 0, 0}};

// Helper functions for step/arcsecond conversions
int32_t arcseconds_to_steps(int32_t arcseconds, float gear_ratio) {
    // 1296000 = 360° * 60 * 60 (arcseconds in a full circle)
    float steps_per_arcsecond = ((float)STEPS_PER_REV * MICROSTEPPING * gear_ratio) / 1296000.0f;
    return (int32_t)(arcseconds * steps_per_arcsecond);
}

int32_t steps_to_arcseconds(int32_t steps, float gear_ratio) {
    float arcseconds_per_step = 1296000.0f / ((float)STEPS_PER_REV * MICROSTEPPING * gear_ratio);
    return (int32_t)(steps * arcseconds_per_step);
}

static inline uint get_step_pin(uint8_t axis) {
    switch (axis) {
        case AXIS_X: return X_STEP_PIN;
        case AXIS_Y: return Y_STEP_PIN;
        case AXIS_Z: return Z_STEP_PIN;
        default: return 0;
    }
}

static inline uint get_dir_pin(uint8_t axis) {
    switch (axis) {
        case AXIS_Z: return Z_DIR_PIN;
        case AXIS_Y: return Y_DIR_PIN;
        case AXIS_X: return X_DIR_PIN;
        default: return 0;
    }
}

static inline float get_gear_ratio(uint8_t axis) {
    switch (axis) {
        case AXIS_X: return X_STEPPER_GEAR_RATIO;
        case AXIS_Y: return Y_STEPPER_GEAR_RATIO;
        case AXIS_Z: return Z_STEPPER_GEAR_RATIO;
        default: return 1.0f;
    }
}

static inline volatile int32_t* get_position_ptr(uint8_t axis) {
    switch (axis) {
        case AXIS_X: return &x_position_steps;
        case AXIS_Y: return &y_position_steps;
        case AXIS_Z: return &z_position_steps;
        default: return NULL;
    }
}

void stepper_init_pins() {
    gpio_init(Y_STEP_PIN); gpio_set_dir(Y_STEP_PIN, GPIO_OUT);
    gpio_init(Y_DIR_PIN); gpio_set_dir(Y_DIR_PIN, GPIO_OUT);
    gpio_init(X_STEP_PIN); gpio_set_dir(X_STEP_PIN, GPIO_OUT);
    gpio_init(X_DIR_PIN); gpio_set_dir(X_DIR_PIN, GPIO_OUT);
    gpio_init(X_DIR_PIN_INV); gpio_set_dir(X_DIR_PIN_INV, GPIO_OUT);
    gpio_init(Z_STEP_PIN); gpio_set_dir(Z_STEP_PIN, GPIO_OUT);
    gpio_init(Z_DIR_PIN); gpio_set_dir(Z_DIR_PIN, GPIO_OUT);
    gpio_init(EN_SENSE_PIN); gpio_set_dir(EN_SENSE_PIN, GPIO_IN);
    gpio_init(EN_PIN); gpio_set_dir(EN_PIN, GPIO_OUT);    

    gpio_put(X_STEP_PIN, 0);
    gpio_put(X_DIR_PIN, 0);
    gpio_put(X_DIR_PIN_INV, 1); // REMEMBER: second X axis stepper must be inverted
    gpio_put(Y_STEP_PIN, 0);
    gpio_put(Y_DIR_PIN, 0);
    gpio_put(Z_STEP_PIN, 0);
    gpio_put(Z_DIR_PIN, 0);
    gpio_put(EN_PIN, 1); // Disable stepper driver initially
}

void stepper_init() {
    stepper_init_pins();
    multicore_launch_core1(stepper_core1_entry);
    printf("Stepper motor control initialized and launched on core 1\n");
}

void stepper_set_enable(bool enable) {
    gpio_put(EN_PIN, enable ? 0 : 1); // Active low
    stepper_enabled = enable;
    printf("Stepper motors %s\n", enable ? "enabled" : "disabled");
}

void stepper_pause() {
    stepper_paused = true;
    printf("Stepper motors paused\n");
}

void stepper_resume() {
    stepper_paused = false;
    printf("Stepper motors resumed\n");
    if(!stepper_enabled)
        stepper_set_enable(true);
}

void stepper_queue_static_move(uint8_t axis, int32_t position_arcsec) {
    if (!stepper_enabled) {
        printf("Stepper not enabled, cannot move!\n");
        return;
    }
    
    if (axis >= NUM_AXES) {
        printf("Invalid axis: %d\n", axis);
        return;
    }
    
    // Stop tracking mode if active
    if (tracking_state.tracking_active) {
        printf("Stopping tracking mode to execute static move\n");
        tracking_state.tracking_active = false;
    }
    
    // Wait until any current command is processed
    while (current_command.valid) {
        sleep_ms(1);
    }
    
    current_command.type = STATIC_MOVE;
    current_command.valid = true;
    current_command.axis = axis;
    current_command.target_position = position_arcsec;

    printf("Queued static move: Axis %d to %ld arcsec\n", axis, position_arcsec);
}

void stepper_start_tracking(float x_rate_arcsec, float y_rate_arcsec, float z_rate_arcsec) {
    if (!stepper_enabled) {
        printf("Stepper not enabled, cannot start tracking!\n");
        return;
    }
    
    // Stop any current movement command
    if (current_command.valid) {
        if(current_command.type == STATIC_MOVE) {
            printf("Stopping current static move to start tracking\n");
            sleep_ms(1);
        }
        current_command.valid = false;
    }
    
    // Set up tracking state
    tracking_state.tracking_active = true;
    tracking_state.rates_arcsec_per_sec[AXIS_X] = x_rate_arcsec;
    tracking_state.rates_arcsec_per_sec[AXIS_Y] = y_rate_arcsec;
    tracking_state.rates_arcsec_per_sec[AXIS_Z] = z_rate_arcsec;
    
    // Initialize last step times to current time
    uint32_t current_time = time_us_32();
    tracking_state.last_step_time[AXIS_X] = current_time;
    tracking_state.last_step_time[AXIS_Y] = current_time;
    tracking_state.last_step_time[AXIS_Z] = current_time;
    
    // Set direction pins based on rates
    gpio_put(X_DIR_PIN, x_rate_arcsec >= 0);
    gpio_put(X_DIR_PIN_INV, x_rate_arcsec < 0);  // Inverted
    gpio_put(Y_DIR_PIN, y_rate_arcsec >= 0);
    gpio_put(Z_DIR_PIN, z_rate_arcsec >= 0);
    
    printf("Started tracking mode: X=%0.2f, Y=%0.2f, Z=%0.2f arcsec/sec\n", 
           x_rate_arcsec, y_rate_arcsec, z_rate_arcsec);
}

void stepper_stop_tracking() {
    if (tracking_state.tracking_active) {
        tracking_state.tracking_active = false;
        printf("Tracking mode stopped\n");
    }
}

int32_t stepper_get_position(uint8_t axis) {
    if (axis >= NUM_AXES) {
        return 0;
    }
    
    volatile int32_t* pos_ptr = get_position_ptr(axis);
    return pos_ptr ? *pos_ptr : 0;
}

int32_t stepper_get_position_arcsec(uint8_t axis) {
    int32_t steps = stepper_get_position(axis);
    return steps_to_arcseconds(steps, get_gear_ratio(axis));
}

void stepper_core1_entry() {
    printf("Stepper core 1 started\n");
        
    while (true) {
        if (!stepper_enabled || stepper_paused) {
            sleep_ms(10);
            continue;
        }
        
        bool active_movement = false;
        
        if (tracking_state.tracking_active) {
            active_movement = true;
            uint32_t current_time = time_us_32();
            
            for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
                float rate = tracking_state.rates_arcsec_per_sec[axis];
                if (rate == 0.0f) continue;
                
                // Calculate interval between steps
                float gear_ratio = get_gear_ratio(axis);
                float steps_per_arcsec = ((float)STEPS_PER_REV * MICROSTEPPING * gear_ratio) / 1296000.0f;
                float steps_per_sec = fabsf(rate) * steps_per_arcsec;
                
                if (steps_per_sec > 0.0f) {
                    uint32_t step_interval_us = (uint32_t)(1000000.0f / steps_per_sec);
                    
                    // Check if it's time for a step
                    if ((current_time - tracking_state.last_step_time[axis]) >= step_interval_us) {
                        gpio_put(get_step_pin(axis), 1);
                        sleep_us(5);
                        gpio_put(get_step_pin(axis), 0);
                        
                        // Update position
                        volatile int32_t* pos_ptr = get_position_ptr(axis);
                        if (pos_ptr) {
                            if (rate > 0.0f) {
                                (*pos_ptr)++;
                            } else {
                                (*pos_ptr)--;
                            }
                        }

                        tracking_state.last_step_time[axis] = current_time;
                    }
                }
            }
        }
        // Process static movement commands
        else if (current_command.valid) {
            active_movement = true;
            uint8_t axis = current_command.axis;
            int32_t target;
            
            if (axis >= NUM_AXES) {
                current_command.valid = false;
                continue;
            }
            
            volatile int32_t* pos_ptr = get_position_ptr(axis);
            if (!pos_ptr) {
                current_command.valid = false;
                continue;
            }
            
            // Calculate target in steps based on command type
            if (current_command.type == STATIC_MOVE) {
                // Convert target position from arcseconds to steps
                target = arcseconds_to_steps(current_command.target_position, get_gear_ratio(axis));
            }
            
            // Determine direction
            bool direction = target >= *pos_ptr;
            gpio_put(get_dir_pin(axis), direction);
            if (axis == AXIS_X) {
                gpio_put(X_DIR_PIN_INV, !direction);
            }
            
            // Calculate steps needed
            int32_t steps = direction ? (target - *pos_ptr) : (*pos_ptr - target);
            
            // Take a step if needed
            if (steps > 0) {
                // Take a step
                gpio_put(get_step_pin(axis), 1);
                sleep_us(5);  // 5μs pulse
                gpio_put(get_step_pin(axis), 0);
                
                // Update position
                if (direction) {
                    (*pos_ptr)++;
                } else {
                    (*pos_ptr)--;
                }
                
                // Sleep between steps (speed control)
                sleep_ms(80);
            } else {
                // Target reached
                current_command.valid = false;
                printf("Axis %d movement complete\n", axis);
            }
        }
    }
}