#include "STEPPER.h"

bool stepper_enabled = false;
volatile bool stepper_paused = true;

volatile int32_t x_position_steps = 0;
volatile int32_t y_position_steps = 0;
volatile int32_t z_position_steps = 0;

// Multi-axis command structures - one command per axis
volatile stepper_command_t axis_commands[NUM_AXES] = {
    {STATIC_MOVE, false, AXIS_X, 0},
    {STATIC_MOVE, false, AXIS_Y, 0},
    {STATIC_MOVE, false, AXIS_Z, 0}
};

volatile tracking_state_t tracking_state = {false, {0.0f, 0.0f, 0.0f}, {0, 0, 0}};

int32_t arcseconds_to_steps(int32_t arcseconds, float gear_ratio) {
    // 1296000 = 360° * 60 * 60 (arcseconds in a full circle)
    float steps_per_arcsecond = ((float)STEPS_PER_REV * MICROSTEPPING * gear_ratio) / 1296000.0f;
    
    float exact_steps = arcseconds * steps_per_arcsecond;
    return (int32_t)(exact_steps >= 0 ? exact_steps + 0.5f : exact_steps - 0.5f);
}

int32_t steps_to_arcseconds(int32_t steps, float gear_ratio) {
    float arcseconds_per_step = 1296000.0f / ((float)STEPS_PER_REV * MICROSTEPPING * gear_ratio);
    
    float exact_arcseconds = steps * arcseconds_per_step;
    return (int32_t)(exact_arcseconds >= 0 ? exact_arcseconds + 0.5f : exact_arcseconds - 0.5f);
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
                                // this could be differently by actually letting these drivers not only share a step pin
                                // but also a direction pin, and swapping the wires of a single coil on the second motor
                                // ie swapping the red and blue wires, but since we already have a gpio for each drivers 
                                // dir pin we can just invert in software and have all the colors be the same way 
    gpio_put(Y_STEP_PIN, 0);
    gpio_put(Y_DIR_PIN, 0);
    gpio_put(Z_STEP_PIN, 0);
    gpio_put(Z_DIR_PIN, 0);
    gpio_put(EN_PIN, 1); // Disable stepper driver initially
}

void stepper_init() {
    stepper_init_pins();
    multicore_launch_core1(stepper_core1_entry);
    DEBUG_PRINT("Stepper motor control initialized and launched on core 1\n");
}

void stepper_set_enable(bool enable) {
    gpio_put(EN_PIN, enable ? 0 : 1); // Active low
    stepper_enabled = enable;
    DEBUG_PRINT("Stepper motors %s\n", enable ? "enabled" : "disabled");
}

void stepper_pause() {
    stepper_paused = true;
    DEBUG_PRINT("Stepper motors paused\n");
}

void stepper_resume() {
    stepper_paused = false;
    DEBUG_PRINT("Stepper motors resumed\n");
    if(!stepper_enabled)
        stepper_set_enable(true);
}

bool stepper_is_enabled(void) {
    return stepper_enabled;
}

bool stepper_is_paused(void) {
    return stepper_paused;
}

void stepper_queue_static_move(uint8_t axis, int32_t position_arcsec) {
    if (!stepper_enabled) {
        DEBUG_PRINT("Stepper not enabled, cannot move!\n");
        return;
    }
    
    if (axis >= NUM_AXES) {
        DEBUG_PRINT("Invalid axis: %d\n", axis);
        return;
    }
    
    if (tracking_state.tracking_active) {
        DEBUG_PRINT("Stopping tracking mode to execute static move\n");
        tracking_state.tracking_active = false;
    }
    
    // Set up command for this specific axis
    axis_commands[axis].axis = axis;
    axis_commands[axis].target_position = position_arcsec;
    axis_commands[axis].type = STATIC_MOVE;
    axis_commands[axis].valid = true;

    DEBUG_PRINT("Queued static move: Axis %d to %ld arcsec\n", axis, position_arcsec);
}

void stepper_stop_all_moves() {
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        axis_commands[axis].valid = false;
    }
    DEBUG_PRINT("All axis movements stopped\n");
}

void stepper_start_tracking(float x_rate_arcsec, float y_rate_arcsec, float z_rate_arcsec) {
    if (!stepper_enabled) {
        DEBUG_PRINT("Stepper not enabled, cannot start tracking!\n");
        return;
    }
    
    stepper_stop_all_moves();
    
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
    
    DEBUG_PRINT("Started tracking mode: X=%0.2f, Y=%0.2f, Z=%0.2f arcsec/sec\n", 
           x_rate_arcsec, y_rate_arcsec, z_rate_arcsec);
}

void stepper_stop_tracking() {
    if (tracking_state.tracking_active) {
        tracking_state.tracking_active = false;
        DEBUG_PRINT("Tracking mode stopped\n");
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
    DEBUG_PRINT("Stepper core 1 started\n");
    
    // Per-axis timing variables for multi-axis support
    static absolute_time_t last_step_time[NUM_AXES] = {0};
    static absolute_time_t last_dir_change_time[NUM_AXES] = {0};

    // Direction tracking per axis
    static bool last_direction[NUM_AXES] = {false, false, false};
    
    while (true) {
        if (!stepper_enabled || stepper_paused) {
            sleep_ms(IDLE_SLEEP_MS);
            continue;
        }
        
        bool active_movement = false;
        absolute_time_t now = get_absolute_time();
        
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
                        sleep_us(TRACKING_STEP_PULSE_US);  // Use defined constant
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
        // Process static movement commands for all axes simultaneously
        else {
            // Process each axis independently
            for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
                if (!axis_commands[axis].valid) continue;
                
                active_movement = true;
                
                volatile int32_t* pos_ptr = get_position_ptr(axis);
                if (!pos_ptr) {
                    axis_commands[axis].valid = false;
                    continue;
                }
                
                // Calculate target in steps
                int32_t target = arcseconds_to_steps(axis_commands[axis].target_position, get_gear_ratio(axis));
                
                // Calculate the difference (can be positive or negative)
                int32_t position_diff = target - *pos_ptr;
                
                // Determine direction based on the sign of the difference
                bool direction = position_diff >= 0;
                
                // Calculate absolute steps needed
                int32_t steps = direction ? position_diff : -position_diff;
                
                if (last_direction[axis] != direction) {
                    gpio_put(get_dir_pin(axis), direction);
                    
                    if (axis == AXIS_X) {
                        gpio_put(X_DIR_PIN_INV, !direction);
                    }
                    
                    last_direction[axis] = direction;
                    last_dir_change_time[axis] = now;
                }
                
                // Check if it's time for the next step (non-blocking timing)
                bool direction_setup_complete = absolute_time_diff_us(last_dir_change_time[axis], now) >= DIR_SETUP_TIME_US;
                bool step_interval_ready = absolute_time_diff_us(last_step_time[axis], now) >= (STEP_INTERVAL_MS * 1000);
                
                if (steps > 0 && direction_setup_complete && step_interval_ready) {
                    gpio_put(get_step_pin(axis), 1);
                    sleep_us(STEP_PULSE_WIDTH_US);
                    gpio_put(get_step_pin(axis), 0);
                    
                    // Update position in the correct direction
                    if (direction) {
                        (*pos_ptr)++;
                    } else {
                        (*pos_ptr)--;
                    }
                    
                    // Update last step time for this axis
                    last_step_time[axis] = now;
                    
                    static int step_counter[NUM_AXES] = {0};
                    if (++step_counter[axis] % 1000 == 0) {
                        DEBUG_PRINT("Axis %d: %ld steps remaining\n", axis, steps - 1);
                    }
                } else if (steps == 0) {
                    // Target reached for this axis
                    axis_commands[axis].valid = false;
                    DEBUG_PRINT("Axis %d movement complete at position %ld steps\n", axis, *pos_ptr);
                }
            }
        }
        
        if (!active_movement) {
            sleep_ms(INACTIVE_SLEEP_MS);
        } else {
            sleep_us(ACTIVE_SLEEP_US);
        }
    }
}