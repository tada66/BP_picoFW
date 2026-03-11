#ifndef STEPPER_H
#define STEPPER_H

#include "PIN_ASSIGNMENTS.h"
#include "DEBUGPRINT.h"
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/timer.h"

#define X_STEPPER_GEAR_RATIO 28.5714285714f // 400:14
#define Y_STEPPER_GEAR_RATIO 23.5714285714f // 330:14
#define Z_STEPPER_GEAR_RATIO 30.00078f        // 420:14, Calibrated from measured 360° delta (1295890 arcsec)

#define STEPS_PER_REV 400 // 0.9deg stepper motor
#define MICROSTEPPING 16

// Timing constants for stepper control
#define STEP_INTERVAL_MS 1          // 1ms = 1000 steps/sec
#define DIR_SETUP_TIME_US 1         // 1μs direction setup time for TMC2209
#define STEP_PULSE_WIDTH_US 1       // 1μs step pulse width
#define TRACKING_STEP_PULSE_US 1    // 1μs step pulse for tracking mode
#define IDLE_SLEEP_MS 10            // Sleep when steppers disabled/paused
#define ACTIVE_SLEEP_US 50          // Sleep between active movement cycles
#define INACTIVE_SLEEP_MS 1         // Sleep when no movement active

// Velocity ramp (trapezoidal profile) for slew moves — prevents step loss
#define RAMP_STEPS 200              // Steps over which to accelerate / decelerate
#define RAMP_MIN_INTERVAL_US 1000   // Fastest step rate  (1 kHz = full speed)
#define RAMP_MAX_INTERVAL_US 5000   // Slowest step rate  (200 Hz = start/stop speed)

enum {
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
    NUM_AXES
};

typedef enum {
    STATIC_MOVE = 0,
    TRACKING_MOVE = 1
} stepper_command_type_t;

typedef struct {
    stepper_command_type_t type;
    bool valid;
    uint8_t axis;
    int32_t target_position;
} stepper_command_t;

typedef struct {
    bool tracking_active;
    float rates_arcsec_per_sec[NUM_AXES];  // Tracking rates in arcseconds per second for each axis
    uint32_t last_step_time[NUM_AXES];     // Time of the last step for each axis
} tracking_state_t;

// Celestial tracking state for alt-az mount autonomous tracking
#define SIDEREAL_RATE_ARCSEC_PER_SEC 15.041 

typedef struct {
    bool active;                    // Is celestial tracking active?
    float target_ra;                // Right Ascension in hours (0.0 to 24.0)
    float target_dec;               // Declination in degrees (-90.0 to +90.0)
    float align_matrix[9];          // 3x3 alignment matrix (row-major)
    float latitude;                 // Observer's latitude in degrees
    uint64_t ref_unix_time;         // Unix timestamp when tracking started
    uint32_t ref_boot_time_us;      // Boot time in microseconds when command was received
    bool needs_unwrap_reset;        // Reset atan2 unwrap state on next computation
    // Motor position offsets: motor_arcsec = geometric_angle_arcsec - offset
    // Computed as: trueAlt_arcsec - motorX_arcsec at calibration reference point
    int32_t offset_x;               // X-axis (altitude) motor offset in arcseconds
    int32_t offset_z;               // Z-axis (azimuth)  motor offset in arcseconds
} celestial_tracking_state_t;

void stepper_init_pins();
void stepper_init();
void stepper_core1_entry();
void stepper_set_enable(bool enable);
void stepper_pause();
void stepper_resume();
bool stepper_is_enabled(void);
bool stepper_is_paused(void);

void stepper_queue_static_move(uint8_t axis, int32_t position);
void stepper_queue_relative_move(uint8_t axis, int32_t offset_arcsec);
void stepper_stop_all_moves();  // NEW: Stop all axis movements
int32_t stepper_get_position(uint8_t axis);
void stepper_start_tracking(float x_rate_arcsec, float y_rate_arcsec, float z_rate_arcsec);
void stepper_start_celestial_tracking(float ra, float dec, const float* align_matrix, uint64_t ref_time, float latitude, int32_t offset_x, int32_t offset_z);
void stepper_stop_celestial_tracking(void);
bool stepper_is_celestial_tracking(void);
int32_t stepper_get_position_arcsec(uint8_t axis);
int32_t arcseconds_to_steps(int32_t arcseconds, float gear_ratio);
int32_t steps_to_arcseconds(int32_t steps, float gear_ratio);

#endif // STEPPER_H