#ifndef _RobotConfig_H_
#define _RobotConfig_H_

// =============================================================
//  ROBOT CONFIGURATION — all tunable values live here
// =============================================================

// --- Line Follow: motor speeds (0–255) ---
#define TRACK_SPEED_FORWARD     160   // straight-line speed
#define TRACK_SPEED_TURN        100   // turning speed when correcting

// --- Line Follow: IR sensor thresholds ---
// Lower value = more IR reflection = brighter surface.
// S/E define the window that counts as "on the line."
// V is the lifted-off-ground threshold (all sensors above this).
#define TRACKING_THRESHOLD_LOW  250   // below this = white tape or bare floor, ignore
#define TRACKING_THRESHOLD_HIGH 930   // above this = too dark, ignore
#define TRACKING_THRESHOLD_AIR  970   // all sensors above this = robot lifted


// --- Sumo: IR boundary detection (per-sensor) ---
// Calibrated floor readings:  L=370-460,  M=60-140,   R=43-50
// Calibrated tape readings:   L=861-865,  M=753-759,  R=685-710
// Thresholds set at midpoint between floor max and tape min for each sensor.
#define SUMO_BOUNDARY_L  600   // L: floor max 460, tape min 861
#define SUMO_BOUNDARY_M  400   // M: floor max 138, tape min 753
#define SUMO_BOUNDARY_R  300   // R: floor max  50, tape min 685

// --- Sumo: motor speeds (0–255) ---
#define SUMO_SEARCH_SPEED        50   // outer-wheel speed while arcing during search
#define SUMO_SEARCH_INNER_SPEED  20   // inner-wheel speed — controls arc tightness (lower = tighter curve)
#define SUMO_CHARGE_SPEED       255   // full-speed charge at opponent
#define SUMO_REVERSE_SPEED      100   // reverse speed at boundary
#define SUMO_SPIN_SPEED         150   // spin speed when turning away from boundary

// --- Sumo: distances and timing ---
#define SUMO_DETECT_DISTANCE_CM     60   // cm — ultrasonic opponent detection range
#define SUMO_CONFIRM_COUNT           2   // consecutive readings required before charging (filters EMI spikes)
#define SUMO_ULTRASONIC_INTERVAL_MS 60   // ms between HC-SR04 pings — prevents echo crosstalk (datasheet min ~60ms)
#define SUMO_DELAY_MS             3000   // ms — motors-off countdown at mode start
#define SUMO_REVERSE_MS            350   // ms — reverse phase of boundary avoidance
#define SUMO_SPIN_MS               700   // ms — spin 180° away from boundary (tune until rotation is clean)

// --- Sumo: servo sweep ---
#define SUMO_SERVO_STEP           5   // degrees per sweep step
#define SUMO_SERVO_INTERVAL_MS   40   // ms between sweep steps
#define SUMO_SERVO_MIN           30   // degrees — sweep left limit
#define SUMO_SERVO_MAX          150   // degrees — sweep right limit

// --- General ---
#define LOW_BATTERY_VOLTAGE    7.00   // volts — below this triggers low-battery warning
#define OBSTACLE_DISTANCE_CM     20   // cm — generic obstacle threshold

#endif
