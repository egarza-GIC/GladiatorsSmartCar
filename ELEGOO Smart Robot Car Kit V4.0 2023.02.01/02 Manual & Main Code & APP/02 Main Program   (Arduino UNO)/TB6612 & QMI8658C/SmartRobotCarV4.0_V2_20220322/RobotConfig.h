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
// Calibrated readings on outer black boundary tape: L=813, M=750, R=715 (±10 each).
// Threshold = calibrated reading - 10 (catches low end of tape range).
// Floor reads significantly lower and will not trigger these thresholds.
#define SUMO_BOUNDARY_L  803   // L sensor triggers at >= 803
#define SUMO_BOUNDARY_M  740   // M sensor triggers at >= 740
#define SUMO_BOUNDARY_R  705   // R sensor triggers at >= 705

// --- Sumo: motor speeds (0–255) ---
#define SUMO_SEARCH_SPEED        80   // creep speed while scanning
#define SUMO_CHARGE_SPEED       255   // full-speed charge at opponent
#define SUMO_REVERSE_SPEED      200   // reverse speed at boundary
#define SUMO_SPIN_SPEED         150   // spin speed when turning away from boundary

// --- Sumo: distances and timing ---
#define SUMO_DETECT_DISTANCE_CM  60   // cm — ultrasonic opponent detection range
#define SUMO_CONFIRM_COUNT        3   // consecutive readings required before charging (filters EMI spikes)
#define SUMO_DELAY_MS          3000   // ms — motors-off countdown at mode start
#define SUMO_REVERSE_MS         350   // ms — reverse phase of boundary avoidance
#define SUMO_SPIN_MS            350   // ms — spin phase of boundary avoidance

// --- Sumo: servo sweep ---
#define SUMO_SERVO_STEP           5   // degrees per sweep step
#define SUMO_SERVO_INTERVAL_MS   80   // ms between sweep steps
#define SUMO_SERVO_MIN           30   // degrees — sweep left limit
#define SUMO_SERVO_MAX          150   // degrees — sweep right limit

// --- General ---
#define LOW_BATTERY_VOLTAGE    7.00   // volts — below this triggers low-battery warning
#define OBSTACLE_DISTANCE_CM     20   // cm — generic obstacle threshold

#endif
