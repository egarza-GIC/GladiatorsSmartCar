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

// --- Line Follow: line-lost recovery ---
// Robot creeps forward for this window trying to reacquire the line.
// Only if the line is still not found after this does the dance trigger.
#define TRACK_RECOVERY_MS       300   // ms — how long to search before giving up
#define TRACK_RECOVERY_SPEED     80   // motor speed during recovery creep

// --- Line Follow: end-of-line dance ---
// Sequence: pause → spin right → spin left → spin right → stop
#define DANCE_PAUSE_MS          400   // ms — brief stop before dancing
#define DANCE_SPIN1_MS          400   // ms — first spin (right)
#define DANCE_SPIN2_MS          800   // ms — second spin (left, double)
#define DANCE_SPIN3_MS          400   // ms — third spin (right)
#define DANCE_SPIN_SPEED        255   // 0–255 — spin motor speed

// --- Sumo: IR boundary detection ---
// Sensors below this value = white boundary tape detected.
// Glossy black floor reflects very little IR (high reading).
// Matte white tape reflects a lot (low reading, ~30–80).
#define SUMO_BOUNDARY_THRESHOLD  80   // tune lower to reduce false triggers

// --- Sumo: motor speeds (0–255) ---
#define SUMO_SEARCH_SPEED        80   // creep speed while scanning
#define SUMO_CHARGE_SPEED       255   // full-speed charge at opponent
#define SUMO_REVERSE_SPEED      200   // reverse speed at boundary
#define SUMO_SPIN_SPEED         150   // spin speed when turning away from boundary

// --- Sumo: distances and timing ---
#define SUMO_DETECT_DISTANCE_CM  60   // cm — ultrasonic opponent detection range
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
