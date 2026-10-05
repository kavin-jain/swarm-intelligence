// Per-robot hardware settings. Pin defaults suit an ESP32 DevKit v1 + L298N/L293D.
// Avoided on purpose: GPIO 6-11 (flash), 34-39 (input only), and the boot-strapping
// pins 0, 2, 5, 12, 15 for the motor lines (a motor pin pulled the wrong way at reset
// can stop the board booting).
#pragma once

// ---- L298N / L293D: each motor = one PWM enable pin + two direction pins ----------
#define PIN_L_EN   14
#define PIN_L_IN1  27
#define PIN_L_IN2  26
#define PIN_R_EN   32
#define PIN_R_IN1  25
#define PIN_R_IN2  33
#define INVERT_L   0     // set to 1 if `motortest` shows this wheel spinning backwards
#define INVERT_R   0
#define PWM_FREQ   1000  // Hz. L298N is slow-switching; low frequency keeps torque at low duty
#define PWM_MIN    90    // 0-255: the duty below which the wheels don't turn (measure with motortest)

// ---- gripper (carry mode) ------------------------------------------------------------
// An electromagnet (or any grip that closes on a high signal) on the robot's front, switched by a
// logic-level MOSFET with a flyback diode across the coil. -1 = no gripper fitted: the robot pushes.
// Peak-and-hold: full power to grab, then a lower duty to hold -- a magnet needs far less current to
// keep holding than to pull in, so this cuts the gripper's draw for the whole carry.
#define GRIPPER_PIN     -1    // e.g. 13 (any output pin outside the flash/strapping/input-only ranges)
#define GRIP_PEAK_MS    150   // full power for this long after switching on
#define GRIP_HOLD_DUTY  90    // 0-255 duty while holding (~35%); raise it if loads slip off

// ---- status LED and battery --------------------------------------------------------
#define PIN_LED    2     // onboard LED on most DevKits
#define PIN_BATT   -1    // ADC1 pin (32-39) through a divider, or -1 if not wired.
                         // Must be ADC1: ADC2 stops working while the radio is on.
#define BATT_DIVIDER 3.0f    // (R1+R2)/R2 of your divider
#define BATT_CUTOFF_MV 6400  // 2S Li-ion: 3.2 V/cell. Below this the robot parks itself.

// ---- radio -------------------------------------------------------------------------
#define WIFI_CHANNEL 1   // must be the same on every robot and the gateway

// ---- motion calibration (run `satellite.py --measure <id>` with motortest flashed) ---
#define CAL_VMAX       300.0f   // mm/s at full PWM
#define CAL_WHEEL_BASE 110.0f   // mm between the wheels' contact points
#define CAL_ROBOT_RADIUS 60.0f  // mm, half the chassis width
