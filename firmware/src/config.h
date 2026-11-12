// Wiring and homing settings. Everything about the arm's geometry lives on the PC
// (arm_config.json); the firmware only deals in motor microsteps.
#pragma once
#include <Arduino.h>

// Joint order everywhere: 0 = base, 1 = shoulder, 2 = elbow.

// ---- A4988 drivers. STEP and DIR pins must be GPIO 0-31 (they are written from the
//      step interrupt through a fast register). Joint 0 matches your homing test rig.
constexpr int PIN_STEP[3] = {25, 18, 16};
constexpr int PIN_DIR[3]  = {26, 19, 17};
constexpr int PIN_EN      = 27;          // shared enable line, -1 if not wired
constexpr bool EN_ACTIVE_LOW = true;

// ---- Hall sensors (open-collector types pull low at the magnet). Any GPIO with an
//      internal pull-up, so not 34-39.
constexpr int PIN_HALL[3] = {32, 33, 23};
constexpr bool HALL_ACTIVE_LOW = true;

// ---- RGB LED on three PWM pins. Set LED_COMMON_ANODE if the common leg goes to 3.3 V.
constexpr int PIN_LED[3] = {13, 14, 4};  // red, green, blue
constexpr bool LED_COMMON_ANODE = false;
constexpr uint32_t LED_PWM_HZ = 5000;

// ---- Homing. Speeds are in microsteps per second at the motor.
//      HOME_DIR is the DIR pin level that moves a joint toward its hall sensor.
constexpr int HOME_ORDER[3] = {0, 1, 2};         // which joint homes first, second, third
constexpr int HOME_DIR[3]   = {1, 1, 1};
constexpr uint32_t HOME_SEEK_SPS = 1600;         // coarse search
constexpr uint32_t HOME_FINE_SPS = 400;          // final approach
constexpr float HOME_SEEK_LIMIT_DEG = 120.0f;    // give up after this much output travel
constexpr float STEPS_PER_OUTPUT_DEG = 200.0f * 16.0f * 29.0f / 360.0f;   // only used for the limit above

// ---- MT6701 encoders on the motor shafts, in I2C mode. They all answer at the same
//      address, so each one needs its own SDA pin; SCL can be shared. Use -1 as the SDA
//      pin of a joint that has no encoder. Joint 0 matches your homing test rig.
constexpr int PIN_ENC_SDA[3] = {21, 15, 5};
constexpr int PIN_ENC_SCL[3] = {22, 22, 22};
constexpr int ENC_DIR[3] = {1, 1, 1};            // -1 if the count falls when the motor steps forward
constexpr int STEPS_PER_MOTOR_REV = 200 * 16;    // microsteps per motor revolution

// ---- Motion engine. The PC side has matching constants in lightpaint/config.py and
//      lightpaint/device.py; change them together.
constexpr uint32_t TICK_HZ = 40000;      // step interrupt rate = highest possible step rate
constexpr int QUEUE_SIZE = 64;           // buffered segments
constexpr uint32_t STATUS_MS = 50;       // how often the position report is sent
constexpr uint32_t BAUD = 115200;
