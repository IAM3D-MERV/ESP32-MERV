#include <Arduino.h>
#include "motors.h"

// =====================================================================
// MOTOR PIN CONFIGURATION — CYTRON MDD20A
// =====================================================================

// Motor 1
#define PWM1 25
#define DIR1 17

// Motor 2
#define PWM2 26
#define DIR2 19

// Connect ESP32 GND to MDD20A signal GND.
// GPIO 16 and GPIO 18 remain disconnected.

// =====================================================================
// PWM CONFIGURATION
// ESP32 Arduino 3.x LEDC PWM
// =====================================================================

constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 10;
constexpr int MAX_PWM = 1023;

// Motor direction configuration
const bool INVERT_MOTOR1 = true;
const bool INVERT_MOTOR2 = false;

// =====================================================================
// MOTOR 1 CONTROL
// =====================================================================

void setMotor1(int pwm) {

    pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

    if (INVERT_MOTOR1)
        pwm = -pwm;

    // MDD20A: LOW = A positive, HIGH = B positive
    digitalWrite(DIR1, pwm < 0);

    // Zero PWM applies braking.
    ledcWrite(PWM1, abs(pwm));
}

// =====================================================================
// MOTOR 2 CONTROL
// =====================================================================

void setMotor2(int pwm) {

    pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

    if (INVERT_MOTOR2)
        pwm = -pwm;

    digitalWrite(DIR2, pwm < 0);

    // Zero PWM applies braking.
    ledcWrite(PWM2, abs(pwm));
}

// =====================================================================
// MOTOR INITIALIZATION
// =====================================================================

bool initMotors() {

    // Configure direction outputs.
    pinMode(DIR1, OUTPUT);
    pinMode(DIR2, OUTPUT);

    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);

    // Hold PWM inputs LOW before attaching LEDC.
    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT);

    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, LOW);

    // Configure PWM outputs.
    bool pwm1Ready = ledcAttach(
        PWM1,
        PWM_FREQUENCY_HZ,
        PWM_RESOLUTION_BITS
    );

    if (pwm1Ready)
        ledcWrite(PWM1, 0);

    bool pwm2Ready = ledcAttach(
        PWM2,
        PWM_FREQUENCY_HZ,
        PWM_RESOLUTION_BITS
    );

    if (pwm2Ready)
        ledcWrite(PWM2, 0);

    if (!pwm1Ready || !pwm2Ready) {
        Serial.println(
            "ERROR: Motor PWM initialization failed"
        );
        return false;
    }

    // Ensure both motors are stopped at startup.
    setMotor1(0);
    setMotor2(0);

    return true;
}