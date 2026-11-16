#include <Arduino.h>
#include "motors.h"

// =====================================================================
// Motor Pin Configuration
// =====================================================================

// Motor 1
#define ENA 25
#define IN1 16
#define IN2 17

// Motor 2
#define ENB 26
#define IN3 18
#define IN4 19

// PWM Configuration
// ESP32 Arduino 3.x LEDC PWM
// 10-bit resolution: 0 to 1023, 20 kHz frequency

constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 10;

// Motor Direction Configuration
const bool INVERT_MOTOR1 = true;
const bool INVERT_MOTOR2 = false;

// Motor 1 Control
void setMotor1(int pwm) {
    pwm = constrain(pwm, -1023, 1023);

// Reverse direction if configured
    if (INVERT_MOTOR1)
        pwm = -pwm;

// Direction control
    digitalWrite(IN1, pwm > 0);
    digitalWrite(IN2, pwm < 0);

// PWM speed control
    ledcWrite(ENA, abs(pwm));
}

// Motor 2 Control
void setMotor2(int pwm) {
    pwm = constrain(pwm, -1023, 1023);

// Reverse direction if configured
    if (INVERT_MOTOR2)
        pwm = -pwm;

// Direction control
    digitalWrite(IN3, pwm > 0);
    digitalWrite(IN4, pwm < 0);

// PWM speed control
    ledcWrite(ENB, abs(pwm));
}

// =====================================================================
// MOTOR INITIALIZATION
// =====================================================================

bool initMotors() {
    // Configure direction pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Start with both motors stopped
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    // Configure PWM pins
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);

    // Initialize ESP32 LEDC PWM
    bool pwmAReady = ledcAttach(
        ENA,
        PWM_FREQUENCY_HZ,
        PWM_RESOLUTION_BITS
    );
    bool pwmBReady = ledcAttach(
        ENB,
        PWM_FREQUENCY_HZ,
        PWM_RESOLUTION_BITS
    );
    if (!pwmAReady || !pwmBReady) {
        if (pwmAReady) ledcDetach(ENA);
        if (pwmBReady) ledcDetach(ENB);
        digitalWrite(ENA, LOW);
        digitalWrite(ENB, LOW);
        Serial.println(
            "ERROR: Motor PWM initialization failed"
        );
        return false;
    }
    // Ensure both motors are stopped at startup
    setMotor1(0);
    setMotor2(0);

    return true;
}
