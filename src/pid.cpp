
#include <Arduino.h>

#include "pid.h"
#include "encoders.h"
#include "motors.h"

// =====================================================================
// ESP32-MERV
// PID WHEEL-SPEED CONTROLLER + WHEEL SYNCHRONIZATION
//
// PURPOSE: Control the RPM of both rover wheels using encoder feedback,
// PID Control, acceleration timing, and wheel synchronization.
//
// The PID controller receives target RPM values and calculates signed
// PWM commands (-1023 to 1023). 
// 
// 1. Verify Encoder Configuration:
// File: encoders.cpp
// [] Confirm encoder GPIO pin assignments
// [] Confirm COUNTS_PER_MOTOR_REV and GEAR_RATIO values
// [] Check that each encoder counts in the expected direction
// [] Adjust INVERT_ENC1 and INVERT_ENC2 if necessary
// 
// Current assumed values:
// COUNTS_PER_MOTOR_REV = 64
// GEAR_RATIO = 50
// COUNTS_PER_WHEEL_REV =  64*50 = 3200
//
// 2. Verify motor direction:
// File: motors.cpp
// [] Verify both motors can rotate forward and backward
// [] Check INVERT_MOTOR1 and INVERT_MOTOR2
// [] Confirm positive PWM commands produce forward motion, negative commands produce reverse motion
// [] Confirm positive wheel movement produces positive RPM
// 
// 3. Calibrate motor speed:
// [] Verify RPM measurements using encoder feedback.
// [] Determine the motor's actual maximum wheel RPM
// [] Updaate FULL_RPM with the measured value
// [] Choose a safe MAX_WHEEL_RPM in main.cpp
//
// Current assumed values:
// FULL_RPM = 173 , used to calculate feedforward gain KFF = 1023/FULL_RPM
// MAX_WHEEL_RPM = 10
// 
// =====================================================================

// These variables and helper functions are private to this file.
namespace {

// =====================================================================
// MOTOR SPECIFICATIONS
// =====================================================================

const float COUNTS_PER_MOTOR_REV = 64.0f;
const float GEAR_RATIO = 50.0f;

const float COUNTS_PER_WHEEL_REV =
    COUNTS_PER_MOTOR_REV * GEAR_RATIO;

const float FULL_RPM = 173.0f;
const float ACCEL_RPM_PER_S = 20.0f;

// =====================================================================
// PID GAINS
// =====================================================================

const float KFF = 1023.0f / FULL_RPM;

float KP = 6.0f;
float KI = 40.0f;
float KD = 0.05f;

// Wheel synchronization
float KSYNC = 50.0f;
const float MAX_SYNC_RPM = 10.0f;

// =====================================================================
// TIMING CONFIGURATION
// =====================================================================

const uint32_t CONTROL_PERIOD_US = 25000;
const unsigned long PRINT_MS = 200;

// =====================================================================
// PID STATE
// =====================================================================

struct Pid {
    float integral;
    float prevMeas;
    float dFilt;
};

Pid pid1 = {0, 0, 0};
Pid pid2 = {0, 0, 0};

// Desired RPM from web joystick or automatic test
float cmdRpm1 = 0;
float cmdRpm2 = 0;

// Acceleration-limited RPM
float rampRpm1 = 0;
float rampRpm2 = 0;

// Measured wheel speeds
float meas1 = 0;
float meas2 = 0;

// Output PWM values
int pwm1 = 0;
int pwm2 = 0;

// Wheel synchronization state
float syncErr = 0;

float targetPos1 = 0;
float targetPos2 = 0;

// Previous encoder counts
long prevCount1 = 0;
long prevCount2 = 0;

// Timing variables
uint32_t lastControlUs = 0;
unsigned long lastPrint = 0;

// =====================================================================
// ACCELERATION HELPER
// =====================================================================

float rampToward(
    float cur,
    float target,
    float maxStep
) {
    if (target > cur)
        return min(cur + maxStep, target);

    return max(cur - maxStep, target);
}

// =====================================================================
// RESET PID
// =====================================================================

void resetPid(Pid &p, float meas) {

    p.integral = 0;
    p.prevMeas = meas;
    p.dFilt = 0;
}

// =====================================================================
// PID CALCULATION
// =====================================================================

int pidStep(
    Pid &p,
    float target,
    float meas,
    float dt
) {

    if (target == 0.0f) {
        resetPid(p, meas);
        return 0;
    }

    float err = target - meas;

    // Derivative on measured speed
    float dMeas =
        (meas - p.prevMeas) / dt;

    p.prevMeas = meas;

    // Filter derivative measurement
    p.dFilt =
        0.8f * p.dFilt +
        0.2f * dMeas;

    // Feedforward
    float ff = KFF * target;

    // Calculate unsaturated controller output
    float unsat =
        ff +
        KP * err +
        KI * p.integral -
        KD * p.dFilt;

    // Anti-windup
    bool saturatedHigh =
        unsat >= 1023.0f && err > 0;

    bool saturatedLow =
        unsat <= -1023.0f && err < 0;

    if (!saturatedHigh && !saturatedLow) {
        p.integral += err * dt;
    }

    // Final controller output
    float out =
        ff +
        KP * err +
        KI * p.integral -
        KD * p.dFilt;

    return constrain((int)out, -1023, 1023);
}

} // End private namespace

// =====================================================================
// INITIALIZE CONTROL SYSTEM
// =====================================================================

void initControl() {

    lastControlUs = micros();
}

// =====================================================================
// SET TARGET WHEEL SPEEDS
// =====================================================================

void setTargetRPM(float rpm1, float rpm2) {

    cmdRpm1 = rpm1;
    cmdRpm2 = rpm2;
}

// =====================================================================
// READ MEASURED WHEEL SPEEDS
// =====================================================================

void getMeasuredRPM(float &rpm1, float &rpm2) {

    rpm1 = meas1;
    rpm2 = meas2;
}

// =====================================================================
// MAIN CONTROL LOOP
// =====================================================================

void controlUpdate() {

    uint32_t nowUs = micros();
    uint32_t dtUs = nowUs - lastControlUs;

    if (dtUs < CONTROL_PERIOD_US)
        return;

    lastControlUs = nowUs;

    float dt = dtUs * 1e-6f;

    // -----------------------------------------------------
    // READ ENCODER COUNTS
    // -----------------------------------------------------

    long c1, c2;
    readCounts(c1, c2);

    long d1 = c1 - prevCount1;
    long d2 = c2 - prevCount2;

    prevCount1 = c1;
    prevCount2 = c2;

    // -----------------------------------------------------
    // CALCULATE MEASURED WHEEL RPM
    // -----------------------------------------------------

    float raw1 =
        d1 / COUNTS_PER_WHEEL_REV / dt * 60.0f;

    float raw2 =
        d2 / COUNTS_PER_WHEEL_REV / dt * 60.0f;

    meas1 = 0.6f * meas1 + 0.4f * raw1;
    meas2 = 0.6f * meas2 + 0.4f * raw2;

    // -----------------------------------------------------
    // SMOOTH ACCELERATION
    // -----------------------------------------------------

    float step = ACCEL_RPM_PER_S * dt;

    rampRpm1 =
        rampToward(rampRpm1, cmdRpm1, step);

    rampRpm2 =
        rampToward(rampRpm2, cmdRpm2, step);

    // -----------------------------------------------------
    // WHEEL POSITION CALCULATION
    // -----------------------------------------------------

    float pos1 = c1 / COUNTS_PER_WHEEL_REV;
    float pos2 = c2 / COUNTS_PER_WHEEL_REV;

    if (
        rampRpm1 == 0.0f &&
        rampRpm2 == 0.0f
    ) {

        targetPos1 = pos1;
        targetPos2 = pos2;

        syncErr = 0;

    } else {

        targetPos1 +=
            rampRpm1 * dt / 60.0f;

        targetPos2 +=
            rampRpm2 * dt / 60.0f;
    }

    // -----------------------------------------------------
    // WHEEL SYNCHRONIZATION
    // -----------------------------------------------------

    syncErr =
        (pos1 - targetPos1) -
        (pos2 - targetPos2);

    float corr = constrain(
        KSYNC * syncErr,
        -MAX_SYNC_RPM,
        MAX_SYNC_RPM
    );

    float tgt1 = rampRpm1;
    float tgt2 = rampRpm2;

    if (tgt1 != 0.0f)
        tgt1 -= corr;

    if (tgt2 != 0.0f)
        tgt2 += corr;

    // -----------------------------------------------------
    // PID OUTPUT
    // -----------------------------------------------------

    pwm1 = pidStep(pid1, tgt1, meas1, dt);
    pwm2 = pidStep(pid2, tgt2, meas2, dt);

    // -----------------------------------------------------
    // APPLY MOTOR COMMANDS
    // -----------------------------------------------------

    setMotor1(pwm1);
    setMotor2(pwm2);
}

// =====================================================================
// SERIAL STATUS
// =====================================================================

void printStatus() {

    unsigned long now = millis();

    if (now - lastPrint < PRINT_MS)
        return;

    lastPrint = now;

    Serial.print("tgt ");
    Serial.print(rampRpm1, 1);

    Serial.print("/");
    Serial.print(rampRpm2, 1);

    Serial.print(" | L rpm ");
    Serial.print(meas1, 1);

    Serial.print(" pwm ");
    Serial.print(pwm1);

    Serial.print(" | R rpm ");
    Serial.print(meas2, 1);

    Serial.print(" pwm ");
    Serial.print(pwm2);

    Serial.print(" | sync ");
    Serial.println(syncErr, 3);
}
