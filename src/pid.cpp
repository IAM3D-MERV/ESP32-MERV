
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

// Acceleration ramp. ENABLE_RAMP = false sends joystick commands to the
// PID immediately (no ramp).
//
// Like the RC rover code, speeding up is gentle and slowing down /
// stopping is 4x faster, so the rover still stops reasonably quickly:
//   0 -> full speed (173 RPM): ~3.5 s
//   full speed -> stop:        ~0.9 s
// Raise ACCEL_RPM_PER_S for a snappier start, lower it for smoother.
const bool ENABLE_RAMP = true;
const float ACCEL_RPM_PER_S = 50.0f;
const float DECEL_RPM_PER_S = 200.0f;

// =====================================================================
// PID GAINS
// =====================================================================

const float KFF = 1023.0f / FULL_RPM;

float KP = 10.0f;
float KI = 10.0f;
float KD = 0.1f;

// Anti-windup: maximum PWM the integral term may contribute (out of
// 1023). Feedforward supplies most of the drive; the integral only needs
// to correct for friction/load, so it is capped well below full scale.
const float I_LIMIT_PWM = 300.0f;

// Motor deadband compensation (this is NOT a joystick deadzone).
// A DC motor through an L298N doesn't turn until the PWM is high enough
// to overcome the driver's voltage drop and static friction; below that
// it just whines. MIN_PWM is that start-up threshold: any nonzero
// command is mapped into MIN_PWM..1023 instead of 0..1023, so the
// smallest stick movement already turns the treads.
//
// To calibrate: set MIN_PWM = 0, raise throttle slowly, and note the
// `pwm` value in the serial monitor at the moment each tread starts
// turning. Use the larger of the two (left/right) here.
const float MIN_PWM = 350.0f;

// Wheel synchronization
// Keeps both wheels on their commanded paths relative to each other,
// both driving straight and turning (each wheel's target position
// advances at its own target RPM).
//
// Requires BOTH motors and encoders. On a single-motor test rig set this
// to false: encoder 2 never moves, so the sync loop holds motor 1 back
// by up to MAX_SYNC_RPM and it barely turns.
const bool ENABLE_SYNC = true;
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
    float prevTarget;
};

Pid pid1 = {0, 0, 0, 0};
Pid pid2 = {0, 0, 0, 0};

// Desired RPM from web joystick or automatic test
float cmdRpm1 = 0;
float cmdRpm2 = 0;

// Acceleration-limited RPM
float rampRpm1 = 0;
float rampRpm2 = 0;

// Measured wheel speeds
float meas1 = 0;
float meas2 = 0;

// Final PID targets (after sync correction), for debug printing
float tgtOut1 = 0;
float tgtOut2 = 0;

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

// Ramp with separate speed-up and slow-down rates. Moving toward zero
// (slowing, stopping, or the first half of a direction reversal) uses
// the faster decel rate; moving away from zero uses the accel rate.
float rampSpeed(float cur, float target, float dt) {

    bool slowingDown =
        fabsf(target) < fabsf(cur) ||
        target * cur < 0.0f;

    float rate = slowingDown
        ? DECEL_RPM_PER_S
        : ACCEL_RPM_PER_S;

    float next = rampToward(cur, target, rate * dt);

    // Don't overshoot past zero at the fast decel rate during a
    // reversal; stop at zero, then accelerate the other way.
    if (cur * next < 0.0f)
        next = 0.0f;

    return next;
}

// =====================================================================
// RESET PID
// =====================================================================

void resetPid(Pid &p, float meas) {

    p.integral = 0;
    p.prevMeas = meas;
    p.dFilt = 0;
    p.prevTarget = 0;
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

    // Anti-windup (3): direction reversal. Integral built up driving one
    // way must not fight the motor when the target flips sign.
    if (target * p.prevTarget < 0.0f)
        p.integral = 0;

    p.prevTarget = target;

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

    // Anti-windup (1): conditional integration. Stop integrating while
    // the output is saturated and the error would push it further.
    bool saturatedHigh =
        unsat >= 1023.0f && err > 0;

    bool saturatedLow =
        unsat <= -1023.0f && err < 0;

    if (!saturatedHigh && !saturatedLow) {
        p.integral += err * dt;
    }

    // Anti-windup (2): clamp the integral so its contribution can never
    // exceed I_LIMIT_PWM, no matter how long an error persists (stall,
    // wheel held, slow ramp, etc.).
    const float iMax = I_LIMIT_PWM / KI;

    p.integral = constrain(p.integral, -iMax, iMax);

    // Final controller output
    float out =
        ff +
        KP * err +
        KI * p.integral -
        KD * p.dFilt;

    out = constrain(out, -1023.0f, 1023.0f);

    // If the PID wants the opposite direction from the target (it's
    // trying to brake), let the motor coast instead of reversing it.
    // Prevents the compensation below from jerking it backwards.
    if (out * target <= 0.0f)
        return 0;

    // Deadband compensation: map 0..1023 to MIN_PWM..1023 (linear)
    float mag =
        MIN_PWM +
        fabsf(out) * (1023.0f - MIN_PWM) / 1023.0f;

    return (int)(out > 0 ? mag : -mag);
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

    if (ENABLE_RAMP) {

        rampRpm1 = rampSpeed(rampRpm1, cmdRpm1, dt);
        rampRpm2 = rampSpeed(rampRpm2, cmdRpm2, dt);

    } else {

        rampRpm1 = cmdRpm1;
        rampRpm2 = cmdRpm2;
    }

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

    float corr = ENABLE_SYNC
        ? constrain(
              KSYNC * syncErr,
              -MAX_SYNC_RPM,
              MAX_SYNC_RPM
          )
        : 0.0f;

    float tgt1 = rampRpm1;
    float tgt2 = rampRpm2;

    if (tgt1 != 0.0f)
        tgt1 -= corr;

    if (tgt2 != 0.0f)
        tgt2 += corr;

    // -----------------------------------------------------
    // PID OUTPUT
    // -----------------------------------------------------

    tgtOut1 = tgt1;
    tgtOut2 = tgt2;

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

    // cmd = joystick request, tgt = what the PID actually receives
    Serial.print("cmd ");
    Serial.print(cmdRpm1, 1);

    Serial.print("/");
    Serial.print(cmdRpm2, 1);

    Serial.print(" tgt ");
    Serial.print(tgtOut1, 1);

    Serial.print("/");
    Serial.print(tgtOut2, 1);

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
