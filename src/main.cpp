
#include <Arduino.h>

#include "encoders.h"
#include "motors.h"
#include "pid.h"
#include "web_server.h"

// =====================================================================
// ESP32-MERV
// MAIN PROGRAM
//
// PURPOSE:
// Coordinate initialization and operation of rover subsystems.
//
// MODULES:
// - motors.cpp      : Motor direction and PWM
// - encoders.cpp    : Quadrature encoder feedback
// - pid.cpp         : PID and wheel synchronization
// - web_server.cpp  : Wi-Fi and joystick communication
// - web_page.h      : Web interface
// =====================================================================

// =====================================================================
// OPERATING MODE
// =====================================================================

const bool AUTO_TEST = false;

// Automatic test settings
const float TEST_RPM = 50.0f;

const unsigned long START_DELAY_MS = 3000;
const unsigned long RUN_DURATION_MS = 15000;

// =====================================================================
// JOYSTICK CONFIGURATION
// =====================================================================

// Maximum requested wheel speed
const float MAX_WHEEL_RPM = 10.0f;

// Communication timeout in milliseconds
const unsigned long TIMEOUT_MS = 500;

// =====================================================================
// SETUP
// =====================================================================

void setup() {

    Serial.begin(9600);

    // -----------------------------------------------------
    // INITIALIZE MOTOR SYSTEM
    // -----------------------------------------------------

    if (!initMotors()) {

        Serial.println(
            "ERROR: Motor PWM initialization failed"
        );

        while (true) {
            delay(1000);
        }
    }

    Serial.println("Motor module initialized");

    // -----------------------------------------------------
    // INITIALIZE ENCODER SYSTEM
    // -----------------------------------------------------

    initEncoders();

    Serial.println("Encoder module initialized");

    // -----------------------------------------------------
    // INITIALIZE WEB SERVER
    // -----------------------------------------------------

    if (!initWebServer(AUTO_TEST, MAX_WHEEL_RPM)) {

        Serial.println(
            "ERROR: Web server initialization failed"
        );

        while (true) {
            delay(1000);
        }
    }

    // -----------------------------------------------------
    // INITIALIZE PID CONTROL SYSTEM
    // -----------------------------------------------------

    initControl();

    Serial.println("PID module initialized");

    // -----------------------------------------------------
    // SYSTEM READY
    // -----------------------------------------------------

    Serial.println("ESP32-MERV Initialized");
}

// =====================================================================
// MAIN LOOP
// =====================================================================

void loop() {

    // -----------------------------------------------------
    // PROCESS WEB REQUESTS
    // -----------------------------------------------------

    handleWebClients();

    // -----------------------------------------------------
    // AUTOMATIC TEST MODE
    // -----------------------------------------------------

    if (AUTO_TEST) {

        unsigned long t = millis();

        bool on =
            t >= START_DELAY_MS &&
            t < START_DELAY_MS + RUN_DURATION_MS;

        float target = on ? TEST_RPM : 0;

        setTargetRPM(target, target);
    }

    // -----------------------------------------------------
    // JOYSTICK COMMUNICATION TIMEOUT
    // -----------------------------------------------------

    else if (joystickTimedOut(TIMEOUT_MS)) {

        setTargetRPM(0, 0);
    }

    // -----------------------------------------------------
    // UPDATE PID CONTROL SYSTEM
    // -----------------------------------------------------

    controlUpdate();

    // -----------------------------------------------------
    // SERIAL DEBUG OUTPUT
    // -----------------------------------------------------

    printStatus();
}
