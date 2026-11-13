
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

    // Receive commands from web joystick
    handleWebClients();

    // Stop both motors if joystick communication is lost
    if (joystickTimedOut(TIMEOUT_MS)) {
        setMotor1(0);
        setMotor2(0);
    }

    // NO PID
    // NO wheel synchronization
    // NO encoder feedback
}
