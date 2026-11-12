
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "web_page.h"
#include "encoders.h"
#include "motors.h"
#include "pid.h"

// =====================================================================
// ESP32-MERV MAIN PROGRAM
// =====================================================================

WebServer server(80);

// Operating mode
const bool AUTO_TEST = false;
const float TEST_RPM = 50.0f;

const unsigned long START_DELAY_MS = 3000;
const unsigned long RUN_DURATION_MS = 15000;

// Joystick settings
const float MAX_WHEEL_RPM = 10.0f;

// Communication timeout
unsigned long lastCommandTime = 0;
const unsigned long timeoutMs = 500;

// =====================================================================
// WEB PAGE HANDLER
// =====================================================================

void handleRoot() {
    server.send_P(200, "text/html", WEB_PAGE);
}

// =====================================================================
// JOYSTICK HANDLER
// =====================================================================

void handleMove() {

    if (!AUTO_TEST) {

        int throttle = constrain(
            server.arg("throttle").toInt(),
            -1023,
            1023
        );

        int steer = constrain(
            server.arg("steer").toInt(),
            -1023,
            1023
        );

        int left = constrain(
            throttle + steer,
            -1023,
            1023
        );

        int right = constrain(
            throttle - steer,
            -1023,
            1023
        );

        float leftRPM =
            left / 1023.0f * MAX_WHEEL_RPM;

        float rightRPM =
            right / 1023.0f * MAX_WHEEL_RPM;

        setTargetRPM(leftRPM, rightRPM);

        lastCommandTime = millis();
    }

    server.send(200, "text/plain", "OK");
}

// =====================================================================
// ENCODER WEB HANDLER
// =====================================================================

void handleEncoder() {

    float rpm1, rpm2;

    getMeasuredRPM(rpm1, rpm2);

    server.send(
        200,
        "text/plain",
        String(rpm1, 0) + " | " + String(rpm2, 0)
    );
}

// =====================================================================
// SETUP
// =====================================================================

void setup() {

    Serial.begin(9600);

    // Initialize motor module
    if (!initMotors()) {

        Serial.println(
            "ERROR: Motor PWM initialization failed"
        );

        while (true) {
            delay(1000);
        }
    }

    // Initialize encoders
    initEncoders();

    // Initialize Wi-Fi
    WiFi.softAP("Rover-AP", "12345678");

    Serial.println();
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    // Configure web server
    server.on("/", handleRoot);
    server.on("/move", handleMove);
    server.on("/encoder", handleEncoder);

    server.begin();

    Serial.println(
        AUTO_TEST
        ? "AUTO TEST mode: motors start in 3 s"
        : "Joystick mode"
    );

    // Initialize PID control system
    initControl();
}

// =====================================================================
// MAIN LOOP
// =====================================================================

void loop() {

    server.handleClient();

    // Automatic motor test
    if (AUTO_TEST) {

        unsigned long t = millis();

        bool on =
            t >= START_DELAY_MS &&
            t < START_DELAY_MS + RUN_DURATION_MS;

        float target = on ? TEST_RPM : 0;

        setTargetRPM(target, target);
    }

    // Stop command when joystick communication times out
    else if (millis() - lastCommandTime > timeoutMs) {

        setTargetRPM(0, 0);
    }

    // Update PID controller
    controlUpdate();

    // Print debug information
    printStatus();
}
