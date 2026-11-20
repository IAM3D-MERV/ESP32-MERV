
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "web_server.h"
#include "web_page.h"
#include "pid.h"

// =====================================================================
// ESP32-MERV
// WEB SERVER + JOYSTICK CONTROL
//
// PURPOSE:
// - Initialize the ESP32 Wi-Fi access point
// - Serve the rover control webpage
// - Receive joystick commands
// - Convert joystick input into target wheel RPM
// - Provide encoder RPM data to the webpage
// - Track joystick communication timeout
// =====================================================================

// Variables and handlers private to this module
namespace {

// =====================================================================
// WI-FI CONFIGURATION
// =====================================================================

const char* AP_SSID = "Rover-AP";
const char* AP_PASSWORD = "12345678";

// =====================================================================
// WEB SERVER CONFIGURATION
// =====================================================================

WebServer server(80);

// Operating mode
bool autoTestMode = false;

// Maximum joystick-commanded wheel RPM
float maxWheelRpm = 10.0f;

// Last time a joystick command was received
unsigned long lastCommandTime = 0;

// =====================================================================
// WEB PAGE HANDLER
// =====================================================================

void handleRoot() {

    // Serve HTML from web_page.h
    server.send_P(200, "text/html", WEB_PAGE);
}

// =====================================================================
// JOYSTICK ANGLE SECTIONS
// =====================================================================

// Size of each joystick direction section, in degrees.
//
// The joystick circle is split into sections of this size. Each section
// has its own pair of left/right target RPMs, computed by
// sectionTargets() below, so changing this one number automatically
// recomputes every section's speeds -- no table to edit.
const float ANGLE_STEP_DEG = 6.0f;

// Rounds a stick angle to the center of its section.
//
// Sections are counted outward from the nearest main axis (forward,
// right, back, left), so each axis sits in the middle of a section:
// within +/- ANGLE_STEP_DEG/2 of straight, both treads get exactly the
// same speed; within +/- ANGLE_STEP_DEG/2 of sideways, they spin at
// exactly equal and opposite speeds. (90 is not always a multiple of
// the step, so counting from forward alone could put "sideways" at
// e.g. 84 or 96 degrees.)
float snapAngle(float angleDeg) {

    float axis = roundf(angleDeg / 90.0f) * 90.0f;
    float offset = angleDeg - axis;   // -45 .. +45

    return axis + roundf(offset / ANGLE_STEP_DEG) * ANGLE_STEP_DEG;
}

// Target speed fractions (-1..1) for the left and right treads in the
// section centered at angleDeg (0 = forward, +90 = right, -90 = left,
// +/-180 = back).
//
// Linear blend: the outer tread runs at full speed; the inner tread
// steps down by the same amount every section:
//
//   angle from straight:   0      45       90
//   inner tread:          +1      0        -1
//                     (straight) (pivot) (spin in place)
//
//   inner = 1 - 2 * angle / 90
//
// Backward is the mirror image, with both treads reversed.
void sectionTargets(float angleDeg, float &left, float &right) {

    float a = fabsf(angleDeg);       // 0..180
    bool turnRight = angleDeg > 0.0f;

    if (a <= 90.0f) {

        // Forward half: outer tread full forward
        float inner = 1.0f - 2.0f * a / 90.0f;

        left  = turnRight ? 1.0f  : inner;
        right = turnRight ? inner : 1.0f;

    } else {

        // Backward half: measure the angle from straight back
        float b = 180.0f - a;        // 0 (straight back) .. 90
        float inner = -(1.0f - 2.0f * b / 90.0f);

        // Stick back-right: right tread is the outer (full reverse)
        // tread, matching the original throttle/steer mixing.
        left  = turnRight ? inner : -1.0f;
        right = turnRight ? -1.0f : inner;
    }
}

// =====================================================================
// JOYSTICK HANDLER
// =====================================================================

void handleMove() {

    if (!autoTestMode) {

        // Read joystick throttle
        int throttle = constrain(
            server.arg("throttle").toInt(),
            -1023,
            1023
        );

        // Read joystick steering
        int steer = constrain(
            server.arg("steer").toInt(),
            -1023,
            1023
        );

        // -------------------------------------------------
        // SECTION-BASED STEERING (no deadzone)
        // -------------------------------------------------

        // Stick position as -1..1 (forward/right positive)
        float t = throttle / 1023.0f;
        float s = steer / 1023.0f;

        // Stick distance from center (0..1) scales the speed
        float dist = constrain(sqrtf(t * t + s * s), 0.0f, 1.0f);

        float left = 0.0f;
        float right = 0.0f;

        if (dist > 0.0f) {

            // Stick direction: 0 = forward, +90 = right
            float angle = atan2f(s, t) * RAD_TO_DEG;

            // Which section the stick is in -> that section's
            // left/right speed pattern
            sectionTargets(snapAngle(angle), left, right);

            left *= dist;
            right *= dist;
        }

        // -------------------------------------------------
        // PID INTEGRATION
        // Each tread's section speed becomes a target RPM;
        // the PID in pid.cpp holds each tread at that speed
        // using encoder feedback.
        // -------------------------------------------------

        float leftRPM = left * maxWheelRpm;
        float rightRPM = right * maxWheelRpm;

        setTargetRPM(leftRPM, rightRPM);

        // Update communication timestamp
        lastCommandTime = millis();
    }

    server.send(200, "text/plain", "OK");
}

// =====================================================================
// ENCODER WEB HANDLER
// =====================================================================

void handleEncoder() {

    float rpm1, rpm2;

    // Read measured RPM from PID module
    getMeasuredRPM(rpm1, rpm2);

    // Send encoder measurements to webpage
    server.send(
        200,
        "text/plain",
        String(rpm1, 0) + " | " + String(rpm2, 0)
    );
}

} // End private namespace

// =====================================================================
// INITIALIZE WEB SERVER
// =====================================================================

bool initWebServer(bool autoTest, float maxRpm) {

    autoTestMode = autoTest;
    maxWheelRpm = maxRpm;
    lastCommandTime = millis();

    // Start ESP32 Wi-Fi access point
    if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {

        Serial.println("ERROR: Wi-Fi AP failed");
        return false;
    }

    // Disable Wi-Fi power saving to reduce command latency
    WiFi.setSleep(false);

    Serial.println();

    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    // Register HTTP routes
    server.on("/", handleRoot);
    server.on("/move", handleMove);
    server.on("/encoder", handleEncoder);

    // Start HTTP server
    server.begin();

    Serial.println("Web server initialized");

    Serial.println(
        autoTestMode
        ? "AUTO TEST mode: motors start in 3 s"
        : "Joystick mode"
    );

    return true;
}

// =====================================================================
// HANDLE INCOMING WEB REQUESTS
// =====================================================================

void handleWebClients() {

    server.handleClient();
}

// =====================================================================
// JOYSTICK COMMUNICATION TIMEOUT
// =====================================================================

bool joystickTimedOut(unsigned long timeoutMs) {

    return millis() - lastCommandTime > timeoutMs;
}
