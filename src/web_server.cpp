
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "web_server.h"
#include "web_page.h"
#include "pid.h"
#include "motors.h"

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
// JOYSTICK HANDLER
// =====================================================================



void handleMove() {

    if (!autoTestMode) {

        // Read forward/backward joystick movement
        int throttle = constrain(
            server.arg("throttle").toInt(),
            -1023,
            1023
        );

        // Read left/right joystick movement
        int steer = constrain(
            server.arg("steer").toInt(),
            -1023,
            1023
        );

        // Calculate command for the LEFT motor
        int left = constrain(
            throttle + steer,
            -1023,
            1023
        );

        // Limit PWM for testing
        const int TEST_PWM_LIMIT = 1023;

        // Convert joystick command to PWM
        int pwmCommand =
            left * TEST_PWM_LIMIT / 1023;

        // Control Motor 1 directly
        setMotor1(pwmCommand);

        // Motor 2 remains stopped
        setMotor2(0);

        // Debug output
        Serial.printf(
            "Throttle: %d | Steer: %d | Motor 1 PWM: %d\n",
            throttle,
            steer,
            pwmCommand
        );

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
