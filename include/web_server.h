
#pragma once

// Initialize Wi-Fi and web server
bool initWebServer(bool autoTest, float maxWheelRpm);

// Handle incoming HTTP requests
void handleWebClients();

// Check whether joystick commands have timed out
bool joystickTimedOut(unsigned long timeoutMs);
