
#pragma once

// Initialize the PID control system
void initControl();

// Set target RPM for both wheels
void setTargetRPM(float rpm1, float rpm2);

// Get measured RPM for both wheels
void getMeasuredRPM(float &rpm1, float &rpm2);

// Update PID and wheel synchronization
void controlUpdate();

// Print controller status
void printStatus();
