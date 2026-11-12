#pragma once

#include <Arduino.h>

// initialize motor pins and PWM
bool initMotors();

// control motor speed and direction
void setMotor1(int pwm);
void setMotor2(int pwm);

