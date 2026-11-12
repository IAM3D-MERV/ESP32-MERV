#pragma once

#include <Arduino.h>


// initialize encoder pins and interrupts
void initEncoders();

//read both encoder counts
void readCounts(long &c1, long &c2);

