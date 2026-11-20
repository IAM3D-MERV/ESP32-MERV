#include <Arduino.h>
#include <encoders.h>


// Encoder Pins
#define ENCODER_A 32
#define ENCODER_B 33

#define ENCODER_2A 27
#define ENCODER_2B 14

// configuration
// Flipped after rewiring: both encoders read negative RPM while their
// motors were driven forward (positive PWM), which made the PID run away
// to full power. Positive PWM must give positive RPM.
const bool INVERT_ENC1 = false;
const bool INVERT_ENC2 = true;

// Encoder state
volatile long encoderCount = 0;
volatile long encoder2Count = 0;

volatile uint8_t enc1State = 0;
volatile uint8_t enc2State = 0;

// Quadrature decoder lookup table
static const int8_t QDEC[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
};


// Encoder Interrupts
// Encoder 1 ISR
void IRAM_ATTR isrEnc1() {

    uint8_t s =
        (digitalRead(ENCODER_A) << 1) |
        digitalRead(ENCODER_B);

    encoderCount += QDEC[(enc1State << 2) | s];

    enc1State = s;
}

// Encoder 2 ISR
void IRAM_ATTR isrEnc2() {

    uint8_t s =
        (digitalRead(ENCODER_2A) << 1) |
        digitalRead(ENCODER_2B);

    encoder2Count += QDEC[(enc2State << 2) | s];

    enc2State = s;
}

// Initialization
void initEncoders() {

    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);

    pinMode(ENCODER_2A, INPUT);
    pinMode(ENCODER_2B, INPUT);
// determines initial encoder states
    enc1State =
        (digitalRead(ENCODER_A) << 1) |
        digitalRead(ENCODER_B);

    enc2State =
        (digitalRead(ENCODER_2A) << 1) |
        digitalRead(ENCODER_2B);

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_A),
        isrEnc1,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_B),
        isrEnc1,
        CHANGE
    );
// Attach interrupts to both channels
    attachInterrupt(
        digitalPinToInterrupt(ENCODER_2A),
        isrEnc2,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_2B),
        isrEnc2,
        CHANGE
    );
}

// Read encoder counts
void readCounts(long &c1, long &c2) {

    noInterrupts();

    c1 = encoderCount;
    c2 = encoder2Count;

    interrupts();

    if (INVERT_ENC1)
        c1 = -c1;

    if (INVERT_ENC2)
        c2 = -c2;
    
}