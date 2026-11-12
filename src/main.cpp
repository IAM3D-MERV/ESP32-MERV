
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// =====================================================================
// ESP32-MERV
// PID WHEEL-SPEED CONTROL + WHEEL SYNC + ARM CONTROLS
// =====================================================================

WebServer server(80);

// ------------------------ PWM SETTINGS -------------------------------
// ESP32 Arduino 3.x LEDC PWM
// 10-bit resolution: 0 to 1023

constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 10;

// ----------------------------- MODE ----------------------------------

const bool AUTO_TEST = false;
const float TEST_RPM = 50.0f;

const unsigned long START_DELAY_MS = 3000;
const unsigned long RUN_DURATION_MS = 15000;

// ----------------------------- PINS ----------------------------------

#define ENA 25
#define IN1 16
#define IN2 17

#define ENB 26
#define IN3 18
#define IN4 19

#define ENCODER_A 32
#define ENCODER_B 33
#define ENCODER_2A 27
#define ENCODER_2B 14

// --------------------------- DIRECTION -------------------------------

const bool INVERT_MOTOR1 = true;
const bool INVERT_MOTOR2 = false;

const bool INVERT_ENC1 = true;
const bool INVERT_ENC2 = false;

// --------------------------- MOTOR SPECS -----------------------------

const float COUNTS_PER_MOTOR_REV = 64.0f;
const float GEAR_RATIO = 50.0f;

const float COUNTS_PER_WHEEL_REV =
    COUNTS_PER_MOTOR_REV * GEAR_RATIO;

const float FULL_RPM = 173.0f;
const float MAX_WHEEL_RPM = 10.0f;
const float ACCEL_RPM_PER_S = 20.0f;

// ---------------------------- PID GAINS ------------------------------

const float KFF = 1023.0f / FULL_RPM;

float KP = 6.0f;
float KI = 40.0f;
float KD = 0.05f;

float KSYNC = 50.0f;
const float MAX_SYNC_RPM = 10.0f;

const uint32_t CONTROL_PERIOD_US = 25000;
const unsigned long PRINT_MS = 200;

// ----------------------------- STATE ---------------------------------

volatile long encoderCount = 0;
volatile long encoder2Count = 0;

volatile uint8_t enc1State = 0;
volatile uint8_t enc2State = 0;

static const int8_t QDEC[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
};

struct Pid {
    float integral;
    float prevMeas;
    float dFilt;
};

Pid pid1 = {0, 0, 0};
Pid pid2 = {0, 0, 0};

float cmdRpm1 = 0;
float cmdRpm2 = 0;

float rampRpm1 = 0;
float rampRpm2 = 0;

float meas1 = 0;
float meas2 = 0;

int pwm1 = 0;
int pwm2 = 0;

float syncErr = 0;

float targetPos1 = 0;
float targetPos2 = 0;

long prevCount1 = 0;
long prevCount2 = 0;

uint32_t lastControlUs = 0;

unsigned long lastPrint = 0;
unsigned long lastCommandTime = 0;

const unsigned long timeoutMs = 500;

// --------------------------- INTERRUPTS ------------------------------

void IRAM_ATTR isrEnc1() {

    uint8_t s =
        (digitalRead(ENCODER_A) << 1) |
        digitalRead(ENCODER_B);

    encoderCount += QDEC[(enc1State << 2) | s];

    enc1State = s;
}

void IRAM_ATTR isrEnc2() {

    uint8_t s =
        (digitalRead(ENCODER_2A) << 1) |
        digitalRead(ENCODER_2B);

    encoder2Count += QDEC[(enc2State << 2) | s];

    enc2State = s;
}

// ------------------------- MOTOR OUTPUT ------------------------------

void setMotor1(int speed) {

    if (INVERT_MOTOR1)
        speed = -speed;

    digitalWrite(IN1, speed > 0);
    digitalWrite(IN2, speed < 0);

    ledcWrite(ENA, constrain(abs(speed), 0, 1023));
}

void setMotor2(int speed) {

    if (INVERT_MOTOR2)
        speed = -speed;

    digitalWrite(IN3, speed > 0);
    digitalWrite(IN4, speed < 0);

    ledcWrite(ENB, constrain(abs(speed), 0, 1023));
}

// ------------------------- ENCODER READING ----------------------------

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

// ----------------------------- HELPERS --------------------------------

float rampToward(
    float cur,
    float target,
    float maxStep
) {

    if (target > cur)
        return min(cur + maxStep, target);

    return max(cur - maxStep, target);
}

void resetPid(Pid &p, float meas) {

    p.integral = 0;
    p.prevMeas = meas;
    p.dFilt = 0;
}

// --------------------------- PID CONTROL -----------------------------

int pidStep(
    Pid &p,
    float target,
    float meas,
    float dt
) {

    if (target == 0.0f) {

        resetPid(p, meas);
        return 0;
    }

    float err = target - meas;

    float dMeas =
        (meas - p.prevMeas) / dt;

    p.prevMeas = meas;

    p.dFilt =
        0.8f * p.dFilt +
        0.2f * dMeas;

    float ff = KFF * target;

    float unsat =
        ff +
        KP * err +
        KI * p.integral -
        KD * p.dFilt;

    bool saturatedHigh =
        unsat >= 1023.0f && err > 0;

    bool saturatedLow =
        unsat <= -1023.0f && err < 0;

    if (!saturatedHigh && !saturatedLow) {
        p.integral += err * dt;
    }

    float out =
        ff +
        KP * err +
        KI * p.integral -
        KD * p.dFilt;

    return constrain((int)out, -1023, 1023);
}

// --------------------------- CONTROL LOOP ----------------------------

void controlUpdate() {

    uint32_t nowUs = micros();
    uint32_t dtUs = nowUs - lastControlUs;

    if (dtUs < CONTROL_PERIOD_US)
        return;

    lastControlUs = nowUs;

    float dt = dtUs * 1e-6f;

    long c1, c2;
    readCounts(c1, c2);

    long d1 = c1 - prevCount1;
    long d2 = c2 - prevCount2;

    prevCount1 = c1;
    prevCount2 = c2;

    float raw1 =
        d1 / COUNTS_PER_WHEEL_REV / dt * 60.0f;

    float raw2 =
        d2 / COUNTS_PER_WHEEL_REV / dt * 60.0f;

    meas1 = 0.6f * meas1 + 0.4f * raw1;
    meas2 = 0.6f * meas2 + 0.4f * raw2;

    // Smooth acceleration

    float step = ACCEL_RPM_PER_S * dt;

    rampRpm1 =
        rampToward(rampRpm1, cmdRpm1, step);

    rampRpm2 =
        rampToward(rampRpm2, cmdRpm2, step);

    // Wheel position

    float pos1 = c1 / COUNTS_PER_WHEEL_REV;
    float pos2 = c2 / COUNTS_PER_WHEEL_REV;

    if (
        rampRpm1 == 0.0f &&
        rampRpm2 == 0.0f
    ) {

        targetPos1 = pos1;
        targetPos2 = pos2;

        syncErr = 0;

    } else {

        targetPos1 +=
            rampRpm1 * dt / 60.0f;

        targetPos2 +=
            rampRpm2 * dt / 60.0f;
    }

    // Wheel synchronization

    syncErr =
        (pos1 - targetPos1) -
        (pos2 - targetPos2);

    float corr = constrain(
        KSYNC * syncErr,
        -MAX_SYNC_RPM,
        MAX_SYNC_RPM
    );

    float tgt1 = rampRpm1;
    float tgt2 = rampRpm2;

    if (tgt1 != 0.0f)
        tgt1 -= corr;

    if (tgt2 != 0.0f)
        tgt2 += corr;

    pwm1 = pidStep(pid1, tgt1, meas1, dt);
    pwm2 = pidStep(pid2, tgt2, meas2, dt);

    setMotor1(pwm1);
    setMotor2(pwm2);
}

// -------------------------- SERIAL OUTPUT ----------------------------

void printStatus() {

    unsigned long now = millis();

    if (now - lastPrint < PRINT_MS)
        return;

    lastPrint = now;

    Serial.print("tgt ");
    Serial.print(rampRpm1, 1);

    Serial.print("/");
    Serial.print(rampRpm2, 1);

    Serial.print(" | L rpm ");
    Serial.print(meas1, 1);

    Serial.print(" pwm ");
    Serial.print(pwm1);

    Serial.print(" | R rpm ");
    Serial.print(meas2, 1);

    Serial.print(" pwm ");
    Serial.print(pwm2);

    Serial.print(" | sync ");
    Serial.println(syncErr, 3);
}

// ---------------------------- WEB SERVER -----------------------------

void handleRoot() {

    String html = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Rover Control</title>
<meta name="viewport"
content="width=device-width, initial-scale=1, user-scalable=no">

<style>
body {
    margin: 0;
    background: #111;
    color: #fff;
    font-family: sans-serif;
    min-height: 100vh;
    display: flex;
    flex-direction: column;
    align-items: center;
    padding: 20px 16px 40px;
    box-sizing: border-box;
}

h2 { margin: 0 0 20px; }
h3 { margin: 0 0 8px; }

.joystick-container {
    width: 220px;
    height: 220px;
    background: #222;
    border-radius: 50%;
    position: relative;
    touch-action: none;
    border: 2px solid #444;
    flex-shrink: 0;
}

.joystick-handle {
    width: 90px;
    height: 90px;
    background: #4a90e2;
    border-radius: 50%;
    position: absolute;
    top: 65px;
    left: 65px;
    touch-action: none;
}

.label {
    text-align: center;
    margin-top: 12px;
    font-size: 14px;
    color: #aaa;
}

.encoder {
    text-align: center;
    margin-top: 8px;
    font-size: 13px;
    color: #6c6;
}

.arm {
    width: 100%;
    max-width: 360px;
    margin-top: 28px;
    border-top: 1px solid #333;
    padding-top: 20px;
}

.row { margin: 12px 0; }

.row label {
    display: flex;
    justify-content: space-between;
    font-size: 14px;
    color: #ccc;
    margin-bottom: 4px;
}

input[type=range] { width: 100%; }

.buttons {
    display: flex;
    gap: 10px;
    margin-top: 16px;
}

button {
    flex: 1;
    padding: 12px;
    font-size: 15px;
    border: none;
    border-radius: 8px;
    background: #333;
    color: #fff;
}

#stopBtn { background: #a33; }
</style>
</head>

<body>

<h2>Rover Control</h2>

<div class="joystick-container" id="stick">
    <div class="joystick-handle" id="handle"></div>
</div>

<div class="label">
Up/Down = Throttle &nbsp;|&nbsp; Left/Right = Turn
</div>

<div class="encoder" id="encoderReading">
Wheel RPM: 0 | 0
</div>

<div class="arm">

<h3>Arm</h3>

<div id="joints"></div>

<div class="row">
<label>
<span>Speed</span>
<span><span id="speedVal">60</span>&deg;/s</span>
</label>

<input type="range"
id="speed" min="10" max="1000" value="60">
</div>

<div class="buttons">
<button id="homeBtn">Home</button>
<button id="stopBtn">Stop</button>
</div>

<div class="encoder" id="armStatus">
Arm: connecting...
</div>

</div>

<script>

// ---------------- DRIVE CONTROLS ----------------

const container = document.getElementById('stick');
const handle = document.getElementById('handle');
const encoderReading =
    document.getElementById('encoderReading');

const maxDist = 65;

let dragging = false;
let throttle = 0;
let steer = 0;

function updatePosition(clientX, clientY) {

    const rect = container.getBoundingClientRect();

    let dx = clientX - (rect.left + rect.width / 2);
    let dy = clientY - (rect.top + rect.height / 2);

    const dist = Math.sqrt(dx*dx + dy*dy);

    if (dist > maxDist) {
        dx = dx / dist * maxDist;
        dy = dy / dist * maxDist;
    }

    handle.style.left = (65 + dx) + 'px';
    handle.style.top = (65 + dy) + 'px';

    throttle = Math.round((-dy / maxDist) * 1023);
    steer = Math.round((dx / maxDist) * 1023);
}

function reset() {
    handle.style.left = '65px';
    handle.style.top = '65px';

    throttle = 0;
    steer = 0;
}

container.addEventListener('touchstart', () => {
    dragging = true;
});

container.addEventListener('touchmove', (e) => {

    if (!dragging) return;

    const t = e.touches[0];

    updatePosition(t.clientX, t.clientY);
    e.preventDefault();

}, { passive: false });

container.addEventListener('touchend', () => {
    dragging = false;
    reset();
});

container.addEventListener('mousedown', () => {
    dragging = true;
});

window.addEventListener('mousemove', (e) => {
    if (dragging)
        updatePosition(e.clientX, e.clientY);
});

window.addEventListener('mouseup', () => {
    if (dragging) {
        dragging = false;
        reset();
    }
});

setInterval(() => {

    fetch(`/move?throttle=${throttle}&steer=${steer}`);

}, 150);

setInterval(() => {

    fetch('/encoder')
        .then(r => r.text())
        .then(t => {
            encoderReading.textContent =
                'Wheel RPM: ' + t;
        });

}, 250);

// ---------------- ARM CONTROLS ----------------

const ARM = 'http://192.168.4.200';

const JOINTS = [
    'Base',
    'Shoulder',
    'Elbow',
    'Gripper'
];

const HOME = [90, 90, 90, 90];

const armStatus = document.getElementById('armStatus');

const sliders = [];

let armTimer = null;
let syncSliders = true;

function armGet(path) {
    return fetch(ARM + path).catch(() => {});
}

function queueArmUpdate() {

    if (armTimer) return;

    armTimer = setTimeout(() => {

        armTimer = null;

        armGet(
            '/armall?a=' +
            sliders.map(s => s.value).join(',')
        );

    }, 50);
}

JOINTS.forEach((name, i) => {

    const row = document.createElement('div');

    row.className = 'row';

    row.innerHTML = `
        <label>
            <span>${name}</span>
            <span>
                <span id="jv${i}">${HOME[i]}</span>&deg;
            </span>
        </label>
        <input type="range" min="0" max="180"
               value="${HOME[i]}">
    `;

    document.getElementById('joints').appendChild(row);

    const s = row.querySelector('input');

    sliders.push(s);

    s.addEventListener('input', () => {

        document.getElementById('jv' + i).textContent = s.value;

        queueArmUpdate();
    });
});

const speed = document.getElementById('speed');

speed.addEventListener('input', () => {

    document.getElementById('speedVal').textContent =
        speed.value >= 1000 ? 'Max ' : speed.value;
});

speed.addEventListener('change', () => {
    armGet('/speed?v=' + speed.value);
});

document.getElementById('homeBtn').addEventListener('click', () => {

    armGet('/home');

    sliders.forEach((s, i) => {
        s.value = HOME[i];

        document.getElementById('jv' + i).textContent =
            HOME[i];
    });
});

document.getElementById('stopBtn').addEventListener('click', () => {

    armGet('/stop');

    syncSliders = true;
});

setInterval(() => {

    fetch(ARM + '/status')

        .then(r => r.json())

        .then(s => {

            if (!s.ok) {

                armStatus.textContent =
                    'Arm: ESP32 online, PCA9685 not responding';

                return;
            }

            let text =
                'Arm: ' + (s.moving ? 'moving' : 'ready');

            if (s.clamped)
                text += ' (angle limited)';

            armStatus.textContent =
                text + ' | ' + s.angles.join(', ');

            if (syncSliders && !s.moving) {

                s.angles.forEach((a, i) => {

                    sliders[i].value = a;

                    document.getElementById('jv' + i)
                        .textContent = a;
                });

                syncSliders = false;
            }
        })

        .catch(() => {
            armStatus.textContent = 'Arm: offline';
        });

}, 500);

</script>
</body>
</html>
)HTML";

    server.send(200, "text/html", html);
}

// -------------------------- MOVE HANDLER -----------------------------

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

        cmdRpm1 =
            left / 1023.0f * MAX_WHEEL_RPM;

        cmdRpm2 =
            right / 1023.0f * MAX_WHEEL_RPM;

        lastCommandTime = millis();
    }

    server.send(200, "text/plain", "OK");
}

// ------------------------- ENCODER HANDLER ---------------------------

void handleEncoder() {

    server.send(
        200,
        "text/plain",
        String(meas1, 0) + " | " + String(meas2, 0)
    );
}

// ------------------------------ SETUP --------------------------------

void setup() {

    Serial.begin(9600);

    // Motor direction pins

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    // PWM output pins

    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    // ESP32 PWM initialization
    // 20 kHz, 10-bit resolution

    bool pwmAReady = ledcAttach(
        ENA,
        PWM_FREQUENCY_HZ,
        PWM_RESOLUTION_BITS
    );

    bool pwmBReady = ledcAttach(
        ENB,
        PWM_FREQUENCY_HZ,
        PWM_RESOLUTION_BITS
    );

    if (!pwmAReady || !pwmBReady) {

        Serial.println(
            "ERROR: Motor PWM initialization failed"
        );

        while (true) {
            delay(1000);
        }
    }

    setMotor1(0);
    setMotor2(0);

    // Encoder pins

    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);

    pinMode(ENCODER_2A, INPUT);
    pinMode(ENCODER_2B, INPUT);

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

    // Wi-Fi access point

    WiFi.softAP("Rover-AP", "12345678");

    Serial.println();

    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    // Web server routes

    server.on("/", handleRoot);
    server.on("/move", handleMove);
    server.on("/encoder", handleEncoder);

    server.begin();

    Serial.println(
        AUTO_TEST
        ? "AUTO TEST mode: motors start in 3 s"
        : "Joystick mode"
    );

    lastControlUs = micros();
}

// ------------------------------- LOOP --------------------------------

void loop() {

    server.handleClient();

    if (AUTO_TEST) {

        unsigned long t = millis();

        bool on =
            t >= START_DELAY_MS &&
            t < START_DELAY_MS + RUN_DURATION_MS;

        cmdRpm1 = on ? TEST_RPM : 0;
        cmdRpm2 = on ? TEST_RPM : 0;

    } else if (
        millis() - lastCommandTime > timeoutMs
    ) {

        cmdRpm1 = 0;
        cmdRpm2 = 0;
    }

    controlUpdate();
    printStatus();
}
