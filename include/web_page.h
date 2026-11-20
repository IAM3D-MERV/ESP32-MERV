
#pragma once
#include <Arduino.h>

static const char WEB_PAGE[] PROGMEM = R"HTML(
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

    sendMove(false);
}

function reset() {
    handle.style.left = '65px';
    handle.style.top = '65px';

    throttle = 0;
    steer = 0;

    sendMove(true);
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

// Send joystick commands without a backlog: only one /move request is in
// flight at a time, and the newest joystick value is always the one sent.
// Requests piling up behind a slow one (the old fixed-interval fetch)
// are what caused the lag.
let moveBusy = false;
let lastSent = '';
let lastSentTime = 0;

function sendMove(force) {

    const q = `throttle=${throttle}&steer=${steer}`;
    const now = Date.now();

    if (moveBusy) return;

    // Send changes immediately; otherwise keepalive every 100 ms
    // (firmware stops the motors after 500 ms without a command)
    if (!force && q === lastSent && now - lastSentTime < 100) return;

    moveBusy = true;
    lastSent = q;
    lastSentTime = now;

    // Give up on a request after 300 ms. Without this, one request that
    // never gets an answer (Wi-Fi hiccup, ESP32 busy) leaves moveBusy
    // stuck at true and the joystick stops sending forever.
    const ctrl = (typeof AbortController !== 'undefined')
        ? new AbortController() : null;

    const timer = setTimeout(() => {
        if (ctrl) ctrl.abort();
        done();
    }, 300);

    let finished = false;

    function done() {
        if (finished) return;
        finished = true;
        clearTimeout(timer);
        moveBusy = false;
        // Joystick moved while waiting: send the new value now
        if (`throttle=${throttle}&steer=${steer}` !== lastSent)
            sendMove(true);
    }

    fetch('/move?' + q, ctrl ? { signal: ctrl.signal } : {})
        .then(done, done);
}

setInterval(() => sendMove(false), 20);

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