// =====================================================================
// 2.5D LiDAR Mapper v2.1 — standalone ESP32 + phone viewer
//
// ESP32: RPLidar C1 (UART2) + NEMA17/DRV8825 pan axis.
//   - Stepper pulses generated in hardware (FastAccelStepper) -> smooth, jitter-free
//   - Lidar read in its own FreeRTOS task -> every sample tagged with the right angle
//   - Dwell = N COMPLETE lidar rotations per pan angle (not a guessed time)
//   - "Scan once" mode, optional home switch, set-zero
// Phone (browser): WebGL viewer with Eye-Dome Lighting, live calibration,
//   measuring, floor-plan view, clipping, scan library, PLY export,
//   and multi-spot merging (scan from several positions, auto-aligned with ICP).
//
// Libraries (Arduino Library Manager):
//   "WebSockets" by Markus Sattler
//   "FastAccelStepper" by gin66
// Open http://lidar.local (iPhone) or the printed IP / 192.168.4.1 (AP mode).
// =====================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include "FastAccelStepper.h"

// ---------- Network — EDIT THESE ----------
const bool USE_AP_MODE = true;             // false = join phone hotspot, true = ESP32 hosts "LidarRig"
const char* WIFI_SSID = "YOUR_HOTSPOT_NAME";
const char* WIFI_PASSWORD = "YOUR_HOTSPOT_PASSWORD";
const char* AP_SSID = "LidarRig";
const char* AP_PASSWORD = "scanner123";
const char* MDNS_NAME = "lidar";

WebServer server(80);
WebSocketsServer webSocket(81);

// ---------- Stepper (DRV8825) ----------
const int STEP_PIN = 25;
const int DIR_PIN = 26;
const int HOME_PIN = -1;                   // GPIO of a microswitch to GND, or -1 if none

const int STEPS_PER_REV = 200;
const int MICROSTEP = 16;                  // must match MS1/MS2/MS3 wiring
const float STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEP) / 360.0f;

const uint32_t MAX_SPEED_HZ = 800;         // steps/s cruise speed
const int32_t ACCEL_STEPS_S2 = 2000;       // steps/s^2 — lower = gentler starts/stops
const uint32_t HOMING_SPEED_HZ = 300;

FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;

// ---------- Sweep parameters (editable from phone) ----------
int panStart = 0;
int panEnd = 180;                          // lidar sees a full vertical circle: 0-180 covers the whole sphere
float panStep = 0.5f;
uint8_t dwellRotations = 2;                // complete lidar rotations collected per pan angle
unsigned long settleMs = 50;               // wait after stopping before collecting (vibration damping)

enum Mode { MODE_MANUAL, MODE_AUTO, MODE_ONCE };
enum SweepState { ST_MOVING, ST_SETTLING, ST_COLLECTING, ST_HOLD };

Mode mode = MODE_MANUAL;
SweepState state = ST_SETTLING;
unsigned long stateSince = 0;
float targetDeg = 0;                       // angle currently commanded
float manualTargetDeg = 0;
float sweepTargetDeg = 0;                  // exact float sweep position (no rounding drift)
int panDirection = 1;
uint32_t sweepPass = 0;
uint32_t lidarTimeouts = 0;
bool scanComplete = false;
bool homed = false;

// ---------- Lidar (UART2, own task) ----------
HardwareSerial LidarSerial(2);
const int LIDAR_RX_PIN = 16;
const int LIDAR_TX_PIN = 17;
const long LIDAR_BAUD = 460800;

struct __attribute__((packed)) PhonePoint {
  float lidar_angle_deg;
  uint16_t distance_mm;
  uint8_t quality;
  uint8_t reserved;
  float pan_angle_deg;
};  // 12 bytes

QueueHandle_t sampleQueue;

// Shared between lidar task and main loop
volatile bool g_collectEnabled = false;
volatile bool g_collecting = false;
volatile bool g_unlimited = true;
volatile bool g_lidarRestart = true;      // task starts the lidar on boot
volatile bool g_lidarOk = false;
volatile uint8_t g_dwellRot = 2;
volatile uint16_t g_rotDone = 0;
volatile float g_collectPan = 0;
volatile uint32_t g_samplesRead = 0;
volatile uint32_t g_rejected = 0;
volatile uint32_t g_dropped = 0;
volatile uint32_t g_rotSeen = 0;

// ---------- WebSocket batching ----------
const int WS_BATCH = 32;
PhonePoint wsBatch[WS_BATCH];
int wsCount = 0;
unsigned long lastWsFlush = 0;

unsigned long bootTime = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastStats = 0;
uint32_t lastRotSeen = 0;
uint32_t lidarHz = 0;

// =====================================================================
// Lidar protocol (runs inside the lidar task only)
// =====================================================================
void sendLidarCommand(uint8_t cmd) {
  LidarSerial.write(0xA5);
  LidarSerial.write(cmd);
}

bool readResponseDescriptor() {
  uint8_t buf[7];
  int idx = 0;
  unsigned long start = millis();
  while (idx < 7) {
    while (LidarSerial.available() && idx < 7) buf[idx++] = LidarSerial.read();
    if (millis() - start > 2000) return false;
    vTaskDelay(1);
  }
  return buf[0] == 0xA5 && buf[1] == 0x5A;
}

bool startLidarScan() {
  sendLidarCommand(0x25);                 // STOP any previous stream
  vTaskDelay(pdMS_TO_TICKS(20));
  while (LidarSerial.available()) LidarSerial.read();

  for (int attempt = 1; attempt <= 3; attempt++) {
    sendLidarCommand(0x20);               // SCAN
    if (readResponseDescriptor()) {
      Serial.println("Lidar scan started");
      return true;
    }
    Serial.printf("Lidar start failed (attempt %d), retrying...\n", attempt);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  Serial.println("WARNING: lidar never confirmed scan start");
  return false;
}

// startFlag = true on the first sample of each new 360 deg lidar rotation
bool readScanSample(float &angle_deg, uint16_t &distance_mm, uint8_t &quality, bool &startFlag) {
  static uint8_t packet[5];
  static int index = 0;

  while (LidarSerial.available()) {
    uint8_t b = LidarSerial.read();

    if (index == 0) {
      uint8_t s = b & 0x01;
      uint8_t sn = (b >> 1) & 0x01;
      if (s == sn) continue;
      packet[0] = b;
      index = 1;
      continue;
    }

    packet[index++] = b;
    if (index < 5) continue;
    index = 0;

    uint8_t S = packet[0] & 0x01;
    uint8_t Sn = (packet[0] >> 1) & 0x01;
    uint8_t C = packet[1] & 0x01;
    if (S == Sn || C == 0) continue;

    startFlag = (S == 1);
    quality = packet[0] >> 2;
    uint16_t angle_q6 = ((uint16_t)packet[2] << 7) | ((uint16_t)packet[1] >> 1);
    angle_deg = angle_q6 / 64.0f;
    uint16_t distance_q2 = ((uint16_t)packet[4] << 8) | packet[3];
    distance_mm = distance_q2 / 4;
    return true;
  }
  return false;
}

void lidarTask(void* param) {
  float a;
  uint16_t d;
  uint8_t q;
  bool startFlag;

  for (;;) {
    if (g_lidarRestart) {
      g_lidarRestart = false;
      g_lidarOk = startLidarScan();
    }

    while (readScanSample(a, d, q, startFlag)) {
      g_samplesRead++;
      if (startFlag) g_rotSeen++;

      if (!g_collectEnabled) {             // stepper moving or settling
        g_collecting = false;
        g_rejected++;
        continue;
      }

      if (startFlag) {
        if (!g_collecting) { g_collecting = true; g_rotDone = 0; }   // begin on a rotation boundary
        else g_rotDone++;
      }
      if (!g_collecting) continue;                                  // wait for rotation start
      if (!g_unlimited && g_rotDone >= g_dwellRot) continue;        // dwell finished

      if (d < 50 || d > 12000 || q < 5) continue;

      PhonePoint p;
      p.lidar_angle_deg = a;
      p.distance_mm = d;
      p.quality = q;
      p.reserved = 0;
      p.pan_angle_deg = g_collectPan;
      if (xQueueSend(sampleQueue, &p, 0) != pdTRUE) g_dropped++;
    }
    vTaskDelay(1);                          // drain UART every ~1 ms
  }
}

// =====================================================================
// Sweep control (main loop)
// =====================================================================
float currentDeg() {
  return stepper->getCurrentPosition() / STEPS_PER_DEGREE;
}

void startMoveTo(float deg) {
  g_collectEnabled = false;
  targetDeg = deg;
  stepper->moveTo(lroundf(deg * STEPS_PER_DEGREE));
  state = ST_MOVING;
  stateSince = millis();
}

float nextSweepAngle() {
  float next = sweepTargetDeg + panStep * panDirection;
  if (mode == MODE_ONCE) {
    if (next > panEnd) next = panEnd;
  } else {
    if (next >= panEnd) { next = panEnd; panDirection = -1; sweepPass++; }
    else if (next <= panStart) { next = panStart; panDirection = 1; sweepPass++; }
  }
  sweepTargetDeg = next;
  return next;
}

void updateSweep() {
  unsigned long now = millis();
  g_unlimited = (mode == MODE_MANUAL);

  if (mode == MODE_MANUAL && fabsf(manualTargetDeg - targetDeg) > 0.01f) {
    startMoveTo(manualTargetDeg);
    return;
  }

  switch (state) {
    case ST_MOVING:
      if (!stepper->isRunning()) { state = ST_SETTLING; stateSince = now; }
      break;

    case ST_SETTLING:
      if (now - stateSince >= settleMs) {
        g_collectPan = currentDeg();        // actual stopped position
        g_rotDone = 0;
        g_collecting = false;
        g_dwellRot = dwellRotations;
        g_collectEnabled = true;
        state = ST_COLLECTING;
        stateSince = now;
      }
      break;

    case ST_COLLECTING: {
      if (mode == MODE_MANUAL) break;       // manual: keep collecting at this angle
      bool done = g_rotDone >= dwellRotations;
      bool timeout = (now - stateSince) > (unsigned long)dwellRotations * 300UL + 1500UL;
      if (!done && !timeout) break;
      if (timeout && !done) lidarTimeouts++;

      if (mode == MODE_ONCE && sweepTargetDeg >= panEnd - 0.001f) {
        g_collectEnabled = false;
        mode = MODE_MANUAL;
        manualTargetDeg = targetDeg;
        scanComplete = true;
        state = ST_HOLD;
        Serial.println("Single scan complete");
        break;
      }
      startMoveTo(nextSweepAngle());
      break;
    }

    case ST_HOLD:
      break;
  }
}

void homeStepper() {
  if (HOME_PIN < 0) return;
  pinMode(HOME_PIN, INPUT_PULLUP);
  Serial.println("Homing...");
  stepper->setSpeedInHz(HOMING_SPEED_HZ);
  stepper->runBackward();
  unsigned long t0 = millis();
  while (digitalRead(HOME_PIN) == HIGH && millis() - t0 < 20000) delay(1);
  stepper->forceStopAndNewPosition(0);
  stepper->setSpeedInHz(MAX_SPEED_HZ);
  homed = (digitalRead(HOME_PIN) == LOW);
  Serial.println(homed ? "Homed at 0 deg" : "Homing FAILED (switch not hit)");
}

// =====================================================================
// Streaming to phone
// =====================================================================
void flushWs() {
  if (wsCount == 0) return;
  webSocket.broadcastBIN((uint8_t*)wsBatch, wsCount * sizeof(PhonePoint));
  wsCount = 0;
  lastWsFlush = millis();
}

void drainSamples() {
  PhonePoint p;
  bool hasClients = webSocket.connectedClients() > 0;
  int n = 0;
  while (n < 256 && xQueueReceive(sampleQueue, &p, 0) == pdTRUE) {
    n++;
    if (!hasClients) continue;
    wsBatch[wsCount++] = p;
    if (wsCount >= WS_BATCH) flushWs();
  }
  if (wsCount > 0 && millis() - lastWsFlush > 60) flushWs();
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) Serial.printf("Viewer #%u connected\n", num);
  else if (type == WStype_DISCONNECTED) Serial.printf("Viewer #%u disconnected\n", num);
}

// =====================================================================
// Web page (viewer + controls)
// =====================================================================
const char* PAGE_HTML = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="mobile-web-app-capable" content="yes">
<title>LiDAR Viewer</title>
<style>
html,body{margin:0;height:100%;background:#0b0d10;color:#ddd;font-family:-apple-system,system-ui,sans-serif;overflow:hidden}
#gl{position:fixed;left:0;top:0;width:100vw;height:100vh;touch-action:none;display:block}
#hud{position:fixed;top:8px;left:8px;font-size:12px;background:rgba(0,0,0,.6);padding:6px 8px;border-radius:6px;line-height:1.5;pointer-events:none;z-index:1;max-width:60vw}
#bar{position:fixed;left:0;bottom:0;height:4px;background:#8bc34a;width:0;z-index:1;transition:width .3s}
#toggle{position:fixed;top:8px;right:8px;z-index:3}
#quick{position:fixed;bottom:12px;right:8px;z-index:1;display:flex;gap:4px}
#panel{position:fixed;right:0;top:0;bottom:0;width:min(330px,90vw);background:rgba(20,22,26,.97);padding:52px 14px 30px;box-sizing:border-box;overflow-y:auto;transform:translateX(100%);transition:transform .2s;z-index:2}
#panel.open{transform:none}
button{background:#2e7d32;color:#fff;border:0;border-radius:6px;padding:8px 10px;margin:2px;font-size:14px}
button.alt{background:#37474f}
button.on{background:#f9a825;color:#111}
.row{display:flex;justify-content:space-between;align-items:center;margin:6px 0;font-size:14px;gap:6px;flex-wrap:wrap}
input[type=number]{width:72px;background:#222;color:#eee;border:1px solid #444;border-radius:4px;padding:5px;font-size:14px}
input[type=range]{flex:1;min-width:120px}
select{background:#222;color:#eee;border:1px solid #444;border-radius:4px;padding:5px;font-size:14px}
h3{margin:18px 0 6px;font-size:14px;color:#8bc34a;border-top:1px solid #333;padding-top:10px}
.hint{font-size:12px;color:#999;line-height:1.4}
.scan{display:flex;justify-content:space-between;align-items:center;font-size:13px;padding:4px 0;border-bottom:1px solid #2a2a2a}
#status{font-size:13px;line-height:1.6}
.st{display:flex;justify-content:space-between;align-items:center;font-size:13px;padding:3px 4px;border-bottom:1px solid #2a2a2a;border-radius:4px}
.st.sel{background:#263238}
.sw{display:inline-block;width:10px;height:10px;border-radius:5px;margin-right:6px}
button:disabled{opacity:.4}
#quick button{padding:8px 9px}
</style>
</head>
<body>
<canvas id="gl"></canvas>
<div id="hud">Connecting...</div>
<div id="bar"></div>
<button id="toggle" class="alt">&#9776; Controls</button>
<div id="quick">
  <button id="qScan">Scan</button>
  <button id="qStation" class="alt">+ Station</button>
  <button id="qMeasure" class="alt">Measure</button>
  <button id="qTop" class="alt">Top</button>
  <button id="qFit" class="alt">Fit</button>
</div>
<div id="panel">
  <h3 style="border:0;padding:0;margin-top:0">Scan</h3>
  <div class="row"><button id="scanBtn">Scan once</button><button id="autoBtn" class="alt">Continuous</button><button id="manualBtn" class="alt">Stop / manual</button></div>
  <div class="row"><span>Pan angle <b id="angleVal">--</b></span></div>
  <div class="row"><input type="range" id="angleSlider" min="0" max="360" step="0.5" value="0"></div>
  <div class="row"><span>Pan start / end (deg)</span><span><input type="number" id="panStart"> <input type="number" id="panEnd"></span></div>
  <div class="row"><span>Pan step (deg)</span><input type="number" id="panStep" step="0.1"></div>
  <div class="row"><span>Rotations per angle</span><input type="number" id="rot" min="1" max="20"></div>
  <div class="row"><span>Settle time (ms)</span><input type="number" id="settle" step="10"></div>
  <div class="row"><button id="applyBtn">Apply</button><button id="zeroBtn" class="alt">Set zero here</button><button id="lidarBtn" class="alt">Restart lidar</button></div>
  <div class="hint">More rotations per angle = denser, cleaner scan but slower. 0&ndash;180&deg; already covers the full sphere.</div>

  <h3>Merge scans from several spots</h3>
  <div class="row"><button id="addStBtn">Add scan as station</button><span id="stInfo">No stations</span></div>
  <div class="row"><span>Auto-add each finished scan</span><input type="checkbox" id="autoAdd"></div>
  <div id="stList"></div>
  <div class="row"><span>Selected:</span>
    <span><button class="alt rotb" data-rot="5">&#10226;5&deg;</button><button class="alt rotb" data-rot="1">&#10226;1&deg;</button><button class="alt rotb" data-rot="-1">&#10227;1&deg;</button><button class="alt rotb" data-rot="-5">&#10227;5&deg;</button><button class="alt rotb" data-rot="180">180&deg;</button></span></div>
  <div class="row"><button id="moveStBtn" class="alt">Move by hand: off</button><button id="snapBtn" class="alt">Snap into place</button><button id="reautoBtn" class="alt">Auto-align again</button></div>
  <div class="row"><button id="clearStBtn" class="alt">Remove all stations</button></div>
  <div class="hint">1) Scan, then <b>Add scan as station</b>. 2) Move the scanner to a new spot that still sees at least half of what you already scanned (doorways are ideal), scan, add again. Each new station aligns itself automatically. Set Color to <b>By station</b> to check the overlap. If one lands wrong: tap it in the list, rotate or drag it roughly into place (Move by hand: one finger drags, two fingers twist), then <b>Snap into place</b>.</div>

  <h3>View</h3>
  <div class="row"><button id="pauseBtn" class="alt">Pause</button><button id="clearBtn" class="alt">Clear live</button><button id="fitBtn" class="alt">Fit</button><button id="topBtn" class="alt">Top view</button></div>
  <div class="row"><span>Color</span><select id="color"><option value="height">Height</option><option value="quality">Signal quality</option><option value="white">White (best with EDL)</option><option value="station">By station</option></select></div>
  <div class="row"><span>Eye-Dome Lighting</span><input type="checkbox" id="edl"></div>
  <div class="row"><span>EDL strength</span><input type="range" id="edlStr" min="0.1" max="2.5" step="0.1"></div>
  <div class="row"><span>Point size</span><input type="range" id="psize" min="1" max="8" step="0.5"></div>

  <h3>Clip &amp; floor plan</h3>
  <div class="row"><span>Hide below / above (m)</span><span><input type="number" id="clipLo" step="0.1"> <input type="number" id="clipHi" step="0.1"></span></div>
  <div class="row"><button id="floorBtn">Floor plan</button><button id="noClipBtn" class="alt">Show all</button></div>
  <div class="hint">Heights are relative to the lidar. Floor plan slices a wall band above the detected floor and shows it from the top.</div>

  <h3>Measure</h3>
  <div class="row"><button id="measureBtn" class="alt">Measure: off</button><span id="measureVal">--</span></div>
  <div class="hint">Turn on, then tap two points.</div>

  <h3>Filters</h3>
  <div class="row"><span>Voxel filter</span><select id="voxel"><option value="0">Off (raw)</option><option value="0.005">5 mm</option><option value="0.01">1 cm</option><option value="0.02">2 cm</option><option value="0.05">5 cm</option></select></div>
  <div class="row"><span>Min quality</span><input type="number" id="minq" min="0" max="63"></div>
  <div class="row"><span>Max range (m)</span><input type="number" id="maxr" step="0.5"></div>
  <div class="hint">The voxel filter keeps the best-quality point in each cube, so repeated passes clean the scan instead of piling up noise.</div>

  <h3>Calibration</h3>
  <div class="row"><span>Lidar angle offset <b id="thOffV"></b></span><input type="range" id="thetaOff" min="-10" max="10" step="0.1"></div>
  <div class="row"><span>Side offset <b id="sideV"></b></span><input type="range" id="sideOff" min="-0.1" max="0.1" step="0.001"></div>
  <div class="row"><span>Forward offset <b id="fwdV"></b></span><input type="range" id="fwdOff" min="-0.1" max="0.1" step="0.001"></div>
  <div class="row"><span>Flip upside down</span><input type="checkbox" id="flip"></div>
  <div class="row"><button id="calResetBtn" class="alt">Reset calibration</button></div>
  <div class="hint">Look at a flat wall. If it appears doubled or tilted, drag angle offset first, then side offset, until it becomes one thin surface. Applies live to the whole scan.</div>

  <h3>Saved scans (on this phone)</h3>
  <div class="row"><button id="saveBtn">Save to phone</button><button id="plyBtn" class="alt">Export .ply</button></div>
  <div id="library" class="hint">--</div>

  <h3>Status</h3>
  <div id="status">--</div>
</div>

<script>
(function () {
'use strict';
var MAX = 400000;
var DEG = Math.PI / 180;
var BG = [0.043, 0.051, 0.063];
var $ = function (id) { return document.getElementById(id); };

var canvas = $('gl');
var gl = canvas.getContext('webgl2', { antialias: false });
var isGL2 = !!gl;
if (!gl) gl = canvas.getContext('webgl', { antialias: false });
if (!gl) { $('hud').textContent = 'WebGL not supported on this browser'; return; }

// ======================= Settings =======================
var DEFAULTS = { psize: 2, voxel: 0.01, minq: 10, maxr: 12, flip: false, color: 'height',
                 edl: true, edlStr: 0.8, thetaOff: 0, sideOff: 0, fwdOff: 0, clipLo: -99, clipHi: 99, autoAdd: false };
var S = {};
for (var dk in DEFAULTS) S[dk] = DEFAULTS[dk];
try {
  var sv = JSON.parse(localStorage.getItem('lidarViewer2') || '{}');
  for (var sk in sv) if (sk in S) S[sk] = sv[sk];
} catch (e) {}
if (!isGL2) S.edl = false;
function saveS() { try { localStorage.setItem('lidarViewer2', JSON.stringify(S)); } catch (e) {} }

// ======================= Shaders =======================
var PT_VS = [
  'attribute vec4 aRaw;',
  'uniform mat4 uMVP;',
  'uniform float uSize, uZLo, uZHi, uQMax, uFlip, uThOff, uSide, uFwd, uClipLo, uClipHi, uMode, uHi;',
  'uniform vec4 uPose;',
  'uniform vec3 uStCol;',
  'varying vec3 vColor;',
  'vec3 jet(float t){ return clamp(vec3(1.5-abs(4.0*t-3.0), 1.5-abs(4.0*t-2.0), 1.5-abs(4.0*t-1.0)), 0.0, 1.0); }',
  'void main(){',
  '  float th = (aRaw.x + uThOff) * 0.0174532925;',
  '  float ps = aRaw.z * 0.0174532925;',
  '  float xl = aRaw.y * cos(th) + uFwd;',
  '  float zl = aRaw.y * sin(th) * uFlip;',
  '  float c = cos(ps);',
  '  float s = sin(ps);',
  '  vec3 p = vec3(xl * c - uSide * s, xl * s + uSide * c, zl);',
  '  float pc = cos(uPose.x);',
  '  float pn = sin(uPose.x);',
  '  p = vec3(pc * p.x - pn * p.y + uPose.y, pn * p.x + pc * p.y + uPose.z, p.z + uPose.w);',
  '  gl_PointSize = uSize;',
  '  vColor = vec3(0.0);',
  '  if (p.z < uClipLo || p.z > uClipHi) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }',
  '  gl_Position = uMVP * vec4(p, 1.0);',
  '  if (uMode < 0.5) vColor = jet(clamp((p.z - uZLo) / max(uZHi - uZLo, 0.001), 0.0, 1.0));',
  '  else if (uMode < 1.5) vColor = jet(clamp(aRaw.w / max(uQMax, 1.0), 0.0, 1.0));',
  '  else if (uMode < 2.5) vColor = vec3(0.92, 0.92, 0.88);',
  '  else vColor = uStCol * (0.45 + 0.55 * clamp((p.z - uZLo) / max(uZHi - uZLo, 0.001), 0.0, 1.0));',
  '  if (uHi > 0.5) vColor = mix(vColor, vec3(1.0, 0.55, 0.1), 0.65);',
  '}'].join('\n');
var PT_FS = [
  'precision mediump float;',
  'varying vec3 vColor;',
  'void main(){ vec2 c = gl_PointCoord - vec2(0.5); if (dot(c, c) > 0.25) discard; gl_FragColor = vec4(vColor, 1.0); }'
].join('\n');
var LN_VS = [
  'attribute vec3 aPos;', 'attribute vec3 aCol;', 'uniform mat4 uMVP;', 'varying vec3 vColor;',
  'void main(){ gl_Position = uMVP * vec4(aPos, 1.0); vColor = aCol; }'].join('\n');
var LN_FS = [
  'precision mediump float;', 'varying vec3 vColor;',
  'void main(){ gl_FragColor = vec4(vColor, 1.0); }'].join('\n');
var Q_VS = [
  '#version 300 es', 'in vec2 aPos;', 'out vec2 vUv;',
  'void main(){ vUv = aPos * 0.5 + 0.5; gl_Position = vec4(aPos, 0.0, 1.0); }'].join('\n');
var EDL_FS = [
  '#version 300 es',
  'precision highp float;',
  'uniform sampler2D uColor;',
  'uniform sampler2D uDepth;',
  'uniform vec2 uTexel;',
  'uniform float uNear, uFar, uOrtho, uStrength, uRadius;',
  'uniform vec3 uBg;',
  'in vec2 vUv;',
  'out vec4 outColor;',
  'float logLin(float d){',
  '  float lin = uOrtho > 0.5 ? uNear + d * (uFar - uNear)',
  '            : (2.0 * uNear * uFar) / (uFar + uNear - (d * 2.0 - 1.0) * (uFar - uNear));',
  '  return log2(lin);',
  '}',
  'void main(){',
  '  float d0 = texture(uDepth, vUv).r;',
  '  bool empty0 = d0 >= 1.0;',
  '  float l0 = empty0 ? 0.0 : logLin(d0);',
  '  float sum = 0.0;',
  '  for (int i = 0; i < 8; i++) {',
  '    float a = float(i) * 0.7853982;',
  '    vec2 uv = vUv + vec2(cos(a), sin(a)) * uTexel * uRadius;',
  '    float dn = texture(uDepth, uv).r;',
  '    if (dn < 1.0) sum += empty0 ? 100.0 : max(0.0, l0 - logLin(dn));',
  '  }',
  '  float shade = exp(-(sum / 8.0) * 300.0 * uStrength);',
  '  vec3 base = empty0 ? uBg : texture(uColor, vUv).rgb;',
  '  outColor = vec4(base * shade, 1.0);',
  '}'].join('\n');

function compile(type, src) {
  var sh = gl.createShader(type);
  gl.shaderSource(sh, src);
  gl.compileShader(sh);
  if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(sh));
  return sh;
}
function makeProgram(vs, fs, attribs) {
  var p = gl.createProgram();
  gl.attachShader(p, compile(gl.VERTEX_SHADER, vs));
  gl.attachShader(p, compile(gl.FRAGMENT_SHADER, fs));
  attribs.forEach(function (name, i) { gl.bindAttribLocation(p, i, name); });
  gl.linkProgram(p);
  if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p));
  return p;
}
function uniforms(p, names) {
  var u = {};
  names.forEach(function (n) { u[n] = gl.getUniformLocation(p, n); });
  return u;
}

var ptProg = makeProgram(PT_VS, PT_FS, ['aRaw']);
var PU = uniforms(ptProg, ['uMVP', 'uSize', 'uZLo', 'uZHi', 'uQMax', 'uFlip', 'uThOff', 'uSide', 'uFwd', 'uClipLo', 'uClipHi', 'uMode', 'uHi', 'uPose', 'uStCol']);
var lnProg = makeProgram(LN_VS, LN_FS, ['aPos', 'aCol']);
var LU = uniforms(lnProg, ['uMVP']);
var edlProg = null, EU = null;
if (isGL2) {
  try {
    edlProg = makeProgram(Q_VS, EDL_FS, ['aPos']);
    EU = uniforms(edlProg, ['uColor', 'uDepth', 'uTexel', 'uNear', 'uFar', 'uOrtho', 'uStrength', 'uRadius', 'uBg']);
  } catch (e) { edlProg = null; S.edl = false; }
}

// ======================= Static geometry =======================
var grid = [];
for (var g = -5; g <= 5; g++) {
  var gc = g === 0 ? 0.35 : 0.2;
  grid.push(g, -5, 0, gc, gc, gc, g, 5, 0, gc, gc, gc);
  grid.push(-5, g, 0, gc, gc, gc, 5, g, 0, gc, gc, gc);
}
grid.push(0, 0, 0, 1, 0.2, 0.2, 0.5, 0, 0, 1, 0.2, 0.2);
grid.push(0, 0, 0, 0.2, 1, 0.2, 0, 0.5, 0, 0.2, 1, 0.2);
grid.push(0, 0, 0, 0.3, 0.5, 1, 0, 0, 0.5, 0.3, 0.5, 1);
var gridData = new Float32Array(grid);
var gridVbo = gl.createBuffer();
gl.bindBuffer(gl.ARRAY_BUFFER, gridVbo);
gl.bufferData(gl.ARRAY_BUFFER, gridData, gl.STATIC_DRAW);

var quadVbo = gl.createBuffer();
gl.bindBuffer(gl.ARRAY_BUFFER, quadVbo);
gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]), gl.STATIC_DRAW);

var measVbo = gl.createBuffer();
var measVerts = 0;

// ======================= Point storage =======================
// raw: [lidarAngleDeg, range_m, panAngleDeg, quality] per point — the GPU computes xyz,
// so calibration sliders re-shape the whole scan instantly.
var raw = new Float32Array(MAX * 4);
var xyz = new Float32Array(MAX * 3);         // CPU copy for picking, filters, export
var slotKey = new Float64Array(MAX).fill(-1);
var vox = new Map();
var count = 0, writeIdx = 0, pending = 0, replLo = MAX, replHi = 0, fullUpload = false;
var vbo = gl.createBuffer();
gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
gl.bufferData(gl.ARRAY_BUFFER, raw.byteLength, gl.DYNAMIC_DRAW);

var zLo = -1, zHi = 2, qMax = 47, floorZ = -1, bounds = null;
var tmp = [0, 0, 0];

function toXYZ(thDeg, r, psDeg, out) {
  var th = (thDeg + S.thetaOff) * DEG, ps = psDeg * DEG;
  var xl = r * Math.cos(th) + S.fwdOff;
  var zl = r * Math.sin(th) * (S.flip ? -1 : 1);
  var c = Math.cos(ps), s = Math.sin(ps);
  out[0] = xl * c - S.sideOff * s;
  out[1] = xl * s + S.sideOff * c;
  out[2] = zl;
}

function voxelKey(x, y, z, s) {
  var ix = Math.floor(x / s) + 65536, iy = Math.floor(y / s) + 65536, iz = Math.floor(z / s) + 65536;
  return (ix * 131072 + iy) * 131072 + iz;
}

function writeSlot(i, th, r, ps, q, x, y, z) {
  raw[i * 4] = th; raw[i * 4 + 1] = r; raw[i * 4 + 2] = ps; raw[i * 4 + 3] = q;
  xyz[i * 3] = x; xyz[i * 3 + 1] = y; xyz[i * 3 + 2] = z;
}

function insertPoint(th, r, ps, q, x, y, z, key) {
  if (key >= 0) {
    var existing = vox.get(key);
    if (existing !== undefined) {
      if (q <= raw[existing * 4 + 3]) return;        // keep the better point already there
      writeSlot(existing, th, r, ps, q, x, y, z);
      if (existing < replLo) replLo = existing;
      if (existing + 1 > replHi) replHi = existing + 1;
      return;
    }
  }
  var slot = writeIdx;
  if (count === MAX) {                                // ring full: forget the oldest point
    var old = slotKey[slot];
    if (old >= 0 && vox.get(old) === slot) vox.delete(old);
  }
  writeSlot(slot, th, r, ps, q, x, y, z);
  slotKey[slot] = key;
  if (key >= 0) vox.set(key, slot);
  writeIdx = (writeIdx + 1) % MAX;
  if (count < MAX) count++;
  if (pending < MAX) pending++;
}

function upload() {
  gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
  if (fullUpload) {
    gl.bufferSubData(gl.ARRAY_BUFFER, 0, raw);
    fullUpload = false; pending = 0; replLo = MAX; replHi = 0;
    return;
  }
  if (pending > 0) {
    if (pending >= MAX) {
      gl.bufferSubData(gl.ARRAY_BUFFER, 0, raw);
    } else {
      var start = (writeIdx - pending + MAX) % MAX;
      if (start + pending <= MAX) {
        gl.bufferSubData(gl.ARRAY_BUFFER, start * 16, raw.subarray(start * 4, (start + pending) * 4));
      } else {
        gl.bufferSubData(gl.ARRAY_BUFFER, start * 16, raw.subarray(start * 4, MAX * 4));
        gl.bufferSubData(gl.ARRAY_BUFFER, 0, raw.subarray(0, writeIdx * 4));
      }
    }
    pending = 0;
  }
  if (replHi > replLo) {
    gl.bufferSubData(gl.ARRAY_BUFFER, replLo * 16, raw.subarray(replLo * 4, replHi * 4));
    replLo = MAX; replHi = 0;
  }
}

function clearPoints() {
  count = 0; writeIdx = 0; pending = 0; replLo = MAX; replHi = 0;
  slotKey.fill(-1);
  vox.clear();
  bounds = null;
  mPts = []; buildMeasure();
  requestRender();
}

// Recompute CPU xyz + voxel index after calibration / filter changes
function rebuildDerived() {
  vox.clear();
  for (var i = 0; i < count; i++) {
    toXYZ(raw[i * 4], raw[i * 4 + 1], raw[i * 4 + 2], tmp);
    xyz[i * 3] = tmp[0]; xyz[i * 3 + 1] = tmp[1]; xyz[i * 3 + 2] = tmp[2];
    if (S.voxel > 0) {
      var key = voxelKey(tmp[0], tmp[1], tmp[2], S.voxel);
      slotKey[i] = key;
      vox.set(key, i);
    } else {
      slotKey[i] = -1;
    }
  }
  stations.forEach(computeLocal);
  computeRanges();
  requestRender();
}

function pct(arr, f) { return arr[Math.min(arr.length - 1, Math.max(0, Math.floor(f * (arr.length - 1))))]; }
var lastRangeCount = -1;
function computeRanges() {
  var total = totalPoints();
  if (total === 0) { bounds = null; return; }
  var step = Math.max(1, Math.floor(total / 20000));
  var xs = [], ys = [], zs = [], qs = [];
  eachPoint(step, function (x, y, z, q) { xs.push(x); ys.push(y); zs.push(z); qs.push(q); });
  var num = function (a, b) { return a - b; };
  xs.sort(num); ys.sort(num); zs.sort(num); qs.sort(num);
  zLo = pct(zs, 0.01); zHi = pct(zs, 0.99); floorZ = pct(zs, 0.02);
  qMax = Math.max(1, pct(qs, 0.99));
  bounds = { x0: pct(xs, 0.02), x1: pct(xs, 0.98), y0: pct(ys, 0.02), y1: pct(ys, 0.98), z0: pct(zs, 0.02), z1: pct(zs, 0.98) };
  lastRangeCount = count;
  requestRender();
}

// ======================= ALIGN-BEGIN =======================
// Multi-station registration. A station's pose maps its local points to the world:
//   world = Rz(yaw) * local + (tx, ty, tz)     (4 DOF: the rig is assumed roughly level)
// Step 1 (coarse): slice a wall band out of both clouds, flatten to 2D, and search
//   every yaw (5 deg) and translation (20 cm) against a distance field of the reference.
// Step 2 (fine): point-to-plane ICP in 3D, solving for (dyaw, dx, dy, dz) each iteration.
var Align = (function () {
  'use strict';
  function pctOf(arr, f) {
    var a = Array.prototype.slice.call(arr).sort(function (x, y) { return x - y; });
    return a.length ? a[Math.min(a.length - 1, Math.floor(f * (a.length - 1)))] : 0;
  }
  function key3(ix, iy, iz) { return ((ix + 32768) * 65536 + (iy + 32768)) * 65536 + (iz + 32768); }

  // Keep one point per voxel, then thin to at most maxOut points
  function voxelSample(pts, n, size, maxOut) {
    var seen = new Map(), keep = [];
    for (var i = 0; i < n; i++) {
      var k = key3(Math.floor(pts[i * 3] / size), Math.floor(pts[i * 3 + 1] / size), Math.floor(pts[i * 3 + 2] / size));
      if (!seen.has(k)) { seen.set(k, 1); keep.push(i); }
    }
    var stride = Math.max(1, keep.length / maxOut), m = Math.floor(keep.length / stride);
    var out = new Float32Array(m * 3);
    for (var j = 0; j < m; j++) {
      var s = keep[Math.floor(j * stride)];
      out[j * 3] = pts[s * 3]; out[j * 3 + 1] = pts[s * 3 + 1]; out[j * 3 + 2] = pts[s * 3 + 2];
    }
    return out;
  }
  function floorOf(p) {
    var zs = [], step = Math.max(1, Math.floor(p.length / 3 / 5000));
    for (var i = 0; i < p.length / 3; i += step) zs.push(p[i * 3 + 2]);
    return pctOf(zs, 0.02);
  }
  function slice2D(p, zLo, zHi, maxOut) {
    var xy = [];
    for (var i = 0; i < p.length / 3; i++) {
      var z = p[i * 3 + 2];
      if (z >= zLo && z <= zHi) xy.push(p[i * 3], p[i * 3 + 1]);
    }
    var n = xy.length / 2, stride = Math.max(1, n / maxOut), m = Math.floor(n / stride), out = new Float32Array(m * 2);
    for (var j = 0; j < m; j++) { var s = Math.floor(j * stride); out[j * 2] = xy[s * 2]; out[j * 2 + 1] = xy[s * 2 + 1]; }
    return out;
  }

  // 2D distance field of the reference wall slice (chamfer transform, metres, capped)
  function distField(xy, cell, cap) {
    var x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity, n = xy.length / 2, i;
    for (i = 0; i < n; i++) {
      x0 = Math.min(x0, xy[i * 2]); x1 = Math.max(x1, xy[i * 2]);
      y0 = Math.min(y0, xy[i * 2 + 1]); y1 = Math.max(y1, xy[i * 2 + 1]);
    }
    x0 -= 1; y0 -= 1; x1 += 1; y1 += 1;
    var w = Math.ceil((x1 - x0) / cell) + 1, h = Math.ceil((y1 - y0) / cell) + 1;
    var d = new Float32Array(w * h).fill(1e9);
    for (i = 0; i < n; i++) d[Math.floor((xy[i * 2 + 1] - y0) / cell) * w + Math.floor((xy[i * 2] - x0) / cell)] = 0;
    var D = 1.4142, x, y, k, v;
    for (y = 0; y < h; y++) for (x = 0; x < w; x++) {
      k = y * w + x; v = d[k];
      if (x > 0) v = Math.min(v, d[k - 1] + 1);
      if (y > 0) {
        v = Math.min(v, d[k - w] + 1);
        if (x > 0) v = Math.min(v, d[k - w - 1] + D);
        if (x < w - 1) v = Math.min(v, d[k - w + 1] + D);
      }
      d[k] = v;
    }
    for (y = h - 1; y >= 0; y--) for (x = w - 1; x >= 0; x--) {
      k = y * w + x; v = d[k];
      if (x < w - 1) v = Math.min(v, d[k + 1] + 1);
      if (y < h - 1) {
        v = Math.min(v, d[k + w] + 1);
        if (x < w - 1) v = Math.min(v, d[k + w + 1] + D);
        if (x > 0) v = Math.min(v, d[k + w - 1] + D);
      }
      d[k] = Math.min(v * cell, cap);
    }
    return { x0: x0, y0: y0, x1: x1, y1: y1, w: w, h: h, cell: cell, cap: cap, d: d };
  }
  function score2D(F, rx, ry, n, tx, ty) {
    var sum = 0, inv = 1 / F.cell;
    for (var i = 0; i < n; i++) {
      var gx = Math.floor((rx[i] + tx - F.x0) * inv), gy = Math.floor((ry[i] + ty - F.y0) * inv);
      sum += (gx < 0 || gy < 0 || gx >= F.w || gy >= F.h) ? F.cap : F.d[gy * F.w + gx];
    }
    return sum / n;
  }

  function coarse(refXY, srcXY) {
    var F = distField(refXY, 0.1, 0.5);
    var n = srcXY.length / 2, rx = new Float32Array(n), ry = new Float32Array(n), best = [];
    for (var yd = 0; yd < 360; yd += 4) {
      var a = yd * Math.PI / 180, c = Math.cos(a), s = Math.sin(a), cx = 0, cy = 0;
      for (var i = 0; i < n; i++) {
        rx[i] = c * srcXY[i * 2] - s * srcXY[i * 2 + 1];
        ry[i] = s * srcXY[i * 2] + c * srcXY[i * 2 + 1];
        cx += rx[i]; cy += ry[i];
      }
      cx /= n; cy /= n;
      var bs = Infinity, btx = 0, bty = 0;
      for (var tx = F.x0 + 1 - cx; tx <= F.x1 - 1 - cx; tx += 0.2)
        for (var ty = F.y0 + 1 - cy; ty <= F.y1 - 1 - cy; ty += 0.2) {
          var sc = score2D(F, rx, ry, n, tx, ty);
          if (sc < bs) { bs = sc; btx = tx; bty = ty; }
        }
      // local refinement of yaw and translation
      var byaw = yd;
      for (var pass = 0; pass < 2; pass++) {
        var st = pass ? 0.03 : 0.08;
        for (var dyaw = -2; dyaw <= 2; dyaw += 1) {
          var a2 = (yd + dyaw) * Math.PI / 180, c2 = Math.cos(a2), s2 = Math.sin(a2);
          for (i = 0; i < n; i++) {
            rx[i] = c2 * srcXY[i * 2] - s2 * srcXY[i * 2 + 1];
            ry[i] = s2 * srcXY[i * 2] + c2 * srcXY[i * 2 + 1];
          }
          for (var ox = -2; ox <= 2; ox++) for (var oy = -2; oy <= 2; oy++) {
            var sc2 = score2D(F, rx, ry, n, btx + ox * st, bty + oy * st);
            if (sc2 < bs) { bs = sc2; byaw = yd + dyaw; btx += ox * st; bty += oy * st; }
          }
        }
      }
      best.push({ yaw: byaw * Math.PI / 180, tx: btx, ty: bty, score: bs });
    }
    best.sort(function (p, q) { return p.score - q.score; });
    var picked = [];
    for (var b = 0; b < best.length && picked.length < 4; b++) {
      var ok = true;
      for (var p2 = 0; p2 < picked.length; p2++) {
        var dy = Math.abs(((best[b].yaw - picked[p2].yaw) * 180 / Math.PI + 540) % 360 - 180);
        if (dy < 15 && Math.hypot(best[b].tx - picked[p2].tx, best[b].ty - picked[p2].ty) < 0.6) ok = false;
      }
      if (ok) picked.push(best[b]);
    }
    return picked;
  }

  // ---------- 3D reference: hash grid + lazy normals ----------
  function Ref(pts) {
    this.p = pts; this.n = pts.length / 3; this.cell = 0.15;
    this.grid = new Map();
    for (var i = 0; i < this.n; i++) {
      var k = this.key(pts[i * 3], pts[i * 3 + 1], pts[i * 3 + 2]);
      var list = this.grid.get(k);
      if (list) list.push(i); else this.grid.set(k, [i]);
    }
    this.nrm = new Float32Array(this.n * 3);
    this.hasN = new Uint8Array(this.n);    // 0 = not computed, 1 = ok, 2 = no plane
  }
  Ref.prototype.key = function (x, y, z) {
    return key3(Math.floor(x / this.cell), Math.floor(y / this.cell), Math.floor(z / this.cell));
  };
  Ref.prototype.nearest = function (x, y, z, maxD) {
    var c = this.cell, r = Math.ceil(maxD / c), ix = Math.floor(x / c), iy = Math.floor(y / c), iz = Math.floor(z / c);
    var best = -1, bd = maxD * maxD, p = this.p;
    for (var a = -r; a <= r; a++) for (var b = -r; b <= r; b++) for (var e = -r; e <= r; e++) {
      var list = this.grid.get(key3(ix + a, iy + b, iz + e));
      if (!list) continue;
      for (var t = 0; t < list.length; t++) {
        var j = list[t], dx = p[j * 3] - x, dy = p[j * 3 + 1] - y, dz = p[j * 3 + 2] - z, d2 = dx * dx + dy * dy + dz * dz;
        if (d2 < bd) { bd = d2; best = j; }
      }
    }
    this.lastD2 = bd;
    return best;
  };
  Ref.prototype.normal = function (j) {
    if (this.hasN[j]) return this.hasN[j] === 1;
    var p = this.p, x = p[j * 3], y = p[j * 3 + 1], z = p[j * 3 + 2], c = this.cell, R2 = 0.12 * 0.12;
    var ix = Math.floor(x / c), iy = Math.floor(y / c), iz = Math.floor(z / c);
    var m = 0, sx = 0, sy = 0, sz = 0, xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
    for (var a = -1; a <= 1; a++) for (var b = -1; b <= 1; b++) for (var e = -1; e <= 1; e++) {
      var list = this.grid.get(key3(ix + a, iy + b, iz + e));
      if (!list) continue;
      for (var t = 0; t < list.length; t++) {
        var k = list[t], dx = p[k * 3] - x, dy = p[k * 3 + 1] - y, dz = p[k * 3 + 2] - z;
        if (dx * dx + dy * dy + dz * dz > R2) continue;
        m++; sx += dx; sy += dy; sz += dz;
        xx += dx * dx; xy += dx * dy; xz += dx * dz; yy += dy * dy; yz += dy * dz; zz += dz * dz;
      }
    }
    if (m < 6) { this.hasN[j] = 2; return false; }
    sx /= m; sy /= m; sz /= m;
    var A = [xx / m - sx * sx, xy / m - sx * sy, xz / m - sx * sz,
             yy / m - sy * sy, yz / m - sy * sz, zz / m - sz * sz];
    var v = smallestEigvec(A);
    if (!v) { this.hasN[j] = 2; return false; }
    this.nrm[j * 3] = v[0]; this.nrm[j * 3 + 1] = v[1]; this.nrm[j * 3 + 2] = v[2];
    this.hasN[j] = 1;
    return true;
  };

  // smallest eigenvector of symmetric 3x3 [a b c; b d e; c e f] given as [a,b,c,d,e,f]
  function smallestEigvec(A) {
    var a = A[0], b = A[1], c = A[2], d = A[3], e = A[4], f = A[5];
    var p1 = b * b + c * c + e * e, q = (a + d + f) / 3;
    var p2 = (a - q) * (a - q) + (d - q) * (d - q) + (f - q) * (f - q) + 2 * p1;
    var p = Math.sqrt(p2 / 6);
    if (p < 1e-12) return null;
    var B = [(a - q) / p, b / p, c / p, (d - q) / p, e / p, (f - q) / p];
    var detB = B[0] * (B[3] * B[5] - B[4] * B[4]) - B[1] * (B[1] * B[5] - B[4] * B[2]) + B[2] * (B[1] * B[4] - B[3] * B[2]);
    var r = Math.max(-1, Math.min(1, detB / 2)), phi = Math.acos(r) / 3;
    var lmin = q + 2 * p * Math.cos(phi + 2 * Math.PI / 3);
    var r0 = [a - lmin, b, c], r1 = [b, d - lmin, e], r2 = [c, e, f - lmin];
    var cands = [cross(r0, r1), cross(r0, r2), cross(r1, r2)], bestV = null, bn = 0;
    for (var i = 0; i < 3; i++) {
      var nn = Math.hypot(cands[i][0], cands[i][1], cands[i][2]);
      if (nn > bn) { bn = nn; bestV = cands[i]; }
    }
    if (bn < 1e-12) return null;
    return [bestV[0] / bn, bestV[1] / bn, bestV[2] / bn];
  }
  function cross(u, v) { return [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]; }

  function solve4(H, g) {      // Gaussian elimination with partial pivoting, H x = g
    var M = [], i, j, k;
    for (i = 0; i < 4; i++) M.push([H[i * 4], H[i * 4 + 1], H[i * 4 + 2], H[i * 4 + 3], g[i]]);
    for (i = 0; i < 4; i++) {
      var piv = i;
      for (j = i + 1; j < 4; j++) if (Math.abs(M[j][i]) > Math.abs(M[piv][i])) piv = j;
      var t = M[i]; M[i] = M[piv]; M[piv] = t;
      if (Math.abs(M[i][i]) < 1e-12) return null;
      for (j = i + 1; j < 4; j++) {
        var f = M[j][i] / M[i][i];
        for (k = i; k < 5; k++) M[j][k] -= f * M[i][k];
      }
    }
    var x = [0, 0, 0, 0];
    for (i = 3; i >= 0; i--) {
      var s = M[i][4];
      for (j = i + 1; j < 4; j++) s -= M[i][j] * x[j];
      x[i] = s / M[i][i];
    }
    return x;
  }

  // Point-to-plane ICP, 4 DOF
  function icp(ref, src, pose, iters, maxPts, d0, d1) {
    var yaw = pose.yaw, tx = pose.tx, ty = pose.ty, tz = pose.tz;
    var n = src.length / 3, stride = Math.max(1, Math.floor(n / maxPts));
    for (var it = 0; it < iters; it++) {
      var maxD = Math.max(d1, d0 * Math.pow(d1 / d0, it / Math.max(1, iters - 1)));
      var sig = maxD / 2, c = Math.cos(yaw), s = Math.sin(yaw);
      var H = new Float64Array(16), g = new Float64Array(4), used = 0;
      for (var i = (it * 7) % stride; i < n; i += stride) {
        var x = src[i * 3], y = src[i * 3 + 1], z = src[i * 3 + 2];
        var px = c * x - s * y + tx, py = s * x + c * y + ty, pz = z + tz;
        var j = ref.nearest(px, py, pz, maxD);
        if (j < 0 || !ref.normal(j)) continue;
        var nx = ref.nrm[j * 3], ny = ref.nrm[j * 3 + 1], nz = ref.nrm[j * 3 + 2];
        var r = nx * (px - ref.p[j * 3]) + ny * (py - ref.p[j * 3 + 1]) + nz * (pz - ref.p[j * 3 + 2]);
        var w = 1 / (1 + (r / sig) * (r / sig));
        var J = [ny * px - nx * py, nx, ny, nz];
        for (var a = 0; a < 4; a++) {
          g[a] -= w * J[a] * r;
          for (var b = 0; b < 4; b++) H[a * 4 + b] += w * J[a] * J[b];
        }
        used++;
      }
      if (used < 30) break;
      for (var dgi = 0; dgi < 4; dgi++) H[dgi * 5] += 1e-6 * used;
      var dx = solve4(H, g);
      if (!dx) break;
      var dyaw = Math.max(-0.1, Math.min(0.1, dx[0])), cd = Math.cos(dyaw), sd = Math.sin(dyaw);
      var ntx = cd * tx - sd * ty + dx[1], nty = sd * tx + cd * ty + dx[2];
      tx = ntx; ty = nty; tz += dx[3]; yaw += dyaw;
    }
    return { yaw: yaw, tx: tx, ty: ty, tz: tz };
  }

  // Fraction of source points within 5 cm of the reference, and their RMS error
  function evaluate(ref, src, pose, maxPts) {
    var n = src.length / 3, stride = Math.max(1, Math.floor(n / maxPts)), c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    var hit = 0, tot = 0, se = 0;
    for (var i = 0; i < n; i += stride) {
      var x = src[i * 3], y = src[i * 3 + 1], z = src[i * 3 + 2];
      tot++;
      if (ref.nearest(c * x - s * y + pose.tx, s * x + c * y + pose.ty, z + pose.tz, 0.05) >= 0) { hit++; se += ref.lastD2; }
    }
    return { fitness: tot ? hit / tot : 0, rmse: hit ? Math.sqrt(se / hit) : 0 };
  }

  function prepare(refPts, refN, srcPts, srcN) {
    return { ref: voxelSample(refPts, refN, 0.04, 150000), src: voxelSample(srcPts, srcN, 0.05, 8000) };
  }

  // Full automatic alignment (no initial guess needed)
  function auto(refPts, refN, srcPts, srcN) {
    var P = prepare(refPts, refN, srcPts, srcN);
    var rf = floorOf(P.ref), sf = floorOf(P.src);
    var refXY = slice2D(P.ref, rf + 0.3, rf + 1.9, 20000), srcXY = slice2D(P.src, sf + 0.3, sf + 1.9, 700);
    var ref = new Ref(P.ref), tz0 = rf - sf;
    if (refXY.length < 40 || srcXY.length < 40) return null;
    var cands = coarse(refXY, srcXY), best = null;
    for (var k = 0; k < cands.length; k++) {
      var pose = icp(ref, P.src, { yaw: cands[k].yaw, tx: cands[k].tx, ty: cands[k].ty, tz: tz0 }, 12, 1500, 0.4, 0.08);
      var ev = evaluate(ref, P.src, pose, 2000);
      if (!best || ev.fitness > best.ev.fitness) best = { pose: pose, ev: ev };
    }
    if (!best) return null;
    var fin = icp(ref, P.src, best.pose, 25, 6000, 0.12, 0.03);
    var ev2 = evaluate(ref, P.src, fin, 8000);
    return { pose: fin, fitness: ev2.fitness, rmse: ev2.rmse };
  }

  // Refine from the current pose (after a manual nudge)
  function refine(refPts, refN, srcPts, srcN, pose) {
    var P = prepare(refPts, refN, srcPts, srcN), ref = new Ref(P.ref);
    var p1 = icp(ref, P.src, pose, 15, 2500, 0.4, 0.08);
    var fin = icp(ref, P.src, p1, 25, 6000, 0.12, 0.03);
    var ev = evaluate(ref, P.src, fin, 8000);
    return { pose: fin, fitness: ev.fitness, rmse: ev.rmse };
  }

  return { auto: auto, refine: refine };
})();
// ======================= ALIGN-END =======================

// ======================= Stations (merged scans) =======================
// Each station = one finished scan from one scanner position, with a pose
// world = Rz(yaw) * local + t. The live scan always sits in the world frame (identity).
var stations = [], selSt = -1, moveOn = false, busy = false, stSeq = 0;
var alignMsg = '', alignMsgUntil = 0;
var MAX_ST = 8;
var PALETTE = [[0.3, 0.75, 1], [1, 0.45, 0.35], [0.45, 0.9, 0.4], [1, 0.8, 0.25],
               [0.8, 0.5, 1], [0.3, 0.95, 0.85], [1, 0.55, 0.8], [0.75, 0.75, 0.75]];
var LIVE_COL = [0.92, 0.92, 0.88];

function makeStation(rawArr, n, pose) {
  var st = { raw: rawArr, n: n, xyz: new Float32Array(n * 3), pose: pose || { yaw: 0, tx: 0, ty: 0, tz: 0 },
             vbo: gl.createBuffer(), visible: true, fit: null, rmse: null, id: ++stSeq };
  computeLocal(st);
  gl.bindBuffer(gl.ARRAY_BUFFER, st.vbo);
  gl.bufferData(gl.ARRAY_BUFFER, rawArr, gl.STATIC_DRAW);
  return st;
}
function computeLocal(st) {
  for (var i = 0; i < st.n; i++) {
    toXYZ(st.raw[i * 4], st.raw[i * 4 + 1], st.raw[i * 4 + 2], tmp);
    st.xyz[i * 3] = tmp[0]; st.xyz[i * 3 + 1] = tmp[1]; st.xyz[i * 3 + 2] = tmp[2];
  }
}
function stationColor(idx) { return PALETTE[idx % PALETTE.length]; }
function totalPoints() {
  var t = count;
  stations.forEach(function (st) { if (st.visible) t += st.n; });
  return t;
}
// Visit points in world coordinates: cb(x, y, z, q, stationIndex) — stationIndex -1 = live
function eachPoint(step, cb) {
  var i;
  for (i = 0; i < count; i += step) cb(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2], raw[i * 4 + 3], -1);
  for (var s = 0; s < stations.length; s++) {
    var st = stations[s];
    if (!st.visible) continue;
    var c = Math.cos(st.pose.yaw), sn = Math.sin(st.pose.yaw), P = st.pose, L = st.xyz;
    for (i = 0; i < st.n; i += step) {
      var x = L[i * 3], y = L[i * 3 + 1];
      cb(c * x - sn * y + P.tx, sn * x + c * y + P.ty, L[i * 3 + 2] + P.tz, st.raw[i * 4 + 3], s);
    }
  }
}
function worldOf(st) {
  var out = new Float32Array(st.n * 3), c = Math.cos(st.pose.yaw), s = Math.sin(st.pose.yaw), P = st.pose, L = st.xyz;
  for (var i = 0; i < st.n; i++) {
    out[i * 3] = c * L[i * 3] - s * L[i * 3 + 1] + P.tx;
    out[i * 3 + 1] = s * L[i * 3] + c * L[i * 3 + 1] + P.ty;
    out[i * 3 + 2] = L[i * 3 + 2] + P.tz;
  }
  return out;
}
function refCloud(exclude) {
  var parts = [], n = 0;
  stations.forEach(function (st, i) { if (i !== exclude) { parts.push(worldOf(st)); n += st.n; } });
  var pts = new Float32Array(n * 3), o = 0;
  parts.forEach(function (p) { pts.set(p, o); o += p.length; });
  return { pts: pts, n: n };
}
function setAlignMsg(m) { alignMsg = m; alignMsgUntil = Date.now() + 15000; updateHud(); }
function runBusy(msg, fn) {
  if (busy) return;
  busy = true;
  setAlignMsg(msg);
  setTimeout(function () {
    try { fn(); } catch (e) { setAlignMsg('Alignment error: ' + e.message); }
    busy = false;
    renderStations();
    updateHud();
  }, 60);
}
function report(st, idx) {
  var name = 'Station ' + (idx + 1);
  if (st.fit === null) return name + ' added';
  var txt = Math.round(st.fit * 100) + '% overlap, ' + (st.rmse * 100).toFixed(1) + ' cm error';
  if (st.fit >= 0.45) return name + ' aligned: ' + txt;
  if (st.fit >= 0.2) return name + ': weak match (' + txt + ') — check it with Color: By station';
  return name + ' could not be matched. Rotate or drag it into place, then Snap into place.';
}
function addStation() {
  if (busy) return;
  if (!count) { alert('Scan something first'); return; }
  if (stations.length >= MAX_ST) { alert('Up to ' + MAX_ST + ' stations'); return; }
  var st = makeStation(raw.slice(0, count * 4), count);
  if (!stations.length) {
    stations.push(st);
    selSt = 0;
    clearPoints();
    setAlignMsg('Station 1 added. Move the scanner to the next spot and scan again.');
    renderStations(); computeRanges(); fitView();
    return;
  }
  runBusy('Aligning station ' + (stations.length + 1) + '… (a few seconds)', function () {
    var ref = refCloud(-1);
    var res = Align.auto(ref.pts, ref.n, st.xyz, st.n);
    if (res) { st.pose = res.pose; st.fit = res.fitness; st.rmse = res.rmse; } else { st.fit = 0; st.rmse = 0; }
    stations.push(st);
    selSt = stations.length - 1;
    clearPoints();
    setAlignMsg(report(st, selSt));
    computeRanges(); fitView();
  });
}
function realign(auto) {
  if (busy || selSt < 0 || stations.length < 2) { if (stations.length < 2) alert('Needs at least two stations'); return; }
  var idx = selSt, st = stations[idx];
  runBusy((auto ? 'Auto-aligning' : 'Snapping') + ' station ' + (idx + 1) + '…', function () {
    var ref = refCloud(idx);
    var res = auto ? Align.auto(ref.pts, ref.n, st.xyz, st.n) : Align.refine(ref.pts, ref.n, st.xyz, st.n, st.pose);
    if (res) { st.pose = res.pose; st.fit = res.fitness; st.rmse = res.rmse; }
    setAlignMsg(report(st, idx));
    computeRanges(); requestRender();
  });
}
function stationCentroid(st) {
  var cx = 0, cy = 0, m = 0, step = Math.max(1, Math.floor(st.n / 3000));
  var c = Math.cos(st.pose.yaw), s = Math.sin(st.pose.yaw);
  for (var i = 0; i < st.n; i += step) {
    cx += c * st.xyz[i * 3] - s * st.xyz[i * 3 + 1] + st.pose.tx;
    cy += s * st.xyz[i * 3] + c * st.xyz[i * 3 + 1] + st.pose.ty;
    m++;
  }
  return [cx / m, cy / m];
}
function rotateSel(deg) {
  if (selSt < 0 || busy) return;
  var st = stations[selSt], cen = stationCentroid(st), a = deg * DEG, c = Math.cos(a), s = Math.sin(a);
  var dx = st.pose.tx - cen[0], dy = st.pose.ty - cen[1];
  st.pose.tx = c * dx - s * dy + cen[0];
  st.pose.ty = s * dx + c * dy + cen[1];
  st.pose.yaw += a;
  st.fit = null;
  renderStations(); requestRender();
}
function moveSel(dx, dy) {
  var V = (lastM || matrices()).V, s = cam.dist / Math.max(1, canvas.clientHeight), st = stations[selSt];
  st.pose.tx += (V[0] * dx - V[1] * dy) * s;
  st.pose.ty += (V[4] * dx - V[5] * dy) * s;
  st.fit = null;
  requestRender();
}
function deleteStation(i) {
  gl.deleteBuffer(stations[i].vbo);
  stations.splice(i, 1);
  if (selSt >= stations.length) selSt = stations.length - 1;
  renderStations(); computeRanges(); requestRender();
}
function clearStations() {
  stations.forEach(function (st) { gl.deleteBuffer(st.vbo); });
  stations = []; selSt = -1;
  setMove(false);
  renderStations(); computeRanges(); requestRender();
}
function renderStations() {
  var el = $('stList');
  el.innerHTML = '';
  $('stInfo').textContent = stations.length ? stations.length + ' station' + (stations.length > 1 ? 's' : '') : 'No stations';
  stations.forEach(function (st, i) {
    var row = document.createElement('div');
    row.className = 'st' + (i === selSt ? ' sel' : '');
    var col = stationColor(i);
    var label = document.createElement('span');
    var sw = document.createElement('span');
    sw.className = 'sw';
    sw.style.background = 'rgb(' + col.map(function (v) { return Math.round(v * 255); }).join(',') + ')';
    label.appendChild(sw);
    var info = (i === 0 && st.fit === null) ? 'reference' :
      (st.fit === null ? 'moved by hand' : Math.round(st.fit * 100) + '% / ' + (st.rmse * 100).toFixed(1) + ' cm');
    label.appendChild(document.createTextNode((i + 1) + ' · ' + Math.round(st.n / 1000) + 'k · ' + info));
    var btns = document.createElement('span');
    var vis = document.createElement('button'); vis.className = 'alt'; vis.textContent = st.visible ? 'Hide' : 'Show';
    var del = document.createElement('button'); del.className = 'alt'; del.textContent = '✕';
    vis.onclick = function (e) { e.stopPropagation(); st.visible = !st.visible; renderStations(); computeRanges(); requestRender(); };
    del.onclick = function (e) { e.stopPropagation(); if (confirm('Remove station ' + (i + 1) + '?')) deleteStation(i); };
    row.onclick = function () { selSt = i; renderStations(); requestRender(); };
    btns.appendChild(vis); btns.appendChild(del);
    row.appendChild(label); row.appendChild(btns);
    el.appendChild(row);
  });
  var has = selSt >= 0 && stations.length > 0;
  document.querySelectorAll('.rotb').forEach(function (b) { b.disabled = !has; });
  $('moveStBtn').disabled = !has;
  $('snapBtn').disabled = !has || stations.length < 2;
  $('reautoBtn').disabled = !has || stations.length < 2;
}
function setMove(on) {
  moveOn = on && selSt >= 0;
  $('moveStBtn').textContent = 'Move by hand: ' + (moveOn ? 'on' : 'off');
  $('moveStBtn').className = moveOn ? 'on' : 'alt';
  if (moveOn && !topView) setTop(true);
  requestRender();
}

// ======================= Live data =======================
var paused = false, rxSec = 0, rxRate = 0, lastPan = null;

function ingest(buf) {
  var dv = new DataView(buf);
  var n = Math.floor(buf.byteLength / 12);
  rxSec += n;
  if (n > 0) lastPan = dv.getFloat32((n - 1) * 12 + 8, true);
  if (paused) return;
  var vs = S.voxel, minq = S.minq, maxr = S.maxr, added = false;
  for (var k = 0; k < n; k++) {
    var o = k * 12;
    var th = dv.getFloat32(o, true);
    var r = dv.getUint16(o + 4, true) / 1000;
    var q = dv.getUint8(o + 6);
    var ps = dv.getFloat32(o + 8, true);
    if (r < 0.05 || r > maxr || q < minq) continue;
    toXYZ(th, r, ps, tmp);
    insertPoint(th, r, ps, q, tmp[0], tmp[1], tmp[2], vs > 0 ? voxelKey(tmp[0], tmp[1], tmp[2], vs) : -1);
    added = true;
  }
  if (added) requestRender();
}

var ws = null, wsConnected = false, retryMs = 1000;
function connect() {
  ws = new WebSocket('ws://' + location.hostname + ':81/');
  ws.binaryType = 'arraybuffer';
  ws.onopen = function () { wsConnected = true; retryMs = 1000; };
  ws.onmessage = function (ev) { if (ev.data instanceof ArrayBuffer) ingest(ev.data); };
  ws.onclose = function () { wsConnected = false; setTimeout(connect, retryMs); retryMs = Math.min(retryMs * 2, 8000); };
  ws.onerror = function () { try { ws.close(); } catch (e) {} };
}

// ======================= Camera =======================
var cam = { yaw: -Math.PI / 4, pitch: 0.55, dist: 7, target: [0, 0, 0] };
var topView = false;
var lastM = null;

function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function norm(a) { var l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; }

function perspective(fovy, aspect, near, far) {
  var f = 1 / Math.tan(fovy / 2), nf = 1 / (near - far);
  return [f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) * nf, -1, 0, 0, 2 * far * near * nf, 0];
}
function ortho(l, r, b, t, n, f) {
  return [2 / (r - l), 0, 0, 0, 0, 2 / (t - b), 0, 0, 0, 0, -2 / (f - n), 0,
          -(r + l) / (r - l), -(t + b) / (t - b), -(f + n) / (f - n), 1];
}
function lookAt(e, c, up) {
  var z = norm(sub(e, c)), x = norm(cross(up, z)), y = cross(z, x);
  return [x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0, -dot(x, e), -dot(y, e), -dot(z, e), 1];
}
function mul(a, b) {
  var o = new Array(16);
  for (var col = 0; col < 4; col++)
    for (var row = 0; row < 4; row++) {
      var s = 0;
      for (var k = 0; k < 4; k++) s += a[k * 4 + row] * b[col * 4 + k];
      o[col * 4 + row] = s;
    }
  return o;
}
function eyePos() {
  var cp = Math.cos(cam.pitch);
  return [cam.target[0] + cam.dist * cp * Math.cos(cam.yaw),
          cam.target[1] + cam.dist * cp * Math.sin(cam.yaw),
          cam.target[2] + cam.dist * Math.sin(cam.pitch)];
}
function matrices() {
  var aspect = canvas.width / Math.max(1, canvas.height);
  var P, V, near, far;
  if (!topView) {
    near = 0.05; far = 500;
    P = perspective(55 * DEG, aspect, near, far);
    V = lookAt(eyePos(), cam.target, [0, 0, 1]);
  } else {
    near = 1; far = 200;
    var hh = cam.dist * 0.5, hw = hh * aspect;
    P = ortho(-hw, hw, -hh, hh, near, far);
    V = lookAt([cam.target[0], cam.target[1], cam.target[2] + 60], cam.target, [-Math.cos(cam.yaw), -Math.sin(cam.yaw), 0]);
  }
  return { P: P, V: V, MVP: mul(P, V), near: near, far: far };
}

function panCamera(dx, dy) {
  var V = (lastM || matrices()).V;
  var right = [V[0], V[4], V[8]], up = [V[1], V[5], V[9]];
  var s = topView ? cam.dist / Math.max(1, canvas.clientHeight) : cam.dist * 0.0016;
  for (var i = 0; i < 3; i++) cam.target[i] += (-right[i] * dx + up[i] * dy) * s;
  requestRender();
}

function fitView() {
  if (!bounds) computeRanges();
  if (!bounds) { cam = { yaw: -Math.PI / 4, pitch: 0.55, dist: 7, target: [0, 0, 0] }; requestRender(); return; }
  var lo = Math.max(S.clipLo, bounds.z0), hi = Math.min(S.clipHi, bounds.z1);
  cam.target = [(bounds.x0 + bounds.x1) / 2, (bounds.y0 + bounds.y1) / 2, (lo + hi) / 2];
  var aspect = Math.min(1, canvas.width / Math.max(1, canvas.height));   // portrait phones are narrow
  if (topView) {
    cam.dist = Math.max(0.5, Math.hypot(bounds.x1 - bounds.x0, bounds.y1 - bounds.y0)) * 1.05 / aspect;
  } else {
    var ext = Math.max(bounds.x1 - bounds.x0, bounds.y1 - bounds.y0, hi - lo, 0.5);
    cam.dist = ext * 1.3 / Math.sqrt(aspect);
  }
  requestRender();
}

// ======================= Interaction =======================
var pointers = new Map(), lastPinch = 0, lastMid = null, tapStart = null;
function pdist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }
function pmid(a, b) { return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 }; }
var lastAng = null;
function pang(a, b) { return Math.atan2(b.y - a.y, b.x - a.x); }
function resetPinch() {
  if (pointers.size === 2) {
    var p = Array.from(pointers.values());
    lastPinch = pdist(p[0], p[1]); lastMid = pmid(p[0], p[1]); lastAng = pang(p[0], p[1]);
  } else { lastPinch = 0; lastMid = null; lastAng = null; }
}
canvas.addEventListener('pointerdown', function (e) {
  canvas.setPointerCapture(e.pointerId);
  pointers.set(e.pointerId, { x: e.clientX, y: e.clientY, button: e.button });
  tapStart = pointers.size === 1 ? { x: e.clientX, y: e.clientY, t: Date.now() } : null;
  resetPinch();
});
canvas.addEventListener('pointermove', function (e) {
  if (!pointers.has(e.pointerId)) return;
  var prev = pointers.get(e.pointerId);
  var cur = { x: e.clientX, y: e.clientY, button: prev.button };
  pointers.set(e.pointerId, cur);
  if (pointers.size === 1) {
    var dx = cur.x - prev.x, dy = cur.y - prev.y;
    if (moveOn && selSt >= 0) moveSel(dx, dy);
    else if (topView || prev.button === 2 || e.shiftKey) panCamera(dx, dy);
    else {
      cam.yaw -= dx * 0.008;
      cam.pitch = Math.max(-1.55, Math.min(1.55, cam.pitch + dy * 0.008));
      requestRender();
    }
  } else if (pointers.size === 2) {
    var p = Array.from(pointers.values());
    var d = pdist(p[0], p[1]), m = pmid(p[0], p[1]);
    if (lastPinch > 0 && d > 0) cam.dist = Math.max(0.2, Math.min(200, cam.dist * lastPinch / d));
    var ang = pang(p[0], p[1]);
    if (moveOn && selSt >= 0) {
      if (lastAng !== null) {
        var da = ang - lastAng;
        if (da > Math.PI) da -= 2 * Math.PI;
        if (da < -Math.PI) da += 2 * Math.PI;
        rotateSel(-da / DEG);
      }
    } else if (lastMid) panCamera(m.x - lastMid.x, m.y - lastMid.y);
    lastPinch = d; lastMid = m; lastAng = ang;
    requestRender();
  }
});
function endPointer(e) {
  if (tapStart && pointers.size === 1 && measureOn) {
    var moved = Math.hypot(e.clientX - tapStart.x, e.clientY - tapStart.y);
    if (moved < 8 && Date.now() - tapStart.t < 400) handleMeasureTap(e.clientX, e.clientY);
  }
  tapStart = null;
  pointers.delete(e.pointerId);
  resetPinch();
}
canvas.addEventListener('pointerup', endPointer);
canvas.addEventListener('pointercancel', function (e) { tapStart = null; pointers.delete(e.pointerId); resetPinch(); });
canvas.addEventListener('contextmenu', function (e) { e.preventDefault(); });
canvas.addEventListener('wheel', function (e) {
  e.preventDefault();
  cam.dist = Math.max(0.2, Math.min(200, cam.dist * Math.exp(e.deltaY * 0.001)));
  requestRender();
}, { passive: false });

// ======================= Measure =======================
var measureOn = false, mPts = [];

function pick(cx, cy) {
  if (!lastM || totalPoints() === 0) return null;
  var m = lastM.MVP, W = canvas.clientWidth, H = canvas.clientHeight;
  var best = null, bestZ = Infinity, R2 = 22 * 22;
  eachPoint(1, function (x, y, z) {
    if (z < S.clipLo || z > S.clipHi) return;
    var w = m[3] * x + m[7] * y + m[11] * z + m[15];
    if (w <= 1e-6) return;
    var px = (m[0] * x + m[4] * y + m[8] * z + m[12]) / w;
    var py = (m[1] * x + m[5] * y + m[9] * z + m[13]) / w;
    var pz = (m[2] * x + m[6] * y + m[10] * z + m[14]) / w;
    if (pz < -1 || pz > 1) return;
    var sx = (px * 0.5 + 0.5) * W, sy = (0.5 - py * 0.5) * H;
    var d2 = (sx - cx) * (sx - cx) + (sy - cy) * (sy - cy);
    if (d2 < R2 && pz < bestZ) { bestZ = pz; best = [x, y, z]; }
  });
  return best;
}

function handleMeasureTap(cx, cy) {
  var p = pick(cx, cy);
  if (!p) return;
  if (mPts.length >= 2) mPts = [];
  mPts.push(p);
  buildMeasure();
  requestRender();
}

function measureText() {
  if (mPts.length < 2) return mPts.length === 1 ? 'Tap the second point' : '--';
  var d = sub(mPts[1], mPts[0]);
  var total = Math.hypot(d[0], d[1], d[2]), horiz = Math.hypot(d[0], d[1]);
  return total.toFixed(3) + ' m (horiz ' + horiz.toFixed(3) + ', height ' + Math.abs(d[2]).toFixed(3) + ')';
}

function buildMeasure() {
  var v = [];
  var sz = cam.dist * 0.015;
  mPts.forEach(function (p) {
    v.push(p[0] - sz, p[1], p[2], 1, 0.8, 0, p[0] + sz, p[1], p[2], 1, 0.8, 0);
    v.push(p[0], p[1] - sz, p[2], 1, 0.8, 0, p[0], p[1] + sz, p[2], 1, 0.8, 0);
    v.push(p[0], p[1], p[2] - sz, 1, 0.8, 0, p[0], p[1], p[2] + sz, 1, 0.8, 0);
  });
  if (mPts.length === 2) v.push(mPts[0][0], mPts[0][1], mPts[0][2], 1, 1, 0.2, mPts[1][0], mPts[1][1], mPts[1][2], 1, 1, 0.2);
  measVerts = v.length / 6;
  gl.bindBuffer(gl.ARRAY_BUFFER, measVbo);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(v.length ? v : [0, 0, 0, 0, 0, 0]), gl.DYNAMIC_DRAW);
  $('measureVal').textContent = measureText();
}

// ======================= Rendering =======================
var fbo = null, colorTex = null, depthTex = null, fboW = 0, fboH = 0, fboOK = false;

function setupFBO(w, h) {
  if (!edlProg) { fboOK = false; return; }
  if (fbo) { gl.deleteFramebuffer(fbo); gl.deleteTexture(colorTex); gl.deleteTexture(depthTex); }
  colorTex = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, colorTex);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
  [gl.TEXTURE_MIN_FILTER, gl.TEXTURE_MAG_FILTER].forEach(function (p) { gl.texParameteri(gl.TEXTURE_2D, p, gl.NEAREST); });
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);

  depthTex = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, depthTex);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.DEPTH_COMPONENT24, w, h, 0, gl.DEPTH_COMPONENT, gl.UNSIGNED_INT, null);
  [gl.TEXTURE_MIN_FILTER, gl.TEXTURE_MAG_FILTER].forEach(function (p) { gl.texParameteri(gl.TEXTURE_2D, p, gl.NEAREST); });
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);

  fbo = gl.createFramebuffer();
  gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
  gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, colorTex, 0);
  gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.TEXTURE_2D, depthTex, 0);
  fboOK = gl.checkFramebufferStatus(gl.FRAMEBUFFER) === gl.FRAMEBUFFER_COMPLETE;
  gl.bindFramebuffer(gl.FRAMEBUFFER, null);
  gl.bindTexture(gl.TEXTURE_2D, null);
  fboW = w; fboH = h;
}

function resize() {
  var dpr = Math.min(window.devicePixelRatio || 1, 2);
  var w = Math.round(window.innerWidth * dpr), h = Math.round(window.innerHeight * dpr);
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  if (edlProg && (fboW !== w || fboH !== h)) setupFBO(w, h);
  return dpr;
}

function drawLines(buf, verts, mvp) {
  if (verts <= 0) return;
  gl.useProgram(lnProg);
  gl.uniformMatrix4fv(LU.uMVP, false, new Float32Array(mvp));
  gl.bindBuffer(gl.ARRAY_BUFFER, buf);
  gl.enableVertexAttribArray(0);
  gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 24, 0);
  gl.enableVertexAttribArray(1);
  gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 24, 12);
  gl.drawArrays(gl.LINES, 0, verts);
  gl.disableVertexAttribArray(1);
}

function drawPoints(mvp, dpr) {
  if (totalPoints() === 0) return;
  gl.useProgram(ptProg);
  gl.uniformMatrix4fv(PU.uMVP, false, new Float32Array(mvp));
  gl.uniform1f(PU.uSize, S.psize * dpr);
  gl.uniform1f(PU.uZLo, zLo);
  gl.uniform1f(PU.uZHi, zHi);
  gl.uniform1f(PU.uQMax, qMax);
  gl.uniform1f(PU.uFlip, S.flip ? -1 : 1);
  gl.uniform1f(PU.uThOff, S.thetaOff);
  gl.uniform1f(PU.uSide, S.sideOff);
  gl.uniform1f(PU.uFwd, S.fwdOff);
  gl.uniform1f(PU.uClipLo, S.clipLo);
  gl.uniform1f(PU.uClipHi, S.clipHi);
  gl.uniform1f(PU.uMode, ({ height: 0, quality: 1, white: 2, station: 3 })[S.color] || 0);
  gl.enableVertexAttribArray(0);
  stations.forEach(function (st, i) {
    if (!st.visible) return;
    gl.uniform4f(PU.uPose, st.pose.yaw, st.pose.tx, st.pose.ty, st.pose.tz);
    var c = stationColor(i);
    gl.uniform3f(PU.uStCol, c[0], c[1], c[2]);
    gl.uniform1f(PU.uHi, moveOn && i === selSt ? 1 : 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, st.vbo);
    gl.vertexAttribPointer(0, 4, gl.FLOAT, false, 16, 0);
    gl.drawArrays(gl.POINTS, 0, st.n);
  });
  if (count > 0) {
    gl.uniform4f(PU.uPose, 0, 0, 0, 0);
    gl.uniform3f(PU.uStCol, LIVE_COL[0], LIVE_COL[1], LIVE_COL[2]);
    gl.uniform1f(PU.uHi, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
    gl.vertexAttribPointer(0, 4, gl.FLOAT, false, 16, 0);
    gl.drawArrays(gl.POINTS, 0, count);
  }
}

var renderPending = false;
function requestRender() {
  if (renderPending) return;
  renderPending = true;
  requestAnimationFrame(render);
}

function render() {
  renderPending = false;
  var dpr = resize();
  upload();
  var M = matrices();
  lastM = M;
  var useEDL = S.edl && fboOK;

  gl.bindFramebuffer(gl.FRAMEBUFFER, useEDL ? fbo : null);
  gl.viewport(0, 0, canvas.width, canvas.height);
  gl.clearColor(BG[0], BG[1], BG[2], 1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  gl.enable(gl.DEPTH_TEST);
  drawLines(gridVbo, gridData.length / 6, M.MVP);
  drawPoints(M.MVP, dpr);

  if (useEDL) {
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.viewport(0, 0, canvas.width, canvas.height);
    gl.disable(gl.DEPTH_TEST);
    gl.useProgram(edlProg);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, colorTex);
    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, depthTex);
    gl.uniform1i(EU.uColor, 0);
    gl.uniform1i(EU.uDepth, 1);
    gl.uniform2f(EU.uTexel, 1 / canvas.width, 1 / canvas.height);
    gl.uniform1f(EU.uNear, M.near);
    gl.uniform1f(EU.uFar, M.far);
    gl.uniform1f(EU.uOrtho, topView ? 1 : 0);
    gl.uniform1f(EU.uStrength, S.edlStr);
    gl.uniform1f(EU.uRadius, 1.5 * dpr);
    gl.uniform3f(EU.uBg, BG[0], BG[1], BG[2]);
    gl.bindBuffer(gl.ARRAY_BUFFER, quadVbo);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, null);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, null);
  }

  gl.disable(gl.DEPTH_TEST);
  if (mPts.length) { buildMeasure(); drawLines(measVbo, measVerts, M.MVP); }
}
window.addEventListener('resize', requestRender);
document.addEventListener('visibilitychange', function () { if (!document.hidden) requestRender(); });

// ======================= Export & library =======================
function jetCPU(t, out) {
  out[0] = Math.max(0, Math.min(1, 1.5 - Math.abs(4 * t - 3)));
  out[1] = Math.max(0, Math.min(1, 1.5 - Math.abs(4 * t - 2)));
  out[2] = Math.max(0, Math.min(1, 1.5 - Math.abs(4 * t - 1)));
}

function exportPLY() {
  var P = [];
  eachPoint(1, function (x, y, z, q, s) { if (z >= S.clipLo && z <= S.clipHi) P.push(x, y, z, q, s); });
  var nOut = P.length / 5;
  if (!nOut) { alert('No visible points to export'); return; }
  var header = 'ply\nformat binary_little_endian 1.0\ncomment 2.5D lidar mapper, ' + Math.max(1, stations.length) + ' station(s)\nelement vertex ' + nOut +
    '\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar quality\nend_header\n';
  var buf = new ArrayBuffer(nOut * 16), dv = new DataView(buf), col = [0, 0, 0];
  for (var k = 0; k < nOut; k++) {
    var o = k * 16, x = P[k * 5], y = P[k * 5 + 1], z = P[k * 5 + 2], q = P[k * 5 + 3], s = P[k * 5 + 4];
    dv.setFloat32(o, x, true);
    dv.setFloat32(o + 4, y, true);
    dv.setFloat32(o + 8, z, true);
    var hz = Math.max(0, Math.min(1, (z - zLo) / Math.max(zHi - zLo, 0.001)));
    if (S.color === 'white') { col[0] = col[1] = col[2] = 0.92; }
    else if (S.color === 'quality') jetCPU(Math.min(1, q / qMax), col);
    else if (S.color === 'station') {
      var sc = s < 0 ? LIVE_COL : stationColor(s), f = 0.45 + 0.55 * hz;
      col[0] = sc[0] * f; col[1] = sc[1] * f; col[2] = sc[2] * f;
    }
    else jetCPU(hz, col);
    dv.setUint8(o + 12, Math.round(col[0] * 255));
    dv.setUint8(o + 13, Math.round(col[1] * 255));
    dv.setUint8(o + 14, Math.round(col[2] * 255));
    dv.setUint8(o + 15, q);
  }
  download(new Blob([header, buf], { type: 'application/octet-stream' }), 'scan_' + stamp() + '.ply');
}

function stamp() {
  var d = new Date(), p = function (v) { return String(v).padStart(2, '0'); };
  return d.getFullYear() + p(d.getMonth() + 1) + p(d.getDate()) + '_' + p(d.getHours()) + p(d.getMinutes());
}
function download(blob, name) {
  var a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  document.body.appendChild(a);
  a.click();
  setTimeout(function () { URL.revokeObjectURL(a.href); a.remove(); }, 2000);
}

var db = null;
function openDB() {
  if (!window.indexedDB) { $('library').textContent = 'Saving not supported in this browser'; return; }
  var req;
  try { req = indexedDB.open('lidarScans', 1); } catch (e) { $('library').textContent = 'Phone storage unavailable here'; return; }
  req.onupgradeneeded = function () {
    req.result.createObjectStore('meta', { keyPath: 'id', autoIncrement: true });
    req.result.createObjectStore('data', { keyPath: 'id' });
  };
  req.onsuccess = function () { db = req.result; listScans(); };
  req.onerror = function () { $('library').textContent = 'Could not open phone storage'; };
}

function saveScan() {
  if (!db) { alert('Storage not ready'); return; }
  if (!totalPoints()) { alert('No points to save'); return; }
  var name = prompt('Name this scan', 'Scan ' + stamp());
  if (!name) return;
  var calib = { thetaOff: S.thetaOff, sideOff: S.sideOff, fwdOff: S.fwdOff, flip: S.flip };
  var tx = db.transaction(['meta', 'data'], 'readwrite');
  var metaReq = tx.objectStore('meta').add({ name: name, date: Date.now(), count: totalPoints(), stations: stations.length });
  var sts = stations.map(function (st) { return { raw: st.raw, pose: st.pose, fit: st.fit, rmse: st.rmse, visible: st.visible }; });
  metaReq.onsuccess = function () {
    tx.objectStore('data').put({ id: metaReq.result, raw: raw.slice(0, count * 4), calib: calib, stations: sts });
  };
  tx.oncomplete = listScans;
  tx.onerror = function () { alert('Save failed (phone storage full?)'); };
}

function listScans() {
  if (!db) return;
  var out = [];
  db.transaction('meta').objectStore('meta').openCursor().onsuccess = function (ev) {
    var cur = ev.target.result;
    if (cur) { out.push(cur.value); cur.continue(); return; }
    var el = $('library');
    el.innerHTML = '';
    if (!out.length) { el.textContent = 'No saved scans yet'; return; }
    out.reverse().forEach(function (m) {
      var row = document.createElement('div');
      row.className = 'scan';
      var label = document.createElement('span');
      label.textContent = m.name + ' (' + Math.round(m.count / 1000) + 'k pts' + (m.stations > 1 ? ', ' + m.stations + ' stations' : '') + ')';
      var btns = document.createElement('span');
      var load = document.createElement('button'); load.className = 'alt'; load.textContent = 'Open';
      var del = document.createElement('button'); del.className = 'alt'; del.textContent = 'Delete';
      load.onclick = function () { loadScan(m.id); };
      del.onclick = function () { if (confirm('Delete "' + m.name + '"?')) deleteScan(m.id); };
      btns.appendChild(load); btns.appendChild(del);
      row.appendChild(label); row.appendChild(btns);
      el.appendChild(row);
    });
  };
}

function loadScan(id) {
  db.transaction('data').objectStore('data').get(id).onsuccess = function (ev) {
    var rec = ev.target.result;
    if (!rec) return;
    paused = true;
    $('pauseBtn').textContent = 'Resume live';
    count = Math.min(MAX, rec.raw.length / 4);
    raw.set(rec.raw.subarray(0, count * 4));
    writeIdx = count % MAX;
    pending = 0; replLo = MAX; replHi = 0; fullUpload = true;
    for (var k in rec.calib) S[k] = rec.calib[k];
    saveS(); syncUI();
    clearStations();
    (rec.stations || []).forEach(function (s) {
      var st = makeStation(s.raw, s.raw.length / 4, s.pose);
      st.fit = s.fit; st.rmse = s.rmse; st.visible = s.visible !== false;
      stations.push(st);
    });
    selSt = stations.length ? 0 : -1;
    renderStations();
    mPts = [];
    rebuildDerived();
    fitView();
    $('panel').classList.remove('open');
  };
}

function deleteScan(id) {
  var tx = db.transaction(['meta', 'data'], 'readwrite');
  tx.objectStore('meta').delete(id);
  tx.objectStore('data').delete(id);
  tx.oncomplete = listScans;
}

// ======================= Scanner commands =======================
function get(url) { return fetch(url).catch(function () {}); }
var esp = null, prevState = null;

function pollStatus() {
  fetch('/status').then(function (r) { return r.json(); }).then(function (d) {
    esp = d;
    $('angleVal').textContent = d.angle.toFixed(1) + '°';
    if (document.activeElement !== $('angleSlider') && d.mode !== 'manual') $('angleSlider').value = d.angle;
    ['panStart', 'panEnd', 'panStep', 'rot', 'settle'].forEach(function (id) {
      if (document.activeElement !== $(id)) $(id).value = d[id];
    });
    $('bar').style.width = (d.mode === 'manual' && d.state !== 'done' ? 0 : d.progress * 100) + '%';
    if (prevState && prevState !== 'done' && d.state === 'done') {
      computeRanges(); fitView();
      if (S.autoAdd && count > 0 && !paused) setTimeout(addStation, 300);
    }
    prevState = d.state;
    $('status').innerHTML =
      'Mode: ' + d.mode + ' (' + d.state + ')' +
      '<br>Lidar: ' + (d.lidarOk ? d.lidarHz + ' Hz' : '<span style="color:#e57373">not started</span>') +
      '<br>Samples: ' + d.samples + ' &nbsp;rejected while moving: ' + d.rejected +
      '<br>Dropped: ' + d.dropped + ' &nbsp;timeouts: ' + d.timeouts +
      '<br>WiFi: ' + d.rssi + ' dBm &nbsp;viewers: ' + d.clients +
      '<br>Homed: ' + (d.homed ? 'yes' : 'no') + ' &nbsp;uptime: ' + d.uptime + ' s';
  }).catch(function () {});
}

function scanOnce() {
  clearPoints();
  if (esp) { esp.state = 'moving'; esp.mode = 'once'; esp.progress = 0; prevState = 'moving'; }
  paused = false;
  $('pauseBtn').textContent = 'Pause';
  get('/scanOnce');
  $('panel').classList.remove('open');
}

// ======================= HUD =======================
function esc(t) { return String(t).replace(/[&<>]/g, function (c) { return { '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]; }); }
function updateHud() {
  var stateLine = '';
  if (esp) {
    if (esp.state === 'done') stateLine = '<br><b style="color:#8bc34a">Scan complete &mdash; tap + Station or save it in Controls</b>';
    else if (esp.mode !== 'manual') stateLine = '<br>Scanning ' + Math.round(esp.progress * 100) + '% (' + esp.state + ')';
  }
  var aLine = (alignMsg && (busy || Date.now() < alignMsgUntil)) ? '<br><b style="color:#ffca28">' + esc(alignMsg) + '</b>' : '';
  $('hud').innerHTML =
    (wsConnected ? '<span style="color:#8bc34a">&#9679; Live</span>' : '<span style="color:#e57373">&#9679; Reconnecting</span>') +
    (paused ? ' (paused)' : '') + stateLine + aLine +
    '<br>Points: ' + totalPoints().toLocaleString() +
    (stations.length ? ' (' + stations.length + ' station' + (stations.length > 1 ? 's' : '') + ' + live)' : '') +
    '<br>Stream: ' + rxRate.toLocaleString() + ' samples/s' +
    '<br>Pan: ' + (lastPan === null ? '--' : lastPan.toFixed(1) + '&deg;') +
    (topView ? '<br>Top view' : '') +
    (measureOn ? '<br>Measure: ' + measureText() : '') +
    (moveOn ? '<br>Moving station ' + (selSt + 1) + ': drag / twist' : '');
}
setInterval(function () {
  rxRate = rxSec; rxSec = 0;
  if (count !== lastRangeCount && count > 0) computeRanges();
  updateHud();
}, 1000);

// ======================= UI wiring =======================
function syncUI() {
  $('psize').value = S.psize;
  $('voxel').value = String(S.voxel);
  $('minq').value = S.minq;
  $('maxr').value = S.maxr;
  $('flip').checked = S.flip;
  $('color').value = S.color;
  $('edl').checked = S.edl;
  $('edl').disabled = !edlProg;
  $('edlStr').value = S.edlStr;
  $('thetaOff').value = S.thetaOff;
  $('sideOff').value = S.sideOff;
  $('fwdOff').value = S.fwdOff;
  $('clipLo').value = S.clipLo;
  $('clipHi').value = S.clipHi;
  $('autoAdd').checked = S.autoAdd;
  calLabels();
}
function calLabels() {
  $('thOffV').textContent = (+S.thetaOff).toFixed(1) + '°';
  $('sideV').textContent = (S.sideOff * 100).toFixed(1) + ' cm';
  $('fwdV').textContent = (S.fwdOff * 100).toFixed(1) + ' cm';
}
function setTop(on) {
  topView = on;
  if (!on && moveOn) { moveOn = false; $('moveStBtn').textContent = 'Move by hand: off'; $('moveStBtn').className = 'alt'; }
  $('topBtn').className = on ? 'on' : 'alt';
  $('qTop').className = on ? 'on' : 'alt';
  fitView();
}
function setMeasure(on) {
  measureOn = on;
  $('measureBtn').textContent = 'Measure: ' + (on ? 'on' : 'off');
  $('measureBtn').className = on ? 'on' : 'alt';
  $('qMeasure').className = on ? 'on' : 'alt';
  if (!on) { mPts = []; buildMeasure(); }
  requestRender();
}

var rebuildTimer = null;
function scheduleRebuild() { clearTimeout(rebuildTimer); rebuildTimer = setTimeout(rebuildDerived, 300); }

$('toggle').onclick = function () { $('panel').classList.toggle('open'); };
$('scanBtn').onclick = scanOnce;
$('qScan').onclick = scanOnce;
$('autoBtn').onclick = function () { paused = false; $('pauseBtn').textContent = 'Pause'; get('/setMode?mode=auto'); };
$('manualBtn').onclick = function () { get('/setMode?mode=manual'); };
$('applyBtn').onclick = function () {
  get('/setSweepParams?panStart=' + $('panStart').value + '&panEnd=' + $('panEnd').value + '&panStep=' + $('panStep').value +
      '&rot=' + $('rot').value + '&settle=' + $('settle').value);
};
$('zeroBtn').onclick = function () { if (confirm('Make the current position 0°?')) get('/setZero'); };
$('lidarBtn').onclick = function () { get('/restartLidar'); };
var lastSlide = 0;
$('angleSlider').oninput = function () {
  $('angleVal').textContent = this.value + '°';
  var now = Date.now();
  if (now - lastSlide > 150) { lastSlide = now; get('/setAngle?angle=' + this.value); }
};
$('angleSlider').onchange = function () { get('/setAngle?angle=' + this.value); };

$('pauseBtn').onclick = function () { paused = !paused; this.textContent = paused ? 'Resume live' : 'Pause'; };
$('clearBtn').onclick = clearPoints;
$('fitBtn').onclick = fitView;
$('qFit').onclick = fitView;
$('topBtn').onclick = function () { setTop(!topView); };
$('qTop').onclick = function () { setTop(!topView); };
$('color').onchange = function () { S.color = this.value; saveS(); requestRender(); };
$('edl').onchange = function () { S.edl = this.checked; saveS(); requestRender(); };
$('edlStr').oninput = function () { S.edlStr = parseFloat(this.value); saveS(); requestRender(); };
$('psize').oninput = function () { S.psize = parseFloat(this.value); saveS(); requestRender(); };

$('clipLo').onchange = function () { S.clipLo = parseFloat(this.value); if (isNaN(S.clipLo)) S.clipLo = -99; saveS(); requestRender(); };
$('clipHi').onchange = function () { S.clipHi = parseFloat(this.value); if (isNaN(S.clipHi)) S.clipHi = 99; saveS(); requestRender(); };
$('floorBtn').onclick = function () {
  computeRanges();
  S.clipLo = +(floorZ + 0.5).toFixed(2);
  S.clipHi = +(floorZ + 1.5).toFixed(2);
  saveS(); syncUI(); setTop(true);
  $('panel').classList.remove('open');
};
$('noClipBtn').onclick = function () { S.clipLo = -99; S.clipHi = 99; saveS(); syncUI(); setTop(false); };

$('measureBtn').onclick = function () { setMeasure(!measureOn); };
$('qMeasure').onclick = function () { setMeasure(!measureOn); };

$('voxel').onchange = function () { S.voxel = parseFloat(this.value); saveS(); scheduleRebuild(); };
$('minq').onchange = function () { S.minq = parseInt(this.value, 10) || 0; saveS(); };
$('maxr').onchange = function () { S.maxr = parseFloat(this.value) || 12; saveS(); };

['thetaOff', 'sideOff', 'fwdOff'].forEach(function (id) {
  $(id).oninput = function () { S[id] = parseFloat(this.value); calLabels(); requestRender(); };
  $(id).onchange = function () { S[id] = parseFloat(this.value); saveS(); scheduleRebuild(); };
});
$('flip').onchange = function () { S.flip = this.checked; saveS(); scheduleRebuild(); };
$('calResetBtn').onclick = function () {
  S.thetaOff = 0; S.sideOff = 0; S.fwdOff = 0; S.flip = false;
  saveS(); syncUI(); scheduleRebuild();
};

$('saveBtn').onclick = saveScan;
$('addStBtn').onclick = addStation;
$('qStation').onclick = addStation;
$('autoAdd').onchange = function () { S.autoAdd = this.checked; saveS(); };
document.querySelectorAll('.rotb').forEach(function (b) {
  b.onclick = function () { rotateSel(parseFloat(b.getAttribute('data-rot'))); };
});
$('moveStBtn').onclick = function () { setMove(!moveOn); if (moveOn) $('panel').classList.remove('open'); };
$('snapBtn').onclick = function () { setMove(false); realign(false); };
$('reautoBtn').onclick = function () { setMove(false); realign(true); };
$('clearStBtn').onclick = function () { if (confirm('Remove all stations?')) clearStations(); };
$('plyBtn').onclick = exportPLY;

// ======================= Start =======================
syncUI();
renderStations();
buildMeasure();
openDB();
connect();
pollStatus();
setInterval(pollStatus, 1000);
requestRender();
})();
</script>
</body>
</html>
)rawliteral";

// =====================================================================
// HTTP handlers
// =====================================================================
const char* modeName() {
  return mode == MODE_AUTO ? "auto" : (mode == MODE_ONCE ? "once" : "manual");
}
const char* stateName() {
  if (scanComplete && state == ST_HOLD) return "done";
  switch (state) {
    case ST_MOVING: return "moving";
    case ST_SETTLING: return "settling";
    case ST_COLLECTING: return "collecting";
    default: return "hold";
  }
}

void handleRoot() { server.send_P(200, "text/html", PAGE_HTML); }   // streams from flash, no 70 KB heap copy

void handleSetAngle() {
  if (server.hasArg("angle")) {
    mode = MODE_MANUAL;
    scanComplete = false;
    manualTargetDeg = constrain(server.arg("angle").toFloat(), 0.0f, 360.0f);
  }
  server.send(200, "text/plain", "OK");
}

void handleSetMode() {
  String m = server.arg("mode");
  if (m == "auto") {
    mode = MODE_AUTO;
    scanComplete = false;
    panDirection = 1;
    sweepTargetDeg = constrain(currentDeg(), (float)panStart, (float)panEnd);
    startMoveTo(sweepTargetDeg);
  } else if (m == "manual") {
    mode = MODE_MANUAL;
    manualTargetDeg = targetDeg;
    if (state == ST_HOLD) { state = ST_SETTLING; stateSince = millis(); }
  }
  server.send(200, "text/plain", "OK");
}

void handleScanOnce() {
  mode = MODE_ONCE;
  scanComplete = false;
  panDirection = 1;
  sweepPass++;
  sweepTargetDeg = panStart;
  startMoveTo(panStart);
  server.send(200, "text/plain", "OK");
}

void handleSetSweepParams() {
  if (server.hasArg("panStart")) panStart = server.arg("panStart").toInt();
  if (server.hasArg("panEnd")) panEnd = server.arg("panEnd").toInt();
  if (server.hasArg("panStep")) panStep = server.arg("panStep").toFloat();
  if (server.hasArg("rot")) dwellRotations = (uint8_t)constrain(server.arg("rot").toInt(), 1, 20);
  if (server.hasArg("settle")) settleMs = constrain(server.arg("settle").toInt(), 0, 2000);

  panStart = constrain(panStart, 0, 359);
  panEnd = constrain(panEnd, 1, 360);
  if (panStart >= panEnd) panEnd = panStart + 1;
  if (panStep < 0.1f) panStep = 0.1f;
  sweepTargetDeg = constrain(sweepTargetDeg, (float)panStart, (float)panEnd);
  server.send(200, "text/plain", "OK");
}

void handleSetZero() {
  if (!stepper->isRunning()) {
    stepper->setCurrentPosition(0);
    targetDeg = 0;
    manualTargetDeg = 0;
    sweepTargetDeg = 0;
    mode = MODE_MANUAL;
  }
  server.send(200, "text/plain", "OK");
}

void handleRestartLidar() {
  g_lidarRestart = true;
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  float progress = (sweepTargetDeg - panStart) / (float)(panEnd - panStart);
  String j = "{";
  j += "\"angle\":" + String(currentDeg(), 2);
  j += ",\"target\":" + String(targetDeg, 2);
  j += ",\"mode\":\"" + String(modeName()) + "\"";
  j += ",\"state\":\"" + String(stateName()) + "\"";
  j += ",\"progress\":" + String(constrain(progress, 0.0f, 1.0f), 3);
  j += ",\"pass\":" + String(sweepPass);
  j += ",\"panStart\":" + String(panStart);
  j += ",\"panEnd\":" + String(panEnd);
  j += ",\"panStep\":" + String(panStep, 2);
  j += ",\"rot\":" + String(dwellRotations);
  j += ",\"settle\":" + String(settleMs);
  j += ",\"lidarOk\":" + String(g_lidarOk ? "true" : "false");
  j += ",\"lidarHz\":" + String(lidarHz);
  j += ",\"samples\":" + String(g_samplesRead);
  j += ",\"rejected\":" + String(g_rejected);
  j += ",\"dropped\":" + String(g_dropped);
  j += ",\"timeouts\":" + String(lidarTimeouts);
  j += ",\"clients\":" + String(webSocket.connectedClients());
  j += ",\"rssi\":" + String(USE_AP_MODE ? 0 : WiFi.RSSI());
  j += ",\"homed\":" + String(homed ? "true" : "false");
  j += ",\"uptime\":" + String((millis() - bootTime) / 1000);
  j += "}";
  server.send(200, "application/json", j);
}

// =====================================================================
// WiFi
// =====================================================================
void startWiFi() {
  if (USE_AP_MODE) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("AP started: join \"%s\", open http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
    Serial.printf("\nConnected. Open http://%s.local or http://%s\n", MDNS_NAME, WiFi.localIP().toString().c_str());
  }
  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("ws", "tcp", 81);
  }
}

void checkWiFi() {
  if (USE_AP_MODE || millis() - lastWifiCheck < 5000) return;
  lastWifiCheck = millis();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi dropped, reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// =====================================================================
// Setup / loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  LidarSerial.setRxBufferSize(4096);
  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  sampleQueue = xQueueCreate(1024, sizeof(PhonePoint));

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    Serial.println("Stepper init FAILED — check STEP_PIN");
    while (true) delay(1000);
  }
  stepper->setDirectionPin(DIR_PIN);
  stepper->setSpeedInHz(MAX_SPEED_HZ);
  stepper->setAcceleration(ACCEL_STEPS_S2);

  if (HOME_PIN >= 0) homeStepper();
  else stepper->setCurrentPosition(lroundf(panStart * STEPS_PER_DEGREE));

  targetDeg = manualTargetDeg = sweepTargetDeg = currentDeg();
  state = ST_SETTLING;
  stateSince = millis();

  startWiFi();

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/setAngle", handleSetAngle);
  server.on("/setMode", handleSetMode);
  server.on("/scanOnce", handleScanOnce);
  server.on("/setSweepParams", handleSetSweepParams);
  server.on("/setZero", handleSetZero);
  server.on("/restartLidar", handleRestartLidar);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(onWsEvent);

  // Lidar task on core 1, higher priority than loop() so it never falls behind
  xTaskCreatePinnedToCore(lidarTask, "lidar", 4096, nullptr, 3, nullptr, 1);

  bootTime = millis();
}

void loop() {
  server.handleClient();
  webSocket.loop();
  checkWiFi();
  updateSweep();
  drainSamples();

  if (millis() - lastStats >= 1000) {
    lastStats = millis();
    uint32_t seen = g_rotSeen;
    lidarHz = seen - lastRotSeen;
    lastRotSeen = seen;
  }
}