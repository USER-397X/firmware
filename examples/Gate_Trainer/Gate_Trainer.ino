// ============================================================
//  Qbead Gate Trainer  ·  Firmware v3  (Bluetooth + USB)
// ------------------------------------------------------------
//  One firmware for both modes of the web app (QbeadGateTrainer.html):
//
//   FREE PLAY  - apply any gate (X, Y, Z, H, S, S-dagger, T), reset to |0>,
//                measure in the Z basis.
//   GATE GOLF  - levels with start state, target state, allowed gates
//                and par. Measuring is disabled here.
//
//  The bead holds the qubit state and executes gates PHYSICALLY.
//  The web app chooses the mode, the level and which gate is armed.
//
//  Connection: Bluetooth (Nordic UART service) or USB cable, same
//  text protocol. The bead does NOT wait for a serial monitor, so it
//  also runs on battery. (For USB: close the Arduino Serial Monitor.)
//
//  How a gate is executed:
//   - Click a gate in the web app -> it is "armed".
//   - The two ends of its rotation axis light up ORANGE.
//   - Hold the bead so the orange axis is roughly HORIZONTAL and roll
//     the bead around that axis. The blue dot stays "up" in the room
//     while the sphere turns underneath it.
//   - After the full rotation (180 deg for X/Y/Z/H, 90 for S and S-dagger,
//     45 for T) the gate snaps in.
//   - Orange blinking = axis is too vertical, tilt the bead first.
//   - Dot turns RED   = you are rolling around the wrong axis.
//
//  Taps on the bead:
//   - single tap = measure in the Z basis (free play only, no gate armed)
//   - double tap = show the reference axes for a moment
//       RED  = x-z meridian (path of the Y gate)
//       BLUE = y-z meridian (path of the X gate)
//
//  Colours:  BLUE = current state, GREEN = target (or where the armed
//            gate will take you), ORANGE = rotation axis of the armed gate.
//
//  Commands (one per line, also typeable in the Serial Monitor):
//    HELLO                     -> bead answers READY
//    LEVEL sx sy sz            -> free play, start state (Bloch vector)
//    LEVEL sx sy sz tx ty tz   -> gate golf, start + target
//    SET x y z                 -> set the state (e.g. INIT: SET 0 0 1)
//    ARM X|Y|Z|H|S|D|T         -> arm a gate (D = S-dagger)
//    CANCEL                    -> cancel the armed gate
//    MEASURE                   -> measure in the Z basis (free play)
//    AXES                      -> show the reference axes
//    WIN n                     -> victory animation with n stars
//    IDLE                      -> waiting mode
//    BRIGHT n                  -> brightness 0..255
//
//  Output:
//    READY ...                 -> ready
//    S x y z p g o v           -> status (~15 times per second): state,
//                                 progress 0..1, armed gate ('-' = none),
//                                 wrong axis 0/1, axis too vertical 0/1
//    DONE G x y z              -> gate G finished, new state
//    MEAS b x y z              -> measured bit b (0/1), new state
//    TAP n                     -> tap that did not measure (n = 1 or 2)
// ============================================================

#include <Qbead.h>

// ---------------- Types ----------------
// All custom types must be defined BEFORE the first function, because the
// Arduino IDE inserts automatic function prototypes at that point.
struct V3 { float x, y, z; };
struct LineBuf { char buf[100]; uint8_t len; };
struct GateDef { char name; V3 n; float ang; };

Qbead::Qbead bead;
BLEUart bleuart;   // Nordic UART service: text over Bluetooth

// ---------------- Settings ----------------
const char          BLE_NAME[]    = "qbead trainer";
const uint8_t       BRIGHT        = 30;     // LED brightness
const float         DONE_TOL_DEG  = 12.0;   // gate snaps in this many degrees before the end
const float         AXIS_TOL_DEG  = 35.0;   // allowed deviation from the correct rotation axis
const float         MIN_PLANE     = 0.45;   // axis must be at least ~27 deg away from vertical
const float         WINDOW_DEG    = 1.0;    // evaluate motion once this much has accumulated
const unsigned long WINDOW_MAX_MS = 500;    // smaller motion after this time = noise, discard
const unsigned long STATUS_MS     = 66;     // status ~15 Hz
const unsigned long DOUBLE_TAP_MS = 400;    // second tap within this time = double tap
const unsigned long TAP_DEBOUNCE  = 80;     // ignore tap echoes shorter than this
const unsigned long AXES_SHOW_MS  = 1500;   // how long the reference axes are shown

// ---------------- Colours ----------------
uint32_t BLUE   = color(0,   90, 255);
uint32_t GREEN  = color(0,  255,  40);
uint32_t ORANGE = color(255, 100,  0);
uint32_t RED    = color(255,   0,  0);
uint32_t WHITE  = color(255, 255, 255);
uint32_t AX_RED  = color(120,   0,  0);   // reference axes, dimmer so dots stay visible
uint32_t AX_BLUE = color(0,    30, 140);

// ---------------- Small vector maths ----------------
static inline V3    mk(float x, float y, float z) { V3 r = {x, y, z}; return r; }
static inline float dotp(V3 a, V3 b)   { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline V3    crossp(V3 a, V3 b) { return mk(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x); }
static inline V3    scl(V3 a, float s) { return mk(a.x*s, a.y*s, a.z*s); }
static inline V3    addv(V3 a, V3 b)   { return mk(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3    subv(V3 a, V3 b)   { return mk(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline float lenv(V3 a)         { return sqrtf(dotp(a, a)); }
static V3 unitv(V3 a) { float l = lenv(a); return (l < 1e-6f) ? mk(0, 0, 1) : scl(a, 1.0f / l); }

// Rotate v around the unit axis n by deg degrees (Rodrigues, right-hand rule)
static V3 rotv(V3 v, V3 n, float deg) {
  float r = deg * DEG_TO_RAD, c = cosf(r), s = sinf(r);
  return addv(addv(scl(v, c), scl(crossp(n, v), s)), scl(n, dotp(n, v) * (1.0f - c)));
}

static float angleBetween(V3 a, V3 b) {
  return acosf(constrain(dotp(unitv(a), unitv(b)), -1.0f, 1.0f)) * RAD_TO_DEG;
}

// Snap to one of the 6 axis states when very close (prevents drift)
static V3 snapToAxes(V3 v) {
  const V3 A[6] = {{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};
  v = unitv(v);
  for (int i = 0; i < 6; i++) if (dotp(v, A[i]) > 0.995f) return A[i];
  return v;
}

// ---------------- Gates: rotation axis + angle ----------------
const float R2 = 0.70710678f;
GateDef GATES[] = {
  {'X', {1, 0, 0},   180},
  {'Y', {0, 1, 0},   180},
  {'Z', {0, 0, 1},   180},
  {'H', {R2, 0, R2}, 180},   // diagonal between x and z
  {'S', {0, 0, 1},    90},
  {'D', {0, 0, -1},   90},   // S-dagger = -90 deg about z
  {'T', {0, 0, 1},    45},
};
const int NGATES = sizeof(GATES) / sizeof(GATES[0]);

// ---------------- State ----------------
enum Mode { M_IDLE, M_PLAY };
Mode mode = M_IDLE;

V3   state     = {0, 0, 1};   // current qubit state (Bloch vector)
V3   target    = {0, 0, 1};
bool hasTarget = false;       // true = gate golf, false = free play

int   armed    = -1;          // index of the armed gate, -1 = none
V3    armStart;               // state at the moment the gate was armed
float beta     = 0;           // angle rolled around the gate axis so far

bool  prevValid = false;
V3    prevG;
float winB = 0, winE = 0;
unsigned long winStart = 0;
float tanTol = 0.7f;

bool axisVertical = false;
unsigned long offAxisUntil = 0;
unsigned long lastStatus = 0;
unsigned long axesUntil = 0;

bool tapPending = false;
unsigned long lastTapAt = 0;

LineBuf usbLine = {{0}, 0};
LineBuf bleLine = {{0}, 0};

// ---------------- Output to USB and Bluetooth ----------------
bool bleReady() { return Bluefruit.connected() && bleuart.notifyEnabled(); }

void out(const String &line) {
  if (Serial && Serial.availableForWrite() > (int)line.length() + 2) Serial.println(line);
  if (bleReady()) {
    // One notification carries at most MTU-3 bytes (20 with the default MTU), and longer
    // writes lose their tail. Send the whole line plus newline in pieces that fit.
    String msg = line + "\n";
    const size_t chunk = max(20, (int)Bluefruit.Connection(Bluefruit.connHandle())->getMtu() - 3);
    for (size_t i = 0; i < msg.length(); i += chunk) {
      size_t n = min(chunk, msg.length() - i);
      bleuart.write((const uint8_t *)msg.c_str() + i, n);
    }
  }
}

String vecStr(V3 v, int d) { return String(v.x, d) + " " + String(v.y, d) + " " + String(v.z, d); }

// ---------------- Display ----------------
void drawV(V3 v, uint32_t c) {
  BlochVector b(v.x, v.y, v.z);
  bead.setBloch_deg(b.theta, b.phi, c);
}

int legForPhi(float phiDeg) {
  int n = (int)roundf(phiDeg / bead.phi_quant);
  return ((n % bead.nlegs) + bead.nlegs) % bead.nlegs;
}

void drawMeridian(float phiDeg, uint32_t c) {
  int a = legForPhi(phiDeg), b = legForPhi(phiDeg + 180);
  for (int s = 0; s <= bead.nsections; s++) {
    bead.setLegPixelColor(a, s, c);
    bead.setLegPixelColor(b, s, c);
  }
}

V3 shownState() {
  if (armed < 0) return state;
  const GateDef &G = GATES[armed];
  // 180 deg gates: real rolling direction (dot stays put in the room).
  // S / S-dagger / T: direction does not matter, the dot always moves the correct way.
  float a = (G.ang >= 179.0f) ? beta : fabsf(beta);
  if (a >  G.ang) a =  G.ang;
  if (a < -G.ang) a = -G.ang;
  return rotv(armStart, G.n, a);
}

float progress() {
  if (armed < 0) return 0;
  return constrain(fabsf(beta) / GATES[armed].ang, 0.0f, 1.0f);
}

void render() {
  unsigned long now = millis();
  bead.clear();

  if (mode == M_IDLE) {
    float b = 0.5f + 0.5f * sinf(now / 400.0f);
    uint8_t v = 15 + (uint8_t)(60 * b);
    // pulsing north pole = ready; blue tint once a device is connected
    bead.setLegPixelColor(0, 0, Bluefruit.connected() ? color(0, v / 2, v) : color(v, v, v));
    bead.show();
    return;
  }

  if (now < axesUntil) {                    // reference axes (double tap / new level)
    drawMeridian(0,  AX_RED);               // x-z meridian
    drawMeridian(90, AX_BLUE);              // y-z meridian
  }

  if (armed >= 0) {
    const GateDef &G = GATES[armed];
    bool on = !axisVertical || ((now / 150) % 2 == 0);   // blinks if axis too vertical
    if (on) {
      drawV(G.n, ORANGE);
      drawV(scl(G.n, -1), ORANGE);
    }
    if (!hasTarget) drawV(rotv(armStart, G.n, G.ang), GREEN);   // free play: where the gate leads
  }
  if (hasTarget) drawV(target, GREEN);

  bool off = (armed >= 0) && (now < offAxisUntil);
  drawV(shownState(), off ? RED : BLUE);
  bead.show();
}

// Wave running across the sphere, starting at point c
void wave(V3 c, int stars) {
  for (int r = 0; r <= 230; r += 12) {
    bead.clear();
    for (int leg = 0; leg < bead.nlegs; leg++) {
      for (int pix = 0; pix <= bead.nsections; pix++) {
        BlochVector d(bead.theta_quant * (float)pix, bead.phi_quant * (float)leg);
        float a = angleBetween(c, mk(d.x, d.y, d.z));
        if (a <= r && a > r - 50) {
          uint32_t col;
          if (stars >= 3)       col = colorWheel_deg(fmodf(a * 2.0f + r * 2.0f, 360.0f));
          else if (stars == 2)  col = color(255, 170, 40);
          else                  col = WHITE;
          bead.setLegPixelColor(leg, pix, col);
        }
      }
    }
    bead.show();
    delay(40);
  }
}

void celebrate(int stars) {
  V3 c = hasTarget ? target : state;
  wave(c, stars);
  if (stars >= 3) {                       // rainbow flashes for par
    for (int k = 0; k < 2; k++) {
      for (int leg = 0; leg < bead.nlegs; leg++)
        for (int pix = 0; pix <= bead.nsections; pix++)
          bead.setLegPixelColor(leg, pix, colorWheel_deg(fmodf(leg * 30.0f + k * 90.0f, 360.0f)));
      bead.show(); delay(180);
      bead.clear(); bead.show(); delay(120);
    }
  }
  bead.clear(); bead.show();
}

void flashDone(V3 v) {
  bead.clear();
  drawV(v, WHITE);
  bead.show();
  delay(120);
}

// ---------------- Gate logic ----------------
void armGate(int idx) {
  if (armed >= 0) state = armStart;   // another gate was armed -> reset it
  armed = idx;
  armStart = state;
  beta = 0;
  prevValid = false;
  winB = winE = 0;
  winStart = millis();
  offAxisUntil = 0;
  mode = M_PLAY;
}

void completeGate() {
  const GateDef &G = GATES[armed];
  state = snapToAxes(rotv(armStart, G.n, G.ang));   // exact gate result
  armed = -1;
  flashDone(state);
  out(String("DONE ") + G.name + " " + vecStr(state, 4));
}

// Measures how far the bead has been rolled around the gate axis.
// g = direction of gravity in the bead's own coordinate frame.
void updateArmed(V3 g) {
  const GateDef &G = GATES[armed];
  float gn  = dotp(g, G.n);
  V3    gp  = subv(g, scl(G.n, gn));      // part perpendicular to the axis
  float gpl = lenv(gp);
  axisVertical = (gpl < MIN_PLANE);

  if (prevValid) {
    float pn  = dotp(prevG, G.n);
    V3    pp  = subv(prevG, scl(G.n, pn));
    float ppl = lenv(pp);
    if (gpl >= MIN_PLANE && ppl >= MIN_PLANE) {
      V3 a = scl(pp, 1.0f / ppl), b = scl(gp, 1.0f / gpl);
      // rotation AROUND the axis (wanted)
      winB += atan2f(dotp(G.n, crossp(a, b)), dotp(a, b)) * RAD_TO_DEG;
      // tilting OF the axis (unwanted)
      winE += (asinf(constrain(gn, -1.0f, 1.0f)) - asinf(constrain(pn, -1.0f, 1.0f))) * RAD_TO_DEG;
    }
  }
  prevG = g;
  prevValid = true;

  // evaluate once enough motion has accumulated
  if (fabsf(winB) + fabsf(winE) > WINDOW_DEG) {
    if (fabsf(winE) > tanTol * fabsf(winB)) offAxisUntil = millis() + 300;   // wrong axis
    else                                     beta += winB;                    // correct axis
    winB = winE = 0;
    winStart = millis();
  } else if (millis() - winStart > WINDOW_MAX_MS) {
    winB = winE = 0;                    // just noise
    winStart = millis();
  }

  if (fabsf(beta) >= G.ang - DONE_TOL_DEG) completeGate();
}

// ---------------- Measurement ----------------
// Z-basis measurement: |0> with probability (1 + z) / 2, then the state collapses.
bool canMeasure() { return mode == M_PLAY && !hasTarget && armed < 0; }

void measure() {
  if (!canMeasure()) return;
  float p0 = (1.0f + state.z) / 2.0f;
  randomSeed(micros());
  int bit = (random(0, 10000) / 10000.0f < p0) ? 0 : 1;
  state = bit ? mk(0, 0, -1) : mk(0, 0, 1);

  // short animation: flash, then the result pole lights up
  for (int leg = 0; leg < bead.nlegs; leg++)
    for (int pix = 0; pix <= bead.nsections; pix++)
      bead.setLegPixelColor(leg, pix, color(40, 40, 40));
  bead.show(); delay(120);
  bead.clear(); drawV(state, WHITE); bead.show(); delay(350);

  out(String("MEAS ") + bit + " " + vecStr(state, 4));
}

// ---------------- Taps ----------------
// The IMU reports single taps; double taps are recognised by timing.
void onTap() {
  unsigned long now = millis();
  if (now - lastTapAt < TAP_DEBOUNCE) return;
  if (tapPending && now - lastTapAt < DOUBLE_TAP_MS) {   // second tap -> double tap
    tapPending = false;
    lastTapAt = now;
    axesUntil = now + AXES_SHOW_MS;
    out("TAP 2");
    return;
  }
  tapPending = true;
  lastTapAt = now;
}

void checkPendingTap() {
  if (!tapPending || millis() - lastTapAt < DOUBLE_TAP_MS) return;
  tapPending = false;                                     // it was a single tap
  if (canMeasure()) measure();
  else out("TAP 1");
}

// ---------------- Communication ----------------
void printReady() { out("READY qbead-trainer 3"); }

void sendStatus() {
  String line = "S " + vecStr(shownState(), 3) + " "
              + String(progress(), 2) + " "
              + String(armed >= 0 ? GATES[armed].name : '-') + " "
              + String((armed >= 0 && millis() < offAxisUntil) ? 1 : 0) + " "
              + String((armed >= 0 && axisVertical) ? 1 : 0);
  out(line);
}

int readFloats(float *v, int maxN) {
  int n = 0; char *t;
  while (n < maxN && (t = strtok(NULL, " \t")) != NULL) v[n++] = atof(t);
  return n;
}

void handleCommand(char *line) {
  char *tok = strtok(line, " \t");
  if (!tok) return;
  for (char *p = tok; *p; p++) *p = toupper(*p);

  if (!strcmp(tok, "HELLO")) {
    printReady();

  } else if (!strcmp(tok, "LEVEL")) {
    float v[6];
    int n = readFloats(v, 6);
    if (n < 3) return;
    state = snapToAxes(mk(v[0], v[1], v[2]));
    hasTarget = (n >= 6);
    if (hasTarget) target = snapToAxes(mk(v[3], v[4], v[5]));
    armed = -1;
    mode = M_PLAY;
    axesUntil = millis() + AXES_SHOW_MS;   // orientation help at the start

  } else if (!strcmp(tok, "SET")) {
    float v[3];
    if (readFloats(v, 3) < 3) return;
    state = snapToAxes(mk(v[0], v[1], v[2]));
    armed = -1;
    mode = M_PLAY;

  } else if (!strcmp(tok, "ARM")) {
    char *t = strtok(NULL, " \t");
    if (!t) return;
    char g = toupper(t[0]);
    for (int i = 0; i < NGATES; i++) if (GATES[i].name == g) { armGate(i); return; }

  } else if (!strcmp(tok, "CANCEL")) {
    if (armed >= 0) { state = armStart; armed = -1; }

  } else if (!strcmp(tok, "MEASURE")) {
    measure();

  } else if (!strcmp(tok, "AXES")) {
    axesUntil = millis() + AXES_SHOW_MS;

  } else if (!strcmp(tok, "WIN")) {
    char *t = strtok(NULL, " \t");
    celebrate(t ? atoi(t) : 1);

  } else if (!strcmp(tok, "IDLE")) {
    armed = -1;
    mode = M_IDLE;

  } else if (!strcmp(tok, "BRIGHT")) {
    char *t = strtok(NULL, " \t");
    if (t) bead.setBrightness(constrain(atoi(t), 0, 255));
  }
}

void feed(LineBuf &lb, char c) {
  if (c == '\n' || c == '\r') {
    if (lb.len) { lb.buf[lb.len] = 0; handleCommand(lb.buf); lb.len = 0; }
  } else if (lb.len < sizeof(lb.buf) - 1) {
    lb.buf[lb.len++] = c;
  }
}

void pollInput() {
  while (Serial.available())  feed(usbLine, (char)Serial.read());
  while (bleuart.available()) feed(bleLine, (char)bleuart.read());
}

// ---------------- Bluetooth setup ----------------
void setupBLE() {
  // Larger MTU and notification queue, so ~15 status lines per second get through.
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin(1, 0);
  Bluefruit.setTxPower(4);
  Bluefruit.setName(BLE_NAME);

  // Use a slightly different address than the old Qbead firmware, so the
  // laptop sees a "new" device and does not reuse cached (outdated) services.
  ble_gap_addr_t addr;
  if (sd_ble_gap_addr_get(&addr) == NRF_SUCCESS) {
    addr.addr[0] ^= 0x5A;
    addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    addr.addr[5] |= 0xC0;                 // keep it a valid random static address
    sd_ble_gap_addr_set(&addr);
  }

  bleuart.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

// ---------------- Arduino ----------------
void setup() {
  // Our own start-up instead of bead.begin(): bead.begin() blocks until a
  // serial monitor is open, which would make Bluetooth-only use impossible.
  Serial.begin(115200);
  bead.pixels.begin();
  bead.setBrightness(BRIGHT);
  bead.clear(); bead.show();
  bead.imu.begin();
  Qbead::Qbead::singletoninstance = &bead;   // needed by the library's tap interrupt
  bead.setupIMUTapDetection();
  setupBLE();

  tanTol = tanf(AXIS_TOL_DEG * DEG_TO_RAD);
  randomSeed(micros());
  wave(mk(0, 0, 1), 3);             // short start-up animation
  bead.clear(); bead.show();
  bead.wasTapped();                 // discard taps from handling during start-up
  printReady();
}

void loop() {
  bead.readIMU(false);
  pollInput();

  if (bead.wasTapped()) onTap();
  checkPendingTap();

  if (mode == M_PLAY && armed >= 0) {
    updateArmed(unitv(mk(bead.x, bead.y, bead.z)));
  }

  render();

  if (mode == M_PLAY && millis() - lastStatus >= STATUS_MS) {
    lastStatus = millis();
    sendStatus();
  }
}
