// # Gate Trainer
//
// In this sketch the Qbead holds the state of a qubit and applies quantum gates
// physically: you roll the bead about a gate's rotation axis, and the state turns
// with it. A web page (the Quantum Gates lesson on the qbead website, or the gate
// trainer app) picks which gate is armed and shows the state on a Bloch sphere.
//
// The page offers two modes:
//
// - **Free play**: apply any gate (X, Y, Z, H, S, S-dagger, T), reset to |0>, and
//   measure in the Z basis.
// - **Gate Golf**: reach a target state from a start state with only a few allowed
//   gates, in as few gates as possible. Measuring is disabled here.
//
// The page and the bead talk over Bluetooth (the Nordic UART service) or a USB
// cable, with the same text protocol. The bead does not wait for a serial monitor,
// so it also runs on battery. (Over USB, close the Arduino Serial Monitor first.)
//
// ### Applying a gate
//
// - Pick a gate on the page: it is "armed" and the two ends of its rotation axis
//   light up orange.
// - Hold the bead so the orange axis is roughly horizontal and roll the bead around
//   it. The blue dot stays "up" in the room while the sphere turns underneath it.
// - After the full rotation (180 degrees for X, Y, Z and H, 90 for S and S-dagger,
//   45 for T) the gate snaps in.
// - Orange blinking means the axis is too vertical: tilt the bead first.
//   A red dot means you are rolling around the wrong axis.
//
// Tapping the bead once measures in the Z basis (free play, no gate armed).
// A double tap shows the reference axes for a moment: red is the x-z meridian
// (the path of the Y gate), blue the y-z meridian (the path of the X gate).
//
// Colours: blue is the current state, green the target (or, in free play, where the
// armed gate will take you), orange the rotation axis of the armed gate.
//
// ### Protocol
//
// One command or message per line. Commands can also be typed in the Serial Monitor.
//
// | Command | Meaning |
// |---|---|
// | `HELLO` | the bead answers `READY` |
// | `LEVEL sx sy sz` | free play, start state (Bloch vector) |
// | `LEVEL sx sy sz tx ty tz` | Gate Golf, start and target state |
// | `SET x y z` | set the state (e.g. reset: `SET 0 0 1`) |
// | `ARM X`, `Y`, `Z`, `H`, `S`, `D` or `T` | arm a gate (`D` is S-dagger) |
// | `CANCEL` | cancel the armed gate |
// | `MEASURE` | measure in the Z basis (free play) |
// | `AXES` | show the reference axes |
// | `WIN n` | victory animation with n stars |
// | `IDLE` | waiting mode |
// | `BRIGHT n` | brightness 0 to 255 |
//
// | Message | Meaning |
// |---|---|
// | `READY ...` | ready |
// | `S x y z p g o v` | status, about 15 times per second: state, progress 0 to 1, armed gate (`-` for none), wrong axis 0/1, axis too vertical 0/1 |
// | `DONE G x y z` | gate G finished, new state |
// | `MEAS b x y z` | measured bit b (0 or 1), new state |
// | `TAP n` | a tap that did not measure (n = 1 or 2) |
//
// ## Libraries and types
//
// First we include the Qbead library.
#include <Qbead.h>

// Custom types:
// All custom types must be defined BEFORE the first function, because the
// Arduino IDE inserts automatic function prototypes at that point.
struct V3 { float x, y, z; };
struct LineBuf { char buf[100]; uint8_t len; };
struct GateDef { char name; V3 n; float ang; };

Qbead::Qbead bead;
BLEUart bleuart;   // Nordic UART service: text over Bluetooth

// ## Settings
//
// Tuning constants: how forgiving the gate detection is, and how often the bead reports its state.
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

// ## Colours
//
// The colours used on the bead.
uint32_t BLUE   = color(0,   90, 255);
uint32_t GREEN  = color(0,  255,  40);
uint32_t ORANGE = color(255, 100,  0);
uint32_t RED    = color(255,   0,  0);
uint32_t WHITE  = color(255, 255, 255);
uint32_t AX_RED  = color(120,   0,  0);   // reference axes, dimmer so dots stay visible
uint32_t AX_BLUE = color(0,    30, 140);

// ## Small vector maths
//
// Bloch vectors are 3D vectors, so a few helpers for adding, scaling and rotating them.
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

// ## Gates
//
// Every gate is a rotation of the Bloch sphere: an axis and an angle in degrees.
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

// ## State
//
// Everything the bead remembers between loop iterations.
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

// ## Output to USB and Bluetooth
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

// ## Display
//
// Drawing states and axes on the LEDs. `render()` is called on every loop iteration and redraws the whole bead.
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

  if (drawAnim(now)) {                      // an animation is playing: it owns the LEDs
    bead.show();
    return;
  }

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

// One frame of a wave running across the sphere from point c: a ring at distance r degrees.
void drawWave(V3 c, int stars, int r) {
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
}

void fillAll(uint32_t (*colorOf)(int leg, int k), int k) {
  for (int leg = 0; leg < bead.nlegs; leg++)
    for (int pix = 0; pix <= bead.nsections; pix++)
      bead.setLegPixelColor(leg, pix, colorOf(leg, k));
}
uint32_t rainbowColor(int leg, int k) { return colorWheel_deg(fmodf(leg * 30.0f + k * 90.0f, 360.0f)); }
uint32_t greyColor(int, int)          { return color(40, 40, 40); }

// ## Animations
// Short light shows (victory, gate done, measurement) are not played with delay(): that would
// stall the motion sensor, taps and Bluetooth. Instead an animation is started here and
// render() draws the right frame for the time that has passed, on every loop iteration.
enum AnimKind { A_NONE, A_WAVE, A_CELEBRATE, A_GATE_DONE, A_MEASURE };   // A_WAVE: wave only

const unsigned long WAVE_STEP_MS  = 40;    // one ring of the victory wave
const int           WAVE_STEPS    = 20;    // rings at 0, 12, ..., 228 degrees
const unsigned long RAINBOW_ON_MS = 180;   // rainbow flash for hitting par ...
const unsigned long RAINBOW_MS    = 300;   // ... followed by darkness, twice
const unsigned long DONE_MS       = 120;   // white flash when a gate snaps in
const unsigned long MEAS_GREY_MS  = 120;   // measurement: grey flash ...
const unsigned long MEAS_MS       = 470;   // ... then the result pole in white

struct Anim { AnimKind kind; unsigned long start; V3 c; int stars; };
Anim anim = {A_NONE, 0, {0, 0, 1}, 0};

void startAnim(AnimKind kind, V3 c, int stars = 0) { anim = {kind, millis(), c, stars}; }

// Draws the current animation frame (on a cleared bead) and returns true while one is playing.
bool drawAnim(unsigned long now) {
  unsigned long t = now - anim.start;
  switch (anim.kind) {
    case A_WAVE:
    case A_CELEBRATE: {
      unsigned long waveMs = WAVE_STEP_MS * WAVE_STEPS;
      if (t < waveMs) { drawWave(anim.c, anim.stars, (t / WAVE_STEP_MS) * 12); return true; }
      t -= waveMs;
      if (anim.kind == A_CELEBRATE && anim.stars >= 3 && t < 2 * RAINBOW_MS) {
        if (t % RAINBOW_MS < RAINBOW_ON_MS) fillAll(rainbowColor, t / RAINBOW_MS);
        return true;
      }
      break;
    }
    case A_GATE_DONE:
      if (t < DONE_MS) { drawV(anim.c, WHITE); return true; }
      break;
    case A_MEASURE:
      if (t < MEAS_GREY_MS) { fillAll(greyColor, 0); return true; }
      if (t < MEAS_MS)      { drawV(anim.c, WHITE); return true; }
      break;
    default:
      return false;
  }
  anim.kind = A_NONE;
  return false;
}

void celebrate(int stars) { startAnim(A_CELEBRATE, hasTarget ? target : state, stars); }

// ## Gate logic
//
// Arming a gate, and following the bead's motion until the gate snaps in.
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
  startAnim(A_GATE_DONE, state);
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

// ## Measurement
//
// A measurement in the Z basis collapses the state to |0> or |1>, with the probabilities the state predicts.
// Z-basis measurement: |0> with probability (1 + z) / 2, then the state collapses.
bool canMeasure() { return mode == M_PLAY && !hasTarget && armed < 0; }

void measure() {
  if (!canMeasure()) return;
  float p0 = (1.0f + state.z) / 2.0f;
  randomSeed(micros());
  int bit = (random(0, 10000) / 10000.0f < p0) ? 0 : 1;
  state = bit ? mk(0, 0, -1) : mk(0, 0, 1);

  // short animation: flash, then the result pole lights up
  startAnim(A_MEASURE, state);
  out(String("MEAS ") + bit + " " + vecStr(state, 4));
}

// ## Taps
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

// ## Communication
//
// Commands arrive line by line, from USB or Bluetooth, and are handled the same way.
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

// ## Bluetooth setup
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

// ## Setup and event loop
//
// `setup()` runs once at power-on. `loop()` then runs over and over: read the motion sensor, handle commands and taps, follow an armed gate, redraw the bead and report the state.
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
  startAnim(A_WAVE, mk(0, 0, 1), 3);        // short start-up wave
  bead.wasTapped();                         // discard taps from handling during start-up
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
