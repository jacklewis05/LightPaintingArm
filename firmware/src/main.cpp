// Light-painting arm firmware: a segment player for three steppers and an RGB LED.
//
// The PC plans the whole motion and streams it as short timed segments ("be at these
// step counts in 20 ms, with the LED this colour"). A fixed-rate timer interrupt spreads
// each segment's steps evenly over its duration on all three motors at once, so the
// joints stay synchronised. Protocol (115200 baud, one command per line):
//
//   ?                         -> "info ..." firmware identification
//   H        / H <joint>      home all joints in HOME_ORDER / one joint (0, 1 or 2)
//   M                         the joints are on their home marks: remember the encoder angles
//   K                         home at the marks: joints are near their marks, the encoders
//                             work out exactly how far off each one is
//   C                         resync: take the encoders' word for where the joints are
//   J <joint> <steps>         slow relative jog; works before homing
//   Z <a> <b> <c>             declare the current position to be these step counts (marks homed)
//   N                         start a new job: reset the segment counter
//   S <seq> <a> <b> <c> <ms> <r> <g> <b>
//                             queue a segment: absolute step targets, duration, LED 0-255
//   L <r> <g> <b>             set the LED now (while idle)
//   E <0|1>                   motors off / on (while off, joints with encoders stay tracked)
//   X                         stop now: drop the queue, LED off
//
// Every STATUS_MS the firmware sends:
//   st <a> <b> <c> <done> <queued> <homed mask> <I|R|H> <r> <g> <b>
//      <encoder mask> <e0> <e1> <e2> <motors on>
// where e is (position measured by the encoder) - (position by counting steps), in
// microsteps: it stays near zero until a motor misses steps.
// Problems are reported as "err <reason>"; anything else is a human-readable log line.

#include <Arduino.h>
#include <Preferences.h>
#include "soc/gpio_struct.h"
#include "esp_rom_sys.h"
#include "config.h"

// ------------------------------------------------------------------ shared state

struct Segment {
  int32_t target[3];
  uint32_t ticks;
  uint8_t rgb[3];
};

static Segment queue[QUEUE_SIZE];
static volatile uint32_t qHead = 0;        // written by loop()
static volatile uint32_t qTail = 0;        // written by the interrupt
static volatile bool flushRequest = false;

static volatile int32_t position[3] = {0, 0, 0};   // steps actually sent to each driver
static volatile uint32_t doneCount = 0;
static volatile uint8_t ledNow[3] = {0, 0, 0};
static volatile bool ledDirty = true;

static int32_t planned[3] = {0, 0, 0};     // position at the end of the last queued segment
static uint32_t nextSeq = 0;
static uint8_t homedMask = 0;
static bool homing = false;
static bool motorsOn = true;

// Encoders: unwrapped count on each motor shaft, and the count that corresponds to step 0.
constexpr int32_t ENC_CPR = 16384;         // MT6701 is 14 bit
constexpr uint8_t MT6701_ADDR = 0x06;
static bool encPresent[3] = {false, false, false};
static int encFails[3] = {0, 0, 0};
static int32_t encRaw[3] = {0, 0, 0};      // angle within one motor turn
static int32_t encTurns[3] = {0, 0, 0};
static int32_t encRef[3] = {0, 0, 0};
static int32_t markRaw[3] = {0, 0, 0};     // encRaw with each joint on its home mark (saved in flash)
static bool marksSaved = false;
static Preferences prefs;

// A constant-speed move on one joint, used for homing.
struct Jog {
  volatile bool active;
  int axis;
  int dir;                 // +1 or -1 in step counts
  uint32_t interval;       // ticks between steps
  uint32_t countdown;
  uint32_t stepsLeft;
  int stopWhenHall;        // 1: stop when the sensor is active, 0: when inactive, -1: never
  volatile bool hit;
};
static Jog jog = {false, 0, 1, 1, 1, 0, -1, false};

static hw_timer_t *stepTimer = nullptr;
static uint32_t stepMask[3], dirMask[3];

// ------------------------------------------------------------------ step interrupt

static inline bool IRAM_ATTR hallActive(int axis) {
  const int pin = PIN_HALL[axis];
  const bool level = pin < 32 ? (GPIO.in >> pin) & 1 : (GPIO.in1.val >> (pin - 32)) & 1;
  return level != HALL_ACTIVE_LOW;
}

static inline void IRAM_ATTR setDir(int axis, bool positive) {
  if (positive) GPIO.out_w1ts = dirMask[axis];
  else GPIO.out_w1tc = dirMask[axis];
}

static inline void IRAM_ATTR pulse(uint32_t mask) {
  GPIO.out_w1ts = mask;
  esp_rom_delay_us(2);       // A4988 needs a step pulse of at least 1 us
  GPIO.out_w1tc = mask;
}

static void IRAM_ATTR onTick() {
  static bool active = false;
  static uint32_t ticks = 0, tick = 0;
  static uint32_t steps[3], acc[3];
  static int32_t dir[3];

  if (jog.active) {
    if ((jog.stopWhenHall >= 0 && hallActive(jog.axis) == (jog.stopWhenHall == 1))) {
      jog.hit = true;
      jog.active = false;
    } else if (jog.stepsLeft == 0) {
      jog.active = false;
    } else if (--jog.countdown == 0) {
      jog.countdown = jog.interval;
      jog.stepsLeft--;
      pulse(stepMask[jog.axis]);
      position[jog.axis] = position[jog.axis] + jog.dir;
    }
    return;
  }

  if (flushRequest) {
    active = false;
    qTail = qHead;
    for (int i = 0; i < 3; i++) ledNow[i] = 0;
    ledDirty = true;
    flushRequest = false;
    return;
  }

  if (!active) {
    if (qTail == qHead) return;
    // Load the next segment. Stepping starts on the following tick, which gives the
    // drivers their direction set-up time.
    const Segment &s = queue[qTail % QUEUE_SIZE];
    for (int j = 0; j < 3; j++) {
      const int32_t delta = s.target[j] - position[j];
      dir[j] = delta >= 0 ? 1 : -1;
      steps[j] = delta >= 0 ? delta : -delta;
      acc[j] = s.ticks / 2;
      setDir(j, delta >= 0);
      ledNow[j] = s.rgb[j];
    }
    ledDirty = true;
    ticks = s.ticks;
    tick = 0;
    active = true;
    return;
  }

  uint32_t mask = 0;
  for (int j = 0; j < 3; j++) {
    acc[j] += steps[j];
    if (acc[j] >= ticks) {
      acc[j] -= ticks;
      mask |= stepMask[j];
      position[j] = position[j] + dir[j];
    }
  }
  if (mask) pulse(mask);
  if (++tick >= ticks) {
    active = false;
    qTail = qTail + 1;
    doneCount = doneCount + 1;
  }
}

// ------------------------------------------------------------------ LED

static void ledSetup() {
  for (int i = 0; i < 3; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_LED[i], LED_PWM_HZ, 8);
#else
    ledcSetup(i, LED_PWM_HZ, 8);
    ledcAttachPin(PIN_LED[i], i);
#endif
  }
}

static void ledApply() {
  if (!ledDirty) return;
  ledDirty = false;
  for (int i = 0; i < 3; i++) {
    const uint32_t duty = LED_COMMON_ANODE ? 255 - ledNow[i] : ledNow[i];
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PIN_LED[i], duty);
#else
    ledcWrite(i, duty);
#endif
  }
}

// ------------------------------------------------------------------ encoders

// Minimal bit-banged I2C master on open-drain pins, so every encoder can have its own
// data line. A chip whose data line stays high never sees a start condition, which is
// why sharing the clock between them is safe.
static inline void i2cWait() { delayMicroseconds(3); }

static void i2cStart(int sda, int scl) {
  digitalWrite(sda, HIGH);
  digitalWrite(scl, HIGH);
  i2cWait();
  digitalWrite(sda, LOW);
  i2cWait();
  digitalWrite(scl, LOW);
  i2cWait();
}

static void i2cStop(int sda, int scl) {
  digitalWrite(sda, LOW);
  i2cWait();
  digitalWrite(scl, HIGH);
  i2cWait();
  digitalWrite(sda, HIGH);
  i2cWait();
}

static bool i2cWrite(int sda, int scl, uint8_t value) {
  for (int bit = 7; bit >= 0; bit--) {
    digitalWrite(sda, (value >> bit) & 1);
    i2cWait();
    digitalWrite(scl, HIGH);
    i2cWait();
    digitalWrite(scl, LOW);
  }
  digitalWrite(sda, HIGH);          // release the line for the acknowledge bit
  i2cWait();
  digitalWrite(scl, HIGH);
  i2cWait();
  const bool acked = digitalRead(sda) == LOW;
  digitalWrite(scl, LOW);
  i2cWait();
  return acked;
}

static uint8_t i2cRead(int sda, int scl, bool ack) {
  uint8_t value = 0;
  digitalWrite(sda, HIGH);
  for (int bit = 0; bit < 8; bit++) {
    i2cWait();
    digitalWrite(scl, HIGH);
    i2cWait();
    value = (value << 1) | (digitalRead(sda) ? 1 : 0);
    digitalWrite(scl, LOW);
  }
  digitalWrite(sda, ack ? LOW : HIGH);
  i2cWait();
  digitalWrite(scl, HIGH);
  i2cWait();
  digitalWrite(scl, LOW);
  digitalWrite(sda, HIGH);
  return value;
}

// Angle register 0x03 holds bits 13..6, register 0x04 holds bits 5..0 in its top six bits.
static bool encReadRaw(int j, int32_t &raw) {
  const int sda = PIN_ENC_SDA[j], scl = PIN_ENC_SCL[j];
  i2cStart(sda, scl);
  bool ok = i2cWrite(sda, scl, MT6701_ADDR << 1) && i2cWrite(sda, scl, 0x03);
  if (ok) {
    i2cStart(sda, scl);
    ok = i2cWrite(sda, scl, (MT6701_ADDR << 1) | 1);
  }
  if (ok) {
    const uint8_t hi = i2cRead(sda, scl, true);
    const uint8_t lo = i2cRead(sda, scl, false);
    raw = ((int32_t)hi << 6) | (lo >> 2);
  }
  i2cStop(sda, scl);
  return ok;
}

static int32_t countsToSteps(int32_t counts) {
  return (int32_t)lround((double)counts * STEPS_PER_MOTOR_REV / ENC_CPR);
}
static int32_t stepsToCounts(int32_t steps) {
  return (int32_t)lround((double)steps * ENC_CPR / STEPS_PER_MOTOR_REV);
}
static int32_t encCounts(int j) { return encTurns[j] * ENC_CPR + encRaw[j]; }

// Where the encoder says the joint is, in the same step units as position[].
static int32_t encSteps(int j) { return ENC_DIR[j] * countsToSteps(encCounts(j) - encRef[j]); }

// Call whenever position[j] has just been set from a trusted source (homing): from now on
// the encoder and the step count agree, until steps are missed.
static void encAlign(int j) {
  encRef[j] = encCounts(j) - ENC_DIR[j] * stepsToCounts(position[j]);
}

static void encSetup() {
  for (int j = 0; j < 3; j++) {
    if (PIN_ENC_SDA[j] < 0) continue;
    pinMode(PIN_ENC_SDA[j], OUTPUT_OPEN_DRAIN | PULLUP);
    pinMode(PIN_ENC_SCL[j], OUTPUT_OPEN_DRAIN | PULLUP);
    digitalWrite(PIN_ENC_SDA[j], HIGH);
    digitalWrite(PIN_ENC_SCL[j], HIGH);
  }
  delay(20);
  for (int j = 0; j < 3; j++) {
    if (PIN_ENC_SDA[j] < 0) continue;
    int32_t raw;
    for (int attempt = 0; attempt < 3 && !encPresent[j]; attempt++) encPresent[j] = encReadRaw(j, raw);
    if (encPresent[j]) encRaw[j] = raw;
    Serial.printf("encoder %d %s\n", j, encPresent[j] ? "ok" : "NOT FOUND (check wiring and pins)");
  }
}

// Read every encoder and follow it across turn boundaries. This has to run often enough
// that a motor never makes half a turn between two reads, so the slow loops call it too.
static void encUpdate() {
  static uint32_t last = 0;
  if (micros() - last < 3000) return;
  last = micros();
  for (int j = 0; j < 3; j++) {
    if (!encPresent[j]) continue;
    int32_t raw;
    if (!encReadRaw(j, raw)) {
      if (++encFails[j] == 50) {
        encPresent[j] = false;
        Serial.printf("err encoder %d stopped answering\n", j);
      }
      continue;
    }
    encFails[j] = 0;
    const int32_t change = raw - encRaw[j];
    if (change > ENC_CPR / 2) encTurns[j]--;
    if (change < -ENC_CPR / 2) encTurns[j]++;
    encRaw[j] = raw;
    // With the motors released a joint can be turned by hand; follow it.
    if (!motorsOn) position[j] = planned[j] = encSteps(j);
  }
}

// ------------------------------------------------------------------ helpers

static void motorsEnable(bool on) {
  if (PIN_EN >= 0) digitalWrite(PIN_EN, (on == EN_ACTIVE_LOW) ? LOW : HIGH);
  motorsOn = on;
  if (!on) {
    // A released joint without an encoder may be moved without us knowing.
    for (int j = 0; j < 3; j++) {
      if (!encPresent[j]) homedMask &= ~(1 << j);
    }
  }
}

static bool queueEmpty() { return qTail == qHead; }
static uint32_t queued() { return qHead - qTail; }

static void sendStatus() {
  const char state = homing ? 'H' : (queueEmpty() ? 'I' : 'R');
  long slip[3];
  int encMask = 0;
  for (int j = 0; j < 3; j++) {
    slip[j] = encPresent[j] ? (long)(encSteps(j) - position[j]) : 0;
    if (encPresent[j]) encMask |= 1 << j;
  }
  Serial.printf("st %ld %ld %ld %lu %lu %u %c %u %u %u %d %ld %ld %ld %d\n",
                (long)position[0], (long)position[1], (long)position[2],
                (unsigned long)doneCount, (unsigned long)queued(), homedMask, state,
                ledNow[0], ledNow[1], ledNow[2], encMask, slip[0], slip[1], slip[2],
                motorsOn ? 1 : 0);
}

static void stopNow() {
  jog.active = false;
  if (!queueEmpty()) {
    flushRequest = true;
    while (flushRequest) delayMicroseconds(50);
  } else {
    for (int i = 0; i < 3; i++) ledNow[i] = 0;
    ledDirty = true;
  }
  for (int j = 0; j < 3; j++) planned[j] = position[j];
}

// ------------------------------------------------------------------ homing

static bool abortRequested() {
  // During homing the only command honoured is X.
  while (Serial.available()) {
    if (Serial.read() == 'X') return true;
  }
  return false;
}

// Move one joint at a constant rate until the hall sensor reaches the wanted state
// (stopWhenHall 1/0) or maxSteps have been taken. Returns true if the sensor stopped it.
static bool jogUntil(int axis, int dirLevel, uint32_t stepsPerSec, int stopWhenHall,
                     uint32_t maxSteps, bool &aborted) {
  setDir(axis, dirLevel == 1);
  delayMicroseconds(20);
  jog.axis = axis;
  jog.dir = dirLevel == 1 ? 1 : -1;
  jog.interval = TICK_HZ / stepsPerSec > 0 ? TICK_HZ / stepsPerSec : 1;
  jog.countdown = jog.interval;
  jog.stepsLeft = maxSteps;
  jog.stopWhenHall = stopWhenHall;
  jog.hit = false;
  jog.active = true;
  uint32_t lastStatus = millis();
  while (jog.active) {
    if (abortRequested()) {
      jog.active = false;
      aborted = true;
      return false;
    }
    if (millis() - lastStatus >= STATUS_MS) {
      lastStatus = millis();
      sendStatus();
    }
    encUpdate();
    delay(1);
  }
  return jog.hit;
}

// Seek the sensor, back off until it releases, then creep back onto it, so the trigger
// point is always approached slowly from the same side.
static bool homeAxis(int axis) {
  const int toward = HOME_DIR[axis];
  const int away = toward == 1 ? 0 : 1;
  const uint32_t seekMax = (uint32_t)(HOME_SEEK_LIMIT_DEG * STEPS_PER_OUTPUT_DEG);
  const uint32_t fineMax = 200 * 16 * 2;      // two motor revolutions
  bool aborted = false;

  homedMask &= ~(1 << axis);
  if (hallActive(axis)) {
    if (!jogUntil(axis, away, HOME_SEEK_SPS, 0, fineMax * 4, aborted)) {
      Serial.printf("err home %d: %s\n", axis, aborted ? "stopped" : "cannot leave the sensor");
      return false;
    }
    jogUntil(axis, away, HOME_SEEK_SPS, -1, 800, aborted);     // a little extra margin
  }
  if (!aborted && !jogUntil(axis, toward, HOME_SEEK_SPS, 1, seekMax, aborted)) {
    Serial.printf("err home %d: %s\n", axis,
                  aborted ? "stopped" : "no hall trigger (check sensor, magnet, HOME_DIR)");
    return false;
  }
  if (!aborted && !jogUntil(axis, away, HOME_FINE_SPS, 0, fineMax, aborted)) {
    Serial.printf("err home %d: %s\n", axis, aborted ? "stopped" : "back-off failed");
    return false;
  }
  if (!aborted && !jogUntil(axis, toward, HOME_FINE_SPS, 1, fineMax, aborted)) {
    Serial.printf("err home %d: %s\n", axis, aborted ? "stopped" : "fine approach failed");
    return false;
  }
  if (aborted) {
    Serial.printf("err home %d: stopped\n", axis);
    return false;
  }
  position[axis] = 0;
  planned[axis] = 0;
  encAlign(axis);
  homedMask |= 1 << axis;
  Serial.printf("homed joint %d\n", axis);
  return true;
}

static void home(int only) {
  if (!queueEmpty()) {
    Serial.println("err busy");
    return;
  }
  motorsEnable(true);
  homing = true;
  for (int i = 0; i < 3; i++) {
    const int axis = HOME_ORDER[i];
    if (only >= 0 && axis != only) continue;
    if (!homeAxis(axis)) break;
  }
  homing = false;
  for (int j = 0; j < 3; j++) planned[j] = position[j];
}

// ---- Homing at marks. Each joint has a mark scribed at its home position. Lining a
// mark up by eye is only good to a degree or so, but the encoder knows the motor's exact
// angle within its current turn, and one motor turn is only 360/29 = 12.4 degrees of joint
// travel. So as long as the mark is within about 6 degrees, comparing the encoder angle
// with the one saved at the mark gives the joint's exact offset from home.

static void saveMarks() {
  for (int j = 0; j < 3; j++) {
    position[j] = planned[j] = 0;
    if (encPresent[j]) markRaw[j] = encRaw[j];
    encAlign(j);
  }
  prefs.putBytes("marks", markRaw, sizeof(markRaw));
  marksSaved = true;
  homedMask = 7;
  Serial.println("marks saved: this pose is home");
}

static void homeAtMarks() {
  long offset[3] = {0, 0, 0};
  for (int j = 0; j < 3; j++) {
    if (encPresent[j] && !marksSaved) {
      Serial.println("err no saved marks: put every joint exactly on its mark and save them first");
      return;
    }
  }
  for (int j = 0; j < 3; j++) {
    int32_t steps = 0;
    if (encPresent[j]) {
      int32_t away = encRaw[j] - markRaw[j];
      if (away > ENC_CPR / 2) away -= ENC_CPR;
      if (away < -ENC_CPR / 2) away += ENC_CPR;
      steps = ENC_DIR[j] * countsToSteps(away);
    }
    position[j] = planned[j] = steps;
    encAlign(j);
    offset[j] = steps;
  }
  homedMask = 7;
  Serial.printf("homed at marks; joints were off by %ld %ld %ld steps\n", offset[0], offset[1], offset[2]);
}

static void resync() {
  for (int j = 0; j < 3; j++) {
    if (encPresent[j]) position[j] = planned[j] = encSteps(j);
  }
  Serial.println("resynced to encoders");
}

static void jogRelative(const char *args) {
  int axis;
  long steps;
  if (sscanf(args, "%d %ld", &axis, &steps) != 2 || axis < 0 || axis > 2 || steps == 0) {
    Serial.println("err bad jog");
    return;
  }
  if (!motorsOn) {
    Serial.println("err motors off");
    return;
  }
  bool aborted = false;
  homing = true;          // reported as busy while it moves
  jogUntil(axis, steps > 0 ? 1 : 0, HOME_SEEK_SPS, -1, (uint32_t)labs(steps), aborted);
  homing = false;
  planned[axis] = position[axis];
}

// ------------------------------------------------------------------ commands

static void queueSegment(const char *args) {
  unsigned long seq, ms;
  long a, b, c;
  unsigned int r, g, bl;
  if (sscanf(args, "%lu %ld %ld %ld %lu %u %u %u", &seq, &a, &b, &c, &ms, &r, &g, &bl) != 8) {
    Serial.println("err bad segment");
    return;
  }
  if (homedMask != 7) {
    Serial.println("err not homed");
    return;
  }
  if (!motorsOn) {
    Serial.println("err motors off");
    return;
  }
  if (seq != nextSeq) {
    Serial.printf("err sequence: expected %lu, got %lu\n", (unsigned long)nextSeq, seq);
    return;
  }
  if (queued() >= (uint32_t)QUEUE_SIZE) {
    Serial.println("err queue full");
    return;
  }
  if (ms == 0 || ms > 60000) {
    Serial.println("err bad duration");
    return;
  }
  Segment s;
  s.target[0] = a;
  s.target[1] = b;
  s.target[2] = c;
  s.ticks = (uint32_t)((uint64_t)ms * TICK_HZ / 1000);
  s.rgb[0] = min(r, 255u);
  s.rgb[1] = min(g, 255u);
  s.rgb[2] = min(bl, 255u);
  for (int j = 0; j < 3; j++) {
    const int32_t delta = s.target[j] - planned[j];
    if ((uint32_t)abs(delta) > s.ticks) {
      Serial.printf("err too fast: joint %d needs %ld steps in %lu ms\n", j, (long)abs(delta), ms);
      return;
    }
  }
  for (int j = 0; j < 3; j++) planned[j] = s.target[j];
  queue[qHead % QUEUE_SIZE] = s;
  qHead = qHead + 1;       // publish to the interrupt only after the slot is filled
  nextSeq++;
}

static void handleLine(char *line) {
  const char cmd = line[0];
  const char *args = line + 1;
  switch (cmd) {
    case 'S':
      queueSegment(args);
      break;
    case 'N':
      if (queueEmpty()) {
        nextSeq = 0;
        doneCount = 0;
      } else {
        Serial.println("err busy");
      }
      break;
    case 'X':
      stopNow();
      Serial.println("stopped");
      break;
    case 'H': {
      int axis = -1;
      sscanf(args, "%d", &axis);
      if (axis > 2) axis = -1;
      home(axis);
      break;
    }
    case 'Z': {
      long a, b, c;
      if (!queueEmpty()) {
        Serial.println("err busy");
      } else if (sscanf(args, "%ld %ld %ld", &a, &b, &c) == 3) {
        position[0] = planned[0] = a;
        position[1] = planned[1] = b;
        position[2] = planned[2] = c;
        for (int j = 0; j < 3; j++) encAlign(j);
        homedMask = 7;
        Serial.println("position set");
      } else {
        Serial.println("err bad position");
      }
      break;
    }
    case 'L': {
      unsigned int r, g, b;
      if (queueEmpty() && sscanf(args, "%u %u %u", &r, &g, &b) == 3) {
        ledNow[0] = min(r, 255u);
        ledNow[1] = min(g, 255u);
        ledNow[2] = min(b, 255u);
        ledDirty = true;
      }
      break;
    }
    case 'E':
      if (queueEmpty()) motorsEnable(atoi(args) != 0);
      else Serial.println("err busy");
      break;
    case 'M':
    case 'K':
    case 'C':
    case 'J':
      if (!queueEmpty()) Serial.println("err busy");
      else if (cmd == 'M') saveMarks();
      else if (cmd == 'K') homeAtMarks();
      else if (cmd == 'C') resync();
      else jogRelative(args);
      break;
    case '?':
      Serial.printf("info lightarm 1 tick_hz=%lu queue=%d\n", (unsigned long)TICK_HZ, QUEUE_SIZE);
      break;
    default:
      break;
  }
}

// ------------------------------------------------------------------ setup / loop

void setup() {
  Serial.setRxBufferSize(4096);
  Serial.begin(BAUD);

  for (int j = 0; j < 3; j++) {
    pinMode(PIN_STEP[j], OUTPUT);
    pinMode(PIN_DIR[j], OUTPUT);
    digitalWrite(PIN_STEP[j], LOW);
    pinMode(PIN_HALL[j], INPUT_PULLUP);
    stepMask[j] = 1UL << PIN_STEP[j];
    dirMask[j] = 1UL << PIN_DIR[j];
  }
  if (PIN_EN >= 0) pinMode(PIN_EN, OUTPUT);
  motorsEnable(true);
  ledSetup();
  encSetup();
  prefs.begin("lightarm", false);
  marksSaved = prefs.getBytes("marks", markRaw, sizeof(markRaw)) == sizeof(markRaw);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  stepTimer = timerBegin(1000000);
  timerAttachInterrupt(stepTimer, &onTick);
  timerAlarm(stepTimer, 1000000 / TICK_HZ, true, 0);
#else
  stepTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(stepTimer, &onTick, true);
  timerAlarmWrite(stepTimer, 1000000 / TICK_HZ, true);
  timerAlarmEnable(stepTimer);
#endif

  Serial.println("light-painting arm ready; home with H");
}

void loop() {
  static char line[96];
  static size_t len = 0;
  static uint32_t lastStatus = 0;

  while (Serial.available()) {
    const char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len > 0) {
        line[len] = 0;
        len = 0;
        handleLine(line);
      }
    } else if (len < sizeof(line) - 1) {
      line[len++] = ch;
    }
  }

  ledApply();
  encUpdate();

  if (millis() - lastStatus >= STATUS_MS) {
    lastStatus = millis();
    sendStatus();
  }
}
