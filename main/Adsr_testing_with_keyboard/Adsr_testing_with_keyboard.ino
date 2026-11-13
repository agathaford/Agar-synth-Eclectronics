// --------------------------------------------------
// RP2040 8-bit R-2R wavetable synth + ADSR envelope
// with keypad notes and rotary-encoder control of the ADSR values
//
// ADSR comes from mo-thunderz's "adsr" library
// (https://github.com/mo-thunderz/adsr). Install it by copying
// adsr.h and adsr.cpp into a folder named "adsr" inside your
// Arduino libraries folder.
//
// Wiring (encoder module powered from 3.3V, NOT 5V):
//   GPIO 0-7   -> R-2R ladder
//   GPIO 8-11  -> keypad rows
//   GPIO 12-15 -> keypad columns
//   GPIO 16    -> encoder CLK (A)
//   GPIO 17    -> encoder DT  (B)
//   GPIO 18    -> encoder SW  (push button)
//
// Use:
//   Press a note key    -> sets the pitch AND starts the envelope (attack/decay/sustain)
//   Release the key     -> starts the release stage (once ALL note keys are up)
//   Click the encoder   -> cycle Attack -> Decay -> Sustain -> Release
//   Turn the encoder    -> change the selected value
//   Serial Monitor (9600): every encoder CLICK and every TURN prints a labeled
//   line with all four ADSR values; the one being edited is in [brackets].
// --------------------------------------------------
#include <math.h>
#include <adsr.h>
#include <Keypad.h>

// Must stay above the first function: the Arduino IDE auto-generates
// function prototypes there, and appendParam() uses this type.
enum Param { P_ATTACK, P_DECAY, P_SUSTAIN, P_RELEASE, NUM_PARAMS };

// Noise-tolerant input filter used for the three encoder pins.
struct Debounced {
  uint8_t stable;        // last accepted (debounced) level
  unsigned long acc;     // time (us) the pin has disagreed with 'stable' (net of agreement)
  unsigned long last;    // time (us) of the previous poll
};

// ---------- Waveform ----------
const int WAVE_SAMPLES = 128;   // Number of points per wave cycle
uint8_t waveTable[WAVE_SAMPLES];
double pitch = 880;             // Hz
unsigned int sampleDelayUs = 9; // recomputed from pitch in setPitch()

enum WaveType { SINE, SQUARE, TRIANGLE, SAWTOOTH };
WaveType currentWave = SINE;  // Change this to change the wave shape

// ---------- Keypad ----------
const int numRows = 4;
const int numCols = 4;

char keys[numRows][numCols] = {
  {'C', 'D', 'E', 'F'},
  {'G', 'A', 'B', 'c'},
  {'1', '2', '3', '4'},
  {'5', '6', '7', '8'}
};

byte rowPins[] = {8, 11, 10, 9};
byte colPins[] = {12, 13, 14, 15};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, numRows, numCols);

#define KEYPAD_SCAN_EVERY 4       // scan the keypad once every N wave cycles (less audio glitching)
unsigned int cycleCount = 0;

// Pitch for each key in Hz; return 0 for keys with no note assigned.
// Keys '1'-'8' are unassigned: give them a pitch here and they will trigger the envelope too.
double pitchForKey(char k) {
  switch (k) {
    case 'C': return 523.25;
    case 'D': return 587.33;
    case 'E': return 659.3;
    case 'F': return 698.6;
    case 'G': return 784;
    case 'A': return 880;
    case 'B': return 987.8;
    case 'c': return 1046.5;
    default:  return 0;
  }
}

// ---------- ADSR settings (starting values) ----------
#define DACSIZE 256                          // 8-bit output -> envelope runs 0..255

unsigned long adsr_attack  = 50000;          // attack time in microseconds
unsigned long adsr_decay   = 100000;         // decay time in microseconds
int           adsr_sustain = 150;            // sustain level, 0..DACSIZE-1
unsigned long adsr_release = 300000;         // release time in microseconds

// Limits and step sizes for the encoder
const unsigned long MIN_TIME_US = 1000;      // 1 ms
const unsigned long MAX_TIME_US = 5000000;   // 5 s
const int SUSTAIN_STEP = 5;                  // sustain change per detent
// Times change by about 12.5% per detent (fine when short, fast when long)

// ---------- Rotary encoder ----------
const int ENC_CLK_PIN = 16;
const int ENC_DT_PIN  = 17;
const int ENC_SW_PIN  = 18;
// Quadrature edges per detent = (pulses x 4) / detents.
// Alps EC11E parts in the catalog (9/18, 15/30, 18/36 pulses/detents) are all 2.
// A part with as many pulses as detents (e.g. 18/18) would be 4.
#define ENC_COUNTS_PER_DETENT 2
#define ENC_REVERSE 0             // set to 1 if turning clockwise lowers the value
#define SERIAL_FEEDBACK 1         // 0 = no Serial output (avoids any tiny print glitch)
#define ENC_DEBUG 0               // 1 = also print raw pin changes and decoded steps, with timestamps (us)
#define ENC_EARLY_FIRE 0          // 1 = act on the FIRST edge of a detent (snappier, less noise-proof)
const unsigned long ENC_DEBOUNCE_US    = 1000;    // A/B must disagree with the old level for this long (1 ms); 0 = no filter
const unsigned long ENC_SW_DEBOUNCE_US = 20000;   // same for the push switch (20 ms)

const char* paramNames[NUM_PARAMS] = { "Attack", "Decay", "Sustain", "Release" };
Param selected = P_ATTACK;

// Quadrature decode table: index = (previous 2 bits << 2) | current 2 bits
const int8_t ENC_TABLE[16] = { 0, -1, 1, 0,  1, 0, 0, -1,  -1, 0, 0, 1,  0, 1, -1, 0 };
uint8_t encState = 0;
int8_t  encAccum = 0;
int8_t  encLock = 0;          // early-fire only: direction already acted on in this detent
bool    encSwLast = true;
Debounced dbA, dbB, dbSw;

// Update the envelope / encoder every 8 samples
#define ENV_UPDATE_MASK 0x07

// ---------- Internal state ----------
adsr adsr_class(DACSIZE);
int envelope = 0;

void setup() {
  // Initialize GPIO 0 through 7 as Outputs
  for (int pin = 0; pin < 8; pin++) {
    pinMode(pin, OUTPUT);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  uint8_t a0 = digitalRead(ENC_CLK_PIN);
  uint8_t b0 = digitalRead(ENC_DT_PIN);
  uint8_t s0 = digitalRead(ENC_SW_PIN);
  dbA  = { a0, 0, 0 };
  dbB  = { b0, 0, 0 };
  dbSw = { s0, 0, 0 };
  encState = (((a0 << 1) | b0) << 2) | ((a0 << 1) | b0);   // start with "no movement"
  encSwLast = s0;

#if SERIAL_FEEDBACK
  Serial.begin(9600);
#endif

  // Keypad press/release events fire from inside keypad.getKey()
  keypad.addEventListener(keypadEvent);

  adsr_class.setAttack(adsr_attack);
  adsr_class.setDecay(adsr_decay);
  adsr_class.setSustain(adsr_sustain);
  adsr_class.setRelease(adsr_release);

  setPitch(pitch);
  generateTables();
}

void loop() {
#if SERIAL_FEEDBACK
  // Print the starting values once, as soon as the Serial Monitor is open
  static bool announced = false;
  if (!announced && Serial) {
    announced = true;
    reportValue("READY");
  }
#endif

  // Keypad: scan every few cycles; keypadEvent() handles press/release
  if ((cycleCount++ % KEYPAD_SCAN_EVERY) == 0) {
    keypad.getKey();
  }

  for (int i = 0; i < WAVE_SAMPLES; i++) {

    // Poll the encoder and refresh the envelope every few samples
    if ((i & ENV_UPDATE_MASK) == 0) {
      unsigned long t = micros();
      updateEncoder(t);
      envelope = adsr_class.getWave(t);   // 0..255
    }

    // Scale the wave around mid-scale (128) so silence sits at the middle
    // of the ladder instead of dragging DC around as the envelope moves.
    int centered = (int)waveTable[i] - 128;           // -128..127
    uint8_t out = (uint8_t)(128 + ((centered * envelope) >> 8));

    // RP2040 direct register write to GPIO 0-7
    sio_hw->gpio_out = (sio_hw->gpio_out & ~0xFF) | out;

    delayMicroseconds(sampleDelayUs);   // sets the pitch
  }
}

// ---------- Pitch ----------
void setPitch(double hz) {
  pitch = hz;
  // Computed once per key press instead of with floating point on every sample
  double us = 1000000.0 / (WAVE_SAMPLES * hz);
  sampleDelayUs = (us < 1.0) ? 1 : (unsigned int)(us + 0.5);
}

// ---------- Gate (driven by the keypad) ----------
void gateOn(unsigned long t) {
  adsr_class.noteOn(t);
  digitalWrite(LED_BUILTIN, HIGH);
}

void gateOff(unsigned long t) {
  adsr_class.noteOff(t);
  digitalWrite(LED_BUILTIN, LOW);
}

// Called by the Keypad library whenever a key changes state.
// The adsr library counts presses: every noteOn must be matched by a noteOff,
// and the release stage only starts once all held keys are up.
void keypadEvent(KeypadEvent key) {
  unsigned long t = micros();
  switch (keypad.getState()) {
    case PRESSED: {
      double hz = pitchForKey(key);
      if (hz > 0) {                       // only keys that have a note
        setPitch(hz);                     // newest key sets the pitch
        gateOn(t);                        // (re)start attack from the current level
#if SERIAL_FEEDBACK
        if (Serial) Serial.println(key);
#endif
      }
      break;
    }
    case RELEASED:
      if (pitchForKey(key) > 0) {         // same keys that triggered noteOn
        gateOff(t);                       // release starts when the last key comes up
      }
      break;
    default:
      break;
  }
}

// ---------- Encoder ----------
// Integrating debounce: the time the pin spends disagreeing with the accepted level is
// added up, and time spent agreeing is subtracted. The level flips only once the net
// disagreement reaches holdUs. Contact chatter averages out, and a stray noise glitch
// can't flip the level or stall it the way a "must stay perfectly quiet" timer would.
uint8_t debounce(Debounced &d, uint8_t reading, unsigned long t, unsigned long holdUs) {
  unsigned long dt = t - d.last;
  d.last = t;
  unsigned long cap = holdUs / 8 + 1;       // one poll after a long gap can't count for much
  if (dt > cap) dt = cap;

  if (reading != d.stable) {
    d.acc += dt;
    if (d.acc >= holdUs) {                  // disagreed long enough: accept the new level
      d.stable = reading;
      d.acc = 0;
    }
  } else {
    d.acc = (d.acc > dt) ? d.acc - dt : 0;  // agreeing drains the evidence
  }
  return d.stable;
}

void updateEncoder(unsigned long t) {
#if SERIAL_FEEDBACK && ENC_DEBUG
  // Show every change on the raw pins, before filtering (a flood while the knob is still means noise)
  static uint8_t lastRaw = 0xFF;
  uint8_t rawNow = (digitalRead(ENC_CLK_PIN) << 1) | digitalRead(ENC_DT_PIN);
  if (rawNow != lastRaw) {
    lastRaw = rawNow;
    if (Serial) {
      Serial.print("raw CLK/DT ");
      Serial.print((int)(rawNow >> 1));
      Serial.print("/");
      Serial.print((int)(rawNow & 1));
      Serial.print(" @");
      Serial.println(t);
    }
  }
#endif

  // Knob: debounce A and B separately, then decode only when the debounced state changes
  uint8_t a = debounce(dbA, digitalRead(ENC_CLK_PIN), t, ENC_DEBOUNCE_US);
  uint8_t b = debounce(dbB, digitalRead(ENC_DT_PIN),  t, ENC_DEBOUNCE_US);
  uint8_t cur  = (a << 1) | b;
  uint8_t prev = encState & 0x03;

  if (cur != prev) {
    encState = ((encState << 2) | cur) & 0x0F;
    int8_t step = ENC_TABLE[encState];
#if SERIAL_FEEDBACK && ENC_DEBUG
    if (Serial) {
      Serial.print("enc step ");
      Serial.print((int)step);
      Serial.print("  accum ");
      Serial.print((int)(encAccum + step));
      Serial.print(" @");
      Serial.println(t);
    }
#endif

#if ENC_EARLY_FIRE
    // Act on the first edge out of a detent instead of waiting for the next one.
    // encLock stops a bounce back through the same detent from firing twice.
    if (step == 0) {
      encAccum = 0;               // skipped a state: sync lost
      encLock = 0;
    } else {
      if (encAccum == 0 && encLock != step) {
        encLock = step;
        adjustValue(ENC_REVERSE ? -step : step);
      }
      encAccum += step;
      if (encAccum >= ENC_COUNTS_PER_DETENT || encAccum <= -ENC_COUNTS_PER_DETENT) {
        encAccum = 0;             // arrived at the next detent
        encLock = 0;
      }
    }
#else
    // Act when the detent is complete (default)
    if (step == 0) {
      encAccum = 0;               // skipped a state: sync lost, drop the partial count
    } else {
      encAccum += step;
    }
    if (encAccum >= ENC_COUNTS_PER_DETENT) {
      encAccum = 0;
      adjustValue(ENC_REVERSE ? -1 : +1);
    } else if (encAccum <= -ENC_COUNTS_PER_DETENT) {
      encAccum = 0;
      adjustValue(ENC_REVERSE ? +1 : -1);
    }
#endif
  }

  // Button: a click selects the next parameter (filtered over 20 ms)
  bool sw = debounce(dbSw, digitalRead(ENC_SW_PIN), t, ENC_SW_DEBOUNCE_US);
  if (sw != encSwLast) {
    encSwLast = sw;
    if (sw == LOW) {                                // pressed
      selected = (Param)((selected + 1) % NUM_PARAMS);
      reportValue("CLICK");
    }
  }
}

unsigned long adjustTime(unsigned long v, int dir) {
  unsigned long step = v / 8;
  if (step < 500) step = 500;
  if (dir > 0) v += step;
  else         v = (v > step) ? v - step : 0;
  if (v < MIN_TIME_US) v = MIN_TIME_US;
  if (v > MAX_TIME_US) v = MAX_TIME_US;
  return v;
}

void adjustValue(int dir) {
  switch (selected) {
    case P_ATTACK:
      adsr_attack = adjustTime(adsr_attack, dir);
      adsr_class.setAttack(adsr_attack);
      break;
    case P_DECAY:
      adsr_decay = adjustTime(adsr_decay, dir);
      adsr_class.setDecay(adsr_decay);
      break;
    case P_SUSTAIN:
      adsr_sustain += dir * SUSTAIN_STEP;
      if (adsr_sustain < 0) adsr_sustain = 0;
      if (adsr_sustain > DACSIZE - 1) adsr_sustain = DACSIZE - 1;
      adsr_class.setSustain(adsr_sustain);
      break;
    case P_RELEASE:
      adsr_release = adjustTime(adsr_release, dir);
      adsr_class.setRelease(adsr_release);
      break;
    default:
      break;
  }
  reportValue(dir > 0 ? "TURN +" : "TURN -");
}

// Append one parameter to buf; the one being edited is wrapped in [ ]. Returns the new length.
size_t appendParam(char *buf, size_t len, size_t cap, Param p) {
  const char *open  = (p == selected) ? "[" : "";
  const char *close = (p == selected) ? "]" : "";
  int n;
  if (p == P_SUSTAIN) {
    n = snprintf(buf + len, cap - len, "%s%s %d/255%s", open, paramNames[p], adsr_sustain, close);
  } else {
    unsigned long us = (p == P_ATTACK) ? adsr_attack : (p == P_DECAY) ? adsr_decay : adsr_release;
    n = snprintf(buf + len, cap - len, "%s%s %lu.%lu ms%s", open, paramNames[p], us / 1000, (us % 1000) / 100, close);
  }
  if (n < 0) return len;
  len += (size_t)n;
  return (len < cap) ? len : cap - 1;
}

// Print a label plus all four ADSR values on one line, e.g.
//   CLICK:   Attack 50.0 ms  [Decay 100.0 ms]  Sustain 150/255  Release 300.0 ms
//   TURN +:  Attack 50.0 ms  [Decay 112.5 ms]  Sustain 150/255  Release 300.0 ms
// The whole line is built first and sent in ONE write, so the audio loop is held up
// for a single USB transfer instead of dozens of tiny ones.
void reportValue(const char* tag) {
#if SERIAL_FEEDBACK
  if (!Serial) return;                              // nobody listening
  char buf[128];
  size_t len = (size_t)snprintf(buf, sizeof(buf), "%s:  ", tag);
  for (int p = 0; p < NUM_PARAMS; p++) {
    len = appendParam(buf, len, sizeof(buf) - 2, (Param)p);
    if (p < NUM_PARAMS - 1 && len + 2 < sizeof(buf) - 2) {
      buf[len++] = ' ';
      buf[len++] = ' ';
      buf[len] = 0;
    }
  }
  buf[len++] = '\n';
  Serial.write((const uint8_t*)buf, len);
#endif
}

// ---------- Wavetable ----------
void generateTables() {
  for (int i = 0; i < WAVE_SAMPLES; i++) {
    switch (currentWave) {
      case SINE:
        // 8-bit resolution scales from 0 to 255
        waveTable[i] = (uint8_t)((sin(((float)i / WAVE_SAMPLES) * 2.0 * M_PI) + 1.0) * 127.5);
        break;

      case SQUARE:
        waveTable[i] = (i < WAVE_SAMPLES / 2) ? 255 : 0;
        break;

      case TRIANGLE:
        if (i < WAVE_SAMPLES / 2) {
          waveTable[i] = (uint8_t)((i * 2 * 255) / WAVE_SAMPLES);
        } else {
          waveTable[i] = (uint8_t)((2 * 255) - ((i * 2 * 255) / WAVE_SAMPLES));
        }
        break;

      case SAWTOOTH:
        waveTable[i] = (uint8_t)((i * 255) / WAVE_SAMPLES);
        break;
    }
  }
}
