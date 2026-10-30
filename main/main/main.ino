// --------------------------------------------------
// RP2040 8-bit R-2R wavetable synth + ADSR envelope
// with rotary-encoder control of the ADSR values
//
// ADSR comes from mo-thunderz's "adsr" library
// (https://github.com/mo-thunderz/adsr). Install it by copying
// adsr.h and adsr.cpp into a folder named "adsr" inside your
// Arduino libraries folder.
//
// Wiring (encoder module powered from 3.3V, NOT 5V):
//   GPIO 0-7  -> R-2R ladder (as before)
//   GPIO 8    -> momentary gate button to GND (optional)
//   GPIO 9    -> encoder CLK (A)
//   GPIO 10   -> encoder DT  (B)
//   GPIO 11   -> encoder SW  (push button)
//
// Use:
//   Click the encoder   -> cycle Attack -> Decay -> Sustain -> Release
//   Turn the encoder    -> change the selected value
//   Open the Serial Monitor (115200): every change prints all four ADSR
//   values, with the one being edited in [brackets].
// --------------------------------------------------
#include <math.h>
#include <adsr.h>
#include <Keypad.h>

// ---------- Waveform ----------
const int WAVE_SAMPLES = 128;   // Number of points per wave cycle
uint8_t waveTable[WAVE_SAMPLES];
double pitch = 880;

enum WaveType { SINE, SQUARE, TRIANGLE, SAWTOOTH };
WaveType currentWave = SINE;  // Change this to change the wave shape

// keypad stuff
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

// ---------- Gate / trigger ----------
#define USE_BUTTON                           // comment out for auto-retriggering demo mode
const int GATE_PIN = 8;                      // button to GND, uses internal pull-up
const unsigned long DEBOUNCE_US = 10000;     // 10 ms

// Demo mode (used when USE_BUTTON is not defined)
unsigned long trigger_duration       = 300000;  // gate held for 300 ms
unsigned long space_between_triggers = 300000;  // then 300 ms of silence

// ---------- Rotary encoder ----------
const int ENC_CLK_PIN = 16;
const int ENC_DT_PIN  = 17;
const int ENC_SW_PIN  = 18;
#define ENC_COUNTS_PER_DETENT 4   // try 2 or 1 if one click moves the value too far/little
#define ENC_REVERSE 0             // set to 1 if turning clockwise lowers the value
#define SERIAL_FEEDBACK 1         // 0 = no Serial output (avoids any tiny print glitch)
const unsigned long ENC_SW_DEBOUNCE_US = 20000;   // 20 ms

enum Param { P_ATTACK, P_DECAY, P_SUSTAIN, P_RELEASE, NUM_PARAMS };
const char* paramNames[NUM_PARAMS] = { "Attack", "Decay", "Sustain", "Release" };
Param selected = P_ATTACK;

// Quadrature decode table: index = (previous 2 bits << 2) | current 2 bits
const int8_t ENC_TABLE[16] = { 0, -1, 1, 0,  1, 0, 0, -1,  -1, 0, 0, 1,  0, 1, -1, 0 };
uint8_t encState = 0;
int8_t  encAccum = 0;
bool    encSwLast = true;
unsigned long encSwChangedAt = 0;

// Update the envelope / gate / encoder every 8 samples
#define ENV_UPDATE_MASK 0x07

// ---------- Internal state ----------
adsr adsr_class(DACSIZE);
bool gate = false;
unsigned long t_last_change = 0;
int envelope = 0;

void setup() {
  // Initialize GPIO 0 through 7 as Outputs
  for (int pin = 0; pin < 8; pin++) {
    pinMode(pin, OUTPUT);
  }

  pinMode(LED_BUILTIN, OUTPUT);
#ifdef USE_BUTTON
  pinMode(GATE_PIN, INPUT_PULLUP);
#endif

  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  uint8_t cur = (digitalRead(ENC_CLK_PIN) << 1) | digitalRead(ENC_DT_PIN);
  encState = (cur << 2) | cur;              // start with "no movement"
  encSwLast = digitalRead(ENC_SW_PIN);

#if SERIAL_FEEDBACK
  Serial.begin(9600);
#endif

  adsr_class.setAttack(adsr_attack);
  adsr_class.setDecay(adsr_decay);
  adsr_class.setSustain(adsr_sustain);
  adsr_class.setRelease(adsr_release);

  generateTables();
}

void loop() {
 // keypad
  char key = keypad.getKey();
  if (key) {
    Serial.println(key);
    switch(key) {
      case 'C':
        pitch = 523.25;
        break;
      case 'D':
        pitch = 587.33;
        break;
      case 'E':
        pitch = 659.3;
        break;
      case 'F':
        pitch = 698.6;
        break;
      case 'G':
        pitch = 784;
        break;
      case 'A':
        pitch = 880;
        break;
      case 'B':
        pitch = 987.8;
        break;
      case 'c':
        pitch = 1046.5;
        break;
    }
  }

  for (int i = 0; i < WAVE_SAMPLES; i++) {

    // Poll the gate, encoder and envelope every few samples
    if ((i & ENV_UPDATE_MASK) == 0) {
      unsigned long t = micros();
      updateGate(t);
      updateEncoder(t);
      envelope = adsr_class.getWave(t);   // 0..255
    }

    // Scale the wave around mid-scale (128) so silence sits at the middle
    // of the ladder instead of dragging DC around as the envelope moves.
    int centered = (int)waveTable[i] - 128;           // -128..127
    uint8_t out = (uint8_t)(128 + ((centered * envelope) >> 8));

    // RP2040 direct register write to GPIO 0-7
    sio_hw->gpio_out = (sio_hw->gpio_out & ~0xFF) | out;

    delayMicroseconds(static_cast<int>(round(1000000.0/(WAVE_SAMPLES*pitch)))); // Adjust this to change the frequency/pitch of the tone
  }
}

// ---------- Gate ----------
void gateOn(unsigned long t) {
  adsr_class.noteOn(t);
  gate = true;
  t_last_change = t;
  digitalWrite(LED_BUILTIN, HIGH);
}

void gateOff(unsigned long t) {
  adsr_class.noteOff(t);
  gate = false;
  t_last_change = t;
  digitalWrite(LED_BUILTIN, LOW);
}

void updateGate(unsigned long t) {
#ifdef USE_BUTTON
  if (t - t_last_change < DEBOUNCE_US) return;     // ignore contact bounce
  bool pressed = (digitalRead(GATE_PIN) == LOW);
  if (pressed && !gate)      gateOn(t);
  else if (!pressed && gate) gateOff(t);
#else
  if (gate) {
    if (t - t_last_change > trigger_duration) gateOff(t);
  } else {
    if (t - t_last_change > space_between_triggers) gateOn(t);
  }
#endif
}

// ---------- Encoder ----------
void updateEncoder(unsigned long t) {
  // Knob: decode both channels so bounce and direction are handled cleanly
  uint8_t cur = (digitalRead(ENC_CLK_PIN) << 1) | digitalRead(ENC_DT_PIN);
  encState = ((encState << 2) | cur) & 0x0F;
  encAccum += ENC_TABLE[encState];

  if (encAccum >= ENC_COUNTS_PER_DETENT) {
    encAccum = 0;
    adjustValue(ENC_REVERSE ? -1 : +1);
  } else if (encAccum <= -ENC_COUNTS_PER_DETENT) {
    encAccum = 0;
    adjustValue(ENC_REVERSE ? +1 : -1);
  }

  // Button: a click selects the next parameter
  bool sw = digitalRead(ENC_SW_PIN);
  if (sw != encSwLast && (t - encSwChangedAt) > ENC_SW_DEBOUNCE_US) {
    encSwLast = sw;
    encSwChangedAt = t;
    if (sw == LOW) {                                // pressed
      selected = (Param)((selected + 1) % NUM_PARAMS);
      reportValue();
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
  reportValue();
}

// Print one parameter; the one being edited is wrapped in [ ]
void printParam(Param p) {
#if SERIAL_FEEDBACK
  if (p == selected) Serial.print("[");
  Serial.print(paramNames[p]);
  Serial.print(" ");
  switch (p) {
    case P_ATTACK:  Serial.print(adsr_attack / 1000);  Serial.print(" ms"); break;
    case P_DECAY:   Serial.print(adsr_decay / 1000);   Serial.print(" ms"); break;
    case P_SUSTAIN: Serial.print(adsr_sustain);        Serial.print("/255"); break;
    case P_RELEASE: Serial.print(adsr_release / 1000); Serial.print(" ms"); break;
    default: break;
  }
  if (p == selected) Serial.print("]");
#endif
}

// Print all four ADSR values on one line, e.g.
//   [Attack 54 ms]  Decay 100 ms  Sustain 150/255  Release 300 ms
void reportValue() {
#if SERIAL_FEEDBACK
  if (!Serial) return;                              // nobody listening
  for (int p = 0; p < NUM_PARAMS; p++) {
    printParam((Param)p);
    if (p < NUM_PARAMS - 1) Serial.print("  ");
  }
  Serial.println("");
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