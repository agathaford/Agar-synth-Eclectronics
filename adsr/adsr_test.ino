// --------------------------------------------------
// RP2040 8-bit R-2R wavetable synth + ADSR envelope
//
// ADSR comes from mo-thunderz's "adsr" library
// (https://github.com/mo-thunderz/adsr). Install it by copying
// adsr.h and adsr.cpp into a folder named "adsr" inside your
// Arduino libraries folder.
//
// Wiring:
//   GPIO 0-7 -> R-2R ladder (as before)
//   GPIO 8   -> momentary button to GND (gate input, optional)
// --------------------------------------------------
#include <math.h>
#include <adsr.h>

// ---------- Waveform ----------
const int WAVE_SAMPLES = 128;   // Number of points per wave cycle
uint8_t waveTable[WAVE_SAMPLES];

enum WaveType { SINE, SQUARE, TRIANGLE, SAWTOOTH };
WaveType currentWave = SQUARE;  // Change this to change the wave shape

// ---------- ADSR settings ----------
#define DACSIZE 256                          // 8-bit output -> envelope runs 0..255

unsigned long adsr_attack  = 50000;          // attack time in microseconds
unsigned long adsr_decay   = 100000;         // decay time in microseconds
int           adsr_sustain = 150;            // sustain level, 0..DACSIZE-1
unsigned long adsr_release = 300000;         // release time in microseconds

// ---------- Gate / trigger ----------
#define USE_BUTTON                           // comment out for auto-retriggering demo mode
const int GATE_PIN = 8;                      // button to GND, uses internal pull-up
const unsigned long DEBOUNCE_US = 10000;     // 10 ms

// Demo mode (used when USE_BUTTON is not defined)
unsigned long trigger_duration       = 300000;  // gate held for 300 ms
unsigned long space_between_triggers = 300000;  // then 300 ms of silence

// Update the envelope every 8 samples (envelope changes slowly vs. the audio)
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

  adsr_class.setAttack(adsr_attack);
  adsr_class.setDecay(adsr_decay);
  adsr_class.setSustain(adsr_sustain);
  adsr_class.setRelease(adsr_release);

  generateTables();
}

void loop() {
  for (int i = 0; i < WAVE_SAMPLES; i++) {

    // Poll the gate and refresh the envelope every few samples
    if ((i & ENV_UPDATE_MASK) == 0) {
      unsigned long t = micros();
      updateGate(t);
      envelope = adsr_class.getWave(t);   // 0..255
    }

    // Scale the wave around mid-scale (128) so silence sits at the middle
    // of the ladder instead of dragging DC around as the envelope moves.
    int centered = (int)waveTable[i] - 128;           // -128..127
    uint8_t out = (uint8_t)(128 + ((centered * envelope) >> 8));

    // RP2040 direct register write to GPIO 0-7
    sio_hw->gpio_out = (sio_hw->gpio_out & ~0xFF) | out;

    delayMicroseconds(5); // Adjust this to change the frequency/pitch of the tone
  }
}

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
