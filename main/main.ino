#include <math.h>

const int WAVE_SAMPLES = 128;   // Number of points per wave cycle
uint8_t waveTable[WAVE_SAMPLES];

enum WaveType { SINE, SQUARE, TRIANGLE, SAWTOOTH };
WaveType currentWave = SQUARE;    // Change this to change the wave shape

void setup() {
  // Initialize GPIO 0 through 7 as Outputs
  for (int pin = 0; pin < 8; pin++) {
    pinMode(pin, OUTPUT);
  }
  
  generateTables();
}

void loop() {
  // Rapidly dump the waveform data directly to the DIY resistor ladder
  for (int i = 0; i < WAVE_SAMPLES; i++) {
    
    // RP2040 direct register write:
    // This instantly sets GPIO 0 to 7 matching the 8 bits of waveTable[i]
    sio_hw->gpio_out = (sio_hw->gpio_out & ~0xFF) | waveTable[i];
    
    delayMicroseconds(5); // Adjust this to change the frequency/pitch of the tone
  }
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