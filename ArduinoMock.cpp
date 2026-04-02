
#include "Arduino.h"
#include "simulator.h"
#include <time.h>

uint8_t TCCR1A = 0;
uint8_t TCCR1B = 0;
uint16_t ICR1 = 0;
uint16_t OCR1A = 0;
uint16_t OCR1B = 0;
uint8_t _analog_reference_mode = DEFAULT;

#ifdef __cplusplus
MockSerial Serial;
#endif

void pinMode(uint8_t pin, uint8_t mode) {}

unsigned long millis(void) {
    return (unsigned long)(simulator.sim_time_sec * 1000.0);
}

unsigned long micros(void) {
    return (unsigned long)(simulator.sim_time_sec * 1000000.0);
}

void delay(unsigned long ms) {
    double end_time = simulator.sim_time_sec + (ms / 1000.0);
    while (simulator.sim_time_sec < end_time) {
        simulator.step(1e-6); // Small steps for physics simulation
    }
}

void delayMicroseconds(unsigned int us) {
    double end_time = simulator.sim_time_sec + (us / 1000000.0);
    while (simulator.sim_time_sec < end_time) {
        simulator.step(1e-6); // Small steps for physics simulation
    }
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrain(float amt, float low, float high) {
    return (amt < low) ? low : ((amt > high) ? high : amt);
}

long arduino_random_1(long howbig) { return rand() % (howbig ? howbig : 1); }
long arduino_random_2(long howsmall, long howbig) { return howsmall + rand() % ((howbig - howsmall) ? (howbig - howsmall) : 1); }

void analogWriteFrequency(uint8_t pin, uint32_t frequency) {}

void wdt_enable(uint8_t timeout) {}
void wdt_reset() {}

float __attribute__((weak)) inductance = 0;

void __attribute__((weak)) digitalWrite(uint8_t pin, uint8_t val) {}
int __attribute__((weak)) digitalRead(uint8_t pin) { return 0; }
int __attribute__((weak)) analogRead(uint8_t pin) { return 0; }
void __attribute__((weak)) analogReference(uint8_t mode) {
    _analog_reference_mode = mode;
}
void __attribute__((weak)) analogWrite(uint8_t pin, int val) {}
