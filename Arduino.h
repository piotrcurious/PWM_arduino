
#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define DEFAULT 1
#define INTERNAL 2
#define INTERNAL1V1 2
#define INTERNAL2V56 3
#define EXTERNAL 0

#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19

typedef uint8_t byte;

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogReference(uint8_t mode);
extern uint8_t _analog_reference_mode;
void analogWrite(uint8_t pin, int val);

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

// Helpers
long map(long x, long in_min, long in_max, long out_min, long out_max);
float constrain(float amt, float low, float high);
long arduino_random_1(long howbig);
long arduino_random_2(long howsmall, long howbig);
#define random(...) arduino_random_OVERLOAD(__VA_ARGS__, arduino_random_2, arduino_random_1)(__VA_ARGS__)
#define arduino_random_OVERLOAD(_1, _2, NAME, ...) NAME

// Placeholder for analogWriteFrequency
void analogWriteFrequency(uint8_t pin, uint32_t frequency);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// Mocking Serial
class MockSerial {
public:
    void begin(unsigned long baud) {}
    void print(const char* s) { printf("%s", s); }
    void print(float f) { printf("%f", f); }
    void print(double f) { printf("%f", f); }
    void print(int i) { printf("%d", i); }
    void println(const char* s) { printf("%s\n", s); }
    void println(float f) { printf("%f\n", f); }
    void println(double f) { printf("%f\n", f); }
    void println(int i) { printf("%d\n", i); }
};
extern MockSerial Serial;
#endif

#endif
