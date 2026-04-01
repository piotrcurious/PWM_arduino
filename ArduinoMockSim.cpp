
#include "Arduino.h"
#include "simulator.h"
#include <math.h>

BoostSimulator simulator;

// Now we need to implement the actual logic for the mock functions using the simulator
void digitalWrite(uint8_t pin, uint8_t val) {
    if (pin == 2 || pin == 9) simulator.key_pin_val = (val == HIGH);
    if (pin == 3 || pin == 10) simulator.sr_pin_val = (val == HIGH);
}

int analogRead(uint8_t pin) {
    return simulator.get_adc(pin);
}

void analogWrite(uint8_t pin, int val) {
    if (pin == 9) simulator.pwm_pin_val = val;
}

// Override the loop/setup if we're doing it in a runner
// For now, let's keep ArduinoMock.cpp for the base and have specific runners
