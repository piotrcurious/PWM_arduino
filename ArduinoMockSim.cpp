
#include "Arduino.h"
#include "simulator.h"
#include "shared_defs.h"
#include <math.h>

BoostSimulator simulator;

// Now we need to implement the actual logic for the mock functions using the simulator
void digitalWrite(uint8_t pin, uint8_t val) {
    if (pin == PIN_KEY_SWITCH || pin == PIN_MAIN_SWITCH) simulator.key_pin_val = (val == HIGH);
    if (pin == PIN_SR_KEY || pin == PIN_SR_SWITCH) simulator.sr_pin_val = (val == HIGH);
}

int analogRead(uint8_t pin) {
    return simulator.get_adc(pin);
}

void analogWrite(uint8_t pin, int val) {
    if (pin == PIN_MAIN_SWITCH) simulator.pwm_pin_val = val;
}

// Override the loop/setup if we're doing it in a runner
// For now, let's keep ArduinoMock.cpp for the base and have specific runners
