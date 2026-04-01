
#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "Arduino.h"

class BoostSimulator {
public:
    double V_in = 5.0;      // Input voltage
    double L = 100e-6;      // Inductance 100uH
    double C = 100e-6;      // Capacitance 100uF
    double R_load = 100.0;   // Load resistance
    double R_shunt = 0.1;   // Shunt resistance

    double V_out = 5.0;     // Initial output voltage
    double I_L = 0.0;       // Inductor current
    double Temp = 25.0;     // Temperature (C)

    double sim_time_sec = 0; // Current simulation time in seconds

    int pwm_pin_val = 0;    // 0-255 or HIGH/LOW
    bool key_pin_val = false;
    bool sr_pin_val = false;

    void step(double dt_step) {
        sim_time_sec += dt_step;

        bool switch_on = false;
        if (pwm_pin_val > 0) {
            static double pwm_accum = 0;
            pwm_accum += dt_step * 100000; // arbitrary freq for simulation
            if (fmod(pwm_accum, 255.0) < pwm_pin_val) switch_on = true;
        } else if (key_pin_val) {
            switch_on = true;
        }

        // Realistic model should consider R_load and parasitic resistances to avoid infinite current
        double R_parasitic = 0.5;
        if (switch_on) {
            I_L += ((V_in - I_L * R_parasitic) / L) * dt_step;
            V_out += (-V_out / (C * R_load)) * dt_step;
        } else {
            if (I_L > 0) {
                I_L += ((V_in - V_out - I_L * R_parasitic) / L) * dt_step;
                V_out += ((I_L - V_out / R_load) / C) * dt_step;
            } else {
                I_L = 0;
                V_out += (-V_out / (C * R_load)) * dt_step;
            }
        }

        if (I_L < 0) I_L = 0; // Ideal diode behavior
        if (V_out < 0) V_out = 0;

        // Temperature model: heating proportional to I_L^2
        Temp += (I_L * I_L * 0.0001 - (Temp - 25.0) * 0.001) * dt_step;
    }

    uint16_t get_adc(uint8_t pin) {
        double val = 0;
        if (pin == A0) val = V_out * (1024.0 / 5.0) * (10000.0 / (10000.0 + 10000.0)); // A0 is V_out through divider
        else if (pin == A1) val = I_L * R_shunt * (1024.0 / 5.0); // A1 is current
        else if (pin == A2) {
            double R = 10000.0 * exp(3950.0 * (1.0 / (Temp + 273.15) - 1.0 / 298.15));
            val = 5.0 * R / (R + 10000.0) * (1024.0 / 5.0);
        }
        else if (pin == A3) val = V_out * (1024.0 / 5.0);
        else if (pin == A4) val = V_in * (1024.0 / 5.0);

        if (val > 1023) val = 1023;
        if (val < 0) val = 0;
        return (uint16_t)val;
    }
};

extern BoostSimulator simulator;

#endif
