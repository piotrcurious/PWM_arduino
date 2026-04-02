
#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "Arduino.h"
#include "shared_defs.h"
#include <avr/io.h>

class BoostSimulator {
public:
    double V_in = 5.0;      // Input voltage
    double L = 10e-3;      // Inductance 10mH (adjusted for 1kHz control)
    double C = 10e-3;      // Capacitance 10mF
    double R_load = 50.0;   // Load resistance
    double R_shunt = 0.1;   // Shunt resistance
    double V_diode = 0.7;   // Diode forward voltage

    double V_out = 5.0;     // Initial output voltage
    double I_L = 0.0;       // Inductor current
    double Temp = 25.0;     // Temperature (C)
    double R_base = 50.0;   // Base load

    double sim_time_sec = 0; // Current simulation time in seconds

    int pwm_pin_val = -1;   // 0-255 or HIGH/LOW, -1 if using registers
    bool key_pin_val = false;
    bool sr_pin_val = false;

    void step(double dt_step) {
        sim_time_sec += dt_step;

        // Update dynamic load: Square wave between R_base and R_base/5
        if (fmod(sim_time_sec, 0.1) < 0.05) {
            R_load = R_base;
        } else {
            R_load = R_base / 5.0; // Heavy load
        }

        bool switch_on = false;

        // Determine switch state from registers or analogWrite
        if (pwm_pin_val >= 0) {
            static double pwm_accum = 0;
            pwm_accum += dt_step * 10000; // ~10kHz default
            if (fmod(pwm_accum, 255.0) < pwm_pin_val) switch_on = true;
        } else {
            // Hardware PWM using OCR1A and ICR1
            if (ICR1 > 0) {
                double period_s = (double)ICR1 / 16e6; // Assuming 16MHz clock
                double time_in_period = fmod(sim_time_sec, period_s);
                if (time_in_period < (double)OCR1A / 16e6) switch_on = true;
            } else if (key_pin_val) {
                switch_on = true;
            }
        }

        double R_parasitic = 0.1;
        if (switch_on) {
            // Inductor charges from V_in
            I_L += ((V_in - I_L * R_parasitic) / L) * dt_step;
            // Capacitor discharges into R_load
            V_out += (-V_out / (C * R_load)) * dt_step;
        } else {
            // Inductor discharges into Capacitor through Diode
            if (I_L > 0) {
                I_L += ((V_in - V_out - V_diode - I_L * R_parasitic) / L) * dt_step;
                V_out += ((I_L - V_out / R_load) / C) * dt_step;
            } else {
                I_L = 0;
                // If V_out < V_in - V_diode, the capacitor is charged through the diode
                if (V_out < V_in - V_diode) {
                    double charging_i = (V_in - V_diode - V_out) / R_parasitic;
                    V_out += (charging_i / C) * dt_step;
                } else {
                    V_out += (-V_out / (C * R_load)) * dt_step;
                }
            }
        }

        if (I_L < 0) I_L = 0;
        if (V_out < 0) V_out = 0;

        // Temperature model: heating proportional to I_L^2
        Temp += (I_L * I_L * 0.0001 - (Temp - 25.0) * 0.001) * dt_step;

        // Dynamic inductance simulation: L shifts slightly over time or temperature
        L = 10e-3 * (1.0 + 0.001 * (Temp - 25.0));
    }

    uint16_t get_adc(uint8_t pin) {
        double val = 0;
        // Simple noise model
        double noise = (double)(rand() % 100 - 50) / 50.0 * 0.001;

        double ref = 5.0;
        if (_analog_reference_mode == INTERNAL) ref = 1.1;

        if (pin == A0) val = V_out * (1 + noise) * (1.0 / VOLTAGE_DIVIDER_RATIO) * (1024.0 / ref);
        else if (pin == A1) val = I_L * R_shunt * (1024.0 / ref);
        else if (pin == A2) {
            double R = 10000.0 * exp(3950.0 * (1.0 / (Temp + 273.15) - 1.0 / 298.15));
            val = 5.0 * R / (R + 10000.0) * (1024.0 / ref);
        }
        else if (pin == A3) val = V_out * (1.0 / VOLTAGE_DIVIDER_RATIO) * (1024.0 / ref);
        else if (pin == A4) val = V_in * (1024.0 / ref);

        if (val > 1023) val = 1023;
        if (val < 0) val = 0;
        return (uint16_t)val;
    }
};

extern BoostSimulator simulator;

#endif
