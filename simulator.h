
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
    double ESR_cap = 0.05;  // Capacitor ESR in ohms
    double I_sat = 10.0;    // Inductor saturation current (A)

    double V_out = 5.0;     // Initial output voltage (capacitor voltage)
    double I_L = 0.0;       // Inductor current
    double Temp = 25.0;     // Temperature (C)
    double R_base = 50.0;   // Base load

    double sim_time_sec = 0; // Current simulation time in seconds

    int pwm_pin_val = -1;   // 0-255 or HIGH/LOW, -1 if using registers
    bool key_pin_val = false;
    bool sr_pin_val = false;

    void step(double dt_step) {
        sim_time_sec += dt_step;

        if (dt_step > 1e-4) dt_step = 1e-4; // Stability cap

        // Chaotic/Aggressive Load Pattern
        double cycle = fmod(sim_time_sec, 0.2);
        if (cycle < 0.04) R_load = R_base;
        else if (cycle < 0.08) R_load = R_base / 2.0;
        else if (cycle < 0.12) R_load = R_base / 10.0; // Very heavy load
        else if (cycle < 0.16) R_load = R_base * 2.0;  // Light load
        else R_load = R_base / 4.0;

        // Saturation model: Inductance decreases when current exceeds I_sat
        double L_eff = L;
        if (I_L > I_sat) {
            L_eff = L * (I_sat / I_L);
        }

        bool switch_on = false;
        bool sr_on = false;

        // Determine switch states from registers, analogWrite, or digital pins
        if (pwm_pin_val >= 0) {
            static double pwm_accum = 0;
            pwm_accum += dt_step * 10000; // ~10kHz default
            if (fmod(pwm_accum, 255.0) < pwm_pin_val) switch_on = true;
        } else {
            // Hardware PWM using OCR1A/B and ICR1
            if (ICR1 > 0) {
                double period_s = (double)ICR1 / 16e6;
                double time_in_period = fmod(sim_time_sec, period_s);
                if (time_in_period < (double)OCR1A / 16e6) switch_on = true;
                if (time_in_period < (double)OCR1B / 16e6) sr_on = true;
            } else {
                if (key_pin_val) switch_on = true;
                if (sr_pin_val) sr_on = true;
            }
        }

        double R_parasitic = 0.1;
        double I_load = V_out / R_load;

        // Cross-conduction protection in simulator
        if (switch_on && sr_on) {
            I_L += ((V_in - I_L * R_parasitic * 10.0) / L_eff) * dt_step;
            Temp += 1.0 * dt_step;
            V_out += (-I_load / C) * dt_step;
            return; // Exit step early for short circuit
        }

        if (switch_on && !sr_on) {
            // Standard Boost phase: Switch ON
            I_L += ((V_in - I_L * R_parasitic) / L_eff) * dt_step;
            V_out += (-I_load / C) * dt_step;
        } else if (!switch_on && sr_on) {
            // SR phase: SR Switch ON (conduction without diode drop)
            I_L += ((V_in - V_out - I_L * R_parasitic) / L_eff) * dt_step;
            V_out += ((I_L - I_load) / C) * dt_step;
        } else if (!switch_on && !sr_on) {
            // Off phase: Diode conduction
            if (I_L > 0) {
                I_L += ((V_in - V_out - V_diode - I_L * R_parasitic) / L_eff) * dt_step;
                V_out += ((I_L - I_load) / C) * dt_step;
            } else {
                I_L = 0;
                if (V_out < V_in - V_diode) {
                    double charging_i = (V_in - V_diode - V_out) / R_parasitic;
                    V_out += ((charging_i - I_load) / C) * dt_step;
                } else {
                    V_out += (-I_load / C) * dt_step;
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

        // Measured voltage includes ESR drop: V_meas = V_cap + I_cap * ESR
        // During step, I_cap = I_L - I_load (if switch off) or -I_load (if switch on)
        // For simplicity, let's just use V_out (capacitor voltage) and a small transient factor
        double V_meas = V_out;

        if (pin == A0) val = V_meas * (1 + noise) * (1.0 / VOLTAGE_DIVIDER_RATIO) * (1024.0 / ref);
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
