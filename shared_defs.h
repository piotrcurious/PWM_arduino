
#ifndef SHARED_DEFS_H
#define SHARED_DEFS_H

// Common constants for simulation and compilation
#ifndef R_SHUNT
#define R_SHUNT 0.1 // Shunt resistor in ohms
#endif

#ifndef R1
#define R1 30000.0   // Resistor divider R1 (30k)
#endif

#ifndef R2
#define R2 10000.0   // Resistor divider R2 (10k)
#endif

#ifndef Rs
#define Rs 0.1      // Current sense resistor (same as R_SHUNT often)
#endif

#define VOLTAGE_DIVIDER_RATIO ((R1 + R2) / R2)
#define HARDWARE_ADC_REF 5.0

// Pin Mapping for Simulator
#define PIN_MAIN_SWITCH 9
#define PIN_SR_SWITCH 10
#define PIN_KEY_SWITCH 2
#define PIN_SR_KEY 3

#ifdef __cplusplus
extern "C" {
#endif
extern uint8_t _analog_reference_mode;
#ifdef __cplusplus
}
#endif

#endif
