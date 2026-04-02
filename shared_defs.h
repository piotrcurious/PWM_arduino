
#ifndef SHARED_DEFS_H
#define SHARED_DEFS_H

// Common constants for simulation and compilation
#ifndef R_SHUNT
#define R_SHUNT 0.1 // Shunt resistor in ohms
#endif

#ifndef R1
#define R1 10000.0   // Resistor divider R1
#endif

#ifndef R2
#define R2 10000.0   // Resistor divider R2
#endif

#ifndef Rs
#define Rs 0.1      // Current sense resistor (same as R_SHUNT often)
#endif

#define VOLTAGE_DIVIDER_RATIO ((R1 + R2) / R2)

#endif
