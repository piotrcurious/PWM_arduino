
#include "shared_defs.h"

// Pins
#define PWM_PIN 9
#define VOLTAGE_PIN A0
#define CURRENT_PIN A1

// Targets (Fixed point, scaled by 100 for 2 decimal places)
#define TARGET_V_CENTI 1200 // 12.00V
#define TARGET_I_CENTI 100  // 1.00A

// Control Gains (Integer)
#define K_V 1
#define K_I 2

// System Parameters
#define VIN_CENTI 500 // 5.00V

// Global State
int32_t duty_scaled = 12800; // Duty cycle * 256 (0-65535)

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  analogReference(DEFAULT);
}

void loop() {
  // Read feedback
  // Using pure integer logic for performance on 8-bit AVR
  // Assuming 5V reference and 10-bit ADC
  // scale_centi = (5.0 / 1024.0) * 100 = 488 / 100 approx

  uint32_t v_raw = (uint32_t)analogRead(VOLTAGE_PIN);
  uint32_t i_raw = (uint32_t)analogRead(CURRENT_PIN);

  // v_centi = (v_raw * 5 * divider_ratio * 100) / 1024
  // For divider_ratio = 4 (30k/10k): v_centi = (v_raw * 2000) / 1024
  int32_t v_out = (int32_t)((v_raw * 2000) >> 10);

  // i_centi = (i_raw * 5 * 100) / (1024 * R_SHUNT)
  // For R_SHUNT = 0.1: i_centi = (i_raw * 5000) / 1024
  int32_t i_l = (int32_t)((i_raw * 5000) >> 10);

  // Errors
  int32_t e_v = TARGET_V_CENTI - v_out;
  int32_t e_i = TARGET_I_CENTI - i_l;

  /**
   * Lyapunov-inspired control Law:
   * We want to minimize energy error.
   * A simplified law for D:
   * Delta_D = K1 * e_v + K2 * e_i
   * We use integer logic to update duty cycle.
   */

  int32_t adjustment = (K_V * e_v) + (K_I * e_i);

  duty_scaled += adjustment;

  // Constraints
  if (duty_scaled > 23040) duty_scaled = 23040; // Max 90% (256 * 0.9 * 100? No, 255 * 0.9 = 230ish)
  // Let's use 0-25500 for duty_scaled where 25500 is 100%
  if (duty_scaled > 23000) duty_scaled = 23000;
  if (duty_scaled < 0) duty_scaled = 0;

  analogWrite(PWM_PIN, duty_scaled / 100);

  // Simulation delay to match loop frequency
  delay(1);
}
