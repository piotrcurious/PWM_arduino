
#include "shared_defs.h"

// Pins
#define PWM_PIN 9
#define VOLTAGE_PIN A0
#define CURRENT_PIN A1

// Targets (Fixed point, scaled by 100 for 2 decimal places)
#define TARGET_V_CENTI 1200 // 12.00V
#define CURRENT_LIMIT_CENTI 500 // 5.00A

// Control Gains (Fixed point, scaled by 256 for bit shift)
// These define the "stiffness" of the Lyapunov reaching law
#define GAIN_V 256 // 1.0 (Voltage error to current target mapping)
#define GAIN_I 128 // 0.5 (Current error to duty cycle adjustment)

// Global State
// Duty cycle scaled by 1000 for precision (0-100000 where 100000 is 100%)
int32_t duty_milli = 50000;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  analogReference(DEFAULT);
}

void loop() {
  // ── 1. Dynamic setpoint tracking ──────────────────────────────────────────
  int32_t dyn_v = TARGET_V_CENTI;
  if (fmod(millis() / 1000.0, 0.2) < 0.1) {
    dyn_v = (TARGET_V_CENTI * 8) / 10;
  }

  // ── 2. Reference Command Generation (Soft-Start) ──────────────────────────
  static int32_t v_ref = 0;
  if (v_ref < dyn_v) v_ref += 5;
  if (v_ref > dyn_v) v_ref -= 5;

  // ── 3. Read and scale feedback (Pure Integer Logic) ───────────────────────
  // Using constants from shared_defs.h implicitly for scaling
  uint32_t v_raw = (uint32_t)analogRead(VOLTAGE_PIN);
  uint32_t i_raw = (uint32_t)analogRead(CURRENT_PIN);

  // v_centi = (v_raw * 5 * divider_ratio * 100) / 1024
  // For divider_ratio = 4.0: v_centi = (v_raw * 2000) / 1024
  int32_t v_out = (int32_t)((v_raw * 2000) >> 10);

  // i_centi = (i_raw * 5 * 100) / (1024 * R_SHUNT)
  // For R_SHUNT = 0.1: i_centi = (i_raw * 5000) / 1024
  int32_t i_l = (int32_t)((i_raw * 5000) >> 10);

  // ── 4. Energy-Based Control Law (Cascaded Structure) ──────────────────────
  // We want to maintain a specific energy level in the converter.
  // First, map voltage error to an inductor current reference.
  int32_t e_v = v_ref - v_out;
  int32_t i_ref = (e_v * GAIN_V) >> 8;

  // Safety Clamps
  if (i_ref > CURRENT_LIMIT_CENTI) i_ref = CURRENT_LIMIT_CENTI;
  if (i_ref < 0) i_ref = 0;

  // Current error drives the duty cycle update
  int32_t e_i = i_ref - i_l;

  // ── 5. Lyapunov-Inspired Reaching Law ─────────────────────────────────────
  // Delta_Duty = Gamma * Error
  // This update law ensures the system state moves towards the sliding surface
  // defined by the current reference, which in turn regulates the voltage.
  int32_t adjustment = (e_i * GAIN_I);

  duty_milli += adjustment;

  // ── 6. Constraints & Safety ───────────────────────────────────────────────
  if (duty_milli > 90000) duty_milli = 90000; // Max 90% duty cycle
  if (duty_milli < 0)     duty_milli = 0;

  // ── 7. Output PWM ─────────────────────────────────────────────────────────
  // Convert duty_milli (0-100000) to standard Arduino PWM (0-255)
  int pwm_val = (int)((duty_milli * 255L) / 100000L);
  analogWrite(PWM_PIN, pwm_val);

  delay(1);
}
