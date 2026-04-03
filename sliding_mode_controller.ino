
#include "shared_defs.h"

// Pins
#define PWM_PIN 9
#define VOLTAGE_PIN A0
#define CURRENT_PIN A1
#define INPUT_V_PIN A4

// Targets
#define TARGET_V 12.0
#define CURRENT_LIMIT_SMC 5.0

// Sliding Surface Coefficients (Gains)
#define ALPHA 50.0f  // Surface bandwidth (outer loop convergence rate)
#define KV 0.6f      // Voltage-to-current mapping factor [A/V]

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  analogReference(DEFAULT);
}

void loop() {
  float ref = (_analog_reference_mode == INTERNAL) ? 1.1 : HARDWARE_ADC_REF;
  float scale = ref / 1024.0;

  float v_out = (float)analogRead(VOLTAGE_PIN) * scale * VOLTAGE_DIVIDER_RATIO;
  float i_l = (float)analogRead(CURRENT_PIN) * scale / R_SHUNT;
  float v_in = (float)analogRead(INPUT_V_PIN) * scale;

  // ── 1. Dynamic Target Generation (Square wave: 12V <-> 9.6V) ──────────────
  float dyn_v = TARGET_V;
  if (fmod(millis() / 1000.0, 0.2) < 0.1) {
    dyn_v = TARGET_V * 0.8;
  }

  // ── 2. Reference Command Generation (Soft-Start) ──────────────────────────
  static float v_ref = 0;
  if (v_ref < dyn_v) v_ref += 0.05;
  if (v_ref > dyn_v) v_ref -= 0.05;

  // ── 3. Outer (Voltage) Sliding Surface: Sv = ev + ALPHA * Integral(ev) ─────
  // This cascaded structure maps voltage error to an inductor current target.
  float e_v = v_ref - v_out;
  static float integral_ev = 0.0f;
  static float i_l_target_prev = 0.0f;
  const float dt = 0.001f;

  // Anti-windup: conditional integration based on previous target saturation
  bool saturated = (i_l_target_prev >= CURRENT_LIMIT_SMC) || (i_l_target_prev <= 0.0f);
  if (!saturated) {
    integral_ev += e_v * dt;
    // Hard clamp: Bound = CURRENT_LIMIT / (ALPHA * KV)
    // This ensures a sustained error cannot wind the integrator beyond useful limits.
    const float INT_MAX = CURRENT_LIMIT_SMC / (ALPHA * KV);
    if (integral_ev >  INT_MAX) integral_ev =  INT_MAX;
    if (integral_ev < -INT_MAX) integral_ev = -INT_MAX;
  }

  float S_v = e_v + ALPHA * integral_ev;
  float i_l_target = S_v * KV;

  // Enforce safe operating range for current target
  if (i_l_target > CURRENT_LIMIT_SMC) i_l_target = CURRENT_LIMIT_SMC;
  if (i_l_target < 0.0f)              i_l_target = 0.0f;
  i_l_target_prev = i_l_target;

  // ── 4. Inner (Current) Sliding Surface: Si = i_l_target - i_l ─────────────
  float S_i = i_l_target - i_l;

  // ── 5. Advanced Control Law: Equivalent Control + Variable Reaching Law ──
  // Deq handles steady-state boost dynamics including estimated parasitic losses
  const float R_p_est = 0.12f; // Estimated parasitic resistance (Ohms)
  float u_eq = 1.0f - (v_in - i_l * R_p_est) / (v_out + 0.1f);
  u_eq = constrain(u_eq, 0.0f, 0.9f);

  // Variable Reaching Law: Dot{S} = -q*S - eps*sat(S/phi)
  // u_reach acts as an integrating term to reject disturbances and model mismatch
  static float u_reach = 0.0f;

  // Model-based gain scaling: The control authority in a boost converter is
  // proportional to Vout/L. We scale our reaching gains by 1/Vout to normalize.
  float reach_scale = 10.0f / (v_out + 1.0f);

  const float q_gain = 0.25f * reach_scale;   // Exponential convergence gain
  const float eps_gain = 0.01f * reach_scale; // Constant reaching gain
  const float phi = 0.05f;                    // Boundary layer width (A)

  // Saturation function for boundary layer to reduce chattering
  float sat_Si = (S_i > phi) ? 1.0f : ((S_i < -phi) ? -1.0f : S_i / phi);
  u_reach += (q_gain * S_i + eps_gain * sat_Si);

  // Authority cap on the reaching/integral term
  u_reach = constrain(u_reach, -0.3f, 0.3f);

  // ── 6. Final Control Action (PWM Output) ──────────────────────────────────
  float u_total = u_eq + u_reach;
  int pwm_val = (int)(constrain(u_total, 0.0f, 0.9f) * 255.0f);

  analogWrite(PWM_PIN, pwm_val);

  delay(1);
}
