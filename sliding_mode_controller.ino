
#include "shared_defs.h"

// Pins
#define PWM_PIN 9
#define VOLTAGE_PIN A0
#define CURRENT_PIN A1

// Targets
#define TARGET_V 12.0
#define CURRENT_LIMIT_SMC 5.0

// Sliding surface parameters
#define ALPHA 100.0

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  analogReference(DEFAULT);
}

void loop() {
  float ref = (_analog_reference_mode == INTERNAL) ? 1.1 : HARDWARE_ADC_REF;
  float scale = ref / 1024.0;

  float v_out = (float)analogRead(VOLTAGE_PIN) * scale * VOLTAGE_DIVIDER_RATIO;
  float i_l = (float)analogRead(CURRENT_PIN) * scale / R_SHUNT;

  // Dynamic setpoint
  float dyn_v = TARGET_V;
  if (fmod(millis() / 1000.0, 0.2) < 0.1) {
    dyn_v = TARGET_V * 0.8;
  }

  // Soft start setpoint
  static float setpoint = 0;
  if (setpoint < dyn_v) setpoint += 0.05;
  if (setpoint > dyn_v) setpoint -= 0.05;

  // Error tracking
  float e_v = setpoint - v_out;

  // Sliding surface: S = e_v + ALPHA * Integral(e_v) -- simplified
  // Here we use a surface based on energy and current demand
  // S = (I_l_target - I_l)
  // I_l_target is derived from voltage error
  float i_l_target = e_v * 2.0;
  if (i_l_target > CURRENT_LIMIT_SMC) i_l_target = CURRENT_LIMIT_SMC;
  if (i_l_target < 0) i_l_target = 0;

  float S = i_l_target - i_l;

  // Control law: if S > 0, switch ON; if S < 0, switch OFF
  // To avoid chattering, we use a small hysteresis or a continuous approximation
  static int pwm_val = 0;
  if (S > 0.1) {
    pwm_val += 5;
  } else if (S < -0.1) {
    pwm_val -= 5;
  }

  pwm_val = constrain(pwm_val, 0, 230); // Max 90% duty
  analogWrite(PWM_PIN, pwm_val);

  delay(1);
}
