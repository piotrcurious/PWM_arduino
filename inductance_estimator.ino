
// Include the watchdog timer library
#include <avr/wdt.h>
#include "shared_defs.h"

// Define pins
#define PWM_PIN 9
#define VOLTAGE_PIN A0
#define CURRENT_PIN A1
#define THERMISTOR_PIN A2
#define INPUT_VOLTAGE_PIN A4

// Define constants
#define TARGET_V 12.0
#define CURRENT_LIMIT 5.0
#define THERMAL_LIMIT 60.0

// PI Controller Constants
#define KP 5.0f
#define KI 50.0f

// Global variables
float inductance = 0; // Estimated coil inductance
float integral_v = 0;
int pwm = 0;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  analogReference(DEFAULT);
  wdt_enable(WDTO_1S);

  // Initial estimate
  inductance = 50e-6;
}

void loop() {
  float ref = (_analog_reference_mode == INTERNAL) ? 1.1 : HARDWARE_ADC_REF;
  float scale = ref / 1024.0;
  
  // Filtered readings
  static float voltage = 0;
  static float current = 0;
  static float temp = 25.0;

  voltage = 0.8 * voltage + 0.2 * ((float)analogRead(VOLTAGE_PIN) * scale * VOLTAGE_DIVIDER_RATIO);
  current = 0.8 * current + 0.2 * ((float)analogRead(CURRENT_PIN) * scale / R_SHUNT);
  
  float therm_raw = (float)analogRead(THERMISTOR_PIN);
  if (therm_raw > 0) {
      float r_therm = 10000.0 * (1024.0 / therm_raw - 1.0);
      temp = 1.0 / (log(r_therm / 10000.0) / 3950.0 + 1.0 / 298.15) - 273.15;
  }
  
  float vin = (float)analogRead(INPUT_VOLTAGE_PIN) * scale;

  // ── 1. Safety Checks (Highest Priority) ───────────────────────────────────
  if (temp > THERMAL_LIMIT || current > CURRENT_LIMIT) {
    pwm = 0;
    integral_v = 0;
  } else {
    // ── 2. PI Control Law ───────────────────────────────────────────────────
    float error_v = TARGET_V - voltage;
    const float dt = 0.001;

    integral_v += error_v * dt;
    integral_v = constrain(integral_v, -1.0, 1.0); // Anti-windup

    float pi_out = KP * error_v + KI * integral_v;

    // ── 3. Inductance-Based Feed-forward ────────────────────────────────────
    // Ideal duty cycle for boost converter
    float D_eq = 1.0 - (vin / (TARGET_V + 0.1));

    // We can use the estimate to adjust the sensitivity of the loop
    // High inductance = slower response needed.
    float L_factor = inductance / 10e-3;
    if (L_factor < 0.1) L_factor = 0.1;

    float total_duty = (D_eq + pi_out / L_factor);
    pwm = (int)(constrain(total_duty, 0.0, 0.85) * 255.0);

    // ── 4. Inductance Observer ──────────────────────────────────────────────
    // V_L = L * di/dt. Over one cycle, delta_I = (Vin * D) / (L * Fsw)
    // We can solve for L if we assume the output is steady.
    // L = (Vin * D) / (delta_I * Fsw)
    // Simplified observer based on voltage discrepancy:
    float expected_v = vin / (1.0 - (float)pwm / 255.0 + 0.01);
    float v_err = expected_v - voltage;

    inductance += v_err * 1e-6; // Learning step
    inductance = constrain(inductance, 1e-6, 100e-3);
  }

  analogWrite(PWM_PIN, pwm);
  wdt_reset();
  delay(1);
}
