
// Include the watchdog timer library
#include <avr/wdt.h>

// Define pins
#define PWM_PIN 9 // PWM output pin for boost converter
#define VOLTAGE_PIN A0 // Analog input pin for voltage feedback
#define CURRENT_PIN A1 // Analog input pin for current feedback
#define THERMISTOR_PIN A2 // Analog input pin for thermistor feedback
#define WATCHDOG_PIN 10 // Digital output pin for watchdog timer
#define DESTINATION_PIN A3 // Analog input pin for destination voltage feedback
#define INPUT_VOLTAGE_PIN A4 // Analog input for inductance calculation, connected to input voltage

#include "shared_defs.h"

// Define constants
#define PWM_FREQ 2000 // PWM frequency in Hz - lowered for simulation stability
#define PWM_MAX 255 // Maximum PWM duty cycle
#define PWM_MIN 0 // Minimum PWM duty cycle
#define VOLTAGE_REF 5.0 // Reference voltage for analog inputs in volts
#define VOLTAGE_SETPOINT 12.0 // Desired output voltage in volts
#define VOLTAGE_TOLERANCE 0.2 // Allowed deviation from setpoint in volts
#define CURRENT_LIMIT 5.0 // Maximum output current in amps
#define CURRENT_TOLERANCE 0.05 // Allowed deviation from limit in amps
#define THERMISTOR_BETA 3950 // Beta coefficient of thermistor in K
#define THERMISTOR_R0 10000 // Resistance of thermistor at 25 C in ohms
#define THERMISTOR_T0 298.15 // Temperature of thermistor at R0 in K
#define THERMISTOR_RL 10000 // Load resistance of thermistor in ohms
#define THERMAL_LIMIT 60.0 // Maximum temperature of thermistor in C
#define CAPACITOR_VALUE 100e-6 // Output capacitor value in farads

// Define variables
int pwm = 127; // Start at ~50% duty cycle for estimation stability
float voltage = 0; // Measured output voltage in volts
float current = 0; // Measured output current in amps
float temperature = 0; // Measured temperature of thermistor in C
float destination = 0; // Measured destination voltage in volts
float inductance = 0; // Estimated coil inductance in henrys

// Initialize the system
void setup() {
  // Set pin modes
  pinMode(PWM_PIN, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);
  pinMode(THERMISTOR_PIN, INPUT);
  pinMode(WATCHDOG_PIN, OUTPUT);
  pinMode(DESTINATION_PIN, INPUT);
  pinMode(INPUT_VOLTAGE_PIN, INPUT);

  // Set PWM frequency and initial duty cycle

  analogWrite(PWM_PIN, pwm);
  // analogWriteFrequency(PWM_PIN, PWM_FREQ);

  // Set the analog reference to the default 5V reference
  analogReference(DEFAULT);

  // Enable the watchdog timer with a timeout period of 1 second
  wdt_enable(WDTO_1S);

  // Initialize the inductance estimate with a random value between 10 and 100 microhenrys
  inductance = random(10, 100) * 1e-6;
}

// Control the system
void loop() {
  // Read and convert analog inputs to physical values
  float ref = (_analog_reference_mode == INTERNAL) ? 1.1 : HARDWARE_ADC_REF;
  // Use exponential moving average for noise reduction
  voltage = 0.9 * voltage + 0.1 * ((float)analogRead(VOLTAGE_PIN) * ref / 1024.0 * VOLTAGE_DIVIDER_RATIO);
  current = 0.9 * current + 0.1 * ((float)analogRead(CURRENT_PIN) * ref / (1024.0 * R_SHUNT)); // R_SHUNT is the shunt resistor value in ohms
  temperature = THERMISTOR_BETA / log((float)analogRead(THERMISTOR_PIN) * THERMISTOR_RL / (1024.0 * THERMISTOR_R0) * exp(THERMISTOR_BETA / THERMISTOR_T0)) - 273.15;
  destination = 0.9 * destination + 0.1 * ((float)analogRead(DESTINATION_PIN) * ref / 1024.0 * VOLTAGE_DIVIDER_RATIO);

  // Check if any of the conditions are violated and adjust PWM accordingly
  // Priority: Safety (Thermal/Current) > Regulation (Voltage)
  if (temperature > THERMAL_LIMIT) { // Thermistor temperature is too high
    pwm = PWM_MIN; // Set PWM to minimum value
    analogWrite(PWM_PIN, pwm); // Update PWM output
    wdt_reset(); // Reset watchdog timer
  }
  
  else if (current > CURRENT_LIMIT + CURRENT_TOLERANCE) { // Output current is too high
    pwm--;
    if (pwm < PWM_MIN) pwm = PWM_MIN; // Limit PWM to minimum value
    analogWrite(PWM_PIN, pwm); // Update PWM output
    wdt_reset(); // Reset watchdog timer
  }

  else if (voltage > VOLTAGE_SETPOINT + VOLTAGE_TOLERANCE) { // Output voltage is too high
    pwm--;
    if (pwm < PWM_MIN) pwm = PWM_MIN; // Limit PWM to minimum value
    analogWrite(PWM_PIN, pwm); // Update PWM output
    wdt_reset(); // Reset watchdog timer
  }
  
  else if (voltage < VOLTAGE_SETPOINT - VOLTAGE_TOLERANCE) { // Output voltage is too low
    if (millis() % 2 == 0) pwm++;
    // Limit PWM to 75% to avoid inductor saturation and output collapse in simulation
    if (pwm > 191) pwm = 191;
    analogWrite(PWM_PIN, pwm); // Update PWM output
    wdt_reset(); // Reset watchdog timer
  }
  
  else { // All conditions are satisfied
    wdt_reset(); // Reset watchdog timer

    // Apply a self-tuning strategy: Use estimated inductance for feed-forward control
    float vin_raw = (float)analogRead(INPUT_VOLTAGE_PIN) * ref / 1024.0;
    float duty_ideal = 1.0 - (vin_raw / VOLTAGE_SETPOINT);
    if (duty_ideal < 0) duty_ideal = 0;

    // Use estimated inductance to adjust duty (compensate for losses proportional to L)
    // D = D_ideal + (I_out * R_L) / Vin ... approximated by a factor of inductance
    float duty_compensated = duty_ideal + (current * (inductance * 1000.0) / vin_raw);

    int target_pwm = (int)(duty_compensated * 255.0);
    if (target_pwm > 191) target_pwm = 191; // Cap at 75%

    // Smoothly approach target PWM
    if (pwm < target_pwm) pwm++;
    else if (pwm > target_pwm) pwm--;

    analogWrite(PWM_PIN, pwm);

    // --- Inductance Observer ---
    float duty_current = (float)pwm / 255.0;
    if (duty_current >= 1.0) duty_current = 0.99;

    // Refined model for Inductance Estimation
    // In a boost converter, the relationship between input and output voltage
    // is affected by the inductor current ripple and load.
    // A simplified model for the voltage gain including losses:
    // Vout = Vin / (1 - D) - I_out * (R_inductor / (1 - D)^2 + R_switch / (1 - D) + ...)

    // For estimation, let's use the current ripple formula: delta_I = (Vin * D) / (L * F)
    // If we can measure the peak current (which some systems do), we could estimate L.
    // Here, we'll use a simplified observer:
    float expected_gain = 1.0 / (1.0 - duty_current);
    float expected_vout = vin_raw * expected_gain;

    // The discrepancy between measured destination voltage and theoretical Vout
    // is partly due to inductance-related losses and dynamics.
    float error = destination - expected_vout;

    // Gradient descent to update inductance estimate (as a proxy for system health)
    // The learning rate 1e-6 is chosen for stability and observability in 0.5s simulation
    inductance += error * 1e-6;

    // Limit the inductance estimate to a reasonable range of values between 10 and 5000 microhenrys
    if (inductance < 10e-6) inductance = 10e-6;
    if (inductance > 5000e-6) inductance = 5000e-6;
  }
}
