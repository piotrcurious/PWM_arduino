
// Define the pins for the PWM signal and the feedback signals
#define PWM_PIN 9
#define VOLTAGE_PIN A0
#define CURRENT_PIN A1

#include "shared_defs.h"

// Define the target output voltage and current in volts and amps
#define TARGET_VOLTAGE 10.0
#define TARGET_CURRENT 0.5

// Define the maximum duty cycle and the switching frequency in percent and hertz
#define MAX_DUTY_CYCLE 90.0
#define SWITCHING_FREQUENCY 100000.0

// Define the voltage and current thresholds for error calculation in volts and amps
#define VOLTAGE_THRESHOLD 0.1
#define CURRENT_THRESHOLD 0.01

// Define the proportional and integral gains for the PI controller
#define KP 1.0
#define KI 200.0

// Define the ADC resolution and reference voltage in bits and volts
#define ADC_RESOLUTION 10
#define ADC_REF_VOLTAGE 5.0

// Define some constants for convenience
#define PWM_MAX_VALUE 255 // The maximum value for analogWrite function

// Declare some global variables for storing the error and the integral term
float voltage_error = 0.0;
float current_error = 0.0;
float voltage_integral = 0.0;
float current_integral = 0.0;

void setup() {
  // Set the PWM pin as output and initialize it to low
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);

  // Set the ADC reference voltage to the defined value
  analogReference(DEFAULT);
}

void loop() {
  // Dynamic target tracking: square wave setpoint
  float dynamic_target = TARGET_VOLTAGE;
  if (fmod(millis() / 1000.0, 0.2) < 0.1) {
    dynamic_target = TARGET_VOLTAGE * 0.8;
  }

  // Soft start logic
  static float soft_start_target = 0;
  if (soft_start_target < dynamic_target) soft_start_target += 0.05;
  if (soft_start_target > dynamic_target) soft_start_target -= 0.05;

  // Read the voltage and current feedback signals from the analog pins and convert them to volts and amps
  float ref = (_analog_reference_mode == INTERNAL) ? 1.1 : HARDWARE_ADC_REF;
  float voltage_feedback = (float)analogRead(VOLTAGE_PIN) * (ref / 1024.0) * VOLTAGE_DIVIDER_RATIO;
  float current_feedback = (float)analogRead(CURRENT_PIN) * (ref / 1024.0);

  // Calculate the voltage and current errors by subtracting the feedback from the target values
  voltage_error = soft_start_target - voltage_feedback;
  current_error = TARGET_CURRENT - current_feedback;

// Update the integral terms by adding the errors multiplied by the loop period (in seconds)
float dt = 0.001; // Adjusted for simulation scale
voltage_integral += voltage_error * dt;
current_integral += current_error * dt;

// Anti-windup
voltage_integral = constrain(voltage_integral, -100.0, 100.0);

  // Calculate the PI controller output by adding the proportional and integral terms multiplied by the gains
  float pi_output = KP * voltage_error + KI * voltage_integral;
  if (pi_output < 0) pi_output = 0;

  // Limit the PI output to the maximum duty cycle
  if (pi_output > 90.0) {
    pi_output = 90.0;
  }
  if (pi_output < 0.0) {
    pi_output = 0.0;
  }
  
  // Convert the PI output to a PWM value by scaling it to the PWM range
  // pi_output is in percent (0-100)
  int pwm_value = (int)(pi_output * 2.55);

  // Write the PWM value to the PWM pin using analogWrite function
  analogWrite(PWM_PIN, pwm_value);

}
