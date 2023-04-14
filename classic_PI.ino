
// Define the pins for the PWM signal and the feedback signals
#define PWM_PIN 9
#define VOLTAGE_PIN A0
#define CURRENT_PIN A1

// Define the target output voltage and current in volts and amps
#define TARGET_VOLTAGE 12.0
#define TARGET_CURRENT 1.0

// Define the maximum duty cycle and the switching frequency in percent and hertz
#define MAX_DUTY_CYCLE 90.0
#define SWITCHING_FREQUENCY 100000.0

// Define the voltage and current thresholds for error calculation in volts and amps
#define VOLTAGE_THRESHOLD 0.1
#define CURRENT_THRESHOLD 0.01

// Define the proportional and integral gains for the PI controller
#define KP 0.5
#define KI 0.1

// Define the ADC resolution and reference voltage in bits and volts
#define ADC_RESOLUTION 10
#define ADC_REF_VOLTAGE 5.0

// Define some constants for convenience
#define PWM_MAX_VALUE 255 // The maximum value for analogWrite function
#define PWM_PERIOD (1000000.0 / SWITCHING_FREQUENCY) // The PWM period in microseconds
#define ADC_SCALE_FACTOR (ADC_REF_VOLTAGE / (1 << ADC_RESOLUTION)) // The scale factor to convert ADC readings to volts

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
  // Read the voltage and current feedback signals from the analog pins and convert them to volts and amps
  float voltage_feedback = analogRead(VOLTAGE_PIN) * ADC_SCALE_FACTOR;
  float current_feedback = analogRead(CURRENT_PIN) * ADC_SCALE_FACTOR;

  // Calculate the voltage and current errors by subtracting the feedback from the target values
  voltage_error = TARGET_VOLTAGE - voltage_feedback;
  current_error = TARGET_CURRENT - current_feedback;

  // Update the integral terms by adding the errors multiplied by the PWM period
  voltage_integral += voltage_error * PWM_PERIOD;
  current_integral += current_error * PWM_PERIOD;

  // Calculate the PI controller output by adding the proportional and integral terms multiplied by the gains
  float pi_output = KP * voltage_error + KI * voltage_integral;

  // Limit the PI output to the maximum duty cycle
  if (pi_output > MAX_DUTY_CYCLE) {
    pi_output = MAX_DUTY_CYCLE;
  }
  
  // Convert the PI output to a PWM value by scaling it to the PWM range
  int pwm_value = map(pi_output, 0, MAX_DUTY_CYCLE, 0, PWM_MAX_VALUE);

  // Write the PWM value to the PWM pin using analogWrite function
  analogWrite(PWM_PIN, pwm_value);

}
