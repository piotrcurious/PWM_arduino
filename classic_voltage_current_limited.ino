
// Define the PWM pin
#define PWM_PIN 9

// Define the feedback pins
#define VOLTAGE_PIN A0
#define CURRENT_PIN A1

// Define the PWM frequency and resolution
#define PWM_FREQ 1000 // Hz
#define PWM_RES 256 // bits

// Define the feedback thresholds
#define VOLTAGE_MAX 5.0 // V
#define CURRENT_MAX 1.0 // A

// Define the feedback gains
#define VOLTAGE_GAIN 0.1 // proportional gain for voltage feedback
#define CURRENT_GAIN 0.1 // proportional gain for current feedback

// Define the global variables
int pwm_value = 0; // PWM duty cycle value
float voltage = 0; // voltage feedback value
float current = 0; // current feedback value
float error_v = 0; // voltage error value
float error_i = 0; // current error value

void setup() {
  // Set the PWM pin as output
  pinMode(PWM_PIN, OUTPUT);

  // Set the timer1 to generate PWM signal on pin 9 with the desired frequency and resolution
  TCCR1A = (1 << COM1A1) | (1 << WGM11); // set non-inverting mode and fast PWM mode for timer1A
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // set fast PWM mode and no prescaler for timer1B
  ICR1 = F_CPU / PWM_FREQ - 1; // set the top value for timer1 to match the PWM frequency
  OCR1A = pwm_value * (ICR1 + 1) / PWM_RES; // set the compare value for timer1A to match the PWM duty cycle
}

void loop() {
  // Read the feedback values from the analog pins and convert them to voltage and current
  voltage = analogRead(VOLTAGE_PIN) * (5.0 / 1023.0); // read the voltage pin and scale it to 0-5V range
  current = analogRead(CURRENT_PIN) * (5.0 / 1023.0); // read the current pin and scale it to 0-5V range

  // Calculate the errors between the feedback values and the thresholds
  error_v = VOLTAGE_MAX - voltage; // calculate the voltage error as the difference between the maximum and the actual voltage
  error_i = CURRENT_MAX - current; // calculate the current error as the difference between the maximum and the actual current

  // Adjust the PWM duty cycle value based on the feedback errors and the gains
  pwm_value += VOLTAGE_GAIN * error_v + CURRENT_GAIN * error_i; // update the PWM value by adding a weighted sum of the errors

  // Constrain the PWM duty cycle value to be within the resolution range
  pwm_value = constrain(pwm_value, 0, PWM_RES - 1); // limit the PWM value to be between 0 and PWM_RES - 1

  // Update the compare value for timer1A to match the new PWM duty cycle value
  OCR1A = pwm_value * (ICR1 + 1) / PWM_RES; // set the compare value for timer1A to match the PWM duty cycle

}
