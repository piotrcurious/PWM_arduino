
// Define pins
#define PWM_PIN 9 // PWM output pin for boost converter
#define VOLTAGE_PIN A0 // Analog input pin for voltage feedback
#define CURRENT_PIN A1 // Analog input pin for current feedback
#define THERMISTOR_PIN A2 // Analog input pin for thermistor feedback

// Define constants
#define PWM_FREQ 20000 // PWM frequency in Hz
#define PWM_MAX 255 // Maximum PWM duty cycle
#define PWM_MIN 0 // Minimum PWM duty cycle
#define VOLTAGE_REF 5.0 // Reference voltage for analog inputs in volts
#define VOLTAGE_SETPOINT 12.0 // Desired output voltage in volts
#define VOLTAGE_TOLERANCE 0.1 // Allowed deviation from setpoint in volts
#define CURRENT_LIMIT 1.0 // Maximum output current in amps
#define CURRENT_TOLERANCE 0.05 // Allowed deviation from limit in amps
#define THERMISTOR_BETA 3950 // Beta coefficient of thermistor in K
#define THERMISTOR_R0 10000 // Resistance of thermistor at 25 C in ohms
#define THERMISTOR_T0 298.15 // Temperature of thermistor at R0 in K
#define THERMISTOR_RL 10000 // Load resistance of thermistor in ohms
#define THERMAL_LIMIT 60.0 // Maximum temperature of thermistor in C

// Define variables
int pwm = PWM_MIN; // Initial PWM duty cycle
float voltage = 0; // Measured output voltage in volts
float current = 0; // Measured output current in amps
float temperature = 0; // Measured temperature of thermistor in C

// Initialize the system
void setup() {
  // Set pin modes
  pinMode(PWM_PIN, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);
  pinMode(THERMISTOR_PIN, INPUT);

  // Set PWM frequency and initial duty cycle
  analogWrite(PWM_PIN, pwm);
  analogWriteFrequency(PWM_PIN, PWM_FREQ);
}

// Control the system
void loop() {
  // Read and convert analog inputs to physical values
  voltage = analogRead(VOLTAGE_PIN) * VOLTAGE_REF / 1024;
  current = analogRead(CURRENT_PIN) * VOLTAGE_REF / (1024 * R_SHUNT); // R_SHUNT is the shunt resistor value in ohms
  temperature = THERMISTOR_BETA / log(analogRead(THERMISTOR_PIN) * THERMISTOR_RL / (1024 * THERMISTOR_R0) * exp(THERMISTOR_BETA / THERMISTOR_T0)) - 273.15;

  // Check if any of the conditions are violated and adjust PWM accordingly
  if (voltage > VOLTAGE_SETPOINT + VOLTAGE_TOLERANCE) { // Output voltage is too high
    pwm--;
    if (pwm < PWM_MIN) pwm = PWM_MIN; // Limit PWM to minimum value
    analogWrite(PWM_PIN, pwm); // Update PWM output 
  }
    }
  
  else if (current > CURRENT_LIMIT + CURRENT_TOLERANCE) { // Output current is too high 
    pwm--;
    if (pwm < PWM_MIN) pwm = PWM_MIN; // Limit PWM to minimum value
    analogWrite(PWM_PIN, pwm); // Update PWM output
  }
  
  else if (temperature > THERMAL_LIMIT) { // Thermistor temperature is too high
    pwm = PWM_MIN; // Set PWM to minimum value
    analogWrite(PWM_PIN, pwm); // Update PWM output
  }
}
  else if (voltage < VOLTAGE_SETPOINT - VOLTAGE_TOLERANCE) { // Output voltage is too low 
    pwm++;
    if (pwm > PWM_MAX) pwm = PWM_MAX; // Limit PWM to maximum value 
    analogWrite(PWM_PIN, pwm); // Update PWM output
  }
  
  else if (current > CURRENT_LIMIT + CURRENT_TOLERANCE) { // Output current is too high 
    pwm--;
    if (pwm < PWM_MIN) pwm = PWM_MIN; // Limit PWM to minimum value
    analogWrite(PWM_PIN, pwm); // Update PWM output
  }
  
  else if (temperature > THERMAL_LIMIT) { // Thermistor temperature is too high
    pwm = PWM_MIN; // Set PWM to minimum value
    analogWrite(PWM_PIN, pwm); // Update PWM output
  }
}
