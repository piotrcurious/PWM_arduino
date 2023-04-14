
// Define the pins for the key transistor and the synchronous rectifier transistor
#define KEY_PIN 2
#define SR_PIN 3

// Define the analog input pins for the voltage and current feedback
#define VOLTAGE_PIN A0
#define CURRENT_PIN A1

// Define the target output voltage and current in volts and amps
#define VOLTAGE_TARGET 12.0
#define CURRENT_TARGET 1.0

// Define the voltage and current thresholds for switching in volts and amps
#define VOLTAGE_THRESHOLD 0.5
#define CURRENT_THRESHOLD 0.1

// Define the switching frequency in Hz
#define FREQUENCY 100000

// Define the duty cycle variable
float duty_cycle = 0.5;

// Define the period and half-period variables in microseconds
unsigned long period = 1000000 / FREQUENCY;
unsigned long half_period = period / 2;

// Define the timer variables
unsigned long timer1 = 0;
unsigned long timer2 = 0;

void setup() {
  // Set the key pin and the SR pin as outputs
  pinMode(KEY_PIN, OUTPUT);
  pinMode(SR_PIN, OUTPUT);

  // Set the initial states of the key pin and the SR pin
  digitalWrite(KEY_PIN, LOW);
  digitalWrite(SR_PIN, HIGH);
}

void loop() {
  // Read the voltage and current feedback from the analog pins
  float voltage = analogRead(VOLTAGE_PIN) * (5.0 / 1023.0) * (R1 + R2) / R2; // R1 and R2 are the voltage divider resistors
  float current = analogRead(CURRENT_PIN) * (5.0 / 1023.0) / Rs; // Rs is the current sense resistor

  // Adjust the duty cycle based on the feedback and the target values
  if (voltage < VOLTAGE_TARGET - VOLTAGE_THRESHOLD) {
    duty_cycle += 0.01;
    if (duty_cycle > 1.0) duty_cycle = 1.0;
  }
  else if (voltage > VOLTAGE_TARGET + VOLTAGE_THRESHOLD) {
    duty_cycle -= 0.01;
    if (duty_cycle < 0.0) duty_cycle = 0.0;
  }

  if (current < CURRENT_TARGET - CURRENT_THRESHOLD) {
    duty_cycle += 0.01;
    if (duty_cycle > 1.0) duty_cycle = 1.0;
  }
  else if (current > CURRENT_TARGET + CURRENT_THRESHOLD) {
    duty_cycle -= 0.01;
    if (duty_cycle < 0.0) duty_cycle = 0.0;
  }

  // Switch the key pin and the SR pin according to the duty cycle and the switching frequency
  unsigned long now = micros();
  
  if (now - timer1 >= period) {
    timer1 = now;
    digitalWrite(KEY_PIN, HIGH);
    digitalWrite(SR_PIN, LOW);
    timer2 = now + half_period * duty_cycle;
    
    // If you want to measure the switching frequency, you can use this line to toggle an LED on pin 13 every cycle
    //digitalWrite(13, !digitalRead(13));
    
    // If you want to print out some debug information, you can use these lines to send data to serial monitor every cycle
    //Serial.print("Voltage: ");
    //Serial.print(voltage);
    //Serial.print(" V, Current: ");
    //Serial.print(current);
    //Serial.print(" A, Duty cycle: ");
    //Serial.println(duty_cycle);
    
   }
   
   if (now >= timer2) {
     digitalWrite(KEY_PIN, LOW);
     digitalWrite(SR_PIN, HIGH);
   }
}
