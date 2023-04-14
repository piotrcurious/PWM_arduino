
// Define the pins for voltage and current feedback, input voltage, and integrating capacitor
const int VFB = A0; // Voltage feedback from load
const int IFB = A1; // Current feedback from current sense resistor
const int VIN = A2; // Input voltage
const int CI = A3; // Integrating capacitor

// Define the pins for the key transistor and the synchronous rectifier transistor
const int KT = 2; // Key transistor
const int SRT = 3; // Synchronous rectifier transistor

// Define the parameters for the dc-dc boost converter
const float VREF = 5.0; // Reference voltage for the output
const float R1 = 10000.0; // Resistance of the resistor divider for the integrating capacitor
const float R2 = 5000.0; // Resistance of the resistor divider for the integrating capacitor
const float KI = 0.01; // Integral gain for the PI controller
const float KP = 0.1; // Proportional gain for the PI controller
const float DT = 0.001; // Switching period in seconds
const float DMAX = 0.9; // Maximum duty cycle

// Define some variables for the PI controller and the duty cycle
float error = 0.0; // Error between the reference voltage and the output voltage
float integral = 0.0; // Integral of the error
float duty = 0.0; // Duty cycle of the key transistor

void setup() {
  // Set the key transistor and the synchronous rectifier transistor as outputs
  pinMode(KT, OUTPUT);
  pinMode(SRT, OUTPUT);

  // Set the analog reference to the internal 1.1V reference
  analogReference(INTERNAL);

  // Set the analog input pins to input mode
  pinMode(VFB, INPUT);
  pinMode(IFB, INPUT);
  pinMode(VIN, INPUT);
  pinMode(CI, INPUT);
}

void loop() {
  // Read the analog values from the feedback pins and convert them to voltages
  float vfb = analogRead(VFB) * (1.1 / 1023.0); // Voltage feedback from load in volts
  float ifb = analogRead(IFB) * (1.1 / 1023.0); // Current feedback from current sense resistor in volts
  float vin = analogRead(VIN) * (1.1 / 1023.0); // Input voltage in volts
  float ci = analogRead(CI) * (1.1 / 1023.0); // Integrating capacitor voltage in volts

  // Calculate the output voltage from the integrating capacitor voltage and the resistor divider ratio
  float vout = ci * (R1 + R2) / R2; // Output voltage in volts

  // Calculate the error between the reference voltage and the output voltage
  error = VREF - vout;

  // Update the integral of the error with anti-windup limit
  integral += error * DT;
  integral = constrain(integral, -DMAX / KI, DMAX / KI);

  // Calculate the duty cycle of the key transistor using a PI controller
  duty = KP * error + KI * integral;
  duty = constrain(duty, 0.0, DMAX);

  // Turn on the key transistor for a fraction of the switching period proportional to the duty cycle
  digitalWrite(KT, HIGH);
  delayMicroseconds(duty * DT * 1000000);

  // Turn off the key transistor and turn on the synchronous rectifier transistor for the rest of the switching period
  digitalWrite(KT, LOW);
  digitalWrite(SRT, HIGH);
  delayMicroseconds((1 - duty) * DT * 1000000);

  // Turn off both transistors at the end of the switching period
  digitalWrite(SRT, LOW);
}
