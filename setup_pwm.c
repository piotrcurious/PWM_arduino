#include <avr/io.h>
#include <avr/interrupt.h>

// Function to setup PWM on pins 9 and 10 of Arduino Uno
void setupPWM(uint8_t bitDepth, uint16_t prescaler)
{
  // Configure pins 9 and 10 as output
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  // Set PWM mode to Fast PWM and configure prescaler
  uint8_t prescalerValue;
  switch (prescaler)
  {
  case 1:
    prescalerValue = _BV(CS10);
    break;
  case 8:
    prescalerValue = _BV(CS11);
    break;
  case 64:
    prescalerValue = _BV(CS11) | _BV(CS10);
    break;
  case 256:
    prescalerValue = _BV(CS12);
    break;
  case 1024:
    prescalerValue = _BV(CS12) | _BV(CS10);
    break;
  default:
    prescalerValue = _BV(CS10);
  }

  TCCR1A = (1 << WGM11) | (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | prescalerValue;

  // Set bit depth and calculate frequency
  ICR1 = (1 << bitDepth) - 1;
  float frequency = (float)F_CPU / (prescaler * (1 << bitDepth));
  Serial.print("PWM Frequency: ");
  Serial.print(frequency);
  Serial.println(" Hz");
}

void setup()
{
  Serial.begin(9600);
  setupPWM(10, 8); // Set bit depth to 10 and prescaler to 8
}

void loop()
{
  // Set duty cycle for pins 9 and 10
  OCR1A = 512; // 50% duty cycle for pin 9
  OCR1B = 256; // 25% duty cycle for pin 10

  delay(1000);
}
