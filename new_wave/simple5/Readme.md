Worked for 20m 18s

I've corrected the current-limit architecture and produced another revision.

Adaptive converter controller v4

Arduino C++ · EEPROM calibration · Dual PWM · Dual ACS712

Download adaptive_converter_controller_v4.ino

### 1\. Independent input and output current control

The two limits are now genuinely independent.

| Setting            | Output / load | Input / source |
| ------------------ | ------------- | -------------- |
| Default limit      | 4.00 A        | 3.00 A         |
| Adjustable range   | 0.50–4.00 A   | 0.50–4.00 A    |
| Software hard trip | 4.50 A        | 4.50 A         |
| Sensor             | ACS712-05B    | ACS712-05B     |
| ADC                | A1            | A4             |

The important correction is that input current is no longer compared directly with output current or used to overwrite the thermally derated output-current limit.

Instead:

* The output-current controller protects the load.

* The input-current controller independently reduces commanded output power as source current approaches its limit.

* At 92% of the input setting, it additionally applies a stronger duty reduction.

* Both controllers have hysteresis to avoid unnecessary switching between operating states.

This also makes the control more appropriate when the source and output voltages differ substantially.

### 2\. Optional potentiometers

The potentiometers are disabled by default, so unconnected ADC pins cannot produce random current settings.

Configuration examples:

Arduino Nano:

```cpp
#define OUTPUT_LIMIT_POT_PIN A5
#define INPUT_LIMIT_POT_PIN  A6
```

Arduino Uno with two potentiometers:

```cpp
#define LIMIT_POT_MUX_ENABLED 1
```

This uses a 74HC4051 two-channel arrangement on A5, with D5 selecting the channel.

With two pots, each is sampled approximately every 100 ms. The filtered settings are updated outside the control routine. Neither pot affects the timing of the 499 Hz feedback loop.

### 3\. Conversion efficiency

The input ACS712 is restored, so the controller can measure both sides:

P_{\\mathrm{in}}=V_{\\mathrm{in}}I_{\\mathrm{in}}P_{\\mathrm{out}}=V_{\\mathrm{out}}I_{\\mathrm{out}}\\eta=100\\frac{P_{\\mathrm{out}}}{P_{\\mathrm{in}}}

The efficiency learner only accepts measurements with valid calibrated sensor data and physically consistent power readings. It retains separate operating-point profiles for primary-only and auxiliary operation.

Calibration of both current sensors and both voltage measurements remains stored in redundant EEPROM records.

### 4\. Verification

Unit tests for EEPROM recovery, both ACS712 conversions, efficiency calculations, thermal derating, input-current guard hysteresis, potentiometer mapping and auxiliary-path transitions.

AddressSanitizer and UndefinedBehaviorSanitizer.

Strict host-side compilation configurations for single-pot, dual-pot, Uno multiplexer and auxiliary-enabled ATmega328P register configurations.

One qualification remains: the environment does not have an AVR cross-compiler, so actual Uno flash/SRAM usage and timing have not been verified. Hardware testing is also outstanding.

Protection distinction: the ACS712 software limits are supervisory, not cycle-by-cycle protection. The independent input and output hardware comparators must directly inhibit the MOSFET gates. The auxiliary branch additionally needs its own fast current protection. Potentiometers adjust the software operating limits, not those emergency hardware trip thresholds.

This revision preserves the adaptive identification and efficiency-learning architecture while keeping source protection, load protection and thermal derating as separate control mechanisms.
