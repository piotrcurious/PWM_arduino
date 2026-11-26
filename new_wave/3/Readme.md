Worked for 23m 26s

Updated the controller to use 80 kHz auxiliary PWM and ACS712-05B current sensing, including automatic zero calibration. I also strengthened the hardware-protection requirements after auditing the primary control path.

Adaptive converter controller — ACS712 revision

Arduino C++ · 112 KB · ATmega328P / Uno

Download the updated sketch

### 1\. PWM configuration

| Parameter       | Primary path | Auxiliary path |
| --------------- | ------------ | -------------- |
| Output          | D9 / OC1A    | D3 / OC2B      |
| Frequency       | 20 kHz       | 80 kHz         |
| Timer           | Timer1       | Timer2         |
| Timer counts    | 800          | 200            |
| Duty resolution | 0.125%       | 0.5%           |
| Maximum duty    | 90%          | 90%            |

The auxiliary frequency is now generated using Timer2 Fast PWM mode 7, with TOP=199 and prescaler 1.

The auxiliary timer is independent of the primary timer and does not interfere with `millis()`.

One important consequence of the frequency change:

With the assumed auxiliary inductance of one-tenth the primary inductance, the switching-frequency ratio is now 4:1 rather than 10:1.

For equal inductor voltage and duty:

\\frac{\\Delta I_{\\mathrm{aux}}}{\\Delta I_{\\mathrm{primary}}}
\\approx
\\frac{L_{\\mathrm{primary}}f_{\\mathrm{primary}}}
{L_{\\mathrm{aux}}f_{\\mathrm{aux}}}
=2.5

The auxiliary branch can therefore experience approximately 2.5 times the current ripple. Its peak current and inductor saturation margin need particular attention.

The RLS identification still learns its averaged dynamics, but the approximately 499 Hz measurement loop cannot reconstruct switching-cycle ripple.

### 2\. ACS712 integration

The code assumes genuine ACS712-05B sensors:

* Measurement range: ±5 A.

* Nominal sensitivity: 185 mV/A.

* Specified sensitivity range: 180–190 mV/A.

* Zero-current output: approximately half the supply voltage.

* Positive current from IP+ towards IP− produces an increasing output voltage.

These are the manufacturer's specifications. allegromicro.com +1

![ACS712 5A Current Sensor Module Buy Online at QuartzComponents.com](https://images.openai.com/static-rsc-4/eR4dJCGjWbauDm86okJoSRw9jNqAlrX90R0iMVnQPLfy0ica46UsxHpGI1Uo5-iZuyTOKzipWhM-vlQN7am5SIWwTXLLGQzaJzPNy7vATgf3NuD0lMe8uYdLvQmMiMaOas3CpM04KXmXewtredkZzU9SOHpMk54ufAIt3_2oCNWFj6-p-eb6isU6IGVr8aNA?purpose=fullsize)

Pin assignments:

| Pin | Function                           |
| --- | ---------------------------------- |
| A0  | Output voltage                     |
| A1  | Aggregate output-current ACS712    |
| A2  | Optional input-voltage measurement |
| A3  | Auxiliary branch-current ACS712    |
| A4  | Optional input-current ACS712      |
| D4  | Startup zero-calibration interlock |
| D2  | Shared hardware-fault status       |
| D9  | Primary PWM                        |
| D3  | Auxiliary PWM                      |

A1 measures total output current; A3 measures only the auxiliary branch current. They must not both measure the same common conductor.

### Automatic zero calibration

The new calibration sequence:

* Forces both PWM outputs low.

* Requires D4 to be LOW.

* Waits 250 ms for sensor stabilization.

* Discards four initial ADC conversions per channel.

* Collects 128 samples for each enabled ACS712.

* Calculates the mean offset in Q8 ADC counts.

* Checks the measurement spread and variance.

* Rechecks the interlock during calibration.

If calibration fails, the controller latches `FAULT_SENSOR_CALIBRATION` and refuses to operate.

Each sensor gets an independent offset. The current conversion uses signed fixed-point arithmetic and supports reversed sensor orientation through the polarity macros.

D4 must be connected to a genuine interlock condition proving zero current, ideally through an auxiliary contact of the physical isolation mechanism. It must not be a manually asserted convenience jumper. Software cannot determine whether a non-zero current has been accidentally incorporated into a zero calibration.

### Measurement resolution

At nominal sensitivity and a 5 V, 10-bit ADC:

I_{\\mathrm{LSB}} =
\\frac{5000}{1023 \\times 185}
\\approx 0.0264\\,A

One ADC count represents approximately 26 mA.

This is adequate for average-current supervision, but it does limit precision at very low auxiliary currents.

### 3\. Sensor filtering and calibration

The firmware assumes the ACS712 supply shares the Arduino AVCC/ADC reference rail.

For the sensor modules, I recommend checking:

* 100 nF local VCC-to-GND bypass capacitor.

* 47 nF FILTER-to-GND capacitor, unless already fitted.

* Correct current conductor direction.

* Actual sensor variant and module gain.

The manufacturer specifies an 80 kHz bandwidth under the 1 nF filter condition. For the 05B version, its datasheet specifies 21 mV peak-to-peak noise at 2 kHz bandwidth with 47 nF and a 10–90% rise time of approximately 88.2 μs. allegromicro.com +2

The 47 nF configuration is intended for average-current feedback, not peak-current protection.

The software limits remain:

| Protection                   | Threshold |
| ---------------------------- | --------- |
| Normal output current        | 4.00 A    |
| Software hard trip           | 4.50 A    |
| Auxiliary soft limit         | 1.00 A    |
| Auxiliary software hard trip | 1.20 A    |

The per-channel sensitivity overrides allow gain calibration against a known current reference. Zero calibration does not calibrate gain.

### 4\. Hardware protection audit

I found a significant weakness in the previous architecture: the primary path could operate with only its approximately 500 Hz software overcurrent measurement.

That is insufficient protection against fast inductor-current excursions.

The revised code now has mandatory compile-time checks for primary hardware protection:

```cpp
#define PRIMARY_FAST_CURRENT_PROTECTION_PRESENT 1
#define POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED 1
```

Only define these when the corresponding physical hardware exists.

The primary and auxiliary fast comparators must independently inhibit their MOSFET gates. Their open-collector fault outputs should be wired together to D2.

The D2 interrupt latches the fault in firmware. The hardware comparator, not the Arduino interrupt, performs the fast shutdown.

The code also checks D2's state immediately during startup, so a fault that was already active before interrupt installation is not missed.

The auxiliary path additionally requires:

```cpp
#define AUX_PATH_ARMED 1
#define AUX_FAST_CURRENT_PROTECTION_PRESENT 1
#define AUX_CURRENT_SENSE_ENABLED 1
```

This intentionally prevents an unprotected primary or auxiliary power stage from being treated as an operational production configuration.

### 5\. Verification

Strict C++11 compilation with warnings treated as errors for the generic path and three ATmega328P configurations.

AddressSanitizer and UndefinedBehaviorSanitizer tests.

ACS712 zero calibration, offset correction, positive/negative current conversion, polarity inversion and monotonicity over the ADC range.

Invalid calibration rejection, zero-interlock checks and sensor-fault handling.

Primary 20 kHz and auxiliary 80 kHz timer configuration, compare values and duty quantisation.

Hardware fault detection when the fault input is already active at startup.

Negative compile tests for missing primary hardware protection and uncalibrated efficiency-measurement gains.

What remains unverified: there is no genuine AVR cross-compilation, physical board execution, or hardware-in-the-loop validation yet. Consequently, actual Uno flash usage, SRAM margin and worst-case control execution time have not been established. I would not describe this as production-certified firmware.

### 6\. Two remaining engineering considerations

First, 80 kHz is a more reasonable auxiliary switching frequency than 200 kHz without a dedicated gate driver, but it is still demanding for direct ATmega328P gate drive. The actual MOSFET gate charge, gate resistor, pin current and transition time must be measured. A PWM frequency setting alone does not establish acceptable switching losses.

Second, ACS712-05B is suitable for the average-current supervision, but its resolution and noise become increasingly relevant in the low-power region where the auxiliary branch is supposed to improve efficiency. The existing empirical efficiency learner should only be enabled after calibrating all enabled current-sensor gains against a known reference:

```cpp
#define ACS712_GAINS_CALIBRATED 1
```

The firmware deliberately refuses empirical efficiency learning otherwise.

The next important validation is to measure the auxiliary inductor's actual peak-to-peak ripple and the MOSFET gate waveform at 80 kHz. Those two measurements will determine whether the assumed low-inductance branch is viable without a dedicated gate driver.
