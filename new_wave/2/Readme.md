Worked for 43m 38s

I've extended the controller with an independently identified auxiliary power path and, importantly, a measurement-based mechanism for deciding whether using it actually improves efficiency.

Updated adaptive converter controller

Arduino C++ · 2,419 lines · 103 KB source

Download adaptive_converter_controller.ino

### 1\. Two independently controlled buck paths

The default ATmega328P configuration is:

| Property              | Primary   | Auxiliary |
| --------------------- | --------- | --------- |
| PWM output            | D9 / OC1A | D3 / OC2B |
| Switching frequency   | 20 kHz    | 200 kHz   |
| Inductance assumption |           |           |
| Timer                 | Timer1    | Timer2    |
| Duty resolution       | 0.125%    | 1.25%     |
| Maximum duty          | 90%       | 90%       |

The auxiliary frequency is ten times higher, approximately compensating for its ten-times-smaller inductance in the idealised current-ripple relationship:

\\Delta I \\propto \\frac{1}{L f_s}

The two MOSFETs must operate as separate buck branches, each with its own inductor and freewheel path, connected to the common output bus. This is not a scheme for independently PWM-driving two MOSFETs connected to the same inductor.

Important Uno limitation: the auxiliary timer provides only 80 PWM counts at 200 kHz. A 1.25% duty increment is relatively coarse for a low-inductance branch. The controller therefore requires branch-current measurement and independent hardware current protection. A3 is not fast enough to protect against switching-cycle current peaks.

### 2\. Auxiliary operation is deliberately restricted

The auxiliary branch is not treated as an alternative main power stage.

Its operating conditions are:

* Low-power entry at 1 A.

* Low-power exit at 1.5 A, with hysteresis.

* Additional load-estimate restriction at 1.2 A.

* Auxiliary identification only below 0.6 A total output current.

* Maximum 15% allocation of the predicted current-control increment.

* Automatic reduction and eventual shutdown when leaving the low-power region.

Its lower inductance is incorporated into the identification scaling:

```cpp
#define AUX_MODEL_DUTY_SCALE_BP 5000L
```

This uses a ten-times-larger normalisation scale than the primary branch, reflecting the assumed ten-times-higher local duty-to-current sensitivity.

### 3\. New empirical efficiency learning

This is the most significant additional feature.

The controller can optionally measure:

P_{\\mathrm{loss}} =
V_{\\mathrm{in}}I_{\\mathrm{in}}
-
V_{\\mathrm{out}}I_{\\mathrm{out}}

It maintains separate loss profiles for primary-only and auxiliary operation, indexed by output-current and source-voltage ranges.

The measurement process:

* Waits for stable operating conditions.

* Collects 16 samples at an unchanged PWM operating point.

* Calculates average input-output power loss.

* Updates the corresponding operating-point profile.

* Requires at least three stable measurement windows for each mode.

* Compares measured losses when ranking auxiliary candidates.

The empirical result replaces the provisional MOSFET-loss estimate once sufficient measurements exist.

This is preferable to assuming that the faster MOSFET is automatically more efficient. Its higher creates a conduction penalty, while its switching characteristics may provide an advantage. The measured system loss determines which effect wins.

The profiles are held in RAM and reset at power-up.

Additional sensor: A4 measures source current. It must be an appropriately amplified, calibrated current-sense output. A4 is also SDA on the Uno, so it cannot be shared with I²C.

The input-current sensor should measure the converter power input rather than accidentally including unrelated Arduino consumption. The measurement estimates total converter electrical loss, not MOSFET junction temperature.

### 4\. Hardware configuration required to enable the auxiliary branch

The auxiliary output is disabled by default.

To arm it, configure:

```cpp
#define AUX_PATH_ARMED 1
#define AUX_FAST_CURRENT_PROTECTION_PRESENT 1
#define AUX_CURRENT_SENSE_ENABLED 1
#define AUX_HARDWARE_FAULT_STATUS_ENABLED 1
```

For empirical efficiency measurement, additionally configure:

```cpp
#define SOURCE_SENSE_ENABLED 1
#define SOURCE_CURRENT_SENSE_ENABLED 1
```

The mandatory protection arrangements are:

* A3: auxiliary branch average-current sensor.

* D2: latched hardware-fault status.

* Independent hardware current protection directly inhibiting the auxiliary gate driver.

The D2 fault signal must be a latched fault indication, not a raw per-cycle current-limit pulse. The Arduino must not be responsible for stopping a dangerous switching-cycle current excursion.

The auxiliary branch current thresholds, shunt values, amplifier gains and MOSFET loss parameters all need calibration against the actual hardware.

### 5\. Verification results

Passed host-side tests

| Test                                          | Result                                  |
| --------------------------------------------- | --------------------------------------- |
| Strict C++11 compilation                      | Passed                                  |
| Fast/slow RLS regression recovery             | Maximum coefficient error 28 Q14 counts |
| Auxiliary candidate evaluation                | Passed                                  |
| PWM register configuration                    | 20 kHz / 200 kHz verified               |
| PWM quantisation                              | Passed                                  |
| Empirical efficiency comparison               | Passed                                  |
| ADC saturation fault                          | Passed                                  |
| External comparator fault latching            | Passed                                  |
| AddressSanitizer / UndefinedBehaviorSanitizer | Passed                                  |

The synthetic efficiency test produced 2.25 W estimated loss for primary-only operation and 1.95 W for auxiliary operation, correctly rewarding the auxiliary candidate. These are test inputs, not measured hardware performance.

One remaining qualification issue: I have not performed a genuine AVR-toolchain build or physical hardware-in-the-loop test. Consequently, actual Uno flash usage, SRAM margin and execution time against the 2 ms control period are not yet verified. The strict compilation tests used host-side Arduino and AVR-register stubs.

My next priority would be validating the auxiliary branch's actual current dynamics and PWM resolution against its inductance. The Uno's 1.25% auxiliary duty granularity may prove to be the limiting factor even when the identification algorithm is working correctly. If finer control is necessary, the eventual solution may require a higher-resolution PWM peripheral rather than further complexity in the control law.
