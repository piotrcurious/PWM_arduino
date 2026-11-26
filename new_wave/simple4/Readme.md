Worked for 38m 47s

I've simplified the hardware arrangement and updated the controller around the calibration and protection scheme you described.

Adaptive converter — EEPROM and thermistor revision

Arduino C++ · 3,490 lines · 146 KB source

Download the updated Arduino sketch

### 1\. Simplified hardware

There is now just one ACS712-05B output-current sensor. It measures aggregate current from both buck paths.

| Pin | Function                                    |
| --- | ------------------------------------------- |
| A0  | Output voltage                              |
| A1  | ACS712-05B aggregate output current         |
| A2  | Optional source-voltage measurement         |
| A3  | Shared heatsink 10k NTC                     |
| D2  | Hardware fault input                        |
| D3  | Auxiliary PWM, 80 kHz                       |
| D4  | Physical zero-current calibration interlock |
| D9  | Primary PWM, 20 kHz                         |

There is no auxiliary branch ACS712 and no input-current sensor.

### 2\. EEPROM calibration

The calibration process now has two stages.

Zero calibration

With the power stage disabled and output physically isolated, assert D4 and send:

```text
ZERO
```

The controller captures 128 ACS712 samples, checks offset, noise and stability, then records the zero reference.

10 Ω load calibration

Connect a measured 10.00 Ω resistor, rated at least 25 W (50 W recommended), release the interlock and send:

```text
LOAD
```

The controller regulates the output to 12 V using the primary path alone.

At 12 V, this produces approximately 1.2 A and 14.4 W of resistor dissipation.

When the output stabilises, measure it with a multimeter and send, for example:

```text
V=1200
```

The controller uses the known resistance and measured voltage to derive current, calibrate ACS712 gain and calibrate the voltage-divider scale.

The optional source-voltage divider can also be calibrated with `VIN=xxxx`.

EEPROM safeguards

* Two alternating calibration records.

* CRC16 validation.

* Sequence numbers.

* Commit marker written last.

* Read-back verification.

* Recovery from an interrupted write using the other valid record.

* Live zero-offset plausibility check on startup.

Normal startup uses the stored calibration. It does not repeat the load test. If both EEPROM records are invalid, or the live ACS712 zero check fails, PWM remains disabled until recalibration.

One detail: the calibration assumes 10.00 Ω. The resistor should therefore be measured; adjust `CAL_LOAD_RESISTANCE_MILLIOHM` if its actual resistance differs materially.

### 3\. Thermal regulation

The separate current sensors have been replaced with a single 10k B3950 NTC on the common heatsink.

![Automatic Fan Controller For An Active Heatsink | Full Electronics Project](https://images.openai.com/static-rsc-4/yYWG6ZZRpRF2NWP3aYnsQ3XhmeuL-FAEy_l7pAMUSGaeyY1ntTokcwG2tB1VqKHxoEh6fC7kPkfANGQgySpSnapQ0ztqPyNhD2BbSQvd6v453wa3ZXurC-W3w5nPWDgxeJrrkNi32-FDmrsZl6gGRV-oyQGN6q4TzG7IO_lz60t-as5bWfdzpx8HYN7gn5Sz?purpose=fullsize)

The initial thermal profile is:

At 95°C the controller latches an overtemperature fault.

This derates the output-current limit, which in turn reduces available output power. The limits are configurable starting values, not verified junction-temperature limits. The NTC needs good thermal contact with the common heatsink.

### 4\. Auxiliary operating envelope

I interpreted your unfinished duty threshold as 10%, with hysteresis.

| Operating condition                                      | Action                |
| -------------------------------------------------------- | --------------------- |
| Primary duty below 18%                                   | Permit overlap        |
| Primary duty above 22%                                   | Primary only          |
| Primary-equivalent duty below 10% and total current ≤1 A | Permit auxiliary-only |
| Primary-equivalent duty above 12%                        | Return to overlap     |
| Total current above 1.25 A                               | Return to overlap     |
| Total current above 1.5 A                                | Exit low-power mode   |

The primary-equivalent duty is model-estimated: it represents the duty the primary branch would need to deliver the requested current increment, even when the primary MOSFET is currently off.

This is important because using the actual primary PWM value would make auxiliary-only operation appear to have zero primary duty by definition.

These thresholds are configurable:

```cpp
#define AUX_ONLY_ENTER_BP          1000L
#define AUX_ONLY_EXIT_BP           1200L
#define AUX_ONLY_MAX_CURRENT_CENTI  100L
```

The auxiliary branch remains disabled by default until explicitly armed.

One qualification: without input-current measurement, the controller does not directly measure conversion efficiency. Its efficiency ranking is based on identified response and configured MOSFET loss assumptions. Actual efficiency improvement must be checked experimentally.

### 5\. Code verification

Strict host C++ compilation for the generic path and ATmega328P primary and dual-PWM configurations.

EEPROM round-trip, redundant-record recovery and CRC corruption tests.

ACS712 zero calibration, signed current conversion and gain tests.

Thermistor conversion, open/short detection and thermal derating tests.

Auxiliary operating-region transition and hysteresis tests.

UndefinedBehaviorSanitizer tests.

One important outstanding qualification: these are host-side tests using Arduino and AVR-register stubs. A genuine AVR build has not been performed. Consequently, the 32 KB Uno flash limit, 2 KB SRAM limit and worst-case execution time have not been verified. This is still a necessary step before treating the firmware as deployable.

Also, 80 kHz PWM does not make an AVR output pin a proper gate driver. Verify the MOSFET gate waveform, transition time and driver dissipation with an oscilloscope, especially for the low-inductance auxiliary branch. Independent fast hardware current limiting must directly inhibit each MOSFET; the ACS712 and thermistor are supervisory sensors, not substitutes for that protection.

The resulting hardware model is now considerably simpler: one aggregate current measurement, one shared thermal measurement, persistent calibration, and two independently controlled power paths whose operating regions deliberately overlap.
