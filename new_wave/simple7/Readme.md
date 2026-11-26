Worked for 46m 46s

I completed another calibration and whole-firmware audit. The calibration path is substantially more robust, but the audit uncovered a major hardware-target problem that prevents me from honestly calling this production-ready for an Arduino Uno.

### Updated firmware

Adaptive converter controller v8

Arduino C++ · EEPROM calibration · MPPT · dual power paths

Download firmware

Download the regression test harness

### 1\. Calibration improvements

I corrected several defects in the previous calibration implementation.

Zero calibration

The ACS712 zero calibration now uses 128 samples per sensor, with:

* Four discarded conversions after channel selection.

* Mean and variance calculation.

* Maximum sample-span validation.

* Zero-offset plausibility checks.

* Continuous verification of the physical isolation interlock.

* Rejection of ADC rail readings and unstable sensors.

10 Ω load calibration

The known resistor is now used to cross-check the sensor gains rather than simply trusting the DMM readings.

At the 12 V calibration point:

I_{\\mathrm{out}}=\\frac{12}{10}=1.2\\ \\mathrm A

The firmware checks the nominal ACS712 response against this expected current before accepting the derived gain.

It also validates input current, both voltage-divider scales and input/output power consistency.

I tightened the current cross-check tolerance to 12%, subject to physical validation of the actual sensor modules.

Calibration regulator

I found a subtle fixed-point bug: the original fractional duty increment was rounded to the PWM hardware resolution on every update. This could prevent the duty ramp from progressing at all.

It now uses a Q16 accumulator, preserving fractional increments until they produce an achievable PWM command.

The downward correction rate is four times the upward ramp rate.

Calibration safety

The revised procedure also:

* Services the optional current-limit potentiometers during calibration.

* Pauses if either operating-current limit is reached.

* Pauses if the heatsink becomes too hot.

* Stops on insufficient source-voltage headroom.

* Detects missing output-voltage response.

* Uses fresh, stable 64-sample DMM measurement windows.

* Disables both PWM outputs before EEPROM writes.

* Resets all adaptive models and filters after successful recalibration.

### 2\. EEPROM reliability

The calibration storage uses two alternating records.

Each record contains the sensor offsets and gains, voltage-divider scales, calibration measurements, configuration fingerprint, sequence number and CRC16.

The commit marker is written last, followed by read-back verification.

This protects the previously valid calibration if power is interrupted during a write.

A stored calibration is rejected if its configuration signature no longer matches the firmware. Earlier incompatible records therefore require recalibration.

### 3\. Whole-code audit findings

The following issues have been addressed:

| Finding                                              | Status |
| ---------------------------------------------------- | ------ |
| Fractional calibration duty stuck at zero            |        |
| Stale DMM measurement window                         |        |
| Incorrect resistor or sensor calibration accepted    |        |
| Current pots ignored during calibration              |        |
| Calibration EEPROM write with PWM active             |        |
| Serial diagnostic blocking during calibration        |        |
| Auxiliary PWM surviving an output inhibit            |        |
| Thermistor readings outside validated range accepted |        |
| Runtime output-voltage feedback failure              |        |
| False feedback-failure trips during current limiting |        |

The output-feedback failure detector now requires persistent failure despite adequate source headroom and sufficient current margin. It also handles `millis()` rollover correctly.

The MPPT, current-limit and thermal-control paths remain separate, with the hardware protection layer taking priority.

### 4\. The significant problem: Arduino Uno memory

This is the most important result of the audit.

I performed an AVR-target compilation of the full dual-path configuration using Clang's ATmega328P backend with `-Oz`.

AVR-target object measurements

### 60,804 B

Program text

### 1,069 B

Static SRAM (.bss)

### 50 B

Initialised data

### 929 B

Remaining theoretical SRAM for stack

Arduino Uno R3 limits

32 KB Flash · 2 KB SRAM · 1 KB EEPROM Arduino Docs +1

The measurement is an unlinked object compiled against mocked Arduino and EEPROM headers, not a final Arduino-core ELF. Link-time garbage collection can remove unused sections, but the live control loop itself occupies a very large section.

The result strongly indicates that the full feature set will not fit the Uno without substantial reduction.

SRAM is also concerning: the measured static allocation leaves less than 1 KB for stack, function frames, interrupt handling and temporary data. Actual maximum stack consumption has not been measured.

I would not attempt to deploy this version on an Uno until the memory issue is resolved.

### 5\. Verification

Host regression suite passed

The tests cover calibration parsing, duty-ramp accumulation, EEPROM recovery, ACS712 conversion, thermistor rejection, current-pot behaviour, no-response detection, MPPT response to irradiance changes and protective PWM shutdown.

Strict C++11 compilation with `-Werror`, GCC static analysis, AddressSanitizer and UndefinedBehaviorSanitizer also passed.

The full dual-path configuration compiled for the AVR instruction set with strict warnings enabled.

However, no genuine `avr-gcc` Arduino build or final link map was available. Physical closed-loop testing has not been performed.

### 6\. What still prevents production qualification

There are three important outstanding areas.

Execution-time validation: the nominal control interval is approximately 2 ms. Worst-case execution time, especially during RLS updates and candidate searches, has not been measured on the actual 16 MHz processor.

Watchdog and reset behaviour: the firmware does not yet configure a watchdog. A production implementation should recover from software lockups, while hardware gate pull-downs and independent protection circuits guarantee that a reset cannot leave the power stage energised.

Hardware validation: the independent input/output current comparators, branch peak-current comparators and output overvoltage shutdown must be physically verified. ACS712 sampling and the shared heatsink NTC are supervisory mechanisms, not fast protection.

My recommendation is to stop adding algorithms for the moment. The next engineering task should be a memory and execution-time redesign, not another adaptive feature.

The current candidate library, dual RLS models, MPPT and calibration machinery have collectively outgrown the Uno. Preserving the architecture while substantially reducing the implementation footprint—or moving the controller to a larger MCU—is now the prerequisite for genuine production qualification.
