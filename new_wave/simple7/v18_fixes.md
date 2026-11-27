Critical & High-Severity Issues
 * Calibration DMM command parser rejects valid 12.00 V readings (1200)
   In parseCalibrationDmmCommand, the V= token is bounded by CAL_TARGET_V_CENTI ± CAL_DMM_VOLTAGE_TOLERANCE_CENTI (1200 ± 20, i.e., 1180..1220), which is fine—but look at parseBoundedDecimal:
   if (parsed > 600U || (parsed == 600U && digit != 0U)) return false;

   parseBoundedDecimal caps parsing at 6000 (fine for VIN= up to 6000 and IIN= up to 450), but V= passes maximum = 1220, which is under 6000. Why does 1200 pass? Because 600 is a hardcoded magic number inside parseBoundedDecimal instead of checking against maximum during digit accumulation! Worse: if maximum were ever configured larger than 6000 (say for VIN=), parseBoundedDecimal silently rejects anything above 6000 even though maximum allows up to 6000U. Pass maximum into the overflow guard instead of hardcoding 600U.
 * Stale/uninitialized sourceRaw in calibrationWizardLoop when SOURCE_SENSE_ENABLED is 0
   Even though the #error guard currently blocks !SOURCE_SENSE_ENABLED, calibrationWizardLoop() calls accumulateCalibrationSample(voltageRaw, currentRaw, ...) passing 0U for sourceRaw when disabled, and calibrationRecordValid() checks record.calibrationRawMean[2] > 900U and reconstructedVin. Keep the conditional structure clean and explicit so no uncalibrated/dummy zero leaks into CalibrationRecord.
 * writeDutyBasisPoints disables timer output on compare == 0, then re-enables COM1A1/COM2B1 on next non-zero write — verify pin state transitions
   When primaryCompare == 0U, you clear COM1A1 and clear PORTB1 (digitalWrite(PWM_PIN, LOW) equivalent). When primaryCompare > 0U, you write OCR1A = primaryCompare and set COM1A1.
   * Check: On AVR Timer1 Fast PWM mode 14 (TOP in ICR1), COM1A1 takes effect at the next TOP or immediately depending on hardware buffering (OCR1A is double-buffered to TOP; TCCR1A bits are not double-buffered on all AVR timers). Toggling COM1A1 asynchronously while TCNT1 is mid-cycle can produce a runt/short pulse on the first cycle after re-enabling. Leaving COM1A1 enabled with OCR1A = 0 in non-inverting Fast PWM produces a clean minimum-width pulse (or zero on some modes), whereas clearing COM1A1 + clearing PORTB1 guarantees 0V DC hold without PWM edges—just verify on an oscilloscope that transitioning from COM1A1=0 back to COM1A1=1 does not glitch OC1A mid-period.
 * currentMeanQ16 and dutyMeanQ16 IIR filters use 1/4096 and 1/256 with integer division toward zero
   In controlUpdate:
   currentMeanQ16 += (currentCenti * 65536L - currentMeanQ16) / 4096L;

   In C++, / on negative numbers truncates toward zero, which creates a slight asymmetric bias near zero, and with dtUs around 2003 us (~500 Hz), / 4096L has a time constant of ~4096 ticks (~8.2 seconds). When transitioning out of calibration (resetAdaptiveRuntimeAfterCalibration()), currentMeanQ16 starts at 0L even if load current is already flowing, taking several seconds to converge. Initializing currentMeanQ16 and dutyMeanQ16 on the first sample (!filtersInitialized) avoids a multi-second ramp-up transient at startup/recal.
 * MPPT mpptOutputCurrentLimitCenti vs runtimeCurrentLimitCenti interaction with outputCurrentSettingCenti
   In updateMppt:
   const uint16_t maximum = (uint16_t)(
    ((uint16_t)runtimeInputCurrentLimitCenti *
     (uint16_t)MPPT_MAX_SOURCE_CURRENT_PERCENT) / 100U);

   runtimeInputCurrentLimitCenti is int32_t (up to 400L), cast to uint16_t (safe), multiplied by 85U (max 34,000, fits uint16_t), divided by 100U. However, mpptOutputCurrentLimitCenti is capped against runtimeCurrentLimitCenti (which is thermally derated). Notice that in controlOutputCurrentLimitCenti(), if mpptOutputCurrentLimitCenti was computed during an earlier tick before thermal derating reduced runtimeCurrentLimitCenti, it still takes min(runtimeCurrentLimitCenti, mpptOutputCurrentLimitCenti), which is safe. However, if sourceVoltageCenti drops below MPPT_MIN_SOURCE_VOLTAGE_CENTI, mpptOutputCurrentLimitCenti is set to 0U, immediately starving the output even if a backup/buffer cap exists—verify that behavior matches your intended PV source loss behavior.
Medium & Low-Severity Observations
 * Watchdog & loop() timing predictability
   loop() calls pollCalibrationSerial() (up to 12 bytes), serviceLimitPotentiometers() (1 analogRead every 50 ms), and then in controlUpdate() performs 5 analogRead() calls sequentially (~100–125 µs each on 16 MHz AVR = ~500–625 µs total). At a 2003 µs control period, ADC conversions consume ~25–30% of CPU time, leaving ~1.4 ms for fixed-point math and NLMS updates—comfortable, provided Serial isn't flooded and MAX_CONTROL_DT_US (5000 µs) is sized with margin against occasional timer/interrupt jitter. Enabling the hardware watchdog (<avr/wdt.h>) in setup() and kicking it in loop() is worth considering if unattended operation is planned.
 * EEPROM wear & write time in saveCalibrationToEeprom
   EEPROM.update() is used byte-by-byte across a ~50-byte CalibrationRecord (plus clearing commit first). Each changed byte on ATmega328P takes ~3.3 ms in EEPROM write hardware. A full calibration save can take tens to ~150 ms of blocking EEPROM write time. Since calibration only happens on explicit DMM commit commands (not in the control loop), this is acceptable, just don't call saveCalibrationToEeprom during active PWM regulation (you already zero PWM right before calling it).
 * Check withinPercentOf overflow risk
   return difference * 100L <= reference * (int32_t)tolerancePercent;

   difference and reference are in centi-units (at most a few thousand), and tolerancePercent is <= 100. difference * 100L is at most ~600,000 and reference * tolerancePercent is at most ~600,000—well within int32_t. Still, casting to int64_t or keeping bounds documented is good defensive practice if constants ever scale up.
 * Code structure & maintainability
   The sketch is remarkably thorough with #error compile-time arithmetic checks, Q8/Q14/Q16 fixed-point discipline, dual-slot CRC-checked EEPROM records, and explicit safety inhibits (enforcePowerStageInhibits, applyInputCurrentGuard). Splitting the large single-file sketch into logical headers/modules (e.g., cal_eeprom.h, control_loop.h, mppt_aux.h) once hardware bring-up stabilizes will make debugging and unit-testing on a host (non-AVR) much easier.
Summary Verdict
The safety architecture—compile-time bounds checks, CRC + sequence dual-slot EEPROM calibration, thermal derating, reverse-current and overvoltage/overcurrent trips, and explicit PWM inhibit gates—is well thought out for an ATmega328P prototype. Before energizing a real power stage, bench-test with a current-limited supply and dummy load, verify V=/VIN=/IIN= serial calibration parsing and OC1A/OC2B waveform transitions on a scope, and initialize the slow IIR means (currentMeanQ16, dutyMeanQ16) cleanly on filter startup.
