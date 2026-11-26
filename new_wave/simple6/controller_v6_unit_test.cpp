#include <cassert>
#include <cstdio>
#include <cstdlib>
#include "Arduino.h"
#define A5 19
#define A6 20
#define AUX_PATH_ARMED 1
#define AUX_FAST_CURRENT_PROTECTION_PRESENT 1
#define __AVR_ATmega328P__ 1
#define F_CPU 16000000UL
#define OUTPUT_FAST_CURRENT_PROTECTION_PRESENT 1
#define PRIMARY_FAST_CURRENT_PROTECTION_PRESENT 1
#define INPUT_FAST_CURRENT_PROTECTION_PRESENT 1
#define POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED 1
#define OUTPUT_LIMIT_POT_PIN A5
#define INPUT_LIMIT_POT_PIN A6
#include <avr/io.h>
volatile uint8_t TCCR1A=0, TCCR1B=0, TCCR2A=0, TCCR2B=0, TCNT2=0;
volatile uint8_t OCR2A=0, OCR2B=0, PORTB=0, PORTD=0;
volatile uint16_t TCNT1=0, OCR1A=0, ICR1=0;
#include "/mnt/data/adaptive_converter_controller_v6.ino"

int main() {
  // New EEPROM record stores both ACS712 channels and both voltage scales.
  EEPROM = FakeEEPROM{};
  initialiseDefaultCalibrationScales();
  calibrationData = {};
  calibrationData.currentZeroRawQ8 = 512L * 256L;
  calibrationData.currentGainCentiPerCountQ16 = 173000L;
  calibrationData.sourceCurrentZeroRawQ8 = 511L * 256L;
  calibrationData.sourceCurrentGainCentiPerCountQ16 = 181000L;
  calibrationData.voltageCentiPerCountQ16 = (uint32_t)((1200ULL * 65536ULL) / 614ULL);
  calibrationData.sourceCentiPerCountQ16 = (uint32_t)((2400ULL * 65536ULL) / 800ULL);
  activeEepromSlot = -1;
  assert(saveCalibrationToEeprom());
  assert(currentSensorsCalibrated);
  CalibrationRecord saved = {};
  EEPROM.get(0, saved);
  assert(calibrationRecordValid(saved));
  assert(saved.version == EEPROM_CAL_VERSION);
  assert(saved.commit == EEPROM_CAL_COMMIT);

  // Reload all calibrated channels independently.
  primaryCurrentZeroRawQ8 = 0;
  currentGainCentiPerCountQ16 = 0;
  sourceCurrentZeroRawQ8 = 0;
  sourceCurrentGainCentiPerCountQ16 = 0;
  voltageCentiPerCountQ16 = 0;
  sourceCentiPerCountQ16 = 0;
  currentSensorsCalibrated = false;
  assert(loadCalibrationFromEeprom());
  assert(primaryCurrentZeroRawQ8 == saved.currentZeroRawQ8);
  assert(currentGainCentiPerCountQ16 == saved.currentGainCentiPerCountQ16);
  assert(sourceCurrentZeroRawQ8 == saved.sourceCurrentZeroRawQ8);
  assert(sourceCurrentGainCentiPerCountQ16 == saved.sourceCurrentGainCentiPerCountQ16);
  assert(voltageCentiPerCountQ16 == saved.voltageCentiPerCountQ16);
  assert(sourceCentiPerCountQ16 == saved.sourceCentiPerCountQ16);

  // Redundant slot and CRC recovery.
  const uint16_t firstSequence = saved.sequence;
  calibrationData.currentGainCentiPerCountQ16 = 174000L;
  calibrationData.sourceCurrentGainCentiPerCountQ16 = 182000L;
  assert(saveCalibrationToEeprom());
  CalibrationRecord second = {};
  EEPROM.get((int)sizeof(CalibrationRecord), second);
  assert(calibrationRecordValid(second));
  assert(second.sequence == (uint16_t)(firstSequence + 1U));
  EEPROM.update((int)sizeof(CalibrationRecord) + 4,
                (uint8_t)(EEPROM.read((int)sizeof(CalibrationRecord) + 4) ^ 0x55U));
  assert(loadCalibrationFromEeprom());
  assert(currentGainCentiPerCountQ16 == 173000L);
  assert(sourceCurrentGainCentiPerCountQ16 == 181000L);

  // Both gains are signed and independent. Nominal test signals give about
  // 1.19 A output and 0.74 A input for these calibrated coefficients.
  primaryCurrentZeroRawQ8 = 512L * 256L;
  currentGainCentiPerCountQ16 = 173000L;
  sourceCurrentZeroRawQ8 = 511L * 256L;
  sourceCurrentGainCentiPerCountQ16 = 181000L;
  const int32_t outputCurrent = adcToCurrentCenti(557U);
  const int32_t inputCurrent = adcToSourceCurrentCenti(539U);
  assert(outputCurrent >= 118L && outputCurrent <= 120L);
  assert(inputCurrent >= 77L && inputCurrent <= 79L);
  assert(adcToCurrentCenti(467U) < -100L);
  assert(adcToSourceCurrentCenti(483U) < -70L);

  // The documented default 5:1 source divider maps about 20.0 V to ADC 819;
  // the calibration wizard rejects readings above ADC 900 (~22.0 V).
  sourceCentiPerCountQ16 = nominalVoltageScaleQ16(SOURCE_DIVIDER_NUM,
                                                   SOURCE_DIVIDER_DEN);
  assert(adcToSourceCenti(819U) >= 1999L && adcToSourceCenti(819U) <= 2002L);
  assert(adcToSourceCenti(900U) >= 2199L && adcToSourceCenti(900U) <= 2202L);

  // Efficiency is measured from actual Pin and Pout, not estimated from load.
  // During a transient the readout remains valid, but the MPPT conversion
  // estimate must not learn from the transient point.
#if MPPT_ENABLED
  mpptEfficiencyEstimateValid = false;
  updateMeasuredEfficiency(2400L, 70L, 1200L, 120L, false);
  assert(latestEfficiencyValid);
  assert(!mpptEfficiencyEstimateValid);
  updateMeasuredEfficiency(2400L, 70L, 1200L, 120L, true);
  assert(mpptEfficiencyEstimateValid);
  assert(mpptEfficiencyEstimatePermille >= 786U && mpptEfficiencyEstimatePermille <= 788U);
#else
  updateMeasuredEfficiency(2400L, 70L, 1200L, 120L);
#endif
  assert(latestEfficiencyValid);
  assert(latestEfficiencyPermille >= 856U && latestEfficiencyPermille <= 858U);
  updateMeasuredEfficiency(2400L, 50L, 1200L, 120L); // apparent Pout > Pin
  assert(!latestEfficiencyValid);

  // Source and output limits remain independent. Refreshing source settings
  // never overwrites the output/thermal limit.
  outputCurrentSettingCenti = 400L;
  inputCurrentSettingCenti = 300L;
  runtimeCurrentLimitCenti = 275L; // output cap after thermal derating
  runtimeInputCurrentLimitCenti = 0L;
  refreshInputCurrentLimit();
  assert(runtimeInputCurrentLimitCenti == 300L);
  assert(runtimeCurrentLimitCenti == 275L);

  // Input guard has independent 90% enter / 97% hard / 85% release
  // thresholds and does not overwrite the output-current setting.
  inputCurrentGuardActive = false;
  commandedCurrentReferenceCenti = 200L;
  currentIntegratorQ16 = 12345L;
  appliedDutyBp = 4000L;
  auxiliaryLowPowerActive = true;
  int32_t guardedPrimaryDuty = 3900L;
  int32_t guardedAuxDuty = 1000L;
  applyInputCurrentGuard(269L, guardedPrimaryDuty, guardedAuxDuty);
  assert(!inputCurrentGuardActive);
  applyInputCurrentGuard(270L, guardedPrimaryDuty, guardedAuxDuty);
  assert(inputCurrentGuardActive);
  assert(commandedCurrentReferenceCenti == 199L);
  assert(runtimeCurrentLimitCenti == 275L);
  assert(currentIntegratorQ16 == 0L);
#if AUX_PATH_ARMED
  assert(!auxiliaryLowPowerActive);
#endif
  guardedPrimaryDuty = 3900L;
  guardedAuxDuty = 1000L;
  applyInputCurrentGuard(291L, guardedPrimaryDuty, guardedAuxDuty);
  assert(guardedPrimaryDuty == 3800L);
  assert(guardedAuxDuty == 0L);
  applyInputCurrentGuard(255L, guardedPrimaryDuty, guardedAuxDuty);
  assert(!inputCurrentGuardActive);

  // Pot calibration mapping is bounded and monotonic.
  assert(mapPotToCurrentLimit(0, OUTPUT_POT_MIN_CENTI,
                              OUTPUT_POT_MAX_CENTI) == OUTPUT_POT_MIN_CENTI);
  assert(mapPotToCurrentLimit(1023, OUTPUT_POT_MIN_CENTI,
                              OUTPUT_POT_MAX_CENTI) == OUTPUT_POT_MAX_CENTI);
  const int32_t mid = mapPotToCurrentLimit(512, 50L, 400L);
  assert(mid >= 224L && mid <= 226L);
  assert(mapPotToCurrentLimit(-100, 50L, 400L) == 50L);
  assert(mapPotToCurrentLimit(1200, 50L, 400L) == 400L);

  // Potentiometers update outside the control loop, one channel per 50 ms.
  outputCurrentSettingCenti = 400L;
  inputCurrentSettingCenti = 300L;
  for (uint8_t i = 0U; i < 25U; ++i) serviceLimitPotentiometers();
  assert(outputCurrentSettingCenti >= 224L && outputCurrentSettingCenti <= 226L);
  assert(inputCurrentSettingCenti == 300L);
  for (uint8_t i = 0U; i < 25U; ++i) serviceLimitPotentiometers();
  assert(inputCurrentSettingCenti >= 224L && inputCurrentSettingCenti <= 226L);

  // NTC derating uses the independently adjustable output current setting.
  faultCode = FAULT_NONE;
  outputCurrentSettingCenti = 350L;
  thermistorInitialized = false;
  assert(updateThermalLimit(204U));
  assert(runtimeCurrentLimitCenti == 350L);
  outputCurrentSettingCenti = 400L;
  thermistorInitialized = false;
  assert(updateThermalLimit(153U));
  assert(runtimeCurrentLimitCenti > 0L && runtimeCurrentLimitCenti < 400L);
  int32_t invalidThermistorTemperature = 0L;
  assert(!thermistorRawToCentiC(945U, invalidThermistorTemperature));
  assert(!thermistorRawToCentiC(1000U, invalidThermistorTemperature));

  // Source and output thresholds are distinct fault identities.
  assert(FAULT_OVERCURRENT != FAULT_INPUT_OVERCURRENT);

  // Auxiliary path transitions remain based on aggregate output-current model.
  faultCode = FAULT_NONE;
  auxiliaryLowPowerActive = true;
  currentGuardActive = false;
  auxiliaryPathMode = AUX_MODE_PRIMARY_ONLY;
  appliedDutyBp = 1900L;
  updateAuxiliaryPathMode(80L, true);
  assert(auxiliaryPathMode == AUX_MODE_PRIMARY_ONLY);
  appliedDutyBp = 1700L;
  updateAuxiliaryPathMode(80L, true);
  assert(auxiliaryPathMode == AUX_MODE_OVERLAP);
  appliedDutyBp = 900L;
  primaryEquivalentDutyDemandBp = 900L;
  updateAuxiliaryPathMode(80L, true);
  assert(auxiliaryPathMode == AUX_MODE_AUXILIARY_ONLY);
  primaryEquivalentDutyDemandBp = 1300L;
  updateAuxiliaryPathMode(80L, true);
  assert(auxiliaryPathMode == AUX_MODE_OVERLAP);
  updateAuxiliaryPathMode(151L, true);
  assert(auxiliaryPathMode == AUX_MODE_PRIMARY_ONLY);


#if MPPT_ENABLED
  // Calibration rejects physically impossible and grossly implausible
  // Pin/Pout DMM combinations; 20 V * 0.84 A -> 12 V * 1.20 A is 85.7%.
  assert(calibrationPowerPlausible(2000L, 84L, 1200L, 120L)); // matches documented 20 V / 0.84 A example
  assert(!calibrationPowerPlausible(1000L, 100L, 1200L, 120L));
  assert(!calibrationPowerPlausible(2400L, 20L, 1200L, 120L));
  assert(sourceCurrentStepDetected(20L));
  assert(sourceCurrentStepDetected(-20L));
  assert(!sourceCurrentStepDetected(19L));

  // MPPT must reject a sampling window containing a moving source point
  // instead of interpreting irradiance/source oscillation as hill-climb slope.
  runtimeInputCurrentLimitCenti = 300L;
  runtimeCurrentLimitCenti = 400L;
  mpptInitialised = true;
  mpptBaselineValid = true;
  mpptLimitActive = false;
  mpptInputCurrentTargetCenti = 50L;
  mpptPreviousSourceVoltageCenti = 2400L;
  mpptPreviousPowerProduct = 120000UL;
  mpptSettleUntilMs = 0UL;
  resetMpptSampleWindow();
  for (uint8_t i = 0U; i < MPPT_SAMPLE_COUNT; ++i) {
    const int32_t movingVoltage = (i & 1U) ? 2425L : 2400L;
    updateMppt(10000UL, movingVoltage, 100L, 1200L, 200L, 200L,
               false, false);
  }
  assert(mpptState == MPPT_STATE_SETTLING);
  assert(!mpptBaselineValid);
  assert(mpptInputCurrentTargetCenti == 50L);

  // MPPT output-power ceiling is independent from the actual output safety
  // limit. 2.4 kV-centi * 1.0 A-centi at 85% maps to 1.70 A output at 12 V.
  runtimeCurrentLimitCenti = 400L;
  runtimeInputCurrentLimitCenti = 300L;
  mpptInputCurrentTargetCenti = 100L;
  recalculateMpptOutputLimit(2400L, 1200L);
  assert(mpptLimitActive);
  assert(mpptOutputCurrentLimitCenti == 157L);
  assert(controlOutputCurrentLimitCenti() == 157L);
  runtimeCurrentLimitCenti = 150L;
  assert(controlOutputCurrentLimitCenti() == 150L); // thermal/load safety wins
  runtimeCurrentLimitCenti = 400L;

  // Finite-difference P-V slope sign. With source voltage falling and
  // power also falling, dP/dV is positive, so draw must be reduced to raise
  // source voltage. The implementation uses exact measured delta-P, not a
  // first-order V*dI + I*dV approximation.
  mpptInitialised = true;
  mpptBaselineValid = true;
  mpptLimitActive = false;
  mpptState = MPPT_STATE_TRACKING;
  mpptInputCurrentTargetCenti = 50L;
  mpptOutputCurrentLimitCenti = 400L;
  mpptPreviousSourceVoltageCenti = 2400L;
  mpptPreviousPowerProduct = 120000UL;
  mpptPowerSum = mpptVoltageSum = mpptCurrentSum = 0UL;
  mpptSampleCount = 0U;
  mpptSettleUntilMs = 0UL;
  mpptStepCenti = 10U;
  mpptDirection = 1;
  for (uint8_t i = 0U; i < MPPT_SAMPLE_COUNT; ++i)
    updateMppt(1000UL, 2380L, 45L, 1200L, 200L, 200L, false, false);
  assert(mpptDirection == -1);
  assert(mpptInputCurrentTargetCenti == 40L);
  assert(mpptBaselineValid); // normal perturbation preserves previous point

  // Opposite side of the P-V peak: source voltage falls while power rises,
  // hence dP/dV is negative and source draw should increase.
  mpptBaselineValid = true;
  mpptInputCurrentTargetCenti = 50L;
  mpptPreviousSourceVoltageCenti = 2400L;
  mpptPreviousPowerProduct = 120000UL;
  mpptPowerSum = mpptVoltageSum = mpptCurrentSum = 0UL;
  mpptSampleCount = 0U;
  mpptSettleUntilMs = 0UL;
  mpptStepCenti = 10U;
  mpptDirection = -1;
  for (uint8_t i = 0U; i < MPPT_SAMPLE_COUNT; ++i)
    updateMppt(2000UL, 2380L, 65L, 1200L, 200L, 200L, false, false);
  assert(mpptDirection == 1);
  assert(mpptInputCurrentTargetCenti == 60L);

  // Near the power peak, exact delta-P is inside the ACS712/divider noise
  // floor. The tracker must hold instead of walking continuously in one
  // direction, and schedule a delayed opposite-direction re-probe.
  mpptBaselineValid = true;
  mpptInputCurrentTargetCenti = 100L;
  mpptPreviousSourceVoltageCenti = 2640L;
  mpptPreviousPowerProduct = 237600UL;
  mpptPowerSum = mpptVoltageSum = mpptCurrentSum = 0UL;
  mpptSampleCount = 0U;
  mpptSettleUntilMs = 0UL;
  mpptStepCenti = 10U;
  mpptDirection = 1;
  for (uint8_t i = 0U; i < MPPT_SAMPLE_COUNT; ++i)
    updateMppt(2500UL, 2400L, 100L, 1200L, 200L, 200L, false, false);
  assert(mpptState == MPPT_STATE_TRACKING);
  assert(mpptInputCurrentTargetCenti == 100L);
  assert(mpptDirection == -1);
  assert(!mpptBaselineValid);
  assert(mpptSettleUntilMs == 2500UL + MPPT_RESCAN_INTERVAL_MS);

  // No-source and insufficient-buck-headroom conditions must suppress the
  // MPPT output-current ceiling rather than drive harder.
  mpptInitialised = true;
  mpptInputCurrentTargetCenti = 50L;
  updateMppt(3000UL, 0L, 0L, 1200L, 0L, 1200L, false, false);
  assert(mpptState == MPPT_STATE_NO_SOURCE);
  assert(mpptLimitActive && mpptOutputCurrentLimitCenti == 0L);
  mpptInputCurrentTargetCenti = 20L;
  mpptStepCenti = 10U;
  mpptSettleUntilMs = 0UL;
  updateMppt(4000UL, 1300L, 20L, 1200L, 50L, 200L, false, false);
  assert(mpptState == MPPT_STATE_HEADROOM_RECOVERY);
  assert(mpptInputCurrentTargetCenti == MPPT_MIN_INPUT_CURRENT_CENTI);
  updateMppt(4300UL, 1300L, 10L, 1200L, 50L, 200L, false, false);
  assert(mpptState == MPPT_STATE_NO_HEADROOM);
  assert(mpptOutputCurrentLimitCenti == 0L);

  // A low source voltage with sustained current indicates a collapsed source,
  // not a disconnected source. The controller must unload it stepwise; a
  // near-zero source current must instead reset the ceiling for reconnection.
  mpptInputCurrentTargetCenti = 100L;
  mpptStepCenti = 10U;
  mpptSettleUntilMs = 0UL;
  updateMppt(5000UL, 400L, 100L, 1200L, 50L, 200L, false, false);
  assert(mpptState == MPPT_STATE_SOURCE_RECOVERY);
  assert(mpptInputCurrentTargetCenti == 90L);
  assert(mpptLimitActive && mpptOutputCurrentLimitCenti == 0L);
  updateMppt(5300UL, 400L, 100L, 1200L, 50L, 200L, false, false);
  assert(mpptInputCurrentTargetCenti == 80L);
  updateMppt(5400UL, 0L, 0L, 1200L, 0L, 1200L, false, false);
  assert(mpptState == MPPT_STATE_NO_SOURCE);
  assert(mpptInputCurrentTargetCenti == MPPT_MIN_INPUT_CURRENT_CENTI);

  // MPPT-originated source response is known and should be ignored only
  // inside the finite command-response window, not indefinitely.
  mpptInitialised = true;
  mpptCommandResponseUntilMs = 100UL;
  assert(mpptCommandResponseActive(99UL));
  assert(!mpptCommandResponseActive(100UL));

  // Algorithm-level MPPT regression against a synthetic linear PV I-V map:
  // Voc=48 V, Isc=2 A, analytic MPP=24 V / 1 A. Requested source current
  // is applied instantaneously and output voltage is fixed at 12 V. This test
  // intentionally omits converter/current-loop/load dynamics, ADC noise,
  // PWM quantisation and gate-level effects; it is not closed-loop HIL proof.
  runtimeInputCurrentLimitCenti = 300L;
  runtimeCurrentLimitCenti = 400L;
  mpptInitialised = true;
  mpptBaselineValid = false;
  mpptLimitActive = false;
  mpptInputCurrentTargetCenti = 50L;
  mpptStepCenti = MPPT_INITIAL_STEP_CENTI;
  mpptDirection = 1;
  mpptSettleUntilMs = 0UL;
  mpptPowerSum = mpptVoltageSum = mpptCurrentSum = 0UL;
  mpptSampleCount = 0U;
  int32_t priorSimulatedVoltage = 0L;
  int32_t priorSimulatedCurrent = 0L;
  bool simulatedPointValid = false;
  for (uint32_t t = 0UL; t < 60000UL; t += 2UL) {
    const int32_t simulatedShortCircuit = (t < 30000UL) ? 200L : 240L;
    const int32_t requested = mpptInputCurrentTargetCenti;
    int32_t simulatedCurrent = requested;
    if (simulatedCurrent > simulatedShortCircuit)
      simulatedCurrent = simulatedShortCircuit;
    const int32_t simulatedVoltage =
        (simulatedCurrent >= simulatedShortCircuit) ? 0 :
        (int32_t)((4800L * (simulatedShortCircuit - simulatedCurrent)) /
                 simulatedShortCircuit);
    const bool simulatedIrradianceTransient =
        (t >= 30000UL && t < 30040UL);
    const bool expectedActuatorResponse = mpptCommandResponseActive(t);
    const bool simulatedMeasuredTransient = simulatedPointValid &&
        !expectedActuatorResponse &&
        (absI32(simulatedVoltage - priorSimulatedVoltage) >=
             SOURCE_STEP_TRANSIENT_CENTI ||
         sourceCurrentStepDetected(simulatedCurrent - priorSimulatedCurrent));
    updateMppt(t, simulatedVoltage, simulatedCurrent, 1200L, 100L,
               200L, simulatedIrradianceTransient ||
               simulatedMeasuredTransient, false);
    priorSimulatedVoltage = simulatedVoltage;
    priorSimulatedCurrent = simulatedCurrent;
    simulatedPointValid = true;
  }
  std::printf("after irradiance step target=%ld cA state=%u step=%u dir=%d\n",
              (long)mpptInputCurrentTargetCenti, (unsigned)mpptState,
              (unsigned)mpptStepCenti, (int)mpptDirection);
  assert(mpptInputCurrentTargetCenti >= 105L &&
         mpptInputCurrentTargetCenti <= 135L);
#endif

  std::puts("controller v6 unit tests passed");
  return 0;
}
