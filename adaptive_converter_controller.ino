#include <Arduino.h>
#if defined(__AVR_ATmega328P__)
#include <avr/io.h>
#endif

/*
 * Adaptive cascaded voltage/current controller
 *
 * Board assumptions:
 *   - Arduino Uno / ATmega328P (10-bit ADC, 5 V default ADC reference)
 *   - PWM output on D9 / OC1A
 *   - Voltage feedback on A0
 *   - Current feedback on A1
 *   - Positive PWM duty increases converter output/current.
 *
 * The ATmega328P backend configures Timer1 for 20 kHz Fast PWM.  This uses
 * Timer1 exclusively; do not combine this sketch with Servo or other code
 * that requires Timer1.  Other Arduino targets use analogWrite() as a
 * compatibility fallback; configure that board's PWM frequency explicitly
 * before using it with a switching power converter.
 *
 * IMPORTANT:
 *   The divider, shunt, amplifier gain, ADC reference, output limits, and
 *   controller gains below are commissioning defaults, not universal values.
 *   Verify them against calibrated instruments and the actual converter.
 *   Software sampling is not a substitute for a hardware current comparator,
 *   gate-driver shutdown, fuse, and independent over-voltage protection.
 */

// ---------------------------------------------------------------------------
// Hardware pins
// ---------------------------------------------------------------------------
#define PWM_PIN       9
#define VOLTAGE_PIN   A0
#define CURRENT_PIN   A1

// ---------------------------------------------------------------------------
// Calibrated measurement configuration
// ---------------------------------------------------------------------------
// Measure the actual ADC reference voltage; do not assume the board's 5 V rail
// is exactly 5000 mV.  The voltage divider ratio is (Rtop + Rbottom)/Rbottom.
#define ADC_REFERENCE_MV             5000UL
#define ADC_COUNTS                   1023UL
#define VOLTAGE_DIVIDER_NUM          4UL
#define VOLTAGE_DIVIDER_DEN          1UL

// Current = shunt voltage / shunt resistance.  CURRENT_SENSE_GAIN_NUM /
// CURRENT_SENSE_GAIN_DEN is the analog amplifier voltage gain.  For a direct
// 0.1-ohm shunt connection use 100 mOhm and gain 1.0.
#define CURRENT_SHUNT_MILLIOHM       100UL
#define CURRENT_SENSE_GAIN_NUM       1UL
#define CURRENT_SENSE_GAIN_DEN       1UL
#define CURRENT_ZERO_RAW             0U

// ---------------------------------------------------------------------------
// Setpoints and protection thresholds
// ---------------------------------------------------------------------------
#define TARGET_V_CENTI               1200L  // 12.00 V
#define CURRENT_LIMIT_CENTI          500L   // 5.00 A software regulation limit

// These are example thresholds for a 12 V / 5 A configuration.  Change them
// to the actual load, chemistry, converter and component limits before power
// is applied.  The hard current trip should be below the hardware safe limit.
#define OUTPUT_OVERVOLTAGE_CENTI     1300L  // 13.00 V example trip threshold
#define HARD_OVERCURRENT_CENTI       550L   // 5.50 A example trip threshold

// ADC readings at/near a rail usually indicate a wiring or sensor fault.
#define ADC_RAIL_TRIP_RAW            1018U

// ---------------------------------------------------------------------------
// Timing and actuator limits
// ---------------------------------------------------------------------------
#define CONTROL_PERIOD_US            1000UL  // nominal 1 kHz control update
#define MAX_CONTROL_DT_US            5000UL  // missed-deadline fail-safe
#define SOFTSTART_RATE_CV_PER_SEC    100L    // 1.00 V/s
#define MAX_DUTY_BASIS_POINTS        9000L   // 90.00%; 10000 = 100.00%

// ---------------------------------------------------------------------------
// Fixed-point controller gains
// ---------------------------------------------------------------------------
// Voltage loop:
//   OUTER_KP_Q8 = 64 -> 0.25 centiamp per centivolt
//   OUTER_KI_Q16 = 655 -> about 0.010 centiamp/(centivolt*second)
// Current loop:
//   INNER_KP_Q8 = 512 -> 2.0 duty basis-points per centiamp
//   INNER_KI_Q16 = 6554 -> about 0.100 duty basis-points/(centiamp*second)
// These are deliberately conservative starting values.  Tune on a current-
// limited bench supply with the actual power stage and output capacitance.
#define OUTER_KP_Q8                  64L
#define OUTER_KI_Q16                 655L
#define INNER_KP_Q8                  512L
#define INNER_KI_Q16                 6554L

// Pattern detector thresholds.  Voltage slope units are centivolts/second.
#define ERROR_DEADBAND_CENTI         5L
#define SLOW_RESPONSE_ERROR_CENTI    150L
#define SLOW_RESPONSE_SLOPE_CV_S     20L
#define FAST_APPROACH_SLOPE_CV_S     120L
#define APPROACH_BAND_CENTI          75L
#define TREND_WINDOW_US              100000UL
#define CROSSING_WINDOW_US           1000000UL
#define SLOW_RESPONSE_TIME_US        750000UL

#if ADC_REFERENCE_MV == 0 || ADC_COUNTS == 0
#error "ADC reference and ADC_COUNTS must be non-zero"
#endif
#if VOLTAGE_DIVIDER_DEN == 0 || CURRENT_SHUNT_MILLIOHM == 0 || CURRENT_SENSE_GAIN_NUM == 0
#error "Invalid voltage/current sensor scale: denominator, shunt and gain must be non-zero"
#endif
#if TARGET_V_CENTI <= 0 || CURRENT_LIMIT_CENTI <= 0
#error "Setpoint and current limit must be positive"
#endif
#if OUTPUT_OVERVOLTAGE_CENTI <= TARGET_V_CENTI
#error "OUTPUT_OVERVOLTAGE_CENTI must be greater than TARGET_V_CENTI"
#endif
#if HARD_OVERCURRENT_CENTI <= CURRENT_LIMIT_CENTI
#error "HARD_OVERCURRENT_CENTI must exceed the regulated current limit"
#endif

// ---------------------------------------------------------------------------
// Types and controller state
// ---------------------------------------------------------------------------
enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_CONTROL_DEADLINE,
  FAULT_ADC_RAIL,
  FAULT_OVERVOLTAGE,
  FAULT_OVERCURRENT
};

enum ControlCandidate : uint8_t {
  CANDIDATE_STARTUP = 0,
  CANDIDATE_NORMAL,
  CANDIDATE_TRACKING,
  CANDIDATE_DAMPED,
  CANDIDATE_CURRENT_LIMITED
};

struct CandidateGains {
  uint16_t outerKpScale;
  uint16_t outerKiScale;
  uint16_t innerKpScale;
  uint16_t innerKiScale;
};

// Finite, prequalified candidate bank.  The response evaluator selects among
// these profiles from observed output behaviour.  It never synthesizes gains.
static const CandidateGains CANDIDATE_GAINS[] = {
  // STARTUP:      slower response while the reference is ramping
  { 650U, 650U, 700U, 700U },
  // NORMAL:       nominal cascaded PI
  {1000U,1000U,1000U,1000U },
  // TRACKING:     modestly faster recovery from a persistent lag
  {1100U,1000U,1050U,1000U },
  // DAMPED:       reduce loop aggressiveness near overshoot/oscillation
  { 450U, 500U, 500U, 600U },
  // CURRENT_LIMITED: reduce controller aggressiveness near the current ceiling
  { 700U, 700U, 700U, 750U }
};

struct ResponseMetrics {
  int32_t voltageSlopeCvPerSec;
  uint8_t signCrossings;
  bool oscillating;
  bool approachingFast;
  bool overshooting;
  bool slowResponse;
};

static FaultCode faultCode = FAULT_NONE;
static ControlCandidate activeCandidate = CANDIDATE_STARTUP;

// Integrators are signed Q16 values.  Units are cA and duty basis-points.
static int32_t voltageIntegratorQ16 = 0;
static int32_t currentIntegratorQ16 = 0;

// Soft-start reference in Q16 centivolts.
static int32_t voltageReferenceQ16 = 0;

// Filtered measurements are Q8 values in centivolts and centiamps.
static int32_t voltageFilteredQ8 = 0;
static int32_t currentFilteredQ8 = 0;
static bool filtersInitialized = false;

static uint32_t lastControlUs = 0;
static uint8_t consecutiveRailSamples = 0;

// Response evaluator state.
static uint32_t trendWindowStartUs = 0;
static int32_t trendWindowStartVoltage = 0;
static bool trendInitialized = false;
static uint32_t crossingWindowStartUs = 0;
static uint32_t slowResponseStartUs = 0;
static bool slowResponseTiming = false;
static int8_t previousErrorSign = 0;
static uint8_t crossingCount = 0;
static ResponseMetrics responseMetrics = {0, 0, false, false, false, false};

// ---------------------------------------------------------------------------
// Small, bounded helpers
// ---------------------------------------------------------------------------
static int32_t clampI32(int32_t value, int32_t low, int32_t high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static int32_t absI32(int32_t value) {
  return (value < 0) ? -value : value;
}

static int32_t scaleGain(int32_t gain, uint16_t scalePermille) {
  return (int32_t)(((int64_t)gain * scalePermille + 500LL) / 1000LL);
}

static int32_t filterQ8(int32_t sample, int32_t &stateQ8) {
  const int32_t sampleQ8 = sample * 256L;
  stateQ8 += (sampleQ8 - stateQ8) / 8L; // alpha = 1/8, bounded integer EMA
  return stateQ8 / 256L;
}

// Round-to-nearest integer ADC conversions.  64-bit intermediates prevent
// overflow if the divider/amplifier calibration constants are changed.
static int32_t adcToVoltageCenti(uint16_t raw) {
  const uint64_t numerator = (uint64_t)raw * ADC_REFERENCE_MV *
                             VOLTAGE_DIVIDER_NUM * 100ULL;
  const uint64_t denominator = (uint64_t)ADC_COUNTS * 1000ULL *
                               VOLTAGE_DIVIDER_DEN;
  return (int32_t)((numerator + denominator / 2ULL) / denominator);
}

static int32_t adcToCurrentCenti(uint16_t raw) {
  const uint16_t correctedRaw = (raw > CURRENT_ZERO_RAW)
                              ? (uint16_t)(raw - CURRENT_ZERO_RAW) : 0U;
  const uint64_t numerator = (uint64_t)correctedRaw * ADC_REFERENCE_MV *
                             100ULL * CURRENT_SENSE_GAIN_DEN;
  const uint64_t denominator = (uint64_t)ADC_COUNTS *
                               CURRENT_SHUNT_MILLIOHM *
                               CURRENT_SENSE_GAIN_NUM;
  return (int32_t)((numerator + denominator / 2ULL) / denominator);
}

// ---------------------------------------------------------------------------
// PWM backend
// ---------------------------------------------------------------------------
static void configurePwm() {
  pinMode(PWM_PIN, OUTPUT);

#if defined(__AVR_ATmega328P__)
  // Timer1 Fast PWM mode 14, TOP=799, prescaler=1:
  // 16 MHz / (799 + 1) = 20 kHz on OC1A / D9.
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  ICR1 = 799U;
  OCR1A = 0U;
  TCCR1A = (uint8_t)(_BV(COM1A1) | _BV(WGM11));
  TCCR1B = (uint8_t)(_BV(WGM13) | _BV(WGM12) | _BV(CS10));
#else
  // Compatibility fallback only.  The board's default analogWrite frequency
  // may be far too low for a switching converter; configure a suitable PWM
  // peripheral/frequency for the target MCU before connecting a power stage.
  analogWrite(PWM_PIN, 0);
#endif
}

static void writeDutyBasisPoints(int32_t dutyBasisPoints) {
  const int32_t bounded = clampI32(dutyBasisPoints, 0L, MAX_DUTY_BASIS_POINTS);
#if defined(__AVR_ATmega328P__)
  if (bounded == 0L) {
    // Disconnect OC1A and force the GPIO low.  In Fast PWM, OCR1A=0 can
    // otherwise produce a very narrow pulse on some AVR timer configurations.
    TCCR1A &= (uint8_t)~_BV(COM1A1);
    PORTB &= (uint8_t)~_BV(PORTB1);
  } else {
    const uint16_t compare = (uint16_t)(((int64_t)bounded * 799LL + 5000LL) / 10000LL);
    OCR1A = compare;
    TCCR1A |= _BV(COM1A1);
  }
#else
  const uint8_t pwm = (uint8_t)(((int64_t)bounded * 255LL + 5000LL) / 10000LL);
  analogWrite(PWM_PIN, pwm);
#endif
}

static void latchFault(FaultCode reason) {
  if (faultCode == FAULT_NONE) {
    faultCode = reason;
  }
  // A fault is latched until reset.  No automatic restart is attempted.
  writeDutyBasisPoints(0L);
  voltageIntegratorQ16 = 0L;
  currentIntegratorQ16 = 0L;
}

// ---------------------------------------------------------------------------
// Output-response evaluator and preemptive candidate selector
// ---------------------------------------------------------------------------
static void updateResponseEvaluator(uint32_t nowUs,
                                    int32_t voltageCenti,
                                    int32_t voltageErrorCenti) {
  // Count meaningful voltage-error sign reversals in a one-second window.
  if ((uint32_t)(nowUs - crossingWindowStartUs) >= CROSSING_WINDOW_US) {
    crossingWindowStartUs = nowUs;
    crossingCount = 0U;
  }

  int8_t errorSign = 0;
  if (voltageErrorCenti > ERROR_DEADBAND_CENTI) {
    errorSign = 1;
  } else if (voltageErrorCenti < -ERROR_DEADBAND_CENTI) {
    errorSign = -1;
  }

  if (errorSign != 0) {
    if (previousErrorSign != 0 && errorSign != previousErrorSign &&
        crossingCount < 255U) {
      ++crossingCount;
    }
    previousErrorSign = errorSign;
  }

  // Estimate output slope over a 100 ms window rather than differentiating
  // adjacent ADC samples.  This is less sensitive to ADC quantisation/noise.
  if (!trendInitialized) {
    trendInitialized = true;
    trendWindowStartUs = nowUs;
    trendWindowStartVoltage = voltageCenti;
    responseMetrics.voltageSlopeCvPerSec = 0L;
  } else if ((uint32_t)(nowUs - trendWindowStartUs) >= TREND_WINDOW_US) {
    const uint32_t elapsedUs = nowUs - trendWindowStartUs;
    const int32_t voltageDelta = voltageCenti - trendWindowStartVoltage;
    responseMetrics.voltageSlopeCvPerSec =
        (int32_t)(((int64_t)voltageDelta * 1000000LL) / elapsedUs);
    trendWindowStartUs = nowUs;
    trendWindowStartVoltage = voltageCenti;
  }

  responseMetrics.signCrossings = crossingCount;
  responseMetrics.oscillating = (crossingCount >= 3U);
  responseMetrics.overshooting = (voltageErrorCenti < -ERROR_DEADBAND_CENTI);
  responseMetrics.approachingFast =
      (voltageErrorCenti > 0L) &&
      (voltageErrorCenti <= APPROACH_BAND_CENTI) &&
      (responseMetrics.voltageSlopeCvPerSec >= FAST_APPROACH_SLOPE_CV_S);

  const bool persistentLag =
      (voltageErrorCenti > SLOW_RESPONSE_ERROR_CENTI) &&
      (absI32(responseMetrics.voltageSlopeCvPerSec) < SLOW_RESPONSE_SLOPE_CV_S);

  if (persistentLag) {
    if (!slowResponseTiming) {
      slowResponseTiming = true;
      slowResponseStartUs = nowUs;
    }
    responseMetrics.slowResponse =
        ((uint32_t)(nowUs - slowResponseStartUs) >= SLOW_RESPONSE_TIME_US);
  } else {
    slowResponseTiming = false;
    responseMetrics.slowResponse = false;
  }

}

static ControlCandidate chooseCandidate(int32_t voltageReferenceCenti,
                                        int32_t voltageErrorCenti,
                                        int32_t currentCenti) {
  // Hard current limiting has priority over all performance objectives.
  if (currentCenti >= (CURRENT_LIMIT_CENTI * 9L) / 10L) {
    return CANDIDATE_CURRENT_LIMITED;
  }

  // Oscillation, overshoot, or a rapidly closing voltage gap selects damping
  // before the next PI update (preemptive profile selection).
  if (responseMetrics.oscillating || responseMetrics.overshooting ||
      responseMetrics.approachingFast) {
    return CANDIDATE_DAMPED;
  }

  // Keep the startup profile while the commanded voltage is still ramping.
  if (voltageReferenceCenti < TARGET_V_CENTI) {
    return CANDIDATE_STARTUP;
  }

  // A persistent lag, with no detected oscillation/current-limit condition,
  // justifies only a modest move to the predeclared tracking candidate.
  if (responseMetrics.slowResponse && voltageErrorCenti > 0L) {
    return CANDIDATE_TRACKING;
  }

  return CANDIDATE_NORMAL;
}

// ---------------------------------------------------------------------------
// Cascaded PI controllers with conditional-integration anti-windup
// ---------------------------------------------------------------------------
static int32_t calculateCurrentReference(int32_t voltageErrorCenti,
                                         uint32_t dtUs,
                                         const CandidateGains &gains) {
  const int32_t kpQ8 = scaleGain(OUTER_KP_Q8, gains.outerKpScale);
  const int32_t kiQ16 = scaleGain(OUTER_KI_Q16, gains.outerKiScale);

  // P term in Q16 centiamps.
  const int64_t proportionalQ16 =
      (int64_t)voltageErrorCenti * kpQ8 * 256LL;

  // Integral increment in Q16 centiamps.  dt is in microseconds.
  const int64_t integralDeltaQ16 =
      ((int64_t)voltageErrorCenti * kiQ16 * dtUs) / 1000000LL;

  const int64_t maxIntegratorQ16 = (int64_t)CURRENT_LIMIT_CENTI * 65536LL;
  int64_t proposedIntegratorQ16 = (int64_t)voltageIntegratorQ16 + integralDeltaQ16;
  if (proposedIntegratorQ16 < 0LL) proposedIntegratorQ16 = 0LL;
  if (proposedIntegratorQ16 > maxIntegratorQ16) proposedIntegratorQ16 = maxIntegratorQ16;

  const int64_t proposedOutputQ16 = proportionalQ16 + proposedIntegratorQ16;
  // Block integration only when it pushes farther into the upper saturation.
  // Negative error is still allowed to unwind a previously charged integrator
  // while the requested output remains clamped at zero.
  const bool pushingHigh = (proposedOutputQ16 > maxIntegratorQ16 && integralDeltaQ16 > 0LL);

  if (!pushingHigh) {
    voltageIntegratorQ16 = (int32_t)proposedIntegratorQ16;
  }

  int64_t outputQ16 = proportionalQ16 + voltageIntegratorQ16;
  if (outputQ16 < 0LL) outputQ16 = 0LL;
  if (outputQ16 > maxIntegratorQ16) outputQ16 = maxIntegratorQ16;

  return (int32_t)((outputQ16 + 32768LL) / 65536LL);
}

static int32_t calculateDutyBasisPoints(int32_t currentErrorCenti,
                                        uint32_t dtUs,
                                        const CandidateGains &gains) {
  const int32_t kpQ8 = scaleGain(INNER_KP_Q8, gains.innerKpScale);
  const int32_t kiQ16 = scaleGain(INNER_KI_Q16, gains.innerKiScale);

  // P term in Q16 duty basis-points.
  const int64_t proportionalQ16 =
      (int64_t)currentErrorCenti * kpQ8 * 256LL;

  // Integral increment in Q16 duty basis-points.
  const int64_t integralDeltaQ16 =
      ((int64_t)currentErrorCenti * kiQ16 * dtUs) / 1000000LL;

  const int64_t maxIntegratorQ16 =
      (int64_t)MAX_DUTY_BASIS_POINTS * 65536LL;
  int64_t proposedIntegratorQ16 = (int64_t)currentIntegratorQ16 + integralDeltaQ16;
  if (proposedIntegratorQ16 < 0LL) proposedIntegratorQ16 = 0LL;
  if (proposedIntegratorQ16 > maxIntegratorQ16) proposedIntegratorQ16 = maxIntegratorQ16;

  const int64_t proposedOutputQ16 = proportionalQ16 + proposedIntegratorQ16;
  // Permit the integrator to unwind after a current request is reduced, even
  // if the instantaneous actuator command is still clamped at zero.
  const bool pushingHigh =
      (proposedOutputQ16 > maxIntegratorQ16 && integralDeltaQ16 > 0LL);

  if (!pushingHigh) {
    currentIntegratorQ16 = (int32_t)proposedIntegratorQ16;
  }

  int64_t outputQ16 = proportionalQ16 + currentIntegratorQ16;
  if (outputQ16 < 0LL) outputQ16 = 0LL;
  if (outputQ16 > maxIntegratorQ16) outputQ16 = maxIntegratorQ16;

  return (int32_t)((outputQ16 + 32768LL) / 65536LL);
}

// ---------------------------------------------------------------------------
// Main control update
// ---------------------------------------------------------------------------
static void controlUpdate(uint32_t nowUs, uint32_t dtUs) {
  const uint16_t voltageRaw = (uint16_t)analogRead(VOLTAGE_PIN);
  const uint16_t currentRaw = (uint16_t)analogRead(CURRENT_PIN);

  // Rail detection uses raw ADC values so a failed/saturated sensor is not
  // hidden by the measurement filter.
  if (voltageRaw >= ADC_RAIL_TRIP_RAW || currentRaw >= ADC_RAIL_TRIP_RAW) {
    if (consecutiveRailSamples < 255U) ++consecutiveRailSamples;
  } else {
    consecutiveRailSamples = 0U;
  }
  if (consecutiveRailSamples >= 2U) {
    latchFault(FAULT_ADC_RAIL);
    return;
  }

  const int32_t voltageInstantCenti = adcToVoltageCenti(voltageRaw);
  const int32_t currentInstantCenti = adcToCurrentCenti(currentRaw);

  // Independent software trip checks use the unfiltered measurement.
  if (voltageInstantCenti >= OUTPUT_OVERVOLTAGE_CENTI) {
    latchFault(FAULT_OVERVOLTAGE);
    return;
  }
  if (currentInstantCenti >= HARD_OVERCURRENT_CENTI) {
    latchFault(FAULT_OVERCURRENT);
    return;
  }

  int32_t voltageCenti;
  int32_t currentCenti;
  if (!filtersInitialized) {
    voltageFilteredQ8 = voltageInstantCenti * 256L;
    currentFilteredQ8 = currentInstantCenti * 256L;
    filtersInitialized = true;
    voltageCenti = voltageInstantCenti;
    currentCenti = currentInstantCenti;
  } else {
    voltageCenti = filterQ8(voltageInstantCenti, voltageFilteredQ8);
    currentCenti = filterQ8(currentInstantCenti, currentFilteredQ8);
  }

  // Slew the reference using elapsed time, not loop count.
  const int32_t targetReferenceQ16 = TARGET_V_CENTI * 65536L;
  const int64_t referenceStepQ16 =
      ((int64_t)SOFTSTART_RATE_CV_PER_SEC * 65536LL * dtUs) / 1000000LL;
  if (voltageReferenceQ16 < targetReferenceQ16) {
    const int64_t next = (int64_t)voltageReferenceQ16 + referenceStepQ16;
    voltageReferenceQ16 = (next >= targetReferenceQ16)
                        ? targetReferenceQ16 : (int32_t)next;
  } else if (voltageReferenceQ16 > targetReferenceQ16) {
    const int64_t next = (int64_t)voltageReferenceQ16 - referenceStepQ16;
    voltageReferenceQ16 = (next <= targetReferenceQ16)
                        ? targetReferenceQ16 : (int32_t)next;
  }

  const int32_t voltageReferenceCenti = voltageReferenceQ16 / 65536L;
  const int32_t voltageErrorCenti = voltageReferenceCenti - voltageCenti;

  updateResponseEvaluator(nowUs, voltageCenti, voltageErrorCenti);
  activeCandidate = chooseCandidate(voltageReferenceCenti,
                                     voltageErrorCenti, currentCenti);
  const CandidateGains &gains = CANDIDATE_GAINS[(uint8_t)activeCandidate];

  int32_t currentReferenceCenti =
      calculateCurrentReference(voltageErrorCenti, dtUs, gains);

  // Predictive deceleration: when measured voltage is close to the reference
  // and still rising quickly, reduce the current request before overshoot.
  if (responseMetrics.approachingFast) {
    currentReferenceCenti = (int32_t)(((int64_t)currentReferenceCenti * 750LL) / 1000LL);
  }

  // Above the reference, stop requesting additional output current.  The
  // output can then decay through its load; this controller cannot sink power.
  if (responseMetrics.overshooting) {
    currentReferenceCenti = 0L;
  }

  currentReferenceCenti = clampI32(currentReferenceCenti,
                                   0L, CURRENT_LIMIT_CENTI);

  const int32_t currentErrorCenti = currentReferenceCenti - currentCenti;
  const int32_t dutyBasisPoints =
      calculateDutyBasisPoints(currentErrorCenti, dtUs, gains);

  writeDutyBasisPoints(dutyBasisPoints);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  analogReference(DEFAULT);
  configurePwm();
  writeDutyBasisPoints(0L);

  lastControlUs = micros();
  trendWindowStartUs = lastControlUs;
  crossingWindowStartUs = lastControlUs;
}

void loop() {
  if (faultCode != FAULT_NONE) {
    writeDutyBasisPoints(0L);
    return;
  }

  const uint32_t nowUs = micros();
  const uint32_t dtUs = nowUs - lastControlUs; // wrap-safe unsigned subtraction
  if (dtUs < CONTROL_PERIOD_US) {
    return;
  }
  lastControlUs = nowUs;

  if (dtUs > MAX_CONTROL_DT_US) {
    latchFault(FAULT_CONTROL_DEADLINE);
    return;
  }

  controlUpdate(nowUs, dtUs);
}
