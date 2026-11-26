#include <Arduino.h>
#include <stdint.h>
#if defined(__AVR__)
#include <avr/io.h>
#include <avr/pgmspace.h>
#elif !defined(PROGMEM)
#define PROGMEM
#endif

/*
 * Online-identifying, model-selected cascaded converter controller.
 *
 * Uno baseline: D9/OC1A PWM, A0 output voltage, A1 current/inductor current.
 * Optional A2 source-voltage feedback is controlled by SOURCE_SENSE_ENABLED.
 *
 * Identified incremental dynamics (local discrete-time models):
 *   dI[k] = aI*dI[k-1] + bD*(D[k-1] - Dmean[k-1])
 *            + cI*(D[k-1]*dVin[k] - dVout[k]) + wI
 *   dV[k] = aV*dV[k-1] + bI*(I[k-1] - Imean) + cV*dVin[k-1] + wV
 *
 * dX denotes a sample-to-sample increment. This incremental current model
 * follows the local incremental inductor-voltage balance. bD is the local
 * duty-to-current gain around a slowly tracked operating duty (not a duty
 * difference); the second input regressor separates D*delta(Vin) - delta(Vout).
 * If source sensing is disabled, Vin changes are unobservable and remain in
 * the net disturbance; the local duty gain is identified from applied-duty
 * deviations, including natural changes and the gated PRBS probe.
 *
 * Two RLS models run in parallel for each stage: fast forgetting for regime
 * changes and slow forgetting for stable operation. A finite flash-resident
 * bank of Lyapunov-style functions is searched according to the active
 * resolution strategy; the selected function is reevaluated every control step.
 * The evaluator uses predicted next-step voltage/current errors, current
 * barrier cost, duty movement and model uncertainty. It prefers a decrease
 * in a discrete Lyapunov proxy and is therefore based on predicted output,
 * not merely a symptom-to-gain lookup.
 *
 * The PI controller is only the conservative identification bootstrap and
 * fallback. All candidate functions are bounded; that is not a formal proof
 * of closed-loop stability. Bench validation with the actual power stage is
 * mandatory. Software trips do not replace hardware protection.
 */

// -------------------------- Hardware/calibration --------------------------
#define PWM_PIN                         9
#define VOLTAGE_PIN                     A0
#define CURRENT_PIN                     A1
#define SOURCE_SENSE_ENABLED            0
#define SOURCE_VOLTAGE_PIN              A2

#define ADC_REFERENCE_MV                5000UL
#define ADC_COUNTS                      1023UL
#define VOLTAGE_DIVIDER_NUM             4UL
#define VOLTAGE_DIVIDER_DEN             1UL
#define CURRENT_SHUNT_MILLIOHM          100UL
#define CURRENT_SENSE_GAIN_NUM          1UL
#define CURRENT_SENSE_GAIN_DEN          1UL
#define CURRENT_ZERO_RAW                0U
#define SOURCE_DIVIDER_NUM              5UL
#define SOURCE_DIVIDER_DEN              1UL

// -------------------------- Setpoints/protection --------------------------
#define TARGET_V_CENTI                  1200L
#define CURRENT_LIMIT_CENTI             500L
#define OUTPUT_OVERVOLTAGE_CENTI        1300L
#define HARD_OVERCURRENT_CENTI          550L
#define ADC_RAIL_TRIP_RAW               1018U

// -------------------------- Timing/actuation ------------------------------
// 500 Hz control gives the Uno more arithmetic headroom for fixed-point RLS.
#define CONTROL_PERIOD_US               2000UL
#define MAX_CONTROL_DT_US               5000UL
#define SOFTSTART_RATE_CV_PER_SEC       100L
#define MAX_DUTY_BASIS_POINTS            9000L
#define MIN_DUTY_BASIS_POINTS            0L

// Conservative PI bootstrap; used until model confidence is sufficient.
// Outer Kp units cA/cV; inner Kp units duty basis-points/cA.
#define BOOT_OUTER_KP_Q8                128L
#define BOOT_OUTER_KI_Q16               655L
#define BOOT_INNER_KP_Q8                256L
#define BOOT_INNER_KI_Q16               3277L
#define BOOT_DUTY_SLEW_BP               25L
#define BOOT_CURRENT_SLEW_CENTI         25L

// -------------------------- Fixed-point identification -------------------
#define MODEL_ONE_Q14                   16384L
#define MODEL_V_SCALE_CENTI             20L
#define MODEL_I_SCALE_CENTI             100L
#define MODEL_CURRENT_DEVIATION_CENTI   250L
#define MODEL_DUTY_SCALE_BP             500L
#define DUTY_MEAN_TAU_SAMPLES            256L
#define MODEL_SOURCE_SCALE_CENTI        50L
#define MODEL_SOURCE_OUTPUT_SCALE_CENTI  100L
#define MODEL_FEATURE_COUNT              3U
#define RLS_P_INITIAL_Q12               65536L
#define RLS_P_MAX_Q12                   262144L
#define RLS_P_MIN_Q12                   16L
#define RLS_LAMBDA_FAST_Q12             3994L
#define RLS_LAMBDA_SLOW_Q12             4076L
#define RLS_FAST_RECIP_Q15              33605L
#define RLS_SLOW_RECIP_Q15              32929L
#define SLOW_RLS_UPDATE_DIVIDER          8U
#define RLS_MIN_EXCITATION_Q14          96L
#define RLS_VALID_P_Q12                 32768L
#define RLS_MIN_EXCITATIONS              24U
#define RLS_MIN_GAIN_Q14                64L
#define RLS_RESIDUAL_TRANSIENT_Q14      1800L
#define RLS_RESIDUAL_BAD_Q14            4200L
#define RLS_PARAMETER_DIVERGENCE_Q14    2600L
#define RLS_DISTURBANCE_LIMIT_Q14       8192L

// -------------------------- Pattern/evaluation thresholds ----------------
#define VOLTAGE_ERROR_DEADBAND_CENTI    5L
#define CURRENT_STEP_TRANSIENT_CENTI    18L
#define SOURCE_STEP_TRANSIENT_CENTI     20L
#define VOLTAGE_SLOPE_WINDOW_US         50000UL
#define CROSSING_WINDOW_US              1000000UL
#define OSCILLATION_CROSSINGS           3U
#define FAST_APPROACH_SLOPE_CV_S        120L
#define APPROACH_BAND_CENTI             75L
#define CURRENT_GUARD_START_PERCENT     80L
#define CURRENT_GUARD_HARD_PERCENT      92L
#define CURRENT_GUARD_RELEASE_PERCENT   75L
#define CURRENT_GUARD_ZERO_PERCENT      98L
#define CURRENT_GUARD_BRAKE_STEP_BP     200L
#define MAX_PREDICTED_DV_CENTI          100L
#define MAX_PREDICTED_DI_CENTI          150L
#define COST_CURRENT_WEIGHT             4LL
#define CANDIDATE_HISTORY_COST_WEIGHT   1000LL

// Small zero-mean excitation, enabled only near regulation and away from limits.
#define IDENTIFICATION_PROBE_ENABLED    1
#define IDENTIFICATION_PROBE_BP         30L
#define IDENTIFICATION_PROBE_SLEW_BP    40L
#define IDENTIFICATION_PROBE_HOLD_TICKS 8U
#define IDENTIFICATION_PROBE_ERROR_BAND_CENTI 50L
#define IDENTIFICATION_PROBE_MAX_CURRENT_PERCENT 70L
#define IDENTIFICATION_PROBE_VOLTAGE_MARGIN_CENTI 100L

#if ADC_REFERENCE_MV == 0 || ADC_COUNTS == 0
#error "ADC reference and ADC_COUNTS must be non-zero"
#endif
#if VOLTAGE_DIVIDER_DEN == 0 || CURRENT_SHUNT_MILLIOHM == 0 || CURRENT_SENSE_GAIN_NUM == 0
#error "Invalid voltage/current sensor calibration"
#endif
#if SOURCE_DIVIDER_DEN == 0
#error "Invalid source-voltage divider calibration"
#endif
#if TARGET_V_CENTI <= 0 || CURRENT_LIMIT_CENTI <= 0
#error "Setpoint and current limit must be positive"
#endif
#if OUTPUT_OVERVOLTAGE_CENTI <= TARGET_V_CENTI
#error "Over-voltage trip must exceed target voltage"
#endif
#if HARD_OVERCURRENT_CENTI <= CURRENT_LIMIT_CENTI
#error "Hard overcurrent trip must exceed regulated current limit"
#endif

// -------------------------------- Types ------------------------------------
enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_CONTROL_DEADLINE,
  FAULT_ADC_RAIL,
  FAULT_OVERVOLTAGE,
  FAULT_OVERCURRENT
};

enum OperatingPattern : uint8_t {
  PATTERN_BOOTSTRAP = 0,
  PATTERN_STEADY,
  PATTERN_LOAD_TRANSIENT,
  PATTERN_SOURCE_TRANSIENT,
  PATTERN_UNATTRIBUTED_TRANSIENT,
  PATTERN_ABOVE_REFERENCE,
  PATTERN_MODEL_DRIFT,
  PATTERN_NONLINEAR,
  PATTERN_OSCILLATORY,
  PATTERN_FAST_APPROACH,
  PATTERN_CURRENT_BOUND,
  PATTERN_MODEL_UNCERTAIN
};

enum CandidateFlag : uint8_t {
  CANDIDATE_FAST          = 0x01,
  CANDIDATE_DAMPED        = 0x02,
  CANDIDATE_DISTURBANCE   = 0x04,
  CANDIDATE_ROBUST        = 0x08,
  CANDIDATE_CURRENT_GUARD = 0x10
};

enum ResolutionStrategy : uint8_t {
  RESOLUTION_BOOTSTRAP = 0,
  RESOLUTION_IDENTIFY,
  RESOLUTION_FAST_TRANSIENT,
  RESOLUTION_DAMPED,
  RESOLUTION_CURRENT_GUARD,
  RESOLUTION_STEADY
};

enum CandidatePatternClass : uint8_t {
  CANDIDATE_CLASS_STEADY = 0,
  CANDIDATE_CLASS_TRANSIENT,
  CANDIDATE_CLASS_DAMPED,
  CANDIDATE_CLASS_CURRENT_GUARD,
  CANDIDATE_CLASS_UNCERTAIN,
  CANDIDATE_PATTERN_CLASS_COUNT
};

struct ResolutionProfile {
  uint8_t candidateMask;
  uint8_t fullSearchPeriod;
  uint8_t allowProbe;
};

struct FunctionCandidate {
  uint16_t alphaVoltageQ14;
  uint16_t alphaCurrentQ14;
  uint16_t dampingVoltageQ14;
  uint16_t dampingCurrentQ14;
  uint16_t disturbanceWeightPermille;
  uint16_t dutySlewBp;
  uint16_t currentSlewCenti;
  uint8_t flags;
};

// Stored in program flash on AVR. All contraction/damping coefficients are Q14.
static const FunctionCandidate FUNCTION_CANDIDATES[] PROGMEM = {
  // alphaV  alphaI  dampV  dampI  disturbance  dDuty  dCurrent  flags
  { 1229U, 4915U,    0U,    819U,       1000U,    80U,      50U, CANDIDATE_FAST },
  {  819U, 3277U,  819U,   1638U,       1000U,    40U,      30U, 0U },
  {  410U, 1638U, 4915U,   3277U,        750U,    20U,      15U, CANDIDATE_DAMPED },
  {  655U, 2458U, 1638U,   1638U,       1000U,    35U,      25U, CANDIDATE_DISTURBANCE },
  {  246U,  983U, 6554U,   4915U,        500U,    10U,      10U, CANDIDATE_ROBUST },
  {  328U, 1311U, 3277U,   3277U,        500U,    80U,      50U, CANDIDATE_CURRENT_GUARD }
};
#define FUNCTION_CANDIDATE_COUNT ((uint8_t)(sizeof(FUNCTION_CANDIDATES) / sizeof(FUNCTION_CANDIDATES[0])))

// Bit positions correspond to FUNCTION_CANDIDATES indices 0..5.
static const ResolutionProfile RESOLUTION_PROFILES[] PROGMEM = {
  // mask   bank-search period   allow probing
  { 0x3EU,       1U,                0U }, // bootstrap
  { 0x3AU,       8U,                1U }, // information gathering / uncertain model
  { 0x38U,       2U,                0U }, // load/source/model transient
  { 0x34U,       2U,                0U }, // oscillation / rapid approach
  { 0x20U,       1U,                0U }, // current guard only
  { 0x12U,      16U,                0U }  // steady: nominal or robust
};

struct RlsModel {
  int16_t thetaQ14[MODEL_FEATURE_COUNT];
  int32_t covarianceQ12[MODEL_FEATURE_COUNT][MODEL_FEATURE_COUNT];
  int16_t disturbanceQ14;
  int16_t residualAbsQ14;
  uint16_t excitationCount;
  uint16_t inputExcitationCount;
};

struct DualModel {
  RlsModel fast;
  RlsModel slow;
};

struct CandidateResult {
  int64_t cost;
  int32_t currentReferenceCenti;
  int32_t dutyBasisPoints;
  int32_t predictedVoltageDeltaCenti;
  int32_t predictedNextVoltageDeltaCenti;
  int32_t predictedCurrentDeltaCenti;
  uint8_t candidateIndex;
  bool valid;
};

struct CandidateOutcome {
  uint16_t predictionErrorQ8;
  uint8_t observations;
};

// -------------------------- Global state ----------------------------------
static FaultCode faultCode = FAULT_NONE;
static OperatingPattern activePattern = PATTERN_BOOTSTRAP;
static uint8_t activeCandidateIndex = 1U;
static ResolutionStrategy activeResolutionStrategy = RESOLUTION_BOOTSTRAP;
static uint8_t candidateSearchCountdown = 0U;
static uint8_t previousResolutionStrategy = 255U;
static bool selectedCandidateAvailable = false;
static uint8_t slowRlsPhase = 0U;
static int32_t dutyMeanQ16 = 0L;
static int32_t dutyMeanBp = 0L;
static int32_t previousDutyCenteredBp = 0L;
static uint8_t probeDivider = 0U;
static uint8_t probeLfsr = 0x5DU;
static int8_t probeSign = 1;
static bool identificationProbeActive = false;
static bool currentGuardActive = false;
static CandidateOutcome candidateOutcomes[FUNCTION_CANDIDATE_COUNT]
                                          [CANDIDATE_PATTERN_CLASS_COUNT] = {};
static bool candidateOutcomePending = false;
static uint8_t pendingCandidateIndex = 0U;
static CandidatePatternClass pendingCandidateClass = CANDIDATE_CLASS_STEADY;
static int32_t pendingPredictedVoltageStepCenti = 0L;
static int32_t pendingPredictedCurrentStepCenti = 0L;

static int32_t voltageIntegratorQ16 = 0L;
static int32_t currentIntegratorQ16 = 0L;
static int32_t voltageReferenceQ16 = 0L;
static int32_t voltageFilteredQ8 = 0L;
static int32_t currentFilteredQ8 = 0L;
static int32_t currentMeanQ16 = 0L;
static int32_t currentMeanCenti = 0L;
#if SOURCE_SENSE_ENABLED
static int32_t sourceFilteredQ8 = 0L;
#endif
static bool filtersInitialized = false;

static int32_t appliedDutyBp = 0L;
static int32_t commandedCurrentReferenceCenti = 0L;
static int32_t previousDutyLevelBp = 0L;
static int32_t previousDeltaVoltageCenti = 0L;
static int32_t previousDeltaCurrentCenti = 0L;
static int32_t previousVoltageCenti = 0L;
static int32_t previousCurrentCenti = 0L;
static int32_t previousCurrentCenteredCenti = 0L;
static int32_t previousSourceCenti = 0L;
static bool previousMeasurementsValid = false;

// Equivalent load-current estimate derived from the voltage-model innovation.
static int32_t estimatedLoadCurrentCenti = 0L;
static int32_t estimatedLoadCurrentQ8 = 0L;
static int32_t loadEstimateDeltaCenti = 0L;
static bool loadEstimateInitialized = false;
static bool loadEstimateValid = false;

static DualModel currentModel;
static DualModel voltageModel;
static uint32_t lastControlUs = 0UL;
static uint8_t consecutiveRailSamples = 0U;

static uint32_t trendWindowStartUs = 0UL;
static int32_t trendWindowStartVoltage = 0L;
static int32_t voltageSlopeCentiPerSec = 0L;
static bool trendInitialized = false;
static uint32_t crossingWindowStartUs = 0UL;
static uint8_t crossingCount = 0U;
static uint8_t transientHoldCounter = 0U;
static uint8_t nonlinearResidualCounter = 0U;
static int8_t previousErrorSign = 0;

// -------------------------- Bounded arithmetic ---------------------------
static int32_t clampI32(int32_t value, int32_t low, int32_t high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static int16_t clampI16(int32_t value, int16_t low, int16_t high) {
  if (value < (int32_t)low) return low;
  if (value > (int32_t)high) return high;
  return (int16_t)value;
}

static int32_t absI32(int32_t value) {
  return (value < 0L) ? -value : value;
}

static int16_t toQ14(int32_t value, int32_t scale) {
  if (scale <= 0L) return 0;
  int64_t q = ((int64_t)value * MODEL_ONE_Q14) / scale;
  if (q > MODEL_ONE_Q14) q = MODEL_ONE_Q14;
  if (q < -MODEL_ONE_Q14) q = -MODEL_ONE_Q14;
  return (int16_t)q;
}

static int32_t fromQ14(int32_t valueQ14, int32_t scale) {
  return (int32_t)(((int64_t)valueQ14 * scale) / MODEL_ONE_Q14);
}

static int32_t filterQ8(int32_t sample, int32_t &stateQ8) {
  const int32_t sampleQ8 = sample * 256L;
  stateQ8 = (int32_t)((int64_t)stateQ8 +
      (((int64_t)sampleQ8 - stateQ8) / 2LL));
  return stateQ8 / 256L;
}

static int32_t limitSlew(int32_t desired, int32_t current, int32_t maxStep) {
  return clampI32(desired, current - maxStep, current + maxStep);
}

// -------------------------- ADC conversion -------------------------------
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

#if SOURCE_SENSE_ENABLED
static int32_t adcToSourceCenti(uint16_t raw) {
  const uint64_t numerator = (uint64_t)raw * ADC_REFERENCE_MV *
                             SOURCE_DIVIDER_NUM * 100ULL;
  const uint64_t denominator = (uint64_t)ADC_COUNTS * 1000ULL *
                               SOURCE_DIVIDER_DEN;
  return (int32_t)((numerator + denominator / 2ULL) / denominator);
}
#endif

static int16_t sourceOutputIncrementQ14(int32_t dutyLevelBp,
                                        int32_t deltaSourceCenti,
                                        int32_t deltaOutputCenti) {
#if SOURCE_SENSE_ENABLED
  const int32_t sourceTermCenti = (int32_t)(
      ((int64_t)dutyLevelBp * deltaSourceCenti) / 10000LL);
#else
  (void)dutyLevelBp;
  (void)deltaSourceCenti;
  const int32_t sourceTermCenti = 0L;
#endif
  return toQ14(sourceTermCenti - deltaOutputCenti,
               MODEL_SOURCE_OUTPUT_SCALE_CENTI);
}

static int16_t maxMagnitudeFeature(int16_t a, int16_t b) {
  return (absI32((int32_t)a) >= absI32((int32_t)b)) ? a : b;
}

// -------------------------- PWM backend ----------------------------------
static void configurePwm() {
  pinMode(PWM_PIN, OUTPUT);
#if defined(__AVR_ATmega328P__)
  // Timer1 Fast PWM mode 14, TOP=799, prescaler=1: 20 kHz at 16 MHz.
  TCCR1A = 0U;
  TCCR1B = 0U;
  TCNT1 = 0U;
  ICR1 = 799U;
  OCR1A = 0U;
  TCCR1A = (uint8_t)(_BV(COM1A1) | _BV(WGM11));
  TCCR1B = (uint8_t)(_BV(WGM13) | _BV(WGM12) | _BV(CS10));
#else
  // Compatibility only: set a suitable PWM frequency on other MCU targets.
  analogWrite(PWM_PIN, 0);
#endif
}

static void writeDutyBasisPoints(int32_t dutyBp) {
  const int32_t bounded = clampI32(dutyBp, MIN_DUTY_BASIS_POINTS,
                                   MAX_DUTY_BASIS_POINTS);
  appliedDutyBp = bounded;
#if defined(__AVR_ATmega328P__)
  if (bounded == 0L) {
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
  if (faultCode == FAULT_NONE) faultCode = reason;
  writeDutyBasisPoints(0L);
  voltageIntegratorQ16 = 0L;
  currentIntegratorQ16 = 0L;
}

// -------------------------- RLS identification ---------------------------
static void resetRlsModel(RlsModel &model) {
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    model.thetaQ14[i] = 0;
    for (uint8_t j = 0U; j < MODEL_FEATURE_COUNT; ++j) {
      model.covarianceQ12[i][j] = (i == j) ? RLS_P_INITIAL_Q12 : 0L;
    }
  }
  model.disturbanceQ14 = 0;
  model.residualAbsQ14 = 0;
  model.excitationCount = 0U;
  model.inputExcitationCount = 0U;
}

static void resetDualModel(DualModel &model) {
  resetRlsModel(model.fast);
  resetRlsModel(model.slow);
}

static int32_t linearPredictionQ14(const RlsModel &model,
                                   const int16_t phiQ14[MODEL_FEATURE_COUNT]) {
  int64_t sum = 0LL;
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    sum += (int32_t)model.thetaQ14[i] * (int32_t)phiQ14[i];
  }
  return (int32_t)(sum >> 14);
}

static void updateRlsModel(RlsModel &model,
                           const int16_t phiQ14[MODEL_FEATURE_COUNT],
                           int16_t measuredYQ14,
                           int16_t inputExcitationQ14,
                           int16_t otherExcitationQ14,
                           uint16_t lambdaQ12,
                           uint16_t lambdaReciprocalQ15) {
  const int32_t predicted = linearPredictionQ14(model, phiQ14) +
                            (int32_t)model.disturbanceQ14;
  const int32_t innovation = (int32_t)measuredYQ14 - predicted;
  const int32_t absInnovation = absI32(innovation);

  model.residualAbsQ14 = clampI16(
      (int32_t)model.residualAbsQ14 +
      (absInnovation - (int32_t)model.residualAbsQ14) / 8L,
      0, 16384);

  // Residual baseline is the local unmodelled disturbance (usually load
  // current/unmeasured source dynamics). RLS gets the remaining innovation.
  const int32_t disturbanceStep = innovation / 16L;
  model.disturbanceQ14 = clampI16(
      (int32_t)model.disturbanceQ14 + disturbanceStep,
      (int16_t)-RLS_DISTURBANCE_LIMIT_Q14,
      (int16_t)RLS_DISTURBANCE_LIMIT_Q14);

  const int16_t anyExcitationQ14 = maxMagnitudeFeature(inputExcitationQ14,
                                                        otherExcitationQ14);
  if (absI32((int32_t)anyExcitationQ14) < RLS_MIN_EXCITATION_Q14) return;
  if (model.excitationCount < 65535U) ++model.excitationCount;
  if (absI32((int32_t)inputExcitationQ14) >= RLS_MIN_EXCITATION_Q14 &&
      model.inputExcitationCount < 65535U) {
    ++model.inputExcitationCount;
  }
  const int32_t parameterErrorQ14 = innovation - disturbanceStep;

  int32_t pPhiQ12[MODEL_FEATURE_COUNT] = {0L, 0L, 0L};
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    int64_t sum = 0LL;
    for (uint8_t j = 0U; j < MODEL_FEATURE_COUNT; ++j) {
      sum += (int64_t)model.covarianceQ12[i][j] * phiQ14[j];
    }
    pPhiQ12[i] = (int32_t)(sum >> 14);
  }

  int64_t quadraticQ12 = 0LL;
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    quadraticQ12 += ((int64_t)phiQ14[i] * pPhiQ12[i]) >> 14;
  }
  const int64_t denominatorQ12 = (int64_t)lambdaQ12 + quadraticQ12;
  if (denominatorQ12 < 1LL) return;

  int32_t gainQ14[MODEL_FEATURE_COUNT] = {0L, 0L, 0L};
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    gainQ14[i] = (int32_t)(((int64_t)pPhiQ12[i] * MODEL_ONE_Q14) /
                           denominatorQ12);
    const int32_t thetaDelta = (int32_t)(((int64_t)gainQ14[i] *
                                          parameterErrorQ14) >> 14);
    model.thetaQ14[i] = clampI16((int32_t)model.thetaQ14[i] + thetaDelta,
                                 -16384, 16384);
  }

  int32_t newP[MODEL_FEATURE_COUNT][MODEL_FEATURE_COUNT];
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    for (uint8_t j = 0U; j < MODEL_FEATURE_COUNT; ++j) {
      const int64_t correctionQ12 =
          ((int64_t)gainQ14[i] * pPhiQ12[j]) >> 14;
      int64_t reducedQ12 = (int64_t)model.covarianceQ12[i][j] - correctionQ12;
      reducedQ12 = (reducedQ12 * lambdaReciprocalQ15) >> 15;
      // Diagonal covariance terms must remain non-negative. Off-diagonal
      // terms are signed covariance and must not be clipped to a positive
      // floor (doing so destroys P's symmetry/conditioning).
      if (i == j) {
        if (reducedQ12 < RLS_P_MIN_Q12) reducedQ12 = RLS_P_MIN_Q12;
        if (reducedQ12 > RLS_P_MAX_Q12) reducedQ12 = RLS_P_MAX_Q12;
      } else {
        if (reducedQ12 < -RLS_P_MAX_Q12) reducedQ12 = -RLS_P_MAX_Q12;
        if (reducedQ12 > RLS_P_MAX_Q12) reducedQ12 = RLS_P_MAX_Q12;
      }
      newP[i][j] = (int32_t)reducedQ12;
    }
  }

  // Fixed-point rounding can introduce a small asymmetry; restore P = P'.
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    model.covarianceQ12[i][i] = newP[i][i];
    for (uint8_t j = (uint8_t)(i + 1U); j < MODEL_FEATURE_COUNT; ++j) {
      const int32_t symmetric = (newP[i][j] + newP[j][i]) / 2L;
      model.covarianceQ12[i][j] = symmetric;
      model.covarianceQ12[j][i] = symmetric;
    }
  }
}

static void updateDualModel(DualModel &dual,
                            const int16_t phiQ14[MODEL_FEATURE_COUNT],
                            int16_t measuredYQ14,
                            int16_t inputExcitationQ14,
                            int16_t otherExcitationQ14,
                            bool updateSlow) {
  updateRlsModel(dual.fast, phiQ14, measuredYQ14, inputExcitationQ14,
                 otherExcitationQ14, RLS_LAMBDA_FAST_Q12,
                 RLS_FAST_RECIP_Q15);
  if (updateSlow) {
    updateRlsModel(dual.slow, phiQ14, measuredYQ14, inputExcitationQ14,
                   otherExcitationQ14, RLS_LAMBDA_SLOW_Q12,
                   RLS_SLOW_RECIP_Q15);
  }
}

static bool modelHasInputConfidence(const RlsModel &model) {
  return (model.inputExcitationCount >= RLS_MIN_EXCITATIONS) &&
         (model.covarianceQ12[1][1] <= RLS_VALID_P_Q12) &&
         ((int32_t)model.thetaQ14[1] > RLS_MIN_GAIN_Q14) &&
         (model.residualAbsQ14 < RLS_RESIDUAL_BAD_Q14);
}

static int32_t modelDivergenceQ14(const RlsModel &fast,
                                  const RlsModel &slow) {
  int32_t divergence = 0L;
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    divergence += absI32((int32_t)fast.thetaQ14[i] - slow.thetaQ14[i]);
  }
  return divergence;
}

static const RlsModel *selectModel(const DualModel &dual, bool transient) {
  const bool fastValid = modelHasInputConfidence(dual.fast);
  const bool slowValid = modelHasInputConfidence(dual.slow);
  if (transient && fastValid) return &dual.fast;
  if (slowValid) return &dual.slow;
  if (fastValid) return &dual.fast;
  return 0;
}


static void updateLoadEstimate(const RlsModel *voltagePlant) {
  loadEstimateValid = false;
  loadEstimateDeltaCenti = 0L;
  if (voltagePlant == 0 || !modelHasInputConfidence(*voltagePlant)) return;

  const int32_t currentGainQ14 = voltagePlant->thetaQ14[1];
  if (absI32(currentGainQ14) < RLS_MIN_GAIN_Q14) return;

  // In the centered voltage model, wV is equivalent to an unmeasured current
  // disturbance: wV ~= -bI*(Iload - Imean). This is not a direct measurement.
  // Without A2, source-side changes are observationally inseparable from load.
  const int64_t equivalentUnmeasuredCurrent =
      -((int64_t)voltagePlant->disturbanceQ14 *
        MODEL_CURRENT_DEVIATION_CENTI) / currentGainQ14;
  int64_t loadTarget = (int64_t)currentMeanCenti + equivalentUnmeasuredCurrent;
  if (loadTarget < 0LL) loadTarget = 0LL;
  if (loadTarget > (int64_t)CURRENT_LIMIT_CENTI * 2LL)
    loadTarget = (int64_t)CURRENT_LIMIT_CENTI * 2LL;

  const int32_t priorEstimate = estimatedLoadCurrentCenti;
  const int32_t targetQ8 = (int32_t)(loadTarget * 256LL);
  if (!loadEstimateInitialized) {
    estimatedLoadCurrentQ8 = targetQ8;
    estimatedLoadCurrentCenti = (int32_t)loadTarget;
    loadEstimateInitialized = true;
  } else {
    estimatedLoadCurrentQ8 = (int32_t)((int64_t)estimatedLoadCurrentQ8 +
        ((int64_t)targetQ8 - estimatedLoadCurrentQ8) / 8LL);
    estimatedLoadCurrentCenti = estimatedLoadCurrentQ8 / 256L;
  }
  loadEstimateDeltaCenti = estimatedLoadCurrentCenti - priorEstimate;
  loadEstimateValid = true;
}

// -------------------------- Pattern evaluation ---------------------------
static void updateVoltageTrend(uint32_t nowUs, int32_t voltageCenti,
                               int32_t voltageErrorCenti) {
  if (!trendInitialized) {
    trendInitialized = true;
    trendWindowStartUs = nowUs;
    trendWindowStartVoltage = voltageCenti;
    voltageSlopeCentiPerSec = 0L;
  } else if ((uint32_t)(nowUs - trendWindowStartUs) >= VOLTAGE_SLOPE_WINDOW_US) {
    const uint32_t elapsedUs = nowUs - trendWindowStartUs;
    const int32_t delta = voltageCenti - trendWindowStartVoltage;
    voltageSlopeCentiPerSec = (int32_t)(((int64_t)delta * 1000000LL) / elapsedUs);
    trendWindowStartUs = nowUs;
    trendWindowStartVoltage = voltageCenti;
  }

  if ((uint32_t)(nowUs - crossingWindowStartUs) >= CROSSING_WINDOW_US) {
    crossingWindowStartUs = nowUs;
    crossingCount = 0U;
  }

  int8_t sign = 0;
  if (voltageErrorCenti > VOLTAGE_ERROR_DEADBAND_CENTI) sign = 1;
  else if (voltageErrorCenti < -VOLTAGE_ERROR_DEADBAND_CENTI) sign = -1;
  if (sign != 0) {
    if (previousErrorSign != 0 && sign != previousErrorSign &&
        crossingCount < 255U) ++crossingCount;
    previousErrorSign = sign;
  }
}

static OperatingPattern classifyPattern(int32_t voltageErrorCenti,
                                        int32_t currentCenti,
                                        int32_t deltaCurrentCenti,
                                        int32_t deltaSourceCenti,
                                        int32_t deltaLoadEstimateCenti,
                                        const RlsModel *currentSelected,
                                        const RlsModel *voltageSelected) {
  if (currentCenti >= (CURRENT_LIMIT_CENTI * CURRENT_GUARD_HARD_PERCENT) / 100L)
    return PATTERN_CURRENT_BOUND;

#if SOURCE_SENSE_ENABLED
  if (absI32(deltaSourceCenti) >= SOURCE_STEP_TRANSIENT_CENTI)
    return PATTERN_SOURCE_TRANSIENT;
#else
  (void)deltaSourceCenti;
#endif

  if (crossingCount >= OSCILLATION_CROSSINGS)
    return PATTERN_OSCILLATORY;
  if (voltageErrorCenti < -VOLTAGE_ERROR_DEADBAND_CENTI)
    return PATTERN_ABOVE_REFERENCE;

  if (voltageErrorCenti > 0L &&
      voltageErrorCenti <= APPROACH_BAND_CENTI &&
      voltageSlopeCentiPerSec >= FAST_APPROACH_SLOPE_CV_S)
    return PATTERN_FAST_APPROACH;

  const int32_t currentDivergence = modelDivergenceQ14(currentModel.fast,
                                                        currentModel.slow);
  const int32_t voltageDivergence = modelDivergenceQ14(voltageModel.fast,
                                                        voltageModel.slow);
  const bool parameterDrift =
      currentDivergence >= RLS_PARAMETER_DIVERGENCE_Q14 ||
      voltageDivergence >= RLS_PARAMETER_DIVERGENCE_Q14;
  const bool persistentModelMismatch =
      currentModel.fast.residualAbsQ14 >= RLS_RESIDUAL_BAD_Q14 ||
      voltageModel.fast.residualAbsQ14 >= RLS_RESIDUAL_BAD_Q14;
  if (persistentModelMismatch) {
    if (nonlinearResidualCounter < 255U) ++nonlinearResidualCounter;
  } else {
    nonlinearResidualCounter = 0U;
  }
  if (nonlinearResidualCounter >= 16U) return PATTERN_NONLINEAR;
  if (parameterDrift) return PATTERN_MODEL_DRIFT;

  const bool largeInnovation =
      ((currentSelected != 0) &&
       (currentSelected->residualAbsQ14 >= RLS_RESIDUAL_TRANSIENT_Q14)) ||
      ((voltageSelected != 0) &&
       (voltageSelected->residualAbsQ14 >= RLS_RESIDUAL_TRANSIENT_Q14));
  if (largeInnovation ||
      absI32(deltaCurrentCenti) >= CURRENT_STEP_TRANSIENT_CENTI ||
      (loadEstimateValid &&
       absI32(deltaLoadEstimateCenti) >= CURRENT_STEP_TRANSIENT_CENTI)) {
#if SOURCE_SENSE_ENABLED
    return PATTERN_LOAD_TRANSIENT;
#else
    return PATTERN_UNATTRIBUTED_TRANSIENT;
#endif
  }

  if (currentSelected == 0 || voltageSelected == 0 ||
      currentSelected->residualAbsQ14 >= RLS_RESIDUAL_BAD_Q14 ||
      voltageSelected->residualAbsQ14 >= RLS_RESIDUAL_BAD_Q14)
    return PATTERN_MODEL_UNCERTAIN;

  return PATTERN_STEADY;
}

// -------------------------- Candidate library access ---------------------
static FunctionCandidate readFunctionCandidate(uint8_t index) {
  FunctionCandidate candidate;
#if defined(__AVR__)
  memcpy_P(&candidate, &FUNCTION_CANDIDATES[index], sizeof(candidate));
#else
  candidate = FUNCTION_CANDIDATES[index];
#endif
  return candidate;
}

static ResolutionProfile readResolutionProfile(ResolutionStrategy strategy) {
  ResolutionProfile profile;
#if defined(__AVR__)
  memcpy_P(&profile, &RESOLUTION_PROFILES[(uint8_t)strategy], sizeof(profile));
#else
  profile = RESOLUTION_PROFILES[(uint8_t)strategy];
#endif
  return profile;
}

static ResolutionStrategy chooseResolutionStrategy(OperatingPattern pattern) {
  switch (pattern) {
    case PATTERN_CURRENT_BOUND:
      return RESOLUTION_CURRENT_GUARD;
    case PATTERN_OSCILLATORY:
    case PATTERN_FAST_APPROACH:
      return RESOLUTION_DAMPED;
    case PATTERN_SOURCE_TRANSIENT:
    case PATTERN_LOAD_TRANSIENT:
    case PATTERN_UNATTRIBUTED_TRANSIENT:
    case PATTERN_MODEL_DRIFT:
      return RESOLUTION_FAST_TRANSIENT;
    case PATTERN_NONLINEAR:
    case PATTERN_MODEL_UNCERTAIN:
      return RESOLUTION_IDENTIFY;
    case PATTERN_STEADY:
      return RESOLUTION_STEADY;
    case PATTERN_ABOVE_REFERENCE:
    case PATTERN_BOOTSTRAP:
    default:
      return RESOLUTION_BOOTSTRAP;
  }
}

static CandidatePatternClass candidateClassForPattern(OperatingPattern pattern) {
  switch (pattern) {
    case PATTERN_SOURCE_TRANSIENT:
    case PATTERN_LOAD_TRANSIENT:
    case PATTERN_UNATTRIBUTED_TRANSIENT:
    case PATTERN_MODEL_DRIFT:
      return CANDIDATE_CLASS_TRANSIENT;
    case PATTERN_OSCILLATORY:
    case PATTERN_FAST_APPROACH:
      return CANDIDATE_CLASS_DAMPED;
    case PATTERN_CURRENT_BOUND:
      return CANDIDATE_CLASS_CURRENT_GUARD;
    case PATTERN_NONLINEAR:
    case PATTERN_MODEL_UNCERTAIN:
      return CANDIDATE_CLASS_UNCERTAIN;
    case PATTERN_BOOTSTRAP:
    case PATTERN_ABOVE_REFERENCE:
    case PATTERN_STEADY:
    default:
      return CANDIDATE_CLASS_STEADY;
  }
}

static void recordCandidateOutcome(int32_t measuredDeltaVoltageCenti,
                                   int32_t measuredDeltaCurrentCenti) {
  if (!candidateOutcomePending) return;

  const int32_t voltageError = absI32(
      measuredDeltaVoltageCenti - pendingPredictedVoltageStepCenti);
  const int32_t currentError = absI32(
      measuredDeltaCurrentCenti - pendingPredictedCurrentStepCenti);
  const uint32_t voltageErrorQ8 = (uint32_t)(
      ((uint64_t)(uint32_t)voltageError * 256ULL) /
      (uint32_t)MODEL_V_SCALE_CENTI);
  const uint32_t currentErrorQ8 = (uint32_t)(
      ((uint64_t)(uint32_t)currentError * 256ULL) /
      (uint32_t)MODEL_I_SCALE_CENTI);
  uint32_t combinedErrorQ8 = (voltageErrorQ8 + currentErrorQ8) / 2UL;
  if (combinedErrorQ8 > 65535UL) combinedErrorQ8 = 65535UL;

  CandidateOutcome &outcome =
      candidateOutcomes[pendingCandidateIndex][pendingCandidateClass];
  if (outcome.observations == 0U) {
    outcome.predictionErrorQ8 = (uint16_t)combinedErrorQ8;
  } else {
    const int32_t difference = (int32_t)combinedErrorQ8 -
                               (int32_t)outcome.predictionErrorQ8;
    outcome.predictionErrorQ8 = (uint16_t)clampI32(
        (int32_t)outcome.predictionErrorQ8 + difference / 4L, 0L, 65535L);
  }
  if (outcome.observations < 255U) ++outcome.observations;
  candidateOutcomePending = false;
}

// -------------------------- Model prediction helpers ---------------------
static int32_t scaledDisturbanceQ14(const RlsModel &model,
                                    uint16_t weightPermille) {
  return (int32_t)(((int64_t)model.disturbanceQ14 * weightPermille) / 1000LL);
}

static int32_t predictCurrentNoDutyQ14(const RlsModel &model,
                                       int32_t deltaCurrentCenti,
                                       int32_t deltaVoltageCenti,
                                       int32_t deltaSourceCenti,
                                       int32_t dutyLevelBp,
                                       uint16_t disturbanceWeightPermille) {
  const int16_t phi0 = toQ14(deltaCurrentCenti, MODEL_I_SCALE_CENTI);
  const int16_t phi2 = sourceOutputIncrementQ14(dutyLevelBp,
                                                 deltaSourceCenti,
                                                 deltaVoltageCenti);
  const int32_t selfTerm = (int32_t)(((int64_t)model.thetaQ14[0] * phi0) >> 14);
  const int32_t sourceOutputTerm = (int32_t)(
      ((int64_t)model.thetaQ14[2] * phi2) >> 14);
  return selfTerm + sourceOutputTerm +
         scaledDisturbanceQ14(model, disturbanceWeightPermille);
}

static int32_t predictCurrentQ14(const RlsModel &model,
                                 int32_t deltaCurrentCenti,
                                 int32_t deltaVoltageCenti,
                                 int32_t deltaSourceCenti,
                                 int32_t dutyDeviationBp,
                                 int32_t dutyLevelBp,
                                 uint16_t disturbanceWeightPermille) {
  const int32_t base = predictCurrentNoDutyQ14(
      model, deltaCurrentCenti, deltaVoltageCenti, deltaSourceCenti,
      dutyLevelBp, disturbanceWeightPermille);
  const int16_t dutyFeature = toQ14(dutyDeviationBp, MODEL_DUTY_SCALE_BP);
  return base + (int32_t)(((int64_t)model.thetaQ14[1] * dutyFeature) >> 14);
}

static int32_t predictVoltageBaseQ14(const RlsModel &model,
                                     int32_t deltaVoltageCenti,
                                     int32_t deltaSourceCenti,
                                     uint16_t disturbanceWeightPermille) {
  const int16_t phi0 = toQ14(deltaVoltageCenti, MODEL_V_SCALE_CENTI);
#if SOURCE_SENSE_ENABLED
  const int16_t phi2 = toQ14(deltaSourceCenti, MODEL_SOURCE_SCALE_CENTI);
#else
  (void)deltaSourceCenti;
  const int16_t phi2 = 0;
#endif
  const int32_t selfTerm = (int32_t)(((int64_t)model.thetaQ14[0] * phi0) >> 14);
  const int32_t sourceTerm = (int32_t)(((int64_t)model.thetaQ14[2] * phi2) >> 14);
  return selfTerm + sourceTerm + scaledDisturbanceQ14(model,
                                                       disturbanceWeightPermille);
}

static int32_t addClampedModelTermQ14(int32_t baseQ14,
                                      int16_t gainQ14,
                                      int32_t valueCenti,
                                      int32_t scaleCenti) {
  const int16_t feature = toQ14(valueCenti, scaleCenti);
  return baseQ14 + (int32_t)(((int64_t)gainQ14 * feature) >> 14);
}

// -------------------------- Bootstrap PI ---------------------------------
static int32_t bootstrapCurrentReference(int32_t voltageErrorCenti,
                                         uint32_t dtUs) {
  const int64_t pQ16 = (int64_t)voltageErrorCenti * BOOT_OUTER_KP_Q8 * 256LL;
  const int64_t deltaI_Q16 =
      ((int64_t)voltageErrorCenti * BOOT_OUTER_KI_Q16 * dtUs) / 1000000LL;
  const int64_t maxI_Q16 = (int64_t)CURRENT_LIMIT_CENTI * 65536LL;
  int64_t proposed = (int64_t)voltageIntegratorQ16 + deltaI_Q16;
  proposed = clampI32((int32_t)proposed,
                      (int32_t)-maxI_Q16, (int32_t)maxI_Q16);
  const bool pushingHigh = (pQ16 + proposed > maxI_Q16 && deltaI_Q16 > 0LL);
  const bool pushingLow = (pQ16 + proposed < 0LL && deltaI_Q16 < 0LL);
  if (!pushingHigh && !pushingLow) voltageIntegratorQ16 = (int32_t)proposed;
  int64_t output = pQ16 + voltageIntegratorQ16;
  if (output < 0LL) output = 0LL;
  if (output > maxI_Q16) output = maxI_Q16;
  return (int32_t)((output + 32768LL) / 65536LL);
}

static int32_t bootstrapDuty(int32_t currentErrorCenti, uint32_t dtUs) {
  const int64_t pQ16 = (int64_t)currentErrorCenti * BOOT_INNER_KP_Q8 * 256LL;
  const int64_t deltaDutyQ16 =
      ((int64_t)currentErrorCenti * BOOT_INNER_KI_Q16 * dtUs) / 1000000LL;
  const int64_t maxDutyQ16 = (int64_t)MAX_DUTY_BASIS_POINTS * 65536LL;
  int64_t proposed = (int64_t)currentIntegratorQ16 + deltaDutyQ16;
  proposed = clampI32((int32_t)proposed,
                      (int32_t)-maxDutyQ16, (int32_t)maxDutyQ16);
  const bool pushingHigh = (pQ16 + proposed > maxDutyQ16 && deltaDutyQ16 > 0LL);
  const bool pushingLow = (pQ16 + proposed < 0LL && deltaDutyQ16 < 0LL);
  if (!pushingHigh && !pushingLow) currentIntegratorQ16 = (int32_t)proposed;
  int64_t output = pQ16 + currentIntegratorQ16;
  if (output < 0LL) output = 0LL;
  if (output > maxDutyQ16) output = maxDutyQ16;
  return (int32_t)((output + 32768LL) / 65536LL);
}

// -------------------------- Predict/evaluate functions -------------------
static CandidateResult evaluateCandidate(uint8_t index,
                                          const RlsModel &currentPlant,
                                          const RlsModel &voltagePlant,
                                          int32_t voltageErrorCenti,
                                          int32_t voltageCenti,
                                          int32_t currentCenti,
                                          int32_t currentReferenceStateCenti,
                                          int32_t deltaVoltageCenti,
                                          int32_t deltaCurrentCenti,
                                          int32_t deltaSourceCenti,
                                          int32_t currentDutyBp,
                                          int32_t currentDutyMeanBp,
                                          OperatingPattern pattern) {
  CandidateResult result;
  result.cost = INT64_MAX;
  result.currentReferenceCenti = currentCenti;
  result.dutyBasisPoints = currentDutyBp;
  result.predictedVoltageDeltaCenti = 0L;
  result.predictedNextVoltageDeltaCenti = 0L;
  result.predictedCurrentDeltaCenti = 0L;
  result.candidateIndex = index;
  result.valid = false;

  const FunctionCandidate candidate = readFunctionCandidate(index);

  const int32_t outerGainQ14 = voltagePlant.thetaQ14[1];
  if (outerGainQ14 <= RLS_MIN_GAIN_Q14) return result;

  // The voltage stage has one sample of relative delay behind the current
  // stage. First predict dV[k+1] from the current state I[k], then calculate
  // the current needed to shape dV[k+2]. This avoids pretending that a duty
  // update can change inductor current and capacitor voltage instantaneously.
  const int32_t voltageBase1Q14 = predictVoltageBaseQ14(
      voltagePlant, deltaVoltageCenti, deltaSourceCenti,
      candidate.disturbanceWeightPermille);
  const int16_t measuredCurrentFeature = toQ14(
      currentCenti - currentMeanCenti, MODEL_CURRENT_DEVIATION_CENTI);
  const int32_t currentTerm1Q14 = (int32_t)(
      ((int64_t)outerGainQ14 * measuredCurrentFeature) >> 14);
  const int32_t predictedDeltaV1Q14 = voltageBase1Q14 + currentTerm1Q14;
  const int32_t predictedDeltaV1 = fromQ14(predictedDeltaV1Q14,
                                           MODEL_V_SCALE_CENTI);
  const int32_t errorAfter1 = voltageErrorCenti - predictedDeltaV1;

  const int32_t voltageBase2Q14 = predictVoltageBaseQ14(
      voltagePlant, predictedDeltaV1, deltaSourceCenti,
      candidate.disturbanceWeightPermille);
  int32_t desiredDeltaV2Centi =
      (int32_t)(((int64_t)candidate.alphaVoltageQ14 * errorAfter1) >> 14) -
      (int32_t)(((int64_t)candidate.dampingVoltageQ14 * predictedDeltaV1) >> 14);
  desiredDeltaV2Centi = clampI32(desiredDeltaV2Centi,
                                 -MAX_PREDICTED_DV_CENTI,
                                 MAX_PREDICTED_DV_CENTI);
  const int32_t desiredDeltaV2Q14 = toQ14(desiredDeltaV2Centi,
                                          MODEL_V_SCALE_CENTI);
  const int32_t voltageResidualQ14 = desiredDeltaV2Q14 - voltageBase2Q14;

  // Voltage model uses absolute inductor current as its manipulated input:
  // dV[k+2] = ... + bI*I[k+1]. Solve for the desired next current state.
  int64_t requestedCurrentCentered =
      ((int64_t)voltageResidualQ14 * MODEL_CURRENT_DEVIATION_CENTI) /
      outerGainQ14;
  int64_t requestedCurrent = (int64_t)currentMeanCenti + requestedCurrentCentered;
  if (requestedCurrent < 0LL) requestedCurrent = 0LL;
  if (requestedCurrent > CURRENT_LIMIT_CENTI) requestedCurrent = CURRENT_LIMIT_CENTI;
  int32_t currentSlew = candidate.currentSlewCenti;
  if ((candidate.flags & CANDIDATE_CURRENT_GUARD) != 0U) {
    const int32_t guardedReference =
        (CURRENT_LIMIT_CENTI * CURRENT_GUARD_HARD_PERCENT) / 100L;
    if (requestedCurrent > guardedReference) requestedCurrent = guardedReference;
    if (currentSlew < 50L) currentSlew = 50L;
  }
  int32_t currentReference = limitSlew((int32_t)requestedCurrent,
                                       currentReferenceStateCenti,
                                       currentSlew);
  currentReference = clampI32(currentReference, 0L, CURRENT_LIMIT_CENTI);

  // Inner discrete CLF target for current increment.
  const int32_t currentError = currentReference - currentCenti;
  int32_t desiredDeltaICenti =
      (int32_t)(((int64_t)candidate.alphaCurrentQ14 * currentError) >> 14) -
      (int32_t)(((int64_t)candidate.dampingCurrentQ14 * deltaCurrentCenti) >> 14);
  desiredDeltaICenti = clampI32(desiredDeltaICenti,
                                -MAX_PREDICTED_DI_CENTI,
                                MAX_PREDICTED_DI_CENTI);

  const int32_t desiredDeltaIQ14 = toQ14(desiredDeltaICenti,
                                         MODEL_I_SCALE_CENTI);
  const int32_t innerGainQ14 = currentPlant.thetaQ14[1];
  if (innerGainQ14 <= RLS_MIN_GAIN_Q14) return result;

  // Duty is identified as a deviation from a slow operating-point estimate.
  // Solve the local inverse twice because the source-feedforward regressor
  // contains D*delta(Vin), which itself depends weakly on the requested duty.
  int32_t requestedDuty = currentDutyBp;
  for (uint8_t solve = 0U; solve < 2U; ++solve) {
    const int32_t baseWithoutDutyQ14 = predictCurrentNoDutyQ14(
        currentPlant, deltaCurrentCenti, deltaVoltageCenti, deltaSourceCenti,
        requestedDuty, candidate.disturbanceWeightPermille);
    const int32_t currentResidualQ14 = desiredDeltaIQ14 - baseWithoutDutyQ14;
    int64_t requiredDeviationBp =
        ((int64_t)currentResidualQ14 * MODEL_DUTY_SCALE_BP) / innerGainQ14;
    if (requiredDeviationBp > MAX_DUTY_BASIS_POINTS)
      requiredDeviationBp = MAX_DUTY_BASIS_POINTS;
    if (requiredDeviationBp < -MAX_DUTY_BASIS_POINTS)
      requiredDeviationBp = -MAX_DUTY_BASIS_POINTS;
    int64_t absoluteDuty = (int64_t)currentDutyMeanBp + requiredDeviationBp;
    if (absoluteDuty > MAX_DUTY_BASIS_POINTS)
      absoluteDuty = MAX_DUTY_BASIS_POINTS;
    if (absoluteDuty < MIN_DUTY_BASIS_POINTS)
      absoluteDuty = MIN_DUTY_BASIS_POINTS;
    requestedDuty = limitSlew((int32_t)absoluteDuty, currentDutyBp,
                              candidate.dutySlewBp);
    requestedDuty = clampI32(requestedDuty, MIN_DUTY_BASIS_POINTS,
                             MAX_DUTY_BASIS_POINTS);
  }

  const int32_t appliedDutyDelta = requestedDuty - currentDutyBp;
  const int32_t predictedCurrentQ14 = predictCurrentQ14(
      currentPlant, deltaCurrentCenti, deltaVoltageCenti, deltaSourceCenti,
      requestedDuty - currentDutyMeanBp, requestedDuty,
      candidate.disturbanceWeightPermille);
  const int32_t predictedCurrentDelta =
      fromQ14(predictedCurrentQ14, MODEL_I_SCALE_CENTI);
  const int32_t predictedCurrent = currentCenti + predictedCurrentDelta;

  // Two-step voltage forecast: first interval is driven by measured I[k],
  // second interval by the model-predicted I[k+1] after the proposed duty.
  const int32_t predictedDeltaV2Q14 = addClampedModelTermQ14(
      voltageBase2Q14, voltagePlant.thetaQ14[1],
      predictedCurrent - currentMeanCenti,
      MODEL_CURRENT_DEVIATION_CENTI);
  const int32_t predictedDeltaV2 = fromQ14(predictedDeltaV2Q14,
                                           MODEL_V_SCALE_CENTI);
  const int32_t predictedVoltageDelta = predictedDeltaV1 + predictedDeltaV2;
  const int32_t predictedVoltage = voltageCenti + predictedVoltageDelta;

  // Do not select a candidate whose one-step prediction crosses a hard
  // protection boundary. The independent raw ADC trip remains authoritative.
  if (predictedCurrent >= HARD_OVERCURRENT_CENTI ||
      predictedVoltage >= OUTPUT_OVERVOLTAGE_CENTI) return result;

  const int32_t nextVoltageError = voltageErrorCenti - predictedVoltageDelta;
  const int32_t nextCurrentError = currentReference - predictedCurrent;
  const int64_t eV = nextVoltageError;
  const int64_t eI = nextCurrentError;
  const int64_t du = appliedDutyDelta;

  // Discrete Lyapunov proxy V = eV^2 + wI*eI^2, plus actuator and barrier costs.
  const int32_t presentCurrentError =
      currentReferenceStateCenti - currentCenti;
  const int64_t presentVoltageError = voltageErrorCenti;
  const int64_t presentCurrentError64 = presentCurrentError;
  const int64_t presentLyapunov =
      (presentVoltageError * presentVoltageError) +
      COST_CURRENT_WEIGHT * (presentCurrentError64 * presentCurrentError64);
  const int64_t predictedLyapunov =
      (eV * eV) + COST_CURRENT_WEIGHT * (eI * eI);

  // Greedy discrete CLF evaluation: first minimize predicted V, then penalize
  // any candidate that fails to decrease it. This is a model-based descent
  // preference, not a stability proof under model error or unmodelled delay.
  int64_t cost = predictedLyapunov;
  if (predictedLyapunov >= presentLyapunov) {
    cost += (predictedLyapunov - presentLyapunov + 1LL) * 4LL;
  }
  cost += (du * du) / 20LL;
  const int32_t softCurrent =
      (CURRENT_LIMIT_CENTI * CURRENT_GUARD_START_PERCENT) / 100L;
  if (predictedCurrent > softCurrent) {
    const int64_t over = predictedCurrent - softCurrent;
    cost += over * over * 16LL;
  }
  if (predictedCurrent < 0L) {
    const int64_t under = predictedCurrent;
    cost += under * under * 16LL;
  }
  if (predictedVoltage >= OUTPUT_OVERVOLTAGE_CENTI - 25L) {
    const int64_t overV = predictedVoltage - (OUTPUT_OVERVOLTAGE_CENTI - 25L);
    cost += overV * overV * 64LL;
  }

  const int32_t uncertainty = (int32_t)currentPlant.residualAbsQ14 +
                              (int32_t)voltagePlant.residualAbsQ14;
  cost += ((int64_t)absI32(appliedDutyDelta) * uncertainty) / 8LL;

  // Pattern priors restrict risky functions while retaining prediction scoring.
  if (pattern == PATTERN_SOURCE_TRANSIENT || pattern == PATTERN_LOAD_TRANSIENT ||
      pattern == PATTERN_UNATTRIBUTED_TRANSIENT ||
      pattern == PATTERN_MODEL_DRIFT) {
    if ((candidate.flags & CANDIDATE_DISTURBANCE) == 0U) cost += 12000LL;
  } else if (pattern == PATTERN_OSCILLATORY ||
             pattern == PATTERN_FAST_APPROACH) {
    if ((candidate.flags & CANDIDATE_DAMPED) == 0U) cost += 25000LL;
    if ((candidate.flags & CANDIDATE_FAST) != 0U) cost += 50000LL;
  } else if (pattern == PATTERN_NONLINEAR || pattern == PATTERN_MODEL_UNCERTAIN) {
    if ((candidate.flags & CANDIDATE_ROBUST) == 0U) cost += 18000LL;
    if ((candidate.flags & CANDIDATE_FAST) != 0U) cost += 25000LL;
  } else if (pattern == PATTERN_CURRENT_BOUND) {
    if ((candidate.flags & CANDIDATE_CURRENT_GUARD) == 0U) cost += 50000LL;
  } else if (pattern == PATTERN_STEADY) {
    if ((candidate.flags & CANDIDATE_ROBUST) != 0U) cost += 1000LL;
  }

  const CandidateOutcome &history =
      candidateOutcomes[index][candidateClassForPattern(pattern)];
  if (history.observations >= 8U) {
    const int64_t errorQ8 = history.predictionErrorQ8;
    cost += ((errorQ8 * errorQ8) * CANDIDATE_HISTORY_COST_WEIGHT) >> 16;
  }

  result.cost = cost;
  result.currentReferenceCenti = currentReference;
  result.dutyBasisPoints = requestedDuty;
  result.predictedVoltageDeltaCenti = predictedVoltageDelta;
  result.predictedNextVoltageDeltaCenti = predictedDeltaV1;
  result.predictedCurrentDeltaCenti = predictedCurrentDelta;
  result.valid = true;
  return result;
}

static CandidateResult chooseBestCandidate(const RlsModel &currentPlant,
                                            const RlsModel &voltagePlant,
                                            int32_t voltageErrorCenti,
                                            int32_t voltageCenti,
                                            int32_t currentCenti,
                                            int32_t currentReferenceStateCenti,
                                            int32_t deltaVoltageCenti,
                                            int32_t deltaCurrentCenti,
                                            int32_t deltaSourceCenti,
                                            int32_t currentDutyBp,
                                            int32_t currentDutyMeanBp,
                                            OperatingPattern pattern,
                                            uint8_t candidateMask) {
  CandidateResult best;
  best.cost = INT64_MAX;
  best.valid = false;
  for (uint8_t i = 0U; i < FUNCTION_CANDIDATE_COUNT; ++i) {
    if ((candidateMask & (uint8_t)(1U << i)) == 0U) continue;
    const CandidateResult candidate = evaluateCandidate(
        i, currentPlant, voltagePlant, voltageErrorCenti, voltageCenti,
        currentCenti, currentReferenceStateCenti,
        deltaVoltageCenti, deltaCurrentCenti, deltaSourceCenti,
        currentDutyBp, currentDutyMeanBp, pattern);
    if (candidate.valid && (!best.valid || candidate.cost < best.cost)) {
      best = candidate;
    }
  }
  return best;
}

// -------------------------- Main control update --------------------------
static void controlUpdate(uint32_t nowUs, uint32_t dtUs) {
  const uint16_t voltageRaw = (uint16_t)analogRead(VOLTAGE_PIN);
  const uint16_t currentRaw = (uint16_t)analogRead(CURRENT_PIN);
#if SOURCE_SENSE_ENABLED
  const uint16_t sourceRaw = (uint16_t)analogRead(SOURCE_VOLTAGE_PIN);
#endif

  if (voltageRaw >= ADC_RAIL_TRIP_RAW || currentRaw >= ADC_RAIL_TRIP_RAW
#if SOURCE_SENSE_ENABLED
      || sourceRaw >= ADC_RAIL_TRIP_RAW
#endif
      ) {
    if (consecutiveRailSamples < 255U) ++consecutiveRailSamples;
  } else {
    consecutiveRailSamples = 0U;
  }
  if (consecutiveRailSamples >= 2U) {
    latchFault(FAULT_ADC_RAIL);
    return;
  }

  const int32_t voltageInstant = adcToVoltageCenti(voltageRaw);
  const int32_t currentInstant = adcToCurrentCenti(currentRaw);
#if SOURCE_SENSE_ENABLED
  const int32_t sourceInstant = adcToSourceCenti(sourceRaw);
#endif
  if (voltageInstant >= OUTPUT_OVERVOLTAGE_CENTI) {
    latchFault(FAULT_OVERVOLTAGE);
    return;
  }
  if (currentInstant >= HARD_OVERCURRENT_CENTI) {
    latchFault(FAULT_OVERCURRENT);
    return;
  }

  int32_t voltageCenti;
  int32_t currentCenti;
  int32_t sourceCenti = 0L;
  if (!filtersInitialized) {
    voltageFilteredQ8 = voltageInstant * 256L;
    currentFilteredQ8 = currentInstant * 256L;
    currentMeanQ16 = currentInstant * 65536L;
    currentMeanCenti = currentInstant;
#if SOURCE_SENSE_ENABLED
    sourceFilteredQ8 = sourceInstant * 256L;
    sourceCenti = sourceInstant;
#endif
    filtersInitialized = true;
    voltageCenti = voltageInstant;
    currentCenti = currentInstant;
  } else {
    voltageCenti = filterQ8(voltageInstant, voltageFilteredQ8);
    currentCenti = filterQ8(currentInstant, currentFilteredQ8);
    currentMeanQ16 = (int32_t)((int64_t)currentMeanQ16 +
        (((int64_t)currentCenti * 65536LL) - currentMeanQ16) / 4096LL);
    currentMeanCenti = currentMeanQ16 / 65536L;
#if SOURCE_SENSE_ENABLED
    sourceCenti = filterQ8(sourceInstant, sourceFilteredQ8);
#endif
  }

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

  const int32_t voltageReference = voltageReferenceQ16 / 65536L;
  const int32_t voltageError = voltageReference - voltageCenti;
  updateVoltageTrend(nowUs, voltageCenti, voltageError);

  int32_t deltaVoltage = 0L;
  int32_t deltaCurrent = 0L;
  int32_t deltaSource = 0L;
  if (previousMeasurementsValid) {
    deltaVoltage = voltageCenti - previousVoltageCenti;
    deltaCurrent = currentCenti - previousCurrentCenti;
#if SOURCE_SENSE_ENABLED
    deltaSource = sourceCenti - previousSourceCenti;
#endif
    recordCandidateOutcome(deltaVoltage, deltaCurrent);

    const int16_t currentDriveInput = toQ14(
        previousDutyCenteredBp, MODEL_DUTY_SCALE_BP);
    const int16_t currentSourceOutput = sourceOutputIncrementQ14(
        previousDutyLevelBp, deltaSource, deltaVoltage);
    const int16_t currentPhi[MODEL_FEATURE_COUNT] = {
      toQ14(previousDeltaCurrentCenti, MODEL_I_SCALE_CENTI),
      currentDriveInput,
      currentSourceOutput
    };
    const int16_t voltageInputExcitation = toQ14(
        previousCurrentCenteredCenti, MODEL_CURRENT_DEVIATION_CENTI);
    const int16_t voltageOtherExcitation = toQ14(
        deltaSource, MODEL_SOURCE_SCALE_CENTI);
    const int16_t voltagePhi[MODEL_FEATURE_COUNT] = {
      toQ14(previousDeltaVoltageCenti, MODEL_V_SCALE_CENTI),
      voltageInputExcitation,
      voltageOtherExcitation
    };
    const bool updateSlowModels = (slowRlsPhase == 0U);
    slowRlsPhase = (uint8_t)((slowRlsPhase + 1U) &
                             (SLOW_RLS_UPDATE_DIVIDER - 1U));
    updateDualModel(currentModel, currentPhi,
                    toQ14(deltaCurrent, MODEL_I_SCALE_CENTI),
                    currentDriveInput, currentSourceOutput, updateSlowModels);
    updateDualModel(voltageModel, voltagePhi,
                    toQ14(deltaVoltage, MODEL_V_SCALE_CENTI),
                    voltageInputExcitation, voltageOtherExcitation,
                    updateSlowModels);
  }

  const bool identificationTransient =
      absI32(deltaCurrent) >= CURRENT_STEP_TRANSIENT_CENTI ||
      currentModel.fast.residualAbsQ14 >= RLS_RESIDUAL_TRANSIENT_Q14 ||
      voltageModel.fast.residualAbsQ14 >= RLS_RESIDUAL_TRANSIENT_Q14 ||
      modelDivergenceQ14(currentModel.fast, currentModel.slow) >=
          RLS_PARAMETER_DIVERGENCE_Q14 ||
      modelDivergenceQ14(voltageModel.fast, voltageModel.slow) >=
          RLS_PARAMETER_DIVERGENCE_Q14;
#if SOURCE_SENSE_ENABLED
  const bool sourceTransient = absI32(deltaSource) >= SOURCE_STEP_TRANSIENT_CENTI;
#else
  const bool sourceTransient = false;
#endif
  if (identificationTransient || sourceTransient) {
    transientHoldCounter = 24U;
  } else if (transientHoldCounter > 0U) {
    --transientHoldCounter;
  }
  const bool useFastIdentification = transientHoldCounter > 0U;
  const RlsModel *selectedCurrentModel = selectModel(currentModel,
                                                       useFastIdentification);
  const RlsModel *selectedVoltageModel = selectModel(voltageModel,
                                                       useFastIdentification);
  updateLoadEstimate(selectedVoltageModel);

  if (previousMeasurementsValid) {
    activePattern = classifyPattern(voltageError, currentInstant, deltaCurrent,
                                    deltaSource, loadEstimateDeltaCenti,
                                    selectedCurrentModel, selectedVoltageModel);
  } else {
    candidateOutcomePending = false;
    activePattern = PATTERN_BOOTSTRAP;
  }

  // Run bootstrap PI continuously to keep its fallback state current.
  int32_t bootstrapIref = bootstrapCurrentReference(voltageError, dtUs);
  bootstrapIref = limitSlew(bootstrapIref, commandedCurrentReferenceCenti,
                            BOOT_CURRENT_SLEW_CENTI);
  bootstrapIref = clampI32(bootstrapIref, 0L, CURRENT_LIMIT_CENTI);
  const int32_t bootstrapDutyRequest = bootstrapDuty(bootstrapIref - currentCenti,
                                                       dtUs);
  const int32_t bootstrapDutyLimited = limitSlew(bootstrapDutyRequest,
                                                  appliedDutyBp,
                                                  BOOT_DUTY_SLEW_BP);

  activeResolutionStrategy = chooseResolutionStrategy(activePattern);
  const ResolutionProfile resolution =
      readResolutionProfile(activeResolutionStrategy);
  CandidateResult best = {};
  best.cost = INT64_MAX;
  best.candidateIndex = 255U;
  best.valid = false;
  const bool transientModelsReady = !useFastIdentification ||
      (selectedCurrentModel == &currentModel.fast &&
       selectedVoltageModel == &voltageModel.fast);
  const bool modelsReady = selectedCurrentModel != 0 &&
                           selectedVoltageModel != 0 &&
                           transientModelsReady;
  const bool strategyChanged =
      ((uint8_t)activeResolutionStrategy != previousResolutionStrategy);
  if (modelsReady) {
    const bool fullSearchDue = !selectedCandidateAvailable || strategyChanged ||
                               candidateSearchCountdown == 0U;
    if (fullSearchDue) {
      best = chooseBestCandidate(*selectedCurrentModel, *selectedVoltageModel,
                                 voltageError, voltageCenti, currentCenti,
                                 commandedCurrentReferenceCenti,
                                 deltaVoltage, deltaCurrent, deltaSource,
                                 appliedDutyBp, dutyMeanBp, activePattern,
                                 resolution.candidateMask);
      if (best.valid) {
        activeCandidateIndex = best.candidateIndex;
        selectedCandidateAvailable = true;
        candidateSearchCountdown = (resolution.fullSearchPeriod > 0U)
                                 ? (uint8_t)(resolution.fullSearchPeriod - 1U)
                                 : 0U;
      } else {
        selectedCandidateAvailable = false;
      }
    } else {
      best = evaluateCandidate(activeCandidateIndex,
                               *selectedCurrentModel, *selectedVoltageModel,
                               voltageError, voltageCenti, currentCenti,
                               commandedCurrentReferenceCenti,
                               deltaVoltage, deltaCurrent, deltaSource,
                               appliedDutyBp, dutyMeanBp, activePattern);
      if (candidateSearchCountdown > 0U) --candidateSearchCountdown;
      if (!best.valid) selectedCandidateAvailable = false;
    }
  } else {
    selectedCandidateAvailable = false;
  }
  previousResolutionStrategy = (uint8_t)activeResolutionStrategy;

  int32_t selectedDuty = bootstrapDutyLimited;
  commandedCurrentReferenceCenti = bootstrapIref;
  if (best.valid) {
    selectedDuty = best.dutyBasisPoints;
    commandedCurrentReferenceCenti = best.currentReferenceCenti;
    activeCandidateIndex = best.candidateIndex;

    // Track PI integrators to model-controller outputs for bumpless fallback.
    const int64_t outerP = (int64_t)voltageError * BOOT_OUTER_KP_Q8 * 256LL;
    int64_t outerI = ((int64_t)best.currentReferenceCenti << 16) - outerP;
    const int64_t maxOuterI = (int64_t)CURRENT_LIMIT_CENTI * 65536LL;
    outerI = clampI32((int32_t)outerI,
                      (int32_t)-maxOuterI, (int32_t)maxOuterI);
    voltageIntegratorQ16 = (int32_t)outerI;

    const int32_t bootstrapCurrentError = best.currentReferenceCenti - currentCenti;
    const int64_t innerP = (int64_t)bootstrapCurrentError * BOOT_INNER_KP_Q8 * 256LL;
    int64_t innerI = ((int64_t)selectedDuty << 16) - innerP;
    const int64_t maxInnerI = (int64_t)MAX_DUTY_BASIS_POINTS * 65536LL;
    innerI = clampI32((int32_t)innerI,
                      (int32_t)-maxInnerI, (int32_t)maxInnerI);
    currentIntegratorQ16 = (int32_t)innerI;
  } else {
    activeCandidateIndex = 255U;
    if (activePattern != PATTERN_BOOTSTRAP &&
        activePattern != PATTERN_ABOVE_REFERENCE) {
      activePattern = PATTERN_MODEL_UNCERTAIN;
    }
  }

  // Gated zero-mean PRBS duty perturbation supplies persistent excitation
  // when the plant models are uncertain and the power stage is near regulation.
  // It is disabled during transients, above-reference operation and near limits.
  identificationProbeActive = false;
#if IDENTIFICATION_PROBE_ENABLED
  const bool probeModelsNeedExcitation =
      !modelHasInputConfidence(currentModel.fast) ||
      !modelHasInputConfidence(voltageModel.fast);
  const bool probeOperatingWindow =
      (activeResolutionStrategy == RESOLUTION_IDENTIFY) &&
      (resolution.allowProbe != 0U) &&
      (voltageError >= 0L) &&
      (voltageError <= IDENTIFICATION_PROBE_ERROR_BAND_CENTI) &&
      (currentInstant < (CURRENT_LIMIT_CENTI *
                         IDENTIFICATION_PROBE_MAX_CURRENT_PERCENT) / 100L) &&
      (voltageCenti < OUTPUT_OVERVOLTAGE_CENTI -
                      IDENTIFICATION_PROBE_VOLTAGE_MARGIN_CENTI) &&
      (transientHoldCounter == 0U) && probeModelsNeedExcitation &&
      (best.valid == false);
  if (probeOperatingWindow) {
    if (probeDivider < IDENTIFICATION_PROBE_HOLD_TICKS) {
      ++probeDivider;
    } else {
      probeDivider = 0U;
      const uint8_t feedback = (uint8_t)(((probeLfsr >> 6) ^
                                           (probeLfsr >> 5)) & 1U);
      probeLfsr = (uint8_t)(((probeLfsr << 1) | feedback) & 0x7FU);
      if (probeLfsr == 0U) probeLfsr = 0x5DU;
      probeSign = ((probeLfsr & 1U) != 0U) ? 1 : -1;
    }
    selectedDuty = (int32_t)((int64_t)selectedDuty +
        ((probeSign > 0) ? (int64_t)IDENTIFICATION_PROBE_BP
                         : -(int64_t)IDENTIFICATION_PROBE_BP));
    selectedDuty = limitSlew(selectedDuty, appliedDutyBp,
                             IDENTIFICATION_PROBE_SLEW_BP);
    identificationProbeActive = true;
  } else {
    probeDivider = 0U;
  }
#endif

  // Independent fast software barrier uses the unfiltered ADC current. This
  // supplements (never replaces) the external comparator/gate-driver shutdown.
  const int32_t guardEnterCenti =
      (CURRENT_LIMIT_CENTI * CURRENT_GUARD_HARD_PERCENT) / 100L;
  const int32_t guardReleaseCenti =
      (CURRENT_LIMIT_CENTI * CURRENT_GUARD_RELEASE_PERCENT) / 100L;
  const int32_t guardZeroCenti =
      (CURRENT_LIMIT_CENTI * CURRENT_GUARD_ZERO_PERCENT) / 100L;
  if (!currentGuardActive && currentInstant >= guardEnterCenti) {
    currentGuardActive = true;
  } else if (currentGuardActive && currentInstant <= guardReleaseCenti) {
    currentGuardActive = false;
  }
  if (currentGuardActive) {
    commandedCurrentReferenceCenti = clampI32(
        currentInstant - CURRENT_GUARD_BRAKE_STEP_BP / 4L,
        0L, CURRENT_LIMIT_CENTI);
    const int32_t reducedDuty = (appliedDutyBp > CURRENT_GUARD_BRAKE_STEP_BP)
        ? appliedDutyBp - CURRENT_GUARD_BRAKE_STEP_BP : 0L;
    if (selectedDuty > reducedDuty) selectedDuty = reducedDuty;
    if (currentInstant >= guardZeroCenti) selectedDuty = 0L;
    currentIntegratorQ16 = 0L;
  }

  // One-quadrant converter: above reference, cease positive power demand.
  if (voltageError < -VOLTAGE_ERROR_DEADBAND_CENTI) {
    selectedDuty = 0L;
    commandedCurrentReferenceCenti = 0L;
    voltageIntegratorQ16 = 0L;
    currentIntegratorQ16 = 0L;
  }

  selectedDuty = clampI32(selectedDuty, MIN_DUTY_BASIS_POINTS,
                          MAX_DUTY_BASIS_POINTS);
  writeDutyBasisPoints(selectedDuty);
  candidateOutcomePending = best.valid && !identificationProbeActive &&
                            !currentGuardActive &&
                            (voltageError >= -VOLTAGE_ERROR_DEADBAND_CENTI) &&
                            (appliedDutyBp == best.dutyBasisPoints);
  if (candidateOutcomePending) {
    pendingCandidateIndex = best.candidateIndex;
    pendingCandidateClass = candidateClassForPattern(activePattern);
    pendingPredictedVoltageStepCenti = best.predictedNextVoltageDeltaCenti;
    pendingPredictedCurrentStepCenti = best.predictedCurrentDeltaCenti;
  }
  previousDutyLevelBp = appliedDutyBp;
  dutyMeanQ16 = (int32_t)((int64_t)dutyMeanQ16 +
      ((((int64_t)appliedDutyBp * 65536LL) - dutyMeanQ16) /
       DUTY_MEAN_TAU_SAMPLES));
  dutyMeanBp = dutyMeanQ16 / 65536L;
  previousDutyCenteredBp = appliedDutyBp - dutyMeanBp;

  if (previousMeasurementsValid) {
    previousDeltaVoltageCenti = deltaVoltage;
    previousDeltaCurrentCenti = deltaCurrent;
  } else {
    previousDeltaVoltageCenti = 0L;
    previousDeltaCurrentCenti = 0L;
  }
  previousVoltageCenti = voltageCenti;
  previousCurrentCenti = currentCenti;
  previousCurrentCenteredCenti = currentCenti - currentMeanCenti;
  previousSourceCenti = sourceCenti;
  previousMeasurementsValid = true;
}

// -------------------------- Arduino entry points -------------------------
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);
#if SOURCE_SENSE_ENABLED
  pinMode(SOURCE_VOLTAGE_PIN, INPUT);
#endif
  analogReference(DEFAULT);
  configurePwm();
  writeDutyBasisPoints(0L);
  resetDualModel(currentModel);
  resetDualModel(voltageModel);

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
  const uint32_t dtUs = nowUs - lastControlUs;
  if (dtUs < CONTROL_PERIOD_US) return;
  lastControlUs = nowUs;
  if (dtUs > MAX_CONTROL_DT_US) {
    latchFault(FAULT_CONTROL_DEADLINE);
    return;
  }

  controlUpdate(nowUs, dtUs);
}
