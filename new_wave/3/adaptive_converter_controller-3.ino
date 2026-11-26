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
 * ATmega328P/Uno: D9/OC1A primary PWM at 20 kHz; D3/OC2B auxiliary PWM
 * at 80 kHz (Timer2, independently configured, TOP=199). A0 output voltage,
 * A1 aggregate output current (ACS712-05B), optional A2 source voltage, A3
 * auxiliary branch current (ACS712-05B; required when auxiliary PWM is armed),
 * optional A4 source current (ACS712-05B) for empirical efficiency learning.
 * D4 is a startup zero-calibration interlock: it must be asserted only when
 * every ACS712 current conductor carries zero current. A4 is also SDA on the
 * Uno and cannot be shared with I2C. Auxiliary duty resolution is 1/200 of a
 * period (0.5%) at 80 kHz.
 * The two MOSFETs must drive electrically independent buck-inductor branches
 * joined at the output bus; this is NOT for paralleled gates into one inductor.
 * With L_aux about 0.1*L_primary and f_aux=4*f_primary, a first-order
 * same-inductor-voltage comparison gives roughly 2.5x auxiliary ripple; check
 * peak-current margin and inductor saturation with the actual Vin/Vout range.
 *
 * Identified incremental dynamics (local discrete-time models):
 *   dI[k] = a1*dI[k-1] + a2*dI[k-2]
 *            + bM*(DM[k-1] - DMmean) + bA*(DA[k-1] - DAmean)
 *            + cI*(DM[k-1]*dVin[k] - dVout[k]) + wI
 *   dV[k] = aV*dV[k-1] + bI*(I[k-1] - Imean) + cV*dVin[k-1] + wV
 *
 * dX denotes a sample-to-sample increment. This incremental current model
 * follows the local incremental inductor-voltage balance. bD is the local
 * duty-to-current gain around a slowly tracked operating duty (not a duty
 * difference); the second input regressor separates D*delta(Vin) - delta(Vout).
 * The second current autoregressive term lets the aggregate model represent
 * two dominant current time constants (the high-L and low-L branches).
 * If source sensing is disabled, Vin changes remain in the net disturbance.
 * The auxiliary actuator is opt-in and is identified with the primary duty
 * held fixed during a bounded low-power ramp/PRBS phase.
 *
 * Two RLS models run in parallel for each stage: fast forgetting for regime
 * changes and slow forgetting for stable operation. A finite flash-resident
 * bank of Lyapunov-style functions is searched according to the active
 * resolution strategy; the selected function is reevaluated every control step.
 * The evaluator uses predicted next-step voltage/current errors, current
 * barrier cost, duty movement and model uncertainty. An optional A4 input-
 * current sensor enables stable-window A/B loss profiles (Pin-Pout), bucketed
 * by load and source voltage; this empirical delta replaces the MOSFET loss
 * prior once both modes are observed. It prefers a decrease in a discrete
 * Lyapunov proxy and is based on predicted output, not merely symptom lookup.
 *
 * The PI controller is only the conservative identification bootstrap and
 * fallback. All candidate functions are bounded; that is not a formal proof
 * of closed-loop stability. Bench validation with the actual power stage is
 * mandatory. Software trips do not replace hardware protection.
 *
 * ACS712-05B current sensing: each current channel is bidirectional, with
 * nominal 185 mV/A sensitivity and VCC/2 output at 0 A. Sensor VCC must share
 * the Uno AVCC/ADC reference rail for the ratiometric conversion used here;
 * fit 100 nF local VCC bypass per module. Startup auto-zero is accepted only
 * with D4 LOW and all enabled current paths physically at 0 A. A 47 nF FILTER
 * capacitor gives about 2 kHz analog bandwidth and 88 us rise time; verify the
 * exact module's filter circuit. ACS712 readings are average feedback, never
 * cycle-by-cycle overcurrent protection. Both enabled power branches require
 * independent fast hardware current comparators that directly inhibit their
 * MOSFET gates. Their open-collector fault outputs are wired-ORed to D2/INT0;
 * the ISR only latches the event while the hardware performs the fast trip.
 * The 80 kHz Timer2 output is not proof that an Uno pin can switch a particular
 * MOSFET gate efficiently; verify gate waveform, gate charge, peak pin current
 * and temperature with an oscilloscope before enabling the auxiliary branch.
 * The build refuses to arm the primary path without an explicit declaration
 * of independent primary hardware protection and D2 fault status.
 */

// -------------------------- Hardware/calibration --------------------------
#define PWM_PIN                         9   // Timer1 OC1A, primary high-L buck path
#define AUX_PWM_PIN                     3   // Timer2 OC2B, auxiliary low-L buck path
#ifndef AUX_PWM_TIMER2_TOP
#define AUX_PWM_TIMER2_TOP              199U // 16 MHz / (199+1) = 80 kHz, prescaler 1
#endif
#define AUX_PWM_TIMER2_COUNTS           (AUX_PWM_TIMER2_TOP + 1U)
#define AUX_PWM_DUTY_QUANTUM_BP         ((10000UL + (AUX_PWM_TIMER2_COUNTS / 2U)) / AUX_PWM_TIMER2_COUNTS)
#ifndef AUX_PATH_ARMED
#define AUX_PATH_ARMED                  0   // Set to 1 only after validating the second power branch
#endif
#ifndef AUX_FAST_CURRENT_PROTECTION_PRESENT
#define AUX_FAST_CURRENT_PROTECTION_PRESENT 0 // Independent fast shunt/comparator -> gate shutdown; not ACS712 ADC feedback
#endif
#ifndef AUX_CURRENT_SENSE_ENABLED
#define AUX_CURRENT_SENSE_ENABLED       0   // Optional A3 branch-current sensor
#endif
#ifndef AUX_HARDWARE_FAULT_STATUS_ENABLED
#define AUX_HARDWARE_FAULT_STATUS_ENABLED 0 // Legacy alias; D2 is a shared power-stage fault input
#endif
#ifndef AUX_HARDWARE_FAULT_ACTIVE_LOW
#define AUX_HARDWARE_FAULT_ACTIVE_LOW   1   // Legacy polarity alias
#endif
#ifndef POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED
#define POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED AUX_HARDWARE_FAULT_STATUS_ENABLED
#endif
#ifndef POWER_STAGE_HARDWARE_FAULT_ACTIVE_LOW
#define POWER_STAGE_HARDWARE_FAULT_ACTIVE_LOW AUX_HARDWARE_FAULT_ACTIVE_LOW
#endif
#define POWER_STAGE_HARDWARE_FAULT_PIN  2   // Uno INT0; wired-OR primary/aux fast-trip status
#ifndef PRIMARY_FAST_CURRENT_PROTECTION_PRESENT
#define PRIMARY_FAST_CURRENT_PROTECTION_PRESENT 0 // Independent cycle-by-cycle comparator directly inhibits primary gate
#endif
#define AUX_HARDWARE_FAULT_PIN          POWER_STAGE_HARDWARE_FAULT_PIN
#define AUX_CURRENT_PIN                 A3
#define VOLTAGE_PIN                     A0
#define CURRENT_PIN                     A1
#ifndef SOURCE_SENSE_ENABLED
#define SOURCE_SENSE_ENABLED            0
#endif
#ifndef SOURCE_CURRENT_SENSE_ENABLED
#define SOURCE_CURRENT_SENSE_ENABLED    0   // Optional A4 input-current sensor for empirical efficiency learning
#endif
#ifndef AUX_EFFICIENCY_LEARNING_ENABLED
#define AUX_EFFICIENCY_LEARNING_ENABLED SOURCE_CURRENT_SENSE_ENABLED
#endif
#define SOURCE_VOLTAGE_PIN              A2
#define SOURCE_CURRENT_PIN              A4  // Conflicts with I2C SDA on Uno; do not use I2C simultaneously

#define ADC_REFERENCE_MV                5000UL // nominal AVCC; ACS712 supply must share AVCC
#define ADC_COUNTS                      1023UL
#define VOLTAGE_DIVIDER_NUM             4UL
#define VOLTAGE_DIVIDER_DEN             1UL
#define SOURCE_DIVIDER_NUM              5UL
#define SOURCE_DIVIDER_DEN              1UL

// ACS712ELCTR-05B-T (bidirectional +/-5 A) nominal sensitivity is 185 mV/A.
// Per-channel overrides permit calibration against a traceable current meter.
#ifndef ACS712_PRIMARY_SENSITIVITY_MV_PER_A
#define ACS712_PRIMARY_SENSITIVITY_MV_PER_A 185UL
#endif
#ifndef ACS712_AUX_SENSITIVITY_MV_PER_A
#define ACS712_AUX_SENSITIVITY_MV_PER_A     185UL
#endif
#ifndef ACS712_SOURCE_SENSITIVITY_MV_PER_A
#define ACS712_SOURCE_SENSITIVITY_MV_PER_A  185UL
#endif
#ifndef ACS712_PRIMARY_POLARITY
#define ACS712_PRIMARY_POLARITY             1
#endif
#ifndef ACS712_AUX_POLARITY
#define ACS712_AUX_POLARITY                 1
#endif
#ifndef ACS712_SOURCE_POLARITY
#define ACS712_SOURCE_POLARITY              1
#endif
#ifndef ACS712_GAINS_CALIBRATED
#define ACS712_GAINS_CALIBRATED             0 // Set to 1 only after known-current calibration of each enabled channel
#endif

// Zero offsets are measured at startup with all sensor conductors at 0 A.
// D4 must be driven LOW by a real zero-current/interlock condition; a floating
// or manually guessed signal is not a substitute for isolating the conductors.
#ifndef ACS712_ZERO_INTERLOCK_PIN
#define ACS712_ZERO_INTERLOCK_PIN           4
#endif
#define ACS712_ZERO_INTERLOCK_ACTIVE_LOW    1
#define ACS712_ZERO_SETTLE_MS               250UL
#define ACS712_ZERO_CAL_SAMPLES              128U
#define ACS712_ZERO_MIN_RAW                  480U
#define ACS712_ZERO_MAX_RAW                  544U
#define ACS712_ZERO_MAX_SPAN_RAW             24U
#define ACS712_ZERO_MAX_STDDEV_SQUARED       36UL // RMS <= 6 ADC counts
#define ACS712_REVERSE_CURRENT_TRIP_CENTI    25L // 0.25 A; reject meaningful reverse flow

// -------------------------- Setpoints/protection --------------------------
#define TARGET_V_CENTI                  1200L
#define CURRENT_LIMIT_CENTI             400L // keep operating range below ACS712 +/-5 A rating
#define OUTPUT_OVERVOLTAGE_CENTI        1300L
#define HARD_OVERCURRENT_CENTI          450L // software trip below +/-5 A sensor full scale
#define ADC_RAIL_TRIP_RAW               1018U

// -------------------------- Timing/actuation ------------------------------
// Approximately 499 Hz control gives Uno arithmetic headroom. The 2003 us
// interval is intentionally incommensurate with 20 kHz / 80 kHz PWM periods,
// reducing coherent ADC sampling of the same switching-ripple phase.
#define CONTROL_PERIOD_US               2003UL // deliberate PWM phase walk; approximately 499.25 Hz
#define MAX_CONTROL_DT_US               5000UL
#define SOFTSTART_RATE_CV_PER_SEC       100L
#define MAX_DUTY_BASIS_POINTS            9000L
#define MIN_DUTY_BASIS_POINTS            0L
#define AUX_MAX_DUTY_BASIS_POINTS        9000L
#define AUX_DUTY_SLEW_BP                 ((int32_t)AUX_PWM_DUTY_QUANTUM_BP)
#define AUX_IDENT_RAMP_BP_PER_TICK       ((int32_t)AUX_PWM_DUTY_QUANTUM_BP)
#define AUX_IDENT_BASE_TOLERANCE_BP      ((int32_t)AUX_PWM_DUTY_QUANTUM_BP)
#define AUX_IDENT_PRBS_BP                ((int32_t)AUX_PWM_DUTY_QUANTUM_BP) // one timer count
#define AUX_IDENT_PRBS_HOLD_TICKS        8U
#define AUX_DISABLE_SLEW_BP              250L
#define AUX_LOW_POWER_ENTER_CENTI        100L  // 1.00 A; tune for the branch and load
#define AUX_LOW_POWER_EXIT_CENTI         150L  // hysteresis at 1.50 A
#define AUX_LOW_POWER_VOLTAGE_ERROR_CENTI 100L
#define AUX_LOW_POWER_MAX_LOAD_CENTI     120L
#define AUX_AUX_CURRENT_LIMIT_CENTI      100L  // soft limit with optional A3 sensing
#define AUX_AUX_HARD_OVERCURRENT_CENTI   120L
#define AUX_MODEL_MIN_EXCITATIONS        24U
#define AUX_IDENT_MAX_TOTAL_CURRENT_CENTI 60L
#ifndef AUX_CANDIDATE_SHARE_PERMILLE
#define AUX_CANDIDATE_SHARE_PERMILLE     150U  // auxiliary handles at most 15% of control increment
#endif
#ifndef AUX_MODEL_DUTY_SCALE_BP
#define AUX_MODEL_DUTY_SCALE_BP          5000L // 10x initial gain normalization; online RLS must identify actual branch gain
#endif

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
#define MODEL_FEATURE_COUNT              5U
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
#define AUXILIARY_DUTY_COST_WEIGHT       2LL
#ifndef AUX_RDS_RELATIVE_Q8
#define AUX_RDS_RELATIVE_Q8              512L // 2.0x primary Rds(on), Q8 ratio
#endif
#ifndef AUX_SWITCHING_LOSS_CREDIT_Q8
#define AUX_SWITCHING_LOSS_CREDIT_Q8      32L // provisional prior; empirical Pin-Pout replaces it
#endif

// Optional empirical loss comparison. Profiles are volatile RAM (no EEPROM
// wear), bucketed by output current and source voltage to reduce A/B bias.
#define EFFICIENCY_CURRENT_BIN_CENTI      25L
#define EFFICIENCY_CURRENT_BIN_COUNT      6U   // covers 0..1.50 A
#define EFFICIENCY_SOURCE_BIN_COUNT       3U
#define EFFICIENCY_SOURCE_BIN_LOW_CENTI   1500L
#define EFFICIENCY_SOURCE_BIN_HIGH_CENTI  2400L
#define EFFICIENCY_BLOCK_SAMPLES          16U
#define EFFICIENCY_MIN_BLOCKS             3U
#define EFFICIENCY_STABLE_VERROR_CENTI    20L
#define EFFICIENCY_STABLE_DCURRENT_CENTI  5L
#define EFFICIENCY_STABLE_DSOURCE_CENTI   15L
#define EFFICIENCY_STABLE_MAIN_DUTY_DELTA_BP 25L
#define EFFICIENCY_STABLE_AUX_DUTY_DELTA_BP 0L // exact timer-count hold for clean A/B windows
#define EFFICIENCY_MIN_OUTPUT_CURRENT_CENTI 10L
#define EFFICIENCY_LOSS_DEADBAND_10MW    10L  // ignore measured differences below 100 mW
#define EFFICIENCY_COST_PER_10MW         20LL

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
#if ACS712_ZERO_CAL_SAMPLES < 32U || ACS712_ZERO_CAL_SAMPLES > 512U
#error "ACS712 zero calibration sample count must be between 32 and 512"
#endif
#if VOLTAGE_DIVIDER_DEN == 0
#error "Invalid voltage-divider calibration"
#endif
#if ACS712_PRIMARY_SENSITIVITY_MV_PER_A == 0 || ACS712_PRIMARY_SENSITIVITY_MV_PER_A > 1000UL
#error "Invalid primary ACS712 sensitivity"
#endif
#if ACS712_PRIMARY_POLARITY != 1 && ACS712_PRIMARY_POLARITY != -1
#error "ACS712_PRIMARY_POLARITY must be +1 or -1"
#endif
#if ACS712_AUX_POLARITY != 1 && ACS712_AUX_POLARITY != -1
#error "ACS712_AUX_POLARITY must be +1 or -1"
#endif
#if ACS712_SOURCE_POLARITY != 1 && ACS712_SOURCE_POLARITY != -1
#error "ACS712_SOURCE_POLARITY must be +1 or -1"
#endif
#if HARD_OVERCURRENT_CENTI > 450L
#error "Keep software overcurrent trip at or below 4.50 A with ACS712-05B sensors"
#endif
#if CURRENT_LIMIT_CENTI > 400L
#error "Keep the normal current limit at or below 4.00 A with ACS712-05B sensors"
#endif
#if SOURCE_DIVIDER_DEN == 0
#error "Invalid source-voltage divider calibration"
#endif
#if SOURCE_CURRENT_SENSE_ENABLED && (ACS712_SOURCE_SENSITIVITY_MV_PER_A == 0 || ACS712_SOURCE_SENSITIVITY_MV_PER_A > 1000UL)
#error "Invalid source ACS712 sensitivity"
#endif
#if AUX_EFFICIENCY_LEARNING_ENABLED && (!SOURCE_SENSE_ENABLED || !SOURCE_CURRENT_SENSE_ENABLED)
#error "Empirical efficiency learning requires both source-voltage and source-current sensing"
#endif
#if AUX_EFFICIENCY_LEARNING_ENABLED && !ACS712_GAINS_CALIBRATED
#error "Empirical efficiency learning requires calibrated ACS712 gains; zero calibration alone is insufficient"
#endif
#if EFFICIENCY_CURRENT_BIN_CENTI <= 0 || EFFICIENCY_BLOCK_SAMPLES == 0
#error "Invalid empirical efficiency binning"
#endif
#if AUX_CURRENT_SENSE_ENABLED && (ACS712_AUX_SENSITIVITY_MV_PER_A == 0 || ACS712_AUX_SENSITIVITY_MV_PER_A > 1000UL)
#error "Invalid auxiliary ACS712 sensitivity"
#endif
#if AUX_PATH_ARMED && !defined(__AVR_ATmega328P__)
#error "The independent 20 kHz / 80 kHz dual-PWM backend is currently implemented only for ATmega328P Uno boards"
#endif
#if defined(__AVR_ATmega328P__) && (!defined(F_CPU) || (F_CPU != 16000000UL))
#error "This PWM configuration requires a 16 MHz ATmega328P (Arduino Uno clock)"
#endif
#if AUX_PATH_ARMED && !AUX_FAST_CURRENT_PROTECTION_PRESENT
#error "AUX_PATH_ARMED requires verified cycle-by-cycle hardware current protection for the low-inductance branch"
#endif
#if AUX_PATH_ARMED && !AUX_CURRENT_SENSE_ENABLED
#error "AUX_PATH_ARMED requires A3 branch-current sensing for average-current supervision"
#endif
#if !PRIMARY_FAST_CURRENT_PROTECTION_PRESENT
#error "Primary PWM requires an independent cycle-by-cycle hardware current comparator that directly inhibits its MOSFET gate"
#endif
#if !POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED
#error "Primary PWM requires a hardware fault status signal on D2/INT0; wire-OR every enabled branch comparator"
#endif
#if AUX_LOW_POWER_EXIT_CENTI <= AUX_LOW_POWER_ENTER_CENTI
#error "Auxiliary low-power hysteresis exit threshold must exceed entry threshold"
#endif
#if AUX_MODEL_DUTY_SCALE_BP <= 0 || AUX_MAX_DUTY_BASIS_POINTS > MAX_DUTY_BASIS_POINTS
#error "Invalid auxiliary PWM calibration/limit"
#endif
#if AUX_PWM_TIMER2_TOP < 15U || AUX_PWM_TIMER2_TOP > 254U
#error "Timer2 TOP must be between 15 and 254 for the auxiliary PWM"
#endif
#if AUX_PWM_TIMER2_COUNTS != 200U
#error "This Uno configuration expects Timer2 TOP=199 for exactly 80 kHz auxiliary PWM"
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
  FAULT_OVERCURRENT,
  FAULT_AUX_OVERCURRENT,
  FAULT_SENSOR_CALIBRATION,
  FAULT_REVERSE_CURRENT,
  FAULT_HARDWARE_TRIP
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
  PATTERN_MODEL_UNCERTAIN,
  PATTERN_AUX_IDENTIFY,
  PATTERN_LOW_POWER
};

enum CandidateFlag : uint8_t {
  CANDIDATE_FAST          = 0x01,
  CANDIDATE_DAMPED        = 0x02,
  CANDIDATE_DISTURBANCE   = 0x04,
  CANDIDATE_ROBUST        = 0x08,
  CANDIDATE_CURRENT_GUARD = 0x10,
  CANDIDATE_AUXILIARY     = 0x20
};

enum ResolutionStrategy : uint8_t {
  RESOLUTION_BOOTSTRAP = 0,
  RESOLUTION_IDENTIFY,
  RESOLUTION_FAST_TRANSIENT,
  RESOLUTION_DAMPED,
  RESOLUTION_CURRENT_GUARD,
  RESOLUTION_STEADY,
  RESOLUTION_AUX_IDENTIFY,
  RESOLUTION_LOW_POWER
};

enum CandidatePatternClass : uint8_t {
  CANDIDATE_CLASS_STEADY = 0,
  CANDIDATE_CLASS_TRANSIENT,
  CANDIDATE_CLASS_DAMPED,
  CANDIDATE_CLASS_CURRENT_GUARD,
  CANDIDATE_CLASS_UNCERTAIN,
  CANDIDATE_CLASS_LOW_POWER,
  CANDIDATE_PATTERN_CLASS_COUNT
};

struct ResolutionProfile {
  uint8_t candidateMask;
  uint8_t fullSearchPeriod;
  uint8_t allowProbe;
  uint8_t allowAuxiliaryProbe;
};

struct FunctionCandidate {
  uint16_t alphaVoltageQ14;
  uint16_t alphaCurrentQ14;
  uint16_t dampingVoltageQ14;
  uint16_t dampingCurrentQ14;
  uint16_t disturbanceWeightPermille;
  uint16_t dutySlewBp;
  uint16_t currentSlewCenti;
  uint16_t auxiliarySharePermille;
  uint16_t auxiliaryDutySlewBp;
  uint8_t flags;
};

// Stored in program flash on AVR. All contraction/damping coefficients are Q14.
static const FunctionCandidate FUNCTION_CANDIDATES[] PROGMEM = {
  // alphaV alphaI dampV dampI disturbance dDuty dCurrent auxShare auxSlew flags
  { 1229U, 4915U,    0U,    819U, 1000U, 80U, 50U,   0U,  0U, CANDIDATE_FAST },
  {  819U, 3277U,  819U,   1638U, 1000U, 40U, 30U,   0U,  0U, 0U },
  {  410U, 1638U, 4915U,   3277U,  750U, 20U, 15U,   0U,  0U, CANDIDATE_DAMPED },
  {  655U, 2458U, 1638U,   1638U, 1000U, 35U, 25U,   0U,  0U, CANDIDATE_DISTURBANCE },
  {  246U,  983U, 6554U,   4915U,  500U, 10U, 10U,   0U,  0U, CANDIDATE_ROBUST },
  {  328U, 1311U, 3277U,   3277U,  500U, 80U, 50U,   0U,  0U, CANDIDATE_CURRENT_GUARD },
  // Low-power-only candidate. It delegates a small incremental share to the
  // low-L branch; the measured aggregate current remains the control feedback.
  {  655U, 2458U, 1638U,   2458U, 1000U, 20U, 12U,
    AUX_CANDIDATE_SHARE_PERMILLE, (uint16_t)AUX_DUTY_SLEW_BP,
    (uint8_t)(CANDIDATE_AUXILIARY | CANDIDATE_ROBUST) }
};
#define FUNCTION_CANDIDATE_COUNT ((uint8_t)(sizeof(FUNCTION_CANDIDATES) / sizeof(FUNCTION_CANDIDATES[0])))

// Bit positions correspond to FUNCTION_CANDIDATES indices 0..6.
static const ResolutionProfile RESOLUTION_PROFILES[] PROGMEM = {
  // mask   bank-search period   main probe   auxiliary identification
  { 0x3EU,       1U,                0U,          0U }, // bootstrap
  { 0x3AU,       8U,                1U,          0U }, // main identification
  { 0x38U,       2U,                0U,          0U }, // fast transient
  { 0x34U,       2U,                0U,          0U }, // damped
  { 0x20U,       1U,                0U,          0U }, // current guard
  { 0x12U,      16U,                0U,          0U }, // steady
  { 0x02U,       1U,                0U,          1U }, // learn auxiliary, primary candidate only
  { 0x42U,       4U,                0U,          0U }  // nominal + auxiliary candidates
};

struct RlsModel {
  int16_t thetaQ14[MODEL_FEATURE_COUNT];
  int32_t covarianceQ12[MODEL_FEATURE_COUNT][MODEL_FEATURE_COUNT];
  int16_t disturbanceQ14;
  int16_t residualAbsQ14;
  uint16_t featureExcitationCount[MODEL_FEATURE_COUNT];
};

struct DualModel {
  RlsModel fast;
  RlsModel slow;
};

struct CandidateResult {
  int64_t cost;
  int32_t currentReferenceCenti;
  int32_t dutyBasisPoints;
  int32_t auxiliaryDutyBasisPoints;
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

#if AUX_EFFICIENCY_LEARNING_ENABLED
struct EfficiencyProfile {
  uint16_t meanLoss10mW;
  uint8_t measuredBlocks; // Saturates at 255; only a small confidence count is needed.
};
#endif

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
static int32_t auxiliaryDutyMeanQ16 = 0L;
static int32_t auxiliaryDutyMeanBp = 0L;
static int32_t previousAuxiliaryDutyCenteredBp = 0L;
static bool auxiliaryLowPowerActive = false;
static bool auxiliaryIdentificationBaseReady = false;
#if AUX_PATH_ARMED
static uint8_t auxiliaryProbeDivider = 0U;
static uint8_t auxiliaryProbeLfsr = 0x6DU;
static int8_t auxiliaryProbeSign = 1;
#endif
static uint8_t probeDivider = 0U;
static uint8_t probeLfsr = 0x5DU;
static int8_t probeSign = 1;
static bool identificationProbeActive = false;
static bool previousAuxiliaryProbeActive = false;
static bool currentGuardActive = false;
#if POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED
static volatile uint8_t powerStageHardwareFaultLatched = 0U;
static void powerStageHardwareFaultISR() {
  // The comparator has already inhibited the gate asynchronously. This ISR
  // merely preserves the event for a synchronous, latched firmware shutdown.
  powerStageHardwareFaultLatched = 1U;
}
#endif
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
#if SOURCE_CURRENT_SENSE_ENABLED
static int32_t sourceCurrentFilteredQ8 = 0L;
#endif
static bool filtersInitialized = false;

static int32_t appliedDutyBp = 0L;
static int32_t appliedAuxiliaryDutyBp = 0L;
static int32_t commandedCurrentReferenceCenti = 0L;
static int32_t previousDutyLevelBp = 0L;
static int32_t auxiliaryCurrentInstantCenti = 0L;
static int32_t previousDeltaVoltageCenti = 0L;
static int32_t previousDeltaCurrentCenti = 0L;
static int32_t previousPreviousDeltaCurrentCenti = 0L;
static int32_t previousVoltageCenti = 0L;
static int32_t previousCurrentCenti = 0L;
static int32_t previousCurrentCenteredCenti = 0L;
static int32_t previousSourceCenti = 0L;
static bool previousMeasurementsValid = false;

#if AUX_EFFICIENCY_LEARNING_ENABLED
// mode 0 = primary-only, mode 1 = auxiliary branch conducting.
static EfficiencyProfile efficiencyProfiles[2][EFFICIENCY_CURRENT_BIN_COUNT]
                                               [EFFICIENCY_SOURCE_BIN_COUNT] = {};
static uint32_t efficiencyBlockLossSum10mW = 0UL;
static uint8_t efficiencyBlockSamples = 0U;
static uint8_t efficiencyBlockMode = 255U;
static uint8_t efficiencyBlockCurrentBin = 255U;
static uint8_t efficiencyBlockSourceBin = 255U;
static int32_t efficiencyPreviousMainDutyBp = -1L;
static int32_t efficiencyPreviousAuxDutyBp = -1L;
#endif

// Equivalent load-current estimate derived from the voltage-model innovation.
static int32_t estimatedLoadCurrentCenti = 0L;
static int32_t estimatedLoadCurrentQ8 = 0L;
static int32_t loadEstimateDeltaCenti = 0L;
static bool loadEstimateInitialized = false;
static bool loadEstimateValid = false;

static DualModel currentModel;
static DualModel voltageModel;
static uint32_t lastControlUs = 0UL;

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

static int32_t quantizePrimaryDutyBp(int32_t dutyBp) {
  const int32_t bounded = clampI32(dutyBp, MIN_DUTY_BASIS_POINTS,
                                    MAX_DUTY_BASIS_POINTS);
#if defined(__AVR_ATmega328P__)
  const uint16_t compare = (uint16_t)(((int64_t)bounded * 800LL + 5000LL) /
                                      10000LL);
  return (int32_t)(((uint32_t)compare * 10000UL + 400UL) / 800UL);
#else
  const uint8_t compare = (uint8_t)(((int64_t)bounded * 255LL + 5000LL) /
                                     10000LL);
  return (int32_t)(((uint16_t)compare * 10000U + 127U) / 255U);
#endif
}

static int32_t quantizeAuxiliaryDutyBp(int32_t dutyBp) {
#if AUX_PATH_ARMED
  const int32_t bounded = clampI32(dutyBp, MIN_DUTY_BASIS_POINTS,
                                    AUX_MAX_DUTY_BASIS_POINTS);
#if defined(__AVR_ATmega328P__)
  const uint16_t compare = (uint16_t)(
      ((uint32_t)bounded * AUX_PWM_TIMER2_COUNTS + 5000UL) / 10000UL);
  return (int32_t)(((uint32_t)compare * 10000UL +
                    (AUX_PWM_TIMER2_COUNTS / 2U)) / AUX_PWM_TIMER2_COUNTS);
#else
  const uint8_t compare = (uint8_t)(((int64_t)bounded * 255LL + 5000LL) /
                                     10000LL);
  return (int32_t)(((uint16_t)compare * 10000U + 127U) / 255U);
#endif
#else
  (void)dutyBp;
  return 0L;
#endif
}

// -------------------------- ADC conversion -------------------------------
static int32_t adcToVoltageCenti(uint16_t raw) {
  const uint64_t numerator = (uint64_t)raw * ADC_REFERENCE_MV *
                             VOLTAGE_DIVIDER_NUM * 100ULL;
  const uint64_t denominator = (uint64_t)ADC_COUNTS * 1000ULL *
                                VOLTAGE_DIVIDER_DEN;
  return (int32_t)((numerator + denominator / 2ULL) / denominator);
}

// Runtime zero offsets are Q8 ADC counts. Q8 storage preserves fractional
// average-count offsets from the startup calibration window.
static int32_t primaryCurrentZeroRawQ8 = 0L;
#if AUX_CURRENT_SENSE_ENABLED
static int32_t auxiliaryCurrentZeroRawQ8 = 0L;
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
static int32_t sourceCurrentZeroRawQ8 = 0L;
#endif
static bool currentSensorsCalibrated = false;

static int32_t adcToAcs712CurrentCenti(uint16_t raw,
                                      int32_t zeroRawQ8,
                                      uint16_t sensitivityMvPerA,
                                      int8_t polarity) {
  const int32_t rawQ8 = (int32_t)raw * 256L;
  const int64_t deltaRawQ8 = (int64_t)rawQ8 - zeroRawQ8;
  const int64_t numerator = deltaRawQ8 * (int64_t)ADC_REFERENCE_MV * 100LL;
  const int64_t denominator = (int64_t)ADC_COUNTS * sensitivityMvPerA * 256LL;
  if (denominator <= 0LL) return 0L;
  int64_t current = (numerator >= 0LL)
      ? (numerator + denominator / 2LL) / denominator
      : -((-numerator + denominator / 2LL) / denominator);
  current *= polarity;
  if (current > 1000LL) current = 1000LL;
  if (current < -1000LL) current = -1000LL;
  return (int32_t)current;
}

static int32_t adcToCurrentCenti(uint16_t raw) {
  return adcToAcs712CurrentCenti(raw, primaryCurrentZeroRawQ8,
      (uint16_t)ACS712_PRIMARY_SENSITIVITY_MV_PER_A,
      (int8_t)ACS712_PRIMARY_POLARITY);
}

#if AUX_CURRENT_SENSE_ENABLED
static int32_t adcToAuxiliaryCurrentCenti(uint16_t raw) {
  return adcToAcs712CurrentCenti(raw, auxiliaryCurrentZeroRawQ8,
      (uint16_t)ACS712_AUX_SENSITIVITY_MV_PER_A,
      (int8_t)ACS712_AUX_POLARITY);
}
#endif

#if SOURCE_SENSE_ENABLED
static int32_t adcToSourceCenti(uint16_t raw) {
  const uint64_t numerator = (uint64_t)raw * ADC_REFERENCE_MV *
                             SOURCE_DIVIDER_NUM * 100ULL;
  const uint64_t denominator = (uint64_t)ADC_COUNTS * 1000ULL *
                               SOURCE_DIVIDER_DEN;
  return (int32_t)((numerator + denominator / 2ULL) / denominator);
}
#endif

#if SOURCE_CURRENT_SENSE_ENABLED
static int32_t adcToSourceCurrentCenti(uint16_t raw) {
  return adcToAcs712CurrentCenti(raw, sourceCurrentZeroRawQ8,
      (uint16_t)ACS712_SOURCE_SENSITIVITY_MV_PER_A,
      (int8_t)ACS712_SOURCE_POLARITY);
}
#endif

static bool zeroCalibrationInterlockActive();

// ACS712 zero calibration. The hardware interlock must ensure zero current
// through every sensor conductor; sampling an unknown live current would
// otherwise silently erase its DC component. The interlock is rechecked while
// sampling, not merely before and after the complete calibration sequence.
// The routine rejects noisy, saturated, or implausibly biased sensor outputs.
static bool calibrateAcs712Zero(uint8_t analogPin, int32_t &zeroRawQ8) {
  for (uint8_t settle = 0U; settle < 4U; ++settle)
    (void)analogRead(analogPin); // discard ADC mux/settling conversions

  uint32_t sum = 0UL;
  uint64_t sumSquares = 0ULL;
  uint16_t minimum = 1023U;
  uint16_t maximum = 0U;
  for (uint16_t sample = 0U; sample < ACS712_ZERO_CAL_SAMPLES; ++sample) {
    if ((sample & 0x0FU) == 0U && !zeroCalibrationInterlockActive())
      return false;
    const uint16_t raw = (uint16_t)analogRead(analogPin);
    if (raw < minimum) minimum = raw;
    if (raw > maximum) maximum = raw;
    sum += raw;
    sumSquares += (uint64_t)raw * raw;
  }

  const uint32_t meanRaw = (sum + (ACS712_ZERO_CAL_SAMPLES / 2U)) /
                           ACS712_ZERO_CAL_SAMPLES;
  if (meanRaw < ACS712_ZERO_MIN_RAW || meanRaw > ACS712_ZERO_MAX_RAW)
    return false;
  if ((uint16_t)(maximum - minimum) > ACS712_ZERO_MAX_SPAN_RAW)
    return false;

  const uint64_t n = ACS712_ZERO_CAL_SAMPLES;
  const uint64_t varianceNumerator = sumSquares * n - (uint64_t)sum * sum;
  const uint64_t varianceLimit =
      (uint64_t)ACS712_ZERO_MAX_STDDEV_SQUARED * n * n;
  if (varianceNumerator > varianceLimit) return false;

  zeroRawQ8 = (int32_t)(((uint64_t)sum * 256ULL + n / 2ULL) / n);
  return true;
}

static bool zeroCalibrationInterlockActive() {
#if ACS712_ZERO_INTERLOCK_ACTIVE_LOW
  return digitalRead(ACS712_ZERO_INTERLOCK_PIN) == LOW;
#else
  return digitalRead(ACS712_ZERO_INTERLOCK_PIN) == HIGH;
#endif
}

static bool calibrateAllCurrentSensors() {
  if (!zeroCalibrationInterlockActive()) return false;
  delay(ACS712_ZERO_SETTLE_MS);
  if (!zeroCalibrationInterlockActive()) return false;

  if (!calibrateAcs712Zero(CURRENT_PIN, primaryCurrentZeroRawQ8))
    return false;
  if (!zeroCalibrationInterlockActive()) return false;
#if AUX_CURRENT_SENSE_ENABLED
  if (!calibrateAcs712Zero(AUX_CURRENT_PIN, auxiliaryCurrentZeroRawQ8))
    return false;
  if (!zeroCalibrationInterlockActive()) return false;
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
  if (!calibrateAcs712Zero(SOURCE_CURRENT_PIN, sourceCurrentZeroRawQ8))
    return false;
  if (!zeroCalibrationInterlockActive()) return false;
#endif
  if (!zeroCalibrationInterlockActive()) return false;
  currentSensorsCalibrated = true;
  return true;
}

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



// -------------------------- PWM backend ----------------------------------
static void configurePwm() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(AUX_PWM_PIN, OUTPUT);
#if defined(__AVR_ATmega328P__)
  // Primary: Timer1 Fast PWM mode 14, TOP=799, prescaler=1: 20 kHz.
  // D9/OC1A. Timer1 channel B is intentionally left unused.
  TCCR1A = 0U;
  TCCR1B = 0U;
  TCNT1 = 0U;
  ICR1 = 799U;
  OCR1A = 0U;
  TCCR1A = (uint8_t)(_BV(WGM11));
  TCCR1B = (uint8_t)(_BV(WGM13) | _BV(WGM12) | _BV(CS10));
#if AUX_PATH_ARMED
  // Auxiliary: Timer2 Fast PWM mode 7, TOP=199, prescaler=1: 80 kHz.
  // D3/OC2B. This carrier is independent of Timer1 and does not affect millis().
  // Timer2 is commandeered; Arduino tone() and Timer2 PWM on D11 are unavailable.
  TCCR2A = 0U;
  TCCR2B = 0U;
  TCNT2 = 0U;
  OCR2A = (uint8_t)AUX_PWM_TIMER2_TOP;
  OCR2B = 0U;
  TCCR2A = (uint8_t)(_BV(WGM21) | _BV(WGM20));
  TCCR2B = (uint8_t)(_BV(WGM22) | _BV(CS20));
#else
  digitalWrite(AUX_PWM_PIN, LOW);
#endif
#else
  // Compatibility path only; only the ATmega328P path configures independent
  // 20 kHz / 80 kHz carriers. Verify timers and pin routing on other targets.
  analogWrite(PWM_PIN, 0);
  analogWrite(AUX_PWM_PIN, 0);
#endif
}

static void writeDutyBasisPoints(int32_t primaryDutyBp,
                                 int32_t auxiliaryDutyBp) {
  const int32_t primaryBounded = clampI32(primaryDutyBp,
      MIN_DUTY_BASIS_POINTS, MAX_DUTY_BASIS_POINTS);
  const int32_t auxiliaryBounded = quantizeAuxiliaryDutyBp(auxiliaryDutyBp);
#if defined(__AVR_ATmega328P__)
  // Quantize the software command to the actual timer compare count, then
  // feed effective duty back to the identifier (never an unachievable command).
  // Timer1 has 800 counts per period.
  const uint16_t primaryCompare = (uint16_t)(
      ((int64_t)primaryBounded * 800LL + 5000LL) / 10000LL);
  appliedDutyBp = (int32_t)(((uint32_t)primaryCompare * 10000UL + 400UL) / 800UL);
  if (primaryCompare == 0U) {
    TCCR1A &= (uint8_t)~_BV(COM1A1);
    PORTB &= (uint8_t)~_BV(PORTB1);
  } else {
    OCR1A = primaryCompare;
    TCCR1A |= (uint8_t)_BV(COM1A1);
  }
#if AUX_PATH_ARMED
  const uint16_t auxiliaryCompare = (uint16_t)(
      ((uint32_t)auxiliaryBounded * AUX_PWM_TIMER2_COUNTS + 5000UL) / 10000UL);
  appliedAuxiliaryDutyBp = (int32_t)(
      ((uint32_t)auxiliaryCompare * 10000UL +
       (AUX_PWM_TIMER2_COUNTS / 2U)) / AUX_PWM_TIMER2_COUNTS);
  if (auxiliaryCompare == 0U) {
    TCCR2A &= (uint8_t)~_BV(COM2B1);
    PORTD &= (uint8_t)~_BV(PORTD3);
  } else {
    OCR2B = (uint8_t)auxiliaryCompare;
    TCCR2A |= (uint8_t)_BV(COM2B1);
  }
#else
  (void)auxiliaryBounded;
  appliedAuxiliaryDutyBp = 0L;
#endif
#else
  const uint8_t primaryPwm = (uint8_t)(((int64_t)primaryBounded * 255LL + 5000LL) / 10000LL);
  const uint8_t auxiliaryPwm = (uint8_t)(((int64_t)auxiliaryBounded * 255LL + 5000LL) / 10000LL);
  appliedDutyBp = (int32_t)(((uint16_t)primaryPwm * 10000U + 127U) / 255U);
  appliedAuxiliaryDutyBp = (int32_t)(((uint16_t)auxiliaryPwm * 10000U + 127U) / 255U);
  analogWrite(PWM_PIN, primaryPwm);
  analogWrite(AUX_PWM_PIN, auxiliaryPwm);
#endif
}

static void latchFault(FaultCode reason) {
  if (faultCode == FAULT_NONE) faultCode = reason;
  writeDutyBasisPoints(0L, 0L);
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
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i)
    model.featureExcitationCount[i] = 0U;
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

  bool hasExcitation = false;
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    if (absI32((int32_t)phiQ14[i]) >= RLS_MIN_EXCITATION_Q14) {
      hasExcitation = true;
      if (model.featureExcitationCount[i] < 65535U)
        ++model.featureExcitationCount[i];
    }
  }
  if (!hasExcitation) return;
  const int32_t parameterErrorQ14 = innovation - disturbanceStep;

  int32_t pPhiQ12[MODEL_FEATURE_COUNT] = {0L};
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

  int32_t gainQ14[MODEL_FEATURE_COUNT] = {0L};
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
                            bool updateSlow) {
  updateRlsModel(dual.fast, phiQ14, measuredYQ14,
                 RLS_LAMBDA_FAST_Q12, RLS_FAST_RECIP_Q15);
  if (updateSlow) {
    updateRlsModel(dual.slow, phiQ14, measuredYQ14,
                   RLS_LAMBDA_SLOW_Q12, RLS_SLOW_RECIP_Q15);
  }
}

static bool modelHasParameterConfidence(const RlsModel &model,
                                         uint8_t featureIndex,
                                         uint16_t minimumExcitations) {
  if (featureIndex >= MODEL_FEATURE_COUNT) return false;
  return (model.featureExcitationCount[featureIndex] >= minimumExcitations) &&
         (model.covarianceQ12[featureIndex][featureIndex] <= RLS_VALID_P_Q12) &&
         ((int32_t)model.thetaQ14[featureIndex] > RLS_MIN_GAIN_Q14);
}

static bool modelHasFeatureConfidence(const RlsModel &model,
                                      uint8_t featureIndex,
                                      uint16_t minimumExcitations) {
  return modelHasParameterConfidence(model, featureIndex, minimumExcitations) &&
         (model.residualAbsQ14 < RLS_RESIDUAL_BAD_Q14);
}

static bool modelHasInputConfidence(const RlsModel &model) {
  // Used for the voltage-stage model: feature 1 is inductor current.
  return modelHasFeatureConfidence(model, 1U, RLS_MIN_EXCITATIONS);
}

static bool modelHasPrimaryCurrentConfidence(const RlsModel &model) {
  return modelHasFeatureConfidence(model, 2U, RLS_MIN_EXCITATIONS);
}

static bool modelHasAuxiliaryConfidence(const RlsModel &model) {
  return modelHasFeatureConfidence(model, 3U, AUX_MODEL_MIN_EXCITATIONS);
}

static bool modelHasBothCurrentActuators(const RlsModel &model) {
  return modelHasPrimaryCurrentConfidence(model) &&
         modelHasAuxiliaryConfidence(model);
}

static int32_t modelDivergenceQ14(const RlsModel &fast,
                                  const RlsModel &slow) {
  int32_t divergence = 0L;
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    divergence += absI32((int32_t)fast.thetaQ14[i] - slow.thetaQ14[i]);
  }
  return divergence;
}

static const RlsModel *selectModel(const DualModel &dual, bool transient,
                                   uint8_t inputFeatureIndex,
                                   bool allowKnownProbeResidual) {
  const bool fastValid = allowKnownProbeResidual
      ? modelHasParameterConfidence(dual.fast, inputFeatureIndex,
                                    RLS_MIN_EXCITATIONS)
      : modelHasFeatureConfidence(dual.fast, inputFeatureIndex,
                                  RLS_MIN_EXCITATIONS);
  const bool slowValid = allowKnownProbeResidual
      ? modelHasParameterConfidence(dual.slow, inputFeatureIndex,
                                    RLS_MIN_EXCITATIONS)
      : modelHasFeatureConfidence(dual.slow, inputFeatureIndex,
                                  RLS_MIN_EXCITATIONS);
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

static void updateAuxiliaryLowPowerState(int32_t currentCenti,
                                         int32_t voltageErrorCenti) {
#if AUX_PATH_ARMED
  const bool voltageWindow =
      (absI32(voltageErrorCenti) <= AUX_LOW_POWER_VOLTAGE_ERROR_CENTI) &&
      (voltageErrorCenti >= -VOLTAGE_ERROR_DEADBAND_CENTI);
  if (!auxiliaryLowPowerActive) {
    if (currentCenti <= AUX_LOW_POWER_ENTER_CENTI && voltageWindow)
      auxiliaryLowPowerActive = true;
  } else {
    if (currentCenti >= AUX_LOW_POWER_EXIT_CENTI || !voltageWindow ||
        transientHoldCounter > 0U || currentGuardActive)
      auxiliaryLowPowerActive = false;
  }
#else
  (void)currentCenti;
  (void)voltageErrorCenti;
  auxiliaryLowPowerActive = false;
#endif
}

static OperatingPattern classifyPattern(int32_t voltageErrorCenti,
                                        int32_t currentCenti,
                                        int32_t deltaCurrentCenti,
                                        int32_t deltaSourceCenti,
                                        int32_t deltaLoadEstimateCenti,
                                        const RlsModel *currentSelected,
                                        const RlsModel *voltageSelected,
                                        bool auxiliaryModelReady,
                                        bool knownControlExcitation) {
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

  if (!knownControlExcitation) {
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
  } else if (nonlinearResidualCounter > 0U) {
    --nonlinearResidualCounter;
  }

  const bool largeInnovation = !knownControlExcitation &&
      ((currentSelected != 0 &&
        currentSelected->residualAbsQ14 >= RLS_RESIDUAL_TRANSIENT_Q14) ||
       (voltageSelected != 0 &&
        voltageSelected->residualAbsQ14 >= RLS_RESIDUAL_TRANSIENT_Q14));
  if (largeInnovation || (!knownControlExcitation &&
      (absI32(deltaCurrentCenti) >= CURRENT_STEP_TRANSIENT_CENTI ||
       (loadEstimateValid &&
        absI32(deltaLoadEstimateCenti) >= CURRENT_STEP_TRANSIENT_CENTI)))) {
#if SOURCE_SENSE_ENABLED
    return PATTERN_LOAD_TRANSIENT;
#else
    return PATTERN_UNATTRIBUTED_TRANSIENT;
#endif
  }

  if (currentSelected == 0 || voltageSelected == 0)
    return PATTERN_MODEL_UNCERTAIN;
  if (!knownControlExcitation &&
      (currentSelected->residualAbsQ14 >= RLS_RESIDUAL_BAD_Q14 ||
       voltageSelected->residualAbsQ14 >= RLS_RESIDUAL_BAD_Q14))
    return PATTERN_MODEL_UNCERTAIN;

#if AUX_PATH_ARMED
  if (auxiliaryLowPowerActive) {
    return auxiliaryModelReady ? PATTERN_LOW_POWER : PATTERN_AUX_IDENTIFY;
  }
#else
  (void)auxiliaryModelReady;
#endif
  if (knownControlExcitation) return PATTERN_MODEL_UNCERTAIN;
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
    case PATTERN_AUX_IDENTIFY:
      return RESOLUTION_AUX_IDENTIFY;
    case PATTERN_LOW_POWER:
      return RESOLUTION_LOW_POWER;
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
    case PATTERN_AUX_IDENTIFY:
      return CANDIDATE_CLASS_UNCERTAIN;
    case PATTERN_LOW_POWER:
      return CANDIDATE_CLASS_LOW_POWER;
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

#if AUX_EFFICIENCY_LEARNING_ENABLED
static uint8_t efficiencyCurrentBin(int32_t outputCurrentCenti) {
  int32_t bin = outputCurrentCenti / EFFICIENCY_CURRENT_BIN_CENTI;
  if (bin < 0L) bin = 0L;
  if (bin >= (int32_t)EFFICIENCY_CURRENT_BIN_COUNT)
    bin = (int32_t)EFFICIENCY_CURRENT_BIN_COUNT - 1L;
  return (uint8_t)bin;
}

static uint8_t efficiencySourceBin(int32_t sourceVoltageCenti) {
  if (sourceVoltageCenti < EFFICIENCY_SOURCE_BIN_LOW_CENTI) return 0U;
  if (sourceVoltageCenti < EFFICIENCY_SOURCE_BIN_HIGH_CENTI) return 1U;
  return 2U;
}

static void recordEfficiencyMeasurement(int32_t sourceVoltageCenti,
                                        int32_t sourceCurrentCenti,
                                        int32_t outputVoltageCenti,
                                        int32_t outputCurrentCenti,
                                        int32_t voltageErrorCenti,
                                        int32_t deltaCurrentCenti,
                                        int32_t deltaSourceCenti,
                                        bool knownControlExcitation,
                                        bool currentLimitActive) {
  // Attribute this sample to the actuator state that produced the preceding
  // interval. Exclude transients and probes so the two modes are compared at
  // nearly equal operating points rather than during different load steps.
  const bool dutyHistoryValid = efficiencyPreviousMainDutyBp >= 0L &&
                                efficiencyPreviousAuxDutyBp >= 0L;
  const bool dutyStable = dutyHistoryValid &&
      absI32(appliedDutyBp - efficiencyPreviousMainDutyBp) <=
          EFFICIENCY_STABLE_MAIN_DUTY_DELTA_BP &&
      absI32(appliedAuxiliaryDutyBp - efficiencyPreviousAuxDutyBp) <=
          EFFICIENCY_STABLE_AUX_DUTY_DELTA_BP;
  efficiencyPreviousMainDutyBp = appliedDutyBp;
  efficiencyPreviousAuxDutyBp = appliedAuxiliaryDutyBp;
  const bool stable = dutyStable && !knownControlExcitation &&
      !currentLimitActive && transientHoldCounter == 0U &&
      absI32(voltageErrorCenti) <= EFFICIENCY_STABLE_VERROR_CENTI &&
      absI32(deltaCurrentCenti) <= EFFICIENCY_STABLE_DCURRENT_CENTI &&
      absI32(deltaSourceCenti) <= EFFICIENCY_STABLE_DSOURCE_CENTI &&
      outputCurrentCenti >= EFFICIENCY_MIN_OUTPUT_CURRENT_CENTI &&
      outputCurrentCenti <= AUX_LOW_POWER_EXIT_CENTI;
  if (!stable) {
    efficiencyBlockLossSum10mW = 0UL;
    efficiencyBlockSamples = 0U;
    efficiencyBlockMode = 255U;
    return;
  }

  const uint8_t mode = (appliedAuxiliaryDutyBp > 0L) ? 1U : 0U;
  const uint8_t currentBin = efficiencyCurrentBin(outputCurrentCenti);
  const uint8_t sourceBin = efficiencySourceBin(sourceVoltageCenti);
  if (mode != efficiencyBlockMode || currentBin != efficiencyBlockCurrentBin ||
      sourceBin != efficiencyBlockSourceBin) {
    efficiencyBlockLossSum10mW = 0UL;
    efficiencyBlockSamples = 0U;
    efficiencyBlockMode = mode;
    efficiencyBlockCurrentBin = currentBin;
    efficiencyBlockSourceBin = sourceBin;
  }

  // cV*cA/100 is 10 mW. A negative apparent loss indicates sensor offset or
  // calibration error; clamp it to zero but do not treat it as negative loss.
  const int64_t inputOutputProductDifference =
      (int64_t)sourceVoltageCenti * sourceCurrentCenti -
      (int64_t)outputVoltageCenti * outputCurrentCenti;
  if (inputOutputProductDifference <= 0LL) {
    // Do not turn sensor-offset errors into an apparent zero-loss advantage.
    efficiencyBlockLossSum10mW = 0UL;
    efficiencyBlockSamples = 0U;
    return;
  }
  uint32_t loss10mW = (uint32_t)(inputOutputProductDifference / 100LL);
  if (loss10mW > 65535UL) loss10mW = 65535UL;
  efficiencyBlockLossSum10mW += loss10mW;
  if (efficiencyBlockSamples < 255U) ++efficiencyBlockSamples;

  if (efficiencyBlockSamples >= EFFICIENCY_BLOCK_SAMPLES) {
    const uint16_t blockMean = (uint16_t)(
        efficiencyBlockLossSum10mW / EFFICIENCY_BLOCK_SAMPLES);
    EfficiencyProfile &profile = efficiencyProfiles[mode][currentBin][sourceBin];
    if (profile.measuredBlocks == 0U) {
      profile.meanLoss10mW = blockMean;
    } else {
      const int32_t difference = (int32_t)blockMean -
                                 (int32_t)profile.meanLoss10mW;
      profile.meanLoss10mW = (uint16_t)clampI32(
          (int32_t)profile.meanLoss10mW + difference / 4L, 0L, 65535L);
    }
    if (profile.measuredBlocks < 255U) ++profile.measuredBlocks;
    efficiencyBlockLossSum10mW = 0UL;
    efficiencyBlockSamples = 0U;
  }
}

static bool empiricalAuxiliaryLossAvailable(int32_t outputCurrentCenti,
                                            int32_t sourceVoltageCenti) {
  const uint8_t currentBin = efficiencyCurrentBin(outputCurrentCenti);
  const uint8_t sourceBin = efficiencySourceBin(sourceVoltageCenti);
  const EfficiencyProfile &primary =
      efficiencyProfiles[0U][currentBin][sourceBin];
  const EfficiencyProfile &auxiliary =
      efficiencyProfiles[1U][currentBin][sourceBin];
  return primary.measuredBlocks >= EFFICIENCY_MIN_BLOCKS &&
         auxiliary.measuredBlocks >= EFFICIENCY_MIN_BLOCKS;
}

static int64_t empiricalAuxiliaryLossCost(int32_t outputCurrentCenti,
                                          int32_t sourceVoltageCenti) {
  const uint8_t currentBin = efficiencyCurrentBin(outputCurrentCenti);
  const uint8_t sourceBin = efficiencySourceBin(sourceVoltageCenti);
  const EfficiencyProfile &primary =
      efficiencyProfiles[0U][currentBin][sourceBin];
  const EfficiencyProfile &auxiliary =
      efficiencyProfiles[1U][currentBin][sourceBin];
  if (!empiricalAuxiliaryLossAvailable(outputCurrentCenti, sourceVoltageCenti))
    return 0LL;

  int32_t difference10mW = (int32_t)auxiliary.meanLoss10mW -
                           (int32_t)primary.meanLoss10mW;
  if (absI32(difference10mW) <= EFFICIENCY_LOSS_DEADBAND_10MW) return 0LL;
  if (difference10mW > 0L) difference10mW -= EFFICIENCY_LOSS_DEADBAND_10MW;
  else difference10mW += EFFICIENCY_LOSS_DEADBAND_10MW;
  return (int64_t)difference10mW * EFFICIENCY_COST_PER_10MW;
}
#endif

// -------------------------- Model prediction helpers ---------------------
static int32_t scaledDisturbanceQ14(const RlsModel &model,
                                    uint16_t weightPermille) {
  return (int32_t)(((int64_t)model.disturbanceQ14 * weightPermille) / 1000LL);
}

static int32_t predictCurrentNoDutyQ14(const RlsModel &model,
                                       int32_t deltaCurrentCenti,
                                       int32_t priorDeltaCurrentCenti,
                                       int32_t deltaVoltageCenti,
                                       int32_t deltaSourceCenti,
                                       int32_t dutyLevelBp,
                                       uint16_t disturbanceWeightPermille) {
  const int16_t phi0 = toQ14(deltaCurrentCenti, MODEL_I_SCALE_CENTI);
  const int16_t phi1 = toQ14(priorDeltaCurrentCenti, MODEL_I_SCALE_CENTI);
  const int16_t phi4 = sourceOutputIncrementQ14(dutyLevelBp,
                                                 deltaSourceCenti,
                                                 deltaVoltageCenti);
  const int32_t lag1Term = (int32_t)(((int64_t)model.thetaQ14[0] * phi0) >> 14);
  const int32_t lag2Term = (int32_t)(((int64_t)model.thetaQ14[1] * phi1) >> 14);
  const int32_t sourceOutputTerm = (int32_t)(
      ((int64_t)model.thetaQ14[4] * phi4) >> 14);
  return lag1Term + lag2Term + sourceOutputTerm +
         scaledDisturbanceQ14(model, disturbanceWeightPermille);
}

static int32_t predictCurrentQ14(const RlsModel &model,
                                 int32_t deltaCurrentCenti,
                                 int32_t priorDeltaCurrentCenti,
                                 int32_t deltaVoltageCenti,
                                 int32_t deltaSourceCenti,
                                 int32_t mainDutyDeviationBp,
                                 int32_t auxiliaryDutyDeviationBp,
                                 int32_t mainDutyLevelBp,
                                 uint16_t disturbanceWeightPermille) {
  const int32_t base = predictCurrentNoDutyQ14(
      model, deltaCurrentCenti, priorDeltaCurrentCenti,
      deltaVoltageCenti, deltaSourceCenti, mainDutyLevelBp,
      disturbanceWeightPermille);
  const int16_t mainFeature = toQ14(mainDutyDeviationBp, MODEL_DUTY_SCALE_BP);
  const int16_t auxiliaryFeature = toQ14(auxiliaryDutyDeviationBp,
                                          AUX_MODEL_DUTY_SCALE_BP);
  const int32_t mainTerm = (int32_t)(((int64_t)model.thetaQ14[2] * mainFeature) >> 14);
  const int32_t auxiliaryTerm = (int32_t)(((int64_t)model.thetaQ14[3] * auxiliaryFeature) >> 14);
  return base + mainTerm + auxiliaryTerm;
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
                                          int32_t priorDeltaCurrentCenti,
                                          int32_t deltaSourceCenti,
                                          int32_t sourceVoltageCenti,
                                          int32_t currentDutyBp,
                                          int32_t currentDutyMeanBp,
                                          int32_t auxiliaryDutyBp,
                                          int32_t auxiliaryMeanDutyBp,
                                          OperatingPattern pattern,
                                          bool auxiliaryAllowed) {
  CandidateResult result;
  result.cost = INT64_MAX;
  result.currentReferenceCenti = currentCenti;
  result.dutyBasisPoints = currentDutyBp;
  result.auxiliaryDutyBasisPoints = auxiliaryDutyBp;
  result.predictedVoltageDeltaCenti = 0L;
  result.predictedNextVoltageDeltaCenti = 0L;
  result.predictedCurrentDeltaCenti = 0L;
  result.candidateIndex = index;
  result.valid = false;

  const FunctionCandidate candidate = readFunctionCandidate(index);
  const int32_t outerGainQ14 = voltagePlant.thetaQ14[1];
  if (outerGainQ14 <= RLS_MIN_GAIN_Q14) return result;

  const bool requestsAuxiliary = candidate.auxiliarySharePermille > 0U;
  if (requestsAuxiliary && (!auxiliaryAllowed ||
      currentPlant.thetaQ14[3] <= RLS_MIN_GAIN_Q14)) return result;

  // Model the output capacitor one sample behind the inductor-current state.
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

  // Solve the outer stage for the required next inductor current.
  int64_t requestedCurrentCentered =
      ((int64_t)voltageResidualQ14 * MODEL_CURRENT_DEVIATION_CENTI) /
      outerGainQ14;
  int64_t requestedCurrent = (int64_t)currentMeanCenti + requestedCurrentCentered;
  if (requestedCurrent < 0LL) requestedCurrent = 0LL;
  if (requestedCurrent > CURRENT_LIMIT_CENTI)
    requestedCurrent = CURRENT_LIMIT_CENTI;
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

  // Inner discrete Lyapunov target for the total inductor-current increment.
  const int32_t currentError = currentReference - currentCenti;
  int32_t desiredDeltaICenti =
      (int32_t)(((int64_t)candidate.alphaCurrentQ14 * currentError) >> 14) -
      (int32_t)(((int64_t)candidate.dampingCurrentQ14 * deltaCurrentCenti) >> 14);
  desiredDeltaICenti = clampI32(desiredDeltaICenti,
                                -MAX_PREDICTED_DI_CENTI,
                                MAX_PREDICTED_DI_CENTI);
  const int32_t desiredDeltaIQ14 = toQ14(desiredDeltaICenti,
                                         MODEL_I_SCALE_CENTI);

  // Base prediction excludes both actuators. Allocate a small fraction of the
  // required current increment to the fast, low-L branch only in its low-power
  // resolution class. Its duty has a much finer local scale because L_aux is
  // assumed to be about 10x smaller than L_primary.
  int32_t requestedAuxiliaryDuty =
      (requestsAuxiliary && auxiliaryAllowed)
          ? auxiliaryDutyBp
          : limitSlew(0L, auxiliaryDutyBp, AUX_DISABLE_SLEW_BP);
  const int32_t baseWithoutDutyQ14 = predictCurrentNoDutyQ14(
      currentPlant, deltaCurrentCenti, priorDeltaCurrentCenti,
      deltaVoltageCenti, deltaSourceCenti, currentDutyBp,
      candidate.disturbanceWeightPermille);
  int32_t auxiliaryContributionQ14 = 0L;
  if (requestsAuxiliary) {
    const int32_t controlResidualQ14 = desiredDeltaIQ14 - baseWithoutDutyQ14;
    const int32_t allocatedAuxQ14 = (int32_t)(
        ((int64_t)controlResidualQ14 * candidate.auxiliarySharePermille) / 1000LL);
    const int64_t requiredAuxDeviationBp =
        ((int64_t)allocatedAuxQ14 * AUX_MODEL_DUTY_SCALE_BP) /
        currentPlant.thetaQ14[3];
    int64_t absoluteAuxDuty = (int64_t)auxiliaryMeanDutyBp +
                              requiredAuxDeviationBp;
    absoluteAuxDuty = clampI32((int32_t)absoluteAuxDuty, 0L,
                               AUX_MAX_DUTY_BASIS_POINTS);
    const int32_t auxSlew = (candidate.auxiliaryDutySlewBp > 0U)
        ? candidate.auxiliaryDutySlewBp : AUX_DUTY_SLEW_BP;
    requestedAuxiliaryDuty = limitSlew((int32_t)absoluteAuxDuty,
                                       auxiliaryDutyBp, auxSlew);
    requestedAuxiliaryDuty = clampI32(requestedAuxiliaryDuty, 0L,
                                      AUX_MAX_DUTY_BASIS_POINTS);
    const int16_t auxFeature = toQ14(
        requestedAuxiliaryDuty - auxiliaryMeanDutyBp,
        AUX_MODEL_DUTY_SCALE_BP);
    auxiliaryContributionQ14 = (int32_t)(
        ((int64_t)currentPlant.thetaQ14[3] * auxFeature) >> 14);
  }

  // Solve the primary inductor path for the unallocated remainder.
  const int32_t innerGainQ14 = currentPlant.thetaQ14[2];
  if (innerGainQ14 <= RLS_MIN_GAIN_Q14) return result;
  int32_t requestedDuty = currentDutyBp;
  for (uint8_t solve = 0U; solve < 2U; ++solve) {
    const int32_t sourceBaseQ14 = predictCurrentNoDutyQ14(
        currentPlant, deltaCurrentCenti, priorDeltaCurrentCenti,
        deltaVoltageCenti, deltaSourceCenti, requestedDuty,
        candidate.disturbanceWeightPermille);
    const int32_t currentResidualQ14 = desiredDeltaIQ14 - sourceBaseQ14 -
                                       auxiliaryContributionQ14;
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
  requestedDuty = quantizePrimaryDutyBp(requestedDuty);
  requestedAuxiliaryDuty = quantizeAuxiliaryDutyBp(requestedAuxiliaryDuty);

  const int32_t primaryDutyDelta = requestedDuty - currentDutyBp;
  const int32_t predictedCurrentQ14 = predictCurrentQ14(
      currentPlant, deltaCurrentCenti, priorDeltaCurrentCenti,
      deltaVoltageCenti, deltaSourceCenti, requestedDuty - currentDutyMeanBp,
      requestedAuxiliaryDuty - auxiliaryDutyMeanBp,
      requestedDuty, candidate.disturbanceWeightPermille);
  const int32_t predictedCurrentDelta =
      fromQ14(predictedCurrentQ14, MODEL_I_SCALE_CENTI);
  const int32_t predictedCurrent = currentCenti + predictedCurrentDelta;

  const int32_t predictedDeltaV2Q14 = addClampedModelTermQ14(
      voltageBase2Q14, voltagePlant.thetaQ14[1],
      predictedCurrent - currentMeanCenti,
      MODEL_CURRENT_DEVIATION_CENTI);
  const int32_t predictedDeltaV2 = fromQ14(predictedDeltaV2Q14,
                                           MODEL_V_SCALE_CENTI);
  const int32_t predictedVoltageDelta = predictedDeltaV1 + predictedDeltaV2;
  const int32_t predictedVoltage = voltageCenti + predictedVoltageDelta;
  if (predictedCurrent >= HARD_OVERCURRENT_CENTI ||
      predictedVoltage >= OUTPUT_OVERVOLTAGE_CENTI) return result;

#if AUX_CURRENT_SENSE_ENABLED
  if (requestedAuxiliaryDuty > 0L &&
      auxiliaryCurrentInstantCenti >= AUX_AUX_CURRENT_LIMIT_CENTI) return result;
#endif

  const int32_t nextVoltageError = voltageErrorCenti - predictedVoltageDelta;
  const int32_t nextCurrentError = currentReference - predictedCurrent;
  const int64_t eV = nextVoltageError;
  const int64_t eI = nextCurrentError;
  const int64_t dMain = primaryDutyDelta;
  const int64_t dAux = (int64_t)requestedAuxiliaryDuty - auxiliaryDutyBp;
  const int64_t presentCurrentError =
      (int64_t)currentReferenceStateCenti - currentCenti;
  const int64_t presentLyapunov =
      ((int64_t)voltageErrorCenti * voltageErrorCenti) +
      COST_CURRENT_WEIGHT * presentCurrentError * presentCurrentError;
  const int64_t predictedLyapunov =
      (eV * eV) + COST_CURRENT_WEIGHT * (eI * eI);

  int64_t cost = predictedLyapunov;
  if (predictedLyapunov >= presentLyapunov)
    cost += (predictedLyapunov - presentLyapunov + 1LL) * 4LL;
  cost += (dMain * dMain) / 20LL;
  // The auxiliary duty is slew-limited in real timer quanta. Its movement
  // penalty is intentionally weak so measured loss, not an arbitrary duty
  // preference, decides whether the branch should remain online.
  cost += (dAux * dAux) * AUXILIARY_DUTY_COST_WEIGHT / 100000000LL;
  bool measuredEfficiencyComparisonAvailable = false;
#if AUX_EFFICIENCY_LEARNING_ENABLED
  if (requestsAuxiliary && pattern == PATTERN_LOW_POWER) {
    measuredEfficiencyComparisonAvailable =
        empiricalAuxiliaryLossAvailable(currentCenti, sourceVoltageCenti);
  }
#endif
  if (requestedAuxiliaryDuty > 0L && !measuredEfficiencyComparisonAvailable) {
    // Compare the auxiliary branch's higher conduction loss with its lower
    // switching-energy estimate. The credit is a hardware-dependent prior,
    // not a universal MOSFET property; calibrate it from Eon/Eoff, Qg and fSW.
#if AUX_CURRENT_SENSE_ENABLED
    const int32_t measuredAuxiliaryCurrent = auxiliaryCurrentInstantCenti;
#else
    const int32_t measuredAuxiliaryCurrent = (int32_t)(
        ((int64_t)currentCenti * candidate.auxiliarySharePermille) / 1000LL);
#endif
    // First-order branch-current forecast: reducing auxiliary duty should
    // reduce its share; A3 feedback is preferred, otherwise use the requested
    // current-share prior. This is only a loss-ranking estimate, not protection.
    int32_t auxiliaryCurrentEstimate = measuredAuxiliaryCurrent;
    if (appliedAuxiliaryDutyBp > 0L) {
      auxiliaryCurrentEstimate = (int32_t)(
          ((int64_t)measuredAuxiliaryCurrent * requestedAuxiliaryDuty) /
          appliedAuxiliaryDutyBp);
    } else if (requestedAuxiliaryDuty == 0L) {
      auxiliaryCurrentEstimate = 0L;
    }
    auxiliaryCurrentEstimate = clampI32(auxiliaryCurrentEstimate, 0L,
                                         currentCenti);
    const int64_t auxI2 = (int64_t)auxiliaryCurrentEstimate *
                          auxiliaryCurrentEstimate;
    const int64_t extraConductionPenalty =
        (auxI2 * (AUX_RDS_RELATIVE_Q8 - 256LL)) / (256LL * 100LL);
    const int64_t switchingCredit =
        ((int64_t)auxiliaryCurrentEstimate *
         AUX_SWITCHING_LOSS_CREDIT_Q8) / 256LL;
    cost += extraConductionPenalty - switchingCredit;
  }

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
    const int64_t overV = predictedVoltage -
                          (OUTPUT_OVERVOLTAGE_CENTI - 25L);
    cost += overV * overV * 64LL;
  }

  const int32_t uncertainty = (int32_t)currentPlant.residualAbsQ14 +
                              (int32_t)voltagePlant.residualAbsQ14;
  cost += ((int64_t)(absI32(primaryDutyDelta) + absI32((int32_t)dAux)) *
           uncertainty) / 8LL;

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
  } else if (pattern == PATTERN_LOW_POWER) {
    // No blanket preference: the loss proxy above decides whether the
    // auxiliary branch is worth keeping online at this measured power level.
    (void)candidate.flags;
  }
#if AUX_EFFICIENCY_LEARNING_ENABLED
  if (measuredEfficiencyComparisonAvailable) {
    // Empirical A/B data replaces the MOSFET prior once both modes have enough
    // stable windows in the same load/Vin bucket.
    cost += empiricalAuxiliaryLossCost(currentCenti, sourceVoltageCenti);
  }
#else
  (void)sourceVoltageCenti;
#endif

  const CandidateOutcome &history =
      candidateOutcomes[index][candidateClassForPattern(pattern)];
  if (history.observations >= 8U) {
    const int64_t errorQ8 = history.predictionErrorQ8;
    cost += ((errorQ8 * errorQ8) * CANDIDATE_HISTORY_COST_WEIGHT) >> 16;
  }

  result.cost = cost;
  result.currentReferenceCenti = currentReference;
  result.dutyBasisPoints = requestedDuty;
  result.auxiliaryDutyBasisPoints = requestedAuxiliaryDuty;
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
                                            int32_t priorDeltaCurrentCenti,
                                            int32_t deltaSourceCenti,
                                            int32_t sourceVoltageCenti,
                                            int32_t currentDutyBp,
                                            int32_t currentDutyMeanBp,
                                            int32_t auxiliaryDutyBp,
                                            int32_t auxiliaryMeanDutyBp,
                                            OperatingPattern pattern,
                                            uint8_t candidateMask,
                                            bool auxiliaryAllowed) {
  CandidateResult best;
  best.cost = INT64_MAX;
  best.valid = false;
  for (uint8_t i = 0U; i < FUNCTION_CANDIDATE_COUNT; ++i) {
    if ((candidateMask & (uint8_t)(1U << i)) == 0U) continue;
    const CandidateResult candidate = evaluateCandidate(
        i, currentPlant, voltagePlant, voltageErrorCenti, voltageCenti,
        currentCenti, currentReferenceStateCenti,
        deltaVoltageCenti, deltaCurrentCenti, priorDeltaCurrentCenti,
        deltaSourceCenti, sourceVoltageCenti, currentDutyBp, currentDutyMeanBp,
        auxiliaryDutyBp,
        auxiliaryMeanDutyBp, pattern, auxiliaryAllowed);
    if (candidate.valid && (!best.valid || candidate.cost < best.cost))
      best = candidate;
  }
  return best;
}

// -------------------------- Main control update --------------------------
static void controlUpdate(uint32_t nowUs, uint32_t dtUs) {
  const uint16_t voltageRaw = (uint16_t)analogRead(VOLTAGE_PIN);
  const uint16_t currentRaw = (uint16_t)analogRead(CURRENT_PIN);
#if AUX_CURRENT_SENSE_ENABLED
  const uint16_t auxiliaryCurrentRaw = (uint16_t)analogRead(AUX_CURRENT_PIN);
#endif
#if SOURCE_SENSE_ENABLED
  const uint16_t sourceRaw = (uint16_t)analogRead(SOURCE_VOLTAGE_PIN);
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
  const uint16_t sourceCurrentRaw = (uint16_t)analogRead(SOURCE_CURRENT_PIN);
#endif

  // Sensor saturation invalidates every downstream conversion. Latch it
  // before calculating voltage/current so it cannot be misreported as a
  // different fault or used by the adaptive controller.
  if (voltageRaw >= ADC_RAIL_TRIP_RAW ||
      currentRaw <= 5U || currentRaw >= ADC_RAIL_TRIP_RAW
#if AUX_CURRENT_SENSE_ENABLED
      || auxiliaryCurrentRaw <= 5U || auxiliaryCurrentRaw >= ADC_RAIL_TRIP_RAW
#endif
#if SOURCE_SENSE_ENABLED
      || sourceRaw >= ADC_RAIL_TRIP_RAW
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
      || sourceCurrentRaw <= 5U || sourceCurrentRaw >= ADC_RAIL_TRIP_RAW
#endif
      ) {
    latchFault(FAULT_ADC_RAIL);
    return;
  }

  const int32_t voltageInstant = adcToVoltageCenti(voltageRaw);
  const int32_t currentSignedInstant = adcToCurrentCenti(currentRaw);
  if (currentSignedInstant < -ACS712_REVERSE_CURRENT_TRIP_CENTI) {
    latchFault(FAULT_REVERSE_CURRENT);
    return;
  }
  const int32_t currentInstant = (currentSignedInstant > 0L)
                               ? currentSignedInstant : 0L;
#if AUX_CURRENT_SENSE_ENABLED
  const int32_t auxiliarySignedInstant =
      adcToAuxiliaryCurrentCenti(auxiliaryCurrentRaw);
  if (auxiliarySignedInstant < -ACS712_REVERSE_CURRENT_TRIP_CENTI) {
    latchFault(FAULT_REVERSE_CURRENT);
    return;
  }
  auxiliaryCurrentInstantCenti = (auxiliarySignedInstant > 0L)
                               ? auxiliarySignedInstant : 0L;
  if (auxiliaryCurrentInstantCenti >= AUX_AUX_HARD_OVERCURRENT_CENTI) {
    latchFault(FAULT_AUX_OVERCURRENT);
    return;
  }
#else
  auxiliaryCurrentInstantCenti = 0L;
#endif
#if SOURCE_SENSE_ENABLED
  const int32_t sourceInstant = adcToSourceCenti(sourceRaw);
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
  const int32_t sourceCurrentSignedInstant =
      adcToSourceCurrentCenti(sourceCurrentRaw);
  if (sourceCurrentSignedInstant < -ACS712_REVERSE_CURRENT_TRIP_CENTI) {
    latchFault(FAULT_REVERSE_CURRENT);
    return;
  }
  const int32_t sourceCurrentInstant =
      (sourceCurrentSignedInstant > 0L) ? sourceCurrentSignedInstant : 0L;
  if (sourceCurrentInstant >= HARD_OVERCURRENT_CENTI) {
    latchFault(FAULT_OVERCURRENT);
    return;
  }
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
#if SOURCE_CURRENT_SENSE_ENABLED
    sourceCurrentFilteredQ8 = sourceCurrentInstant * 256L;
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
#if SOURCE_CURRENT_SENSE_ENABLED
    (void)filterQ8(sourceCurrentInstant, sourceCurrentFilteredQ8);
#endif
  }
#if SOURCE_CURRENT_SENSE_ENABLED
  const int32_t sourceCurrentCenti = sourceCurrentFilteredQ8 / 256L;
#endif

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
    const int16_t auxiliaryDriveInput =
#if AUX_PATH_ARMED
        auxiliaryIdentificationBaseReady
            ? toQ14(previousAuxiliaryDutyCenteredBp, AUX_MODEL_DUTY_SCALE_BP) : 0;
#else
        0;
#endif
    const int16_t currentSourceOutput = sourceOutputIncrementQ14(
        previousDutyLevelBp, deltaSource, deltaVoltage);
    const int16_t currentPhi[MODEL_FEATURE_COUNT] = {
      toQ14(previousDeltaCurrentCenti, MODEL_I_SCALE_CENTI),
      toQ14(previousPreviousDeltaCurrentCenti, MODEL_I_SCALE_CENTI),
      currentDriveInput,
      auxiliaryDriveInput,
      currentSourceOutput
    };
    const int16_t voltageInputExcitation = toQ14(
        previousCurrentCenteredCenti, MODEL_CURRENT_DEVIATION_CENTI);
    const int16_t voltageOtherExcitation = toQ14(
        deltaSource, MODEL_SOURCE_SCALE_CENTI);
    const int16_t voltagePhi[MODEL_FEATURE_COUNT] = {
      toQ14(previousDeltaVoltageCenti, MODEL_V_SCALE_CENTI),
      voltageInputExcitation,
      voltageOtherExcitation,
      0,
      0
    };
    const bool updateSlowModels = (slowRlsPhase == 0U);
    slowRlsPhase = (uint8_t)((slowRlsPhase + 1U) &
                             (SLOW_RLS_UPDATE_DIVIDER - 1U));
    updateDualModel(currentModel, currentPhi,
                    toQ14(deltaCurrent, MODEL_I_SCALE_CENTI),
                    updateSlowModels);
    updateDualModel(voltageModel, voltagePhi,
                    toQ14(deltaVoltage, MODEL_V_SCALE_CENTI),
                    updateSlowModels);
  }

  const bool knownControlExcitation =
      identificationProbeActive || previousAuxiliaryProbeActive;
  const bool identificationTransient = !knownControlExcitation &&
      (absI32(deltaCurrent) >= CURRENT_STEP_TRANSIENT_CENTI ||
       currentModel.fast.residualAbsQ14 >= RLS_RESIDUAL_TRANSIENT_Q14 ||
       voltageModel.fast.residualAbsQ14 >= RLS_RESIDUAL_TRANSIENT_Q14 ||
       modelDivergenceQ14(currentModel.fast, currentModel.slow) >=
           RLS_PARAMETER_DIVERGENCE_Q14 ||
       modelDivergenceQ14(voltageModel.fast, voltageModel.slow) >=
           RLS_PARAMETER_DIVERGENCE_Q14);
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
#if AUX_EFFICIENCY_LEARNING_ENABLED
  recordEfficiencyMeasurement(sourceCenti, sourceCurrentCenti,
                               voltageCenti, currentCenti, voltageError,
                               deltaCurrent, deltaSource,
                               identificationProbeActive ||
                                   previousAuxiliaryProbeActive,
                               currentInstant >= (CURRENT_LIMIT_CENTI *
                                   CURRENT_GUARD_HARD_PERCENT) / 100L);
#endif
  const RlsModel *selectedCurrentModel = selectModel(
      currentModel, useFastIdentification, 2U, knownControlExcitation);
  const RlsModel *selectedVoltageModel = selectModel(
      voltageModel, useFastIdentification, 1U, knownControlExcitation);
  updateLoadEstimate(selectedVoltageModel);
  const bool fastAuxiliaryModelReady =
      modelHasBothCurrentActuators(currentModel.fast);
  const bool slowAuxiliaryModelReady =
      modelHasBothCurrentActuators(currentModel.slow);
  const bool auxiliaryModelReady =
      (fastAuxiliaryModelReady || slowAuxiliaryModelReady) &&
      auxiliaryIdentificationBaseReady;
  if (auxiliaryModelReady && !useFastIdentification) {
    if (slowAuxiliaryModelReady)
      selectedCurrentModel = &currentModel.slow;
    else
      selectedCurrentModel = &currentModel.fast;
  }
  updateAuxiliaryLowPowerState(currentInstant, voltageError);
#if AUX_PATH_ARMED
  if (loadEstimateValid && estimatedLoadCurrentCenti > AUX_LOW_POWER_MAX_LOAD_CENTI)
    auxiliaryLowPowerActive = false;
#endif

  if (previousMeasurementsValid) {
    activePattern = classifyPattern(voltageError, currentInstant, deltaCurrent,
                                    deltaSource, loadEstimateDeltaCenti,
                                    selectedCurrentModel, selectedVoltageModel,
                                    auxiliaryModelReady,
                                    identificationProbeActive ||
                                        previousAuxiliaryProbeActive);
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
                                 deltaVoltage, deltaCurrent,
                                 previousDeltaCurrentCenti, deltaSource,
                                 sourceCenti, appliedDutyBp, dutyMeanBp,
                                 appliedAuxiliaryDutyBp, auxiliaryDutyMeanBp,
                                 activePattern, resolution.candidateMask,
                                 (AUX_PATH_ARMED != 0) && auxiliaryLowPowerActive &&
                                 auxiliaryModelReady);
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
                               deltaVoltage, deltaCurrent,
                               previousDeltaCurrentCenti, deltaSource,
                               sourceCenti, appliedDutyBp, dutyMeanBp,
                               appliedAuxiliaryDutyBp, auxiliaryDutyMeanBp,
                               activePattern,
                               (AUX_PATH_ARMED != 0) && auxiliaryLowPowerActive &&
                               auxiliaryModelReady);
      if (candidateSearchCountdown > 0U) --candidateSearchCountdown;
      if (!best.valid) selectedCandidateAvailable = false;
    }
  } else {
    selectedCandidateAvailable = false;
  }
  previousResolutionStrategy = (uint8_t)activeResolutionStrategy;

  int32_t selectedDuty = bootstrapDutyLimited;
  int32_t selectedAuxiliaryDuty = limitSlew(0L, appliedAuxiliaryDutyBp,
                                             AUX_DISABLE_SLEW_BP);
  commandedCurrentReferenceCenti = bootstrapIref;
  if (best.valid) {
    selectedDuty = best.dutyBasisPoints;
    selectedAuxiliaryDuty = best.auxiliaryDutyBasisPoints;
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

  // Auxiliary identification is deliberately a separate, low-power phase.
  // Hold the primary PWM fixed and slowly bring the low-L branch toward the
  // primary operating duty, then inject a small zero-mean auxiliary-only PRBS.
  // The aggregate current/voltage response identifies current-model theta[3].
  // This is only enabled by AUX_PATH_ARMED and requires external
  // cycle-by-cycle current protection because the auxiliary inductor is small.
  bool auxiliaryIdentificationActive = false;
#if AUX_PATH_ARMED
  const bool auxiliaryProbeWindow =
      (activeResolutionStrategy == RESOLUTION_AUX_IDENTIFY) &&
      (resolution.allowAuxiliaryProbe != 0U) &&
      auxiliaryLowPowerActive &&
      (currentInstant <= AUX_IDENT_MAX_TOTAL_CURRENT_CENTI) &&
      (absI32(voltageError) <= AUX_LOW_POWER_VOLTAGE_ERROR_CENTI) &&
      (transientHoldCounter == 0U) && !currentGuardActive;
  if (auxiliaryProbeWindow) {
    selectedDuty = appliedDutyBp;
    if (!auxiliaryIdentificationBaseReady) {
      selectedAuxiliaryDuty = limitSlew(appliedDutyBp,
          appliedAuxiliaryDutyBp, AUX_IDENT_RAMP_BP_PER_TICK);
      selectedAuxiliaryDuty = clampI32(selectedAuxiliaryDuty, 0L,
                                        AUX_MAX_DUTY_BASIS_POINTS);
      if (absI32(selectedAuxiliaryDuty - appliedDutyBp) <=
          AUX_IDENT_BASE_TOLERANCE_BP) {
        auxiliaryIdentificationBaseReady = true;
        auxiliaryProbeDivider = 0U;
      }
    } else {
      if (auxiliaryProbeDivider < AUX_IDENT_PRBS_HOLD_TICKS) {
        ++auxiliaryProbeDivider;
      } else {
        auxiliaryProbeDivider = 0U;
        const uint8_t feedback = (uint8_t)(((auxiliaryProbeLfsr >> 6) ^
                                             (auxiliaryProbeLfsr >> 5)) & 1U);
        auxiliaryProbeLfsr = (uint8_t)(((auxiliaryProbeLfsr << 1) | feedback) & 0x7FU);
        if (auxiliaryProbeLfsr == 0U) auxiliaryProbeLfsr = 0x6DU;
        auxiliaryProbeSign = ((auxiliaryProbeLfsr & 1U) != 0U) ? 1 : -1;
      }
      const int32_t probeOffset = (auxiliaryProbeSign > 0)
          ? (int32_t)AUX_IDENT_PRBS_BP
          : (int32_t)-AUX_IDENT_PRBS_BP;
      const int32_t probeTarget = clampI32(
          appliedDutyBp + probeOffset, 0L,
          (int32_t)AUX_MAX_DUTY_BASIS_POINTS);
      selectedAuxiliaryDuty = limitSlew(probeTarget,
          appliedAuxiliaryDutyBp, AUX_DUTY_SLEW_BP);
    }
    auxiliaryIdentificationActive = true;
  }
#endif

  // Gated zero-mean PRBS duty perturbation supplies persistent excitation
  // when the plant models are uncertain and the power stage is near regulation.
  // It is disabled during transients, above-reference operation and near limits.
  identificationProbeActive = false;
#if IDENTIFICATION_PROBE_ENABLED
  const bool probeModelsNeedExcitation =
      !modelHasPrimaryCurrentConfidence(currentModel.fast) ||
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
      !auxiliaryIdentificationActive;
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
    selectedAuxiliaryDuty = 0L;
#if AUX_PATH_ARMED
    auxiliaryLowPowerActive = false;
#endif
    if (currentInstant >= guardZeroCenti) selectedDuty = 0L;
    currentIntegratorQ16 = 0L;
  }

  // One-quadrant converter: above reference, cease positive power demand.
  if (voltageError < -VOLTAGE_ERROR_DEADBAND_CENTI) {
    selectedDuty = 0L;
    selectedAuxiliaryDuty = 0L;
    auxiliaryLowPowerActive = false;
    commandedCurrentReferenceCenti = 0L;
    voltageIntegratorQ16 = 0L;
    currentIntegratorQ16 = 0L;
  }

#if AUX_CURRENT_SENSE_ENABLED
  if (auxiliaryCurrentInstantCenti >= AUX_AUX_CURRENT_LIMIT_CENTI)
    selectedAuxiliaryDuty = 0L;
#endif
#if AUX_PATH_ARMED
  selectedAuxiliaryDuty = clampI32(selectedAuxiliaryDuty, 0L,
                                    AUX_MAX_DUTY_BASIS_POINTS);
#else
  selectedAuxiliaryDuty = 0L;
#endif
  selectedDuty = clampI32(selectedDuty, MIN_DUTY_BASIS_POINTS,
                          MAX_DUTY_BASIS_POINTS);
  writeDutyBasisPoints(selectedDuty, selectedAuxiliaryDuty);
  candidateOutcomePending = best.valid && !identificationProbeActive &&
                            !auxiliaryIdentificationActive &&
                            !currentGuardActive &&
                            (voltageError >= -VOLTAGE_ERROR_DEADBAND_CENTI) &&
                            (appliedDutyBp == best.dutyBasisPoints) &&
                            (appliedAuxiliaryDutyBp == best.auxiliaryDutyBasisPoints);
  if (candidateOutcomePending) {
    pendingCandidateIndex = best.candidateIndex;
    pendingCandidateClass = candidateClassForPattern(activePattern);
    pendingPredictedVoltageStepCenti = best.predictedNextVoltageDeltaCenti;
    pendingPredictedCurrentStepCenti = best.predictedCurrentDeltaCenti;
  }
#if AUX_PATH_ARMED
  if (!auxiliaryLowPowerActive &&
      appliedAuxiliaryDutyBp <= AUX_IDENT_BASE_TOLERANCE_BP) {
    auxiliaryIdentificationBaseReady = false;
    auxiliaryProbeDivider = 0U;
  }
#endif
  previousAuxiliaryProbeActive = auxiliaryIdentificationActive &&
                                 (appliedAuxiliaryDutyBp > 0L);
  previousDutyLevelBp = appliedDutyBp;
  dutyMeanQ16 = (int32_t)((int64_t)dutyMeanQ16 +
      ((((int64_t)appliedDutyBp * 65536LL) - dutyMeanQ16) /
       DUTY_MEAN_TAU_SAMPLES));
  dutyMeanBp = dutyMeanQ16 / 65536L;
  previousDutyCenteredBp = appliedDutyBp - dutyMeanBp;
  auxiliaryDutyMeanQ16 = (int32_t)((int64_t)auxiliaryDutyMeanQ16 +
      ((((int64_t)appliedAuxiliaryDutyBp * 65536LL) - auxiliaryDutyMeanQ16) /
       DUTY_MEAN_TAU_SAMPLES));
  auxiliaryDutyMeanBp = auxiliaryDutyMeanQ16 / 65536L;
  previousAuxiliaryDutyCenteredBp = appliedAuxiliaryDutyBp - auxiliaryDutyMeanBp;

  if (previousMeasurementsValid) {
    previousDeltaVoltageCenti = deltaVoltage;
    previousPreviousDeltaCurrentCenti = previousDeltaCurrentCenti;
    previousDeltaCurrentCenti = deltaCurrent;
  } else {
    previousDeltaVoltageCenti = 0L;
    previousPreviousDeltaCurrentCenti = 0L;
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
  pinMode(AUX_PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  digitalWrite(AUX_PWM_PIN, LOW);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);
  pinMode(ACS712_ZERO_INTERLOCK_PIN, INPUT_PULLUP);
#if AUX_CURRENT_SENSE_ENABLED
  pinMode(AUX_CURRENT_PIN, INPUT);
#endif
#if POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED
#if POWER_STAGE_HARDWARE_FAULT_ACTIVE_LOW
  pinMode(POWER_STAGE_HARDWARE_FAULT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(POWER_STAGE_HARDWARE_FAULT_PIN),
                  powerStageHardwareFaultISR, FALLING);
  // A fault can already be asserted before attachInterrupt() is installed.
  if (digitalRead(POWER_STAGE_HARDWARE_FAULT_PIN) == LOW)
    powerStageHardwareFaultLatched = 1U;
#else
  pinMode(POWER_STAGE_HARDWARE_FAULT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(POWER_STAGE_HARDWARE_FAULT_PIN),
                  powerStageHardwareFaultISR, RISING);
  if (digitalRead(POWER_STAGE_HARDWARE_FAULT_PIN) == HIGH)
    powerStageHardwareFaultLatched = 1U;
#endif
#endif
#if SOURCE_SENSE_ENABLED
  pinMode(SOURCE_VOLTAGE_PIN, INPUT);
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
  pinMode(SOURCE_CURRENT_PIN, INPUT);
#endif
  analogReference(DEFAULT);
  configurePwm();
  writeDutyBasisPoints(0L, 0L);
  if (!calibrateAllCurrentSensors()) {
    latchFault(FAULT_SENSOR_CALIBRATION);
    return;
  }
  resetDualModel(currentModel);
  resetDualModel(voltageModel);

  lastControlUs = micros();
  trendWindowStartUs = lastControlUs;
  crossingWindowStartUs = lastControlUs;
}

void loop() {
  if (!currentSensorsCalibrated && faultCode == FAULT_NONE) {
    latchFault(FAULT_SENSOR_CALIBRATION);
    return;
  }
#if POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED
  if (powerStageHardwareFaultLatched != 0U && faultCode == FAULT_NONE)
    latchFault(FAULT_HARDWARE_TRIP);
#endif
  if (faultCode != FAULT_NONE) {
    writeDutyBasisPoints(0L, 0L);
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
