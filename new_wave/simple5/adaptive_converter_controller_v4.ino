#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#if defined(__AVR__)
#include <avr/io.h>
#include <avr/pgmspace.h>
#elif !defined(PROGMEM)
#define PROGMEM
#endif

/*
 * Online-identifying cascaded buck controller with two overlapping power paths.
 *
 * ATmega328P/Uno:
 *   D9/OC1A primary high-inductance buck path, 20 kHz.
 *   D3/OC2B auxiliary low-inductance path, 80 kHz.
 *   A0 output voltage divider; A1 output ACS712-05B; A2 input-voltage divider;
 *      A3 shared-heatsink NTC; A4 input ACS712-05B.
 *   D2 wired-OR fast hardware fault status from output/input current
 *      comparators and output overvoltage comparator; hardware inhibits gates.
 *   A5 optional output-current-limit potentiometer. A6, when present (Nano),
 *      optionally adjusts input-current limit; on Uno, a 74HC4051 can multiplex
 *      two pots onto A5 with D5 as select (LIMIT_POT_MUX_ENABLED=1; S1/S2 low,
 *      inhibit low, Y0/Y1 to pot wipers, Z to A5).
 *   D4 physical zero-current interlock; both converter input and output must
 *      be isolated/verified at zero current during ACS712 offset calibration.
 *
 * Calibration:
 *   Dual-slot EEPROM records store output/input ACS712 zero offsets and gains,
 *   output/input voltage scales, sequence, version and CRC. The first-boot
 *   10-ohm load calibration uses DMM readings for Vout, Vin and input current;
 *   the known resistor derives output current. Normal boots use the committed
 *   record and do not repeat the load test.
 *
 * Power-path scheduling:
 *   Both branches have an intentional overlap region. The auxiliary branch
 *   may operate alone only below a configurable virtual primary-duty threshold
 *   and only in the low-current, identified operating region. Output and input
 *   ACS712-05B sensors provide independent software current limits. The input
 *   limiter backs off requested power independently; it does not overwrite the
 *   output/thermal current setting. Fast hardware comparators protect source,
 *   output and each inductor branch.
 *
 * Thermal management:
 *   One 10k NTC (B=3950) on the shared heatsink progressively reduces the
 *   allowable output-current limit. An open/short sensor or overtemperature
 *   condition inhibits the power stage.
 *
 * Online identification:
 *   Fast/slow RLS models identify local source, load and actuator dynamics.
 *   A finite flash-resident candidate bank is evaluated against predicted
 *   Lyapunov-proxy decrease, actuator movement, current and voltage limits,
 *   and candidate-specific prediction history. It is not a formal stability
 *   proof. Hardware-in-the-loop validation remains mandatory.
 *
 * The 80 kHz Timer2 output does not imply a MOSFET is safe to drive directly
 * from an ATmega pin. Gate charge, source/sink current, edge time, dead time,
 * switching loss and temperature must be measured before enabling the branch.
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
#define AUX_CURRENT_SENSE_ENABLED       0   // Deliberately removed: one aggregate output-current ACS712 only
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
#define POWER_STAGE_HARDWARE_FAULT_PIN  2   // Uno INT0; wired-OR branch fast-current trips and output OVP
#ifndef PRIMARY_FAST_CURRENT_PROTECTION_PRESENT
#define PRIMARY_FAST_CURRENT_PROTECTION_PRESENT 0 // Independent cycle-by-cycle comparator directly inhibits primary gate
#endif
#ifndef INPUT_FAST_CURRENT_PROTECTION_PRESENT
#define INPUT_FAST_CURRENT_PROTECTION_PRESENT 0 // Independent input/source current comparator directly inhibits all power gates
#endif
#define AUX_HARDWARE_FAULT_PIN          POWER_STAGE_HARDWARE_FAULT_PIN
#define THERMISTOR_PIN                  A3  // Shared heatsink 10k NTC input
#define VOLTAGE_PIN                     A0  // Output voltage
#define CURRENT_PIN                     A1  // Output current ACS712-05B
#ifndef SOURCE_SENSE_ENABLED
#define SOURCE_SENSE_ENABLED            1   // Required for input power and efficiency measurement
#endif
#ifndef SOURCE_CURRENT_SENSE_ENABLED
#define SOURCE_CURRENT_SENSE_ENABLED    1   // Input current ACS712-05B on A4
#endif
#ifndef AUX_EFFICIENCY_LEARNING_ENABLED
#define AUX_EFFICIENCY_LEARNING_ENABLED 1   // Uses measured Vin*Iin and Vout*Iout; runtime-gated by valid calibration
#endif
#define SOURCE_VOLTAGE_PIN              A2  // Input voltage
#define SOURCE_CURRENT_PIN              A4  // Input current ACS712-05B

// Optional current-limit potentiometers are disabled unless explicitly wired
// and enabled below; floating ADC pins must never adjust safety settings.
// Direct two-pot wiring (Nano): set OUTPUT_LIMIT_POT_PIN=A5 and INPUT_LIMIT_POT_PIN=A6.
// Uno two-pot wiring: set LIMIT_POT_MUX_ENABLED=1 and use a 74HC4051 on A5/D5.
// One-pot wiring: assign only the desired limit pin; leave the other at -1.
// On an Uno, A0..A4 are occupied by sensors, so A5 is the only spare ADC input.
#ifndef LIMIT_POT_MUX_ENABLED
#define LIMIT_POT_MUX_ENABLED           0
#endif
#if LIMIT_POT_MUX_ENABLED
#ifndef LIMIT_POT_MUX_ADC_PIN
#define LIMIT_POT_MUX_ADC_PIN           A5
#endif
#ifndef LIMIT_POT_MUX_SELECT_PIN
#define LIMIT_POT_MUX_SELECT_PIN        5
#endif
#ifndef OUTPUT_LIMIT_POT_PIN
#define OUTPUT_LIMIT_POT_PIN            LIMIT_POT_MUX_ADC_PIN
#endif
#ifndef INPUT_LIMIT_POT_PIN
#define INPUT_LIMIT_POT_PIN             LIMIT_POT_MUX_ADC_PIN
#endif
#else
#ifndef OUTPUT_LIMIT_POT_PIN
#define OUTPUT_LIMIT_POT_PIN            -1  // opt in explicitly to avoid floating ADC inputs
#endif
#ifndef INPUT_LIMIT_POT_PIN
#define INPUT_LIMIT_POT_PIN             -1  // opt in explicitly to avoid floating ADC inputs
#endif
#endif
#if LIMIT_POT_MUX_ENABLED && (OUTPUT_LIMIT_POT_PIN != INPUT_LIMIT_POT_PIN)
#error "LIMIT_POT_MUX_ENABLED requires both pot channels to share the same ADC pin"
#endif
#define LIMIT_POT_SAMPLE_INTERVAL_MS    50UL  // one ADC conversion service every 50 ms
#define LIMIT_POT_FILTER_SHIFT          3U    // slow EMA; independent of 499 Hz control loop
#ifndef OUTPUT_POT_MIN_CENTI
#define OUTPUT_POT_MIN_CENTI            50L
#endif
#ifndef OUTPUT_POT_MAX_CENTI
#define OUTPUT_POT_MAX_CENTI            400L
#endif
#ifndef INPUT_POT_MIN_CENTI
#define INPUT_POT_MIN_CENTI             50L
#endif
#ifndef INPUT_POT_MAX_CENTI
#define INPUT_POT_MAX_CENTI             400L
#endif

// Calibration uses one fixed 10-ohm output resistor.
// Use at least a 25 W resistor for a 12 V / 1.2 A calibration point.
#define CAL_LOAD_RESISTANCE_MILLIOHM    10000L
#define CAL_TARGET_V_CENTI              TARGET_V_CENTI
#define CAL_MIN_DMM_V_CENTI             800L
#define CAL_MAX_DMM_V_CENTI             1400L
#define CAL_EXPECTED_CURRENT_CENTI       ((CAL_TARGET_V_CENTI * 1000L) / CAL_LOAD_RESISTANCE_MILLIOHM)
#define CAL_SAMPLE_COUNT                64U
#define CAL_STABLE_BAND_CENTI           25L
#define CAL_STABLE_TIME_MS              1000UL
#define CAL_TIMEOUT_MS                  30000UL

// Shared heatsink NTC: 10k at 25 C, Beta 3950, 10k pull-up to AVCC,
// thermistor from A3 to GND. All thresholds are heatsink temperatures.
#define THERMISTOR_NOMINAL_OHMS         10000UL
#define THERMISTOR_PULLUP_OHMS          10000UL
#define THERMISTOR_BETA                 3950UL
#define THERMISTOR_OPEN_RAW             1018U
#define THERMISTOR_SHORT_RAW            8U
#define THERMAL_DERATE_START_CENTI      6000L
#define THERMAL_ZERO_POWER_CENTI        8500L
#define THERMAL_TRIP_CENTI              9500L
#define THERMAL_FILTER_SHIFT            3U

// Hysteretic path overlap. Values are basis points: 10000 = 100%.
// The auxiliary path may run alone only below 10% duty and <=1 A total output
// current; it returns to overlap above 12%. The wider 18-22% band lets the
// branches overlap during handover. Duty is a conversion-ratio indicator,
// not a substitute for the aggregate-current low-power condition.
#define AUX_OVERLAP_ENTER_BP            1800L // enter overlap below 18%
#define AUX_OVERLAP_EXIT_BP             2200L // leave overlap above 22%
#define AUX_ONLY_ENTER_BP               1000L // auxiliary-only permitted below 10%
#define AUX_ONLY_EXIT_BP                1200L // return to overlap above 12%
#define AUX_ONLY_MAX_CURRENT_CENTI       100L // enter auxiliary-only at <=1.00 A total
#define AUX_ONLY_EXIT_CURRENT_CENTI      125L // return to overlap above 1.25 A

// A valid EEPROM calibration is used at every normal boot. A calibration
// wizard is entered only when neither EEPROM slot passes validation.
#define EEPROM_CAL_MAGIC                0xC71AU
#define EEPROM_CAL_VERSION              3U
#define EEPROM_CAL_COMMIT               0xA5U
#define EEPROM_CAL_SLOT_COUNT            2U
#define SERIAL_BAUD_RATE                 115200UL

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
#define ACS712_SOURCE_SENSITIVITY_MV_PER_A  185UL // input-current ACS712-05B
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

// D4 must be LOW only when input and output currents are physically zero and isolated.
#define ACS712_ZERO_INTERLOCK_PIN            4
#define ACS712_ZERO_INTERLOCK_ACTIVE_LOW     1
#define ACS712_ZERO_SETTLE_MS                250UL
#define ACS712_ZERO_CAL_SAMPLES              128U
#define ACS712_ZERO_MIN_RAW                  480U
#define ACS712_ZERO_MAX_RAW                  544U
#define ACS712_ZERO_MAX_SPAN_RAW             24U
#define ACS712_ZERO_MAX_STDDEV_SQUARED       36UL
#define ACS712_REVERSE_CURRENT_TRIP_CENTI    25L

// -------------------------- Setpoints/protection --------------------------
#define TARGET_V_CENTI                  1200L
#ifndef OUTPUT_CURRENT_LIMIT_CENTI
#define OUTPUT_CURRENT_LIMIT_CENTI      400L // output/load limit in centiamps
#endif
#ifndef INPUT_CURRENT_LIMIT_CENTI
#define INPUT_CURRENT_LIMIT_CENTI       300L // independent source/input limit in centiamps
#endif
#ifndef OUTPUT_HARD_OVERCURRENT_CENTI
#define OUTPUT_HARD_OVERCURRENT_CENTI   450L // fixed software trip, separate from adjustable setpoint
#endif
#ifndef INPUT_HARD_OVERCURRENT_CENTI
#define INPUT_HARD_OVERCURRENT_CENTI    450L // fixed software trip, separate from adjustable setpoint
#endif
#define CURRENT_LIMIT_CENTI             OUTPUT_CURRENT_LIMIT_CENTI // legacy internal alias
#define HARD_OVERCURRENT_CENTI          OUTPUT_HARD_OVERCURRENT_CENTI
#define INPUT_CURRENT_GUARD_START_PERCENT 80L
#define INPUT_CURRENT_GUARD_HARD_PERCENT  92L
#define INPUT_CURRENT_GUARD_RELEASE_PERCENT 75L
#define INPUT_CURRENT_BRAKE_STEP_BP       200L
#define OUTPUT_OVERVOLTAGE_CENTI        1300L
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
#if !SOURCE_CURRENT_SENSE_ENABLED
#error "The input ACS712 channel is required for independent source-current limiting"
#endif
#if OUTPUT_POT_MIN_CENTI <= 0L || OUTPUT_POT_MAX_CENTI < OUTPUT_POT_MIN_CENTI
#error "Invalid output current potentiometer range"
#endif
#if INPUT_POT_MIN_CENTI <= 0L || INPUT_POT_MAX_CENTI < INPUT_POT_MIN_CENTI
#error "Invalid input current potentiometer range"
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
#if OUTPUT_HARD_OVERCURRENT_CENTI > 450L || INPUT_HARD_OVERCURRENT_CENTI > 450L
#error "Keep both software overcurrent trips at or below 4.50 A with ACS712-05B sensors"
#endif
#if OUTPUT_CURRENT_LIMIT_CENTI > 400L || INPUT_CURRENT_LIMIT_CENTI > 400L
#error "Keep both normal current settings at or below 4.00 A with ACS712-05B sensors"
#endif
#if OUTPUT_HARD_OVERCURRENT_CENTI <= OUTPUT_CURRENT_LIMIT_CENTI || OUTPUT_POT_MAX_CENTI >= OUTPUT_HARD_OVERCURRENT_CENTI
#error "Output hard trip must exceed the full potentiometer operating range"
#endif
#if INPUT_HARD_OVERCURRENT_CENTI <= INPUT_CURRENT_LIMIT_CENTI || INPUT_POT_MAX_CENTI >= INPUT_HARD_OVERCURRENT_CENTI
#error "Input hard trip must exceed the full potentiometer operating range"
#endif
#if INPUT_CURRENT_GUARD_START_PERCENT >= INPUT_CURRENT_GUARD_HARD_PERCENT || INPUT_CURRENT_GUARD_HARD_PERCENT >= 100L
#error "Invalid input-current guard percentages"
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
#if !PRIMARY_FAST_CURRENT_PROTECTION_PRESENT
#error "Primary PWM requires an independent cycle-by-cycle output-current comparator that directly inhibits its MOSFET gate"
#endif
#if !INPUT_FAST_CURRENT_PROTECTION_PRESENT
#error "Input-current protection requires an independent source-current comparator that directly inhibits all power gates"
#endif
#if !POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED
#error "PWM requires D2 hardware fault status; wire-OR input/output fast-current trips and the independent output overvoltage trip"
#endif
#if THERMAL_ZERO_POWER_CENTI <= THERMAL_DERATE_START_CENTI
#error "Thermal zero-power temperature must exceed derating start"
#endif
#if THERMAL_TRIP_CENTI <= THERMAL_ZERO_POWER_CENTI
#error "Thermal trip temperature must exceed zero-power temperature"
#endif
#if AUX_ONLY_EXIT_BP <= AUX_ONLY_ENTER_BP
#error "Auxiliary-only duty hysteresis must be positive"
#endif
#if AUX_ONLY_EXIT_CURRENT_CENTI <= AUX_ONLY_MAX_CURRENT_CENTI || AUX_ONLY_EXIT_CURRENT_CENTI > AUX_LOW_POWER_EXIT_CENTI
#error "Auxiliary-only current hysteresis must fit inside the low-power region"
#endif
#if AUX_OVERLAP_EXIT_BP <= AUX_OVERLAP_ENTER_BP
#error "Auxiliary overlap duty hysteresis must be positive"
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
#if CAL_EXPECTED_CURRENT_CENTI <= 0 || CAL_EXPECTED_CURRENT_CENTI >= HARD_OVERCURRENT_CENTI
#error "The 10-ohm calibration load current must remain below the software hard-current trip"
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
  FAULT_OVERCURRENT, // output current protects the load
  FAULT_INPUT_OVERCURRENT, // input current protects the source
  FAULT_AUX_OVERCURRENT,
  FAULT_SENSOR_CALIBRATION,
  FAULT_REVERSE_CURRENT,
  FAULT_HARDWARE_TRIP,
  FAULT_THERMISTOR,
  FAULT_OVERTEMPERATURE,
  FAULT_EEPROM
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
  int32_t primaryEquivalentDutyBp; // duty the primary path would need on its own
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

// EEPROM record is deliberately compact and versioned. CRC excludes crc16 and
// commit; commit is written last so an interrupted update cannot become valid.
struct __attribute__((packed)) CalibrationRecord {
  uint16_t magic;
  uint8_t version;
  uint8_t length;
  uint16_t sequence;
  int32_t currentZeroRawQ8; // output ACS712 zero
  int32_t currentGainCentiPerCountQ16; // signed output gain
  int32_t sourceCurrentZeroRawQ8; // input ACS712 zero
  int32_t sourceCurrentGainCentiPerCountQ16; // signed input gain
  uint32_t voltageCentiPerCountQ16; // output divider scale
  uint32_t sourceCentiPerCountQ16; // input divider scale
  uint16_t crc16;
  uint8_t commit;
};

enum CalibrationWizardState : uint8_t {
  CAL_WIZARD_READY = 0,
  CAL_WIZARD_WAIT_ZERO,
  CAL_WIZARD_WAIT_LOAD,
  CAL_WIZARD_RAMP,
  CAL_WIZARD_WAIT_DMM,
  CAL_WIZARD_FAILED
};

enum AuxiliaryPathMode : uint8_t {
  AUX_MODE_PRIMARY_ONLY = 0,
  AUX_MODE_OVERLAP,
  AUX_MODE_AUXILIARY_ONLY
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
static int32_t latestOutputCurrentCenti = 0L;
static int32_t latestInputCurrentCenti = 0L;
static uint16_t latestEfficiencyPermille = 0U;
static bool latestEfficiencyValid = false;
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
  if (value == INT32_MIN) return INT32_MAX;
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
static uint32_t nominalVoltageScaleQ16(uint32_t dividerNumerator,
                                             uint32_t dividerDenominator) {
  const uint64_t numerator = (uint64_t)ADC_REFERENCE_MV *
      dividerNumerator * 100ULL * 65536ULL;
  const uint64_t denominator = (uint64_t)ADC_COUNTS * 1000ULL *
      dividerDenominator;
  return (uint32_t)((numerator + denominator / 2ULL) / denominator);
}

static uint32_t voltageCentiPerCountQ16 = 0UL;

static int32_t adcToVoltageCenti(uint16_t raw) {
  return (int32_t)(((uint64_t)raw * voltageCentiPerCountQ16 + 32768ULL) >> 16);
}

// Runtime zero offsets are Q8 ADC counts. Q8 storage preserves fractional
// average-count offsets from the startup calibration window.
static int32_t primaryCurrentZeroRawQ8 = 0L;
static int32_t currentGainCentiPerCountQ16 = 0L;
static int32_t sourceCurrentZeroRawQ8 = 0L;
static int32_t sourceCurrentGainCentiPerCountQ16 = 0L;
static uint32_t sourceCentiPerCountQ16 = 0UL;
static CalibrationRecord calibrationData = {};
static int8_t activeEepromSlot = -1;
static bool currentSensorsCalibrated = false;
static CalibrationWizardState calibrationWizardState = CAL_WIZARD_WAIT_ZERO;
static uint32_t calibrationStateStartMs = 0UL;
static uint32_t calibrationStableStartMs = 0UL;
static int32_t calibrationDutyBp = 0L;
static uint32_t calibrationVoltageRawSum = 0UL;
static uint32_t calibrationCurrentRawSum = 0UL;
#if SOURCE_SENSE_ENABLED
static uint32_t calibrationSourceRawSum = 0UL;
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
static uint32_t calibrationSourceCurrentRawSum = 0UL;
#endif
static uint8_t calibrationSampleCount = 0U;
static int32_t calibrationLastVoltageCenti = -1L;
static char calibrationCommandBuffer[48] = {};
static uint8_t calibrationCommandLength = 0U;
static bool calibrationCommandOverflow = false;
static int32_t thermistorRawFilteredQ8 = 0L;
static int32_t thermistorTemperatureCenti = 2500L;
static int32_t outputCurrentSettingCenti = OUTPUT_CURRENT_LIMIT_CENTI;
static int32_t inputCurrentSettingCenti = INPUT_CURRENT_LIMIT_CENTI;
static int32_t runtimeCurrentLimitCenti = OUTPUT_CURRENT_LIMIT_CENTI; // output setting, thermally derated
static int32_t runtimeInputCurrentLimitCenti = INPUT_CURRENT_LIMIT_CENTI;
static bool inputCurrentGuardActive = false;
static bool thermistorInitialized = false;
static AuxiliaryPathMode auxiliaryPathMode = AUX_MODE_PRIMARY_ONLY;
// Last modelled primary-only duty requirement. It remains meaningful while
// the primary switch is off in auxiliary-only mode.
static int32_t primaryEquivalentDutyDemandBp = 0L;
#if AUX_CURRENT_SENSE_ENABLED
static int32_t auxiliaryCurrentZeroRawQ8 = 0L;
#endif

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
  const int64_t deltaRawQ8 = (int64_t)raw * 256LL -
                              primaryCurrentZeroRawQ8;
  return (int32_t)((deltaRawQ8 * currentGainCentiPerCountQ16) /
                   (256LL * 65536LL));
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
  return (int32_t)(((uint64_t)raw * sourceCentiPerCountQ16 + 32768ULL) >> 16);
}
#endif

#if SOURCE_CURRENT_SENSE_ENABLED
static int32_t adcToSourceCurrentCenti(uint16_t raw) {
  const int64_t deltaRawQ8 = (int64_t)raw * 256LL -
                              sourceCurrentZeroRawQ8;
  return (int32_t)((deltaRawQ8 * sourceCurrentGainCentiPerCountQ16) /
                   (256LL * 65536LL));
}
#endif

static uint16_t calibrationCrc16(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFFU;
  for (uint8_t i = 0U; i < length; ++i) {
    crc = (uint16_t)(crc ^ (uint16_t)((uint16_t)data[i] << 8U));
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                            : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static bool calibrationRecordValid(const CalibrationRecord &record) {
  if (record.magic != EEPROM_CAL_MAGIC ||
      record.version != EEPROM_CAL_VERSION ||
      record.length != sizeof(CalibrationRecord) ||
      record.commit != EEPROM_CAL_COMMIT) return false;

  const uint16_t expectedCrc = calibrationCrc16(
      (const uint8_t *)&record,
      (uint8_t)offsetof(CalibrationRecord, crc16));
  if (expectedCrc != record.crc16) return false;
  if (record.currentZeroRawQ8 <
          (int32_t)ACS712_ZERO_MIN_RAW * 256L ||
      record.currentZeroRawQ8 >
          (int32_t)ACS712_ZERO_MAX_RAW * 256L ||
      record.sourceCurrentZeroRawQ8 <
          (int32_t)ACS712_ZERO_MIN_RAW * 256L ||
      record.sourceCurrentZeroRawQ8 >
          (int32_t)ACS712_ZERO_MAX_RAW * 256L) return false;
  const int32_t gainMagnitude = absI32(record.currentGainCentiPerCountQ16);
  const int32_t inputGainMagnitude =
      absI32(record.sourceCurrentGainCentiPerCountQ16);
  if (gainMagnitude < 50000L || gainMagnitude > 1000000L ||
      inputGainMagnitude < 50000L || inputGainMagnitude > 1000000L) return false;
  if (record.voltageCentiPerCountQ16 < 20000UL ||
      record.voltageCentiPerCountQ16 > 1000000UL) return false;
#if SOURCE_SENSE_ENABLED
  if (record.sourceCentiPerCountQ16 < 20000UL ||
      record.sourceCentiPerCountQ16 > 1000000UL) return false;
#endif
  return true;
}

static bool calibrationSequenceNewer(uint16_t lhs, uint16_t rhs) {
  return (int16_t)(lhs - rhs) > 0;
}

static bool loadCalibrationFromEeprom() {
  CalibrationRecord records[EEPROM_CAL_SLOT_COUNT];
  bool valid[EEPROM_CAL_SLOT_COUNT] = { false, false };
  for (uint8_t slot = 0U; slot < EEPROM_CAL_SLOT_COUNT; ++slot) {
    EEPROM.get((int)(slot * sizeof(CalibrationRecord)), records[slot]);
    valid[slot] = calibrationRecordValid(records[slot]);
  }
  if (!valid[0] && !valid[1]) return false;

  uint8_t selected = 0U;
  if (!valid[0]) selected = 1U;
  else if (valid[1] &&
           calibrationSequenceNewer(records[1].sequence,
                                    records[0].sequence)) selected = 1U;
  calibrationData = records[selected];
  activeEepromSlot = (int8_t)selected;
  primaryCurrentZeroRawQ8 = calibrationData.currentZeroRawQ8;
  currentGainCentiPerCountQ16 =
      calibrationData.currentGainCentiPerCountQ16;
  sourceCurrentZeroRawQ8 = calibrationData.sourceCurrentZeroRawQ8;
  sourceCurrentGainCentiPerCountQ16 =
      calibrationData.sourceCurrentGainCentiPerCountQ16;
  voltageCentiPerCountQ16 = calibrationData.voltageCentiPerCountQ16;
#if SOURCE_SENSE_ENABLED
  sourceCentiPerCountQ16 = calibrationData.sourceCentiPerCountQ16;
#endif
  currentSensorsCalibrated = true;
  return true;
}

static bool saveCalibrationToEeprom() {
  CalibrationRecord record = calibrationData;
  record.magic = EEPROM_CAL_MAGIC;
  record.version = EEPROM_CAL_VERSION;
  record.length = (uint8_t)sizeof(CalibrationRecord);
  record.sequence = (activeEepromSlot < 0)
      ? 1U : (uint16_t)(calibrationData.sequence + 1U);
  record.crc16 = calibrationCrc16(
      (const uint8_t *)&record,
      (uint8_t)offsetof(CalibrationRecord, crc16));
  record.commit = 0U;

  const uint8_t targetSlot = (activeEepromSlot == 0) ? 1U : 0U;
  const int base = (int)(targetSlot * sizeof(CalibrationRecord));
  const uint8_t *bytes = (const uint8_t *)&record;

  // Invalidate destination first. Write payload+CRC, then commit marker last.
  EEPROM.update(base + (int)offsetof(CalibrationRecord, commit), 0U);
  for (uint8_t i = 0U;
       i < (uint8_t)offsetof(CalibrationRecord, commit); ++i) {
    EEPROM.update(base + i, bytes[i]);
  }
  EEPROM.update(base + (int)offsetof(CalibrationRecord, commit),
                EEPROM_CAL_COMMIT);

  CalibrationRecord verify = {};
  EEPROM.get(base, verify);
  if (!calibrationRecordValid(verify)) return false;

  calibrationData = verify;
  activeEepromSlot = (int8_t)targetSlot;
  primaryCurrentZeroRawQ8 = verify.currentZeroRawQ8;
  currentGainCentiPerCountQ16 = verify.currentGainCentiPerCountQ16;
  sourceCurrentZeroRawQ8 = verify.sourceCurrentZeroRawQ8;
  sourceCurrentGainCentiPerCountQ16 =
      verify.sourceCurrentGainCentiPerCountQ16;
  voltageCentiPerCountQ16 = verify.voltageCentiPerCountQ16;
#if SOURCE_SENSE_ENABLED
  sourceCentiPerCountQ16 = verify.sourceCentiPerCountQ16;
#endif
  currentSensorsCalibrated = true;
  return true;
}

static void initialiseDefaultCalibrationScales() {
  voltageCentiPerCountQ16 =
      nominalVoltageScaleQ16(VOLTAGE_DIVIDER_NUM, VOLTAGE_DIVIDER_DEN);
  sourceCentiPerCountQ16 =
      nominalVoltageScaleQ16(SOURCE_DIVIDER_NUM, SOURCE_DIVIDER_DEN);
  // ACS712-05B nominal sensitivity is 185 mV/A; this is only a provisional
  // gain for the calibration ramp, replaced by the 10-ohm/DMM result.
  const uint64_t numerator = (uint64_t)ADC_REFERENCE_MV *
                             100ULL * 65536ULL;
  const uint64_t denominator = (uint64_t)ADC_COUNTS *
                               ACS712_PRIMARY_SENSITIVITY_MV_PER_A;
  currentGainCentiPerCountQ16 =
      (int32_t)((numerator + denominator / 2ULL) / denominator);
  sourceCurrentGainCentiPerCountQ16 =
      (int32_t)((numerator + denominator / 2ULL) / denominator);
}

// ADC table for a 10k NTC, Beta 3950, 10k pull-up to AVCC.
// Table points are -20 C through +120 C in 10 C increments.
static const uint16_t THERMISTOR_RAW_TABLE[] PROGMEM = {
  934U, 873U, 788U, 684U, 569U, 456U, 354U, 270U,
  204U, 153U, 115U, 87U, 67U, 51U, 40U
};

static uint16_t thermistorTableRaw(uint8_t index) {
#if defined(__AVR__)
  return pgm_read_word(&THERMISTOR_RAW_TABLE[index]);
#else
  return THERMISTOR_RAW_TABLE[index];
#endif
}

static bool thermistorRawToCentiC(uint16_t raw, int32_t &temperatureCenti) {
  if (raw <= THERMISTOR_SHORT_RAW || raw >= THERMISTOR_OPEN_RAW)
    return false;
  const uint16_t coldest = thermistorTableRaw(0U);
  const uint16_t hottest = thermistorTableRaw(14U);
  if (raw >= coldest) {
    temperatureCenti = -2000L;
    return true;
  }
  if (raw <= hottest) {
    temperatureCenti = 12000L;
    return true;
  }
  for (uint8_t i = 0U; i < 14U; ++i) {
    const uint16_t r0 = thermistorTableRaw(i);
    const uint16_t r1 = thermistorTableRaw((uint8_t)(i + 1U));
    if (raw <= r0 && raw >= r1) {
      const int32_t fraction = (int32_t)(
          ((int64_t)(r0 - raw) * 1000LL) / (int32_t)(r0 - r1));
      temperatureCenti = (int32_t)(-2000LL +
          (int64_t)i * 1000LL + fraction);
      return true;
    }
  }
  return false;
}

static bool zeroCalibrationInterlockActive() {
#if ACS712_ZERO_INTERLOCK_ACTIVE_LOW
  return digitalRead(ACS712_ZERO_INTERLOCK_PIN) == LOW;
#else
  return digitalRead(ACS712_ZERO_INTERLOCK_PIN) == HIGH;
#endif
}

// ACS712 zero calibration. This only runs with PWM disabled and current isolated.
static bool calibrateAcs712Zero(uint8_t analogPin, int32_t &zeroRawQ8) {
  for (uint8_t settle = 0U; settle < 4U; ++settle)
    (void)analogRead(analogPin);

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
  const uint64_t varianceNumerator = sumSquares * n -
                                      (uint64_t)sum * sum;
  const uint64_t varianceLimit =
      (uint64_t)ACS712_ZERO_MAX_STDDEV_SQUARED * n * n;
  if (varianceNumerator > varianceLimit) return false;

  zeroRawQ8 = (int32_t)(((uint64_t)sum * 256ULL + n / 2ULL) / n);
  return true;
}

static bool zeroOffsetPlausibleOnPin(uint8_t pin, int32_t expectedZeroRawQ8) {
  uint32_t sum = 0UL;
  uint16_t minimum = 1023U;
  uint16_t maximum = 0U;
  for (uint8_t i = 0U; i < 32U; ++i) {
    const uint16_t raw = (uint16_t)analogRead(pin);
    if (raw <= 5U || raw >= ADC_RAIL_TRIP_RAW) return false;
    sum += raw;
    if (raw < minimum) minimum = raw;
    if (raw > maximum) maximum = raw;
  }
  const int32_t meanQ8 = (int32_t)(
      ((uint64_t)sum * 256ULL + 16ULL) / 32ULL);
  return absI32(meanQ8 - expectedZeroRawQ8) <= (8L * 256L) &&
         (uint16_t)(maximum - minimum) <= 20U;
}

static bool storedZeroOffsetPlausible() {
  if (!zeroOffsetPlausibleOnPin(CURRENT_PIN, primaryCurrentZeroRawQ8))
    return false;
#if SOURCE_CURRENT_SENSE_ENABLED
  if (!zeroOffsetPlausibleOnPin(SOURCE_CURRENT_PIN,
                                sourceCurrentZeroRawQ8)) return false;
#endif
  return true;
}

static void resetDualModel(DualModel &model);

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

static bool updateThermalLimit(uint16_t thermistorRaw) {
  if (thermistorRaw <= THERMISTOR_SHORT_RAW ||
      thermistorRaw >= THERMISTOR_OPEN_RAW) return false;

  if (!thermistorInitialized) {
    thermistorRawFilteredQ8 = (int32_t)thermistorRaw * 256L;
    thermistorInitialized = true;
  } else {
    const int32_t targetQ8 = (int32_t)thermistorRaw * 256L;
    thermistorRawFilteredQ8 +=
        (targetQ8 - thermistorRawFilteredQ8) >> THERMAL_FILTER_SHIFT;
  }

  const uint16_t filteredRaw =
      (uint16_t)clampI32(thermistorRawFilteredQ8 / 256L, 0L, 1023L);
  if (!thermistorRawToCentiC(filteredRaw, thermistorTemperatureCenti))
    return false;

  if (thermistorTemperatureCenti >= THERMAL_TRIP_CENTI) {
    latchFault(FAULT_OVERTEMPERATURE);
    return false;
  }

  if (thermistorTemperatureCenti <= THERMAL_DERATE_START_CENTI) {
    runtimeCurrentLimitCenti = outputCurrentSettingCenti;
  } else if (thermistorTemperatureCenti >= THERMAL_ZERO_POWER_CENTI) {
    runtimeCurrentLimitCenti = 0L;
  } else {
    const int32_t remaining =
        THERMAL_ZERO_POWER_CENTI - thermistorTemperatureCenti;
    const int32_t span =
        THERMAL_ZERO_POWER_CENTI - THERMAL_DERATE_START_CENTI;
    runtimeCurrentLimitCenti = (int32_t)(
        ((int64_t)outputCurrentSettingCenti * remaining) / span);
  }
  return true;
}

static void printCalibrationInstructions() {
  Serial.println(F("CALIBRATION REQUIRED: PWM outputs are disabled."));
  Serial.println(F("1) Isolate BOTH source input and converter output; verify zero current in both ACS712 conductors."));
  Serial.println(F("2) Assert D4 interlock (must confirm both paths isolated), then send ZERO."));
  Serial.println(F("3) Connect a measured 10.00-ohm resistor rated >=25 W (50 W preferred)."));
  Serial.println(F("4) Release D4 interlock and send LOAD."));
  Serial.println(F("5) At stable load, measure Vout, Vin and source input current with DMMs."));
  Serial.println(F("6) Send: V=1200 VIN=2400 IIN=70 (all in centivolts / centiamps)."));
  Serial.println(F("10.00 ohm load derives output current; IIN DMM calibrates input ACS712 gain."));
  Serial.println(F("Keep the input-current DMM in series at the source. ZERO calibration requires BOTH paths isolated."));
  Serial.println(F("Complete the LOAD/DMM stage within 30 s."));
}

static void resetCalibrationAverages() {
  calibrationVoltageRawSum = 0UL;
  calibrationCurrentRawSum = 0UL;
#if SOURCE_SENSE_ENABLED
  calibrationSourceRawSum = 0UL;
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
  calibrationSourceCurrentRawSum = 0UL;
#endif
  calibrationSampleCount = 0U;
}

static bool parsePositiveCentivolts(const char *text, int32_t &value) {
  if (text == 0 || *text == '\0') return false;
  char *end = 0;
  const long parsed = strtol(text, &end, 10);
  if (end == text || *end != '\0') return false;
  if (parsed < CAL_MIN_DMM_V_CENTI || parsed > CAL_MAX_DMM_V_CENTI)
    return false;
  value = (int32_t)parsed;
  return true;
}

#if SOURCE_SENSE_ENABLED
static bool parseSourceCentivolts(const char *text, int32_t &value) {
  if (text == 0 || *text == '\0') return false;
  char *end = 0;
  const long parsed = strtol(text, &end, 10);
  if (end == text || *end != '\0') return false;
  if (parsed < 100L || parsed > 6000L) return false;
  value = (int32_t)parsed;
  return true;
}
#endif

static void handleCalibrationCommand(char *command) {
  if (strcmp(command, "STATUS") == 0) {
    Serial.print(F("cal_state="));
    Serial.print((int)calibrationWizardState);
    Serial.print(F(" calibrated="));
    Serial.print(currentSensorsCalibrated ? 1 : 0);
    Serial.print(F(" temperature_cC="));
    Serial.print(thermistorTemperatureCenti);
    Serial.print(F(" iout_cA="));
    Serial.print(latestOutputCurrentCenti);
    Serial.print(F(" iin_cA="));
    Serial.print(latestInputCurrentCenti);
    Serial.print(F(" out_limit_cA="));
    Serial.print(runtimeCurrentLimitCenti);
    Serial.print(F(" in_limit_cA="));
    Serial.print(runtimeInputCurrentLimitCenti);
    Serial.print(F(" efficiency_permille="));
    Serial.print(latestEfficiencyValid ? (int)latestEfficiencyPermille : -1);
    Serial.println();
    return;
  }

  if (strcmp(command, "RECAL") == 0 &&
      calibrationWizardState == CAL_WIZARD_READY) {
    writeDutyBasisPoints(0L, 0L);
    currentSensorsCalibrated = false;
    calibrationWizardState = CAL_WIZARD_WAIT_ZERO;
    calibrationStateStartMs = (uint32_t)millis();
    printCalibrationInstructions();
    return;
  }

  if (calibrationWizardState == CAL_WIZARD_WAIT_ZERO &&
      strcmp(command, "ZERO") == 0) {
    writeDutyBasisPoints(0L, 0L);
    if (!zeroCalibrationInterlockActive()) {
      Serial.println(F("ZERO rejected: D4 interlock is not asserted."));
      return;
    }
    delay(ACS712_ZERO_SETTLE_MS);
    if (!calibrateAcs712Zero(CURRENT_PIN, primaryCurrentZeroRawQ8)) {
      Serial.println(F("ZERO rejected: output ACS712 offset/noise/interlock check failed."));
      return;
    }
#if SOURCE_CURRENT_SENSE_ENABLED
    if (!calibrateAcs712Zero(SOURCE_CURRENT_PIN, sourceCurrentZeroRawQ8)) {
      Serial.println(F("ZERO rejected: input ACS712 offset/noise/interlock check failed."));
      return;
    }
#endif
    initialiseDefaultCalibrationScales();
    calibrationData.currentZeroRawQ8 = primaryCurrentZeroRawQ8;
    calibrationData.sourceCurrentZeroRawQ8 = sourceCurrentZeroRawQ8;
    calibrationWizardState = CAL_WIZARD_WAIT_LOAD;
    calibrationStateStartMs = (uint32_t)millis();
    Serial.println(F("Zero captured. Connect the 10-ohm load, release D4, then send LOAD."));
    return;
  }

  if (calibrationWizardState == CAL_WIZARD_WAIT_LOAD &&
      strcmp(command, "LOAD") == 0) {
    if (zeroCalibrationInterlockActive()) {
      Serial.println(F("LOAD rejected: release D4 interlock before energising."));
      return;
    }
    calibrationDutyBp = 0L;
    calibrationStableStartMs = 0UL;
    calibrationLastVoltageCenti = -1L;
    resetCalibrationAverages();
    calibrationStateStartMs = (uint32_t)millis();
    lastControlUs = (uint32_t)micros();
    calibrationWizardState = CAL_WIZARD_RAMP;
    Serial.println(F("Calibration load test started; primary path only."));
    return;
  }

  if (calibrationWizardState == CAL_WIZARD_WAIT_DMM &&
      strncmp(command, "V=", 2U) == 0) {
    bool haveOutputVoltage = false;
    bool haveInputVoltage = false;
    bool haveInputCurrent = false;
    int32_t measuredVoltageCenti = 0L;
    int32_t measuredSourceCenti = 0L;
    int32_t measuredInputCurrentCenti = 0L;
    char *token = strtok(command, " ");
    while (token != 0) {
      if (strncmp(token, "V=", 2U) == 0) {
        haveOutputVoltage = parsePositiveCentivolts(token + 2,
                                                    measuredVoltageCenti);
      } else if (strncmp(token, "VIN=", 4U) == 0) {
#if SOURCE_SENSE_ENABLED
        haveInputVoltage = parseSourceCentivolts(token + 4,
                                                 measuredSourceCenti);
#else
        haveInputVoltage = true;
#endif
      } else if (strncmp(token, "IIN=", 4U) == 0) {
#if SOURCE_CURRENT_SENSE_ENABLED
        char *end = 0;
        const long parsed = strtol(token + 4, &end, 10);
        if (end != token + 4 && *end == '\0' && parsed >= 10L &&
            parsed <= INPUT_HARD_OVERCURRENT_CENTI) {
          measuredInputCurrentCenti = (int32_t)parsed;
          haveInputCurrent = true;
        }
#else
        haveInputCurrent = true;
#endif
      }
      token = strtok(0, " ");
    }

    if (!haveOutputVoltage || !haveInputVoltage || !haveInputCurrent) {
      Serial.println(F("Required: V=xxxx VIN=xxxx IIN=xx; values are centivolts and centiamps."));
      return;
    }
    if (calibrationSampleCount != CAL_SAMPLE_COUNT) {
      Serial.println(F("Calibration sample window incomplete; wait for stable indication."));
      return;
    }

    const uint32_t meanVoltageRaw =
        (calibrationVoltageRawSum + CAL_SAMPLE_COUNT / 2U) /
        CAL_SAMPLE_COUNT;
    const uint32_t meanCurrentRaw =
        (calibrationCurrentRawSum + CAL_SAMPLE_COUNT / 2U) /
        CAL_SAMPLE_COUNT;
#if SOURCE_SENSE_ENABLED
    const uint32_t meanSourceRaw =
        (calibrationSourceRawSum + CAL_SAMPLE_COUNT / 2U) /
        CAL_SAMPLE_COUNT;
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
    const uint32_t meanInputCurrentRaw =
        (calibrationSourceCurrentRawSum + CAL_SAMPLE_COUNT / 2U) /
        CAL_SAMPLE_COUNT;
#endif
    if (meanVoltageRaw < 100U || meanVoltageRaw > 900U) {
      Serial.println(F("Calibration rejected: voltage ADC reading outside range."));
      return;
    }

    const int64_t deltaCurrentRawQ8 =
        (int64_t)meanCurrentRaw * 256LL - primaryCurrentZeroRawQ8;
    if (deltaCurrentRawQ8 > -2560LL && deltaCurrentRawQ8 < 2560LL) {
      Serial.println(F("Calibration rejected: insufficient output ACS712 signal."));
      return;
    }

    const int32_t expectedOutputCurrentCenti = (int32_t)(
        ((int64_t)measuredVoltageCenti * 1000LL) /
        CAL_LOAD_RESISTANCE_MILLIOHM);
    if (expectedOutputCurrentCenti < 40L ||
        expectedOutputCurrentCenti >= OUTPUT_HARD_OVERCURRENT_CENTI) {
      Serial.println(F("Calibration rejected: calculated output load current outside safe range."));
      return;
    }

    const uint32_t newVoltageScale = (uint32_t)(
        ((uint64_t)measuredVoltageCenti * 65536ULL + meanVoltageRaw / 2U) /
        meanVoltageRaw);
    const int64_t newOutputCurrentGain = (
        (int64_t)expectedOutputCurrentCenti * 65536LL * 256LL) /
        deltaCurrentRawQ8;
    const int32_t outputGainMagnitude = absI32((int32_t)newOutputCurrentGain);
    if (newVoltageScale < 20000UL || newVoltageScale > 1000000UL ||
        outputGainMagnitude < 50000L || outputGainMagnitude > 1000000L) {
      Serial.println(F("Calibration rejected: output scale/gain implausible."));
      return;
    }

    calibrationData.currentZeroRawQ8 = primaryCurrentZeroRawQ8;
    calibrationData.currentGainCentiPerCountQ16 =
        (int32_t)newOutputCurrentGain;
    calibrationData.voltageCentiPerCountQ16 = newVoltageScale;
#if SOURCE_SENSE_ENABLED
    if (meanSourceRaw < 50U || meanSourceRaw > 900U) {
      Serial.println(F("Calibration rejected: input-voltage ADC reading invalid."));
      return;
    }
    calibrationData.sourceCentiPerCountQ16 = (uint32_t)(
        ((uint64_t)measuredSourceCenti * 65536ULL + meanSourceRaw / 2U) /
        meanSourceRaw);
    if (calibrationData.sourceCentiPerCountQ16 < 20000UL ||
        calibrationData.sourceCentiPerCountQ16 > 1000000UL) {
      Serial.println(F("Calibration rejected: input-voltage divider scale implausible."));
      return;
    }
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
    const int64_t deltaInputCurrentRawQ8 =
        (int64_t)meanInputCurrentRaw * 256LL - sourceCurrentZeroRawQ8;
    if (deltaInputCurrentRawQ8 > -1280LL &&
        deltaInputCurrentRawQ8 < 1280LL) {
      Serial.println(F("Calibration rejected: insufficient input ACS712 signal."));
      return;
    }
    const int64_t newInputCurrentGain = (
        (int64_t)measuredInputCurrentCenti * 65536LL * 256LL) /
        deltaInputCurrentRawQ8;
    const int32_t inputGainMagnitude = absI32((int32_t)newInputCurrentGain);
    if (inputGainMagnitude < 50000L || inputGainMagnitude > 1000000L) {
      Serial.println(F("Calibration rejected: input ACS712 gain implausible; check IIN meter and orientation."));
      return;
    }
    calibrationData.sourceCurrentZeroRawQ8 = sourceCurrentZeroRawQ8;
    calibrationData.sourceCurrentGainCentiPerCountQ16 =
        (int32_t)newInputCurrentGain;
#endif
    if (!saveCalibrationToEeprom()) {
      latchFault(FAULT_EEPROM);
      calibrationWizardState = CAL_WIZARD_FAILED;
      Serial.println(F("EEPROM verification failed; output disabled."));
      return;
    }

    writeDutyBasisPoints(0L, 0L);
    calibrationWizardState = CAL_WIZARD_READY;
    voltageIntegratorQ16 = 0L;
    currentIntegratorQ16 = 0L;
    resetDualModel(currentModel);
    resetDualModel(voltageModel);
    previousMeasurementsValid = false;
    selectedCandidateAvailable = false;
    latestEfficiencyValid = false;
    lastControlUs = (uint32_t)micros();
    Serial.println(F("Output/input ACS712 and voltage calibration committed to EEPROM; normal control enabled."));
    return;
  }

  Serial.println(F("Unknown command for current calibration state."));
}

static void pollCalibrationSerial() {
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0) break;
    const char ch = (char)incoming;
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (!calibrationCommandOverflow && calibrationCommandLength > 0U) {
        calibrationCommandBuffer[calibrationCommandLength] = '\0';
        handleCalibrationCommand(calibrationCommandBuffer);
      }
      calibrationCommandLength = 0U;
      calibrationCommandOverflow = false;
    } else if (!calibrationCommandOverflow) {
      if (calibrationCommandLength <
          (uint8_t)(sizeof(calibrationCommandBuffer) - 1U)) {
        calibrationCommandBuffer[calibrationCommandLength++] = ch;
      } else {
        calibrationCommandOverflow = true;
      }
    }
  }
}

static void calibrationWizardLoop() {
  pollCalibrationSerial();
  if (calibrationWizardState != CAL_WIZARD_RAMP &&
      calibrationWizardState != CAL_WIZARD_WAIT_DMM) {
    writeDutyBasisPoints(0L, 0L);
    return;
  }

  const uint32_t nowUs = (uint32_t)micros();
  const uint32_t dtUs = nowUs - lastControlUs;
  if (dtUs < CONTROL_PERIOD_US) return;
  lastControlUs = nowUs;
  if (dtUs > MAX_CONTROL_DT_US) {
    latchFault(FAULT_CONTROL_DEADLINE);
    calibrationWizardState = CAL_WIZARD_FAILED;
    return;
  }
  if (faultCode != FAULT_NONE) {
    calibrationWizardState = CAL_WIZARD_FAILED;
    return;
  }

  const uint16_t voltageRaw = (uint16_t)analogRead(VOLTAGE_PIN);
  const uint16_t currentRaw = (uint16_t)analogRead(CURRENT_PIN);
  const uint16_t thermistorRaw = (uint16_t)analogRead(THERMISTOR_PIN);
#if SOURCE_CURRENT_SENSE_ENABLED
  const uint16_t sourceCurrentRaw = (uint16_t)analogRead(SOURCE_CURRENT_PIN);
#endif
  if (voltageRaw >= ADC_RAIL_TRIP_RAW ||
      currentRaw <= 5U || currentRaw >= ADC_RAIL_TRIP_RAW
#if SOURCE_CURRENT_SENSE_ENABLED
      || sourceCurrentRaw <= 5U || sourceCurrentRaw >= ADC_RAIL_TRIP_RAW
#endif
      ) {
    latchFault(FAULT_ADC_RAIL);
    calibrationWizardState = CAL_WIZARD_FAILED;
    return;
  }
  if (!updateThermalLimit(thermistorRaw)) {
    latchFault(FAULT_THERMISTOR);
    calibrationWizardState = CAL_WIZARD_FAILED;
    return;
  }
  if ((calibrationWizardState == CAL_WIZARD_RAMP ||
       calibrationWizardState == CAL_WIZARD_WAIT_DMM) &&
      runtimeCurrentLimitCenti < CAL_EXPECTED_CURRENT_CENTI) {
    writeDutyBasisPoints(0L, 0L);
    calibrationDutyBp = 0L;
    resetCalibrationAverages();
    calibrationWizardState = CAL_WIZARD_WAIT_LOAD;
    Serial.println(F("Calibration paused: heatsink too warm for the 10-ohm test; cool it and send LOAD."));
    return;
  }

  const int32_t voltageCenti = adcToVoltageCenti(voltageRaw);
  const int32_t currentSigned = adcToAcs712CurrentCenti(
      currentRaw, primaryCurrentZeroRawQ8,
      (uint16_t)ACS712_PRIMARY_SENSITIVITY_MV_PER_A, 1);
  // Polarity is learned from the known 10-ohm forward-load test and stored in
  // the sign of currentGainCentiPerCountQ16. Use magnitude for calibration
  // supervision so a reversed ACS712 installation can be corrected in EEPROM.
  const int32_t currentCenti = absI32(currentSigned);
  if (voltageCenti >= OUTPUT_OVERVOLTAGE_CENTI) {
    latchFault(FAULT_OVERVOLTAGE);
    calibrationWizardState = CAL_WIZARD_FAILED;
    return;
  }
  if (currentCenti >= OUTPUT_HARD_OVERCURRENT_CENTI) {
    latchFault(FAULT_OVERCURRENT);
    calibrationWizardState = CAL_WIZARD_FAILED;
    return;
  }
#if SOURCE_CURRENT_SENSE_ENABLED
  const int32_t inputCurrentSigned = adcToAcs712CurrentCenti(
      sourceCurrentRaw, sourceCurrentZeroRawQ8,
      (uint16_t)ACS712_SOURCE_SENSITIVITY_MV_PER_A,
      (int8_t)ACS712_SOURCE_POLARITY);
  if (absI32(inputCurrentSigned) >= INPUT_HARD_OVERCURRENT_CENTI) {
    latchFault(FAULT_INPUT_OVERCURRENT);
    calibrationWizardState = CAL_WIZARD_FAILED;
    return;
  }
#endif

#if SOURCE_SENSE_ENABLED
  const uint16_t sourceRaw = (uint16_t)analogRead(SOURCE_VOLTAGE_PIN);
  if (sourceRaw >= ADC_RAIL_TRIP_RAW) {
    latchFault(FAULT_ADC_RAIL);
    calibrationWizardState = CAL_WIZARD_FAILED;
    return;
  }
#endif

  // Stability requires a small absolute setpoint error and low sample-to-
  // sample movement. Do not calibrate gain from a drifting output.
  int32_t voltageStepCenti = 0L;
  if (calibrationLastVoltageCenti >= 0L)
    voltageStepCenti = absI32(voltageCenti - calibrationLastVoltageCenti);
  calibrationLastVoltageCenti = voltageCenti;

  if (calibrationWizardState == CAL_WIZARD_RAMP) {
    const int32_t error = CAL_TARGET_V_CENTI - voltageCenti;
    if (absI32(error) <= CAL_STABLE_BAND_CENTI &&
        voltageStepCenti <= 5L) {
      if (calibrationStableStartMs == 0UL)
        calibrationStableStartMs = (uint32_t)millis();
      if ((uint32_t)(millis() - calibrationStableStartMs) >=
          CAL_STABLE_TIME_MS) {
        resetCalibrationAverages();
        calibrationWizardState = CAL_WIZARD_WAIT_DMM;
        Serial.println(F("Output stable. Measure Vout with a DMM; send V=xxxx in centivolts."));
      }
    } else {
      calibrationStableStartMs = 0UL;
    }
  }

  if ((calibrationWizardState == CAL_WIZARD_RAMP ||
       calibrationWizardState == CAL_WIZARD_WAIT_DMM) &&
      (uint32_t)(millis() - calibrationStateStartMs) > CAL_TIMEOUT_MS) {
    latchFault(FAULT_SENSOR_CALIBRATION);
    calibrationWizardState = CAL_WIZARD_FAILED;
    return;
  }

  if (calibrationWizardState == CAL_WIZARD_WAIT_DMM &&
      (absI32(CAL_TARGET_V_CENTI - voltageCenti) >
           (CAL_STABLE_BAND_CENTI * 2L) ||
       voltageStepCenti > 10L)) {
    resetCalibrationAverages();
    calibrationStableStartMs = 0UL;
    calibrationLastVoltageCenti = voltageCenti;
    calibrationWizardState = CAL_WIZARD_RAMP;
    Serial.println(F("Calibration drifted; reacquiring stable load window."));
  }

  if (calibrationWizardState == CAL_WIZARD_WAIT_DMM &&
      calibrationSampleCount < CAL_SAMPLE_COUNT) {
    if (absI32(CAL_TARGET_V_CENTI - voltageCenti) <=
            (CAL_STABLE_BAND_CENTI * 2L) &&
        voltageStepCenti <= 5L) {
      calibrationVoltageRawSum += voltageRaw;
      calibrationCurrentRawSum += currentRaw;
#if SOURCE_SENSE_ENABLED
      calibrationSourceRawSum += sourceRaw;
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
      calibrationSourceCurrentRawSum += sourceCurrentRaw;
#endif
      ++calibrationSampleCount;
      if (calibrationSampleCount == CAL_SAMPLE_COUNT) {
        Serial.print(F("Samples ready. Vout ADC="));
        Serial.print((calibrationVoltageRawSum + CAL_SAMPLE_COUNT / 2U) /
                     CAL_SAMPLE_COUNT);
        Serial.print(F(" output ACS ADC="));
        Serial.print((calibrationCurrentRawSum + CAL_SAMPLE_COUNT / 2U) /
                     CAL_SAMPLE_COUNT);
        Serial.print(F(" input ACS ADC="));
        Serial.println((calibrationSourceCurrentRawSum + CAL_SAMPLE_COUNT / 2U) /
                       CAL_SAMPLE_COUNT);
      }
    } else {
      resetCalibrationAverages();
    }
  }

  // Conservative primary-only calibration regulator. It is intentionally
  // separate from adaptive control; AUX PWM remains disabled throughout.
  const int32_t error = CAL_TARGET_V_CENTI - voltageCenti;
  int32_t dutyStep = clampI32(error * 2L, -100L, 100L);
  if (absI32(error) <= 3L) dutyStep = 0L;
  calibrationDutyBp = clampI32(calibrationDutyBp + dutyStep, 0L,
                               MAX_DUTY_BASIS_POINTS);
  calibrationDutyBp = quantizePrimaryDutyBp(calibrationDutyBp);
  writeDutyBasisPoints(calibrationDutyBp, 0L);
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

static void updateAuxiliaryPathMode(int32_t measuredCurrentCenti,
                                    bool auxiliaryModelReady) {
#if AUX_PATH_ARMED
  if (!auxiliaryLowPowerActive || !auxiliaryModelReady ||
      measuredCurrentCenti > AUX_LOW_POWER_EXIT_CENTI ||
      currentGuardActive) {
    auxiliaryPathMode = AUX_MODE_PRIMARY_ONLY;
    return;
  }

  switch (auxiliaryPathMode) {
    case AUX_MODE_PRIMARY_ONLY:
      if (appliedDutyBp <= AUX_OVERLAP_ENTER_BP)
        auxiliaryPathMode = AUX_MODE_OVERLAP;
      break;

    case AUX_MODE_OVERLAP:
      if (appliedDutyBp >= AUX_OVERLAP_EXIT_BP) {
        auxiliaryPathMode = AUX_MODE_PRIMARY_ONLY;
      } else if (primaryEquivalentDutyDemandBp <= AUX_ONLY_ENTER_BP &&
                 measuredCurrentCenti <= AUX_ONLY_MAX_CURRENT_CENTI) {
        auxiliaryPathMode = AUX_MODE_AUXILIARY_ONLY;
      }
      break;

    case AUX_MODE_AUXILIARY_ONLY:
      if (primaryEquivalentDutyDemandBp >= AUX_ONLY_EXIT_BP ||
          measuredCurrentCenti > AUX_ONLY_EXIT_CURRENT_CENTI) {
        auxiliaryPathMode = AUX_MODE_OVERLAP;
      }
      break;

    default:
      auxiliaryPathMode = AUX_MODE_PRIMARY_ONLY;
      break;
  }
#else
  (void)measuredCurrentCenti;
  (void)auxiliaryModelReady;
  auxiliaryPathMode = AUX_MODE_PRIMARY_ONLY;
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
  if (currentCenti >= (runtimeCurrentLimitCenti * CURRENT_GUARD_HARD_PERCENT) / 100L)
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

// Keep source-current setting independent from the output-current setting.
// Input protection acts in its own current-feedback guard below; it does not
// overwrite the thermally derated output-current ceiling.
static void refreshInputCurrentLimit() {
  runtimeInputCurrentLimitCenti = inputCurrentSettingCenti;
}

// Independent source-current feedback guard. It backs off requested output
// power when source current approaches its own setpoint, without conflating
// source amps with load amps or overwriting the output/thermal current limit.
static void applyInputCurrentGuard(int32_t sourceCurrentInstant,
                                   int32_t &selectedDuty,
                                   int32_t &selectedAuxiliaryDuty) {
#if SOURCE_CURRENT_SENSE_ENABLED
  if (runtimeInputCurrentLimitCenti <= 0L) {
    selectedDuty = 0L;
    selectedAuxiliaryDuty = 0L;
    inputCurrentGuardActive = true;
    currentIntegratorQ16 = 0L;
    return;
  }
  const int32_t inputGuardEnterCenti = (int32_t)(
      ((int64_t)runtimeInputCurrentLimitCenti * INPUT_CURRENT_GUARD_START_PERCENT) / 100LL);
  const int32_t inputGuardHardCenti = (int32_t)(
      ((int64_t)runtimeInputCurrentLimitCenti * INPUT_CURRENT_GUARD_HARD_PERCENT) / 100LL);
  const int32_t inputGuardReleaseCenti = (int32_t)(
      ((int64_t)runtimeInputCurrentLimitCenti * INPUT_CURRENT_GUARD_RELEASE_PERCENT) / 100LL);
  if (!inputCurrentGuardActive && sourceCurrentInstant >= inputGuardEnterCenti)
    inputCurrentGuardActive = true;
  else if (inputCurrentGuardActive &&
           sourceCurrentInstant <= inputGuardReleaseCenti)
    inputCurrentGuardActive = false;

  if (inputCurrentGuardActive) {
    const int32_t excessCenti = sourceCurrentInstant - inputGuardEnterCenti;
    const int32_t referenceBrakeCenti = clampI32(
        1L + (excessCenti > 0L ? excessCenti / 8L : 0L), 1L, 16L);
    commandedCurrentReferenceCenti = clampI32(
        commandedCurrentReferenceCenti - referenceBrakeCenti,
        0L, runtimeCurrentLimitCenti);
    if (sourceCurrentInstant >= inputGuardHardCenti) {
      const int32_t inputReducedDuty =
          (appliedDutyBp > INPUT_CURRENT_BRAKE_STEP_BP)
          ? appliedDutyBp - INPUT_CURRENT_BRAKE_STEP_BP : 0L;
      if (selectedDuty > inputReducedDuty) selectedDuty = inputReducedDuty;
      selectedAuxiliaryDuty = 0L;
    }
    currentIntegratorQ16 = 0L;
#if AUX_PATH_ARMED
    auxiliaryLowPowerActive = false;
#endif
  }
#else
  (void)sourceCurrentInstant;
  (void)selectedDuty;
  (void)selectedAuxiliaryDuty;
#endif
}

// This runs between control deadlines, never inside controlUpdate(). With
// two channels, each pot is sampled every 100 ms. Its update rate is deliberately
// slow and filtered; no fast-loop timing depends on potentiometer movement.
struct LimitPotState {
  int32_t filteredRawQ8;
  uint8_t initialised;
};
static LimitPotState outputLimitPotState = { 0L, 0U };
static LimitPotState inputLimitPotState = { 0L, 0U };
static uint32_t lastLimitPotSampleMs = 0UL;
static uint8_t nextLimitPotChannel = 0U;

static int32_t mapPotToCurrentLimit(int32_t raw, int32_t minimum,
                                    int32_t maximum) {
  raw = clampI32(raw, 0L, 1023L);
  return minimum + (int32_t)(((int64_t)raw * (maximum - minimum) + 511LL) /
                             1023LL);
}

static void serviceOneLimitPot(uint8_t pin, LimitPotState &state,
                               int32_t minimum, int32_t maximum,
                               int32_t &setting, uint8_t muxChannel) {
#if LIMIT_POT_MUX_ENABLED
  // D5 selects one of two 74HC4051 channels. Discard the first conversion
  // after switching so the ADC sample capacitor settles to the new wiper.
  digitalWrite(LIMIT_POT_MUX_SELECT_PIN,
               (muxChannel != 0U) ? HIGH : LOW);
  (void)analogRead(pin);
#else
  (void)muxChannel;
#endif
  const uint16_t raw = (uint16_t)analogRead(pin);
  if (!state.initialised) {
    state.filteredRawQ8 = (int32_t)raw * 256L;
    state.initialised = 1U;
  } else {
    const int32_t targetQ8 = (int32_t)raw * 256L;
    state.filteredRawQ8 += (targetQ8 - state.filteredRawQ8) >>
                           LIMIT_POT_FILTER_SHIFT;
  }
  const int32_t filtered = state.filteredRawQ8 / 256L;
  setting = mapPotToCurrentLimit(filtered, minimum, maximum);
}

static void serviceLimitPotentiometers() {
#if (OUTPUT_LIMIT_POT_PIN >= 0) || (INPUT_LIMIT_POT_PIN >= 0)
  const uint32_t nowMs = (uint32_t)millis();
  if ((uint32_t)(nowMs - lastLimitPotSampleMs) < LIMIT_POT_SAMPLE_INTERVAL_MS)
    return;
  lastLimitPotSampleMs = nowMs;

#if (OUTPUT_LIMIT_POT_PIN >= 0) && (INPUT_LIMIT_POT_PIN >= 0)
  if (nextLimitPotChannel == 0U) {
    serviceOneLimitPot((uint8_t)OUTPUT_LIMIT_POT_PIN, outputLimitPotState,
                       OUTPUT_POT_MIN_CENTI, OUTPUT_POT_MAX_CENTI,
                       outputCurrentSettingCenti, 0U);
    nextLimitPotChannel = 1U;
  } else {
    serviceOneLimitPot((uint8_t)INPUT_LIMIT_POT_PIN, inputLimitPotState,
                       INPUT_POT_MIN_CENTI, INPUT_POT_MAX_CENTI,
                       inputCurrentSettingCenti, 1U);
    nextLimitPotChannel = 0U;
  }
#elif (OUTPUT_LIMIT_POT_PIN >= 0)
  serviceOneLimitPot((uint8_t)OUTPUT_LIMIT_POT_PIN, outputLimitPotState,
                     OUTPUT_POT_MIN_CENTI, OUTPUT_POT_MAX_CENTI,
                     outputCurrentSettingCenti, 0U);
#elif (INPUT_LIMIT_POT_PIN >= 0)
  serviceOneLimitPot((uint8_t)INPUT_LIMIT_POT_PIN, inputLimitPotState,
                     INPUT_POT_MIN_CENTI, INPUT_POT_MAX_CENTI,
                     inputCurrentSettingCenti, 1U);
#endif
#endif
}

static void updateMeasuredEfficiency(int32_t sourceVoltageCenti,
                                     int32_t sourceCurrentCenti,
                                     int32_t outputVoltageCenti,
                                     int32_t outputCurrentCenti) {
  const int64_t inputProduct = (int64_t)sourceVoltageCenti * sourceCurrentCenti;
  const int64_t outputProduct = (int64_t)outputVoltageCenti * outputCurrentCenti;
  if (inputProduct <= 0LL || outputProduct < 0LL || outputProduct > inputProduct) {
    latestEfficiencyPermille = 0U;
    latestEfficiencyValid = false;
    return;
  }
  const int64_t permille = (outputProduct * 1000LL) / inputProduct;
  if (permille <= 0LL || permille > 1000LL) {
    latestEfficiencyPermille = 0U;
    latestEfficiencyValid = false;
    return;
  }
  latestEfficiencyPermille = (uint16_t)permille;
  latestEfficiencyValid = true;
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
static int32_t effectiveDutyLevelBp(int32_t primaryDutyBp,
                                    int32_t auxiliaryDutyBp) {
  if (primaryDutyBp > 0L && auxiliaryDutyBp > 0L)
    return (primaryDutyBp + auxiliaryDutyBp) / 2L;
  return (primaryDutyBp > 0L) ? primaryDutyBp : auxiliaryDutyBp;
}

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
  const int64_t maxI_Q16 = (int64_t)runtimeCurrentLimitCenti * 65536LL;
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
  result.primaryEquivalentDutyBp = currentDutyBp;
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
  const bool auxiliaryOnlyMode =
      (auxiliaryPathMode == AUX_MODE_AUXILIARY_ONLY);
  if ((requestsAuxiliary || auxiliaryOnlyMode) &&
      (!auxiliaryAllowed ||
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
  if (requestedCurrent > runtimeCurrentLimitCenti)
    requestedCurrent = runtimeCurrentLimitCenti;
  int32_t currentSlew = candidate.currentSlewCenti;
  if ((candidate.flags & CANDIDATE_CURRENT_GUARD) != 0U) {
    const int32_t guardedReference = (int32_t)(
        ((int64_t)runtimeCurrentLimitCenti * CURRENT_GUARD_HARD_PERCENT) / 100LL);
    if (requestedCurrent > guardedReference) requestedCurrent = guardedReference;
    if (currentSlew < 50L) currentSlew = 50L;
  }
  int32_t currentReference = limitSlew((int32_t)requestedCurrent,
                                       currentReferenceStateCenti,
                                       currentSlew);
  currentReference = clampI32(currentReference, 0L, runtimeCurrentLimitCenti);

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

  // Estimate the primary-only duty that would produce the same requested
  // current change. This shadow value governs auxiliary-only eligibility even
  // while the actual primary PWM output is held at zero.
  const int32_t innerGainQ14 = currentPlant.thetaQ14[2];
  if (innerGainQ14 <= RLS_MIN_GAIN_Q14) return result;
  const int32_t presentDutyLevelBp =
      effectiveDutyLevelBp(currentDutyBp, auxiliaryDutyBp);
  const int32_t primaryOnlyBaseQ14 = predictCurrentNoDutyQ14(
      currentPlant, deltaCurrentCenti, priorDeltaCurrentCenti,
      deltaVoltageCenti, deltaSourceCenti, presentDutyLevelBp,
      candidate.disturbanceWeightPermille);
  const int32_t primaryOnlyResidualQ14 =
      desiredDeltaIQ14 - primaryOnlyBaseQ14;
  int64_t primaryOnlyDeviationBp =
      ((int64_t)primaryOnlyResidualQ14 * MODEL_DUTY_SCALE_BP) /
      innerGainQ14;
  primaryOnlyDeviationBp = clampI32((int32_t)primaryOnlyDeviationBp,
                                    -MAX_DUTY_BASIS_POINTS,
                                    MAX_DUTY_BASIS_POINTS);
  result.primaryEquivalentDutyBp = quantizePrimaryDutyBp(
      clampI32((int32_t)((int64_t)currentDutyMeanBp +
                         primaryOnlyDeviationBp),
               MIN_DUTY_BASIS_POINTS, MAX_DUTY_BASIS_POINTS));

  // Base prediction excludes both actuators. Allocate a small fraction of the
  // required current increment to the fast, low-L branch only in its low-power
  // resolution class. Its duty has a much finer local scale because L_aux is
  // assumed to be about 10x smaller than L_primary.
  int32_t requestedAuxiliaryDuty =
      (requestsAuxiliary && auxiliaryAllowed)
          ? auxiliaryDutyBp
          : limitSlew(0L, auxiliaryDutyBp, AUX_DISABLE_SLEW_BP);
  int32_t auxiliaryContributionQ14 = 0L;
  int32_t requestedDuty = currentDutyBp;

  if (auxiliaryOnlyMode) {
    // In auxiliary-only operation the primary switch is held OFF. Solve the
    // complete requested incremental-current action using the identified
    // auxiliary actuator gain rather than copying the primary duty command.
    requestedDuty = 0L;
    for (uint8_t solve = 0U; solve < 2U; ++solve) {
      const int32_t baseQ14 = predictCurrentNoDutyQ14(
          currentPlant, deltaCurrentCenti, priorDeltaCurrentCenti,
          deltaVoltageCenti, deltaSourceCenti,
          effectiveDutyLevelBp(0L, requestedAuxiliaryDuty),
          candidate.disturbanceWeightPermille);
      const int32_t residualQ14 = desiredDeltaIQ14 - baseQ14;
      int64_t requiredDeviationBp =
          ((int64_t)residualQ14 * AUX_MODEL_DUTY_SCALE_BP) /
          currentPlant.thetaQ14[3];
      if (requiredDeviationBp > AUX_MAX_DUTY_BASIS_POINTS)
        requiredDeviationBp = AUX_MAX_DUTY_BASIS_POINTS;
      if (requiredDeviationBp < -AUX_MAX_DUTY_BASIS_POINTS)
        requiredDeviationBp = -AUX_MAX_DUTY_BASIS_POINTS;
      int64_t absoluteAuxDuty =
          (int64_t)auxiliaryMeanDutyBp + requiredDeviationBp;
      absoluteAuxDuty = clampI32((int32_t)absoluteAuxDuty, 0L,
                                  AUX_MAX_DUTY_BASIS_POINTS);
      requestedAuxiliaryDuty = limitSlew(
          (int32_t)absoluteAuxDuty, auxiliaryDutyBp, AUX_DUTY_SLEW_BP);
    }
    requestedAuxiliaryDuty = quantizeAuxiliaryDutyBp(
        clampI32(requestedAuxiliaryDuty, 0L, AUX_MAX_DUTY_BASIS_POINTS));
    const int16_t auxFeature = toQ14(
        requestedAuxiliaryDuty - auxiliaryMeanDutyBp,
        AUX_MODEL_DUTY_SCALE_BP);
    auxiliaryContributionQ14 = (int32_t)(
        ((int64_t)currentPlant.thetaQ14[3] * auxFeature) >> 14);
  } else {
    if (requestsAuxiliary) {
      const int32_t controlResidualQ14 =
          desiredDeltaIQ14 - predictCurrentNoDutyQ14(
              currentPlant, deltaCurrentCenti, priorDeltaCurrentCenti,
              deltaVoltageCenti, deltaSourceCenti,
              effectiveDutyLevelBp(currentDutyBp, requestedAuxiliaryDuty),
              candidate.disturbanceWeightPermille);
      const int32_t allocatedAuxQ14 = (int32_t)(
          ((int64_t)controlResidualQ14 * candidate.auxiliarySharePermille) /
          1000LL);
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
    for (uint8_t solve = 0U; solve < 2U; ++solve) {
      const int32_t sourceBaseQ14 = predictCurrentNoDutyQ14(
          currentPlant, deltaCurrentCenti, priorDeltaCurrentCenti,
          deltaVoltageCenti, deltaSourceCenti,
          effectiveDutyLevelBp(requestedDuty, requestedAuxiliaryDuty),
          candidate.disturbanceWeightPermille);
      const int32_t currentResidualQ14 =
          desiredDeltaIQ14 - sourceBaseQ14 - auxiliaryContributionQ14;
      int64_t requiredDeviationBp =
          ((int64_t)currentResidualQ14 * MODEL_DUTY_SCALE_BP) / innerGainQ14;
      if (requiredDeviationBp > MAX_DUTY_BASIS_POINTS)
        requiredDeviationBp = MAX_DUTY_BASIS_POINTS;
      if (requiredDeviationBp < -MAX_DUTY_BASIS_POINTS)
        requiredDeviationBp = -MAX_DUTY_BASIS_POINTS;
      int64_t absoluteDuty = (int64_t)currentDutyMeanBp +
                             requiredDeviationBp;
      absoluteDuty = clampI32((int32_t)absoluteDuty,
                              MIN_DUTY_BASIS_POINTS,
                              MAX_DUTY_BASIS_POINTS);
      requestedDuty = limitSlew((int32_t)absoluteDuty, currentDutyBp,
                                candidate.dutySlewBp);
      requestedDuty = clampI32(requestedDuty, MIN_DUTY_BASIS_POINTS,
                                MAX_DUTY_BASIS_POINTS);
    }
  }

  requestedDuty = quantizePrimaryDutyBp(requestedDuty);
  requestedAuxiliaryDuty = quantizeAuxiliaryDutyBp(requestedAuxiliaryDuty);

  const int32_t primaryDutyDelta = requestedDuty - currentDutyBp;
  const int32_t predictedCurrentQ14 = predictCurrentQ14(
      currentPlant, deltaCurrentCenti, priorDeltaCurrentCenti,
      deltaVoltageCenti, deltaSourceCenti, requestedDuty - currentDutyMeanBp,
      requestedAuxiliaryDuty - auxiliaryDutyMeanBp,
      effectiveDutyLevelBp(requestedDuty, requestedAuxiliaryDuty),
      candidate.disturbanceWeightPermille);
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
  if ((requestedAuxiliaryDuty > 0L || auxiliaryOnlyMode) &&
      !measuredEfficiencyComparisonAvailable) {
    // Compare the auxiliary branch's higher conduction loss with its lower
    // switching-energy estimate. The credit is a hardware-dependent prior,
    // not a universal MOSFET property; calibrate it from Eon/Eoff, Qg and fSW.
    const int32_t measuredAuxiliaryCurrent = auxiliaryOnlyMode
        ? currentCenti
        : (int32_t)(((int64_t)currentCenti *
                     candidate.auxiliarySharePermille) / 1000LL);
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

  const int32_t softCurrent = (int32_t)(
      ((int64_t)runtimeCurrentLimitCenti * CURRENT_GUARD_START_PERCENT) / 100LL);
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
  const uint16_t thermistorRaw = (uint16_t)analogRead(THERMISTOR_PIN);
#if AUX_CURRENT_SENSE_ENABLED
  const uint16_t auxiliaryCurrentRaw = (uint16_t)analogRead(AUX_CURRENT_PIN);
#endif
#if SOURCE_SENSE_ENABLED
  const uint16_t sourceRaw = (uint16_t)analogRead(SOURCE_VOLTAGE_PIN);
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
  const uint16_t sourceCurrentRaw = (uint16_t)analogRead(SOURCE_CURRENT_PIN);
#endif

  // The shared-heatsink thermistor is part of the power-limit safety loop.
  // Sensor open/short is a fail-safe shutdown, not a request for nominal power.
  if (!updateThermalLimit(thermistorRaw)) {
    if (faultCode == FAULT_NONE) latchFault(FAULT_THERMISTOR);
    return;
  }

  // Sensor saturation invalidates downstream conversion.
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
  if (sourceCurrentInstant >= INPUT_HARD_OVERCURRENT_CENTI) {
    latchFault(FAULT_INPUT_OVERCURRENT);
    return;
  }
#endif
  if (voltageInstant >= OUTPUT_OVERVOLTAGE_CENTI) {
    latchFault(FAULT_OVERVOLTAGE);
    return;
  }
  if (currentInstant >= OUTPUT_HARD_OVERCURRENT_CENTI) {
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
  latestInputCurrentCenti = sourceCurrentCenti;
  refreshInputCurrentLimit();
  updateMeasuredEfficiency(sourceCenti, sourceCurrentCenti,
                           voltageCenti, currentCenti);
#endif
  latestOutputCurrentCenti = currentCenti;

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
                               currentInstant >= (runtimeCurrentLimitCenti *
                                   CURRENT_GUARD_HARD_PERCENT) / 100L ||
                               sourceCurrentInstant >=
                                   (runtimeInputCurrentLimitCenti *
                                    INPUT_CURRENT_GUARD_START_PERCENT) / 100L);
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
  updateAuxiliaryPathMode(currentInstant, auxiliaryModelReady);
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
  bootstrapIref = clampI32(bootstrapIref, 0L, runtimeCurrentLimitCenti);
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
    primaryEquivalentDutyDemandBp = best.primaryEquivalentDutyBp;
    commandedCurrentReferenceCenti = best.currentReferenceCenti;
    activeCandidateIndex = best.candidateIndex;

    // Track PI integrators to model-controller outputs for bumpless fallback.
    const int64_t outerP = (int64_t)voltageError * BOOT_OUTER_KP_Q8 * 256LL;
    int64_t outerI = ((int64_t)best.currentReferenceCenti << 16) - outerP;
    const int64_t maxOuterI = (int64_t)runtimeCurrentLimitCenti * 65536LL;
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
    primaryEquivalentDutyDemandBp = appliedDutyBp;
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
      (currentInstant < (runtimeCurrentLimitCenti *
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
  const int32_t guardEnterCenti = (int32_t)(
      ((int64_t)runtimeCurrentLimitCenti * CURRENT_GUARD_HARD_PERCENT) / 100LL);
  const int32_t guardReleaseCenti = (int32_t)(
      ((int64_t)runtimeCurrentLimitCenti * CURRENT_GUARD_RELEASE_PERCENT) / 100LL);
  const int32_t guardZeroCenti = (int32_t)(
      ((int64_t)runtimeCurrentLimitCenti * CURRENT_GUARD_ZERO_PERCENT) / 100LL);
  if (!currentGuardActive && currentInstant >= guardEnterCenti) {
    currentGuardActive = true;
  } else if (currentGuardActive && currentInstant <= guardReleaseCenti) {
    currentGuardActive = false;
  }
  if (currentGuardActive) {
    commandedCurrentReferenceCenti = clampI32(
        currentInstant - CURRENT_GUARD_BRAKE_STEP_BP / 4L,
        0L, runtimeCurrentLimitCenti);
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
#if SOURCE_CURRENT_SENSE_ENABLED
  applyInputCurrentGuard(sourceCurrentInstant, selectedDuty,
                         selectedAuxiliaryDuty);
#endif

  // One-quadrant converter: above reference, cease positive power demand.
  if (voltageError < -VOLTAGE_ERROR_DEADBAND_CENTI) {
    selectedDuty = 0L;
    selectedAuxiliaryDuty = 0L;
    auxiliaryLowPowerActive = false;
    commandedCurrentReferenceCenti = 0L;
    voltageIntegratorQ16 = 0L;
    currentIntegratorQ16 = 0L;
  }

#if AUX_PATH_ARMED
  selectedAuxiliaryDuty = clampI32(selectedAuxiliaryDuty, 0L,
                                    AUX_MAX_DUTY_BASIS_POINTS);
  selectedDuty = clampI32(selectedDuty, MIN_DUTY_BASIS_POINTS,
                          MAX_DUTY_BASIS_POINTS);

  if (!auxiliaryIdentificationActive) {
    if (auxiliaryPathMode == AUX_MODE_PRIMARY_ONLY) {
      selectedAuxiliaryDuty = limitSlew(0L, appliedAuxiliaryDutyBp,
                                        AUX_DISABLE_SLEW_BP);
    } else if (auxiliaryPathMode == AUX_MODE_AUXILIARY_ONLY) {
      if (best.valid) {
        selectedDuty = 0L;
      } else {
        // Never translate the primary bootstrap command onto an unidentified
        // auxiliary plant. Fall back through the overlap state instead.
        auxiliaryPathMode = AUX_MODE_OVERLAP;
        selectedAuxiliaryDuty = limitSlew(0L, appliedAuxiliaryDutyBp,
                                          AUX_DISABLE_SLEW_BP);
      }
    }
  }
#else
  selectedAuxiliaryDuty = 0L;
  selectedDuty = clampI32(selectedDuty, MIN_DUTY_BASIS_POINTS,
                          MAX_DUTY_BASIS_POINTS);
#endif
  if (runtimeCurrentLimitCenti <= 0L) {
    selectedDuty = 0L;
    selectedAuxiliaryDuty = 0L;
  }
  writeDutyBasisPoints(selectedDuty, selectedAuxiliaryDuty);
  candidateOutcomePending = best.valid &&
                            (auxiliaryPathMode != AUX_MODE_AUXILIARY_ONLY) &&
                            !identificationProbeActive &&
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
  previousDutyLevelBp = effectiveDutyLevelBp(
      appliedDutyBp, appliedAuxiliaryDutyBp);
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
  pinMode(THERMISTOR_PIN, INPUT);
#if SOURCE_CURRENT_SENSE_ENABLED
  pinMode(SOURCE_CURRENT_PIN, INPUT);
#endif
#if (OUTPUT_LIMIT_POT_PIN >= 0)
  pinMode(OUTPUT_LIMIT_POT_PIN, INPUT);
#endif
#if (INPUT_LIMIT_POT_PIN >= 0)
  pinMode(INPUT_LIMIT_POT_PIN, INPUT);
#endif
#if LIMIT_POT_MUX_ENABLED
  pinMode(LIMIT_POT_MUX_SELECT_PIN, OUTPUT);
  digitalWrite(LIMIT_POT_MUX_SELECT_PIN, LOW);
#endif
  pinMode(ACS712_ZERO_INTERLOCK_PIN, INPUT_PULLUP);
#if POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED
#if POWER_STAGE_HARDWARE_FAULT_ACTIVE_LOW
  pinMode(POWER_STAGE_HARDWARE_FAULT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(POWER_STAGE_HARDWARE_FAULT_PIN),
                  powerStageHardwareFaultISR, FALLING);
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
  Serial.begin(SERIAL_BAUD_RATE);
  analogReference(DEFAULT);
  configurePwm();
  writeDutyBasisPoints(0L, 0L);
  initialiseDefaultCalibrationScales();

  if (!loadCalibrationFromEeprom()) {
    currentSensorsCalibrated = false;
    calibrationWizardState = CAL_WIZARD_WAIT_ZERO;
    calibrationStateStartMs = (uint32_t)millis();
    printCalibrationInstructions();
  } else if (!storedZeroOffsetPlausible()) {
    // A valid CRC does not guarantee that the current sensor is still healthy.
    // Require recalibration if its zero point has moved substantially.
    currentSensorsCalibrated = false;
    calibrationWizardState = CAL_WIZARD_WAIT_ZERO;
    calibrationStateStartMs = (uint32_t)millis();
    Serial.println(F("EEPROM record valid, but ACS712 zero check failed."));
    printCalibrationInstructions();
  } else {
    calibrationWizardState = CAL_WIZARD_READY;
    Serial.println(F("EEPROM calibration loaded; CRC and zero-offset checks passed."));
  }

  resetDualModel(currentModel);
  resetDualModel(voltageModel);
  lastControlUs = (uint32_t)micros();
  trendWindowStartUs = lastControlUs;
  crossingWindowStartUs = lastControlUs;
}

void loop() {
  pollCalibrationSerial();

  if (faultCode != FAULT_NONE) {
    writeDutyBasisPoints(0L, 0L);
    return;
  }
#if POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED
  if (powerStageHardwareFaultLatched != 0U) {
    latchFault(FAULT_HARDWARE_TRIP);
    return;
  }
#endif

  if (!currentSensorsCalibrated) {
    calibrationWizardLoop();
    return;
  }

  if (calibrationWizardState != CAL_WIZARD_READY) {
    // A corrupted/rejected calibration transaction can never fall through
    // into normal voltage regulation.
    writeDutyBasisPoints(0L, 0L);
    return;
  }

  const uint32_t nowUs = (uint32_t)micros();
  const uint32_t dtUs = nowUs - lastControlUs;
  if (dtUs < CONTROL_PERIOD_US) {
    serviceLimitPotentiometers();
    return;
  }
  lastControlUs = nowUs;
  if (dtUs > MAX_CONTROL_DT_US) {
    latchFault(FAULT_CONTROL_DEADLINE);
    return;
  }

  controlUpdate(nowUs, dtUs);
}
