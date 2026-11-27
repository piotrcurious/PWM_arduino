#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// AVR flash-size tradeoff: noinline avoids duplicating shared fixed-point and
// orchestration code at many call sites. On ATmega328P it is enabled by
// default; define FW_SIZE_OPTIMIZATION=0 to permit normal inlining after timing
// and flash measurements on a larger target.
#ifndef FW_SIZE_OPTIMIZATION
#if defined(__AVR_ATmega328P__)
#define FW_SIZE_OPTIMIZATION 1
#else
#define FW_SIZE_OPTIMIZATION 0
#endif
#endif
#if (FW_SIZE_OPTIMIZATION != 0) && (FW_SIZE_OPTIMIZATION != 1)
#error "FW_SIZE_OPTIMIZATION must be 0 or 1"
#endif
#if FW_SIZE_OPTIMIZATION && defined(__GNUC__)
#define FW_NOINLINE __attribute__((noinline))
#else
#define FW_NOINLINE
#endif
#if defined(__AVR__)
#include <avr/io.h>
#include <avr/pgmspace.h>
#elif !defined(PROGMEM)
#define PROGMEM
#endif

/*
 * Adaptive dual-path buck controller for a 16 MHz ATmega328P Nano.
 *
 * PWM: D9/OC1A primary, 20 kHz; D3/OC2B auxiliary, 80 kHz.
 * Sensors: A0 Vout, A1 Iout (ACS712-05B), A2 Vin, A3 heatsink NTC,
 *          A4 Iin (ACS712-05B). Optional limit pots: A5 and A6.
 *
 * Both PWM branches and auxiliary efficiency learning are always compiled.
 * ADC-based output/input current limits, hard current trips, thermal derating,
 * ADC rail checks and output overvoltage shutdown remain enabled. No source-side
 * overvoltage threshold is configured; ADC rail detection is not source OVP.
 * There is no interlock pin, analog mux, comparator-status input or sub-cycle
 * fast-current protection in this firmware.
 *
 * Calibration uses dual CRC-checked EEPROM slots, a measured 10-ohm load and
 * DMM readings. ZERO requires the operator to isolate source and load current;
 * this is a procedural requirement, not something firmware can verify.
 *
 * MPPT perturbs an allowed input-power budget and compares measured source
 * power; the independent input-current guard remains authoritative. Online
 * normalized-LMS models are local approximations, not a stability proof. The
 * Timer2 80 kHz output requires an external gate driver and power-stage testing.
 *
 * This sketch is not hardware-qualified. Build/link with the actual Nano core,
 * then verify flash/RAM, timing, current calibration, gate drive and fault
 * response before connecting a power stage.
 */
// -------------------------- Hardware/calibration --------------------------
#define PWM_PIN                         9   // Timer1 OC1A, primary high-L buck path
#define AUX_PWM_PIN                     3   // Timer2 OC2B, auxiliary low-L buck path
#ifndef AUX_PWM_TIMER2_TOP
#define AUX_PWM_TIMER2_TOP              199U // 16 MHz / (199+1) = 80 kHz, prescaler 1
#endif
#define AUX_PWM_TIMER2_COUNTS           (AUX_PWM_TIMER2_TOP + 1U)
#define AUX_PWM_DUTY_QUANTUM_BP         ((10000UL + (AUX_PWM_TIMER2_COUNTS / 2U)) / AUX_PWM_TIMER2_COUNTS)
#define AUX_PATH_ARMED                  1
#define THERMISTOR_PIN                  A3  // Shared heatsink 10k NTC input
#define VOLTAGE_PIN                     A0  // Output voltage
#define CURRENT_PIN                     A1  // Output current ACS712-05B
#define SOURCE_SENSE_ENABLED            1
#define SOURCE_CURRENT_SENSE_ENABLED    1
#define AUX_EFFICIENCY_LEARNING_ENABLED 1
#define SOURCE_VOLTAGE_PIN              A2  // Input voltage
#define SOURCE_CURRENT_PIN              A4  // Input current ACS712-05B

// Optional direct Nano potentiometers. A6 is analog-input-only.
#ifndef OUTPUT_LIMIT_POT_PIN
#define OUTPUT_LIMIT_POT_PIN            -1
#endif
#ifndef INPUT_LIMIT_POT_PIN
#define INPUT_LIMIT_POT_PIN             -1
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
#define CAL_EXPECTED_CURRENT_CENTI       ((CAL_TARGET_V_CENTI * 1000L) / CAL_LOAD_RESISTANCE_MILLIOHM)
#define CAL_SAMPLE_COUNT                64U
#define CAL_STABLE_BAND_CENTI           25L
#define CAL_STABLE_TIME_MS              1000UL
#define CAL_TIMEOUT_MS                  30000UL
#define CAL_SNAPSHOT_MAX_AGE_MS         500UL
#define CAL_MIN_SOURCE_HEADROOM_PERCENT 112L
#define CAL_MIN_EFFICIENCY_PERMILLE     250L // reject grossly inconsistent DMM/sensor calibration readings
#define EFFICIENCY_MAX_SOURCE_VOLTAGE_CENTI 6000L
#define EFFICIENCY_MAX_CURRENT_CENTI        600L
#define CAL_DMM_VOLTAGE_TOLERANCE_CENTI 20L  // DMM Vout must be within 0.20 V of the 12 V target
#define CAL_DIVIDER_SCALE_TOL_PERCENT   10L  // reject wrong divider/configuration, allow resistor tolerance
#define CAL_CURRENT_CROSSCHECK_PERCENT  12L  // catches wrong load/sensor while allowing module noise/tolerance
#define CAL_OUTPUT_VOLTAGE_MAX_SIGMA_RAW 3U
#define CAL_OUTPUT_CURRENT_MAX_SIGMA_RAW 6U
#define CAL_SOURCE_VOLTAGE_MAX_SIGMA_RAW 4U
#define CAL_SOURCE_CURRENT_MAX_SIGMA_RAW 6U
#define CAL_OUTPUT_VOLTAGE_MAX_SPAN_RAW  16U
#define CAL_OUTPUT_CURRENT_MAX_SPAN_RAW  28U
#define CAL_SOURCE_VOLTAGE_MAX_SPAN_RAW  18U
#define CAL_SOURCE_CURRENT_MAX_SPAN_RAW  28U
#define CAL_NO_RESPONSE_DUTY_BP          2500L
#define CAL_NO_RESPONSE_MIN_V_CENTI      300L
#define CAL_NO_RESPONSE_TIMEOUT_MS       1200UL
#define RUN_NO_RESPONSE_DUTY_BP          2500L
#define RUN_NO_RESPONSE_MIN_REF_CENTI     500L
#define RUN_NO_RESPONSE_MIN_V_CENTI       300L
#define RUN_NO_RESPONSE_HEADROOM_PERCENT 112L
#define RUN_NO_RESPONSE_CURRENT_MARGIN_PERCENT 80L
#define RUN_NO_RESPONSE_TIMEOUT_MS       1200UL
#define CAL_SOFT_LIMIT_PERCENT            85L  // pause calibration before either user current limit

// Shared heatsink NTC: 10k at 25 C, Beta 3950, 10k pull-up to AVCC,
// thermistor from A3 to GND. All thresholds are heatsink temperatures.
#define THERMISTOR_OPEN_RAW             945U  // reject open or colder-than-calibrated NTC readings
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
#define EEPROM_CAL_VERSION              8U
#define EEPROM_CAL_COMMIT               0xA5U
#define EEPROM_CAL_SLOT_COUNT            2U
#define SERIAL_BAUD_RATE                 115200UL

#define ADC_REFERENCE_MV                5000UL // nominal AVCC; ACS712 supply must share AVCC
#define ADC_COUNTS                      1023UL
#ifndef VOLTAGE_DIVIDER_NUM
#define VOLTAGE_DIVIDER_NUM             4UL
#endif
#ifndef VOLTAGE_DIVIDER_DEN
#define VOLTAGE_DIVIDER_DEN             1UL
#endif
#ifndef SOURCE_DIVIDER_NUM
#define SOURCE_DIVIDER_NUM              5UL
#endif
#ifndef SOURCE_DIVIDER_DEN
#define SOURCE_DIVIDER_DEN              1UL
#endif

// ACS712ELCTR-05B-T (bidirectional +/-5 A) nominal sensitivity is 185 mV/A.
// Fit CF=47nF from each ACS712 FILTER pin to analog ground, plus local 100nF
// VCC bypass. Zero calibration thresholds below assume this filtered signal.
// Per-channel overrides permit calibration against a traceable current meter.
#ifndef ACS712_PRIMARY_SENSITIVITY_MV_PER_A
#define ACS712_PRIMARY_SENSITIVITY_MV_PER_A 185UL
#endif
#ifndef ACS712_SOURCE_SENSITIVITY_MV_PER_A
#define ACS712_SOURCE_SENSITIVITY_MV_PER_A  185UL // input-current ACS712-05B
#endif

#define ACS712_ZERO_SETTLE_MS                250UL
#define ACS712_ZERO_CAL_SAMPLES              128U
#define ACS712_ZERO_MIN_RAW                  480U
#define ACS712_ZERO_MAX_RAW                  544U
#define ACS712_ZERO_MAX_SPAN_RAW             12U
#define ACS712_ZERO_MAX_STDDEV_SQUARED        9UL
#define ACS712_REVERSE_CURRENT_TRIP_CENTI    25L

// -------------------------- Setpoints/protection --------------------------
#define TARGET_V_CENTI                  1200L
#ifndef OUTPUT_CURRENT_LIMIT_CENTI
#define OUTPUT_CURRENT_LIMIT_CENTI      400L // output/load limit in centiamps
#endif
#ifndef INPUT_CURRENT_LIMIT_CENTI
#define INPUT_CURRENT_LIMIT_CENTI       300L // independent source/input limit in centiamps
#endif
// ADC-sampled software trips; these are not cycle-by-cycle hardware limits.
#ifndef OUTPUT_HARD_OVERCURRENT_CENTI
#define OUTPUT_HARD_OVERCURRENT_CENTI   450L
#endif
#ifndef INPUT_HARD_OVERCURRENT_CENTI
#define INPUT_HARD_OVERCURRENT_CENTI    450L
#endif
#define CURRENT_LIMIT_CENTI             OUTPUT_CURRENT_LIMIT_CENTI // legacy internal alias
#define HARD_OVERCURRENT_CENTI          OUTPUT_HARD_OVERCURRENT_CENTI
#define INPUT_CURRENT_GUARD_START_PERCENT 90L
#define INPUT_CURRENT_GUARD_HARD_PERCENT  97L
#define INPUT_CURRENT_GUARD_RELEASE_PERCENT 85L

// PV-source MPPT searches by perturbing the allowed input-current/power
// ceiling. This is intentionally subordinate to the user input-current limit.
#ifndef MPPT_ENABLED
#define MPPT_ENABLED                    1
#endif
#ifndef MPPT_MAX_SOURCE_CURRENT_PERCENT
#define MPPT_MAX_SOURCE_CURRENT_PERCENT          85L      // maximum input-current target as % of user limit
#endif
#ifndef MPPT_MIN_INPUT_CURRENT_CENTI
#define MPPT_MIN_INPUT_CURRENT_CENTI             10L      // minimum tracked input current in centiamps
#endif
#ifndef MPPT_START_INPUT_CURRENT_CENTI
#define MPPT_START_INPUT_CURRENT_CENTI           50L      // startup input-current ceiling in centiamps
#endif
#ifndef MPPT_STEP_CENTI
#define MPPT_STEP_CENTI                          10L      // fixed 0.10 A perturbation
#endif
#ifndef MPPT_SETTLE_MS
#define MPPT_SETTLE_MS                           250UL    // settling time after changing the input-current ceiling
#endif
#define SOURCE_CURRENT_STEP_TRANSIENT_CENTI      20L      // input-current step; avoids learning across irradiance/source changes
#ifndef MPPT_MIN_SOURCE_VOLTAGE_CENTI
#define MPPT_MIN_SOURCE_VOLTAGE_CENTI            500L     // below 5 V: source is absent or collapsed
#endif
#ifndef MPPT_MIN_HEADROOM_PERCENT
#define MPPT_MIN_HEADROOM_PERCENT                112L     // buck requires Vin >= this % of Vout
#endif
#ifndef MPPT_LOAD_LIMIT_ERROR_CENTI
#define MPPT_LOAD_LIMIT_ERROR_CENTI              75L      // output-voltage error indicating downstream power demand
#endif
#ifndef MPPT_LOAD_LIMIT_CURRENT_PERCENT
#define MPPT_LOAD_LIMIT_CURRENT_PERCENT          70L      // source current below this % of target indicates load-limited
#endif
#ifndef MPPT_POWER_DEADBAND_PERMILLE
#define MPPT_POWER_DEADBAND_PERMILLE             10UL     // 1% relative P&O deadband
#endif
#ifndef MPPT_ASSUMED_EFFICIENCY_PERMILLE
#define MPPT_ASSUMED_EFFICIENCY_PERMILLE         600L     // conservative startup fallback until measured Pin/Pout is credible
#endif
#ifndef MPPT_EFFICIENCY_MARGIN_PERMILLE
#define MPPT_EFFICIENCY_MARGIN_PERMILLE          70L      // conservative margin below measured efficiency
#endif
#ifndef MPPT_EFFICIENCY_MIN_INPUT_PRODUCT
#define MPPT_EFFICIENCY_MIN_INPUT_PRODUCT        50000L  // 5 W minimum input power product in cV*cA units
#endif
#ifndef MPPT_EFFICIENCY_MIN_OUTPUT_PRODUCT
#define MPPT_EFFICIENCY_MIN_OUTPUT_PRODUCT       30000L  // 3 W minimum output power product in cV*cA units
#endif

#define CAL_DUTY_SLEW_BP_PER_SEC        1500L // 15% duty/s rising ramp
#define CAL_DUTY_FALL_BP_PER_SEC        6000L // 60% duty/s correction when output is above target
#define INPUT_CURRENT_BRAKE_STEP_BP       200L
#define OUTPUT_OVERVOLTAGE_CENTI        1300L // ADC-sampled software trip
#define ADC_RAIL_TRIP_RAW               1018U

// -------------------------- Timing/actuation ------------------------------
// Approximately 499 Hz control gives Uno arithmetic headroom. The 2003 us
// interval is intentionally incommensurate with 20 kHz / 80 kHz PWM periods,
// reducing coherent ADC sampling of the same switching-ripple phase.
#define CONTROL_PERIOD_US               2003UL // deliberate PWM phase walk; approximately 499.25 Hz
#if MPPT_ENABLED
#define MPPT_SETTLE_TICKS ((MPPT_SETTLE_MS * 1000UL + CONTROL_PERIOD_US - 1UL) / CONTROL_PERIOD_US)
#if MPPT_SETTLE_TICKS < 1UL || MPPT_SETTLE_TICKS >= 255UL
#error "MPPT_SETTLE_MS must fit an 8-bit control-tick countdown"
#endif
#endif
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
#define AUX_MODEL_MIN_EXCITATIONS        24U
#define AUX_IDENT_MAX_TOTAL_CURRENT_CENTI 60L
#ifndef AUX_CONTROL_SHARE_PERMILLE
#define AUX_CONTROL_SHARE_PERMILLE     150U  // auxiliary handles at most 15% of control increment
#endif
#ifndef AUX_MODEL_DUTY_SCALE_BP
#define AUX_MODEL_DUTY_SCALE_BP          5000L // initial branch-gain normalization for online identification
#endif

// Conservative PI bootstrap; used until model confidence is sufficient.
// Outer Kp units cA/cV; inner Kp units duty basis-points/cA.
#define BOOT_OUTER_KP_Q8                128L
#define BOOT_OUTER_KI_Q16               655L
#define BOOT_INNER_KP_Q8                256L
#define BOOT_INNER_KI_Q16               3277L
#define BOOT_DUTY_SLEW_BP               25L
#define BOOT_CURRENT_SLEW_CENTI         25L

// -------------------------- Fixed-point adaptive identification -------------------
#define MODEL_ONE_Q14                   16384L
#define MODEL_V_SCALE_CENTI             20L
#define MODEL_I_SCALE_CENTI             100L
#define MODEL_CURRENT_DEVIATION_CENTI   250L
#define MODEL_DUTY_SCALE_BP             500L
#define DUTY_MEAN_TAU_SAMPLES            256L
#define MODEL_SOURCE_SCALE_CENTI        50L
#define MODEL_SOURCE_OUTPUT_SCALE_CENTI  100L
#define MODEL_FEATURE_COUNT              4U
#define MODEL_SLOW_UPDATE_DIVIDER          8U
#define MODEL_MIN_EXCITATION_Q14        96L
#define MODEL_MIN_EXCITATIONS            24U
#define MODEL_MIN_GAIN_Q14              64L
#define MODEL_RESIDUAL_TRANSIENT_Q14    1800L
#define MODEL_RESIDUAL_BAD_Q14          4200L
#define MODEL_DISTURBANCE_LIMIT_Q14     8192L
#define MODEL_NLMS_FAST_MU_Q14          512L  // 3.125% normalized update
#define MODEL_NLMS_SLOW_MU_Q14          128L  // 0.781% update, sampled every 8 cycles

// -------------------------- Pattern/evaluation thresholds ----------------
#define VOLTAGE_ERROR_DEADBAND_CENTI    5L
#define CURRENT_STEP_TRANSIENT_CENTI    18L
#define SOURCE_STEP_TRANSIENT_CENTI     20L
#define VOLTAGE_SLOPE_WINDOW_TICKS       25U    // 25 * 2003 us ~= 50 ms
#define CROSSING_WINDOW_TICKS            500U   // 500 * 2003 us ~= 1 s
#define OSCILLATION_CROSSINGS           3U
#define FAST_APPROACH_SLOPE_CV_S        120L
#define APPROACH_BAND_CENTI             75L
#define OUTPUT_CURRENT_BOUND_PERCENT      92L
#define MAX_PREDICTED_DV_CENTI          100L
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
#define EFFICIENCY_COST_PER_10MW         20L

// Small zero-mean excitation, enabled only near regulation and away from limits.
#ifndef IDENTIFICATION_PROBE_ENABLED
#define IDENTIFICATION_PROBE_ENABLED    1
#endif
#define IDENTIFICATION_PROBE_BP         30L
#define IDENTIFICATION_PROBE_SLEW_BP    40L
#define IDENTIFICATION_PROBE_HOLD_TICKS 8U
#define IDENTIFICATION_PROBE_ERROR_BAND_CENTI 50L
#define IDENTIFICATION_PROBE_MAX_CURRENT_PERCENT 70L
#define IDENTIFICATION_PROBE_VOLTAGE_MARGIN_CENTI 100L

#if (OUTPUT_OVERVOLTAGE_CENTI * OUTPUT_HARD_OVERCURRENT_CENTI) > 2000000000L
#error "Output power product exceeds signed 32-bit power arithmetic"
#endif

#if ADC_REFERENCE_MV == 0 || ADC_COUNTS == 0
#error "ADC reference and ADC_COUNTS must be non-zero"
#endif
#if !SOURCE_SENSE_ENABLED || !SOURCE_CURRENT_SENSE_ENABLED
#error "This firmware release requires both source-voltage and source-current sensing for calibration, input protection and source-power measurement"
#endif
#if MPPT_MAX_SOURCE_CURRENT_PERCENT <= 0L || MPPT_MAX_SOURCE_CURRENT_PERCENT >= INPUT_CURRENT_GUARD_START_PERCENT
#error "MPPT current ceiling must remain positive and below the independent input-current guard threshold"
#endif
// ADC-calibrated source power is bounded below 3.7 MW in cV*cA units;
// eight samples fit safely in uint32_t.
#if MPPT_ENABLED && MPPT_POWER_DEADBAND_PERMILLE > 100U
#error "MPPT deadband must not exceed 10 percent"
#endif
#if MPPT_ENABLED && (MPPT_ASSUMED_EFFICIENCY_PERMILLE <= 0L || MPPT_ASSUMED_EFFICIENCY_PERMILLE >= 1000L || MPPT_EFFICIENCY_MARGIN_PERMILLE >= 1000L)
#error "Invalid MPPT efficiency prior or margin"
#endif
#if MPPT_ENABLED && MPPT_MIN_HEADROOM_PERCENT <= 100L
#error "Buck MPPT headroom threshold must exceed 100 percent of output voltage"
#endif
#if OUTPUT_POT_MIN_CENTI <= 0L || OUTPUT_POT_MAX_CENTI < OUTPUT_POT_MIN_CENTI
#error "Invalid output current potentiometer range"
#endif
#if INPUT_POT_MIN_CENTI <= 0L || INPUT_POT_MAX_CENTI < INPUT_POT_MIN_CENTI
#error "Invalid input current potentiometer range"
#endif
#if CAL_DUTY_SLEW_BP_PER_SEC <= 0L || CAL_DUTY_FALL_BP_PER_SEC < CAL_DUTY_SLEW_BP_PER_SEC
#error "Calibration duty fall slew must be positive and no slower than its rising slew"
#endif
#if RUN_NO_RESPONSE_DUTY_BP <= 0L || RUN_NO_RESPONSE_TIMEOUT_MS < CONTROL_PERIOD_US / 1000UL || RUN_NO_RESPONSE_HEADROOM_PERCENT <= 100L || RUN_NO_RESPONSE_CURRENT_MARGIN_PERCENT <= 0L || RUN_NO_RESPONSE_CURRENT_MARGIN_PERCENT >= 100L
#error "Invalid runtime output-response monitor configuration"
#endif
#if CAL_LOAD_RESISTANCE_MILLIOHM <= 0L || CAL_CURRENT_CROSSCHECK_PERCENT <= 0L || CAL_CURRENT_CROSSCHECK_PERCENT >= 100L
#error "Invalid 10-ohm calibration load or ACS712 cross-check tolerance"
#endif
#if ACS712_ZERO_CAL_SAMPLES < 32U || ACS712_ZERO_CAL_SAMPLES > 512U
#error "ACS712 zero calibration sample count must be between 32 and 512"
#endif
#if VOLTAGE_DIVIDER_NUM == 0 || VOLTAGE_DIVIDER_DEN == 0
#error "Invalid voltage-divider calibration"
#endif
#if ACS712_PRIMARY_SENSITIVITY_MV_PER_A == 0 || ACS712_PRIMARY_SENSITIVITY_MV_PER_A > 1000UL
#error "Invalid primary ACS712 sensitivity"
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
#if SOURCE_DIVIDER_NUM == 0 || SOURCE_DIVIDER_DEN == 0
#error "Invalid source-voltage divider calibration"
#endif
#if SOURCE_CURRENT_SENSE_ENABLED && (ACS712_SOURCE_SENSITIVITY_MV_PER_A == 0 || ACS712_SOURCE_SENSITIVITY_MV_PER_A > 1000UL)
#error "Invalid source ACS712 sensitivity"
#endif
#if EFFICIENCY_CURRENT_BIN_CENTI <= 0 || EFFICIENCY_BLOCK_SAMPLES == 0
#error "Invalid empirical efficiency binning"
#endif
#if MODEL_SLOW_UPDATE_DIVIDER == 0U || MODEL_SLOW_UPDATE_DIVIDER > 256U || (MODEL_SLOW_UPDATE_DIVIDER & (MODEL_SLOW_UPDATE_DIVIDER - 1U)) != 0U
#error "MODEL_SLOW_UPDATE_DIVIDER must be a power of two from 1 through 256"
#endif
#if AUX_PATH_ARMED && !defined(__AVR_ATmega328P__)
#error "Dual-PWM backend requires an ATmega328P Nano/Uno target"
#endif
#if defined(__AVR_ATmega328P__) && (!defined(F_CPU) || (F_CPU != 16000000UL))
#error "Dual-PWM configuration requires a 16 MHz ATmega328P"
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
#if CAL_MIN_EFFICIENCY_PERMILLE <= 0L || CAL_MIN_EFFICIENCY_PERMILLE >= 1000L
#error "Calibration efficiency plausibility floor must be between 0 and 1000 permille"
#endif
#if (6000L * INPUT_HARD_OVERCURRENT_CENTI * CAL_MIN_EFFICIENCY_PERMILLE) > 2147483647L
#error "Calibration input-power plausibility product exceeds signed 32-bit range"
#endif
#if (EFFICIENCY_MAX_SOURCE_VOLTAGE_CENTI * EFFICIENCY_MAX_CURRENT_CENTI) > 2147483647L
#error "Efficiency input power product exceeds signed 32-bit range"
#endif
#if (OUTPUT_OVERVOLTAGE_CENTI * EFFICIENCY_MAX_CURRENT_CENTI * 1000L) > 2147483647L
#error "Efficiency output power ratio exceeds signed 32-bit range"
#endif
#if ADC_REFERENCE_MV > 167772UL
#error "ADC_REFERENCE_MV makes nominal sensor-scale arithmetic overflow"
#endif
#if ((CAL_TARGET_V_CENTI + CAL_DMM_VOLTAGE_TOLERANCE_CENTI) * CAL_EXPECTED_CURRENT_CENTI * 1000L) > 2147483647L
#error "Calibration output-power plausibility product exceeds signed 32-bit range"
#endif
#if CAL_DMM_VOLTAGE_TOLERANCE_CENTI <= 0L || CAL_DIVIDER_SCALE_TOL_PERCENT <= 0L || CAL_DIVIDER_SCALE_TOL_PERCENT > 35L
#error "Invalid calibration plausibility tolerances"
#endif
#if CAL_SAMPLE_COUNT != 64U
#error "Calibration window statistics currently require exactly 64 samples"
#endif
#if CAL_SOFT_LIMIT_PERCENT <= 0L || CAL_SOFT_LIMIT_PERCENT >= 100L
#error "Calibration soft-limit margin must be between 0 and 100 percent"
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
  FAULT_SENSOR_CALIBRATION,
  FAULT_REVERSE_CURRENT,
  FAULT_THERMISTOR,
  FAULT_OVERTEMPERATURE,
  FAULT_EEPROM,
  FAULT_VOLTAGE_NO_RESPONSE
};



enum ControlRegime : uint8_t {
  REGIME_NORMAL = 0,
  REGIME_FAST,
  REGIME_DAMPED,
  REGIME_CURRENT_BOUND
};

struct AdaptiveModel {
  int16_t thetaQ14[MODEL_FEATURE_COUNT];
  int16_t disturbanceQ14;
  int16_t residualAbsQ14;
  uint8_t featureExcitationCount[MODEL_FEATURE_COUNT];
};
static_assert(sizeof(AdaptiveModel) <= 16U, "Adaptive model RAM budget exceeded");


// EEPROM record is deliberately compact and versioned. CRC excludes crc16 and
// commit; commit is written last so an interrupted update cannot become valid.
struct __attribute__((packed)) CalibrationRecord {
  uint16_t magic;
  uint8_t version;
  uint8_t length;
  uint16_t sequence;
  uint32_t configurationSignature;
  uint32_t loadResistanceMilliohm;
  int32_t currentZeroRawQ8; // output ACS712 zero
  int16_t currentGainCentiPerCountQ8; // signed output gain, Q8 cA/ADC-count
  int32_t sourceCurrentZeroRawQ8; // input ACS712 zero
  int16_t sourceCurrentGainCentiPerCountQ8; // signed input gain, Q8
  uint16_t voltageCentiPerCountQ8; // output divider scale, Q8 cV/ADC-count
  uint16_t sourceCentiPerCountQ8; // input divider scale, Q8 cV/ADC-count
  uint16_t calibrationOutputVoltageCenti;
  uint16_t calibrationSourceVoltageCenti;
  uint16_t calibrationInputCurrentCenti;
  uint16_t calibrationRawMean[4]; // Vout, Iout, Vin, Iin ADC means
  uint8_t outputZeroSpanRaw;
  uint8_t inputZeroSpanRaw;
  uint8_t calibrationQuality;
  uint16_t crc16;
  uint8_t commit;
};
static_assert(sizeof(CalibrationRecord) <= 64U,
              "Compact calibration record unexpectedly grew");

struct CalibrationWindowSnapshot {
  uint16_t meanRaw[4]; // Vout, Iout, Vin, Iin
  uint8_t spanRaw[4];
  uint32_t capturedAtMs;
  bool valid;
};

struct CalibrationWindowAccumulator {
  uint16_t sum[4];
  uint32_t sumSquares[4];
  uint16_t minimum[4];
  uint16_t maximum[4];
  uint8_t samples;
};
static_assert((uint64_t)CAL_SAMPLE_COUNT * ADC_COUNTS <= UINT16_MAX,
              "Calibration sample sum no longer fits uint16_t");
static_assert((uint64_t)CAL_SAMPLE_COUNT * CAL_SAMPLE_COUNT *
                  ADC_COUNTS * ADC_COUNTS <= UINT32_MAX,
              "Calibration variance numerator no longer fits uint32_t");

#if defined(__AVR_ATmega328P__)
static_assert(sizeof(CalibrationRecord) * EEPROM_CAL_SLOT_COUNT <= 1024U,
              "Two calibration records must fit the ATmega328P EEPROM");
#endif

enum CalibrationWizardState : uint8_t {
  CAL_WIZARD_READY = 0,
  CAL_WIZARD_WAIT_ZERO,
  CAL_WIZARD_ZERO_SETTLE,
  CAL_WIZARD_WAIT_LOAD,
  CAL_WIZARD_RAMP,
  CAL_WIZARD_WAIT_DMM,
  CAL_WIZARD_FAILED
};

enum CalibrationSaveResult : uint8_t {
  CAL_SAVE_OK = 0,
  CAL_SAVE_INVALID,
  CAL_SAVE_IO_ERROR
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
static uint32_t runNoResponseStartMs = 0UL;
static bool runNoResponseTiming = false;
static uint8_t modelUpdatePhase = 0U;
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
static int32_t latestOutputCurrentCenti = 0L;
static int32_t latestInputCurrentCenti = 0L;
#if MPPT_ENABLED
static uint8_t efficiencyMeasurementDivider = 0U;
#endif
static int32_t previousDeltaVoltageCenti = 0L;
static int32_t previousDeltaCurrentCenti = 0L;
static int32_t previousVoltageCenti = 0L;
static int32_t previousCurrentCenti = 0L;
static int32_t previousCurrentCenteredCenti = 0L;
static int32_t previousSourceCenti = 0L;
static int32_t previousSourceCurrentCenti = 0L;
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
static bool loadEstimateInitialized = false;
static bool loadEstimateValid = false;

static AdaptiveModel currentModel;
static AdaptiveModel voltageModel;
static uint32_t lastControlUs = 0UL;

static int32_t trendWindowStartVoltage = 0L;
static int32_t voltageSlopeCentiPerSec = 0L;
static uint8_t trendWindowTicks = 0U;
static uint16_t crossingWindowTicks = 0U;
static uint8_t crossingCount = 0U;
static uint8_t transientHoldCounter = 0U;
static uint8_t nonlinearResidualCounter = 0U;
static int8_t previousErrorSign = 0;

// -------------------------- Bounded arithmetic ---------------------------
static FW_NOINLINE int32_t clampI32(int32_t value, int32_t low, int32_t high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static FW_NOINLINE int16_t clampI16(int32_t value, int16_t low, int16_t high) {
  if (value < (int32_t)low) return low;
  if (value > (int32_t)high) return high;
  return (int16_t)value;
}

static FW_NOINLINE int32_t absI32(int32_t value) {
  if (value == INT32_MIN) return INT32_MAX;
  return (value < 0L) ? -value : value;
}

static FW_NOINLINE bool sourceCurrentStepDetected(int32_t deltaCurrentCenti) {
  return absI32(deltaCurrentCenti) >= SOURCE_CURRENT_STEP_TRANSIENT_CENTI;
}

static FW_NOINLINE uint32_t calibrationConfigurationSignature() {
  uint32_t hash = 2166136261UL;
  const uint32_t values[] = {
    ADC_REFERENCE_MV, ADC_COUNTS,
    VOLTAGE_DIVIDER_NUM, VOLTAGE_DIVIDER_DEN,
    SOURCE_DIVIDER_NUM, SOURCE_DIVIDER_DEN,
    ACS712_PRIMARY_SENSITIVITY_MV_PER_A,
    ACS712_SOURCE_SENSITIVITY_MV_PER_A,
    (uint32_t)CAL_LOAD_RESISTANCE_MILLIOHM,
    (uint32_t)CAL_TARGET_V_CENTI,
    (uint32_t)CAL_DMM_VOLTAGE_TOLERANCE_CENTI,
    (uint32_t)CAL_DIVIDER_SCALE_TOL_PERCENT,
    (uint32_t)CAL_CURRENT_CROSSCHECK_PERCENT,
    (uint32_t)CAL_MIN_SOURCE_HEADROOM_PERCENT,
    (uint32_t)CAL_MIN_EFFICIENCY_PERMILLE,
    (uint32_t)CAL_SAMPLE_COUNT,
    (uint32_t)CAL_SOFT_LIMIT_PERCENT,
    (uint32_t)ACS712_ZERO_CAL_SAMPLES,
    (uint32_t)ACS712_ZERO_MIN_RAW, (uint32_t)ACS712_ZERO_MAX_RAW,
    (uint32_t)ACS712_ZERO_MAX_SPAN_RAW,
    (uint32_t)ACS712_ZERO_MAX_STDDEV_SQUARED,
    (uint32_t)OUTPUT_HARD_OVERCURRENT_CENTI,
    (uint32_t)INPUT_HARD_OVERCURRENT_CENTI,
    (uint32_t)SOURCE_SENSE_ENABLED,
    (uint32_t)SOURCE_CURRENT_SENSE_ENABLED,
    (uint32_t)VOLTAGE_PIN, (uint32_t)CURRENT_PIN,
    (uint32_t)SOURCE_VOLTAGE_PIN, (uint32_t)SOURCE_CURRENT_PIN,
    (uint32_t)THERMISTOR_PIN
  };
  for (uint8_t i = 0U; i < (uint8_t)(sizeof(values) / sizeof(values[0])); ++i) {
    hash ^= values[i];
    hash *= 16777619UL;
  }
#ifdef F_CPU
  hash ^= (uint32_t)F_CPU;
  hash *= 16777619UL;
#endif
  return hash;
}

static FW_NOINLINE int16_t nominalAcsGainCentiPerCountQ8(uint16_t sensitivityMvPerA) {
  const uint32_t numerator = ADC_REFERENCE_MV * 100UL * 256UL;
  const uint32_t denominator = ADC_COUNTS * (uint32_t)sensitivityMvPerA;
  if (denominator == 0UL) return 0;
  if (numerator > UINT32_MAX - denominator / 2U) return 0;
  const uint32_t result = (uint32_t)((numerator + denominator / 2U) / denominator);
  // An unrepresentable or implausible scale is invalid; saturation here could
  // turn a bad sensor configuration into a superficially valid calibration.
  return (result == 0UL || result > 32767UL) ? 0 : (int16_t)result;
}

static FW_NOINLINE bool withinPercentOf(int32_t measured, int32_t reference,
                            uint8_t tolerancePercent) {
  if (measured <= 0L || reference <= 0L) return false;
  const int32_t difference = (measured >= reference)
      ? measured - reference : reference - measured;
  return difference * 100L <= reference * (int32_t)tolerancePercent;
}

static FW_NOINLINE bool calibrationPowerPlausible(int32_t inputVoltageCenti,
                                      int32_t inputCurrentCenti,
                                      int32_t outputVoltageCenti,
                                      int32_t outputCurrentCenti) {
  if (inputVoltageCenti <= 0L || inputCurrentCenti <= 0L ||
      outputVoltageCenti <= 0L || outputCurrentCenti <= 0L) return false;
  const int32_t inputPowerProduct = inputVoltageCenti * inputCurrentCenti;
  const int32_t outputPowerProduct = outputVoltageCenti * outputCurrentCenti;
  if (inputPowerProduct <= 0L || outputPowerProduct > inputPowerProduct)
    return false;
  return outputPowerProduct * 1000L >=
      inputPowerProduct * (int32_t)CAL_MIN_EFFICIENCY_PERMILLE;
}

static FW_NOINLINE int16_t toQ14(int32_t value, int32_t scale) {
  if (scale <= 0L) return 0;
  if (value >= scale) return (int16_t)MODEL_ONE_Q14;
  if (value <= -scale) return (int16_t)-MODEL_ONE_Q14;
  // The range check above bounds value * 16384 to at most ~2^29 for
  // every positive signed 32-bit scale.
  return (int16_t)((value * MODEL_ONE_Q14) / scale);
}

static int32_t fromQ14(int32_t valueQ14, int32_t scale) {
  // Model states are bounded by four Q14 terms plus a bounded disturbance.
  return (valueQ14 * scale) / MODEL_ONE_Q14;
}

static FW_NOINLINE int32_t filterQ8(int32_t sample, int32_t &stateQ8) {
  // All ADC-derived engineering values are bounded well below INT32_MAX/256.
  const int32_t sampleQ8 = sample * 256L;
  stateQ8 += (sampleQ8 - stateQ8) / 2;
  return stateQ8 / 256L;
}

static int32_t limitSlew(int32_t desired, int32_t current, int32_t maxStep) {
  return clampI32(desired, current - maxStep, current + maxStep);
}

static FW_NOINLINE int32_t quantizePrimaryDutyBp(int32_t dutyBp) {
  const int32_t bounded = clampI32(dutyBp, MIN_DUTY_BASIS_POINTS,
                                    MAX_DUTY_BASIS_POINTS);
#if defined(__AVR_ATmega328P__)
  const uint16_t compare = (uint16_t)(((int32_t)bounded * 800L + 5000L) /
                                      10000L);
  return (int32_t)(((uint32_t)compare * 10000UL + 400UL) / 800UL);
#else
  const uint8_t compare = (uint8_t)(((int32_t)bounded * 255L + 5000L) /
                                     10000L);
  return (int32_t)(((uint16_t)compare * 10000U + 127U) / 255U);
#endif
}

static FW_NOINLINE int32_t quantizeAuxiliaryDutyBp(int32_t dutyBp) {
#if AUX_PATH_ARMED
  const int32_t bounded = clampI32(dutyBp, MIN_DUTY_BASIS_POINTS,
                                    AUX_MAX_DUTY_BASIS_POINTS);
#if defined(__AVR_ATmega328P__)
  const uint16_t compare = (uint16_t)(
      ((uint32_t)bounded * AUX_PWM_TIMER2_COUNTS + 5000UL) / 10000UL);
  return (int32_t)(((uint32_t)compare * 10000UL +
                    (AUX_PWM_TIMER2_COUNTS / 2U)) / AUX_PWM_TIMER2_COUNTS);
#else
  const uint8_t compare = (uint8_t)(((int32_t)bounded * 255L + 5000L) /
                                     10000L);
  return (int32_t)(((uint16_t)compare * 10000U + 127U) / 255U);
#endif
#else
  (void)dutyBp;
  return 0L;
#endif
}

// -------------------------- ADC conversion -------------------------------
// Q8 is more than sufficient for 10-bit ADC calibration: one coefficient
// LSB is 1/256 of a centivolt per ADC count. This keeps every runtime ADC
// conversion in bounded 32-bit arithmetic and stores the scale in 16 bits.
static FW_NOINLINE uint16_t nominalVoltageScaleQ8(uint32_t dividerNumerator,
                                      uint32_t dividerDenominator) {
  const uint32_t numeratorBase = ADC_REFERENCE_MV * 100U * 256U;
  const uint32_t denominatorBase = ADC_COUNTS * 1000U;
  if (dividerNumerator == 0U || dividerDenominator == 0U ||
      dividerNumerator > UINT32_MAX / numeratorBase ||
      dividerDenominator > UINT32_MAX / denominatorBase) return 0U;
  const uint32_t numerator = numeratorBase * dividerNumerator;
  const uint32_t denominator = denominatorBase * dividerDenominator;
  if (denominator == 0U || numerator > UINT32_MAX - denominator / 2U)
    return 0U;
  const uint32_t result = (numerator + denominator / 2U) / denominator;
  return (result > 65535U) ? 0U : (uint16_t)result;
}

static uint16_t voltageCentiPerCountQ8 = 0U;

static FW_NOINLINE int32_t adcToVoltageCenti(uint16_t raw) {
  return (int32_t)(((uint32_t)raw * voltageCentiPerCountQ8 + 128UL) >> 8);
}

// Runtime zero offsets are Q8 ADC counts. Q8 storage preserves fractional
// average-count offsets from the startup calibration window.
static int32_t primaryCurrentZeroRawQ8 = 0L;
static int16_t currentGainCentiPerCountQ8 = 0;
static int32_t sourceCurrentZeroRawQ8 = 0L;
static int16_t sourceCurrentGainCentiPerCountQ8 = 0;
static uint16_t sourceCentiPerCountQ8 = 0U;
static uint16_t calibrationSequence = 0U;
static int8_t activeEepromSlot = -1;
static bool currentSensorsCalibrated = false;
static CalibrationWizardState calibrationWizardState = CAL_WIZARD_WAIT_ZERO;
static uint32_t calibrationStateStartMs = 0UL;
static uint32_t calibrationStableStartMs = 0UL;
static int32_t calibrationDutyBp = 0L;
// Unquantized Q16 duty command prevents the 3 bp/slew update from being
// rounded to zero on every 12.5 bp Timer1 count.
static int32_t calibrationDutyCommandQ16 = 0L;
static int32_t calibrationLastVoltageCenti = -1L;
static uint8_t calibrationOutputZeroSpanRaw = 0U;
static uint8_t calibrationInputZeroSpanRaw = 0U;
static CalibrationWindowAccumulator calibrationWindow = {};
static CalibrationWindowSnapshot calibrationSnapshot = {};
static uint32_t calibrationNoResponseStartMs = 0UL;
static bool calibrationWindowReportSent = false;
static char calibrationCommandBuffer[32] = {};
static_assert(sizeof(calibrationCommandBuffer) >= 24U,
              "Calibration command buffer cannot hold the canonical full-width DMM command");
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

#if MPPT_ENABLED
// Zero sentinels replace separate valid/initialised flags. Settling is counted
// in control ticks (125 ticks is approximately 250 ms at 2003 us/tick).
static uint16_t mpptInputCurrentTargetCenti = 0U;
static uint16_t mpptOutputCurrentLimitCenti = OUTPUT_CURRENT_LIMIT_CENTI;
static uint16_t mpptEfficiencyEstimatePermille = 0U; // zero selects the conservative prior
static uint32_t mpptPreviousPowerProduct = 0UL;      // measured Pin; zero means no baseline
static uint8_t mpptSettleTicks = 0U;
static int8_t mpptDirection = 1;
#endif
// Last modelled primary-only duty requirement. It remains meaningful while
// the primary switch is off in auxiliary-only mode.
static int32_t primaryEquivalentDutyDemandBp = 0L;

static FW_NOINLINE int32_t adcToAcs712CurrentCenti(uint16_t raw,
                                      int32_t zeroRawQ8,
                                      uint16_t sensitivityMvPerA,
                                      int8_t polarity) {
  const int32_t deltaRawQ8 = (int32_t)((int32_t)raw * 256 - zeroRawQ8);
  const int32_t gainQ8 = nominalAcsGainCentiPerCountQ8(sensitivityMvPerA);
  // raw is a 10-bit ADC value, and supported ACS712 gains are < 1,000 Q8.
  // This nominal conversion is used only for calibration plausibility checks.
  int32_t current = (deltaRawQ8 * gainQ8) / 65536L;
  current *= polarity;
  return clampI32(current, -1000L, 1000L);
}

static FW_NOINLINE int32_t adcToCurrentCenti(uint16_t raw) {
  const int32_t deltaRawQ8 = (int32_t)((int32_t)raw * 256 - primaryCurrentZeroRawQ8);
  const int32_t measured =
      (deltaRawQ8 * (int32_t)currentGainCentiPerCountQ8) / 65536L;
  // ACS712-05B is only specified over +/-5 A. A wider +/-6 A software bound
  // ensures sensor saturation still trips the 4.5 A software threshold while
  // keeping all subsequent power products bounded for 32-bit arithmetic.
  return clampI32(measured, -600L, 600L);
}

#if SOURCE_SENSE_ENABLED
static FW_NOINLINE int32_t adcToSourceCenti(uint16_t raw) {
  return (int32_t)(((uint32_t)raw * sourceCentiPerCountQ8 + 128UL) >> 8);
}
#endif

#if SOURCE_CURRENT_SENSE_ENABLED
static FW_NOINLINE int32_t adcToSourceCurrentCenti(uint16_t raw) {
  const int32_t deltaRawQ8 = (int32_t)((int32_t)raw * 256 - sourceCurrentZeroRawQ8);
  const int32_t measured =
      (deltaRawQ8 * (int32_t)sourceCurrentGainCentiPerCountQ8) / 65536L;
  return clampI32(measured, -600L, 600L);
}
#endif

static FW_NOINLINE uint16_t calibrationCrc16(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFFU;
  for (uint8_t i = 0U; i < length; ++i) {
    crc = (uint16_t)(crc ^ (uint16_t)((uint16_t)data[i] << 8U));
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      const uint16_t shifted = (uint16_t)(crc << 1U);
      crc = (crc & 0x8000U)
          ? (uint16_t)(shifted ^ (uint16_t)0x1021U)
          : shifted;
    }
  }
  return crc;
}

static FW_NOINLINE bool calibrationRecordValid(const CalibrationRecord &record) {
  if (record.magic != EEPROM_CAL_MAGIC ||
      record.version != EEPROM_CAL_VERSION ||
      record.length != sizeof(CalibrationRecord) ||
      record.commit != EEPROM_CAL_COMMIT ||
      record.configurationSignature != calibrationConfigurationSignature() ||
      record.loadResistanceMilliohm != (uint32_t)CAL_LOAD_RESISTANCE_MILLIOHM ||
      record.calibrationQuality != 0x07U) return false;

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

  const int32_t outputCalibrationDeltaQ8 = (int32_t)(
      (int32_t)record.calibrationRawMean[1] * 256 - record.currentZeroRawQ8);
  const int32_t inputCalibrationDeltaQ8 = (int32_t)(
      (int32_t)record.calibrationRawMean[3] * 256 - record.sourceCurrentZeroRawQ8);
  // Signed gain preserves sensor orientation. A valid forward calibration
  // measurement and its stored gain must therefore have matching signs.
  if (outputCalibrationDeltaQ8 == 0L || inputCalibrationDeltaQ8 == 0L ||
      ((outputCalibrationDeltaQ8 > 0L) !=
       (record.currentGainCentiPerCountQ8 > 0)) ||
      ((inputCalibrationDeltaQ8 > 0L) !=
       (record.sourceCurrentGainCentiPerCountQ8 > 0))) return false;

  const int32_t outputGain = absI32(record.currentGainCentiPerCountQ8);
  const int32_t inputGain = absI32(record.sourceCurrentGainCentiPerCountQ8);
  const int32_t nominalOutputGain = nominalAcsGainCentiPerCountQ8(
      (uint16_t)ACS712_PRIMARY_SENSITIVITY_MV_PER_A);
  const int32_t nominalInputGain = nominalAcsGainCentiPerCountQ8(
      (uint16_t)ACS712_SOURCE_SENSITIVITY_MV_PER_A);
  if (!withinPercentOf(outputGain, nominalOutputGain, 35U) ||
      !withinPercentOf(inputGain, nominalInputGain, 35U)) return false;

  const uint32_t nominalOutputScale =
      nominalVoltageScaleQ8(VOLTAGE_DIVIDER_NUM, VOLTAGE_DIVIDER_DEN);
  const uint32_t nominalInputScale =
      nominalVoltageScaleQ8(SOURCE_DIVIDER_NUM, SOURCE_DIVIDER_DEN);
  if (record.voltageCentiPerCountQ8 <
          (nominalOutputScale * (100UL - CAL_DIVIDER_SCALE_TOL_PERCENT)) / 100UL ||
      record.voltageCentiPerCountQ8 >
          (nominalOutputScale * (100UL + CAL_DIVIDER_SCALE_TOL_PERCENT)) / 100UL ||
      record.sourceCentiPerCountQ8 <
          (nominalInputScale * (100UL - CAL_DIVIDER_SCALE_TOL_PERCENT)) / 100UL ||
      record.sourceCentiPerCountQ8 >
          (nominalInputScale * (100UL + CAL_DIVIDER_SCALE_TOL_PERCENT)) / 100UL)
    return false;

  if (record.calibrationOutputVoltageCenti <
          (uint16_t)(CAL_TARGET_V_CENTI - CAL_DMM_VOLTAGE_TOLERANCE_CENTI) ||
      record.calibrationOutputVoltageCenti >
          (uint16_t)(CAL_TARGET_V_CENTI + CAL_DMM_VOLTAGE_TOLERANCE_CENTI) ||
      record.calibrationSourceVoltageCenti <
          (uint16_t)((CAL_TARGET_V_CENTI * CAL_MIN_SOURCE_HEADROOM_PERCENT) / 100L) ||
      record.calibrationInputCurrentCenti < 20U ||
      record.calibrationInputCurrentCenti >= (uint16_t)INPUT_HARD_OVERCURRENT_CENTI)
    return false;

  for (uint8_t i = 0U; i < 4U; ++i) {
    if (record.calibrationRawMean[i] < 50U ||
        record.calibrationRawMean[i] > 950U) return false;
  }
  if (record.calibrationRawMean[2] > 900U ||
      record.outputZeroSpanRaw > ACS712_ZERO_MAX_SPAN_RAW ||
      record.inputZeroSpanRaw > ACS712_ZERO_MAX_SPAN_RAW) return false;

  const int32_t expectedOutputCurrent = (int32_t)(
      ((int32_t)record.calibrationOutputVoltageCenti * 1000) /
      (int32_t)record.loadResistanceMilliohm);
  const int32_t nominalOutputCurrent = absI32(adcToAcs712CurrentCenti(
      record.calibrationRawMean[1], record.currentZeroRawQ8,
      (uint16_t)ACS712_PRIMARY_SENSITIVITY_MV_PER_A, 1));
  const int32_t nominalInputCurrent = absI32(adcToAcs712CurrentCenti(
      record.calibrationRawMean[3], record.sourceCurrentZeroRawQ8,
      (uint16_t)ACS712_SOURCE_SENSITIVITY_MV_PER_A, 1));
  if (!withinPercentOf(nominalOutputCurrent, expectedOutputCurrent,
                       (uint8_t)CAL_CURRENT_CROSSCHECK_PERCENT) ||
      !withinPercentOf(nominalInputCurrent,
                       record.calibrationInputCurrentCenti,
                       (uint8_t)CAL_CURRENT_CROSSCHECK_PERCENT)) return false;

  const int32_t reconstructedVout = (int32_t)(
      ((uint32_t)record.calibrationRawMean[0] *
       record.voltageCentiPerCountQ8 + 128U) >> 8);
  const int32_t reconstructedVin = (int32_t)(
      ((uint32_t)record.calibrationRawMean[2] *
       record.sourceCentiPerCountQ8 + 128U) >> 8);
  if (absI32(reconstructedVout - record.calibrationOutputVoltageCenti) > 2L ||
      absI32(reconstructedVin - record.calibrationSourceVoltageCenti) > 2L)
    return false;
  if (!calibrationPowerPlausible(record.calibrationSourceVoltageCenti,
          record.calibrationInputCurrentCenti,
          record.calibrationOutputVoltageCenti, expectedOutputCurrent))
    return false;
  return true;
}

static FW_NOINLINE bool calibrationSequenceNewer(uint16_t lhs, uint16_t rhs) {
  return (int16_t)(lhs - rhs) > 0;
}

static FW_NOINLINE void applyCalibrationRecord(const CalibrationRecord &record,
                                   uint8_t slot) {
  calibrationSequence = record.sequence;
  activeEepromSlot = (int8_t)slot;
  primaryCurrentZeroRawQ8 = record.currentZeroRawQ8;
  currentGainCentiPerCountQ8 = record.currentGainCentiPerCountQ8;
  sourceCurrentZeroRawQ8 = record.sourceCurrentZeroRawQ8;
  sourceCurrentGainCentiPerCountQ8 =
      record.sourceCurrentGainCentiPerCountQ8;
  voltageCentiPerCountQ8 = record.voltageCentiPerCountQ8;
#if SOURCE_SENSE_ENABLED
  sourceCentiPerCountQ8 = record.sourceCentiPerCountQ8;
#endif
  calibrationOutputZeroSpanRaw = record.outputZeroSpanRaw;
  calibrationInputZeroSpanRaw = record.inputZeroSpanRaw;
  currentSensorsCalibrated = true;
}

static FW_NOINLINE bool loadCalibrationFromEeprom() {
  if ((uint32_t)EEPROM.length() <
      (uint32_t)EEPROM_CAL_SLOT_COUNT * sizeof(CalibrationRecord)) return false;

  // Read one EEPROM slot at a time. This avoids a two-record (100-byte) local
  // array on the 2 KB Uno stack. A valid older slot is applied first; a newer
  // valid slot replaces it. Invalid or interrupted records are ignored.
  CalibrationRecord record = {};
  bool found = false;
  for (uint8_t slot = 0U; slot < EEPROM_CAL_SLOT_COUNT; ++slot) {
    EEPROM.get((int)(slot * sizeof(CalibrationRecord)), record);
    if (!calibrationRecordValid(record)) continue;
    if (!found || calibrationSequenceNewer(record.sequence,
                                            calibrationSequence)) {
      applyCalibrationRecord(record, slot);
      found = true;
    }
  }
  return found;
}

static FW_NOINLINE CalibrationSaveResult saveCalibrationToEeprom(
    CalibrationRecord &record) {
  if ((uint32_t)EEPROM.length() <
      (uint32_t)EEPROM_CAL_SLOT_COUNT * sizeof(CalibrationRecord))
    return CAL_SAVE_IO_ERROR;

  record.magic = EEPROM_CAL_MAGIC;
  record.version = EEPROM_CAL_VERSION;
  record.length = (uint8_t)sizeof(CalibrationRecord);
  record.configurationSignature = calibrationConfigurationSignature();
  record.loadResistanceMilliohm = (uint32_t)CAL_LOAD_RESISTANCE_MILLIOHM;
  record.sequence = (activeEepromSlot < 0)
      ? 1U : (uint16_t)(calibrationSequence + 1U);
  record.crc16 = calibrationCrc16(
      (const uint8_t *)&record,
      (uint8_t)offsetof(CalibrationRecord, crc16));
  record.commit = EEPROM_CAL_COMMIT;
  if (!calibrationRecordValid(record)) return CAL_SAVE_INVALID;

  const uint8_t targetSlot = (activeEepromSlot == 0) ? 1U : 0U;
  const int base = (int)(targetSlot * sizeof(CalibrationRecord));
  const uint8_t *bytes = (const uint8_t *)&record;
  EEPROM.update(base + (int)offsetof(CalibrationRecord, commit), 0U);
  for (uint8_t i = 0U;
       i < (uint8_t)offsetof(CalibrationRecord, commit); ++i)
    EEPROM.update(base + i, bytes[i]);
  EEPROM.update(base + (int)offsetof(CalibrationRecord, commit),
                EEPROM_CAL_COMMIT);

  // Read back into the same record; no second 50-byte stack object.
  EEPROM.get(base, record);
  if (!calibrationRecordValid(record)) return CAL_SAVE_IO_ERROR;
  applyCalibrationRecord(record, targetSlot);
  return CAL_SAVE_OK;
}

static FW_NOINLINE void initialiseDefaultCalibrationScales() {
  voltageCentiPerCountQ8 =
      nominalVoltageScaleQ8(VOLTAGE_DIVIDER_NUM, VOLTAGE_DIVIDER_DEN);
  sourceCentiPerCountQ8 =
      nominalVoltageScaleQ8(SOURCE_DIVIDER_NUM, SOURCE_DIVIDER_DEN);
  // ACS712 nominal gains are provisional until the 10-ohm/DMM calibration.
  currentGainCentiPerCountQ8 = nominalAcsGainCentiPerCountQ8(
      (uint16_t)ACS712_PRIMARY_SENSITIVITY_MV_PER_A);
  sourceCurrentGainCentiPerCountQ8 = nominalAcsGainCentiPerCountQ8(
      (uint16_t)ACS712_SOURCE_SENSITIVITY_MV_PER_A);
}
// ADC table for a 10k NTC, Beta 3950, 10k pull-up to AVCC.
// Table points are -20 C through +120 C in 10 C increments. Readings colder
// than the validated table range are treated as a sensor fault (fail closed).
static const uint16_t THERMISTOR_RAW_TABLE[] PROGMEM = {
  934U, 873U, 788U, 684U, 569U, 456U, 354U, 270U,
  204U, 153U, 115U, 87U, 67U, 51U, 40U
};

static FW_NOINLINE uint16_t thermistorTableRaw(uint8_t index) {
#if defined(__AVR__)
  return pgm_read_word(&THERMISTOR_RAW_TABLE[index]);
#else
  return THERMISTOR_RAW_TABLE[index];
#endif
}

static FW_NOINLINE bool thermistorRawToCentiC(uint16_t raw, int32_t &temperatureCenti) {
  if (raw <= THERMISTOR_SHORT_RAW || raw >= THERMISTOR_OPEN_RAW)
    return false;
  const uint16_t coldest = thermistorTableRaw(0U);
  const uint16_t hottest = thermistorTableRaw(14U);
  if (raw > coldest) {
    // Above the coldest validated table point is out-of-range. Treat it as
    // sensor fault (including an open/high-resistance thermistor), never as
    // a safe cold temperature.
    return false;
  }
  if (raw == coldest) {
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
      const int32_t fraction =
          (((int32_t)r0 - (int32_t)raw) * 1000) /
          ((int32_t)r0 - (int32_t)r1);
      temperatureCenti = -2000 + (int32_t)i * 1000 + fraction;
      return true;
    }
  }
  return false;
}

// ACS712 zero calibration. This only runs with PWM disabled and current isolated.
static FW_NOINLINE bool calibrateAcs712Zero(uint8_t analogPin, int32_t &zeroRawQ8,
                                uint8_t &spanRaw) {
  for (uint8_t settle = 0U; settle < 4U; ++settle)
    (void)analogRead(analogPin);

  int32_t baseRaw = -1L;
  int32_t offsetSum = 0L;
  uint32_t offsetSquares = 0UL;
  uint16_t minimum = 1023U;
  uint16_t maximum = 0U;
  for (uint16_t sample = 0U; sample < ACS712_ZERO_CAL_SAMPLES; ++sample) {
    const uint16_t raw = (uint16_t)analogRead(analogPin);
    if (raw < minimum) minimum = raw;
    if (raw > maximum) maximum = raw;
    if ((uint16_t)(maximum - minimum) > ACS712_ZERO_MAX_SPAN_RAW)
      return false;
    if (baseRaw < 0L) baseRaw = (int32_t)raw;
    const int32_t offset = (int32_t)raw - baseRaw;
    offsetSum += offset;
    offsetSquares += (uint32_t)(offset * offset);
  }

  if (baseRaw < (int32_t)ACS712_ZERO_MIN_RAW ||
      baseRaw > (int32_t)ACS712_ZERO_MAX_RAW) return false;
  const uint32_t n = (uint32_t)ACS712_ZERO_CAL_SAMPLES;
  const int32_t varianceNumerator =
      (int32_t)(offsetSquares * n) - offsetSum * offsetSum;
  const uint32_t sigma = (uint32_t)ACS712_ZERO_MAX_STDDEV_SQUARED;
  const uint32_t varianceLimit = sigma * n * n;
  if (varianceNumerator < 0L ||
      (uint32_t)varianceNumerator > varianceLimit) return false;
  const int32_t meanOffsetQ8 =
      (offsetSum * 256) / (int32_t)n;
  zeroRawQ8 = (int32_t)(baseRaw * 256 + meanOffsetQ8);
  spanRaw = (uint8_t)(maximum - minimum);
  return zeroRawQ8 >= (int32_t)ACS712_ZERO_MIN_RAW * 256L &&
         zeroRawQ8 <= (int32_t)ACS712_ZERO_MAX_RAW * 256L;
}

static FW_NOINLINE bool zeroOffsetPlausibleOnPin(uint8_t pin, int32_t expectedZeroRawQ8) {
  (void)analogRead(pin); // discard after ADC channel change
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
  const int32_t meanQ8 = (int32_t)((sum * 256UL + 16UL) / 32UL);
  return absI32(meanQ8 - expectedZeroRawQ8) <= (6L * 256L) &&
         (uint16_t)(maximum - minimum) <= ACS712_ZERO_MAX_SPAN_RAW;
}

static FW_NOINLINE bool storedZeroOffsetPlausible() {
  if (!zeroOffsetPlausibleOnPin(CURRENT_PIN, primaryCurrentZeroRawQ8))
    return false;
#if SOURCE_CURRENT_SENSE_ENABLED
  if (!zeroOffsetPlausibleOnPin(SOURCE_CURRENT_PIN,
                                sourceCurrentZeroRawQ8)) return false;
#endif
  return true;
}

static FW_NOINLINE void resetAdaptiveModel(AdaptiveModel &model);

static FW_NOINLINE int16_t sourceOutputIncrementQ14(int32_t dutyLevelBp,
                                        int32_t deltaSourceCenti,
                                        int32_t deltaOutputCenti) {
#if SOURCE_SENSE_ENABLED
  // duty is 0..9000 bp and a calibrated 10-bit source divider bounds
  // deltaSource well below 3000 cV: the product stays below 27M.
  const int32_t sourceTermCenti =
      (dutyLevelBp * deltaSourceCenti) / 10000;
#else
  (void)dutyLevelBp;
  (void)deltaSourceCenti;
  const int32_t sourceTermCenti = 0L;
#endif
  return toQ14(sourceTermCenti - deltaOutputCenti,
               MODEL_SOURCE_OUTPUT_SCALE_CENTI);
}



// -------------------------- PWM backend ----------------------------------
static FW_NOINLINE void configurePwm() {
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

static FW_NOINLINE void writeDutyBasisPoints(int32_t primaryDutyBp,
                                 int32_t auxiliaryDutyBp) {
  const int32_t primaryBounded = clampI32(primaryDutyBp,
      MIN_DUTY_BASIS_POINTS, MAX_DUTY_BASIS_POINTS);
  const int32_t auxiliaryBounded = quantizeAuxiliaryDutyBp(auxiliaryDutyBp);
#if defined(__AVR_ATmega328P__)
  // Quantize the software command to the actual timer compare count, then
  // feed effective duty back to the identifier (never an unachievable command).
  // Timer1 has 800 counts per period.
  const uint16_t primaryCompare = (uint16_t)(
      ((int32_t)primaryBounded * 800L + 5000L) / 10000L);
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
  const uint8_t primaryPwm = (uint8_t)(((int32_t)primaryBounded * 255L + 5000L) / 10000L);
  const uint8_t auxiliaryPwm = (uint8_t)(((int32_t)auxiliaryBounded * 255L + 5000L) / 10000L);
  appliedDutyBp = (int32_t)(((uint16_t)primaryPwm * 10000U + 127U) / 255U);
  appliedAuxiliaryDutyBp = (int32_t)(((uint16_t)auxiliaryPwm * 10000U + 127U) / 255U);
  analogWrite(PWM_PIN, primaryPwm);
  analogWrite(AUX_PWM_PIN, auxiliaryPwm);
#endif
}

static FW_NOINLINE void latchFault(FaultCode reason) {
  if (faultCode == FAULT_NONE) faultCode = reason;
  writeDutyBasisPoints(0L, 0L);
  voltageIntegratorQ16 = 0L;
  currentIntegratorQ16 = 0L;
}

static FW_NOINLINE bool updateThermalLimit(uint16_t thermistorRaw) {
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
    runtimeCurrentLimitCenti = (outputCurrentSettingCenti * remaining) / span;
  }
  return true;
}

static FW_NOINLINE void printCalibrationInstructions() {
  Serial.println(F("OFF; ISOLATE SRC+LOAD"));
  Serial.println(F("ZERO, THEN 10R/50W LOAD"));
  Serial.println(F("DMM V= VIN= IIN= (cV/cA)"));
}

static FW_NOINLINE void resetCalibrationAverages(bool invalidateSnapshot = true) {
  calibrationWindow.samples = 0U;
  for (uint8_t i = 0U; i < 4U; ++i) {
    calibrationWindow.sum[i] = 0UL;
    calibrationWindow.sumSquares[i] = 0UL;
    calibrationWindow.minimum[i] = 1023U;
    calibrationWindow.maximum[i] = 0U;
  }
  if (invalidateSnapshot) calibrationSnapshot.valid = false;
}

template <typename Message>
static FW_NOINLINE void calibrationPrint(Message message);

static FW_NOINLINE bool calibrationWindowWithinNoise(const CalibrationWindowAccumulator &w) {
  if (w.samples != CAL_SAMPLE_COUNT) return false;
  // The compile-time bounds prove that the configured ADC range and 64-sample
  // window fit the 16-bit sum and 32-bit variance numerator. Promote before
  // squaring because AVR int is 16-bit.
  const uint32_t n = (uint32_t)CAL_SAMPLE_COUNT;
  for (uint8_t i = 0U; i < 4U; ++i) {
    const uint16_t span = (uint16_t)(w.maximum[i] - w.minimum[i]);
    const uint16_t spanLimit = (i == 0U) ? CAL_OUTPUT_VOLTAGE_MAX_SPAN_RAW
        : (i == 1U) ? CAL_OUTPUT_CURRENT_MAX_SPAN_RAW
        : (i == 2U) ? CAL_SOURCE_VOLTAGE_MAX_SPAN_RAW
                    : CAL_SOURCE_CURRENT_MAX_SPAN_RAW;
    const uint16_t sigmaLimit = (i == 0U) ? CAL_OUTPUT_VOLTAGE_MAX_SIGMA_RAW
        : (i == 1U) ? CAL_OUTPUT_CURRENT_MAX_SIGMA_RAW
        : (i == 2U) ? CAL_SOURCE_VOLTAGE_MAX_SIGMA_RAW
                    : CAL_SOURCE_CURRENT_MAX_SIGMA_RAW;
    if (span > spanLimit) return false;
    const uint32_t sum = (uint32_t)w.sum[i];
    const uint32_t varianceNumerator = w.sumSquares[i] * n - sum * sum;
    const uint32_t sigma = (uint32_t)sigmaLimit;
    const uint32_t varianceLimit = sigma * sigma * n * n;
    if (varianceNumerator > varianceLimit) return false;
  }
  return true;
}

static FW_NOINLINE void accumulateCalibrationSample(uint16_t voutRaw, uint16_t ioutRaw,
                                        uint16_t vinRaw, uint16_t iinRaw) {
  const uint16_t sample[4] = { voutRaw, ioutRaw, vinRaw, iinRaw };
  // Invalidate a previously completed window immediately when the operating
  // point moves materially; a user must never calibrate against stale means.
  if (calibrationSnapshot.valid) {
    for (uint8_t i = 0U; i < 4U; ++i) {
      const uint8_t driftLimit = (i < 2U) ? 8U : 18U;
      const int32_t difference = (int32_t)sample[i] -
                                 (int32_t)calibrationSnapshot.meanRaw[i];
      if (absI32(difference) > driftLimit) {
        calibrationSnapshot.valid = false;
        calibrationWindowReportSent = false;
        break;
      }
    }
  }
  for (uint8_t i = 0U; i < 4U; ++i) {
    calibrationWindow.sum[i] += sample[i];
    calibrationWindow.sumSquares[i] += (uint32_t)sample[i] * sample[i];
    if (sample[i] < calibrationWindow.minimum[i])
      calibrationWindow.minimum[i] = sample[i];
    if (sample[i] > calibrationWindow.maximum[i])
      calibrationWindow.maximum[i] = sample[i];
  }
  if (calibrationWindow.samples < CAL_SAMPLE_COUNT)
    ++calibrationWindow.samples;

  if (calibrationWindow.samples == CAL_SAMPLE_COUNT) {
    if (calibrationWindowWithinNoise(calibrationWindow)) {
      for (uint8_t i = 0U; i < 4U; ++i) {
        calibrationSnapshot.meanRaw[i] = (uint16_t)(
            ((uint32_t)calibrationWindow.sum[i] + CAL_SAMPLE_COUNT / 2U) /
            CAL_SAMPLE_COUNT);
        calibrationSnapshot.spanRaw[i] = (uint8_t)(
            calibrationWindow.maximum[i] - calibrationWindow.minimum[i]);
      }
      calibrationSnapshot.capturedAtMs = (uint32_t)millis();
      calibrationSnapshot.valid = true;
      if (!calibrationWindowReportSent) {
        calibrationPrint(F("ADC OK; SEND DMM"));
        calibrationWindowReportSent = true;
      }
    } else {
      calibrationSnapshot.valid = false;
      calibrationWindowReportSent = false;
    }
    // Immediately begin a new window. The snapshot always contains the most
    // recent completed stable batch and cannot go stale while the user reads meters.
    resetCalibrationAverages(false);
  }
}

// DMM fields are unsigned centi-units and never exceed 6000.
// A 16-bit accumulator is sufficient; reject overflow before multiply/add.
static FW_NOINLINE bool parseBoundedDecimal(const char *text, uint16_t minimum,
                                uint16_t maximum, int32_t &value) {
  if (text == 0 || *text == '\0' || maximum > 6000U || minimum > maximum)
    return false;
  uint16_t parsed = 0U;
  while (*text != '\0') {
    const char ch = *text++;
    if (ch < '0' || ch > '9') return false;
    const uint8_t digit = (uint8_t)(ch - '0');
    if (parsed > 600U || (parsed == 600U && digit != 0U)) return false;
    parsed = (uint16_t)(parsed * 10U + digit);
  }
  if (parsed < minimum || parsed > maximum) return false;
  value = (int32_t)parsed;
  return true;
}

static FW_NOINLINE bool equalsLiteral(const char *text, const char *literal) {
  if (text == 0 || literal == 0) return false;
  while (*text != '\0' && *literal != '\0') {
    if (*text++ != *literal++) return false;
  }
  return *text == '\0' && *literal == '\0';
}

static FW_NOINLINE bool parseCalibrationDmmCommand(char *command,
                                       int32_t &voutCenti,
                                       int32_t &vinCenti,
                                       int32_t &iinCenti) {
  bool haveVout = false, haveVin = false, haveIin = false;
  char *cursor = command;
  while (*cursor != '\0') {
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == '\0') break;
    char *token = cursor;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') ++cursor;
    if (*cursor != '\0') *cursor++ = '\0';

    if (token[0] == 'V' && token[1] == '=') {
      if (haveVout || !parseBoundedDecimal(token + 2,
          (uint16_t)(CAL_TARGET_V_CENTI - CAL_DMM_VOLTAGE_TOLERANCE_CENTI),
          (uint16_t)(CAL_TARGET_V_CENTI + CAL_DMM_VOLTAGE_TOLERANCE_CENTI),
          voutCenti)) return false;
      haveVout = true;
    } else if (token[0] == 'V' && token[1] == 'I' &&
               token[2] == 'N' && token[3] == '=') {
      if (haveVin || !parseBoundedDecimal(token + 4, 100U,
          (uint16_t)EFFICIENCY_MAX_SOURCE_VOLTAGE_CENTI, vinCenti)) return false;
      haveVin = true;
    } else if (token[0] == 'I' && token[1] == 'I' &&
               token[2] == 'N' && token[3] == '=') {
      if (haveIin || !parseBoundedDecimal(token + 4, 20U,
          (uint16_t)INPUT_HARD_OVERCURRENT_CENTI, iinCenti)) return false;
      haveIin = true;
    } else {
      return false;
    }
  }
  return haveVout && haveVin && haveIin;
}

static FW_NOINLINE void resetRunNoResponseMonitor();

static FW_NOINLINE void resetAdaptiveRuntimeAfterCalibration() {
  voltageIntegratorQ16 = 0L;
  currentIntegratorQ16 = 0L;
  resetRunNoResponseMonitor();
  voltageReferenceQ16 = 0L;
  voltageFilteredQ8 = 0L;
  currentFilteredQ8 = 0L;
  currentMeanQ16 = 0L;
  currentMeanCenti = 0L;
#if SOURCE_SENSE_ENABLED
  sourceFilteredQ8 = 0L;
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
  sourceCurrentFilteredQ8 = 0L;
#endif
  filtersInitialized = false;
  thermistorRawFilteredQ8 = 0L;
  thermistorTemperatureCenti = 2500L;
  thermistorInitialized = false;
  runtimeCurrentLimitCenti = outputCurrentSettingCenti;
  runtimeInputCurrentLimitCenti = inputCurrentSettingCenti;
  inputCurrentGuardActive = false;
  auxiliaryPathMode = AUX_MODE_PRIMARY_ONLY;
  auxiliaryLowPowerActive = false;
  auxiliaryIdentificationBaseReady = false;
  identificationProbeActive = false;
  previousAuxiliaryProbeActive = false;
  probeDivider = 0U;
  probeLfsr = 0x5DU;
  probeSign = 1;
  modelUpdatePhase = 0U;
  dutyMeanQ16 = 0L;
  dutyMeanBp = 0L;
  previousDutyCenteredBp = 0L;
  auxiliaryDutyMeanQ16 = 0L;
  auxiliaryDutyMeanBp = 0L;
  previousAuxiliaryDutyCenteredBp = 0L;
  previousDutyLevelBp = 0L;
  commandedCurrentReferenceCenti = 0L;
  primaryEquivalentDutyDemandBp = 0L;
  previousMeasurementsValid = false;
  previousDeltaVoltageCenti = 0L;
  previousDeltaCurrentCenti = 0L;
  previousVoltageCenti = 0L;
  previousCurrentCenti = 0L;
  previousCurrentCenteredCenti = 0L;
  previousSourceCenti = 0L;
  previousSourceCurrentCenti = 0L;
  estimatedLoadCurrentCenti = 0L;
  estimatedLoadCurrentQ8 = 0L;
  loadEstimateInitialized = false;
  loadEstimateValid = false;
  trendWindowStartVoltage = 0L;
  voltageSlopeCentiPerSec = 0L;
  trendWindowTicks = 0U;
  crossingWindowTicks = 0U;
  crossingCount = 0U;
  transientHoldCounter = 0U;
  nonlinearResidualCounter = 0U;
  previousErrorSign = 0;
  latestOutputCurrentCenti = 0L;
  latestInputCurrentCenti = 0L;
#if MPPT_ENABLED
  efficiencyMeasurementDivider = 0U;
#endif
#if AUX_EFFICIENCY_LEARNING_ENABLED
  memset(efficiencyProfiles, 0, sizeof(efficiencyProfiles));
  efficiencyBlockLossSum10mW = 0UL;
  efficiencyBlockSamples = 0U;
  efficiencyBlockMode = 255U;
  efficiencyBlockCurrentBin = 255U;
  efficiencyBlockSourceBin = 255U;
  efficiencyPreviousMainDutyBp = -1L;
  efficiencyPreviousAuxDutyBp = -1L;
#endif
  resetAdaptiveModel(currentModel);
  resetAdaptiveModel(voltageModel);
#if MPPT_ENABLED
  mpptInputCurrentTargetCenti = 0U;
  mpptOutputCurrentLimitCenti = (uint16_t)outputCurrentSettingCenti;
  mpptEfficiencyEstimatePermille = 0U;
  mpptPreviousPowerProduct = 0UL;
  mpptSettleTicks = 0U;
  mpptDirection = 1;
#endif
  writeDutyBasisPoints(0L, 0L);
  lastControlUs = (uint32_t)micros();
}

static FW_NOINLINE bool calibrationLoadSettingAllowsRun() {
  const int32_t outputGuard =
      outputCurrentSettingCenti * (int32_t)CAL_SOFT_LIMIT_PERCENT / 100L;
  return outputGuard > CAL_EXPECTED_CURRENT_CENTI + 10L &&
         runtimeInputCurrentLimitCenti > 0L;
}

static FW_NOINLINE void resetCalibrationDutyCommand() {
  calibrationDutyCommandQ16 = 0L;
  calibrationDutyBp = 0L;
}

static FW_NOINLINE int32_t advanceCalibrationDutyCommand(int32_t voltageErrorCenti,
                                             uint32_t dtUs) {
  // Preserve the original sign-integrating calibration regulator, but keep
  // its sub-PWM-count increment in Q16. Rounding each 3 bp update to the
  // 12.5 bp Timer1 quantum before accumulating would otherwise stick at 0.
  const uint32_t slewRateBpPerSec = (voltageErrorCenti >= 0L)
      ? (uint32_t)CAL_DUTY_SLEW_BP_PER_SEC
      : (uint32_t)CAL_DUTY_FALL_BP_PER_SEC;
  // Cap a delayed foreground iteration so a serial stall can never turn into
  // a large calibration duty jump. The remainder conversion is exact in Q16:
  // 65536 / 1,000,000 == 4096 / 62,500.
  const uint32_t boundedDtUs = (dtUs > 20000UL) ? 20000UL : dtUs;
  const uint32_t rateTime = slewRateBpPerSec * boundedDtUs;
  uint32_t maximumStepQ16 = (uint32_t)(
      (rateTime / 1000000UL) * 65536UL +
      ((rateTime % 1000000UL) * 4096UL) / 62500UL);
  if (maximumStepQ16 == 0UL) maximumStepQ16 = 1UL;

  int32_t requestedStepQ16 = 0L;
  if (absI32(voltageErrorCenti) > 3L) {
    requestedStepQ16 = voltageErrorCenti * 2L * 65536L;
    if (requestedStepQ16 > (int32_t)maximumStepQ16)
      requestedStepQ16 = (int32_t)maximumStepQ16;
    else if (requestedStepQ16 < -(int32_t)maximumStepQ16)
      requestedStepQ16 = -(int32_t)maximumStepQ16;
  }

  const int32_t maximumDutyQ16 = MAX_DUTY_BASIS_POINTS * 65536L;
  calibrationDutyCommandQ16 = clampI32(
      calibrationDutyCommandQ16 + requestedStepQ16,
      0L, maximumDutyQ16);

  const int32_t requestedBp = (int32_t)(
      (calibrationDutyCommandQ16 + 32768L) / 65536L);
  calibrationDutyBp = quantizePrimaryDutyBp(requestedBp);
  return calibrationDutyBp;
}

static FW_NOINLINE void resetRunNoResponseMonitor() {
  runNoResponseStartMs = 0UL;
  runNoResponseTiming = false;
}

// Detect a failed/low output-voltage feedback path during normal operation.
// This is deliberately time-qualified and requires source headroom: a source
// collapse or disconnected source must not be misreported as a feedback fault.
static FW_NOINLINE bool updateRunNoResponseMonitor(uint32_t nowMs,
                                       int32_t voltageCenti,
                                       int32_t voltageReferenceCenti,
                                       int32_t sourceVoltageCenti,
                                       int32_t outputCurrentCenti,
                                       int32_t outputCurrentLimitCenti,
                                       int32_t sourceCurrentCenti,
                                       int32_t sourceCurrentLimitCenti,
                                       int32_t primaryDutyBp,
                                       int32_t auxiliaryDutyBp) {
  const int32_t activeDuty = (primaryDutyBp > auxiliaryDutyBp)
      ? primaryDutyBp : auxiliaryDutyBp;
  const bool sourceHasHeadroom =
      (sourceVoltageCenti * 100L) >=
      (voltageReferenceCenti * RUN_NO_RESPONSE_HEADROOM_PERCENT);
  const bool outputHasCurrentMargin = outputCurrentLimitCenti > 0L &&
      (outputCurrentCenti * 100L) <
      (outputCurrentLimitCenti * RUN_NO_RESPONSE_CURRENT_MARGIN_PERCENT);
  const bool sourceHasCurrentMargin = sourceCurrentLimitCenti > 0L &&
      (sourceCurrentCenti * 100L) <
      (sourceCurrentLimitCenti * RUN_NO_RESPONSE_CURRENT_MARGIN_PERCENT);
  const bool noResponse =
      voltageReferenceCenti >= RUN_NO_RESPONSE_MIN_REF_CENTI &&
      voltageCenti < RUN_NO_RESPONSE_MIN_V_CENTI &&
      activeDuty >= RUN_NO_RESPONSE_DUTY_BP && sourceHasHeadroom &&
      outputHasCurrentMargin && sourceHasCurrentMargin;

  if (!noResponse) {
    resetRunNoResponseMonitor();
    return false;
  }
  if (!runNoResponseTiming) {
    runNoResponseStartMs = nowMs;
    runNoResponseTiming = true;
    return false;
  }
  return (uint32_t)(nowMs - runNoResponseStartMs) >=
         RUN_NO_RESPONSE_TIMEOUT_MS;
}

// HardwareSerial.availableForWrite() lets calibration diagnostics remain
// non-blocking while the power stage is energised. If the TX ring lacks room,
// the message is dropped; STATUS can be requested again.
template <typename Message>
static FW_NOINLINE void calibrationPrint(Message message) {
  const bool powerMayBeActive = currentSensorsCalibrated ||
      calibrationWizardState == CAL_WIZARD_RAMP ||
      calibrationWizardState == CAL_WIZARD_WAIT_DMM;
  if (powerMayBeActive && Serial.availableForWrite() < 32) return;
  Serial.println(message);
}

static FW_NOINLINE bool calibrationCurrentWithinUserLimits(int32_t outputCurrentCenti,
                                               int32_t inputCurrentCenti) {
  if (runtimeCurrentLimitCenti <= 0L || runtimeInputCurrentLimitCenti <= 0L)
    return false;
  const int32_t outputGuard = (int32_t)(
      (runtimeCurrentLimitCenti * CAL_SOFT_LIMIT_PERCENT) / 100L);
  const int32_t inputGuard = (int32_t)(
      (runtimeInputCurrentLimitCenti * CAL_SOFT_LIMIT_PERCENT) / 100L);
  return outputCurrentCenti < outputGuard && inputCurrentCenti < inputGuard;
}

static FW_NOINLINE void handleCalibrationCommand(char *command) {
  if (equalsLiteral(command, "STATUS")) {
      // Avoid blocking while PWM may be active.
    if ((currentSensorsCalibrated ||
         calibrationWizardState == CAL_WIZARD_RAMP ||
         calibrationWizardState == CAL_WIZARD_WAIT_DMM) &&
        Serial.availableForWrite() < 60) return;
    Serial.print(F("S="));
    Serial.print((int)calibrationWizardState);
    Serial.print(F(" C="));
    Serial.print(currentSensorsCalibrated ? 1 : 0);
    Serial.print(F(" W="));
    Serial.print(calibrationSnapshot.valid ? 1 : 0);
    Serial.print(F(" T="));
    Serial.print(thermistorTemperatureCenti);
    Serial.print(F(" O="));
    Serial.print(latestOutputCurrentCenti);
    Serial.print(F(" I="));
    Serial.print(latestInputCurrentCenti);
    Serial.print(F(" OL="));
    Serial.print(runtimeCurrentLimitCenti);
    Serial.print(F(" IL="));
    Serial.print(runtimeInputCurrentLimitCenti);
    Serial.println();
    return;
  }

  if (equalsLiteral(command, "RECAL") &&
      calibrationWizardState == CAL_WIZARD_READY && faultCode == FAULT_NONE) {
    writeDutyBasisPoints(0L, 0L);
    resetCalibrationDutyCommand();
    currentSensorsCalibrated = false;
    calibrationWizardState = CAL_WIZARD_WAIT_ZERO;
    calibrationStateStartMs = (uint32_t)millis();
    calibrationWindowReportSent = false;
    resetCalibrationAverages();
    printCalibrationInstructions();
    return;
  }

  if (equalsLiteral(command, "ABORT") &&
      (calibrationWizardState == CAL_WIZARD_RAMP ||
       calibrationWizardState == CAL_WIZARD_WAIT_DMM)) {
    writeDutyBasisPoints(0L, 0L);
    resetCalibrationDutyCommand();
    calibrationStateStartMs = (uint32_t)millis();
    resetCalibrationAverages();
    calibrationWizardState = CAL_WIZARD_WAIT_LOAD;
    calibrationPrint(F("ABORT; PWM=0; LOAD"));
    return;
  }

  if (calibrationWizardState == CAL_WIZARD_WAIT_ZERO &&
      equalsLiteral(command, "ZERO")) {
    writeDutyBasisPoints(0L, 0L);
    calibrationOutputZeroSpanRaw = 0U;
    calibrationInputZeroSpanRaw = 0U;
    calibrationStateStartMs = (uint32_t)millis();
    calibrationWizardState = CAL_WIZARD_ZERO_SETTLE;
    calibrationPrint(F("ZERO; WAIT 250ms"));
    return;
  }

  if (calibrationWizardState == CAL_WIZARD_WAIT_LOAD &&
      equalsLiteral(command, "LOAD")) {
    if (!calibrationLoadSettingAllowsRun()) {
      calibrationPrint(F("RAISE OUTPUT LIMIT"));
      return;
    }
    resetCalibrationDutyCommand();
    calibrationStableStartMs = 0UL;
    calibrationLastVoltageCenti = -1L;
    calibrationNoResponseStartMs = 0UL;
    calibrationWindowReportSent = false;
    resetCalibrationAverages();
    calibrationStateStartMs = (uint32_t)millis();
    calibrationWizardState = CAL_WIZARD_RAMP;
    calibrationPrint(F("LOAD RAMP"));
    lastControlUs = (uint32_t)micros();
    return;
  }

  if (calibrationWizardState == CAL_WIZARD_WAIT_DMM) {
    int32_t measuredVoltageCenti = 0L;
    int32_t measuredSourceCenti = 0L;
    int32_t measuredInputCurrentCenti = 0L;
    if (!parseCalibrationDmmCommand(command, measuredVoltageCenti,
                                    measuredSourceCenti,
                                    measuredInputCurrentCenti)) {
      calibrationPrint(F("DMM V= VIN= IIN="));
      return;
    }
    if (!calibrationSnapshot.valid ||
        (uint32_t)((uint32_t)millis() - calibrationSnapshot.capturedAtMs) >
            CAL_SNAPSHOT_MAX_AGE_MS) {
      calibrationPrint(F("ADC WINDOW STALE"));
      return;
    }

    const uint16_t meanVoltageRaw = calibrationSnapshot.meanRaw[0];
    const uint16_t meanCurrentRaw = calibrationSnapshot.meanRaw[1];
    const uint16_t meanSourceRaw = calibrationSnapshot.meanRaw[2];
    const uint16_t meanInputCurrentRaw = calibrationSnapshot.meanRaw[3];
    if (meanVoltageRaw == 0U || meanSourceRaw == 0U) {
      calibrationPrint(F("ADC RANGE"));
      return;
    }

    const int32_t deltaOutputCurrentRawQ8 =
        (int32_t)meanCurrentRaw * 256L - primaryCurrentZeroRawQ8;
    const int32_t deltaInputCurrentRawQ8 =
        (int32_t)meanInputCurrentRaw * 256L - sourceCurrentZeroRawQ8;
    if (absI32(deltaOutputCurrentRawQ8) < 2048L ||
        absI32(deltaInputCurrentRawQ8) < 2048L) {
      calibrationPrint(F("ACS SIGNAL LOW"));
      return;
    }

    // Values are constructed once, then the EEPROM validator performs every
    // range, orientation, sensor-crosscheck, power and divider check.
    const int32_t expectedOutputCurrentCenti = (int32_t)(
        ((int32_t)measuredVoltageCenti * 1000L) /
        (int32_t)CAL_LOAD_RESISTANCE_MILLIOHM);
    const uint32_t newVoltageScale = (uint32_t)(
        ((uint32_t)measuredVoltageCenti * 256U + meanVoltageRaw / 2U) /
        meanVoltageRaw);
    const uint32_t newSourceScale = (uint32_t)(
        ((uint32_t)measuredSourceCenti * 256U + meanSourceRaw / 2U) /
        meanSourceRaw);
    const int32_t newOutputCurrentGain = (int32_t)(
        (expectedOutputCurrentCenti * 65536L) / deltaOutputCurrentRawQ8);
    const int32_t newInputCurrentGain = (int32_t)(
        (measuredInputCurrentCenti * 65536L) / deltaInputCurrentRawQ8);

    if (newVoltageScale == 0UL || newVoltageScale > 65535UL ||
        newSourceScale == 0UL || newSourceScale > 65535UL ||
        newOutputCurrentGain < INT16_MIN || newOutputCurrentGain > INT16_MAX ||
        newInputCurrentGain < INT16_MIN || newInputCurrentGain > INT16_MAX) {
      calibrationPrint(F("CAL DATA RANGE"));
      return;
    }

    CalibrationRecord candidate = {};
    candidate.currentZeroRawQ8 = primaryCurrentZeroRawQ8;
    candidate.currentGainCentiPerCountQ8 = (int16_t)newOutputCurrentGain;
    candidate.sourceCurrentZeroRawQ8 = sourceCurrentZeroRawQ8;
    candidate.sourceCurrentGainCentiPerCountQ8 = (int16_t)newInputCurrentGain;
    candidate.voltageCentiPerCountQ8 = (uint16_t)newVoltageScale;
    candidate.sourceCentiPerCountQ8 = (uint16_t)newSourceScale;
    candidate.calibrationOutputVoltageCenti = (uint16_t)measuredVoltageCenti;
    candidate.calibrationSourceVoltageCenti = (uint16_t)measuredSourceCenti;
    candidate.calibrationInputCurrentCenti = (uint16_t)measuredInputCurrentCenti;
    for (uint8_t i = 0U; i < 4U; ++i)
      candidate.calibrationRawMean[i] = calibrationSnapshot.meanRaw[i];
    candidate.outputZeroSpanRaw = calibrationOutputZeroSpanRaw;
    candidate.inputZeroSpanRaw = calibrationInputZeroSpanRaw;
    candidate.calibrationQuality = 0x07U;

    writeDutyBasisPoints(0L, 0L);
    resetCalibrationDutyCommand();
    const CalibrationSaveResult result = saveCalibrationToEeprom(candidate);
    if (result == CAL_SAVE_INVALID) {
      calibrationPrint(F("CAL DATA REJECT"));
      return;
    }
    if (result != CAL_SAVE_OK) {
      latchFault(FAULT_EEPROM);
      calibrationWizardState = CAL_WIZARD_FAILED;
      calibrationPrint(F("EEPROM VERIFY"));
      return;
    }

    currentSensorsCalibrated = true;
    calibrationWizardState = CAL_WIZARD_READY;
    resetAdaptiveRuntimeAfterCalibration();
    calibrationPrint(F("CAL SAVED"));
    return;
  }

  calibrationPrint(F("BAD CAL CMD"));
}

static FW_NOINLINE void pollCalibrationSerial() {
  // Bound serial work so command input cannot starve the control deadline.
  uint8_t budget = 12U;
  while (budget-- > 0U && Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0) break;
    const char ch = (char)incoming;
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (calibrationCommandOverflow) {
        calibrationPrint(F("CMD TOO LONG"));
      } else if (calibrationCommandLength > 0U) {
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

static FW_NOINLINE void calibrationWizardLoop() {
  if (calibrationWizardState == CAL_WIZARD_ZERO_SETTLE) {
    writeDutyBasisPoints(0L, 0L);
    if ((uint32_t)(millis() - calibrationStateStartMs) < ACS712_ZERO_SETTLE_MS)
      return;
    if (!calibrateAcs712Zero(CURRENT_PIN, primaryCurrentZeroRawQ8,
                             calibrationOutputZeroSpanRaw)) {
      calibrationWizardState = CAL_WIZARD_WAIT_ZERO;
      calibrationPrint(F("ZERO IOUT NOISE"));
      return;
    }
#if SOURCE_CURRENT_SENSE_ENABLED
    if (!calibrateAcs712Zero(SOURCE_CURRENT_PIN, sourceCurrentZeroRawQ8,
                             calibrationInputZeroSpanRaw)) {
      calibrationWizardState = CAL_WIZARD_WAIT_ZERO;
      calibrationPrint(F("ZERO IIN NOISE"));
      return;
    }
#endif
    initialiseDefaultCalibrationScales();
    calibrationWizardState = CAL_WIZARD_WAIT_LOAD;
    calibrationStateStartMs = (uint32_t)millis();
    calibrationPrint(F("ZERO OK; CONNECT 10R"));
    return;
  }

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
    resetCalibrationDutyCommand();
    resetCalibrationAverages();
    calibrationWizardState = CAL_WIZARD_WAIT_LOAD;
    calibrationPrint(F("HOT; COOL; RETRY"));
    return;
  }

  const int32_t voltageCenti = adcToVoltageCenti(voltageRaw);
  const int32_t currentSigned = adcToAcs712CurrentCenti(
      currentRaw, primaryCurrentZeroRawQ8,
      (uint16_t)ACS712_PRIMARY_SENSITIVITY_MV_PER_A, 1);
  // Polarity is learned from the known 10-ohm forward-load test and stored in
  // the sign of currentGainCentiPerCountQ8. Use magnitude for calibration
  // supervision so a reversed ACS712 installation can be corrected in EEPROM.
  const int32_t currentCenti = absI32(currentSigned);
  if (calibrationDutyBp >= CAL_NO_RESPONSE_DUTY_BP &&
      voltageCenti < CAL_NO_RESPONSE_MIN_V_CENTI) {
    if (calibrationNoResponseStartMs == 0UL)
      calibrationNoResponseStartMs = (uint32_t)millis();
    else if ((uint32_t)(millis() - calibrationNoResponseStartMs) >=
             CAL_NO_RESPONSE_TIMEOUT_MS) {
      latchFault(FAULT_SENSOR_CALIBRATION);
      calibrationWizardState = CAL_WIZARD_FAILED;
      calibrationPrint(F("NO VOUT RESPONSE"));
      return;
    }
  } else {
    calibrationNoResponseStartMs = 0UL;
  }
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
      1);
  const int32_t inputCurrentCenti = absI32(inputCurrentSigned);
  if (inputCurrentCenti >= INPUT_HARD_OVERCURRENT_CENTI) {
    latchFault(FAULT_INPUT_OVERCURRENT);
    calibrationWizardState = CAL_WIZARD_FAILED;
    return;
  }
#else
  const int32_t inputCurrentCenti = 0L;
#endif
  if (!calibrationCurrentWithinUserLimits(currentCenti,
                                           inputCurrentCenti)) {
    writeDutyBasisPoints(0L, 0L);
    resetCalibrationDutyCommand();
    resetCalibrationAverages();
    calibrationWizardState = CAL_WIZARD_WAIT_LOAD;
    calibrationPrint(F("CURRENT LIMIT; RETRY"));
    return;
  }

#if SOURCE_SENSE_ENABLED
  const uint16_t sourceRaw = (uint16_t)analogRead(SOURCE_VOLTAGE_PIN);
  if (sourceRaw >= ADC_RAIL_TRIP_RAW) {
    latchFault(FAULT_ADC_RAIL);
    calibrationWizardState = CAL_WIZARD_FAILED;
    return;
  }
  const int32_t calibrationSourceVoltageCenti = adcToSourceCenti(sourceRaw);
  if ((calibrationWizardState == CAL_WIZARD_RAMP ||
       calibrationWizardState == CAL_WIZARD_WAIT_DMM) &&
      (calibrationSourceVoltageCenti * 100L) <
      (CAL_TARGET_V_CENTI * CAL_MIN_SOURCE_HEADROOM_PERCENT)) {
    writeDutyBasisPoints(0L, 0L);
    resetCalibrationDutyCommand();
    resetCalibrationAverages();
    calibrationWizardState = CAL_WIZARD_WAIT_LOAD;
    calibrationPrint(F("VIN LOW; RETRY"));
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
        calibrationWindowReportSent = false;
        calibrationWizardState = CAL_WIZARD_WAIT_DMM;
        calibrationPrint(F("STABLE; SEND DMM"));
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
    calibrationWindowReportSent = false;
    calibrationStableStartMs = 0UL;
    calibrationLastVoltageCenti = voltageCenti;
    calibrationWizardState = CAL_WIZARD_RAMP;
    calibrationPrint(F("DRIFT; RETRY ADC"));
  }

  if (calibrationWizardState == CAL_WIZARD_WAIT_DMM) {
    if (absI32(CAL_TARGET_V_CENTI - voltageCenti) <=
            (CAL_STABLE_BAND_CENTI * 2L) &&
        voltageStepCenti <= 5L) {
      accumulateCalibrationSample(voltageRaw, currentRaw,
#if SOURCE_SENSE_ENABLED
                                  sourceRaw,
#else
                                  0U,
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
                                  sourceCurrentRaw
#else
                                  0U
#endif
                                  );
    } else {
      resetCalibrationAverages();
    }
  }

  // Conservative primary-only calibration regulator. It is intentionally
  // separate from adaptive control; AUX PWM remains disabled throughout.
  const int32_t error = CAL_TARGET_V_CENTI - voltageCenti;
  const int32_t appliedCalibrationDuty =
      advanceCalibrationDutyCommand(error, dtUs);
  writeDutyBasisPoints(appliedCalibrationDuty, 0L);
}

// -------------------------- Adaptive identification -----------------------
static FW_NOINLINE void resetAdaptiveModel(AdaptiveModel &model) {
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    model.thetaQ14[i] = 0;
    model.featureExcitationCount[i] = 0U;
  }
  model.disturbanceQ14 = 0;
  model.residualAbsQ14 = 0;
}

static FW_NOINLINE int32_t linearPredictionQ14(const AdaptiveModel &model,
                                   const int16_t phiQ14[MODEL_FEATURE_COUNT]) {
  int32_t sum = 0L;
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i)
    sum += ((int32_t)model.thetaQ14[i] * phiQ14[i]) >> 14;
  return sum;
}

// L1-normalized LMS estimator for the 8-bit AVR. The same four-input plant
// model, disturbance observer, excitation counters and confidence gates remain;
// only the costly RLS covariance matrix is replaced by a normalized gradient.
// A faster step is used during transients; steady-state updates run every eighth
// control cycle with a smaller step. This is an adaptive estimator, not RLS.
static FW_NOINLINE void updateAdaptiveModel(AdaptiveModel &model,
                                const int16_t phiQ14[MODEL_FEATURE_COUNT],
                                int16_t measuredYQ14,
                                bool fastAdaptation) {
  const int32_t innovation = (int32_t)measuredYQ14 -
      linearPredictionQ14(model, phiQ14) - model.disturbanceQ14;
  const int32_t absInnovation = absI32(innovation);
  model.residualAbsQ14 = clampI16(
      (int32_t)model.residualAbsQ14 +
      (absInnovation - model.residualAbsQ14) / 8L, 0, 16384);
  const int32_t disturbanceStep = innovation / 16L;
  model.disturbanceQ14 = clampI16(
      (int32_t)model.disturbanceQ14 + disturbanceStep,
      (int16_t)-MODEL_DISTURBANCE_LIMIT_Q14,
      (int16_t)MODEL_DISTURBANCE_LIMIT_Q14);

  uint32_t normQ14 = 256UL;
  bool excited = false;
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    const int32_t magnitude = absI32(phiQ14[i]);
    normQ14 += (uint32_t)magnitude;
    if (magnitude >= MODEL_MIN_EXCITATION_Q14) {
      excited = true;
      if (model.featureExcitationCount[i] < 255U)
        ++model.featureExcitationCount[i];
    }
  }
  if (!excited) return;

  const int32_t muQ14 = fastAdaptation
      ? MODEL_NLMS_FAST_MU_Q14 : MODEL_NLMS_SLOW_MU_Q14;
  const int32_t stepQ14 = (muQ14 * (innovation - disturbanceStep)) /
                          (int32_t)normQ14;
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    const int32_t deltaTheta = (stepQ14 * (int32_t)phiQ14[i]) >> 14;
    model.thetaQ14[i] = clampI16(
        (int32_t)model.thetaQ14[i] + deltaTheta, -16384, 16384);
  }
}

static FW_NOINLINE bool modelHasParameterConfidence(const AdaptiveModel &model,
                                         uint8_t featureIndex,
                                         uint16_t minimumExcitations) {
  return featureIndex < MODEL_FEATURE_COUNT &&
      model.featureExcitationCount[featureIndex] >= minimumExcitations &&
      model.thetaQ14[featureIndex] > MODEL_MIN_GAIN_Q14;
}

static FW_NOINLINE bool modelHasFeatureConfidence(const AdaptiveModel &model,
                                      uint8_t featureIndex,
                                      uint16_t minimumExcitations) {
  return modelHasParameterConfidence(model, featureIndex, minimumExcitations) &&
      model.residualAbsQ14 < MODEL_RESIDUAL_BAD_Q14;
}

static FW_NOINLINE bool modelHasInputConfidence(const AdaptiveModel &model) {
  return modelHasFeatureConfidence(model, 1U, MODEL_MIN_EXCITATIONS);
}

static FW_NOINLINE bool modelHasPrimaryCurrentConfidence(const AdaptiveModel &model) {
  return modelHasFeatureConfidence(model, 1U, MODEL_MIN_EXCITATIONS);
}

static FW_NOINLINE bool modelHasAuxiliaryConfidence(const AdaptiveModel &model) {
  return modelHasFeatureConfidence(model, 2U, AUX_MODEL_MIN_EXCITATIONS);
}

static FW_NOINLINE bool modelHasBothCurrentActuators(const AdaptiveModel &model) {
  return modelHasPrimaryCurrentConfidence(model) &&
         modelHasAuxiliaryConfidence(model);
}

static const AdaptiveModel *selectModel(const AdaptiveModel &model,
                                   uint8_t inputFeatureIndex,
                                   bool allowKnownProbeResidual) {
  const bool valid = allowKnownProbeResidual
      ? modelHasParameterConfidence(model, inputFeatureIndex, MODEL_MIN_EXCITATIONS)
      : modelHasFeatureConfidence(model, inputFeatureIndex, MODEL_MIN_EXCITATIONS);
  return valid ? &model : 0;
}

static FW_NOINLINE void updateLoadEstimate(const AdaptiveModel *voltagePlant) {
  loadEstimateValid = false;
  if (voltagePlant == 0 || !modelHasInputConfidence(*voltagePlant)) return;
  const int32_t gain = voltagePlant->thetaQ14[1];
  if (gain < MODEL_MIN_GAIN_Q14) return;
  const int32_t disturbanceCurrent =
      -((int32_t)voltagePlant->disturbanceQ14 * MODEL_CURRENT_DEVIATION_CENTI) / gain;
  const int32_t target = clampI32(currentMeanCenti + disturbanceCurrent,
                                  0L, CURRENT_LIMIT_CENTI * 2L);
  if (!loadEstimateInitialized) {
    estimatedLoadCurrentQ8 = target * 256L;
    estimatedLoadCurrentCenti = target;
    loadEstimateInitialized = true;
  } else {
    estimatedLoadCurrentQ8 += (target * 256L - estimatedLoadCurrentQ8) / 8L;
    estimatedLoadCurrentCenti = estimatedLoadCurrentQ8 / 256L;
  }
  loadEstimateValid = true;
}

// -------------------------- Pattern evaluation ---------------------------
static FW_NOINLINE void updateVoltageTrend(int32_t voltageCenti,
                               int32_t voltageErrorCenti) {
  if (trendWindowTicks == 0U) trendWindowStartVoltage = voltageCenti;
  if (++trendWindowTicks >= VOLTAGE_SLOPE_WINDOW_TICKS) {
    // 25 controller ticks are 50.075 ms, so x20 gives cV/s with <0.2% scale error.
    voltageSlopeCentiPerSec = (voltageCenti - trendWindowStartVoltage) * 20L;
    trendWindowTicks = 0U;
  }
  if (++crossingWindowTicks >= CROSSING_WINDOW_TICKS) {
    crossingWindowTicks = 0U;
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

static FW_NOINLINE void updateAuxiliaryLowPowerState(int32_t currentCenti,
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
        transientHoldCounter > 0U ||
        currentCenti >= (runtimeCurrentLimitCenti * OUTPUT_CURRENT_BOUND_PERCENT) / 100L)
      auxiliaryLowPowerActive = false;
  }
#else
  (void)currentCenti;
  (void)voltageErrorCenti;
  auxiliaryLowPowerActive = false;
#endif
}

#if AUX_PATH_ARMED
static FW_NOINLINE void updateAuxiliaryPathMode(int32_t measuredCurrentCenti,
                                    bool auxiliaryModelReady,
                                    bool efficiencyPreferred) {
  if (!auxiliaryLowPowerActive || !auxiliaryModelReady ||
      !efficiencyPreferred || measuredCurrentCenti > AUX_LOW_POWER_EXIT_CENTI ||
      measuredCurrentCenti >=
          (runtimeCurrentLimitCenti * OUTPUT_CURRENT_BOUND_PERCENT) / 100L) {
    auxiliaryPathMode = AUX_MODE_PRIMARY_ONLY;
    return;
  }
  if (auxiliaryPathMode == AUX_MODE_PRIMARY_ONLY) {
    if (appliedDutyBp <= AUX_OVERLAP_ENTER_BP)
      auxiliaryPathMode = AUX_MODE_OVERLAP;
  } else if (auxiliaryPathMode == AUX_MODE_OVERLAP) {
    if (appliedDutyBp >= AUX_OVERLAP_EXIT_BP)
      auxiliaryPathMode = AUX_MODE_PRIMARY_ONLY;
    else if (primaryEquivalentDutyDemandBp <= AUX_ONLY_ENTER_BP &&
             measuredCurrentCenti <= AUX_ONLY_MAX_CURRENT_CENTI)
      auxiliaryPathMode = AUX_MODE_AUXILIARY_ONLY;
  } else if (auxiliaryPathMode == AUX_MODE_AUXILIARY_ONLY) {
    if (primaryEquivalentDutyDemandBp >= AUX_ONLY_EXIT_BP ||
        measuredCurrentCenti > AUX_ONLY_EXIT_CURRENT_CENTI)
      auxiliaryPathMode = AUX_MODE_OVERLAP;
  } else {
    auxiliaryPathMode = AUX_MODE_PRIMARY_ONLY;
  }
}
#endif

static FW_NOINLINE ControlRegime classifyControlRegime(int32_t voltageErrorCenti,
                                        int32_t currentCenti,
                                        int32_t deltaCurrentCenti,
                                        int32_t deltaSourceCenti,
                                        int32_t deltaSourceCurrentCenti,
                                        const AdaptiveModel *currentSelected,
                                        const AdaptiveModel *voltageSelected,
                                        bool knownControlExcitation) {
  if (currentCenti >= (runtimeCurrentLimitCenti * OUTPUT_CURRENT_BOUND_PERCENT) / 100L)
    return REGIME_CURRENT_BOUND;
  if (absI32(deltaSourceCenti) >= SOURCE_STEP_TRANSIENT_CENTI ||
      sourceCurrentStepDetected(deltaSourceCurrentCenti)) return REGIME_FAST;
  if (crossingCount >= OSCILLATION_CROSSINGS) return REGIME_DAMPED;
  if (voltageErrorCenti < -VOLTAGE_ERROR_DEADBAND_CENTI) return REGIME_NORMAL;
  if (voltageErrorCenti > 0L && voltageErrorCenti <= APPROACH_BAND_CENTI &&
      voltageSlopeCentiPerSec >= FAST_APPROACH_SLOPE_CV_S) return REGIME_DAMPED;

  if (!knownControlExcitation &&
      (currentModel.residualAbsQ14 >= MODEL_RESIDUAL_BAD_Q14 ||
       voltageModel.residualAbsQ14 >= MODEL_RESIDUAL_BAD_Q14)) {
    if (nonlinearResidualCounter < 255U) ++nonlinearResidualCounter;
  } else if (nonlinearResidualCounter > 0U) {
    --nonlinearResidualCounter;
  }
  if (nonlinearResidualCounter >= 24U) return REGIME_DAMPED;
  if (nonlinearResidualCounter >= 8U || transientHoldCounter > 0U ||
      (!knownControlExcitation &&
       absI32(deltaCurrentCenti) >= CURRENT_STEP_TRANSIENT_CENTI))
    return REGIME_FAST;
  if (currentSelected == 0 || voltageSelected == 0 ||
      currentSelected->residualAbsQ14 >= MODEL_RESIDUAL_BAD_Q14 ||
      voltageSelected->residualAbsQ14 >= MODEL_RESIDUAL_BAD_Q14)
    return REGIME_DAMPED;
  return REGIME_NORMAL;
}

// Keep source-current setting independent from the output-current setting.
// Input protection acts in its own current-feedback guard below; it does not
// overwrite the thermally derated output-current ceiling.
static FW_NOINLINE void refreshInputCurrentLimit() {
  runtimeInputCurrentLimitCenti = inputCurrentSettingCenti;
}

static FW_NOINLINE int32_t controlOutputCurrentLimitCenti() {
  int32_t limit = runtimeCurrentLimitCenti;
#if MPPT_ENABLED
  if ((int32_t)mpptOutputCurrentLimitCenti < limit)
    limit = mpptOutputCurrentLimitCenti;
#endif
  return (limit > 0L) ? limit : 0L;
}

// Apply PWM inhibits after mode scheduling and slew limiting.
static FW_NOINLINE void enforcePowerStageInhibits(bool forceAuxiliaryOff,
                                      bool forceBothPwmOff,
                                      int32_t &primaryDuty,
                                      int32_t &auxiliaryDuty) {
  if (forceAuxiliaryOff) auxiliaryDuty = 0L;
  if (forceBothPwmOff) {
    primaryDuty = 0L;
    auxiliaryDuty = 0L;
  }
}

// Independent source-current feedback guard. MPPT is intentionally limited
// to 85% of this setting; this guard begins at 90%, leaving margin and avoiding
// a fight between the MPPT search and the source protection loop.
static FW_NOINLINE void applyInputCurrentGuard(int32_t sourceCurrentInstant,
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
      (runtimeInputCurrentLimitCenti * INPUT_CURRENT_GUARD_START_PERCENT) / 100L);
  const int32_t inputGuardHardCenti = (int32_t)(
      (runtimeInputCurrentLimitCenti * INPUT_CURRENT_GUARD_HARD_PERCENT) / 100L);
  const int32_t inputGuardReleaseCenti = (int32_t)(
      (runtimeInputCurrentLimitCenti * INPUT_CURRENT_GUARD_RELEASE_PERCENT) / 100L);
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
        0L, controlOutputCurrentLimitCenti());
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

// Direct A5/A6 pots are sampled between control updates.
#if (OUTPUT_LIMIT_POT_PIN >= 0) || (INPUT_LIMIT_POT_PIN >= 0)
struct LimitPotState {
  int32_t filteredRawQ8;
  uint8_t initialised;
};
static LimitPotState outputLimitPotState = { 0L, 0U };
static LimitPotState inputLimitPotState = { 0L, 0U };
static uint32_t lastLimitPotSampleMs = 0UL;
static uint8_t nextLimitPotChannel = 0U;

static FW_NOINLINE int32_t mapPotToCurrentLimit(int32_t raw, int32_t minimum,
                                    int32_t maximum) {
  raw = clampI32(raw, 0L, 1023L);
  // ADC raw <=1023 and configured current span <=400 cA, so this
  // mapping is safely bounded below 410k in signed 32-bit arithmetic.
  return minimum + (int32_t)(((uint32_t)raw *
      (uint32_t)(maximum - minimum) + 511U) / 1023U);
}

static FW_NOINLINE void serviceOneLimitPot(uint8_t pin, LimitPotState &state,
                               int32_t minimum, int32_t maximum,
                               int32_t &setting) {
  const uint16_t raw = (uint16_t)analogRead(pin);
  const int32_t requestedLimit = mapPotToCurrentLimit(raw, minimum, maximum);
  if (!state.initialised) {
    state.filteredRawQ8 = (int32_t)raw * 256L;
    state.initialised = 1U;
    setting = requestedLimit;
    return;
  }

  // A user turning a limit down takes effect immediately; only increases are
  // smoothed. This makes the pot responsive in the protective direction.
  if (requestedLimit < setting) {
    state.filteredRawQ8 = (int32_t)raw * 256L;
    setting = requestedLimit;
    return;
  }

  const int32_t targetQ8 = (int32_t)raw * 256L;
  state.filteredRawQ8 += (targetQ8 - state.filteredRawQ8) >>
                         LIMIT_POT_FILTER_SHIFT;
  const int32_t filtered = state.filteredRawQ8 / 256L;
  setting = mapPotToCurrentLimit(filtered, minimum, maximum);
}
#endif

static FW_NOINLINE void serviceLimitPotentiometers() {
#if (OUTPUT_LIMIT_POT_PIN >= 0) || (INPUT_LIMIT_POT_PIN >= 0)
  const uint32_t nowMs = (uint32_t)millis();
  if ((uint32_t)(nowMs - lastLimitPotSampleMs) < LIMIT_POT_SAMPLE_INTERVAL_MS)
    return;
  lastLimitPotSampleMs = nowMs;

#if (OUTPUT_LIMIT_POT_PIN >= 0) && (INPUT_LIMIT_POT_PIN >= 0)
  if (nextLimitPotChannel == 0U) {
    serviceOneLimitPot((uint8_t)OUTPUT_LIMIT_POT_PIN, outputLimitPotState,
                       OUTPUT_POT_MIN_CENTI, OUTPUT_POT_MAX_CENTI,
                       outputCurrentSettingCenti);
    nextLimitPotChannel = 1U;
  } else {
    serviceOneLimitPot((uint8_t)INPUT_LIMIT_POT_PIN, inputLimitPotState,
                       INPUT_POT_MIN_CENTI, INPUT_POT_MAX_CENTI,
                       inputCurrentSettingCenti);
    nextLimitPotChannel = 0U;
  }
#elif (OUTPUT_LIMIT_POT_PIN >= 0)
  serviceOneLimitPot((uint8_t)OUTPUT_LIMIT_POT_PIN, outputLimitPotState,
                     OUTPUT_POT_MIN_CENTI, OUTPUT_POT_MAX_CENTI,
                     outputCurrentSettingCenti);
#elif (INPUT_LIMIT_POT_PIN >= 0)
  serviceOneLimitPot((uint8_t)INPUT_LIMIT_POT_PIN, inputLimitPotState,
                     INPUT_POT_MIN_CENTI, INPUT_POT_MAX_CENTI,
                     inputCurrentSettingCenti);
#endif
  refreshInputCurrentLimit();
#endif
}

#if MPPT_ENABLED
static FW_NOINLINE void updateMeasuredEfficiency(int32_t sourceVoltageCenti,
                                     int32_t sourceCurrentCenti,
                                     int32_t outputVoltageCenti,
                                     int32_t outputCurrentCenti) {
  // Valid measurement envelope also bounds outputProduct*1000 below INT32_MAX.
  if (sourceVoltageCenti <= 0L ||
      sourceVoltageCenti > EFFICIENCY_MAX_SOURCE_VOLTAGE_CENTI ||
      sourceCurrentCenti <= 0L ||
      sourceCurrentCenti > EFFICIENCY_MAX_CURRENT_CENTI ||
      outputVoltageCenti <= 0L ||
      outputVoltageCenti > OUTPUT_OVERVOLTAGE_CENTI ||
      outputCurrentCenti <= 0L ||
      outputCurrentCenti > EFFICIENCY_MAX_CURRENT_CENTI) return;

  const int32_t inputProduct = sourceVoltageCenti * sourceCurrentCenti;
  const int32_t outputProduct = outputVoltageCenti * outputCurrentCenti;
  if (inputProduct < MPPT_EFFICIENCY_MIN_INPUT_PRODUCT ||
      outputProduct < MPPT_EFFICIENCY_MIN_OUTPUT_PRODUCT ||
      outputProduct >= inputProduct) return;

  const int32_t measured = (outputProduct * 1000L) / inputProduct;
  if (measured < 400L || measured > 980L) return;
  const uint16_t conservative = (uint16_t)clampI32(
      measured - MPPT_EFFICIENCY_MARGIN_PERMILLE, 400L, 900L);
  if (mpptEfficiencyEstimatePermille == 0U) {
    mpptEfficiencyEstimatePermille = conservative;
  } else {
    const int16_t difference = (int16_t)conservative -
                               (int16_t)mpptEfficiencyEstimatePermille;
    mpptEfficiencyEstimatePermille = (uint16_t)clampI32(
        (int32_t)mpptEfficiencyEstimatePermille + difference / 64,
        400L, 900L);
  }
}

static FW_NOINLINE void updateMppt(int32_t sourceVoltageCenti,
                       int32_t sourceCurrentCenti,
                       int32_t outputVoltageCenti,
                       int32_t voltageErrorCenti,
                       bool externalTransient,
                       bool currentGuardNear) {
  const uint16_t minimum = (uint16_t)MPPT_MIN_INPUT_CURRENT_CENTI;
  const uint16_t maximum = (uint16_t)(
      ((uint16_t)runtimeInputCurrentLimitCenti *
       (uint16_t)MPPT_MAX_SOURCE_CURRENT_PERCENT) / 100U);

  if (maximum < minimum || runtimeCurrentLimitCenti <= 0L) {
    mpptInputCurrentTargetCenti = 0U;
    mpptOutputCurrentLimitCenti = (uint16_t)((runtimeCurrentLimitCenti > 0L)
        ? runtimeCurrentLimitCenti : 0L);
    mpptDirection = 1;
    mpptPreviousPowerProduct = 0UL;
    mpptSettleTicks = 0U;
    return;
  }
  if (mpptInputCurrentTargetCenti == 0U) {
    const int32_t start = clampI32(MPPT_START_INPUT_CURRENT_CENTI,
                                  minimum, maximum);
    mpptInputCurrentTargetCenti = (uint16_t)start;
    mpptDirection = 1;
    mpptPreviousPowerProduct = 0UL;
    // The initialization call itself immediately enters the settling branch
    // below; compensate for that first decrement.
    mpptSettleTicks = (uint8_t)(MPPT_SETTLE_TICKS + 1UL);
  }

  // No usable source or buck headroom: inhibit the load. Back off the
  // allowed source-current ceiling one step per settled interval.
  const int32_t vout = (outputVoltageCenti > 100L) ? outputVoltageCenti : 100L;
  if (sourceVoltageCenti < MPPT_MIN_SOURCE_VOLTAGE_CENTI) {
    mpptInputCurrentTargetCenti = minimum;
    mpptOutputCurrentLimitCenti = 0U;
    mpptDirection = 1;
    mpptPreviousPowerProduct = 0UL;
    mpptSettleTicks = (uint8_t)MPPT_SETTLE_TICKS;
    return;
  }
  if (sourceVoltageCenti * 100L < vout * MPPT_MIN_HEADROOM_PERCENT) {
    mpptOutputCurrentLimitCenti = 0U;
    mpptDirection = 1;
    mpptPreviousPowerProduct = 0UL;
    if (mpptSettleTicks != 0U) --mpptSettleTicks;
    if (mpptSettleTicks == 0U) {
      mpptInputCurrentTargetCenti = (mpptInputCurrentTargetCenti >
          (uint16_t)(minimum + MPPT_STEP_CENTI))
          ? (uint16_t)(mpptInputCurrentTargetCenti - MPPT_STEP_CENTI)
          : minimum;
      mpptSettleTicks = (uint8_t)MPPT_SETTLE_TICKS;
    }
    return;
  }

  // Convert the allowed input power to an output-current ceiling using the
  // measured, deliberately derated conversion efficiency.
  const uint16_t efficiency = (mpptEfficiencyEstimatePermille != 0U)
      ? mpptEfficiencyEstimatePermille
      : (uint16_t)MPPT_ASSUMED_EFFICIENCY_PERMILLE;
  const uint32_t allowedInputPower = (uint32_t)sourceVoltageCenti *
      (uint32_t)mpptInputCurrentTargetCenti;
  const uint32_t outputCap = (allowedInputPower * efficiency) /
      ((uint32_t)vout * 1000UL);
  mpptOutputCurrentLimitCenti = (uint16_t)((outputCap <
      (uint32_t)runtimeCurrentLimitCenti) ? outputCap :
      (uint32_t)runtimeCurrentLimitCenti);

  if (externalTransient || currentGuardNear) {
    mpptPreviousPowerProduct = 0UL;
    mpptSettleTicks = (uint8_t)MPPT_SETTLE_TICKS;
    return;
  }
  if (voltageErrorCenti <= MPPT_LOAD_LIMIT_ERROR_CENTI &&
      sourceCurrentCenti * 100L <
          (int32_t)mpptInputCurrentTargetCenti * MPPT_LOAD_LIMIT_CURRENT_PERCENT) {
    mpptInputCurrentTargetCenti = maximum;
    mpptOutputCurrentLimitCenti = (uint16_t)runtimeCurrentLimitCenti;
    mpptDirection = 1;
    mpptPreviousPowerProduct = 0UL;
    mpptSettleTicks = (uint8_t)MPPT_SETTLE_TICKS;
    return;
  }
  if (mpptSettleTicks != 0U) {
    --mpptSettleTicks;
    if (mpptSettleTicks != 0U) return;
  }

  // Compare measured Pin, not the current ceiling itself. Sensor low-pass
  // filtering provides the averaging; perturb once per settled interval.
  const uint32_t measuredPower = (uint32_t)sourceVoltageCenti *
                                 (uint32_t)sourceCurrentCenti;
  if (measuredPower == 0UL) {
    mpptPreviousPowerProduct = 0UL;
    mpptSettleTicks = (uint8_t)MPPT_SETTLE_TICKS;
    return;
  }
  if (mpptPreviousPowerProduct != 0UL) {
    const uint32_t deadband = measuredPower *
        (uint32_t)MPPT_POWER_DEADBAND_PERMILLE / 1000UL;
    if (measuredPower + deadband < mpptPreviousPowerProduct)
      mpptDirection = (int8_t)-mpptDirection;
  }
  mpptPreviousPowerProduct = measuredPower;

  int32_t nextTarget = (int32_t)mpptInputCurrentTargetCenti +
      ((mpptDirection > 0) ? MPPT_STEP_CENTI : -MPPT_STEP_CENTI);
  if (nextTarget < minimum) nextTarget = minimum;
  if (nextTarget > maximum) nextTarget = maximum;
  mpptInputCurrentTargetCenti = (uint16_t)nextTarget;
  mpptSettleTicks = (uint8_t)MPPT_SETTLE_TICKS;
}
#endif
#if AUX_EFFICIENCY_LEARNING_ENABLED
static FW_NOINLINE uint8_t efficiencyCurrentBin(int32_t outputCurrentCenti) {
  int32_t bin = outputCurrentCenti / EFFICIENCY_CURRENT_BIN_CENTI;
  if (bin < 0L) bin = 0L;
  if (bin >= (int32_t)EFFICIENCY_CURRENT_BIN_COUNT)
    bin = (int32_t)EFFICIENCY_CURRENT_BIN_COUNT - 1L;
  return (uint8_t)bin;
}

static FW_NOINLINE uint8_t efficiencySourceBin(int32_t sourceVoltageCenti) {
  if (sourceVoltageCenti < EFFICIENCY_SOURCE_BIN_LOW_CENTI) return 0U;
  if (sourceVoltageCenti < EFFICIENCY_SOURCE_BIN_HIGH_CENTI) return 1U;
  return 2U;
}

static FW_NOINLINE void recordEfficiencyMeasurement(int32_t sourceVoltageCenti,
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
  const int32_t inputOutputProductDifference =
      sourceVoltageCenti * sourceCurrentCenti -
      outputVoltageCenti * outputCurrentCenti;
  if (inputOutputProductDifference <= 0L) {
    // Do not turn sensor-offset errors into an apparent zero-loss advantage.
    efficiencyBlockLossSum10mW = 0UL;
    efficiencyBlockSamples = 0U;
    return;
  }
  uint32_t loss10mW = (uint32_t)(inputOutputProductDifference / 100L);
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

static FW_NOINLINE bool empiricalAuxiliaryLossAvailable(int32_t outputCurrentCenti,
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

static FW_NOINLINE int32_t empiricalAuxiliaryLossCost(int32_t outputCurrentCenti,
                                          int32_t sourceVoltageCenti) {
  const uint8_t currentBin = efficiencyCurrentBin(outputCurrentCenti);
  const uint8_t sourceBin = efficiencySourceBin(sourceVoltageCenti);
  const EfficiencyProfile &primary =
      efficiencyProfiles[0U][currentBin][sourceBin];
  const EfficiencyProfile &auxiliary =
      efficiencyProfiles[1U][currentBin][sourceBin];
  if (!empiricalAuxiliaryLossAvailable(outputCurrentCenti, sourceVoltageCenti))
    return 0L;

  int32_t difference10mW = (int32_t)auxiliary.meanLoss10mW -
                           (int32_t)primary.meanLoss10mW;
  if (absI32(difference10mW) <= EFFICIENCY_LOSS_DEADBAND_10MW) return 0L;
  if (difference10mW > 0L) difference10mW -= EFFICIENCY_LOSS_DEADBAND_10MW;
  else difference10mW += EFFICIENCY_LOSS_DEADBAND_10MW;
  return difference10mW * (int32_t)EFFICIENCY_COST_PER_10MW;
}
#endif

// -------------------------- Model prediction helpers ---------------------
static FW_NOINLINE int32_t effectiveDutyLevelBp(int32_t primaryDutyBp,
                                    int32_t auxiliaryDutyBp) {
  if (primaryDutyBp > 0L && auxiliaryDutyBp > 0L)
    return (primaryDutyBp + auxiliaryDutyBp) / 2L;
  return (primaryDutyBp > 0L) ? primaryDutyBp : auxiliaryDutyBp;
}

static FW_NOINLINE int32_t scaledDisturbanceQ14(const AdaptiveModel &model,
                                    uint16_t weightPermille) {
  return (int32_t)(((int32_t)model.disturbanceQ14 * (int32_t)weightPermille) / 1000L);
}

static FW_NOINLINE int32_t predictCurrentNoDutyQ14(const AdaptiveModel &model,
                                       int32_t deltaCurrentCenti,
                                       int32_t deltaVoltageCenti,
                                       int32_t deltaSourceCenti,
                                       int32_t dutyLevelBp,
                                       uint16_t disturbanceWeightPermille) {
  const int16_t phi0 = toQ14(deltaCurrentCenti, MODEL_I_SCALE_CENTI);
  const int16_t phi3 = sourceOutputIncrementQ14(dutyLevelBp,
                                                 deltaSourceCenti,
                                                 deltaVoltageCenti);
  const int32_t lagTerm =
      ((int32_t)model.thetaQ14[0] * phi0) >> 14;
  const int32_t sourceOutputTerm =
      ((int32_t)model.thetaQ14[3] * phi3) >> 14;
  return lagTerm + sourceOutputTerm +
         scaledDisturbanceQ14(model, disturbanceWeightPermille);
}

static FW_NOINLINE int32_t predictVoltageBaseQ14(const AdaptiveModel &model,
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
  const int32_t selfTerm = (int32_t)(((int32_t)model.thetaQ14[0] * phi0) >> 14);
  const int32_t sourceTerm = (int32_t)(((int32_t)model.thetaQ14[2] * phi2) >> 14);
  return selfTerm + sourceTerm + scaledDisturbanceQ14(model,
                                                       disturbanceWeightPermille);
}

// -------------------------- Bootstrap PI ---------------------------------
// Computes (error * gain * dtUs) / 1e6 without a 64-bit intermediate.
// Bounds: |error| <= 3000, gain <= 3277, dtUs <= MAX_CONTROL_DT_US (5000).
// The two quotient/remainder decompositions keep every product below 50M.
static FW_NOINLINE int32_t boundedRateIntegralDeltaQ16(int32_t error, int32_t gain,
                                            uint32_t dtUs) {
  if (dtUs > MAX_CONTROL_DT_US) dtUs = MAX_CONTROL_DT_US;
  const bool negative = ((error < 0L) != (gain < 0L));
  const uint32_t magnitudeError = (uint32_t)absI32(error);
  const uint32_t magnitudeGain = (uint32_t)absI32(gain);
  const uint32_t product = magnitudeError * magnitudeGain;
  const uint32_t productWhole = product / 1000U;
  const uint32_t productRemainder = product % 1000U;
  const uint32_t dtWholeMs = dtUs / 1000U;
  const uint32_t dtRemainderUs = dtUs % 1000U;
  // Exact expansion:
  // product*dt/1e6 = q*m + (q*s + r*m)/1000 + r*s/1e6.
  const uint32_t crossNumerator =
      productWhole * dtRemainderUs + productRemainder * dtWholeMs;
  const uint32_t fractionalNumerator =
      (crossNumerator % 1000U) * 1000U +
      productRemainder * dtRemainderUs;
  const uint32_t magnitude = productWhole * dtWholeMs +
      crossNumerator / 1000U + fractionalNumerator / 1000000U;
  return negative ? -(int32_t)magnitude : (int32_t)magnitude;
}

static FW_NOINLINE int32_t bootstrapCurrentReference(int32_t voltageErrorCenti,
                                         uint32_t dtUs) {
  const int32_t error = clampI32(voltageErrorCenti, -3000L, 3000L);
  const int32_t pQ16 = error * BOOT_OUTER_KP_Q8 * 256L;
  const int32_t deltaI_Q16 = boundedRateIntegralDeltaQ16(
      error, (int32_t)BOOT_OUTER_KI_Q16, dtUs);
  const int32_t maxI_Q16 = controlOutputCurrentLimitCenti() * 65536L;
  const int32_t proposed = clampI32(voltageIntegratorQ16 + deltaI_Q16,
                                    -maxI_Q16, maxI_Q16);
  const bool pushingHigh = (pQ16 + proposed > maxI_Q16 && deltaI_Q16 > 0L);
  const bool pushingLow = (pQ16 + proposed < 0L && deltaI_Q16 < 0L);
  if (!pushingHigh && !pushingLow) voltageIntegratorQ16 = proposed;
  int32_t output = pQ16 + voltageIntegratorQ16;
  output = clampI32(output, 0L, maxI_Q16);
  return (int32_t)((output + 32768L) / 65536L);
}

static FW_NOINLINE int32_t bootstrapDuty(int32_t currentErrorCenti, uint32_t dtUs) {
  const int32_t error = clampI32(currentErrorCenti, -1000L, 1000L);
  const int32_t pQ16 = error * BOOT_INNER_KP_Q8 * 256L;
  const int32_t deltaDutyQ16 = boundedRateIntegralDeltaQ16(
      error, (int32_t)BOOT_INNER_KI_Q16, dtUs);
  const int32_t maxDutyQ16 = MAX_DUTY_BASIS_POINTS * 65536L;
  const int32_t proposed = clampI32(currentIntegratorQ16 + deltaDutyQ16,
                                    -maxDutyQ16, maxDutyQ16);
  const bool pushingHigh = (pQ16 + proposed > maxDutyQ16 && deltaDutyQ16 > 0L);
  const bool pushingLow = (pQ16 + proposed < 0L && deltaDutyQ16 < 0L);
  if (!pushingHigh && !pushingLow) currentIntegratorQ16 = proposed;
  int32_t output = pQ16 + currentIntegratorQ16;
  output = clampI32(output, 0L, maxDutyQ16);
  return (int32_t)((output + 32768L) / 65536L);
}

// -------------------------- Adaptive actuator selection -------------------
static FW_NOINLINE bool auxiliaryEfficiencyPreferred(int32_t outputCurrentCenti,
                                                       int32_t sourceVoltageCenti) {
#if AUX_EFFICIENCY_LEARNING_ENABLED
  if (empiricalAuxiliaryLossAvailable(outputCurrentCenti, sourceVoltageCenti))
    return empiricalAuxiliaryLossCost(outputCurrentCenti, sourceVoltageCenti) < 0L;
#endif
  // Conservative prior before enough matched A/B blocks exist. The auxiliary
  // branch is preferred only when its estimated switching credit exceeds its
  // additional conduction loss at this current.
  const int32_t current = clampI32(outputCurrentCenti, 0L, EFFICIENCY_MAX_CURRENT_CENTI);
  const int32_t extraLoss = (current * current *
      ((int32_t)AUX_RDS_RELATIVE_Q8 - 256L)) / 25600L;
  const int32_t switchingCredit =
      (current * (int32_t)AUX_SWITCHING_LOSS_CREDIT_Q8) / 256L;
  return switchingCredit > extraLoss;
}

// -------------------------- Main control update --------------------------
static FW_NOINLINE void controlUpdate(uint32_t dtUs) {
  const uint16_t voltageRaw = (uint16_t)analogRead(VOLTAGE_PIN);
  const uint16_t currentRaw = (uint16_t)analogRead(CURRENT_PIN);
  const uint16_t thermistorRaw = (uint16_t)analogRead(THERMISTOR_PIN);
  const uint16_t sourceRaw = (uint16_t)analogRead(SOURCE_VOLTAGE_PIN);
  const uint16_t sourceCurrentRaw = (uint16_t)analogRead(SOURCE_CURRENT_PIN);

  if (!updateThermalLimit(thermistorRaw)) {
    if (faultCode == FAULT_NONE) latchFault(FAULT_THERMISTOR);
    return;
  }
  if (voltageRaw >= ADC_RAIL_TRIP_RAW || currentRaw <= 5U ||
      currentRaw >= ADC_RAIL_TRIP_RAW || sourceRaw >= ADC_RAIL_TRIP_RAW ||
      sourceCurrentRaw <= 5U || sourceCurrentRaw >= ADC_RAIL_TRIP_RAW) {
    latchFault(FAULT_ADC_RAIL);
    return;
  }

  const int32_t voltageInstant = adcToVoltageCenti(voltageRaw);
  const int32_t currentSignedInstant = adcToCurrentCenti(currentRaw);
  const int32_t sourceInstant = adcToSourceCenti(sourceRaw);
  const int32_t sourceCurrentSignedInstant = adcToSourceCurrentCenti(sourceCurrentRaw);
  if (currentSignedInstant < -ACS712_REVERSE_CURRENT_TRIP_CENTI ||
      sourceCurrentSignedInstant < -ACS712_REVERSE_CURRENT_TRIP_CENTI) {
    latchFault(FAULT_REVERSE_CURRENT);
    return;
  }
  const int32_t currentInstant = (currentSignedInstant > 0L) ? currentSignedInstant : 0L;
  const int32_t sourceCurrentInstant =
      (sourceCurrentSignedInstant > 0L) ? sourceCurrentSignedInstant : 0L;
  if (sourceCurrentInstant >= INPUT_HARD_OVERCURRENT_CENTI) {
    latchFault(FAULT_INPUT_OVERCURRENT);
    return;
  }
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
  int32_t sourceCenti;
  if (!filtersInitialized) {
    voltageFilteredQ8 = voltageInstant * 256L;
    currentFilteredQ8 = currentInstant * 256L;
    currentMeanQ16 = currentInstant * 65536L;
    currentMeanCenti = currentInstant;
    sourceFilteredQ8 = sourceInstant * 256L;
    sourceCurrentFilteredQ8 = sourceCurrentInstant * 256L;
    filtersInitialized = true;
    voltageCenti = voltageInstant;
    currentCenti = currentInstant;
    sourceCenti = sourceInstant;
  } else {
    voltageCenti = filterQ8(voltageInstant, voltageFilteredQ8);
    currentCenti = filterQ8(currentInstant, currentFilteredQ8);
    sourceCenti = filterQ8(sourceInstant, sourceFilteredQ8);
    (void)filterQ8(sourceCurrentInstant, sourceCurrentFilteredQ8);
    currentMeanQ16 += (currentCenti * 65536L - currentMeanQ16) / 4096L;
    currentMeanCenti = currentMeanQ16 / 65536L;
  }
  const int32_t sourceCurrentCenti = sourceCurrentFilteredQ8 / 256L;
  latestOutputCurrentCenti = currentCenti;
  latestInputCurrentCenti = sourceCurrentCenti;
  refreshInputCurrentLimit();

  // Slew the voltage reference independently of the plant controller.
  const uint32_t rateTime = (uint32_t)SOFTSTART_RATE_CV_PER_SEC * dtUs;
  const uint32_t referenceStepQ16 = (rateTime / 1000000UL) * 65536UL +
      ((rateTime % 1000000UL) * 4096UL) / 62500UL;
  const int32_t targetReferenceQ16 = TARGET_V_CENTI * 65536L;
  if (voltageReferenceQ16 < targetReferenceQ16) {
    const uint32_t next = (uint32_t)voltageReferenceQ16 + referenceStepQ16;
    voltageReferenceQ16 = (next >= (uint32_t)targetReferenceQ16)
        ? targetReferenceQ16 : (int32_t)next;
  } else if (voltageReferenceQ16 > targetReferenceQ16) {
    const int32_t next = voltageReferenceQ16 - (int32_t)referenceStepQ16;
    voltageReferenceQ16 = (next <= targetReferenceQ16) ? targetReferenceQ16 : next;
  }
  const int32_t voltageReference = voltageReferenceQ16 / 65536L;
  const int32_t voltageError = voltageReference - voltageCenti;
  const uint32_t nowMs = (uint32_t)millis();
  if (updateRunNoResponseMonitor(nowMs, voltageCenti, voltageReference,
      sourceCenti, currentInstant, controlOutputCurrentLimitCenti(),
      sourceCurrentInstant, runtimeInputCurrentLimitCenti,
      appliedDutyBp, appliedAuxiliaryDutyBp)) {
    latchFault(FAULT_VOLTAGE_NO_RESPONSE);
    return;
  }
  updateVoltageTrend(voltageCenti, voltageError);

  int32_t deltaVoltage = 0L;
  int32_t deltaCurrent = 0L;
  int32_t deltaSource = 0L;
  int32_t deltaSourceCurrent = 0L;
  if (previousMeasurementsValid) {
    deltaVoltage = voltageCenti - previousVoltageCenti;
    deltaCurrent = currentCenti - previousCurrentCenti;
    deltaSource = sourceCenti - previousSourceCenti;
    deltaSourceCurrent = sourceCurrentCenti - previousSourceCurrentCenti;

    const int16_t currentPhi[MODEL_FEATURE_COUNT] = {
      toQ14(previousDeltaCurrentCenti, MODEL_I_SCALE_CENTI),
      toQ14(previousDutyCenteredBp, MODEL_DUTY_SCALE_BP),
      auxiliaryIdentificationBaseReady
          ? toQ14(previousAuxiliaryDutyCenteredBp, AUX_MODEL_DUTY_SCALE_BP) : 0,
      sourceOutputIncrementQ14(previousDutyLevelBp, deltaSource, deltaVoltage)
    };
    const int16_t voltagePhi[MODEL_FEATURE_COUNT] = {
      toQ14(previousDeltaVoltageCenti, MODEL_V_SCALE_CENTI),
      toQ14(previousCurrentCenteredCenti, MODEL_CURRENT_DEVIATION_CENTI),
      toQ14(deltaSource, MODEL_SOURCE_SCALE_CENTI),
      0
    };
    const bool fastAdaptation = identificationProbeActive ||
        previousAuxiliaryProbeActive || transientHoldCounter > 0U ||
        absI32(deltaCurrent) >= CURRENT_STEP_TRANSIENT_CENTI ||
        absI32(deltaSource) >= SOURCE_STEP_TRANSIENT_CENTI ||
        sourceCurrentStepDetected(deltaSourceCurrent) ||
        currentModel.residualAbsQ14 >= MODEL_RESIDUAL_TRANSIENT_Q14 ||
        voltageModel.residualAbsQ14 >= MODEL_RESIDUAL_TRANSIENT_Q14;
    modelUpdatePhase = (uint8_t)((modelUpdatePhase + 1U) &
                                 (MODEL_SLOW_UPDATE_DIVIDER - 1U));
    if (fastAdaptation || modelUpdatePhase == 0U) {
      updateAdaptiveModel(currentModel, currentPhi,
                     toQ14(deltaCurrent, MODEL_I_SCALE_CENTI), fastAdaptation);
      updateAdaptiveModel(voltageModel, voltagePhi,
                     toQ14(deltaVoltage, MODEL_V_SCALE_CENTI), fastAdaptation);
    }
  }

  const bool knownControlExcitation =
      identificationProbeActive || previousAuxiliaryProbeActive;
#if MPPT_ENABLED
  const bool mpptOwnActuatorResponse = mpptSettleTicks != 0U;
#else
  const bool mpptOwnActuatorResponse = false;
#endif
  const bool identificationTransient = !knownControlExcitation &&
      !mpptOwnActuatorResponse &&
      (absI32(deltaCurrent) >= CURRENT_STEP_TRANSIENT_CENTI ||
       currentModel.residualAbsQ14 >= MODEL_RESIDUAL_TRANSIENT_Q14 ||
       voltageModel.residualAbsQ14 >= MODEL_RESIDUAL_TRANSIENT_Q14);
  const bool sourceTransient = !mpptOwnActuatorResponse &&
      (absI32(deltaSource) >= SOURCE_STEP_TRANSIENT_CENTI ||
       sourceCurrentStepDetected(deltaSourceCurrent));
  if (identificationTransient || sourceTransient) transientHoldCounter = 24U;
  else if (transientHoldCounter > 0U) --transientHoldCounter;

#if MPPT_ENABLED
  if (++efficiencyMeasurementDivider >= 8U) {
    efficiencyMeasurementDivider = 0U;
    const bool stable = !identificationTransient && !sourceTransient &&
        transientHoldCounter == 0U && !knownControlExcitation &&
        absI32(voltageError) <= EFFICIENCY_STABLE_VERROR_CENTI &&
        absI32(deltaCurrent) <= EFFICIENCY_STABLE_DCURRENT_CENTI &&
        absI32(deltaSource) <= EFFICIENCY_STABLE_DSOURCE_CENTI &&
        currentInstant < runtimeCurrentLimitCenti * OUTPUT_CURRENT_BOUND_PERCENT / 100L &&
        sourceCurrentInstant < runtimeInputCurrentLimitCenti * INPUT_CURRENT_GUARD_START_PERCENT / 100L;
    if (stable) updateMeasuredEfficiency(sourceCenti, sourceCurrentCenti,
                                         voltageCenti, currentCenti);
  }
  const bool mpptNearGuard =
      currentInstant >= runtimeCurrentLimitCenti * OUTPUT_CURRENT_BOUND_PERCENT / 100L ||
      sourceCurrentInstant >= runtimeInputCurrentLimitCenti * INPUT_CURRENT_GUARD_START_PERCENT / 100L;
  updateMppt(sourceCenti, sourceCurrentCenti, voltageCenti,
             voltageError, transientHoldCounter > 0U, mpptNearGuard);
#endif
#if AUX_EFFICIENCY_LEARNING_ENABLED
  recordEfficiencyMeasurement(sourceCenti, sourceCurrentCenti,
      voltageCenti, currentCenti, voltageError, deltaCurrent, deltaSource,
      knownControlExcitation,
      currentInstant >= runtimeCurrentLimitCenti * OUTPUT_CURRENT_BOUND_PERCENT / 100L ||
      sourceCurrentInstant >= runtimeInputCurrentLimitCenti * INPUT_CURRENT_GUARD_START_PERCENT / 100L);
#endif

  const AdaptiveModel *currentPlant = selectModel(currentModel, 1U, knownControlExcitation);
  const AdaptiveModel *voltagePlant = selectModel(voltageModel, 1U, knownControlExcitation);
  updateLoadEstimate(voltagePlant);
  const bool auxiliaryModelReady = modelHasBothCurrentActuators(currentModel) &&
                                   auxiliaryIdentificationBaseReady;
  updateAuxiliaryLowPowerState(currentInstant, voltageError);
  if (loadEstimateValid && estimatedLoadCurrentCenti > AUX_LOW_POWER_MAX_LOAD_CENTI)
    auxiliaryLowPowerActive = false;
  const bool auxPreferred = auxiliaryEfficiencyPreferred(currentCenti, sourceCenti);
  primaryEquivalentDutyDemandBp = appliedDutyBp;
#if AUX_PATH_ARMED
  updateAuxiliaryPathMode(currentInstant, auxiliaryModelReady, auxPreferred);
#endif

  const ControlRegime regime = previousMeasurementsValid
      ? classifyControlRegime(voltageError, currentInstant, deltaCurrent,
          deltaSource, deltaSourceCurrent, currentPlant, voltagePlant,
          knownControlExcitation)
      : REGIME_NORMAL;

  // Select exactly one cascade: the learned inverse when both models are
  // confident, otherwise the conventional PI bootstrap. The PI state is
  // synchronized below for a bumpless fallback.
  int32_t currentReference = commandedCurrentReferenceCenti;
  int32_t selectedDuty = appliedDutyBp;
  int32_t selectedAuxiliaryDuty = limitSlew(0L, appliedAuxiliaryDutyBp,
                                            AUX_DISABLE_SLEW_BP);

  const bool modelControllerActive = currentPlant != 0 && voltagePlant != 0 &&
      currentPlant->thetaQ14[1] > MODEL_MIN_GAIN_Q14 &&
      voltagePlant->thetaQ14[1] > MODEL_MIN_GAIN_Q14;
  int32_t modelCurrentResidualQ14 = 0L;
  int32_t dutySlew = 40L;
  int32_t currentSlew = 30L;
  int32_t alphaV = 819L;
  int32_t alphaI = 3277L;
  int32_t dampingV = 819L;
  int32_t dampingI = 1638L;
  if (regime == REGIME_FAST) {
    alphaV = 1229L; alphaI = 4915L;
    dutySlew = 80L; currentSlew = 50L;
  } else if (regime == REGIME_DAMPED) {
    alphaV = 410L; alphaI = 1638L;
    dampingV = 4915L; dampingI = 3277L;
    dutySlew = 20L; currentSlew = 15L;
  } else if (regime == REGIME_CURRENT_BOUND) {
    alphaV = 328L; alphaI = 1311L;
    dampingV = 3277L; dampingI = 2458L;
    dutySlew = 10L; currentSlew = 10L;
  }

  if (modelControllerActive) {
    const int32_t predictedDvBaseQ14 = predictVoltageBaseQ14(
        *voltagePlant, deltaVoltage, deltaSource, 1000U);
    const int32_t predictedDvBase = fromQ14(
        predictedDvBaseQ14, MODEL_V_SCALE_CENTI);
    int32_t desiredDv = ((alphaV * voltageError) >> 14) -
                        ((dampingV * predictedDvBase) >> 14);
    desiredDv = clampI32(desiredDv, -MAX_PREDICTED_DV_CENTI,
                         MAX_PREDICTED_DV_CENTI);
    const int32_t voltageResidualQ14 =
        toQ14(desiredDv, MODEL_V_SCALE_CENTI) - predictedDvBaseQ14;
    const int32_t requestedCurrent = clampI32(
        currentMeanCenti + (voltageResidualQ14 * MODEL_CURRENT_DEVIATION_CENTI) /
                           voltagePlant->thetaQ14[1],
        0L, controlOutputCurrentLimitCenti());
    currentReference = limitSlew(requestedCurrent,
        commandedCurrentReferenceCenti, currentSlew);
    currentReference = clampI32(currentReference, 0L, controlOutputCurrentLimitCenti());
    commandedCurrentReferenceCenti = currentReference;

    int32_t desiredDi = ((alphaI * (currentReference - currentCenti)) >> 14) -
                        ((dampingI * deltaCurrent) >> 14);
    desiredDi = clampI32(desiredDi, -100L, 100L);
    const int32_t desiredDiQ14 = toQ14(desiredDi, MODEL_I_SCALE_CENTI);
    const int32_t predictedDiBaseQ14 = predictCurrentNoDutyQ14(
        *currentPlant, deltaCurrent, deltaVoltage, deltaSource,
        previousDutyLevelBp, 1000U);
    modelCurrentResidualQ14 = desiredDiQ14 - predictedDiBaseQ14;

    // Primary-equivalent demand is also the handover variable for the low-L path.
    const int32_t primaryDemand = clampI32(
        dutyMeanBp + modelCurrentResidualQ14 * MODEL_DUTY_SCALE_BP /
                    currentPlant->thetaQ14[1],
        0L, MAX_DUTY_BASIS_POINTS);
    primaryEquivalentDutyDemandBp = quantizePrimaryDutyBp(primaryDemand);
#if AUX_PATH_ARMED
    updateAuxiliaryPathMode(currentInstant, auxiliaryModelReady, auxPreferred);
    if (auxiliaryPathMode == AUX_MODE_AUXILIARY_ONLY && auxiliaryModelReady) {
      selectedDuty = 0L;
      const int32_t auxDemand = clampI32(
          auxiliaryDutyMeanBp + modelCurrentResidualQ14 * AUX_MODEL_DUTY_SCALE_BP /
                                currentPlant->thetaQ14[2],
          0L, AUX_MAX_DUTY_BASIS_POINTS);
      selectedAuxiliaryDuty = limitSlew(auxDemand, appliedAuxiliaryDutyBp,
                                        AUX_DUTY_SLEW_BP);
    } else if (auxiliaryPathMode == AUX_MODE_OVERLAP && auxiliaryModelReady) {
      const int32_t auxIncrementQ14 = modelCurrentResidualQ14 *
          (int32_t)AUX_CONTROL_SHARE_PERMILLE / 1000L;
      const int32_t auxDemand = clampI32(
          auxiliaryDutyMeanBp + auxIncrementQ14 * AUX_MODEL_DUTY_SCALE_BP /
                                currentPlant->thetaQ14[2],
          0L, AUX_MAX_DUTY_BASIS_POINTS);
      selectedAuxiliaryDuty = limitSlew(auxDemand, appliedAuxiliaryDutyBp,
                                        AUX_DUTY_SLEW_BP);
      const int32_t auxFeature = toQ14(
          selectedAuxiliaryDuty - auxiliaryDutyMeanBp, AUX_MODEL_DUTY_SCALE_BP);
      const int32_t auxContributionQ14 =
          ((int32_t)currentPlant->thetaQ14[2] * auxFeature) >> 14;
      const int32_t mainResidualQ14 = modelCurrentResidualQ14 - auxContributionQ14;
      const int32_t mainDemand = clampI32(
          dutyMeanBp + mainResidualQ14 * MODEL_DUTY_SCALE_BP /
                      currentPlant->thetaQ14[1],
          0L, MAX_DUTY_BASIS_POINTS);
      selectedDuty = limitSlew(mainDemand, appliedDutyBp, dutySlew);
    } else {
      selectedAuxiliaryDuty = limitSlew(0L, appliedAuxiliaryDutyBp,
                                        AUX_DISABLE_SLEW_BP);
      selectedDuty = limitSlew(primaryDemand, appliedDutyBp, dutySlew);
    }
#else
    selectedDuty = limitSlew(primaryDemand, appliedDutyBp, dutySlew);
#endif
    // Track PI integrators to model-controller outputs for bumpless fallback.
    voltageIntegratorQ16 = clampI32(
        currentReference * 65536L - voltageError * BOOT_OUTER_KP_Q8 * 256L,
        -controlOutputCurrentLimitCenti() * 65536L,
        controlOutputCurrentLimitCenti() * 65536L);
    currentIntegratorQ16 = clampI32(
        selectedDuty * 65536L - (currentReference - currentCenti) *
            BOOT_INNER_KP_Q8 * 256L,
        -MAX_DUTY_BASIS_POINTS * 65536L,
        MAX_DUTY_BASIS_POINTS * 65536L);
  } else {
    currentReference = bootstrapCurrentReference(voltageError, dtUs);
    currentReference = limitSlew(currentReference, commandedCurrentReferenceCenti,
                                 BOOT_CURRENT_SLEW_CENTI);
    currentReference = clampI32(currentReference, 0L, controlOutputCurrentLimitCenti());
    const int32_t bootstrapDutyRequest = bootstrapDuty(
        currentReference - currentCenti, dtUs);
    selectedDuty = limitSlew(bootstrapDutyRequest, appliedDutyBp,
                             BOOT_DUTY_SLEW_BP);
    primaryEquivalentDutyDemandBp = selectedDuty;
#if AUX_PATH_ARMED
    selectedAuxiliaryDuty = limitSlew(0L, appliedAuxiliaryDutyBp,
                                      AUX_DISABLE_SLEW_BP);
#endif
  }
  commandedCurrentReferenceCenti = currentReference;

  // Identify the auxiliary actuator only in the low-current, near-regulation
  // window. First match its DC duty to the primary path, then use a small PRBS.
  bool auxiliaryIdentificationActive = false;
#if AUX_PATH_ARMED
  const bool auxiliaryProbeWindow = auxiliaryLowPowerActive &&
      !auxiliaryModelReady && currentInstant <= AUX_IDENT_MAX_TOTAL_CURRENT_CENTI &&
      absI32(voltageError) <= AUX_LOW_POWER_VOLTAGE_ERROR_CENTI &&
      transientHoldCounter == 0U &&
      currentInstant < runtimeCurrentLimitCenti * OUTPUT_CURRENT_BOUND_PERCENT / 100L;
  if (auxiliaryProbeWindow) {
    selectedDuty = appliedDutyBp;
    if (!auxiliaryIdentificationBaseReady) {
      selectedAuxiliaryDuty = limitSlew(appliedDutyBp, appliedAuxiliaryDutyBp,
                                        AUX_IDENT_RAMP_BP_PER_TICK);
      if (absI32(selectedAuxiliaryDuty - appliedDutyBp) <= AUX_IDENT_BASE_TOLERANCE_BP) {
        auxiliaryIdentificationBaseReady = true;
        auxiliaryProbeDivider = 0U;
      }
    } else {
      if (++auxiliaryProbeDivider >= AUX_IDENT_PRBS_HOLD_TICKS) {
        auxiliaryProbeDivider = 0U;
        const uint8_t feedback = (uint8_t)(((auxiliaryProbeLfsr >> 6) ^
                                             (auxiliaryProbeLfsr >> 5)) & 1U);
        auxiliaryProbeLfsr = (uint8_t)((((uint16_t)auxiliaryProbeLfsr << 1U) | (uint16_t)feedback) & 0x7FU);
        if (auxiliaryProbeLfsr == 0U) auxiliaryProbeLfsr = 0x6DU;
        auxiliaryProbeSign = (auxiliaryProbeLfsr & 1U) ? 1 : -1;
      }
      const int32_t offset = (auxiliaryProbeSign > 0)
          ? AUX_IDENT_PRBS_BP : -AUX_IDENT_PRBS_BP;
      selectedAuxiliaryDuty = limitSlew(
          clampI32(appliedDutyBp + offset, 0L, AUX_MAX_DUTY_BASIS_POINTS),
          appliedAuxiliaryDutyBp, AUX_DUTY_SLEW_BP);
    }
    auxiliaryIdentificationActive = true;
  }
#endif

  // Small zero-mean primary excitation is permitted only away from all limits.
  identificationProbeActive = false;
#if IDENTIFICATION_PROBE_ENABLED
  const bool needMainExcitation = !modelHasPrimaryCurrentConfidence(currentModel) ||
                                  !modelHasInputConfidence(voltageModel);
  const bool probeWindow = needMainExcitation && !auxiliaryIdentificationActive &&
      auxiliaryPathMode == AUX_MODE_PRIMARY_ONLY && voltageError >= 0L &&
      voltageError <= IDENTIFICATION_PROBE_ERROR_BAND_CENTI &&
      currentInstant < controlOutputCurrentLimitCenti() *
                       IDENTIFICATION_PROBE_MAX_CURRENT_PERCENT / 100L &&
      voltageCenti < OUTPUT_OVERVOLTAGE_CENTI - IDENTIFICATION_PROBE_VOLTAGE_MARGIN_CENTI &&
      transientHoldCounter == 0U;
  if (probeWindow) {
    if (++probeDivider >= IDENTIFICATION_PROBE_HOLD_TICKS) {
      probeDivider = 0U;
      const uint8_t feedback = (uint8_t)(((probeLfsr >> 6) ^ (probeLfsr >> 5)) & 1U);
      probeLfsr = (uint8_t)((((uint16_t)probeLfsr << 1U) | (uint16_t)feedback) & 0x7FU);
      if (probeLfsr == 0U) probeLfsr = 0x5DU;
      probeSign = (probeLfsr & 1U) ? 1 : -1;
    }
    selectedDuty = limitSlew(selectedDuty +
        ((probeSign > 0) ? IDENTIFICATION_PROBE_BP : -IDENTIFICATION_PROBE_BP),
        appliedDutyBp, IDENTIFICATION_PROBE_SLEW_BP);
    identificationProbeActive = true;
  } else {
    probeDivider = 0U;
  }
#endif

  bool forceAuxiliaryOff = false;
  bool forceBothPwmOff = false;
  applyInputCurrentGuard(sourceCurrentInstant, selectedDuty, selectedAuxiliaryDuty);
  if (inputCurrentGuardActive) forceAuxiliaryOff = true;
  if (voltageError < -VOLTAGE_ERROR_DEADBAND_CENTI) {
    forceBothPwmOff = true;
    auxiliaryLowPowerActive = false;
    commandedCurrentReferenceCenti = 0L;
    voltageIntegratorQ16 = 0L;
    currentIntegratorQ16 = 0L;
  }
  selectedDuty = clampI32(selectedDuty, MIN_DUTY_BASIS_POINTS, MAX_DUTY_BASIS_POINTS);
  selectedAuxiliaryDuty = clampI32(selectedAuxiliaryDuty, 0L, AUX_MAX_DUTY_BASIS_POINTS);
  if (auxiliaryPathMode == AUX_MODE_PRIMARY_ONLY && !auxiliaryIdentificationActive)
    selectedAuxiliaryDuty = limitSlew(0L, appliedAuxiliaryDutyBp, AUX_DISABLE_SLEW_BP);
  if (auxiliaryPathMode == AUX_MODE_AUXILIARY_ONLY && !auxiliaryIdentificationActive)
    selectedDuty = 0L;
  enforcePowerStageInhibits(forceAuxiliaryOff, forceBothPwmOff,
                            selectedDuty, selectedAuxiliaryDuty);
  if (controlOutputCurrentLimitCenti() <= 0L) {
    selectedDuty = 0L;
    selectedAuxiliaryDuty = 0L;
  }
  writeDutyBasisPoints(selectedDuty, selectedAuxiliaryDuty);

  if (!auxiliaryLowPowerActive &&
      appliedAuxiliaryDutyBp <= AUX_IDENT_BASE_TOLERANCE_BP) {
    auxiliaryIdentificationBaseReady = false;
    auxiliaryProbeDivider = 0U;
  }
  previousAuxiliaryProbeActive = auxiliaryIdentificationActive &&
                                 appliedAuxiliaryDutyBp > 0L;
  previousDutyLevelBp = effectiveDutyLevelBp(appliedDutyBp, appliedAuxiliaryDutyBp);
  dutyMeanQ16 += (appliedDutyBp * 65536L - dutyMeanQ16) / DUTY_MEAN_TAU_SAMPLES;
  dutyMeanBp = dutyMeanQ16 / 65536L;
  previousDutyCenteredBp = appliedDutyBp - dutyMeanBp;
  auxiliaryDutyMeanQ16 += (appliedAuxiliaryDutyBp * 65536L - auxiliaryDutyMeanQ16) /
                          DUTY_MEAN_TAU_SAMPLES;
  auxiliaryDutyMeanBp = auxiliaryDutyMeanQ16 / 65536L;
  previousAuxiliaryDutyCenteredBp = appliedAuxiliaryDutyBp - auxiliaryDutyMeanBp;

  previousDeltaVoltageCenti = previousMeasurementsValid ? deltaVoltage : 0L;
  previousDeltaCurrentCenti = previousMeasurementsValid ? deltaCurrent : 0L;
  previousVoltageCenti = voltageCenti;
  previousCurrentCenti = currentCenti;
  previousCurrentCenteredCenti = currentCenti - currentMeanCenti;
  previousSourceCenti = sourceCenti;
  previousSourceCurrentCenti = sourceCurrentCenti;
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
#if SOURCE_SENSE_ENABLED
  pinMode(SOURCE_VOLTAGE_PIN, INPUT);
#endif
  Serial.begin(SERIAL_BAUD_RATE);
  analogReference(DEFAULT);
  configurePwm();
  writeDutyBasisPoints(0L, 0L);
  initialiseDefaultCalibrationScales();
  resetCalibrationAverages();

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
    Serial.println(F("EEPROM ZERO ERR"));
    printCalibrationInstructions();
  } else {
    calibrationWizardState = CAL_WIZARD_READY;
    Serial.println(F("CAL OK"));
  }

  resetAdaptiveModel(currentModel);
  resetAdaptiveModel(voltageModel);
  lastControlUs = (uint32_t)micros();
  trendWindowTicks = 0U;
  crossingWindowTicks = 0U;
}

void loop() {
  pollCalibrationSerial();

  if (faultCode != FAULT_NONE) {
    writeDutyBasisPoints(0L, 0L);
    return;
  }
  // Sample optional direct A5/A6 limit pots outside the control update.
  serviceLimitPotentiometers();

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
  if (dtUs < CONTROL_PERIOD_US) return;
  lastControlUs = nowUs;
  if (dtUs > MAX_CONTROL_DT_US) {
    latchFault(FAULT_CONTROL_DEADLINE);
    return;
  }

  controlUpdate(dtUs);
}
