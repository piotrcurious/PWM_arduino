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
 * Online-identifying cascaded buck controller with two overlapping power paths.
 *
 * RELEASE STATUS — NOT QUALIFIED FOR ATmega328P/UNO DEPLOYMENT.
 * v14 baseline measurements below were made as an AVR-target Clang 17 -Oz
 * object with full dual-path/protection configuration and mocked Arduino/EEPROM
 * headers:
 * size reports text=46,400 bytes, data=55 bytes, bss=804 bytes. The equivalent
 * v8 object under identical target flags reports text=68,908, data=51,
 * bss=1,181 bytes. v14 therefore reduces object text by 32.7% and static
 * data+bss by 30.3%. Normal-inlining v14 on the same target reports text=
 * 58,364 bytes; the default size profile saves 20.5% versus that build.
 * These are unlinked object measurements, not a final avr-gcc/Arduino-core
 * ELF. Mocked Serial print methods omit real Print/HardwareSerial code, so
 * the final flash image will be larger, not smaller.
 * These are v14 baseline figures, not a measured v15 build. The v15 source
 * revision narrows bounded RLS covariance cells and calibration-window sums,
 * removes write-only state, shortens the bounded serial command buffer, and
 * avoids compiling empirical auxiliary-loss learning when the auxiliary path
 * is disarmed. MPPT's independent measured-efficiency estimate remains on.
 * The v14 default primary-only build (auxiliary disarmed, MPPT and empirical
 * learning compiled in) reports text=39,728, data=48, bss=781 bytes. In v15,
 * empirical auxiliary-loss learning defaults off while AUX_PATH_ARMED=0;
 * MPPT's measured-efficiency estimate remains active. Those v14 profile
 * measurements are historical, not v15 build numbers. The reduced primary-only
 * v14 profile with MPPT, empirical auxiliary learning and excitation probe
 * disabled reports text=32,394 and data=16 bytes, leaving only 358 bytes of
 * the Uno's 32 KiB flash before Arduino core/startup code. No tested profile
 * is a viable Uno application image.
 * v14 full-profile static data+bss is 859 bytes before real Arduino-core
 * buffers. v15 source changes reduce the four RLS models by 80 bytes, narrow
 * the calibration accumulator's sample sums, remove write-only state, reduce
 * the serial command buffer by 16 bytes, and compile out the unusable
 * auxiliary-loss profile when the auxiliary path is disarmed. These are
 * source-level changes, not an AVR-linked measurement. The largest
 * measured individual v14 frame is 158 bytes (controlUpdate);
 * this is not a whole call-chain/ISR stack proof. No 2 ms worst-case execution
 * proof exists. Noinline saves Flash but adds call overhead; timing must be
 * measured on the final board/toolchain.
 * Do not deploy this image to an Uno. v15 has not been measured with a final
 * AVR toolchain or qualified for flash, stack, timing, thermal, or power-stage
 * limits; the v14 figures above must not be presented as v15 measurements.
 * Use a larger MCU with a ported timer backend, or produce a separately
 * qualified reduced-feature Uno build.
 *
 * ATmega328P/Uno hardware mapping (subject to the release qualification above):
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
 *   output/thermal current setting. Separate hardware protection is required
 *   for source current, load output current and peak current in each inductor
 *   branch. D2 reports a latched fault; it does not interrupt fault current.
 *   An independent output overvoltage comparator is also mandatory and must
 *   asynchronously inhibit both gate drives (D2 is status only).
 *
 * Thermal management:
 *   One 10k NTC (B=3950) on the shared heatsink progressively reduces the
 *   allowable output-current limit. An open/short sensor or overtemperature
 *   condition inhibits the power stage.
 *
 * Source MPPT:
 *   Finite-difference P-V hill-climbing with a perturb-and-observe fallback.
 *   This is not a textbook incremental-conductance implementation. The
 *   actuator is a supervisory input-power/current ceiling translated into an
 *   output-current ceiling; it does not directly command source voltage. It
 *   can reach MPP only when the downstream load/storage can accept the power.
 *   The independent input-current guard and hardware comparator always win.
 *
 * Online identification:
 *   Fast/slow RLS regressions estimate local state/actuator response and a
 *   disturbance term. They are local linear approximations, not a complete
 *   nonlinear source/load identification.
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
#ifndef POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED
#define POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED 0
#endif
#ifndef POWER_STAGE_HARDWARE_FAULT_ACTIVE_LOW
#define POWER_STAGE_HARDWARE_FAULT_ACTIVE_LOW 1
#endif
#define POWER_STAGE_HARDWARE_FAULT_PIN  2   // Uno INT0; shared fault-status input
#ifndef OUTPUT_FAST_CURRENT_PROTECTION_PRESENT
#define OUTPUT_FAST_CURRENT_PROTECTION_PRESENT 0 // Independent output/load overcurrent comparator or e-fuse disables both power paths
#endif
#ifndef OUTPUT_FAST_OVERVOLTAGE_PROTECTION_PRESENT
#define OUTPUT_FAST_OVERVOLTAGE_PROTECTION_PRESENT 0 // Independent output overvoltage comparator asynchronously inhibits both gates
#endif
#ifndef PRIMARY_FAST_CURRENT_PROTECTION_PRESENT
#define PRIMARY_FAST_CURRENT_PROTECTION_PRESENT 0 // Independent primary-inductor peak-current comparator inhibits primary switch
#endif
#ifndef INPUT_FAST_CURRENT_PROTECTION_PRESENT
#define INPUT_FAST_CURRENT_PROTECTION_PRESENT 0 // Independent source/input current comparator disables all power paths
#endif
#define THERMISTOR_PIN                  A3  // Shared heatsink 10k NTC input
#define VOLTAGE_PIN                     A0  // Output voltage
#define CURRENT_PIN                     A1  // Output current ACS712-05B
#ifndef SOURCE_SENSE_ENABLED
#define SOURCE_SENSE_ENABLED            1   // Required source-voltage measurement on A2
#endif
#ifndef SOURCE_CURRENT_SENSE_ENABLED
#define SOURCE_CURRENT_SENSE_ENABLED    1   // Required source-current ACS712-05B measurement on A4
#endif
#ifndef AUX_EFFICIENCY_LEARNING_ENABLED
#if AUX_PATH_ARMED
#define AUX_EFFICIENCY_LEARNING_ENABLED 1   // Compare measured losses only when both paths can conduct
#else
#define AUX_EFFICIENCY_LEARNING_ENABLED 0   // No auxiliary observations are possible while that path is disarmed
#endif
#endif
#if AUX_EFFICIENCY_LEARNING_ENABLED && !AUX_PATH_ARMED
#error "Auxiliary efficiency learning requires AUX_PATH_ARMED; otherwise the auxiliary profile can never be measured"
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
#define THERMISTOR_NOMINAL_OHMS         10000UL
#define THERMISTOR_PULLUP_OHMS          10000UL
#define THERMISTOR_BETA                 3950UL
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
#define EEPROM_CAL_VERSION              7U
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

// D4 must be LOW only when input and output currents are physically zero and isolated.
#define ACS712_ZERO_INTERLOCK_PIN            4
#define ACS712_ZERO_INTERLOCK_ACTIVE_LOW     1
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
#ifndef OUTPUT_HARD_OVERCURRENT_CENTI
#define OUTPUT_HARD_OVERCURRENT_CENTI   450L // fixed software trip, separate from adjustable setpoint
#endif
#ifndef INPUT_HARD_OVERCURRENT_CENTI
#define INPUT_HARD_OVERCURRENT_CENTI    450L // fixed software trip, separate from adjustable setpoint
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
#ifndef MPPT_INITIAL_STEP_CENTI
#define MPPT_INITIAL_STEP_CENTI                  10U      // initial perturbation in centiamps
#endif
#ifndef MPPT_MIN_STEP_CENTI
#define MPPT_MIN_STEP_CENTI                      10U      // minimum perturbation; several ACS712 ADC counts
#endif
#ifndef MPPT_MAX_STEP_CENTI
#define MPPT_MAX_STEP_CENTI                      20U      // maximum perturbation in centiamps
#endif
#ifndef MPPT_SETTLE_MS
#define MPPT_SETTLE_MS                           250UL    // settling time after changing the input-current ceiling
#endif
#ifndef MPPT_RESCAN_INTERVAL_MS
#define MPPT_RESCAN_INTERVAL_MS                  2000UL   // periodic re-probe interval after reaching a quiet local maximum
#endif
#ifndef MPPT_COMMAND_RESPONSE_IGNORE_MS
#define MPPT_COMMAND_RESPONSE_IGNORE_MS         75UL     // ignore known actuator-caused response while it settles
#endif
#ifndef MPPT_SAMPLE_COUNT
#define MPPT_SAMPLE_COUNT                        16U      // samples in each source power window
#endif
#ifndef MPPT_MAX_WINDOW_VOLTAGE_SPAN_CENTI
#define MPPT_MAX_WINDOW_VOLTAGE_SPAN_CENTI       20L      // reject windows moving by >0.20 V
#endif
#ifndef MPPT_MAX_WINDOW_CURRENT_SPAN_CENTI
#define MPPT_MAX_WINDOW_CURRENT_SPAN_CENTI       18L      // reject windows moving by >0.18 A
#endif
#define SOURCE_CURRENT_STEP_TRANSIENT_CENTI      20L      // input-current step; avoids learning across irradiance/source changes
#ifndef MPPT_MIN_SOURCE_VOLTAGE_CENTI
#define MPPT_MIN_SOURCE_VOLTAGE_CENTI            500L     // below 5 V: source is absent or collapsed
#endif
#ifndef MPPT_MIN_SOURCE_CURRENT_CENTI
#define MPPT_MIN_SOURCE_CURRENT_CENTI            5L       // below 50 mA after zero calibration: likely disconnected
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
#ifndef MPPT_CURRENT_NOISE_FLOOR_CENTI
#define MPPT_CURRENT_NOISE_FLOOR_CENTI           3L       // approx. one ACS712-05B ADC count
#endif
#ifndef MPPT_SOURCE_VOLTAGE_NOISE_FLOOR_CENTI
#define MPPT_SOURCE_VOLTAGE_NOISE_FLOOR_CENTI    5L       // conservative source divider noise floor
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
// Four local regressors are sufficient for the present incremental plant:
// current lag, primary duty, auxiliary duty, and source/output disturbance.
// The previous second current-lag term consumed RAM but added little robust
// identifiability at this controller's 499 Hz sample rate.
#define MODEL_FEATURE_COUNT              4U
#define RLS_COVARIANCE_COUNT             ((MODEL_FEATURE_COUNT * (MODEL_FEATURE_COUNT + 1U)) / 2U)
// Q12 covariance preserves sub-count precision. Four regressors use a
// packed 4x4 symmetric matrix (10 cells versus the former 5x5/25 cells).
// The bounded 4.0 maximum keeps all update products within signed 32-bit range.
#define RLS_P_INITIAL_Q12               16384L // 4.0
#define RLS_P_MAX_Q12                   16384L // 4.0
#define RLS_P_MIN_Q12                   16L
#define RLS_VALID_P_Q12                 8192L // 2.0
#define RLS_LAMBDA_FAST_Q12             3994U
#define RLS_LAMBDA_SLOW_Q12             4076U
#define RLS_FAST_RECIP_Q15              33554U
#define RLS_SLOW_RECIP_Q15              32929U
#ifndef SLOW_RLS_UPDATE_DIVIDER
#define SLOW_RLS_UPDATE_DIVIDER          8U
#endif
#define RLS_MIN_EXCITATION_Q14          96L
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
#define COST_CURRENT_WEIGHT             4L
#define CANDIDATE_HISTORY_COST_WEIGHT   1000L
#define AUXILIARY_DUTY_COST_WEIGHT       2L
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
#if (SOURCE_OVERVOLTAGE_CENTI * INPUT_HARD_OVERCURRENT_CENTI) > 2000000000L
#error "Source power product exceeds signed 32-bit power arithmetic"
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
// ADC rail rejection bounds raw source voltage to 1017 counts. The stored
// source scale is uint16_t (max 65535 Q8 cV/count), and ACS conversion clamps
// input current to 600 cA. Thus Psample <= 260348*600 = 156208800; 27 samples
// plus rounding and a 27-permille deadband multiplier remain below UINT32_MAX.
#if MPPT_ENABLED && (MPPT_SAMPLE_COUNT < 4U || MPPT_SAMPLE_COUNT > 27U)
#error "MPPT_SAMPLE_COUNT must be 4..27 to retain a useful window without overflowing 32-bit power sums"
#endif
#if MPPT_ENABLED && MPPT_POWER_DEADBAND_PERMILLE > 27U
#error "MPPT power deadband multiplier exceeds the proven 32-bit power-product bound"
#endif
#if MPPT_ENABLED && (MPPT_MIN_STEP_CENTI == 0U || MPPT_MIN_STEP_CENTI > MPPT_INITIAL_STEP_CENTI || MPPT_INITIAL_STEP_CENTI > MPPT_MAX_STEP_CENTI)
#error "MPPT step sizes must satisfy 0 < minimum <= initial <= maximum"
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
#if SLOW_RLS_UPDATE_DIVIDER == 0U || SLOW_RLS_UPDATE_DIVIDER > 256U || (SLOW_RLS_UPDATE_DIVIDER & (SLOW_RLS_UPDATE_DIVIDER - 1U)) != 0U
#error "SLOW_RLS_UPDATE_DIVIDER must be a power of two from 1 through 256 (slowRlsPhase is uint8_t)"
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
#if !OUTPUT_FAST_CURRENT_PROTECTION_PRESENT
#error "Output/load protection requires an independent hardware overcurrent comparator or e-fuse that disables both power paths"
#endif
#if !OUTPUT_FAST_OVERVOLTAGE_PROTECTION_PRESENT
#error "Output/load protection requires an independent hardware overvoltage comparator that asynchronously inhibits both gate drives"
#endif
#if !PRIMARY_FAST_CURRENT_PROTECTION_PRESENT
#error "Primary buck branch requires an independent inductor peak-current comparator that inhibits its MOSFET"
#endif
#if !INPUT_FAST_CURRENT_PROTECTION_PRESENT
#error "Input/source protection requires an independent source-current comparator that disables all power paths"
#endif
#if !POWER_STAGE_HARDWARE_FAULT_STATUS_ENABLED
#error "PWM requires D2 fault status; connect the hardware protection outputs and independent output overvoltage trip to the latch/status network"
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
  FAULT_HARDWARE_TRIP,
  FAULT_THERMISTOR,
  FAULT_OVERTEMPERATURE,
  FAULT_EEPROM,
  FAULT_VOLTAGE_NO_RESPONSE
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
  // Symmetric Q12 covariance, packed lower triangle.
  int16_t covarianceQ12[RLS_COVARIANCE_COUNT];
  int16_t disturbanceQ14;
  int16_t residualAbsQ14;
  // Confidence only needs to distinguish 0..255; saturate instead of storing
  // 16-bit counters for every regressor in every fast/slow model.
  uint8_t featureExcitationCount[MODEL_FEATURE_COUNT];
};
static_assert(RLS_P_MIN_Q12 >= 0L &&
              RLS_P_INITIAL_Q12 >= RLS_P_MIN_Q12 &&
              RLS_P_INITIAL_Q12 <= RLS_P_MAX_Q12 &&
              RLS_P_MAX_Q12 <= INT16_MAX,
              "RLS covariance bounds must fit signed 16-bit cells");
static_assert(sizeof(RlsModel) <= 36U, "RLS model RAM budget exceeded");

struct DualModel {
  RlsModel fast;
  RlsModel slow;
};

struct CandidateResult {
  int32_t cost;
  uint16_t currentReferenceCenti;
  uint16_t dutyBasisPoints;
  uint16_t primaryEquivalentDutyBp; // duty the primary path would need on its own
  uint16_t auxiliaryDutyBasisPoints;
  int16_t predictedVoltageDeltaCenti;
  int16_t predictedNextVoltageDeltaCenti;
  int16_t predictedCurrentDeltaCenti;
  uint8_t candidateIndex;
  uint8_t valid;
};
static_assert(sizeof(CandidateResult) <= 20U,
              "Candidate result must fit the compact 20-byte RAM budget");

struct CandidateOutcome {
  uint8_t predictionErrorQ4; // mean absolute error in Q4 units, saturating at ~16 Q8 units
  uint8_t observations;
};
static_assert(sizeof(CandidateOutcome) == 2U,
              "Candidate history must remain a two-byte RAM record");

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

enum AuxiliaryPathMode : uint8_t {
  AUX_MODE_PRIMARY_ONLY = 0,
  AUX_MODE_OVERLAP,
  AUX_MODE_AUXILIARY_ONLY
};

enum MpptState : uint8_t {
  MPPT_STATE_DISABLED = 0,
  MPPT_STATE_SETTLING,
  MPPT_STATE_TRACKING,
  MPPT_STATE_LOAD_LIMITED,
  MPPT_STATE_HEADROOM_RECOVERY,
  MPPT_STATE_NO_SOURCE,
  MPPT_STATE_SOURCE_RECOVERY,
  MPPT_STATE_NO_HEADROOM
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
static FW_NOINLINE void powerStageHardwareFaultISR() {
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
static int32_t latestOutputCurrentCenti = 0L;
static int32_t latestInputCurrentCenti = 0L;
static uint8_t efficiencyMeasurementDivider = 0U;
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
    (uint32_t)ACS712_ZERO_INTERLOCK_PIN,
    (uint32_t)ACS712_ZERO_INTERLOCK_ACTIVE_LOW,
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
static MpptState mpptState = MPPT_STATE_DISABLED;
static bool mpptInitialised = false;
static bool mpptBaselineValid = false;
static bool mpptLimitActive = false;
static int32_t mpptInputCurrentTargetCenti = 0L;
static int32_t mpptOutputCurrentLimitCenti = OUTPUT_CURRENT_LIMIT_CENTI;
static uint32_t mpptCommandResponseUntilMs = 0UL;
static uint16_t mpptEfficiencyEstimatePermille =
    (uint16_t)MPPT_ASSUMED_EFFICIENCY_PERMILLE;
static bool mpptEfficiencyEstimateValid = false;
static int32_t mpptPreviousSourceVoltageCenti = 0L;
static uint32_t mpptPreviousPowerProduct = 0UL;
static uint32_t mpptPowerSum = 0UL;
static uint32_t mpptVoltageSum = 0UL;
static uint32_t mpptCurrentSum = 0UL;
static int32_t mpptWindowMinVoltageCenti = INT32_MAX;
static int32_t mpptWindowMaxVoltageCenti = INT32_MIN;
static int32_t mpptWindowMinCurrentCenti = INT32_MAX;
static int32_t mpptWindowMaxCurrentCenti = INT32_MIN;
static uint32_t mpptSettleUntilMs = 0UL;
static uint8_t mpptSampleCount = 0U;
static uint8_t mpptStepCenti = MPPT_INITIAL_STEP_CENTI;
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

static FW_NOINLINE bool saveCalibrationToEeprom(const CalibrationRecord &candidateData) {
  if ((uint32_t)EEPROM.length() <
      (uint32_t)EEPROM_CAL_SLOT_COUNT * sizeof(CalibrationRecord)) return false;
  CalibrationRecord record = candidateData;
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
  if (!calibrationRecordValid(record)) return false;

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

  // Reuse the same 50-byte stack record for read-back. This removes a second
  // local CalibrationRecord and keeps peak stack usage bounded.
  EEPROM.get(base, record);
  if (!calibrationRecordValid(record)) return false;
  applyCalibrationRecord(record, targetSlot);
  return true;
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

static FW_NOINLINE bool zeroCalibrationInterlockActive() {
#if ACS712_ZERO_INTERLOCK_ACTIVE_LOW
  return digitalRead(ACS712_ZERO_INTERLOCK_PIN) == LOW;
#else
  return digitalRead(ACS712_ZERO_INTERLOCK_PIN) == HIGH;
#endif
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
    if ((sample & 0x0FU) == 0U && !zeroCalibrationInterlockActive())
      return false;
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
  if (!zeroCalibrationInterlockActive()) return false;

  const int32_t meanOffsetQ8 =
      (offsetSum * 256) / (int32_t)n;
  zeroRawQ8 = (int32_t)(baseRaw * 256 + meanOffsetQ8);
  spanRaw = (uint8_t)(maximum - minimum);
  return zeroRawQ8 >= (int32_t)ACS712_ZERO_MIN_RAW * 256L &&
         zeroRawQ8 <= (int32_t)ACS712_ZERO_MAX_RAW * 256L;
}

static FW_NOINLINE bool zeroOffsetPlausibleOnPin(uint8_t pin, int32_t expectedZeroRawQ8) {
  (void)analogRead(pin); // discard after ADC mux selection
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

static FW_NOINLINE void resetDualModel(DualModel &model);

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
  Serial.println(F("CALIBRATION REQUIRED: PWM outputs are disabled."));
  Serial.println(F("1) Isolate BOTH source input and output; verify both ACS712 conductors carry zero current."));
  Serial.println(F("2) Assert D4 isolation interlock, send ZERO; 128 samples follow a 250 ms settle."));
  Serial.println(F("3) Connect a measured 10.00-ohm, 1% low-tempco resistor rated >=50 W."));
  Serial.println(F("4) Release D4 and send LOAD. Calibration runs primary path only."));
  Serial.println(F("5) At stable load, measure Vout, Vin and source Iin with DMMs."));
  Serial.println(F("6) Send: V=1200 VIN=2000 IIN=84 (centivolts / centiamps)."));
  Serial.println(F("Verify the physical voltage dividers before calibration: output is 4:1 and source is 5:1 by default."));
  Serial.println(F("If either physical ratio differs, change the matching *_DIVIDER_NUM/DEN macros before calibration."));
  Serial.println(F("Default 5:1 source divider accepts <=22.0 V at calibration; for higher source Voc, increase the physical divider ratio and matching SOURCE_DIVIDER_NUM before use."));
  Serial.println(F("The known 10-ohm load derives Iout; the input DMM calibrates input ACS712 gain."));
  Serial.println(F("Calibration checks nominal ACS712 response (12% cross-check), divider ratios, stable sample statistics and Pin/Pout plausibility."));
  Serial.println(F("Output/input current pots remain active during calibration; test aborts safely at 85% of either configured current limit."));
  Serial.println(F("Keep the input-current DMM in series at the converter source input."));
  Serial.println(F("D4 must remain deasserted while LOAD is active; asserting it aborts the test. D4 only qualifies calibration and is not the runtime gate shutdown."));
  Serial.println(F("Complete LOAD/DMM stage within 30 s. EEPROM uses redundant CRC-checked slots."));
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
static FW_NOINLINE void calibrationPrint(Message message, uint8_t bytesRequired);

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
        calibrationPrint(F("Stable ADC window; enter V= VIN= IIN= (cV/cA)."), 48U);
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

// Strict, bounded decimal parsing avoids strtol/libc code and rejects signs,
// trailing junk and overflow before performing the multiply-by-ten step.
static FW_NOINLINE bool parseBoundedDecimal(const char *text, int32_t minimum,
                                int32_t maximum, int32_t &value) {
  if (text == 0 || *text == '\0' || minimum < 0L || maximum < minimum)
    return false;
  uint32_t parsed = 0UL;
  const uint32_t upper = (uint32_t)maximum;
  while (*text != '\0') {
    if (*text < '0' || *text > '9') return false;
    const uint8_t digit = (uint8_t)(*text - '0');
    if (parsed > upper / 10UL ||
        (parsed == upper / 10UL && digit > upper % 10UL)) return false;
    parsed = (uint32_t)(parsed * (uint32_t)10U + (uint32_t)digit);
    ++text;
  }
  if (parsed < (uint32_t)minimum || parsed > upper) return false;
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

static FW_NOINLINE bool hasPrefix(const char *text, const char *prefix) {
  if (text == 0 || prefix == 0) return false;
  while (*prefix != '\0') {
    if (*text == '\0' || *text != *prefix) return false;
    ++text;
    ++prefix;
  }
  return true;
}

static FW_NOINLINE bool containsEquals(const char *text) {
  if (text == 0) return false;
  while (*text != '\0') {
    if (*text++ == '=') return true;
  }
  return false;
}

static FW_NOINLINE bool parsePositiveCentivolts(const char *text, int32_t &value) {
  return parseBoundedDecimal(text,
      CAL_TARGET_V_CENTI - CAL_DMM_VOLTAGE_TOLERANCE_CENTI,
      CAL_TARGET_V_CENTI + CAL_DMM_VOLTAGE_TOLERANCE_CENTI, value);
}

#if SOURCE_SENSE_ENABLED
static FW_NOINLINE bool parseSourceCentivolts(const char *text, int32_t &value) {
  return parseBoundedDecimal(text, 100L,
      EFFICIENCY_MAX_SOURCE_VOLTAGE_CENTI, value);
}
#endif

#if MPPT_ENABLED
static FW_NOINLINE void resetMpptSampleWindow();
#endif

static FW_NOINLINE bool parseCalibrationDmmCommand(char *command,
                                       int32_t &voutCenti,
                                       int32_t &vinCenti,
                                       int32_t &iinCenti) {
  bool haveVout = false;
  bool haveVin = false;
  bool haveIin = false;
  char *cursor = command;
  while (*cursor != '\0') {
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == '\0') break;
    char *token = cursor;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') ++cursor;
    if (*cursor != '\0') *cursor++ = '\0';
    if (hasPrefix(token, "V=")) {
      if (haveVout || !parsePositiveCentivolts(token + 2, voutCenti)) return false;
      haveVout = true;
    } else if (hasPrefix(token, "VIN=")) {
      if (haveVin || !parseSourceCentivolts(token + 4, vinCenti)) return false;
      haveVin = true;
    } else if (hasPrefix(token, "IIN=")) {
      if (haveIin || !parseBoundedDecimal(token + 4, 20L,
          INPUT_HARD_OVERCURRENT_CENTI, iinCenti)) return false;
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
  currentGuardActive = false;
  auxiliaryPathMode = AUX_MODE_PRIMARY_ONLY;
  auxiliaryLowPowerActive = false;
  auxiliaryIdentificationBaseReady = false;
  identificationProbeActive = false;
  previousAuxiliaryProbeActive = false;
  probeDivider = 0U;
  probeLfsr = 0x5DU;
  probeSign = 1;
  slowRlsPhase = 0U;
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
  loadEstimateDeltaCenti = 0L;
  loadEstimateInitialized = false;
  loadEstimateValid = false;
  trendInitialized = false;
  trendWindowStartUs = 0UL;
  voltageSlopeCentiPerSec = 0L;
  crossingWindowStartUs = 0UL;
  crossingCount = 0U;
  transientHoldCounter = 0U;
  nonlinearResidualCounter = 0U;
  previousErrorSign = 0;
  activePattern = PATTERN_BOOTSTRAP;
  activeCandidateIndex = 1U;
  activeResolutionStrategy = RESOLUTION_BOOTSTRAP;
  candidateSearchCountdown = 0U;
  previousResolutionStrategy = 255U;
  selectedCandidateAvailable = false;
  candidateOutcomePending = false;
  memset(candidateOutcomes, 0, sizeof(candidateOutcomes));
  latestOutputCurrentCenti = 0L;
  latestInputCurrentCenti = 0L;
  efficiencyMeasurementDivider = 0U;
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
  resetDualModel(currentModel);
  resetDualModel(voltageModel);
#if MPPT_ENABLED
  mpptState = MPPT_STATE_DISABLED;
  mpptInitialised = false;
  mpptBaselineValid = false;
  mpptLimitActive = false;
  mpptInputCurrentTargetCenti = 0L;
  mpptOutputCurrentLimitCenti = outputCurrentSettingCenti;
  mpptCommandResponseUntilMs = 0UL;
  mpptEfficiencyEstimatePermille = (uint16_t)MPPT_ASSUMED_EFFICIENCY_PERMILLE;
  mpptEfficiencyEstimateValid = false;
  mpptPreviousSourceVoltageCenti = 0L;
  mpptPreviousPowerProduct = 0UL;
  mpptSettleUntilMs = 0UL;
  mpptSampleCount = 0U;
  mpptStepCenti = MPPT_INITIAL_STEP_CENTI;
  mpptDirection = 1;
  resetMpptSampleWindow();
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
static FW_NOINLINE void calibrationPrint(Message message, uint8_t bytesRequired) {
  const bool powerMayBeActive =
      currentSensorsCalibrated ||
      calibrationWizardState == CAL_WIZARD_RAMP ||
      calibrationWizardState == CAL_WIZARD_WAIT_DMM;
  if (powerMayBeActive && Serial.availableForWrite() < bytesRequired)
    return;
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
    // Keep the entire response below the Uno HardwareSerial TX ring capacity.
    // When the power stage may be active, queue only if the whole line fits.
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
    calibrationPrint(F("Calibration aborted; output off. Correct setup, send LOAD."), 60U);
    return;
  }

  if (calibrationWizardState == CAL_WIZARD_WAIT_ZERO &&
      equalsLiteral(command, "ZERO")) {
    writeDutyBasisPoints(0L, 0L);
    if (!zeroCalibrationInterlockActive()) {
      calibrationPrint(F("ZERO rejected: assert D4 isolation interlock."), 47U);
      return;
    }
    calibrationOutputZeroSpanRaw = 0U;
    calibrationInputZeroSpanRaw = 0U;
    calibrationStateStartMs = (uint32_t)millis();
    calibrationWizardState = CAL_WIZARD_ZERO_SETTLE;
    calibrationPrint(F("Zero settling; keep D4 asserted 250 ms."), 41U);
    return;
  }

  if (calibrationWizardState == CAL_WIZARD_WAIT_LOAD &&
      equalsLiteral(command, "LOAD")) {
    if (zeroCalibrationInterlockActive()) {
      calibrationPrint(F("LOAD rejected: release D4 first."), 34U);
      return;
    }
    if (!calibrationLoadSettingAllowsRun()) {
      calibrationPrint(F("LOAD rejected: output limit too low for 10-ohm test."), 54U);
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
    calibrationPrint(F("Calibration load started; primary path only."), 48U);
    lastControlUs = (uint32_t)micros();
    return;
  }

  if (calibrationWizardState == CAL_WIZARD_WAIT_DMM &&
      containsEquals(command)) {
    int32_t measuredVoltageCenti = 0L;
    int32_t measuredSourceCenti = 0L;
    int32_t measuredInputCurrentCenti = 0L;
    if (!parseCalibrationDmmCommand(command, measuredVoltageCenti,
                                    measuredSourceCenti,
                                    measuredInputCurrentCenti)) {
      calibrationPrint(F("Bad DMM fields; use V=1200 VIN=2000 IIN=84."), 45U);
      return;
    }
    if (!calibrationSnapshot.valid ||
        (uint32_t)((uint32_t)millis() - calibrationSnapshot.capturedAtMs) >
            CAL_SNAPSHOT_MAX_AGE_MS) {
      calibrationPrint(F("No fresh stable ADC window; wait and retry."), 45U);
      return;
    }

    const uint32_t meanVoltageRaw = calibrationSnapshot.meanRaw[0];
    const uint32_t meanCurrentRaw = calibrationSnapshot.meanRaw[1];
    const uint32_t meanSourceRaw = calibrationSnapshot.meanRaw[2];
    const uint32_t meanInputCurrentRaw = calibrationSnapshot.meanRaw[3];
    if (meanVoltageRaw < 100U || meanVoltageRaw > 900U ||
        meanSourceRaw < 50U || meanSourceRaw > 900U ||
        meanCurrentRaw <= 5U || meanCurrentRaw >= ADC_RAIL_TRIP_RAW ||
        meanInputCurrentRaw <= 5U || meanInputCurrentRaw >= ADC_RAIL_TRIP_RAW) {
      calibrationPrint(F("Rejected: ADC mean outside range."), 35U);
      return;
    }

    if (absI32(measuredVoltageCenti - CAL_TARGET_V_CENTI) >
        CAL_DMM_VOLTAGE_TOLERANCE_CENTI) {
      calibrationPrint(F("Rejected: DMM Vout is not near 12 V."), 38U);
      return;
    }
    if ((measuredSourceCenti * 100L) <
        (measuredVoltageCenti * CAL_MIN_SOURCE_HEADROOM_PERCENT)) {
      calibrationPrint(F("Rejected: source voltage lacks buck headroom."), 47U);
      return;
    }

    // Reject a wrong divider ratio or a DMM transcription error before
    // allowing it to rewrite the persistent ADC scale.
    const int32_t nominalOutputVoltage = (int32_t)(
        ((uint32_t)meanVoltageRaw *
         nominalVoltageScaleQ8(VOLTAGE_DIVIDER_NUM, VOLTAGE_DIVIDER_DEN) +
         128UL) >> 8);
    const int32_t nominalInputVoltage = (int32_t)(
        ((uint32_t)meanSourceRaw *
         nominalVoltageScaleQ8(SOURCE_DIVIDER_NUM, SOURCE_DIVIDER_DEN) +
         128UL) >> 8);
    if (!withinPercentOf(measuredVoltageCenti, nominalOutputVoltage,
                         (uint8_t)CAL_DIVIDER_SCALE_TOL_PERCENT) ||
        !withinPercentOf(measuredSourceCenti, nominalInputVoltage,
                         (uint8_t)CAL_DIVIDER_SCALE_TOL_PERCENT)) {
      calibrationPrint(F("Rejected: DMM voltage/divider ratio mismatch."), 47U);
      return;
    }

    const int32_t expectedOutputCurrentCenti = (int32_t)(
        ((int32_t)measuredVoltageCenti * 1000) /
        (int32_t)CAL_LOAD_RESISTANCE_MILLIOHM);
    if (expectedOutputCurrentCenti < 40L ||
        expectedOutputCurrentCenti >= OUTPUT_HARD_OVERCURRENT_CENTI) {
      calibrationPrint(F("Rejected: 10-ohm load current outside range."), 46U);
      return;
    }

    // A nominal-sensitivity check catches an absent, wrong-value or wrongly
    // placed 10-ohm load. The final gain is still calibrated from the resistor.
    const int32_t nominalOutputCurrentCenti = absI32(
        adcToAcs712CurrentCenti((uint16_t)meanCurrentRaw,
            primaryCurrentZeroRawQ8,
            (uint16_t)ACS712_PRIMARY_SENSITIVITY_MV_PER_A, 1));
    const int32_t nominalInputCurrentCenti = absI32(
        adcToAcs712CurrentCenti((uint16_t)meanInputCurrentRaw,
            sourceCurrentZeroRawQ8,
            (uint16_t)ACS712_SOURCE_SENSITIVITY_MV_PER_A, 1));
    if (!withinPercentOf(nominalOutputCurrentCenti,
                         expectedOutputCurrentCenti,
                         (uint8_t)CAL_CURRENT_CROSSCHECK_PERCENT)) {
      calibrationPrint(F("Rejected: output ACS712 disagrees with 10-ohm load."), 53U);
      return;
    }
    if (!withinPercentOf(nominalInputCurrentCenti,
                         measuredInputCurrentCenti,
                         (uint8_t)CAL_CURRENT_CROSSCHECK_PERCENT)) {
      calibrationPrint(F("Rejected: input ACS712 disagrees with DMM IIN."), 48U);
      return;
    }

    if (!calibrationPowerPlausible(measuredSourceCenti,
                                   measuredInputCurrentCenti,
                                   measuredVoltageCenti,
                                   expectedOutputCurrentCenti)) {
      calibrationPrint(F("Rejected: Pin/Pout implausible; check DMM readings."), 53U);
      return;
    }

    const int32_t deltaOutputCurrentRawQ8 = (int32_t)(
        (int32_t)meanCurrentRaw * 256 - primaryCurrentZeroRawQ8);
    const int32_t deltaInputCurrentRawQ8 = (int32_t)(
        (int32_t)meanInputCurrentRaw * 256 - sourceCurrentZeroRawQ8);
    if (absI32(deltaOutputCurrentRawQ8) < 2048L ||
        absI32(deltaInputCurrentRawQ8) < 2048L) {
      calibrationPrint(F("Rejected: ACS712 signal too close to zero."), 44U);
      return;
    }

    CalibrationRecord candidate = {};
    const uint32_t newVoltageScale = (uint32_t)(
        ((uint32_t)measuredVoltageCenti * 256U + meanVoltageRaw / 2U) /
        meanVoltageRaw);
    const uint32_t newSourceScale = (uint32_t)(
        ((uint32_t)measuredSourceCenti * 256U + meanSourceRaw / 2U) /
        meanSourceRaw);
    // Q8 gain: current[cA] = deltaADC[Q8] * gain[Q8] / 65536.
    // expected current * 65536 is comfortably bounded for the 10-ohm test.
    const int32_t newOutputCurrentGain = (int32_t)(
        (expectedOutputCurrentCenti * 65536) / deltaOutputCurrentRawQ8);
    const int32_t newInputCurrentGain = (int32_t)(
        (measuredInputCurrentCenti * 65536) / deltaInputCurrentRawQ8);
    const int32_t nominalOutputGain = nominalAcsGainCentiPerCountQ8(
        (uint16_t)ACS712_PRIMARY_SENSITIVITY_MV_PER_A);
    const int32_t nominalInputGain = nominalAcsGainCentiPerCountQ8(
        (uint16_t)ACS712_SOURCE_SENSITIVITY_MV_PER_A);

    if (newVoltageScale == 0UL || newVoltageScale > 65535UL ||
        newSourceScale == 0UL || newSourceScale > 65535UL ||
        !withinPercentOf((int32_t)newVoltageScale,
            nominalVoltageScaleQ8(VOLTAGE_DIVIDER_NUM, VOLTAGE_DIVIDER_DEN),
            (uint8_t)CAL_DIVIDER_SCALE_TOL_PERCENT) ||
        !withinPercentOf((int32_t)newSourceScale,
            nominalVoltageScaleQ8(SOURCE_DIVIDER_NUM, SOURCE_DIVIDER_DEN),
            (uint8_t)CAL_DIVIDER_SCALE_TOL_PERCENT) ||
        newOutputCurrentGain < INT16_MIN || newOutputCurrentGain > INT16_MAX ||
        newInputCurrentGain < INT16_MIN || newInputCurrentGain > INT16_MAX ||
        !withinPercentOf(absI32(newOutputCurrentGain), nominalOutputGain, 35U) ||
        !withinPercentOf(absI32(newInputCurrentGain), nominalInputGain, 35U)) {
      calibrationPrint(F("Rejected: derived sensor/divider scale out of range."), 54U);
      return;
    }

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

    // EEPROM.update() is synchronous on the Uno and a record write may take
    // tens to hundreds of milliseconds. Remove gate drive before committing;
    // never leave the converter open-loop at its last calibration duty.
    writeDutyBasisPoints(0L, 0L);
    resetCalibrationDutyCommand();
    if (!saveCalibrationToEeprom(candidate)) {
      latchFault(FAULT_EEPROM);
      calibrationWizardState = CAL_WIZARD_FAILED;
      calibrationPrint(F("EEPROM verify failed; output remains off."), 43U);
      return;
    }

    currentSensorsCalibrated = true;
    calibrationWizardState = CAL_WIZARD_READY;
    resetAdaptiveRuntimeAfterCalibration();
    calibrationPrint(F("Calibration saved; EEPROM read-back verified."), 47U);
    return;
  }

  calibrationPrint(F("Unknown command for this calibration state."), 45U);
}

static FW_NOINLINE void pollCalibrationSerial() {
  // Bound serial work so a host that continuously streams garbage cannot
  // starve the control deadline or delay observation of a hardware fault.
  uint8_t budget = 12U;
  while (budget-- > 0U && Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0) break;
    const char ch = (char)incoming;
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (calibrationCommandOverflow) {
        calibrationPrint(F("Command too long; rejected."), 29U);
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
    if (!zeroCalibrationInterlockActive()) {
      calibrationWizardState = CAL_WIZARD_WAIT_ZERO;
      calibrationPrint(F("ZERO aborted: D4 released during settling."), 44U);
      return;
    }
    if ((uint32_t)(millis() - calibrationStateStartMs) < ACS712_ZERO_SETTLE_MS)
      return;
    if (!calibrateAcs712Zero(CURRENT_PIN, primaryCurrentZeroRawQ8,
                             calibrationOutputZeroSpanRaw)) {
      calibrationWizardState = CAL_WIZARD_WAIT_ZERO;
      calibrationPrint(F("ZERO rejected: output ACS712 offset/noise."), 44U);
      return;
    }
#if SOURCE_CURRENT_SENSE_ENABLED
    if (!calibrateAcs712Zero(SOURCE_CURRENT_PIN, sourceCurrentZeroRawQ8,
                             calibrationInputZeroSpanRaw)) {
      calibrationWizardState = CAL_WIZARD_WAIT_ZERO;
      calibrationPrint(F("ZERO rejected: input ACS712 offset/noise."), 43U);
      return;
    }
#endif
    if (!zeroCalibrationInterlockActive()) {
      calibrationWizardState = CAL_WIZARD_WAIT_ZERO;
      calibrationPrint(F("ZERO rejected: D4 changed during acquisition."), 47U);
      return;
    }
    initialiseDefaultCalibrationScales();
    calibrationWizardState = CAL_WIZARD_WAIT_LOAD;
    calibrationStateStartMs = (uint32_t)millis();
    calibrationPrint(F("Zero captured. Connect 10-ohm load; release D4; send LOAD."), 60U);
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
  if (zeroCalibrationInterlockActive()) {
    writeDutyBasisPoints(0L, 0L);
    resetCalibrationDutyCommand();
    resetCalibrationAverages();
    calibrationWizardState = CAL_WIZARD_WAIT_LOAD;
    calibrationPrint(F("Calibration aborted: D4 asserted; output off."), 47U);
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
    calibrationPrint(F("Calibration paused: heatsink too warm; cool and retry LOAD."), 61U);
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
      calibrationPrint(F("Calibration fault: no output response; check stage/divider."), 61U);
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
    calibrationPrint(F("Cal paused: current limit; adjust pot and retry LOAD."), 58U);
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
    calibrationPrint(F("Calibration paused: insufficient Vin headroom; retry LOAD."), 60U);
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
        calibrationPrint(F("Output steady; wait for ADC window, then send DMM values."), 59U);
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
    calibrationPrint(F("Calibration drifted; reacquiring stable window."), 49U);
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

// -------------------------- RLS identification ---------------------------
static FW_NOINLINE uint8_t rlsCovarianceIndex(uint8_t row, uint8_t column) {
  if (row < column) { const uint8_t tmp = row; row = column; column = tmp; }
  return (uint8_t)(row * (row + 1U) / 2U + column);
}

static FW_NOINLINE int32_t rlsCovarianceGet(const RlsModel &model,
                                uint8_t row, uint8_t column) {
  if (row < column) { const uint8_t tmp = row; row = column; column = tmp; }
  return model.covarianceQ12[rlsCovarianceIndex(row, column)];
}

static FW_NOINLINE void resetRlsModel(RlsModel &model) {
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    model.thetaQ14[i] = 0;
    model.featureExcitationCount[i] = 0U;
    for (uint8_t j = 0U; j <= i; ++j)
      model.covarianceQ12[(uint8_t)(i * (i + 1U) / 2U + j)] =
          (int16_t)((i == j) ? RLS_P_INITIAL_Q12 : 0L);
  }
  model.disturbanceQ14 = 0;
  model.residualAbsQ14 = 0;
}

static FW_NOINLINE void resetDualModel(DualModel &model) {
  resetRlsModel(model.fast);
  resetRlsModel(model.slow);
}

static FW_NOINLINE int32_t linearPredictionQ14(const RlsModel &model,
                                   const int16_t phiQ14[MODEL_FEATURE_COUNT]) {
  int32_t sum = 0L;
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    sum += ((int32_t)model.thetaQ14[i] * (int32_t)phiQ14[i]) >> 14;
  }
  return sum;
}

static FW_NOINLINE void updateRlsModel(RlsModel &model,
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
  const int32_t disturbanceStep = innovation / 16L;
  model.disturbanceQ14 = clampI16(
      (int32_t)model.disturbanceQ14 + disturbanceStep,
      (int16_t)-RLS_DISTURBANCE_LIMIT_Q14,
      (int16_t)RLS_DISTURBANCE_LIMIT_Q14);

  bool hasExcitation = false;
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    if (absI32((int32_t)phiQ14[i]) >= RLS_MIN_EXCITATION_Q14) {
      hasExcitation = true;
      if (model.featureExcitationCount[i] < 255U)
        ++model.featureExcitationCount[i];
    }
  }
  if (!hasExcitation) return;

  const int32_t parameterErrorQ14 = innovation - disturbanceStep;
  int32_t pPhiQ12[MODEL_FEATURE_COUNT];
  int32_t quadraticQ12 = 0L;
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    int32_t sum = 0L;
    for (uint8_t j = 0U; j < MODEL_FEATURE_COUNT; ++j)
      sum += (rlsCovarianceGet(model, i, j) * (int32_t)phiQ14[j]) >> 14;
    pPhiQ12[i] = sum;
    quadraticQ12 += (pPhiQ12[i] * (int32_t)phiQ14[i]) >> 14;
  }
  const int32_t denominatorQ12 = (int32_t)lambdaQ12 + quadraticQ12;
  if (denominatorQ12 <= 0L) return;

  int32_t gainQ14[MODEL_FEATURE_COUNT];
  for (uint8_t i = 0U; i < MODEL_FEATURE_COUNT; ++i) {
    gainQ14[i] = (int32_t)((pPhiQ12[i] * MODEL_ONE_Q14) / denominatorQ12);
    const int32_t thetaDelta = (gainQ14[i] * parameterErrorQ14) >> 14;
    model.thetaQ14[i] = clampI16(
        (int32_t)model.thetaQ14[i] + thetaDelta, -16384, 16384);
  }
  for (uint8_t row = 0U; row < MODEL_FEATURE_COUNT; ++row) {
    for (uint8_t column = 0U; column <= row; ++column) {
      const uint8_t index = (uint8_t)(row * (row + 1U) / 2U + column);
      const int32_t correctionQ12 = (gainQ14[row] * pPhiQ12[column]) >> 14;
      int32_t reducedQ12 = model.covarianceQ12[index] - correctionQ12;
      reducedQ12 = (reducedQ12 * (int32_t)lambdaReciprocalQ15) >> 15;
      if (row == column)
        reducedQ12 = clampI32(reducedQ12, RLS_P_MIN_Q12, RLS_P_MAX_Q12);
      else
        reducedQ12 = clampI32(reducedQ12, -RLS_P_MAX_Q12, RLS_P_MAX_Q12);
      // Every covariance cell was clamped to +/-RLS_P_MAX_Q12 above;
      // the compile-time range assertion proves this conversion is lossless.
      model.covarianceQ12[index] = (int16_t)reducedQ12;
    }
  }
}

static FW_NOINLINE void updateDualModel(DualModel &dual,
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

static FW_NOINLINE bool modelHasParameterConfidence(const RlsModel &model,
                                         uint8_t featureIndex,
                                         uint16_t minimumExcitations) {
  if (featureIndex >= MODEL_FEATURE_COUNT) return false;
  return (model.featureExcitationCount[featureIndex] >= minimumExcitations) &&
         (rlsCovarianceGet(model, featureIndex, featureIndex) <= RLS_VALID_P_Q12) &&
         ((int32_t)model.thetaQ14[featureIndex] > RLS_MIN_GAIN_Q14);
}

static FW_NOINLINE bool modelHasFeatureConfidence(const RlsModel &model,
                                      uint8_t featureIndex,
                                      uint16_t minimumExcitations) {
  return modelHasParameterConfidence(model, featureIndex, minimumExcitations) &&
         (model.residualAbsQ14 < RLS_RESIDUAL_BAD_Q14);
}

static FW_NOINLINE bool modelHasInputConfidence(const RlsModel &model) {
  // Used for the voltage-stage model: feature 1 is inductor current.
  return modelHasFeatureConfidence(model, 1U, RLS_MIN_EXCITATIONS);
}

static FW_NOINLINE bool modelHasPrimaryCurrentConfidence(const RlsModel &model) {
  return modelHasFeatureConfidence(model, 1U, RLS_MIN_EXCITATIONS);
}

static FW_NOINLINE bool modelHasAuxiliaryConfidence(const RlsModel &model) {
  return modelHasFeatureConfidence(model, 2U, AUX_MODEL_MIN_EXCITATIONS);
}

static FW_NOINLINE bool modelHasBothCurrentActuators(const RlsModel &model) {
  return modelHasPrimaryCurrentConfidence(model) &&
         modelHasAuxiliaryConfidence(model);
}

static FW_NOINLINE int32_t modelDivergenceQ14(const RlsModel &fast,
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


static FW_NOINLINE void updateLoadEstimate(const RlsModel *voltagePlant) {
  loadEstimateValid = false;
  loadEstimateDeltaCenti = 0L;
  if (voltagePlant == 0 || !modelHasInputConfidence(*voltagePlant)) return;

  const int32_t currentGainQ14 = voltagePlant->thetaQ14[1];
  if (absI32(currentGainQ14) < RLS_MIN_GAIN_Q14) return;

  // In the centered voltage model, wV is equivalent to an unmeasured current
  // disturbance: wV ~= -bI*(Iload - Imean). This is not a direct measurement.
  // Without A2, source-side changes are observationally inseparable from load.
  // Identified disturbance and current gain are bounded Q14 values. Their
  // product is below 2.1M; the resulting load estimate is clamped before Q8.
  const int32_t equivalentUnmeasuredCurrent =
      -(((int32_t)voltagePlant->disturbanceQ14 *
         (int32_t)MODEL_CURRENT_DEVIATION_CENTI) / currentGainQ14);
  int32_t loadTarget = currentMeanCenti + equivalentUnmeasuredCurrent;
  const int32_t maximumLoad = CURRENT_LIMIT_CENTI * 2;
  loadTarget = clampI32(loadTarget, 0L, maximumLoad);

  const int32_t priorEstimate = estimatedLoadCurrentCenti;
  const int32_t targetQ8 = loadTarget * 256;
  if (!loadEstimateInitialized) {
    estimatedLoadCurrentQ8 = targetQ8;
    estimatedLoadCurrentCenti = loadTarget;
    loadEstimateInitialized = true;
  } else {
    estimatedLoadCurrentQ8 += (targetQ8 - estimatedLoadCurrentQ8) / 8;
    estimatedLoadCurrentCenti = estimatedLoadCurrentQ8 / 256;
  }
  loadEstimateDeltaCenti = estimatedLoadCurrentCenti - priorEstimate;
  loadEstimateValid = true;
}

// -------------------------- Pattern evaluation ---------------------------
static FW_NOINLINE void updateVoltageTrend(uint32_t nowUs, int32_t voltageCenti,
                               int32_t voltageErrorCenti) {
  if (!trendInitialized) {
    trendInitialized = true;
    trendWindowStartUs = nowUs;
    trendWindowStartVoltage = voltageCenti;
    voltageSlopeCentiPerSec = 0L;
  } else if ((uint32_t)(nowUs - trendWindowStartUs) >= VOLTAGE_SLOPE_WINDOW_US) {
    const uint32_t elapsedUs = nowUs - trendWindowStartUs;
    const int32_t delta = voltageCenti - trendWindowStartVoltage;
    // The trend window is >=50 ms and the ADC-derived output span is below
    // 2500 cV. Split the scale quotient/remainder to avoid 64-bit division.
    const uint32_t scaleWhole = 1000000UL / elapsedUs;
    const uint32_t scaleRemainder = 1000000UL % elapsedUs;
    const int32_t wholeTerm = (int32_t)(delta * (int32_t)scaleWhole);
    const int32_t remainderProduct =
        (int32_t)(delta * (int32_t)scaleRemainder);
    voltageSlopeCentiPerSec = wholeTerm +
        remainderProduct / (int32_t)elapsedUs;
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
        transientHoldCounter > 0U || currentGuardActive)
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
                                    bool auxiliaryModelReady) {
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
}
#endif

static FW_NOINLINE OperatingPattern classifyPattern(int32_t voltageErrorCenti,
                                        int32_t currentCenti,
                                        int32_t deltaCurrentCenti,
                                        int32_t deltaSourceCenti,
                                        int32_t deltaSourceCurrentCenti,
                                        int32_t deltaLoadEstimateCenti,
                                        const RlsModel *currentSelected,
                                        const RlsModel *voltageSelected,
                                        bool auxiliaryModelReady,
                                        bool knownControlExcitation) {
  if (currentCenti >= (runtimeCurrentLimitCenti * CURRENT_GUARD_HARD_PERCENT) / 100L)
    return PATTERN_CURRENT_BOUND;

#if SOURCE_SENSE_ENABLED && SOURCE_CURRENT_SENSE_ENABLED
  if (absI32(deltaSourceCenti) >= SOURCE_STEP_TRANSIENT_CENTI ||
      sourceCurrentStepDetected(deltaSourceCurrentCenti))
    return PATTERN_SOURCE_TRANSIENT;
#elif SOURCE_SENSE_ENABLED
  if (absI32(deltaSourceCenti) >= SOURCE_STEP_TRANSIENT_CENTI)
    return PATTERN_SOURCE_TRANSIENT;
  (void)deltaSourceCurrentCenti;
#elif SOURCE_CURRENT_SENSE_ENABLED
  if (sourceCurrentStepDetected(deltaSourceCurrentCenti))
    return PATTERN_SOURCE_TRANSIENT;
  (void)deltaSourceCenti;
#else
  (void)deltaSourceCenti;
  (void)deltaSourceCurrentCenti;
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

static FW_NOINLINE ResolutionStrategy chooseResolutionStrategy(OperatingPattern pattern) {
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

static FW_NOINLINE CandidatePatternClass candidateClassForPattern(OperatingPattern pattern) {
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

static FW_NOINLINE void recordCandidateOutcome(int32_t measuredDeltaVoltageCenti,
                                   int32_t measuredDeltaCurrentCenti) {
  if (!candidateOutcomePending) return;

  const int32_t voltageError = absI32(
      measuredDeltaVoltageCenti - pendingPredictedVoltageStepCenti);
  const int32_t currentError = absI32(
      measuredDeltaCurrentCenti - pendingPredictedCurrentStepCenti);
  // Errors are bounded before scaling; the maximum products are <2.1M.
  const uint32_t voltageErrorBounded =
      (uint32_t)clampI32(voltageError, 0L, 8192L);
  const uint32_t currentErrorBounded =
      (uint32_t)clampI32(currentError, 0L, 8192L);
  const uint32_t voltageErrorQ8 =
      (voltageErrorBounded * 256U) / (uint32_t)MODEL_V_SCALE_CENTI;
  const uint32_t currentErrorQ8 =
      (currentErrorBounded * 256U) / (uint32_t)MODEL_I_SCALE_CENTI;
  uint32_t combinedErrorQ8 = (voltageErrorQ8 + currentErrorQ8) / 2UL;
  if (combinedErrorQ8 > 65535UL) combinedErrorQ8 = 65535UL;

  CandidateOutcome &outcome =
      candidateOutcomes[pendingCandidateIndex][pendingCandidateClass];
  const int32_t targetErrorQ4 = (int32_t)clampI32(
      (int32_t)((combinedErrorQ8 + 8UL) >> 4), 0L, 255L);
  if (outcome.observations == 0U) {
    outcome.predictionErrorQ4 = (uint8_t)targetErrorQ4;
  } else {
    const int32_t difference = targetErrorQ4 -
                               (int32_t)outcome.predictionErrorQ4;
    outcome.predictionErrorQ4 = (uint8_t)clampI32(
        (int32_t)outcome.predictionErrorQ4 + difference / 4L, 0L, 255L);
  }
  if (outcome.observations < 255U) ++outcome.observations;
  candidateOutcomePending = false;
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
  if (mpptLimitActive && mpptOutputCurrentLimitCenti < limit)
    limit = mpptOutputCurrentLimitCenti;
#endif
  return (limit > 0L) ? limit : 0L;
}

// Apply protective PWM decisions after all mode schedulers and slew limiters.
// This final stage prevents a later path transition from restoring a command
// that an overvoltage/current guard has already withdrawn.
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

// This runs between control deadlines, never inside controlUpdate(). With
// two channels, each pot is sampled every 100 ms. Its update rate is deliberately
// slow and filtered; no fast-loop timing depends on potentiometer movement.
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
  refreshInputCurrentLimit();
#endif
}

static FW_NOINLINE void updateMeasuredEfficiency(int32_t sourceVoltageCenti,
                                     int32_t sourceCurrentCenti,
                                     int32_t outputVoltageCenti,
                                     int32_t outputCurrentCenti,
                                     bool allowMpptEstimateUpdate = true) {
  (void)allowMpptEstimateUpdate; // also permits builds with MPPT_ENABLED=0
  // Measurements outside the calibrated operating envelope are invalid.
  // These bounds also prove that outputProduct*1000 remains below INT32_MAX:
  // Vout <= 13.00 V, Iout <= 6.00 A => 780,000 * 1,000 = 780,000,000.
  // Vin <= 60.00 V, Iin <= 6.00 A => 3,600,000.
  if (sourceVoltageCenti <= 0L ||
      sourceVoltageCenti > EFFICIENCY_MAX_SOURCE_VOLTAGE_CENTI ||
      sourceCurrentCenti <= 0L ||
      sourceCurrentCenti > EFFICIENCY_MAX_CURRENT_CENTI ||
      outputVoltageCenti <= 0L ||
      outputVoltageCenti > OUTPUT_OVERVOLTAGE_CENTI ||
      outputCurrentCenti < 0L ||
      outputCurrentCenti > EFFICIENCY_MAX_CURRENT_CENTI) {
    return;
  }
  const int32_t inputProduct = sourceVoltageCenti * sourceCurrentCenti;
  const int32_t outputProduct = outputVoltageCenti * outputCurrentCenti;
  if (inputProduct <= 0L || outputProduct < 0L || outputProduct > inputProduct) {
    return;
  }
  const int32_t permille = (int32_t)((outputProduct * 1000L) / inputProduct);
  if (permille <= 0L || permille > 1000L) {
    return;
  }
#if MPPT_ENABLED
  // Translate the source-current ceiling using actual measured Pin/Pout once
  // enough power is flowing for sensor quantisation and offset to be material.
  // Subtract a margin before using the estimate so the output cap is
  // conservative; retain the last estimate during low-load/invalid windows.
  if (allowMpptEstimateUpdate &&
      inputProduct >= MPPT_EFFICIENCY_MIN_INPUT_PRODUCT &&
      outputProduct >= MPPT_EFFICIENCY_MIN_OUTPUT_PRODUCT &&
      permille >= 400L && permille <= 980L) {
    const int32_t conservativeEfficiency = clampI32(
        (int32_t)permille - MPPT_EFFICIENCY_MARGIN_PERMILLE,
        400L, 900L);
    if (!mpptEfficiencyEstimateValid) {
      mpptEfficiencyEstimatePermille =
          (uint16_t)conservativeEfficiency;
      mpptEfficiencyEstimateValid = true;
    } else {
      const int32_t difference = conservativeEfficiency -
          (int32_t)mpptEfficiencyEstimatePermille;
      mpptEfficiencyEstimatePermille = (uint16_t)clampI32(
          (int32_t)mpptEfficiencyEstimatePermille + difference / 64L,
          400L, 900L);
    }
  }
#endif
}

#if MPPT_ENABLED
static FW_NOINLINE bool mpptCommandResponseActive(uint32_t nowMs) {
  return mpptInitialised &&
      (int32_t)(nowMs - mpptCommandResponseUntilMs) < 0;
}

static FW_NOINLINE void resetMpptSampleWindow() {
  mpptPowerSum = 0UL;
  mpptVoltageSum = 0UL;
  mpptCurrentSum = 0UL;
  mpptWindowMinVoltageCenti = INT32_MAX;
  mpptWindowMaxVoltageCenti = INT32_MIN;
  mpptWindowMinCurrentCenti = INT32_MAX;
  mpptWindowMaxCurrentCenti = INT32_MIN;
  mpptSampleCount = 0U;
}

static FW_NOINLINE void setMpptCurrentTarget(int32_t targetCenti, int32_t minimumCenti,
                                 int32_t maximumCenti, uint32_t nowMs,
                                 MpptState nextState) {
  const int32_t bounded = clampI32(targetCenti, minimumCenti, maximumCenti);
  if (bounded != mpptInputCurrentTargetCenti) {
    mpptInputCurrentTargetCenti = bounded;
    resetMpptSampleWindow();
    mpptSettleUntilMs = nowMs + MPPT_SETTLE_MS;
    mpptCommandResponseUntilMs = nowMs + MPPT_COMMAND_RESPONSE_IGNORE_MS;
  }
  mpptState = nextState;
}

static FW_NOINLINE void recalculateMpptOutputLimit(int32_t sourceVoltageCenti,
                                       int32_t outputVoltageCenti) {
  mpptOutputCurrentLimitCenti = runtimeCurrentLimitCenti;
  mpptLimitActive = false;
  if (runtimeCurrentLimitCenti <= 0L || mpptInputCurrentTargetCenti <= 0L)
    return;
  if (sourceVoltageCenti < MPPT_MIN_SOURCE_VOLTAGE_CENTI) {
    mpptOutputCurrentLimitCenti = 0L;
    mpptLimitActive = true;
    return;
  }
  // Avoid a division singularity while the output soft-start is near zero.
  const uint32_t denominatorVoltage = (uint32_t)(
      (outputVoltageCenti < 200L) ? 200L : outputVoltageCenti);
  // Validated calibration scales and 4.00 A software limits bound the
  // product below 32-bit range on the supported Uno configuration. Staged
  // division keeps a 64-bit divide out of the 499 Hz control path.
  const uint32_t inputCurrentEquivalent =
      ((uint32_t)sourceVoltageCenti *
       (uint32_t)mpptInputCurrentTargetCenti) / denominatorVoltage;
  const uint32_t outputCurrentCap =
      (inputCurrentEquivalent * mpptEfficiencyEstimatePermille) / 1000UL;
  const int32_t boundedCap = (outputCurrentCap > (uint32_t)INT32_MAX)
      ? INT32_MAX : (int32_t)outputCurrentCap;
  if (boundedCap < runtimeCurrentLimitCenti) {
    mpptOutputCurrentLimitCenti = (boundedCap > 0L) ? boundedCap : 0L;
    mpptLimitActive = true;
  }
}

// Finite-difference P-V hill-climb with a P&O fallback. The controller
// perturbs a source-current ceiling, translates it into an output-current
// ceiling using Pin ~= Pout/eta, and lets the existing output regulator respond.
// This can seek MPP only while the load/storage can absorb the generated power.
static FW_NOINLINE void updateMppt(uint32_t nowMs,
                       int32_t sourceVoltageCenti,
                       int32_t sourceCurrentCenti,
                       int32_t outputVoltageCenti,
                       int32_t outputCurrentCenti,
                       int32_t voltageErrorCenti,
                       bool externalTransient,
                       bool currentGuardNear) {
  const int32_t maximumTarget = (int32_t)(
      runtimeInputCurrentLimitCenti * (int32_t)MPPT_MAX_SOURCE_CURRENT_PERCENT / 100L);
  const int32_t minimumTarget = MPPT_MIN_INPUT_CURRENT_CENTI;

  if (maximumTarget < minimumTarget || runtimeCurrentLimitCenti <= 0L) {
    mpptState = MPPT_STATE_DISABLED;
    mpptLimitActive = false;
    mpptOutputCurrentLimitCenti = runtimeCurrentLimitCenti;
    resetMpptSampleWindow();
    mpptBaselineValid = false;
    return;
  }

  if (!mpptInitialised) {
    mpptInputCurrentTargetCenti = clampI32(
        MPPT_START_INPUT_CURRENT_CENTI, minimumTarget, maximumTarget);
    mpptStepCenti = MPPT_INITIAL_STEP_CENTI;
    mpptDirection = 1;
    mpptInitialised = true;
    mpptBaselineValid = false;
    mpptSettleUntilMs = nowMs + MPPT_SETTLE_MS;
    mpptCommandResponseUntilMs = nowMs + MPPT_COMMAND_RESPONSE_IGNORE_MS;
    resetMpptSampleWindow();
    mpptState = MPPT_STATE_SETTLING;
  }

  if (mpptInputCurrentTargetCenti > maximumTarget) {
    setMpptCurrentTarget(maximumTarget, minimumTarget, maximumTarget,
                         nowMs, MPPT_STATE_SETTLING);
    mpptBaselineValid = false;
  }
  if (mpptInputCurrentTargetCenti < minimumTarget) {
    setMpptCurrentTarget(minimumTarget, minimumTarget, maximumTarget,
                         nowMs, MPPT_STATE_SETTLING);
    mpptBaselineValid = false;
  }

  if (sourceVoltageCenti < MPPT_MIN_SOURCE_VOLTAGE_CENTI) {
    // Distinguish an unplugged source (near-zero current) from a source that
    // collapsed under excessive draw. On collapse, back the current ceiling
    // down one bounded step per settle interval; on disconnect, restart from
    // the minimum ceiling so reconnection cannot inherit a stale high demand.
    mpptLimitActive = true;
    mpptOutputCurrentLimitCenti = 0L;
    mpptBaselineValid = false;
    resetMpptSampleWindow();
    if (sourceCurrentCenti <= MPPT_MIN_SOURCE_CURRENT_CENTI) {
      setMpptCurrentTarget(minimumTarget, minimumTarget, maximumTarget,
                           nowMs, MPPT_STATE_NO_SOURCE);
      mpptState = MPPT_STATE_NO_SOURCE;
    } else if (mpptInputCurrentTargetCenti > minimumTarget) {
      if ((int32_t)(nowMs - mpptSettleUntilMs) >= 0) {
        const int32_t reduced = mpptInputCurrentTargetCenti - mpptStepCenti;
        setMpptCurrentTarget(reduced, minimumTarget, maximumTarget,
                             nowMs, MPPT_STATE_SOURCE_RECOVERY);
      }
      mpptState = MPPT_STATE_SOURCE_RECOVERY;
    } else {
      mpptState = MPPT_STATE_NO_HEADROOM;
    }
    return;
  }

  // A buck cannot regulate when Vin is too close to Vout. Back away from the
  // source by reducing requested power; stop if the minimum target cannot
  // restore the required conversion headroom.
  const int32_t headroomVoltage = (outputVoltageCenti > 0L)
      ? outputVoltageCenti : 100L;
  const bool insufficientHeadroom =
      (sourceVoltageCenti * 100L) <
      (headroomVoltage * MPPT_MIN_HEADROOM_PERCENT);
  if (insufficientHeadroom) {
    if ((int32_t)(nowMs - mpptSettleUntilMs) >= 0) {
      if (mpptInputCurrentTargetCenti > minimumTarget) {
        const int32_t reduced = mpptInputCurrentTargetCenti - mpptStepCenti;
        setMpptCurrentTarget(reduced, minimumTarget, maximumTarget,
                             nowMs, MPPT_STATE_HEADROOM_RECOVERY);
        mpptBaselineValid = false;
      } else {
        mpptState = MPPT_STATE_NO_HEADROOM;
        mpptOutputCurrentLimitCenti = 0L;
        mpptLimitActive = true;
        mpptBaselineValid = false;
        resetMpptSampleWindow();
        return;
      }
    }
    recalculateMpptOutputLimit(sourceVoltageCenti, outputVoltageCenti);
    mpptState = MPPT_STATE_HEADROOM_RECOVERY;
    return;
  }

  if (externalTransient || currentGuardNear) {
    resetMpptSampleWindow();
    mpptBaselineValid = false;
    mpptSettleUntilMs = nowMs + MPPT_SETTLE_MS;
    mpptState = MPPT_STATE_SETTLING;
    recalculateMpptOutputLimit(sourceVoltageCenti, outputVoltageCenti);
    return;
  }

  // If output voltage is regulated and source current is well below the
  // requested ceiling, the load cannot accept more power. Do not perturb the
  // source point pointlessly: release the MPPT cap to its configured maximum.
  const bool outputNeedsMorePower =
      (voltageErrorCenti > MPPT_LOAD_LIMIT_ERROR_CENTI);
  const bool sourceCeilingNotBinding =
      (sourceCurrentCenti * 100L) <
      (mpptInputCurrentTargetCenti * MPPT_LOAD_LIMIT_CURRENT_PERCENT);
  if (!outputNeedsMorePower && sourceCeilingNotBinding) {
    if (mpptState != MPPT_STATE_LOAD_LIMITED ||
        mpptInputCurrentTargetCenti != maximumTarget) {
      setMpptCurrentTarget(maximumTarget, minimumTarget, maximumTarget,
                           nowMs, MPPT_STATE_LOAD_LIMITED);
    } else {
      mpptState = MPPT_STATE_LOAD_LIMITED;
    }
    mpptBaselineValid = false;
    resetMpptSampleWindow();
    recalculateMpptOutputLimit(sourceVoltageCenti, outputVoltageCenti);
    return;
  }

  if (mpptState == MPPT_STATE_LOAD_LIMITED) {
    mpptBaselineValid = false;
    resetMpptSampleWindow();
    mpptSettleUntilMs = nowMs + MPPT_SETTLE_MS;
    mpptState = MPPT_STATE_SETTLING;
  }

  if ((int32_t)(nowMs - mpptSettleUntilMs) < 0) {
    mpptState = MPPT_STATE_SETTLING;
    recalculateMpptOutputLimit(sourceVoltageCenti, outputVoltageCenti);
    return;
  }

  if (sourceCurrentCenti < 0L || outputCurrentCenti < 0L) {
    mpptBaselineValid = false;
    resetMpptSampleWindow();
    mpptState = MPPT_STATE_SETTLING;
    mpptSettleUntilMs = nowMs + MPPT_SETTLE_MS;
    recalculateMpptOutputLimit(sourceVoltageCenti, outputVoltageCenti);
    return;
  }

  const uint32_t powerProduct =
      (uint32_t)sourceVoltageCenti * (uint32_t)sourceCurrentCenti;
  mpptPowerSum += powerProduct;
  mpptVoltageSum += (uint32_t)sourceVoltageCenti;
  mpptCurrentSum += (uint32_t)sourceCurrentCenti;
  if (sourceVoltageCenti < mpptWindowMinVoltageCenti)
    mpptWindowMinVoltageCenti = sourceVoltageCenti;
  if (sourceVoltageCenti > mpptWindowMaxVoltageCenti)
    mpptWindowMaxVoltageCenti = sourceVoltageCenti;
  if (sourceCurrentCenti < mpptWindowMinCurrentCenti)
    mpptWindowMinCurrentCenti = sourceCurrentCenti;
  if (sourceCurrentCenti > mpptWindowMaxCurrentCenti)
    mpptWindowMaxCurrentCenti = sourceCurrentCenti;
  if (mpptSampleCount < 255U) ++mpptSampleCount;
  if (mpptSampleCount < MPPT_SAMPLE_COUNT) {
    mpptState = MPPT_STATE_TRACKING;
    recalculateMpptOutputLimit(sourceVoltageCenti, outputVoltageCenti);
    return;
  }

  const uint32_t averagePower =
      (mpptPowerSum + MPPT_SAMPLE_COUNT / 2U) / MPPT_SAMPLE_COUNT;
  const int32_t averageVoltage = (int32_t)(
      (mpptVoltageSum + MPPT_SAMPLE_COUNT / 2U) / MPPT_SAMPLE_COUNT);
  const int32_t averageCurrent = (int32_t)(
      (mpptCurrentSum + MPPT_SAMPLE_COUNT / 2U) / MPPT_SAMPLE_COUNT);
  const int32_t windowVoltageSpan =
      mpptWindowMaxVoltageCenti - mpptWindowMinVoltageCenti;
  const int32_t windowCurrentSpan =
      mpptWindowMaxCurrentCenti - mpptWindowMinCurrentCenti;
  resetMpptSampleWindow();

  // Do not compare two power points if either point was itself moving.
  // This rejects irradiance changes, source oscillation and large current
  // disturbances occurring inside one averaging window.
  if (windowVoltageSpan > MPPT_MAX_WINDOW_VOLTAGE_SPAN_CENTI ||
      windowCurrentSpan > MPPT_MAX_WINDOW_CURRENT_SPAN_CENTI) {
    mpptBaselineValid = false;
    mpptSettleUntilMs = nowMs + MPPT_SETTLE_MS;
    mpptState = MPPT_STATE_SETTLING;
    recalculateMpptOutputLimit(sourceVoltageCenti, outputVoltageCenti);
    return;
  }

  if (mpptBaselineValid) {
    const int32_t deltaV = averageVoltage - mpptPreviousSourceVoltageCenti;
    // Use the exact finite difference of measured P=V*I. The first-order
    // expression V*dI + I*dV omits dV*dI and can have the wrong sign when
    // the perturbation is large, precisely around the power peak.
    const int32_t signedDeltaPower = (int32_t)averagePower -
                                     (int32_t)mpptPreviousPowerProduct;
    const uint32_t deltaPower = (uint32_t)absI32(signedDeltaPower);
    const uint32_t relativePowerDeadband =
        (mpptPreviousPowerProduct * (uint32_t)MPPT_POWER_DEADBAND_PERMILLE) / 1000UL;
    // ACS712-05B has only about 26 mA per ADC count at 5 V / 10 bit.
    // Do not let a nominally small relative deadband turn quantisation into
    // false hill-climb direction changes at low source current.
    const uint32_t quantisationPowerFloor = (uint32_t)(
        (uint32_t)averageVoltage * (uint32_t)MPPT_CURRENT_NOISE_FLOOR_CENTI +
        (uint32_t)averageCurrent *
            (uint32_t)MPPT_SOURCE_VOLTAGE_NOISE_FLOOR_CENTI);
    const uint32_t powerDeadband =
        (relativePowerDeadband > quantisationPowerFloor)
            ? relativePowerDeadband : quantisationPowerFloor;
    int8_t desiredDirection = mpptDirection;
    bool holdAtLocalMaximum = false;

    if (absI32(deltaV) >= 10L) {
      // The finite-difference slope sign is sign(deltaP / deltaV).
      // Positive dP/dV means source voltage is below Vmp: reduce draw.
      const bool slopePositive =
          ((signedDeltaPower > 0L) && (deltaV > 0L)) ||
          ((signedDeltaPower < 0L) && (deltaV < 0L));
      const uint32_t derivativeMagnitude = (uint32_t)absI32(signedDeltaPower);
      if (derivativeMagnitude > powerDeadband)
        desiredDirection = slopePositive
            ? -1 : 1; // positive slope: reduce draw to raise source voltage
      else if (deltaPower <= powerDeadband) {
        if (mpptStepCenti > MPPT_MIN_STEP_CENTI) {
          const uint8_t reducedStep = (uint8_t)(mpptStepCenti / 2U);
          mpptStepCenti = (reducedStep < MPPT_MIN_STEP_CENTI)
              ? MPPT_MIN_STEP_CENTI : reducedStep;
        }
        holdAtLocalMaximum = true;
      }
    } else {
      // P&O fallback when source-voltage movement is below ADC resolution.
      if (averagePower + powerDeadband < mpptPreviousPowerProduct)
        desiredDirection = (int8_t)-mpptDirection;
      else if (deltaPower <= powerDeadband) {
        if (mpptStepCenti > MPPT_MIN_STEP_CENTI) {
          const uint8_t reducedStep = (uint8_t)(mpptStepCenti / 2U);
          mpptStepCenti = (reducedStep < MPPT_MIN_STEP_CENTI)
              ? MPPT_MIN_STEP_CENTI : reducedStep;
        }
        holdAtLocalMaximum = true;
      }
    }

    if (holdAtLocalMaximum) {
      // Stop drifting when the measured slope is within the sensor noise
      // floor. Revisit periodically so slow irradiance/temperature changes
      // can be followed without continuously dithering the source.
      mpptDirection = (int8_t)-mpptDirection;
      mpptBaselineValid = false;
      mpptSettleUntilMs = nowMs + MPPT_RESCAN_INTERVAL_MS;
      mpptState = MPPT_STATE_TRACKING;
      recalculateMpptOutputLimit(sourceVoltageCenti, outputVoltageCenti);
      return;
    }

    if (desiredDirection != mpptDirection) {
      mpptStepCenti = (uint8_t)((mpptStepCenti / 2U < MPPT_MIN_STEP_CENTI)
          ? MPPT_MIN_STEP_CENTI : mpptStepCenti / 2U);
    } else if (deltaPower > powerDeadband && mpptStepCenti < MPPT_MAX_STEP_CENTI) {
      ++mpptStepCenti;
    }
    mpptDirection = desiredDirection;
  }

  mpptPreviousPowerProduct = averagePower;
  mpptPreviousSourceVoltageCenti = averageVoltage;
  mpptBaselineValid = true;

  int32_t nextTarget = mpptInputCurrentTargetCenti +
      ((mpptDirection > 0) ? (int32_t)mpptStepCenti
                           : -(int32_t)mpptStepCenti);
  if (nextTarget > maximumTarget) {
    nextTarget = maximumTarget;
    mpptDirection = -1;
  } else if (nextTarget < minimumTarget) {
    nextTarget = minimumTarget;
    mpptDirection = 1;
  }
  setMpptCurrentTarget(nextTarget, minimumTarget, maximumTarget,
                       nowMs, MPPT_STATE_SETTLING);
  mpptSettleUntilMs = nowMs + MPPT_SETTLE_MS;
  recalculateMpptOutputLimit(sourceVoltageCenti, outputVoltageCenti);
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

static FW_NOINLINE int32_t scaledDisturbanceQ14(const RlsModel &model,
                                    uint16_t weightPermille) {
  return (int32_t)(((int32_t)model.disturbanceQ14 * (int32_t)weightPermille) / 1000L);
}

static FW_NOINLINE int32_t predictCurrentNoDutyQ14(const RlsModel &model,
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

static FW_NOINLINE int32_t predictCurrentQ14(const RlsModel &model,
                                 int32_t deltaCurrentCenti,
                                 int32_t deltaVoltageCenti,
                                 int32_t deltaSourceCenti,
                                 int32_t mainDutyDeviationBp,
                                 int32_t auxiliaryDutyDeviationBp,
                                 int32_t mainDutyLevelBp,
                                 uint16_t disturbanceWeightPermille) {
  const int32_t base = predictCurrentNoDutyQ14(
      model, deltaCurrentCenti,
      deltaVoltageCenti, deltaSourceCenti, mainDutyLevelBp,
      disturbanceWeightPermille);
  const int16_t mainFeature = toQ14(mainDutyDeviationBp, MODEL_DUTY_SCALE_BP);
  const int16_t auxiliaryFeature = toQ14(auxiliaryDutyDeviationBp,
                                          AUX_MODEL_DUTY_SCALE_BP);
  const int32_t mainTerm = ((int32_t)model.thetaQ14[1] * mainFeature) >> 14;
  const int32_t auxiliaryTerm = ((int32_t)model.thetaQ14[2] * auxiliaryFeature) >> 14;
  return base + mainTerm + auxiliaryTerm;
}

static FW_NOINLINE int32_t predictVoltageBaseQ14(const RlsModel &model,
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

static FW_NOINLINE int32_t addClampedModelTermQ14(int32_t baseQ14,
                                      int16_t gainQ14,
                                      int32_t valueCenti,
                                      int32_t scaleCenti) {
  const int16_t feature = toQ14(valueCenti, scaleCenti);
  return baseQ14 + (int32_t)(((int32_t)gainQ14 * feature) >> 14);
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

// -------------------------- Predict/evaluate functions -------------------
static FW_NOINLINE CandidateResult evaluateCandidate(uint8_t index,
                                          const RlsModel &currentPlant,
                                          const RlsModel &voltagePlant,
                                          int32_t voltageErrorCenti,
                                          int32_t voltageCenti,
                                          int32_t currentCenti,
                                          int32_t currentReferenceStateCenti,
                                          int32_t deltaVoltageCenti,
                                          int32_t deltaCurrentCenti,
                                          int32_t deltaSourceCenti,
                                          int32_t sourceVoltageCenti,
                                          int32_t currentDutyBp,
                                          int32_t currentDutyMeanBp,
                                          int32_t auxiliaryDutyBp,
                                          int32_t auxiliaryMeanDutyBp,
                                          OperatingPattern pattern,
                                          bool auxiliaryAllowed) {
  CandidateResult result;
  result.cost = INT32_MAX;
  result.currentReferenceCenti = 0U;
  result.dutyBasisPoints = (uint16_t)clampI32(currentDutyBp, 0L,
                                               MAX_DUTY_BASIS_POINTS);
  result.primaryEquivalentDutyBp = result.dutyBasisPoints;
  result.auxiliaryDutyBasisPoints = (uint16_t)clampI32(auxiliaryDutyBp, 0L,
                                                AUX_MAX_DUTY_BASIS_POINTS);
  result.predictedVoltageDeltaCenti = 0;
  result.predictedNextVoltageDeltaCenti = 0;
  result.predictedCurrentDeltaCenti = 0;
  result.candidateIndex = index;
  result.valid = 0U;

  const FunctionCandidate candidate = readFunctionCandidate(index);
  const int32_t outerGainQ14 = voltagePlant.thetaQ14[1];
  if (outerGainQ14 <= RLS_MIN_GAIN_Q14) return result;

  const bool requestsAuxiliary = candidate.auxiliarySharePermille > 0U;
  const bool auxiliaryOnlyMode =
      (auxiliaryPathMode == AUX_MODE_AUXILIARY_ONLY);
  if ((requestsAuxiliary || auxiliaryOnlyMode) &&
      (!auxiliaryAllowed ||
       currentPlant.thetaQ14[2] <= RLS_MIN_GAIN_Q14)) return result;

  // Model the output capacitor one sample behind the inductor-current state.
  const int32_t voltageBase1Q14 = predictVoltageBaseQ14(
      voltagePlant, deltaVoltageCenti, deltaSourceCenti,
      candidate.disturbanceWeightPermille);
  const int16_t measuredCurrentFeature = toQ14(
      currentCenti - currentMeanCenti, MODEL_CURRENT_DEVIATION_CENTI);
  const int32_t currentTerm1Q14 = (int32_t)(
      ((int32_t)outerGainQ14 * measuredCurrentFeature) >> 14);
  const int32_t predictedDeltaV1Q14 = voltageBase1Q14 + currentTerm1Q14;
  const int32_t predictedDeltaV1 = fromQ14(predictedDeltaV1Q14,
                                           MODEL_V_SCALE_CENTI);
  const int32_t errorAfter1 = voltageErrorCenti - predictedDeltaV1;

  const int32_t voltageBase2Q14 = predictVoltageBaseQ14(
      voltagePlant, predictedDeltaV1, deltaSourceCenti,
      candidate.disturbanceWeightPermille);
  int32_t desiredDeltaV2Centi =
      (int32_t)(((int32_t)candidate.alphaVoltageQ14 * errorAfter1) >> 14) -
      (int32_t)(((int32_t)candidate.dampingVoltageQ14 * predictedDeltaV1) >> 14);
  desiredDeltaV2Centi = clampI32(desiredDeltaV2Centi,
                                 -MAX_PREDICTED_DV_CENTI,
                                 MAX_PREDICTED_DV_CENTI);
  const int32_t desiredDeltaV2Q14 = toQ14(desiredDeltaV2Centi,
                                          MODEL_V_SCALE_CENTI);
  const int32_t voltageResidualQ14 = desiredDeltaV2Q14 - voltageBase2Q14;

  // Solve the outer stage for the required next inductor current.
  const int32_t requestedCurrentCentered = (int32_t)(
      (voltageResidualQ14 * (int32_t)MODEL_CURRENT_DEVIATION_CENTI) / outerGainQ14);
  int32_t requestedCurrent = currentMeanCenti + requestedCurrentCentered;
  requestedCurrent = clampI32(requestedCurrent, 0L,
                               controlOutputCurrentLimitCenti());
  int32_t currentSlew = candidate.currentSlewCenti;
  if ((candidate.flags & CANDIDATE_CURRENT_GUARD) != 0U) {
    const int32_t guardedReference = (int32_t)(
        runtimeCurrentLimitCenti * (int32_t)CURRENT_GUARD_HARD_PERCENT / 100L);
    if (requestedCurrent > guardedReference) requestedCurrent = guardedReference;
    if (currentSlew < 50L) currentSlew = 50L;
  }
  int32_t currentReference = limitSlew((int32_t)requestedCurrent,
                                       currentReferenceStateCenti,
                                       currentSlew);
  currentReference = clampI32(currentReference, 0L, controlOutputCurrentLimitCenti());

  // Inner discrete Lyapunov target for the total inductor-current increment.
  const int32_t currentError = currentReference - currentCenti;
  int32_t desiredDeltaICenti =
      (int32_t)(((int32_t)candidate.alphaCurrentQ14 * currentError) >> 14) -
      (int32_t)(((int32_t)candidate.dampingCurrentQ14 * deltaCurrentCenti) >> 14);
  desiredDeltaICenti = clampI32(desiredDeltaICenti,
                                -MAX_PREDICTED_DI_CENTI,
                                MAX_PREDICTED_DI_CENTI);
  const int32_t desiredDeltaIQ14 = toQ14(desiredDeltaICenti,
                                         MODEL_I_SCALE_CENTI);

  // Estimate the primary-only duty that would produce the same requested
  // current change. This shadow value governs auxiliary-only eligibility even
  // while the actual primary PWM output is held at zero.
  const int32_t innerGainQ14 = currentPlant.thetaQ14[1];
  if (innerGainQ14 <= RLS_MIN_GAIN_Q14) return result;
  const int32_t presentDutyLevelBp =
      effectiveDutyLevelBp(currentDutyBp, auxiliaryDutyBp);
  const int32_t primaryOnlyBaseQ14 = predictCurrentNoDutyQ14(
      currentPlant, deltaCurrentCenti,
      deltaVoltageCenti, deltaSourceCenti, presentDutyLevelBp,
      candidate.disturbanceWeightPermille);
  const int32_t primaryOnlyResidualQ14 =
      desiredDeltaIQ14 - primaryOnlyBaseQ14;
  int32_t primaryOnlyDeviationBp = (int32_t)(
      (primaryOnlyResidualQ14 * (int32_t)MODEL_DUTY_SCALE_BP) / innerGainQ14);
  primaryOnlyDeviationBp = clampI32(primaryOnlyDeviationBp,
                                    -MAX_DUTY_BASIS_POINTS,
                                    MAX_DUTY_BASIS_POINTS);
  result.primaryEquivalentDutyBp = (uint16_t)quantizePrimaryDutyBp(
      clampI32(currentDutyMeanBp + primaryOnlyDeviationBp,
               MIN_DUTY_BASIS_POINTS, MAX_DUTY_BASIS_POINTS));

  // Base prediction excludes both actuators. Allocate a small fraction of the
  // required current increment to the fast, low-L branch only in its low-power
  // resolution class. Its duty has a much finer local scale because L_aux is
  // assumed to be about 10x smaller than L_primary.
  int32_t requestedAuxiliaryDuty =
      (requestsAuxiliary && auxiliaryAllowed)
          ? auxiliaryDutyBp
          : limitSlew(0L, auxiliaryDutyBp, AUX_DISABLE_SLEW_BP);
  int32_t requestedDuty = currentDutyBp;

  if (auxiliaryOnlyMode) {
    // In auxiliary-only operation the primary switch is held OFF. Solve the
    // complete requested incremental-current action using the identified
    // auxiliary actuator gain rather than copying the primary duty command.
    requestedDuty = 0L;
    for (uint8_t solve = 0U; solve < 2U; ++solve) {
      const int32_t baseQ14 = predictCurrentNoDutyQ14(
          currentPlant, deltaCurrentCenti,
          deltaVoltageCenti, deltaSourceCenti,
          effectiveDutyLevelBp(0L, requestedAuxiliaryDuty),
          candidate.disturbanceWeightPermille);
      const int32_t residualQ14 = desiredDeltaIQ14 - baseQ14;
      int32_t requiredDeviationBp = (int32_t)(
          (residualQ14 * (int32_t)AUX_MODEL_DUTY_SCALE_BP) /
          currentPlant.thetaQ14[2]);
      requiredDeviationBp = clampI32(requiredDeviationBp,
          -AUX_MAX_DUTY_BASIS_POINTS, AUX_MAX_DUTY_BASIS_POINTS);
      const int32_t absoluteAuxDuty = clampI32(
          auxiliaryMeanDutyBp + requiredDeviationBp, 0L,
          AUX_MAX_DUTY_BASIS_POINTS);
      requestedAuxiliaryDuty = limitSlew(
          absoluteAuxDuty, auxiliaryDutyBp, AUX_DUTY_SLEW_BP);
    }
    requestedAuxiliaryDuty = quantizeAuxiliaryDutyBp(
        clampI32(requestedAuxiliaryDuty, 0L, AUX_MAX_DUTY_BASIS_POINTS));
  } else {
    int32_t auxiliaryContributionQ14 = 0L;
    if (requestsAuxiliary) {
      const int32_t controlResidualQ14 =
          desiredDeltaIQ14 - predictCurrentNoDutyQ14(
              currentPlant, deltaCurrentCenti,
              deltaVoltageCenti, deltaSourceCenti,
              effectiveDutyLevelBp(currentDutyBp, requestedAuxiliaryDuty),
              candidate.disturbanceWeightPermille);
      const int32_t allocatedAuxQ14 =
          (controlResidualQ14 * (int32_t)candidate.auxiliarySharePermille) /
          1000L;
      const int32_t requiredAuxDeviationBp = (int32_t)(
          (allocatedAuxQ14 * (int32_t)AUX_MODEL_DUTY_SCALE_BP) /
          currentPlant.thetaQ14[2]);
      const int32_t absoluteAuxDuty = clampI32(
          auxiliaryMeanDutyBp + requiredAuxDeviationBp, 0L,
          AUX_MAX_DUTY_BASIS_POINTS);
      const int32_t auxSlew = (candidate.auxiliaryDutySlewBp > 0U)
          ? candidate.auxiliaryDutySlewBp : AUX_DUTY_SLEW_BP;
      requestedAuxiliaryDuty = limitSlew(absoluteAuxDuty,
                                         auxiliaryDutyBp, auxSlew);
      requestedAuxiliaryDuty = clampI32(requestedAuxiliaryDuty, 0L,
                                        AUX_MAX_DUTY_BASIS_POINTS);
      const int16_t auxFeature = toQ14(
          requestedAuxiliaryDuty - auxiliaryMeanDutyBp,
          AUX_MODEL_DUTY_SCALE_BP);
      auxiliaryContributionQ14 = (int32_t)(
          ((int32_t)currentPlant.thetaQ14[2] * auxFeature) >> 14);
    }

    // Solve the primary inductor path for the unallocated remainder.
    for (uint8_t solve = 0U; solve < 2U; ++solve) {
      const int32_t sourceBaseQ14 = predictCurrentNoDutyQ14(
          currentPlant, deltaCurrentCenti,
          deltaVoltageCenti, deltaSourceCenti,
          effectiveDutyLevelBp(requestedDuty, requestedAuxiliaryDuty),
          candidate.disturbanceWeightPermille);
      const int32_t currentResidualQ14 =
          desiredDeltaIQ14 - sourceBaseQ14 - auxiliaryContributionQ14;
      int32_t requiredDeviationBp = (int32_t)(
          (currentResidualQ14 * (int32_t)MODEL_DUTY_SCALE_BP) / innerGainQ14);
      requiredDeviationBp = clampI32(requiredDeviationBp,
          -MAX_DUTY_BASIS_POINTS, MAX_DUTY_BASIS_POINTS);
      const int32_t absoluteDuty = clampI32(currentDutyMeanBp +
          requiredDeviationBp, MIN_DUTY_BASIS_POINTS,
          MAX_DUTY_BASIS_POINTS);
      requestedDuty = limitSlew(absoluteDuty, currentDutyBp,
                                candidate.dutySlewBp);
      requestedDuty = clampI32(requestedDuty, MIN_DUTY_BASIS_POINTS,
                                MAX_DUTY_BASIS_POINTS);
    }
  }

  requestedDuty = quantizePrimaryDutyBp(requestedDuty);
  requestedAuxiliaryDuty = quantizeAuxiliaryDutyBp(requestedAuxiliaryDuty);

  const int32_t primaryDutyDelta = requestedDuty - currentDutyBp;
  const int32_t predictedCurrentQ14 = predictCurrentQ14(
      currentPlant, deltaCurrentCenti,
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

  const int32_t nextVoltageError = clampI32(
      voltageErrorCenti - predictedVoltageDelta, -3000L, 3000L);
  const int32_t nextCurrentError = clampI32(
      currentReference - predictedCurrent, -1000L, 1000L);
  const int32_t presentCurrentError = clampI32(
      currentReferenceStateCenti - currentCenti, -1000L, 1000L);
  const int32_t eV = clampI32(voltageErrorCenti, -3000L, 3000L);
  const int32_t dMain = clampI32(primaryDutyDelta, -9000L, 9000L);
  const int32_t dAux = clampI32(requestedAuxiliaryDuty - auxiliaryDutyBp,
                                -9000L, 9000L);
  const int32_t presentLyapunov =
      eV * eV + (int32_t)COST_CURRENT_WEIGHT *
      presentCurrentError * presentCurrentError;
  const int32_t predictedLyapunov =
      nextVoltageError * nextVoltageError + (int32_t)COST_CURRENT_WEIGHT *
      nextCurrentError * nextCurrentError;

  // All operands are bounded by the physical ADC, current and PWM ranges.
  // Saturating addition keeps malformed model estimates from wrapping cost.
  int32_t cost = predictedLyapunov;
  if (predictedLyapunov >= presentLyapunov) {
    const int32_t increase = predictedLyapunov - presentLyapunov;
    if (increase > (INT32_MAX - cost) / 4L) cost = INT32_MAX;
    else cost += (int32_t)((increase + 1L) * 4L);
  }
  const int32_t mainMoveCost = (dMain * dMain) / 20L;
  if (mainMoveCost > INT32_MAX - cost) cost = INT32_MAX;
  else cost += mainMoveCost;
  const uint32_t auxMoveSquare = (uint32_t)(dAux * dAux);
  const int32_t auxMoveCost = (int32_t)(
      (auxMoveSquare * (uint32_t)AUXILIARY_DUTY_COST_WEIGHT) / 100000000UL);
  if (auxMoveCost > INT32_MAX - cost) cost = INT32_MAX;
  else cost += auxMoveCost;

  bool measuredEfficiencyComparisonAvailable = false;
#if AUX_EFFICIENCY_LEARNING_ENABLED
  if (requestsAuxiliary && pattern == PATTERN_LOW_POWER) {
    measuredEfficiencyComparisonAvailable =
        empiricalAuxiliaryLossAvailable(currentCenti, sourceVoltageCenti);
  }
#endif
  if ((requestedAuxiliaryDuty > 0L || auxiliaryOnlyMode) &&
      !measuredEfficiencyComparisonAvailable) {
    const int32_t measuredAuxiliaryCurrent = auxiliaryOnlyMode
        ? currentCenti
        : (currentCenti * (int32_t)candidate.auxiliarySharePermille) / 1000;
    int32_t auxiliaryCurrentEstimate = measuredAuxiliaryCurrent;
    if (appliedAuxiliaryDutyBp > 0L) {
      auxiliaryCurrentEstimate = (measuredAuxiliaryCurrent *
          requestedAuxiliaryDuty) / appliedAuxiliaryDutyBp;
    } else if (requestedAuxiliaryDuty == 0L) {
      auxiliaryCurrentEstimate = 0L;
    }
    auxiliaryCurrentEstimate = clampI32(auxiliaryCurrentEstimate, 0L,
                                         currentCenti);
    const int32_t auxI2 = auxiliaryCurrentEstimate * auxiliaryCurrentEstimate;
    const int32_t rdsPenaltyFactor =
        (int32_t)AUX_RDS_RELATIVE_Q8 - 256;
    const int32_t extraConductionPenalty =
        (auxI2 * rdsPenaltyFactor) / 25600;
    const int32_t switchingCredit =
        (auxiliaryCurrentEstimate *
         (int32_t)AUX_SWITCHING_LOSS_CREDIT_Q8) / 256;
    const int32_t netAuxPenalty = extraConductionPenalty - switchingCredit;
    if (netAuxPenalty >= 0) {
      if (netAuxPenalty > INT32_MAX - cost) cost = INT32_MAX;
      else cost += netAuxPenalty;
    } else {
      cost = (cost > -netAuxPenalty) ? cost + netAuxPenalty : 0L;
    }
  }

  const int32_t softCurrent = (int32_t)(
      (runtimeCurrentLimitCenti * CURRENT_GUARD_START_PERCENT) / 100L);
  if (predictedCurrent > softCurrent) {
    const int32_t over = clampI32(predictedCurrent - softCurrent, 0L, 1000L);
    const int32_t penalty = over * over * 16L;
    if (penalty > INT32_MAX - cost) cost = INT32_MAX;
    else cost += penalty;
  }
  if (predictedCurrent < 0L) {
    const int32_t under = clampI32(-predictedCurrent, 0L, 1000L);
    const int32_t penalty = under * under * 16L;
    if (penalty > INT32_MAX - cost) cost = INT32_MAX;
    else cost += penalty;
  }
  if (predictedVoltage >= OUTPUT_OVERVOLTAGE_CENTI - 25L) {
    const int32_t overV = clampI32(
        predictedVoltage - (OUTPUT_OVERVOLTAGE_CENTI - 25L), 0L, 1000L);
    const int32_t penalty = overV * overV * 64L;
    if (penalty > INT32_MAX - cost) cost = INT32_MAX;
    else cost += penalty;
  }

  const int32_t uncertainty = (int32_t)currentPlant.residualAbsQ14 +
                              (int32_t)voltagePlant.residualAbsQ14;
  const int32_t uncertaintyPenalty =
      ((absI32(dMain) + absI32(dAux)) * uncertainty) / 8L;
  if (uncertaintyPenalty > INT32_MAX - cost) cost = INT32_MAX;
  else cost += uncertaintyPenalty;

  if (pattern == PATTERN_SOURCE_TRANSIENT || pattern == PATTERN_LOAD_TRANSIENT ||
      pattern == PATTERN_UNATTRIBUTED_TRANSIENT ||
      pattern == PATTERN_MODEL_DRIFT) {
    if ((candidate.flags & CANDIDATE_DISTURBANCE) == 0U)
      cost = (cost > INT32_MAX - 12000L) ? INT32_MAX : cost + 12000L;
  } else if (pattern == PATTERN_OSCILLATORY ||
             pattern == PATTERN_FAST_APPROACH) {
    if ((candidate.flags & CANDIDATE_DAMPED) == 0U)
      cost = (cost > INT32_MAX - 25000L) ? INT32_MAX : cost + 25000L;
    if ((candidate.flags & CANDIDATE_FAST) != 0U)
      cost = (cost > INT32_MAX - 50000L) ? INT32_MAX : cost + 50000L;
  } else if (pattern == PATTERN_NONLINEAR || pattern == PATTERN_MODEL_UNCERTAIN) {
    if ((candidate.flags & CANDIDATE_ROBUST) == 0U)
      cost = (cost > INT32_MAX - 18000L) ? INT32_MAX : cost + 18000L;
    if ((candidate.flags & CANDIDATE_FAST) != 0U)
      cost = (cost > INT32_MAX - 25000L) ? INT32_MAX : cost + 25000L;
  } else if (pattern == PATTERN_CURRENT_BOUND) {
    if ((candidate.flags & CANDIDATE_CURRENT_GUARD) == 0U)
      cost = (cost > INT32_MAX - 50000L) ? INT32_MAX : cost + 50000L;
  } else if (pattern == PATTERN_STEADY &&
             (candidate.flags & CANDIDATE_ROBUST) != 0U) {
    cost = (cost > INT32_MAX - 1000L) ? INT32_MAX : cost + 1000L;
  }
#if AUX_EFFICIENCY_LEARNING_ENABLED
  if (measuredEfficiencyComparisonAvailable) {
    const int32_t lossCost = empiricalAuxiliaryLossCost(currentCenti,
                                                         sourceVoltageCenti);
    const int32_t boundedLossCost = clampI32(lossCost,
                                              -1000000L, 1000000L);
    cost = (boundedLossCost < 0L && cost < -boundedLossCost)
        ? 0L : cost + boundedLossCost;
  }
#else
  (void)sourceVoltageCenti;
#endif

  const CandidateOutcome &history =
      candidateOutcomes[index][candidateClassForPattern(pattern)];
  if (history.observations >= 8U) {
    const uint32_t errorQ8 = (uint32_t)history.predictionErrorQ4 << 4;
    const uint32_t normalizedHistory =
        ((errorQ8 * errorQ8) >> 16) * (uint32_t)CANDIDATE_HISTORY_COST_WEIGHT;
    const int32_t historyPenalty = (normalizedHistory > (uint32_t)INT32_MAX)
        ? INT32_MAX : (int32_t)normalizedHistory;
    if (historyPenalty > INT32_MAX - cost) cost = INT32_MAX;
    else cost += historyPenalty;
  }

  result.cost = cost;
  result.currentReferenceCenti = (uint16_t)clampI32(currentReference, 0L,
      controlOutputCurrentLimitCenti());
  result.dutyBasisPoints = (uint16_t)clampI32(requestedDuty, 0L,
                                               MAX_DUTY_BASIS_POINTS);
  result.auxiliaryDutyBasisPoints = (uint16_t)clampI32(
      requestedAuxiliaryDuty, 0L, AUX_MAX_DUTY_BASIS_POINTS);
  result.predictedVoltageDeltaCenti = (int16_t)clampI32(
      predictedVoltageDelta, INT16_MIN, INT16_MAX);
  result.predictedNextVoltageDeltaCenti = (int16_t)clampI32(
      predictedDeltaV1, INT16_MIN, INT16_MAX);
  result.predictedCurrentDeltaCenti = (int16_t)clampI32(
      predictedCurrentDelta, INT16_MIN, INT16_MAX);
  result.valid = 1U;
  return result;
}

static FW_NOINLINE CandidateResult chooseBestCandidate(const RlsModel &currentPlant,
                                            const RlsModel &voltagePlant,
                                            int32_t voltageErrorCenti,
                                            int32_t voltageCenti,
                                            int32_t currentCenti,
                                            int32_t currentReferenceStateCenti,
                                            int32_t deltaVoltageCenti,
                                            int32_t deltaCurrentCenti,
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
  best.cost = INT32_MAX;
  best.valid = 0U;
  for (uint8_t i = 0U; i < FUNCTION_CANDIDATE_COUNT; ++i) {
    if ((candidateMask & (uint8_t)(1U << i)) == 0U) continue;
    const CandidateResult candidate = evaluateCandidate(
        i, currentPlant, voltagePlant, voltageErrorCenti, voltageCenti,
        currentCenti, currentReferenceStateCenti,
        deltaVoltageCenti, deltaCurrentCenti,
        deltaSourceCenti, sourceVoltageCenti, currentDutyBp, currentDutyMeanBp,
        auxiliaryDutyBp,
        auxiliaryMeanDutyBp, pattern, auxiliaryAllowed);
    if (candidate.valid && (!best.valid || candidate.cost < best.cost))
      best = candidate;
  }
  return best;
}

// -------------------------- Main control update --------------------------
static FW_NOINLINE void controlUpdate(uint32_t nowUs, uint32_t dtUs) {
  const uint16_t voltageRaw = (uint16_t)analogRead(VOLTAGE_PIN);
  const uint16_t currentRaw = (uint16_t)analogRead(CURRENT_PIN);
  const uint16_t thermistorRaw = (uint16_t)analogRead(THERMISTOR_PIN);
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
    currentMeanQ16 = (int32_t)(currentMeanQ16 +
        ((currentCenti * 65536L - currentMeanQ16) / 4096L));
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
#endif
  latestOutputCurrentCenti = currentCenti;

  const int32_t targetReferenceQ16 = TARGET_V_CENTI * 65536L;
  // Decompose the product to keep Q16 soft-start arithmetic in 32 bits.
  const uint32_t rateTime = (uint32_t)SOFTSTART_RATE_CV_PER_SEC * dtUs;
  const uint32_t referenceStepQ16 = (uint32_t)(
      (rateTime / 1000000UL) * 65536UL +
      ((rateTime % 1000000UL) * 4096UL) / 62500UL);
  if (voltageReferenceQ16 < targetReferenceQ16) {
    const uint32_t next = (uint32_t)voltageReferenceQ16 + referenceStepQ16;
    voltageReferenceQ16 = (next >= (uint32_t)targetReferenceQ16)
                        ? targetReferenceQ16 : (int32_t)next;
  } else if (voltageReferenceQ16 > targetReferenceQ16) {
    const int32_t next = voltageReferenceQ16 - (int32_t)referenceStepQ16;
    voltageReferenceQ16 = (next <= targetReferenceQ16)
                        ? targetReferenceQ16 : next;
  }

  const int32_t voltageReference = voltageReferenceQ16 / 65536L;
  const int32_t voltageError = voltageReference - voltageCenti;
  if (updateRunNoResponseMonitor((uint32_t)millis(), voltageCenti,
                                 voltageReference, sourceCenti,
                                 currentInstant, controlOutputCurrentLimitCenti(),
                                 sourceCurrentInstant, runtimeInputCurrentLimitCenti,
                                 appliedDutyBp, appliedAuxiliaryDutyBp)) {
    latchFault(FAULT_VOLTAGE_NO_RESPONSE);
    return;
  }
  updateVoltageTrend(nowUs, voltageCenti, voltageError);

  int32_t deltaVoltage = 0L;
  int32_t deltaCurrent = 0L;
  int32_t deltaSource = 0L;
  int32_t deltaSourceCurrent = 0L;
  if (previousMeasurementsValid) {
    deltaVoltage = voltageCenti - previousVoltageCenti;
    deltaCurrent = currentCenti - previousCurrentCenti;
#if SOURCE_SENSE_ENABLED
    deltaSource = sourceCenti - previousSourceCenti;
#endif
#if SOURCE_CURRENT_SENSE_ENABLED
    deltaSourceCurrent = sourceCurrentCenti - previousSourceCurrentCenti;
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
#if MPPT_ENABLED
  const bool mpptOwnActuatorResponse =
      mpptCommandResponseActive((uint32_t)millis());
#else
  const bool mpptOwnActuatorResponse = false;
#endif
  const bool identificationTransient = !knownControlExcitation &&
      !mpptOwnActuatorResponse &&
      (absI32(deltaCurrent) >= CURRENT_STEP_TRANSIENT_CENTI ||
       currentModel.fast.residualAbsQ14 >= RLS_RESIDUAL_TRANSIENT_Q14 ||
       voltageModel.fast.residualAbsQ14 >= RLS_RESIDUAL_TRANSIENT_Q14 ||
       modelDivergenceQ14(currentModel.fast, currentModel.slow) >=
           RLS_PARAMETER_DIVERGENCE_Q14 ||
       modelDivergenceQ14(voltageModel.fast, voltageModel.slow) >=
           RLS_PARAMETER_DIVERGENCE_Q14);
#if SOURCE_SENSE_ENABLED && SOURCE_CURRENT_SENSE_ENABLED
  const bool sourceTransient = !mpptOwnActuatorResponse &&
      (absI32(deltaSource) >= SOURCE_STEP_TRANSIENT_CENTI ||
       sourceCurrentStepDetected(deltaSourceCurrent));
#elif SOURCE_SENSE_ENABLED
  const bool sourceTransient = !mpptOwnActuatorResponse &&
      absI32(deltaSource) >= SOURCE_STEP_TRANSIENT_CENTI;
#elif SOURCE_CURRENT_SENSE_ENABLED
  const bool sourceTransient = !mpptOwnActuatorResponse &&
      sourceCurrentStepDetected(deltaSourceCurrent);
#else
  const bool sourceTransient = false;
#endif
  if (identificationTransient || sourceTransient) {
    transientHoldCounter = 24U;
  } else if (transientHoldCounter > 0U) {
    --transientHoldCounter;
  }
  const bool useFastIdentification = transientHoldCounter > 0U;
#if SOURCE_CURRENT_SENSE_ENABLED && SOURCE_SENSE_ENABLED
  if (++efficiencyMeasurementDivider >= 8U) {
    efficiencyMeasurementDivider = 0U;
    const bool efficiencyEstimatorStable =
        !identificationTransient && !sourceTransient &&
        (transientHoldCounter == 0U) && !knownControlExcitation &&
        (absI32(voltageError) <= EFFICIENCY_STABLE_VERROR_CENTI) &&
        (absI32(deltaCurrent) <= EFFICIENCY_STABLE_DCURRENT_CENTI) &&
        (absI32(deltaSource) <= EFFICIENCY_STABLE_DSOURCE_CENTI) &&
        (currentInstant < (runtimeCurrentLimitCenti *
                           CURRENT_GUARD_HARD_PERCENT) / 100L) &&
        (sourceCurrentInstant < (runtimeInputCurrentLimitCenti *
                                 INPUT_CURRENT_GUARD_START_PERCENT) / 100L);
    updateMeasuredEfficiency(sourceCenti, sourceCurrentCenti,
                             voltageCenti, currentCenti,
                             efficiencyEstimatorStable);
  }
#endif
#if MPPT_ENABLED
  const bool mpptNearCurrentGuard =
      currentInstant >= (runtimeCurrentLimitCenti * CURRENT_GUARD_HARD_PERCENT) / 100L ||
      sourceCurrentInstant >= (runtimeInputCurrentLimitCenti * INPUT_CURRENT_GUARD_START_PERCENT) / 100L;
  updateMppt((uint32_t)millis(), sourceCenti, sourceCurrentCenti,
             voltageCenti, currentCenti, voltageError,
             (transientHoldCounter > 0U), mpptNearCurrentGuard);
#endif
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
                                    deltaSource, deltaSourceCurrent,
                                    loadEstimateDeltaCenti,
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
  bootstrapIref = clampI32(bootstrapIref, 0L, controlOutputCurrentLimitCenti());
  const int32_t bootstrapDutyRequest = bootstrapDuty(bootstrapIref - currentCenti,
                                                       dtUs);
  const int32_t bootstrapDutyLimited = limitSlew(bootstrapDutyRequest,
                                                  appliedDutyBp,
                                                  BOOT_DUTY_SLEW_BP);

  activeResolutionStrategy = chooseResolutionStrategy(activePattern);
  const ResolutionProfile resolution =
      readResolutionProfile(activeResolutionStrategy);
  CandidateResult best = {};
  best.cost = INT32_MAX;
  best.candidateIndex = 255U;
  best.valid = 0U;
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
                               deltaVoltage, deltaCurrent, deltaSource,
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
    const int32_t outerP = voltageError * BOOT_OUTER_KP_Q8 * 256L;
    const int32_t maxOuterI = controlOutputCurrentLimitCenti() * 65536L;
    voltageIntegratorQ16 = clampI32((int32_t)(
        best.currentReferenceCenti * 65536L - outerP),
        -maxOuterI, maxOuterI);

    const int32_t bootstrapCurrentError = best.currentReferenceCenti - currentCenti;
    const int32_t innerP = bootstrapCurrentError * BOOT_INNER_KP_Q8 * 256L;
    const int32_t maxInnerI = MAX_DUTY_BASIS_POINTS * 65536L;
    currentIntegratorQ16 = clampI32((int32_t)(
        selectedDuty * 65536L - innerP), -maxInnerI, maxInnerI);
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
        const uint8_t shifted = (uint8_t)(auxiliaryProbeLfsr << 1U);
        auxiliaryProbeLfsr = (uint8_t)((uint8_t)(shifted | feedback) &
                                       (uint8_t)0x7FU);
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
      (currentInstant < (controlOutputCurrentLimitCenti() *
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
      const uint8_t shifted = (uint8_t)(probeLfsr << 1U);
      probeLfsr = (uint8_t)((uint8_t)(shifted | feedback) &
                            (uint8_t)0x7FU);
      if (probeLfsr == 0U) probeLfsr = 0x5DU;
      probeSign = ((probeLfsr & 1U) != 0U) ? 1 : -1;
    }
    selectedDuty = (int32_t)(selectedDuty + ((probeSign > 0)
        ? (int32_t)IDENTIFICATION_PROBE_BP : -(int32_t)IDENTIFICATION_PROBE_BP));
    selectedDuty = limitSlew(selectedDuty, appliedDutyBp,
                             IDENTIFICATION_PROBE_SLEW_BP);
    identificationProbeActive = true;
  } else {
    probeDivider = 0U;
  }
#endif

  // Protective inhibits are reapplied after auxiliary-path scheduling below.
  // Without this final override, a path-mode slew could accidentally restore
  // a nonzero auxiliary PWM after the above-reference or current-guard branch
  // had requested zero power.
  bool forceAuxiliaryOff = false;
  bool forceBothPwmOff = false;

  // Independent fast software barrier uses the unfiltered ADC current. This
  // supplements (never replaces) the external comparator/gate-driver shutdown.
  const int32_t guardEnterCenti = (int32_t)(
      runtimeCurrentLimitCenti * (int32_t)CURRENT_GUARD_HARD_PERCENT / 100L);
  const int32_t guardReleaseCenti = (int32_t)(
      runtimeCurrentLimitCenti * (int32_t)CURRENT_GUARD_RELEASE_PERCENT / 100L);
  const int32_t guardZeroCenti = (int32_t)(
      runtimeCurrentLimitCenti * (int32_t)CURRENT_GUARD_ZERO_PERCENT / 100L);
  if (!currentGuardActive && currentInstant >= guardEnterCenti) {
    currentGuardActive = true;
  } else if (currentGuardActive && currentInstant <= guardReleaseCenti) {
    currentGuardActive = false;
  }
  if (currentGuardActive) {
    commandedCurrentReferenceCenti = clampI32(
        currentInstant - CURRENT_GUARD_BRAKE_STEP_BP / 4L,
        0L, controlOutputCurrentLimitCenti());
    const int32_t reducedDuty = (appliedDutyBp > CURRENT_GUARD_BRAKE_STEP_BP)
        ? appliedDutyBp - CURRENT_GUARD_BRAKE_STEP_BP : 0L;
    if (selectedDuty > reducedDuty) selectedDuty = reducedDuty;
    forceAuxiliaryOff = true;
#if AUX_PATH_ARMED
    auxiliaryLowPowerActive = false;
#endif
    if (currentInstant >= guardZeroCenti) selectedDuty = 0L;
    currentIntegratorQ16 = 0L;
  }
#if SOURCE_CURRENT_SENSE_ENABLED
  applyInputCurrentGuard(sourceCurrentInstant, selectedDuty,
                         selectedAuxiliaryDuty);
  if (inputCurrentGuardActive) forceAuxiliaryOff = true;
#endif

  // One-quadrant converter: above reference, cease positive power demand.
  if (voltageError < -VOLTAGE_ERROR_DEADBAND_CENTI) {
    forceBothPwmOff = true;
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
  enforcePowerStageInhibits(forceAuxiliaryOff, forceBothPwmOff,
                            selectedDuty, selectedAuxiliaryDuty);
  if (controlOutputCurrentLimitCenti() <= 0L) {
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
  dutyMeanQ16 = (int32_t)(dutyMeanQ16 +
      ((appliedDutyBp * 65536L - dutyMeanQ16) / DUTY_MEAN_TAU_SAMPLES));
  dutyMeanBp = dutyMeanQ16 / 65536L;
  previousDutyCenteredBp = appliedDutyBp - dutyMeanBp;
  auxiliaryDutyMeanQ16 = (int32_t)(auxiliaryDutyMeanQ16 +
      ((appliedAuxiliaryDutyBp * 65536L - auxiliaryDutyMeanQ16) /
       DUTY_MEAN_TAU_SAMPLES));
  auxiliaryDutyMeanBp = auxiliaryDutyMeanQ16 / 65536L;
  previousAuxiliaryDutyCenteredBp = appliedAuxiliaryDutyBp - auxiliaryDutyMeanBp;

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
#if SOURCE_CURRENT_SENSE_ENABLED
  previousSourceCurrentCenti = sourceCurrentCenti;
#endif
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
    Serial.println(F("EEPROM record valid, but ACS712 zero check failed."));
    printCalibrationInstructions();
  } else {
    calibrationWizardState = CAL_WIZARD_READY;
    Serial.println(F("EEPROM calibration loaded; config fingerprint, CRC and ACS712 zero checks passed."));
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

  // Potentiometer service is deliberately outside the fixed-rate control
  // update and remains active during first-boot calibration as well.
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

  controlUpdate(nowUs, dtUs);
}
