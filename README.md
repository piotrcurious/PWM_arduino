# PWM Arduino Simulation and Testing Framework

This repository contains Arduino sketches and C code for power converter control (Boost Converters), along with a custom-built simulation environment for testing and verification without physical hardware.

## Project Structure

- **Sketches (`.ino`, `.c`):**
  - `classic_PI.ino`: Proportional-Integral controller for voltage and current.
  - `lyapunov_controller.ino`: Advanced fixed-point controller based on Lyapunov stability theory.
  - `classic_voltage_current_limited.ino`: Basic feedback controller with limits.
  - `inductance_estimator.ino`: Adaptive controller that estimates circuit inductance.
  - `dumb_SR.ino`: Simple synchronous rectification logic.
  - `weird_SR.ino`: Software-timed synchronous rectification.
  - `very_simple_thermal_limited.ino`: Controller with basic over-temperature protection.
  - `very_simple_thermal_limited_with_WDT.ino`: Thermal limited controller with Watchdog Timer.
  - `setup_pwm.c`: Low-level AVR register setup for high-frequency PWM.

- **Simulation Framework:**
  - `Arduino.h`, `ArduinoMock.cpp`: Mock implementation of the Arduino API.
  - `simulator.h`, `ArduinoMockSim.cpp`: Physics-based boost converter simulator (models voltage, current, and temperature).
  - `avr/`: Mock AVR hardware headers.
  - `test_runner.cpp`: C++ entry point that executes the Arduino code within the simulator.
  - `generate_graphs.py`: Python script to visualize simulation results.

## Getting Started

### Prerequisites

- `g++` (GCC C++ compiler)
- `make`
- `python3` with `pandas` and `matplotlib`

```bash
pip install pandas matplotlib
```

### Running Simulations

To compile and run all simulations:

```bash
make
```

To run a specific test and see the output:

```bash
./classic_PI.test
```

### Generating Performance Reports

To generate CSV data and PNG graphs for all controllers under dynamic load stress:

```bash
make report
```

Graphs will be saved as `[filename]_results.png`.

## Simulation Model

The `BoostSimulator` models a boost converter circuit using Euler integration. It includes:
- Inductor current dynamics ($dI_L/dt$)
- Output capacitor voltage dynamics ($dV_{out}/dt$)
- Diode forward voltage drops
- Input-to-output passive charging
- Basic thermal model (resistive heating and ambient cooling)
- Measurement noise on ADC inputs

## Test Results

The framework captures time-series data for each controller. Below is a summary of the expected behavior observed in simulation:

| Controller | Status | Observation |
|------------|--------|-------------|
| Classic PI | Functional | Reliable voltage regulation using standard PI control with anti-windup. |
| Voltage/Current Limited | Functional | Efficiently clamps output voltage and current within safe operating limits. |
| Inductance Estimator | Functional | Advanced adaptive controller that observes and estimates inductor health while regulating voltage. |
| Lyapunov Controller | Functional | High-performance stability-centric control using efficient integer logic. |
| Setup PWM | Functional | High-fidelity hardware register manipulation for optimized switching frequencies. |

*Note: Graphs can be found in the project root after running `make report`.*
