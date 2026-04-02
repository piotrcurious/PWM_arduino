
#include "Arduino.h"
#include "simulator.h"
#include <stdio.h>
#include <assert.h>

extern void setup();
extern void loop();

int main() {
    setup();
    printf("Simulation starting for 0.5 seconds of virtual time...\n");
    // Run for 0.5 seconds of virtual time
    double total_time = 0.5;
    double dt_loop = 1e-4; // 100us per loop call
    int steps = (int)(total_time / dt_loop);

    for (int i = 0; i < steps; i++) {
        double loop_start_time = simulator.sim_time_sec;
        loop();

        // Ensure time progresses if loop() didn't contain enough delay() calls
        double loop_elapsed = simulator.sim_time_sec - loop_start_time;
        if (loop_elapsed < dt_loop) {
            // Step the simulator until the end of the loop interval
            while (simulator.sim_time_sec < loop_start_time + dt_loop) {
                simulator.step(1e-6);
            }
        }

        if (i % (steps/10) == 0) {
            printf("Time: %.3fs, V_out=%.2fV, I_L=%.2fA, Temp=%.2fC, PWM=%d\n",
                   simulator.sim_time_sec, simulator.V_out, simulator.I_L, simulator.Temp, simulator.pwm_pin_val);
        }
    }
    printf("Final state: V_out=%.2fV, I_L=%.2fA, Temp=%.2fC\n", simulator.V_out, simulator.I_L, simulator.Temp);

    // Basic stability check: V_out should not be zero after 0.5s if PWM is active
    if (simulator.V_out < 0.1 && simulator.pwm_pin_val > 0) {
        printf("WARNING: V_out is very low despite PWM being active!\n");
    }

    return 0;
}
