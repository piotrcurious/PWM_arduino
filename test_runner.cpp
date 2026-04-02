
#include "Arduino.h"
#include "simulator.h"
#include <stdio.h>
#include <string.h>

extern void setup();
extern void loop();

int main(int argc, char** argv) {
    FILE* csv = NULL;
    if (argc > 1) {
        csv = fopen(argv[1], "w");
        if (csv) {
            fprintf(csv, "time,v_out,i_l,temp,pwm,inductance\n");
        }
    }

    simulator.load_config("sim_config.txt");
    setup();
    printf("Simulation starting for 1.0 seconds of virtual time...\n");

    double total_time = 1.0;
    double dt_loop = 0.001; // Match common controller dt
    int steps = (int)(total_time / dt_loop);

    for (int i = 0; i < steps; i++) {
        double loop_start_time = simulator.sim_time_sec;
        loop();

        double loop_elapsed = simulator.sim_time_sec - loop_start_time;
        if (loop_elapsed < dt_loop) {
            while (simulator.sim_time_sec < loop_start_time + dt_loop) {
                simulator.step(1e-6);
            }
        }

        if (csv && (i % 10 == 0)) {
            extern float inductance;
            fprintf(csv, "%.6f,%.2f,%.2f,%.2f,%d,%.10f\n",
                    simulator.sim_time_sec, simulator.V_out, simulator.I_L, simulator.Temp, simulator.pwm_pin_val, (double)inductance);
        }

        if (i % (steps/10) == 0) {
            printf("Time: %.3fs, V_out=%.2fV, I_L=%.2fA, Temp=%.2fC, PWM=%d\n",
                   simulator.sim_time_sec, simulator.V_out, simulator.I_L, simulator.Temp, simulator.pwm_pin_val);
        }
    }
    printf("Final state: V_out=%.2fV, I_L=%.2fA, Temp=%.2fC\n", simulator.V_out, simulator.I_L, simulator.Temp);

    if (csv) fclose(csv);
    return 0;
}
