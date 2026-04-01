
#include "Arduino.h"
#include "simulator.h"
#include <stdio.h>

extern void setup();
extern void loop();

int main() {
    setup();
    printf("Simulation starting for %d steps...\n", 10000);
    for (int i = 0; i < 100000; i++) {
        loop();
        for(int s=0; s<10; s++) simulator.step(1e-6); // 10us per step in 1us sub-steps
        if (i % 1000 == 0) {
            printf("Step %d: V_out=%.2f, I_L=%.2f, Temp=%.2f, PWM=%d\n", i, simulator.V_out, simulator.I_L, simulator.Temp, simulator.pwm_pin_val);
        }
    }
    printf("Final state: V_out=%.2f, I_L=%.2f, Temp=%.2f\n", simulator.V_out, simulator.I_L, simulator.Temp);
    return 0;
}
