
import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

def generate_graphs(csv_file, output_prefix):
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading {csv_file}: {e}")
        return

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    # Voltage Plot
    ax1.plot(df['time'], df['v_out'], label='V_out', color='blue')
    ax1.set_ylabel('Voltage (V)')
    ax1.set_title(f'Simulation Results: {output_prefix}')
    ax1.grid(True)
    ax1.legend()

    # Current Plot
    ax2.plot(df['time'], df['i_l'], label='I_inductor', color='green')
    ax2.set_ylabel('Current (A)')
    ax2.grid(True)
    ax2.legend()

    # PWM and Temperature
    ax3.plot(df['time'], df['pwm'], label='PWM', color='red', alpha=0.5)
    ax3_temp = ax3.twinx()
    ax3_temp.plot(df['time'], df['temp'], label='Temperature', color='orange')
    ax3.set_ylabel('PWM Value')
    ax3_temp.set_ylabel('Temp (C)')
    ax3.set_xlabel('Time (s)')
    ax3.grid(True)
    ax3.legend(loc='upper left')
    ax3_temp.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(f"{output_prefix}_results.png")
    plt.close()
    print(f"Generated {output_prefix}_results.png")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python generate_graphs.py <csv_file> <output_prefix>")
    else:
        generate_graphs(sys.argv[1], sys.argv[2])
