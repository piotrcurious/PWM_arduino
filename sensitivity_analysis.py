import subprocess
import pandas as pd
import numpy as np
import os
import sys

def run_simulation(test_exe, l_val, c_val):
    # Set environment variables for the simulator if possible, or use a config file
    with open("sim_config.txt", "w") as f:
        f.write(f"L={l_val}\nC={c_val}\n")

    csv_file = f"tmp_{test_exe}_{l_val}_{c_val}.csv"
    subprocess.run([f"./{test_exe}", csv_file], stdout=subprocess.DEVNULL)
    return csv_file

def analyze_results(csv_file):
    if not os.path.exists(csv_file): return 1.0, 0.0
    df = pd.read_csv(csv_file)
    if df.empty: return 1.0, 0.0

    # Define settling time as the time it takes to stay within 10% of target (12V)
    target_v = 12.0
    final_v = df['v_out'].iloc[-1]

    error = np.abs(df['v_out'] - target_v)
    # Find last time error was > 1V
    settling_idx = np.where(error > 1.0)[0]
    settling_time = df['time'].iloc[settling_idx[-1]] if len(settling_idx) > 0 else 0.0

    overshoot = (df['v_out'].max() - target_v) / target_v * 100
    if overshoot < 0: overshoot = 0
    return settling_time, overshoot

def main():
    if len(sys.argv) < 2:
        print("Usage: python sensitivity_analysis.py <test_exe>")
        return

    test_exe = sys.argv[1]
    l_range = [5e-3, 10e-3, 15e-3]
    c_range = [5e-3, 10e-3, 15e-3]

    results = []
    for l in l_range:
        for c in c_range:
            csv = run_simulation(test_exe, l, c)
            st, os_val = analyze_results(csv)
            results.append({'L': l, 'C': c, 'SettlingTime': st, 'Overshoot': os_val})
            if os.path.exists(csv): os.remove(csv)

    df_results = pd.DataFrame(results)
    print(f"--- Sensitivity Analysis for {test_exe} ---")
    print(df_results)
    df_results.to_csv(f"{test_exe}_sensitivity.csv", index=False)

if __name__ == "__main__":
    main()
