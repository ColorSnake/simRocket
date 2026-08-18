import csv
import matplotlib.pyplot as plt
import os
import argparse
import math

def generate_plots(csv_path, output_dir):
    if not os.path.exists(csv_path):
        print(f"File {csv_path} not found.")
        return

    os.makedirs(output_dir, exist_ok=True)
    
    times = []
    pos_z = []
    vel_z = []
    vel_mag = []
    mass_kg = []
    cg_z = []
    thrust_z = []
    aero_z = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row['time_s']))
            pos_z.append(float(row['pos_z']))
            
            vz = float(row['vel_z'])
            vx = float(row['vel_x'])
            vy = float(row['vel_y'])
            vel_z.append(vz)
            vel_mag.append(math.sqrt(vx**2 + vy**2 + vz**2))
            
            mass_kg.append(float(row['mass_kg']))
            cg_z.append(float(row['cg_z']))
            thrust_z.append(float(row['thrust_z']))
            aero_z.append(float(row['aero_z']))

    # 1. Altitude (Z) over time
    plt.figure(figsize=(10, 5))
    plt.plot(times, pos_z, label='Altitude (Z)', color='blue')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.title('Altitude over Time')
    plt.grid(True)
    plt.legend()
    plt.savefig(os.path.join(output_dir, 'altitude.png'))
    plt.close()

    # 2. Velocity over time
    plt.figure(figsize=(10, 5))
    plt.plot(times, vel_z, label='Vertical Velocity (Z)', color='orange')
    plt.plot(times, vel_mag, label='Velocity Magnitude', color='red', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Velocity over Time')
    plt.grid(True)
    plt.legend()
    plt.savefig(os.path.join(output_dir, 'velocity.png'))
    plt.close()

    # 3. Mass and CG over time
    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Mass (kg)', color='tab:blue')
    ax1.plot(times, mass_kg, color='tab:blue', label='Mass')
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    
    ax2 = ax1.twinx()
    ax2.set_ylabel('CG Z (m)', color='tab:green')
    ax2.plot(times, cg_z, color='tab:green', label='Center of Gravity Z', linestyle='--')
    ax2.tick_params(axis='y', labelcolor='tab:green')
    
    fig.tight_layout()
    plt.title('Mass and CG over Time')
    plt.grid(True)
    plt.savefig(os.path.join(output_dir, 'mass_cg.png'))
    plt.close()

    # 4. Thrust and Aero Forces
    plt.figure(figsize=(10, 5))
    plt.plot(times, thrust_z, label='Thrust Z', color='purple')
    plt.plot(times, aero_z, label='Aero Z (Drag/Lift)', color='brown')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Thrust and Aerodynamic Forces over Time')
    plt.grid(True)
    plt.legend()
    plt.savefig(os.path.join(output_dir, 'forces.png'))
    plt.close()

    print(f"Plots saved to {os.path.abspath(output_dir)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot simulation CSV logs")
    parser.add_argument("csv_file", nargs='?', default="logs/sim_log.csv", help="Path to CSV file")
    parser.add_argument("--out", default="logs/plots", help="Output directory for plots")
    args = parser.parse_args()

    generate_plots(args.csv_file, args.out)
