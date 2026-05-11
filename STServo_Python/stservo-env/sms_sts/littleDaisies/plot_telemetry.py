#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import pandas as pd
import matplotlib.pyplot as plt

def get_latest_csv(folder="datas"):
    """Finds the most recently created CSV file in the specified folder."""
    if not os.path.exists(folder):
        print(f"[ERROR] The folder '{folder}' does not exist.")
        return None
        
    list_of_files = glob.glob(f"{folder}/*.csv")
    if not list_of_files:
        print(f"[ERROR] No CSV files found in the '{folder}' folder.")
        return None
        
    # Return the file with the most recent creation/modification time
    return max(list_of_files, key=os.path.getctime)

def plot_telemetry(csv_file):
    print(f"Loading data from: {csv_file}")
    
    # Read the CSV file
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"[ERROR] Could not read CSV: {e}")
        return
        
    # Ensure data is sorted chronologically
    df = df.sort_values(by=['Timestamp'])
    
    # Create a 'Relative Time' column starting at 0 seconds
    start_time = df['Timestamp'].min()
    df['Relative_Time'] = df['Timestamp'] - start_time
    
    # The 6 metrics we want to plot
    metrics = ['Position', 'Speed', 'Load', 'Voltage', 'Temperature', 'Current']
    
    # Setup a 3x2 grid of plots. 'sharex=True' links the zoom/pan across all plots!
    fig, axes = plt.subplots(nrows=3, ncols=2, figsize=(15, 10), sharex=True)
    fig.suptitle(f'Tutan-Khamun Gripper Telemetry\nFile: {os.path.basename(csv_file)}', fontsize=16)
    
    # Flatten the 3x2 array of axes into a 1D list for easy looping
    axes = axes.flatten()
    
    # Find which Servo IDs are in the data (usually 1 and 2)
    servo_ids = sorted(df['ID'].unique())
    colors = {1: '#1f77b4', 2: '#ff7f0e'} # Blue for 1, Orange for 2
    
    for i, metric in enumerate(metrics):
        ax = axes[i]
        
        for servo_id in servo_ids:
            # Filter data for this specific servo
            servo_data = df[df['ID'] == servo_id]
            
            # Plot the data
            # marker='.' adds a small dot for every single data point
            # alpha=0.8 makes the lines slightly transparent so overlaps are visible
            ax.plot(servo_data['Relative_Time'], servo_data[metric], 
                    marker='.', linestyle='-', markersize=5, alpha=0.8,
                    color=colors.get(servo_id, 'black'), 
                    label=f'Servo {servo_id}')
        
        # Formatting for each subplot
        ax.set_title(metric, fontweight='bold')
        ax.set_ylabel(metric)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.legend(loc='best')
        
        # Only add the "Time (s)" label to the bottom two plots to keep it clean
        if i >= 4:
            ax.set_xlabel('Time (seconds)')
            
    # Adjust spacing so titles and labels don't overlap
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Show the interactive plot window
    print("Opening plot window... (Close the window to exit)")
    plt.show()

if __name__ == "__main__":
    import sys
    
    # If the user passed a specific file via terminal (e.g., python plot.py data.csv)
    if len(sys.argv) > 1:
        target_file = sys.argv[1]
    else:
        # Otherwise, auto-find the newest file
        target_file = get_latest_csv()
        
    if target_file:
        plot_telemetry(target_file)