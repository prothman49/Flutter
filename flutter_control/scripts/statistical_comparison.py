#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Statistical Comparison: Control ON vs OFF
Runs multiple short trials at fixed phase to generate statistical data
"""

import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
from datetime import datetime
from flutter_control_node import PhaseLockNode
import json

def run_single_trial(node, control_enabled, duration=30):
    """
    Run a single trial with control enabled or disabled
    
    Args:
        node: PhaseLockNode instance
        control_enabled: True for control ON, False for control OFF
        duration: Duration of trial in seconds
        
    Returns:
        Average magnitude during trial
    """
    
    node.control_enabled = control_enabled
    
    if control_enabled and node.waveform_running:
        pass  # Already running
    elif control_enabled:
        # Start waveform at detected frequency
        freq_samples = []
        detect_start = time.time()
        while (time.time() - detect_start) < 3:
            node.update()
            if len(node.buffer) >= node.buffer_size:
                data = np.array(node.buffer)
                freqs, mags = node.compute_fft(data)
                idx = np.argmax(mags[1:]) + 1
                freq_samples.append(freqs[idx])
            time.sleep(0.1)
        
        if freq_samples:
            freq = np.median(freq_samples)
            node.start_waveform(freq)
    else:
        node.stop_waveform()
    
    # Collect data
    magnitudes = []
    start_time = time.time()
    
    while (time.time() - start_time) < duration:
        node.update()
        
        if len(node.magnitude_history) > 0:
            magnitudes.append(node.magnitude_history[-1])
        
        elapsed = time.time() - start_time
        if len(magnitudes) % 20 == 0 and len(magnitudes) > 0:
            print(f"  Trial: {elapsed:.0f}s / {duration}s | Mag={magnitudes[-1]:.1f}", end='\r')
        
        time.sleep(0.05)
    
    if len(magnitudes) > 0:
        avg_mag = np.mean(magnitudes)
    else:
        avg_mag = 0
    
    print()  # New line
    return avg_mag, magnitudes


def run_statistical_trials(node, phase_offset, n_trials=10, trial_duration=30, settling_time=5):
    """
    Run multiple trials for statistical comparison
    
    Args:
        node: PhaseLockNode instance
        phase_offset: Phase offset in degrees
        n_trials: Number of trials per condition
        trial_duration: Duration of each trial in seconds
        settling_time: Wait time between trials
        
    Returns:
        Dictionary with control ON and OFF data
    """
    
    node.set_phase(phase_offset)
    
    print("\n" + "="*70)
    print("STATISTICAL COMPARISON: CONTROL ON vs OFF")
    print("="*70)
    print(f"Phase offset: {phase_offset}°")
    print(f"Trials per condition: {n_trials}")
    print(f"Trial duration: {trial_duration}s")
    print(f"Settling time: {settling_time}s")
    print(f"Total duration: ~{(n_trials * 2 * (trial_duration + settling_time))/60:.0f} minutes")
    print("="*70)
    
    control_off_data = []
    control_on_data = []
    
    # Run control OFF trials
    print(f"\n{'='*70}")
    print(f"CONTROL OFF TRIALS (Flutter baseline)")
    print(f"{'='*70}")
    
    for i in range(n_trials):
        print(f"\nTrial {i+1}/{n_trials} (Control OFF):")
        avg_mag, _ = run_single_trial(node, control_enabled=False, duration=trial_duration)
        control_off_data.append(avg_mag)
        print(f"  ✓ Average magnitude: {avg_mag:.1f}")
        
        if i < n_trials - 1:  # Don't wait after last trial
            print(f"  Waiting {settling_time}s...")
            time.sleep(settling_time)
    
    # Run control ON trials
    print(f"\n{'='*70}")
    print(f"CONTROL ON TRIALS (Suppression)")
    print(f"{'='*70}")
    
    for i in range(n_trials):
        print(f"\nTrial {i+1}/{n_trials} (Control ON):")
        avg_mag, _ = run_single_trial(node, control_enabled=True, duration=trial_duration)
        control_on_data.append(avg_mag)
        print(f"  ✓ Average magnitude: {avg_mag:.1f}")
        
        if i < n_trials - 1:
            print(f"  Waiting {settling_time}s...")
            time.sleep(settling_time)
    
    # Stop control
    node.control_enabled = False
    node.stop_waveform()
    
    # Calculate statistics
    print("\n" + "="*70)
    print("RESULTS SUMMARY")
    print("="*70)
    
    control_off_mean = np.mean(control_off_data)
    control_off_std = np.std(control_off_data, ddof=1)
    
    control_on_mean = np.mean(control_on_data)
    control_on_std = np.std(control_on_data, ddof=1)
    
    reduction = ((control_off_mean - control_on_mean) / control_off_mean) * 100 if control_off_mean > 0 else 0
    
    t_stat, p_value = stats.ttest_ind(control_off_data, control_on_data)
    
    print(f"\nControl OFF (Flutter):")
    print(f"  Mean: {control_off_mean:.1f} ± {control_off_std:.1f}")
    print(f"  Data: {[f'{x:.1f}' for x in control_off_data]}")
    
    print(f"\nControl ON (Suppression):")
    print(f"  Mean: {control_on_mean:.1f} ± {control_on_std:.1f}")
    print(f"  Data: {[f'{x:.1f}' for x in control_on_data]}")
    
    print(f"\nControl Effectiveness:")
    print(f"  Magnitude Reduction: {reduction:.1f}%")
    print(f"  t-statistic: {t_stat:.3f}")
    print(f"  p-value: {p_value:.4f}")
    
    if p_value < 0.05:
        print(f"  → Statistically significant! (p < 0.05)")
    else:
        print(f"  → Not statistically significant (p ≥ 0.05)")
    
    print("="*70)
    
    return {
        'control_off': control_off_data,
        'control_on': control_on_data,
        'phase_offset': phase_offset,
        'trial_duration': trial_duration,
        'n_trials': n_trials
    }


def create_comparison_plot(data, output_path='statistical_comparison.png'):
    """Create scatter plot with statistics"""
    
    control_off = np.array(data['control_off'])
    control_on = np.array(data['control_on'])
    
    control_off_mean = np.mean(control_off)
    control_on_mean = np.mean(control_on)
    
    control_off_std = np.std(control_off, ddof=1)
    control_on_std = np.std(control_on, ddof=1)
    
    # 95% CI
    control_off_ci = stats.t.interval(0.95, len(control_off)-1, 
                                     loc=control_off_mean, 
                                     scale=control_off_std/np.sqrt(len(control_off)))
    
    control_on_ci = stats.t.interval(0.95, len(control_on)-1, 
                                    loc=control_on_mean, 
                                    scale=control_on_std/np.sqrt(len(control_on)))
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Add jitter
    def add_jitter(n, width=0.15):
        return np.random.uniform(-width, width, n)
    
    # Plot data
    x_off = np.zeros(len(control_off)) + add_jitter(len(control_off))
    ax.scatter(x_off, control_off, alpha=0.6, s=100, color='red', 
              edgecolors='darkred', linewidth=1.5, label='Control OFF', zorder=3)
    
    x_on = np.ones(len(control_on)) + add_jitter(len(control_on))
    ax.scatter(x_on, control_on, alpha=0.6, s=100, color='green', 
              edgecolors='darkgreen', linewidth=1.5, label='Control ON', zorder=3)
    
    # Means
    ax.plot([-0.3, 0.3], [control_off_mean, control_off_mean], 
           color='darkred', linewidth=3, label=f'Mean (OFF): {control_off_mean:.1f}', zorder=4)
    ax.plot([0.7, 1.3], [control_on_mean, control_on_mean], 
           color='darkgreen', linewidth=3, label=f'Mean (ON): {control_on_mean:.1f}', zorder=4)
    
    # 95% CI boxes
    ax.add_patch(plt.Rectangle((-0.3, control_off_ci[0]), 0.6, control_off_ci[1] - control_off_ci[0],
                              fill=False, edgecolor='darkred', linewidth=2, linestyle='--', 
                              label='95% CI (OFF)', zorder=2))
    
    ax.add_patch(plt.Rectangle((0.7, control_on_ci[0]), 0.6, control_on_ci[1] - control_on_ci[0],
                              fill=False, edgecolor='darkgreen', linewidth=2, linestyle='--', 
                              label='95% CI (ON)', zorder=2))
    
    # Statistics
    reduction = ((control_off_mean - control_on_mean) / control_off_mean) * 100
    _, p_value = stats.ttest_ind(control_off, control_on)
    
    ax.text(0.5, ax.get_ylim()[1] * 0.95, 
           f'Magnitude Reduction: {reduction:.1f}%\np-value: {p_value:.4f}',
           ha='center', fontsize=12, fontweight='bold',
           bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.3))
    
    # Formatting
    ax.set_xticks([0, 1])
    ax.set_xticklabels(['Control OFF\n(Flutter)', 'Control ON\n(Suppression)'], 
                       fontsize=13, fontweight='bold')
    ax.set_ylabel('Flutter Magnitude', fontsize=14, fontweight='bold')
    ax.set_title(f'Flutter Control Effectiveness (Phase = {data["phase_offset"]}°)', 
                 fontsize=16, fontweight='bold')
    ax.set_xlim(-0.5, 1.5)
    ax.grid(True, alpha=0.3, axis='y', linewidth=1.5)
    ax.legend(loc='upper right', fontsize=10)
    
    # Sample sizes
    ax.text(0, ax.get_ylim()[0] + (ax.get_ylim()[1] - ax.get_ylim()[0]) * 0.02, 
           f'n={len(control_off)}', ha='center', fontsize=11, fontweight='bold')
    ax.text(1, ax.get_ylim()[0] + (ax.get_ylim()[1] - ax.get_ylim()[0]) * 0.02, 
           f'n={len(control_on)}', ha='center', fontsize=11, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\n✓ Figure saved: {output_path}")


def main():
    rospy.init_node('statistical_comparison', anonymous=True)
    
    node = PhaseLockNode(enable_gui=False)
    
    # Start ROS spin
    import threading
    t = threading.Thread(target=rospy.spin)
    t.daemon = True
    t.start()
    
    print("Waiting for optic flow data...")
    while len(node.buffer) < 10 and not rospy.is_shutdown():
        time.sleep(0.1)
    
    print("\n" + "="*70)
    print("STATISTICAL COMPARISON - CONTROL ON vs OFF")
    print("="*70)
    print("\nThis will run multiple short trials to generate statistical data")
    print("for comparing control effectiveness.")
    print("="*70)
    
    # Parameters
    phase_offset = float(input("\nPhase offset (deg) [255]: ") or "255")
    n_trials = int(input("Number of trials per condition [10]: ") or "10")
    trial_duration = int(input("Trial duration (s) [30]: ") or "30")
    settling_time = int(input("Settling time between trials (s) [5]: ") or "5")
    
    input("\n▶ Make sure AIRFLOW IS ON and flutter is established. Press Enter to start...")
    
    try:
        # Run trials
        data = run_statistical_trials(node, phase_offset, n_trials, trial_duration, settling_time)
        
        # Create plot
        create_comparison_plot(data, '../data/results/statistical_comparison.png')
        
        # Save data
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        data_file = f'../data/results/statistical_comparison_{timestamp}.json'
        
        save_data = {
            'control_off': [float(x) for x in data['control_off']],
            'control_on': [float(x) for x in data['control_on']],
            'phase_offset': data['phase_offset'],
            'trial_duration': data['trial_duration'],
            'n_trials': data['n_trials']
        }
        
        with open(data_file, 'w') as f:
            json.dump(save_data, f, indent=2)
        
        print(f"\nData saved: {data_file}")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        node.cleanup()


if __name__ == '__main__':
    main()