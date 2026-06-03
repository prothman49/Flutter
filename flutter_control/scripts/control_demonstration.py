#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Advisor Demonstration Script
Simple automated sequence: No Wind → Wind (Flutter) → Wind + Control
Creates clean magnitude vs time plot
"""

import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from flutter_control_node import PhaseLockNode
import json

def advisor_demonstration(node, phase_offset=145, no_wind_duration=30, flutter_duration=60, control_duration=60, flutter_return_duration=60):
    """
    Automated demonstration sequence
    
    Phase 1: No wind (baseline) - duration: no_wind_duration
    Phase 2: Wind on, no control (flutter) - duration: flutter_duration  
    Phase 3: Wind on, control active (suppression) - duration: control_duration
    Phase 4: Wind on, no control (flutter returns) - duration: flutter_return_duration
    """
    
    node.set_phase(phase_offset)
    
    # Storage
    timestamps = []
    magnitudes = []
    control_voltages = []
    phase_labels = []  # 1, 2, or 3
    
    print("\n" + "="*70)
    print("ADVISOR DEMONSTRATION - AUTOMATED SEQUENCE")
    print("="*70)
    print(f"Phase offset: {phase_offset}°")
    print(f"Phase 1 (No Wind):     {no_wind_duration}s")
    print(f"Phase 2 (Flutter):     {flutter_duration}s")
    print(f"Phase 3 (Control):     {control_duration}s")
    print(f"Phase 4 (Flutter Return): {flutter_return_duration}s")
    print(f"Total duration:        {no_wind_duration + flutter_duration + control_duration + flutter_return_duration}s ({(no_wind_duration + flutter_duration + control_duration + flutter_return_duration)/60:.1f} min)")
    print("="*70)
    
    start_time = time.time()
    
    # ============================================================
    # PHASE 1: NO WIND (Baseline)
    # ============================================================
    input("\n▶ PHASE 1: Ensure airflow is OFF. Press ENTER to start recording...")
    
    print(f"\n{'='*70}")
    print(f"PHASE 1: NO WIND - Recording baseline for {no_wind_duration}s")
    print(f"{'='*70}")
    
    phase_start = time.time()
    
    while (time.time() - phase_start) < no_wind_duration:
        node.update()
        
        current_time = time.time() - start_time
        
        if len(node.magnitude_history) > 0:
            timestamps.append(current_time)
            magnitudes.append(node.magnitude_history[-1])
            control_voltages.append(0.0)
            phase_labels.append(1)
            
            if len(timestamps) % 20 == 0:
                elapsed = time.time() - phase_start
                remaining = no_wind_duration - elapsed
                print(f"  Phase 1: {elapsed:.0f}s / {no_wind_duration}s (remaining: {remaining:.0f}s) | Mag={magnitudes[-1]:.1f}", end='\r')
        
        time.sleep(0.05)
    
    phase1_avg = np.mean([m for m, p in zip(magnitudes, phase_labels) if p == 1])
    print(f"\n✓ Phase 1 complete. Avg magnitude: {phase1_avg:.1f}")
    
    # ============================================================
    # PHASE 2: WIND ON, NO CONTROL (Flutter)
    # ============================================================
    input(f"\n▶ PHASE 2: TURN AIRFLOW ON NOW. Press ENTER when airflow is on...")
    
    print(f"\n{'='*70}")
    print(f"PHASE 2: FLUTTER - Recording flutter for {flutter_duration}s")
    print(f"{'='*70}")
    
    node.control_enabled = False
    node.stop_waveform()
    
    phase_start = time.time()
    
    while (time.time() - phase_start) < flutter_duration:
        node.update()
        
        current_time = time.time() - start_time
        
        if len(node.magnitude_history) > 0:
            timestamps.append(current_time)
            magnitudes.append(node.magnitude_history[-1])
            control_voltages.append(0.0)
            phase_labels.append(2)
            
            if len(timestamps) % 20 == 0:
                elapsed = time.time() - phase_start
                remaining = flutter_duration - elapsed
                print(f"  Phase 2: {elapsed:.0f}s / {flutter_duration}s (remaining: {remaining:.0f}s) | Mag={magnitudes[-1]:.1f}", end='\r')
        
        time.sleep(0.05)
    
    phase2_mags = [m for m, p in zip(magnitudes, phase_labels) if p == 2]
    
    # Use second half of Phase 2 for steady-state flutter (flutter has time to develop)
    if len(phase2_mags) > 20:
        steady_state_start = len(phase2_mags) // 2  # Second half
        phase2_avg = np.mean(phase2_mags[steady_state_start:])
        print(f"\n✓ Phase 2 complete. Steady-state avg magnitude: {phase2_avg:.1f} (using last {len(phase2_mags) - steady_state_start} samples)")
    else:
        phase2_avg = np.mean(phase2_mags)
        print(f"\n✓ Phase 2 complete. Avg magnitude: {phase2_avg:.1f}")
    
    # ============================================================
    # PHASE 3: WIND + CONTROL (Suppression)
    # ============================================================
    print(f"\n{'='*70}")
    print(f"PHASE 3: CONTROL - Activating control for {control_duration}s")
    print(f"{'='*70}")
    
    # Detect frequency by running updates and extracting dominant frequency
    print("Detecting flutter frequency...")
    detection_start = time.time()
    detected_freq = None
    
    # Run updates for a few seconds to get a stable frequency reading
    freq_samples = []
    
    while (time.time() - detection_start) < 5:
        node.update()
        
        # Extract dominant frequency from buffer
        if len(node.buffer) >= node.buffer_size:
            data = np.array(node.buffer)
            freqs, magnitudes_fft = node.compute_fft(data)
            idx = np.argmax(magnitudes_fft[1:]) + 1  # Skip DC component
            dom_freq = freqs[idx]
            freq_samples.append(dom_freq)
        
        time.sleep(0.1)
    
    if len(freq_samples) > 0:
        detected_freq = np.median(freq_samples)  # Use median for stability
        print(f"✓ Flutter detected at {detected_freq:.2f} Hz")
    else:
        detected_freq = 5.8
        print(f"⚠ No flutter frequency detected, using default {detected_freq} Hz")
    
    # Activate control
    node.control_enabled = True
    node.start_waveform(detected_freq)
    
    phase_start = time.time()
    
    while (time.time() - phase_start) < control_duration:
        node.update()
        
        current_time = time.time() - start_time
        
        if len(node.magnitude_history) > 0:
            timestamps.append(current_time)
            magnitudes.append(node.magnitude_history[-1])
            phase_labels.append(3)
            
            # Get actual square wave control voltage (0-4.2V)
            if node.waveform_running:
                # Calculate phase with offset
                phase = (2 * np.pi * node.waveform_freq * current_time + np.deg2rad(phase_offset)) % (2 * np.pi)
                # Square wave: high (4.2V) for first half of cycle, low (0V) for second half
                if phase < np.pi:
                    control_v = 4.2
                else:
                    control_v = 0.0
                control_voltages.append(control_v)
            else:
                control_voltages.append(0.0)
            
            if len(timestamps) % 20 == 0:
                elapsed = time.time() - phase_start
                remaining = control_duration - elapsed
                print(f"  Phase 3: {elapsed:.0f}s / {control_duration}s (remaining: {remaining:.0f}s) | Mag={magnitudes[-1]:.1f}", end='\r')
        
        time.sleep(0.05)
    
    phase3_mags = [m for m, p in zip(magnitudes, phase_labels) if p == 3]
    
    # Use second half of Phase 3 for steady-state average (control has time to work)
    if len(phase3_mags) > 20:
        steady_state_start = len(phase3_mags) // 2  # Second half
        phase3_avg = np.mean(phase3_mags[steady_state_start:])
        print(f"\n✓ Phase 3 complete. Steady-state avg magnitude: {phase3_avg:.1f} (using last {len(phase3_mags) - steady_state_start} samples)")
    else:
        phase3_avg = np.mean(phase3_mags)
        print(f"\n✓ Phase 3 complete. Avg magnitude: {phase3_avg:.1f}")
    
    # ============================================================
    # PHASE 4: WIND ON, NO CONTROL (Flutter Returns)
    # ============================================================
    print(f"\n{'='*70}")
    print(f"PHASE 4: FLUTTER RETURN - Deactivating control for {flutter_return_duration}s")
    print(f"{'='*70}")
    
    # Deactivate control
    node.control_enabled = False
    node.stop_waveform()
    
    phase_start = time.time()
    
    while (time.time() - phase_start) < flutter_return_duration:
        node.update()
        
        current_time = time.time() - start_time
        
        if len(node.magnitude_history) > 0:
            timestamps.append(current_time)
            magnitudes.append(node.magnitude_history[-1])
            control_voltages.append(0.0)
            phase_labels.append(4)
            
            if len(timestamps) % 20 == 0:
                elapsed = time.time() - phase_start
                remaining = flutter_return_duration - elapsed
                print(f"  Phase 4: {elapsed:.0f}s / {flutter_return_duration}s (remaining: {remaining:.0f}s) | Mag={magnitudes[-1]:.1f}", end='\r')
        
        time.sleep(0.05)
    
    phase4_mags = [m for m, p in zip(magnitudes, phase_labels) if p == 4]
    
    # Use second half for steady-state
    if len(phase4_mags) > 20:
        steady_state_start = len(phase4_mags) // 2
        phase4_avg = np.mean(phase4_mags[steady_state_start:])
        print(f"\n✓ Phase 4 complete. Steady-state avg magnitude: {phase4_avg:.1f} (using last {len(phase4_mags) - steady_state_start} samples)")
    else:
        phase4_avg = np.mean(phase4_mags)
        print(f"\n✓ Phase 4 complete. Avg magnitude: {phase4_avg:.1f}")
    
    # Stop control
    node.control_enabled = False
    node.stop_waveform()
    
    # Summary
    print("\n" + "="*70)
    print("DEMONSTRATION COMPLETE")
    print("="*70)
    print(f"Phase 1 (No Wind):       {phase1_avg:.1f}")
    print(f"Phase 2 (Flutter):       {phase2_avg:.1f}")
    print(f"Phase 3 (Control):       {phase3_avg:.1f}")
    print(f"Phase 4 (Flutter Return): {phase4_avg:.1f}")
    
    if phase2_avg > 0:
        reduction = ((phase2_avg - phase3_avg) / phase2_avg) * 100
        print(f"\nControl effectiveness (Phase 2→3): {reduction:.1f}% magnitude reduction")
    
    if phase4_avg > 0 and phase3_avg > 0:
        return_increase = ((phase4_avg - phase3_avg) / phase3_avg) * 100
        print(f"Flutter return (Phase 3→4): {return_increase:+.1f}% magnitude change")
        print(f"  → Demonstrates control was actively suppressing flutter")
    
    print("="*70)
    
    return {
        'timestamps': timestamps,
        'magnitudes': magnitudes,
        'control_voltages': control_voltages,
        'phase_labels': phase_labels,
        'phase_offset': phase_offset,
        'durations': {
            'no_wind': no_wind_duration,
            'flutter': flutter_duration,
            'control': control_duration,
            'flutter_return': flutter_return_duration
        }
    }


def create_advisor_plot(data, output_path='advisor_demonstration.png'):
    """Create clean publication plot for advisor"""
    
    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(2, 1, height_ratios=[2, 1], hspace=0.3)
    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    
    timestamps = np.array(data['timestamps'])
    magnitudes = np.array(data['magnitudes'])
    control_voltages = np.array(data['control_voltages'])
    phase_labels = np.array(data['phase_labels'])
    
    # Find actual phase transitions from the data
    phase_transitions = [0]  # Start at 0
    for i in range(1, len(phase_labels)):
        if phase_labels[i] != phase_labels[i-1]:
            # Phase changed - use midpoint between samples
            transition_time = (timestamps[i-1] + timestamps[i]) / 2
            phase_transitions.append(transition_time)
    phase_transitions.append(timestamps[-1])  # End time
    
    # Extract boundary times (should have 5 values for 4 phases: start, t1, t2, t3, end)
    if len(phase_transitions) >= 5:
        t0 = phase_transitions[0]
        t1 = phase_transitions[1]
        t2 = phase_transitions[2]
        t3 = phase_transitions[3]
        t4 = phase_transitions[4]
    else:
        # Fallback to duration-based calculation if phase_labels are wrong
        durations = data['durations']
        t0 = 0
        t1 = durations['no_wind']
        t2 = t1 + durations['flutter']
        t3 = t2 + durations['control']
        t4 = t3 + durations['flutter_return']
    
    # Top: Magnitude
    ax1.plot(timestamps, magnitudes, color='darkblue', linewidth=1.5)
    
    # Background shading using actual phase transitions
    ax1.axvspan(t0, t1, alpha=0.15, color='gray', label='No Wind (Baseline)')
    ax1.axvspan(t1, t2, alpha=0.15, color='red', label='Wind ON (Flutter)')
    ax1.axvspan(t2, t3, alpha=0.15, color='green', label='Wind + Control (Suppression)')
    ax1.axvspan(t3, t4, alpha=0.15, color='orange', label='Wind ON (Flutter Returns)')
    
    # Phase dividers at actual transitions
    ax1.axvline(x=t1, color='black', linestyle='--', linewidth=2, alpha=0.7)
    ax1.axvline(x=t2, color='black', linestyle='--', linewidth=2, alpha=0.7)
    ax1.axvline(x=t3, color='black', linestyle='--', linewidth=2, alpha=0.7)
    
    # Calculate averages
    for phase in [1, 2, 3, 4]:
        phase_times = timestamps[phase_labels == phase]
        phase_mags = magnitudes[phase_labels == phase]
        if len(phase_mags) > 0:
            avg = np.mean(phase_mags)
            t_start = phase_times[0]
            t_end = phase_times[-1]
            color = 'gray' if phase == 1 else 'red' if phase == 2 else 'green' if phase == 3 else 'orange'
            ax1.plot([t_start, t_end], [avg, avg], 
                    color=color, linewidth=3, linestyle=':', alpha=0.8)
            ax1.text((t_start + t_end)/2, avg + 8, f'Avg: {avg:.1f}',
                    ha='center', fontsize=11, fontweight='bold',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    ax1.set_ylabel('Magnitude', fontsize=14, fontweight='bold')
    ax1.set_title(f'Flutter Control Demonstration (Phase Offset = {data["phase_offset"]}°)', 
                  fontsize=16, fontweight='bold')
    ax1.grid(True, alpha=0.3, linewidth=1.5)
    ax1.legend(loc='upper right', fontsize=11)
    ax1.tick_params(labelsize=12)
    
    # Bottom: Control voltage with reference signal
    # Create twin axis for flutter magnitude reference
    ax2_twin = ax2.twinx()
    
    # Plot control voltage (square wave)
    ax2.plot(timestamps, control_voltages, color='darkgreen', linewidth=1.5, label='Control Voltage (Square Wave)', alpha=0.9)
    
    # Plot filtered flutter signal in Phase 3 to show phase relationship
    phase3_times = timestamps[phase_labels == 3]
    phase3_mags = magnitudes[phase_labels == 3]
    if len(phase3_times) > 0:
        # Normalize magnitude to similar scale for visual comparison
        mag_normalized = (phase3_mags - np.mean(phase3_mags)) / np.std(phase3_mags) * 30 + 120
        ax2_twin.plot(phase3_times, mag_normalized, color='blue', linewidth=1, 
                     label='Flutter Magnitude (normalized)', alpha=0.6, linestyle='--')
        ax2_twin.set_ylabel('Flutter Magnitude (normalized)', fontsize=12, fontweight='bold', color='blue')
        ax2_twin.tick_params(axis='y', labelcolor='blue', labelsize=10)
    
    # Background shading using actual transitions
    ax2.axvspan(t0, t1, alpha=0.15, color='gray')
    ax2.axvspan(t1, t2, alpha=0.15, color='red')
    ax2.axvspan(t2, t3, alpha=0.15, color='green')
    ax2.axvspan(t3, t4, alpha=0.15, color='orange')
    
    # Phase dividers at actual transitions
    ax2.axvline(x=t1, color='black', linestyle='--', linewidth=2, alpha=0.7)
    ax2.axvline(x=t2, color='black', linestyle='--', linewidth=2, alpha=0.7)
    ax2.axvline(x=t3, color='black', linestyle='--', linewidth=2, alpha=0.7)
    
    ax2.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
    
    ax2.set_xlabel('Time (seconds)', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Control Voltage (V)', fontsize=14, fontweight='bold', color='darkgreen')
    ax2.set_title(f'Piezoelectric Control Signal (Square Wave, Phase Offset = {data["phase_offset"]}°)', 
                  fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3, linewidth=1.5)
    ax2.tick_params(axis='y', labelcolor='darkgreen', labelsize=12)
    ax2.set_ylim(-0.5, 4.7)  # Show full 0-4.2V range
    
    # Combine legends from both axes
    lines1, labels1 = ax2.get_legend_handles_labels()
    if len(phase3_times) > 0:
        lines2, labels2 = ax2_twin.get_legend_handles_labels()
        ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=10)
    
    # Add phase labels at bottom using actual phase centers
    y_pos = ax2.get_ylim()[0] + (ax2.get_ylim()[1] - ax2.get_ylim()[0]) * 0.85
    ax2.text((t0+t1)/2, y_pos, 'No Wind', ha='center', fontsize=12, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='gray', alpha=0.3))
    ax2.text((t1+t2)/2, y_pos, 'Flutter', ha='center', fontsize=12, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='red', alpha=0.3))
    ax2.text((t2+t3)/2, y_pos, 'Control ON', ha='center', fontsize=12, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='green', alpha=0.3))
    ax2.text((t3+t4)/2, y_pos, 'Flutter Returns', ha='center', fontsize=12, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='orange', alpha=0.3))
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\nFigure saved: {output_path}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Flutter Control Demonstration')
    parser.add_argument('--gui', action='store_true', help='Enable live GUI display')
    args = parser.parse_args()
    
    rospy.init_node('advisor_demonstration', anonymous=True)
    
    node = PhaseLockNode(enable_gui=args.gui)
    
    # Start ROS spin in background
    import threading
    t = threading.Thread(target=rospy.spin)
    t.daemon = True
    t.start()
    
    print("Waiting for optic flow data...")
    while len(node.buffer) < 10 and not rospy.is_shutdown():
        time.sleep(0.1)
    
    print("\n" + "="*70)
    print("ADVISOR DEMONSTRATION")
    print("No Wind → Flutter → Control (Automated)")
    print("="*70)
    
    # Parameters
    phase_offset = float(input("\nPhase offset (deg) [145]: ") or "145")
    no_wind = int(input("No wind duration (s) [30]: ") or "30")
    flutter = int(input("Flutter duration (s) [60]: ") or "60")
    control = int(input("Control duration (s) [60]: ") or "60")
    flutter_return = int(input("Flutter return duration (s) [60]: ") or "60")
    
    try:
        # Run demonstration
        data = advisor_demonstration(node, phase_offset, no_wind, flutter, control, flutter_return)
        
        # Create plot
        create_advisor_plot(data, '../data/results/advisor_demonstration.png')
        
        # Save data
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        data_file = f'../data/results/advisor_demo_{timestamp}.json'
        
        save_data = {
            'timestamps': [float(t) for t in data['timestamps']],
            'magnitudes': [float(m) for m in data['magnitudes']],
            'control_voltages': [float(v) for v in data['control_voltages']],
            'phase_labels': [int(p) for p in data['phase_labels']],
            'phase_offset': data['phase_offset'],
            'durations': data['durations']
        }
        
        with open(data_file, 'w') as f:
            json.dump(save_data, f, indent=2)
        
        print(f"Data saved: {data_file}")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    finally:
        node.cleanup()


if __name__ == '__main__':
    main()