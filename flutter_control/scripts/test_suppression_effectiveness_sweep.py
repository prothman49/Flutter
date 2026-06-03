#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Suppression Effectiveness Sweep
Measures actual flutter suppression for each phase by comparing control ON vs OFF
"""

import rospy
import time
import numpy as np
import json
from datetime import datetime
from flutter_control_node import PhaseLockNode

def suppression_test_single_phase(node, phase, baseline_duration=30, controlled_duration=30, reset_duration=20):
    """
    Test suppression effectiveness at a single phase
    
    Returns dict with baseline and controlled metrics
    """
    print(f"\n[TEST] Phase = {phase} deg")
    print("-" * 60)
    
    # === PHASE 1: BASELINE (Control OFF) ===
    print("[1/3] Recording BASELINE (Control OFF)...")
    node.control_enabled = False
    node.stop_waveform()
    
    # Wait for flutter to establish
    print("  Waiting for flutter to establish...")
    flutter_established = False
    timeout = 60  # 60 second timeout
    start_wait = time.time()
    
    while not flutter_established and (time.time() - start_wait < timeout):
        node.update()
        time.sleep(0.1)
        
        if len(node.magnitude_history) > 10:
            recent_mags = node.magnitude_history[-10:]
            if np.mean(recent_mags) > node.flutter_threshold:
                flutter_established = True
                print(f"  Flutter established! Magnitude = {np.mean(recent_mags):.1f}")
    
    if not flutter_established:
        print("  WARNING: Flutter did not establish within timeout!")
        print("  Continuing anyway... (manually disturb wing if needed)")
        time.sleep(5)
    
    # Record baseline
    baseline_magnitudes = []
    baseline_flutter_time = 0
    
    print(f"  Recording baseline for {baseline_duration}s...")
    start_time = time.time()
    last_update = start_time
    
    while (time.time() - start_time) < baseline_duration:
        node.update()
        
        # Record data every 0.2 seconds
        if time.time() - last_update > 0.2:
            if len(node.magnitude_history) > 0:
                current_mag = node.magnitude_history[-1]
                baseline_magnitudes.append(current_mag)
                
                if current_mag > node.flutter_threshold:
                    baseline_flutter_time += 0.2
            
            last_update = time.time()
        
        time.sleep(0.05)
    
    baseline_avg = np.mean(baseline_magnitudes) if baseline_magnitudes else 0
    baseline_flutter_pct = (baseline_flutter_time / baseline_duration) * 100
    
    print(f"  BASELINE: Avg magnitude = {baseline_avg:.1f}, Flutter time = {baseline_flutter_pct:.1f}%")
    
    # === PHASE 2: CONTROLLED (Control ON at test phase) ===
    print(f"[2/3] Recording CONTROLLED (Control ON at {phase}°)...")
    
    # Set phase and enable control
    node.set_phase(phase)
    node.control_enabled = True
    
    # Small delay for control to activate
    time.sleep(2)
    
    # Record controlled performance
    controlled_magnitudes = []
    controlled_flutter_time = 0
    
    print(f"  Recording controlled for {controlled_duration}s...")
    start_time = time.time()
    last_update = start_time
    
    while (time.time() - start_time) < controlled_duration:
        node.update()
        
        # Record data every 0.2 seconds
        if time.time() - last_update > 0.2:
            if len(node.magnitude_history) > 0:
                current_mag = node.magnitude_history[-1]
                controlled_magnitudes.append(current_mag)
                
                if current_mag > node.flutter_threshold:
                    controlled_flutter_time += 0.2
            
            last_update = time.time()
        
        time.sleep(0.05)
    
    controlled_avg = np.mean(controlled_magnitudes) if controlled_magnitudes else 0
    controlled_flutter_pct = (controlled_flutter_time / controlled_duration) * 100
    
    print(f"  CONTROLLED: Avg magnitude = {controlled_avg:.1f}, Flutter time = {controlled_flutter_pct:.1f}%")
    
    # Calculate suppression
    if baseline_avg > 0:
        magnitude_reduction_pct = ((baseline_avg - controlled_avg) / baseline_avg) * 100
        flutter_time_reduction_pct = ((baseline_flutter_pct - controlled_flutter_pct) / baseline_flutter_pct) * 100 if baseline_flutter_pct > 0 else 0
    else:
        magnitude_reduction_pct = 0
        flutter_time_reduction_pct = 0
    
    print(f"  SUPPRESSION: Magnitude reduction = {magnitude_reduction_pct:.1f}%, Flutter time reduction = {flutter_time_reduction_pct:.1f}%")
    
    # === PHASE 3: RESET (Control OFF, wait for flutter to reset) ===
    print(f"[3/3] Resetting (Control OFF for {reset_duration}s)...")
    node.control_enabled = False
    node.stop_waveform()
    time.sleep(reset_duration)
    
    return {
        'phase': phase,
        'baseline': {
            'avg_magnitude': float(baseline_avg),
            'flutter_percent': float(baseline_flutter_pct),
            'magnitudes': [float(m) for m in baseline_magnitudes]
        },
        'controlled': {
            'avg_magnitude': float(controlled_avg),
            'flutter_percent': float(controlled_flutter_pct),
            'magnitudes': [float(m) for m in controlled_magnitudes]
        },
        'suppression': {
            'magnitude_reduction_percent': float(magnitude_reduction_pct),
            'flutter_time_reduction_percent': float(flutter_time_reduction_pct)
        }
    }


def suppression_sweep(node, phases, baseline_duration=30, controlled_duration=30, reset_duration=20):
    """
    Run suppression effectiveness sweep across multiple phases
    """
    results = []
    
    print("\n" + "="*60)
    print("SUPPRESSION EFFECTIVENESS SWEEP")
    print("="*60)
    print(f"Phases to test: {phases}")
    print(f"Baseline duration: {baseline_duration}s")
    print(f"Controlled duration: {controlled_duration}s")
    print(f"Reset duration: {reset_duration}s")
    print(f"Total time per phase: {baseline_duration + controlled_duration + reset_duration}s")
    print(f"Estimated total time: {len(phases) * (baseline_duration + controlled_duration + reset_duration) / 60:.1f} minutes")
    print("="*60)
    
    for i, phase in enumerate(phases):
        print(f"\n[PROGRESS: {i+1}/{len(phases)}]")
        
        result = suppression_test_single_phase(
            node, 
            phase, 
            baseline_duration=baseline_duration,
            controlled_duration=controlled_duration,
            reset_duration=reset_duration
        )
        
        results.append(result)
    
    return results


def print_summary(results):
    """Print summary of suppression sweep results"""
    print("\n" + "="*80)
    print("SUPPRESSION EFFECTIVENESS SWEEP RESULTS")
    print("="*80)
    print(f"{'Phase':>6} | {'Baseline':>10} | {'Controlled':>10} | {'Mag Red %':>10} | {'Flutter Red %':>12}")
    print("-"*80)
    
    # Sort by magnitude reduction (descending)
    sorted_results = sorted(results, key=lambda x: x['suppression']['magnitude_reduction_percent'], reverse=True)
    
    for result in sorted_results:
        phase = result['phase']
        baseline_mag = result['baseline']['avg_magnitude']
        controlled_mag = result['controlled']['avg_magnitude']
        mag_red = result['suppression']['magnitude_reduction_percent']
        flutter_red = result['suppression']['flutter_time_reduction_percent']
        
        print(f"{phase:>6.0f} | {baseline_mag:>10.1f} | {controlled_mag:>10.1f} | {mag_red:>10.1f} | {flutter_red:>12.1f}")
    
    print("="*80)
    
    # Find best and worst
    best = max(results, key=lambda x: x['suppression']['magnitude_reduction_percent'])
    worst = min(results, key=lambda x: x['suppression']['magnitude_reduction_percent'])
    
    print(f"\n✅ BEST PHASE: {best['phase']:.0f}° ({best['suppression']['magnitude_reduction_percent']:.1f}% reduction)")
    print(f"❌ WORST PHASE: {worst['phase']:.0f}° ({worst['suppression']['magnitude_reduction_percent']:.1f}% reduction)")
    print(f"📊 Difference: {best['suppression']['magnitude_reduction_percent'] - worst['suppression']['magnitude_reduction_percent']:.1f} percentage points")
    print("="*80)


def main():
    rospy.init_node('suppression_effectiveness_sweep', anonymous=True)
    
    # Create node without GUI for speed
    node = PhaseLockNode(enable_gui=False)
    
    # Start ROS spin in background
    import threading
    t = threading.Thread(target=rospy.spin)
    t.daemon = True
    t.start()
    
    print("Waiting for optic flow data...")
    while len(node.buffer) < 10 and not rospy.is_shutdown():
        time.sleep(0.1)
    
    print("\n" + "="*60)
    print("SUPPRESSION EFFECTIVENESS SWEEP")
    print("="*60)
    print("\nSelect sweep type:")
    print("  1. Coarse sweep (0, 45, 90, 135, 180, 225, 270, 315)")
    print("  2. Fine sweep (custom range)")
    print("  3. Custom phases")
    
    choice = input("\nEnter choice (1-3): ").strip()
    
    if choice == "1":
        phases = [0, 45, 90, 135, 180, 225, 270, 315]
    elif choice == "2":
        start = float(input("Start phase (deg): "))
        phases = [start + i * 15 for i in range(9)]  # 9 phases, 15° increments
    elif choice == "3":
        phases_str = input("Enter phases separated by commas (e.g., 0,90,180,270): ")
        phases = [float(p.strip()) for p in phases_str.split(',')]
    else:
        print("Invalid choice, using coarse sweep")
        phases = [0, 45, 90, 135, 180, 225, 270, 315]
    
    # Get duration parameters
    baseline_dur = int(input("\nBaseline duration per phase (default=30s): ") or "30")
    controlled_dur = int(input("Controlled duration per phase (default=30s): ") or "30")
    reset_dur = int(input("Reset duration between phases (default=20s): ") or "20")
    
    print("\n⚠️  IMPORTANT: Make sure AIRFLOW IS ON")
    input("Press Enter when ready to start...")
    
    try:
        # Run sweep
        results = suppression_sweep(
            node, 
            phases,
            baseline_duration=baseline_dur,
            controlled_duration=controlled_dur,
            reset_duration=reset_dur
        )
        
        # Print summary
        print_summary(results)
        
        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'../data/results/suppression_sweep_{timestamp}.json'
        
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"\nResults saved to: {filename}")
        
    except KeyboardInterrupt:
        print("\n\nSweep interrupted by user")
    finally:
        node.cleanup()


if __name__ == '__main__':
    main()