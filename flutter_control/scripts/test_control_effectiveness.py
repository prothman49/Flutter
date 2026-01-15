#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test 3: Control Effectiveness Validation
Alternating control ON/OFF to measure effectiveness
"""

import rospy
import time
import numpy as np
import json
from datetime import datetime
from flutter_control_node import PhaseLockNode

def control_effectiveness_test(node, optimal_phase, cycle_duration=120, num_cycles=3):
    """
    Alternating control ON/OFF test
    
    Args:
        node: PhaseLockNode instance
        optimal_phase: Phase to use when control is ON (degrees)
        cycle_duration: Duration of each ON/OFF period (seconds)
        num_cycles: Number of ON/OFF cycles to run
    
    Returns:
        Dictionary with comparison statistics
    """
    print("\n" + "="*60)
    print("CONTROL EFFECTIVENESS TEST")
    print("="*60)
    print(f"Optimal phase: {optimal_phase} deg")
    print(f"Cycle duration: {cycle_duration}s (ON) / {cycle_duration}s (OFF)")
    print(f"Number of cycles: {num_cycles}")
    print(f"Total test time: {(num_cycles * 2 * cycle_duration) / 60:.1f} minutes")
    print("="*60 + "\n")
    
    input("Make sure AIRFLOW IS ON. Press Enter to start...")
    
    # Set optimal phase
    node.set_phase(optimal_phase)
    
    control_on_data = []
    control_off_data = []
    
    for cycle in range(num_cycles):
        print(f"\n[CYCLE {cycle + 1}/{num_cycles}]")
        
        # Control OFF period
        print(f"  Control OFF for {cycle_duration}s...")
        node.control_enabled = False
        
        start_time = time.time()
        off_magnitudes = []
        off_flutter_time = 0
        
        while time.time() - start_time < cycle_duration:
            node.update()
            if len(node.magnitude_history) > 0:
                mag = node.magnitude_history[-1]
                off_magnitudes.append(mag)
                if mag > node.flutter_threshold:
                    off_flutter_time += 0.2
            time.sleep(0.2)
        
        off_stats = {
            'avg_magnitude': float(np.mean(off_magnitudes)) if off_magnitudes else 0,
            'peak_magnitude': float(np.max(off_magnitudes)) if off_magnitudes else 0,
            'flutter_time': float(off_flutter_time),
            'flutter_percent': float((off_flutter_time / cycle_duration) * 100)
        }
        control_off_data.append(off_stats)
        
        print(f"    OFF: Avg={off_stats['avg_magnitude']:.1f}, Flutter={off_stats['flutter_percent']:.1f}%")
        
        # Control ON period
        print(f"  Control ON for {cycle_duration}s...")
        node.control_enabled = True
        
        start_time = time.time()
        on_magnitudes = []
        on_flutter_time = 0
        
        while time.time() - start_time < cycle_duration:
            node.update()
            if len(node.magnitude_history) > 0:
                mag = node.magnitude_history[-1]
                on_magnitudes.append(mag)
                if mag > node.flutter_threshold:
                    on_flutter_time += 0.2
            time.sleep(0.2)
        
        on_stats = {
            'avg_magnitude': float(np.mean(on_magnitudes)) if on_magnitudes else 0,
            'peak_magnitude': float(np.max(on_magnitudes)) if on_magnitudes else 0,
            'flutter_time': float(on_flutter_time),
            'flutter_percent': float((on_flutter_time / cycle_duration) * 100)
        }
        control_on_data.append(on_stats)
        
        print(f"    ON:  Avg={on_stats['avg_magnitude']:.1f}, Flutter={on_stats['flutter_percent']:.1f}%")
    
    # Calculate overall statistics
    off_avg_mag = np.mean([d['avg_magnitude'] for d in control_off_data])
    off_flutter_pct = np.mean([d['flutter_percent'] for d in control_off_data])
    
    on_avg_mag = np.mean([d['avg_magnitude'] for d in control_on_data])
    on_flutter_pct = np.mean([d['flutter_percent'] for d in control_on_data])
    
    mag_reduction = ((off_avg_mag - on_avg_mag) / off_avg_mag) * 100 if off_avg_mag > 0 else 0
    flutter_time_reduction = ((off_flutter_pct - on_flutter_pct) / off_flutter_pct) * 100 if off_flutter_pct > 0 else 0
    
    results = {
        'optimal_phase': float(optimal_phase),
        'control_off': {
            'avg_magnitude': float(off_avg_mag),
            'flutter_percent': float(off_flutter_pct),
            'cycles': control_off_data
        },
        'control_on': {
            'avg_magnitude': float(on_avg_mag),
            'flutter_percent': float(on_flutter_pct),
            'cycles': control_on_data
        },
        'effectiveness': {
            'magnitude_reduction_percent': float(mag_reduction),
            'flutter_time_reduction_percent': float(flutter_time_reduction)
        }
    }
    
    # Print summary
    print("\n" + "="*60)
    print("CONTROL EFFECTIVENESS SUMMARY")
    print("="*60)
    print(f"\nControl OFF (Baseline):")
    print(f"  Average magnitude: {off_avg_mag:.1f}")
    print(f"  Flutter time: {off_flutter_pct:.1f}%")
    print(f"\nControl ON (Phase = {optimal_phase} deg):")
    print(f"  Average magnitude: {on_avg_mag:.1f}")
    print(f"  Flutter time: {on_flutter_pct:.1f}%")
    print(f"\nImprovement:")
    print(f"  Magnitude reduction: {mag_reduction:.1f}%")
    print(f"  Flutter time reduction: {flutter_time_reduction:.1f}%")
    
    if mag_reduction > 30:
        print(f"\n✓ EXCELLENT: Control is highly effective!")
    elif mag_reduction > 15:
        print(f"\n✓ GOOD: Control shows clear improvement")
    elif mag_reduction > 5:
        print(f"\n⚠ MARGINAL: Control has small effect")
    else:
        print(f"\n✗ INEFFECTIVE: Control not working")
    
    print("="*60 + "\n")
    
    return results

def main():
    rospy.init_node('control_effectiveness_test', anonymous=True)
    
    node = PhaseLockNode(enable_gui=True)
    
    import threading
    t = threading.Thread(target=rospy.spin)
    t.daemon = True
    t.start()
    
    print("Waiting for optic flow data...")
    while len(node.buffer) < 10 and not rospy.is_shutdown():
        time.sleep(0.1)
    
    # Get optimal phase from user
    optimal_phase = float(input("\nEnter optimal phase from previous test (deg): "))
    cycle_duration = int(input("Enter cycle duration (seconds, default=120): ") or "120")
    num_cycles = int(input("Enter number of cycles (default=3): ") or "3")
    
    try:
        results = control_effectiveness_test(node, optimal_phase, cycle_duration, num_cycles)
        
        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'../data/results/control_effectiveness_{timestamp}.json'
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"\nResults saved to: {filename}")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    finally:
        node.cleanup()

if __name__ == '__main__':
    main()