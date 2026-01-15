#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test 2: Flutter Phase Sweep
With airflow - find optimal phase for flutter damping
"""

import rospy
import time
import numpy as np
import json
from datetime import datetime
from flutter_control_node import PhaseLockNode

def flutter_phase_sweep(node, phases, test_duration=60, settling_time=10):
    """
    Automated phase sweep test with flutter
    
    Args:
        node: PhaseLockNode instance
        phases: List of phase values to test (degrees)
        test_duration: Duration to test each phase (seconds)
        settling_time: Wait time between tests (seconds)
    
    Returns:
        List of results dictionaries
    """
    print("\n" + "="*60)
    print("FLUTTER PHASE SWEEP TEST")
    print("="*60)
    print(f"Testing {len(phases)} phases: {phases}")
    print(f"Test duration per phase: {test_duration}s")
    print(f"Settling time between tests: {settling_time}s")
    print(f"Total estimated time: {(len(phases) * (test_duration + settling_time)) / 60:.1f} minutes")
    print("="*60 + "\n")
    
    input("Make sure AIRFLOW IS ON and flutter can be established. Press Enter to continue...")
    
    results = []
    
    for i, phase in enumerate(phases):
        print(f"\n[TEST {i+1}/{len(phases)}] Phase = {phase} deg")
        print("-" * 40)
        
        # Set phase
        node.set_phase(phase)
        
        # Make sure control is enabled
        if not node.control_enabled:
            node.control_enabled = True
        
        # Wait for flutter to establish (or manual intervention)
        print("  Waiting for flutter to establish...")
        print("  (Manually disturb wing if needed)")
        
        # Wait for magnitude to exceed threshold
        wait_start = time.time()
        while time.time() - wait_start < 30:  # 30 second timeout
            node.update()
            if len(node.magnitude_history) > 0 and node.magnitude_history[-1] > node.flutter_threshold:
                print(f"  Flutter detected! Magnitude = {node.magnitude_history[-1]:.1f}")
                break
            time.sleep(0.5)
        
        if len(node.magnitude_history) == 0 or node.magnitude_history[-1] < node.flutter_threshold:
            print("  WARNING: Flutter not established. Proceeding anyway...")
        
        # Collect data during test
        print(f"  Recording for {test_duration}s...")
        start_time = time.time()
        magnitudes_during_test = []
        flutter_time = 0
        
        while time.time() - start_time < test_duration:
            node.update()
            if len(node.magnitude_history) > 0:
                current_mag = node.magnitude_history[-1]
                magnitudes_during_test.append(current_mag)
                
                if current_mag > node.flutter_threshold:
                    flutter_time += 0.2
            
            time.sleep(0.2)
        
        # Calculate statistics
        if magnitudes_during_test:
            avg_mag = np.mean(magnitudes_during_test)
            peak_mag = np.max(magnitudes_during_test)
            min_mag = np.min(magnitudes_during_test)
            std_mag = np.std(magnitudes_during_test)
            flutter_percent = (flutter_time / test_duration) * 100
        else:
            avg_mag = peak_mag = min_mag = std_mag = flutter_percent = 0
        
        result = {
            'phase': float(phase),
            'avg_magnitude': float(avg_mag),
            'peak_magnitude': float(peak_mag),
            'min_magnitude': float(min_mag),
            'std_magnitude': float(std_mag),
            'flutter_time_s': float(flutter_time),
            'flutter_percent': float(flutter_percent),
            'test_duration': test_duration
        }
        results.append(result)
        
        print(f"  Results:")
        print(f"    Avg magnitude: {avg_mag:.1f}")
        print(f"    Peak magnitude: {peak_mag:.1f}")
        print(f"    Flutter time: {flutter_time:.1f}s ({flutter_percent:.1f}%)")
        
        # Settling time
        print(f"  Waiting {settling_time}s before next test...")
        node.stop_waveform()
        time.sleep(settling_time)
    
    # Print summary
    print("\n" + "="*60)
    print("PHASE SWEEP RESULTS SUMMARY")
    print("="*60)
    print(f"{'Phase':>8} | {'Avg Mag':>8} | {'Peak Mag':>9} | {'Flutter %':>10}")
    print("-" * 60)
    
    for r in results:
        print(f"{r['phase']:>8.0f} | {r['avg_magnitude']:>8.1f} | {r['peak_magnitude']:>9.1f} | {r['flutter_percent']:>9.1f}%")
    
    # Find best and worst phases
    best = min(results, key=lambda x: x['avg_magnitude'])
    worst = max(results, key=lambda x: x['avg_magnitude'])
    
    print("\n" + "="*60)
    print(f"BEST PHASE FOR DAMPING: {best['phase']:.0f} deg")
    print(f"  Average magnitude: {best['avg_magnitude']:.1f}")
    print(f"  Flutter time: {best['flutter_time_s']:.1f}s ({best['flutter_percent']:.1f}%)")
    print(f"\nWORST PHASE (AMPLIFICATION): {worst['phase']:.0f} deg")
    print(f"  Average magnitude: {worst['avg_magnitude']:.1f}")
    print(f"  Flutter time: {worst['flutter_time_s']:.1f}s ({worst['flutter_percent']:.1f}%)")
    
    # Control effectiveness
    if worst['avg_magnitude'] > 0:
        improvement = ((worst['avg_magnitude'] - best['avg_magnitude']) / worst['avg_magnitude']) * 100
        print(f"\nCONTROL EFFECTIVENESS:")
        print(f"  Magnitude reduction: {improvement:.1f}%")
    print("="*60 + "\n")
    
    return results

def main():
    rospy.init_node('flutter_phase_sweep', anonymous=True)
    
    # Create node WITH GUI for monitoring
    node = PhaseLockNode(enable_gui=True)
    
    import threading
    t = threading.Thread(target=rospy.spin)
    t.daemon = True
    t.start()
    
    # Wait for data
    print("Waiting for optic flow data...")
    while len(node.buffer) < 10 and not rospy.is_shutdown():
        time.sleep(0.1)
    
    print("Data received.\n")
    
    # Choose test type
    print("Select test type:")
    print("  1. Coarse sweep (45 deg increments)")
    print("  2. Fine sweep (15 deg increments)")
    print("  3. Custom phases")
    
    choice = input("Enter choice (1/2/3): ").strip()
    
    if choice == '1':
        phases = [0, 45, 90, 135, 180, 225, 270, 315]
        test_duration = 60
    elif choice == '2':
        start = int(input("Start phase (deg): "))
        phases = list(range(start - 60, start + 75, 15))
        test_duration = 90
    else:
        phases_str = input("Enter phases separated by commas (e.g., 150,165,180,195): ")
        phases = [float(p.strip()) for p in phases_str.split(',')]
        test_duration = int(input("Test duration per phase (seconds): "))
    
    try:
        # Run test
        results = flutter_phase_sweep(node, phases, test_duration=test_duration)
        
        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'../data/results/flutter_phase_sweep_{timestamp}.json'
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"\nResults saved to: {filename}")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    finally:
        node.cleanup()

if __name__ == '__main__':
    main()