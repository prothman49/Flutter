#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Off-Frequency Control Effectiveness Test
Tests flutter control at frequencies that do NOT match flutter frequency
to prove that synchronization is necessary, not just piezo vibration
"""

import rospy
import time
import numpy as np
import json
from datetime import datetime
from flutter_control_node import PhaseLockNode

def control_effectiveness_test_fixed_freq(node, target_freq, phase_offset, cycle_duration=120, num_cycles=3):
    """
    Test control effectiveness at a FIXED frequency (not matched to flutter)
    
    Args:
        node: PhaseLockNode instance
        target_freq: Fixed frequency for piezo actuation (Hz)
        phase_offset: Phase offset in degrees
        cycle_duration: Duration of each ON/OFF cycle in seconds
        num_cycles: Number of cycles to run
    
    Returns:
        dict with test results
    """
    
    results = {
        'target_frequency': target_freq,
        'phase_offset': phase_offset,
        'control_off': {'cycles': []},
        'control_on': {'cycles': []}
    }
    
    node.set_phase(phase_offset)
    
    print("\n" + "="*60)
    print("OFF-FREQUENCY CONTROL TEST")
    print("="*60)
    print(f"Target frequency: {target_freq} Hz (FIXED, not matched to flutter)")
    print(f"Phase offset: {phase_offset} deg")
    print(f"Cycle duration: {cycle_duration}s (ON) / {cycle_duration}s (OFF)")
    print(f"Number of cycles: {num_cycles}")
    print(f"Total test time: {num_cycles * 2 * cycle_duration / 60:.1f} minutes")
    print("="*60)
    
    input("\nMake sure AIRFLOW IS ON. Press Enter to start...")
    
    for cycle in range(num_cycles):
        print(f"\n{'='*60}")
        print(f"CYCLE {cycle + 1}/{num_cycles}")
        print(f"{'='*60}")
        
        # === CONTROL OFF PERIOD ===
        print(f"\n[Cycle {cycle + 1}] CONTROL OFF - {cycle_duration}s")
        node.control_enabled = False
        node.stop_waveform()
        
        off_magnitudes = []
        off_flutter_time = 0
        off_peak_mag = 0
        
        start_time = time.time()
        last_update = start_time
        
        while (time.time() - start_time) < cycle_duration:
            node.update()
            
            if time.time() - last_update > 1.0:
                if len(node.magnitude_history) > 0:
                    current_mag = node.magnitude_history[-1]
                    off_magnitudes.append(current_mag)
                    
                    if current_mag > node.flutter_threshold:
                        off_flutter_time += 1.0
                    
                    if current_mag > off_peak_mag:
                        off_peak_mag = current_mag
                    
                    elapsed = time.time() - start_time
                    print(f"  OFF: t={elapsed:.0f}s, Avg={np.mean(off_magnitudes):.1f}, Flutter={off_flutter_time:.0f}s", end='\r')
                
                last_update = time.time()
            
            time.sleep(0.05)
        
        off_avg = np.mean(off_magnitudes) if off_magnitudes else 0
        off_flutter_pct = (off_flutter_time / cycle_duration) * 100
        
        print(f"\n    OFF:  Avg={off_avg:.1f}, Flutter={off_flutter_pct:.1f}%")
        
        # === CONTROL ON PERIOD (FIXED FREQUENCY) ===
        print(f"\n[Cycle {cycle + 1}] CONTROL ON at {target_freq} Hz (FIXED) - {cycle_duration}s")
        
        # CRITICAL: Start waveform at FIXED frequency, not detected frequency
        node.waveform_freq = target_freq  # Force fixed frequency
        node.control_enabled = True
        node.start_waveform(target_freq)  # Use fixed frequency
        
        on_magnitudes = []
        on_flutter_time = 0
        on_peak_mag = 0
        
        start_time = time.time()
        last_update = start_time
        
        while (time.time() - start_time) < cycle_duration:
            node.update()
            
            # KEEP frequency fixed even if flutter frequency detected
            if node.waveform_running and node.waveform_freq != target_freq:
                node.waveform_freq = target_freq
            
            if time.time() - last_update > 1.0:
                if len(node.magnitude_history) > 0:
                    current_mag = node.magnitude_history[-1]
                    on_magnitudes.append(current_mag)
                    
                    if current_mag > node.flutter_threshold:
                        on_flutter_time += 1.0
                    
                    if current_mag > on_peak_mag:
                        on_peak_mag = current_mag
                    
                    elapsed = time.time() - start_time
                    print(f"  ON:  t={elapsed:.0f}s, Avg={np.mean(on_magnitudes):.1f}, Flutter={on_flutter_time:.0f}s", end='\r')
                
                last_update = time.time()
            
            time.sleep(0.05)
        
        on_avg = np.mean(on_magnitudes) if on_magnitudes else 0
        on_flutter_pct = (on_flutter_time / cycle_duration) * 100
        
        print(f"\n    ON:  Avg={on_avg:.1f}, Flutter={on_flutter_pct:.1f}%")
        
        # Stop control
        node.control_enabled = False
        node.stop_waveform()
        
        # Save cycle results
        results['control_off']['cycles'].append({
            'avg_magnitude': float(off_avg),
            'peak_magnitude': float(off_peak_mag),
            'flutter_time': float(off_flutter_time),
            'flutter_percent': float(off_flutter_pct)
        })
        
        results['control_on']['cycles'].append({
            'avg_magnitude': float(on_avg),
            'peak_magnitude': float(on_peak_mag),
            'flutter_time': float(on_flutter_time),
            'flutter_percent': float(on_flutter_pct)
        })
    
    # Calculate averages
    off_avg_mag = np.mean([c['avg_magnitude'] for c in results['control_off']['cycles']])
    off_avg_flutter = np.mean([c['flutter_percent'] for c in results['control_off']['cycles']])
    
    on_avg_mag = np.mean([c['avg_magnitude'] for c in results['control_on']['cycles']])
    on_avg_flutter = np.mean([c['flutter_percent'] for c in results['control_on']['cycles']])
    
    results['control_off']['avg_magnitude'] = float(off_avg_mag)
    results['control_off']['flutter_percent'] = float(off_avg_flutter)
    
    results['control_on']['avg_magnitude'] = float(on_avg_mag)
    results['control_on']['flutter_percent'] = float(on_avg_flutter)
    
    # Calculate effectiveness
    if off_avg_mag > 0:
        mag_reduction = ((off_avg_mag - on_avg_mag) / off_avg_mag) * 100
    else:
        mag_reduction = 0
    
    if off_avg_flutter > 0:
        flutter_reduction = ((off_avg_flutter - on_avg_flutter) / off_avg_flutter) * 100
    else:
        flutter_reduction = 0
    
    results['effectiveness'] = {
        'magnitude_reduction_percent': float(mag_reduction),
        'flutter_time_reduction_percent': float(flutter_reduction)
    }
    
    # Print summary
    print("\n" + "="*60)
    print("OFF-FREQUENCY CONTROL TEST SUMMARY")
    print("="*60)
    print(f"Fixed frequency: {target_freq} Hz")
    print(f"Phase offset: {phase_offset} deg")
    print(f"\nControl OFF (Baseline):")
    print(f"  Average magnitude: {off_avg_mag:.1f}")
    print(f"  Flutter time: {off_avg_flutter:.1f}%")
    print(f"\nControl ON (at {target_freq} Hz):")
    print(f"  Average magnitude: {on_avg_mag:.1f}")
    print(f"  Flutter time: {on_avg_flutter:.1f}%")
    print(f"\nImprovement:")
    print(f"  Magnitude reduction: {mag_reduction:.1f}%")
    print(f"  Flutter time reduction: {flutter_reduction:.1f}%")
    
    if mag_reduction < 3:
        print("⚠ MINIMAL/NO EFFECT: Off-frequency control shows little improvement")
        print("   This suggests frequency synchronization is critical!")
    elif mag_reduction > 8:
        print("✓ SIGNIFICANT EFFECT: Control works even off-frequency")
        print("   This suggests mechanical damping contributes significantly")
    else:
        print("⚠ MARGINAL: Small effect observed")
    
    print("="*60)
    
    return results


def main():
    rospy.init_node('off_frequency_control_test', anonymous=True)
    
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
    print("OFF-FREQUENCY CONTROL EFFECTIVENESS TEST")
    print("="*60)
    print("\nThis test will run piezo control at a FIXED frequency")
    print("that does NOT match the flutter frequency.")
    print("\nIf control only works when frequency is matched,")
    print("this test should show minimal/no improvement.")
    print("="*60)
    
    # Get test parameters
    target_freq = float(input("\nEnter fixed piezo frequency (Hz) [e.g., 3 or 8]: "))
    phase_offset = float(input("Enter phase offset (deg) [typically 120]: "))
    cycle_duration = int(input("Enter cycle duration (s) [default=120]: ") or "120")
    num_cycles = int(input("Enter number of cycles [default=3]: ") or "3")
    
    try:
        # Run test
        results = control_effectiveness_test_fixed_freq(
            node, 
            target_freq,
            phase_offset,
            cycle_duration=cycle_duration,
            num_cycles=num_cycles
        )
        
        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'../data/results/off_frequency_test_{target_freq}Hz_{timestamp}.json'
        
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"\nResults saved to: {filename}")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    finally:
        node.cleanup()


if __name__ == '__main__':
    main()