#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test 1: Structural Phase Characterization
No airflow - measure structural phase lag from piezo to ROI
"""

import rospy
import time
import numpy as np
from flutter_control_node import PhaseLockNode

def structural_phase_test(node, test_frequencies=[5.0, 6.0, 7.0]):
    """
    Measure structural phase lag at different frequencies
    
    Args:
        node: PhaseLockNode instance
        test_frequencies: List of frequencies to test (Hz)
    
    Returns:
        Dictionary of results
    """
    print("\n" + "="*60)
    print("STRUCTURAL PHASE LAG TEST")
    print("="*60)
    print("NO AIRFLOW - Measuring pure structural response")
    print(f"Testing frequencies: {test_frequencies} Hz")
    print("="*60 + "\n")
    
    input("Make sure there is NO AIRFLOW. Press Enter to continue...")
    
    results = []
    
    for freq in test_frequencies:
        print(f"\n[TEST] Frequency = {freq} Hz")
        print("-" * 40)
        
        freq_results = []
        
        for excitation_phase in [0, 90, 180, 270]:
            # Set excitation parameters
            node.set_phase(excitation_phase)
            node.control_enabled = True
            node.waveform_freq = freq
            
            # Start excitation
            node.start_waveform(freq)
            
            print(f"  Exciting at {excitation_phase} deg...")
            
            # Wait for steady state
            time.sleep(5)
            
            # Collect several samples and update
            for _ in range(5):
                node.update()
                time.sleep(0.2)
            
            # Measure response phase (from Hilbert transform)
            response_phase_rad = node.instantaneous_phase_latest
            response_phase_deg = response_phase_rad * 180 / np.pi
            
            # Calculate phase lag
            phase_lag = (response_phase_deg - excitation_phase) % 360
            
            freq_results.append({
                'excitation_phase': excitation_phase,
                'response_phase': response_phase_deg,
                'phase_lag': phase_lag
            })
            
            print(f"    Response phase: {response_phase_deg:.1f} deg")
            print(f"    Phase lag: {phase_lag:.1f} deg")
            
            node.stop_waveform()
            time.sleep(2)
        
        # Average phase lag for this frequency
        avg_lag = np.mean([r['phase_lag'] for r in freq_results])
        std_lag = np.std([r['phase_lag'] for r in freq_results])
        
        results.append({
            'frequency': freq,
            'avg_phase_lag': avg_lag,
            'std_phase_lag': std_lag,
            'measurements': freq_results
        })
        
        print(f"\n  Average phase lag at {freq} Hz: {avg_lag:.1f} ± {std_lag:.1f} deg")
    
    # Print summary
    print("\n" + "="*60)
    print("STRUCTURAL PHASE LAG SUMMARY")
    print("="*60)
    print(f"{'Frequency':>12} | {'Phase Lag':>12} | {'Std Dev':>12}")
    print("-" * 60)
    
    for r in results:
        print(f"{r['frequency']:>10.1f} Hz | {r['avg_phase_lag']:>10.1f} deg | {r['std_phase_lag']:>10.1f} deg")
    
    # Predicted damping phase for flutter (assuming flutter at ~6 Hz)
    flutter_freq_result = [r for r in results if abs(r['frequency'] - 6.0) < 0.5]
    if flutter_freq_result:
        lag = flutter_freq_result[0]['avg_phase_lag']
        predicted_damping_phase = (180 + lag) % 360
        
        print("\n" + "="*60)
        print(f"PREDICTION FOR FLUTTER DAMPING (~6 Hz):")
        print(f"  Structural phase lag: {lag:.1f} deg")
        print(f"  Predicted optimal damping phase: {predicted_damping_phase:.0f} deg")
        print(f"  (This is 180 deg + structural lag)")
        print("="*60 + "\n")
    
    return results

def main():
    print("DEBUG: Starting main()")
    
    try:
        print("DEBUG: Initializing ROS node...")
        rospy.init_node('structural_phase_test', anonymous=True)
        print("DEBUG: ROS node initialized")
    except Exception as e:
        print(f"DEBUG: ROS init failed: {e}")
        return
    
    try:
        print("DEBUG: Creating PhaseLockNode...")
        node = PhaseLockNode(enable_gui=False)
        print("DEBUG: PhaseLockNode created")
    except Exception as e:
        print(f"DEBUG: Node creation failed: {e}")
        import traceback
        traceback.print_exc()
        return
    
    print("\nDEBUG: Starting ROS spinner in background...")
    import threading
    t = threading.Thread(target=rospy.spin)
    t.daemon = True
    t.start()
    
    # Wait for data to start flowing
    print("DEBUG: Waiting for optic flow data...")
    timeout = 30
    start_wait = time.time()
    while len(node.buffer) < 10 and not rospy.is_shutdown():
        if time.time() - start_wait > timeout:
            print("DEBUG: Timeout waiting for data!")
            print(f"DEBUG: Buffer size: {len(node.buffer)}")
            print("DEBUG: Is /optic_flow_mean_vx publishing?")
            return
        time.sleep(0.1)
    
    print(f"DEBUG: Data received. Buffer size: {len(node.buffer)}")
    print("DEBUG: Starting test...\n")
    
    try:
        # Run test
        results = structural_phase_test(node, test_frequencies=[5.0, 6.0, 7.0])
        
        # Save results
        import json
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'../data/results/structural_phase_{timestamp}.json'
        with open(filename, 'w') as f:
            # Convert numpy types to native Python types for JSON serialization
            results_serializable = []
            for r in results:
                r_copy = r.copy()
                r_copy['frequency'] = float(r_copy['frequency'])
                r_copy['avg_phase_lag'] = float(r_copy['avg_phase_lag'])
                r_copy['std_phase_lag'] = float(r_copy['std_phase_lag'])
                results_serializable.append(r_copy)
            json.dump(results_serializable, f, indent=2)
        print(f"\nResults saved to: {filename}")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"DEBUG: Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.cleanup()

if __name__ == '__main__':
    print("DEBUG: Script is being run directly")
    main()