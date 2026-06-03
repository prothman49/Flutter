#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
0 Hz Control Test - DC Voltage Only (with detailed data logging)
Saves both summary JSON and detailed time-series JSON matching bag format
"""

import rospy
import time
import numpy as np
import json
from datetime import datetime
from std_msgs.msg import Float32
from flutter_control_node import PhaseLockNode

class DCControlLogger:
    """Logs detailed time-series data during DC control test"""
    
    def __init__(self):
        self.data = {
            'metadata': {
                'test_type': 'dc_control_0hz',
                'timestamp': datetime.now().isoformat(),
                'dc_voltage': 0.0,
                'duration_seconds': 0,
                'sample_count': 0
            },
            'time': [],
            'optic_flow_vx': [],
            'amplitude_envelope': [],
            'phase_mod': [],
            'piezo_voltages': [],
            'control_state': []  # 'OFF' or 'DC_ON'
        }
        
        self.start_time = None
        self.optic_flow_sub = None
        self.amplitude_sub = None
        
    def start(self, dc_voltage):
        """Start logging"""
        self.data['metadata']['dc_voltage'] = dc_voltage
        self.start_time = time.time()
        
        # Subscribe to topics
        self.optic_flow_sub = rospy.Subscriber('/optic_flow_mean_vx', Float32, self.optic_flow_callback)
        self.amplitude_sub = rospy.Subscriber('/amplitude_envelope', Float32, self.amplitude_callback)
        
    def optic_flow_callback(self, msg):
        """Record optical flow data"""
        if self.start_time is None:
            return
        
        t = time.time() - self.start_time
        self.data['time'].append(t)
        self.data['optic_flow_vx'].append(float(msg.data))
        
    def amplitude_callback(self, msg):
        """Record amplitude envelope"""
        if self.start_time is None:
            return
        self.data['amplitude_envelope'].append(float(msg.data))
    
    def log_piezo_state(self, v0, v1, control_state):
        """Log piezo voltages and control state"""
        if self.start_time is None:
            return
        
        t = time.time() - self.start_time
        self.data['piezo_voltages'].append({
            'time': t,
            'voltages': [v0, v1]
        })
        self.data['control_state'].append({
            'time': t,
            'state': control_state
        })
    
    def save(self, filename):
        """Save logged data to JSON"""
        # Update metadata
        if self.start_time:
            self.data['metadata']['duration_seconds'] = time.time() - self.start_time
        
        self.data['metadata']['sample_count'] = len(self.data['optic_flow_vx'])
        
        # Calculate statistics
        if self.data['optic_flow_vx']:
            self.data['metadata']['sample_rate_hz'] = len(self.data['optic_flow_vx']) / self.data['metadata']['duration_seconds']
            self.data['metadata']['optic_flow_stats'] = {
                'mean': float(np.mean(self.data['optic_flow_vx'])),
                'std': float(np.std(self.data['optic_flow_vx'])),
                'min': float(np.min(self.data['optic_flow_vx'])),
                'max': float(np.max(self.data['optic_flow_vx']))
            }
        
        if self.data['amplitude_envelope']:
            self.data['metadata']['amplitude_stats'] = {
                'mean': float(np.mean(self.data['amplitude_envelope'])),
                'std': float(np.std(self.data['amplitude_envelope'])),
                'min': float(np.min(self.data['amplitude_envelope'])),
                'max': float(np.max(self.data['amplitude_envelope']))
            }
        
        # Save to file
        with open(filename, 'w') as f:
            json.dump(self.data, f, indent=2)
        
        print(f"\n✓ Detailed data saved: {filename}")
        print(f"  Duration: {self.data['metadata']['duration_seconds']:.1f}s")
        print(f"  Samples: {self.data['metadata']['sample_count']}")
        print(f"  Rate: {self.data['metadata'].get('sample_rate_hz', 0):.1f} Hz")


def control_effectiveness_test_dc(node, dc_voltage=4.2, cycle_duration=120, num_cycles=5):
    """
    Test control with DC voltage only (no square wave)
    Logs detailed time-series data
    """
    
    # Create logger
    logger = DCControlLogger()
    logger.start(dc_voltage)
    
    results = {
        'target_frequency': 0.0,  # 0 Hz = DC
        'dc_voltage': dc_voltage,
        'cycle_duration': cycle_duration,
        'num_cycles': num_cycles,
        'control_off': {'cycles': []},
        'control_on': {'cycles': []}
    }
    
    print("\n" + "="*60)
    print("0 Hz DC CONTROL TEST (with detailed logging)")
    print("="*60)
    print(f"DC voltage: {dc_voltage}V on one piezo (NO SQUARE WAVE)")
    print(f"Cycle duration: {cycle_duration}s (ON) / {cycle_duration}s (OFF)")
    print(f"Number of cycles: {num_cycles}")
    print(f"Total test time: {num_cycles * 2 * cycle_duration / 60:.1f} minutes")
    print(f"📊 Logging detailed time-series data...")
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
        
        # Make sure both piezos are off
        node.voltage_output_0.setVoltage(0.0)
        node.voltage_output_1.setVoltage(0.0)
        logger.log_piezo_state(0.0, 0.0, 'OFF')
        
        off_magnitudes = []
        off_flutter_time = 0
        off_peak_mag = 0
        
        start_time = time.time()
        last_update = start_time
        last_log = start_time
        
        while (time.time() - start_time) < cycle_duration:
            node.update()
            
            # Log piezo state every second
            if time.time() - last_log > 1.0:
                logger.log_piezo_state(0.0, 0.0, 'OFF')
                last_log = time.time()
            
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
        
        # === CONTROL ON PERIOD (DC VOLTAGE) ===
        print(f"\n[Cycle {cycle + 1}] DC VOLTAGE ON ({dc_voltage}V constant) - {cycle_duration}s")
        
        # Apply DC voltage to ONE piezo only (no square wave)
        node.voltage_output_0.setVoltage(dc_voltage)
        node.voltage_output_1.setVoltage(0.0)
        logger.log_piezo_state(dc_voltage, 0.0, 'DC_ON')
        
        on_magnitudes = []
        on_flutter_time = 0
        on_peak_mag = 0
        
        start_time = time.time()
        last_update = start_time
        last_log = start_time
        
        while (time.time() - start_time) < cycle_duration:
            node.update()
            
            # Log piezo state every second
            if time.time() - last_log > 1.0:
                logger.log_piezo_state(dc_voltage, 0.0, 'DC_ON')
                last_log = time.time()
            
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
        
        # Turn off DC voltage
        node.voltage_output_0.setVoltage(0.0)
        node.voltage_output_1.setVoltage(0.0)
        logger.log_piezo_state(0.0, 0.0, 'OFF')
        
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
        
        # 1 minute wait between cycles
        print("\n⏸  Waiting 60s for flutter to reset...")
        time.sleep(60)
    
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
    
    results['effectiveness'] = {
        'magnitude_reduction_percent': float(mag_reduction)
    }
    
    # Print summary
    print("\n" + "="*60)
    print("0 Hz DC CONTROL TEST SUMMARY")
    print("="*60)
    print(f"DC voltage: {dc_voltage}V constant (no oscillation)")
    print(f"\nControl OFF (Baseline):")
    print(f"  Average magnitude: {off_avg_mag:.1f}")
    print(f"\nDC ON (constant voltage, no square wave):")
    print(f"  Average magnitude: {on_avg_mag:.1f}")
    print(f"\nImprovement:")
    print(f"  Magnitude reduction: {mag_reduction:.1f}%")
    
    if abs(mag_reduction) < 2:
        print("\n✓ EXPECTED: DC voltage shows no effect")
        print("   This proves control requires oscillation")
    else:
        print(f"\n⚠ UNEXPECTED: DC voltage shows {mag_reduction:.1f}% change")
    
    print("="*60)
    
    return results, logger


def main():
    rospy.init_node('dc_control_test', anonymous=True)
    
    # Create node without GUI
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
    print("0 Hz DC CONTROL TEST")
    print("="*60)
    print("\nThis test applies constant DC voltage with NO oscillation.")
    print("Expected result: NO improvement (proves oscillation is required)")
    print("="*60)
    
    # Get test parameters
    dc_voltage = float(input("\nEnter DC voltage (V) [default=4.2]: ") or "4.2")
    cycle_duration = int(input("Enter cycle duration (s) [default=120]: ") or "120")
    num_cycles = int(input("Enter number of cycles [default=5]: ") or "5")
    
    try:
        # Run test
        results, logger = control_effectiveness_test_dc(
            node, 
            dc_voltage=dc_voltage,
            cycle_duration=cycle_duration,
            num_cycles=num_cycles
        )
        
        # Save files
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save summary JSON
        summary_file = f'../data/results/0hz/dc_control_summary_{timestamp}.json'
        with open(summary_file, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"\n✓ Summary saved: {summary_file}")
        
        # Save detailed time-series JSON (bag format)
        detailed_file = f'../data/results/0hz/json/dc_control_detailed_{timestamp}.json'
        logger.save(detailed_file)
        
        print("\n📁 Both files saved:")
        print(f"  Summary: {summary_file}")
        print(f"  Detailed: {detailed_file}")
        print("\nThe detailed JSON can be analyzed with analyze_bag_json.py on Mac!")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    finally:
        node.cleanup()


if __name__ == '__main__':
    main()