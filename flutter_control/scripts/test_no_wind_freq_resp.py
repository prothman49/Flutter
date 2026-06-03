#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
No-Wind Frequency Response Test
Actuate piezos at different frequencies WITHOUT airflow
Shows mechanical excitation capability across frequency range
"""

import rospy
import time
import numpy as np
import json
from datetime import datetime
from std_msgs.msg import Float32
from pathlib import Path
from flutter_control_node import PhaseLockNode

class FrequencyResponseLogger:
    """Logs data during frequency response test"""
    
    def __init__(self):
        self.data = {
            'metadata': {
                'test_type': 'no_wind_frequency_response',
                'timestamp': datetime.now().isoformat(),
                'airflow': 'OFF',
                'duration_seconds': 0,
                'sample_count': 0,
                'frequencies_tested': []
            },
            'time': [],
            'optic_flow_vx': [],
            'amplitude_envelope': [],
            'phase_mod': [],
            'piezo_voltages': [],
            'actuation_frequency': []  # Track which frequency is being tested
        }
        
        self.start_time = None
        self.optic_flow_sub = None
        self.amplitude_sub = None
        
    def start(self):
        """Start logging"""
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
    
    def log_frequency_change(self, frequency, duration):
        """Log when actuation frequency changes"""
        if self.start_time is None:
            return
        
        t = time.time() - self.start_time
        self.data['actuation_frequency'].append({
            'time': t,
            'frequency': frequency,
            'duration': duration
        })
        
        # Add to metadata
        if frequency not in self.data['metadata']['frequencies_tested']:
            self.data['metadata']['frequencies_tested'].append(frequency)
    
    def log_piezo_state(self, v0, v1, frequency):
        """Log piezo voltages"""
        if self.start_time is None:
            return
        
        t = time.time() - self.start_time
        self.data['piezo_voltages'].append({
            'time': t,
            'voltages': [v0, v1],
            'frequency': frequency
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
        
        print(f"\n✓ Data saved: {filename}")
        print(f"  Duration: {self.data['metadata']['duration_seconds']:.1f}s")
        print(f"  Samples: {self.data['metadata']['sample_count']}")
        print(f"  Rate: {self.data['metadata'].get('sample_rate_hz', 0):.1f} Hz")
        print(f"  Frequencies tested: {sorted(self.data['metadata']['frequencies_tested'])}")


def actuate_at_frequency(node, frequency, duration, amplitude=4.2):
    """
    Actuate piezos at specified frequency using square wave
    
    Args:
        node: PhaseLockNode instance
        frequency: Actuation frequency in Hz
        duration: How long to actuate in seconds
        amplitude: Voltage amplitude (0-4.2V)
    """
    
    if frequency == 0:
        # DC case - no oscillation
        node.voltage_output_0.setVoltage(amplitude)
        node.voltage_output_1.setVoltage(0.0)
        time.sleep(duration)
        node.voltage_output_0.setVoltage(0.0)
        return
    
    # Square wave actuation
    half_period = 1.0 / (2.0 * frequency)  # Time for half cycle
    
    start_time = time.time()
    state = 0  # Alternates between 0 and 1
    
    while (time.time() - start_time) < duration:
        if state == 0:
            # Piezo 1 ON, Piezo 2 OFF
            node.voltage_output_0.setVoltage(amplitude)
            node.voltage_output_1.setVoltage(0.0)
        else:
            # Piezo 1 OFF, Piezo 2 ON
            node.voltage_output_0.setVoltage(0.0)
            node.voltage_output_1.setVoltage(amplitude)
        
        time.sleep(half_period)
        state = 1 - state  # Toggle
    
    # Turn off both
    node.voltage_output_0.setVoltage(0.0)
    node.voltage_output_1.setVoltage(0.0)


def run_frequency_sweep(node, frequencies, duration_per_freq, settling_time):
    """
    Sweep through frequencies with no airflow
    
    Args:
        node: PhaseLockNode instance
        frequencies: List of frequencies to test (Hz)
        duration_per_freq: Duration at each frequency (seconds)
        settling_time: Wait time between frequencies (seconds)
    """
    
    # Create logger
    logger = FrequencyResponseLogger()
    logger.start()
    
    print("\n" + "="*70)
    print("NO-WIND FREQUENCY RESPONSE TEST")
    print("="*70)
    print(f"Frequencies to test: {frequencies} Hz")
    print(f"Duration per frequency: {duration_per_freq}s")
    print(f"Settling time: {settling_time}s")
    print(f"Total test time: {len(frequencies) * (duration_per_freq + settling_time)}s")
    print("="*70)
    
    input("\nMake sure AIRFLOW IS OFF. Press Enter to start...")
    
    for i, freq in enumerate(frequencies):
        print(f"\n{'='*70}")
        print(f"FREQUENCY {i+1}/{len(frequencies)}: {freq} Hz")
        print(f"{'='*70}")
        
        # Log frequency change
        logger.log_frequency_change(freq, duration_per_freq)
        
        # Actuate at this frequency
        print(f"Actuating at {freq} Hz for {duration_per_freq}s...")
        
        start_time = time.time()
        last_log = start_time
        
        # Run actuation in a separate thread-like manner
        if freq == 0:
            # DC case
            node.voltage_output_0.setVoltage(4.2)
            node.voltage_output_1.setVoltage(0.0)
            logger.log_piezo_state(4.2, 0.0, freq)
            
            while (time.time() - start_time) < duration_per_freq:
                elapsed = time.time() - start_time
                print(f"  {freq} Hz: {elapsed:.1f}/{duration_per_freq}s", end='\r')
                
                if time.time() - last_log > 1.0:
                    logger.log_piezo_state(4.2, 0.0, freq)
                    last_log = time.time()
                
                time.sleep(0.1)
            
            node.voltage_output_0.setVoltage(0.0)
            node.voltage_output_1.setVoltage(0.0)
        else:
            # Square wave
            half_period = 1.0 / (2.0 * freq)
            state = 0
            
            while (time.time() - start_time) < duration_per_freq:
                if state == 0:
                    node.voltage_output_0.setVoltage(4.2)
                    node.voltage_output_1.setVoltage(0.0)
                    if time.time() - last_log > 1.0:
                        logger.log_piezo_state(4.2, 0.0, freq)
                        last_log = time.time()
                else:
                    node.voltage_output_0.setVoltage(0.0)
                    node.voltage_output_1.setVoltage(4.2)
                    if time.time() - last_log > 1.0:
                        logger.log_piezo_state(0.0, 4.2, freq)
                        last_log = time.time()
                
                elapsed = time.time() - start_time
                print(f"  {freq} Hz: {elapsed:.1f}/{duration_per_freq}s", end='\r')
                
                time.sleep(half_period)
                state = 1 - state
            
            # Turn off
            node.voltage_output_0.setVoltage(0.0)
            node.voltage_output_1.setVoltage(0.0)
        
        print(f"\n  ✓ {freq} Hz complete")
        
        # Settling time
        if i < len(frequencies) - 1:  # Not last frequency
            print(f"\n  ⏸ Settling {settling_time}s...")
            logger.log_piezo_state(0.0, 0.0, 0.0)
            time.sleep(settling_time)
    
    print("\n" + "="*70)
    print("FREQUENCY SWEEP COMPLETE")
    print("="*70)
    
    return logger


def main():
    rospy.init_node('frequency_response_test', anonymous=True)
    
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
    
    print("\n" + "="*70)
    print("NO-WIND FREQUENCY RESPONSE TEST")
    print("="*70)
    print("\nThis test actuates the piezos at different frequencies")
    print("WITHOUT airflow to show mechanical frequency response.")
    print("\nTypical frequencies: 3, 5.8, 8, 10 Hz")
    print("="*70)
    
    # Get test parameters
    freq_input = input("\nFrequencies to test (comma-separated) [default=3,5.8,8,10]: ") or "3,5.8,8,10"
    frequencies = [float(f.strip()) for f in freq_input.split(',')]
    
    duration_per_freq = int(input("Duration per frequency (s) [default=20]: ") or "20")
    settling_time = int(input("Settling time between frequencies (s) [default=5]: ") or "5")
    
    try:
        # Run test
        logger = run_frequency_sweep(node, frequencies, duration_per_freq, settling_time)
        
        # Save data
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create output directory if needed
        output_dir = Path('../data/results/frequency_response/json')
        output_dir.mkdir(parents=True, exist_ok=True)
        
        filename = output_dir / f"no_wind_frequency_response_{timestamp}.json"
        logger.save(str(filename))
        
        print(f"\n✓ Test complete! Data saved to:")
        print(f"  {filename}")
        print("\nThis JSON can be analyzed on Mac with analyze_bag_json.py")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    finally:
        node.cleanup()


if __name__ == '__main__':
    main()