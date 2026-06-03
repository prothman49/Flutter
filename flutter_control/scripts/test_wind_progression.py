#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Wind Progression Test - No Control
Shows natural flutter onset as wind speed increases
No wind → Light wind → Flutter wind (all without control)
"""

import rospy
import time
import numpy as np
import json
from datetime import datetime
from std_msgs.msg import Float32
from pathlib import Path

class WindProgressionLogger:
    """Logs data during wind progression test"""
    
    def __init__(self):
        self.data = {
            'metadata': {
                'test_type': 'wind_progression_no_control',
                'timestamp': datetime.now().isoformat(),
                'duration_seconds': 0,
                'sample_count': 0,
                'phases': []  # Will track: no_wind, light_wind, flutter_wind
            },
            'time': [],
            'optic_flow_vx': [],
            'amplitude_envelope': [],
            'phase_mod': [],
            'test_phase': []  # 'NO_WIND', 'LIGHT_WIND', 'FLUTTER_WIND'
        }
        
        self.start_time = None
        self.optic_flow_sub = None
        self.amplitude_sub = None
        self.phase_sub = None
        
    def start(self):
        """Start logging"""
        self.start_time = time.time()
        
        # Subscribe to topics
        self.optic_flow_sub = rospy.Subscriber('/optic_flow_mean_vx', Float32, self.optic_flow_callback)
        self.amplitude_sub = rospy.Subscriber('/amplitude_envelope', Float32, self.amplitude_callback)
        self.phase_sub = rospy.Subscriber('/phase_mod', Float32, self.phase_callback)
        
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
    
    def phase_callback(self, msg):
        """Record phase modulation"""
        if self.start_time is None:
            return
        self.data['phase_mod'].append(float(msg.data))
    
    def log_phase_transition(self, phase_name, duration):
        """Log when test phase changes"""
        if self.start_time is None:
            return
        
        t = time.time() - self.start_time
        self.data['test_phase'].append({
            'time': t,
            'phase': phase_name,
            'duration': duration
        })
        
        # Add to metadata
        self.data['metadata']['phases'].append({
            'name': phase_name,
            'start_time': t,
            'duration': duration
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


def run_wind_progression_test(phase_durations):
    """
    Run wind progression test with manual operator control
    
    Args:
        phase_durations: dict with 'no_wind', 'light_wind', 'flutter_wind' durations in seconds
    """
    
    print("\n" + "="*70)
    print("WIND PROGRESSION TEST - NO CONTROL")
    print("="*70)
    print("\nThis test documents natural flutter onset as wind increases.")
    print("The operator will manually adjust wind speed through 3 phases.")
    print("\nPhase durations:")
    print(f"  1. No wind:      {phase_durations['no_wind']}s")
    print(f"  2. Light wind:   {phase_durations['light_wind']}s")
    print(f"  3. Flutter wind: {phase_durations['flutter_wind']}s")
    print(f"\nTotal test time: {sum(phase_durations.values())}s (~{sum(phase_durations.values())/60:.1f} min)")
    print("="*70)
    
    # Create logger
    logger = WindProgressionLogger()
    logger.start()
    
    # PHASE 1: NO WIND
    print("\n" + "="*70)
    print("PHASE 1: NO WIND")
    print("="*70)
    input("Turn OFF the wind tunnel fan. Press Enter when ready...")
    
    print(f"\nRecording {phase_durations['no_wind']}s with no wind...")
    logger.log_phase_transition('NO_WIND', phase_durations['no_wind'])
    
    start_time = time.time()
    while (time.time() - start_time) < phase_durations['no_wind']:
        elapsed = time.time() - start_time
        print(f"  No wind: {elapsed:.0f}/{phase_durations['no_wind']}s", end='\r')
        time.sleep(0.5)
    
    print(f"\n✓ No wind phase complete")
    
    # PHASE 2: LIGHT WIND
    print("\n" + "="*70)
    print("PHASE 2: LIGHT WIND (subcritical)")
    print("="*70)
    input("Turn on wind to LIGHT speed (below flutter threshold). Press Enter when ready...")
    
    print(f"\nRecording {phase_durations['light_wind']}s with light wind...")
    logger.log_phase_transition('LIGHT_WIND', phase_durations['light_wind'])
    
    start_time = time.time()
    while (time.time() - start_time) < phase_durations['light_wind']:
        elapsed = time.time() - start_time
        print(f"  Light wind: {elapsed:.0f}/{phase_durations['light_wind']}s", end='\r')
        time.sleep(0.5)
    
    print(f"\n✓ Light wind phase complete")
    
    # PHASE 3: FLUTTER WIND
    print("\n" + "="*70)
    print("PHASE 3: FLUTTER WIND (supercritical)")
    print("="*70)
    input("Increase wind to FLUTTER speed (sustained flutter). Press Enter when ready...")
    
    print(f"\nRecording {phase_durations['flutter_wind']}s with flutter wind...")
    logger.log_phase_transition('FLUTTER_WIND', phase_durations['flutter_wind'])
    
    start_time = time.time()
    while (time.time() - start_time) < phase_durations['flutter_wind']:
        elapsed = time.time() - start_time
        print(f"  Flutter wind: {elapsed:.0f}/{phase_durations['flutter_wind']}s", end='\r')
        time.sleep(0.5)
    
    print(f"\n✓ Flutter wind phase complete")
    
    # Done
    print("\n" + "="*70)
    print("TEST COMPLETE")
    print("="*70)
    input("Turn OFF the wind tunnel. Press Enter to save data...")
    
    return logger


def main():
    rospy.init_node('wind_progression_test', anonymous=True)
    
    print("\n" + "="*70)
    print("WIND PROGRESSION TEST - NO CONTROL")
    print("="*70)
    print("\nThis test documents the natural progression from:")
    print("  No wind → Light wind → Flutter wind")
    print("\nNO CONTROL IS APPLIED - this establishes baseline behavior.")
    print("="*70)
    
    # Get test parameters
    no_wind_duration = int(input("\nDuration for NO WIND phase (s) [default=30]: ") or "30")
    light_wind_duration = int(input("Duration for LIGHT WIND phase (s) [default=30]: ") or "30")
    flutter_wind_duration = int(input("Duration for FLUTTER WIND phase (s) [default=60]: ") or "60")
    
    phase_durations = {
        'no_wind': no_wind_duration,
        'light_wind': light_wind_duration,
        'flutter_wind': flutter_wind_duration
    }
    
    try:
        # Run test
        logger = run_wind_progression_test(phase_durations)
        
        # Save data
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create output directory if needed
        output_dir = Path('../data/results/wind_progression/json')
        output_dir.mkdir(parents=True, exist_ok=True)
        
        filename = output_dir / f"wind_progression_no_control_{timestamp}.json"
        logger.save(str(filename))
        
        print(f"\n✓ Test complete! Data saved to:")
        print(f"  {filename}")
        print("\nThis JSON can be analyzed on Mac with analyze_bag_json.py")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()