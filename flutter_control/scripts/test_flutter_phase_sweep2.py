#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Flutter Phase Sweep Test
Systematically tests flutter control effectiveness across 360° phase sweep
"""

import rospy
import json
import numpy as np
from std_msgs.msg import Float32, Bool
from datetime import datetime
from pathlib import Path
import time
import signal
import sys


class NumpyEncoder(json.JSONEncoder):
    """Handle numpy types that aren't JSON serializable by default"""
    def default(self, obj):
        if isinstance(obj, (np.integer, np.int64, np.int32, np.int16, np.int8)):
            return int(obj)
        if isinstance(obj, (np.floating, np.float64, np.float32, np.float16)):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.bool_):
            return bool(obj)
        return super().default(obj)


def to_python(val):
    """Force convert any numpy scalar to native Python type"""
    if isinstance(val, np.integer):
        return int(val)
    if isinstance(val, np.floating):
        return float(val)
    if isinstance(val, np.bool_):
        return bool(val)
    return val


class PhaseSweepTest:
    def __init__(self):
        rospy.init_node('phase_sweep_test', anonymous=True)
        
        # Publishers
        self.phase_pub = rospy.Publisher('/flutter_control/phase_offset', Float32, queue_size=10)
        self.control_enable_pub = rospy.Publisher('/flutter_control/enable', Bool, queue_size=10)
        
        # Subscribers
        self.magnitude_sub = rospy.Subscriber('/dominant_magnitude', Float32, self.magnitude_callback)
        self.frequency_sub = rospy.Subscriber('/dominant_frequency', Float32, self.frequency_callback)

        # Data storage
        self.current_magnitude = 0.0
        self.current_frequency = 0.0
        self.magnitude_history = []
        self.frequency_history = []
        self.flutter_threshold = 105.0
        
        # Save location
        self.save_dir = Path('/home/flutter/catkin_ws/src/flutter_control/data/results/phase_sweep')
        self.save_dir.mkdir(parents=True, exist_ok=True)
        print(f"[INFO] Results will be saved to: {self.save_dir}")
        
        # Results
        self.results = []
        
        rospy.sleep(2.0)
        print("[INFO] Phase sweep test initialized")
        print("[INFO] Subscribed to /dominant_magnitude and /dominant_frequency")
        
    def magnitude_callback(self, msg):
        self.current_magnitude = msg.data
        self.magnitude_history.append(float(msg.data))  # Force Python float
        
    def frequency_callback(self, msg):
        self.current_frequency = msg.data
        self.frequency_history.append(float(msg.data))  # Force Python float
    
    def test_phase(self, phase, duration=40.0):
        """Test a single phase for specified duration"""
        print(f"\n--- Testing Phase {phase}° ---")
        
        # Reset data
        self.magnitude_history = []
        self.frequency_history = []
        
        # Set phase
        self.phase_pub.publish(Float32(phase))
        rospy.sleep(0.5)
        
        # Enable control
        self.control_enable_pub.publish(Bool(True))
        print(f"Control enabled at {phase}°, collecting data for {duration}s...")
        
        # Collect data
        start_time = time.time()
        last_print = start_time
        
        while (time.time() - start_time) < duration:
            rospy.sleep(0.1)
            if time.time() - last_print >= 5.0:
                elapsed = time.time() - start_time
                print(f"  Progress: {elapsed:.1f}/{duration}s - Current mag: {self.current_magnitude:.1f}, freq: {self.current_frequency:.2f}Hz")
                last_print = time.time()
        
        # Disable control
        self.control_enable_pub.publish(Bool(False))
        print("Control disabled, analyzing data...")
        
        # Analyze - convert everything to plain Python types
        if len(self.magnitude_history) > 0:
            mags = np.array(self.magnitude_history, dtype=np.float64)
            freqs = np.array(self.frequency_history, dtype=np.float64)
            
            # Flutter time - use Python sum to avoid int64
            flutter_count = int(sum(1 for m in self.magnitude_history if m > self.flutter_threshold))
            flutter_time = round(flutter_count * 0.1, 3)
            flutter_percent = round((flutter_time / duration) * 100, 3)
            
            # Frequency stats
            valid_freqs = [f for f in self.frequency_history if f > 0]
            avg_freq = round(float(np.mean(valid_freqs)), 4) if valid_freqs else 0.0
            std_freq = round(float(np.std(valid_freqs)), 4) if valid_freqs else 0.0
            
            # Build result with all plain Python types - no numpy scalars
            result = {
                'phase': round(float(phase), 2),
                'avg_magnitude': round(float(np.mean(mags)), 4),
                'peak_magnitude': round(float(np.max(mags)), 4),
                'min_magnitude': round(float(np.min(mags)), 4),
                'std_magnitude': round(float(np.std(mags)), 4),
                'flutter_time_s': flutter_time,
                'flutter_percent': flutter_percent,
                'flutter_count': flutter_count,
                'test_duration': round(float(duration), 1),
                'avg_frequency': avg_freq,
                'std_frequency': std_freq,
                'flutter_threshold': round(float(self.flutter_threshold), 1),
                'sample_count': len(self.magnitude_history)
            }
            
            # Verify all values are JSON serializable before returning
            for k, v in result.items():
                if not isinstance(v, (int, float, str, bool, type(None))):
                    print(f"  [WARN] Non-serializable value for {k}: {type(v)} - converting...")
                    result[k] = to_python(v)
            
            print(f"  Results: Avg Mag={result['avg_magnitude']:.1f}, Peak={result['peak_magnitude']:.1f}, Flutter={flutter_percent:.1f}%, Freq={avg_freq:.2f}±{std_freq:.2f}Hz")
            
            return result
        else:
            print("  ERROR: No data collected!")
            return None
    
    def run_full_sweep(self, start_phase=0, end_phase=345, step=15, duration_per_phase=40):
        """Run full 360° phase sweep"""
        phases = np.arange(start_phase, end_phase + step, step)
        
        print("\n" + "="*70)
        print("FLUTTER PHASE SWEEP TEST")
        print("="*70)
        print(f"Phases: {start_phase}° to {end_phase}° in {step}° steps")
        print(f"Duration per phase: {duration_per_phase}s")
        print(f"Total phases: {len(phases)}")
        print(f"Estimated time: {len(phases) * (duration_per_phase + 10) / 60:.1f} minutes")
        print(f"Flutter threshold: {self.flutter_threshold}")
        print(f"Save location: {self.save_dir}")
        print("="*70)
        
        input("\nPress Enter to start...")
        
        for i, phase in enumerate(phases):
            print(f"\n[{i+1}/{len(phases)}] Testing phase {phase}°")
            
            result = self.test_phase(float(phase), duration_per_phase)
            if result:
                self.results.append(result)
            
            if i < len(phases) - 1:
                print("Waiting 10s before next phase...")
                rospy.sleep(10.0)
        
        self.save_results()
    
    def run_custom_phases(self, phases, duration_per_phase=40):
        """Run test on specific custom phases"""
        print("\n" + "="*70)
        print("CUSTOM PHASE TEST")
        print("="*70)
        print(f"Testing phases: {phases}")
        print(f"Duration per phase: {duration_per_phase}s")
        print(f"Flutter threshold: {self.flutter_threshold}")
        print(f"Save location: {self.save_dir}")
        print("="*70)
        
        input("\nPress Enter to start...")
        
        for i, phase in enumerate(phases):
            print(f"\n[{i+1}/{len(phases)}] Testing phase {phase}°")
            
            result = self.test_phase(float(phase), duration_per_phase)
            if result:
                self.results.append(result)
            
            if i < len(phases) - 1:
                print("Waiting 10s settling time...")
                rospy.sleep(10.0)
        
        self.save_results()
    
    def save_results(self):
        """Save results to JSON file"""
        if len(self.results) == 0:
            print("\n⚠️  No results to save!")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = self.save_dir / f"flutter_phase_sweep_{timestamp}.json"
        
        try:
            # Test serialize first to catch any issues before writing file
            test_json = json.dumps(self.results, cls=NumpyEncoder)
            
            # If that worked, write to file
            with open(filepath, 'w') as f:
                f.write(test_json)
            
            print("\n" + "="*70)
            print("TEST COMPLETE!")
            print("="*70)
            print(f"✓ Results saved to: {filepath}")
            print(f"  Total phases tested: {len(self.results)}")
            
            # Quick summary
            phases = [r['phase'] for r in self.results]
            mags = [r['avg_magnitude'] for r in self.results]
            freqs = [r['avg_frequency'] for r in self.results]
            
            best_idx = int(np.argmin(mags))
            worst_idx = int(np.argmax(mags))
            
            print(f"\n  Best phase:    {phases[best_idx]}° (mag: {mags[best_idx]:.1f}, freq: {freqs[best_idx]:.2f}Hz)")
            print(f"  Worst phase:   {phases[worst_idx]}° (mag: {mags[worst_idx]:.1f}, freq: {freqs[worst_idx]:.2f}Hz)")
            print(f"  Effectiveness: {((mags[worst_idx] - mags[best_idx]) / mags[worst_idx]) * 100:.1f}% reduction")
            print(f"  Mean freq:     {np.mean(freqs):.2f} ± {np.std(freqs):.2f} Hz")
            print("="*70)
            
        except TypeError as e:
            print(f"\n❌ Serialization error: {e}")
            print("Attempting fallback save with forced type conversion...")
            
            # Fallback: force convert everything
            clean_results = json.loads(
                json.dumps(self.results, cls=NumpyEncoder, 
                          default=lambda o: float(o) if hasattr(o, '__float__') else str(o))
            )
            fallback_path = self.save_dir / f"flutter_phase_sweep_fallback_{timestamp}.json"
            with open(fallback_path, 'w') as f:
                json.dump(clean_results, f, indent=2)
            print(f"✓ Fallback save successful: {fallback_path}")
            
        except Exception as e:
            print(f"\n❌ Failed to save: {e}")
            import traceback
            traceback.print_exc()


def main():
    print("\n" + "="*70)
    print("FLUTTER PHASE SWEEP TEST")
    print("="*70)
    print("\nTest Options:")
    print("1. Full 360° sweep (0-345° in 15° steps)")
    print("2. Coarse sweep (0-345° in 30° steps)")
    print("3. Custom phases")
    print("="*70)
    
    tester = None
    
    def signal_handler(sig, frame):
        print('\n\n⚠️  Interrupt received! Saving partial results...')
        if tester and tester.results:
            tester.save_results()
        else:
            print("No results to save.")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        tester = PhaseSweepTest()
        
        choice = input("\nSelect test type (1-3): ").strip()
        
        if choice == '1':
            duration = float(input("Test duration per phase (s) [default 40]: ") or 40)
            tester.run_full_sweep(start_phase=0, end_phase=345, step=15, duration_per_phase=duration)
            
        elif choice == '2':
            duration = float(input("Test duration per phase (s) [default 40]: ") or 40)
            tester.run_full_sweep(start_phase=0, end_phase=345, step=30, duration_per_phase=duration)
            
        elif choice == '3':
            phases_str = input("Enter phases (comma-separated, e.g., 90,120,150): ")
            phases = [float(p.strip()) for p in phases_str.split(',')]
            duration = float(input("Test duration per phase (s) [default 60]: ") or 60)
            tester.run_custom_phases(phases, duration_per_phase=duration)
            
        else:
            print("Invalid choice!")
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        if tester and tester.results:
            print("\nAttempting to save partial results...")
            tester.save_results()


if __name__ == '__main__':
    main()