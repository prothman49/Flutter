#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Flutter Control ON/OFF Validation Test
---------------------------------------
Interleaves control ON and OFF windows at a fixed phase to directly
measure control effectiveness with statistical significance.

Each pair (OFF then ON) cancels slow flutter drift.
After N pairs, runs a paired t-test to get p-value.

Save location: ~/catkin_ws/src/flutter_control/data/results/on_off/
"""

import rospy
import json
import numpy as np
from std_msgs.msg import Float32, Bool
from datetime import datetime
from pathlib import Path
from scipy import stats
import time
import signal
import sys


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, (np.integer,)): return int(obj)
        if isinstance(obj, (np.floating,)): return float(obj)
        if isinstance(obj, np.ndarray): return obj.tolist()
        if isinstance(obj, np.bool_): return bool(obj)
        return super().default(obj)


class OnOffTest:
    def __init__(self):
        rospy.init_node('flutter_on_off_test', anonymous=True)

        # Publishers
        self.phase_pub = rospy.Publisher('/flutter_control/phase_offset', Float32, queue_size=10)
        self.control_enable_pub = rospy.Publisher('/flutter_control/enable', Bool, queue_size=10)

        # Subscribers
        self.magnitude_sub = rospy.Subscriber('/dominant_magnitude', Float32, self.magnitude_callback)
        self.frequency_sub = rospy.Subscriber('/dominant_frequency', Float32, self.frequency_callback)

        # State
        self.current_magnitude = 0.0
        self.current_frequency = 0.0
        self.magnitude_history = []
        self.frequency_history = []
        self.flutter_threshold = 105.0

        # Results storage
        self.off_magnitudes = []   # Baseline (control OFF) avg mag per window
        self.on_magnitudes = []    # Controlled avg mag per window
        self.off_flutter = []      # Baseline flutter % per window
        self.on_flutter = []       # Controlled flutter % per window
        self.pairs = []            # Full data per pair

        # Save location
        self.save_dir = Path('/home/flutter/catkin_ws/src/flutter_control/data/results/on_off')
        self.save_dir.mkdir(parents=True, exist_ok=True)

        rospy.sleep(2.0)
        print(f"[INFO] ON/OFF test initialized")
        print(f"[INFO] Results will save to: {self.save_dir}")

    def magnitude_callback(self, msg):
        self.current_magnitude = msg.data
        self.magnitude_history.append(float(msg.data))

    def frequency_callback(self, msg):
        self.current_frequency = msg.data
        self.frequency_history.append(float(msg.data))

    def collect_window(self, duration, label):
        """Collect data for one window. Returns dict of stats."""
        self.magnitude_history = []
        self.frequency_history = []

        start = time.time()
        last_print = start

        while (time.time() - start) < duration:
            rospy.sleep(0.1)
            if time.time() - last_print >= 5.0:
                elapsed = time.time() - start
                print(f"    [{label}] {elapsed:.0f}/{duration:.0f}s | "
                      f"mag: {self.current_magnitude:.1f} | "
                      f"freq: {self.current_frequency:.2f} Hz")
                last_print = time.time()

        if len(self.magnitude_history) == 0:
            print(f"  ERROR: No data collected for {label}!")
            return None

        mags = np.array(self.magnitude_history, dtype=np.float64)
        freqs = [f for f in self.frequency_history if f > 0]

        flutter_count = int(sum(1 for m in self.magnitude_history if m > self.flutter_threshold))
        flutter_pct = round((flutter_count * 0.1 / duration) * 100, 3)

        return {
            'label': label,
            'avg_magnitude': round(float(np.mean(mags)), 4),
            'std_magnitude': round(float(np.std(mags)), 4),
            'peak_magnitude': round(float(np.max(mags)), 4),
            'min_magnitude': round(float(np.min(mags)), 4),
            'flutter_percent': flutter_pct,
            'flutter_count': flutter_count,
            'avg_frequency': round(float(np.mean(freqs)), 4) if freqs else 0.0,
            'std_frequency': round(float(np.std(freqs)), 4) if freqs else 0.0,
            'duration': round(float(duration), 1),
            'sample_count': len(self.magnitude_history),
        }

    def run_test(self, phase_deg=345.0, window_duration=60.0, n_pairs=10, settling_time=10.0):
        """
        Run N pairs of (OFF, ON) windows at a fixed phase.
        
        Parameters:
            phase_deg:       Phase offset for control ON windows (degrees)
            window_duration: Duration of each ON or OFF window (seconds)
            n_pairs:         Number of ON/OFF pairs to collect
            settling_time:   Wait time between switching ON/OFF (seconds)
        """
        total_time = n_pairs * (2 * window_duration + 2 * settling_time) / 60
        print("\n" + "="*65)
        print("FLUTTER CONTROL ON/OFF VALIDATION TEST")
        print("="*65)
        print(f"  Phase offset:      {phase_deg}°")
        print(f"  Window duration:   {window_duration}s each (ON and OFF)")
        print(f"  Number of pairs:   {n_pairs}")
        print(f"  Settling time:     {settling_time}s between switches")
        print(f"  Estimated total:   {total_time:.1f} minutes")
        print(f"  Save location:     {self.save_dir}")
        print("="*65)
        print("\nEach pair: [OFF baseline] → [settling] → [ON control] → [settling]")
        print("Paired t-test will be run at the end.\n")

        # Set phase before starting
        self.phase_pub.publish(Float32(phase_deg))
        rospy.sleep(1.0)
        print(f"[INFO] Phase set to {phase_deg}°")

        # Make sure control starts OFF
        self.control_enable_pub.publish(Bool(False))
        rospy.sleep(1.0)

        input("Press Enter to begin...")

        for pair_num in range(1, n_pairs + 1):
            print(f"\n{'='*65}")
            print(f"PAIR {pair_num}/{n_pairs}")
            print(f"{'='*65}")

            # --- OFF window (baseline) ---
            print(f"\n  [CONTROL OFF] Collecting {window_duration:.0f}s baseline...")
            self.control_enable_pub.publish(Bool(False))
            rospy.sleep(settling_time)  # Let system settle after switching off

            off_data = self.collect_window(window_duration, f"OFF pair {pair_num}")
            if off_data is None:
                print(f"  Skipping pair {pair_num} - no OFF data")
                continue

            print(f"  → OFF result: avg={off_data['avg_magnitude']:.1f}, "
                  f"flutter={off_data['flutter_percent']:.1f}%, "
                  f"freq={off_data['avg_frequency']:.2f}Hz")

            # --- ON window (controlled) ---
            print(f"\n  [CONTROL ON @ {phase_deg}°] Collecting {window_duration:.0f}s controlled...")
            self.control_enable_pub.publish(Bool(True))
            rospy.sleep(settling_time)  # Let control engage

            on_data = self.collect_window(window_duration, f"ON pair {pair_num}")
            if on_data is None:
                print(f"  Skipping pair {pair_num} - no ON data")
                self.control_enable_pub.publish(Bool(False))
                continue

            print(f"  → ON result:  avg={on_data['avg_magnitude']:.1f}, "
                  f"flutter={on_data['flutter_percent']:.1f}%, "
                  f"freq={on_data['avg_frequency']:.2f}Hz")

            # Compute pair reduction
            reduction = (off_data['avg_magnitude'] - on_data['avg_magnitude']) / off_data['avg_magnitude'] * 100
            print(f"\n  → Pair {pair_num} reduction: {reduction:.1f}%  "
                  f"({off_data['avg_magnitude']:.1f} → {on_data['avg_magnitude']:.1f})")

            # Store results
            self.off_magnitudes.append(off_data['avg_magnitude'])
            self.on_magnitudes.append(on_data['avg_magnitude'])
            self.off_flutter.append(off_data['flutter_percent'])
            self.on_flutter.append(on_data['flutter_percent'])
            self.pairs.append({
                'pair': pair_num,
                'off': off_data,
                'on': on_data,
                'reduction_percent': round(float(reduction), 4),
            })

        # Turn control off at end
        self.control_enable_pub.publish(Bool(False))

        self.analyze_and_save(phase_deg, window_duration, n_pairs)

    def analyze_and_save(self, phase_deg, window_duration, n_pairs):
        """Run paired t-test and save results."""
        if len(self.off_magnitudes) < 2:
            print("\n⚠️  Not enough pairs to analyze!")
            return

        off = np.array(self.off_magnitudes)
        on = np.array(self.on_magnitudes)
        reductions = np.array([(off[i] - on[i]) / off[i] * 100 for i in range(len(off))])

        # Paired t-test
        t_stat, p_value = stats.ttest_rel(off, on)

        # Effect size (Cohen's d for paired data)
        diff = off - on
        cohens_d = np.mean(diff) / np.std(diff)

        print("\n" + "="*65)
        print("STATISTICAL ANALYSIS")
        print("="*65)
        print(f"\n  Pairs collected:    {len(off)}")
        print(f"\n  OFF (baseline):")
        print(f"    Mean:             {np.mean(off):.2f} ± {np.std(off):.2f}")
        print(f"\n  ON (controlled @ {phase_deg}°):")
        print(f"    Mean:             {np.mean(on):.2f} ± {np.std(on):.2f}")
        print(f"\n  Magnitude reduction:")
        print(f"    Mean:             {np.mean(reductions):.1f}%")
        print(f"    Std:              {np.std(reductions):.1f}%")
        print(f"    Range:            {np.min(reductions):.1f}% to {np.max(reductions):.1f}%")
        print(f"\n  Paired t-test:")
        print(f"    t-statistic:      {t_stat:.4f}")
        print(f"    p-value:          {p_value:.6f}")
        print(f"    Cohen's d:        {cohens_d:.3f}")

        if p_value < 0.001:
            sig = "*** HIGHLY SIGNIFICANT (p < 0.001)"
        elif p_value < 0.01:
            sig = "** SIGNIFICANT (p < 0.01)"
        elif p_value < 0.05:
            sig = "* SIGNIFICANT (p < 0.05)"
        else:
            sig = "NOT significant (p >= 0.05)"
        print(f"\n  Result: {sig}")
        print("="*65)

        # Save JSON
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = self.save_dir / f"flutter_on_off_{timestamp}.json"

        output = {
            'test_info': {
                'phase_deg': float(phase_deg),
                'window_duration': float(window_duration),
                'n_pairs_attempted': int(n_pairs),
                'n_pairs_collected': len(off),
                'flutter_threshold': float(self.flutter_threshold),
                'timestamp': timestamp,
            },
            'statistics': {
                'off_mean': round(float(np.mean(off)), 4),
                'off_std': round(float(np.std(off)), 4),
                'on_mean': round(float(np.mean(on)), 4),
                'on_std': round(float(np.std(on)), 4),
                'mean_reduction_percent': round(float(np.mean(reductions)), 4),
                'std_reduction_percent': round(float(np.std(reductions)), 4),
                'min_reduction_percent': round(float(np.min(reductions)), 4),
                'max_reduction_percent': round(float(np.max(reductions)), 4),
                't_statistic': round(float(t_stat), 6),
                'p_value': round(float(p_value), 8),
                'cohens_d': round(float(cohens_d), 4),
                'significant_001': bool(p_value < 0.001),
                'significant_005': bool(p_value < 0.05),
            },
            'pairs': self.pairs,
        }

        try:
            test_json = json.dumps(output, cls=NumpyEncoder)
            with open(filepath, 'w') as f:
                f.write(test_json)
            print(f"\n✓ Results saved to: {filepath}")
        except Exception as e:
            print(f"\n❌ Save failed: {e}")
            import traceback
            traceback.print_exc()


def main():
    print("\n" + "="*65)
    print("FLUTTER CONTROL ON/OFF VALIDATION TEST")
    print("="*65)
    print("\nThis test alternates control OFF and ON to directly measure")
    print("the control effect with paired statistical analysis.\n")

    tester = None

    def signal_handler(sig, frame):
        print('\n\n⚠️  Interrupted! Saving partial results...')
        if tester and len(tester.off_magnitudes) >= 2:
            tester.analyze_and_save(phase_deg, window_duration, n_pairs)
        else:
            print("Not enough pairs to analyze.")
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        tester = OnOffTest()

        print("Test Parameters:")
        phase_input = input("  Phase offset in degrees [default 345]: ").strip()
        phase_deg = float(phase_input) if phase_input else 345.0

        duration_input = input("  Window duration in seconds [default 60]: ").strip()
        window_duration = float(duration_input) if duration_input else 60.0

        pairs_input = input("  Number of ON/OFF pairs [default 10]: ").strip()
        n_pairs = int(pairs_input) if pairs_input else 10

        settling_input = input("  Settling time between switches in seconds [default 10]: ").strip()
        settling_time = float(settling_input) if settling_input else 10.0

        tester.run_test(
            phase_deg=phase_deg,
            window_duration=window_duration,
            n_pairs=n_pairs,
            settling_time=settling_time,
        )

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        if tester and len(tester.off_magnitudes) >= 2:
            print("\nAttempting to save partial results...")
            tester.analyze_and_save(phase_deg, window_duration, n_pairs)


if __name__ == '__main__':
    main()