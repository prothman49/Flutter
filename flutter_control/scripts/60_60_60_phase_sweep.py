#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Automated Full Phase Sweep with ON/OFF Protocol
------------------------------------------------
Runs ON/OFF tests at multiple phases automatically.

Protocol per pair (per advisor recommendation):
  1. 60s settling time (control OFF)
  2. 60s baseline recording (control OFF)
  3. 60s active control recording (control ON at specified phase)

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


class AutomatedPhaseSweep:
    def __init__(self):
        rospy.init_node('automated_phase_sweep', anonymous=True)

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

        # Save location
        self.save_dir = Path('/home/flutter/catkin_ws/src/flutter_control/data/results/on_off')
        self.save_dir.mkdir(parents=True, exist_ok=True)

        rospy.sleep(2.0)
        print(f"[INFO] Automated phase sweep initialized")
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
            if time.time() - last_print >= 10.0:
                elapsed = time.time() - start
                print(f"      [{label}] {elapsed:.0f}/{duration:.0f}s | "
                      f"mag: {self.current_magnitude:.1f} | "
                      f"freq: {self.current_frequency:.2f} Hz")
                last_print = time.time()

        if len(self.magnitude_history) == 0:
            print(f"    ERROR: No data collected for {label}!")
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

    def test_single_phase(self, phase_deg, window_duration=60.0, n_pairs=3, settling_time=60.0):
        """Run ON/OFF test at a single phase."""
        print(f"\n{'='*70}")
        print(f"TESTING PHASE {phase_deg}°")
        print(f"{'='*70}")

        # Set phase
        self.phase_pub.publish(Float32(phase_deg))
        rospy.sleep(1.0)

        # Start with control OFF
        self.control_enable_pub.publish(Bool(False))
        rospy.sleep(1.0)

        pairs = []
        off_magnitudes = []
        on_magnitudes = []

        for pair_num in range(1, n_pairs + 1):
            print(f"\n  Pair {pair_num}/{n_pairs}:")

            # --- 1. Settling ---
            print(f"    [SETTLING] {settling_time:.0f}s...")
            self.control_enable_pub.publish(Bool(False))
            
            start = time.time()
            last_print = start
            while (time.time() - start) < settling_time:
                rospy.sleep(0.1)
                if time.time() - last_print >= 10.0:
                    print(f"      {time.time() - start:.0f}/{settling_time:.0f}s...")
                    last_print = time.time()

            # --- 2. Baseline ---
            print(f"    [BASELINE] {window_duration:.0f}s...")
            off_data = self.collect_window(window_duration, "BASELINE")
            if off_data is None:
                continue

            # --- 3. Control ---
            print(f"    [CONTROL ON] {window_duration:.0f}s...")
            self.control_enable_pub.publish(Bool(True))
            rospy.sleep(0.5)

            on_data = self.collect_window(window_duration, "CONTROL")
            if on_data is None:
                self.control_enable_pub.publish(Bool(False))
                continue

            reduction = (off_data['avg_magnitude'] - on_data['avg_magnitude']) / off_data['avg_magnitude'] * 100
            print(f"    → Reduction: {reduction:.1f}% ({off_data['avg_magnitude']:.1f} → {on_data['avg_magnitude']:.1f})")

            off_magnitudes.append(off_data['avg_magnitude'])
            on_magnitudes.append(on_data['avg_magnitude'])
            pairs.append({
                'pair': pair_num,
                'off': off_data,
                'on': on_data,
                'reduction_percent': round(float(reduction), 4),
            })

        # Turn off control
        self.control_enable_pub.publish(Bool(False))

        # Save results for this phase
        self.save_phase_results(phase_deg, window_duration, settling_time, n_pairs, 
                                off_magnitudes, on_magnitudes, pairs)

    def save_phase_results(self, phase_deg, window_duration, settling_time, n_pairs,
                          off_mags, on_mags, pairs):
        """Save results for a single phase."""
        if len(off_mags) < 2:
            print(f"  ⚠️  Not enough pairs for phase {phase_deg}°")
            return

        off = np.array(off_mags)
        on = np.array(on_mags)
        reductions = (off - on) / off * 100

        t_stat, p_value = stats.ttest_rel(off, on)
        cohens_d = np.mean(off - on) / np.std(off - on)

        print(f"\n  Results for {phase_deg}°:")
        print(f"    OFF:  {np.mean(off):.1f} ± {np.std(off):.1f}")
        print(f"    ON:   {np.mean(on):.1f} ± {np.std(on):.1f}")
        print(f"    Reduction: {np.mean(reductions):.1f} ± {np.std(reductions):.1f}%")
        print(f"    p = {p_value:.4f}, d = {cohens_d:.2f}")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = self.save_dir / f"flutter_on_off_phase{int(phase_deg):03d}_{timestamp}.json"

        output = {
            'test_info': {
                'phase_deg': float(phase_deg),
                'window_duration': float(window_duration),
                'settling_time': float(settling_time),
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
            'pairs': pairs,
        }

        with open(filepath, 'w') as f:
            json.dump(output, f, cls=NumpyEncoder, indent=2)
        print(f"  ✓ Saved: {filepath.name}")

    def run_full_sweep(self, phases, window_duration=60.0, n_pairs=3, settling_time=60.0):
        """Run automated sweep through all phases."""
        total_time = len(phases) * n_pairs * (settling_time + 2 * window_duration) / 60
        
        print("\n" + "="*70)
        print("AUTOMATED FULL PHASE SWEEP")
        print("="*70)
        print(f"  Phases:            {len(phases)} phases ({phases[0]}° to {phases[-1]}°)")
        print(f"  Pairs per phase:   {n_pairs}")
        print(f"  Settling time:     {settling_time}s")
        print(f"  Window duration:   {window_duration}s")
        print(f"  Time per phase:    {n_pairs * (settling_time + 2*window_duration) / 60:.1f} min")
        print(f"  Total time:        {total_time:.1f} minutes ({total_time/60:.1f} hours)")
        print(f"  Save location:     {self.save_dir}")
        print("="*70)

        input("\nPress Enter to begin automated sweep...")

        start_time = time.time()
        for i, phase in enumerate(phases, 1):
            print(f"\n\n{'#'*70}")
            print(f"# PHASE {i}/{len(phases)}: {phase}°")
            print(f"{'#'*70}")
            
            self.test_single_phase(phase, window_duration, n_pairs, settling_time)
            
            elapsed = (time.time() - start_time) / 60
            remaining = total_time - elapsed
            print(f"\n  Progress: {i}/{len(phases)} phases complete")
            print(f"  Elapsed: {elapsed:.1f} min | Remaining: {remaining:.1f} min")

        print("\n" + "="*70)
        print("SWEEP COMPLETE!")
        print("="*70)
        print(f"Total time: {(time.time() - start_time) / 60:.1f} minutes")
        print(f"Results saved to: {self.save_dir}")


def main():
    sweep = AutomatedPhaseSweep()

    print("\n" + "="*70)
    print("AUTOMATED PHASE SWEEP CONFIGURATION")
    print("="*70)

    # Get phases
    phases_input = input("\nPhases to test (comma-separated) [default: 0,30,60,90,120,150,180,210,240,270,300,330]: ").strip()
    if phases_input:
        phases = [float(p.strip()) for p in phases_input.split(',')]
    else:
        phases = [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330]

    # Get other parameters
    settling_input = input("Settling time (s) [default 60]: ").strip()
    settling_time = float(settling_input) if settling_input else 60.0

    window_input = input("Window duration (s) [default 60]: ").strip()
    window_duration = float(window_input) if window_input else 60.0

    pairs_input = input("Pairs per phase [default 3]: ").strip()
    n_pairs = int(pairs_input) if pairs_input else 3

    try:
        sweep.run_full_sweep(phases, window_duration, n_pairs, settling_time)
    except KeyboardInterrupt:
        print("\n\n⚠️  Sweep interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()