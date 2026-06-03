#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Flutter Control Experimental Protocol
======================================
Runs the full 60/60/60 protocol across all conditions, 9 samples each.

Segment order per sample:
  1. SETTLE      (60s) — control OFF, no recording, let flow stabilize
  2. UNCONTROLLED (60s) — control OFF, recording ON
  3. CONTROLLED   (60s) — control ON,  recording ON

Conditions (9 samples each, all samples of one condition before next):
  OPEN_LOOP_54 | POISSON | UNIFORM_JITTER | PHASE_RANDOMIZED

Recording uses:
  roslaunch flutter_control record_experiment.launch experiment_name:=<name>

Usage:
  rosrun flutter_control run_experiment_protocol.py
  rosrun flutter_control run_experiment_protocol.py --settle 60 --uncontrolled 60 --controlled 60
  rosrun flutter_control run_experiment_protocol.py --conditions POISSON UNIFORM_JITTER  # subset
  rosrun flutter_control run_experiment_protocol.py --start-condition UNIFORM_JITTER     # resume
  rosrun flutter_control run_experiment_protocol.py --start-sample 4                     # resume mid-condition
"""

import argparse
import json
import os
import subprocess
import sys
import time
import threading
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Float32, Bool
from flutter_control_node_replay import PhaseLockNode, CONDITIONS, CONDITION_DESCRIPTIONS

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------
ALL_CONDITIONS       = ['REPLAY']
N_SAMPLES            = 9
DEFAULT_SETTLE       = 60   # seconds
DEFAULT_UNCONTROLLED = 60
DEFAULT_CONTROLLED   = 60
DEFAULT_PHASE        = 225.0  # degrees
JSON_OUTPUT_DIR      = os.path.expanduser('~/catkin_ws/src/flutter_control/data/results/json')


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def countdown(label, seconds):
    """Print a live countdown to stdout, overwriting the same line."""
    for remaining in range(seconds, 0, -1):
        sys.stdout.write(f"\r  {label}: {remaining:3d}s remaining  ")
        sys.stdout.flush()
        time.sleep(1)
    sys.stdout.write(f"\r  {label}: DONE                      \n")
    sys.stdout.flush()


def timestamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def banner(text, width=60):
    print("\n" + "=" * width)
    print(text)
    print("=" * width)


def start_rosbag(condition, sample_num):
    """
    Launch rosbag recorder via roslaunch record_experiment.launch.
    Returns the subprocess.Popen handle — caller must terminate it.
    """
    experiment_name = f"{condition}_sample{sample_num:02d}_{timestamp()}"
    cmd = [
        "roslaunch", "flutter_control", "record_experiment.launch",
        f"experiment_name:={experiment_name}"
    ]
    print(f"  [BAG] Starting: {experiment_name}")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    # Brief pause to let roslaunch and rosbag node come up
    time.sleep(2)
    return proc, experiment_name


def stop_rosbag(proc, experiment_name):
    """Terminate the rosbag recorder gracefully."""
    if proc and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    print(f"  [BAG] Stopped: {experiment_name}")


# ---------------------------------------------------------------------------
# Data collection helpers
# ---------------------------------------------------------------------------
def snapshot_magnitudes(node):
    """Return a copy of the current magnitude history with timestamps."""
    return (
        list(node.magnitude_history),
        list(node.magnitude_time_stamps),
        list(node.threshold_exceeded_history),
        list(node.voltage_active_history),
    )


def save_sample_json(condition, sample, bag_name, phase_offset,
                     uncontrolled_snap, controlled_snap,
                     t_uncontrolled, t_controlled, flutter_threshold):
    """
    Compute per-segment stats and write one JSON file per sample.
    uncontrolled_snap / controlled_snap: tuples from snapshot_magnitudes()
    """
    os.makedirs(JSON_OUTPUT_DIR, exist_ok=True)

    def seg_stats(mags, thresh_arr, volt_arr, duration):
        mags      = np.array(mags, dtype=float)
        thresh    = np.array(thresh_arr, dtype=bool)
        volt      = np.array(volt_arr,   dtype=bool)
        n         = len(mags)
        if n == 0:
            return {}
        flutter_frac = float(np.sum(thresh) / n) if n > 0 else 0.0
        return {
            'n_samples':            int(n),
            'mean_magnitude':       float(np.mean(mags)),
            'std_magnitude':        float(np.std(mags)),
            'peak_magnitude':       float(np.max(mags)),
            'min_magnitude':        float(np.min(mags)),
            'flutter_fraction':     flutter_frac,
            'flutter_time_s':       float(flutter_frac * duration),
            'control_active_frac':  float(np.sum(volt) / n) if n > 0 else 0.0,
        }

    unc_mags, unc_times, unc_thresh, unc_volt = uncontrolled_snap
    ctl_mags, ctl_times, ctl_thresh, ctl_volt = controlled_snap

    unc_stats = seg_stats(unc_mags, unc_thresh, unc_volt, t_uncontrolled)
    ctl_stats = seg_stats(ctl_mags, ctl_thresh, ctl_volt, t_controlled)

    # Reduction metrics
    mag_reduction    = 0.0
    flutter_reduction = 0.0
    if unc_stats.get('mean_magnitude', 0) > 0:
        mag_reduction = (
            (unc_stats['mean_magnitude'] - ctl_stats.get('mean_magnitude', 0))
            / unc_stats['mean_magnitude'] * 100
        )
    if unc_stats.get('flutter_fraction', 0) > 0:
        flutter_reduction = (
            (unc_stats['flutter_fraction'] - ctl_stats.get('flutter_fraction', 0))
            / unc_stats['flutter_fraction'] * 100
        )

    result = {
        'meta': {
            'condition':        condition,
            'sample':           sample,
            'bag_name':         bag_name,
            'phase_offset_deg': phase_offset,
            'flutter_threshold': flutter_threshold,
            'timestamp':        timestamp(),
            't_uncontrolled_s': t_uncontrolled,
            't_controlled_s':   t_controlled,
        },
        'uncontrolled': unc_stats,
        'controlled':   ctl_stats,
        'reduction': {
            'magnitude_reduction_pct': float(mag_reduction),
            'flutter_reduction_pct':   float(flutter_reduction),
        },
    }

    filename = os.path.join(
        JSON_OUTPUT_DIR,
        f"{condition}_sample{sample:02d}_{timestamp()}.json"
    )
    with open(filename, 'w') as f:
        json.dump(result, f, indent=2)

    print(f"  [JSON] Saved: {filename}")
    print(f"  [JSON] Mag reduction: {mag_reduction:.1f}% | Flutter reduction: {flutter_reduction:.1f}%")
    return result


# ---------------------------------------------------------------------------
# Protocol runner
# ---------------------------------------------------------------------------
def timed_segment(node, seconds, label):
    """
    Run node.update() + GUI event loop for `seconds` seconds,
    collecting magnitude snapshots. Returns snapshot tuple.
    """
    end_time = time.time() + seconds
    snap_mags, snap_times, snap_thresh, snap_volt = [], [], [], []

    while time.time() < end_time:
        remaining = int(end_time - time.time())
        sys.stdout.write(f"\r  {label}: {remaining:3d}s remaining  ")
        sys.stdout.flush()

        node.update()
        plt.pause(0.1)

        # Grab latest data point
        if node.magnitude_history:
            snap_mags.append(node.magnitude_history[-1])
            snap_times.append(node.magnitude_time_stamps[-1])
            snap_thresh.append(node.threshold_exceeded_history[-1])
            snap_volt.append(node.voltage_active_history[-1])

    sys.stdout.write(f"\r  {label}: DONE                      \n")
    sys.stdout.flush()
    return snap_mags, snap_times, snap_thresh, snap_volt


def run_protocol(node, conditions, n_samples, t_settle, t_uncontrolled, t_controlled):
    total_samples  = len(conditions) * n_samples
    sample_counter = 0
    start_wall     = time.time()

    for condition in conditions:
        banner(f"CONDITION: {condition}\n  {CONDITION_DESCRIPTIONS[condition]}")
        node.set_condition(condition)

        for sample in range(1, n_samples + 1):
            sample_counter += 1

            banner(
                f"  Sample {sample}/{n_samples}  |  {condition}\n"
                f"  Overall: {sample_counter}/{total_samples}",
                width=60
            )

            elapsed_total     = time.time() - start_wall
            remaining_samples = total_samples - sample_counter
            if sample_counter > 1 and remaining_samples > 0:
                eta_seconds = (elapsed_total / sample_counter) * remaining_samples
                print(f"  ETA: ~{int(eta_seconds//60)}m {int(eta_seconds%60)}s remaining")

            # ----------------------------------------------------------
            # SEGMENT 1 — SETTLE (no recording, GUI live)
            # ----------------------------------------------------------
            print(f"\n[1/3] SETTLE ({t_settle}s) — no control, no recording")
            node.control_enabled = False
            node.stop_waveform()
            timed_segment(node, t_settle, "Settle")

            # ----------------------------------------------------------
            # SEGMENT 2 — UNCONTROLLED (recording ON, control OFF)
            # ----------------------------------------------------------
            print(f"\n[2/3] UNCONTROLLED ({t_uncontrolled}s) — recording, no control")
            node.control_enabled = False
            node.stop_waveform()
            bag_proc, bag_name = start_rosbag(condition, sample)

            uncontrolled_snap = timed_segment(node, t_uncontrolled, "Uncontrolled")

            # ----------------------------------------------------------
            # SEGMENT 3 — CONTROLLED (recording continues, control ON)
            # ----------------------------------------------------------
            print(f"\n[3/3] CONTROLLED ({t_controlled}s) — recording, control ON | {condition}")
            node.control_enabled = True

            controlled_snap = timed_segment(node, t_controlled, "Controlled")

            # ----------------------------------------------------------
            # End of sample
            # ----------------------------------------------------------
            node.control_enabled = False
            node.stop_waveform()
            stop_rosbag(bag_proc, bag_name)

            save_sample_json(
                condition, sample, bag_name, node.phase_offset_degrees,
                uncontrolled_snap, controlled_snap,
                t_uncontrolled, t_controlled, node.flutter_threshold
            )

            print(f"\n  ✓ Sample {sample}/{n_samples} complete [{condition}]")

        print(f"\n{'='*60}")
        print(f"  CONDITION COMPLETE: {condition} ({n_samples} samples)")
        print(f"{'='*60}")

    total_time = time.time() - start_wall
    banner(
        f"SESSION COMPLETE\n"
        f"  Conditions: {len(conditions)}\n"
        f"  Samples:    {total_samples}\n"
        f"  Duration:   {int(total_time//60)}m {int(total_time%60)}s"
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Flutter Control Experimental Protocol')
    parser.add_argument('--settle',       type=int,   default=DEFAULT_SETTLE,
                        help=f'Settle period in seconds (default {DEFAULT_SETTLE})')
    parser.add_argument('--uncontrolled', type=int,   default=DEFAULT_UNCONTROLLED,
                        help=f'Uncontrolled period in seconds (default {DEFAULT_UNCONTROLLED})')
    parser.add_argument('--controlled',   type=int,   default=DEFAULT_CONTROLLED,
                        help=f'Controlled period in seconds (default {DEFAULT_CONTROLLED})')
    parser.add_argument('--conditions',   type=str,   nargs='+', default=ALL_CONDITIONS,
                        choices=ALL_CONDITIONS,
                        help='Conditions to run (default: all four)')
    parser.add_argument('--samples',      type=int,   default=N_SAMPLES,
                        help=f'Samples per condition (default {N_SAMPLES})')
    parser.add_argument('--phase',        type=float, default=DEFAULT_PHASE,
                        help=f'Phase offset in degrees (default {DEFAULT_PHASE})')
    parser.add_argument('--start-condition', type=str, default=None,
                        choices=ALL_CONDITIONS,
                        help='Resume from this condition (skips earlier ones)')
    parser.add_argument('--start-sample', type=int, default=1,
                        help='Resume from this sample number within the start condition')
    args, _ = parser.parse_known_args()

    # Handle resume — trim condition list
    conditions = args.conditions
    if args.start_condition:
        if args.start_condition not in conditions:
            print(f"[ERROR] --start-condition {args.start_condition} not in selected conditions")
            sys.exit(1)
        idx = conditions.index(args.start_condition)
        conditions = conditions[idx:]
        print(f"[RESUME] Starting from condition {args.start_condition}")

    # Estimate total time
    t_per_sample = args.settle + args.uncontrolled + args.controlled + 2  # +2 for bag startup
    total_samples = len(conditions) * args.samples
    # Adjust for start-sample offset on first condition
    if args.start_sample > 1 and args.start_condition:
        total_samples -= (args.start_sample - 1)
    eta_min = (t_per_sample * total_samples) // 60

    banner(
        f"FLUTTER CONTROL EXPERIMENTAL PROTOCOL\n"
        f"  Conditions:   {', '.join(conditions)}\n"
        f"  Samples each: {args.samples}\n"
        f"  Settle:       {args.settle}s\n"
        f"  Uncontrolled: {args.uncontrolled}s\n"
        f"  Controlled:   {args.controlled}s\n"
        f"  Phase offset: {args.phase} deg\n"
        f"  Est. total:   ~{eta_min} min"
    )

    print("\nMake sure:")
    print("  - camera_only.launch is running (T1)")
    print("  - Airflow is on and stable")
    print("  - Wing is mounted")
    print("\nStarting in 5 seconds — Ctrl+C to abort")
    countdown("Starting", 5)

    # --- ROS init ---
    rospy.init_node('flutter_experiment_protocol', anonymous=True)
    node = PhaseLockNode(condition=conditions[0], enable_gui=True)
    node.set_phase(args.phase)

    spin_thread = threading.Thread(target=rospy.spin)
    spin_thread.daemon = True
    spin_thread.start()

    plt.show(block=False)

    # Give node a moment to initialize and fill buffer
    print("\n[INFO] Waiting for optic flow buffer to fill...")
    wait_start = time.time()
    while len(node.buffer) < node.buffer_size and not rospy.is_shutdown():
        node.update()
        plt.pause(0.1)
        if time.time() - wait_start > 30:
            print("[WARN] Buffer fill timeout — proceeding anyway")
            break
    print("[INFO] Buffer ready, starting protocol")

    # --- Run ---
    try:
        # Handle start-sample offset
        if args.start_sample > 1 and args.start_condition and conditions[0] == args.start_condition:
            print(f"[RESUME] Skipping to sample {args.start_sample} of {conditions[0]}")
            # We need a modified first condition run; handle inline
            first_condition  = conditions[0]
            remaining_conditions = conditions[1:]

            node.set_condition(first_condition)
            banner(f"CONDITION: {first_condition} (resuming at sample {args.start_sample})")

            for sample in range(args.start_sample, args.samples + 1):
                _run_sample(node, first_condition, sample, args.samples,
                            args.settle, args.uncontrolled, args.controlled)

            run_protocol(node, remaining_conditions, args.samples,
                         args.settle, args.uncontrolled, args.controlled)
        else:
            run_protocol(node, conditions, args.samples,
                         args.settle, args.uncontrolled, args.controlled)

    except KeyboardInterrupt:
        print("\n\n[ABORT] Protocol interrupted by user")
    finally:
        node.control_enabled = False
        node.cleanup()


def _run_sample(node, condition, sample, n_samples, t_settle, t_uncontrolled, t_controlled):
    """Single sample runner, extracted so resume logic can call it."""
    banner(f"  Sample {sample}/{n_samples}  |  {condition}")

    print(f"\n[1/3] SETTLE ({t_settle}s) — no control, no recording")
    node.control_enabled = False
    node.stop_waveform()
    timed_segment(node, t_settle, "Settle")

    print(f"\n[2/3] UNCONTROLLED ({t_uncontrolled}s) — recording, no control")
    node.control_enabled = False
    node.stop_waveform()
    bag_proc, bag_name = start_rosbag(condition, sample)
    uncontrolled_snap = timed_segment(node, t_uncontrolled, "Uncontrolled")

    print(f"\n[3/3] CONTROLLED ({t_controlled}s) — recording, control ON")
    node.control_enabled = True
    controlled_snap = timed_segment(node, t_controlled, "Controlled")

    node.control_enabled = False
    node.stop_waveform()
    stop_rosbag(bag_proc, bag_name)

    save_sample_json(
        condition, sample, bag_name, node.phase_offset_degrees,
        uncontrolled_snap, controlled_snap,
        t_uncontrolled, t_controlled, node.flutter_threshold
    )

    print(f"\n  ✓ Sample {sample}/{n_samples} complete [{condition}]")


if __name__ == '__main__':
    main()