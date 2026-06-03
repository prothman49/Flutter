#!/usr/bin/env python3
"""
extract_replay_signals.py
=========================
Extracts the controlled segment piezo voltage signal from closed loop bags
and saves each as a numpy .npz file for replay.

Usage:
    python3 extract_replay_signals.py
    python3 extract_replay_signals.py --bag-dir /path/to/bags --output-dir /path/to/output
"""

import argparse
import glob
import os
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import get_typestore, Stores

TYPESTORE = get_typestore(Stores.ROS1_NOETIC)

T_UNCONTROLLED = 60.0   # seconds — controlled segment starts here
T_CONTROLLED   = 60.0   # seconds — controlled segment length


def extract_piezo(bag_path, t_start, t_end):
    """
    Extract piezo voltage from a bag between t_start and t_end (seconds from bag start).
    Returns (times, ch0, ch1) as numpy arrays with times relative to t_start.
    """
    times, ch0, ch1 = [], [], []

    with Reader(bag_path) as reader:
        t0_ns = reader.start_time
        conns = [c for c in reader.connections if c.topic == '/piezo_voltages']
        if not conns:
            print(f"  [WARN] No /piezo_voltages in {Path(bag_path).name}")
            return None

        for conn, ts, raw in reader.messages(connections=conns):
            t_sec = (ts - t0_ns) / 1e9
            if t_sec < t_start or t_sec >= t_end:
                continue
            try:
                msg = TYPESTORE.deserialize_ros1(raw, conn.msgtype)
                if hasattr(msg, 'data') and len(msg.data) >= 2:
                    times.append(t_sec - t_start)   # relative to segment start
                    ch0.append(float(msg.data[0]))
                    ch1.append(float(msg.data[1]))
            except Exception:
                continue

    if not times:
        return None

    return np.array(times), np.array(ch0), np.array(ch1)


def main():
    parser = argparse.ArgumentParser(description='Extract piezo replay signals from closed loop bags')
    parser.add_argument('--bag-dir',    type=str, default=None)
    parser.add_argument('--output-dir', type=str, default=None)
    args = parser.parse_args()

    # Auto-detect directories
    if args.bag_dir:
        bag_dir = args.bag_dir
    else:
        candidates = [
            os.path.expanduser('~/catkin_ws/src/flutter_control/data/results/bags'),
            '.',
        ]
        bag_dir = next((d for d in candidates if os.path.isdir(d)), '.')

    output_dir = args.output_dir or os.path.join(bag_dir, 'replay_signals')
    os.makedirs(output_dir, exist_ok=True)

    pattern   = os.path.join(bag_dir, 'CLOSED_LOOP_sample*.bag')
    bag_paths = sorted(glob.glob(pattern))

    if not bag_paths:
        print(f'[ERROR] No CLOSED_LOOP_sample*.bag files found in {bag_dir}')
        return

    print(f'Found {len(bag_paths)} closed loop bags')
    print(f'Output directory: {output_dir}')
    print()

    saved = []
    for bag_path in bag_paths:
        stem = Path(bag_path).stem
        print(f'Processing {stem}...')

        result = extract_piezo(bag_path, T_UNCONTROLLED, T_UNCONTROLLED + T_CONTROLLED)
        if result is None:
            print(f'  [SKIP] No piezo data')
            continue

        times, ch0, ch1 = result
        n_msgs  = len(times)
        dur     = times[-1] - times[0] if len(times) > 1 else 0
        ch0_active = np.sum(ch0 > 0.1)
        ch1_active = np.sum(ch1 > 0.1)

        print(f'  {n_msgs} messages over {dur:.1f}s')
        print(f'  ch0 active: {ch0_active} msgs | ch1 active: {ch1_active} msgs')

        out_path = os.path.join(output_dir, f'replay_{stem}.npz')
        np.savez(out_path,
                 times=times,
                 ch0=ch0,
                 ch1=ch1,
                 source_bag=str(bag_path),
                 t_start=T_UNCONTROLLED,
                 t_end=T_UNCONTROLLED + T_CONTROLLED)
        print(f'  [SAVED] {out_path}')
        saved.append(out_path)

    print(f'\nDone. Extracted {len(saved)}/{len(bag_paths)} signals.')
    print(f'Replay files saved to: {output_dir}')

    # Save manifest
    manifest_path = os.path.join(output_dir, 'replay_manifest.txt')
    with open(manifest_path, 'w') as f:
        for p in saved:
            f.write(p + '\n')
    print(f'Manifest: {manifest_path}')


if __name__ == '__main__':
    main()