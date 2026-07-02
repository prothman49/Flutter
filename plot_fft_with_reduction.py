#!/usr/bin/env python3
"""
plot_phase_triggered_avg.py
============================
Phase-triggered averaging analysis for all flutter control conditions.

For each condition, loads all available bag files, finds negative peaks
in /optic_flow_mean_vx (one per flutter cycle), extracts windows around
each peak, and overlays + averages them. Does the same for:
  - /optic_flow_mean_vx    (raw wing signal)
  - /amplitude_envelope    (Hilbert envelope)
  - /phase_mod             (instantaneous phase)
  - /piezo_voltages        (actuator output, ch0 and ch1)

Separate figures for uncontrolled and controlled segments.

Usage:
    python3 plot_phase_triggered_avg.py
    python3 plot_phase_triggered_avg.py --bag-dir /path/to/bags
    python3 plot_phase_triggered_avg.py --window 0.4 --min-peaks 5
"""

import argparse
import os
import glob
from pathlib import Path
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.signal import find_peaks, butter, filtfilt
from rosbags.rosbag1 import Reader
from rosbags.typesys import get_typestore, Stores

TYPESTORE = get_typestore(Stores.ROS1_NOETIC)

matplotlib.rcParams.update({
    'font.family':    'sans-serif',
    'font.size':      9,
    'axes.labelsize': 9,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'axes.linewidth': 0.8,
    'lines.linewidth': 1.2,
    'pdf.fonttype':   42,
    'ps.fonttype':    42,
})

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
FLUTTER_FREQ   = 5.4
PULSE_PERIOD   = 1.0 / FLUTTER_FREQ   # ~0.185s

# Segment boundaries (seconds from bag start)
# Protocol: 60s uncontrolled then 60s controlled
T_UNCONTROLLED = 60.0
T_CONTROLLED   = 60.0

CONDITIONS = ['CLOSED_LOOP', 'OPEN_LOOP_54', 'POISSON', 'UNIFORM_JITTER', 'PHASE_RANDOMIZED', 'REPLAY']

CONDITION_LABELS = {
    'CLOSED_LOOP':       'Closed Loop',
    'OPEN_LOOP_54':      'Open Loop 5.4 Hz',
    'POISSON':           'Poisson',
    'UNIFORM_JITTER':    'Uniform Jitter',
    'PHASE_RANDOMIZED':  'Polarity Randomized',
    'REPLAY':            'Replay',
}

CONDITION_COLORS = {
    'CLOSED_LOOP':       '#4CAF50',
    'OPEN_LOOP_54':      '#2196F3',
    'POISSON':           '#FF9800',
    'UNIFORM_JITTER':    '#9C27B0',
    'PHASE_RANDOMIZED':  '#F44336',
    'REPLAY':            '#795548',
}

TOPICS = [
    '/optic_flow_mean_vx',
    '/amplitude_envelope',
    '/phase_mod',
    '/piezo_voltages',
]

TOPIC_LABELS = {
    '/optic_flow_mean_vx': 'Optic flow (raw)',
    '/amplitude_envelope': 'Amplitude envelope',
    '/phase_mod':          'Instantaneous phase (rad)',
    '/piezo_voltages':     'Piezo voltage (V)',
}

TOPIC_COLORS = {
    '/optic_flow_mean_vx': '#2c7bb6',
    '/amplitude_envelope': '#1a9641',
    '/phase_mod':          '#d4a017',
    '/piezo_voltages':     '#d7191c',
}


# ---------------------------------------------------------------------------
# Bag reading
# ---------------------------------------------------------------------------
def read_bag(bag_path):
    """
    Read a rosbag1 file and return a dict of topic -> (times_ns, values).
    For /piezo_voltages (Float32MultiArray), returns (times_ns, ch0, ch1).
    """
    data = defaultdict(lambda: {'t': [], 'v': []})

    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            topic = connection.topic
            if topic not in TOPICS:
                continue
            try:
                msg = TYPESTORE.deserialize_ros1(rawdata, connection.msgtype)
                if topic == '/piezo_voltages':
                    if hasattr(msg, 'data') and len(msg.data) >= 2:
                        data[topic]['t'].append(timestamp)
                        data[topic]['v'].append([float(msg.data[0]), float(msg.data[1])])
                else:
                    data[topic]['t'].append(timestamp)
                    data[topic]['v'].append(float(msg.data))
            except Exception:
                continue

    # Convert to numpy, normalize time to seconds from bag start
    result = {}
    all_times = []
    for topic, d in data.items():
        if d['t']:
            all_times.extend(d['t'])

    if not all_times:
        return {}

    t0 = min(all_times)

    for topic, d in data.items():
        if not d['t']:
            continue
        t = (np.array(d['t']) - t0) / 1e9  # ns -> seconds
        if topic == '/piezo_voltages':
            v = np.array(d['v'])
            result[topic] = {
                't':   t,
                'ch0': v[:, 0],
                'ch1': v[:, 1],
            }
        else:
            result[topic] = {
                't': t,
                'v': np.array(d['v']),
            }

    return result


# ---------------------------------------------------------------------------
# Signal interpolation onto uniform grid
# ---------------------------------------------------------------------------
def interp_uniform(t, v, fs=100.0):
    """Interpolate irregularly sampled signal onto uniform grid at fs Hz."""
    if len(t) < 2:
        return np.array([]), np.array([])
    t_uniform = np.arange(t[0], t[-1], 1.0 / fs)
    v_uniform = np.interp(t_uniform, t, v)
    return t_uniform, v_uniform


# ---------------------------------------------------------------------------
# Find negative peaks (troughs) in optic flow
# ---------------------------------------------------------------------------
def find_troughs(t, v, fs=100.0, min_separation=None):
    """
    Find negative peaks (troughs) in signal.
    min_separation: minimum samples between peaks (default ~0.8 flutter periods)
    """
    if min_separation is None:
        min_separation = int(0.8 / PULSE_PERIOD * fs)  # 80% of one period

    # Find peaks of the negated signal = troughs of original
    peaks, props = find_peaks(
        -v,
        height=np.std(v) * 0.5,        # must be at least 0.5 std below mean
        distance=min_separation,
    )
    return peaks


# ---------------------------------------------------------------------------
# Extract epochs around trigger points
# ---------------------------------------------------------------------------
def extract_epochs(t_uniform, v_uniform, trigger_indices, window_s, fs=100.0):
    """
    Extract fixed-length windows starting at each trigger index (trough at t=0).
    Time axis runs from 0 to window_s with trough at the left edge.
    Returns array of shape (n_epochs, n_samples).
    """
    n_samples = int(window_s * fs) + 1
    epochs = []

    for idx in trigger_indices:
        i0 = idx
        i1 = idx + n_samples
        if i0 < 0 or i1 > len(v_uniform):
            continue
        epochs.append(v_uniform[i0:i1])

    time_axis = np.linspace(0, window_s, n_samples)

    if not epochs:
        return np.array([]), time_axis

    return np.array(epochs), time_axis


# ---------------------------------------------------------------------------
# Load and process all bags for one condition
# ---------------------------------------------------------------------------
def process_condition(bag_paths, window_s=0.4, fs=100.0, min_peaks=3):
    """
    Process all bags for one condition.
    Returns dict with epochs and averages for each topic, split by segment.
    """
    results = {
        'uncontrolled': defaultdict(list),
        'controlled':   defaultdict(list),
    }

    for bag_path in bag_paths:
        print(f"    Reading {Path(bag_path).name}")
        data = read_bag(bag_path)

        if '/optic_flow_mean_vx' not in data:
            print(f"    [SKIP] No optic flow topic")
            continue

        # Get optic flow on uniform grid
        t_raw = data['/optic_flow_mean_vx']['t']
        v_raw = data['/optic_flow_mean_vx']['v']
        t_u, v_u = interp_uniform(t_raw, v_raw, fs=fs)

        if len(t_u) < 10:
            continue

        # Split into uncontrolled and controlled segments
        for seg_name, t_start, t_end in [
            ('uncontrolled', 0.0,            T_UNCONTROLLED),
            ('controlled',   T_UNCONTROLLED,  T_UNCONTROLLED + T_CONTROLLED),
        ]:
            mask = (t_u >= t_start) & (t_u < t_end)
            if np.sum(mask) < int(fs * 2):
                continue

            t_seg = t_u[mask]
            v_seg = v_u[mask]
            idx_offset = np.where(mask)[0][0]

            # Find troughs in this segment
            troughs = find_troughs(t_seg, v_seg, fs=fs)
            if len(troughs) < min_peaks:
                print(f"    [WARN] {seg_name}: only {len(troughs)} troughs found, skipping")
                continue

            # For each topic, extract epochs aligned to troughs
            for topic in TOPICS:
                if topic not in data:
                    continue

                if topic == '/piezo_voltages':
                    for ch_key, ch_name in [('ch0', 'piezo_ch0'), ('ch1', 'piezo_ch1')]:
                        t_topic = data[topic]['t']
                        v_topic = data[topic][ch_key]
                        # Only use times within this segment
                        seg_mask = (t_topic >= t_start) & (t_topic < t_end)
                        if np.sum(seg_mask) < 5:
                            continue
                        t_t_u, v_t_u = interp_uniform(
                            t_topic[seg_mask] - t_start,
                            v_topic[seg_mask], fs=fs
                        )
                        # Map trough indices to this topic's grid
                        trough_times = t_seg[troughs] - t_start
                        trough_idx_mapped = np.array([
                            np.argmin(np.abs(t_t_u - tt)) for tt in trough_times
                        ])
                        epochs, time_axis = extract_epochs(
                            t_t_u, v_t_u, trough_idx_mapped, window_s, fs
                        )
                        if len(epochs):
                            results[seg_name][ch_name].extend(epochs)
                            results[seg_name][f'{ch_name}_time'] = time_axis
                else:
                    t_topic = data[topic]['t']
                    v_topic = data[topic]['v']
                    seg_mask = (t_topic >= t_start) & (t_topic < t_end)
                    if np.sum(seg_mask) < 5:
                        continue
                    t_t_u, v_t_u = interp_uniform(
                        t_topic[seg_mask] - t_start,
                        v_topic[seg_mask], fs=fs
                    )
                    trough_times = t_seg[troughs] - t_start
                    trough_idx_mapped = np.array([
                        np.argmin(np.abs(t_t_u - tt)) for tt in trough_times
                    ])
                    epochs, time_axis = extract_epochs(
                        t_t_u, v_t_u, trough_idx_mapped, window_s, fs
                    )
                    if len(epochs):
                        results[seg_name][topic].extend(epochs)
                        results[seg_name][f'{topic}_time'] = time_axis

    # Convert lists to arrays
    for seg in results:
        for key in list(results[seg].keys()):
            if not key.endswith('_time') and isinstance(results[seg][key], list):
                if results[seg][key]:
                    results[seg][key] = np.array(results[seg][key])
                else:
                    results[seg][key] = np.array([])

    return results


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_condition(condition, results, output_dir, window_s):
    """
    Plot phase-triggered average for one condition.
    Two panels: uncontrolled (left) and controlled (right).
    Rows: optic flow, envelope, phase, piezo ch0, piezo ch1.
    """
    color = CONDITION_COLORS[condition]
    label = CONDITION_LABELS[condition]

    row_topics = [
        ('/optic_flow_mean_vx', 'Optic flow',  '#2c7bb6'),
        # ('/amplitude_envelope', 'Amplitude envelope', '#1a9641'),
        ('/phase_mod',          'Phase (rad)', '#d4a017'),
        ('piezo_ch0',           'Piezo ch0 (V)', color),
        # ('piezo_ch1',           'Piezo ch1 (V)', color),
    ]

    n_rows = len(row_topics)
    fig, axes = plt.subplots(
        n_rows, 2,
        figsize=(12, 2.2 * n_rows),
        gridspec_kw={'hspace': 0.55, 'wspace': 0.3}
    )
    fig.suptitle(
        f'Phase-Triggered Average — {label}',
        fontsize=11, y=0.99
    )

    for col, seg_name in enumerate(['uncontrolled', 'controlled']):
        seg_data = results[seg_name]
        axes[0, col].set_title(
            f'{"Uncontrolled" if col == 0 else "Controlled"} segment',
            fontsize=10, pad=6
        )

        for row, (topic_key, topic_label, topic_color) in enumerate(row_topics):
            ax = axes[row, col]

            epochs = seg_data.get(topic_key, np.array([]))
            time_key = f'{topic_key}_time'
            time_axis = seg_data.get(time_key, None)

            if time_axis is None or not len(epochs):
                ax.text(0.5, 0.5, 'No data', ha='center', va='center',
                        transform=ax.transAxes, color='#aaa', fontsize=8)
                ax.set_ylabel(topic_label, fontsize=8)
                ax.spines[['top', 'right']].set_visible(False)
                continue

            # Plot individual epochs (faint)
            for epoch in epochs:
                ax.plot(time_axis, epoch, color=topic_color, alpha=0.08, lw=0.7)

            # Plot mean
            mean = np.mean(epochs, axis=0)
            std  = np.std(epochs, axis=0)
            ax.plot(time_axis, mean, color=topic_color, lw=2.0, label=f'Mean (n={len(epochs)})')
            ax.fill_between(time_axis, mean - std, mean + std,
                            color=topic_color, alpha=0.18, label='±1 SD')

            # Mark trigger (t=0 = trough)
            ax.axvline(0, color='#555', lw=0.8, ls='--', alpha=0.7)

            # Mark 1 and 2 flutter periods after trough
            for tp in [PULSE_PERIOD, 2 * PULSE_PERIOD]:
                if tp <= window_s:
                    ax.axvline(tp, color='#aaa', lw=0.5, ls=':', alpha=0.5)

            ax.set_ylabel(topic_label, fontsize=8)
            ax.spines[['top', 'right']].set_visible(False)
            ax.tick_params(labelbottom=True, bottom=True)
            ax.tick_params(labelleft=True, left=True)
            ax.xaxis.set_tick_params(labelsize=7)
            ax.yaxis.set_tick_params(labelsize=7)
            ax.xaxis.set_major_locator(plt.MaxNLocator(4))

            # Fixed y-limits for optic flow — cap at ±10
            if topic_key == '/optic_flow_mean_vx':
                ax.set_ylim(-10, 10)

            if row == n_rows - 1:
                ax.set_xlabel('Time from trough (s)', fontsize=8)

            if row == 0:
                ax.legend(fontsize=7, loc='upper right')

    # After plotting all panels, match y-limits across columns for each row
    for row, (topic_key, _, _) in enumerate(row_topics):
        if topic_key == '/optic_flow_mean_vx':
            continue  # already fixed at ±10
        y_min = min(axes[row, col].get_ylim()[0] for col in range(2))
        y_max = max(axes[row, col].get_ylim()[1] for col in range(2))
        for col in range(2):
            axes[row, col].set_ylim(y_min, y_max)

    # Save
    stem = f'phase_triggered_{condition.lower()}'
    for ext in ['pdf', 'png']:
        path = os.path.join(output_dir, f'{stem}.{ext}')
        plt.savefig(path, dpi=150, bbox_inches='tight')
    print(f'  [SAVED] {stem}.pdf / .png')
    plt.close(fig)


def plot_comparison(all_results, output_dir, window_s, segment='controlled'):
    """
    Single figure comparing all conditions side by side for the controlled segment.
    Rows = topics, columns = conditions.
    """
    row_topics = [
        ('/optic_flow_mean_vx', 'Optic flow'),
        # ('/amplitude_envelope', 'Amplitude envelope'),
        ('/phase_mod',          'Phase (rad)'),
        ('piezo_ch0',           'Piezo ch0 (V)'),
        # ('piezo_ch1',           'Piezo ch1 (V)'),
    ]

    # Only plot conditions that have data
    active_conditions = [c for c in CONDITIONS if c in all_results]
    n_rows = len(row_topics)
    n_cols = len(active_conditions)

    fig, axes = plt.subplots(
        n_rows, n_cols,
        figsize=(3.2 * n_cols, 2.4 * n_rows),
        gridspec_kw={'hspace': 0.55, 'wspace': 0.35}
    )
    if n_cols == 1:
        axes = axes.reshape(n_rows, 1)

    # Build filename from condition names
    cond_slug = '_vs_'.join(c.lower() for c in active_conditions)
    fig_stem  = f'phase_triggered_{cond_slug}_{segment}'

    fig.suptitle(
        f'Phase-Triggered Average — {" vs ".join(CONDITION_LABELS[c] for c in active_conditions)} — {segment.title()} Segment',
        fontsize=11, y=0.99
    )

    for col, condition in enumerate(active_conditions):
        color = CONDITION_COLORS[condition]
        label = CONDITION_LABELS[condition]
        axes[0, col].set_title(label, fontsize=9, color=color, pad=6)

        if condition not in all_results:
            continue

        seg_data = all_results[condition][segment]

        for row, (topic_key, topic_label) in enumerate(row_topics):
            ax = axes[row, col]

            epochs    = seg_data.get(topic_key, np.array([]))
            time_axis = seg_data.get(f'{topic_key}_time', None)

            if time_axis is None or not len(epochs):
                ax.text(0.5, 0.5, 'No data', ha='center', va='center',
                        transform=ax.transAxes, color='#aaa', fontsize=7)
            else:
                for epoch in epochs:
                    ax.plot(time_axis, epoch, color=color, alpha=0.07, lw=0.6)
                mean = np.mean(epochs, axis=0)
                std  = np.std(epochs, axis=0)
                ax.plot(time_axis, mean, color=color, lw=1.8)
                ax.fill_between(time_axis, mean - std, mean + std,
                                color=color, alpha=0.18)
                ax.axvline(0, color='#555', lw=0.7, ls='--', alpha=0.7)
                for tp in [PULSE_PERIOD, 2 * PULSE_PERIOD]:
                    if tp <= window_s:
                        ax.axvline(tp, color='#ccc', lw=0.5, ls=':')

            ax.spines[['top', 'right']].set_visible(False)

            # Fixed y-limits for optic flow — cap at ±10 to clip outlier epochs
            if topic_key == '/optic_flow_mean_vx':
                ax.set_ylim(-10, 10)

            # Y-axis: label on left column only, ticks on all
            if col == 0:
                ax.set_ylabel(topic_label, fontsize=8)
            ax.tick_params(labelleft=True, left=True)
            ax.yaxis.set_tick_params(labelsize=7)

            # X-axis: label on bottom row only, ticks on all
            if row == n_rows - 1:
                ax.set_xlabel('Time from trough (s)', fontsize=7)
            ax.tick_params(labelbottom=True, bottom=True)
            ax.xaxis.set_tick_params(labelsize=7)
            ax.xaxis.set_major_locator(plt.MaxNLocator(4))

    for ext in ['pdf', 'png']:
        path = os.path.join(output_dir, f'{fig_stem}.{ext}')
        plt.savefig(path, dpi=150, bbox_inches='tight')
    print(f'  [SAVED] {fig_stem}.pdf / .png')
    plt.close(fig)


def plot_combined(all_results, output_dir, window_s):
    """
    Combined figure: 5 rows × N conditions
    Row 1: Optic flow — Uncontrolled
    Row 2: Phase — Uncontrolled
    Row 3: Optic flow — Controlled
    Row 4: Phase — Controlled
    Row 5: Piezo ch0 — Controlled
    """
    active_conditions = [c for c in CONDITIONS if c in all_results]
    n_cols = len(active_conditions)

    ROW_DEFS = [
        ('uncontrolled', '/optic_flow_mean_vx', 'Optic flow\n(Uncontrolled)'),
        ('uncontrolled', '/phase_mod',           'Phase\n(Uncontrolled)'),
        ('controlled',   '/optic_flow_mean_vx', 'Optic flow\n(Controlled)'),
        ('controlled',   '/phase_mod',           'Phase\n(Controlled)'),
        ('controlled',   'piezo_ch0',            'Piezo ch0\n(Controlled)'),
    ]
    n_rows = len(ROW_DEFS)

    fig, axes = plt.subplots(
        n_rows, n_cols,
        figsize=(3.0 * n_cols, 2.2 * n_rows),
        gridspec_kw={'hspace': 0.45, 'wspace': 0.25}
    )
    if n_cols == 1:
        axes = axes.reshape(n_rows, 1)

    cond_slug = '_vs_'.join(c.lower() for c in active_conditions)
    fig_stem  = f'phase_triggered_combined_{cond_slug}'

    fig.suptitle(
        'Phase-Triggered Average — ' + ' vs '.join(CONDITION_LABELS[c] for c in active_conditions),
        fontsize=10, y=1.01
    )

    # Dividing line between uncontrolled and controlled
    fig.add_artist(plt.Line2D(
        [0.02, 0.98], [1 - 2/n_rows, 1 - 2/n_rows],
        transform=fig.transFigure, color='#aaa', lw=1.0, ls='--'
    ))

    n_samples = int(window_s * 100) + 1
    t_axis    = np.linspace(0, window_s, n_samples)

    for col, condition in enumerate(active_conditions):
        color = CONDITION_COLORS[condition]
        axes[0, col].set_title(CONDITION_LABELS[condition],
                               fontsize=9, color=color, fontweight='bold', pad=4)

        for row, (segment, topic_key, row_label) in enumerate(ROW_DEFS):
            ax   = axes[row, col]
            data = all_results[condition][segment]

            seg_color = '#555555' if segment == 'uncontrolled' else color

            # Get epochs
            if topic_key == 'piezo_ch0':
                epochs = data.get('piezo_ch0', [])
            else:
                epochs = data.get(topic_key, [])

            if epochs:
                mat  = np.array([e for e in epochs if len(e) == n_samples])
                if mat.shape[0] > 0:
                    for ep in mat:
                        ax.plot(t_axis, ep, color=seg_color, alpha=0.10, lw=0.5, zorder=2)
                    mean = np.mean(mat, axis=0)
                    std  = np.std(mat, axis=0)
                    ax.plot(t_axis, mean, color=seg_color, lw=1.8, zorder=4)
                    ax.fill_between(t_axis, mean - std, mean + std,
                                    color=seg_color, alpha=0.20, zorder=3)

            ax.axvline(0, color='#666', lw=0.8, ls='--', alpha=0.6)
            for tp in [PULSE_PERIOD, 2 * PULSE_PERIOD]:
                if tp <= window_s:
                    ax.axvline(tp, color='#ccc', lw=0.5, ls=':', zorder=1)

            ax.spines[['top', 'right']].set_visible(False)
            ax.xaxis.set_tick_params(labelsize=7)
            ax.yaxis.set_tick_params(labelsize=7)

            # Cap optic flow at ±10
            if topic_key == '/optic_flow_mean_vx':
                ax.set_ylim(-10, 10)

            # Row label left column only
            if col == 0:
                ax.set_ylabel(row_label, fontsize=8)
            else:
                ax.tick_params(labelleft=False)

            # X label bottom row only
            if row == n_rows - 1:
                ax.set_xlabel('Time from trough (s)', fontsize=8)

    # Match y-limits across conditions for each row (except optic flow fixed)
    for row, (_, topic_key, _) in enumerate(ROW_DEFS):
        if topic_key == '/optic_flow_mean_vx':
            continue
        y_mins = [axes[row, c].get_ylim()[0] for c in range(n_cols)]
        y_maxs = [axes[row, c].get_ylim()[1] for c in range(n_cols)]
        for c in range(n_cols):
            axes[row, c].set_ylim(min(y_mins), max(y_maxs))

    plt.tight_layout()
    for ext in ['pdf', 'png']:
        path = os.path.join(output_dir, f'{fig_stem}.{ext}')
        plt.savefig(path, dpi=150, bbox_inches='tight')
    print(f'  [SAVED] {fig_stem}.pdf / .png')
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Phase-triggered averaging of flutter bags')
    parser.add_argument('--bag-dir',   type=str, default=None,
                        help='Directory containing bag files (default: auto-detect)')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Output directory for figures (default: same as bag-dir)')
    parser.add_argument('--window',    type=float, default=0.37,
                        help='Window size in seconds around each trough (default: 0.37 = 2 flutter periods)')
    parser.add_argument('--fs',        type=float, default=100.0,
                        help='Interpolation sample rate Hz (default: 100)')
    parser.add_argument('--min-peaks', type=int,   default=3,
                        help='Minimum troughs required per segment (default: 3)')
    parser.add_argument('--conditions', nargs='+', default=CONDITIONS,
                        choices=CONDITIONS,
                        help='Conditions to process (default: all)')
    args = parser.parse_args()

    # --- Find bag directory ---
    if args.bag_dir:
        bag_dir = args.bag_dir
    else:
        # Try common locations
        candidates = [
            os.path.expanduser('~/catkin_ws/src/flutter_control/data/results/bags'),
            os.path.expanduser('~/Desktop/Flutter_analysis/figures_for_dissertation/off_frequency_tests/OL_RDM'),
            '.',
        ]
        bag_dir = next((d for d in candidates if os.path.isdir(d)), '.')

    output_dir = args.output_dir or bag_dir
    os.makedirs(output_dir, exist_ok=True)

    print(f'Bag directory:    {bag_dir}')
    print(f'Output directory: {output_dir}')
    print(f'Window:           {args.window}s ({args.window / PULSE_PERIOD:.1f} flutter periods)')
    print(f'Interp rate:      {args.fs} Hz')
    print()

    # --- Process each condition ---
    all_results = {}

    for condition in args.conditions:
        pattern = os.path.join(bag_dir, f'{condition}_sample*.bag')
        bag_paths = sorted(glob.glob(pattern))

        if not bag_paths:
            print(f'[{condition}] No bags found matching {pattern}')
            continue

        print(f'[{condition}] Found {len(bag_paths)} bags')
        results = process_condition(
            bag_paths,
            window_s=args.window,
            fs=args.fs,
            min_peaks=args.min_peaks,
        )
        all_results[condition] = results

        # Print epoch counts
        for seg in ['uncontrolled', 'controlled']:
            n_epochs = len(results[seg].get('/optic_flow_mean_vx', []))
            print(f'  {seg}: {n_epochs} epochs')

        plot_condition(condition, results, output_dir, args.window)

    # --- Comparison figure ---
    if len(all_results) > 1:
        print('\nGenerating comparison figures...')
        for seg in ['uncontrolled', 'controlled']:
            plot_comparison(all_results, output_dir, args.window, segment=seg)
        print('\nGenerating combined figure...')
        plot_combined(all_results, output_dir, args.window)

    print('\nDone.')


if __name__ == '__main__':
    main()