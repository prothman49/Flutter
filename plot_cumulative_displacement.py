#!/usr/bin/env python3
"""
plot_cumulative_displacement.py
================================
Cumulative ABSOLUTE displacement (i.e. cumulative path length of wing
motion, integral of |velocity| dt) over time, for uncontrolled vs
controlled segments, per condition.

Why absolute value and not raw integral of velocity: flutter is an
oscillation — a raw integral of velocity (net displacement) swings back
and forth and returns close to zero over a full cycle, regardless of
flutter severity. That would NOT distinguish good vs poor control
performance. Integrating |velocity| instead gives a monotonically
increasing "total distance traveled" curve: a wing flapping violently
racks up path length fast, a well-controlled wing racks it up slowly.
This is the quantity that should separate good performers (small slope,
low cumulative total) from poor performers (steep slope, high total).

Each bag contains:
  - Uncontrolled segment: [0, T_UNCONTROLLED)
  - Controlled segment:   [T_UNCONTROLLED, T_UNCONTROLLED + T_CONTROLLED)
Both segments are re-zeroed to their own t=0 so they can be plotted and
compared on the same relative time axis.

Per condition, per bag:
  1. Interpolate /optic_flow_mean_vx onto a uniform time grid.
  2. Split into uncontrolled / controlled segments.
  3. Linearly detrend each segment (removes slow sensor drift/offset —
     critical before integrating, since any residual DC bias in velocity
     would otherwise integrate into a fake runaway ramp).
  4. Cumulative-trapezoidal-integrate |v| over each segment -> dx(t).
Across bags within a condition, all dx(t) curves are interpolated onto a
shared time grid and averaged (mean +/- SEM band).

Usage:
    python3 plot_cumulative_displacement.py
    python3 plot_cumulative_displacement.py --bag-dir /path/to/bags
"""

import argparse
import glob
import os
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import detrend
from scipy.integrate import cumulative_trapezoid
from scipy import stats
from rosbags.rosbag1 import Reader
from rosbags.typesys import get_typestore, Stores

TYPESTORE = get_typestore(Stores.ROS1_NOETIC)

matplotlib.rcParams.update({
    'font.family':    'sans-serif',
    'font.size':      9,
    'axes.labelsize': 10,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'axes.linewidth': 0.8,
    'pdf.fonttype':   42,
    'ps.fonttype':    42,
})

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
T_UNCONTROLLED = 60.0
T_CONTROLLED   = 60.0
MIN_DURATION   = 100.0
FS_INTERP      = 100.0
N_TIME_POINTS  = 400   # resolution of the shared time grid used for averaging

CONDITIONS = ['CLOSED_LOOP', 'OPEN_LOOP_55', 'POISSON', 'UNIFORM_JITTER', 'PHASE_RANDOMIZED', 'REPLAY']

CONDITION_LABELS = {
    'CLOSED_LOOP':       'Closed Loop',
    'OPEN_LOOP_55':      'Open Loop 5.5 Hz',
    'POISSON':           'Poisson',
    'UNIFORM_JITTER':    'Uniform Jitter',
    'PHASE_RANDOMIZED':  'Polarity Randomized',
    'REPLAY':            'Replay',
}

CONDITION_COLORS = {
    'CLOSED_LOOP':       '#4CAF50',
    'OPEN_LOOP_55':      '#03A9F4',
    'POISSON':           '#FF9800',
    'UNIFORM_JITTER':    '#9C27B0',
    'PHASE_RANDOMIZED':  '#F44336',
    'REPLAY':            '#795548',
}

COLOR_UNCONTROLLED = '#888888'

# ---------------------------------------------------------------------------
# Bag reading
# ---------------------------------------------------------------------------
def read_optic_flow(bag_path):
    times, vals = [], []
    with Reader(bag_path) as reader:
        t0_ns = reader.start_time
        conns = [c for c in reader.connections if c.topic == '/optic_flow_mean_vx']
        if not conns:
            return None, None
        for conn, ts, raw in reader.messages(connections=conns):
            try:
                msg = TYPESTORE.deserialize_ros1(raw, conn.msgtype)
                times.append((ts - t0_ns) / 1e9)
                vals.append(float(msg.data))
            except Exception:
                continue
    if not times:
        return None, None
    return np.array(times), np.array(vals)


def interp_uniform(t, v, t_start, t_end, fs=FS_INTERP):
    mask = (t >= t_start) & (t < t_end)
    t_seg, v_seg = t[mask], v[mask]
    if len(t_seg) < 10:
        return None, None
    t_uniform = np.arange(0, t_end - t_start, 1.0 / fs)
    v_uniform = np.interp(t_uniform + t_start, t_seg, v_seg)
    return t_uniform, v_uniform


def cumulative_abs_displacement(t_uniform, v_uniform):
    """Detrend (remove slow drift/offset), then cumulative-integrate |v|."""
    if v_uniform is None or len(v_uniform) < 10:
        return None
    v_dt = detrend(v_uniform, type='linear')
    dx = cumulative_trapezoid(np.abs(v_dt), t_uniform, initial=0.0)
    return dx


def collect_condition_curves(condition, bag_dir):
    """
    Returns (t_grid, unc_curves, ctrl_curves) where unc_curves/ctrl_curves
    are lists of dx(t) arrays (one per bag), all interpolated onto a
    shared time grid so they can be averaged.
    """
    pattern = os.path.join(bag_dir, f'{condition}_sample*.bag')
    bag_paths = sorted(glob.glob(pattern))

    t_grid = np.linspace(0, T_UNCONTROLLED, N_TIME_POINTS)
    unc_curves, ctrl_curves = [], []

    for bag_path in bag_paths:
        try:
            with Reader(bag_path) as r:
                duration = (r.end_time - r.start_time) / 1e9
        except Exception as e:
            print(f"    [WARN] Could not open {Path(bag_path).name}: {e}")
            continue
        if duration < MIN_DURATION:
            continue

        t_raw, v_raw = read_optic_flow(bag_path)
        if t_raw is None:
            continue

        t_unc, v_unc = interp_uniform(t_raw, v_raw, 0.0, T_UNCONTROLLED)
        t_ctrl, v_ctrl = interp_uniform(t_raw, v_raw, T_UNCONTROLLED,
                                        T_UNCONTROLLED + T_CONTROLLED)

        dx_unc = cumulative_abs_displacement(t_unc, v_unc)
        dx_ctrl = cumulative_abs_displacement(t_ctrl, v_ctrl)

        if dx_unc is not None:
            unc_curves.append(np.interp(t_grid, t_unc, dx_unc))
        if dx_ctrl is not None:
            ctrl_curves.append(np.interp(t_grid, t_ctrl, dx_ctrl))

    return t_grid, unc_curves, ctrl_curves


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
def plot_cumulative_displacement(all_data, output_dir):
    cond_keys = [c for c in all_data if all_data[c][1] or all_data[c][2]]
    n_cols = min(3, len(cond_keys))
    n_rows = int(np.ceil(len(cond_keys) / n_cols))

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(4.2 * n_cols, 3.6 * n_rows),
                             squeeze=False)

    # Shared y-limit across all panels so cumulative totals are directly
    # comparable between conditions, not just within one panel.
    global_max = 0.0
    for cond in cond_keys:
        _, unc_curves, ctrl_curves = all_data[cond]
        for curves in (unc_curves, ctrl_curves):
            if curves:
                global_max = max(global_max, float(np.max([c[-1] for c in curves])))
    y_max = global_max * 1.1

    print("\n--- Final cumulative |displacement| (mean at end of segment) ---")

    for i, cond in enumerate(cond_keys):
        ax = axes[i // n_cols][i % n_cols]
        t_grid, unc_curves, ctrl_curves = all_data[cond]
        color = CONDITION_COLORS[cond]

        final_unc = final_ctrl = None

        if unc_curves:
            unc_arr = np.vstack(unc_curves)
            mean_unc = unc_arr.mean(axis=0)
            sem_unc  = stats.sem(unc_arr, axis=0)
            ax.plot(t_grid, mean_unc, color=COLOR_UNCONTROLLED, lw=1.8,
                    ls='--', label='Uncontrolled', zorder=3)
            ax.fill_between(t_grid, mean_unc - sem_unc, mean_unc + sem_unc,
                            color=COLOR_UNCONTROLLED, alpha=0.2, zorder=1)
            final_unc = mean_unc[-1]

        if ctrl_curves:
            ctrl_arr = np.vstack(ctrl_curves)
            mean_ctrl = ctrl_arr.mean(axis=0)
            sem_ctrl  = stats.sem(ctrl_arr, axis=0)
            ax.plot(t_grid, mean_ctrl, color=color, lw=2.0,
                    ls='-', label='Controlled', zorder=4)
            ax.fill_between(t_grid, mean_ctrl - sem_ctrl, mean_ctrl + sem_ctrl,
                            color=color, alpha=0.25, zorder=2)
            final_ctrl = mean_ctrl[-1]

        title = f'{CONDITION_LABELS[cond]}\n'
        if final_unc is not None and final_ctrl is not None:
            # Positive = displacement INCREASED under control (worse).
            # Negative = displacement DECREASED under control (better).
            pct_change = (final_ctrl - final_unc) / final_unc * 100
            title += f'n_unc={len(unc_curves)}, n_ctrl={len(ctrl_curves)}  |  {pct_change:+.1f}% displacement change'
            print(f"  {CONDITION_LABELS[cond]:<22} uncontrolled={final_unc:.2f}  "
                  f"controlled={final_ctrl:.2f}  ({pct_change:+.1f}%)")
        else:
            title += f'n_unc={len(unc_curves)}, n_ctrl={len(ctrl_curves)}'

        ax.set_title(title, fontsize=9.5, fontweight='bold', color=color)
        ax.set_ylim(0, y_max)
        ax.set_xlim(0, T_UNCONTROLLED)
        ax.grid(True, alpha=0.2, linestyle=':')
        ax.spines[['top', 'right']].set_visible(False)
        ax.legend(fontsize=7.5, loc='upper left')

        if i % n_cols == 0:
            ax.set_ylabel('Cumulative |Displacement| (a.u.)', fontsize=9, fontweight='bold')
        if i // n_cols == n_rows - 1:
            ax.set_xlabel('Time (s)', fontsize=9, fontweight='bold')

    # Hide any unused subplot slots
    for j in range(len(cond_keys), n_rows * n_cols):
        axes[j // n_cols][j % n_cols].set_visible(False)

    fig.suptitle('Cumulative Absolute Displacement Over Time\n'
                 'Uncontrolled vs Controlled, by Condition',
                 fontsize=13, fontweight='bold', y=1.02)

    plt.tight_layout()
    for ext in ['png', 'pdf']:
        p = os.path.join(output_dir, f'cumulative_displacement.{ext}')
        plt.savefig(p, dpi=300, bbox_inches='tight')
    print(f'\n[SAVED] cumulative_displacement.png / .pdf')
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Cumulative absolute displacement, uncontrolled vs controlled')
    parser.add_argument('--bag-dir', type=str, default=None)
    parser.add_argument('--output-dir', type=str, default=None)
    parser.add_argument('--conditions', nargs='+', default=CONDITIONS, choices=CONDITIONS)
    args = parser.parse_args()

    if args.bag_dir:
        bag_dir = args.bag_dir
    else:
        candidates = [
            os.path.expanduser('~/Desktop/Flutter_analysis/figures_for_dissertation/off_frequency_tests/OL_RDM'),
            '.',
        ]
        bag_dir = next((d for d in candidates if os.path.isdir(d)), '.')

    output_dir = args.output_dir or bag_dir
    dated_dir = os.path.join(output_dir, datetime.now().strftime('%Y%m%d_%H%M%S') + '_cumulative_displacement')
    os.makedirs(dated_dir, exist_ok=True)
    print(f'Bag directory:    {bag_dir}')
    print(f'Output directory: {dated_dir}\n')

    all_data = {}
    for condition in args.conditions:
        print(f'[{condition}] Collecting cumulative displacement curves...')
        t_grid, unc_curves, ctrl_curves = collect_condition_curves(condition, bag_dir)
        print(f'  n_uncontrolled={len(unc_curves)}, n_controlled={len(ctrl_curves)}')
        all_data[condition] = (t_grid, unc_curves, ctrl_curves)

    if not any(unc or ctrl for _, unc, ctrl in all_data.values()):
        print('[ERROR] No usable data loaded for any condition.')
        return

    print('\nGenerating figure...')
    plot_cumulative_displacement(all_data, dated_dir)
    print('\nDone.')


if __name__ == '__main__':
    main()