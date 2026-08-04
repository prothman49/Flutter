#!/usr/bin/env python3
"""
plot_fft_with_reduction.py
==========================
Overlay figure: mean uncontrolled FFT spectrum as background,
with mean flutter reduction % per condition plotted at the
actual trial frequency on a continuous x-axis.

Usage:
    python3 plot_fft_with_reduction.py
    python3 plot_fft_with_reduction.py --bag-dir /path/to/bags
"""

import argparse
import glob
import json
import os
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import stats
from rosbags.rosbag1 import Reader
from rosbags.typesys import get_typestore, Stores

TYPESTORE = get_typestore(Stores.ROS1_NOETIC)

matplotlib.rcParams.update({
    'font.family':    'sans-serif',
    'font.size':      10,
    'axes.labelsize': 11,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'axes.linewidth': 0.8,
    'pdf.fonttype':   42,
    'ps.fonttype':    42,
})

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
BASE_PATH  = Path.home() / 'Desktop' / 'Flutter_analysis' / 'figures_for_dissertation' / 'off_frequency_tests'
OL_RDM_DIR = BASE_PATH / 'OL_RDM'
OUTPUT_DIR = BASE_PATH

T_UNCONTROLLED = 60.0
MIN_DURATION   = 100.0
F_MAX          = 12.0   # Hz to show on x-axis
FLUTTER_FREQ   = 5.4
FLUTTER_BAND   = (4.0, 9.0)

# ---------------------------------------------------------------------------
# Conditions: label, trial frequency, color, json glob, x jitter
# ---------------------------------------------------------------------------
CONDITIONS = [
    dict(key='dc',       label='0 Hz\n(DC)',           freq=0.0,  color='#888888', x_jitter=0.0),
    dict(key='3hz',      label='3 Hz',                 freq=3.0,  color='#e07070', x_jitter=0.0),
    dict(key='ol55',     label='5.5 Hz\n(open loop)',  freq=5.5,  color='#e07070', x_jitter=0.0),
    dict(key='cl',       label='5.5 Hz\n(closed loop)',freq=5.5,  color='#2e8b57', x_jitter=0.0),
    dict(key='8hz',      label='8 Hz',                 freq=8.0,  color='#e07070', x_jitter=0.0),
    dict(key='10hz',     label='10 Hz',                freq=10.0, color='#e07070', x_jitter=0.0),
]

# ---------------------------------------------------------------------------
# Load reduction % from JSON files
# ---------------------------------------------------------------------------


def load_json_files(base_path, folder_name, frequency):
    folder = base_path / folder_name
    if not folder.exists():
        return []
    results = []
    patterns = [
        f'off_frequency_test_{frequency}*.json',
        f'*{frequency}hz*.json',
        'dc_control_summary*.json' if frequency == 0 else None,
    ]
    for pattern in patterns:
        if pattern is None:
            continue
        for json_file in sorted(folder.glob(pattern)):
            if any(x in json_file.name for x in ['run1__', 'run2__', 'run3__']):
                continue
            try:
                with open(json_file) as f:
                    data = json.load(f)
                if 'effectiveness' in data or 'statistics' in data:
                    results.append(data)
            except Exception:
                continue
    return results


def extract_magnitude_reduction(results):
    reductions = []
    for result in results:
        if 'pairs' in result:
            for pair in result['pairs']:
                if 'reduction_percent' in pair:
                    reductions.append(pair['reduction_percent'])
            continue
        if 'control_off' in result and 'control_on' in result:
            off_cycles = result['control_off'].get('cycles', [])
            on_cycles  = result['control_on'].get('cycles', [])
            if len(off_cycles) > 0 and len(on_cycles) > 0 and len(off_cycles) == len(on_cycles):
                for off_cyc, on_cyc in zip(off_cycles, on_cycles):
                    off_mag = off_cyc.get('avg_magnitude', 0)
                    on_mag  = on_cyc.get('avg_magnitude', 0)
                    if off_mag > 0:
                        reductions.append(((off_mag - on_mag) / off_mag) * 100)
                continue
        if 'effectiveness' in result:
            if 'magnitude_reduction_percent' in result['effectiveness']:
                reductions.append(result['effectiveness']['magnitude_reduction_percent'])
            elif 'reduction_percent' in result['effectiveness']:
                reductions.append(result['effectiveness']['reduction_percent'])
        elif 'statistics' in result:
            if 'mean_reduction_percent' in result['statistics']:
                reductions.append(result['statistics']['mean_reduction_percent'])
        elif 'magnitude_reduction_percent' in result:
            reductions.append(result['magnitude_reduction_percent'])
        elif 'reduction_percent' in result:
            reductions.append(result['reduction_percent'])
    return np.array(reductions)


def load_all_reductions(ol_rdm_dir, base_path):
    data = {}

    # Off-frequency conditions using same loader as analyze_off_frequency.py
    for folder, freq, key in [
        ('0hz',  0.0,  'dc'),
        ('3hz',  3.0,  '3hz'),
        ('8hz',  8.0,  '8hz'),
        ('10hz', 10.0, '10hz'),
    ]:
        results = load_json_files(base_path, folder, freq)
        vals    = extract_magnitude_reduction(results)
        if len(vals):
            data[key] = vals
            print(f"  {key}: n={len(vals)}, mean={np.mean(vals):.1f}%")
        else:
            print(f"  {key}: no data found")

    # Open loop 5.4 Hz
    vals = []
    for jf in sorted(ol_rdm_dir.glob('OPEN_LOOP_55_sample*.json')):
        try:
            with open(jf) as f:
                d = json.load(f)
            pct = d.get('reduction', {}).get('magnitude_reduction_pct', None)
            if pct is not None:
                vals.append(pct)
        except Exception:
            continue
    if vals:
        data['ol55'] = np.array(vals)
        print(f"  ol55: n={len(vals)}, mean={np.mean(vals):.1f}%")

    # Closed loop — new per-sample JSONs
    vals = []
    for jf in sorted(ol_rdm_dir.glob('CLOSED_LOOP_sample*.json')):
        try:
            with open(jf) as f:
                d = json.load(f)
            pct = d.get('reduction', {}).get('magnitude_reduction_pct', None)
            if pct is not None:
                vals.append(pct)
        except Exception:
            continue
    if vals:
        data['cl'] = np.array(vals)
        print(f"  cl: n={len(vals)}, mean={np.mean(vals):.1f}%")

    return data


# ---------------------------------------------------------------------------
# Compute mean uncontrolled FFT from all bags
# ---------------------------------------------------------------------------
def read_optic_flow_segment(bag_path, t_start, t_end):
    times, vals = [], []
    with Reader(bag_path) as reader:
        t0_ns = reader.start_time
        conns = [c for c in reader.connections if c.topic == '/optic_flow_mean_vx']
        if not conns:
            return None, None
        for conn, ts, raw in reader.messages(connections=conns):
            t_sec = (ts - t0_ns) / 1e9
            if t_sec < t_start or t_sec >= t_end:
                continue
            try:
                msg = TYPESTORE.deserialize_ros1(raw, conn.msgtype)
                times.append(t_sec - t_start)
                vals.append(float(msg.data))
            except Exception:
                continue
    if not times:
        return None, None
    return np.array(times), np.array(vals)


def compute_fft_from_segment(times, signal):
    if len(times) < 100:
        return None, None
    dt = np.median(np.diff(times))
    t_uniform = np.arange(times[0], times[-1], dt)
    sig = np.interp(t_uniform, times, signal)
    sig -= np.mean(sig)
    N     = len(sig)
    freqs = np.fft.rfftfreq(N, d=dt)
    mags  = np.abs(np.fft.rfft(sig)) / N
    mask  = freqs <= F_MAX
    return freqs[mask], mags[mask]


def compute_mean_uncontrolled_fft(bag_dir):
    """Average FFT across all available bag uncontrolled segments."""
    all_bags = sorted(glob.glob(os.path.join(bag_dir, '*.bag')))
    all_mags = []
    common_f = None

    print(f"Computing mean uncontrolled FFT from {len(all_bags)} bags...")
    for bag_path in all_bags:
        try:
            with Reader(bag_path) as r:
                dur = (r.end_time - r.start_time) / 1e9
            if dur < MIN_DURATION:
                continue
        except Exception:
            continue

        times, signal = read_optic_flow_segment(bag_path, 0.0, T_UNCONTROLLED)
        if times is None or len(times) < 100:
            continue

        freqs, mags = compute_fft_from_segment(times, signal)
        if freqs is None:
            continue

        if common_f is None:
            common_f = freqs

        mags_interp = np.interp(common_f, freqs, mags)
        all_mags.append(mags_interp)

    if not all_mags:
        return None, None

    n_bags = len(all_mags)
    print(f"  Used {n_bags} bags")
    mean_mags = np.mean(all_mags, axis=0)
    return common_f, mean_mags, n_bags


# ---------------------------------------------------------------------------
# Main figure
# ---------------------------------------------------------------------------
def plot_figure(reduction_data, fft_freqs, fft_mean, n_bags, output_dir):
    fig, ax1 = plt.subplots(figsize=(11, 6))

    # ── Background FFT — plotted on its own secondary axis, mean line only ──
    ax2 = ax1.twinx()
    ax2.plot(fft_freqs, fft_mean, color='#999999', lw=1.0, zorder=1)
    ax2.set_ylabel(f'Uncontrolled FFT Magnitude (a.u., n={n_bags} bags)',
                   fontsize=10, color='#777777')
    ax2.tick_params(axis='y', labelcolor='#777777', labelsize=8)

    # Align the FFT's noise floor with ax1's y=0 gridline (purely visual —
    # ax2 keeps its own real units/ticks, we're just choosing where its
    # origin sits so the curve's baseline reads at zero like before).
    ax1_ymin, ax1_ymax = -20.0, float(max(
        np.concatenate([reduction_data[c['key']] for c in CONDITIONS
                        if c['key'] in reduction_data])) + 12)
    frac_zero = (0 - ax1_ymin) / (ax1_ymax - ax1_ymin)

    floor_val = float(np.min(fft_mean))
    peak_val  = float(np.max(fft_mean))
    ax2_ymax  = peak_val * 1.1
    ax2_ymin  = (floor_val - frac_zero * ax2_ymax) / (1 - frac_zero)
    ax2.set_ylim(ax2_ymin, ax2_ymax)

    ax2.spines[['top', 'right']].set_color('#bbbbbb')
    ax2.spines['right'].set_visible(True)
    ax2.spines['top'].set_visible(False)

    # Keep ax1 (the % axis, with markers/error bars) drawn on top of ax2's line
    ax1.set_zorder(ax2.get_zorder() + 1)
    ax1.patch.set_visible(False)

    # Find actual FFT peak and mark it
    peak_idx  = np.argmax(fft_mean)
    peak_freq = fft_freqs[peak_idx]
    print(f"  Actual FFT peak at: {peak_freq:.3f} Hz")

    ax1.axvline(peak_freq, color='#e65100', lw=0.8, ls=':', alpha=0.6, zorder=1)
    ax1.text(peak_freq + 0.1, 2, f'{peak_freq:.2f} Hz\n(FFT peak)',
             fontsize=7.5, color='#e65100', va='bottom')

    # ── Zero line ─────────────────────────────────────────────────────────────
    ax1.axhline(0, color='black', lw=0.8, alpha=0.3, zorder=2)

    # ── Plot each condition ───────────────────────────────────────────────────
    plotted = []
    for cond in CONDITIONS:
        key = cond['key']
        if key not in reduction_data:
            print(f"  [SKIP] {key} — no data")
            continue

        vals  = reduction_data[key]
        mean  = float(np.mean(vals))
        sem   = float(stats.sem(vals))
        # Closed loop plots at actual measured flutter frequency
        if key == 'cl':
            freq = peak_freq
        else:
            freq = cond['freq'] + cond['x_jitter']
        color = cond['color']
        label = cond['label']

        print(f"  {key}: mean={mean:.2f}, sem={sem:.2f}, n={len(vals)}, freq={freq}")

        marker = 'D' if key == 'cl' else 'o'
        ms     = 6  if key == 'cl' else 10

        # Plot marker first, then error bar on top
        ax1.plot(freq, mean, marker=marker, color=color,
                 markersize=ms, zorder=5,
                 markeredgecolor='white', markeredgewidth=0.8,
                 label=label)
        ax1.errorbar(freq, mean, yerr=sem,
                     fmt='none', color=color,
                     capsize=6, capthick=2.5, elinewidth=2.5,
                     zorder=6)

        # Label — always above the top of the error bar
        label_y = mean + sem + 1.5
        ax1.text(freq, label_y, f'{mean:.1f}%',
                 ha='center', va='bottom', fontsize=8.5,
                 color=color, fontweight='bold')

        plotted.append(key)

    # ── Axes ──────────────────────────────────────────────────────────────────
    ax1.set_xlabel('Control Frequency (Hz)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Flutter Magnitude Reduction (%)', fontsize=12, fontweight='bold')
    ax1.set_title('Control Effectiveness vs Trial Frequency\n'
                  'Background = mean uncontrolled FFT spectrum',
                  fontsize=12, fontweight='bold')
    ax1.set_xlim(-0.8, F_MAX)
    ax1.set_xticks([0, 2, 4, 6, 8, 10, 12])
    ax1.grid(True, axis='y', alpha=0.2, linestyle=':')
    ax1.spines[['top', 'right']].set_visible(False)

    # Dynamic y limits with headroom for labels
    all_vals = np.concatenate([reduction_data[c['key']] for c in CONDITIONS
                               if c['key'] in reduction_data])
    ax1.set_ylim(-22, max(all_vals) + 12)

    plt.tight_layout()
    for ext in ['png', 'pdf']:
        p = output_dir / f'fft_reduction_overlay.{ext}'
        plt.savefig(p, dpi=300, bbox_inches='tight')
        print(f'[SAVED] fft_reduction_overlay.{ext}')
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag-dir', type=str, default=None,
                        help='Directory containing .bag files for FFT computation')
    args = parser.parse_args()

    bag_dir = args.bag_dir or str(OL_RDM_DIR)

    print('Loading reduction data...')
    reduction_data = load_all_reductions(OL_RDM_DIR, BASE_PATH)

    print('\nComputing mean uncontrolled FFT...')
    result = compute_mean_uncontrolled_fft(bag_dir)
    if result[0] is None:
        print('[ERROR] Could not compute FFT — no valid bags found')
        return
    fft_freqs, fft_mean, n_bags = result

    print('\nGenerating figure...')
    from datetime import datetime
    dated_dir = OUTPUT_DIR / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_fft_reduction')
    dated_dir.mkdir(parents=True, exist_ok=True)
    plot_figure(reduction_data, fft_freqs, fft_mean, n_bags, dated_dir)
    print('Done.')


if __name__ == '__main__':
    main()