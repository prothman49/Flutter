#!/usr/bin/env python3
"""
analyze_jitter_freq_sweep.py
=============================
Analyzes Uniform Jitter control at off-flutter frequencies.
Compares against existing fixed open-loop off-frequency data.

Produces:
  Figure 1 — Standalone: Jitter at each frequency, individual points + CI
  Figure 2 — Overlaid: Jitter vs fixed open-loop on same axes

Usage:
  python3 analyze_jitter_freq_sweep.py
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from datetime import datetime
from pathlib import Path
from scipy import stats

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
BASE_PATH       = Path.home() / 'Desktop' / 'Flutter_analysis' / 'figures_for_dissertation' / 'off_frequency_tests'
JITTER_DIR      = BASE_PATH / 'jitter_freq_sweep'
OFF_FREQ_DIR    = BASE_PATH  # for existing fixed open-loop data

# Existing fixed open-loop off-frequency data folders
FIXED_OL_DIRS = {
    3.0:  BASE_PATH / '3hz',
    8.0:  BASE_PATH / '8hz',
    10.0: BASE_PATH / '10hz',
}

# Colors
COLOR_JITTER   = '#9C27B0'   # purple — same as Uniform Jitter
COLOR_FIXED_OL = '#e07070'   # coral — same as off-frequency fixed open loop
COLOR_CL       = '#2e8b57'   # green — closed loop reference

# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------
def load_jitter_sweep(jitter_dir):
    """Load jitter frequency sweep JSONs — keyed by control_freq_hz."""
    data = {}
    for jf in sorted(Path(jitter_dir).glob('*.json')):
        try:
            with open(jf) as f:
                d = json.load(f)
            freq = d['meta']['control_freq_hz']
            pct  = d['reduction']['magnitude_reduction_pct']
            if freq not in data:
                data[freq] = []
            data[freq].append(pct)
            print(f"  {jf.name}: {freq} Hz → {pct:.1f}%")
        except Exception as e:
            print(f"  [ERROR] {jf.name}: {e}")
    return {k: np.array(v) for k, v in sorted(data.items())}


def load_fixed_ol(dirs):
    """Load existing fixed open-loop off-frequency data."""
    data = {}
    for freq, folder in dirs.items():
        if not folder.exists():
            print(f"  [SKIP] {folder} not found")
            continue
        reductions = []
        for jf in sorted(folder.glob('*.json')):
            try:
                with open(jf) as f:
                    d = json.load(f)
                # Handle both old and new JSON formats
                pct = None
                if 'pairs' in d:
                    for pair in d['pairs']:
                        if 'reduction_percent' in pair:
                            reductions.append(pair['reduction_percent'])
                    continue
                if 'control_off' in d and 'control_on' in d:
                    off_c = d['control_off'].get('cycles', [])
                    on_c  = d['control_on'].get('cycles', [])
                    for oc, nc in zip(off_c, on_c):
                        om = oc.get('avg_magnitude', 0)
                        nm = nc.get('avg_magnitude', 0)
                        if om > 0:
                            reductions.append((om - nm) / om * 100)
                    continue
                if 'effectiveness' in d:
                    pct = (d['effectiveness'].get('magnitude_reduction_percent') or
                           d['effectiveness'].get('reduction_percent'))
                elif 'reduction' in d:
                    pct = d['reduction'].get('magnitude_reduction_pct')
                if pct is not None:
                    reductions.append(pct)
            except Exception:
                continue
        if reductions:
            data[freq] = np.array(reductions)
            print(f"  Fixed OL {freq} Hz: n={len(reductions)}, mean={np.mean(reductions):.1f}%")
    return data


def load_closed_loop_ref(ol_rdm_dir):
    """Load closed loop reference from OL_RDM JSONs."""
    ol_rdm = BASE_PATH / 'OL_RDM'
    reductions = []
    for jf in sorted(ol_rdm.glob('CLOSED_LOOP_sample*.json')):
        try:
            with open(jf) as f:
                d = json.load(f)
            pct = d['reduction']['magnitude_reduction_pct']
            reductions.append(pct)
        except Exception:
            continue
    if reductions:
        print(f"  Closed Loop ref: n={len(reductions)}, mean={np.mean(reductions):.1f}%")
    return np.array(reductions) if reductions else None


# ---------------------------------------------------------------------------
# Plot helper
# ---------------------------------------------------------------------------
def draw_ci_box(ax, x, values, color, box_width=0.35):
    mean  = np.mean(values)
    n     = len(values)
    ci    = stats.sem(values) * stats.t.ppf(0.975, df=max(n - 1, 1))
    half  = box_width / 2.0
    rect  = mpatches.FancyBboxPatch(
        (x - half, mean - ci), box_width, 2 * ci,
        boxstyle="square,pad=0",
        linewidth=1.8, linestyle="--",
        edgecolor=color, facecolor="none", zorder=4
    )
    ax.add_patch(rect)
    ax.plot([x - half, x + half], [mean, mean],
            color=color, linewidth=2.5, zorder=5)
    return mean, mean - ci, mean + ci


# ---------------------------------------------------------------------------
# Figure 1 — Standalone jitter sweep
# ---------------------------------------------------------------------------
def plot_standalone(jitter_data, cl_ref, output_dir):
    freqs     = sorted(jitter_data.keys())
    all_vals  = np.concatenate(list(jitter_data.values()))
    if cl_ref is not None:
        all_vals = np.concatenate([all_vals, cl_ref])
    y_min = min(all_vals) - 8
    y_max = max(all_vals) + 8

    fig, ax = plt.subplots(figsize=(8, 6))
    np.random.seed(42)

    for i, freq in enumerate(freqs):
        vals   = jitter_data[freq]
        jitter = np.random.normal(0, 0.06, size=len(vals))
        ax.scatter(i + jitter, vals, color=COLOR_JITTER, alpha=0.85,
                   s=55, zorder=3, edgecolors=COLOR_JITTER, linewidth=0.5)
        mean, ci_lo, ci_hi = draw_ci_box(ax, i, vals, COLOR_JITTER)
        ax.annotate(f'{mean:.1f}%',
                    xy=(i + 0.2, mean), xytext=(i + 0.32, mean),
                    fontsize=10, fontweight='bold', color=COLOR_JITTER,
                    va='center',
                    arrowprops=dict(arrowstyle='->', color=COLOR_JITTER, lw=1.1))
        ax.text(i, y_min + 1, f'n={len(vals)}',
                ha='center', va='top', fontsize=9, color='#555')

    ax.axhline(0, color='black', lw=0.8, alpha=0.3)

    # Closed loop reference line
    if cl_ref is not None:
        cl_mean = np.mean(cl_ref)
        ax.axhline(cl_mean, color=COLOR_CL, lw=1.5, ls='--', alpha=0.7,
                   label=f'Closed Loop ref ({cl_mean:.1f}%)')
        ax.legend(fontsize=9, loc='upper right')

    ax.set_xticks(range(len(freqs)))
    ax.set_xticklabels([f'{f:.0f} Hz' for f in freqs], fontsize=11)
    ax.set_ylabel('Flutter Magnitude Reduction (%)', fontsize=12, fontweight='bold')
    ax.set_xlabel('Jitter Control Frequency (Hz)', fontsize=12, fontweight='bold')
    ax.set_title('Uniform Jitter at Off-Flutter Frequencies\nIndividual samples with mean and 95% CI',
                 fontsize=13, fontweight='bold')
    ax.grid(True, axis='y', alpha=0.25, linestyle=':')
    ax.set_xlim(-0.6, len(freqs) - 0.4)
    ax.set_ylim(y_min, y_max)
    ax.spines[['top', 'right']].set_visible(False)

    plt.tight_layout()
    for ext in ['png', 'pdf']:
        plt.savefig(output_dir / f'jitter_freq_sweep_standalone.{ext}',
                    dpi=300, bbox_inches='tight')
    print(f"[SAVED] jitter_freq_sweep_standalone.png / .pdf")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 2 — Overlaid: jitter vs fixed open-loop
# ---------------------------------------------------------------------------
def plot_overlaid(jitter_data, fixed_ol_data, cl_ref, output_dir):
    all_freqs = sorted(set(list(jitter_data.keys()) + list(fixed_ol_data.keys())))
    all_vals  = []
    for d in [jitter_data, fixed_ol_data]:
        for v in d.values():
            all_vals.extend(v)
    if cl_ref is not None:
        all_vals.extend(cl_ref)
    y_min = min(all_vals) - 8
    y_max = max(all_vals) + 10

    fig, ax = plt.subplots(figsize=(10, 6))
    np.random.seed(42)

    x_pos = {f: i for i, f in enumerate(all_freqs)}

    # Plot fixed open-loop (offset left)
    for freq in all_freqs:
        if freq not in fixed_ol_data:
            continue
        vals   = fixed_ol_data[freq]
        x      = x_pos[freq] - 0.18
        jit    = np.random.normal(0, 0.05, size=len(vals))
        ax.scatter(x + jit, vals, color=COLOR_FIXED_OL, alpha=0.75,
                   s=45, zorder=3, edgecolors=COLOR_FIXED_OL, linewidth=0.5)
        mean, _, _ = draw_ci_box(ax, x, vals, COLOR_FIXED_OL, box_width=0.28)
        ax.text(x, mean + 1.5, f'{mean:.1f}%',
                ha='center', fontsize=8, color=COLOR_FIXED_OL, fontweight='bold')

    # Plot jitter (offset right)
    for freq in all_freqs:
        if freq not in jitter_data:
            continue
        vals   = jitter_data[freq]
        x      = x_pos[freq] + 0.18
        jit    = np.random.normal(0, 0.05, size=len(vals))
        ax.scatter(x + jit, vals, color=COLOR_JITTER, alpha=0.85,
                   s=55, zorder=3, edgecolors=COLOR_JITTER, linewidth=0.5)
        mean, _, _ = draw_ci_box(ax, x, vals, COLOR_JITTER, box_width=0.28)
        ax.text(x, mean + 1.5, f'{mean:.1f}%',
                ha='center', fontsize=8, color=COLOR_JITTER, fontweight='bold')

        # t-test vs fixed OL at same frequency
        if freq in fixed_ol_data:
            _, p = stats.ttest_ind(fixed_ol_data[freq], vals)
            sig  = ('***' if p < 0.001 else '**' if p < 0.01 else
                    '*'   if p < 0.05  else 'ns')
            y_br = max(np.max(vals), np.max(fixed_ol_data[freq])) + 3
            fx   = x_pos[freq]
            ax.plot([fx - 0.18, fx - 0.18, fx + 0.18, fx + 0.18],
                    [y_br - 0.5, y_br, y_br, y_br - 0.5], color='#555', lw=1.0)
            ax.text(fx, y_br + 0.3, sig, ha='center', va='bottom',
                    fontsize=10, color='#444')

    # Closed loop reference line
    if cl_ref is not None:
        cl_mean = np.mean(cl_ref)
        ax.axhline(cl_mean, color=COLOR_CL, lw=1.5, ls='--', alpha=0.7,
                   label=f'Closed Loop ref ({cl_mean:.1f}%)')

    ax.axhline(0, color='black', lw=0.8, alpha=0.3)

    # Legend
    legend_elements = [
        mpatches.Patch(color=COLOR_FIXED_OL, label='Fixed Open Loop'),
        mpatches.Patch(color=COLOR_JITTER,   label='Uniform Jitter'),
    ]
    if cl_ref is not None:
        legend_elements.append(
            plt.Line2D([0], [0], color=COLOR_CL, lw=1.5, ls='--',
                       label=f'Closed Loop ref ({np.mean(cl_ref):.1f}%)'))
    ax.legend(handles=legend_elements, fontsize=9, loc='upper right')

    ax.set_xticks(range(len(all_freqs)))
    ax.set_xticklabels([f'{f:.0f} Hz' for f in all_freqs], fontsize=11)
    ax.set_ylabel('Flutter Magnitude Reduction (%)', fontsize=12, fontweight='bold')
    ax.set_xlabel('Control Frequency (Hz)', fontsize=12, fontweight='bold')
    ax.set_title('Fixed Open Loop vs Uniform Jitter at Off-Flutter Frequencies\n'
                 'Individual samples with mean and 95% CI',
                 fontsize=13, fontweight='bold')
    ax.grid(True, axis='y', alpha=0.25, linestyle=':')
    ax.set_xlim(-0.6, len(all_freqs) - 0.4)
    ax.set_ylim(y_min, y_max)
    ax.spines[['top', 'right']].set_visible(False)

    plt.tight_layout()
    for ext in ['png', 'pdf']:
        plt.savefig(output_dir / f'jitter_freq_sweep_overlaid.{ext}',
                    dpi=300, bbox_inches='tight')
    print(f"[SAVED] jitter_freq_sweep_overlaid.png / .pdf")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    dated_dir = BASE_PATH / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_jitter_freq_sweep')
    dated_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output: {dated_dir}\n")

    print("Loading Uniform Jitter sweep data:")
    jitter_data = load_jitter_sweep(JITTER_DIR)

    print("\nLoading fixed open-loop off-frequency data:")
    fixed_ol_data = load_fixed_ol(FIXED_OL_DIRS)

    print("\nLoading closed loop reference:")
    cl_ref = load_closed_loop_ref(BASE_PATH / 'OL_RDM')

    if not jitter_data:
        print("[ERROR] No jitter sweep data found.")
        return

    print("\n--- Summary ---")
    for freq, vals in jitter_data.items():
        ci = stats.sem(vals) * stats.t.ppf(0.975, df=max(len(vals)-1, 1))
        print(f"  Jitter {freq:.0f} Hz: mean={np.mean(vals):.1f}%  95CI±{ci:.1f}%  n={len(vals)}")

    print("\nGenerating figures...")
    plot_standalone(jitter_data, cl_ref, dated_dir)
    plot_overlaid(jitter_data, fixed_ol_data, cl_ref, dated_dir)
    print("\nDone.")


if __name__ == '__main__':
    main()