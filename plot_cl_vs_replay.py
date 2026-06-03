#!/usr/bin/env python3
"""
plot_CL_vs_replay.py
=====================
Compares CLOSED_LOOP vs REPLAY conditions.
Loads per-sample JSON files from the OL_RDM folder.

Usage:
  python3 plot_CL_vs_replay.py
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path
from scipy import stats

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
BASE_PATH  = Path.home() / 'Desktop' / 'Flutter_analysis' / 'figures_for_dissertation' / 'off_frequency_tests'
OL_RDM_DIR = BASE_PATH / 'OL_RDM'
OUTPUT_DIR = BASE_PATH / 'OL_RDM'

# ---------------------------------------------------------------------------
# Colors / labels
# ---------------------------------------------------------------------------
CONDITION_COLORS = {
    'CLOSED_LOOP': '#2e8b57',
    'REPLAY':      '#795548',
}

CONDITION_LABELS = {
    'CLOSED_LOOP': 'Closed Loop\n(Live)',
    'REPLAY':      'Closed Loop\n(Replay)',
}

# ---------------------------------------------------------------------------
# Loader
# ---------------------------------------------------------------------------
def load_conditions(ol_rdm_dir):
    folder = Path(ol_rdm_dir)
    data = {'CLOSED_LOOP': [], 'REPLAY': []}

    for jf in sorted(folder.glob('*.json')):
        try:
            with open(jf) as f:
                d = json.load(f)
            condition = d.get('meta', {}).get('condition', None)
            pct       = d.get('reduction', {}).get('magnitude_reduction_pct', None)
            if condition in data and pct is not None:
                data[condition].append(pct)
                print(f"  {jf.name}: {condition} → {pct:.1f}%")
        except Exception as e:
            print(f"  [ERROR] {jf.name}: {e}")

    for cond, vals in data.items():
        print(f"  Total {cond}: {len(vals)} samples")

    return {k: np.array(v) for k, v in data.items() if len(v) > 0}

# ---------------------------------------------------------------------------
# Plot helper
# ---------------------------------------------------------------------------
def draw_ci_box(ax, x, values, color, box_width=0.42):
    mean  = np.mean(values)
    se    = stats.sem(values)
    ci    = se * stats.t.ppf(0.975, df=max(len(values) - 1, 1))
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
# Main
# ---------------------------------------------------------------------------
def main():
    print(f"\nLoading from {OL_RDM_DIR}")
    all_data = load_conditions(OL_RDM_DIR)

    if not all_data:
        print("[ERROR] No data found.")
        return

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    np.random.seed(42)

    cond_keys = [c for c in ['CLOSED_LOOP', 'REPLAY'] if c in all_data]
    all_vals  = np.concatenate(list(all_data.values()))
    y_min     = min(all_vals) - 10
    y_max     = max(all_vals) + 10

    fig, ax = plt.subplots(figsize=(6, 7))

    for i, cond in enumerate(cond_keys):
        values = all_data[cond]
        color  = CONDITION_COLORS[cond]
        jitter = np.random.normal(0, 0.07, size=len(values))
        ax.scatter(i + jitter, values,
                   color=color, alpha=0.85, s=60, zorder=3,
                   edgecolors=color, linewidth=0.5)

        mean, ci_lo, ci_hi = draw_ci_box(ax, i, values, color)

        ax.annotate(
            f'{mean:.1f}%',
            xy=(i + 0.22, mean),
            xytext=(i + 0.35, mean),
            fontsize=11, fontweight='bold', color=color,
            va='center',
            arrowprops=dict(arrowstyle='->', color=color, lw=1.2)
        )
        ax.text(i, y_min + 1, f'n={len(values)}',
                ha='center', va='top', fontsize=9, color='#555')

    # Stats
    if 'CLOSED_LOOP' in all_data and 'REPLAY' in all_data:
        cl   = all_data['CLOSED_LOOP']
        rep  = all_data['REPLAY']
        t, p = stats.ttest_ind(cl, rep)
        sig  = ('***' if p < 0.001 else '**' if p < 0.01 else
                '*'   if p < 0.05  else 'ns')
        n1, n2 = len(cl), len(rep)
        pooled = np.sqrt(((n1-1)*np.var(cl, ddof=1) + (n2-1)*np.var(rep, ddof=1)) / (n1+n2-2))
        d      = (np.mean(cl) - np.mean(rep)) / pooled

        # Draw significance bracket
        x1, x2 = 0, 1
        y_br = max(all_vals) + 4
        ax.plot([x1, x1, x2, x2], [y_br - 1, y_br, y_br, y_br - 1],
                color='#444', lw=1.2)
        ax.text((x1 + x2) / 2, y_br + 0.3, f'{sig}  (d={d:.2f})',
                ha='center', va='bottom', fontsize=11, color='#444')

        print(f"\n{'='*50}")
        print(f"CLOSED_LOOP: mean={np.mean(cl):.1f}%  n={n1}")
        print(f"REPLAY:      mean={np.mean(rep):.1f}%  n={n2}")
        print(f"t={t:.3f}  p={p:.4f} {sig}  Cohen's d={d:.2f}")
        print(f"{'='*50}")

    ax.axhline(0, color='black', lw=0.8, alpha=0.3)
    ax.set_xticks(range(len(cond_keys)))
    ax.set_xticklabels([CONDITION_LABELS[c] for c in cond_keys], fontsize=12)
    ax.set_ylabel('Flutter Magnitude Reduction (%)', fontsize=12, fontweight='bold')
    ax.set_title('Live Closed Loop vs Closed Loop Replay\nIndividual samples with mean and 95% CI',
                 fontsize=12, fontweight='bold')
    ax.grid(True, axis='y', alpha=0.25, linestyle=':')
    ax.set_xlim(-0.6, len(cond_keys) - 0.4)
    ax.set_ylim(y_min, y_max)

    plt.tight_layout()
    for ext in ['png', 'pdf']:
        path = OUTPUT_DIR / f'CL_vs_replay.{ext}'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f"[SAVED] CL_vs_replay.{ext}")

    plt.show()


if __name__ == '__main__':
    main()