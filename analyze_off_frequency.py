#!/usr/bin/env python3
"""
Off-Frequency Control Validation
Individual points + mean line + 95% CI box, styled like reference image.
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path
from scipy import stats

def load_closed_loop_n30(base_path):
    """
    Load n=30 closed-loop reference from statistical_comparison JSONs.
    Looks in OL_RDM subfolder. Per-trial reduction = (off - on) / off * 100.
    """
    files = [
        base_path / 'OL_RDM' / 'statistical_comparison_20260202_150833.json',
        base_path / 'OL_RDM' / 'statistical_comparison_20260202_155303.json',
    ]
    reductions = []
    for jf in files:
        if not jf.exists():
            # fallback: same folder as script
            jf = Path(jf.name)
        if not jf.exists():
            print(f"  [WARN] Not found: {jf.name}")
            continue
        with open(jf) as f:
            data = json.load(f)
        off_vals = data.get('control_off', [])
        on_vals  = data.get('control_on',  [])
        for off, on in zip(off_vals, on_vals):
            if off > 0:
                reductions.append((off - on) / off * 100)
        print(f"  Loaded: {jf.name}  ({data.get('n_trials', len(off_vals))} trials)")
    print(f"  → {len(reductions)} trials, mean = {np.mean(reductions):.1f}%\n")
    return np.array(reductions)


def load_json_files(base_path, folder_name, frequency):
    folder = base_path / folder_name
    if not folder.exists():
        print(f"Warning: {folder} doesn't exist")
        return []
    results = []
    patterns = [
        f'off_frequency_test_{frequency}*.json',
        f'*{frequency}hz*.json',
        'dc_control_summary*.json' if frequency == 0 else None,
        'phase_90deg*.json' if folder_name == 'phase_90' else None,
    ]
    for pattern in patterns:
        if pattern is None:
            continue
        for json_file in sorted(folder.glob(pattern)):
            if any(x in json_file.name for x in ['run1__', 'run2__', 'run3__']):
                continue
            try:
                with open(json_file, 'r') as f:
                    data = json.load(f)
                if 'effectiveness' in data or 'statistics' in data:
                    results.append(data)
                    print(f"  Loaded: {json_file.name}")
            except Exception as e:
                print(f"  Warning: Could not load {json_file.name}: {e}")
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

def draw_ci_box(ax, x_center, values, color, box_width=0.35):
    """Draw mean line + dashed 95% CI rectangle"""
    mean   = np.mean(values)
    n      = len(values)
    se     = stats.sem(values)
    ci     = se * stats.t.ppf(0.975, df=n - 1)
    ci_lo  = mean - ci
    ci_hi  = mean + ci

    half = box_width / 2

    # Dashed CI rectangle
    rect = mpatches.FancyBboxPatch(
        (x_center - half, ci_lo), box_width, ci_hi - ci_lo,
        boxstyle="square,pad=0",
        linewidth=1.8, linestyle="--",
        edgecolor=color, facecolor="none", zorder=4
    )
    ax.add_patch(rect)

    # Solid mean line
    ax.plot([x_center - half, x_center + half], [mean, mean],
            color=color, linewidth=2.5, zorder=5)

    return mean, ci_lo, ci_hi

def load_new_session_jsons(json_dir, condition):
    """
    Load magnitude_reduction_pct from new-style per-sample JSONs
    produced by run_full_protocol.py.
    """
    json_dir = Path(json_dir)
    reductions = []
    for jf in sorted(json_dir.glob(f'{condition}_sample*.json')):
        try:
            with open(jf) as f:
                d = json.load(f)
            pct = d.get('reduction', {}).get('magnitude_reduction_pct', None)
            if pct is not None:
                reductions.append(pct)
                print(f"  {jf.name}: {pct:.1f}%")
        except Exception as e:
            print(f"  [ERROR] {jf.name}: {e}")
    return np.array(reductions)


def main():
    base_path = Path.home() / 'Desktop' / 'Flutter_analysis' / 'figures_for_dissertation' / 'off_frequency_tests'

    # New session JSON directory — OL_RDM folder on Mac
    new_json_dir = base_path / 'OL_RDM'

    print(f"Looking for off-frequency data in: {base_path}")
    print(f"Looking for new session data in:   {new_json_dir}\n")

    test_configs = [
        {'folder': '0hz',  'freq': 0.0,  'label': '0 Hz\nPiezo 1 ON at 2140V DC'},
        {'folder': '3hz',  'freq': 3.0,  'label': '3 Hz'},
        {'folder': '8hz',  'freq': 8.0,  'label': '8 Hz'},
        {'folder': '10hz', 'freq': 10.0, 'label': '10 Hz'},
    ]

    all_data = {}
    print("Loading data files...\n")

    # ── Closed-loop — new session JSONs ──────────────────────────────────────
    print("Closed Loop (new session):")
    cl_reductions = load_new_session_jsons(new_json_dir, 'CLOSED_LOOP')
    if len(cl_reductions) > 0:
        all_data[5.4] = {
            'reductions': cl_reductions,
            'mean': np.mean(cl_reductions),
            'std':  np.std(cl_reductions, ddof=1),
            'sem':  stats.sem(cl_reductions),
            'n':    len(cl_reductions),
            'label': '5.4 Hz',
        }
        print(f"  → {len(cl_reductions)} trials, mean = {np.mean(cl_reductions):.1f}%\n")
    else:
        print(f"  → No data found in {new_json_dir}\n")

    # ── Open loop 5.5 Hz — new session JSONs ─────────────────────────────────
    print("Open Loop 5.5 Hz (new session):")
    ol_reductions = load_new_session_jsons(new_json_dir, 'OPEN_LOOP_55')
    if len(ol_reductions) > 0:
        all_data['ol55'] = {
            'reductions': ol_reductions,
            'mean': np.mean(ol_reductions),
            'std':  np.std(ol_reductions, ddof=1),
            'sem':  stats.sem(ol_reductions),
            'n':    len(ol_reductions),
            'label': '5.5 Hz\nOpen Loop\n(no sync)',
        }
        print(f"  → {len(ol_reductions)} trials, mean = {np.mean(ol_reductions):.1f}%\n")
    else:
        print(f"  → No data found\n")

    # ── Off-frequency conditions ──────────────────────────────────────────────
    for config in test_configs:
        print(f"{config['label'].replace(chr(10), ' ')}:")
        results    = load_json_files(base_path, config['folder'], config['freq'])
        reductions = extract_magnitude_reduction(results)
        if len(reductions) > 0:
            all_data[config['freq']] = {
                'reductions': reductions,
                'mean': np.mean(reductions),
                'std':  np.std(reductions, ddof=1),
                'sem':  stats.sem(reductions),
                'n':    len(reductions),
                'label': config['label'],
            }
            print(f"  → {len(reductions)} trials, mean = {np.mean(reductions):.1f}%\n")
        else:
            print(f"  → No data found\n")

    # Build ordered condition list — ol55 merges at the 5.4 Hz position
    ordered_keys = []
    for f in sorted(k for k in all_data.keys() if k != 'ol55'):
        ordered_keys.append(f)

    n_cond = len(ordered_keys)

    color_map = {
        0.0:    '#888888',
        3.0:    '#e07070',
        5.4:    '#2e8b57',
        'ol55': '#e07070',
        8.0:    '#e07070',
        10.0:   '#e07070',
    }

    fig, ax = plt.subplots(figsize=(13, 7))
    np.random.seed(42)

    x_positions = {k: i for i, k in enumerate(ordered_keys)}
    if 'ol55' in all_data:
        x_positions['ol55'] = x_positions[5.4]

    plot_order = ordered_keys.copy()
    if 'ol55' in all_data:
        plot_order.append('ol55')

    for key in plot_order:
        d      = all_data[key]
        x      = x_positions[key]
        color  = color_map[key]
        values = d['reductions']

        jitter = np.random.normal(0, 0.07, size=len(values))
        ax.scatter(x + jitter, values,
                   color=color, alpha=0.85, s=55, zorder=3,
                   edgecolors=color, linewidth=0.5)

        mean, ci_lo, ci_hi = draw_ci_box(ax, x, values, color, box_width=0.42)

        sig_label = ''
        if 5.4 in all_data and key != 5.4:
            matched = all_data[5.4]['reductions']
            _, p = stats.ttest_ind(matched, values)
            sig_label = '***' if p < 0.001 else '**' if p < 0.01 else '*' if p < 0.05 else 'ns'

        ANN_XY   = x + 0.22
        ANN_TEXT = x + 0.38

        if key == 'ol55':
            ax.annotate(f'{mean:.1f}%\n(open loop)',
                        xy=(ANN_XY, mean), xytext=(ANN_TEXT, mean),
                        fontsize=9, fontweight='bold', color=color, va='center',
                        arrowprops=dict(arrowstyle='->', color=color, lw=1.0))
        else:
            pct_label = f'{mean:.1f}%' if key != 5.4 else f'{mean:.1f}%\n(closed loop, 255°)'
            ax.annotate(pct_label,
                        xy=(ANN_XY, mean), xytext=(ANN_TEXT, mean),
                        fontsize=9, fontweight='bold', color=color, va='center',
                        arrowprops=dict(arrowstyle='->', color=color, lw=1.2))

        if sig_label:
            ax.text(ANN_TEXT, mean + 2.5, sig_label,
                    ha='center', va='bottom', fontsize=11,
                    color='#444444', fontweight='bold')

    # ── Zero line ─────────────────────────────────────────────────────────────
    ax.axhline(0, color='black', linestyle='-', linewidth=0.8, alpha=0.3)

    # ── Axes formatting ───────────────────────────────────────────────────────
    ax.set_xticks(range(n_cond))
    ax.set_xticklabels([all_data[k]['label'] for k in ordered_keys], fontsize=11)
    ax.set_ylabel('Flutter Magnitude Reduction (%)', fontsize=13, fontweight='bold')
    ax.set_xlabel('Control Frequency', fontsize=13, fontweight='bold')
    ax.set_title('Off-Frequency Control Effectiveness\nIndividual trials with mean and 95% CI',
                 fontsize=14, fontweight='bold')
    ax.grid(True, axis='y', alpha=0.25, linestyle=':')
    ax.set_xlim(-0.6, n_cond - 0.4)

    # Dynamic y limits
    all_vals = np.concatenate([all_data[k]['reductions'] for k in list(ordered_keys) + (['ol55'] if 'ol55' in all_data else [])])
    ax.set_ylim(min(all_vals) - 8, max(all_vals) + 8)

    plt.tight_layout()

    from datetime import datetime
    dated_dir   = base_path / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_off_frequency')
    dated_dir.mkdir(parents=True, exist_ok=True)
    output_path = dated_dir / 'off_frequency_validation_new.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.savefig(str(output_path).replace('.png', '.pdf'), bbox_inches='tight')
    print(f"\nFigure saved to: {output_path}")

    # ── Print stats ───────────────────────────────────────────────────────────
    print("\n" + "="*65)
    print("SUMMARY STATISTICS")
    print("="*65)
    for key in ordered_keys:
        d = all_data[key]
        n  = d['n']
        ci = stats.sem(d['reductions']) * stats.t.ppf(0.975, df=n-1)
        print(f"{d['label'].replace(chr(10),' '):25} {d['mean']:6.1f}% ± {ci:.2f}% CI  (n={n})")
    if 'ol55' in all_data:
        d = all_data['ol55']
        n  = d['n']
        ci = stats.sem(d['reductions']) * stats.t.ppf(0.975, df=n-1)
        print(f"{d['label'].replace(chr(10),' '):25} {d['mean']:6.1f}% ± {ci:.2f}% CI  (n={n})")

    if 5.4 in all_data:
        print("\n" + "="*65)
        print("STATISTICAL TESTS vs Closed Loop")
        print("="*65)
        matched = all_data[5.4]['reductions']
        test_keys = [k for k in ordered_keys if k != 5.4] + (['ol55'] if 'ol55' in all_data else [])
        for key in test_keys:
            test = all_data[key]['reductions']
            _, p = stats.ttest_ind(matched, test)
            n1, n2 = len(matched), len(test)
            pooled = np.sqrt(((n1-1)*np.var(matched,ddof=1) + (n2-1)*np.var(test,ddof=1))/(n1+n2-2))
            d_val  = (np.mean(matched) - np.mean(test)) / pooled
            sig    = '***' if p < 0.001 else '**' if p < 0.01 else '*' if p < 0.05 else 'ns'
            lbl    = all_data[key]['label'].replace('\n',' ')
            print(f"Closed Loop vs {lbl:25}  p={p:.4f} {sig}  Cohen's d={d_val:.2f}")

    print("="*65)
    plt.show()


# ── ORIGINAL PLOT (bar + line) ────────────────────────────────────────────────
def plot_original(all_data):
    numeric_freqs = sorted(k for k in all_data.keys() if k != 'ol55')
    freqs  = numeric_freqs
    means  = [all_data[f]['mean'] for f in freqs]
    sems   = [all_data[f]['sem']  for f in freqs]
    stds   = [all_data[f]['std']  for f in freqs]
    labels = [all_data[f]['label'] for f in freqs]
    ns     = [all_data[f]['n']    for f in freqs]

    colors  = ['gray' if f == 0 else ('darkgreen' if f == 5.4 else 'lightcoral') for f in freqs]
    markers = ['X'    if f == 0 else ('s'         if f == 5.4 else 'o')          for f in freqs]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

    # Line plot with SEM error bars
    for i, (freq, mean, sem, color, marker) in enumerate(zip(freqs, means, sems, colors, markers)):
        lbl = ('No Control' if freq == 0 else
               'Matched Frequency' if freq == 5.8 else
               'Off-Frequency' if i == 1 else None)
        ax1.errorbar(freq, mean, yerr=sem, fmt=marker, color=color,
                     markersize=14, capsize=6, capthick=2.5, linewidth=2.5,
                     label=lbl, elinewidth=2.5, zorder=5)
    ax1.plot(freqs, means, '--', color='gray', alpha=0.4, linewidth=1.5, zorder=1)
    ax1.axhline(0, color='black', linestyle='-', alpha=0.3, linewidth=1)
    ax1.set_xlabel('Control Frequency (Hz)', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Flutter Magnitude Reduction (%)', fontsize=14, fontweight='bold')
    ax1.set_title('Off-Frequency Control Effectiveness\n(Error bars = SEM)',
                  fontsize=15, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=12, framealpha=0.9)
    ax1.set_ylim(min(means) - max(sems) - 3, max(means) + max(sems) + 3)

    # Bar chart with individual points
    x_pos = np.arange(len(freqs))
    bars  = ax2.bar(x_pos, means, yerr=stds, capsize=5,
                    color=colors, edgecolor='black', linewidth=1.5,
                    alpha=0.7, error_kw={'linewidth': 2})

    for i, freq in enumerate(freqs):
        values   = all_data[freq]['reductions']
        x_jitter = np.random.normal(i, 0.045, size=len(values))
        ax2.scatter(x_jitter, values, alpha=0.8, s=45,
                    color='white', edgecolors='black', linewidth=1.0, zorder=10)

    if 5.4 in all_data and len(all_data) > 2:
        matched_data = all_data[5.4]['reductions']
        for i, (bar, freq, mean, std, n) in enumerate(zip(bars, freqs, means, stds, ns)):
            if freq != 5.8:
                _, p_val = stats.ttest_ind(matched_data, all_data[freq]['reductions'])
                sig = '***' if p_val < 0.001 else '**' if p_val < 0.01 else '*' if p_val < 0.05 else 'ns'
                label_text = f'{sig}\n{mean:.1f}%\n(n={n})'
            else:
                label_text = f'{mean:.1f}%\n(n={n})'
            ax2.text(bar.get_x() + bar.get_width()/2., -20,
                     label_text, ha='center', va='bottom', fontsize=10, fontweight='bold')

    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(labels, fontsize=11)
    ax2.set_ylabel('Flutter Magnitude Reduction (%)', fontsize=14, fontweight='bold')
    ax2.set_title('Individual Trial Results\n(Error bars = Std Dev, dots = individual trials)',
                  fontsize=15, fontweight='bold')
    ax2.grid(True, alpha=0.3, axis='y')
    ax2.axhline(0, color='black', linestyle='-', alpha=0.3, linewidth=1)
    ax2.set_ylim(min(-5, min(means) - max(stds) - 10),
                 max(means) + max(stds) + 10)

    plt.tight_layout()
    base_path = Path.home() / 'Desktop' / 'Flutter_analysis' / 'figures_for_dissertation' / 'off_frequency_tests'
    output_path = base_path.parent / 'off_frequency_tests' / 'off_frequency_validation_original.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Original figure saved to: {output_path}")
    plt.show()


# ── PATCH MAIN TO ALSO CALL ORIGINAL PLOT ─────────────────────────────────────
if __name__ == '__main__':
    main()
    # Re-load data and call original plot
    base_path    = Path.home() / 'Desktop' / 'Flutter_analysis' / 'figures_for_dissertation' / 'off_frequency_tests'
    new_json_dir = base_path / 'OL_RDM'

    test_configs2 = [
        {'folder': '0hz',  'freq': 0.0,  'label': '0 Hz\n(DC)'},
        {'folder': '3hz',  'freq': 3.0,  'label': '3 Hz'},
        {'folder': '8hz',  'freq': 8.0,  'label': '8 Hz'},
        {'folder': '10hz', 'freq': 10.0, 'label': '10 Hz'},
    ]
    all_data2 = {}

    # Closed loop — new session
    cl2 = load_new_session_jsons(new_json_dir, 'CLOSED_LOOP')
    if len(cl2) > 0:
        all_data2[5.4] = {
            'reductions': cl2,
            'mean': np.mean(cl2),
            'std':  np.std(cl2, ddof=1),
            'sem':  stats.sem(cl2),
            'n':    len(cl2),
            'label': '5.4 Hz\nFrequency Sync\n255° Phase Delay',
        }

    # Off-frequency
    for config in test_configs2:
        results    = load_json_files(base_path, config['folder'], config['freq'])
        reductions = extract_magnitude_reduction(results)
        if len(reductions) > 0:
            all_data2[config['freq']] = {
                'reductions': reductions,
                'mean': np.mean(reductions),
                'std':  np.std(reductions, ddof=1),
                'sem':  stats.sem(reductions),
                'n':    len(reductions),
                'label': config['label'],
            }

    # Open loop 5.5 Hz — new session
    ol2 = load_new_session_jsons(new_json_dir, 'OPEN_LOOP_55')
    if len(ol2) > 0:
        all_data2['ol55'] = {
            'reductions': ol2,
            'mean': np.mean(ol2),
            'std':  np.std(ol2, ddof=1),
            'sem':  stats.sem(ol2),
            'n':    len(ol2),
            'label': '5.5 Hz\nOpen Loop\n(no sync)',
        }

    plot_original(all_data2)