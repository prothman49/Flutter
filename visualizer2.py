#!/usr/bin/env python3
"""
plot_conditions_cartoon.py
==========================
Simulates and plots all five flutter control conditions as a cartoon figure,
mirroring the logic from flutter_control_node.py.

Usage:
    python3 plot_conditions_cartoon.py
    python3 plot_conditions_cartoon.py --cycles 20 --seed 42 --output my_figure.pdf
"""

import argparse
import math
import random

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

matplotlib.rcParams.update({
    'font.family':    'sans-serif',
    'font.size':      9,
    'axes.labelsize': 9,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'axes.linewidth': 0.8,
    'lines.linewidth': 1.6,
    'pdf.fonttype':   42,
    'ps.fonttype':    42,
})

# ---------------------------------------------------------------------------
# Constants — match flutter_control_node.py exactly
# ---------------------------------------------------------------------------
FLUTTER_FREQ = 5.4
PULSE_PERIOD = 1.0 / FLUTTER_FREQ   # ~0.1852 s
PULSE_WIDTH  = PULSE_PERIOD          # one full period per pulse
MIN_IPI      = PULSE_PERIOD          # floor for stochastic conditions
PHASE_OFFSET = math.radians(225.0)   # fixed phase offset for CL/Poisson/Uniform

DT = 0.0005  # 2 kHz — fine enough to make square edges look sharp

CONDITIONS = [
    dict(key='CLOSED_LOOP',         label='Closed loop',         color='#2ca02c'),
    dict(key='OPEN_LOOP_54',        label='Open loop 5.4 Hz',    color='#777777'),
    dict(key='POISSON',             label='Poisson',             color='#d7191c'),
    dict(key='UNIFORM_JITTER',      label='Uniform jitter',      color='#9c27b0'),
    dict(key='POLARITY_RANDOMIZED', label='Polarity randomized', color='#f44336'),
]

WING_COLOR   = '#2c7bb6'
SEG_COLORS   = ['#fff0ee', '#f0fff4', '#f4f4ff']   # unc / ctrl / quiet
SEG_ALPHAS   = [0.55,       0.55,      0.45]
THRESH_COLOR = '#c98a00'


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def make_segments(n_cycles):
    """Return (T, t1, t2) — segment boundaries in seconds."""
    T  = n_cycles / FLUTTER_FREQ
    t1 = 2.0 / FLUTTER_FREQ          # uncontrolled → controlled
    t2 = (n_cycles - 2) / FLUTTER_FREQ  # controlled → below threshold
    return T, t1, t2


def wing_amp(t, t1, t2):
    if t < t1:  return 1.0
    if t < t2:  return 0.55
    return 0.18


def pulse_value(t_local, phase_offset_rad):
    """
    Mirror _emit_pulse():
      phase < π  → first half HIGH (+1), second half LOW  (-1)
      phase >= π → first half LOW  (-1), second half HIGH (+1)
    """
    first_half_high = phase_offset_rad < math.pi
    in_first_half   = t_local < PULSE_WIDTH / 2.0
    if in_first_half:
        return 1.0 if first_half_high else -1.0
    else:
        return -1.0 if first_half_high else 1.0


# ---------------------------------------------------------------------------
# Simulators
# ---------------------------------------------------------------------------
def simulate(key, n_cycles, rng):
    T, t1, t2 = make_segments(n_cycles)
    N = int(round(T / DT))
    time = np.arange(N) * DT

    # Wing signal
    amp_arr = np.where(time < t1, 1.0, np.where(time < t2, 0.55, 0.18))
    wing    = amp_arr * np.sin(2 * math.pi * FLUTTER_FREQ * time)

    # Actuator signal — NaN where off
    act = np.full(N, np.nan)

    in_ctrl = (time >= t1) & (time < t2)

    if key == 'CLOSED_LOOP':
        ctrl_idx = np.where(in_ctrl)[0]
        if len(ctrl_idx):
            t_start = time[ctrl_idx[0]]
            phases  = (2 * math.pi * FLUTTER_FREQ * (time[ctrl_idx] - t_start) + PHASE_OFFSET) % (2 * math.pi)
            act[ctrl_idx] = np.where(phases < math.pi, 1.0, -1.0)

    elif key == 'OPEN_LOOP_54':
        # No phase offset — starts at 0°, visually distinct from closed loop
        ctrl_idx = np.where(in_ctrl)[0]
        if len(ctrl_idx):
            t_start = time[ctrl_idx[0]]
            phases  = (2 * math.pi * FLUTTER_FREQ * (time[ctrl_idx] - t_start)) % (2 * math.pi)
            act[ctrl_idx] = np.where(phases < math.pi, 1.0, -1.0)

    elif key == 'POISSON':
        t_pulse = t1
        while t_pulse < t2:
            k0 = int(round(t_pulse / DT))
            k1 = min(int(round((t_pulse + PULSE_WIDTH) / DT)), N)
            for k in range(k0, k1):
                act[k] = pulse_value(time[k] - t_pulse, PHASE_OFFSET)
            ipi = max(MIN_IPI, rng.expovariate(1.0 / PULSE_PERIOD))
            t_pulse += ipi

    elif key == 'UNIFORM_JITTER':
        t_pulse   = t1
        half_period = PULSE_PERIOD / 2.0
        while t_pulse < t2:
            k0 = int(round(t_pulse / DT))
            k1 = min(int(round((t_pulse + PULSE_WIDTH) / DT)), N)
            for k in range(k0, k1):
                act[k] = pulse_value(time[k] - t_pulse, PHASE_OFFSET)
            ipi = max(MIN_IPI, rng.uniform(PULSE_PERIOD - half_period,
                                           PULSE_PERIOD + half_period))
            t_pulse += ipi

    elif key == 'POLARITY_RANDOMIZED':
        t_pulse = t1
        while t_pulse < t2:
            k0 = int(round(t_pulse / DT))
            k1 = min(int(round((t_pulse + PULSE_WIDTH) / DT)), N)
            rand_phase = rng.uniform(0, 2 * math.pi)  # coin flip via continuous draw
            for k in range(k0, k1):
                act[k] = pulse_value(time[k] - t_pulse, rand_phase)
            t_pulse += PULSE_PERIOD  # exact 5.4 Hz

    return time, wing, act, T, t1, t2


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
def plot(n_cycles=20, seed=42, output=None):
    fig, axes = plt.subplots(
        len(CONDITIONS), 1,
        figsize=(12, 8),
        gridspec_kw={'hspace': 0.55},
    )
    fig.subplots_adjust(left=0.13, right=0.99, top=0.95, bottom=0.10)

    for i, (ax, cond) in enumerate(zip(axes, CONDITIONS)):
        rng = random.Random(seed + i * 1000)
        time, wing, act, T, t1, t2 = simulate(cond['key'], n_cycles, rng)

        # Segment backgrounds
        for (ta, tb), col, alpha in zip(
            [(0, t1), (t1, t2), (t2, T)],
            SEG_COLORS, SEG_ALPHAS
        ):
            ax.axvspan(ta, tb, color=col, alpha=alpha, zorder=0)

        # Segment labels (top panel only)
        if i == 0:
            for (ta, tb), lbl in zip(
                [(0, t1), (t1, t2), (t2, T)],
                ['uncontrolled', 'controlled', 'below threshold']
            ):
                ax.text((ta + tb) / 2, 1.55, lbl,
                        ha='center', va='bottom', fontsize=7.5,
                        color='#555', style='italic',
                        transform=ax.get_xaxis_transform())

        # Threshold lines
        ax.axhline( 0.55, color=THRESH_COLOR, lw=0.9, ls=':', alpha=0.7, zorder=1)
        ax.axhline(-0.55, color=THRESH_COLOR, lw=0.9, ls=':', alpha=0.7, zorder=1)
        ax.axhline(0,     color='#ccc',        lw=0.5, ls='--',           zorder=1)

        # Segment dividers
        for tb in [t1, t2]:
            ax.axvline(tb, color='#aaa', lw=0.7, zorder=2)

        # Wing
        ax.plot(time, wing, color=WING_COLOR, lw=1.6, zorder=3, label='Wing')

        # Actuator — use drawstyle='steps-post' equivalent via manual step plot
        # Build a masked array so gaps (NaN) break the line naturally
        ax.plot(time, act,
                color=cond['color'], lw=1.8, zorder=4,
                drawstyle='steps-post',
                label=cond['label'])

        # Condition label (left of plot)
        ax.text(-0.005, 0.5, cond['label'],
                transform=ax.transAxes,
                fontsize=9, fontweight='bold',
                ha='right', va='center',
                color=cond['color'])

        ax.set_xlim(0, T)
        ax.set_ylim(-1.3, 1.7)
        ax.set_yticks([])
        ax.spines[['top', 'right', 'left']].set_visible(False)

        if i < len(CONDITIONS) - 1:
            ax.tick_params(bottom=False, labelbottom=False)
            ax.spines['bottom'].set_visible(False)
        else:
            ax.set_xlabel('Time (s)', fontsize=9)
            tick_times = np.arange(0, T + 0.001, PULSE_PERIOD * 4)
            ax.set_xticks(tick_times)
            ax.set_xticklabels([f'{x:.2f}' for x in tick_times])

    # Legend
    legend_elements = [
        mpatches.Patch(color=WING_COLOR,  label='Wing oscillation'),
        mpatches.Patch(color='#fff0ee',   alpha=0.8, label='Uncontrolled'),
        mpatches.Patch(color='#f0fff4',   alpha=0.8, label='Control active'),
        mpatches.Patch(color='#f4f4ff',   alpha=0.8, label='Below threshold'),
        mpatches.Patch(color=THRESH_COLOR, label='Flutter threshold'),
    ] + [
        mpatches.Patch(color=c['color'], label=c['label']) for c in CONDITIONS
    ]
    fig.legend(
        handles=legend_elements,
        loc='lower center',
        bbox_to_anchor=(0.56, 0.0),
        ncol=5,
        fontsize=7.5,
        frameon=True,
        framealpha=0.9,
        edgecolor='#ccc',
    )

    fig.suptitle('Flutter Control Conditions — Simulated Cartoon', fontsize=11, y=0.98)

    if output:
        plt.savefig(output, dpi=300, bbox_inches='tight')
        # Also save PNG alongside PDF
        if output.endswith('.pdf'):
            plt.savefig(output.replace('.pdf', '.png'), dpi=150, bbox_inches='tight')
        print(f'[SAVED] {output}')
    else:
        plt.savefig('control_conditions_cartoon.pdf', dpi=300, bbox_inches='tight')
        plt.savefig('control_conditions_cartoon.png', dpi=150, bbox_inches='tight')
        print('[SAVED] control_conditions_cartoon.pdf / .png')

    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Plot flutter control condition cartoon')
    parser.add_argument('--cycles', type=int,   default=20,
                        help='Number of flutter cycles to show (default 20)')
    parser.add_argument('--seed',   type=int,   default=42,
                        help='Random seed for stochastic conditions (default 42)')
    parser.add_argument('--output', type=str,   default=None,
                        help='Output filename (default: control_conditions_cartoon.pdf)')
    args = parser.parse_args()

    plot(n_cycles=args.cycles, seed=args.seed, output=args.output)


if __name__ == '__main__':
    main()