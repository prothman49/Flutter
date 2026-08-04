#!/usr/bin/env python3
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import random
import math

matplotlib.rcParams.update({
    'font.family':      'sans-serif',
    'font.size':        9,
    'axes.labelsize':   9,
    'xtick.labelsize':  8,
    'ytick.labelsize':  8,
    'axes.linewidth':   0.8,
    'lines.linewidth':  1.6,
    'pdf.fonttype':     42,
    'ps.fonttype':      42,
})

FLUTTER_FREQ     = 5.4
PULSE_PERIOD     = 1.0 / FLUTTER_FREQ
FIXED_PHASE_RAD  = math.radians(225.0)
OPEN_LOOP_FREQ   = 5.9   # enough mismatch to drift visibly over 20 cycles

# 20 cycles: 2 uncontrolled, 14 controlled, 4 quiet
N_CYCLES   = 20
T_END      = N_CYCLES / FLUTTER_FREQ
SEG_BOUNDS = [0, 2/FLUTTER_FREQ, 16/FLUTTER_FREQ, T_END]
SEG_AMPS   = [1.0, 0.55, 0.22]
SEG_CTRL   = [False, True, False]
THRESHOLD  = 0.50

DT   = 0.0002
N    = int(T_END / DT)
TIME = np.linspace(0, T_END, N)

random.seed(42)
np.random.seed(42)

def in_ctrl(t):
    for t0, t1, c in zip(SEG_BOUNDS[:-1], SEG_BOUNDS[1:], SEG_CTRL):
        if c and t0 <= t < t1:
            return True
    return False

# ---------------------------------------------------------------------------
# Wing
# ---------------------------------------------------------------------------
def build_wing():
    sig  = np.zeros(N)
    RAMP = int(0.3 * PULSE_PERIOD / DT)
    for t0, t1, amp in zip(SEG_BOUNDS[:-1], SEG_BOUNDS[1:], SEG_AMPS):
        i0 = int(t0 / DT)
        i1 = min(int(t1 / DT), N)
        for k in range(i0, i1):
            r_in  = min((k - i0) / max(RAMP, 1), 1.0)
            r_out = min((i1 - k) / max(RAMP, 1), 1.0)
            sig[k] = amp * min(r_in, r_out) * math.sin(2*math.pi*FLUTTER_FREQ * k*DT)
    return sig

wing = build_wing()

# ---------------------------------------------------------------------------
# Continuous square wave (array-based, no inter-pulse boundary issues)
# ---------------------------------------------------------------------------
def simulate_continuous(freq, phase_offset):
    sig     = np.zeros(N)
    running = False
    t_start = 0.0
    for k in range(N):
        t = k * DT
        if in_ctrl(t) and not running:
            running = True
            t_start = t
        if not in_ctrl(t):
            running = False
        if running:
            phase  = (2*math.pi*freq*(t - t_start) + phase_offset) % (2*math.pi)
            sig[k] = 1.0 if phase < math.pi else -1.0
    return ('array', sig)

# ---------------------------------------------------------------------------
# NaN-separated pulse segments
# ---------------------------------------------------------------------------
def pulse_segments(t_start, phi, scale=0.42):
    t_end  = t_start + PULSE_PERIOD
    t_half = t_start + PULSE_PERIOD / 2.0
    phi_norm = phi % (2*math.pi)
    fv =  scale if phi_norm < math.pi else -scale
    sv = -fv
    xs = [t_start, t_start, t_half,  t_half,  t_half, t_half,  t_end,  t_end,  np.nan]
    ys = [0,       fv,      fv,      fv,      sv,     sv,      sv,     0,      np.nan]
    return xs, ys

def build_xy(events, scale=0.42):
    xs_all, ys_all = [], []
    for t_start, phi in events:
        if t_start >= T_END:
            break
        xs, ys = pulse_segments(t_start, phi, scale)
        xs_all.extend(xs)
        ys_all.extend(ys)
    return ('xy', (np.array(xs_all), np.array(ys_all)))

# ---------------------------------------------------------------------------
# Event generators
# ---------------------------------------------------------------------------
def poisson_events():
    events, t = [], 0.0
    while t < T_END:
        if in_ctrl(t):
            events.append((t, FIXED_PHASE_RAD))
        t += max(PULSE_PERIOD, random.expovariate(1.0/PULSE_PERIOD))
    return events

def uniform_jitter_events():
    events, t = [], 0.0
    while t < T_END:
        if in_ctrl(t):
            events.append((t, FIXED_PHASE_RAD))
        t += max(PULSE_PERIOD, random.uniform(PULSE_PERIOD*0.5, PULSE_PERIOD*1.5))
    return events

def phase_randomized_events():
    events, t = [], 0.0
    while t < T_END:
        if in_ctrl(t):
            events.append((t, random.uniform(0, 2*math.pi)))
        t += PULSE_PERIOD
    return events

# ---------------------------------------------------------------------------
# Build signals
# ---------------------------------------------------------------------------
SCALE = 0.42

signals = [
    simulate_continuous(FLUTTER_FREQ,   FIXED_PHASE_RAD),
    simulate_continuous(OPEN_LOOP_FREQ, FIXED_PHASE_RAD),
    build_xy(poisson_events()),
    build_xy(uniform_jitter_events()),
    build_xy(phase_randomized_events()),
]

LABELS = ['Closed Loop', 'Open Loop 5.4 Hz', 'Poisson Pulses',
          'Uniform Jitter', 'Polarity Randomized']
DESCS  = [
    f'Tracks flutter frequency in real time; phase locked to wing',
    f'Fixed freq ({OPEN_LOOP_FREQ} Hz cartoon); no feedback — phase drifts relative to wing over time',
    'Fixed phase offset; inter-pulse interval ~ Exp(185 ms), floor 185 ms',
    'Fixed phase offset; inter-pulse interval ~ Uniform(93–278 ms)',
    'Exact 5.4 Hz rate; actuation polarity (push/pull) randomized each pulse',
]

COLOR_WING = '#2c7bb6'
COLOR_ACT  = '#d7191c'
SEG_COLORS = ['#fff0f0', '#f0fff0', '#f5f5ff']
SEG_LABELS = ['Flutter (no control)', 'Control ON', 'Below threshold']

# ---------------------------------------------------------------------------
# Plot — wide figure
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(5, 1, figsize=(12.0, 8.0), gridspec_kw={'hspace': 0.65})
fig.subplots_adjust(left=0.13, right=0.99, top=0.96, bottom=0.09)

for i, (ax, label, desc, sig_tuple) in enumerate(zip(axes, LABELS, DESCS, signals)):

    for t0, t1, col in zip(SEG_BOUNDS[:-1], SEG_BOUNDS[1:], SEG_COLORS):
        ax.axvspan(t0, t1, color=col, alpha=0.5, zorder=0)
    ax.axhline( THRESHOLD, color='#999999', lw=0.8, ls=':', zorder=2)
    ax.axhline(-THRESHOLD, color='#999999', lw=0.8, ls=':', zorder=2)
    ax.axhline(0, color='#cccccc', lw=0.5, ls='--', zorder=1)
    for tb in SEG_BOUNDS[1:-1]:
        ax.axvline(tb, color='#aaaaaa', lw=0.8, zorder=2)

    ax.plot(TIME, wing, color=COLOR_WING, lw=1.8, zorder=3)

    mode, data = sig_tuple
    if mode == 'array':
        ax.plot(TIME, data * SCALE, color=COLOR_ACT, lw=1.3, zorder=4)
    else:
        xs, ys = data
        ax.plot(xs, ys, color=COLOR_ACT, lw=1.3, zorder=4)

    ax.set_xlim(0, T_END)
    ax.set_ylim(-1.25, 1.65)
    ax.set_yticks([])
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)

    ax.text(-0.005, 0.5, label, transform=ax.transAxes,
            fontsize=9, fontweight='bold', ha='right', va='center', color='#222222')
    ax.text(0.005, 0.97, desc, transform=ax.transAxes,
            fontsize=6.5, ha='left', va='top', color='#555555', style='italic')

    if i == 0:
        for t0, t1, sl in zip(SEG_BOUNDS[:-1], SEG_BOUNDS[1:], SEG_LABELS):
            ax.text((t0+t1)/2, 1.62, sl, ha='center', va='bottom', fontsize=6.5,
                    color='#444444', style='italic',
                    bbox=dict(fc='white', ec='none', pad=1.5))

    if i < 4:
        ax.tick_params(bottom=False, labelbottom=False)
        ax.spines['bottom'].set_visible(False)
    else:
        ax.tick_params(bottom=True, labelbottom=True)
        ax.set_xlabel('Time (s)', fontsize=9)
        tick_times = np.arange(0, T_END+0.001, PULSE_PERIOD*4)
        ax.set_xticks(tick_times)
        ax.set_xticklabels([f'{x:.2f}' for x in tick_times])

legend_elements = [
    mpatches.Patch(color=COLOR_WING, label='Wing oscillation'),
    mpatches.Patch(color=COLOR_ACT,  label='Actuator signal'),
    mpatches.Patch(color='#fff0f0',  alpha=0.8, label='Flutter (uncontrolled)'),
    mpatches.Patch(color='#f0fff0',  alpha=0.8, label='Control active'),
    mpatches.Patch(color='#f5f5ff',  alpha=0.8, label='Below threshold'),
]
fig.legend(handles=legend_elements, loc='lower center',
           bbox_to_anchor=(0.55, 0.0), ncol=5,
           fontsize=7.5, frameon=True, framealpha=0.9, edgecolor='#cccccc')

plt.savefig('control_conditions_v12.pdf', dpi=300, bbox_inches='tight')
plt.savefig('control_conditions_v12.png', dpi=300, bbox_inches='tight')
print("Done.")