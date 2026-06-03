#!/usr/bin/env python3
import json
import numpy as np
import os

# Load all sweep files
sweep_dir = os.path.expanduser("~/catkin_ws/src/flutter_control/data/results/phase_sweeps/coarse_sweeps/json")
sweep_files = sorted([f for f in os.listdir(sweep_dir) if f.startswith("sweep_run")])

print(f"Found {len(sweep_files)} sweep files")
print("="*70)

# Collect data by phase
phase_data = {}

for sweep_file in sweep_files:
    with open(os.path.join(sweep_dir, sweep_file), 'r') as f:
        data = json.load(f)
    
    for phase_result in data:
        phase = phase_result['phase']
        
        if phase not in phase_data:
            phase_data[phase] = {
                'magnitudes': [],
                'flutter_percents': [],
                'peak_mags': []
            }
        
        phase_data[phase]['magnitudes'].append(phase_result['avg_magnitude'])
        phase_data[phase]['flutter_percents'].append(phase_result['flutter_percent'])
        phase_data[phase]['peak_mags'].append(phase_result['peak_magnitude'])

# Calculate statistics
print("\nPHASE SWEEP ANALYSIS")
print("="*70)
print(f"{'Phase':>6} | {'Avg Mag':>10} | {'Std Dev':>8} | {'Flutter %':>10} | {'Rank':>6}")
print("-"*70)

# Sort by average magnitude
phase_stats = []
for phase in sorted(phase_data.keys()):
    avg_mag = np.mean(phase_data[phase]['magnitudes'])
    std_mag = np.std(phase_data[phase]['magnitudes'])
    avg_flutter = np.mean(phase_data[phase]['flutter_percents'])
    
    phase_stats.append({
        'phase': phase,
        'avg_mag': avg_mag,
        'std_mag': std_mag,
        'avg_flutter': avg_flutter
    })

# Sort by magnitude (lowest = best)
phase_stats.sort(key=lambda x: x['avg_mag'])

for i, stats in enumerate(phase_stats):
    rank_symbol = "🥇" if i == 0 else "🥈" if i == 1 else "🥉" if i == 2 else "🏴" if i == len(phase_stats)-1 else " "
    print(f"{stats['phase']:>6.0f} | {stats['avg_mag']:>10.1f} | {stats['std_mag']:>8.1f} | {stats['avg_flutter']:>9.1f}% | {rank_symbol:>6}")

print("="*70)
print(f"\n✅ BEST PHASE: {phase_stats[0]['phase']:.0f}° (Magnitude: {phase_stats[0]['avg_mag']:.1f} ± {phase_stats[0]['std_mag']:.1f})")
print(f"❌ WORST PHASE: {phase_stats[-1]['phase']:.0f}° (Magnitude: {phase_stats[-1]['avg_mag']:.1f} ± {phase_stats[-1]['std_mag']:.1f})")
print(f"\n📊 Difference: {((phase_stats[-1]['avg_mag'] - phase_stats[0]['avg_mag']) / phase_stats[0]['avg_mag'] * 100):.1f}%")
print("\n" + "="*70)
print("\nRECOMMENDATIONS FOR VALIDATION:")
print(f"  Good phase to validate: {phase_stats[0]['phase']:.0f}° or {phase_stats[1]['phase']:.0f}°")
print(f"  Bad phase to validate: {phase_stats[-1]['phase']:.0f}° or {phase_stats[-2]['phase']:.0f}°")
print("="*70)