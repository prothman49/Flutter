#!/usr/bin/env python3
"""
Analyze phase_90 files and create summary JSONs
"""

import json
import numpy as np
from pathlib import Path

def analyze_phase_90_file(json_file):
    """Analyze a single phase_90 file with known timing structure"""
    print(f"\nAnalyzing: {json_file.name}")
    
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    print(f"  Keys: {list(data.keys())}")
    
    # Get data arrays
    time = np.array(data['time'])
    optic_flow_vx = np.array(data['optic_flow_vx'])
    
    print(f"  Samples: {len(optic_flow_vx)}")
    print(f"  Duration: {time[-1] - time[0]:.1f}s")
    
    # Timing: OFF (120s) → settling (60s) → ON (120s) → settling (60s) → repeat 5×
    off_duration = 120
    on_duration = 120
    settling_duration = 60
    n_pairs = 5
    
    # Total per cycle: 120 + 60 + 120 + 60 = 360s
    cycle_total = off_duration + settling_duration + on_duration + settling_duration
    
    off_periods = []
    on_periods = []
    
    start_time = time[0]
    for i in range(n_pairs):
        base = start_time + i * cycle_total
        
        # OFF period
        off_start = base
        off_end = base + off_duration
        off_periods.append((off_start, off_end))
        
        # ON period (after OFF + settling)
        on_start = base + off_duration + settling_duration
        on_end = on_start + on_duration
        on_periods.append((on_start, on_end))
    
    print(f"  Expected {n_pairs} OFF/ON pairs")
    print(f"  OFF: {off_duration}s, ON: {on_duration}s, settling: {settling_duration}s")
    
    # Compute FFT magnitude for each period
    off_fft_mags = []
    on_fft_mags = []
    
    for off_start, off_end in off_periods:
        mask = (time >= off_start) & (time < off_end)
        if np.any(mask):
            window = optic_flow_vx[mask]
            time_window = time[mask]
            
            # Compute FFT
            dt = np.mean(np.diff(time_window))
            freqs = np.fft.rfftfreq(len(window), dt)
            fft = np.abs(np.fft.rfft(window))
            
            # Find peak in flutter range (1-15 Hz)
            freq_mask = (freqs > 1.0) & (freqs < 15.0)
            peak_idx = np.argmax(fft[freq_mask])
            peak_mag = fft[freq_mask][peak_idx]
            peak_freq = freqs[freq_mask][peak_idx]
            
            off_fft_mags.append(peak_mag)
            print(f"    OFF: {off_start:.0f}-{off_end:.0f}s, FFT peak={peak_mag:.1f} @ {peak_freq:.1f}Hz")
    
    for on_start, on_end in on_periods:
        mask = (time >= on_start) & (time < on_end)
        if np.any(mask):
            window = optic_flow_vx[mask]
            time_window = time[mask]
            
            # Compute FFT
            dt = np.mean(np.diff(time_window))
            freqs = np.fft.rfftfreq(len(window), dt)
            fft = np.abs(np.fft.rfft(window))
            
            # Find peak in flutter range
            freq_mask = (freqs > 1.0) & (freqs < 15.0)
            peak_idx = np.argmax(fft[freq_mask])
            peak_mag = fft[freq_mask][peak_idx]
            peak_freq = freqs[freq_mask][peak_idx]
            
            on_fft_mags.append(peak_mag)
            print(f"    ON:  {on_start:.0f}-{on_end:.0f}s, FFT peak={peak_mag:.1f} @ {peak_freq:.1f}Hz")
    
    off_fft_mags = np.array(off_fft_mags)
    on_fft_mags = np.array(on_fft_mags)
    
    if len(off_fft_mags) == 0 or len(on_fft_mags) == 0:
        print("  ERROR: No data in OFF or ON periods!")
        return None
    
    # Calculate overall averages
    off_avg = np.mean(off_fft_mags)
    on_avg = np.mean(on_fft_mags)
    
    print(f"\n  OFF avg FFT magnitude: {off_avg:.2f} ± {np.std(off_fft_mags):.2f}")
    print(f"  ON avg FFT magnitude: {on_avg:.2f} ± {np.std(on_fft_mags):.2f}")
    
    # Calculate reduction
    mag_reduction = ((off_avg - on_avg) / off_avg) * 100
    print(f"  Magnitude reduction: {mag_reduction:.1f}%")
    
    # Create summary with individual cycles
    off_cycles = []
    on_cycles = []
    
    for off_mag, on_mag in zip(off_fft_mags, on_fft_mags):
        off_cycles.append({
            'avg_magnitude': float(off_mag),
            'peak_magnitude': float(off_mag),  # Same as avg for FFT peak
            'flutter_time': 0,
            'flutter_percent': 0
        })
        on_cycles.append({
            'avg_magnitude': float(on_mag),
            'peak_magnitude': float(on_mag),
            'flutter_time': 0,
            'flutter_percent': 0
        })
    
    summary = {
        'source_file': json_file.name,
        'target_frequency': 5.8,
        'phase_offset': 90.0,
        'control_off': {
            'cycles': off_cycles,
            'avg_magnitude': off_avg,
            'flutter_percent': 0
        },
        'control_on': {
            'cycles': on_cycles,
            'avg_magnitude': on_avg,
            'flutter_percent': 0
        },
        'effectiveness': {
            'magnitude_reduction_percent': mag_reduction,
            'flutter_time_reduction_percent': 0
        }
    }
    
    return summary

def main():
    phase_90_folder = Path.cwd() / 'phase_90'
    
    print("="*70)
    print("ANALYZING PHASE_90 FILES")
    print("="*70)
    print(f"Looking in: {phase_90_folder}\n")
    
    if not phase_90_folder.exists():
        print(f"ERROR: Folder doesn't exist!")
        return
    
    # Find all phase_90deg files
    json_files = list(phase_90_folder.glob('phase_90deg*.json'))
    
    print(f"Found {len(json_files)} files:\n")
    for f in json_files:
        print(f"  - {f.name}")
    
    print()
    
    for json_file in json_files:
        try:
            summary = analyze_phase_90_file(json_file)
            
            # Save summary
            timestamp = json_file.stem.split('_')[-1]
            output_name = f"off_frequency_test_5.8Hz_{timestamp}.json"
            output_path = phase_90_folder / output_name
            
            with open(output_path, 'w') as f:
                json.dump(summary, f, indent=2)
            
            print(f"  ✓ Saved: {output_name}\n")
            
        except Exception as e:
            print(f"  ✗ ERROR: {e}\n")
    
    print("="*70)
    print("Done!")
    print("="*70)

if __name__ == '__main__':
    main()