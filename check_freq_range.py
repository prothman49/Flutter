#!/usr/bin/env python3
"""
Check the actual dominant frequency range from matched frequency tests
"""

import json
import numpy as np
from pathlib import Path

def main():
    phase_90_folder = Path.cwd() / 'phase_90'
    
    print("="*70)
    print("CHECKING DOMINANT FREQUENCY RANGE")
    print("="*70)
    
    all_frequencies = []
    
    # Check Feb 24 files (they have avg_frequency in pairs)
    feb_24_files = [
        'flutter_on_off_phase090_20260224_121137.json',
        'flutter_on_off_phase090_20260224_140358.json'
    ]
    
    print("\nFeb 24 files (from pairs data):")
    for filename in feb_24_files:
        filepath = phase_90_folder / filename
        if filepath.exists():
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            if 'pairs' in data:
                for pair in data['pairs']:
                    if 'off' in pair and 'avg_frequency' in pair['off']:
                        freq = pair['off']['avg_frequency']
                        all_frequencies.append(freq)
                        print(f"  {filename}: OFF = {freq:.2f} Hz")
                    if 'on' in pair and 'avg_frequency' in pair['on']:
                        freq = pair['on']['avg_frequency']
                        all_frequencies.append(freq)
                        print(f"  {filename}: ON  = {freq:.2f} Hz")
    
    # For the bag file, we need to re-analyze with frequency extraction
    print("\nBag-converted file (phase_90deg_matched_freq_20260306_161150.json):")
    bag_file = phase_90_folder / 'phase_90deg_matched_freq_20260306_161150.json'
    
    if bag_file.exists():
        with open(bag_file, 'r') as f:
            data = json.load(f)
        
        time = np.array(data['time'])
        optic_flow_vx = np.array(data['optic_flow_vx'])
        
        # Timing structure
        off_duration = 120
        settling_duration = 60
        cycle_total = 360
        n_pairs = 5
        start_time = time[0]
        
        for i in range(n_pairs):
            base = start_time + i * cycle_total
            
            # OFF period
            off_start = base
            off_end = base + off_duration
            mask = (time >= off_start) & (time < off_end)
            
            if np.any(mask):
                window = optic_flow_vx[mask]
                time_window = time[mask]
                dt = np.mean(np.diff(time_window))
                freqs = np.fft.rfftfreq(len(window), dt)
                fft = np.abs(np.fft.rfft(window))
                
                freq_mask = (freqs > 1.0) & (freqs < 15.0)
                peak_idx = np.argmax(fft[freq_mask])
                peak_freq = freqs[freq_mask][peak_idx]
                all_frequencies.append(peak_freq)
                print(f"  Pair {i+1} OFF: {peak_freq:.2f} Hz")
            
            # ON period
            on_start = base + off_duration + settling_duration
            on_end = on_start + 120
            mask = (time >= on_start) & (time < on_end)
            
            if np.any(mask):
                window = optic_flow_vx[mask]
                time_window = time[mask]
                dt = np.mean(np.diff(time_window))
                freqs = np.fft.rfftfreq(len(window), dt)
                fft = np.abs(np.fft.rfft(window))
                
                freq_mask = (freqs > 1.0) & (freqs < 15.0)
                peak_idx = np.argmax(fft[freq_mask])
                peak_freq = freqs[freq_mask][peak_idx]
                all_frequencies.append(peak_freq)
                print(f"  Pair {i+1} ON:  {peak_freq:.2f} Hz")
    
    # Summary
    all_frequencies = np.array(all_frequencies)
    
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    print(f"All frequencies: {len(all_frequencies)} measurements")
    print(f"Range: {np.min(all_frequencies):.2f} - {np.max(all_frequencies):.2f} Hz")
    print(f"Mean:  {np.mean(all_frequencies):.2f} ± {np.std(all_frequencies):.2f} Hz")
    print(f"Median: {np.median(all_frequencies):.2f} Hz")
    print("="*70)
    
    # Suggest label
    min_freq = np.min(all_frequencies)
    max_freq = np.max(all_frequencies)
    mean_freq = np.mean(all_frequencies)
    
    print(f"\nSuggested label: '{min_freq:.1f}-{max_freq:.1f} Hz (Matched)'")
    print(f"Or: '~{mean_freq:.1f} Hz (Matched)'")

if __name__ == '__main__':
    main()