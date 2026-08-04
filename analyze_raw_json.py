#!/usr/bin/env python3
"""
Analyze raw bag-to-JSON files from off-frequency tests
Detects ON/OFF periods and calculates reduction statistics
"""

import json
import numpy as np
from pathlib import Path

def detect_control_periods(voltages_data, time_array):
    """
    Detect when control is ON vs OFF based on piezo voltages
    Returns: list of (start_time, end_time, state) tuples
    """
    # Extract voltage magnitudes
    control_active = []
    times = []
    
    for entry in voltages_data:
        v = entry['voltages']
        t = entry['time']
        # Control is ON if either voltage is non-zero
        is_on = (v[0] > 1.0) or (v[1] > 1.0)
        control_active.append(is_on)
        times.append(t)
    
    # Find transitions
    control_active = np.array(control_active)
    times = np.array(times)
    
    # Detect state changes
    transitions = np.where(np.diff(control_active.astype(int)) != 0)[0]
    
    periods = []
    current_start = times[0]
    current_state = control_active[0]
    
    for trans_idx in transitions:
        periods.append((current_start, times[trans_idx], current_state))
        current_start = times[trans_idx + 1]
        current_state = control_active[trans_idx + 1]
    
    # Add final period
    periods.append((current_start, times[-1], current_state))
    
    return periods

def analyze_period(time, amplitude, t_start, t_end, flutter_threshold=105):
    """Analyze a single ON or OFF period"""
    mask = (time >= t_start) & (time <= t_end)
    
    if not np.any(mask):
        return None
    
    amp_window = amplitude[mask]
    time_window = time[mask]
    duration = t_end - t_start
    
    avg_magnitude = np.mean(amp_window)
    peak_magnitude = np.max(amp_window)
    
    # Flutter time: time spent above threshold
    flutter_mask = amp_window > flutter_threshold
    flutter_time = np.sum(flutter_mask) * np.mean(np.diff(time_window)) if len(time_window) > 1 else 0
    flutter_percent = (flutter_time / duration) * 100 if duration > 0 else 0
    
    return {
        'avg_magnitude': avg_magnitude,
        'peak_magnitude': peak_magnitude,
        'flutter_time': flutter_time,
        'flutter_percent': flutter_percent,
        'duration': duration
    }

def analyze_test_file(json_file, min_period_duration=60):
    """
    Analyze a single test file
    Handles two formats:
    1. Raw bag conversion (piezo voltages)
    2. test_flutter_on_off.py output (phase_labels)
    """
    print(f"\nAnalyzing: {json_file.name}")
    
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    # Check format
    if 'phase_labels' in data:
        # Format from test_flutter_on_off.py
        return analyze_phase_labels_format(json_file, data)
    else:
        # Raw bag conversion format
        return analyze_raw_format(json_file, data, min_period_duration)

def analyze_phase_labels_format(json_file, data):
    """Analyze files with phase_labels (from test_flutter_on_off.py)"""
    print(f"  Format: phase_labels (test_flutter_on_off.py)")
    
    time = np.array(data.get('timestamps', data.get('time', [])))
    amplitude = np.array(data.get('magnitudes', data.get('amplitude_envelope', [])))
    phase_labels = np.array(data['phase_labels'])
    
    # Labels: typically 1=OFF, 2=ON, 3=OFF_return
    # Combine 1 and 3 as OFF periods
    off_mask = (phase_labels == 1) | (phase_labels == 3)
    on_mask = (phase_labels == 2)
    
    print(f"  OFF samples: {np.sum(off_mask)}")
    print(f"  ON samples: {np.sum(on_mask)}")
    
    # Analyze OFF period
    off_amp = amplitude[off_mask]
    off_result = {
        'avg_magnitude': np.mean(off_amp),
        'peak_magnitude': np.max(off_amp),
        'flutter_time': 0,  # Not calculated from this format
        'flutter_percent': 0
    }
    
    # Analyze ON period
    on_amp = amplitude[on_mask]
    on_result = {
        'avg_magnitude': np.mean(on_amp),
        'peak_magnitude': np.max(on_amp),
        'flutter_time': 0,
        'flutter_percent': 0
    }
    
    # Calculate effectiveness
    mag_reduction = ((off_result['avg_magnitude'] - on_result['avg_magnitude']) / 
                     off_result['avg_magnitude']) * 100
    
    print(f"  OFF avg: {off_result['avg_magnitude']:.1f}")
    print(f"  ON avg: {on_result['avg_magnitude']:.1f}")
    print(f"  Magnitude reduction: {mag_reduction:.1f}%")
    
    # Extract frequency
    target_freq = data.get('phase_offset', None)
    if target_freq == 90.0:
        target_freq = 5.8  # 90° is matched frequency
    
    return {
        'source_file': json_file.name,
        'target_frequency': target_freq,
        'phase_offset': data.get('phase_offset', 90.0),
        'control_off': {
            'cycles': [off_result],
            'avg_magnitude': off_result['avg_magnitude'],
            'flutter_percent': 0
        },
        'control_on': {
            'cycles': [on_result],
            'avg_magnitude': on_result['avg_magnitude'],
            'flutter_percent': 0
        },
        'effectiveness': {
            'magnitude_reduction_percent': mag_reduction,
            'flutter_time_reduction_percent': 0
        }
    }

def analyze_raw_format(json_file, data, min_period_duration):
    """Analyze raw bag-to-JSON format (original function)"""
    print(f"  Format: raw bag conversion")
    
    time = np.array(data['time'])
    amplitude = np.array(data['amplitude_envelope'])
    
    # Detect control periods
    periods = detect_control_periods(data['piezo_voltages'], time)
    
    print(f"  Detected {len(periods)} periods")
    
    # Filter out very short periods (transitions/noise)
    periods = [(t0, t1, state) for t0, t1, state in periods if (t1 - t0) > min_period_duration]
    
    print(f"  {len(periods)} periods > {min_period_duration}s")
    
    # Separate ON and OFF
    off_periods = [p for p in periods if not p[2]]
    on_periods = [p for p in periods if p[2]]
    
    print(f"  OFF periods: {len(off_periods)}")
    print(f"  ON periods: {len(on_periods)}")
    
    # Analyze each period
    off_cycles = []
    for t0, t1, _ in off_periods:
        result = analyze_period(time, amplitude, t0, t1)
        if result:
            off_cycles.append(result)
            print(f"    OFF: {t0:.1f}-{t1:.1f}s, avg={result['avg_magnitude']:.1f}")
    
    on_cycles = []
    for t0, t1, _ in on_periods:
        result = analyze_period(time, amplitude, t0, t1)
        if result:
            on_cycles.append(result)
            print(f"    ON:  {t0:.1f}-{t1:.1f}s, avg={result['avg_magnitude']:.1f}")
    
    if not off_cycles or not on_cycles:
        print("  ERROR: Missing ON or OFF periods!")
        return None
    
    # Calculate averages
    off_avg_mag = np.mean([c['avg_magnitude'] for c in off_cycles])
    on_avg_mag = np.mean([c['avg_magnitude'] for c in on_cycles])
    
    off_flutter_pct = np.mean([c['flutter_percent'] for c in off_cycles])
    on_flutter_pct = np.mean([c['flutter_percent'] for c in on_cycles])
    
    # Calculate effectiveness
    mag_reduction = ((off_avg_mag - on_avg_mag) / off_avg_mag) * 100
    flutter_reduction = ((off_flutter_pct - on_flutter_pct) / off_flutter_pct) * 100 if off_flutter_pct > 0 else 0
    
    print(f"  Magnitude reduction: {mag_reduction:.1f}%")
    print(f"  Flutter reduction: {flutter_reduction:.1f}%")
    
    # Extract frequency from metadata or filename
    target_freq = None
    if 'target_frequency' in data:
        target_freq = data['target_frequency']
    elif 'metadata' in data:
        # Try to extract from filename in metadata
        bag_name = data['metadata'].get('bag_file', json_file.stem)
        if '3hz' in bag_name.lower():
            target_freq = 3.0
        elif '8hz' in bag_name.lower():
            target_freq = 8.0
        elif '10hz' in bag_name.lower():
            target_freq = 10.0
        elif 'phase_90' in bag_name.lower():
            target_freq = 5.8
    
    return {
        'source_file': json_file.name,
        'target_frequency': target_freq,
        'phase_offset': 120.0,  # Default - update if different
        'control_off': {
            'cycles': off_cycles,
            'avg_magnitude': off_avg_mag,
            'flutter_percent': off_flutter_pct
        },
        'control_on': {
            'cycles': on_cycles,
            'avg_magnitude': on_avg_mag,
            'flutter_percent': on_flutter_pct
        },
        'effectiveness': {
            'magnitude_reduction_percent': mag_reduction,
            'flutter_time_reduction_percent': flutter_reduction
        }
    }

def main():
    base_path = Path.home() / 'Desktop' / 'Flutter_analysis' / 'off_frequency_tests'
    
    folders = ['0hz', '3hz', '8hz', '10hz', 'phase_90']
    
    print("="*70)
    print("ANALYZING RAW BAG-TO-JSON FILES")
    print("="*70)
    
    for folder_name in folders:
        folder = base_path / folder_name
        if not folder.exists():
            continue
        
        print(f"\n{'='*70}")
        print(f"Processing: {folder_name}")
        print(f"{'='*70}")
        
        # Find files to process
        if folder_name == 'phase_90':
            # Look for phase_90deg files
            pattern = 'phase_90deg*.json'
        else:
            # Look for run files
            pattern = '*run*.json'
        
        for json_file in sorted(folder.glob(pattern)):
            # Skip already-processed summary files
            if 'off_frequency_test' in json_file.name or 'summary' in json_file.name:
                continue
            
            result = analyze_test_file(json_file)
            
            if result:
                # Save summary file
                freq_str = f"{result['target_frequency']:.1f}" if result['target_frequency'] else "unknown"
                output_name = f"off_frequency_test_{freq_str}Hz_{json_file.stem.split('__')[-1]}.json"
                output_path = folder / output_name
                
                with open(output_path, 'w') as f:
                    json.dump(result, f, indent=2)
                
                print(f"  Saved: {output_name}")

    print("\n" + "="*70)
    print("Analysis complete!")
    print("="*70)

if __name__ == '__main__':
    main()