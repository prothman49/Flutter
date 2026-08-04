#!/usr/bin/env python3
"""
Convert Feb 24 'pairs' format to standard 'cycles' format
"""

import json
from pathlib import Path

def convert_pairs_to_cycles(input_file, output_file):
    """Convert pairs format to control_off/control_on cycles format"""
    
    with open(input_file, 'r') as f:
        data = json.load(f)
    
    if 'pairs' not in data:
        print(f"  Skipping {input_file.name} - already in correct format")
        return
    
    # Extract cycles from pairs
    off_cycles = []
    on_cycles = []
    
    for pair in data['pairs']:
        off_data = pair['off']
        on_data = pair['on']
        
        off_cycles.append({
            'avg_magnitude': off_data['avg_magnitude'],
            'peak_magnitude': off_data['peak_magnitude'],
            'flutter_time': off_data.get('flutter_count', 0) * 0.1,  # Approximate
            'flutter_percent': off_data.get('flutter_percent', 0)
        })
        
        on_cycles.append({
            'avg_magnitude': on_data['avg_magnitude'],
            'peak_magnitude': on_data['peak_magnitude'],
            'flutter_time': on_data.get('flutter_count', 0) * 0.1,
            'flutter_percent': on_data.get('flutter_percent', 0)
        })
    
    # Calculate averages
    import numpy as np
    off_avg = np.mean([c['avg_magnitude'] for c in off_cycles])
    on_avg = np.mean([c['avg_magnitude'] for c in on_cycles])
    
    # Create new format
    converted = {
        'target_frequency': 5.8,
        'phase_offset': data['test_info']['phase_deg'],
        'control_off': {
            'cycles': off_cycles,
            'avg_magnitude': off_avg,
            'flutter_percent': np.mean([c['flutter_percent'] for c in off_cycles])
        },
        'control_on': {
            'cycles': on_cycles,
            'avg_magnitude': on_avg,
            'flutter_percent': np.mean([c['flutter_percent'] for c in on_cycles])
        },
        'effectiveness': {
            'magnitude_reduction_percent': ((off_avg - on_avg) / off_avg) * 100,
            'flutter_time_reduction_percent': 0
        }
    }
    
    # Save
    with open(output_file, 'w') as f:
        json.dump(converted, f, indent=2)
    
    print(f"  ✓ Converted {input_file.name} → {output_file.name}")

def main():
    phase_90_folder = Path.cwd() / 'phase_90'
    
    print("="*70)
    print("CONVERTING FEB 24 FILES TO STANDARD FORMAT")
    print("="*70)
    
    files_to_convert = [
        'off_frequency_test_5.8Hz_121137.json',
        'off_frequency_test_5.8Hz_140358.json'
    ]
    
    for filename in files_to_convert:
        input_path = phase_90_folder / filename
        if input_path.exists():
            # Backup original
            backup_path = phase_90_folder / (filename.replace('.json', '_original.json'))
            import shutil
            shutil.copy(input_path, backup_path)
            print(f"\n  Backed up: {backup_path.name}")
            
            # Convert
            convert_pairs_to_cycles(input_path, input_path)
        else:
            print(f"\n  Not found: {filename}")
    
    print("\n" + "="*70)
    print("Done!")
    print("="*70)

if __name__ == '__main__':
    main()