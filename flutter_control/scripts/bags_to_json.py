#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Convert rosbag files to JSON format for analysis on Mac
Extracts all relevant data from flutter control bags
"""

import rosbag
import json
import sys
from pathlib import Path
import numpy as np

def bag_to_json(bag_file, output_dir=None):
    """
    Convert a rosbag file to JSON format
    
    Args:
        bag_file: Path to .bag file
        output_dir: Directory to save JSON (defaults to same dir as bag)
    
    Returns:
        Path to created JSON file
    """
    
    bag_path = Path(bag_file)
    
    if not bag_path.exists():
        print(f"Error: Bag file not found: {bag_file}")
        return None
    
    print(f"\nProcessing: {bag_path.name}")
    
    # Determine output path
    if output_dir is None:
        output_dir = bag_path.parent
    else:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
    
    json_file = output_dir / (bag_path.stem + '.json')
    
    # Initialize data structure
    data = {
        'metadata': {
            'bag_file': str(bag_path.name),
            'duration_seconds': 0,
            'sample_count': 0
        },
        'time': [],
        'optic_flow_vx': [],  # Mean optical flow
        'amplitude_envelope': [],  # Flutter amplitude
        'phase_mod': [],  # Phase modulation
        'piezo_voltages': []  # Piezo voltage commands
    }
    
    try:
        bag = rosbag.Bag(str(bag_path))
        
        # Get bag info
        info = bag.get_type_and_topic_info()
        print(f"  Topics: {list(info.topics.keys())}")
        
        start_time = None
        end_time = None
        
        # Extract optical flow data (primary signal)
        print("  Extracting optical flow data...")
        for topic, msg, t in bag.read_messages(topics=['/optic_flow_mean_vx']):
            if start_time is None:
                start_time = t.to_sec()
            end_time = t.to_sec()
            
            rel_time = t.to_sec() - start_time
            data['time'].append(rel_time)
            data['optic_flow_vx'].append(float(msg.data))
        
        print(f"  ✓ {len(data['optic_flow_vx'])} optical flow samples")
        
        # Extract amplitude envelope (flutter magnitude)
        print("  Extracting amplitude envelope...")
        amplitude_times = []
        amplitude_values = []
        for topic, msg, t in bag.read_messages(topics=['/amplitude_envelope']):
            if start_time is None:
                continue
            rel_time = t.to_sec() - start_time
            amplitude_times.append(rel_time)
            amplitude_values.append(float(msg.data))
        
        # Interpolate amplitude to match time vector
        if amplitude_times:
            data['amplitude_envelope'] = list(np.interp(data['time'], amplitude_times, amplitude_values))
            print(f"  ✓ {len(amplitude_times)} amplitude samples (interpolated to match)")
        
        # Extract phase modulation
        print("  Extracting phase modulation...")
        phase_times = []
        phase_values = []
        for topic, msg, t in bag.read_messages(topics=['/phase_mod']):
            if start_time is None:
                continue
            rel_time = t.to_sec() - start_time
            phase_times.append(rel_time)
            phase_values.append(float(msg.data))
        
        # Interpolate phase to match time vector
        if phase_times:
            data['phase_mod'] = list(np.interp(data['time'], phase_times, phase_values))
            print(f"  ✓ {len(phase_times)} phase samples (interpolated to match)")
        
        # Extract piezo voltages
        print("  Extracting piezo voltages...")
        for topic, msg, t in bag.read_messages(topics=['/piezo_voltages']):
            if start_time is None:
                continue
            rel_time = t.to_sec() - start_time
            data['piezo_voltages'].append({
                'time': rel_time,
                'voltages': [float(v) for v in msg.data]
            })
        
        bag.close()
        
        # Update metadata
        if start_time and end_time:
            data['metadata']['duration_seconds'] = end_time - start_time
        data['metadata']['sample_count'] = len(data['optic_flow_vx'])
        
        # Calculate statistics
        if data['optic_flow_vx']:
            data['metadata']['sample_rate_hz'] = len(data['optic_flow_vx']) / data['metadata']['duration_seconds'] if data['metadata']['duration_seconds'] > 0 else 0
            data['metadata']['optic_flow_stats'] = {
                'mean': float(np.mean(data['optic_flow_vx'])),
                'std': float(np.std(data['optic_flow_vx'])),
                'min': float(np.min(data['optic_flow_vx'])),
                'max': float(np.max(data['optic_flow_vx']))
            }
        
        if data['amplitude_envelope']:
            data['metadata']['amplitude_stats'] = {
                'mean': float(np.mean(data['amplitude_envelope'])),
                'std': float(np.std(data['amplitude_envelope'])),
                'min': float(np.min(data['amplitude_envelope'])),
                'max': float(np.max(data['amplitude_envelope']))
            }
        
        if data['phase_mod']:
            data['metadata']['phase_stats'] = {
                'mean': float(np.mean(data['phase_mod'])),
                'std': float(np.std(data['phase_mod'])),
                'min': float(np.min(data['phase_mod'])),
                'max': float(np.max(data['phase_mod']))
            }
        
        if data['piezo_voltages']:
            data['metadata']['piezo_message_count'] = len(data['piezo_voltages'])

        
        # Save JSON
        with open(json_file, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"\n✓ Saved: {json_file.name}")
        print(f"  Duration: {data['metadata']['duration_seconds']:.1f}s")
        print(f"  Samples: {data['metadata']['sample_count']}")
        if 'sample_rate_hz' in data['metadata']:
            print(f"  Rate: {data['metadata']['sample_rate_hz']:.1f} Hz")

        
        return json_file
        
    except Exception as e:
        print(f"  ✗ Error processing bag: {e}")
        return None


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Convert rosbag files to JSON')
    parser.add_argument('input', help='Bag file or directory containing bags')
    parser.add_argument('-o', '--output', help='Output directory for JSON files', default=None)
    
    args = parser.parse_args()
    
    input_path = Path(args.input)
    
    print("="*70)
    print("ROSBAG TO JSON CONVERTER")
    print("="*70)
    
    # Process single file or directory
    if input_path.is_file():
        bag_to_json(input_path, args.output)
        
    elif input_path.is_dir():
        # Find all bag files
        bag_files = sorted(input_path.glob('*.bag'))
        
        if not bag_files:
            print(f"\nNo .bag files found in {input_path}")
            sys.exit(1)
        
        print(f"\nFound {len(bag_files)} bag file(s)")
        
        success_count = 0
        for bag_file in bag_files:
            result = bag_to_json(bag_file, args.output)
            if result:
                success_count += 1
        
        print("\n" + "="*70)
        print(f"Converted {success_count}/{len(bag_files)} bag files successfully")
        print("="*70)
        
    else:
        print(f"Error: {input_path} is neither a file nor directory")
        sys.exit(1)


if __name__ == '__main__':
    main()