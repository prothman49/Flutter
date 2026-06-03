#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Analyze ROS bag file for flutter frequency information
Extracts frequency data to understand if flutter frequency is consistent
"""

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

def analyze_bag_frequency(bag_file):
    """Extract frequency data from bag file"""
    
    print(f"\nAnalyzing: {bag_file}")
    print("="*70)
    
    try:
        bag = rosbag.Bag(bag_file)
        
        # Get bag info
        info = bag.get_type_and_topic_info()
        print("\nAvailable topics:")
        for topic in info.topics.keys():
            print(f"  {topic}")
        
        # Extract frequency data
        frequencies = []
        timestamps = []
        magnitudes = []
        
        freq_topic = '/optic_flow/frequency'
        mag_topic = '/optic_flow/magnitude'
        
        print(f"\nExtracting data from:")
        print(f"  - {freq_topic}")
        print(f"  - {mag_topic}")
        
        for topic, msg, t in bag.read_messages(topics=[freq_topic, mag_topic]):
            if topic == freq_topic:
                frequencies.append(msg.data)
                timestamps.append(t.to_sec())
            elif topic == mag_topic:
                magnitudes.append(msg.data)
        
        bag.close()
        
        # Analyze
        if len(frequencies) == 0:
            print("\n❌ No frequency data found in bag file!")
            print("Available topics:")
            for topic in info.topics.keys():
                print(f"  - {topic}")
            return
        
        freqs = np.array(frequencies)
        times = np.array(timestamps)
        times = times - times[0]  # Normalize to start at 0
        
        # Filter out zeros and outliers
        valid_mask = (freqs > 0) & (freqs < 20)  # Reasonable flutter range
        freqs_valid = freqs[valid_mask]
        times_valid = times[valid_mask]
        
        print(f"\n✓ Extracted {len(frequencies)} frequency measurements")
        print(f"✓ Valid measurements (0-20 Hz): {len(freqs_valid)}")
        
        if len(freqs_valid) > 0:
            print("\n" + "="*70)
            print("FREQUENCY STATISTICS")
            print("="*70)
            print(f"Mean frequency:    {np.mean(freqs_valid):.3f} Hz")
            print(f"Std deviation:     {np.std(freqs_valid):.3f} Hz")
            print(f"Min frequency:     {np.min(freqs_valid):.3f} Hz")
            print(f"Max frequency:     {np.max(freqs_valid):.3f} Hz")
            print(f"Frequency range:   {np.max(freqs_valid) - np.min(freqs_valid):.3f} Hz")
            print(f"Coeff of variation: {(np.std(freqs_valid) / np.mean(freqs_valid)) * 100:.2f}%")
            
            # Check if in target range
            mean_freq = np.mean(freqs_valid)
            if 5.0 <= mean_freq <= 5.8:
                print(f"\n✓ Mean frequency IS in target range (5.0-5.8 Hz)")
            else:
                print(f"\n⚠️  Mean frequency NOT in target range (5.0-5.8 Hz)")
            
            # Create plots
            fig, axes = plt.subplots(2, 2, figsize=(15, 10))
            
            # Plot 1: Frequency vs time
            ax1 = axes[0, 0]
            ax1.plot(times_valid, freqs_valid, 'b-', alpha=0.5, linewidth=0.5)
            ax1.axhline(np.mean(freqs_valid), color='r', linestyle='--', label=f'Mean: {np.mean(freqs_valid):.2f} Hz')
            ax1.axhline(5.0, color='g', linestyle=':', alpha=0.5, label='Target range')
            ax1.axhline(5.8, color='g', linestyle=':', alpha=0.5)
            ax1.fill_between([times_valid.min(), times_valid.max()], 5.0, 5.8, alpha=0.1, color='green')
            ax1.set_xlabel('Time (s)', fontweight='bold')
            ax1.set_ylabel('Frequency (Hz)', fontweight='bold')
            ax1.set_title('Flutter Frequency Over Time', fontweight='bold')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # Plot 2: Frequency histogram
            ax2 = axes[0, 1]
            ax2.hist(freqs_valid, bins=50, color='steelblue', edgecolor='black', alpha=0.7)
            ax2.axvline(np.mean(freqs_valid), color='r', linestyle='--', linewidth=2, label=f'Mean: {np.mean(freqs_valid):.2f} Hz')
            ax2.axvline(5.0, color='g', linestyle=':', linewidth=2, alpha=0.7, label='Target: 5.0-5.8 Hz')
            ax2.axvline(5.8, color='g', linestyle=':', linewidth=2, alpha=0.7)
            ax2.set_xlabel('Frequency (Hz)', fontweight='bold')
            ax2.set_ylabel('Count', fontweight='bold')
            ax2.set_title('Frequency Distribution', fontweight='bold')
            ax2.legend()
            ax2.grid(True, alpha=0.3, axis='y')
            
            # Plot 3: Magnitude vs time (if available)
            if len(magnitudes) > 0:
                ax3 = axes[1, 0]
                mags = np.array(magnitudes)
                mag_times = np.linspace(0, times[-1], len(mags))
                ax3.plot(mag_times, mags, 'r-', alpha=0.5, linewidth=0.5)
                ax3.axhline(105, color='orange', linestyle='--', label='Threshold: 105')
                ax3.set_xlabel('Time (s)', fontweight='bold')
                ax3.set_ylabel('Magnitude', fontweight='bold')
                ax3.set_title('Flutter Magnitude Over Time', fontweight='bold')
                ax3.legend()
                ax3.grid(True, alpha=0.3)
            
            # Plot 4: Statistics summary
            ax4 = axes[1, 1]
            ax4.axis('off')
            
            stats_text = f"""
FREQUENCY ANALYSIS SUMMARY
{'='*45}

Total measurements:     {len(frequencies)}
Valid measurements:     {len(freqs_valid)}

FREQUENCY STATISTICS:
  Mean:                 {np.mean(freqs_valid):.3f} Hz
  Std Dev:              {np.std(freqs_valid):.3f} Hz
  Min:                  {np.min(freqs_valid):.3f} Hz
  Max:                  {np.max(freqs_valid):.3f} Hz
  Range:                {np.max(freqs_valid) - np.min(freqs_valid):.3f} Hz
  CV:                   {(np.std(freqs_valid) / np.mean(freqs_valid)) * 100:.2f}%

TARGET RANGE CHECK:
  Target:               5.0 - 5.8 Hz
  Actual:               {np.mean(freqs_valid):.3f} ± {np.std(freqs_valid):.3f} Hz
  In range:             {'YES ✓' if 5.0 <= np.mean(freqs_valid) <= 5.8 else 'NO ✗'}

INTERPRETATION:
  CV < 5%:              Excellent consistency
  CV 5-10%:             Good consistency
  CV > 10%:             Variable frequency
  
  Your CV:              {(np.std(freqs_valid) / np.mean(freqs_valid)) * 100:.2f}%
            """
            
            ax4.text(0.05, 0.95, stats_text, transform=ax4.transAxes,
                    fontsize=10, verticalalignment='top', family='monospace',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
            
            plt.suptitle(f'Flutter Frequency Analysis\n{Path(bag_file).name}', 
                        fontsize=14, fontweight='bold')
            plt.tight_layout()
            
            # Save figure
            output_name = Path(bag_file).stem + '_frequency_analysis.png'
            plt.savefig(output_name, dpi=300, bbox_inches='tight')
            print(f"\n✓ Analysis plot saved: {output_name}")
            
            plt.show()
            
        else:
            print("\n❌ No valid frequency data found!")
        
    except Exception as e:
        print(f"\n❌ Error analyzing bag file: {e}")
        import traceback
        traceback.print_exc()

def main():
    if len(sys.argv) < 2:
        print("\nUsage: python3 analyze_bag_frequency.py <path_to_bag_file>")
        print("\nExample:")
        print("  python3 analyze_bag_frequency.py control_time_trace_fresh__2026-01-28-16-09-01.bag")
        return
    
    bag_file = sys.argv[1]
    
    if not Path(bag_file).exists():
        print(f"\n❌ File not found: {bag_file}")
        return
    
    analyze_bag_frequency(bag_file)

if __name__ == '__main__':
    main()
