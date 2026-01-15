#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Quick Start Menu for Flutter Control Tests
"""

import subprocess
import sys
import os

def print_menu():
    print("\n" + "="*60)
    print("FLUTTER CONTROL - QUICK START MENU")
    print("="*60)
    print("\n1. Start System (Camera + Optic Flow)")
    print("2. Live Control (GUI)")
    print("3. Test: Structural Phase Characterization")
    print("4. Test: Flutter Phase Sweep")
    print("5. Test: Control Effectiveness Validation")
    print("6. Kill All ROS Nodes")
    print("0. Exit")
    print("\n" + "="*60)

def run_command(cmd, description):
    print(f"\n[Running] {description}")
    print(f"Command: {' '.join(cmd)}\n")
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"\nError: {e}")
        input("Press Enter to continue...")
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    while True:
        print_menu()
        choice = input("\nSelect option: ").strip()
        
        if choice == '0':
            print("Exiting...")
            break
        
        elif choice == '1':
            run_command(
                ['roslaunch', 'flutter_control', 'camera_only.launch'],
                "Starting Camera + Optic Flow"
            )
        
        elif choice == '2':
            run_command(
                ['python3', os.path.join(script_dir, 'run_live_control.py')],
                "Starting Live Control"
            )
        
        elif choice == '3':
            run_command(
                ['python3', os.path.join(script_dir, 'test_structural_phase.py')],
                "Structural Phase Test"
            )
        
        elif choice == '4':
            run_command(
                ['python3', os.path.join(script_dir, 'test_flutter_phase_sweep.py')],
                "Flutter Phase Sweep Test"
            )
        
        elif choice == '5':
            run_command(
                ['python3', os.path.join(script_dir, 'test_control_effectiveness.py')],
                "Control Effectiveness Test"
            )
        
        elif choice == '6':
            run_command(
                ['rosnode', 'kill', '-a'],
                "Killing all ROS nodes"
            )
        
        else:
            print("Invalid option. Please try again.")

if __name__ == '__main__':
    main()