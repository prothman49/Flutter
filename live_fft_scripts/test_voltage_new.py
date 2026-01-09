#!/usr/bin/env python3

import time
import math
from Phidget22.Devices.VoltageOutput import VoltageOutput
from Phidget22.PhidgetException import PhidgetException

def test_piezos():
    """Simple test script for piezo voltage outputs"""
    
    try:
        # Initialize voltage outputs
        voltage_output_0 = VoltageOutput()
        voltage_output_1 = VoltageOutput()

        voltage_output_0.setChannel(0)
        voltage_output_0.setHubPort(0)

        voltage_output_1.setChannel(0)
        voltage_output_1.setHubPort(1)

        voltage_output_0.openWaitForAttachment(5000)
        voltage_output_1.openWaitForAttachment(5000)

        print("[INFO] Both Voltage Outputs attached successfully!")
        print("\n" + "="*50)
        print("PIEZO TEST SEQUENCE")
        print("="*50 + "\n")

        # Test 1: Set both to mid-level voltage
        print("Test 1: Setting both piezos to 2.0V for 3 seconds...")
        voltage_output_0.setVoltage(2.0)
        voltage_output_1.setVoltage(2.0)
        time.sleep(3)
        
        # Test 2: Test piezo 0 only
        print("\nTest 2: Testing Piezo 0 only (5Hz sine wave for 5 seconds)...")
        voltage_output_1.setVoltage(0.0)
        start_time = time.time()
        while time.time() - start_time < 5:
            t = time.time() - start_time
            voltage = 2.1 + 2.1 * math.sin(2 * math.pi * 5 * t)
            voltage_output_0.setVoltage(voltage)
            time.sleep(0.01)
        
        voltage_output_0.setVoltage(0.0)
        time.sleep(1)
        
        # Test 3: Test piezo 1 only
        print("\nTest 3: Testing Piezo 1 only (5Hz sine wave for 5 seconds)...")
        start_time = time.time()
        while time.time() - start_time < 5:
            t = time.time() - start_time
            voltage = 2.1 + 2.1 * math.sin(2 * math.pi * 5 * t)
            voltage_output_1.setVoltage(voltage)
            time.sleep(0.01)
        
        voltage_output_1.setVoltage(0.0)
        time.sleep(1)
        
        # Test 4: Both piezos in phase
        print("\nTest 4: Both piezos IN PHASE (5Hz for 5 seconds)...")
        start_time = time.time()
        while time.time() - start_time < 5:
            t = time.time() - start_time
            voltage = 2.1 + 2.1 * math.sin(2 * math.pi * 5 * t)
            voltage_output_0.setVoltage(voltage)
            voltage_output_1.setVoltage(voltage)
            time.sleep(0.01)
        
        time.sleep(1)
        
        # Test 5: Both piezos out of phase (inverse)
        print("\nTest 5: Both piezos OUT OF PHASE (inverse, 5Hz for 30 seconds)...")
        start_time = time.time()
        while time.time() - start_time < 30:
            t = time.time() - start_time
            voltage_0 = 2.1 + 2.1 * math.sin(2 * math.pi * 5 * t)
            voltage_1 = 2.1 + 2.1 * math.sin(2 * math.pi * 5 * t + math.pi)
            voltage_output_0.setVoltage(voltage_0)
            voltage_output_1.setVoltage(voltage_1)
            time.sleep(0.01)
        
        # Test 6: Frequency sweep
        print("\nTest 6: Frequency sweep from 5Hz to 10Hz (both piezos, 10 seconds)...")
        start_time = time.time()
        duration = 10.0
        while time.time() - start_time < duration:
            t = time.time() - start_time
            # Linear frequency sweep from 5 to 10 Hz
            freq = 5.0 + (5.0 * t / duration)
            voltage = 2.1 + 2.1 * math.sin(2 * math.pi * freq * t)
            voltage_output_0.setVoltage(voltage)
            voltage_output_1.setVoltage(voltage)
            print(f"  Frequency: {freq:.2f} Hz", end='\r')
            time.sleep(0.01)
        
        print("\n")
        
        # Return to zero
        print("\nTest complete! Returning to 0V...")
        voltage_output_0.setVoltage(0.0)
        voltage_output_1.setVoltage(0.0)
        
        print("\n" + "="*50)
        print("ALL TESTS COMPLETED SUCCESSFULLY!")
        print("="*50)
        
        # Close devices
        voltage_output_0.close()
        voltage_output_1.close()

    except PhidgetException as e:
        print(f"[ERROR] Phidget error: {e}")
        return False
    except KeyboardInterrupt:
        print("\n\n[INFO] Test interrupted by user")
        voltage_output_0.setVoltage(0.0)
        voltage_output_1.setVoltage(0.0)
        voltage_output_0.close()
        voltage_output_1.close()
        return False
    
    return True

if __name__ == '__main__':
    print("Starting piezo test sequence...")
    print("Watch/listen for piezo vibrations during each test.\n")
    success = test_piezos()
    if success:
        print("\nTest script finished successfully!")
    else:
        print("\nTest script encountered errors.")
