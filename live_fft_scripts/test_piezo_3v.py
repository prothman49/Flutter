#!/usr/bin/env python3

import time
from Phidget22.Devices.VoltageOutput import VoltageOutput
from Phidget22.PhidgetException import PhidgetException

def test_piezo_controller():
    """Send 3V to both channels for testing piezo controller"""
    
    voltage_output_0 = None
    voltage_output_1 = None
    
    try:
        print("[INFO] Initializing Phidget voltage outputs...")
        
        # Setup Channel 0 (Hub Port 0)
        voltage_output_0 = VoltageOutput()
        voltage_output_0.setChannel(0)
        voltage_output_0.setHubPort(0)
        
        # Setup Channel 1 (Hub Port 1) 
        voltage_output_1 = VoltageOutput()
        voltage_output_1.setChannel(0)
        voltage_output_1.setHubPort(1)
        
        print("[INFO] Opening connections...")
        voltage_output_0.openWaitForAttachment(5000)  # 5 second timeout
        voltage_output_1.openWaitForAttachment(5000)
        
        print("[INFO] ✅ Both voltage outputs connected successfully!")
        
        # Set both channels to 3V
        test_voltage = 3.0
        print(f"[INFO] Setting both channels to {test_voltage}V...")
        
        voltage_output_0.setVoltage(test_voltage)
        voltage_output_1.setVoltage(test_voltage)
        
        print(f"[SUCCESS] ✅ Both channels now outputting {test_voltage}V")
        print("[INFO] Press Ctrl+C to stop and set voltage to 0V")
        
        # Keep running until user stops
        while True:
            print(f"[STATUS] Channel 0: {test_voltage}V | Channel 1: {test_voltage}V", end='\r')
            time.sleep(1)
            
    except PhidgetException as e:
        print(f"[ERROR] ❌ Phidget error: {e}")
        print("[HELP] Check:")
        print("  - Phidget device is connected via USB")
        print("  - Phidget drivers are installed") 
        print("  - No other programs are using the device")
        
    except KeyboardInterrupt:
        print(f"\n[INFO] 🛑 User interrupted - shutting down safely...")
        
    except Exception as e:
        print(f"[ERROR] ❌ Unexpected error: {e}")
        
    finally:
        # Always set voltage to 0V before closing
        print("[INFO] Setting voltages to 0V...")
        try:
            if voltage_output_0:
                voltage_output_0.setVoltage(0.0)
                print("[INFO] ✅ Channel 0 set to 0V")
        except:
            print("[WARN] ⚠️  Could not set Channel 0 to 0V")
            
        try:
            if voltage_output_1:
                voltage_output_1.setVoltage(0.0)
                print("[INFO] ✅ Channel 1 set to 0V")
        except:
            print("[WARN] ⚠️  Could not set Channel 1 to 0V")
            
        # Close connections
        try:
            if voltage_output_0:
                voltage_output_0.close()
        except:
            pass
            
        try:
            if voltage_output_1:
                voltage_output_1.close()
        except:
            pass
            
        print("[INFO] 🔌 Connections closed. Test complete!")

if __name__ == '__main__':
    print("=" * 50)
    print("🔧 PIEZO CONTROLLER TEST - 3V OUTPUT")
    print("=" * 50)
    test_piezo_controller()

