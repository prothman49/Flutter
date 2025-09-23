#!/usr/bin/env python3
# test_vint_voltage_outputs.py
#
# Simple script to test Phidgets VINT Hub analog outputs
# Adjusts output voltage on channel 0 and channel 1

from Phidget22.Phidget import *
from Phidget22.Devices.VoltageOutput import *
import time

def setup_channel(channel_number, voltage):
    ch = VoltageOutput()
    ch.setHubPort(channel_number)   # choose channel (0 or 1)
    ch.setDeviceSerialNumber(0)     # 0 = any connected VINT Hub
    ch.openWaitForAttachment(5000)

    # Set initial voltage
    ch.setVoltage(voltage)
    print(f"Channel {channel_number} set to {voltage:.2f} V")
    return ch

def main():
    try:
        # Ask user for voltages
        v0 = float(input("Enter voltage for channel 0 (0–3V): "))
        v1 = float(input("Enter voltage for channel 1 (0–3V): "))

        # Clamp to valid range
        v0 = max(0.0, min(3.0, v0))
        v1 = max(0.0, min(3.0, v1))

        # Setup channels
        ch0 = setup_channel(0, v0)
        ch1 = setup_channel(1, v1)

        print("Press Ctrl+C to stop. Outputs will stay active until program exits.")
        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nExiting and closing channels...")
    finally:
        try:
            ch0.close()
            ch1.close()
        except:
            pass

if __name__ == "__main__":
    main()

