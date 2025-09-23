#!/usr/bin/env python3

import tkinter as tk
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageOutput import *

HUB_SERIAL = 767289  # replace with your hub serial if different

# --- Setup functions ---
def setup_channel(port):
    ch = VoltageOutput()
    ch.setDeviceSerialNumber(HUB_SERIAL)
    ch.setHubPort(port)
    ch.setIsHubPortDevice(False)
    ch.openWaitForAttachment(5000)
    return ch

# --- Slider callback functions ---
def set_voltage_ch0(val):
    voltage = float(val)
    ch0.setVoltage(voltage)
    label0.config(text=f"Channel 0 Voltage: {voltage:.2f} V")

def set_voltage_ch1(val):
    voltage = float(val)
    ch1.setVoltage(voltage)
    label1.config(text=f"Channel 1 Voltage: {voltage:.2f} V")

# --- Main ---
if __name__ == "__main__":
    # Open channels
    ch0 = setup_channel(0)
    ch1 = setup_channel(1)

    # Build GUI
    root = tk.Tk()
    root.title("Phidget Voltage Output Control")

    # Channel 0
    tk.Label(root, text="Channel 0").pack()
    slider0 = tk.Scale(root, from_=0, to=3, resolution=0.01, orient=tk.HORIZONTAL,
                       command=set_voltage_ch0, length=300)
    slider0.pack()
    label0 = tk.Label(root, text="Channel 0 Voltage: 0.00 V")
    label0.pack()

    # Channel 1
    tk.Label(root, text="Channel 1").pack()
    slider1 = tk.Scale(root, from_=0, to=3, resolution=0.01, orient=tk.HORIZONTAL,
                       command=set_voltage_ch1, length=300)
    slider1.pack()
    label1 = tk.Label(root, text="Channel 1 Voltage: 0.00 V")
    label1.pack()

    # Run GUI
    root.mainloop()

    # Cleanup
    ch0.close()
    ch1.close()
