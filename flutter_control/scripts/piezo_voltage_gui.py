#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Piezo Voltage Control GUI
Simple interface for testing piezo deflection at different voltages
"""

import tkinter as tk
from tkinter import ttk
import time
from Phidget22.Devices.VoltageOutput import VoltageOutput
from Phidget22.PhidgetException import PhidgetException

class PiezoVoltageGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Piezo Voltage Control")
        self.root.geometry("600x400")
        
        # Setup Phidgets
        self.setup_phidgets()
        
        # Current voltages
        self.voltage_0 = 0.0
        self.voltage_1 = 0.0
        
        # Create GUI
        self.create_widgets()
        
        # Update display
        self.update_display()
        
    def setup_phidgets(self):
        """Initialize Phidget voltage outputs"""
        try:
            self.voltage_output_0 = VoltageOutput()
            self.voltage_output_1 = VoltageOutput()
            
            self.voltage_output_0.setChannel(0)
            self.voltage_output_0.setHubPort(0)
            
            self.voltage_output_1.setChannel(0)
            self.voltage_output_1.setHubPort(1)
            
            self.voltage_output_0.openWaitForAttachment(5000)
            self.voltage_output_1.openWaitForAttachment(5000)
            
            # Set initial voltage to 0
            self.voltage_output_0.setVoltage(0.0)
            self.voltage_output_1.setVoltage(0.0)
            
            self.connected = True
            print("[INFO] Phidgets connected successfully")
            
        except PhidgetException as e:
            print(f"[ERROR] Failed to connect Phidgets: {e}")
            self.voltage_output_0 = None
            self.voltage_output_1 = None
            self.connected = False
    
    def create_widgets(self):
        """Create all GUI widgets"""
        
        # Title
        title = tk.Label(self.root, text="Piezo Deflection Test", 
                        font=("Arial", 16, "bold"))
        title.pack(pady=10)
        
        # Connection status
        status_text = "✓ Connected" if self.connected else "✗ Not Connected"
        status_color = "green" if self.connected else "red"
        self.status_label = tk.Label(self.root, text=status_text, 
                                     fg=status_color, font=("Arial", 12))
        self.status_label.pack(pady=5)
        
        # Separator
        ttk.Separator(self.root, orient='horizontal').pack(fill='x', padx=20, pady=10)
        
        # ===== CHANNEL 0 =====
        frame_0 = tk.LabelFrame(self.root, text="Channel 0 (Top Piezo)", 
                               font=("Arial", 12, "bold"), padx=20, pady=10)
        frame_0.pack(padx=20, pady=10, fill='both')
        
        # Slider 0
        self.slider_0 = tk.Scale(frame_0, from_=0, to=4.2, resolution=0.1,
                                orient='horizontal', length=400, 
                                command=self.update_voltage_0,
                                label="Voltage (V)", font=("Arial", 10))
        self.slider_0.set(0.0)
        self.slider_0.pack(pady=5)
        
        # Display 0
        self.display_0 = tk.Label(frame_0, text="0.0 V", 
                                 font=("Arial", 14, "bold"), fg="blue")
        self.display_0.pack(pady=5)
        
        # ===== CHANNEL 1 =====
        frame_1 = tk.LabelFrame(self.root, text="Channel 1 (Bottom Piezo)", 
                               font=("Arial", 12, "bold"), padx=20, pady=10)
        frame_1.pack(padx=20, pady=10, fill='both')
        
        # Slider 1
        self.slider_1 = tk.Scale(frame_1, from_=0, to=4.2, resolution=0.1,
                                orient='horizontal', length=400,
                                command=self.update_voltage_1,
                                label="Voltage (V)", font=("Arial", 10))
        self.slider_1.set(0.0)
        self.slider_1.pack(pady=5)
        
        # Display 1
        self.display_1 = tk.Label(frame_1, text="0.0 V", 
                                 font=("Arial", 14, "bold"), fg="blue")
        self.display_1.pack(pady=5)
        
        # Separator
        ttk.Separator(self.root, orient='horizontal').pack(fill='x', padx=20, pady=10)
        
        # ===== PRESET BUTTONS =====
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)
        
        tk.Button(button_frame, text="Both OFF", command=self.preset_both_off,
                 font=("Arial", 10), width=15, bg='lightgray').grid(row=0, column=0, padx=5)
        
        tk.Button(button_frame, text="Ch0 MAX", command=self.preset_ch0_max,
                 font=("Arial", 10), width=15, bg='lightblue').grid(row=0, column=1, padx=5)
        
        tk.Button(button_frame, text="Ch1 MAX", command=self.preset_ch1_max,
                 font=("Arial", 10), width=15, bg='lightgreen').grid(row=0, column=2, padx=5)
        
        tk.Button(button_frame, text="BOTH MAX", command=self.preset_both_max,
                 font=("Arial", 10), width=15, bg='yellow').grid(row=1, column=0, padx=5, pady=5)
        
        tk.Button(button_frame, text="DIFFERENTIAL MAX", command=self.preset_differential,
                 font=("Arial", 10), width=15, bg='orange').grid(row=1, column=1, padx=5, pady=5)
        
        tk.Button(button_frame, text="QUIT", command=self.quit_app,
                 font=("Arial", 10, "bold"), width=15, bg='red', fg='white').grid(row=1, column=2, padx=5, pady=5)
    
    def update_voltage_0(self, value):
        """Update channel 0 voltage when slider moves"""
        self.voltage_0 = float(value)
        if self.voltage_output_0:
            try:
                self.voltage_output_0.setVoltage(self.voltage_0)
            except Exception as e:
                print(f"[ERROR] Ch0: {e}")
    
    def update_voltage_1(self, value):
        """Update channel 1 voltage when slider moves"""
        self.voltage_1 = float(value)
        if self.voltage_output_1:
            try:
                self.voltage_output_1.setVoltage(self.voltage_1)
            except Exception as e:
                print(f"[ERROR] Ch1: {e}")
    
    def update_display(self):
        """Update voltage displays"""
        self.display_0.config(text=f"{self.voltage_0:.1f} V")
        self.display_1.config(text=f"{self.voltage_1:.1f} V")
        
        # Schedule next update
        self.root.after(100, self.update_display)
    
    # ===== PRESET FUNCTIONS =====
    
    def preset_both_off(self):
        """Set both channels to 0V"""
        self.slider_0.set(0.0)
        self.slider_1.set(0.0)
        print("[PRESET] Both OFF")
    
    def preset_ch0_max(self):
        """Channel 0 max, Channel 1 off"""
        self.slider_0.set(4.2)
        self.slider_1.set(0.0)
        print("[PRESET] Ch0 MAX (4.2V), Ch1 OFF")
    
    def preset_ch1_max(self):
        """Channel 1 max, Channel 0 off"""
        self.slider_0.set(0.0)
        self.slider_1.set(4.2)
        print("[PRESET] Ch0 OFF, Ch1 MAX (4.2V)")
    
    def preset_both_max(self):
        """Both channels at max (axial extension)"""
        self.slider_0.set(4.2)
        self.slider_1.set(4.2)
        print("[PRESET] BOTH MAX (4.2V) - Axial extension")
    
    def preset_differential(self):
        """Differential bending (Ch0=MAX, Ch1=0)"""
        self.slider_0.set(4.2)
        self.slider_1.set(0.0)
        print("[PRESET] DIFFERENTIAL MAX - Maximum bending")
        print("         Ch0=4.2V (top expands), Ch1=0V (bottom relaxed)")
    
    def quit_app(self):
        """Cleanup and quit"""
        print("\n[INFO] Shutting down...")
        
        # Set voltages to 0
        if self.voltage_output_0:
            self.voltage_output_0.setVoltage(0.0)
            self.voltage_output_0.close()
        
        if self.voltage_output_1:
            self.voltage_output_1.setVoltage(0.0)
            self.voltage_output_1.close()
        
        print("[INFO] Voltages set to 0V")
        print("[INFO] Phidgets closed")
        
        self.root.quit()
        self.root.destroy()
    
    def run(self):
        """Start the GUI"""
        self.root.protocol("WM_DELETE_WINDOW", self.quit_app)
        self.root.mainloop()


if __name__ == '__main__':
    print("="*60)
    print("PIEZO VOLTAGE CONTROL GUI")
    print("="*60)
    print("\nStarting GUI...")
    
    app = PiezoVoltageGUI()
    app.run()
    
    print("\n[INFO] Application closed")