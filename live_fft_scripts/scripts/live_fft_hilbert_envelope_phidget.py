#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.signal import butter, filtfilt, hilbert
import threading
import time
import math
from Phidget22.Devices.VoltageOutput import VoltageOutput
from Phidget22.PhidgetException import PhidgetException

class PhaseLockNode:
    def __init__(self):
        print("[INFO] PhaseLockNode started")

        self.buffer = []
        self.buffer_size = 180
        self.time_stamps = []

        # New buffer for tracking dominant frequency magnitude over time
        self.magnitude_history = []
        self.magnitude_history_size = 100  # Keep last 100 measurements
        self.magnitude_time_stamps = []
        self.voltage_active_history = []  # Track when voltage output is active

        self.sampling_rate = 50

        self.lowcut = 5.0
        self.highcut = 9.0

        # Initialize waveform parameters before creating UI elements
        self.waveform_thread = None
        self.waveform_running = False
        self.waveform_freq = 10.0 #Adjust for output voltage frequency
        self.phase_delay_percent = 0.33  # Adjust for percent delay of waveform .15=15%
        self.update_rate = 1000  # Hz update rate for voltage waveform

        rospy.Subscriber('/optic_flow_mean_vx', Float32, self.callback)

        # Figure with FFT, envelope, and magnitude history plots
        plt.ion()
        self.fig = plt.figure(figsize=(10, 10))
        
        # Create subplot layout with space for slider
        gs = self.fig.add_gridspec(4, 1, height_ratios=[3, 3, 3, 0.5], hspace=0.4)
        
        self.ax_fft = self.fig.add_subplot(gs[0])
        self.ax_env = self.fig.add_subplot(gs[1])
        self.ax_mag_hist = self.fig.add_subplot(gs[2])

        self.line_fft, = self.ax_fft.plot([], [], lw=2)
        self.ax_fft.set_title("FFT Spectrum")
        self.ax_fft.set_xlabel("Frequency (Hz)")
        self.ax_fft.set_ylabel("Magnitude")
        self.ax_fft.set_xlim(0, 20)
        self.ax_fft.set_ylim(0, 14)

        self.line_env, = self.ax_env.plot([], [], lw=2)
        self.ax_env.set_title("Amplitude Envelope")
        self.ax_env.set_xlabel("Sample")
        self.ax_env.set_ylabel("Amplitude")

        # New plot for magnitude history with voltage activation highlighting
        self.line_mag_hist, = self.ax_mag_hist.plot([], [], lw=2, color='blue', label='Magnitude')
        self.ax_mag_hist.set_title("Dominant Frequency Magnitude Over Time")
        self.ax_mag_hist.set_xlabel("Time (s)")
        self.ax_mag_hist.set_ylabel("Magnitude")
        self.ax_mag_hist.grid(True, alpha=0.3)
        self.ax_mag_hist.legend()

        # Create slider for phase delay adjustment
        ax_slider = self.fig.add_subplot(gs[3])
        self.phase_slider = Slider(
            ax_slider, 'Phase Delay %', 
            0.0, 100.0, 
            valinit=self.phase_delay_percent * 100,
            valfmt='%.1f%%'
        )
        self.phase_slider.on_changed(self.update_phase_delay)

        plt.tight_layout()

        self.phase_pub = rospy.Publisher('/phase_mod', Float32, queue_size=10)
        self.envelope_pub = rospy.Publisher('/amplitude_envelope', Float32, queue_size=10)

        try:
            self.voltage_output_0 = VoltageOutput()
            self.voltage_output_1 = VoltageOutput()

            self.voltage_output_0.setChannel(0)
            self.voltage_output_0.setHubPort(0)

            self.voltage_output_1.setChannel(0)
            self.voltage_output_1.setHubPort(1)

            self.voltage_output_0.openWaitForAttachment(5000)
            self.voltage_output_1.openWaitForAttachment(5000)

            print("[INFO] Voltage Outputs attached")

        except PhidgetException as e:
            print(f"[ERROR] Phidget error: {e}")
            self.voltage_output_0 = None
            self.voltage_output_1 = None

        # Track start time for relative time plotting
        self.start_time = rospy.get_time()

    def update_phase_delay(self, val):
        """Callback for phase delay slider"""
        self.phase_delay_percent = val / 100.0
        print(f"[INFO] Phase delay updated to {val:.1f}% ({self.phase_delay_percent:.3f})")

    def callback(self, msg):
        self.buffer.append(msg.data)
        self.time_stamps.append(rospy.get_time())

        if len(self.buffer) > self.buffer_size:
            self.buffer.pop(0)
            self.time_stamps.pop(0)

        if len(self.time_stamps) >= 2:
            dt = np.diff(self.time_stamps)
            avg_dt = np.mean(dt)
            if avg_dt > 0:
                self.sampling_rate = 1.0 / avg_dt

    def compute_fft(self, data):
        N = len(data)
        fft_vals = np.fft.fft(data)
        freqs = np.fft.fftfreq(N, d=1.0 / self.sampling_rate)
        magnitudes = np.abs(fft_vals)
        pos_mask = freqs >= 0
        return freqs[pos_mask], magnitudes[pos_mask]

    def bandpass_filter(self, data):
        nyq = 0.5 * self.sampling_rate
        low = self.lowcut / nyq
        high = self.highcut / nyq
        b, a = butter(N=4, Wn=[low, high], btype='band')
        return filtfilt(b, a, data)

    def start_waveform(self, frequency):
        if self.waveform_running:
            self.waveform_freq = frequency
            return

        self.waveform_freq = frequency
        self.waveform_running = True

        def run_waveform():
            start_time = time.time()
            while self.waveform_running:
                t = time.time() - start_time
                phase_delay = self.phase_delay_percent * (1.0 / self.waveform_freq)  # seconds
                t_delayed = t + phase_delay

                voltage = 1.5 + 1.5 * math.sin(2 * math.pi * self.waveform_freq * t_delayed)
                voltage = max(0.0, min(3.0, voltage))

                try:
                    if self.voltage_output_0:
                        self.voltage_output_0.setVoltage(voltage)
                    if self.voltage_output_1:
                        self.voltage_output_1.setVoltage(voltage)

                    print(f"[Waveform Output] t={t_delayed:.3f}s | Voltage Sent = {voltage:.3f} V")
                except Exception as e:
                    print(f"[Waveform Output Error] {e}")

                time.sleep(0.01)

        self.waveform_thread = threading.Thread(target=run_waveform)
        self.waveform_thread.daemon = True
        self.waveform_thread.start()

    def stop_waveform(self):
        self.waveform_running = False
        if self.waveform_thread:
            self.waveform_thread.join()
        if self.voltage_output_0:
            self.voltage_output_0.setVoltage(0.0)
        if self.voltage_output_1:
            self.voltage_output_1.setVoltage(0.0)

    def update_magnitude_history(self, magnitude, voltage_active):
        """Update the magnitude history buffer with timestamp and voltage status"""
        current_time = rospy.get_time() - self.start_time  # Relative time in seconds
        
        self.magnitude_history.append(magnitude)
        self.magnitude_time_stamps.append(current_time)
        self.voltage_active_history.append(voltage_active)
        
        # Keep buffer size manageable
        if len(self.magnitude_history) > self.magnitude_history_size:
            self.magnitude_history.pop(0)
            self.magnitude_time_stamps.pop(0)
            self.voltage_active_history.pop(0)

    def update(self):
        if len(self.buffer) < self.buffer_size:
            return

        data = np.array(self.buffer)

        freqs, magnitudes = self.compute_fft(data)
        idx = np.argmax(magnitudes[1:]) + 1
        dom_freq = freqs[idx]
        dom_mag = magnitudes[idx]

        print(f"[INFO] Dominant Freq: {dom_freq:.2f} Hz | Magnitude: {dom_mag:.2f}")

        # Determine if voltage output should be active
        voltage_should_be_active = dom_mag > 42     ###Sensitivity Cutoff Magnitude
        
        # Update magnitude history with voltage status
        self.update_magnitude_history(dom_mag, voltage_should_be_active)

        try:
            filtered = self.bandpass_filter(data)
        except Exception as e:
            print(f"[ERROR] Filter failed: {e}")
            return

        if np.all(filtered == 0):
            print("[WARN] Filtered signal all zeros!")
            return

        try:
            analytic_signal = hilbert(filtered)
            instantaneous_phase = np.unwrap(np.angle(analytic_signal))
            amplitude_envelope = np.abs(analytic_signal)
        except Exception as e:
            print(f"[ERROR] Hilbert failed: {e}")
            return

        phase_mod = instantaneous_phase[-1] % (2 * np.pi)
        env_val = amplitude_envelope[-1]

        print(f"[INFO] Phase mod 2π: {phase_mod:.2f} rad")
        print(f"[INFO] Envelope: {env_val:.4f}")

        self.phase_pub.publish(Float32(phase_mod))
        self.envelope_pub.publish(Float32(env_val))

        # Update FFT plot
        self.line_fft.set_data(freqs, magnitudes)
        self.ax_fft.set_xlim(0, 20)
        self.ax_fft.set_ylim(0, np.max(magnitudes) * 1.1)

        # Update envelope plot
        self.line_env.set_data(np.arange(len(amplitude_envelope)), amplitude_envelope)
        self.ax_env.set_xlim(0, len(amplitude_envelope))
        self.ax_env.set_ylim(0, np.max(amplitude_envelope) * 1.2)

        # Update magnitude history plot with voltage activation highlighting
        if len(self.magnitude_history) > 1:
            # Clear previous background highlighting
            self.ax_mag_hist.collections.clear()  # Remove previous fill_between
            
            # Plot the main magnitude line
            self.line_mag_hist.set_data(self.magnitude_time_stamps, self.magnitude_history)
            
            # Highlight regions where voltage is active
            times = np.array(self.magnitude_time_stamps)
            magnitudes = np.array(self.magnitude_history)
            voltage_active = np.array(self.voltage_active_history)
            
            # Create highlighted background for active periods
            if len(times) > 0:
                # Find continuous segments where voltage is active
                active_indices = np.where(voltage_active)[0]
                if len(active_indices) > 0:
                    # Group consecutive indices
                    segments = []
                    start_idx = active_indices[0]
                    
                    for i in range(1, len(active_indices)):
                        if active_indices[i] != active_indices[i-1] + 1:
                            # End of segment
                            segments.append((start_idx, active_indices[i-1]))
                            start_idx = active_indices[i]
                    
                    # Add final segment
                    segments.append((start_idx, active_indices[-1]))
                    
                    # Highlight each segment
                    for start_idx, end_idx in segments:
                        self.ax_mag_hist.fill_between(
                            times[start_idx:end_idx+1], 
                            0, 
                            magnitudes[start_idx:end_idx+1],
                            alpha=0.3, 
                            color='green', 
                            label='Voltage Active' if segments.index((start_idx, end_idx)) == 0 else ""
                        )
            
            # Auto-scale x-axis to show recent data
            time_window = 30  # Show last 30 seconds
            current_time = self.magnitude_time_stamps[-1]
            self.ax_mag_hist.set_xlim(max(0, current_time - time_window), current_time + 1)
            
            # Auto-scale y-axis with some padding
            if self.magnitude_history:
                min_mag = min(self.magnitude_history)
                max_mag = max(self.magnitude_history)
                padding = (max_mag - min_mag) * 0.1 if max_mag > min_mag else 1
                self.ax_mag_hist.set_ylim(min_mag - padding, max_mag + padding)
            
            # Update legend if voltage highlighting is present
            handles, labels = self.ax_mag_hist.get_legend_handles_labels()
            if len(handles) > 1:  # If we have both magnitude and voltage active
                self.ax_mag_hist.legend()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        if self.voltage_output_0 and self.voltage_output_1:
            if voltage_should_be_active:
                self.start_waveform(dom_freq)
            else:
                self.stop_waveform()

def ros_spin():
    rospy.spin()

def main():
    rospy.init_node('phase_lock_node', anonymous=True)
    node = PhaseLockNode()

    t = threading.Thread(target=ros_spin)
    t.daemon = True
    t.start()

    plt.show(block=False)

    try:
        while not rospy.is_shutdown():
            node.update()
            plt.pause(0.1)
    except KeyboardInterrupt:
        print("Shutting down.")
        if node.voltage_output_0:
            node.voltage_output_0.setVoltage(0.0)
        if node.voltage_output_1:
            node.voltage_output_1.setVoltage(0.0)

if __name__ == '__main__':
    main()
