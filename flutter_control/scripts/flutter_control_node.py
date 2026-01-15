#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Core Flutter Control Node
Contains PhaseLockNode class used by all test scripts
"""

import rospy
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from scipy.signal import butter, filtfilt, hilbert
import threading
import time
import math
from Phidget22.Devices.VoltageOutput import VoltageOutput
from Phidget22.PhidgetException import PhidgetException

class PhaseLockNode:
    def __init__(self, enable_gui=True):
        print("[INFO] PhaseLockNode started")

        self.buffer = []
        self.buffer_size = 180
        self.time_stamps = []

        # Tracking buffers
        self.magnitude_history = []
        self.magnitude_history_size = 100
        self.magnitude_time_stamps = []
        self.voltage_active_history = []
        self.control_state_history = []
        self.threshold_exceeded_history = []

        self.sampling_rate = 50
        self.lowcut = 5.0
        self.highcut = 9.0

        # Waveform parameters
        self.waveform_thread = None
        self.waveform_running = False
        self.waveform_freq = 10.0
        self.phase_offset_degrees = 0.0
        self.update_rate = 1000
        
        # Control flags
        self.control_enabled = True
        self.flutter_threshold = 130
        
        # Flutter metrics
        self.flutter_duration = 0.0
        self.last_flutter_time = None
        self.flutter_event_count = 0
        self.total_flutter_time = 0.0

        # Latest values
        self.instantaneous_phase_latest = 0.0
        self.envelope_latest = 0.0

        rospy.Subscriber('/optic_flow_mean_vx', Float32, self.callback)

        # GUI setup (optional)
        self.enable_gui = enable_gui
        if self.enable_gui:
            self._setup_gui()

        # Publishers
        self.phase_pub = rospy.Publisher('/phase_mod', Float32, queue_size=10)
        self.envelope_pub = rospy.Publisher('/amplitude_envelope', Float32, queue_size=10)

        # Phidget setup
        self._setup_phidgets()

        # Track start time
        self.start_time = rospy.get_time()

    def _setup_gui(self):
        """Setup matplotlib GUI"""
        plt.ion()
        self.fig = plt.figure(figsize=(10, 12))
        
        gs = self.fig.add_gridspec(6, 2, height_ratios=[3, 3, 3, 1, 0.5, 0.5], 
                                   width_ratios=[1, 1], hspace=0.4, wspace=0.3)
        
        self.ax_fft = self.fig.add_subplot(gs[0, :])
        self.ax_env = self.fig.add_subplot(gs[1, :])
        self.ax_mag_hist = self.fig.add_subplot(gs[2, :])
        
        # Stats text area
        self.ax_stats = self.fig.add_subplot(gs[3, :])
        self.ax_stats.axis('off')
        self.stats_text = self.ax_stats.text(0.05, 0.5, '', fontsize=10, 
                                             verticalalignment='center', family='monospace')

        # Setup plot lines
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

        self.line_mag_hist, = self.ax_mag_hist.plot([], [], lw=2, color='blue', label='Magnitude')
        self.threshold_line = self.ax_mag_hist.axhline(y=self.flutter_threshold, color='orange', 
                                                        linestyle='--', linewidth=2, label='Threshold')
        self.ax_mag_hist.set_title("Dominant Frequency Magnitude Over Time")
        self.ax_mag_hist.set_xlabel("Time (s)")
        self.ax_mag_hist.set_ylabel("Magnitude")
        self.ax_mag_hist.grid(True, alpha=0.3)
        self.ax_mag_hist.legend()

        # Phase slider
        ax_slider = self.fig.add_subplot(gs[4, :])
        self.phase_slider = Slider(ax_slider, 'Phase Offset (deg)', 
                                   0.0, 360.0, valinit=self.phase_offset_degrees, valfmt='%.1f deg')
        self.phase_slider.on_changed(self.update_phase_offset)

        # Control button
        ax_button = self.fig.add_subplot(gs[5, :])
        self.control_button = Button(ax_button, 'Control: ENABLED', color='lightgreen', hovercolor='green')
        self.control_button.on_clicked(self.toggle_control_button)

        plt.tight_layout()

    def _setup_phidgets(self):
        """Setup Phidget voltage outputs"""
        try:
            self.voltage_output_0 = VoltageOutput()
            self.voltage_output_1 = VoltageOutput()

            self.voltage_output_0.setChannel(0)
            self.voltage_output_0.setHubPort(0)

            self.voltage_output_1.setChannel(0)
            self.voltage_output_1.setHubPort(1)

            self.voltage_output_0.openWaitForAttachment(5000)
            self.voltage_output_1.openWaitForAttachment(5000)

            print("[INFO] Voltage Outputs attached (0-4.2V range)")

        except PhidgetException as e:
            print(f"[ERROR] Phidget error: {e}")
            self.voltage_output_0 = None
            self.voltage_output_1 = None

    def update_phase_offset(self, val):
        """Callback for phase offset slider"""
        self.phase_offset_degrees = val
        print(f"[INFO] Phase offset updated to {val:.1f} deg ({val * np.pi / 180.0:.3f} rad)")

    def toggle_control_button(self, event):
        """Toggle control on/off via button press"""
        self.control_enabled = not self.control_enabled
        
        if self.enable_gui:
            if self.control_enabled:
                self.control_button.label.set_text('Control: ENABLED')
                self.control_button.color = 'lightgreen'
                self.control_button.hovercolor = 'green'
            else:
                self.control_button.label.set_text('Control: DISABLED')
                self.control_button.color = 'lightcoral'
                self.control_button.hovercolor = 'red'
            
            self.fig.canvas.draw()
        
        print(f"\n{'='*50}")
        print(f"[CONTROL] {'ENABLED' if self.control_enabled else 'DISABLED'}")
        print(f"{'='*50}\n")

    def callback(self, msg):
        """ROS callback for optic flow data"""
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
        """Compute FFT of data"""
        N = len(data)
        fft_vals = np.fft.fft(data)
        freqs = np.fft.fftfreq(N, d=1.0 / self.sampling_rate)
        magnitudes = np.abs(fft_vals)
        pos_mask = freqs >= 0
        return freqs[pos_mask], magnitudes[pos_mask]

    def bandpass_filter(self, data):
        """Bandpass filter data"""
        nyq = 0.5 * self.sampling_rate
        low = self.lowcut / nyq
        high = self.highcut / nyq
        b, a = butter(N=4, Wn=[low, high], btype='band')
        return filtfilt(b, a, data)

    def start_waveform(self, frequency):
        """Start square wave output"""
        if self.waveform_running:
            self.waveform_freq = frequency
            return

        self.waveform_freq = frequency
        self.waveform_running = True

        def run_waveform():
            start_time = time.time()
            last_print_time = start_time
            print_interval = 1.0 / max(self.waveform_freq, 1.0)
            
            while self.waveform_running:
                t = time.time() - start_time
                
                phase_offset_rad = self.phase_offset_degrees * math.pi / 180.0
                phase = (2 * math.pi * self.waveform_freq * t + phase_offset_rad) % (2 * math.pi)
                
                # Square wave (0-4.2V range - SAFE)
                if phase < math.pi:
                    voltage_0 = 4.2
                    voltage_1 = 0.0
                else:
                    voltage_0 = 0.0
                    voltage_1 = 4.2
                
                try:
                    if self.voltage_output_0:
                        self.voltage_output_0.setVoltage(voltage_0)
                    if self.voltage_output_1:
                        self.voltage_output_1.setVoltage(voltage_1)
                    
                    if t - last_print_time >= print_interval:
                        print(f"[Square Wave] f={self.waveform_freq:.1f}Hz | V0={voltage_0:.1f}V, V1={voltage_1:.1f}V | Phase={self.phase_offset_degrees:.0f}deg")
                        last_print_time = t
                        print_interval = 1.0 / max(self.waveform_freq, 1.0)
                except Exception as e:
                    print(f"[Waveform Output Error] {e}")
                
                time.sleep(0.01)
        
        self.waveform_thread = threading.Thread(target=run_waveform)
        self.waveform_thread.daemon = True
        self.waveform_thread.start()

    def stop_waveform(self):
        """Stop waveform output"""
        self.waveform_running = False
        if self.waveform_thread:
            self.waveform_thread.join()
        if self.voltage_output_0:
            self.voltage_output_0.setVoltage(0.0)
        if self.voltage_output_1:
            self.voltage_output_1.setVoltage(0.0)

    def set_phase(self, phase_degrees):
        """Set phase offset programmatically"""
        self.phase_offset_degrees = phase_degrees
        if self.enable_gui and hasattr(self, 'phase_slider'):
            self.phase_slider.set_val(phase_degrees)

    def update_magnitude_history(self, magnitude, threshold_exceeded):
        """Update the magnitude history buffer with timestamp and threshold status"""
        current_time = rospy.get_time() - self.start_time
        
        self.magnitude_history.append(magnitude)
        self.magnitude_time_stamps.append(current_time)
        self.threshold_exceeded_history.append(threshold_exceeded)
        self.control_state_history.append(self.control_enabled)
        
        voltage_active = threshold_exceeded and self.control_enabled
        self.voltage_active_history.append(voltage_active)
        
        if len(self.magnitude_history) > self.magnitude_history_size:
            self.magnitude_history.pop(0)
            self.magnitude_time_stamps.pop(0)
            self.threshold_exceeded_history.pop(0)
            self.control_state_history.pop(0)
            self.voltage_active_history.pop(0)

    def update_flutter_stats(self, magnitude, threshold_exceeded):
        """Track flutter statistics"""
        current_time = rospy.get_time()
        
        if threshold_exceeded:
            if self.last_flutter_time is None:
                self.flutter_event_count += 1
                self.flutter_duration = 0.0
                print(f"[FLUTTER START] Event #{self.flutter_event_count}")
            else:
                dt = current_time - self.last_flutter_time
                self.flutter_duration += dt
                self.total_flutter_time += dt
            
            self.last_flutter_time = current_time
        else:
            if self.last_flutter_time is not None:
                print(f"[FLUTTER STOP] Duration: {self.flutter_duration:.2f}s")
                self.flutter_duration = 0.0
            self.last_flutter_time = None

    def update_stats_display(self, dom_mag, dom_freq):
        """Update the statistics text display"""
        if not self.enable_gui:
            return
            
        runtime = rospy.get_time() - self.start_time
        flutter_percent = (self.total_flutter_time / runtime * 100) if runtime > 0 else 0
        
        stats_str = (
            f"Flutter Events: {self.flutter_event_count}  |  "
            f"Total Flutter Time: {self.total_flutter_time:.1f}s ({flutter_percent:.1f}%)  |  "
            f"Current Magnitude: {dom_mag:.1f}  |  "
            f"Frequency: {dom_freq:.1f} Hz"
        )
        
        if self.last_flutter_time is not None:
            stats_str += f"  |  Current Event: {self.flutter_duration:.1f}s"
        
        self.stats_text.set_text(stats_str)

    def update(self):
        """Main update function - call this in a loop"""
        if len(self.buffer) < self.buffer_size:
            return

        data = np.array(self.buffer)

        freqs, magnitudes = self.compute_fft(data)
        idx = np.argmax(magnitudes[1:]) + 1
        dom_freq = freqs[idx]
        dom_mag = magnitudes[idx]

        threshold_exceeded = dom_mag > self.flutter_threshold
        
        self.update_flutter_stats(dom_mag, threshold_exceeded)
        
        print(f"[INFO] Freq: {dom_freq:.2f} Hz | Mag: {dom_mag:.2f} | Threshold: {'EXCEEDED' if threshold_exceeded else 'OK'} | Control: {'ON' if self.control_enabled else 'OFF'}")
        
        self.update_magnitude_history(dom_mag, threshold_exceeded)

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
        
        self.instantaneous_phase_latest = phase_mod
        self.envelope_latest = env_val

        self.phase_pub.publish(Float32(phase_mod))
        self.envelope_pub.publish(Float32(env_val))

        if self.enable_gui:
            self._update_plots(freqs, magnitudes, amplitude_envelope, dom_mag, dom_freq)

        # Control logic
        if self.control_enabled and threshold_exceeded:
            if self.voltage_output_0 and self.voltage_output_1:
                self.start_waveform(dom_freq)
        else:
            self.stop_waveform()

    def _update_plots(self, freqs, magnitudes, amplitude_envelope, dom_mag, dom_freq):
        """Update all plots"""
        # Update FFT plot
        self.line_fft.set_data(freqs, magnitudes)
        self.ax_fft.set_xlim(0, 20)
        self.ax_fft.set_ylim(0, np.max(magnitudes) * 1.1 if np.max(magnitudes) > 0 else 1)

        # Update envelope plot
        self.line_env.set_data(np.arange(len(amplitude_envelope)), amplitude_envelope)
        self.ax_env.set_xlim(0, len(amplitude_envelope))
        self.ax_env.set_ylim(0, np.max(amplitude_envelope) * 1.2 if np.max(amplitude_envelope) > 0 else 1)

        # Update statistics
        self.update_stats_display(dom_mag, dom_freq)

        # Update magnitude history plot
        if len(self.magnitude_history) > 1:
            self.ax_mag_hist.collections.clear()
            
            self.line_mag_hist.set_data(self.magnitude_time_stamps, self.magnitude_history)
            
            times = np.array(self.magnitude_time_stamps)
            magnitudes_array = np.array(self.magnitude_history)
            threshold_exceeded_array = np.array(self.threshold_exceeded_history)
            voltage_active = np.array(self.voltage_active_history)
            control_active = np.array(self.control_state_history)
            
            if len(times) > 0:
                # Green: Control active
                active_indices = np.where(voltage_active)[0]
                if len(active_indices) > 0:
                    self._highlight_segments(times, magnitudes_array, active_indices, 'green', 'Control Active')
                
                # Yellow: Threshold exceeded but control disabled
                threshold_no_control = threshold_exceeded_array & ~control_active
                threshold_no_control_indices = np.where(threshold_no_control)[0]
                if len(threshold_no_control_indices) > 0:
                    self._highlight_segments(times, magnitudes_array, threshold_no_control_indices, 'yellow', 'Threshold Exceeded (No Control)')
                
                # Light red: Control disabled, below threshold
                below_threshold_no_control = ~threshold_exceeded_array & ~control_active
                disabled_indices = np.where(below_threshold_no_control)[0]
                if len(disabled_indices) > 0:
                    self._highlight_segments(times, magnitudes_array, disabled_indices, 'red', 'Control Disabled', alpha=0.15)
            
            # Auto-scale
            time_window = 30
            current_time = self.magnitude_time_stamps[-1]
            self.ax_mag_hist.set_xlim(max(0, current_time - time_window), current_time + 1)
            
            if self.magnitude_history:
                min_mag = min(self.magnitude_history)
                max_mag = max(self.magnitude_history)
                padding = (max_mag - min_mag) * 0.1 if max_mag > min_mag else 1
                self.ax_mag_hist.set_ylim(min_mag - padding, max_mag + padding)
            
            handles, labels = self.ax_mag_hist.get_legend_handles_labels()
            if len(handles) > 0:
                self.ax_mag_hist.legend(loc='upper left', fontsize=8)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def _highlight_segments(self, times, magnitudes, indices, color, label, alpha=0.3):
        """Helper to highlight contiguous segments"""
        if len(indices) == 0:
            return
            
        segments = []
        start_idx = indices[0]
        
        for i in range(1, len(indices)):
            if indices[i] != indices[i-1] + 1:
                segments.append((start_idx, indices[i-1]))
                start_idx = indices[i]
        
        segments.append((start_idx, indices[-1]))
        
        for idx, (start_idx, end_idx) in enumerate(segments):
            self.ax_mag_hist.fill_between(
                times[start_idx:end_idx+1], 
                0, 
                magnitudes[start_idx:end_idx+1],
                alpha=alpha, 
                color=color, 
                label=label if idx == 0 else ""
            )

    def cleanup(self):
        """Cleanup on shutdown"""
        print("\n[INFO] Cleaning up...")
        self.stop_waveform()
        if self.voltage_output_0:
            self.voltage_output_0.setVoltage(0.0)
            self.voltage_output_0.close()
        if self.voltage_output_1:
            self.voltage_output_1.setVoltage(0.0)
            self.voltage_output_1.close()