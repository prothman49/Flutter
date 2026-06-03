#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Flutter Control Node — Multi-Condition
=======================================
Conditions:
  CLOSED_LOOP       — tracks real-time flutter frequency (original)
  OPEN_LOOP_54      — locks to 5.4 Hz on trigger, no frequency tracking
  POISSON           — single-period pulses, exponential IPI (mean=185ms, floor=185ms)
  UNIFORM_JITTER    — single-period pulses, uniform IPI (185ms ± 92.5ms, floor=185ms)
  PHASE_RANDOMIZED  — exact 5.4 Hz IPI, phase offset randomized fresh each trigger event

Launch:
  rosrun <pkg> flutter_control_node.py --condition CLOSED_LOOP
  rosrun <pkg> flutter_control_node.py --condition POISSON

Condition can also be changed live via the GUI dropdown.

Changelog:
  - Added /piezo_voltages (Float32MultiArray) publisher wired into _set_voltages()
    so every hardware write is logged to the bag simultaneously.
  - Moved bandpass + Hilbert computation into the ROS callback so /phase_mod and
    /amplitude_envelope publish at the full optic flow rate (~55 Hz) instead of
    the GUI update rate (~2.6 Hz). GUI still updates at its own cadence.
  - Added /dominant_frequency and /dominant_magnitude publishers in update()
    so FFT results are recorded to the bag at the GUI update rate (~10 Hz).
"""

import argparse
import math
import os
import random
import threading
import time

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import rospy
from matplotlib.widgets import Slider, Button
from scipy.signal import butter, filtfilt, hilbert
from std_msgs.msg import Float32, Float32MultiArray

from Phidget22.Devices.VoltageOutput import VoltageOutput
from Phidget22.PhidgetException import PhidgetException

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
OPEN_LOOP_FREQ   = 5.4
PULSE_PERIOD     = 1.0 / OPEN_LOOP_FREQ
PULSE_WIDTH      = PULSE_PERIOD
MIN_IPI          = PULSE_PERIOD

REPLAY_SIGNALS_DIR = os.path.expanduser(
    '~/catkin_ws/src/flutter_control/data/results/bags/replay_signals'
)

CONDITIONS = ['CLOSED_LOOP', 'OPEN_LOOP_54', 'POISSON', 'UNIFORM_JITTER', 'PHASE_RANDOMIZED', 'REPLAY']

CONDITION_COLORS = {
    'CLOSED_LOOP':      '#4CAF50',
    'OPEN_LOOP_54':     '#2196F3',
    'POISSON':          '#FF9800',
    'UNIFORM_JITTER':   '#9C27B0',
    'PHASE_RANDOMIZED': '#F44336',
    'REPLAY':           '#795548',
}

CONDITION_DESCRIPTIONS = {
    'CLOSED_LOOP':      'Closed Loop — tracks flutter frequency in real time',
    'OPEN_LOOP_54':     'Open Loop 5.4 Hz — locks to fixed frequency on trigger',
    'POISSON':          'Poisson — single-period pulses, exponential IPI (floor=1 period)',
    'UNIFORM_JITTER':   'Uniform Jitter — single-period pulses, uniform IPI ±50%',
    'PHASE_RANDOMIZED': 'Phase Randomized — exact 5.4 Hz, random phase each trigger',
    'REPLAY':           'Replay — plays back pre-recorded closed loop actuation signal',
}


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class PhaseLockNode:
    def __init__(self, condition='CLOSED_LOOP', enable_gui=True):
        print(f"[INFO] PhaseLockNode started | condition={condition}")

        # --- signal buffers ---
        self.buffer            = []
        self.buffer_size       = 180
        self.time_stamps       = []
        self.sampling_rate     = 50

        self.lowcut  = 5.0
        self.highcut = 9.0

        # --- magnitude history ---
        self.magnitude_history          = []
        self.magnitude_history_size     = 100
        self.magnitude_time_stamps      = []
        self.voltage_active_history     = []
        self.control_state_history      = []
        self.threshold_exceeded_history = []

        # --- control state ---
        self.condition         = condition.upper()
        self.control_enabled   = True
        self.flutter_threshold = 42

        # --- flutter stats ---
        self.flutter_duration    = 0.0
        self.last_flutter_time   = None
        self.flutter_event_count = 0
        self.total_flutter_time  = 0.0

        # --- waveform state ---
        self.waveform_thread  = None
        self.waveform_running = False
        self.waveform_freq    = OPEN_LOOP_FREQ
        self.phase_offset_degrees = 0.0

        # --- hilbert / phase ---
        self.instantaneous_phase_latest = 0.0
        self.envelope_latest            = 0.0

        # --- current piezo voltages (for publisher, updated by _set_voltages) ---
        # FIX: track last-written voltages so the ROS publisher always reflects
        # the true hardware state even when called from the waveform thread.
        self._v0 = 0.0
        self._v1 = 0.0
        self._voltage_lock = threading.Lock()

        # --- ROS ---
        rospy.Subscriber('/optic_flow_mean_vx', Float32, self.callback)
        self.phase_pub    = rospy.Publisher('/phase_mod',            Float32,           queue_size=10)
        self.envelope_pub = rospy.Publisher('/amplitude_envelope',   Float32,           queue_size=10)
        # FIX: new publisher — data[0]=ch0 voltage, data[1]=ch1 voltage
        self.piezo_pub    = rospy.Publisher('/piezo_voltages',       Float32MultiArray, queue_size=10)
        self.dom_freq_pub = rospy.Publisher('/dominant_frequency',   Float32,           queue_size=10)
        self.dom_mag_pub  = rospy.Publisher('/dominant_magnitude',   Float32,           queue_size=10)

        # --- Phidget ---
        self.voltage_output_0 = None
        self.voltage_output_1 = None
        self._init_phidget()

        self.start_time = rospy.get_time()

        # --- GUI ---
        self.enable_gui = enable_gui
        if enable_gui:
            self._init_gui()

    # -----------------------------------------------------------------------
    # Init helpers
    # -----------------------------------------------------------------------
    def _init_phidget(self):
        try:
            self.voltage_output_0 = VoltageOutput()
            self.voltage_output_1 = VoltageOutput()
            self.voltage_output_0.setChannel(0)
            self.voltage_output_0.setHubPort(0)
            self.voltage_output_1.setChannel(0)
            self.voltage_output_1.setHubPort(1)
            self.voltage_output_0.openWaitForAttachment(5000)
            self.voltage_output_1.openWaitForAttachment(5000)
            print("[INFO] Voltage outputs attached (0–4.2 V)")
        except PhidgetException as e:
            print(f"[ERROR] Phidget: {e}")

    def _init_gui(self):
        plt.ion()
        self.fig = plt.figure(figsize=(11, 13))
        gs = self.fig.add_gridspec(
            7, 2,
            height_ratios=[3, 3, 3, 0.8, 0.5, 0.5, 0.5],
            width_ratios=[1, 1],
            hspace=0.45, wspace=0.3
        )

        self.ax_fft      = self.fig.add_subplot(gs[0, :])
        self.ax_env      = self.fig.add_subplot(gs[1, :])
        self.ax_mag_hist = self.fig.add_subplot(gs[2, :])
        self.ax_stats    = self.fig.add_subplot(gs[3, :])

        self.ax_stats.axis('off')
        self.stats_text = self.ax_stats.text(
            0.02, 0.5, '', fontsize=9,
            verticalalignment='center', family='monospace',
            transform=self.ax_stats.transAxes
        )

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
        self.threshold_line  = self.ax_mag_hist.axhline(
            y=self.flutter_threshold, color='orange',
            linestyle='--', linewidth=2, label='Threshold'
        )
        self.ax_mag_hist.set_title("Dominant Frequency Magnitude Over Time")
        self.ax_mag_hist.set_xlabel("Time (s)")
        self.ax_mag_hist.set_ylabel("Magnitude")
        self.ax_mag_hist.grid(True, alpha=0.3)
        self.ax_mag_hist.legend(loc='upper left', fontsize=8)

        ax_slider = self.fig.add_subplot(gs[4, :])
        self.phase_slider = Slider(
            ax_slider, 'Phase Offset (deg)',
            0.0, 360.0,
            valinit=self.phase_offset_degrees,
            valfmt='%.1f deg'
        )
        self.phase_slider.on_changed(self._cb_phase_slider)

        ax_btn_ctrl = self.fig.add_subplot(gs[5, 0])
        self.control_button = Button(ax_btn_ctrl, 'Control: ENABLED',
                                     color='lightgreen', hovercolor='green')
        self.control_button.on_clicked(self._cb_toggle_control)

        ax_btn_cond = self.fig.add_subplot(gs[5, 1])
        self.condition_button = Button(
            ax_btn_cond,
            f'Condition: {self.condition}',
            color='lightyellow', hovercolor='yellow'
        )
        self.condition_button.on_clicked(self._cb_cycle_condition)

        ax_desc = self.fig.add_subplot(gs[6, :])
        ax_desc.axis('off')
        self.condition_desc_text = ax_desc.text(
            0.5, 0.5,
            CONDITION_DESCRIPTIONS[self.condition],
            fontsize=9, ha='center', va='center',
            style='italic',
            transform=ax_desc.transAxes
        )

        plt.tight_layout()

    # -----------------------------------------------------------------------
    # GUI callbacks
    # -----------------------------------------------------------------------
    def _cb_phase_slider(self, val):
        self.phase_offset_degrees = val
        print(f"[INFO] Phase offset → {val:.1f} deg")

    def _cb_toggle_control(self, event):
        self.control_enabled = not self.control_enabled
        if self.control_enabled:
            self.control_button.label.set_text('Control: ENABLED')
            self.control_button.color = 'lightgreen'
            self.control_button.hovercolor = 'green'
        else:
            self.control_button.label.set_text('Control: DISABLED')
            self.control_button.color = 'lightcoral'
            self.control_button.hovercolor = 'red'
            self.stop_waveform()
        print(f"[CONTROL] {'ENABLED' if self.control_enabled else 'DISABLED'}")
        self.fig.canvas.draw()

    def _cb_cycle_condition(self, event):
        idx = CONDITIONS.index(self.condition)
        self.condition = CONDITIONS[(idx + 1) % len(CONDITIONS)]
        self.stop_waveform()
        self.condition_button.label.set_text(f'Condition: {self.condition}')
        self.condition_desc_text.set_text(CONDITION_DESCRIPTIONS[self.condition])
        print(f"[CONDITION] Switched to {self.condition}")
        self.fig.canvas.draw()

    # -----------------------------------------------------------------------
    # Public interface
    # -----------------------------------------------------------------------
    def set_phase(self, degrees):
        self.phase_offset_degrees = degrees
        if self.enable_gui:
            self.phase_slider.set_val(degrees)

    def set_condition(self, condition):
        condition = condition.upper()
        assert condition in CONDITIONS, f"Unknown condition: {condition}"
        self.stop_waveform()
        self.condition = condition
        if self.enable_gui:
            self.condition_button.label.set_text(f'Condition: {self.condition}')
            self.condition_desc_text.set_text(CONDITION_DESCRIPTIONS[self.condition])
        print(f"[CONDITION] Set to {self.condition}")

    def cleanup(self):
        self.stop_waveform()
        for ch in (self.voltage_output_0, self.voltage_output_1):
            if ch:
                try:
                    ch.setVoltage(0.0)
                    ch.close()
                except Exception:
                    pass
        print("[INFO] Cleanup complete")

    # -----------------------------------------------------------------------
    # ROS callback
    # FIX: Hilbert transform now computed here at full optic flow rate (~55 Hz)
    #      so /phase_mod and /amplitude_envelope are published at full rate.
    #      GUI updates still happen in update() at the slower display cadence.
    # -----------------------------------------------------------------------
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

        # FIX: compute + publish Hilbert at full callback rate once buffer is full
        if len(self.buffer) >= self.buffer_size:
            try:
                filtered = self.bandpass_filter(np.array(self.buffer))
                if not np.all(filtered == 0):
                    analytic_signal     = hilbert(filtered)
                    instantaneous_phase = np.unwrap(np.angle(analytic_signal))
                    amplitude_envelope  = np.abs(analytic_signal)

                    phase_mod = float(instantaneous_phase[-1] % (2 * np.pi))
                    env_val   = float(amplitude_envelope[-1])

                    self.instantaneous_phase_latest = phase_mod
                    self.envelope_latest            = env_val

                    self.phase_pub.publish(Float32(phase_mod))
                    self.envelope_pub.publish(Float32(env_val))
            except Exception as e:
                print(f"[ERROR] Hilbert in callback: {e}")

    # -----------------------------------------------------------------------
    # DSP
    # -----------------------------------------------------------------------
    def compute_fft(self, data):
        N          = len(data)
        fft_vals   = np.fft.fft(data)
        freqs      = np.fft.fftfreq(N, d=1.0 / self.sampling_rate)
        magnitudes = np.abs(fft_vals)
        pos_mask   = freqs >= 0
        return freqs[pos_mask], magnitudes[pos_mask]

    def bandpass_filter(self, data):
        nyq  = 0.5 * self.sampling_rate
        low  = self.lowcut  / nyq
        high = self.highcut / nyq
        b, a = butter(N=4, Wn=[low, high], btype='band')
        return filtfilt(b, a, data)

    # -----------------------------------------------------------------------
    # Waveform output
    # -----------------------------------------------------------------------
    def _set_voltages(self, v0, v1):
        """
        Write to both Phidget channels and publish to /piezo_voltages.
        FIX: publishes Float32MultiArray([v0, v1]) on every hardware write
        so the bag captures the exact actuation signal at full rate.
        """
        try:
            if self.voltage_output_0:
                self.voltage_output_0.setVoltage(v0)
            if self.voltage_output_1:
                self.voltage_output_1.setVoltage(v1)
        except Exception as e:
            print(f"[Voltage Error] {e}")

        # FIX: publish immediately after hardware write
        with self._voltage_lock:
            self._v0 = v0
            self._v1 = v1
        msg = Float32MultiArray()
        msg.data = [v0, v1]
        self.piezo_pub.publish(msg)

    def _emit_pulse(self, phase_offset_rad=None):
        """
        Fire one square-wave pulse of exactly PULSE_WIDTH seconds.
        phase_offset_rad: if None, uses self.phase_offset_degrees.
        """
        if phase_offset_rad is None:
            phase_offset_rad = self.phase_offset_degrees * math.pi / 180.0

        half    = PULSE_WIDTH / 2.0
        t_start = time.time()

        if phase_offset_rad < math.pi:
            self._set_voltages(4.2, 0.0)
        else:
            self._set_voltages(0.0, 4.2)

        time.sleep(half)

        if phase_offset_rad < math.pi:
            self._set_voltages(0.0, 4.2)
        else:
            self._set_voltages(4.2, 0.0)

        elapsed   = time.time() - t_start
        remaining = PULSE_WIDTH - elapsed
        if remaining > 0:
            time.sleep(remaining)

        self._set_voltages(0.0, 0.0)

    # -----------------------------------------------------------------------
    # Waveform runners
    # -----------------------------------------------------------------------
    def _run_closed_loop(self):
        start_time = time.time()
        last_print = start_time
        while self.waveform_running:
            t                = time.time() - start_time
            phase_offset_rad = self.phase_offset_degrees * math.pi / 180.0
            phase            = (2 * math.pi * self.waveform_freq * t + phase_offset_rad) % (2 * math.pi)
            v0, v1           = (4.2, 0.0) if phase < math.pi else (0.0, 4.2)
            self._set_voltages(v0, v1)
            if t - last_print >= 1.0 / max(self.waveform_freq, 1.0):
                print(f"[CLOSED_LOOP] f={self.waveform_freq:.2f}Hz | V0={v0:.1f} V1={v1:.1f} | phase_off={self.phase_offset_degrees:.0f}deg")
                last_print = t
            time.sleep(0.01)

    def _run_open_loop_54(self):
        start_time = time.time()
        last_print = start_time
        freq       = OPEN_LOOP_FREQ
        while self.waveform_running:
            t                = time.time() - start_time
            phase_offset_rad = self.phase_offset_degrees * math.pi / 180.0
            phase            = (2 * math.pi * freq * t + phase_offset_rad) % (2 * math.pi)
            v0, v1           = (4.2, 0.0) if phase < math.pi else (0.0, 4.2)
            self._set_voltages(v0, v1)
            if t - last_print >= 1.0 / freq:
                print(f"[OPEN_LOOP_54] f={freq:.2f}Hz (locked) | V0={v0:.1f} V1={v1:.1f}")
                last_print = t
            time.sleep(0.01)

    def _run_poisson(self):
        print(f"[POISSON] Running — pulse={PULSE_WIDTH*1000:.1f}ms, mean_IPI={PULSE_PERIOD*1000:.1f}ms, floor={MIN_IPI*1000:.1f}ms")
        pulse_count = 0
        while self.waveform_running:
            self._emit_pulse()
            pulse_count += 1
            ipi = max(MIN_IPI, random.expovariate(1.0 / PULSE_PERIOD))
            gap = ipi - PULSE_WIDTH
            print(f"[POISSON] pulse #{pulse_count} | next IPI={ipi*1000:.1f}ms | gap={gap*1000:.1f}ms")
            if gap > 0 and self.waveform_running:
                time.sleep(gap)

    def _run_uniform_jitter(self):
        half_period = PULSE_PERIOD / 2.0
        print(f"[UNIFORM_JITTER] Running — pulse={PULSE_WIDTH*1000:.1f}ms, IPI=185±{half_period*1000:.1f}ms")
        pulse_count = 0
        while self.waveform_running:
            self._emit_pulse()
            pulse_count += 1
            ipi = max(MIN_IPI, random.uniform(PULSE_PERIOD - half_period, PULSE_PERIOD + half_period))
            gap = ipi - PULSE_WIDTH
            print(f"[UNIFORM_JITTER] pulse #{pulse_count} | IPI={ipi*1000:.1f}ms | gap={gap*1000:.1f}ms")
            if gap > 0 and self.waveform_running:
                time.sleep(gap)

    def _run_phase_randomized(self):
        print(f"[PHASE_RAND] Running — exact 5.4 Hz, phase randomized each trigger event")
        event_phase_rad = random.uniform(0, 2 * math.pi)
        print(f"[PHASE_RAND] Event phase offset = {math.degrees(event_phase_rad):.1f} deg")
        pulse_count = 0
        while self.waveform_running:
            self._emit_pulse(phase_offset_rad=event_phase_rad)
            pulse_count += 1
            gap = PULSE_PERIOD - PULSE_WIDTH
            if gap > 0 and self.waveform_running:
                time.sleep(gap)

    def _run_replay(self):
        """
        Replay condition: plays back a pre-recorded closed loop piezo signal.
        Loads the next .npz file from REPLAY_SIGNALS_DIR in alphabetical order,
        cycling through all 13 source recordings across samples.
        Timing is preserved from the original recording.
        """
        import glob as _glob

        signal_files = sorted(_glob.glob(
            os.path.join(REPLAY_SIGNALS_DIR, 'replay_*.npz')
        ))

        if not signal_files:
            print(f"[REPLAY] ERROR: No replay signal files found in {REPLAY_SIGNALS_DIR}")
            print(f"[REPLAY] Run extract_replay_signals.py first")
            return

        if not hasattr(self, '_replay_counter'):
            self._replay_counter = 0

        signal_path = signal_files[self._replay_counter % len(signal_files)]
        self._replay_counter += 1

        print(f"[REPLAY] Signal {self._replay_counter}/{len(signal_files)}: "
              f"{os.path.basename(signal_path)}")

        try:
            data   = np.load(signal_path, allow_pickle=True)
            times  = data['times']
            ch0    = data['ch0']
            ch1    = data['ch1']
            source = str(data['source_bag'])
        except Exception as e:
            print(f"[REPLAY] ERROR loading signal: {e}")
            return

        print(f"[REPLAY] Source: {os.path.basename(source)}")
        print(f"[REPLAY] {len(times)} messages over {times[-1]:.1f}s")

        t_start = time.time()

        for i in range(len(times)):
            if not self.waveform_running:
                break

            target_wall = t_start + float(times[i])
            now         = time.time()
            wait        = target_wall - now
            if wait > 0:
                time.sleep(wait)
            elif wait < -0.01:
                # More than 10ms behind — skip to catch up
                continue

            self._set_voltages(float(ch0[i]), float(ch1[i]))

        self._set_voltages(0.0, 0.0)
        print(f"[REPLAY] Playback complete")
    # -----------------------------------------------------------------------
    # Waveform start / stop
    # -----------------------------------------------------------------------
    def start_waveform(self, detected_freq=None):
        if self.condition == 'CLOSED_LOOP':
            if detected_freq is not None:
                self.waveform_freq = detected_freq
            if self.waveform_running:
                return
        else:
            if self.waveform_running:
                return

        self.waveform_running = True

        target = {
            'CLOSED_LOOP':      self._run_closed_loop,
            'OPEN_LOOP_54':     self._run_open_loop_54,
            'POISSON':          self._run_poisson,
            'UNIFORM_JITTER':   self._run_uniform_jitter,
            'PHASE_RANDOMIZED': self._run_phase_randomized,
            'REPLAY':           self._run_replay,
        }[self.condition]

        self.waveform_thread = threading.Thread(target=target)
        self.waveform_thread.daemon = True
        self.waveform_thread.start()
        print(f"[WAVEFORM] Started — condition={self.condition}")

    def stop_waveform(self):
        self.waveform_running = False
        if self.waveform_thread:
            self.waveform_thread.join(timeout=1.0)
            self.waveform_thread = None
        self._set_voltages(0.0, 0.0)

    # -----------------------------------------------------------------------
    # Stats & history
    # -----------------------------------------------------------------------
    def update_flutter_stats(self, magnitude, threshold_exceeded):
        current_time = rospy.get_time()
        if threshold_exceeded:
            if self.last_flutter_time is None:
                self.flutter_event_count += 1
                self.flutter_duration = 0.0
                print(f"[FLUTTER START] Event #{self.flutter_event_count}")
            else:
                dt = current_time - self.last_flutter_time
                self.flutter_duration   += dt
                self.total_flutter_time += dt
            self.last_flutter_time = current_time
        else:
            if self.last_flutter_time is not None:
                print(f"[FLUTTER STOP] Duration: {self.flutter_duration:.2f}s")
                self.flutter_duration = 0.0
            self.last_flutter_time = None

    def update_magnitude_history(self, magnitude, threshold_exceeded):
        current_time   = rospy.get_time() - self.start_time
        voltage_active = threshold_exceeded and self.control_enabled

        self.magnitude_history.append(magnitude)
        self.magnitude_time_stamps.append(current_time)
        self.threshold_exceeded_history.append(threshold_exceeded)
        self.control_state_history.append(self.control_enabled)
        self.voltage_active_history.append(voltage_active)

        if len(self.magnitude_history) > self.magnitude_history_size:
            self.magnitude_history.pop(0)
            self.magnitude_time_stamps.pop(0)
            self.threshold_exceeded_history.pop(0)
            self.control_state_history.pop(0)
            self.voltage_active_history.pop(0)

    def update_stats_display(self, dom_mag, dom_freq):
        if not self.enable_gui:
            return
        runtime     = rospy.get_time() - self.start_time
        flutter_pct = (self.total_flutter_time / runtime * 100) if runtime > 0 else 0
        s = (
            f"Condition: {self.condition}   |   "
            f"Flutter Events: {self.flutter_event_count}   |   "
            f"Flutter Time: {self.total_flutter_time:.1f}s ({flutter_pct:.1f}%)   |   "
            f"Mag: {dom_mag:.1f}   |   Freq: {dom_freq:.2f} Hz"
        )
        if self.last_flutter_time is not None:
            s += f"   |   Current Event: {self.flutter_duration:.1f}s"
        self.stats_text.set_text(s)

    # -----------------------------------------------------------------------
    # Main update loop — GUI display only, no longer responsible for Hilbert
    # -----------------------------------------------------------------------
    def update(self):
        if len(self.buffer) < self.buffer_size:
            return

        data = np.array(self.buffer)

        freqs, magnitudes = self.compute_fft(data)
        idx      = np.argmax(magnitudes[1:]) + 1
        dom_freq = freqs[idx]
        dom_mag  = magnitudes[idx]

        threshold_exceeded = dom_mag > self.flutter_threshold

        self.dom_freq_pub.publish(Float32(dom_freq))
        self.dom_mag_pub.publish(Float32(dom_mag))

        self.update_flutter_stats(dom_mag, threshold_exceeded)
        self.update_magnitude_history(dom_mag, threshold_exceeded)

        print(
            f"[INFO] Freq={dom_freq:.2f}Hz | Mag={dom_mag:.2f} | "
            f"Threshold={'EXCEEDED' if threshold_exceeded else 'OK'} | "
            f"Control={'ON' if self.control_enabled else 'OFF'} | "
            f"Condition={self.condition}"
        )

        # actuation decision
        if self.control_enabled and threshold_exceeded:
            if self.voltage_output_0 and self.voltage_output_1:
                self.start_waveform(detected_freq=dom_freq)
        else:
            self.stop_waveform()

        # GUI updates
        if not self.enable_gui:
            return

        self.update_stats_display(dom_mag, dom_freq)

        # Use latest Hilbert values computed in callback
        # (no recompute here — callback handles it at full rate)
        try:
            filtered = self.bandpass_filter(data)
        except Exception as e:
            print(f"[ERROR] Filter in update: {e}")
            return

        if np.all(filtered == 0):
            return

        try:
            analytic_signal    = hilbert(filtered)
            amplitude_envelope = np.abs(analytic_signal)
        except Exception as e:
            print(f"[ERROR] Hilbert in update: {e}")
            return

        self.line_fft.set_data(freqs, magnitudes)
        self.ax_fft.set_xlim(0, 20)
        self.ax_fft.set_ylim(0, np.max(magnitudes) * 1.1)

        self.line_env.set_data(np.arange(len(amplitude_envelope)), amplitude_envelope)
        self.ax_env.set_xlim(0, len(amplitude_envelope))
        self.ax_env.set_ylim(0, np.max(amplitude_envelope) * 1.2)

        if len(self.magnitude_history) > 1:
            self.ax_mag_hist.collections.clear()
            self.line_mag_hist.set_data(self.magnitude_time_stamps, self.magnitude_history)

            times      = np.array(self.magnitude_time_stamps)
            mags       = np.array(self.magnitude_history)
            thresh_arr = np.array(self.threshold_exceeded_history)
            ctrl_arr   = np.array(self.control_state_history)
            volt_arr   = np.array(self.voltage_active_history)

            cond_color = CONDITION_COLORS.get(self.condition, 'green')

            def _fill_segments(mask, color, alpha, label_str):
                indices = np.where(mask)[0]
                if len(indices) == 0:
                    return
                segs, s = [], indices[0]
                for i in range(1, len(indices)):
                    if indices[i] != indices[i-1] + 1:
                        segs.append((s, indices[i-1]))
                        s = indices[i]
                segs.append((s, indices[-1]))
                for k, (si, ei) in enumerate(segs):
                    self.ax_mag_hist.fill_between(
                        times[si:ei+1], 0, mags[si:ei+1],
                        alpha=alpha, color=color,
                        label=label_str if k == 0 else ""
                    )

            _fill_segments(volt_arr,              cond_color, 0.35, f'Control Active ({self.condition})')
            _fill_segments(thresh_arr & ~ctrl_arr, 'yellow',   0.3,  'Threshold Exceeded (Control OFF)')
            _fill_segments(~thresh_arr & ~ctrl_arr,'red',      0.12, 'Control Disabled')

            time_window  = 30
            current_time = self.magnitude_time_stamps[-1]
            self.ax_mag_hist.set_xlim(max(0, current_time - time_window), current_time + 1)

            mn, mx = min(self.magnitude_history), max(self.magnitude_history)
            pad    = (mx - mn) * 0.1 if mx > mn else 1
            self.ax_mag_hist.set_ylim(mn - pad, mx + pad)

            handles, labels = self.ax_mag_hist.get_legend_handles_labels()
            if handles:
                self.ax_mag_hist.legend(loc='upper left', fontsize=7)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


# ---------------------------------------------------------------------------
# Spin thread
# ---------------------------------------------------------------------------
def _ros_spin():
    rospy.spin()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Flutter Control Node')
    parser.add_argument('--condition', type=str, default='CLOSED_LOOP', choices=CONDITIONS)
    parser.add_argument('--no-gui',    action='store_true')
    args, _ = parser.parse_known_args()

    rospy.init_node('phase_lock_node', anonymous=True)
    node = PhaseLockNode(condition=args.condition, enable_gui=not args.no_gui)

    spin_thread = threading.Thread(target=_ros_spin)
    spin_thread.daemon = True
    spin_thread.start()

    print("\n" + "=" * 60)
    print(f"FLUTTER CONTROL NODE")
    print(f"Condition: {node.condition}")
    print(f"  {CONDITION_DESCRIPTIONS[node.condition]}")
    print(f"Open-loop freq: {OPEN_LOOP_FREQ} Hz | Pulse width: {PULSE_WIDTH*1000:.1f} ms")
    print(f"Threshold: {node.flutter_threshold}")
    print("=" * 60)
    print("FIX: /piezo_voltages published at hardware write rate")
    print("FIX: /phase_mod + /amplitude_envelope published at ~55 Hz")
    print("=" * 60 + "\n")

    if node.enable_gui:
        plt.show(block=False)

    try:
        while not rospy.is_shutdown():
            node.update()
            if node.enable_gui:
                plt.pause(0.1)
            else:
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.cleanup()
        runtime     = rospy.get_time() - node.start_time
        flutter_pct = (node.total_flutter_time / runtime * 100) if runtime > 0 else 0
        print(f"\nFinal Stats:")
        print(f"  Runtime:        {runtime:.1f}s")
        print(f"  Flutter Events: {node.flutter_event_count}")
        print(f"  Flutter Time:   {node.total_flutter_time:.1f}s ({flutter_pct:.1f}%)")
        print(f"  Condition:      {node.condition}")


if __name__ == '__main__':
    main()