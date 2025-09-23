#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, hilbert
import threading

class PhaseLockNode:
    def __init__(self):
        print("[INFO] PhaseLockNode started")

        self.buffer = []
        self.buffer_size = 360
        self.time_stamps = []

        self.sampling_rate = 50

        self.lowcut = 5.0
        self.highcut = 9.0

        rospy.Subscriber('/optic_flow_mean_vx', Float32, self.callback)

        # figure with FFT and envelope plots
        plt.ion()
        self.fig, (self.ax_fft, self.ax_env) = plt.subplots(2, 1, figsize=(8, 6))

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

        self.phase_pub = rospy.Publisher('/phase_mod', Float32, queue_size=10)
        self.envelope_pub = rospy.Publisher('/amplitude_envelope', Float32, queue_size=10)

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

    def update(self):
        if len(self.buffer) < self.buffer_size:
            return

        data = np.array(self.buffer)

        freqs, magnitudes = self.compute_fft(data)

        idx = np.argmax(magnitudes[1:]) + 1
        dom_freq = freqs[idx]
        print(f"[INFO] Dominant Freq: {dom_freq:.2f} Hz")

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

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

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

if __name__ == '__main__':
    main()

