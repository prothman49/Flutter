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
        self.buffer_size = 256  # keep small for fast testing
        self.time_stamps = []

        self.sampling_rate = 92.0

        self.lowcut = 5.0
        self.highcut = 9.0

        rospy.Subscriber('/optic_flow_mean_vx', Float32, self.callback)

        # ✅ Only one plot: FFT
        plt.ion()
        self.fig, self.ax_fft = plt.subplots(figsize=(8, 4))

        self.line_fft, = self.ax_fft.plot([], [], lw=2)
        self.ax_fft.set_title("FFT Spectrum")
        self.ax_fft.set_xlabel("Frequency (Hz)")
        self.ax_fft.set_ylabel("Magnitude")

        self.phase_pub = rospy.Publisher('/phase_mod', Float32, queue_size=10)

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
        except Exception as e:
            print(f"[ERROR] Hilbert failed: {e}")
            return

        phase_mod = instantaneous_phase[-1] % (2 * np.pi)
        print(f"[INFO] Phase mod 2π: {phase_mod:.2f} rad")

        self.phase_pub.publish(Float32(phase_mod))

        # FFT plot update
        self.line_fft.set_data(freqs, magnitudes)
        self.ax_fft.set_xlim(0, 20)
        self.ax_fft.set_ylim(0, np.max(magnitudes) * 1.1)

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

