#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
import threading

class LiveFFTNode:
    def __init__(self):
        self.buffer = []
        self.buffer_size = 256  # Try lowering to 128 for quicker plot
        self.sampling_rate = 92.0

        rospy.Subscriber('/optic_flow_mean_vx', Float32, self.callback)

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], lw=2)
        self.ax.set_xlabel('Frequency (Hz)')
        self.ax.set_ylabel('Magnitude')
        self.ax.set_title('Live FFT Spectrum')
        self.ax.set_xlim(0, self.sampling_rate / 2)
        self.ax.set_ylim(0, 10)

    def callback(self, msg):
        self.buffer.append(msg.data)
        if len(self.buffer) > self.buffer_size:
            self.buffer.pop(0)

    def update_plot(self):
        if len(self.buffer) < self.buffer_size:
            return
        data = np.array(self.buffer)
        fft_vals = np.fft.fft(data)
        freqs = np.fft.fftfreq(len(data), d=1.0 / self.sampling_rate)
        magnitudes = np.abs(fft_vals)

        pos_mask = freqs >= 0
        freqs = freqs[pos_mask]
        magnitudes = magnitudes[pos_mask]

        self.line.set_data(freqs, magnitudes)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        idx = np.argmax(magnitudes[1:]) + 1
        dominant_freq = freqs[idx]
        print(f"Dominant frequency: {dominant_freq:.2f} Hz")

def ros_thread():
    rospy.spin()

def main():
    rospy.init_node('live_fft_node', anonymous=True)
    node = LiveFFTNode()

    t = threading.Thread(target=ros_thread)
    t.daemon = True
    t.start()

    plt.show(block=False)

    try:
        while not rospy.is_shutdown():
            node.update_plot()
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

