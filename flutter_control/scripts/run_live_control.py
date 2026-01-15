#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Live Flutter Control
Real-time control with GUI
"""

import rospy
import threading
import matplotlib.pyplot as plt
from flutter_control_node import PhaseLockNode

def ros_spin():
    rospy.spin()

def main():
    rospy.init_node('flutter_live_control', anonymous=True)
    node = PhaseLockNode(enable_gui=True)

    t = threading.Thread(target=ros_spin)
    t.daemon = True
    t.start()

    plt.show(block=False)
    
    print("\n" + "="*60)
    print("LIVE FLUTTER CONTROL")
    print("="*60)
    print("Use GUI to:")
    print("  - Adjust phase offset slider")
    print("  - Toggle control on/off")
    print("  - Monitor flutter in real-time")
    print("\nPress Ctrl+C to exit")
    print("="*60 + "\n")

    try:
        while not rospy.is_shutdown():
            node.update()
            plt.pause(0.1)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        node.cleanup()

if __name__ == '__main__':
    main()