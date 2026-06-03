#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Live Flutter Control
Real-time control with GUI
"""
import argparse
import time
import threading

import rospy
from std_msgs.msg import Float32, Bool
from flutter_control_node import PhaseLockNode, CONDITIONS, CONDITION_DESCRIPTIONS

def main():
    parser = argparse.ArgumentParser(description='Live Flutter Control')
    parser.add_argument(
        '--condition',
        type=str,
        default='CLOSED_LOOP',
        choices=CONDITIONS,
        help='Control condition'
    )
    args, _ = parser.parse_known_args()

    rospy.init_node('flutter_live_control', anonymous=True)
    node = PhaseLockNode(condition=args.condition, enable_gui=True)

    t = threading.Thread(target=rospy.spin)
    t.daemon = True
    t.start()

    print("\n" + "="*60)
    print("LIVE FLUTTER CONTROL")
    print("="*60)
    print(f"Condition: {node.condition}")
    print(f"  {CONDITION_DESCRIPTIONS[node.condition]}")
    print("="*60)
    print("Default phase offset: 225 degrees")
    print("\nListening for phase commands on:")
    print("  /flutter_control/phase_offset  (Float32, degrees)")
    print("  /flutter_control/enable        (Bool)")
    print("\nTo set phase manually:")
    print("  rostopic pub /flutter_control/phase_offset std_msgs/Float32 'data: 90.0'")
    print("\nPress Ctrl+C to exit")
    print("="*60 + "\n")

    node.set_phase(225)

    def phase_callback(msg):
        node.set_phase(msg.data)
        print(f"[PHASE] Updated to {msg.data:.1f} deg")

    def enable_callback(msg):
        node.control_enabled = msg.data
        print(f"[CONTROL] {'ENABLED' if msg.data else 'DISABLED'}")

    rospy.Subscriber('/flutter_control/phase_offset', Float32, phase_callback)
    rospy.Subscriber('/flutter_control/enable', Bool, enable_callback)

    print("[INFO] Subscribers active")

    try:
        while not rospy.is_shutdown():
            node.update()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.cleanup()

if __name__ == '__main__':
    main()