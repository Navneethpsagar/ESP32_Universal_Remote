#!/usr/bin/env python

import rospy
from esp32_universal_remote.msg import UR_Data  # Replace 'your_package_name' with the actual package name
import sys, select, termios, tty

# Define the joint names and key mappings
joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
key_bindings = {
    '1': (0, 1),   # Joint 1: Increase angle
    'q': (0, -1),  # Joint 1: Decrease angle
    'a': (0, 0),   # Joint 1: Set zero
    '2': (1, 1),   # Joint 2: Increase angle
    'w': (1, -1),  # Joint 2: Decrease angle
    's': (1, 0),   # Joint 2: Set zero
    '3': (2, 1),   # Joint 3: Increase angle
    'e': (2, -1),  # Joint 3: Decrease angle
    'd': (2, 0),   # Joint 3: Set zero
    '4': (3, 1),   # Joint 4: Increase angle
    'r': (3, -1),  # Joint 4: Decrease angle
    'f': (3, 0),   # Joint 4: Set zero
    '5': (4, 1),   # Joint 5: Increase angle
    't': (4, -1),  # Joint 5: Decrease angle
    'g': (4, 0),   # Joint 5: Set zero
    '6': (5, 1),   # Joint 6: Increase angle
    'y': (5, -1),  # Joint 6: Decrease angle
    'h': (5, 0),   # Joint 6: Set zero
    'z': (-1, 0),  # Set all joints to 0
}

def getKey():
    # Get the pressed key from the user
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return key

def universal_remote():
    # Initialize the ROS node and publisher for the joint angles
    rospy.init_node('universal_remote')
    publisher = rospy.Publisher('ur_data', UR_Data, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        key = getKey()

        if key in key_bindings:
            # Update the joint angles based on the key
            joint_index, direction = key_bindings[key]
            if joint_index >= 0:
                # Create a JointAngles message and populate the angles array
                joint_angles_msg = UR_Data()
                joint_angles_msg.ur_data = [0] * len(joint_names)
                joint_angles_msg.ur_data[joint_index] = direction

                # Publish the joint angles message
                publisher.publish(joint_angles_msg)
        elif key == '\x03':  # Ctrl-C
            break
        elif key == '0':
            # Set all joints to 0
            joint_angles_msg = UR_Data()
            joint_angles_msg.ur_data = [0] * len(joint_names)
            publisher.publish(joint_angles_msg)

        rate.sleep()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        universal_remote()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
