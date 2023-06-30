#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32
from esp32_universal_remote.msg import UR_Data

def data_callback(msg):
    # Split the received data into individual components
    data = msg.data.split(" ")
    float_data=[]
    for i in range(0,10):
        float_data.append(float(data[i]))

    # Publish the normalized value as control_data topic
    control_data_pub.publish(float_data)

rospy.init_node('data_splitter_node')

# Create a publisher for the control_data topic
control_data_pub = rospy.Publisher('control_data', UR_Data, queue_size=10)

# Subscribe to the ur_data topic
rospy.Subscriber('ur_data', String, data_callback)

rospy.spin()
