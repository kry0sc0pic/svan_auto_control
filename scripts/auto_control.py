#!/usr/bin/env python3
"""
Works by emulating keyboard_interface inputs from the svan_utils package: refer to `src/svan_utils/scripts/keyboard_interface.py` for the original script.
"""
import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node('auto_control')

key_data = Float32MultiArray()
key_data.data = [4,0,0,0,0,0,0,0,0] # trot gait
publisher = rospy.Publisher('/svan/io_interface',Float32MultiArray,queue_size=1)
print("Publishing Trot")
publisher.publish(key_data)

x_vel = 0.3 # norm scaled
y_vel = 1
max_h = 0.35
min_h = 0.15


if __name__ == '__main__':
    while not rospy.is_shutdown():
        rate = rospy.Rate(1)

        key_data.data[2] = x_vel
        publisher.publish(key_data)
        rate.sleep()