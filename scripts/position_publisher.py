#!/usr/bin/env python
import rospy
from unittest_python.msg import Position_32, Position_64

def box_publisher():
    pub_32 = rospy.Publisher('camera_position_32', Position_32, queue_size=10)
    pub_64 = rospy.Publisher('camera_position_64', Position_64, queue_size=10)
    rospy.init_node('position_publisher', anonymous=True)
    message_32 = Position_32()
    message_32.x = 0.250360752237
    message_32.y = -0.0315066584742
    message_32.z = 1.29800144243

    message_64 = Position_64()
    message_64.x = 0.250360752237
    message_64.y = -0.0315066584742
    message_64.z = 1.29800144243

    rospy.loginfo("Publishing message.")
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub_32.publish(message_32)
        pub_64.publish(message_64)
        rate.sleep()

if __name__ == '__main__':
    try:
        box_publisher()
    except rospy.ROSInterruptException:
        pass