#!/usr/bin/env python3

import json
#ROS
import rospy
from shared_msgs.msg import temp_msg

temp = 1

def _temperature(tempdata):
    global temp
    temp = tempdata

    print(json.dumps(temp))

if __name__ == '__main__':
    rospy.init_node('temperature_surface')
    stat = rospy.Subscriber('/rov/Pi_TEMP', temp_msg, _temperature)

    while not rospy.is_shutdown():
        rospy.spin()
