#!/usr/bin/env python3

import json
#ROS
import rospy
from std_msgs.msg import Float32

temp = 1

def _temperature(tempdata):
    global temp
    temp = tempdata.data

    print(json.dumps(temp))

if __name__ == '__main__':
    rospy.init_node('temperature_surface')
    stat = rospy.Subscriber('/rov/depth', Float32, _temperature)
    rate = rospy.Rate(2) #Get twice per Second

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()