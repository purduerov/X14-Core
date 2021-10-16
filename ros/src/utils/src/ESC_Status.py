#! /usr/bin/python3

import sys

import can

import rospy
from shared_msgs.msg import esc_test_data_msg

lastEsc = [0] * 3

def sendEscData(can_bus):
    pub = rospy.Publisher('ESCdata', esc_test_data_msg, queue_size=10) # Modify queue size based on rate
    rospy.init_node('ESC_Logger', anonymous=True)
    rate = rospy.Rate(1) # Figure out what rate to use
    
    while not rospy.is_shutdown():
        for can_rx in can_bus:
            if not rospy.is_shutdown():
                can_id = can_rx.arbitration_id

                if (0x301 <= can_id <= 0x303): 
                    msg = esc_test_data_msg()
                    msg.escNum  = lastEsc[can_id - 0x301] # Add ESC Number
                    data = list(can_rx.data) # Get data from ESC
                    msg.temperature = data[0]
                    msg.voltage = data[1] / 255 + 10
                    msg.current = (data[2] << 8 + data[3]) / 100
                    msg.energy = (data[4] << 8 + data[5])
                    msg.speed = (data[6] << 8 + data[7]) * 100

                    # Send information from node
                    rospy.loginfo(msg)
                    pub.publish(msg)
                    rate.sleep()

def main(args: list) -> None:
    """"""
    channel = "can0"
    can_bus = can.interface.Bus(channel = channel, bustype = 'socketcan')
    sendEscData(can_bus)

if(__name__ == "__main__"):
    try:
        main(sys.argv)
    except ROSInterruptException:
        pass
