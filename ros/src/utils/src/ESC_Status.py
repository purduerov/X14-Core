#! /usr/bin/python3

import sys

import can

import rospy
from shared_msgs.msg import esc_test_data_msg

lastEsc = [0] * 3

def sendEscData(can_bus):
    pub = rospy.Publisher('rov/ESCdata', esc_test_data_msg, queue_size=10) # Modify queue size based on rate
    rospy.init_node('ESC_Logger', anonymous=True)
    rate = rospy.Rate(1) # Figure out what rate to use
    
    while not rospy.is_shutdown():
        for can_rx in can_bus:
            if rospy.is_shutdown():
                break
            can_id = can_rx.arbitration_id
            if (0x301 <= can_id <= 0x303): 
                lastEsc_ = lastEsc[can_id - 0x301]
                lastEsc[can_id - 0x301] = (lastEsc_ + 1) % 4
                data = list(can_rx.data) # Get data from ESC
                msg = esc_test_data_msg()
                msg.escNum  = lastEsc_ # Add ESC Number
                msg.temperature = data[0] # Add temperature data (deg C)
                msg.voltage = data[1] / 255 + 10 # Add voltage data (V)
                msg.current = (data[2] << 8 + data[3]) / 100 # Add current data (A)
                msg.energy = (data[4] << 8 + data[5]) # Add energy data (mAHr)
                msg.speed = (data[6] << 8 + data[7]) * 100 # Add speed data (erpm)

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
