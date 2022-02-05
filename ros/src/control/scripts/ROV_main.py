#! /usr/bin/python3
import rospy
import enum
from shared_msgs.msg import controller_msg, thrust_command_msg, thrust_disable_inverted_msg, rov_velocity_command
from std_msgs.msg import Bool, Float32, Float64
from geometry_msgs.msg import Twist
class Coord(enum.Enum):
    ROV_Centric = 1
    POOL_Centric = 2
class Contr_Type(enum.Enum):
    Percent_Power = 1
    Thrust = 2

controller_percent_power = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
controller_tools_command = [0,0,0,0]
translation_Scaling = 3.2
rotation_Scaling = 1.5
mode_fine = True
fine_multiplier = 1.041
pid_enabled = False
pid_control_effort = None
last_control_effort_msg = None

def onLoop():
    #Thruster Control
    thrust_command = thrust_command_msg()
#    if pid_enabled == True:
#            controller_percent_power[2] = Float32(last_control_effort_msg)
    controller_percent_power[2] = Float32(last_control_effort_msg)
    thrust_command.desired_thrust = controller_percent_power
    thrust_command.isFine = mode_fine
    thrust_command.multiplier = fine_multiplier
    thrust_command_pub.publish(thrust_command)

def _is_pid_enabled(msg):
    pid_enabled = msg
    rospy.logdebug("PID is enabled")

def _control_effort_input(msg):
    last_control_effort_msg = msg

def _velocity_input(msg):
    global mode_fine, fine_multiplier
    controller_percent_power[0] = msg.twist.linear.x
    controller_percent_power[1] = msg.twist.linear.y
    controller_percent_power[2] = msg.twist.linear.z
    controller_percent_power[3] = msg.twist.angular.x
    controller_percent_power[4] = msg.twist.angular.y
    controller_percent_power[5] = msg.twist.angular.z
    mode_fine = msg.isFine
    fine_multiplier = msg.multiplier

if __name__ == '__main__':
    rospy.init_node('ROV_main')
    pid_status = rospy.Subscriber('/depth_control/pid_enable', Bool, _is_pid_enabled)
    pid_control_effort = rospy.Subscriber('/depth_control/control_effort', Float64, _control_effort_input)

    velocity_sub = rospy.Subscriber('/rov_velocity', rov_velocity_command,_velocity_input)
    thrust_command_pub = rospy.Publisher('/thrust_command', thrust_command_msg, queue_size=1)
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        onLoop()
        r.sleep()
