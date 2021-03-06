#! /usr/bin/python3
import rospy
import enum
from shared_msgs.msg import controller_msg, thrust_command_msg, thrust_disable_inverted_msg, tools_command_msg, rov_velocity_command
from std_msgs.msg import Bool
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

def onLoop():
    #Thruster Control
    thrust_command = thrust_command_msg()
    thrust_command.desired_thrust = controller_percent_power
    thrust_command_pub.publish(thrust_command)
    #Tools Control
    tools_command = tools_command_msg()
    tools_command.pm = controller_tools_command[0]
    tools_command.ghost = controller_tools_command[1]
    tools_command.secondary = controller_tools_command[2]
    tools_command_pub.publish(tools_command)

def _velocity_input(msg):
    controller_percent_power[0] = msg.twist.linear.x
    controller_percent_power[1] = msg.twist.linear.y
    controller_percent_power[2] = msg.twist.linear.z
    controller_percent_power[3] = msg.twist.angular.x
    controller_percent_power[4] = msg.twist.angular.y
    controller_percent_power[5] = msg.twist.angular.z

def _controller_input(contr):
    controller_percent_power[0] = contr.LY_axis * translation_Scaling # translational
    controller_percent_power[1] = contr.LX_axis * translation_Scaling * .5 # translation
    controller_percent_power[2] = ((contr.Rtrigger) - (contr.Ltrigger)) * translation_Scaling 
    if contr.a == 1:
        controller_percent_power[3] = 1 * rotation_Scaling
    elif contr.b == 1:
        controller_percent_power[3] = -1 * rotation_Scaling
    else:
        controller_percent_power[3] = 0.0
    controller_percent_power[4] = -contr.RY_axis * rotation_Scaling * .5# pitch
    controller_percent_power[5] = contr.RX_axis * rotation_Scaling # yaw
    if contr.x == 1:
        controller_tools_command[0] = 1
    if contr.y ==1:
        controller_tools_command[0] = 0
def _pm_set_state(msg):
    if(msg.data):
        controller_tools_command[0] = 1
    else:
        controller_tools_command[0] = 0
def _gh_set_state(msg):
    if(msg.data):
        controller_tools_command[1] = 1
    else:
        controller_tools_command[1] = 0
def _bs_set_state(msg):
    if(msg.data):
        controller_tools_command[2] = 1
    else:
        controller_tools_command[2] = 0

if __name__ == '__main__':
    rospy.init_node('ROV_main')
    velocity_sub = rospy.Subscriber('/rov_velocity', rov_velocity_command,_velocity_input)
    pm_sub = rospy.Subscriber('/pm_cmd',Bool, _pm_set_state)
    gh_sub = rospy.Subscriber('/gh_cmd',Bool, _gh_set_state)
    bs_sub = rospy.Subscriber('/bs_cmd',Bool, _bs_set_state)
    #controller_sub = rospy.Subscriber('/gamepad_listener', controller_msg,_controller_input)
    thrust_command_pub = rospy.Publisher('/thrust_command', thrust_command_msg, queue_size=1)
    tools_command_pub = rospy.Publisher('/tools_proc', tools_command_msg, queue_size=10)
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        onLoop()
        r.sleep()
