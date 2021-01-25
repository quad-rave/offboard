#!/usr/bin/env python2
# vim:set ts=4 sw=4 et:

import rospy
import _thread as thread
import threading
import time
import mavros

from math import *
from mavros.utils import *
from mavros import setpoint as SP
#from tf.transformations import quaternion_from_euler
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandBool
from std_msgs.msg import String
from mavros import command
# from mavros_msgs.msg import
# from mavsdk import System
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from missions import *
from uav import UAV
from mission_factory import MissionFactory
from triangle_missions import TriangleLeader
from triangle_missions_decentralized import *
from utility import *
import signal

def interupt_handler(sig_num, frame):
    print ("Exiting")
    sys.exit(-2) #Terminate process here as catching the signal removes the close process behavior of Ctrl-C

def hmm(topic):
    print("hmmmmmmmm: " + topic.data)
def start_missions():
    rospy.init_node('quadrave')
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
    rate = rospy.Rate(10)

    # define the UAV objects
    uavs = []
    for i in range(6):
        uav = UAV("uav" + str(i))
        uavs.append(uav)
    time.sleep(3)

    # define a mission and assign it to uavs
    member_mission = TriangleMember()
    member_mission.uav_count = 6


    formation_points_star = []
    step = 2 * pi / 6
    radius = 6
    angle = 0.0 - 2*pi * (-30.0/360.0)
    for i in range(6):
        step_radius = radius
        if(i % 2 == 1):
            step_radius = radius * 0.5
        formation_points_star.append(vec(cos(step * i) * step_radius * 1.6, sin(step * i )* step_radius, 5))


    formation_points_hex = []
    step = 2 * pi / 6
    radius = 12
    angle = 0.0
    for i in range(6):
        step_radius = radius
        formation_points_hex.append(vec(cos(step * i) * step_radius ,sin(step * i) * step_radius,5) )
        
        
    formation_points_line = [np.array([0,  5, 0]), np.array([5,  5, 0]),
                        np.array([10, 5, 0]), np.array([15, 5, 0]), 
                        np.array([20, 5, 0]), np.array([25, 5, 0]), ]

    formation_points_rect = [np.array([0,5, 0]), np.array([0,10, 0]),
                        np.array([5,5, 0]), np.array([5,10,0]), 
                        np.array([10,5, 0]), np.array([10,10, 0]), ]
    for i in range(len(formation_points_rect)):
        formation_points_rect[i] = formation_points_rect[i] * 1.6



    
    for i in range(6):
        member_mission.uav_id = i    
        uavs[i].assign_mission(member_mission, ("uav" + str(i)))

    time.sleep(2)
    formation_points_info = NetworkedInfo.get_or_create("/triangle_mission/formation_points", [])
    formation_targetpos = NetworkedInfo.get_or_create("/triangle_mission/formation_targetpos", vec())
    formation_targetrot = NetworkedInfo.get_or_create("/triangle_mission/formation_targetrot", FormationRotationData())

    time.sleep(2)
    
    formation_points_info.set_data(formation_points_star)
    formation_targetpos.set_data(vec(0.0,0.0,15.0))
    time.sleep(4)

    i = 0
    while True:
        formation_targetpos.set_data(vec(0.0,0.0,15.0))
        time.sleep(10)

        formation_targetpos.set_data(vec(20.0,0.0,15.0))
        time.sleep(6)

        formation_targetrot.set_data(FormationRotationData('Z', 10, 0.5 * math.pi))
        time.sleep(10)

        formation_targetpos.set_data(vec(20.0,20.0,15.0))
        time.sleep(6)

        formation_targetrot.set_data(FormationRotationData('Z', 10, 0.5 * math.pi))
        time.sleep(10)

        formation_targetpos.set_data(vec(0.0,20.0,15.0))
        time.sleep(6)

        formation_targetrot.set_data(FormationRotationData('Z', 10, 0.5 * math.pi))
        time.sleep(10)

        i += 1



    a = input()
    #signal.signal(signal.SIGINT, interupt_handler)

    
    return



if __name__ == '__main__':
    try:
        start_missions()
    except rospy.ROSInterruptException:
        pass

