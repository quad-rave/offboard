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
from tf.transformations import quaternion_from_euler
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandBool
from std_msgs.msg import String
from mavros import command
#from mavros_msgs.msg import 
#from mavsdk import System
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from buffer import DataBuffer
from missions import *

class MissionThread:
    def __init__(self, mission, uav, rate):
        self.mission = mission
        self.rate = rate
        self.uav = uav
        thread.start_new_thread(self.apply_mission, ())
    def apply_mission(self):
        time.sleep(1) # wait for subscribers to recieve first info
        self.mission.execute_mission(self.uav, self.rate)

class UAV(object):
    # access uav internal topics
    # handle missions
    def __init__(self, uav_name):
        self.uav_name = uav_name
        self.pub_state = rospy.Publisher("sqr_state", String, queue_size = 10)
        self.pub_pose = rospy.Publisher(uav_name +'/mavros/setpoint_position/local', SP.PoseStamped, queue_size=10)
        self.sub_pose = rospy.Subscriber(uav_name + '/mavros/local_position/pose', SP.PoseStamped, self._read_position_from_topic)
        self.pub_twist =  rospy.Publisher(uav_name + '/mavros/setpoint_velocity/cmd_vel_unstamped',TwistStamped, queue_size=10)

        self.offb_set_mode = SetMode()
        self.arming_cl = rospy.ServiceProxy(uav_name +'/mavros/cmd/arming', CommandBool)
        self.takeoff_cl = rospy.ServiceProxy(uav_name +'/mavros/cmd/takeoff', CommandTOL)
        self.change_mode = rospy.ServiceProxy(uav_name +'/mavros/set_mode', SetMode)

        self.sub_mission = rospy.Subscriber(uav_name + '/mission_assign', String, self._add_mission_from_topic)
        print("b")
        #a = SP.PoseStamped()
        #a.pose.position.x
        # current physical position
        self.pose_stamped =  SP.PoseStamped(
            header=SP.Header(
            frame_id="base_footprint",  # no matter, plugin don't use TF
            stamp=rospy.Time.now()),    # stamp should update
        )
        self._last_target_pose = None
        self.rate = rospy.Rate(10) 
        self.active_mission_threads = []
        print("c")


    def _add_mission_from_topic(self, topic):
        print(self.uav_name + " recieved a mission")
        buffer = DataBuffer.from_string(topic)
        mission_factory = MissionFactory()
        mission = mission_factory.mission_from_buffer(buffer)
        mission_thread = MissionThread(mission, self, self.rate)
        self.active_mission_threads.append(mission_thread)

    # assign mission to any uav, to used by leader by any uav can assign any mission to any other uav
    def assign_mission(self, mission, uav_name):
        buffer = DataBuffer()
        mission_factory = MissionFactory()
        mission_factory.mission_into_buffer(buffer,mission)
        msg = buffer.to_string()
        mission_pub = rospy.Publisher(uav_name +'/mission_assign', String, queue_size=10)
        mission_pub.publish(msg)

    def _read_position_from_topic(self, topic):
        self.pose_stamped = topic
    
    def get_last_target_pose(self):
        return self._last_target_pose

    def get_current_time(self):
        return rospy.rostime.Time.now()

    def get_current_pose(self):
        return self.pose_stamped


    def set_target_velocity(self, x,y,z):
        msg = TwistStamped()
        msg.twist.linear = Vector3(x,y,z)
        self.pub_twist.publish(msg)

    # send target position to drone
    def set_target_pose(self,x,y,z):

        msg = SP.PoseStamped(
            header=SP.Header(
            frame_id="base_footprint",  # no matter, plugin don't use TF
            stamp=rospy.Time.now()),    # stamp should update
        )

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation = SP.Quaternion(*quaternion)
        self._last_target_pose = msg
        self.pub_pose.publish(msg)
        

    def arm(self,bool):
        rospy.wait_for_service(self.uav_name + '/mavros/cmd/arming')
        response = self.arming_cl(value = True)
        #rospy.loginfo(response)
    def set_offboard(self):
        rospy.wait_for_service(self.uav_name + '/mavros/set_mode')
        response = self.change_mode(custom_mode="OFFBOARD")
        #rospy.loginfo(response)