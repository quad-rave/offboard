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
# from mavros_msgs.msg import
# from mavsdk import System
from geometry_msgs.msg import TwistStamped, Twist
from geometry_msgs.msg import Vector3
from buffer import DataBuffer
from missions import *
import numpy as np
from mission_factory import MissionFactory
from utility import *

class MissionThread:
    def __init__(self, mission, uav, rate):
        self.mission = mission
        self.rate = rate
        self.uav = uav
        thread.start_new_thread(self.apply_mission, ())

    def apply_mission(self):
        time.sleep(1)  # wait for subscribers to recieve first info
        self.mission.execute_mission(self.uav, self.rate)


class UAV(object):
    # access uav internal topics
    # handle missions
    def __init__(self, uav_name):
        self.uav_name = uav_name
        self.pub_state = rospy.Publisher("sqr_state", String, queue_size=10)
        self.pub_pose = rospy.Publisher(uav_name + '/mavros/setpoint_position/local', SP.PoseStamped, queue_size=10)
        self.sub_pose = rospy.Subscriber(uav_name + '/mavros/local_position/pose', SP.PoseStamped,
                                         self._read_position_from_topic)
        self.pub_twist = rospy.Publisher(uav_name + '/mavros/setpoint_velocity/cmd_vel_unstamped', Twist,
                                         queue_size=10) # type was TwistStamped, it wa causing error but was still working
        self.sub_vel = rospy.Subscriber(uav_name + '/mavros/local_position/velocity_local', TwistStamped,
                                                                    self._read_velocity_from_topic)
        self.offb_set_mode = SetMode()
        self.arming_cl = rospy.ServiceProxy(uav_name + '/mavros/cmd/arming', CommandBool)
        self.takeoff_cl = rospy.ServiceProxy(uav_name + '/mavros/cmd/takeoff', CommandTOL)
        self.change_mode = rospy.ServiceProxy(uav_name + '/mavros/set_mode', SetMode)

        self.sub_mission = rospy.Subscriber(uav_name + '/mission_assign', String, self._add_mission_from_topic)
        print("listening to mission assign: " + uav_name + "/mission_assign")
        # a = SP.PoseStamped()
        # a.pose.position.x
        # current physical position
        self.pose_stamped = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),  # stamp should update
        )
        self.vel_read_topic = Twist()
        self._last_target_pose = None
        self.rate = rospy.Rate(10)
        self.active_mission_threads = []
        self.initial_pos = vec(0.0,0.0,0.0)
        print("c")

        
    def _add_mission_from_topic(self, topic):
        print(self.uav_name + " recieved a mission")
        buffer = DataBuffer.from_string(topic.data)
        mission_factory = MissionFactory()
        mission = mission_factory.mission_from_buffer(buffer)
        mission_thread = MissionThread(mission, self, self.rate)
        self.active_mission_threads.append(mission_thread)

    # assign mission to any uav, to beused by leader by any uav can assign any mission to any other uav
    def assign_mission(self, mission, uav_name):
        buffer = DataBuffer()
        mission_factory = MissionFactory()
        mission_factory.mission_into_buffer(buffer, mission)
        msg = buffer.to_string()
        print("try assign mission")
        mission_pub = rospy.Publisher(uav_name + '/mission_assign', String, queue_size=10)
        time.sleep(1)
        
        mission_pub.publish(msg)
        print("published assign mission: " + uav_name + '/mission_assign' + ", data: " + msg)
        #time.sleep(2)

    def _read_velocity_from_topic(self, topic):
        #print("calback received!*---------------------------------------------")
        self.vel_read_topic = topic
    def _read_position_from_topic(self, topic):
        self.pose_stamped = topic
        

    def get_last_target_pose(self):
        return np.array([self._last_target_pose.pos.x,
                        self._last_target_pose.pos.y,
                        self._last_target_pose.pos.z])

    def get_current_time(self):
        return rospy.rostime.Time.now()

    def get_current_pose(self): # this should return vector3
        return np.array([self.pose_stamped.pose.position.x,
                                self.pose_stamped.pose.position.y, 
                                self.pose_stamped.pose.position.z]) + self.initial_pos

    def get_current_velocity(self):
        return np.array([self.vel_read_topic.twist.linear.x,  
                            self.vel_read_topic.twist.linear.y,
                            self.vel_read_topic.twist.linear.z])

    def set_target_velocity(self, vec): # this should work with vectors
        msg = Twist()
        msg_vel = Vector3(vec[0],vec[1],vec[2])
        msg.linear = msg_vel

        print("vel control")
        print(msg)
        self.pub_twist.publish(msg)

    # send target position to drone
    def set_target_pose(self, vec):
        msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),  # stamp should update
        )
        msg.pose.position.x = vec[0] - self.initial_pos[0]
        msg.pose.position.y = vec[1] - self.initial_pos[1]
        msg.pose.position.z = vec[2] - self.initial_pos[2]
        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation = SP.Quaternion(*quaternion)
        self._last_target_pose = msg
        #print("pos control")
        #print(msg)
        self.pub_pose.publish(msg)

    def arm(self, bool):
        rospy.wait_for_service(self.uav_name + '/mavros/cmd/arming')
        response = self.arming_cl(value=True)
        # rospy.loginfo(response)

    def set_offboard(self):
        rospy.wait_for_service(self.uav_name + '/mavros/set_mode')
        response = self.change_mode(custom_mode="OFFBOARD")
        # rospy.loginfo(response)
