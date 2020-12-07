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
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3

from missions import initialize_missions

## New task: selecting a leader which give commands from the gcs to the rest of the group
# the command is applied by the group.
# How do we make the connections 
# Create a mission which accepts its leader

def start_missions():   
    rospy.init_node('destroyer')
    rate = rospy.Rate(10) 

    uav_count = 3
    # uav name : uav object
    uav_nameobject = {"uav{}".format(i) : UAV("uav{}".format(i)) for i in range(uav_count)}
    # uav name : uav mission
    drone_missions = {uav_name: None  for uav_name in uav_nameobject.keys()}
    
    for drone_index, uav_name in enumerate(uav_nameobject.keys()):
        uav = uav_nameobject[uav_name]
        pose = uav.get_current_pose().pose.position
        
        missions = [
                TakeOff(uav, rate),

                GoAndWait(uav, rate, 3,  drone_index * 5, 0, pose.z),
                #2 helix
                Helix(uav, rate, 5, 5, 3),
                Wait(uav, rate, 1.5),
                Helix(uav, rate, 5, 5, 3),
                
                #Go down
                GoAndWait(uav, rate, 3,  drone_index * 5, 0, pose.z),

                #2 helix
                Helix(uav, rate, 5, 5, 3),
                Wait(uav, rate, 1.5),
                Helix(uav, rate, 5, 5, 3),

                #Go down
                GoAndWait(uav, rate, 3,  drone_index * 5, 0, pose.z),
                
                #2 helix
                Helix(uav, rate, 5, 5, 3),
                Wait(uav, rate, 1.5),
                Helix(uav, rate, 5, 5, 3),
                
                GoAndWait(uav, rate, 3,  drone_index * 5, 0, pose.z),


                #GoAndWait(uav, rate, 3,  drone_index * 5,0, 5),
                #MakeCircle(uav, rate, 6, 2),
                #GoAndWait(uav, rate, 2,  drone_index * 5,0,10),
                #MakeCircle(uav, rate, 6, 2),
                #GoAndWait(uav, rate, 2,  drone_index * 5,0,10),
                #MakeCircle(uav, rate, 6, 2),
                #GoAndWait(uav, rate, 2,  drone_index * 5,0,10)
                
                ] 
        drone_missions[uav_name] = missions
        
    uav_threads = []
    drone_index = 0
    for uav_name in drone_missions.keys():
        uav_threads.append(UAVThread(drone_missions[uav_name]))
        drone_index += 1
        
    while input() != '.':
        time.sleep()
class UAVThread:
    def __init__(self, missions):
        self.missions = missions
        thread.start_new_thread(self.apply_missions, ())
    def apply_missions(self):
        time.sleep(5) # wait for subscribers to recieve first info
        for mission in self.missions:
            mission.execute_mission()



class NetworkedInomfo(object):
    def __init__(self, topic):
        self.topic_pub = rospy.Publisher(topic, String, queue_size = 10)
        self.topic_sub = rospy.Subscriber(topic, String, self._on_topic_recieve)
    
    # dont override these two:
    def _on_topic_recieve(self,topic):
        topic_buffer = DataBuffer.from_string(topic)
        self.deserialize_from_buffer(topic_buffer)

    def publish_data(self):
        buffer = DataBuffer()
        self.serialize_into_buffer(buffer)
        str_data = buffer.to_string()
        self.topic_pub.publish(str_data)

    # these two are for overriding:
    def serialize_into_buffer(self, buffer):
        raise Exception("NetworkedInfo does not implement required method")
    def deserialize_from_buffer(self, buffer);
        raise Exception("NetworkedInfo does not implement required method")

# my_vector_info.value = myVector
# my_vector_info.publish()

class VectorInfo(NetworkedInfo): 
    def __init__(self, topic):
        super(VectorInfo, self).__init__(topic)
        self.value = Vector3()

    def serialize_into_buffer(self, buffer):
        buffer.write_vector3(self.value)
    def deserialize_from_buffer(self, buffer);
        self.value = buffer.read_vector3()


class LeaderMission(object):
    def __init__(self, topic):
        

class FollowerMission:
    def __init__(self, topic):
        self.posinfo = VectorInfo("/network/uav0/position")
        self.alldrone_positions =[]
        for i in range(5):
        self.alldrone_positions.append(VectorInfo("/network/uav"+ str(i)+ "/position"))
        
    def mission_started(self)
        wait(0.5)
    
    def mission_loop(self):
        self.posinfo.vector3 = uav.position
        self.postinfo.publish()

        other_pos = self.other_pos.vector3
    


if __name__ == '__main__':
    try:
        start_missions()
    except rospy.ROSInterruptException:
        pass
