#!/usr/bin/env python2
# vim:set ts=4 sw=4 et:

# this is for type serialization/deserialization purposes
from buffer import DataBuffer
#from uav import UAV
from networked_info import *
from utility import TwoWayDict

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from math import *

from missions import Mission


class TriangleLeader(Mission):
    
    def __init__(self, ):
        # these are the NetworkedInfos for sending vector3 data to slaves
        self.target_pos_infos = []
        self.point_cloud = []
        self.uav_count = None

    def deserialize_data_from_buffer(self, buffer):
        #self.target_position = buffer.read_vector3()
        size = buffer.read_int()
        for i in range(size):
            self.point_cloud.append(buffer.read_vector3())
        self.uav_count(buffer.read_int())
        
    def serialize_data_into_buffer(self, buffer):
        #buffer.write_vector3(self.target_position)
        buffer.write_int(len(self.point_cloud))
        for (vector in self.point_cloud):
            buffer.write_vector3(vector)
        buffer.write_int(self.uav_count)

    
    def mission_started(self,uav,rate):
        print("leader mission started")
        super(TriangleLeader, self).mission_started(uav, rate)

        # define a slave mission
        slave_mission = TriangleSlave()
        slave_mission.uav_count = self.uav_count
        for i in range(self.uav_count):
            slave_name = "uav" + str(i)
            slave_mission.uav_id = i 

            # define the vector NetworkedInfo
            topic = slave_name + "/triangle_mission/position_target"
            position_target_info = VectorInfo(topic)

            self.target_pos_infos.append(position_target_info)

            # give the slave mission to uav
            uav.assign_mission(slave_mission, slave_name)


    def mission_loop(self, uav, rate):
        for i in range(self.uav_count):
            # get the NetworkedInfo
            position_target_info = self.target_pos_infos[i]
            # set its value
            position_target_info.value = self.point_cloud[i]
            # publish, next time you see that value, we are on slave mission
            position_target_info.publish_data()




class TriangleSlave(Mission):
    def __init__(self):
        self.setpoint = SetpointPosition()
        self.uav_count = None
        self.uav_id = None
        self.uav_pos_infos = []
        self.target_pos_info = None

    def deserialize_data_from_buffer(self, buffer):
        #self.target_position = buffer.read_vector3()
        self.uav_id = buffer.read_int()
        self.uav_count = buffer.read_int()
        
    def serialize_data_into_buffer(self, buffer):
        #buffer.write_vector3(self.target_position)
        buffer.write_int(self.uav_id)
        buffer.write_int(self.uav_count)
        
    def mission_started(self,uav,rate):
        print("slave mission started")
        super(TriangleSlave, self).mission_started(uav, rate)

        topic = uav.uav_name + "/triangle_mission/position_target"
        self.target_pos_info = VectorInfo(topic)

        for i in range(self.uav_count):
            self.uav_pos_infos.append(VectorInfo())

        takeoff = TakeOff()
        takeoff.execute_mission(uav, rate)


    def mission_loop(self, uav, rate):
        uav_pos_infos[self.uav_id].value = uav.get_current_pose() # change for the Vector3 ds
        uav_pos_infos[self.uav_id].publish_data()
        
        #uav.set_target_velocity = 



        
def get_collision_avoidance_u(selfpos, otherpos):
    #u_ca(i) = alpha * sum(exp(- beta * abs(r_ij)))
    distance = np.linalg.norm(selfpos, otherpos)
    u = alpha * math.exp(- beta * abs(distance))
    return u

def get_position_u(selfpos, targetpos):
    #u_f(i) = kp*sum(p(j) - p(i) - delta_ij) 
    