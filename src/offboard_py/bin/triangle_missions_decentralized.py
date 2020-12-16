#!/usr/bin/env python2
# vim:set ts=4 sw=4 et:

# this is for type serialization/deserialization purposes
from buffer import DataBuffer
#from uav import UAV
from networked_info import *
from utility import TwoWayDict
import numpy as np
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from math import *

from missions import *



class TriangleMember(Mission):
    def __init__(self):
        self.setpoint = SetpointPosition()
        self.uav_count = None
        self.uav_id = None
        self.uav_pos_infos = []
        self.point_cloud = []
        self.u_integral = np.array([0.0,0.0,0.0])

    def deserialize_data_from_buffer(self, buffer):
        #self.target_position = buffer.read_vector3()
        self.uav_id = buffer.read_int()
        self.uav_count = buffer.read_int()

        size = buffer.read_int()
        for i in range(size):
            self.point_cloud.append(buffer.read_vector3())

        
        
    def serialize_data_into_buffer(self, buffer):
        #buffer.write_vector3(self.target_position)
        buffer.write_int(self.uav_id)
        buffer.write_int(self.uav_count)

        buffer.write_int(len(self.point_cloud))
        for vector in self.point_cloud:
            buffer.write_vector3(vector)
        
    def mission_started(self,uav,rate):
        print("slave mission started")
        super(TriangleMember, self).mission_started(uav, rate)

        for i in range(self.uav_count):
            #self.uav_pos_infos.append(VectorInfo(topic))
            self.uav_pos_infos.append(VectorInfo("uav" + str(i) + "/triangle_mission/position_target"))


        takeoff = TakeOff()
        takeoff.execute_mission(uav, rate)


    def mission_loop(self, uav, rate):
        self.uav_pos_infos[self.uav_id].value = uav.get_current_pose() # change for the Vector3 ds
        self.uav_pos_infos[self.uav_id].publish_data()
        print("my id: ", self.uav_id)

        collision_avoidance_u = np.array([0.,0.,0.])
        i = 0
        for uav_pos_info in self.uav_pos_infos:
            if i != self.uav_id:
                my_pos = uav.get_current_pose()
                others_pos = uav_pos_info.value

                u = self.get_collision_avoidance_u(my_pos, others_pos)
                collision_avoidance_u += u
                
            i+=1

        targetpos_u = self.get_targetpos_u()

        dampen_u = uav.get_current_velocity() * (-0.8)

        altitude_u = self.get_altitude_u()

        gain = targetpos_u * (0.01) + dampen_u * 0.2 + collision_avoidance_u* 0.1 + altitude_u * 0.1

        self.u_integral += gain

        uav.set_target_pose(uav.get_current_pose() + self.u_integral )

    def get_altitude_u(self):
        my_index = self.uav_id
        my_pos = self.uav_pos_infos[my_index].value
        if(my_pos[2] < 4):
            closeness = 1 - my_pos[2]/3
            force = closeness * closeness * 10
            return np.array([0.0,0.0,1.0 * force])
        else:
            return np.array([0.0,0.0,0.0])
    def get_targetpos_u(self):
        i = 0
        my_index = self.uav_id
        my_pos = self.uav_pos_infos[my_index].value
        u = np.array([0.0,0.0,0.0])
        for uav_pos_info in self.uav_pos_infos:
            if(i != my_index):
                other_pos = uav_pos_info.value
                delta = other_pos - my_pos
                target_delta = self.point_cloud[i] - self.point_cloud[my_index]
                u -= target_delta - delta
            i+=1
        return u
    
    def get_collision_avoidance_u(self, selfpos, otherpos, dist=10):
        #u_ca(i) = alpha * sum(exp(- beta * abs(r_ij)))
        distance = np.linalg.norm(selfpos - otherpos)
        direction = selfpos - otherpos
        direction = direction / np.linalg.norm(direction)

        alpha = 3
        beta = 0.5
        
        if distance < dist:
            return direction * alpha * (math.exp(-beta*distance) - math.exp(-beta*dist))
        else:
            return np.array([0.0,0.0,0.0])


    def get_position_u(self, selfpos, targetpos):
        #u_f(i) = kp*sum(p(j) - p(i) - delta_ij) 
        u = targetpos - selfpos
        u = u * 1
        return u
''' 
if(distance < 5.0):
    #force_mag = 20 * math.exp(- 2 * abs(distance))
    force_mag = 15 * math.exp(- 1.5 * abs(distance))
    #4 * e^(- 0.25 * abs(x))

    direction = (otherpos - selfpos) / distance
    force = -direction * force_mag
    if(self.uav_id == 2):
        print("selfpos: ", selfpos, "otherpos: ", otherpos)
        print("distance: ", distance, "force: ", force)
    return force
return np.array([0.0,0.0,0.0])
'''