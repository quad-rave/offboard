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
from scipy.optimize import linear_sum_assignment
from utility import *


class TakeOffAndWaitOthers(Mission):
    def __init__(self):
        self.takeoff = TakeOff()
        self.uav_count = None
        self.uav_id = None
        self.uav_confirmation_infos = []
        self.mission_complete_info = None

    def deserialize_data_from_buffer(self, buffer):
        self.uav_id = buffer.read_int()
        self.uav_count = buffer.read_int()
    def serialize_data_into_buffer(self, buffer):
        buffer.write_int(self.uav_id)
        buffer.write_int(self.uav_count)

    def mission_started(self,uav,rate):
        super(TakeOffAndWaitOthers, self).mission_started(uav, rate)

        for i in range(self.uav_count):
            uav_confirmation_info = BoolInfo("uav" + str(i) + "/TakeOffAndWaitOthers_mission/ready")
            uav_confirmation_info.value = False # initialize local value with false
            self.uav_confirmation_infos.append(BoolInfo("uav" + str(i) + "/TakeOffAndWaitOthers_mission/ready"))
        
        self.mission_complete_info = BoolInfo("TakeOffAndWaitOthers_mission/complete")
        self.mission_complete_info.value = False

        # start the synced takeoff mission
        self.takeoff = TakeOff()
        self.takeoff.mission_started(uav,rate)

    def mission_loop(self, uav, rate):
        # did i takeoff successfully yet? publish the data.
        self.uav_confirmation_infos[self.uav_id].value = self.takeoff.mission_ended(uav,rate) 
        self.uav_confirmation_infos[self.uav_id].publish_data() 

        # check if every uav have taken off
        all_uavs_taken_off = True
        i = 0
        for uav_confirmation_info in self.uav_confirmation_infos:
            if i != self.uav_id:
                value = uav_confirmation_info.value
                if(value == False):
                    all_uavs_taken_off = False
                
            i+=1
        
        # This uav saw a step where everybody had taken off. This uav calls the mission complete.
        if(all_uavs_taken_off):
            self.mission_complete_info.value = True
            self.mission_complete_info.publish_data()

        # update takeoff mission (synced mission)
        self.takeoff.mission_loop(uav,rate)


    def mission_ended(self,uav,rate):
        return self.mission_complete_info.value

class TriangleMember(Mission):
    def __init__(self):
        self.setpoint = SetpointPosition()
        self.uav_count = None
        self.uav_id = None
        self.uav_pos_infos = []
        self.point_cloud_info = None
        self.u_integral = np.array([0.0,0.0,0.0])

    def deserialize_data_from_buffer(self, buffer):
        #self.target_position = buffer.read_vector3()
        self.uav_id = buffer.read_int()
        self.uav_count = buffer.read_int()

        #size = buffer.read_int()
        #for i in range(size):
        #    self.point_cloud.append(buffer.read_vector3())

        
        
    def serialize_data_into_buffer(self, buffer):
        #buffer.write_vector3(self.target_position)
        buffer.write_int(self.uav_id)
        buffer.write_int(self.uav_count)

        #buffer.write_int(len(self.point_cloud))
        #for vector in self.point_cloud:
        #    buffer.write_vector3(vector)
        
    def mission_started(self,uav,rate):
        print("slave mission started")
        super(TriangleMember, self).mission_started(uav, rate)

        # point cloud is now recieved dynamically as a VectorArrayInfo
        self.point_cloud_info = VectorArrayInfo("/triangle_mission/point_cloud")

        for i in range(self.uav_count):
            #self.uav_pos_infos.append(VectorInfo(topic))
            self.uav_pos_infos.append(VectorInfo("uav" + str(i) + "/triangle_mission/position_target"))

        # this mission will end when all uavs have taken off and ready
        swarm_takeoff = TakeOffAndWaitOthers()
        swarm_takeoff.uav_count = self.uav_count
        swarm_takeoff.uav_id = self.uav_id
        swarm_takeoff.execute_mission(uav, rate)



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

        targetpos_u = self.get_targetpos_u_via_effective_point_cloud() #self.get_targetpos_u()

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
                target_delta = self.point_cloud_info.value[i] - self.point_cloud_info.value[my_index]
                u -= target_delta - delta
            i+=1
        return u

    def get_targetpos_u_via_effective_point_cloud(self):
        effective_point_cloud = self.get_effective_point_cloud()

        i = 0
        my_index = self.uav_id
        my_pos = self.uav_pos_infos[my_index].value
        u = np.array([0.0,0.0,0.0])
        for uav_pos_info in self.uav_pos_infos:
            if(i != my_index):
                other_pos = uav_pos_info.value
                delta = other_pos - my_pos
                target_delta = effective_point_cloud[i] - effective_point_cloud[my_index]
                u -= target_delta - delta
            i+=1
        return u

    def get_formation_u_with_other(my_pos, other_pos, my_target_pos, other_target_pos):
        delta = other_pos - my_pos
        target_delta = other_target_pos - my_target_pos
        return (target_delta - delta)

    def get_effective_point_cloud(self):
        uav_poses = []
        for uav_pos_info in self.uav_pos_infos:
            uav_poses.append(uav_pos_info.value)
        point_cloud = self.point_cloud_info.value
        uav_count = self.uav_count

        
        cost_matrix = np.zeros((uav_count, uav_count), dtype=np.float64)

        for uav in range(uav_count):
            for point in range(uav_count):
                cost_matrix[uav, point] = dist(uav_poses[uav], point_cloud[point])
                

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        print(row_ind, "\n----------")
        print(col_ind)

        effective_point_cloud = [vec(0.0,0.0,0.0) for i in range(len(point_cloud))]
        for i in range(len(row_ind)):
            effective_point_cloud[row_ind[i]] = point_cloud[col_ind[i]]
        
        return effective_point_cloud
    
    def get_collision_avoidance_u(self, selfpos, otherpos, dist=10):
        #u_ca(i) = alpha * sum(exp(- beta * abs(r_ij)))
        distance = np.linalg.norm(selfpos - otherpos)
        direction = selfpos - otherpos
        direction = direction / np.linalg.norm(direction)

        alpha = 6
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