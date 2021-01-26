#!/usr/bin/env python2
# vim:set ts=4 sw=4 et:

# this is for type serialization/deserialization purposes
from buffer import DataBuffer
# from uav import UAV
from networked_info import *
from utility import TwoWayDict
import numpy as np
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from math import *

from missions import *
from scipy.optimize import linear_sum_assignment
from utility import *
from integrator_system import IntegratorSystem


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

    def mission_started(self, uav, rate):
        super(TakeOffAndWaitOthers, self).mission_started(uav, rate)

        for i in range(self.uav_count):
            uav_confirmation_info = NetworkedInfo.get_or_create("uav" + str(i) + "/TakeOffAndWaitOthers_mission/ready",
                                                                False)
            self.uav_confirmation_infos.append(
                NetworkedInfo.get_or_create(("uav" + str(i) + "/TakeOffAndWaitOthers_mission/ready"), False))

        self.mission_complete_info = NetworkedInfo.get_or_create("TakeOffAndWaitOthers_mission/complete", False)

        # start the synced takeoff mission
        self.takeoff = TakeOff()
        self.takeoff.mission_started(uav, rate)

    def mission_loop(self, uav, rate):
        # did i takeoff successfully yet? publish the data.
        self.uav_confirmation_infos[self.uav_id].set_data(self.takeoff.mission_ended(uav, rate))

        # check if every uav have taken off
        all_uavs_taken_off = True
        i = 0
        for uav_confirmation_info in self.uav_confirmation_infos:
            if i != self.uav_id:
                value = uav_confirmation_info.get_data()
                if (value == False):
                    all_uavs_taken_off = False

            i += 1

        # This uav saw a step where everybody had taken off. This uav calls the mission complete.
        if (all_uavs_taken_off):
            self.mission_complete_info.set_data(True)

        # update takeoff mission (synced mission)
        self.takeoff.mission_loop(uav, rate)

    def mission_ended(self, uav, rate):
        return self.mission_complete_info.get_data()


class FormationRotationData:
    def __init__(self, axis="Z", duration=5.0, angle=0.0):
        self.axis = axis
        self.duration = duration
        self.angle = angle


class TriangleMember(Mission):
    # constants:

    # formation
    k_p = 0.1
    k_d = 1.6
    k_r = 5.0

    # collision avoidance (between uavs)
    ca_alpha = 6.0
    ca_beta = 0.5
    ca_safety_region = 10.0

    # tracking
    k_tp = 0.25
    k_td = 0.8

    def __init__(self):
        super(TriangleMember, self).__init__()
        self.setpoint = SetpointPosition()
        self.uav_count = None
        self.uav_id = None
        self.uav_pos_infos = []
        self.uav_vel_infos = []
        self.formation_points_info = None
        self.u_integral = np.array([0.0, 0.0, 0.0])
        self.formation_pos = vec(0.0, 0.0, 0.0)  # center
        self.formation_vel = vec(0.0, 0.0, 0.0)
        self.formation_targetpos_info = None
        self.formation_targetrot_info = None

        # formation rotation:
        self.formation_rot_cur_angle = 0.0
        self.formation_rot_cur_angle_integrator = None
        self.formation_rot_target_angle = 0.0
        self.formation_rot_axis = "Z"
        self.formation_rot_starttime = 0.0
        self.formation_rot_duration = 5.0
        self.formation_rot_info = None

        self.formation_rot_matrix = rotation_matrix(0.0)
        self.formation_rot_matrix_derivative = rotation_matrix(0.0)
        self.formation_rot_start_matrix = rotation_matrix(0.0)
        self.formation_rot_cur_angle_integrator = IntegratorSystem()

    def formation_rot_info_callback(self):
        self.formation_rot_axis = self.formation_rot_info.get_data().axis
        self.formation_rot_duration = self.formation_rot_info.get_data().duration
        self.formation_rot_target_angle = self.formation_rot_info.get_data().angle
        self.formation_rot_starttime = self.uav.get_current_time().to_sec()
        self.formation_rot_cur_angle = 0.0
        self.formation_rot_cur_angle_integrator = IntegratorSystem()
        self.formation_rot_start_matrix = self.formation_rot_matrix

        # print("formation_rot callback -A------------------AAAAAAAAAAAAAAAAAAAAAAAAA")

    def formation_rot_update(self):
        t = self.uav.get_current_time().to_sec()
        t_0 = self.formation_rot_starttime
        t_1 = self.formation_rot_starttime + self.formation_rot_duration
        t_mid = (t_0 + t_1) * 0.5

        accel = 4 * (self.formation_rot_target_angle - 0.0) / ((t_1 - t_0) ** 2)

        if (t > t_mid):
            accel = -accel
        if (t > t_1):
            accel = 0.0

        self.formation_rot_cur_angle_integrator.update(self.delta_time, accel)

        new_formation_matrix = np.matmul(
            rotation_matrix(self.formation_rot_cur_angle_integrator.f, self.formation_rot_axis),
            self.formation_rot_start_matrix)
        self.formation_rot_matrix_derivative = (new_formation_matrix - self.formation_rot_matrix) / self.delta_time
        self.formation_rot_matrix = new_formation_matrix

    def deserialize_data_from_buffer(self, buffer):
        self.uav_id = buffer.read_int()
        self.uav_count = buffer.read_int()

    def serialize_data_into_buffer(self, buffer):
        buffer.write_int(self.uav_id)
        buffer.write_int(self.uav_count)

    def mission_started(self, uav, rate):
        print("slave mission started")
        super(TriangleMember, self).mission_started(uav, rate)

        # point cloud is now recieved dynamically as a VectorArrayInfo
        self.formation_points_info = NetworkedInfo.get_or_create("/triangle_mission/formation_points", [])
        self.formation_targetpos_info = NetworkedInfo.get_or_create("/triangle_mission/formation_targetpos", vec())
        self.formation_rot_info = NetworkedInfo.get_or_create("/triangle_mission/formation_targetrot",
                                                              FormationRotationData())
        self.formation_rot_info.assign_callback(self.formation_rot_info_callback)

        for i in range(self.uav_count):
            # self.uav_pos_infos.append(VectorInfo(topic))
            self.uav_pos_infos.append(
                NetworkedInfo.get_or_create("uav" + str(i) + "/triangle_mission/current_pos", vec()))
            self.uav_vel_infos.append(
                NetworkedInfo.get_or_create("uav" + str(i) + "/triangle_mission/current_vel", vec()))
        # this mission will end when all uavs have taken off and ready
        swarm_takeoff = TakeOffAndWaitOthers()
        swarm_takeoff.uav_count = self.uav_count
        swarm_takeoff.uav_id = self.uav_id
        swarm_takeoff.execute_mission(uav, rate)

    def mission_loop(self, uav, rate):
        super(TriangleMember, self).mission_loop(uav, rate)

        self.communications()

        targetpos_u_traction = self.get_targetpos_u_traction()
        dampen_u_traction = self.get_dampen_u_traction()
        traction_u = targetpos_u_traction + dampen_u_traction

        self.formation_rot_update()

        collision_avoidance_u = np.array([0., 0., 0.])
        i = 0
        for uav_pos_info in self.uav_pos_infos:
            if i != self.uav_id:
                my_pos = self.uav.get_current_pose()
                others_pos = uav_pos_info.get_data()

                u = self.get_collision_avoidance_u(my_pos, others_pos)
                collision_avoidance_u += u
            i += 1

        targetpos_u = self.get_targetpos_u_via_effective_formation_points()  # self.get_targetpos_u()
        dampen_u = (self.formation_vel - uav.get_current_velocity()) * (TriangleMember.k_d)
        altitude_u = self.get_altitude_u()
        formation_u = targetpos_u + dampen_u + collision_avoidance_u + altitude_u * 0.1
        combined_u = formation_u + traction_u
        self.u_integral += combined_u  * self.delta_time

        uav.set_target_pose(uav.get_current_pose() + self.u_integral)

    def get_targetpos_u_traction(self):
        target = self.formation_targetpos_info.get_data()
        current = self.formation_pos

        return TriangleMember.k_tp * (target - current)
    def get_dampen_u_traction(self):
        return self.formation_vel * (-1.0) * TriangleMember.k_td

    def communications(self):
        self.uav_pos_infos[self.uav_id].set_data(self.uav.get_current_pose())

        self.uav_vel_infos[self.uav_id].set_data(self.uav.get_current_velocity())

        pos_sum = vec(0.0, 0.0, 0.0)
        for uav_pos_info in self.uav_pos_infos:
            pos_sum += uav_pos_info.get_data()
        self.formation_pos = pos_sum / self.uav_count

        vel_sum = vec(0.0, 0.0, 0.0)
        i = 0
        for uav_vel_info in self.uav_vel_infos:
            vel_sum += uav_vel_info.get_data()
            i += 1
        self.formation_vel = vel_sum / self.uav_count

    def get_altitude_u(self):
        my_index = self.uav_id
        my_pos = self.uav_pos_infos[my_index].get_data()
        if (my_pos[2] < 4):
            closeness = 1 - my_pos[2] / 3
            force = closeness * closeness * 10
            return np.array([0.0, 0.0, 1.0 * force])
        else:
            return np.array([0.0, 0.0, 0.0])

    def get_targetpos_u_via_effective_formation_points(self):
        effective_formation_points = self.get_effective_formation_points()

        i = 0
        my_index = self.uav_id
        my_pos = self.uav_pos_infos[my_index].get_data()
        u = np.array([0.0, 0.0, 0.0])
        for uav_pos_info in self.uav_pos_infos:
            if i != my_index:
                other_pos = uav_pos_info.get_data()
                delta = other_pos - my_pos

                target_delta = effective_formation_points[i] - effective_formation_points[my_index]
                rotated_target_delta = np.matmul(self.formation_rot_matrix, effective_formation_points[i]) - np.matmul(
                    self.formation_rot_matrix, effective_formation_points[my_index])

                cu = np.matmul((self.formation_rot_matrix_derivative * self.formation_rot_cur_angle_integrator.f_d1),
                               target_delta)

                u -= (rotated_target_delta - delta + cu * TriangleMember.k_r) * TriangleMember.k_p
            i += 1
        return u

    def get_formation_u_with_other(self, my_pos, other_pos, my_target_pos, other_target_pos):
        delta = other_pos - my_pos
        target_delta = other_target_pos - my_target_pos
        return target_delta - delta

    def get_effective_formation_points(self):
        uav_poses = []
        for uav_pos_info in self.uav_pos_infos:
            uav_poses.append(uav_pos_info.get_data())
        formation_points = self.formation_points_info.get_data()
        uav_count = self.uav_count

        cost_matrix = np.zeros((uav_count, uav_count), dtype=np.float64)

        for uav in range(uav_count):
            for point in range(uav_count):
                cost_matrix[uav, point] = dist(uav_poses[uav], (
                    np.matmul(self.formation_rot_matrix, formation_points[point])) + self.formation_pos)

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        # print(row_ind, "\n----------")
        # print(col_ind)

        effective_formation_points = [vec(0.0, 0.0, 0.0) for i in range(len(formation_points))]
        for i in range(len(row_ind)):
            effective_formation_points[row_ind[i]] = formation_points[col_ind[i]]

        return effective_formation_points

    def get_collision_avoidance_u(self, selfpos, otherpos):
        distance = np.linalg.norm(selfpos - otherpos)
        direction = selfpos - otherpos
        direction = direction / np.linalg.norm(direction)

        if distance < TriangleMember.ca_safety_region:
            return direction * TriangleMember.ca_alpha * (math.exp(-TriangleMember.ca_beta * distance) - math.exp(-TriangleMember.ca_beta * TriangleMember.ca_safety_region))
        else:
            return np.array([0.0, 0.0, 0.0])

    def get_position_u(self, selfpos, targetpos):
        # u_f(i) = kp*sum(p(j) - p(i) - delta_ij)
        u = targetpos - selfpos
        return u

