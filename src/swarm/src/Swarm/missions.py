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




class Mission(object):
    # constuctor paramaters must all have default values, or there will be issues with serialization/deserialization

    def __init__(self):
        self.start_time = None
        self.type = -1
        self.delta_time = 1.0
        self.last_frame_time = 0.0
        

    def execute_mission(self, uav, rate):
        self.uav = uav
        self.rate = rate

        self.mission_started(uav, rate)

        while (True):
            if (self.mission_ended(uav, rate)):
                break
            else:
                self.mission_loop(uav, rate)
            self.rate.sleep()



    # serialize/deserialize the data only, no object creation, if you want to make whole object into string, use issionFactory
    def deserialize_data_from_buffer(self, buffer):
        pass
        # raise Exception("Mission does not implement required method")

    def serialize_data_into_buffer(self, buffer):
        pass
        # raise Exception("Mission does not implement required method")

    def mission_started(self, uav, rate):
        self.start_time = uav.get_current_time()
        pass

    def mission_loop(self, uav, rate):
        self.delta_time = self.get_time_since_start(uav, rate) - self.last_frame_time
        self.last_frame_time = self.get_time_since_start(uav, rate)
        pass

    def mission_ended(self, uav, rate):
        pass

    def get_time_since_start(self, uav, rate):
        sincestart = self.uav.get_current_time().to_sec() - self.start_time.to_sec()
        return sincestart

class FormationLeader(Mission):

    def __init__(self):
        # these are the NetworkedInfos for sending vector3 data to slaves
        self.target_pos_infos = []
    
    def mission_started(self,uav,rate):
        print("leader mission started")
        super(FormationLeader, self).mission_started(uav, rate)

        # define a slave mission
        slave_mission = FormationSlave()
        for i in range(3):
            slave_name = "uav" + str(i)
            # define the vector NetworkedInfo
            topic = slave_name + "/formation_mission/position_target"
            position_target_info = VectorInfo(topic)
            self.target_pos_infos.append(position_target_info)

            # give the slave mission to uav
            uav.assign_mission(slave_mission, slave_name)


    def mission_loop(self, uav, rate):
        for i in range(3):
            # get the NetworkedInfo
            position_target_info = self.target_pos_infos[i]
            # set its value
            position_target_info.value = np.array([5, sin(2*pi * self.get_time_since_start(uav, rate) * 0.1) * 10 , i * 5])
            # publish, next time you see that value, we are on slave mission
            position_target_info.publish_data()

class FormationSlave(Mission):
    def __init__(self):
        self.target_pos_info = None
        # we are syncing SetpointPosition mission to this mission as usual
        self.setpoint = SetpointPosition()
    
    def mission_started(self,uav,rate):
        print("slave mission started")
        super(FormationSlave, self).mission_started(uav, rate)
        # define the same networked info here, to communicate with leader
        topic = uav.uav_name + "/formation_mission/position_target"
        self.target_pos_info = VectorInfo(topic)
        print("subscribed to: " + topic)

        # make a new TakeOff mission and wait for its completion 
        # we cant be a good slave if we dont takeoff first
        takeoff = TakeOff()
        takeoff.execute_mission(uav, rate)

        # syncing a mission to this mission means calling mission_started and mission_loop manually here
        # synced mission
        self.setpoint.mission_started(uav, rate)
    
    def mission_loop(self, uav, rate):
        # target_pos_info is a NetworkedInfo, therefore its updated automatically 
        # everytime someone calls publish on it
        # we set parameters of setpoint mission from this NetworkedInfo to move the uav
        self.setpoint.target_position = self.target_pos_info.value

        # synced mission
        self.setpoint.mission_loop(uav, rate)



class SetpointPosition(Mission):
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        super(SetpointPosition, self).__init__()
        self.target_position = np.array([0,0,0])
    
    def deserialize_data_from_buffer(self, buffer):
        self.target_position = buffer.read_vector3()
        
    def serialize_data_into_buffer(self, buffer):
        buffer.write_vector3(self.target_position)

    def mission_loop(self, uav, rate):
        uav.set_target_pose(self.target_position)

    def mission_ended(self, uav, rate):
        current_pos = uav.get_current_pose()

        def is_near(cur, tar):
            return abs(cur - tar) < 0.5
        if is_near(current_pos[0], self.target_position[0]) and \
            is_near(current_pos[1], self.target_position[1]) and \
            is_near(current_pos[2], self.target_position[2]):
            return True
        return False

class Helix(Mission):
    def __init__(self, uav, rate, period, radius, slope):
            #x(t) = acos(t),
            #y(t) = asin(t)
            #z(t) = bt
            #slope b/a, pitch 2*pi*b
        super(Helix ,self).__init__(uav, rate)
        self.T = period #(sec)
        self.a = radius
        self.b = slope * radius

    def mission_started(self):
        super(Helix ,self).mission_started()
        self.center = self.uav.get_current_pose().pose.position
        self.center.x = self.center.x - self.a

    def mission_loop(self):
        T = self.get_time_since_start() / self.T
        x = self.a * cos(T * 2 * pi)
        y = self.a * sin(T * 2 * pi)
        self.uav.set_target_pose(self.center.x + x, self.center.y + y, self.center.z + self.b)

        dx = - self.a * sin(T * 2 * pi)
        dy = self.a * cos(T * 2 * pi)
        dz = self.b
        self.uav.set_target_velocity(dx, dy, dz)

    def mission_ended(self):
        if(self.get_time_since_start() > self.T):
            return True
        else:
            return False

class Wait(Mission):
    
    def __init__(self, uav, rate, duration):
        super(Wait, self).__init__(uav, rate)
        self.go_and_wait = None
        self.duration = duration

    def mission_started(self):
        super(Wait, self).mission_started()
        last_setpoint = self.uav.get_last_target_pose()
        self.go_and_wait = GoAndWait(self.uav, self.rate, self.duration, last_setpoint.pose.x,last_setpoint.pose.y,last_setpoint.pose.z)

    def mission_loop(self):
        self.go_and_wait.mission_loop()

    def mission_ended(self):
        return self.go_and_wait.mission_ended()

class GoAndWait(Mission):
    def __init__(self, uav, rate, duration, x, y, z):
        super(GoAndWait, self).__init__(uav, rate)
        self.setpoint = SetpointPosition(uav, rate, x,y,z)
        self.duration = duration

    def mission_loop(self):
        self.setpoint.mission_loop()
        
    def mission_ended(self):
        reached = self.setpoint.mission_ended()
        timesup = self.get_time_since_start() > self.duration
        if(reached and timesup):
            return True
        else:
            return False

class MakeCircle(Mission):
    def __init__(self, uav, rate, period, radius):
        super(MakeCircle, self).__init__(uav, rate)
        self.period = period
        self.radius = radius
        self.center = None
    
    def mission_started(self):
        super(MakeCircle, self).mission_started()
        self.center = self.uav.get_current_pose().pose.position
        self.center.x = self.center.x -self.radius
        
    def mission_loop(self):
        time = self.get_time_since_start() / self.period # 0 to 1
        dx = cos(time * pi * 2) * self.radius
        dy = sin(time * pi * 2) * self.radius
        self.uav.set_target_pose(self.center.x + dx, self.center.y + dy, self.center.z)

        vx = -sin(time * pi * 2) * self.radius
        vy = cos(time * pi * 2) * self.radius
        self.uav.set_target_velocity(vx,vy,0)
    
    def mission_ended(self):
        if(self.get_time_since_start() > self.period):
            return True
        else:
            return False

class TakeOff(Mission):
    def __init__(self):
        super(TakeOff, self).__init__()
        self.type = 4
        self.setpoint = None

    def mission_started(self, uav, rate):
        super(TakeOff, self).mission_started(uav, rate)
        uav_start_pose = uav.get_current_pose()
        pos = uav_start_pose
        pos[2] += 10
        self.setpoint = SetpointPosition()
        self.setpoint.target_position = pos
        self.setpoint.mission_started(uav, rate)


    def mission_loop(self, uav, rate):
        uav.arm(True)
        uav.set_offboard()

        self.setpoint.mission_loop(uav, rate)
    
    def mission_ended(self, uav, rate):
        return self.setpoint.mission_ended(uav, rate)

    def deserialize_data_from_buffer(self, buffer):
        pass
    def serialize_data_into_buffer(self, buffer):
        pass


