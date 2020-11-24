#!/usr/bin/env python2
# vim:set ts=4 sw=4 et:

# this is for type serialization/deserialization purposes
from buffer import DataBuffer
from uav import UAV
from networked_info import NetworkedInfo
from utility import TwoWayDict

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3

class MissionFactory(object):
    def __init__(self, missiontype_to_constructor):
        self.missiontype_to_constructor = missiontype_to_constructor
        self.mission_from_buffer
    
    def mission_from_buffer(buffer : DataBuffer):
        mission_class = self.missiontype_to_constructor[buffer.read_int()]

        mission_obj =  mission_class()
        mission_obj.deserialize_from_buffer(buffer)

class Mission(object):
    # constuctor paramaters must all have default values, or there will be issues with MissionFactory
    def __init__(self):
        pass
        
    def execute_mission(self, uav, rate):
        self.uav = uav
        self.rate = rate

        mission_started(self, uav, rate)

        while(True):
            if(self.mission_ended(uav, rate)):
                break
            else:
                self.mission_loop(uav rate)
            self.rate.sleep()


    def deserialize_from_buffer(self, buffer):
        raise Exception("Mission does not implement required method")

    def serialize_into_buffer(self, buffer):
        raise Exception("Mission does not implement required method")

    def mission_started(self, uav, rate):
        self.start_time = self.uav.get_current_time()
        pass

    def mission_loop(self, uav, rate):
        pass

    def mission_ended(self, uav, rate):
        pass

    def get_time_since_start(self, uav, rate):
        sincestart = self.uav.get_current_time().to_sec() - self.start_time.to_sec()
        return sincestart

class SetpointPosition(Mission):
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self, target_position = Vector3()):
        super(SetpointPosition, self).__init__()
        self.target_position = target_position
    
        

    def deserialize_from_buffer(self, buffer):
        self.target_position = buffer.read_vector3()
        
    def serialize_into_buffer(self, buffer):
        buffer.write_vector3(self.target_position)

    
    def mission_loop(self, uav, rate):
        uav.set_target_pose(self.target_position.x,self.target_position.y,self.target_position.z)

    def mission_ended(self, uav, rate):
        current_pos = uav.get_current_pose()

        def is_near(cur, tar):
            return abs(cur - tar) < 0.5
        if is_near(current_pos.pose.position.x, self.x) and \
            is_near(current_pos.pose.position.y, self.y) and \
            is_near(current_pos.pose.position.z, self.z):
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
    ## Bu wait kısmının update'e girmesi lazım mission_loop'da bir setpoint yapılması gerekiyor diğer türlü otopilot kendini anlık failsafe'e alıyor.
    ## Wait diye bir methodun olması da önemli mesela 2 helix yaptırırken değerleri çekmek çok zor rostopic'den burada halletmek daha kolay olur.
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
        super(TakeOff, self).__init__(uav, rate)
        self.setpoint = None

    def mission_started(self, uav, rate):
        super(TakeOff, self).mission_started()
        uav_start_pose = uav.get_current_pose()
        pos = Vector3(uav_start_pose.pose.position.x, uav_start_pose.pose.position.y, uav_start_pose.pose.position.z + 10)
        self.setpoint = SetpointPosition(pos)
        self.setpoint.mission_started(uav, rate)


    def mission_loop(self, uav, rate):
        uav.arm(True)
        uav.set_offboard()

        self.setpoint.mission_loop(uav, rate)
    
    def mission_ended(self, uav, rate):
        return self.setpoint.mission_ended(uav, rate)

    def deserialize_from_buffer(self, buffer):
        pass
    def serialize_into_buffer(self, buffer):
        pass


def get_missiontype_to_constructor():
    missiontype_to_constructor = TwoWayDict()
    missiontype_to_constructor.add(0, Helix)
    missiontype_to_constructor.add(1, Wait)
    missiontype_to_constructor.add(2, GoAndWait)
    missiontype_to_constructor.add(3, MakeCircle)
    missiontype_to_constructor.add(4, TakeOff)
    missiontype_to_constructor.add(5, SetpointPosition)
    return missiontype_to_constructor