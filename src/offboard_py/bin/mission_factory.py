#!/usr/bin/env python2
# vim:set ts=4 sw=4 et:
from missions import *
from triangle_missions import TriangleLeader, TriangleSlave


class MissionFactory(object):
    def __init__(self):
        missiontype_to_constructor = TwoWayDict()
        #missiontype_to_constructor.add(0, Helix)
        #missiontype_to_constructor.add(1, Wait)
        #missiontype_to_constructor.add(2, GoAndWait)
        missiontype_to_constructor.add(3, TriangleLeader)
        missiontype_to_constructor.add(4, TakeOff)
        missiontype_to_constructor.add(5, SetpointPosition)
        #missiontype_to_constructor.add(6, FormationLeader)
        #missiontype_to_constructor.add(7, FormationSlave)
        missiontype_to_constructor.add(8, MakeCircle)
        missiontype_to_constructor.add(9, TriangleSlave)
        self.missiontype_to_constructor = missiontype_to_constructor

    
    def mission_from_buffer(self, buffer):
        mission_class = self.missiontype_to_constructor.get_value(buffer.read_int())

        mission_obj = mission_class()
        mission_obj.deserialize_data_from_buffer(buffer)
        return mission_obj

    def mission_into_buffer(self, buffer, mission):
        mission_type = self.missiontype_to_constructor.get_key(type(mission))
        buffer.write_int(mission_type)
        mission.serialize_data_into_buffer(buffer)