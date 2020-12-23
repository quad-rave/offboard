#!/usr/bin/env python2
# vim:set ts=4 sw=4 et:

from std_msgs.msg import String
from mavros import command
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
import rospy
from buffer import  DataBuffer
import math
import numpy as np

class NetworkedInfo(object):
    def __init__(self, topic):
        self.topic_pub = rospy.Publisher(topic, String, queue_size = 10)
        self.topic_sub = rospy.Subscriber(topic, String, self._on_topic_recieve)
        self.callbacks = []
    # dont override these two:
    def _on_topic_recieve(self,topic):
        topic_buffer = DataBuffer.from_string(topic.data)
        self.deserialize_from_buffer(topic_buffer)
        
        for callback in self.callbacks:
            callback()

    def publish_data(self):
        buffer = DataBuffer()
        self.serialize_into_buffer(buffer)
        str_data = buffer.to_string()
        self.topic_pub.publish(str_data)

    def assign_callback(self, callback):
        self.callbacks.append(callback)

    # these two are for overriding:
    def serialize_into_buffer(self, buffer):
        raise Exception("NetworkedInfo does not implement required method")
    def deserialize_from_buffer(self, buffer):
        raise Exception("NetworkedInfo does not implement required method")

# my_vector_info.value = myVector
# my_vector_info.publish()
class VectorInfo(NetworkedInfo): 
    def __init__(self, topic):
        super(VectorInfo, self).__init__(topic)
        self.value = np.array([0,0,0])

    def serialize_into_buffer(self, buffer):
        buffer.write_vector3(self.value)
    def deserialize_from_buffer(self, buffer):
        self.value = buffer.read_vector3()

class BoolInfo(NetworkedInfo):
    def __init__(self, topic):
        super(BoolInfo, self).__init__(topic)
        self.value = False

    def serialize_into_buffer(self, buffer):
        if(self.value == True):
            buffer.write_int(1)
        else:
            buffer.write_int(0)

    def deserialize_from_buffer(self, buffer):
        intval = buffer.read_int()
        if(intval == 1):
            self.value = True
        else:
            self.value = False

class VectorArrayInfo(NetworkedInfo):
    def __init__(self, topic):
        super(VectorArrayInfo, self).__init__(topic)
        self.value = []

    def serialize_into_buffer(self, buffer):
        length = len(self.value)

        buffer.write_int(length)
        for i in range(length):
            buffer.write_vector3(self.value[i])

    def deserialize_from_buffer(self, buffer):
        self.value = []
        length = buffer.read_int()
        for i in range(length):
            self.value.append(buffer.read_vector3())
    







    