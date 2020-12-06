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
    
    # dont override these two:
    def _on_topic_recieve(self,topic):
        topic_buffer = DataBuffer.from_string(topic.data)
        self.deserialize_from_buffer(topic_buffer)

    def publish_data(self):
        buffer = DataBuffer()
        self.serialize_into_buffer(buffer)
        str_data = buffer.to_string()
        self.topic_pub.publish(str_data)

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



