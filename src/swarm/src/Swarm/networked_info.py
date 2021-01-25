#!/usr/bin/env python2
# vim:set ts=4 sw=4 et:

from std_msgs.msg import String
from mavros import command
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
import rospy
from buffer import DataBuffer
import math
import numpy as np


class NetworkedInfo(object):
    topic_to_network_info = {}

    @staticmethod
    def get_or_create(topic, default_data = None):
        if topic in NetworkedInfo.topic_to_network_info.keys():
            return NetworkedInfo.topic_to_network_info[topic]
        else:
            new_instance = NetworkedInfo(topic, default_data)
            NetworkedInfo.topic_to_network_info[topic] = new_instance
            return new_instance

    # don't ever call this from outside, get NetworkInfo's via "get_or_create" instead
    def __init__(self, topic, data):
        self.callbacks = []
        self.data = data

    def set_data(self, data):
        self.data = data
        for callback in self.callbacks:
            callback()

    def get_data(self):
        return self.data

    def assign_callback(self, callback):
        self.callbacks.append(callback)
