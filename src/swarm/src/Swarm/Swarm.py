#!/usr/bin/env python
import rospy
from swarm.msg import CNN_out
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty


TEST_PHASE=0

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os 
#from tkinter import *


class Swarm(object):
    def __init__(self):
        rospy.init_node("swarm", anonymous=True)
        print("i am alive")

    
        
