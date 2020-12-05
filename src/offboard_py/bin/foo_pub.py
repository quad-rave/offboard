
import rospy
import _thread as thread
import threading
import time
import mavros
import sys
from std_msgs.msg import String
import signal


def interupt_handler(signum, frame):
    print ("Exiting")
    sys.exit(-2) #Terminate process here as catching the signal removes the close process behaviour of Ctrl-C


def publish():
    node = rospy.init_node("foo")
    rate = rospy.Rate(1)  # 10hz

    status_pub = rospy.Publisher("gcs/mission/{}/status".format("alpha"), String, queue_size=10)
    while True:
        status_pub.publish("idle")
        for _ in range(10):
            rate.sleep()    
        status_pub.publish("busy")
        ## Do something
        for _ in range(10):
            rate.sleep()      
              ## Finish


    print("Exiting")
    status_pub.publish(None)
    sys.exit(0)


    print("publishing")
    

if __name__ == "__main__":
    signal.signal(signal.SIGINT, interupt_handler)
    publish()    