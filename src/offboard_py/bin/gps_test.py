#!/usr/bin/env python2
# vim:set ts=4 sw=4 et:

import rospy, sensor_msgs.msg # interesting
import mavros
from mavros.utils import *
from mavros import setpoint as SP
from geographic_msgs.msg import GeoPointStamped as GS

#from tf.transformations import quaternion_from_euler
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandBool
from std_msgs.msg import String
from mavros import command


class UAV(object):
    def __init__(self):

        self.pos = SP.PoseStamped()
        self.set_pos = rospy.Publisher('mavros/setpoint_position/local', SP.PoseStamped, queue_size=1)
        self.sub_pose = rospy.Subscriber('/mavros/local_position/pose', SP.PoseStamped,
                                         self._read_position_from_topic)    
        self.sub_gps_pose = rospy.Subscriber('/mavros/global_position/global', sensor_msgs.msg.NavSatFix,
                                        self._read_gps_from_topic)    
        self.pose_stamped = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),  # stamp should update
        )
        self.pose_stamped_gps = sensor_msgs.msg.NavSatFix()

        self.offb_set_mode = SetMode()
        self.arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    def _read_gps_from_topic(self, topic):
        self.pose_stamped_gps = topic
    def _read_position_from_topic(self, topic):
        self.pose_stamped = topic

    def _get_current_position(self):
        return self.pose_stamped

    def set_target_pose(self, x, y, z):

        msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),  # stamp should update
        )

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        # For demo purposes we will lock yaw/heading to north.
        self._last_target_pose = msg
        self.set_pos.publish(msg)

    def arm(self, bool):
        rospy.wait_for_service("/mavros/cmd/arming")
        response = self.arming_cl(value=True)
        #rospy.loginfo(response)

    def set_offboard(self):
        rospy.wait_for_service('/mavros/set_mode')
        response = self.change_mode(custom_mode="OFFBOARD")
        #rospy.loginfo(response)


msg = [50, 50, 50]
def mission_ended(uav, rate):
    current_pos = uav._get_current_position()

    def is_near(cur, tar):
        return abs(cur - tar) < 0.5
    if is_near(current_pos.pose.position.x, msg[0]) and \
        is_near(current_pos.pose.position.y, msg[1]) and \
        is_near(current_pos.pose.position.z, msg[2]):
        return True
    return False

def test():
    rospy.init_node("gps_test")
    rate = rospy.Rate(10)
    uav = UAV()
    uav.arm(True)
    uav.set_offboard()
    print(uav.pose_stamped_gps)

    while not mission_ended(uav, rate):
        #print("helloo")
        uav.arm(True)
        uav.set_offboard()
        uav.set_target_pose(msg[0], msg[1], msg[2])

        rate.sleep()
    print(uav.pose_stamped_gps)

if __name__ == "__main__":
    test()