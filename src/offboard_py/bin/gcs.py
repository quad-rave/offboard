#!/usr/bin/env python2
# vim:set ts=4 sw=4 et:

import rospy
import sys
from swarm.msg import Mission, Team
from std_msgs.msg import String
import _thread as thread
import signal
from uav import UAV

def interupt_handler(sig_num, frame):
    print ("Exiting")
    sys.exit(-2) #Terminate process here as catching the signal removes the close process behavior of Ctrl-C

class GCS():

    def __init__(self, mission_file_path="swarm/gcs-messages/missions.gcs", team_file_path="swarm/gcs-messages/teams.gcs"):
        rospy.init_node('ground_control', anonymous=False)
        self.rate = rospy.Rate(1)  # 10hz


        self.missions = []
        self.teams = []
        
        self.mission_file_path = mission_file_path
        self.team_file_path =  team_file_path

        try:
            with open(self.mission_file_path) as f:
                self.missions = f.readlines()
            with open(self.team_file_path) as f:
                self.teams = f.readlines()
        except Exception as e:
            print("Check the .gcs files. ")
            print(e)    
        
        self.body = { }
        self.branch = None
        self.status = ""

    def assign(self, publisher, team):
        self.check_status(self.branch, team)
        # Wait until the team is reachable.
        while self.status != "idle":
            self.rate.sleep()
        while not rospy.is_shutdown() and self.status != "busy":      
            rospy.loginfo(self.body)
            publisher.publish(Mission(**self.body))
            self.rate.sleep()
        rospy.loginfo("Task is completed!")

    def check_status(self, branch="", team="", verbose=False):            
        def status(data):
            if data.data == "":
                rospy.loginfo("Team {} is unreachable".format(team))
            else:
                rospy.loginfo("Status: {}".format(data.data))

            self.status = data.data


        rospy.Subscriber("gcs/{}/{}/status".format(branch, team), String, status, queue_size=10)
        self.rate.sleep()

class AssignTeams(GCS):
    
    def __init__(self):
        GCS.__init__(self)
        self.body = {"team_name": None, "uav_list": None}         # first is the leader
        self.branch = "team"

        
        for team in self.teams:            
            team_components = team.rstrip().split(",")
            self.body["team"] = team_components[0]
            self.body["uav_list"] = team_components[1:]
            team = self.body["team"]

            team_publisher = rospy.Publisher("gcs/team/{}".format(team), Team, queue_size=10)

            self.check_status(branch="team", team=team)
            self.assign(team_publisher, team)


class AssignMissions(GCS):

    def __init__(self):
        GCS.__init__(self)
        self.body = {"team": None, "command": None, "args": None}
        self.branch = "mission"

        for mission in self.missions:            
            mission_components = mission.rstrip().split(",")
            self.body["team"] = mission_components[0]
            self.body["command"] = mission_components[1]
            self.body["args"] = mission_components[2:]
            team = self.body["team"]

            mission_publisher = rospy.Publisher("gcs/mission/{}".format(team), Mission, queue_size=10)
            rospy.loginfo(self.body)
            self.check_status(branch=self.branch, team=team)
            self.assign(mission_publisher, team)


base_commands = {
    "m": AssignMissions,
    "t": AssignTeams,
}


if __name__ == '__main__':
    try:
        signal.signal(signal.SIGINT, interupt_handler)
        while (raw_input(">") != "exit"):
            try: 
                key = raw_input(">")
                base_commands[key]
            except KeyError:
                print("Mission not found")

    except rospy.ROSInterruptException:
        pass