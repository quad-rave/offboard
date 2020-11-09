if __name__ == '__main__':
    rospy.init_node('gapter_pilot_node', anonymous=True)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    position_pub = rospy.Publisher('/mavros/setpoint_position/local', TwistStamped, queue_size=10)

    myLoop()


def myLoop():
   x='1'
   while ((not rospy.is_shutdown())and (x in ['1','2','3','4','5','6','7'])):
       menu()
       x = raw_input("Enter your input: ");
       if (x=='1'):
           setGuidedMode()
       elif(x=='2'):
           setStabilizeMode()
       elif(x=='3'):
           setArm()
       elif(x=='4'):
           setDisarm()
       elif(x=='5'):
           setTakeoffMode()
       elif(x=='6'):
           setLandMode()
       elif(x=='7'):
           global latitude
           global longitude
           print ("longitude: %.7f" %longitude)
           print ("latitude: %.7f" %latitude)
       else:
           print "Exit"


def setGuidedMode():
   rospy.wait_for_service('/mavros/set_mode')
   try:
       flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
       isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
   except rospy.ServiceException, e:
       print "service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e


def setStabilizeMode():
   rospy.wait_for_service('/mavros/set_mode')
   try:
       flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
       isModeChanged = flightModeService(custom_mode='STABILIZE') #return true or false
   except rospy.ServiceException, e:
       print "service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e


def setArm():
   rospy.wait_for_service('/mavros/cmd/arming')
   try:
       armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
       armService(True)
   except rospy.ServiceException, e:
       print "Service arm call failed: %s"%e


def setDisarm():
   rospy.wait_for_service('/mavros/cmd/arming')
   try:
       armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
       armService(False)
   except rospy.ServiceException, e:
       print "Service arm call failed: %s"%e

def setTakeoffMode():
   rospy.wait_for_service('/mavros/cmd/takeoff')
   try:
       takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
       takeoffService(altitude = 2, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
   except rospy.ServiceException, e:
       print "Service takeoff call failed: %s"%e

def setLandMode():
   rospy.wait_for_service('/mavros/cmd/land')
   try:
       landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
       #http://wiki.ros.org/mavros/CustomModes for custom modes
       isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
   except rospy.ServiceException, e:
       print "service land call failed: %s. The vehicle cannot land "%e
