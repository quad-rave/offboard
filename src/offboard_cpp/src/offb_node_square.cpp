#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "std_msgs/String.h"

#include <string.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_position;

void state_callback(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}
double lerp(double a, double b, double interval){
  double result = a + (b-a) * interval;
  return result;
}

double manhattan_distance(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b){
  return abs(a.pose.position.x - b.pose.position.x) + abs(a.pose.position.y - b.pose.position.y) + abs(a.pose.position.z - b.pose.position.z);
}

void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node_feedback");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("mavros/state", 10, state_callback);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    ("mavros/setpoint_position/local", 10);
  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
    ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    ("mavros/set_mode");

  ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, position_cb);
  ros::Publisher state_publisher = nh.advertise<std_msgs::String>("state_publisher", 1000);
  std_msgs::String state;
  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ROS_INFO("Initializing...");
  // wait for FCU connection
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Connected.");

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;

  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;
  
  //send a few setpoints before starting
  for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  ros::Time time_start = ros::Time::now();  

  int waypoint_state = 0;


  while(ros::ok()){
    if( current_state.mode != "OFFBOARD" &&
	(ros::Time::now() - last_request > ros::Duration(5.0))){
      if( set_mode_client.call(offb_set_mode) &&
	  offb_set_mode.response.mode_sent){
	ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if( !current_state.armed &&
	  (ros::Time::now() - last_request > ros::Duration(5.0))){
	if( arming_client.call(arm_cmd) &&
	    arm_cmd.response.success){
	  ROS_INFO("Vehicle armed");
	}
	last_request = ros::Time::now();
      }
    }
    double time = (ros::Time::now()-time_start).toSec();
    int timeInt = time * 1000;
    double loopTime = timeInt % 16000;
    loopTime = loopTime / 1000.0;
    pose.pose.position.z = 6;

    double sqrLen = 8;

    double corners [8] = {
      0.0 * sqrLen, 0.0 * sqrLen, 
      1.0 * sqrLen, 0.0 * sqrLen,
      1.0 * sqrLen, 1.0 * sqrLen,
      0.0 * sqrLen, 1.0 * sqrLen
    };
    geometry_msgs::PoseStamped waypoints[4];
    waypoints[0].pose.position.x = 0;
    waypoints[0].pose.position.y = 0;
    waypoints[0].pose.position.z = 5;
    waypoints[1].pose.position.x = 10;
    waypoints[1].pose.position.y = 0;
    waypoints[1].pose.position.z = 5;
    waypoints[2].pose.position.x = 10;
    waypoints[2].pose.position.y = 10;
    waypoints[2].pose.position.z = 5;
    waypoints[3].pose.position.x = 0;
    waypoints[3].pose.position.y = 10;
    waypoints[3].pose.position.z = 5;

    //ROS_INFO("POS X:" + std::to_string(local_position.pose.position.x));
    //ROS_INFO("POS Y:" + std::to_string(local_position.pose.position.y));
    double error_margin = 1;
    state.data = std::to_string(waypoint_state);

    if(waypoint_state == 0){
      pose.pose.position = waypoints[0].pose.position;
      double dist = manhattan_distance(local_position, waypoints[0]);
      if(dist < error_margin){
        waypoint_state = 1;
        state_publisher.publish(state);
      }
    }
    else if(waypoint_state == 1){
      pose.pose.position = waypoints[1].pose.position;
      double dist = manhattan_distance(local_position, waypoints[1]);
      if(dist < error_margin){
        waypoint_state = 2;
        state_publisher.publish(state);

      }
    }
    else if(waypoint_state == 2){
      pose.pose.position = waypoints[2].pose.position;
      double dist = manhattan_distance(local_position, waypoints[2]);
      if(dist < error_margin){
        waypoint_state = 3;
        state_publisher.publish(state);

      }
    }
    else if(waypoint_state == 3){
      pose.pose.position = waypoints[3].pose.position;
      double dist = manhattan_distance(local_position, waypoints[3]);
      if(dist < error_margin){
        waypoint_state = 4;
        state_publisher.publish(state);

      }
    }
    else if(waypoint_state == 4)
    {
      pose.pose.position = waypoints[0].pose.position;
      double dist = manhattan_distance(local_position, waypoints[0]);
      if(dist < error_margin){
        waypoint_state = 5;
        state_publisher.publish(state);

      }
    }
    else if(waypoint_state == 5){
      ROS_INFO("Landing. . .");
      arm_cmd.request.value = false;
      state_publisher.publish(state);
      break;
    }
    
    local_pos_pub.publish(pose);


    // Update the desired pose:
    //pose.pose.position.x = sin(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());
    //pose.pose.position.y = cos(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());

    //Update the desired velocity:
    //vel.linear.x = 2.0*M_PI*0.1*cos(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());
    //vel.linear.y = -2.0*M_PI*0.1*sin(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());
    
    //local_pos_pub.publish(pose);
    //local_vel_pub.publish(vel);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

