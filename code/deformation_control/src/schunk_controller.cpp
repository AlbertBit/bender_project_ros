   #include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <std_msgs/Float64MultiArray.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>
#include <ros/package.h>

#define THRESH M_PI/50
#define N_JOINTS 7

void open_schunk(std_msgs::Float64MultiArray& schunk_positions) {
  schunk_positions.data.clear();
  schunk_positions.data.resize(7);
  schunk_positions.data[0] = 0.0;//sdh_knuckle_joint
  schunk_positions.data[1] = -M_PI/4;//sdh_thumb_2_joint
  schunk_positions.data[2] = 0.0;//sdh_thumb_3_joint
  schunk_positions.data[3] = -M_PI/4;//sdh_finger12_joint
  schunk_positions.data[4] = 0.0;//sdh_finger13_joint
  schunk_positions.data[5] = -M_PI/4;//sdh_finger22_joint
  schunk_positions.data[6] = 0.0;//sdh_finger23_joint

}

void close_schunk(std_msgs::Float64MultiArray& schunk_positions) {
  schunk_positions.data.clear();
  schunk_positions.data.resize(7);
  schunk_positions.data[0] = 0;//sdh_knuckle_joint
  schunk_positions.data[1] = -0.85*M_PI/6;//sdh_thumb_2_joint
  schunk_positions.data[2] = M_PI/3;//sdh_thumb_3_joint
  schunk_positions.data[3] = -0.85*M_PI/6;//sdh_finger12_joint
  schunk_positions.data[4] = M_PI/3;//sdh_finger13_joint
  schunk_positions.data[5] = -0.85*M_PI/6;//sdh_finger22_joint
  schunk_positions.data[6] = M_PI/3;//sdh_finger23_joint

}

void joints_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg, control_msgs::JointTrajectoryControllerState* state) {
  state->error = msg->error;
}

bool all_joints_at_home(auto errors) {

    for(int k = 0; k < N_JOINTS; k++) {

        if (!(abs(errors[k])<THRESH)) {
            return false;
        }

    }
    ROS_INFO("DESIRED POSITION REACHED");
    return true;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "schunk_controller");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/schunk_sdh/set_position", 1);

  control_msgs::JointTrajectoryControllerState current_positions;
  ros::Subscriber sub = n.subscribe<control_msgs::JointTrajectoryControllerState>("/schunk_sdh/joint_trajectory_controller/state", 2, boost::bind(joints_callback, _1, &current_positions));

  ros::Rate loop_rate(5);

  for(unsigned int i=0; i<3*5; i++) {
      ros::spinOnce();
      loop_rate.sleep();
  }


  std_msgs::Float64MultiArray desired_positions;


  close_schunk(desired_positions);

  while (ros::ok()){// && !all_joints_at_home(current_positions.error.positions)) {

    pub.publish(desired_positions);
    loop_rate.sleep();
    ros::spinOnce();
    ROS_INFO("GRIPPER CLOSING");
  }


  return 0;
}
