
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <std_msgs/Bool.h>
#include <cmath>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <tf2_eigen/tf2_eigen.h>

#define N_JOINTS 7
#define N_INPUTS 4
#define N_ITERATIONS 7
#define N_KEYPOINTS 3
#define N_MOTIONS 5


void forward_kinematics(geometry_msgs::Pose& pose_left, geometry_msgs::Pose& pose_right, sensor_msgs::JointState joint_state_left, sensor_msgs::JointState joint_state_right, const robot_state::JointModelGroup* joint_model_group_left, const robot_state::JointModelGroup* joint_model_group_right, robot_state::RobotStatePtr kinematic_state) {
  std::vector<double> joint_values_right(N_JOINTS);
  std::vector<double> joint_values_left(N_JOINTS);
  joint_values_left = joint_state_left.position;
  joint_values_right = joint_state_right.position;
  kinematic_state->setJointGroupPositions(joint_model_group_right, joint_values_right);
  kinematic_state->setJointGroupPositions(joint_model_group_left, joint_values_left);
  const Eigen::Affine3d& end_effector_state_right = kinematic_state->getGlobalLinkTransform("kuka_right_A6");
  const Eigen::Affine3d& end_effector_state_left = kinematic_state->getGlobalLinkTransform("kuka_left_A6");
  pose_left = tf2::toMsg(end_effector_state_left);
  pose_right = tf2::toMsg(end_effector_state_right);

}

void joints_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg, sensor_msgs::JointState* joint_state) {
    joint_state->position = joint_state_msg->position;
}

void deformation_callback(const std_msgs::Float64MultiArray::ConstPtr& deformation_input_msg, std_msgs::Float64MultiArray* deformation_input) {
    deformation_input->data = deformation_input_msg->data;
}

void ack_callback(const std_msgs::Bool::ConstPtr& ack_msg, std_msgs::Bool* ack) {
    ack->data = ack_msg->data;
}


int main(int argc, char **argv)
{

        ros::init(argc, argv, "ee_position_ctrl");
        ros::NodeHandle n;

        const double dt = 5e-3;
        ros::Rate loop_rate(1/dt);

        ros::Publisher pub_left = n.advertise<geometry_msgs::PoseArray>("ee_pose_left", 1);
        ros::Publisher pub_right = n.advertise<geometry_msgs::PoseArray>("ee_pose_right", 1);

        sensor_msgs::JointState joint_state_left, joint_state_right;
        ros::Subscriber sub_left = n.subscribe<sensor_msgs::JointState>("left_joint_states", 1, boost::bind(joints_callback, _1, &joint_state_left));
        ros::Subscriber sub_right = n.subscribe<sensor_msgs::JointState>("right_joint_states", 1, boost::bind(joints_callback, _1, &joint_state_right));

        std_msgs::Float64MultiArray deformation_input;
        ros::Subscriber sub_deformation = n.subscribe<std_msgs::Float64MultiArray>("deformation_input", 1, boost::bind(deformation_callback, _1, &deformation_input));


        std_msgs::Bool joint_position_reached;
        ros::Subscriber sub_ack = n.subscribe<std_msgs::Bool>("ack_joint_position", 1, boost::bind(ack_callback, _1, &joint_position_reached));



        for(unsigned int i=0; i<3*1/dt; i++) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        for(int i=0; i<deformation_input.data.size(); i++) {
            ROS_INFO_STREAM(deformation_input.data[i]);
        }



        if((joint_state_left.position.size() < N_JOINTS) || (joint_state_right.position.size() < N_JOINTS)) {
            ROS_ERROR(
                "After some time the joint state still does not contain enough values for"
                " initialization; the current number of joint positions is %d (left) and "
                "%d (right), while at least %d values are needed.",
                (int)joint_state_left.position.size(), (int)joint_state_right.position.size(),
                (int)(N_JOINTS)
            );
            return 1;
        }

        //load the kinelatic model for forward kinematics to extract ee pose
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

        const robot_state::JointModelGroup* joint_model_group_right = kinematic_model->getJointModelGroup("arm_world_right");
        const robot_state::JointModelGroup* joint_model_group_left = kinematic_model->getJointModelGroup("arm_world_left");

        const std::vector<std::string>& joint_names_right = joint_model_group_right->getVariableNames();
        const std::vector<std::string>& joint_names_left = joint_model_group_left->getVariableNames();

        std::vector<double> joint_values_right(N_JOINTS);
        std::vector<double> joint_values_left(N_JOINTS);

        geometry_msgs::PoseArray pose_array_des_left, pose_array_des_right;

        double delta_yd, delta_ym, delta_zd, delta_zm, delta_xm;


        geometry_msgs::Pose pose_left, pose_des_left, pose_des_left_base;
        geometry_msgs::Pose pose_right, pose_des_right, pose_des_right_base;


        //TO BE TESTED

        pose_array_des_left.poses.clear();
        pose_array_des_right.poses.clear();
        joint_position_reached.data=false;
        int i=0;
        int j=0;
        //forward_kinematics( pose_left, pose_right, joint_state_left, joint_state_right, joint_model_group_left, joint_model_group_right, kinematic_state );

        while(ros::ok() && i<N_INPUTS*N_ITERATIONS) {

            ROS_INFO_STREAM("Iteration "<<j);
            if(i%N_INPUTS==0) {
                forward_kinematics( pose_left, pose_right, joint_state_left, joint_state_right, joint_model_group_left, joint_model_group_right, kinematic_state );
                j++;

                pose_des_left = pose_left;
                pose_des_right = pose_right;
                ROS_INFO_STREAM("current position left: \n" << pose_des_left << "\n");
                ROS_INFO_STREAM("current position right: \n" << pose_des_right << "\n");



                delta_yd = deformation_input.data[0];
                delta_ym = deformation_input.data[1];
                delta_zd = deformation_input.data[2];
                delta_zm = deformation_input.data[3];
                delta_xm = deformation_input.data[4];


                pose_des_left.position.y += -delta_yd/2;
                pose_des_right.position.y += delta_yd/2;
                pose_array_des_left.poses.push_back(pose_des_left);
                pose_array_des_right.poses.push_back(pose_des_right);

                pose_des_left.position.y += delta_ym;
                pose_des_right.position.y += delta_ym;
                pose_array_des_left.poses.push_back(pose_des_left);
                pose_array_des_right.poses.push_back(pose_des_right);

                pose_des_left.position.z += -delta_zd/2;
                pose_des_right.position.z += delta_zd/2;
                pose_array_des_left.poses.push_back(pose_des_left);
                pose_array_des_right.poses.push_back(pose_des_right);

                pose_des_left.position.z += delta_zm;
                pose_des_right.position.z += delta_zm;
                pose_array_des_left.poses.push_back(pose_des_left);
                pose_array_des_right.poses.push_back(pose_des_right);
                //ROS_INFO_STREAM("desired positions left: \n" << pose_array_des_left << "\n");
                //ROS_INFO_STREAM("desired positions right: \n" << pose_array_des_right << "\n");
                ROS_INFO_STREAM("deformation_input: \n" << deformation_input << "\n");
            }


            if(i==N_ITERATIONS*N_INPUTS-1){
                pose_des_left.position.x+= delta_xm;
                pose_des_right.position.x+= delta_xm;

                pose_array_des_left.poses.push_back(pose_des_left);
                pose_array_des_right.poses.push_back(pose_des_right);
            }




            ROS_INFO_STREAM("waiting for ack...");
            while(ros::ok() && !joint_position_reached.data) {
                pub_left.publish(pose_array_des_left);
                pub_right.publish(pose_array_des_right);
                loop_rate.sleep();
                ros::spinOnce();
            }
            joint_position_reached.data=false;
            i++;


        }

        return 0;

}
