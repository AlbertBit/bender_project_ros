//INPUT chi_des = {yd_des, ym_des, zd_des, zm_des}, deformation inputs
//OUTPUT X_des = {Xl_des, Xr_des}, end effectors' positions

//shape and position control is performed in the YZ plane
// with positive directions on the left and upward respectively

//in an initial stage the INPUT will be hardcoded, while secondly it
//will be provided by subscribing to another node


/*void deformation_callback(const deformation_msgs::deformation::ConstPtr& deformation_input_msg, deformation_msgs::deformation* deformation_input) {
  *deformation_input = deformation_input_msg;
}*/
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
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <tf2_eigen/tf2_eigen.h>

#define N_JOINTS 7
#define DELTA 0.03
#define OFFSET_Y 0.0//due to different end effectors
#define OFFSET_Z 0.061//due to different end effectors
#define N_INPUTS 4
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

double get_input(int index, geometry_msgs::Pose pose_left, geometry_msgs::Pose pose_right) {

    switch (index) {
        case 0:
              return pose_left.position.y-pose_right.position.y;
        case 1:
              return (pose_left.position.y+pose_right.position.y)/2;
        case 2:
              return pose_left.position.z-pose_right.position.z;
        case 3:
              return (pose_left.position.z+pose_right.position.z)/2;
        default:
            return 10e9;

    }

}

int main(int argc, char **argv)
{

        ros::init(argc, argv, "calibration");
        ros::NodeHandle n;

        const double dt = 5e-3;
        ros::Rate loop_rate(1/dt);

        ros::Publisher pub_left = n.advertise<geometry_msgs::PoseArray>("ee_pose_left", 1);
        ros::Publisher pub_right = n.advertise<geometry_msgs::PoseArray>("ee_pose_right", 1);
        sensor_msgs::JointState joint_state_left, joint_state_right;
        ros::Subscriber sub_left = n.subscribe<sensor_msgs::JointState>("left_joint_states", 1, boost::bind(joints_callback, _1, &joint_state_left));
        ros::Subscriber sub_right = n.subscribe<sensor_msgs::JointState>("right_joint_states", 1, boost::bind(joints_callback, _1, &joint_state_right));

        //maybe create yaml file
        double xm_des_base = 0.60;
        double yd_des_base = 0.50;
        double ym_des_base = 0.00;
        double zd_des_base = 0.0;
        double zm_des_base = 1.22;

        std::vector<double> xm_des, yd_des, ym_des, zd_des, zm_des;
        std::vector<std::vector<double>> ee_inputs(N_INPUTS);

        xm_des.clear();
        xm_des.push_back(xm_des_base);

        yd_des.clear();
        yd_des.push_back(yd_des_base);
        yd_des.push_back(yd_des_base+DELTA);
        yd_des.push_back(yd_des_base);
        yd_des.push_back(yd_des_base-DELTA);
        yd_des.push_back(yd_des_base);

        ym_des.clear();
        ym_des.push_back(ym_des_base);
        ym_des.push_back(ym_des_base+DELTA);
        ym_des.push_back(ym_des_base);
        ym_des.push_back(ym_des_base-DELTA);
        ym_des.push_back(ym_des_base);

        zd_des.clear();
        zd_des.push_back(zd_des_base);
        zd_des.push_back(zd_des_base+DELTA);
        zd_des.push_back(zd_des_base);
        zd_des.push_back(zd_des_base-DELTA);
        zd_des.push_back(zd_des_base);

        zm_des.clear();
        zm_des.push_back(zm_des_base);
        zm_des.push_back(zm_des_base+DELTA);
        zm_des.push_back(zm_des_base);
        zm_des.push_back(zm_des_base-DELTA);
        zm_des.push_back(zm_des_base);

        for(unsigned int i=0; i<3*1/dt; i++) {
            ros::spinOnce();
            loop_rate.sleep();
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

        double xm, yd, ym, zd, zm;
        xm = yd = ym = zd = zm = 0.0;
        double delta_xm, delta_yd, delta_ym, delta_zd, delta_zm;
        delta_xm = delta_yd = delta_ym = delta_zd = delta_zm = 0.0;


        geometry_msgs::Pose pose_left, pose_des_left, pose_des_left_base;
        geometry_msgs::Pose pose_right, pose_des_right, pose_des_right_base;

        while(ros::ok()) {
                pose_array_des_left.poses.clear();
                pose_array_des_right.poses.clear();

                forward_kinematics( pose_left, pose_right, joint_state_left, joint_state_right, joint_model_group_left, joint_model_group_right, kinematic_state );

                //offset correction

                pose_des_left = pose_left;
                pose_des_right = pose_right;

                pose_des_left.position.y = ym_des_base+yd_des_base/2;
                pose_des_left.position.z = zm_des_base+zd_des_base/2;
                pose_des_right.position.y = ym_des_base-yd_des_base/2+OFFSET_Y;
                pose_des_right.position.z = zm_des_base-zd_des_base/2+OFFSET_Z;

                pose_des_left_base = pose_des_left;
                pose_des_right_base = pose_des_right;

                for(int i=0; i<yd_des.size(); ++i) {



                  yd = pose_des_left.position.y-pose_des_right.position.y;
                  delta_yd = (yd_des[i]-OFFSET_Y)-yd;
                  ROS_INFO_STREAM("delta yd "<<delta_yd);


                  pose_des_left.position.y += delta_yd/2;
                  pose_des_right.position.y += -delta_yd/2;
                  pose_array_des_left.poses.push_back(pose_des_left);
                  pose_array_des_right.poses.push_back(pose_des_right);
                  pose_des_left = pose_des_left_base;
                  pose_des_right = pose_des_right_base;
              }

                for(int i=0; i<ym_des.size(); ++i) {

                  ym = (pose_des_left.position.y+pose_des_right.position.y) / 2;
                  delta_ym = (ym_des[i]+OFFSET_Y/2)-ym;
                  ROS_INFO_STREAM("delta ym "<<delta_ym);

                  pose_des_left.position.y += delta_ym;
                  pose_des_right.position.y += delta_ym;
                  pose_array_des_left.poses.push_back(pose_des_left);
                  pose_array_des_right.poses.push_back(pose_des_right);
                  pose_des_left = pose_des_left_base;
                  pose_des_right = pose_des_right_base;

                }

                for(int i=0; i<zd_des.size(); ++i) {

                  zd = pose_des_left.position.z-pose_des_right.position.z;
                  delta_zd = (zd_des[i]-OFFSET_Z)-zd;
                  ROS_INFO_STREAM("delta zd "<<delta_zd);

                  pose_des_left.position.z += delta_zd/2;
                  pose_des_right.position.z += -delta_zd/2;
                  pose_array_des_left.poses.push_back(pose_des_left);
                  pose_array_des_right.poses.push_back(pose_des_right);
                  pose_des_left = pose_des_left_base;
                  pose_des_right = pose_des_right_base;

                }

                for(int i=0; i<zm_des.size(); ++i) {

                  zm = (pose_des_left.position.z+pose_des_right.position.z) / 2;
                  delta_zm = (zm_des[i]+OFFSET_Z/2)-zm;
                  ROS_INFO_STREAM("delta zm "<<delta_zm);

                  pose_des_left.position.z += delta_zm;
                  pose_des_right.position.z += delta_zm;
                  pose_array_des_left.poses.push_back(pose_des_left);
                  pose_array_des_right.poses.push_back(pose_des_right);
                  pose_des_left = pose_des_left_base;
                  pose_des_right = pose_des_right_base;

                }

                for(int i=0; i<N_INPUTS*N_MOTIONS; ++i) {
                    ROS_INFO_STREAM("Desired poses left" <<i<<" "<<pose_array_des_left.poses[i] << "\n");
                    ROS_INFO_STREAM("Desired poses right" <<i<<" "<<pose_array_des_right.poses[i] << "\n");
                }
                pub_left.publish(pose_array_des_left);
                pub_right.publish(pose_array_des_right);
                loop_rate.sleep();
                ros::spinOnce();

        }

        return 0;

}
