#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <std_msgs/Bool.h>
#include <cmath>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <marker_msgs/Marker.h>
#include <marker_msgs/MarkerDetection.h>
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
#define N_KEYPOINTS 3
#define N_MOTIONS 5

void ack_callback(const std_msgs::Bool::ConstPtr& ack_msg, std_msgs::Bool* ack) {
    ack->data = ack_msg->data;
}

void detection_callback(const marker_msgs::MarkerDetection::ConstPtr& detection_msg, marker_msgs::MarkerDetection* detection) {

    if(detection_msg->markers.size()==N_KEYPOINTS) {
        detection->markers = detection_msg->markers;
    }
}

void joints_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg, sensor_msgs::JointState* joint_state) {
    joint_state->position = joint_state_msg->position;
}

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

void marker_callback(const geometry_msgs::PoseArray::ConstPtr& marker_detected_msg, geometry_msgs::PoseArray* marker_detected) {
    marker_detected->poses = marker_detected_msg->poses;
}

int main(int argc, char **argv)
{

    double sensitivity_jacobian[2*N_KEYPOINTS][N_INPUTS];
    ros::init(argc, argv, "jacobian_computation");
    ros::NodeHandle n;

    std::vector<std::vector<double>> ee_inputs(N_INPUTS);
    std::vector<std::vector<geometry_msgs::PoseArray>> xyz_beam(N_INPUTS);
    std::vector<double> joint_values_right(N_JOINTS);
    std::vector<double> joint_values_left(N_JOINTS);

    const double dt = 5e-3;
    ros::Rate loop_rate(1/dt);

    ros::Publisher pub_left = n.advertise<geometry_msgs::PoseArray>("ee_pose_left", 1);
    ros::Publisher pub_right = n.advertise<geometry_msgs::PoseArray>("ee_pose_right", 1);

    sensor_msgs::JointState joint_state_left, joint_state_right;
    ros::Subscriber sub_left = n.subscribe<sensor_msgs::JointState>("left_joint_states", 1, boost::bind(joints_callback, _1, &joint_state_left));
    ros::Subscriber sub_right = n.subscribe<sensor_msgs::JointState>("right_joint_states", 1, boost::bind(joints_callback, _1, &joint_state_right));

    std_msgs::Bool joint_position_reached;
    ros::Subscriber sub_ack = n.subscribe<std_msgs::Bool>("ack_joint_position", 1, boost::bind(ack_callback, _1, &joint_position_reached));

    /*marker_msgs::MarkerDetection detection;
    ros::Subscriber sub_marker = n.subscribe<marker_msgs::MarkerDetection>("markersAruco", 1, boost::bind(detection_callback, _1, &detection));*/

    geometry_msgs::PoseArray beam_markers;
    beam_markers.poses.clear();
    beam_markers.poses.resize(N_KEYPOINTS);
    ros::Subscriber sub_beam = n.subscribe<geometry_msgs::PoseArray>("beam_markers", 1, boost::bind(marker_callback, _1, &beam_markers));


    //load the kinelatic model for forward kinematics to extract ee pose
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    const robot_state::JointModelGroup* joint_model_group_right = kinematic_model->getJointModelGroup("arm_world_right");
    const robot_state::JointModelGroup* joint_model_group_left = kinematic_model->getJointModelGroup("arm_world_left");

    const std::vector<std::string>& joint_names_right = joint_model_group_right->getVariableNames();
    const std::vector<std::string>& joint_names_left = joint_model_group_left->getVariableNames();

    geometry_msgs::PoseArray pose_array_des_left, pose_array_des_right;
    geometry_msgs::Pose pose_left, pose_des_left, pose_des_left_base;
    geometry_msgs::Pose pose_right, pose_des_right, pose_des_right_base;

    joint_state_left.position.clear();
    joint_state_right.position.clear();


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

    if((beam_markers.poses.size() < N_KEYPOINTS) ) {
        ROS_ERROR(
            "After some time the marker detection still does not contain enough values for"
            " initialization; the current marker detection "
            "%d, while at least %d values are needed.",
            (int)beam_markers.poses.size(),
            (int)(N_KEYPOINTS)
        );
        return 1;
    }


    for(int i=0; i<N_INPUTS; i++) {
        for(int j=0; j<N_MOTIONS; j++) {

            while(ros::ok() && !joint_position_reached.data) {
                forward_kinematics( pose_left, pose_right, joint_state_left, joint_state_right, joint_model_group_left, joint_model_group_right, kinematic_state );
                loop_rate.sleep();
                ros::spinOnce();
            }

            joint_position_reached.data = false;
            //take a screenshot of the end effectors and the markers
            double ee_input_f = get_input(i, pose_left, pose_right);
            ee_inputs[i].push_back(ee_input_f);
            xyz_beam[i].push_back(beam_markers);

            for(int k=0; k<N_KEYPOINTS; k++) {
                double x = xyz_beam[i][j].poses[k].position.x;
                double y = xyz_beam[i][j].poses[k].position.y;
                double z = xyz_beam[i][j].poses[k].position.z;
                ROS_INFO("input %i=%lf, motion %d, marker %d, pose [x y z]: %lf %lf %lf\n\n", i, ee_input_f, j, k, x, y, z);
            }
        }
    }

    for(int i=0; i < 2*N_KEYPOINTS; i+=2){//for each marker
        for(int j=0; j<N_INPUTS; j++){//for each input
            double sensitivity_jacobian_p_y = (xyz_beam[j][1].poses[i/2].position.y-xyz_beam[j][0].poses[i/2].position.y)/(ee_inputs[j][1]-ee_inputs[j][0]);
            double sensitivity_jacobian_m_y = (xyz_beam[j][3].poses[i/2].position.y-xyz_beam[j][2].poses[i/2].position.y)/(ee_inputs[j][3]-ee_inputs[j][2]);
            sensitivity_jacobian[i+1][j] = (sensitivity_jacobian_p_y + sensitivity_jacobian_m_y) / 2;
            double sensitivity_jacobian_p_z = (xyz_beam[j][1].poses[i/2].position.z-xyz_beam[j][0].poses[i/2].position.z)/(ee_inputs[j][1]-ee_inputs[j][0]);
            double sensitivity_jacobian_m_z = (xyz_beam[j][3].poses[i/2].position.z-xyz_beam[j][2].poses[i/2].position.z)/(ee_inputs[j][3]-ee_inputs[j][2]);
            sensitivity_jacobian[i][j] = (sensitivity_jacobian_p_z + sensitivity_jacobian_m_z) / 2;
            ROS_INFO_STREAM("delta_var "<<i<< "/ delta_input"<<j<<"="<< sensitivity_jacobian[i][j]);
            ROS_INFO_STREAM("delta_var "<<i+1<< "/ delta_input"<<j<<"="<< sensitivity_jacobian[i+1][j]);
        }
    }

    std::string path = ros::package::getPath("deformation_control")+"/src/joint_positions/sensitivity_jacobian.txt";
    std::ofstream ofs;
    ofs.open(path, std::ios::out);

    ofs << 2*N_KEYPOINTS <<" "<<N_INPUTS <<"\n";
    for(int i=2*N_KEYPOINTS-1; i >=0 ; i--){ //for each marker
        for(int j=0; j<N_INPUTS; j++){ //for each input
          ofs<<sensitivity_jacobian[i][j]<<" ";
        }
        ofs<<"\n";
    }

    ofs.close();
    return 0;
}
