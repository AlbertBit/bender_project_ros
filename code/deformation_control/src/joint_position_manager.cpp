#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <control_blocks_msgs/ArrayStamped.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>
#include <algorithm>
#include <ros/package.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#define KUKA_PC
// #define ALBERTO_PC

#if defined(KUKA_PC) && defined(ALBERTO_PC)
  #error "This node must be compiled by defining *EITHER* ALBERTO_PC OR KUKA_PC (not both)"
#elif defined(KUKA_PC)
  #define CONTROL_MSG sensor_msgs::JointState
#elif defined(ALBERTO_PC)
  #define CONTROL_MSG control_blocks_msgs::ArrayStamped
#else
  #error "This node must be compiled by defining ALBERTO_PC or KUKA_PC"
#endif


// #define DELTA_W M_PI/6  // 30 deg/s
#define DELTA_W M_PI/50
#define DELTA_W_LOW 0.7*DELTA_W

#define N_JOINTS 7
#define THRESH M_PI/1600
// sensor_msgs::JointState joint_state;

double get_sign(double x) {

   if (x > 0) return 1;
   if (x < 0) return -1;
   return 0;

}

bool all_joints_at_home(bool* joint_at_home) {

    for(int k=0; k<2*N_JOINTS; k++) {

        if( !joint_at_home[k] ) {
            return false;
        }
    }
    return true;
}

void joints_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg, sensor_msgs::JointState* joint_state) {
    joint_state->position = joint_state_msg->position;
}

void ee_pose_callback(const geometry_msgs::PoseArray::ConstPtr& pose_msg, geometry_msgs::PoseArray* pose) {
  pose->poses = pose_msg->poses;
}




bool read_joint_position(std::string path, double* desired_position, std::vector<double> joint_values_left, std::vector<double> joint_values_right, bool* joint_at_home) {



  std::ifstream ifs(path);       // open the file
  std::string tempstr;            // declare a temporary string
  char delimiter;

  if(!std::getline(ifs, tempstr)) {
      ROS_ERROR("Some issue in reading the file joint_positions.txt");
      return false;
  }

  if(std::getline(ifs, tempstr)) {
    std::istringstream iss(tempstr);
    for(int k=0; k<2*N_JOINTS; k++) {
      iss >> desired_position[k];
      joint_at_home[k] = false;
      if(k < N_JOINTS) {
          joint_values_left[k] = desired_position[k];

      } else {
          joint_values_right[k-N_JOINTS] = desired_position[k];
      }
    }
    iss >> delimiter;
  }
  return true;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "joint_position_manager");
    CONTROL_MSG array_left;
    CONTROL_MSG array_right;

    ros::NodeHandle n;

    ros::Publisher pub_left = n.advertise<CONTROL_MSG>("left_position", 1);
    ros::Publisher pub_right = n.advertise<CONTROL_MSG>("right_position", 1);

    std_msgs::Bool reached;
    reached.data = false;
    ros::Publisher pub_ack = n.advertise<std_msgs::Bool>("ack_joint_position", 1);

    sensor_msgs::JointState left_state, right_state;
    ros::Subscriber sub_left = n.subscribe<sensor_msgs::JointState>("left_joint_states", 1, boost::bind(joints_callback, _1, &left_state));
    ros::Subscriber sub_right = n.subscribe<sensor_msgs::JointState>("right_joint_states", 1, boost::bind(joints_callback, _1, &right_state));

    geometry_msgs::PoseArray ee_pose_array_left, ee_pose_array_right;
    ros::Subscriber sub_left_pose = n.subscribe<geometry_msgs::PoseArray>("ee_pose_left", 1, boost::bind(ee_pose_callback, _1, &ee_pose_array_left));
    ros::Subscriber sub_right_pose = n.subscribe<geometry_msgs::PoseArray>("ee_pose_right", 1, boost::bind(ee_pose_callback, _1, &ee_pose_array_right));

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


    #if defined(ALBERTO_PC)
     auto& positions_left = array_left.data;
     auto& positions_right = array_right.data;
    #elif defined(KUKA_PC)
     auto& positions_left = array_left.position;
     auto& positions_right = array_right.position;
    #endif

    int n_rows=-1;
    bool joint_at_home[2*N_JOINTS];
    double desired_position[2*N_JOINTS];

    const double dt = 5e-3;
    assert( (dt > 1e-3) && "Control period should be more than 1ms in the kuka box; it is useless to try faster control" );
    ros::Rate loop_rate(1/dt);

    std::string path = ros::package::getPath("deformation_control")+"/src/joint_positions/joint_positions.txt";
    if(!read_joint_position(path, desired_position, joint_values_left, joint_values_right, joint_at_home)) {
        return 1;
    }

    //wait 3 seconds
    for(unsigned int i=0; i<3/dt; i++) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    if((left_state.position.size() < N_JOINTS) || (right_state.position.size() < N_JOINTS)) {
        ROS_ERROR(
            "After some time the joint state still does not contain enough values for"
            " initialization; the current number of joint positions is %d (left) and "
            "%d (right), while at least %d values are needed.",
            (int)left_state.position.size(), (int)right_state.position.size(),
            (int)(N_JOINTS)
        );
        return 1;
    }
    //use auxiliaries variables
    sensor_msgs::JointState vleft_state(left_state), vright_state(right_state);

    //get to positions from file
    positions_left.clear();
    positions_left.resize(N_JOINTS);
    positions_right.clear();
    positions_right.resize(N_JOINTS);

    geometry_msgs::Pose pose_left;
    geometry_msgs::Pose pose_right;

    array_left.header.stamp = array_right.header.stamp = ros::Time::now();


    while(!all_joints_at_home(joint_at_home)) {


        for(int k = 0; k < N_JOINTS; k++) {

            if (!(abs( desired_position[k]-left_state.position[k])<THRESH)) {
                double sign = get_sign(desired_position[k]-left_state.position[k]);
                vleft_state.position[k] += sign*DELTA_W*dt;//control law
            } else {
                joint_at_home[k] = true;
            }

            if (!(abs( desired_position[k+N_JOINTS]-right_state.position[k])<THRESH)) {
                double sign = get_sign(desired_position[k+N_JOINTS]-right_state.position[k]);
                vright_state.position[k] += sign*DELTA_W*dt;//control law
            } else {
                joint_at_home[k+N_JOINTS] = true;
            }

            positions_left[k]=vleft_state.position[k];
            positions_right[k]=vright_state.position[k];

            joint_values_left[k] = positions_left[k];
            joint_values_right[k] = positions_right[k];
        }

        kinematic_state->setJointGroupPositions(joint_model_group_right, joint_values_right);
        kinematic_state->setJointGroupPositions(joint_model_group_left, joint_values_left);
        // Forward Kinematics

        const Eigen::Affine3d& end_effector_state_right = kinematic_state->getGlobalLinkTransform("kuka_right_A6");
        const Eigen::Affine3d& end_effector_state_left = kinematic_state->getGlobalLinkTransform("kuka_left_A6");

        pose_left = tf2::toMsg(end_effector_state_left);
        pose_right = tf2::toMsg(end_effector_state_right);

        pub_left.publish(array_left);
        pub_right.publish(array_right);
        loop_rate.sleep();
        ros::spinOnce();

    }

    //read vector of poses to be executed
    static const std::string PLANNING_GROUP_LEFT = "arm_world_left";
    static const std::string PLANNING_GROUP_RIGHT = "arm_world_right";


    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    moveit_msgs::RobotTrajectory trajectory_left, trajectory_right;
    double fraction = 0;

    std::vector<geometry_msgs::Pose> waypoints_left, waypoints_right;
    geometry_msgs::Pose target_pose_left, target_pose_right;
    double desired_position_left[N_JOINTS];
    double desired_position_right[N_JOINTS];

    int n_ee_poses_left = ee_pose_array_left.poses.size();
    int n_ee_poses_right = ee_pose_array_left.poses.size();
    int n_ee_poses = std::max(n_ee_poses_left, n_ee_poses_right);

    ROS_INFO("n_ee_poses %d", n_ee_poses);



    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group_left(PLANNING_GROUP_LEFT);
    const robot_state::JointModelGroup* jmg_left = move_group_left.getCurrentState()->getJointModelGroup(PLANNING_GROUP_LEFT);
    moveit::planning_interface::MoveGroupInterface move_group_right(PLANNING_GROUP_RIGHT);
    const robot_state::JointModelGroup* jmg_right = move_group_right.getCurrentState()->getJointModelGroup(PLANNING_GROUP_RIGHT);
    //geometry_msgs::PoseArray ee_goals_left = ee_pose_array_left;
    //geometry_msgs::PoseArray ee_goals_right = ee_pose_array_right;

    for(int h=0; h<std::max(ee_pose_array_left.poses.size(), ee_pose_array_left.poses.size()); h++) {

        waypoints_left.clear();
        waypoints_right.clear();

        waypoints_left.push_back(pose_left);
        target_pose_left = ee_pose_array_left.poses[h];
        waypoints_left.push_back(target_pose_left);

        waypoints_right.push_back(pose_right);
        target_pose_right = ee_pose_array_right.poses[h];
        waypoints_right.push_back(target_pose_right);


        fraction = move_group_left.computeCartesianPath(waypoints_left, eef_step, jump_threshold, trajectory_left);
        ROS_INFO_STREAM("left arm, to\n"<<target_pose_left.position);
        ROS_INFO_NAMED("left arm", "Cartesian path (%d): %.2f%% acheived, %lu points found", h, fraction * 100.0, trajectory_left.joint_trajectory.points.size());
        fraction = move_group_right.computeCartesianPath(waypoints_right, eef_step, jump_threshold, trajectory_right);
        ROS_INFO_STREAM("right arm, to\n"<<target_pose_right.position);
        ROS_INFO_NAMED("right arm", "Cartesian path (%d): %.2f%% acheived, %lu points found", h, fraction * 100.0, trajectory_right.joint_trajectory.points.size());

        int n_waypoints_left = trajectory_left.joint_trajectory.points.size();
        int n_waypoints_right = trajectory_right.joint_trajectory.points.size();
        int n_waypoints_max = std::max(n_waypoints_left, n_waypoints_right);
        int i=0;


        while(ros::ok() && i < n_waypoints_max) {

            for(int k=0; k<2*N_JOINTS; k++) {
                joint_at_home[k] = false;
            }

            for(int k=0; k<N_JOINTS; k++) {
                desired_position_left[k]  = trajectory_left.joint_trajectory.points[n_waypoints_left-1].positions[k];
                desired_position_right[k] = trajectory_right.joint_trajectory.points[n_waypoints_right-1].positions[k];
                if( i < n_waypoints_left ) {
                  desired_position_left[k]  = trajectory_left.joint_trajectory.points[i].positions[k];
                }
                if( i < n_waypoints_right ) {
                  desired_position_right[k]  = trajectory_right.joint_trajectory.points[i].positions[k];
                }

            }

            array_left.header.stamp = array_right.header.stamp = ros::Time::now();

            int l=0;
            while(!all_joints_at_home(joint_at_home)) {

                for(int k = 0; k < N_JOINTS; k++) {

                    if (!(abs( desired_position_left[k]-left_state.position[k])<THRESH)) {
                        double sign = get_sign(desired_position_left[k]-left_state.position[k]);
                        vleft_state.position[k] += sign*DELTA_W_LOW*dt;//control law
                    }
                    else {
                        joint_at_home[k] = true;
                    }

                    if (!(abs( desired_position_right[k]-right_state.position[k])<THRESH)) {
                        double sign = get_sign(desired_position_right[k]-right_state.position[k]);
                        vright_state.position[k] += sign*DELTA_W_LOW*dt;//control law
                    }
                    else {
                        joint_at_home[k+N_JOINTS] = true;
                    }

                    positions_left[k]=vleft_state.position[k];
                    positions_right[k]=vright_state.position[k];

                    joint_values_left[k] = positions_left[k];
                    joint_values_right[k] = positions_right[k];
                }

                kinematic_state->setJointGroupPositions(joint_model_group_right, joint_values_right);
                kinematic_state->setJointGroupPositions(joint_model_group_left, joint_values_left);

                // Forward Kinematics

                const Eigen::Affine3d& end_effector_state_right = kinematic_state->getGlobalLinkTransform("kuka_right_A6");
                const Eigen::Affine3d& end_effector_state_left = kinematic_state->getGlobalLinkTransform("kuka_left_A6");

                pose_left = tf2::toMsg(end_effector_state_left);
                pose_right = tf2::toMsg(end_effector_state_right);

                path = ros::package::getPath("deformation_control")+"/src/joint_positions/robot_end_effectors.txt";
                std::ofstream ofs2;
                ofs2.open(path, std::ios::app);
                ofs2<<pose_left.position.y<<" "<<pose_left.position.z<<" ";
                ofs2<<pose_right.position.y<<" "<<pose_right.position.z<<" ";
                ofs2<<"\n";
                ofs2.close();


                if(l%2==0) {
                  pub_left.publish(array_left);
                  pub_right.publish(array_right);
                }else {
                  pub_right.publish(array_right);
                  pub_left.publish(array_left);
                }
                l++;
                loop_rate.sleep();
                ros::spinOnce();


            }
            i++;

        }

        reached.data = true;
        pub_ack.publish(reached);
        ros::spinOnce();

        //wait 1 seconds
        for(unsigned int i=0; i<0.2/dt; i++) {
            ros::spinOnce();
            loop_rate.sleep();
        }



    }

    spinner.stop();
    ROS_INFO_STREAM("path reached");
    /*while(ros::ok()) {

        pub_left.publish(array_left);
        pub_right.publish(array_right);
        loop_rate.sleep();
        ros::spinOnce();
    }*/


    return 0;

}
