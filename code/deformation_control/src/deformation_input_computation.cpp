#include "ros/ros.h"
#include <ros/package.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_blocks_msgs/ArrayStamped.h>
#include <sensor_msgs/JointState.h>
#include <marker_msgs/Marker.h>
#include <marker_msgs/MarkerDetection.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>
#include "eiquadprog.hpp"
#include <Eigen/Dense>
using namespace Eigen;
#define N_KEYPOINTS 3
#define N_INPUTS 4

//development: use different jacobian matrices
void marker_callback(const geometry_msgs::PoseArray::ConstPtr& marker_detected_msg, geometry_msgs::PoseArray* marker_detected) {
    marker_detected->poses = marker_detected_msg->poses;
}

bool read_matrix(std::string path, MatrixXd& matrix) {

    ROS_INFO_STREAM(path);
    std::ifstream ifs(path);       // open the file
    std::string tempstr;            // declare a temporary string
    char delimiter;
    double tempval=0;
    int m,n;

    if(!std::getline(ifs, tempstr)) {
        ROS_ERROR("Some issue in reading the file");
        return false;
    }

    std::istringstream iss(tempstr);
    iss >> m >> n;

    for(int i=0; i<m; i++) {
        std::getline(ifs, tempstr);
        std::istringstream iss(tempstr);
        for(int j=0; j<n; j++) {
            iss>>tempval;
            matrix(i,j)=tempval;
        }
        iss >> delimiter;
    }
    return true;
}
//revise
bool compute_error(VectorXd& alignment_error, geometry_msgs::PoseArray beam_markers, geometry_msgs::PoseArray fixture_markers) {

    for(int i=0; i<2*N_KEYPOINTS; i+=2) {
        alignment_error[i]=fixture_markers.poses[i/2].position.y-beam_markers.poses[i/2].position.y;
        alignment_error[i+1]=fixture_markers.poses[i/2].position.z-beam_markers.poses[i/2].position.z;
    }

    ROS_INFO_STREAM(alignment_error);

    std::string path = ros::package::getPath("deformation_control")+"/src/joint_positions/alignment_error.txt";
    std::ofstream ofs;
    ofs.open(path, std::ios::app);

    ROS_INFO_STREAM(alignment_error.norm()/N_KEYPOINTS);
    ofs<<alignment_error.norm()/N_KEYPOINTS<<"\n";

    ofs.close();

    path = ros::package::getPath("deformation_control")+"/src/joint_positions/beam_markers.txt";
    std::ofstream ofs1;
    ofs1.open(path, std::ios::app);

    for(int i=0; i<N_KEYPOINTS; i++) {
        ofs1<<beam_markers.poses[i].position.y<<" "<<beam_markers.poses[i].position.z<<" ";
    }
    ofs1<<"\n";
    ofs1.close();

    path = ros::package::getPath("deformation_control")+"/src/joint_positions/fixture_markers.txt";
    std::ofstream ofs2;
    ofs2.open(path, std::ios::app);

    for(int i=0; i<N_KEYPOINTS; i++) {
        ofs2<<fixture_markers.poses[i].position.y<<" "<<fixture_markers.poses[i].position.z<<" ";
    }
    ofs2<<"\n";
    ofs2.close();


    return true;
}


double compute_error_x(geometry_msgs::PoseArray beam_markers, geometry_msgs::PoseArray fixture_markers) {


    double scaling=0.45;
    double sum_beam=0;
    double sum_fixture=0;
    for(int i=0; i<N_KEYPOINTS; i++) {

        sum_beam+=beam_markers.poses[i].position.x;
        sum_fixture+=fixture_markers.poses[i].position.x;

    }

    return scaling*(sum_fixture-sum_beam)/N_KEYPOINTS;


}


bool get_beam_marker(geometry_msgs::PoseArray& beam_markers_replay) {

    std::string path = ros::package::getPath("joint_position_publisher")+"/src/joint_positions/beam_markers.txt";
    std::ifstream ifs(path);       // open the file
    std::string tempstr;
    double tempval=0;
    char delimiter;

    for(int i=0; i<2*N_KEYPOINTS; i++) {
        std::getline(ifs, tempstr);
        std::istringstream iss(tempstr);
        iss>>tempval;
        if(i%2==0) {
            beam_markers_replay.poses[i/2].position.y=tempval;
        } else {
            beam_markers_replay.poses[i/2].position.z=tempval;
        }

    }

    return true;


}




int main(int argc, char **argv)
{

    ros::init(argc, argv, "deformation_input_computation");
    ros::NodeHandle n;

    std_msgs::Float64MultiArray deformation_input_array;
    deformation_input_array.data.clear();
    deformation_input_array.data.resize(N_INPUTS+1);
    ros::Publisher pub_deformation = n.advertise<std_msgs::Float64MultiArray>("deformation_input", 1);

    geometry_msgs::PoseArray beam_markers, fixture_markers;
    beam_markers.poses.clear();
    fixture_markers.poses.clear();
    ros::Subscriber sub_beam = n.subscribe<geometry_msgs::PoseArray>("beam_markers", 1, boost::bind(marker_callback, _1, &beam_markers));
    ros::Subscriber sub_fixture = n.subscribe<geometry_msgs::PoseArray>("fixture_markers", 1, boost::bind(marker_callback, _1, &fixture_markers));

    const double dt = 5e-3;
    ros::Rate loop_rate(1/dt);

    std::string path = ros::package::getPath("deformation_control")+"/src/joint_positions/sensitivity_jacobian.txt";
    MatrixXd sensitivity_jacobian(2*N_KEYPOINTS, N_INPUTS);
    VectorXd alignment_error(2*N_KEYPOINTS);
    VectorXd deformation_input(N_INPUTS);
    VectorXd deformation_input_pub(N_INPUTS+1);
    if (!read_matrix(path, sensitivity_jacobian))
        return 1;

    ROS_INFO_STREAM("\n"<<sensitivity_jacobian);


    geometry_msgs::PoseArray fixture_markers_calc;//=fixture_markers;
    //get_beam_marker(fixture_markers_calc);//use last beam position
    while(ros::ok()){
         fixture_markers_calc=fixture_markers;
        int n_beam_markers = beam_markers.poses.size();
        int n_fixture_markers = fixture_markers_calc.poses.size();

        ROS_INFO_STREAM("\nfixture\n"<<fixture_markers_calc<<"beam\n"<<beam_markers);
        if(n_beam_markers != N_KEYPOINTS) {
            ROS_INFO_STREAM("\nfixture: "<<n_fixture_markers<<" beam: "<<n_beam_markers);
        } else {

            compute_error(alignment_error, beam_markers, fixture_markers_calc);
            ROS_INFO_STREAM("\n"<<alignment_error);
            //ROS_INFO_STREAM("fixture\n"<<fixture_markers<<"beam\n"<<beam_markers);

            MatrixXd J(2*N_KEYPOINTS, N_INPUTS);
            J = sensitivity_jacobian;
            VectorXd e(2*N_KEYPOINTS);
            e = alignment_error;
            MatrixXd G(N_INPUTS, N_INPUTS);
            G = J.transpose()*J;
            VectorXd g0(N_INPUTS);
            g0 = -e.transpose()*J;

            MatrixXd CI;//(4,8);
            //CI << 1, -1, 0,  0, 0,  0, 0,  0,
            //      0,  0, 1, -1, 0,  0, 0,  0,
            //      0,  0, 0,  0, 1, -1, 0,  0,
            //      0,  0, 0,  0, 0,  0, 1, -1;



            VectorXd ci0;//(8);
            //ci0 << 0.03, -0.03, 0.03, -0.03, 0.03, -0.03, 0.03, -0.03;//, -0.08, 0.115, 0.1;

            MatrixXd CE;
            VectorXd ce0;
            solve_quadprog(G, g0,  CE, ce0,  CI, ci0, deformation_input);
            //deformation_input=sensitivity_jacobian.bdcSvd(ComputeThinU | ComputeThinV).solve(alignment_error);
            //deformation_input=sensitivity_jacobian.colPivHouseholderQr().solve(alignment_error);

            double deformation_input_x=compute_error_x(beam_markers, fixture_markers_calc);


            deformation_input_pub.block(0,0,N_INPUTS,1) = 0.25*deformation_input;
            deformation_input_pub(N_INPUTS)=deformation_input_x;

            path = ros::package::getPath("deformation_control")+"/src/joint_positions/current_input.txt";
            std::ofstream ofs3;
            ofs3.open(path, std::ios::app);
            //deformation_input=(sensitivity_jacobian.transpose() * sensitivity_jacobian).ldlt().solve(sensitivity_jacobian.transpose() * alignment_error);
            ROS_INFO_STREAM("\n"<<deformation_input_pub);
            for(int i=0; i<N_INPUTS+1; i++) {
                deformation_input_array.data[i] = deformation_input_pub[i];


                ofs3<<deformation_input_array.data[i]<<" ";



            }
            ofs3<<"\n";
            ofs3.close();
            pub_deformation.publish(deformation_input_array);
        }

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
