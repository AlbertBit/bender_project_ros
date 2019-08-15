#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>
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
#include <ros/package.h>
#include <Eigen/Dense>
using namespace Eigen;
#define N_KEYPOINTS 3
void detection_callback(const marker_msgs::MarkerDetection::ConstPtr& detection_msg, marker_msgs::MarkerDetection* detection) {
    detection->markers = detection_msg->markers;
}

void transform_cf_to_wf(geometry_msgs::Pose& pose_wf, geometry_msgs::Pose pose_cf) {

    //R_x(-pi/2) R_y(pi/2)

    MatrixXf wf_T_cf(4,4);
    MatrixXf wf_R_cf(3,3);
    VectorXf wf_t_cf(3);
    VectorXf pose_cf4(4), pose_wf4(4);

    wf_R_cf <<  0, 0, 1,
                -1, 0, 0,
                0, -1, 0;
    wf_t_cf << -0.33, 0, 1.80;

    wf_T_cf.block(0,0,3,3) = wf_R_cf;
    wf_T_cf.block(0,3,3,1) = wf_t_cf;
    wf_T_cf.block(3,0,1,4) << 0, 0, 0, 1;//to be monitored

    pose_cf4<<pose_cf.position.x, pose_cf.position.y, pose_cf.position.z, 1;

    pose_wf4=wf_T_cf*pose_cf4;

    pose_wf.position.x=pose_wf4[0];
    pose_wf.position.y=pose_wf4[1];
    pose_wf.position.z=pose_wf4[2];

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_detector");
    ros::NodeHandle n;
    ros::Publisher pub_beam = n.advertise<geometry_msgs::PoseArray>("beam_markers", 1);
    ros::Publisher pub_fixture = n.advertise<geometry_msgs::PoseArray>("fixture_markers", 1);
    marker_msgs::MarkerDetection detection;
    ros::Subscriber sub_marker = n.subscribe<marker_msgs::MarkerDetection>("markersAruco", 1, boost::bind(detection_callback, _1, &detection));

    bool operation_mode = true;//true for alignment, false for calibration
    const double dt = 5e-3;
    ros::Rate loop_rate(1/dt);
    geometry_msgs::PoseArray beam_markers, fixture_markers;
    geometry_msgs::Pose marker_pose_wf;
    beam_markers.poses.resize(N_KEYPOINTS);
    fixture_markers.poses.resize(N_KEYPOINTS);

    int n_keypoints=N_KEYPOINTS;
    if(operation_mode) {
      n_keypoints = 2*N_KEYPOINTS;
    }

    bool fixture_detected=false;
    bool detection_ok=false;

    while(ros::ok()){

        detection_ok=false;
        int detection_size=detection.markers.size();
        if(!operation_mode) {

            if(detection_size == n_keypoints) {
              detection_ok=true;
            }

        } else {

            if(detection_size == n_keypoints) {
                detection_ok=true;

                if(!fixture_detected) {
                    fixture_detected=true;
                }
            }

            if(detection_size < n_keypoints && detection_size >= n_keypoints/2) {

                if(fixture_detected) {
                    int beam_markers_cnt=0;
                    for(int i=0; i<detection_size; i++) {
                        if(detection.markers[i].ids[0]<n_keypoints/2) {
                            ROS_INFO_STREAM(detection.markers[i].ids[0]);
                            beam_markers_cnt++;
                        }
                    }

                    if(beam_markers_cnt==n_keypoints/2) {
                        detection_ok=true;
                    }
                }

            }
        }



        if(detection_ok) {


            ROS_INFO_STREAM("camera frame");
            ROS_INFO_STREAM(detection);

            //you have to transform in the world frame
            ROS_INFO_STREAM("world frame");
            for(int i=0; i<detection_size; i++) {
                transform_cf_to_wf(marker_pose_wf, detection.markers[i].pose);

                ROS_INFO_STREAM(marker_pose_wf);
                if(i<N_KEYPOINTS){
                    beam_markers.poses[i]=marker_pose_wf;
                } else if(detection_size == 2*N_KEYPOINTS){
                    fixture_markers.poses[i-N_KEYPOINTS]=marker_pose_wf;
                }
            }

            pub_beam.publish(beam_markers);

            if(detection_size == 2*N_KEYPOINTS) {
                pub_fixture.publish(fixture_markers);
            }

        } else  {
            ROS_INFO("Waiting for beam markers %d/%d", detection_size, n_keypoints);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
