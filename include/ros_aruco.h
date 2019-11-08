//
// Created by askat on 5/15/19.
//

#ifndef ROS_ARUCO_ROS_ARUCO_H
#define ROS_ARUCO_ROS_ARUCO_H

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include <map>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#define deg2rad M_PI/180.0f


class ArUcoNode{
private:
    /*camera calibration parameters */
    cv::Mat cameraMatrix; // camera matrix
    cv::Mat distCoeffs; // distortion coefficients
    int width, height, fps; // camera parameters

    /*aruco module parameters*/
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters; // detector parameters
    float aruco_len; //size of the marker (meter)
    float axis_len; //drawn axis length

    // ROS
    ros::NodeHandle n_h_; // ROS NodeHandle
    ros::Publisher pose_pub; // ROS topic publisher
    ros::ServiceClient gazebo_client;

    // Transformation map
    std::map<int,Eigen::Matrix4f> transMap;

    // text file name
    std::string  fileName = "/home/askat/catkin_ws/src/ros_aruco/data_test.txt";

    // thresholds for y_max and y_min
    float y_min = 1.2, y_max = 1.8;


public:
    ArUcoNode();        // constructor
    // to initialize homogeneous matrices
    void initialization();
    Eigen::Matrix4f trMat(Eigen::Matrix4f &Hm, float &x, float &y, float &z, float &rx, float &ry, float &rz);
    // returns the rotation matrix: camera -> marker
    Eigen::Matrix3f rotMat(double qw, double qx, double qy, double qz);
    // Euclidean distance between two points
    static double dist(const cv::Point2f &p1, const cv::Point2f &p2);
    // Compute area in image of a fiducial, using Heron's formula to find the area of two triangles
    static double calcFiducialArea(const std::vector<cv::Point2f> &pts);
    int arucoDetect();

};

#endif //ROS_ARUCO_ROS_ARUCO_H
