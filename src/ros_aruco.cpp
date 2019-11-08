//
// Created by askat on 5/15/19.
//

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Transform.h>
#include "gazebo_msgs/SetModelState.h"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <vector>
#include <iostream>
#include "ros_aruco.h"

int ArUcoNode::arucoDetect() {
    cv::VideoCapture cap(1); // open the default camera

    // set camera parameters
    cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G'));
    cap.set(CV_CAP_PROP_FPS,fps);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,height);
    cap.set(CV_CAP_PROP_AUTOFOCUS, 0);

    cv::namedWindow("ArUcO Detection",1);
    cv::Mat frame, image;

    ros::Rate r(10); // 10 hz

    if(!cap.isOpened()) {
        ROS_WARN("%s", "Camera is not opened");
        return -1;
    } else {
        while(ros::ok()){
            //std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            cap >> image; // get a new frame from camera
            // sharpen the frame
            //cv::GaussianBlur(frame, image, cv::Size(0, 0), 5);
            //cv::addWeighted(frame, 1.5, image, -0.5, 0, image);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;

            // detect markers
            cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters);
            // if at least one marker detected
            if (!ids.empty()) {
                // for gazebo simulator
                gazebo_msgs::ModelState modelstate;

                geometry_msgs::PointStamped ps;
                ps.header.stamp = ros::Time::now();

                // draw detected markers
                cv::aruco::drawDetectedMarkers(image, corners, ids);
                std::vector<cv::Vec3d> rvecs, tvecs;
                // estimate pose of each marker
                cv::aruco::estimatePoseSingleMarkers(corners, aruco_len, cameraMatrix, distCoeffs, rvecs, tvecs);
                Eigen::Vector3f pos_in_cf; // position in the camera frame
                Eigen::Vector4f pos_in_mf; // position in the marker frame
                Eigen::Vector4f pos_in_bf; // position in the base frame
                Eigen::Matrix3f Rm; // rotation matrix: camera -> marker
                Eigen::Matrix4f Hb; // homogeneous matrix: marker -> base
                std::vector<Eigen::Vector4f> pos_vec;
                double area = 0.0;
                unsigned long ind = 0;
                // draw axis for each marker
                for(unsigned long i=0; i<ids.size(); ++i){

                    tf2::Quaternion q;
                    double angle = norm(rvecs[i]);
                    cv::Vec3d axis = rvecs[i] / angle;
                    q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

                    // marker position in the camera frame
                    pos_in_cf << tvecs[i][0], tvecs[i][1], tvecs[i][2];
                    // obtain the homogeneous matrix based on the marker id
                    Hb = transMap[ids[i]];
                    // obtain the rotation matrix: camera -> marker
                    Rm = rotMat(q.w(), q.x(), q.y(), q.z());
                    // obtain the camera position in the marker frame
                    pos_in_mf << -Rm*pos_in_cf, 1;
                    // obtain the camera position in the base frame
                    pos_in_bf = Hb*pos_in_mf;
                    // push to the vector
                    pos_vec.push_back(pos_in_bf);

                    if(i==0){
                        area =  calcFiducialArea(corners[i]);
                        ind = i;
                    }else{
                        if(area < calcFiducialArea(corners[i])){
                            area =  calcFiducialArea(corners[i]);
                            ind = i;
                        }
                    }

                    //std::cout << "Marker ID: " << ids[i] << "; Fiducial marker area pxls: " << area << std::endl;
                    std::cout << "Marker ID: " << ids[i] << "; Marker position in the CF: " << pos_in_cf.transpose() << std::endl;
                    std::cout << "Marker ID: " << ids[i] << "; Camera position in the MF: " << pos_in_mf.transpose() << std::endl;
                    std::cout << "Marker ID: " << ids[i] << "; Camera position in the BF: " << pos_in_bf.transpose() << std::endl;

                    cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], axis_len);
                }
                ps.point.x = pos_vec[ind](0);
                ps.point.y = pos_vec[ind](1);
                ps.point.z = pos_vec[ind](2);

                //std::cout << "The largest fiducial marker area pxls: " << area << std::endl;
                std::cout << "The closest marker ID: " << ids[ind] << "; Camera position in the BF: "  << ps.point.x << " " << ps.point.y << " " << ps.point.z << std::endl;

                modelstate.model_name = (std::string) "unit_sphere_0";
                modelstate.pose.position.x = pos_vec[ind](0);
                modelstate.pose.position.y = pos_vec[ind](1);
                modelstate.pose.position.z = pos_vec[ind](2);
                modelstate.pose.orientation.x = 0.0;
                modelstate.pose.orientation.y = 0.0;
                modelstate.pose.orientation.z = 0.0;
                modelstate.pose.orientation.w = 1.0;

                gazebo_msgs::SetModelState srv;
                srv.request.model_state = modelstate;
                
                std::cout << "------------------" << std::endl;
                if(ps.point.y < y_max && ps.point.y > y_min){
                    pose_pub.publish(ps);
                    if(gazebo_client.call(srv)){
                        ROS_INFO("Successfully sent to gazebo!!");
                    }
                    else{
                        ROS_ERROR("Failed to sent to gazebo! Error msg:%s",srv.response.status_message.c_str());
                    }
                }

            }
            cv::imshow("ArUcO Detection", image);
            cv::waitKey(1);
            r.sleep();
            /*std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
            std::cout<<1.0/time_span.count() <<std::endl;*/
        }
    }
}

double ArUcoNode::dist(const cv::Point2f &p1, const cv::Point2f &p2) {
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}

double ArUcoNode::calcFiducialArea(const std::vector<cv::Point2f> &pts) {
    const cv::Point2f &p0 = pts.at(0);
    const cv::Point2f &p1 = pts.at(1);
    const cv::Point2f &p2 = pts.at(2);
    const cv::Point2f &p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
    a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
    return a1+a2;
}

Eigen::Matrix4f ArUcoNode::trMat(Eigen::Matrix4f &Hm, float &x, float &y, float &z, float &rx, float &ry, float &rz) {
    float cos_t = std::cos(ry*deg2rad);
    float sin_t = std::sin(ry*deg2rad);

    Hm << cos_t, 0.0, sin_t, x,
           0.0, 1.0, 0.0, y,
           -sin_t, 0.0, cos_t, z,
           0, 0, 0, 1;

    return Hm;
}

Eigen::Matrix3f ArUcoNode::rotMat(double qw, double qx, double qy, double qz) {
    // Auxiliary variables to avoid repeated arithmetic
    double qw2 = qw*qw, qx2 = qx*qx, qy2 = qy*qy, qz2 = qz*qz;
    double qxqy = qx*qy, qwqz = qw*qz, qwqy = qw*qy, qxqz = qx*qz, qyqz = qy*qz, qwqx = qw*qx;
    Eigen::Matrix3f rotM;
    rotM << qw2 + qx2 - qy2 - qz2, 2*(qxqy - qwqz), 2*(qwqy + qxqz),
            2*(qwqz + qxqy), qw2 - qx2 + qy2 - qz2, 2*(qyqz - qwqx),
            2*(qxqz - qwqy), 2*(qwqx + qyqz), qw2 - qx2 - qy2 + qz2;

    //cout << "Rotation Matrix "<<rotM<<endl;
    return rotM.transpose();

}

void ArUcoNode::initialization() {
    Eigen::Matrix4f H;
    int id;
    float x, y, z, rx, ry, rz;

    std::string line;
    std::ifstream file(fileName);

    if (file.is_open())
    {
        std::cout << "Initialization";
        while ( getline(file, line) )
        {
            std::istringstream ss(line);
            ss >> id >> x >> y >> z >> rx >> ry >> rz;

            transMap[id] = trMat(H, x, y, z, rx, ry, rz);

            //std::cout << transMap[id] << '\n';
            std::cout << "." ;

        }
        std::cout << '\n';
        file.close();
    }

    else std::cout << "Unable to open file";

}

ArUcoNode::ArUcoNode():n_h_("") {
    // set camera parameters
    cameraMatrix = (cv::Mat_<float>(3,3) << 1378.438623, 0.000000, 924.854927, 0.000000, 1381.202474, 560.433664, 0.000000, 0.000000, 1.000000);
    distCoeffs = (cv::Mat_<float>(1,5) << 0.103206, -0.139285, 0.001162, -0.005657, 0.000000);
    width=1920, height=1080, fps=30;

    // set marker parameters
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    aruco_len = 0.14; axis_len = 0.2;

// set detector parameters
    parameters = new cv::aruco::DetectorParameters();
    n_h_.param<double>("adaptiveThreshConstant", parameters->adaptiveThreshConstant, 7);
    n_h_.param<int>("adaptiveThreshWinSizeMax", parameters->adaptiveThreshWinSizeMax, 23);
    n_h_.param<int>("adaptiveThreshWinSizeMin", parameters->adaptiveThreshWinSizeMin, 3);
    n_h_.param<int>("adaptiveThreshWinSizeStep", parameters->adaptiveThreshWinSizeStep, 10);
    n_h_.param<int>("cornerRefinementMaxIterations", parameters->cornerRefinementMaxIterations, 50);
    n_h_.param<double>("cornerRefinementMinAccuracy", parameters->cornerRefinementMinAccuracy, 0.001);
    n_h_.param<int>("cornerRefinementWinSize", parameters->cornerRefinementWinSize, 3);
#if OPENCV_MINOR_VERSION==2
    nh.param<bool>("doCornerRefinement",detectorParams->doCornerRefinement, true);
#else
    bool doCornerRefinement = true;
    n_h_.param<bool>("doCornerRefinement", doCornerRefinement, true);
    if (doCornerRefinement) {
        bool cornerRefinementSubPix = true;
        n_h_.param<bool>("cornerRefinementSubPix", cornerRefinementSubPix, true);
        if (cornerRefinementSubPix) {
            parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        }
        else {
            parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
        }
    }
    else {
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    }
#endif
    n_h_.param<double>("errorCorrectionRate", parameters->errorCorrectionRate , 0.6);
    n_h_.param<double>("minCornerDistanceRate", parameters->minCornerDistanceRate , 0.05);
    n_h_.param<int>("markerBorderBits", parameters->markerBorderBits, 1);
    n_h_.param<double>("maxErroneousBitsInBorderRate", parameters->maxErroneousBitsInBorderRate, 0.04);

    n_h_.param<int>("minDistanceToBorder", parameters->minDistanceToBorder, 1);
    n_h_.param<double>("minMarkerDistanceRate", parameters->minMarkerDistanceRate, 0.05);
    n_h_.param<double>("minMarkerPerimeterRate", parameters->minMarkerPerimeterRate, 0.04); // default 0.3
    n_h_.param<double>("maxMarkerPerimeterRate", parameters->maxMarkerPerimeterRate, 4.0);
    n_h_.param<double>("minOtsuStdDev", parameters->minOtsuStdDev, 5.0);
    n_h_.param<double>("perspectiveRemoveIgnoredMarginPerCell", parameters->perspectiveRemoveIgnoredMarginPerCell, 0.13);
    n_h_.param<int>("perspectiveRemovePixelPerCell", parameters->perspectiveRemovePixelPerCell, 4);
    n_h_.param<double>("polygonalApproxAccuracyRate", parameters->polygonalApproxAccuracyRate, 0.01);

    // ROS publisher parameters
    pose_pub = n_h_.advertise<geometry_msgs::PointStamped>("/camera_pose", 1);
    gazebo_client = n_h_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    // initialize transformation matrix
    initialization();
    // start pose estimation algorithm
    arucoDetect();
}
