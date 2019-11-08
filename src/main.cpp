//
// Created by askat on 5/15/19.
//

#include <ros/ros.h>
#include "ros_aruco.h"

int main(int argc, char ** argv) {
    ros::init(argc, argv, "aruco_node");
    ArUcoNode node;
    ros::spin();
    return 0;
}