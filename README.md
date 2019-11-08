# ros_aruco
This is a ROS package for pose estimation using ArUco markers.
The package doesn't subscribe to topics such as Image and Camera Info compared to other existing ROS packages. 
This is done to avoid latency between image publisher and subscriber. Therefore, images are acquired directly inside of this package and parameters of the camera are predefined.

How to use? 
1. Clone or download the package to your catkin workspace/src folder.
2. Set your camera parameters such a camera matrix, distortion coefficients,
width, height and frames per second in src/ros_aruco.cpp
3. Compile the package using catkin_make or catkin build.
4. Connect your USB camera to your PC.
5. Run the detection node via typing the following command on the terminal:
rosrun ros_aruco aruco_node
