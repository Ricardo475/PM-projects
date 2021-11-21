#ifndef CALIBRATION_NODE_H
#define CALIBRATION_NODE_H


///ROS include
#include <ros/ros.h>

#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include<fstream>

///MSGs includes
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>


/// OpenCV include
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


//Global vars
std::string str;
float chess_dist = 0;
int chess_x = 0;
int chess_y = 0;

#endif
