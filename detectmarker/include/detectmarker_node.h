#ifndef DETECTMARKER_NODE_H
#define DETECTMARKER_NODE_H


///ROS include
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

///MSGs includes
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>


// OpenCV include
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

///TF includes
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

///ROS Publisher
ros::Publisher pub; //Global to be use outside main
geometry_msgs::Twist vel;
bool color_red,color_blue,color_green,cross,tri,circle;

#endif
