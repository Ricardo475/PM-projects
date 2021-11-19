#ifndef DETECTMARKER_NODE_H
#define DETECTMARKER_NODE_H


///ROS include
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <math.h>
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
#include <geometry_msgs/Twist.h>

///ROS Publisher
ros::Publisher pub; //Global to be use outside main
geometry_msgs::Twist vel;
geometry_msgs::PoseStamped pose_out_left;
geometry_msgs::PoseStamped pose_out_right;

//Global vars
tf::TransformListener *listener ;
image_transport::Publisher imagePub;
sensor_msgs::ImagePtr imageMsg;
std::string color,shape;
bool orientation= false,init=true;
struct dock{
 bool color_red,color_blue,color_green,cross,tri,circle,identified, avaiability;
 std::string shape;
 std::string color;
};
dock dock_left,dock_right;

bool first = true;
bool first_arm = false;
int counter_odo = 0;

int counter1 = 0;
int counter2 = 0;
//int stop_rate = 0;
int print_rate = 50;
#endif
