#ifndef TP1TUTORIALNODE_H
#define TP1TUTORIALNODE_H


///ROS include
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

///MSGs includes
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>


///TF includes
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Twist.h>//ROS msg that will be published to turtlesim

///ROS Publisher
ros::Publisher pub; //Global to be use outside main
//Global vars
tf::TransformListener *listener ;

geometry_msgs::Twist vel;
geometry_msgs::PoseStamped pose_out_left;
geometry_msgs::PoseStamped pose_out_right;
bool first = true;
int counter2 = 0;
int counter1 = 0;
int print_rate = 50;
#endif
