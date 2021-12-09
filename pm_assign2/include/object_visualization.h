#ifndef PMASSIGN2_NODE_H
#define PMASSIGN2_NODE_H

///ROS include
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
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
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"


///PCL
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/transforms.h"
#include <pcl/features/normal_3d.h>


//PCL Registration
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>


///TF includes
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//Global vars
ros::Publisher pub;
cv::Mat glob_image;
sensor_msgs::CameraInfo::_K_type intrinsic_matrix;
std::string frame_id;
tf::TransformListener *listener ;
std::vector<float> dists;

#endif
