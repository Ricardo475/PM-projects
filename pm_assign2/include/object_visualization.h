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
#include <pcl/common/distances.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/surface/gp3.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <shape_msgs/Mesh.h>


///TF includes
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

///Defines
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//Global vars
ros::Publisher pub;
cv::Mat glob_image;
sensor_msgs::CameraInfo::_K_type intrinsic_matrix;
std::vector<cv::Point3f> depth_map;
PointCloud::Ptr cloud_map,cloud_car;
std::string frame_id;
tf::TransformListener *listener ;
darknet_ros_msgs::BoundingBox closest_car;
int cam_height,cam_width, increment;
std::string min_print;
cv::Point3f left,right,up,down;
float dist_car = 999999;
float car_width, car_height;

#endif
