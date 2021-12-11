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
#include <pcl_msgs/PolygonMesh.h>

///PCL
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/transforms.h"
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <shape_msgs/Mesh.h>

///TF includes
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


///Defines
#define MAX_DEPTH 30
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

typedef pcl::Normal Normal; ; //!< Data type definition for a PointXYZNormal
typedef pcl::PointCloud<Normal> PointCloudNormal;
//Global vars

ros::Publisher pub,pub_car,pub_visualization, pub_pose,pub_car_mesh,pub_cloudmap;
cv::Mat glob_image,depth_map_image;
tf::TransformListener *listener ;
std::vector<cv::Point3f> depth_map;
cv::Point3f left,right,up,down; // variables to calculate shape(left -> right) and height  (down -> up)
PointCloud::Ptr cloud_to_work,cloud_vision_field;
PointCloudRGB::Ptr cloud_car;
sensor_msgs::CameraInfo::_K_type intrinsic_matrix;
std::string frame_id;
bool flag_cloud, flag_image, flag_detections;
darknet_ros_msgs::BoundingBoxes detections;
int cam_width, cam_height;
float car_min_dist=0;
bool erase_this = false;
#endif
