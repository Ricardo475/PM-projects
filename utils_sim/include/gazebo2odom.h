#ifndef GAZEBO2ODOM_H
#define GAZEBO2ODOM_H

/// C++ includes
#include <algorithm>

/// Ros includes
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

///MSGs includes
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

///TF includes
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

/// Services includes
#include<gazebo_msgs/GetWorldProperties.h>
#include<std_srvs/Empty.h>


/**
 * @brief The GAZ2ODOParams struct defines the configuration parameters, if new parameters defined increment in the struct
 */
struct GAZ2ODOParams
{
  std::string world_frame;
  std::string robot_frame;

  std::string topic_pub_base;
  std::string topic_gazebo_properties;

  std::string element_descriptor;

  double time;
  double time_wait_data;

  bool global_verbose;
  bool debug;

  GAZ2ODOParams() {}
};


struct Element
{
    std::string name;

    std_msgs::Header header;

    geometry_msgs::Pose pose;
    geometry_msgs::Twist velocity;

    uint index;

    ros::Publisher pub_odom;

    Element(std::string name_, uint index_, ros::Publisher pub_odom_): name(name_), index(index_), pub_odom(pub_odom_) {}

    Element(std::string name_, std_msgs::Header header_, geometry_msgs::Pose pose_, geometry_msgs::Twist velocity_, uint index_, ros::Publisher pub_odom_): name(name_), header(header_), pose(pose_), velocity(velocity_), index(index_), pub_odom(pub_odom_) {}
};

///Global Variables

//Publishers
GAZ2ODOParams g_method_params;
//TF Broadcaster
tf::TransformBroadcaster* broadcaster;
//Flags
bool g_has_model = false;
//Global Vars
std::vector<Element> g_element_list;

/// Callbacks

/**
 * @brief elementsCallback - get models and publishes odom to base_footprint
 * @param msg_
 *
 */
void elementsCallback(const gazebo_msgs::ModelStates::ConstPtr msg_);

///Util Functions

/**
 * @brief getParameters - Defines the parameters required to configure the method execution
 * @param n_public_ - public nodehandle
 * @param n_private_ - private nodehandle
 * @return Method parameters configurarion
 */
GAZ2ODOParams getParameters(ros::NodeHandle n_public_, ros::NodeHandle n_private_);

/**
 * @brief publishData - data publisher template for any type of msg
 * @param pub_
 * @param data_
 */
template <class DataType>
void publishData(ros::Publisher& pub_, DataType data_)
{
  if(pub_.getNumSubscribers()>0)
    pub_.publish(data_);

  return;
}

/**
 * @brief tf_broad - Broadcast the pose to the tf tree
 * @param broadcaster_
 * @param data - Pose stamped to broadcast. VERY IMPORTANT TO FILL CORRECTLY THE HEADER!
 * @param parent_frame - Top level frame
 * @param child_frame - Bottom leve frame
 */
void tf_broad(tf::TransformBroadcaster* broadcaster_, geometry_msgs::PoseStamped data, std::string parent_frame, std::string child_frame)
{
  geometry_msgs::TransformStamped broad_data;
  broad_data.header.stamp = data.header.stamp;
  broad_data.header.frame_id = parent_frame;
  broad_data.child_frame_id = child_frame;
  broad_data.transform.translation.x = data.pose.position.x;
  broad_data.transform.translation.y = data.pose.position.y;
  broad_data.transform.translation.z = data.pose.position.z;
  broad_data.transform.rotation =  data.pose.orientation;

  try
  {
    broadcaster_->sendTransform(broad_data);
  }
  catch (tf::TransformException& e) {
    ROS_INFO ("Not saving scan due to tf lookup exception: %s",
              e.what());
  }
}

#endif
