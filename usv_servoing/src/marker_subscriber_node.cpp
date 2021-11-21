#include "marker_subscriber_node.h"


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_subscriber_node");
  //Create the node handles to establish the program as a ROS node
  ros::NodeHandle n_public;
  ros::NodeHandle n_private("~"); //Private definition node namespace

  ros::Subscriber sub = n_public.subscribe("marker", 1, chatterCallback);

  ros::spin();

  return 0;
}
