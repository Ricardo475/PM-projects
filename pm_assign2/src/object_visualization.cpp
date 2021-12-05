#include "object_visualization.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_visualization");
  ros::NodeHandle n_public;
  ros::NodeHandle n_private("~"); //Private definition node namespace

  n_private.param<std::string>("frame_id", frame_id , "vision_frame");

  //ROS_INFO("Hello world!");


}
