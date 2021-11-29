#include "heronsafe_node.h"



void cb_odometry(const nav_msgs::Odometry::ConstPtr &msg)
{
  static tf::TransformBroadcaster br;

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform,ros::Time::now(), "odom","base_footprint"));

  if(pose_out_right.pose.position.x >= -2 && abs(pose_out_right.pose.position.y) <=1.5)
  {

    vel.linear.x = 0.2;


  } else if(pose_out_left.pose.position.x <= 2 && abs(pose_out_left.pose.position.y) <= 1.5)
  {
    vel.linear.x = -0.2;

  }

  pub.publish(vel);
   return;
}

void cb_nearest_left( const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  counter1++;
 listener->transformPose("/base_link", *msg, pose_out_left);
 if(counter1 > print_rate){
    ROS_INFO("NEAREST[L]: [%f, %f]", pose_out_left.pose.position.x,  pose_out_left.pose.position.y);
    //ROS_INFO("NEAR[R]: %s", near_R ? "yes" : "no");
    counter1 = 0;
  }
 return;
}
void cb_nearest_right( const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  counter2++;
 listener->transformPose("/base_link", *msg, pose_out_right);
 if(counter2 > print_rate){
    ROS_INFO("NEAREST[R]: [%f, %f]", pose_out_right.pose.position.x,  pose_out_right.pose.position.y);
    //ROS_INFO("NEAR[R]: %s", near_R ? "yes" : "no");
    counter2 = 0;
  }
 return;
}

int main(int argc, char **argv)
{

  //Init the ros system
  ros::init(argc,argv,"heronsafe_node");
  if(first)
  {
    vel.linear.x = 0.2;
    first = !first;
  }
  //Create the node handles to establish the program as a ROS node
  ros::NodeHandle n_public;
  ros::NodeHandle n_private("~"); //Private definition node namespace
  //Create a subscriber object
  ros::Subscriber sub=n_public.subscribe("/heron/odom",1,cb_odometry);
  ros::Subscriber sub_nearest_left = n_public.subscribe("/lidar_left/nearest", 1, cb_nearest_left);
  ros::Subscriber sub_nearest_right = n_public.subscribe("/lidar_right/nearest", 1, cb_nearest_right);

  listener = new tf::TransformListener;
  pub = n_public.advertise<geometry_msgs::Twist>("cmd_vel",1);

  try
  {
    listener->waitForTransform("/lidar_right_link", "base_link", ros::Time::now(), ros::Duration(3.0));
    listener->waitForTransform("/lidar_left_link", "base_link", ros::Time::now(), ros::Duration(3.0));

  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  ros::spin();
  delete listener;
}
