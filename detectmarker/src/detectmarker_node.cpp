#include "detectmarker_node.h"


void cb_odometry(const nav_msgs::Odometry::ConstPtr &msg)
{
  static tf::TransformBroadcaster br;

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform,ros::Time::now(), "odom","base_footprint"));


  pub.publish(vel);
   return;
}

void cb_image_raw_left(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat image = cv_ptr -> image;

  // Convert input image to HSV
  cv::Mat hsv_image;
  cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv_image,cv::Scalar(40, 40,40),cv::Scalar(70, 255,255),mask);

  cv::Mat grey_image, tresh_image;
  cv::cvtColor(image, grey_image, cv::COLOR_BGR2GRAY);
  cv::threshold(grey_image,tresh_image,100,255,cv::THRESH_BINARY_INV);

  // Show the image inside it.
  cv::imshow( "normal image", image);
  cv::imshow( "mask", mask);
  cv::imshow( "grey_image", grey_image);
  cv::imshow( "thresh_image", tresh_image);

  // Wait for a keystroke.
  cv::waitKey(0);
}

int main(int argc, char **argv)
{
  //Init the ros system
  ros::init(argc,argv,"detectmarker_node");
  //Create the node handles to establish the program as a ROS node
  ros::NodeHandle n_public;
  ros::NodeHandle n_private("~"); //Private definition node namespace
  //Create a subscriber object
  ros::Subscriber sub=n_public.subscribe("/heron/odom",1,cb_odometry);
  pub = n_public.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  vel.angular.z = 1;

  ros::Subscriber sub_nearest_left = n_public.subscribe("/camera/left/image_raw", 1, cb_image_raw_left);

  ros::spin();
}
