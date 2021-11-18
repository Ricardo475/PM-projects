
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

  cv::Mat mask_green,mask_blue,mask_red;
  cv::inRange(hsv_image,cv::Scalar(40, 40,40),cv::Scalar(70, 255,255),mask_green); //green
  cv::inRange(hsv_image,cv::Scalar(120, 40,40),cv::Scalar(130, 255,255),mask_blue); //blue
  cv::inRange(hsv_image,cv::Scalar(0, 40,40),cv::Scalar(10, 255,255),mask_red); //red




  std::vector<std::vector<cv::Point>> countors_green,countors_red,countors_blue,countors;
  std::vector<cv::Vec4i> hierarchy;

  cv::Mat object_outliers_green,object_outliers_red,object_outliers_blue,object_outliers;
  cv::Canny(mask_red,object_outliers_red,100,200,3,true);
  cv::Canny(mask_green,object_outliers_green,100,200,3,true);
  cv::Canny(mask_blue,object_outliers_blue,1500,3000,3,true);//100 e 200

  cv::findContours(object_outliers_green,countors_green,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
  cv::findContours(object_outliers_red,countors_red,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
  cv::findContours(object_outliers_blue,countors_blue,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

  if(countors_green.size()>0)
  {
    countors = countors_green;
    color_green = true;
    object_outliers = object_outliers_green;
    ROS_INFO("GREEN COLOR");
  } else if(countors_red.size()>0)
  {
    countors = countors_red;
    color_red = true;
    object_outliers = object_outliers_red;
    ROS_INFO("RED COLOR");
  } else if(countors_blue.size()>0)
  {
    countors = countors_blue;
    color_blue = true;
    object_outliers = object_outliers_blue;
    ROS_INFO("BLUE COLOR");
  }

  std::vector<cv::Point> result;

  for(size_t i = 0 ;i<countors.size();i++)
  {
    double epsilon  = 0.02* cv::arcLength(countors.at(i),true);
    cv::approxPolyDP(countors.at(i),result,epsilon,true);
    ROS_INFO("num of vertices: %d", result.size());
    for(size_t j =0;j<result.size();j++)
    {
      ROS_INFO("Point %d:  (%d, %d)", j,result.at(j).x,result.at(j).y);
    }

  }


  cv::Moments m = cv::moments(mask_red,false);
  cv::Point p(m.m10/m.m00, m.m01/m.m00);
  //cv::circle(object_outliers, p, 5, cv::Scalar(128,0,0), -1);
  //cv::rectangle(object_outliers,cv::Point(p.x-30,p.y-30),cv::Point(p.x+30,p.y+30),cv::Scalar(128,0,0),1,cv::LINE_8,0);


  // Show the image inside it.
  cv::imshow( "normal image", hsv_image);
  cv::imshow( "mask", object_outliers);
  cv::imshow( "mask_blue", mask_red);


  // Wait for a keystroke.
  cv::waitKey(0);
  cv::destroyAllWindows();
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
  listener = new tf::TransformListener;
  pub = n_public.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  vel.angular.z = 1;

  ros::Subscriber sub_nearest_left = n_public.subscribe("/camera/left/image_raw", 1, cb_image_raw_left);

  try
  {
    //listener->waitForTransform()

  }
  catch (tf::TransformException ex)
  {

  }

  ros::spin();
}
