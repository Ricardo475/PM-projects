#include "object_3d_estimation.h"


void pointCloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::PointCloud2 inputCloud;
  PointCloud::Ptr cloud_to_work(new PointCloud);
  inputCloud = *input;
  cloud_to_work->width = inputCloud.width;
  cloud_to_work->height = 1;
  cloud_to_work->resize(cloud_to_work->width*cloud_to_work->height);
  pcl::fromROSMsg(inputCloud,*cloud_to_work);

  sensor_msgs::PointCloud2 msg_pub;
  std_msgs::Header header;

  pcl::toROSMsg(*cloud_to_work,msg_pub);

  header.frame_id = "velodyne";
  header.stamp    = ros::Time::now();
  msg_pub.header = header;
  pub.publish(msg_pub);
}
void image_left_callback(const sensor_msgs::ImageConstPtr& msg)
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
}


int main(int argc, char **argv)
{
  //Init the ros system
  ros::init(argc,argv,"pointcloud_node");
  ros::NodeHandle n_public;
  ros::NodeHandle n_private("~"); //Private definition node namespace

  //Create a subscriber object
  ros::Subscriber sub_left = n_public.subscribe("/stereo/left/image_rect_color",1,image_left_callback);
  ros::Subscriber sub_cloud = n_public.subscribe("velodyne_points",1,pointCloud_callback);
  pub = n_public.advertise<PointCloud> ("/stereo/pointcloud", 1);
  ros::spin();
}
