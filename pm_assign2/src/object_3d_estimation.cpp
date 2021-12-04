#include "object_3d_estimation.h"


void draw_rectangles(const darknet_ros_msgs::BoundingBoxes msg)
{
  if(glob_image.ptr() != nullptr)
  {
    for(uint8_t i = 0; i< msg.bounding_boxes.size();i++)
    {
      cv::rectangle(glob_image,cv::Point(msg.bounding_boxes.at(i).xmax,msg.bounding_boxes.at(i).ymax),cv::Point(msg.bounding_boxes.at(i).xmin,msg.bounding_boxes.at(i).ymin),cv::Scalar(255,255,255),1,cv::LINE_8);

      double prob = msg.bounding_boxes.at(i).probability;

      //cv::putText(glob_image,"type: "+msg.bounding_boxes.at(i).Class+" prob: "+ std::to_string(prob),cv::Point(msg.bounding_boxes.at(i).ymin,msg.bounding_boxes.at(i).ymin),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,0),2);
    }

    cv::imshow("darknet iamge", glob_image );
    cv::waitKey();
    cv::destroyAllWindows();
  }
}
void pointCloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::PointCloud2 inputCloud;
  PointCloud::Ptr cloud_to_work(new PointCloud);
  inputCloud = *input;
  cloud_to_work->width = inputCloud.width;
  cloud_to_work->height = 1;
  cloud_to_work->resize(cloud_to_work->width*cloud_to_work->height);
  pcl::fromROSMsg(inputCloud,*cloud_to_work);

  sensor_msgs::PointCloud2 msg_pub,msg_trasnformed_pub;
  std_msgs::Header header;

  pcl::toROSMsg(*cloud_to_work,msg_pub);

  header.frame_id = "base_link";
  header.stamp    = ros::Time::now();
  msg_pub.header = header;
  pcl_ros::transformPointCloud("vision_frame",msg_pub,msg_trasnformed_pub,*listener);
  pub.publish(msg_trasnformed_pub);
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
  glob_image = image;
}

void image_darkNet_callback(const darknet_ros_msgs::BoundingBoxes& msg)
{
  //draw_rectangles(msg);
}
int main(int argc, char **argv)
{
  //Init the ros system
  ros::init(argc,argv,"pointcloud_node");
  ros::NodeHandle n_public;
  ros::NodeHandle n_private("~"); //Private definition node namespace
  glob_image = cv::Mat();
  //Create a subscriber object
  ros::Subscriber sub_left = n_public.subscribe("/stereo/left/image_rect_color",1,image_left_callback);
  ros::Subscriber sub_cloud = n_public.subscribe("velodyne_points",1,pointCloud_callback);
  ros::Subscriber sub_dark = n_public.subscribe("/objects/left/bounding_boxes",1,image_darkNet_callback);
  pub = n_public.advertise<PointCloud> ("/stereo/pointcloud", 1);
  listener = new tf::TransformListener;

  try
  {
    listener->waitForTransform("/vision_frame", "velodyne", ros::Time::now(), ros::Duration(3.0));
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  ros::spin();
}
