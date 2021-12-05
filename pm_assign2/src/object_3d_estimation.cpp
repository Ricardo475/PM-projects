#include "object_3d_estimation.h"

void pointToPixel(const float point[3],int pixel[2]){
  float x = (float) (point[0]/(point[2] + 1E-5)),y = (float)(point[1]/(point[2]+1E-5));
  pixel[0] =(int) (x * intrinsic_matrix.elems[0] + intrinsic_matrix.elems[2]);
  pixel[1] = (int)(y * intrinsic_matrix.elems[4] + intrinsic_matrix.elems[5]);
}
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

void calc_map_depth(){

  cloud_vision_field.reset(new PointCloud);
  cloud_vision_field->width = cloud_to_work->width;
  cloud_vision_field->height = 1;
  cloud_vision_field->resize(cloud_vision_field->width*cloud_vision_field->height);

  for(uint16_t i = 0; i<cloud_to_work->size(); i++)
  {
    int point[2];
    float threedpoint[3];


    threedpoint[0] = cloud_to_work->points[i].x;
    threedpoint[1] = cloud_to_work->points[i].y;
    threedpoint[2] = cloud_to_work->points[i].z;

    pointToPixel(threedpoint,point);
    //ROS_INFO("POINT %d, %d   ", point[0],point[1]);

    if(point[0]>0 && point[0]< glob_image.size().width)
    {
      if( point[1]>0 && point[1]< glob_image.size().height)
      {
        depth_map.push_back(cv::Point(point[0],point[1]));
        cloud_vision_field->push_back(cloud_to_work->points[i]);
        ROS_INFO("POINT %d, %d  ADDED ",point[0],point[1]);
      }
    }
  }
  sensor_msgs::PointCloud2 msg_trasnformed_pub;
  std_msgs::Header header;

  pcl::toROSMsg(*cloud_vision_field,msg_trasnformed_pub);

  header.frame_id = "vision_frame";
  header.stamp    = ros::Time::now();
  msg_trasnformed_pub.header = header;
  pub.publish(msg_trasnformed_pub);
}
void pointCloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::PointCloud2 inputCloud;
  inputCloud = *input;
  cloud_to_work.reset(new PointCloud);
  cloud_to_work->width = inputCloud.width;
  cloud_to_work->height = 1;
  cloud_to_work->resize(cloud_to_work->width*cloud_to_work->height);
  sensor_msgs::PointCloud2 msg_trasnformed_pub;
  std_msgs::Header header;

  pcl_ros::transformPointCloud("vision_frame",inputCloud,msg_trasnformed_pub,*listener);
  header.frame_id = "vision_frame";
  header.stamp    = ros::Time::now();
  msg_trasnformed_pub.header = header;
  pcl::fromROSMsg(msg_trasnformed_pub,*cloud_to_work);

  flag_cloud = true;
  if(flag_image && flag_cloud)
  {
      flag_image = false;
      flag_cloud = false;
      calc_map_depth();
  }
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
  flag_image = true;
  if(flag_image && flag_cloud)
  {
      flag_image = false;
      flag_cloud = false;
      calc_map_depth();
  }
}
void camera_callback(const sensor_msgs::CameraInfo& camera_inf)
{
  intrinsic_matrix = camera_inf.K;
  //std::cout<< "IT: "<<intrinsic_matrix.elems[0]<<" "<<intrinsic_matrix.elems[1]<<" "<<intrinsic_matrix.elems[2]<<"\n"<<intrinsic_matrix.elems[3]<<" "<<intrinsic_matrix.elems[4]<<" "<<intrinsic_matrix.elems[5]<<"\n"<<intrinsic_matrix.elems[6]<<" "<<intrinsic_matrix.elems[7]<<" "<<intrinsic_matrix.elems[8]<<std::endl;
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
  ros::Subscriber cam_inf = n_public.subscribe("/stereo/left/camera_info",1,camera_callback);
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
