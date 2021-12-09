#include "object_3d_estimation.h"

void pointToPixel(const float point[3],float pixel[3]){
  float x =  (point[0]/(point[2] + 1E-5)),y = (point[1]/(point[2]+1E-5));
  pixel[0] = static_cast<int>((x * intrinsic_matrix.elems[0] + intrinsic_matrix.elems[2]));
  pixel[1] = static_cast<int>((y * intrinsic_matrix.elems[4] + intrinsic_matrix.elems[5]));
  pixel[2] = point[2];
}
void pixelToPoint(const float pixel[3],float point[3])
{
  point[0] = (pixel[0] - intrinsic_matrix.elems[2]) * pixel[2]/(intrinsic_matrix.elems[0]+1E-5);
  point[1] = (pixel[1] - intrinsic_matrix.elems[5]) * pixel[2]/(intrinsic_matrix.elems[4]+1E-5);
  point[2] = pixel[2];
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

    cv::imshow("darknet image", glob_image );
    cv::waitKey();
    cv::destroyAllWindows();
  }
}
bool exist_pixel_depth(const std::vector<cv::Point3f> &depth,const int &irow, const int &icol)
{
  for(size_t i=0;i<depth.size();i++)
  {
    if(depth.at(i).x == icol && depth.at(i).y == irow)
      return true;
  }
  return false;
}
void image_create_from_depth_map(cv::Mat &src, cv::Mat &dst, const std::vector<cv::Point3f> &depth)
{
  dst = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
  for(int irow = 0; irow < src.rows; irow++)
  {
    uchar * pixel     = src.ptr<uchar>(irow);
    uchar * pixel_mask = dst.ptr<uchar>(irow);
    for(int icol = 0; icol < src.cols; icol++)
    {
      if(exist_pixel_depth(depth,irow,icol))
        pixel_mask[icol] = 255;
    }
  }
  return;
}
bool inside_boundary(const cv::Point3f& point,const float& t_x_min,const float& t_x_max,const float& t_y_min,const float& t_y_max)
{
  return point.x > t_x_min && point.x < t_x_max && point.y > t_y_min && point.y < t_y_max;
}
float norm_dist(cv::Point3f& pixel)
{
  float coord[3],pixel3f[3];

  pixel3f[0] = pixel.x;
  pixel3f[1] = pixel.y;
  pixel3f[2] = pixel.z;

  pixelToPoint(pixel3f,coord);

  if(erase_this)
  {
    //ROS_INFO("MINIMUM CAR AT  [ %.2f, %.2f, %.2f ]   with dist:  %.2f",coord[0], coord[1], coord[2], cv::sqrt(coord[0] * coord[0] + coord[1]*coord[1] + coord[2]*coord[2]));
  }
  return cv::sqrt(coord[0] * coord[0] + coord[1]*coord[1] + coord[2]*coord[2]);
}
float check_dist_to_car(const darknet_ros_msgs::BoundingBox& carr)
{
  float min_dist = 9999;
  float result;
  float thresh_x_min = (carr.xmax - carr.xmin)/6 + carr.xmin,
        thresh_x_max = (carr.xmax - carr.xmin)*5/6 + carr.xmin,
        thresh_y_min = (carr.ymax - carr.ymin)/6 + carr.ymin,
        thresh_y_max = (carr.ymax - carr.ymin)*5/6 + carr.ymin;
  //cv::Point3f aux_depth_map;
  //ROS_INFO(" ");
  //ROS_INFO(" ");
  //ROS_INFO(" ");
  for(size_t i=0; i< depth_map.size(); i++)
  {

    if(inside_boundary(depth_map.at(i), thresh_x_min, thresh_x_max, thresh_y_min, thresh_y_max) )
    {
      result = norm_dist(depth_map.at(i));
      if(min_dist > result)
      {
        min_dist = result;
       // aux_depth_map = depth_map.at(i);
      }

    }
  }

//  ROS_INFO("COORD_CLOSEST: [%.2f ; %.2f, %.2f]", float(aux_depth_map.x) , float(aux_depth_map.y) , float(aux_depth_map.z));

  return min_dist;
}

void make_car_point_cloud(darknet_ros_msgs::BoundingBox& carr)
{
  float thresh_x_min = (carr.xmax - carr.xmin)/6 + carr.xmin,
        thresh_x_max = (carr.xmax - carr.xmin)*5/6 + carr.xmin,
        thresh_y_min = (carr.ymax - carr.ymin)/6 + carr.ymin,
        thresh_y_max = (carr.ymax - carr.ymin)*5/6 + carr.ymin;
  float pixel[3];
  float point[3];
  cloud_car.reset(new PointCloudRGB);
  cloud_car->width = cloud_to_work->width;
  cloud_car->height = 1;
  cloud_car->resize(cloud_car->width*cloud_car->height);

  for(size_t i=0; i< depth_map.size(); i++)
  {

    if(inside_boundary(depth_map.at(i), thresh_x_min, thresh_x_max, thresh_y_min, thresh_y_max) )
    {
      pixel[0] = depth_map.at(i).x;
      pixel[1] = depth_map.at(i).y;
      pixel[2] = depth_map.at(i).z;
      pixelToPoint(pixel,point);
      pcl::PointXYZRGB cloud_point;
      cloud_point._PointXYZRGB::x = point[0];
      cloud_point.PointXYZRGB::y = point[1];
      cloud_point.PointXYZRGB::z = point[2];
      cloud_point.PointXYZRGB::b = glob_image.data[glob_image.channels() * (glob_image.cols * static_cast<int>(depth_map.at(i).y) + static_cast<int>(depth_map.at(i).x) +0)];
      cloud_point.PointXYZRGB::g = glob_image.data[glob_image.channels() * (glob_image.cols * static_cast<int>(depth_map.at(i).y) + static_cast<int>(depth_map.at(i).x) +1)];
      cloud_point.PointXYZRGB::r = glob_image.data[glob_image.channels() * (glob_image.cols * static_cast<int>(depth_map.at(i).y) + static_cast<int>(depth_map.at(i).x) +2)];

      cloud_car->push_back(cloud_point);
    }
  }


  sensor_msgs::PointCloud2 msg_trasnformed_pub;
  std_msgs::Header header;

  pcl::toROSMsg(*cloud_car,msg_trasnformed_pub);

  header.frame_id = frame_id;
  header.stamp    = ros::Time::now();
  msg_trasnformed_pub.header = header;
  pub_car.publish(msg_trasnformed_pub);
}
void calc_closest_car(){

  erase_this = false;
  darknet_ros_msgs::BoundingBox closest_car;
  float min_dist = 9999;
  bool closest_car_find= false;

  for(uint8_t i = 0; i< detections.bounding_boxes.size();i++)
      {
        if(detections.bounding_boxes.at(i).Class == "car")
        {
          darknet_ros_msgs::BoundingBox carr = detections.bounding_boxes.at(i);
          float dist = check_dist_to_car(carr);   
          //ROS_INFO("SIZE: [%.2f ; %.2f]", float(carr.xmax) , float(carr.ymax));

          if(min_dist > dist && (carr.xmax - carr.xmin) > 50 && (carr.ymax- carr.ymin) > 50)
          {
            closest_car = carr;
            min_dist = dist;
            closest_car_find = true;
          }
        }
      }

  //ROS_INFO("DIST: %.2f ",min_dist);

  if(closest_car_find)
  {
    erase_this = true;
    make_car_point_cloud(closest_car);
    cv::Mat imageROI(glob_image,cv::Rect(closest_car.xmin,closest_car.ymin,(closest_car.xmax- closest_car.xmin),(closest_car.ymax - closest_car.ymin)));

    // Create a window.
    cv::namedWindow( "closest car", cv::WINDOW_NORMAL );
    cv::imshow("closest car", imageROI );
    cv::waitKey(1000);
    cv::destroyAllWindows();
  }

 /* cv::rectangle(glob_image,cv::Point(closest_car.xmax,closest_car.ymax),cv::Point(closest_car.xmin,closest_car.ymin),cv::Scalar(255,255,255),1,cv::LINE_8);
  cv::imshow("darknet iamge", glob_image );
  cv::waitKey();
  cv::destroyAllWindows();*/
}
void calc_map_depth(){

  cloud_vision_field.reset(new PointCloud);
  cloud_vision_field->width = cloud_to_work->width;
  cloud_vision_field->height = 1;
  cloud_vision_field->resize(cloud_vision_field->width*cloud_vision_field->height);
  depth_map.clear();

  for(uint16_t i = 0; i<cloud_to_work->size(); i++)
  {
    float point[3];
    float threedpoint[3];


    threedpoint[0] = cloud_to_work->points[i].x;
    threedpoint[1] = cloud_to_work->points[i].y;
    threedpoint[2] = cloud_to_work->points[i].z;
    pointToPixel(threedpoint,point);

    if(point[0]>=0 && point[0]<= cam_width)
    {
      if( point[1]>=0 && point[1]<= cam_height && point [2] >0 && point[2]<MAX_DEPTH)
      {

        depth_map.push_back(cv::Point3f(point[0],point[1],point[2]));

        //ROS_INFO("POINT: [%d,%d]  at DIST: %2.f  ", (int)point[0],(int)point[1],point[2]);
        cloud_vision_field->push_back(cloud_to_work->points[i]);
       // ROS_INFO("POINT %d, %d  ADDED ",point[0],point[1]);
      }
    }
  }

  //
  //IDEIA - ADICIONAR AQUI AS CORES AO cloud_vision_field a partir da
  //        glob_image
  //

  pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;

  for(uint16_t i = 0; i<depth_map.size(); i++)
  {


  }




/*
  cv::Mat image_depth;
  image_create_from_depth_map(glob_image,image_depth,depth_map);

  cv::imshow("depth iamge", image_depth);
  cv::waitKey();
  cv::destroyAllWindows();*/

  sensor_msgs::PointCloud2 msg_trasnformed_pub;
  std_msgs::Header header;

  //ROS_ERROR("error in to");
  pcl::toROSMsg(*cloud_vision_field,msg_trasnformed_pub);
  //ROS_ERROR("error in to");

  header.frame_id = frame_id;
  header.stamp    = ros::Time::now();
  msg_trasnformed_pub.header = header;
  pub.publish(msg_trasnformed_pub);
}


void pointCloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{

  //RGB STILL NOT WORKING FFS
  /*

  sensor_msgs::PointCloud2 inputCloud;
  sensor_msgs::PointCloud2 msg_trasnformed_pub;
  inputCloud = *input;
  pcl_ros::transformPointCloud(frame_id,inputCloud,msg_trasnformed_pub,*listener);

  pcl::PCLPointCloud2 pc2_input;
  pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud;
  pcl_conversions::toPCL(msg_trasnformed_pub, pc2_input);
  pcl::fromPCLPointCloud2(pc2_input, temp_cloud);

  cloud_to_work.reset(new PointCloud);
  cloud_to_work->width = temp_cloud.width;
  cloud_to_work->height = temp_cloud.height;
  cloud_to_work->is_dense = false;
  cloud_to_work->points.resize(temp_cloud.points.size());
  //cloud_to_work->resize(cloud_to_work->width*cloud_to_work->height);

  for (size_t i = 0; i < temp_cloud.points.size(); i++)
    {
      cloud_to_work->points[i].x = temp_cloud.points[i].x;
      cloud_to_work->points[i].y = temp_cloud.points[i].y;
      cloud_to_work->points[i].z = temp_cloud.points[i].z;
      cloud_to_work->points[i].r =  1024 * rand () / (RAND_MAX + 1);;
      cloud_to_work->points[i].g =  1024 * rand () / (RAND_MAX + 1);;
      cloud_to_work->points[i].b =  1024 * rand () / (RAND_MAX + 1);;
    }


  */

  sensor_msgs::PointCloud2 inputCloud;
    inputCloud = *input;
    cloud_to_work.reset(new PointCloud);
    cloud_to_work->width = inputCloud.width;
    cloud_to_work->height = 1;
    cloud_to_work->resize(cloud_to_work->width*cloud_to_work->height);
    sensor_msgs::PointCloud2 msg_trasnformed_pub;
    std_msgs::Header header;

    pcl_ros::transformPointCloud(frame_id,inputCloud,msg_trasnformed_pub,*listener);

    pcl::fromROSMsg(msg_trasnformed_pub,*cloud_to_work);

  flag_cloud = true;
  if(flag_image && flag_cloud && flag_detections)
  {
      flag_image = false;
      flag_cloud = false;
      flag_detections = false;
      calc_map_depth();
      calc_closest_car();
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
  if(flag_image && flag_cloud && flag_detections)
  {
      flag_image = false;
      flag_cloud = false;
      flag_detections = false;
      calc_map_depth();
      calc_closest_car();
  }
}
void camera_callback(const sensor_msgs::CameraInfo& camera_inf)
{
  intrinsic_matrix = camera_inf.K;
  cam_width = static_cast<int>(camera_inf.width);
  cam_height = static_cast<int>(camera_inf.height);
  //std::cout<< "IT: "<<intrinsic_matrix.elems[0]<<" "<<intrinsic_matrix.elems[1]<<" "<<intrinsic_matrix.elems[2]<<"\n"<<intrinsic_matrix.elems[3]<<" "<<intrinsic_matrix.elems[4]<<" "<<intrinsic_matrix.elems[5]<<"\n"<<intrinsic_matrix.elems[6]<<" "<<intrinsic_matrix.elems[7]<<" "<<intrinsic_matrix.elems[8]<<std::endl;

}
void image_darkNet_callback(const darknet_ros_msgs::BoundingBoxes& msg)
{
  detections = msg;
  flag_detections = true;

  //draw_rectangles(msg);
  if(flag_image && flag_cloud && flag_detections)
  {
      flag_image = false;
      flag_cloud = false;
      flag_detections = false;
      calc_map_depth();
      calc_closest_car();

  }
  pub_visualization.publish(detections); //preciso de todos os carros no "object_visualization" :)
}
int main(int argc, char **argv)
{
  //Init the ros system
  ros::init(argc,argv,"pointcloud_node");
  ros::NodeHandle n_public;
  ros::NodeHandle n_private("~"); //Private definition node namespace

  n_private.param<std::string>("frame_id", frame_id , "vision_frame");
  glob_image = cv::Mat();
  //Create a subscriber object
  ros::Subscriber cam_inf = n_public.subscribe("/stereo/left/camera_info",1,camera_callback);
  ros::Subscriber sub_left = n_public.subscribe("/stereo/left/image_rect_color",1,image_left_callback);
  ros::Subscriber sub_cloud = n_public.subscribe("velodyne_points",1,pointCloud_callback);
  ros::Subscriber sub_dark = n_public.subscribe("/objects/left/bounding_boxes",1,image_darkNet_callback);
  pub = n_public.advertise<PointCloud> ("/stereo/pointcloud", 1);
  pub_car = n_public.advertise<PointCloudRGB> ("/stereo/car_pointcloud", 1);
  pub_visualization = n_public.advertise<darknet_ros_msgs::BoundingBoxes> ("visual", 1);


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
