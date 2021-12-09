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

    if(inside_boundary(depth_map.at(i), thresh_x_min, thresh_x_max, thresh_y_min, thresh_y_max) && norm_dist(depth_map.at(i)) < (car_min_dist + 3)) //dar 3m de offset devido ao comprimento de um carro normal
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

/*  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  cloud.reset(new PointCloud);
  cloud->width = cloud_car->width;
  cloud->height = 1;
  cloud->resize(cloud_car->width*cloud_car->height);

  pcl::copyPointCloud(*cloud_car,*cloud);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

  // Create search tree*
 pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
 pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
 pcl::PolygonMesh triangles;

 // Set the maximum distance between connected points (maximum edge length)
 gp3.setSearchRadius (0.025);            //It was 0.025
 // Set typical values for the parameters
 gp3.setMu (2.5);                                            //It was 2.5
 gp3.setMaximumNearestNeighbors (100);    //It was 100
 gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees    //it was 4
 gp3.setMinimumAngle(M_PI/18); // 10 degrees        //It was 18
 gp3.setMaximumAngle(M_PI/1.5); // 120 degrees        //it was 1.5
 gp3.setNormalConsistency(false);                    //It was false

 // Get result
 gp3.setInputCloud (cloud_with_normals);
 gp3.setSearchMethod (tree2);
 gp3.reconstruct (triangles);

 //publish mesh
 pcl_msgs::PolygonMesh msg_poly;
 shape_msgs::Mesh::Ptr ros_mesh_ptr;
 pcl_conversions::fromPCL(triangles,msg_poly);

 sensor_msgs::PointCloud2Modifier pcd_modifier(msg_poly.cloud);

 size_t size = pcd_modifier.size();
 ros_mesh_ptr.reset(new shape_msgs::Mesh);
 ros_mesh_ptr->vertices.resize(size);

 ROS_INFO_STREAM("polys: " << msg_poly.polygons.size()
                           << " vertices: " << pcd_modifier.size());

 sensor_msgs::PointCloud2ConstIterator<float> pt_iter(msg_poly.cloud, "x");

 for (size_t i = 0u; i < size; i++, ++pt_iter) {
   ros_mesh_ptr->vertices[i].x = pt_iter[0];
   ros_mesh_ptr->vertices[i].y = pt_iter[1];
   ros_mesh_ptr->vertices[i].z = pt_iter[2];
 }

 ROS_INFO_STREAM("Updated vertices");

 ros_mesh_ptr->triangles.resize(triangles.polygons.size());

 for (size_t i = 0u; i < triangles.polygons.size(); ++i) {
   if (triangles.polygons[i].vertices.size() < 3u) {
     ROS_WARN_STREAM("Not enough points in polygon. Ignoring it.");
     continue;
   }

   for (size_t j = 0u; j < 3u; ++j) {
     ros_mesh_ptr->triangles[i].vertex_indices[j] =
         triangles.polygons[i].vertices[j];
   }
 }
 ROS_INFO("Conversion from PCL PolygonMesh to ROS Mesh ended.");*/
 std_msgs::Header header;

 header.frame_id = frame_id;
 header.stamp    = ros::Time::now();
// msg_poly.header = header;

// pub_car_mesh.publish(msg_poly);

 sensor_msgs::PointCloud2 msg_trasnformed_pub;

// pub_car_mesh.publish(ros_mesh_ptr);


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
            car_min_dist = min_dist;
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
    cv::Mat gray_image;
    cv::cvtColor(imageROI,gray_image,cv::COLOR_BGR2GRAY);
    /*cv::GaussianBlur(gray_image,gray_image,cv::Size(7,7),0);

    cv::Mat edges;
    cv::Canny(gray_image,edges,50,100);
    cv::dilate(edges,edges,NULL,cv::Point(-1,-1),1);
    cv::erode(edges,edges,NULL);*/
    // Create a window.
    cv::namedWindow( "closest car", cv::WINDOW_NORMAL );
    cv::imshow("closest car", gray_image );
    cv::waitKey();
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

/*
  cv::Mat image_depth;
  image_create_from_depth_map(glob_image,image_depth,depth_map);

  cv::imshow("depth iamge", image_depth);
  cv::waitKey();
  cv::destroyAllWindows();*/

  sensor_msgs::PointCloud2 msg_trasnformed_pub;
  std_msgs::Header header;

  pcl::toROSMsg(*cloud_vision_field,msg_trasnformed_pub);

  header.frame_id = frame_id;
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
  pub_pose = n_public.advertise<geometry_msgs::Pose>("/car_pose",1);
  pub_car_mesh = n_public.advertise<shape_msgs::Mesh>("/stereo/car_mesh",1);

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
