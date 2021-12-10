#include "object_visualization.h"



//TODO: 1.Decide when RED rectangle and GREEN rectangle
//            - Have to know the distances of all cars in between centroid of
//              selfdrive car and the other car in the base_link reference
//      2.Put the messages of distance and 'danger' in image when is RED
//            - X < 10m OR |Y| < 5m

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

float norm_dist(cv::Point3f& pixel)
{
  float coord[3],pixel3f[3];

  pixel3f[0] = pixel.x;
  pixel3f[1] = pixel.y;
  pixel3f[2] = pixel.z;

  pixelToPoint(pixel3f,coord);

  return cv::sqrt(coord[0] * coord[0] + coord[1]*coord[1] + coord[2]*coord[2]);
}


float check_dist_to_car(const darknet_ros_msgs::BoundingBox& carr, cv::Point3f* coords)
{
  float min_dist = 9999;
  float result;

  for(size_t i=0; i< depth_map.size(); i++)
  {

    ROS_INFO("XMIM: %.2ld | XMAX: %.2ld | YMIN: %.2ld | YMAX: %.2ld", carr.xmin,carr.xmax,carr.ymin,carr.ymax);
    ROS_INFO("PIXEL map: X =%.2f | Y=%.2f | Z=%.2f", depth_map.at(i).x,depth_map.at(i).y,depth_map.at(i).z);



    if(depth_map.at(i).x > carr.xmin && depth_map.at(i).x < carr.xmax && depth_map.at(i).y > carr.ymin && depth_map.at(i).y < carr.ymax){

      result = norm_dist(depth_map.at(i));

      //ROS_INFO("DIST: %.2f", result);
      if(min_dist > result)
      {
        min_dist = result;
        float pixel[3];
        float coordinates[3];
        pixel[0] = depth_map.at(i).x;
        pixel[1] = depth_map.at(i).y;
        pixel[2] = depth_map.at(i).z;


        pixelToPoint(pixel,coordinates);
        //ROS_INFO("MIN PIXEL: [%.2f , %.2f , %.2f] ",pixel[0],pixel[1],pixel[2]);
        //ROS_INFO("MIN COORDSS: [%.2f , %.2f , %.2f] ",coordinates[0],coordinates[1],coordinates[2]);

        coords->x = coordinates[0];
        coords->y = coordinates[1];
        coords->z = coordinates[2];
      }

    }

  }

  return min_dist;
}

void draw_rectangles(const darknet_ros_msgs::BoundingBoxes msg)
{
  if(glob_image.ptr() != nullptr){

    for(uint8_t i = 0; i< msg.bounding_boxes.size();i++){

      //darknet_ros_msgs::BoundingBox bb = msg.bounding_boxes.at(i);
      if(msg.bounding_boxes.at(i).Class == "car"){

        cv::Point3f coords = cv::Point3f(0,0,0);

        float dist = check_dist_to_car(msg.bounding_boxes.at(i),&coords);

        //ROS_INFO("COORDS POINT %d = [%.2f , %.2f]",i,coords.x,coords.y);

        if(coords.x >=10 || abs(coords.y) >=5){

          cv::rectangle(glob_image,cv::Point(msg.bounding_boxes.at(i).xmax,msg.bounding_boxes.at(i).ymax),cv::Point(msg.bounding_boxes.at(i).xmin,msg.bounding_boxes.at(i).ymin),cv::Scalar(0,255,0),2,cv::LINE_8);

        }

        else{
          cv::rectangle(glob_image,cv::Point(msg.bounding_boxes.at(i).xmax,msg.bounding_boxes.at(i).ymax),cv::Point(msg.bounding_boxes.at(i).xmin,msg.bounding_boxes.at(i).ymin),cv::Scalar(0,0,255),2,cv::LINE_8);

        }


        //GREEN


        //cv::putText(glob_image,"type: "+msg.bounding_boxes.at(i).Class+" prob: "+ std::to_string(prob),cv::Point(msg.bounding_boxes.at(i).ymin,msg.bounding_boxes.at(i).ymin),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,0),2);

        //cv::putText(glob_image,"(x,y,z)=(1,2,4)",cv::Point(msg.bounding_boxes.at(i).xmin,msg.bounding_boxes.at(i).ymin-5),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,0,255),1,cv::LINE_AA);

        //RED
        //cv::rectangle(glob_image,cv::Point(msg.bounding_boxes.at(i).xmax,msg.bounding_boxes.at(i).ymax),cv::Point(msg.bounding_boxes.at(i).xmin,msg.bounding_boxes.at(i).ymin),cv::Scalar(0,0,255),1,cv::LINE_8);
      }
      else{
        cv::rectangle(glob_image,cv::Point(msg.bounding_boxes.at(i).xmax,msg.bounding_boxes.at(i).ymax),cv::Point(msg.bounding_boxes.at(i).xmin,msg.bounding_boxes.at(i).ymin),cv::Scalar(0,255,0),2,cv::LINE_8);

      }



    }

    cv::imshow("darknet image", glob_image );
    cv::waitKey(2000);
    //cv::destroyAllWindows();
  }
}


void camera_callback(const sensor_msgs::CameraInfo& camera_inf)
{
  cam_width = static_cast<int>(camera_inf.width);
  cam_height = static_cast<int>(camera_inf.height);
  intrinsic_matrix = camera_inf.K;
  //std::cout<< "IT: "<<intrinsic_matrix.elems[0]<<" "<<intrinsic_matrix.elems[1]<<" "<<intrinsic_matrix.elems[2]<<"\n"<<intrinsic_matrix.elems[3]<<" "<<intrinsic_matrix.elems[4]<<" "<<intrinsic_matrix.elems[5]<<"\n"<<intrinsic_matrix.elems[6]<<" "<<intrinsic_matrix.elems[7]<<" "<<intrinsic_matrix.elems[8]<<std::endl;
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
  glob_image = cv_ptr -> image;


  /*
  glob_image = image;
  flag_image = true;
  if(flag_image && flag_cloud)
  {
      flag_image = false;
      flag_cloud = false;
      calc_map_depth();
  }*/
}

void calc_map_depth(){


  depth_map.clear();


  for(uint16_t i = 0; i<cloud_to_work->size(); i++)
  {
    float point[3];
    float threedpoint[3];

    threedpoint[0] = cloud_to_work->points[i].x;
    threedpoint[1] = cloud_to_work->points[i].y;
    threedpoint[2] = cloud_to_work->points[i].z;
    pointToPixel(threedpoint,point);

    ROS_INFO("POINT %f, %f, %f",point[0],point[1],point[2]);


    if(point[0]>=0 && point[0]<= cam_width)
    {

      if( point[1]>=0 && point[1]<= cam_height && point [2] >0 && point[2]<MAX_DEPTH)
      {

        depth_map.push_back(cv::Point3f(point[0],point[1],point[2]));
        //ROS_INFO("POINT: [%d,%d]  at DIST: %2.f  ", (int)point[0],(int)point[1],point[2]);
        //ROS_INFO("POINT %f, %f  ADDED ",point[0],point[1]);
      }
    }
  }


}

void pointCloud_callback(const sensor_msgs::PointCloud2ConstPtr& input){

  sensor_msgs::PointCloud2 inputCloud;
  inputCloud = *input;
  cloud_to_work.reset(new PointCloud);
  cloud_to_work->width = inputCloud.width;
  cloud_to_work->height = inputCloud.height;
  cloud_to_work->resize(cloud_to_work->width*cloud_to_work->height);
  sensor_msgs::PointCloud2 msg_trasnformed_pub;
  std_msgs::Header header;

  pcl_ros::transformPointCloud("base_link",inputCloud,msg_trasnformed_pub,*listener);

  pcl::fromROSMsg(msg_trasnformed_pub,*cloud_to_work);
  calc_map_depth();

}


void visual_callback(const darknet_ros_msgs::BoundingBoxes& car){

  draw_rectangles(car);

}

//void dist_callback(const std::vector<float>& msg){
  //dists = msg;
//}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "object_visualization");
  ros::NodeHandle n_public;
  ros::NodeHandle n_private("~"); //Private definition node namespace

  n_private.param<std::string>("frame_id", frame_id , "vision_frame");

  ros::Subscriber cam_inf = n_public.subscribe("/stereo/left/camera_info",1,camera_callback);
  ros::Subscriber sub_left = n_public.subscribe("/stereo/left/image_rect_color",1,image_left_callback);
  ros::Subscriber sub_cloud = n_public.subscribe("velodyne_points",1,pointCloud_callback);
  //ros::Subscriber sub_dists = n_public.subscribe("dist",1,dist_callback);
  ros::Subscriber sub_visual = n_public.subscribe("visual",1,visual_callback);

  listener = new tf::TransformListener;
    try
    {
      listener->waitForTransform("/base_link", "velodyne", ros::Time::now(), ros::Duration(3.0));
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }


  ros::spin();

}
