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


float check_dist_to_car(const darknet_ros_msgs::BoundingBox& carr, geometry_msgs::PointStamped* coords)
{
  float min_dist = 9999;
  float result;
  float coordinates[3];

  for(size_t i=0; i< depth_map.size(); i++)
  {

  //  ROS_INFO("XMIM: %.2ld | XMAX: %.2ld | YMIN: %.2ld | YMAX: %.2ld", carr.xmin,carr.xmax,carr.ymin,carr.ymax);
   // ROS_INFO("PIXEL %ld map: X =%.2f | Y=%.2f | Z=%.2f", i,depth_map.at(i).x,depth_map.at(i).y,depth_map.at(i).z);



    if(depth_map.at(i).x > carr.xmin && depth_map.at(i).x < carr.xmax && depth_map.at(i).y > carr.ymin && depth_map.at(i).y < carr.ymax){

      result = norm_dist(depth_map.at(i));

      //ROS_INFO("DIST: %.2f", result);
      if(min_dist > result)
      {

        float pixel[3];

        pixel[0] = depth_map.at(i).x;
        pixel[1] = depth_map.at(i).y;
        pixel[2] = depth_map.at(i).z;


        pixelToPoint(pixel,coordinates);
        //ROS_INFO("MIN PIXEL: [%.2f , %.2f , %.2f] ",pixel[0],pixel[1],pixel[2]);
        //ROS_INFO("MIN COORDSS: [%.2f , %.2f , %.2f] ",coordinates[0],coordinates[1],coordinates[2]);

        if(coordinates[0] > 0){
          coords->point.x = coordinates[0];
          coords->point.y = coordinates[1];
          coords->point.z = coordinates[2];
          min_dist = result;
        }
      }

    }

  }


  return min_dist;
}

void transform_to_PointCloud(const darknet_ros_msgs::BoundingBox& carr, float car_min_dist){


  float pixel[3];
  float point[3];

  float coordinates[3];

  for(size_t i=0; i< depth_map.size(); i++)
  {

    //pass_float_to_float(depth_map.at(i),coordinates);
    coordinates[0] = depth_map.at(i).x;
    coordinates[1] = depth_map.at(i).y;
    coordinates[2] = depth_map.at(i).z;

    pixelToPoint(coordinates,coordinates);
   // ROS_INFO("X: %.2f  Y: %.2f   Z:  %.2f", coordinates[0], coordinates[1],coordinates[2]);
    if(/*inside_boundary(depth_map.at(i), carr.xmin, carr.xmax, carr.xmax, carr.ymax) &&*/ norm_dist(depth_map.at(i)) < (car_min_dist + 3) && norm_dist(depth_map.at(i)) > (car_min_dist - 1) && coordinates[1] <1.1) //dar 3m de offset devido ao comprimento de um carro normal
    {


      pixel[0] = depth_map.at(i).x;
      pixel[1] = depth_map.at(i).y;
      pixel[2] = depth_map.at(i).z;
      //ROS_INFO("PIXEL: [%.2lf ; %.2lf, %.2lf]", float(pixel[0]) , float(pixel[1]) , float(pixel[2]));
      pixelToPoint(pixel,point);
      pcl::PointXYZ cloud_point;
      cloud_point.PointXYZ::x = point[0];
      cloud_point.PointXYZ::y = point[1];
      cloud_point.PointXYZ::z = point[2];
      cloud_car->push_back(cloud_point);
    }
  }



}

void draw_rectangles(const darknet_ros_msgs::BoundingBoxes msg)
{

  dist_car = 999999;
  increment = -1;

  if(glob_image.ptr() != nullptr){

    for(uint8_t i = 0; i< msg.bounding_boxes.size();i++){

      //darknet_ros_msgs::BoundingBox bb = msg.bounding_boxes.at(i);
      if(msg.bounding_boxes.at(i).Class == "car"){

        listener = new tf::TransformListener;
        tf::StampedTransform transform;

        geometry_msgs::PointStamped coords;
        coords.header.frame_id = frame_id;
        coords.header.stamp = ros::Time::now();
        coords.point.x = 999;
        coords.point.y = 999;
        coords.point.z = 999;

        float dist = check_dist_to_car(msg.bounding_boxes.at(i),&coords);


        cloud_car.reset(new PointCloud);
        cloud_car->width = cloud_map->width;
        cloud_car->height = 1;
        cloud_car->resize(cloud_car->width*cloud_car->height);



        transform_to_PointCloud(msg.bounding_boxes.at(i),dist);

        pcl::PointXYZ cl;

        pcl::computeCentroid(*cloud_car,cl);

        //coords.point.x = cl.x;
        //coords.point.y = cl.y;
        //coords.point.z = cl.z;

        //ROS_INFO("[%.2f %2.f %2.f]", cl.x, cl.y, cl.z);


       //ROS_INFO("COORDS POINT MIN IN VF= [%.2f , %.2f , %.2f]",coords.point.x,coords.point.y,coords.point.z);

        geometry_msgs::PointStamped coordsLink;
       // coordsLink.header.frame_id = "base_link";
       // coordsLink.header.stamp = ros::Time::now();
       // coordsLink.point.x = 999;
       // coordsLink.point.x = 999;
       // coordsLink.point.x = 999;
        try{
                listener->waitForTransform(frame_id, "base_link", ros::Time::now(), ros::Duration(3.0));
                listener->lookupTransform(frame_id, "base_link",ros::Time::now(),transform);
               // std::cout << "transform exist\n";

                listener->transformPoint("base_link",coords,coordsLink);
        }
        catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    coordsLink.point.x = 999;
                    coordsLink.point.y = 999;
                    coordsLink.point.z = 999;

             }

        //ROS_INFO("COORDS POINT MIN IN BL= [%.2f , %.2f , %.2f]",coordsLink.point.x,coordsLink.point.y,coordsLink.point.z);
        //ROS_INFO("COORDS CAR %d = [%.2f , %.2f , %.2f]",i,coords.x,coords.y,coords.z);

        //ROS_INFO("vision_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f)",
        //         coords.point.x,coords.point.y,coords.point.z,
        //        coordsLink.point.x,coordsLink.point.y,coordsLink.point.z);



        if(coordsLink.point.x >=10 || abs(coordsLink.point.y) >=5 || coordsLink.point.x < 0 ){


          cv::rectangle(glob_image,cv::Point(msg.bounding_boxes.at(i).xmax,msg.bounding_boxes.at(i).ymax),cv::Point(msg.bounding_boxes.at(i).xmin,msg.bounding_boxes.at(i).ymin),cv::Scalar(0,255,0),2,cv::LINE_8);

        }

        else{
          int valueX = coordsLink.point.x + 0.5;
          int valueY = coordsLink.point.y + 0.5;
          int valueZ = coordsLink.point.z + 0.5;



          std::string print = "(x,y,z)=(" + std::to_string(valueX) +","+ std::to_string(valueY) + "," + std::to_string(valueZ) + ")";
          cv::putText(glob_image,print,cv::Point(msg.bounding_boxes.at(i).xmin,msg.bounding_boxes.at(i).ymin-5),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,0,255),1.2,cv::LINE_AA);

          cv::rectangle(glob_image,cv::Point(msg.bounding_boxes.at(i).xmax,msg.bounding_boxes.at(i).ymax),cv::Point(msg.bounding_boxes.at(i).xmin,msg.bounding_boxes.at(i).ymin),cv::Scalar(0,0,255),2,cv::LINE_8);

          if( dist_car > dist ){

            increment = i;
            dist_car = dist;
            closest_car = msg.bounding_boxes.at(i);
            min_print = print;
          }

        }


        //GREEN


        //cv::putText(glob_image,"type: "+msg.bounding_boxes.at(i).Class+" prob: "+ std::to_string(prob),cv::Point(msg.bounding_boxes.at(i).ymin,msg.bounding_boxes.at(i).ymin),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,0),2);

        //RED
        //cv::rectangle(glob_image,cv::Point(msg.bounding_boxes.at(i).xmax,msg.bounding_boxes.at(i).ymax),cv::Point(msg.bounding_boxes.at(i).xmin,msg.bounding_boxes.at(i).ymin),cv::Scalar(0,0,255),1,cv::LINE_8);
      }
      else{
        cv::rectangle(glob_image,cv::Point(msg.bounding_boxes.at(i).xmax,msg.bounding_boxes.at(i).ymax),cv::Point(msg.bounding_boxes.at(i).xmin,msg.bounding_boxes.at(i).ymin),cv::Scalar(0,255,0),2,cv::LINE_8);

      }
    }

    if(increment != -1){

      int cw = car_width + 0.5;
      int ch = car_height + 0.5;

       std::string print = min_print +" Width=" + std::to_string(cw) +" Height=" + std::to_string(ch);
       cv::putText(glob_image,print,cv::Point(msg.bounding_boxes.at(increment).xmin,msg.bounding_boxes.at(increment).ymin-5),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,0,255),1.2,cv::LINE_AA);
       //ROS_INFO("HEYYY");
    }

    cv::imshow("Warning System", glob_image );
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


  for(uint16_t i = 0; i<cloud_map->size(); i++)
  {
    float point[3];
    float threedpoint[3];

    threedpoint[0] = cloud_map->points[i].x;
    threedpoint[1] = cloud_map->points[i].y;
    threedpoint[2] = cloud_map->points[i].z;
    pointToPixel(threedpoint,point);

    depth_map.push_back(cv::Point3f(point[0],point[1],point[2]));
    //ROS_INFO("POINT %f, %f, %f",cloud_map->points[i].x,cloud_map->points[i].y,cloud_map->points[i].z);
   // ROS_INFO("PIXEL %f, %f, %f ADDED",point[0],point[1],point[2]);


    //if(point[0]>=0 && point[0]<= cam_width)
   // {

      //if( point[1]>=0 && point[1]<= cam_height && point [2] >0 && point[2]<MAX_DEPTH)
     // {


        //ROS_INFO("POINT: [%d,%d]  at DIST: %2.f  ", (int)point[0],(int)point[1],point[2]);
        //ROS_INFO("POINT %f, %f  ADDED ",point[0],point[1]);
      //}
    //}
  }


}

void pointCloud_callback(const sensor_msgs::PointCloud2ConstPtr& input){


  sensor_msgs::PointCloud2 inputCloud;
  sensor_msgs::PointCloud2 msg_trasnformed;
  inputCloud = *input;

  cloud_map.reset(new PointCloud);
  cloud_map->width = inputCloud.width;
  cloud_map->height = inputCloud.height;
  cloud_map->resize(cloud_map->width*cloud_map->height);

  pcl::fromROSMsg(inputCloud,*cloud_map);

  calc_map_depth();



}


void visual_callback(const darknet_ros_msgs::BoundingBoxes& car){



  draw_rectangles(car);

}

void dist_callback(const geometry_msgs::PointStamped& vec){


  car_width = vec.point.x;
  car_height = vec.point.y;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "object_visualization");
  ros::NodeHandle n_public;
  ros::NodeHandle n_private("~"); //Private definition node namespace

  n_private.param<std::string>("frame_id", frame_id , "vision_frame");
  glob_image = cv::Mat();

  ros::Subscriber cam_inf = n_public.subscribe("/stereo/left/camera_info",1,camera_callback);
  ros::Subscriber sub_left = n_public.subscribe("/stereo/left/image_rect_color",1,image_left_callback);
  ros::Subscriber sub_cloud = n_public.subscribe("cloud_map",1,pointCloud_callback);
  ros::Subscriber sub_dists = n_public.subscribe("dist",1,dist_callback);
  ros::Subscriber sub_visual = n_public.subscribe("/objects/left/bounding_boxes",1,visual_callback);

   //pub = n_public.advertise<PointCloud> ("/stereo/v_car_cloud", 1);

   ros::spin();

}
