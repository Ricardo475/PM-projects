#include "object_visualization.h"



//TODO: 1.Decide when RED rectangle and GREEN rectangle
//            - Have to know the distances of all cars in between centroid of
//              selfdrive car and the other car in the base_link reference
//      2.Put the messages of distance and 'danger' in image when is RED
//            - X < 10m OR |Y| < 5m

void draw_rectangles(const darknet_ros_msgs::BoundingBoxes msg)
{
  if(glob_image.ptr() != nullptr){

    for(uint8_t i = 0; i< msg.bounding_boxes.size();i++){

      if(msg.bounding_boxes.at(i).Class == "car"){

        //GREEN
         cv::rectangle(glob_image,cv::Point(msg.bounding_boxes.at(i).xmax,msg.bounding_boxes.at(i).ymax),cv::Point(msg.bounding_boxes.at(i).xmin,msg.bounding_boxes.at(i).ymin),cv::Scalar(0,255,0),1,cv::LINE_8);

        //RED
        //cv::rectangle(glob_image,cv::Point(msg.bounding_boxes.at(i).xmax,msg.bounding_boxes.at(i).ymax),cv::Point(msg.bounding_boxes.at(i).xmin,msg.bounding_boxes.at(i).ymin),cv::Scalar(0,0,255),1,cv::LINE_8);
      }



    }

    cv::imshow("darknet image", glob_image );
    cv::waitKey(1000);
    cv::destroyAllWindows();
  }
}


void camera_callback(const sensor_msgs::CameraInfo& camera_inf)
{
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
  //ros::Subscriber sub_dists = n_public.subscribe("dist",1,dist_callback);
  ros::Subscriber sub_visual = n_public.subscribe("visual",1,visual_callback);

  ros::spin();

}
