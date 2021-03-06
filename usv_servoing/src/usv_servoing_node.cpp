
#include "usv_servoing_node.h"


//Stop all heron's movement
void heron_stop(){
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.angular.z = 0;
  return;
}

//Starts going foward (velocity = 1)
void heron_foward(){

  vel.linear.x = 1;
  return;
}

//Starts going backwards (velocity = -1)
void heron_backwards(){
  vel.linear.x = -1;
  return;
}

//Slows down the velocity (velocity = 0.5 OR -0.5)
void heron_slowdown(){
  if(vel.linear.x>0)
    vel.linear.x = 0.5;
  else if (vel.linear.x<0)
    vel.linear.x = -0.5;
  return;
}

//Rotates to the side definided (Left: side = "L", Right: side="R")
void heron_rotate(char side){

  if(side=='L')
    vel.angular.z = 1;

  else if(side=='R')
    vel.angular.z = -1;

  return;
}


void cb_odometry(const nav_msgs::Odometry::ConstPtr &msg)
{
  static tf::TransformBroadcaster br;

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform,ros::Time::now(), "odom","base_footprint"));


  if (counter_odo>50){

    // ROS_INFO("ODOMETRY [X, Y, theta]: [%f, %f, %f]   state = %d", msg->pose.pose.position.x, msg->pose.pose.position.y,msg->pose.pose.orientation.w,state);
     counter_odo = 0;
   }
  counter_odo++;


  //transições
  switch (state) {
     case 0:
        if(abs(msg->pose.pose.position.y)<1.25 )
          next_state= 1;
        break;
     case 1:
        if(dock_left.identified)
          next_state=2;
        break;
     case 2:
        if(dock_left.tri)
           next_state = 3;
        else if(dock_left.cross || dock_left.circle)
           next_state = 6;
        break;
     case 3:
         //if(msg->pose.pose.orientation.w < 0.000005)
         if(usv_oriented)
           next_state = 4;
        break;
     case 4:
         if(pose_out_left.pose.position.x <= 2.2 && (msg->pose.pose.position.x <= -12.2 || msg->pose.pose.position.x >= 12.2) && need_sensors)
           next_state = 5;
         break;
     case 5:
        //END
        break;
     case 6:
        if(dock_right.identified)
          next_state = 7;
        break;
     case 7:
        if(dock_right.tri)
          next_state = 8;
        else if(dock_right.circle || (dock_right.cross && dock_left.circle))
          next_state = 9;
        else if(dock_right.cross)
          next_state = 12;
        break;
     case 8:
        //if(msg->pose.pose.orientation.w > 0.999995)
        if(usv_oriented)
          next_state = 4;
        break;
     case 9:
        if(msg->pose.pose.orientation.w < 0.707376 && quart ==3)
         next_state = 10;
        break;
     case 10:
        if(msg->pose.pose.position.y < (init_pos_y+1))
         next_state = 11;
        break;
     case 11:
        if(abs(msg->pose.pose.orientation.w) > 0.707376 && quart ==4)
          next_state = 5;
        break;
     case 12:
        if(abs(msg->pose.pose.orientation.w) < 0.707376)
          next_state = 13;
         break;
     case 13:
        //END FOWARD
        break;

  }
  state=next_state;
  //Outputs
  switch (state) {
      case 0:
          heron_foward();
          if(abs(msg->pose.pose.position.y)<2.5 && abs(msg->pose.pose.position.y)>1.5)
             heron_slowdown();
         break;
      case 1:
         heron_stop();
         heron_rotate('L');
         break;
      case 2:
         heron_stop();
         break;
      case 3:
         need_sensors = true;
         heron_rotate('L');
         break;
      case 4:

         heron_stop();
         heron_foward();
         if(pose_out_left.pose.position.x <= 3 && abs(pose_out_left.pose.position.y) <= 3 && need_sensors)
           heron_slowdown();
         break;
      case 5:
         heron_stop();  // FIM
         break;
      case 6:
         heron_rotate('R');
         break;
      case 7:
         heron_stop();
         break;
      case 8:
         need_sensors = true;
         heron_rotate('R');
         break;
      case 9:
         if(msg->pose.pose.orientation.w >0.99)
           quart = 2;
         else if(quart== 2 &&  msg->pose.pose.orientation.w < 0.9)
           quart = 3;
         heron_rotate('R');
         break;
      case 10:
         heron_stop();
         heron_foward();
         if(abs(msg->pose.pose.position.y)>(init_pos_y-2))
            heron_slowdown();
         break;
      case 11:
        if(msg->pose.pose.orientation.w < 0.1 && quart ==3)
          quart = 4;
         heron_stop();
         heron_rotate('R');
         break;
      case 12:
         heron_rotate('L');
         break;
      case 13:
         heron_stop();
         heron_foward();
         break;

  }

  if(!init_odemetry)
  {
    init_odemetry = true;
    init_pos_x = msg->pose.pose.position.x;
    init_pos_y = msg->pose.pose.position.y;
    init_pos_rot = msg->pose.pose.orientation.w;
  }
  pub.publish(vel);
   return;
}

void cb_nearest_left( const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if(need_sensors)
  {
    counter1++;
   listener->transformPose("/base_link", *msg, pose_out_left);
   if(counter1 > print_rate){
      ROS_INFO("NEAREST[L]: [%f, %f]", pose_out_left.pose.position.x,  pose_out_left.pose.position.y);
      counter1 = 0;
    }
  }
 return;
}




int count_real_corners(const std::vector<cv::Point> &result)
{
  int count=0;
  cv::Point p;

  for(size_t i = 0;i < result.size();i++)
  {
    p=result[i];
    for(size_t j =0;j<result.size();j++)
    {
      if(j != i)
      {
        if(cv::sqrt((p.x-result.at(j).x)*(p.x-result.at(j).x) + (p.y-result.at(j).y)*(p.y-result.at(j).y)) <10)
        {
          count++;
        }
      }
    }
  }
  if(count == 0) return result.size();
  return result.size()-(count/2);
}

void fill_dock_avaiability(dock& item)
{
  if(item.circle || item.cross)
  {
    item.avaiability = false;
  }else if(item.tri)
  {
    item.avaiability = true;
  }
}
void marker_publish(const std::string& color, const std::string& shape, const std::string& dock)
{
  std_msgs::String text;
  text.data= color+" "+shape+" at "+dock;

  marker_detected_pub.publish(text);
}
void fill_dock(dock& item,const bool& tri,const bool& cross,const bool& circle,const bool& color_red,const bool& color_blue,const bool& color_green,const int& corners)
{
  item.identified = true;
  item.shape = shape;
  item.color = color;
  item.tri = tri;
  item.cross = cross;
  item.circle = circle;
  item.color_red = color_red;
  item.color_blue = color_blue;
  item.color_green = color_green;
  item.corners = corners;
}
void cb_image_raw_left(const sensor_msgs::ImageConstPtr& msg)
{
  bool color_red=false,color_blue=false,color_green=false,cross=false,tri=false,circle=false;
  bool detect = 0;
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

   int thresh_low=image.size().width/2-50, thres_high=image.size().width/2+50;

  // Convert input image to HSV
  cv::Mat hsv_image,gray_image,mask_thresh;

  cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);



  cv::Mat mask_green,mask_blue,mask_red;
  cv::inRange(hsv_image,cv::Scalar(40, 40,40),cv::Scalar(70, 255,255),mask_green); //green
  cv::inRange(hsv_image,cv::Scalar(120, 40,40),cv::Scalar(130, 255,255),mask_blue); //blue
  cv::inRange(hsv_image,cv::Scalar(0, 40,40),cv::Scalar(10, 255,255),mask_red); //red

  std::vector<std::vector<cv::Point>> countors_green,countors_red,countors_blue,countors;
  std::vector<cv::Vec4i> hierarchy;



  //not needed was only causing troubles
 /* cv::Mat object_outliers_green,object_outliers_red,object_outliers_blue,object_outliers;
  cv::Canny(mask_red,object_outliers_red,100,200,3,true);
  cv::Canny(mask_green,object_outliers_green,100,200,3,true);
  cv::Canny(mask_blue,object_outliers_blue,100,200,3,true);//100 e 200*/

  //with masks the result is cleaner in terms of countors
  cv::findContours(mask_green,countors_green,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
  cv::findContours(mask_red,countors_red,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
  cv::findContours(mask_blue,countors_blue,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

  cv::Mat img;
  if(countors_green.size()>0)
  {
    countors = countors_green;
    color_green = true;
    img = mask_green;
    color = "GREEN";
  } else if(countors_red.size()>0)
  {
    countors = countors_red;
    color_red = true;
    color = "red";
    img = mask_red;
  } else if(countors_blue.size()>0)
  {
    countors = countors_blue;
    color_blue = true;
    color = "blue";
    img = mask_blue;
  }

  std::vector<cv::Point> result;
  if(countors.size() > 0)
  {
    for(size_t i = 0 ;i<countors.size();i++)
    {

      double epsilon  = 0.02* cv::arcLength(countors.at(i),true);
      cv::approxPolyDP(countors.at(i),result,epsilon,true);

      if(result.size() > 2 && abs(cv::contourArea(result))>100)
      {

       // ROS_INFO("num of vertices: %d", result.size());
        detect = true;
        int count=0;
        if(result.size() < 7 )
          count = count_real_corners(result);
        else
          count = result.size();
        //ROS_INFO("Point %d", count);
        if(count == 3 && cv::isContourConvex(result))
        {
          //ROS_INFO("ITS A  %s TRIANGLE", color.c_str());
          shape = "TRIANGLE";
          tri= true;
        } else if(count == 12)
        {
          //ROS_INFO("ITS A %s CROSS", color.c_str());
          shape = "CROSS";
          cross = true;

        }else if(count > 7 && cv::isContourConvex(result)) {
          //ROS_INFO("ITS A %s CIRCLE", color.c_str());
          shape = "CIRCLE";
          circle = true;
        }

        cv::Moments m = cv::moments(img,false);
        cv::Point p(m.m10/m.m00, m.m01/m.m00);

        if(p.x > thresh_low && p.x < thres_high )
        {
           //ROS_INFO("Point: %d  TRHESH: [ %d: %d]", p.x,thresh_low, thres_high);
          if(!color.empty() && !shape.empty())
          {
                if(p.x < (image.size().width/2 + 5) && p.x > (image.size().width/2 -5))
                {
                  usv_oriented = true;
                }else {
                  usv_oriented = false;
                }
                //ROS_INFO("ITS A %s %s", color.c_str(),shape.c_str());
                if(!orientation)
                {

                 // ROS_INFO("DOCK_LEFT ITS A %s %s", color.c_str(),shape.c_str());
                  flag_dock = true;
                  if(dock_left.corners < count)
                  {
                    fill_dock(dock_left,tri,cross,circle,color_red,color_blue,color_green,count);
                    fill_dock_avaiability(dock_left);
                    marker_publish(color,shape,"DOCK_LEFT");
                  }

                } else if(orientation)
                {
               //   ROS_INFO("DOCK_RIGHT ITS A %s %s", color.c_str(),shape.c_str());

                  flag_dock = false;
                  if(dock_right.corners < count)
                  {
                    fill_dock(dock_right,tri,cross,circle,color_red,color_blue,color_green,count);
                    fill_dock_avaiability(dock_right);
                    marker_publish(color,shape,"DOCK_RIGHT");
                  }
                }

           /* if(dock_left.identified && dock_right.identified)
              ROS_INFO("DOCK LEFT: %s   DOCK RIGHT:  %s", dock_left.shape.c_str(), dock_right.shape.c_str());*/

            /*
            cv::putText(image,"ITS a "+color+" "+shape,p,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,0),2);
            cv::imshow( "image", image);

            // Wait for a keystroke.
            cv::waitKey(1000);
            cv::destroyAllWindows();*/
          }
        }
      }

    }



    //cv::circle(object_outliers, p, 5, cv::Scalar(128,0,0), -1);
    //cv::rectangle(object_outliers,cv::Point(p.x-30,p.y-30),cv::Point(p.x+30,p.y+30),cv::Scalar(128,0,0),1,cv::LINE_8,0);


    // Show the image inside it.
 /*   cv::imshow( "normal image", hsv_image);
    cv::imshow( "mask", img);
    cv::imshow( "mask_blue", mask_blue);


    // Wait for a keystroke.
    cv::waitKey(0);
    cv::destroyAllWindows();*/
  }

  if(!detect)
  {
    if(flag_dock)
     orientation = true;
    else
     orientation = false;
  }

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
  if(init)
  {
    state=0;
    next_state = 0;
    init = false;
   // flag_dock = true;
    dock_left.corners=0;
   /* dock_left.circle = true;
    dock_left.identified = true;
    dock_left.shape = "circle";
    dock_left.color = "blue";*/
    dock_right.corners=0;
  }


  ros::Subscriber sub_nearest_left = n_public.subscribe("/lidar_left/nearest", 1, cb_nearest_left);


  ros::Subscriber left_camera_sub = n_public.subscribe("/camera/left/image_raw", 1, cb_image_raw_left);
  listener = new tf::TransformListener;
  pub = n_public.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  marker_detected_pub = n_public.advertise<std_msgs::String>("marker",1);
  //vel.angular.z = 1;

   try
   {
     listener->waitForTransform("/lidar_right_link", "base_link", ros::Time::now(), ros::Duration(3.0));
     listener->waitForTransform("/lidar_left_link", "base_link", ros::Time::now(), ros::Duration(3.0));

   }
   catch (tf::TransformException ex)
   {
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
   }

  ros::spin();
  delete listener;
}
