/**
  *
  * @file      lidar_nearest.cpp
  * @version   0.1
  * @date      2021-10-01
  * @authors   Daniel Campos <daniel.f.campos@inesctec.pt>
  *
  * @brief     ROS node to find nearest object
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * @copyright Copyright (c) 2020, INESC TEC - CRAS, All rights reserved.
  *
  */

#include "lidar_nearest.h"

void nearestPose(const sensor_msgs::LaserScan& scan, const float& min_ang, const float& max_ang, geometry_msgs::PoseStamped& pose_out)
{
  float min_ang_found = INFINITY, min_dist = INFINITY;
  for(int i = 0; i < scan.ranges.size(); i++)
  {
    float angle = scan.angle_min + static_cast<float>(i)*scan.angle_increment;

    if(angle > max_ang || angle < min_ang)
      continue;

    if(min_dist > scan.ranges[i])
    {
      min_dist = scan.ranges[i];
      min_ang_found = angle;
    }
  }

  if(min_ang_found != INFINITY && min_dist != INFINITY)
  {
    pose_out.pose.position.x = min_dist*cos(min_ang_found);
    pose_out.pose.position.y = min_dist*sin(min_ang_found);
    pose_out.pose.position.z = 0.0;

    pose_out.pose.orientation.x = 0.0;
    pose_out.pose.orientation.y = 0.0;
    pose_out.pose.orientation.z = 0.0;
    pose_out.pose.orientation.w = 1.0;
  }

  return;
}

void cbLaserLeft(const sensor_msgs::LaserScan& msg)
{
  geometry_msgs::PoseStamped near_left;
  near_left.header = msg.header;
  nearestPose(msg, -M_PI_2, M_PI_2, near_left);
  pub_left.publish(near_left);
}

void cbLaserRight(const sensor_msgs::LaserScan& msg)
{
  geometry_msgs::PoseStamped near_right;
  near_right.header = msg.header;
  nearestPose(msg, -M_PI_2, M_PI_2, near_right);
  pub_right.publish(near_right);
}

int main(int argc, char **argv)
{
    //Init the ros system
    ros::init(argc, argv, "lidar_nearest");

    //Create the node handles to establish the program as a ROS node
    ros::NodeHandle n_public;
    ros::NodeHandle n_private("~");

    ros::Subscriber sub_l = n_public.subscribe("/lidar_left/scan", 1, cbLaserLeft);
    ros::Subscriber sub_r = n_public.subscribe("/lidar_right/scan", 1, cbLaserRight);

    pub_left  = n_public.advertise<geometry_msgs::PoseStamped>("lidar_left/nearest", 1);
    pub_right = n_public.advertise<geometry_msgs::PoseStamped>("lidar_right/nearest", 1);

    ros::spin();

    return 0;
}
