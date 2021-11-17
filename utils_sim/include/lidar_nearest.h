/**
*
* @file      lidar_nearest.h
* @version   0.1
* @date      2021-10-01
* @authors   Daniel Campos <daniel.f.campos@inesctec.pt>
*
* @brief     ROS node for find nearest object
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

#ifndef LIDARNEAREST_H
#define LINVELCONT_H

///ROS include
#include <ros/ros.h>

///ROS MSGs includes
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

///ROS Publisher
ros::Publisher pub_left, pub_right;

void cbLaserLeft(const sensor_msgs::LaserScan& msg);
void cbLaserRight(const sensor_msgs::LaserScan& msg);

void nearestPose(const sensor_msgs::LaserScan& scan, const float& min_ang, const float& max_ang, geometry_msgs::PoseStamped& pose_out);


#endif
