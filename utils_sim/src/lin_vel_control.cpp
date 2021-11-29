/**
  *
  * @file      lin_vel_control.cpp
  * @version   0.1
  * @date      2021-10-01
  * @authors   Daniel Campos <daniel.f.campos@inesctec.pt>
  *
  * @brief     ROS node to control Heron
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

#include "lin_vel_control.h"

void cbTwist(const geometry_msgs::Twist& msg)
{
  std_msgs::Float32 left, right;
  left.data = msg.linear.x - 0.6*msg.angular.z;
  right.data = msg.linear.x + 0.6*msg.angular.z;
  pub_left.publish(left);
  pub_right.publish(right);
}

int main(int argc, char **argv)
{
    //Init the ros system
    ros::init(argc, argv, "lin_vel_control");

    //Create the node handles to establish the program as a ROS node
    ros::NodeHandle n_public;
    ros::NodeHandle n_private("~");

    ros::Subscriber sub = n_public.subscribe("cmd_vel", 1, cbTwist);
    pub_left  = n_public.advertise<std_msgs::Float32>("left/cmd", 1);
    pub_right = n_public.advertise<std_msgs::Float32>("right/cmd", 1);

    ros::spin();

    return 0;
}
