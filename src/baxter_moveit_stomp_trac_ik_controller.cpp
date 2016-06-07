/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

geometry_msgs::PoseStamped endEffectorPoseStamped;
ros::Publisher moveitPoseCommandPublisher;

void endEffectorCommandCallback(const geometry_msgs::PoseStamped &targetPoseStamped) {
    ROS_INFO("Got end effector command");
    endEffectorPoseStamped = targetPoseStamped;
}

void moveitControlCommandCallBack(const std_msgs::String commandString) {
    ROS_INFO("Got moveit command: %s", commandString.data.c_str());
    moveitPoseCommandPublisher.publish(endEffectorPoseStamped);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "baxter_moveit_trac_ik_controller");
  ros::NodeHandle nodeHandle;

  ros::Subscriber endEffectorCommandSubscriber = nodeHandle.subscribe("end_effector_command_pose_stamped", 10, endEffectorCommandCallback);
  ros::Subscriber moveitControlSubscriber = nodeHandle.subscribe("moveit_control_command", 10, moveitControlCommandCallBack);
  moveitPoseCommandPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("moveit_pose_command", 10);

  ros::spin();
  return 0;
}
