/*
Software License Agreement (BSD License)

Authors : Brighten Lee <shlee@roas.co.kr>

Copyright (c) 2021, ROAS Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "unitree_base/unitree_base.h"

//TODO
using namespace UNITREE_LEGGED_SDK;

UnitreeBase::UnitreeBase(ros::NodeHandle& nh, ros::NodeHandle& nh_priv, UNITREE_LEGGED_SDK::LCM& roslcm)
  : nh_(nh), nh_priv_(nh_priv), roslcm_(roslcm), stop_(false)
{
  sub_cmd_vel_ = nh_.subscribe("unitree/cmd_vel", 1, &UnitreeBase::cmdVelCallback, this);
  sub_stop_ = nh_.subscribe("unitree/stop", 10, &UnitreeBase::stopCallback, this);

  state_.init(nh_, "unitree/state", 1);
}

bool UnitreeBase::init()
{
  return true;
}

void UnitreeBase::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (!stop_)
  {
    send_high_ros_.mode = 2;  // walk mode
    send_high_ros_.forwardSpeed = msg->linear.x;
    send_high_ros_.sideSpeed = msg->linear.y;
    send_high_ros_.rotateSpeed = msg->angular.z;

    // std::cout << "send_high_ros_.rotateSpeed : " << send_high_ros_.rotateSpeed << std::endl; // TODO

    if (!(msg->linear.x || msg->linear.y || msg->angular.z))
      send_high_ros_.mode = 1;  // forced stand mode
  }
}

void UnitreeBase::stopCallback(const std_msgs::Bool::ConstPtr& msg)
{
  stop_ = msg->data;

  if (stop_)
  {
    send_high_ros_.mode = 0;  // idle mode
    send_high_ros_.forwardSpeed = 0.0;
    send_high_ros_.sideSpeed = 0.0;
    send_high_ros_.rotateSpeed = 0.0;
  }
}

void UnitreeBase::publishState()
{
  if (state_.trylock())
  {
    state_.msg_ = recv_high_ros_;
    state_.unlockAndPublish();
  }
}

void UnitreeBase::publishLoop()
{
  while (ros::ok())
  {
    publishState();

    ros::Rate(50).sleep(); //50Hz
  }
}

void UnitreeBase::controlLoop()
{
  roslcm_.SubscribeState();

  while (ros::ok())
  {
    roslcm_.Get(recv_high_LCM_);
    recv_high_ros_ = ToRos(recv_high_LCM_);

    // std::cout << "recv_high_ros_.rotateSpeed : " << recv_high_ros_.rotateSpeed << std::endl; // TODO

    send_high_LCM_ = ToLcm(send_high_ros_, send_high_LCM_); //UNITREE_LEGGED_SDK::HighCmd ToLcm
    roslcm_.Send(send_high_LCM_);

    ros::spinOnce();
    ros::Rate(500).sleep(); //500Hz
  }
}

void UnitreeBase::updateLoop()
{
  while (ros::ok())
  {
    roslcm_.Recv();
    usleep(2000);
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "unitree_base_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  UNITREE_LEGGED_SDK::InitEnvironment();

  UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
  auto unitree = make_shared<UnitreeBase>(nh, nh_priv, roslcm);

  if (unitree->init())
  {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create thread for publishing the data
    thread publish([&unitree]() -> void { unitree->publishLoop(); });

    // Create thread for control
    thread contol([&unitree]() -> void { unitree->controlLoop(); });

    // Create thread for updating the data
    thread update([&unitree]() -> void { unitree->updateLoop(); });

    ros::waitForShutdown();
    spinner.stop();
  }

  return EXIT_FAILURE;
}