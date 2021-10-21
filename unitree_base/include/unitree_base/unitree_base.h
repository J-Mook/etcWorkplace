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

#ifndef UNITREE_BASE__UNITREE_BASE_H_
#define UNITREE_BASE__UNITREE_BASE_H_

#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_msgs/HighCmd.h"
#include "unitree_legged_msgs/HighState.h"
#include "unitree_base/convert.h"

using namespace std;

using namespace UNITREE_LEGGED_SDK;

class UnitreeBase
{
public:
  UnitreeBase(ros::NodeHandle& nh, ros::NodeHandle& nh_priv, UNITREE_LEGGED_SDK::LCM& roslcm);//, UNITREE_LEGGED_SDK::UDP& udp);

  virtual ~UnitreeBase() = default;

  /**
   * \brief Initialize
   */
  bool init();

  /** 
   * \brief Velocity command callback
   * \param msg Velocity command
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  /**
   * \brief Stop command callback
   * \param msg Stop command
   */
  void stopCallback(const std_msgs::Bool::ConstPtr& msg);

  void cmdPoseCallback();

  /**
   * \brief Publish the state of the robot
   */
  void publishState();

  /**
   * \brief Loop for publishing
   */
  void publishLoop();

  /**
   * \brief Loop for control
   */
  void controlLoop();

  /**
   * \brief Loop for updating
   */
  void updateLoop();

  /// Controller manager for the infrastructure to interact with controllers
  shared_ptr<controller_manager::ControllerManager> cm_;

private:
  /// ROS parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  realtime_tools::RealtimePublisher<unitree_legged_msgs::HighState> state_;

  ros::Subscriber sub_cmd_vel_;
  ros::Subscriber sub_stop_;

  /// Unitree SDK
  UNITREE_LEGGED_SDK::LCM roslcm_;

  UNITREE_LEGGED_SDK::HighCmd send_high_LCM_;
  UNITREE_LEGGED_SDK::HighState recv_high_LCM_;

  unitree_legged_msgs::HighCmd send_high_ros_;
  unitree_legged_msgs::HighState recv_high_ros_;

  /// Stop state
  bool stop_;
};

#endif  // UNITREE_BASE__UNITREE_BASE_H_
