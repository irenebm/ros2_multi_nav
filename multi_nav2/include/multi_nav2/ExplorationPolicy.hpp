// Copyright 2022 Irene Bandera Moreno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "rclcpp/rclcpp.hpp"

#ifndef MULTINAV2__EXPLORATION_POLICY_HPP_
#define MULTINAV2__EXPLORATION_POLICY_HPP_

namespace multi_nav2
{

class ExplorationPolicy
{
public:
  ExplorationPolicy(
    const std::string & xml_tag_name)
  {}

  virtual geometry_msgs::msg::PoseStamped get_pose(
    geometry_msgs::msg::PoseArray & msg,
    geometry_msgs::msg::Pose robot_pos_)  = 0;

  virtual geometry_msgs::msg::PoseStamped get_pose_2_robots(
    geometry_msgs::msg::PoseArray & msg,
    geometry_msgs::msg::Pose robot_pos_,
    geometry_msgs::msg::PoseStamped goal_pos_other_) = 0;

};

}  // namespace multi_nav2

#endif  // MULTINAV2__EXPLORATION_POLICY_HPP_
