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

#ifndef MULTINAV2__BIG_DISTANCE_POLICY_HPP_
#define MULTINAV2__BIG_DISTANCE_POLICY_HPP_

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "multi_nav2/ExplorationPolicy.hpp"

namespace multi_nav2
{

class BigDistancePolicy : public multi_nav2::ExplorationPolicy
{
public:
  BigDistancePolicy(
    const std::string & xml_tag_name)
  : multi_nav2::ExplorationPolicy("exp")
  {
  }

  geometry_msgs::msg::PoseStamped
  get_pose(
    geometry_msgs::msg::PoseArray & msg,
    geometry_msgs::msg::Pose robot_pos_) override
  {
    double higher_distance_ = -1;

    for (size_t i = 0; i < msg.poses.size(); i++) {
      float x_ = msg.poses[i].position.x;
      float y_ = msg.poses[i].position.y;
      double distance_ =
        sqrt(
        (x_ - robot_pos_.position.x) * (x_ - robot_pos_.position.x) +
        (y_ - robot_pos_.position.y) * (y_ - robot_pos_.position.y));
      if (higher_distance_ < 0.0 || distance_ > higher_distance_) {
        higher_distance_ = distance_;
        goal_pos_.header.frame_id = "map";
        goal_pos_.pose.position.x = x_;
        goal_pos_.pose.position.y = y_;
      }
    }
    return goal_pos_;
  }

  geometry_msgs::msg::PoseStamped
  get_pose_2_robots(
    geometry_msgs::msg::PoseArray & msg,
    geometry_msgs::msg::Pose robot_pos_,
    geometry_msgs::msg::PoseStamped goal_pos_other_) override
  {
    double higher_distance_ = -1;

    for (size_t i = 0; i < msg.poses.size(); i++) {
      float x_ = msg.poses[i].position.x;
      float y_ = msg.poses[i].position.y;
      double distance_1_ =
        sqrt(
        (x_ - goal_pos_other_.pose.position.x) * (x_ - goal_pos_other_.pose.position.x) +
        (y_ - goal_pos_other_.pose.position.y) * (y_ - goal_pos_other_.pose.position.y));
      double distance_2_ =
        sqrt(
        (x_ - robot_pos_.position.x) * (x_ - robot_pos_.position.x) +
        (y_ - robot_pos_.position.y) * (y_ - robot_pos_.position.y));
      if (higher_distance_ == -1 || (distance_1_ + distance_2_) > higher_distance_) {
        higher_distance_ = (distance_1_ + distance_2_);
        goal_pos_.header.frame_id = "map";
        goal_pos_.pose.position.x = x_;
        goal_pos_.pose.position.y = y_;
      }
    }

    // si el goal es al que va ottro robot nos quedamos donde estamos
    if (goal_pos_other_.pose.position.x == goal_pos_.pose.position.x &&
      goal_pos_other_.pose.position.y == goal_pos_.pose.position.y)
    {
      goal_pos_.pose.position.x = robot_pos_.position.x;
      goal_pos_.pose.position.y = robot_pos_.position.y;
    }

    if (int(msg.poses.size()) < 1) {
      goal_pos_.pose.position.x = robot_pos_.position.x;
      goal_pos_.pose.position.y = robot_pos_.position.y;
    }

    return goal_pos_;
  }

private:
  geometry_msgs::msg::PoseStamped goal_pos_;

};

}  // namespace multi_nav2

#endif  // MULTINAV2__BIG_DISTANCE_POLICY_HPP_
