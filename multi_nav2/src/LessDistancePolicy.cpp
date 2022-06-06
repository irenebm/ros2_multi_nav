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


#include "multi_nav2/LessDistancePolicy.hpp"

using namespace std::placeholders;


namespace multi_nav2
{

LessDistancePolicy::LessDistancePolicy(
  const std::string & xml_tag_name)
: multi_nav2::ExplorationPolicy(xml_tag_name)
{

}

geometry_msgs::msg::PoseStamped
LessDistancePolicy::get_pose(
  geometry_msgs::msg::PoseArray & msg,
  geometry_msgs::msg::Pose robot_pos_)
{
  double higher_distance_ = -1;

  for (int i = 0; i < int(msg.poses.size()); i++) {
    float x_ = msg.poses[i].position.x;
    float y_ = msg.poses[i].position.y;
    double distance_ = abs(x_ - robot_pos_.position.x) + abs(y_ - robot_pos_.position.y);
    if (higher_distance_ == -1 || distance_ > higher_distance_) {
      higher_distance_ = distance_;
      goal_pos_.header.frame_id = "map";
      goal_pos_.pose.position.x = x_;
      goal_pos_.pose.position.y = y_;
      // RCLCPP_INFO(node_->get_logger(), "goal asigno x  %f y  %f ", x_, y_);
    }
  }
  return goal_pos_;
}


geometry_msgs::msg::PoseStamped
LessDistancePolicy::get_pose_2_robots(
  geometry_msgs::msg::PoseArray & msg,
  geometry_msgs::msg::Pose robot_pos_,
  geometry_msgs::msg::PoseStamped goal_pos_other_)
{
  double higher_distance_ = -1;

  for (int i = 0; i < int(msg.poses.size()); i++) {
    float x_ = msg.poses[i].position.x;
    float y_ = msg.poses[i].position.y;
    double distance_1_ = abs(x_ - goal_pos_other_.pose.position.x) + abs(
      y_ - goal_pos_other_.pose.position.y);
    double distance_2_ = abs(x_ - robot_pos_.position.x) + abs(y_ - robot_pos_.position.y);
    if (higher_distance_ == -1 || (distance_1_ + distance_2_) > higher_distance_) {
      higher_distance_ = (distance_1_ + distance_2_);
      goal_pos_.header.frame_id = "map";
      goal_pos_.pose.position.x = x_;
      goal_pos_.pose.position.y = y_;
    }
  }

  return goal_pos_;
}

}  // namespace multi_nav2
