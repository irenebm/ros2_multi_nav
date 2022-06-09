// Copyright 2021 Irene Bandera Moreno
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


#include "multi_nav2/SelectPF.hpp"

using namespace std::placeholders;

using namespace std::chrono_literals;

namespace multi_nav2
{

SelectPF::SelectPF(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  pub_goal_marker_ =
    node_->create_publisher<visualization_msgs::msg::Marker>("my_marker_goal", 10);
  timer_goal_marker_ = node_->create_wall_timer(
    1s, std::bind(&SelectPF::timer_callback_goal_marker, this));

  // publica la posicion a la que va a ir
  pub_goal_pose_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_goal", 10);

  sub_goal_pose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/pose_goal", 10, std::bind(&SelectPF::callback_goal_poses_, this, _1));

  // se subscribe a la posicion del robot en el mundo, no en el mapa
  sub_robot_pos_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&SelectPF::callback_robot_pos_, this, _1));

  // se subscribe a las posiciones de los pf que publica check_pf.cpp
  sub_poses_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    "/my_poses", 10, std::bind(&SelectPF::callback_poses_, this, _1));

  // if (a)
  //   policy_ = new LessDistancePolicy();
  // else
  //   policy_ = new BigDistancePolicy();

  policy_ = new multi_nav2::BigDistancePolicy("policy");

  timer_init_ = time(NULL);

}

void
SelectPF::halt()
{
  std::cout << "SelectPF halt" << std::endl;
}

BT::NodeStatus
SelectPF::tick()
{
  std::cout << "SelectPF tick " << std::endl;
  if (set_goal_) {
    set_goal_ = false;
    RCLCPP_INFO(
      node_->get_logger(), "SelectPF sending goal: %f %f", goal_pos_.pose.position.x,
      goal_pos_.pose.position.y);
    setOutput("waypoint", goal_pos_);
    pub_goal_pose_->publish(goal_pos_);
    return BT::NodeStatus::SUCCESS;
  } else {
    std::cout << "Still don't have a goal " << std::endl;
    return BT::NodeStatus::RUNNING;
  }

}

void
SelectPF::timer_callback_goal_marker()
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.header.frame_id = "map";
  // marker.header.stamp = ros::Time::now();
  marker.id = 1;
  marker.lifetime.sec = 5;
  marker.pose.position.x = goal_pos_.pose.position.x;
  marker.pose.position.y = goal_pos_.pose.position.y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0; // Don't forget to set the alpha! or your marker will be invisible!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  pub_goal_marker_->publish(marker);

}

void
SelectPF::callback_goal_poses_(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  other_robot_ = true;
  goal_pos_other_.pose.position.x = msg->pose.position.x;
  goal_pos_other_.pose.position.y = msg->pose.position.y;
  goal_pos_other_.pose.position.z = msg->pose.position.z;
}

void
SelectPF::callback_robot_pos_(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pos_.position.x = msg->pose.pose.position.x;
  robot_pos_.position.y = msg->pose.pose.position.y;
  robot_pos_.position.z = msg->pose.pose.position.z;
}

void
SelectPF::callback_poses_(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  write_info(*msg);

  if (other_robot_) {
    std::cout << "other robot " << std::endl;
    goal_pos_ = policy_->get_pose_2_robots(*msg, robot_pos_, goal_pos_other_);
    set_goal_ = true;
    std::cout << "before other robot " << std::endl;
  } else {
    std::cout << "no other robot " << std::endl;
    goal_pos_ = policy_->get_pose(*msg, robot_pos_);
    set_goal_ = true;
  }
}

void
SelectPF::write_info(geometry_msgs::msg::PoseArray & msg)
{
  RCLCPP_INFO(
    node_->get_logger(), "cantidad de puntos: %d tiempo %ld", int(msg.poses.size()),
    (time(NULL) - timer_init_));

  std::ofstream myfile;
  myfile.open("info_file.txt", std::ios::app);
  myfile << "cantidad de puntos: " << int(msg.poses.size()) << " tiempo " <<
  (time(NULL) - timer_init_) << "\n";
  myfile.close();
}


}  // namespace multi_nav2

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multi_nav2::SelectPF>("SelectPF");
}
