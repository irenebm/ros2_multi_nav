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

#include "rclcpp/rclcpp.hpp"

namespace multi_nav2
{

class ExplorationPolicy : public BT::ActionNodeBase
{
public:
  explicit ExplorationPolicy(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  virtual void set_policy(geometry_msgs::msg::PoseArray poses, std_msgs::msg::String policy);

private:
  rclcpp::Node::SharedPtr node_;

};

}  // namespace multi_nav2
