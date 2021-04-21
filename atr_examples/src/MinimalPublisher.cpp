/*! \file
 *
 * \author Emmanuel Dean
 *
 * \version 0.1
 * \date 15.03.2021
 *
 * \copyright Copyright 2021 Chalmers
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received financial  support  from  Chalmers  AI  Re-search Centre
 *  (CHAIR) and AB Volvo (Project ViMCoR).
 */

#include <atr_examples/MinimalPublisher.h>

#include <Eigen/Sparse>
#include <geometry_msgs/msg/point32.hpp>
#include <atr_interfaces/msg/object_class.hpp>
#include <atr_interfaces/msg/object_type.hpp>

namespace atr_examples
{
MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"), count_(0)
{
  publisher_ = this->create_publisher<atr_interfaces::msg::ObjectStamped>("object_list", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
}

void MinimalPublisher::timer_callback()
{
  auto message = atr_interfaces::msg::ObjectStamped();

  message.header.frame_id = "/map";
  message.header.stamp = this->now();

  message.object_c.o_class = atr_interfaces::msg::ObjectClass::FORKLIFT;
  message.object_t.o_type = atr_interfaces::msg::ObjectType::DYNAMIC;
  message.object_id = 1;
  message.object_idx = 0;

  std::vector<Eigen::Vector3d> points;

  points.resize(5);

  points[0] << 0, 0, 0;
  points[1] << 0, 1, 0;
  points[2] << 1, 1.5, 0;
  points[3] << 2, 1, 0;
  points[4] << 2.5, 0.1, 0;

  geometry_msgs::msg::Point32 p;

  for (auto&& i : points)
  {
    p.x = i[0];
    p.y = i[1];
    p.z = i[2];

    message.polygon.points.push_back(p);
  }

  // now().seconds();

  RCLCPP_INFO_STREAM(get_logger(), "MP- Publishing: " << now().nanoseconds());
  publisher_->publish(message);
}
}  // namespace atr_examples