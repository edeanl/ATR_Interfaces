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

#include <atr_examples/MinimalSubscriber.h>

// #include <geometry_msgs/msg/polygon_stamped.hpp>

namespace atr_examples
{
MinimalSubscriber::MinimalSubscriber() : Node("minimal_subscriber")
{
  subscription_ = create_subscription<atr_interfaces::msg::ObjectStamped>(
      "/object_list", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

  publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/ObjPoly", 10);
}

MinimalSubscriber::~MinimalSubscriber()
{
}

void MinimalSubscriber::topic_callback(const atr_interfaces::msg::ObjectStamped::SharedPtr msg) const
{
  RCLCPP_INFO_STREAM(get_logger(), "I hear: " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec);

  geometry_msgs::msg::PolygonStamped aux_poly;

  aux_poly.header = msg->header;
  aux_poly.polygon = msg->polygon;

  publisher_->publish(aux_poly);
}
}  // namespace atr_examples