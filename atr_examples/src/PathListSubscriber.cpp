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

#include <atr_examples/PathListSubscriber.h>

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <atr_interfaces/convert_extended.h>

using std::placeholders::_1;

namespace atr_examples
{
PathListSubscriber::PathListSubscriber() : Node("atr_path_list_subs")
{
  // Init parameters

  init();

  // Marker publisher
  v_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_name_, 10);

  // Subscription to ATR  Path list topic
  atr_path_list_subs_ = create_subscription<atr_interfaces::msg::ATRPathList>(
      atr_path_topic_name_, 10, std::bind(&PathListSubscriber::atr_path_subs_callback, this, _1));

  RCLCPP_INFO(get_logger(), "Ready to visualize ATR Paths!");
}

PathListSubscriber::~PathListSubscriber()
{
}

void PathListSubscriber::init()
{
  std::vector<std::string> param_names = {
    "atr_path_topic_name", "period_ms", "frame_id", "path_marker_topic_name", "color_id", "size", "alpha"
  };

  for (auto&& i : param_names)
    declare_parameter(i);

  std::vector<rclcpp::Parameter> params = get_parameters(param_names);
  atr_path_topic_name_ = params.at(0).as_string();
  period_ms_ = params.at(1).as_int();
  frame_id_ = params.at(2).as_string();
  marker_topic_name_ = params.at(3).as_string();
  color_id_ = params.at(4).as_int();
  m_size_ = params.at(5).as_double();
  alpha_color_ = params.at(6).as_double();
}

void PathListSubscriber::atr_path_subs_callback(const ATRPathListShPt msg)
{
  // if the path list contains data
  if (msg->paths.size() > 0)
  {
    // Now we use the created vectors with the map
    rclcpp::Time aux_time = now();

    visualization_msgs::msg::MarkerArray m_line_msg;

    int16_t idx = 0;
    for (auto&& path : msg->paths)
    {
      visualization_msgs::msg::Marker aux_marker;
      aux_marker.action = visualization_msgs::msg::Marker::ADD;
      aux_marker.pose.orientation.w = 1.0;
      aux_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
      aux_marker.scale.x = m_size_;
      aux_marker.scale.y = m_size_;
      // Alpha channel
      aux_marker.color.a = alpha_color_;
      // Marker header
      aux_marker.header.frame_id = frame_id_;
      aux_marker.header.stamp = aux_time;

      // Get the atr id
      int16_t atr_id = path.atr_path.atr_id;

      // Use the same object ID for the marker ID
      aux_marker.id = atr_id;
      aux_marker.ns = "path_atr_lines_" + std::to_string(atr_id);

      // Define Color
      // TODO: for now, max 20 paths can be used

      aux_marker.color.r = Colors_(idx + color_id_, 0);
      aux_marker.color.g = Colors_(idx + color_id_, 1);
      aux_marker.color.b = Colors_(idx + color_id_, 2);

      for (auto&& poses : path.atr_path.path_w_time.poses)
      {
        aux_marker.points.push_back(poses.pose.position);
      }

      // Create the msg
      m_line_msg.markers.push_back(aux_marker);
      // // Add points
      aux_marker.type = visualization_msgs::msg::Marker::POINTS;
      aux_marker.ns = "path_atr_points_" + std::to_string(atr_id);

      aux_marker.scale.x = m_size_ * 2.0;
      aux_marker.scale.y = m_size_ * 2.0;
      aux_marker.color.r = Colors_(idx + color_id_ + 4, 0);
      aux_marker.color.g = Colors_(idx + color_id_ + 4, 1);
      aux_marker.color.b = Colors_(idx + color_id_ + 4, 2);

      idx++;

      m_line_msg.markers.push_back(aux_marker);

      // Call the service using the idx
    }

    // Publish Polygons as Line Markers
    v_marker_publisher_->publish(m_line_msg);
  }  // if there is new msg
  else
  {
    std::string error_message = "The ATR Path List is empty";
    RCLCPP_WARN_STREAM(get_logger(), error_message);
  }
}

}  // namespace atr_examples
