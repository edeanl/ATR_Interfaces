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

#ifndef OBJECT_LIST_SUBSCRIBER_H
#define OBJECT_LIST_SUBSCRIBER_H

/*! \file ObjectListSubscriber.h
 *  \brief Object list topic subscriber and transformations.
 *
 *  Provides the following functionalities:
 *      - Object List topic subscriber
 *      - Transformation from atr_interfaces/ObjectList to geometry_msgs/PolygonStamped
 *      - Two methods to visualize the Polygons in Rviz:
 *          PolygonStamped topic publisher dynamically instantiated
 *          Line Markers as MarkerArray publisher
 */

#include <memory>
#include "atr_examples/AuxTools.h"
#include "atr_interfaces/msg/object_list.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace atr_examples
{
/**
 * @brief Class for the Object list topic subscriber and transformations to Polygon message and Marker Array for
 * visualization
 *
 */
class ObjectListSubscriber : public rclcpp::Node, public AuxTools
{
public:
  using PolygonPublisher = rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr;

private:
  rclcpp::Subscription<atr_interfaces::msg::ObjectList>::SharedPtr subscription_;

  std::vector<PolygonPublisher> v_publishers_;

  MarkerArrayPublisher v_marker_publisher_;  ///< Marker array pub. Publishes the Object List as Line Markers

  bool first_message_;

  // clang-format off
  std::unordered_map<int, int> map_id_index_;  ///< Map to connect Object ID to index in the local memory map[id] =   index)
  // clang-format on

  std::string topic_prefix_;       ///< Prefix used to generate the Polygon publishers
  std::string marker_topic_name_;  ///< topic name for the marker array
  std::string frame_id_;           ///< reference frame for all the objects, usually "map"

public:
  /**
   * @brief standard constructor
   *
   */
  ObjectListSubscriber(/* args */);
  ~ObjectListSubscriber();

  /**
   * @brief Initialize the object
   *          It load the parameters from a yaml file
   */
  void init();

private:
  void topic_callback(const atr_interfaces::msg::ObjectList::SharedPtr msg);
};
}  // namespace atr_examples

#endif