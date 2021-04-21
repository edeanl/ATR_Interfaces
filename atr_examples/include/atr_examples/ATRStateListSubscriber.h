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

/*! \file ATRStateListSubscriber.h
 *  \brief ATRState list topic subscriber and transformations.
 *
 *  Provides the following functionalities:
 *      - ATRState List topic subscriber
 *      - MarkerArray publisher to visualize the ATRs in Rviz
 */

#include <memory>
#include "atr_examples/AuxTools.h"
#include "atr_interfaces/msg/atr_state_list_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

namespace atr_examples
{
/** \class ATRStateListSubscriber
 *
 * \brief This class publishes the ATRStateList as visualization markers
 */
class ATRStateListSubscriber : public rclcpp::Node, public AuxTools
{
public:
private:
  rclcpp::Subscription<atr_interfaces::msg::ATRStateListStamped>::SharedPtr subscription_;  ///< Subscriber for the
                                                                                            ///< ATRStateList topic

  MarkerArrayPublisher v_marker_publisher_;  ///< Marker array pub. Publishes the Object List as Line Markers

  bool first_message_;  ///< to control when a message has been received

  // clang-format off
  std::unordered_map<int, int> map_id_index_;  ///< Map to connect Object ID to index in the local memory map[id] =   index)
  // clang-format on

  std::string marker_topic_name_;  ///< topic name for the marker array
  std::string marker_namespace_;   ///< namespace for the marker array
  std::string frame_id_;           ///< reference frame for all the objects, usually "map"
  std::string subs_topic_name_;    ///< the name of the ATRStateList topic which we will subscribe to

  visualization_msgs::msg::Marker robot_marker_;  ///< template for the ATR Marker mesh

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  ///< tf broadcaster for the c.f. references

public:
  /**
   * @brief standard constructor
   *
   */
  ATRStateListSubscriber(/* args */);
  ~ATRStateListSubscriber();

  /**
   * @brief Initialize the object
   *          It load the parameters from a yaml file
   */
  void init();

private:
  /**
   * @brief Callback function for the subscriber
   *
   * @param msg ATRState List message
   */
  void topic_callback(const atr_interfaces::msg::ATRStateListStamped::SharedPtr msg);
};
}  // namespace atr_examples

#endif