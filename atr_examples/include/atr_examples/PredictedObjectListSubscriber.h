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

#ifndef PREDICTED_OBJECT_LIST_SUBSCRIBER_H
#define PREDICTED_OBJECT_LIST_SUBSCRIBER_H

/*! \file PredictedObjectListSubscriber.h
 *  \brief Publishes predicted objects markers and tfs.
 *
 *  Provides the following functionalities:
 *    - Subscriber to the PredictedObjectsEllipsesList topic
 *    - Transforms message from PredictedObjectEllipseList to tfs and MarkerArray
 *    - Publishes MarkerArray with the predicted objects as Ellipses
 *    - Publishes the Tfs for the Markers.
 *
 */

#include <memory>

#include "atr_examples/AuxTools.h"
#include "atr_interfaces/msg/predicted_object_ellipse_list.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <visualization_msgs/msg/marker_array.hpp>

using std::placeholders::_1;

namespace atr_examples
{
/**
 * @brief This class publishes predicted objects markers and tfs
 *
 */
class PredictedObjectListSubscriber : public rclcpp::Node, public AuxTools
{
public:
private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  ///< TF broadcaster for the predicted objs
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr v_marker_publisher_;  ///< Marker array pub
  rclcpp::Subscription<atr_interfaces::msg::PredictedObjectEllipseList>::SharedPtr
      subscription_;  ///< PredictedObjectEllipseList
                      ///< topic subscriber

  bool first_message_;  ///< Flag to control when the first message has been received

  // clang-format off
  std::unordered_map<int, int> map_id_index_;  ///< Map to connect Object ID to index in the local memory map[id] =   index)
  // clang-format on

  std::string topic_name_;              ///< topic name for the visualization markers
  std::string topic_namespace_;         ///< ns for the MarkerArray
  std::string pred_object_topic_name_;  ///< topic name for the PredictedObjectEllipseList subscriber
  std::string pred_frame_name_;         ///< frame name prefix for the dynamic object tfs

  size_t pred_horizon_;  ///< number of steps for the prediction horizon

  visualization_msgs::msg::MarkerArray m_array_;  ///< base markers

  std::string frame_id_;  ///< reference frame for the objects, usually "map"

public:
  /**
   * @brief standard constructor
   *
   */
  PredictedObjectListSubscriber(/* args */);
  ~PredictedObjectListSubscriber();

  /**
   * @brief Initialize the object
   *          It load the parameters from a yaml file
   */
  void init();

private:
  /**
   * @brief Subscriber callback function for the PredictedObjectEllipseList topic
   *
   * @param msg received message
   */
  void topic_callback(const atr_interfaces::msg::PredictedObjectEllipseList::SharedPtr msg);
};
}  // namespace atr_examples

#endif