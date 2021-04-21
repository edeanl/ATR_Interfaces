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

#ifndef OBJECT_LIST_SERVER_H
#define OBJECT_LIST_SERVER_H

/*! \file ObjectListServer.h
 *  \brief Generates dummy predicted objects and publish them as an PredictedObjectEllipseList message.
 *
 *  Provides the following functionalities:
 *    - Object List server (to receive the List of Dynamic and Static Objects as Polygons as a request).
 *      This is the version where the semantic segmentation uses a client to send the
 *      new object list.
 *    - Generates predicted objects (dummy predicted objects)
 *
 */

#include <memory>

#include "atr_interfaces/srv/update_object_list.hpp"
#include <atr_interfaces/msg/predicted_object_ellipse_list.hpp>
#include <atr_examples/AuxTools.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "rclcpp/rclcpp.hpp"

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <visualization_msgs/msg/marker_array.hpp>

namespace atr_examples
{
/**
 * @brief This class generates simulated predicted objects and publish them as a PredictedObjectEllipseList message and
 * service
 *
 */
class ObjectListServer : public rclcpp::Node, public AuxTools
{
private:
  rclcpp::Service<atr_interfaces::srv::UpdateObjectList>::SharedPtr service_;  ///< service to send new object list

  std::string update_o_list_srv_name_;      ///< service name
  std::string update_po_list_client_name_;  ///< client name
  std::string pred_obj_topic_name_;         ///< topic name for the predicted object list topic
  std::string frame_id_;                    ///< reference frame for the published data, typically "map".

  atr_interfaces::msg::ObjectList object_list_;  ///< internal allocation of the
                                                 ///< object list

  rclcpp::Publisher<atr_interfaces::msg::PredictedObjectEllipseList>::SharedPtr
      pred_objects_publisher_;  ///< Publisher of the list of Predicted Objects as Ellipses

  atr_interfaces::msg::PredictedObjectEllipseList pred_obj_msg_;  ///< Predicted objects message container

  rclcpp::TimerBase::SharedPtr timer_;  ///< timer to control the predicted object message publisher

  size_t pred_horizon_;  ///< prediction horizon

  bool got_data_;  ///< flag to control when new data has arrived for the first time

  double delta_time_;  ///< delta time needed in the PredictedObjectEllipseList message

  std::mutex data_mutex_;  ///< Mutex to protect write/read access between the service and the publisher

  // clang-format off
  std::unordered_map<int, int> map_id_index_;  ///< Map to connect Object ID to index in the local memory map[id] =   index)
  // clang-format on

public:
  /**
   * @brief Default constructor
   *
   */
  ObjectListServer(/* args */);
  ~ObjectListServer();

  void init();

private:
  /**
   * @brief Callback function for the service
   *
   * @param request New list of objects (static and dynamic)
   * @param response true success, false failure with message describing the failure
   */
  void updateObjectListCB(const std::shared_ptr<atr_interfaces::srv::UpdateObjectList::Request> request,
                          std::shared_ptr<atr_interfaces::srv::UpdateObjectList::Response> response);

  /**
   * @brief Timer callback function to trigger the topic publisher
   *
   */
  void timer_callback();
};

}  // namespace atr_examples

#endif