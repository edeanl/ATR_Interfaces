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

#ifndef NONA_GENERATOR_H
#define NONA_GENERATOR_H

/*! \file NONAGenerator.h
 *  \brief Generates dummy objects and publish them as an NONAObjectList message.
 *
 *  Provides the following functionalities:
 *    - NONA Object List topic publisher (List of NONA Objects as Line Markers)
 *    - Generates objects (dummy objects)
 *    - GetPredictedObjectList service (to provide nona object list to the ATRTrajectoryGenerator module)
 *
 */

// Standard
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2
#include "atr_examples/AuxTools.h"
#include "atr_interfaces/msg/object_list.hpp"
#include "atr_interfaces/srv/get_object_list.hpp"

#include "rclcpp/rclcpp.hpp"

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

namespace atr_examples
{
/** \class NonaGenerator
 *
 * \brief This class simulates the Non-Accessible Objects generator node
 */
class NonaGenerator : public rclcpp::Node, public AuxTools
{
  // Public member variables
public:
  using SrvObjectList = rclcpp::Service<atr_interfaces::srv::GetObjectList>::SharedPtr;
  // Private member variables
private:
  rclcpp::TimerBase::SharedPtr timer_;                                       ///< Timer to trigger the publisher
  rclcpp::Publisher<atr_interfaces::msg::ObjectList>::SharedPtr publisher_;  ///< ObjectList publisher

  rclcpp::TimerBase::SharedPtr update_timer_;  ///< timer to trigger new object list

  MarkerArrayPublisher v_marker_publisher_;  ///< Marker array pub. Publishes the NONA polygons as line Markers

  SrvObjectList service_;  ///< Service to provide the NONA Object List

  std::string topic_name_;           ///< topic name for the nona object list message
  std::string marker_topic_name_;    ///< topic name for the marker array
  std::string frame_id_;             ///< reference frame for all the objects, usually "map"
  std::string get_o_list_srv_name_;  ///< service name to provide the NONA object list

  atr_interfaces::msg::ObjectList nona_list_;  ///< list of nona objects (shared variable between publisher and service)
  std::mutex data_mutex_;  ///< Mutex to protect write/read access between the service and the publisher

  // Public member methods
public:
  /**
   * @brief Construct a new Object List Publisher object
   *
   */
  NonaGenerator();

  /**
   * @brief Parameter loading and initialization using yaml file
   *
   */
  void init();

  // Private member methods
private:
  /**
   * @brief Callback function for the service to provide NONA Obj list
   *
   * @param request (empty)
   * @param response List of NONA objects and success flag with error message describing the failure
   */
  void getObjectListCB(const std::shared_ptr<atr_interfaces::srv::GetObjectList::Request> request,
                       std::shared_ptr<atr_interfaces::srv::GetObjectList::Response> response);

  /**
   * @brief callback function for the timer.
   *
   * In this case, this function creates the objects and publish them.
   *
   */
  void timer_callback();
};
}  // namespace atr_examples

#endif
