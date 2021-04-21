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

#ifndef SEMANTIC_SEGMENTATION_H
#define SEMANTIC_SEGMENTATION_H

/*! \file SemanticSegmentation.h
 *  \brief Generates simulated objects and publish them as an ObjectList message.
 *
 *  Provides the following functionalities:
 *    - Object List topic publisher (List of Dynamic and Static Objects as Polygons)
 *    - Generates objects (dummy objects)
 *    - UpdateObjectList client to communicate with ObstacleListServer.h
 *    - UpdatePredictedObjectList client (to request predicted object list update in the ATRTrajectoryGenerator module)
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
#include "rclcpp/rclcpp.hpp"

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atr_interfaces/srv/update_object_list.hpp>

using namespace std::chrono_literals;

namespace atr_examples
{
/**
 * @brief This class generates simulated objects and publish them as an ObjectList message
 *
 */
class SemanticSegmentation : public rclcpp::Node, public AuxTools
{
  // Public member variables
public:
  // Private member variables
  using ServiceResponseFuture = rclcpp::Client<atr_interfaces::srv::UpdateObjectList>::SharedFuture;

private:
  rclcpp::TimerBase::SharedPtr timer_;                                       ///< Timer to trigger the publisher
  rclcpp::Publisher<atr_interfaces::msg::ObjectList>::SharedPtr publisher_;  ///< ObjectList publisher

  rclcpp::TimerBase::SharedPtr update_timer_;  ///< timer to trigger new object list

  std::mutex data_mutex_;  ///< mutex to protect write/read access between publisher and client

  std::string topic_name_;  ///< topic name for the object list message
  std::string frame_id_;    ///< reference frame for all the objects, usually "map"

  // Public member methods
public:
  /**
   * @brief Construct a new Object List Publisher object
   *
   */
  SemanticSegmentation();

  /**
   * @brief Parameter loading and initialization using yaml file
   *
   */
  void init();

  // Private member methods
private:
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
