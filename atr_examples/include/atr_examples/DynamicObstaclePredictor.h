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

#ifndef DYNAMIC_OBSTACLE_PREDICTOR_H
#define DYNAMIC_OBSTACLE_PREDICTOR_H

/*! \file DynamicObstaclePredictor.h
 *  \brief Generates predicted poses of dynamic obstacles (Interface prototype).
 *
 *  Provides the following functionalities:
 *    - Object List topic subscriber (List of Dynamic and Static Objects as Polygons)
 *    - Generates predictions (dummy predictions)
 *    - PredictedObjectEllipseList topic publisher. List of dynamic object predictions.
 *    - UpdatePredictedObjectList client (to request predicted object list update in the ATRTrajectoryGenerator module)
 *
 */

#include <memory>

#include "atr_examples/AuxTools.h"
#include "atr_interfaces/msg/object_list.hpp"
#include "atr_interfaces/msg/predicted_object_ellipse_list.hpp"
#include "atr_interfaces/srv/update_predicted_object_list.hpp"

#include "rclcpp/rclcpp.hpp"

namespace atr_examples
{
/** \class DynamicObstaclePredictor
 *
 * \brief This class provides the interfaces of the Dynamic Obstacle Predictor
 */
class DynamicObstaclePredictor : public rclcpp::Node, public AuxTools
{
public:
  using PObjEllipseList = atr_interfaces::msg::PredictedObjectEllipseList;
  using ClientUpdatePredObjList = rclcpp::Client<atr_interfaces::srv::UpdatePredictedObjectList>::SharedPtr;
  using ReqUpdatePredObjList = atr_interfaces::srv::UpdatePredictedObjectList::Request::SharedPtr;
  using SrvRespFutureUpdatePObj = rclcpp::Client<atr_interfaces::srv::UpdatePredictedObjectList>::SharedFuture;

private:
  rclcpp::Subscription<atr_interfaces::msg::ObjectList>::SharedPtr subscription_;  ///< topic subscription to get the
                                                                                   ///< Object List message

  bool first_message_;  ///< Flag to resize the publisher based on the data

  std::unordered_map<int, int> map_id_index_;  ///< map to connect atr ids with indexes

  std::string subs_topic_name_;  ///< topic name for the subscriber to the topic ObjectList

  std::string pred_obj_topic_name_;  ///< topic name for the publisher of predicted objects

  std::string update_po_list_client_name_;  ///< service name to request the predicted objects update

  std::string frame_id_;  ///< reference frame for the objects, usually "map"

  size_t pred_horizon_;  ///< prediction horizon number

  double delta_time_;  ///< delta time between predictions

  rclcpp::Publisher<atr_interfaces::msg::PredictedObjectEllipseList>::SharedPtr
      pred_objects_publisher_;  ///< Publisher of the list of Predicted Objects as Ellipses

  ClientUpdatePredObjList p_obj_client_;  ///< client to update the predicted object list (ATRTrajectoryGenerator)

  std::string p_obj_service_name;  //< service name to update the predicted object list

public:
  /**
   * @brief default constructor
   *
   */
  DynamicObstaclePredictor(/* args */);
  ~DynamicObstaclePredictor();

  /**
   * @brief Initialize parameters
   *
   */
  void init();

private:
  /**
   * @brief Callback function for the subscriber
   *
   * @param msg object list message
   */
  void topic_callback(const atr_interfaces::msg::ObjectList::SharedPtr msg);
};
}  // namespace atr_examples

#endif