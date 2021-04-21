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

#ifndef ATR_STATE_PUBLISHER_H
#define ATR_STATE_PUBLISHER_H

/*! \file ATRStatePublisher.h
 *  \brief ATRState topic and ATR tf publishers.
 *
 *  This class simulates the ATR Control node
 *  Provides the following functionalities:
 *      - ATRState topic publisher (emulates the ATR)
 *      - ATR tf publisher using the fused_odom data
 */

// Standard
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2
#include <atr_examples/AuxTools.h>
#include "atr_interfaces/msg/atr_state_stamped.hpp"
#include "atr_interfaces/srv/update_atr_path.hpp"

#include "rclcpp/rclcpp.hpp"
#include <std_srvs/srv/trigger.hpp>

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

namespace atr_examples
{
/** \class ATRStatePublisher
 *
 * \brief This class simulates the ATR Control node (ATRs)
 */
class ATRStatePublisher : public rclcpp::Node, public AuxTools
{
  // Public member variables
public:
  using PubATRState = rclcpp::Publisher<atr_interfaces::msg::ATRStateStamped>::SharedPtr;
  using SrvUpdateATRPath = rclcpp::Service<atr_interfaces::srv::UpdateATRPath>::SharedPtr;
  using v_PoseWDTime =
      std::vector<atr_interfaces::msg::PoseWithDTime, std::allocator<atr_interfaces::msg::PoseWithDTime>>;

  // Private member variables
private:
  rclcpp::TimerBase::SharedPtr timer_;                             ///< timer to trigger the publisher
  PubATRState publisher_;                                          ///< ATRState publisher
  SrvUpdateATRPath atr_path_service_;                              ///< Service to receive the target ATR Path
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  ///< tf broadcaster to publish the ATR tf

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr broadcast_data_srv_;  ///< service to toggle the publisher
  atr_interfaces::msg::PathWithDTime target_path_;                         ///< path object to share the information
  std::mutex path_mutex_;                                                  ///< mutex to protect W/R

  int atr_id_;                      ///< ATR ID
  std::string atr_name_;            ///< ATR name
  std::string atr_topic_name_;      ///< Topic name for the ATRState
  std::string atr_service_name_;    ///< Toggle service name
  std::string atr_frame_id_;        ///< Reference frame for the data
  std::string path_service_name_;   ///< name of the service to receive the target ATR path
  int atr_period_ms_;               ///< sample rate for the publisher
  std::vector<double> v_atr_goal_;  ///< ATR position goal
  double atr_radius_;               ///< dummy variable to create a dummy trajectory
  bool broadcast_data_flag;         ///< flag to toggle the publisher
  bool atr_path_available;          ///< controls if the service cb function has already be called at least once

  int count_;  ///< Aux variable to define the dummy trajectory for the ATR

  // Public member methods
public:
  /**
   * @brief Construct a new ATRStatePublisher object
   *
   */
  ATRStatePublisher();
  /**
   * @brief Parameters initialization. Load parameters.
   *
   */
  void init();

  // Private member methods
private:
  /**
   * @brief Update the ATRPath callback function
   *
   * @param request the new atr path (e.g. from ATRFleetControl)
   * @param response success false/true, and in case of error, it provides error information
   */
  void updateATRPathCB(const std::shared_ptr<atr_interfaces::srv::UpdateATRPath::Request> request,
                       std::shared_ptr<atr_interfaces::srv::UpdateATRPath::Response> response);
  /**
   * @brief callback function triggered by the timer. It controls the publisher
   *
   */
  void timer_callback();

  /**
   * @brief Callback function for the toggle publisher service
   *
   * @param request request data (empty)
   * @param response two variables, success (true/false) and message (description of failure)
   */
  void toggle_publisher_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};
}  // namespace atr_examples

#endif
