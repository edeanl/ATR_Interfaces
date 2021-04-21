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

#ifndef ATR_FORMATION_LIST_PUBLISHER_H
#define ATR_FORMATION_LIST_PUBLISHER_H

/*! \file ATRFormationListPublisher.h
 *  \brief Provides a publisher for the ATR Formation List
 *
 *  This class is the publisher version of ATRFormationListServer class
 *  Provides the following functionalities:
 *      - Publisher
 *          publish the ATR Formation List as a topic
 */

// Standard
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2
#include "atr_interfaces/msg/atr_formation_list.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace atr_examples
{
/** \class ATRFormationListPublisher
 *
 * \brief Provides the ros interfaces to generate the ATR Formation information as a topic
 */
class ATRFormationListPublisher : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer_;  ///< timer to control the frequency of the publisher
  rclcpp::Publisher<atr_interfaces::msg::ATRFormationList>::SharedPtr publisher_;  ///< ros publisher

  // Public methods
public:
  /**
   * @brief Default constructor
   *
   */
  ATRFormationListPublisher(/* args */);
  ~ATRFormationListPublisher();

  //   Private methods
private:
  /**
   * @brief Defines the ATR Formation and publish it as a topic
   *
   */
  void timer_callback();
};

}  // namespace atr_examples
#endif