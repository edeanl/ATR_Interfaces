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

#ifndef MINIMAL_PUBLISHER_H
#define MINIMAL_PUBLISHER_H

/*! \file MinimalPublisher.h
 *  \brief Simple Publisher class.
 *
 *  Provides the following functionalities:
 *      - Publishers
 *          Simple ObjectStamped topic
 */

// Standard
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2
#include "atr_interfaces/msg/object_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace atr_examples
{
/** \class MinimalPublisher
 *
 * \brief Simple Publisher class
 */
class MinimalPublisher : public rclcpp::Node
{
  // Public member variables
public:
  // Private member variables
private:
  rclcpp::TimerBase::SharedPtr timer_;                                          ///< Timer to trigger the publisher
  rclcpp::Publisher<atr_interfaces::msg::ObjectStamped>::SharedPtr publisher_;  ///< Publisher for the ObjectStamped
                                                                                ///< topic
  size_t count_;                                                                ///< Counter
  // Public member methods
public:
  /**
   * @brief Construct a new Minimal Publisher object
   *
   */
  MinimalPublisher();

private:
  /**
   * @brief callback function for the timer to publish the data
   *
   */
  void timer_callback();
};

}  // namespace atr_examples

#endif
