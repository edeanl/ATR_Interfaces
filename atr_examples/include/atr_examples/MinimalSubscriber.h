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

#ifndef MINIMAL_SUBSCRIBER_H
#define MINIMAL_SUBSCRIBER_H

/*! \file MinimalSubscriber.h
 *  \brief Transforms ObjectStamped message to PolygonStamped.
 *
 *  Provides the following functionalities:
 *      - Subscriber
 *          ObjectStamped
 *      - Publishers
 *          PolygonStamped
 */

#include <memory>

#include "atr_interfaces/msg/object_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace atr_examples
{
/** \class MinimalSubscriber
 *
 * \brief This class transforms ObjectStamped message to PolygonStamped
 */
class MinimalSubscriber : public rclcpp::Node
{
public:
private:
  rclcpp::Subscription<atr_interfaces::msg::ObjectStamped>::SharedPtr subscription_;  ///< Subscriber for the
                                                                                      ///< ObjectStamped topic
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;  ///< Publisher of the PolygonStamped
                                                                                ///< topic

public:
  /**
   * @brief Construct a new Minimal Subscriber object
   *
   */
  MinimalSubscriber();

  ~MinimalSubscriber();

private:
  /**
   * @brief callback function for the timer to publish the data
   *
   */
  void topic_callback(const atr_interfaces::msg::ObjectStamped::SharedPtr msg) const;
};
}  // namespace atr_examples
#endif