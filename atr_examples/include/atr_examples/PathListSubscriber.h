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

#ifndef PATH_LIST_SUBSCRIBER_H
#define PATH_LIST_SUBSCRIBER_H

/*! \file PathListSubscriber.h
 *  \brief Transforms the ATRPathList topic to a MarkerArray.
 *
 *  Provides the following functionalities:
 *      - Subscribers
 *          ATRPath List (to get the generated Paths, e.g. TrajectoryGenerator node)
 *      - Publishers
 *          MarkerArray, publishes all the ATR Paths as a MarkerArray (to visualize them in rviz)
 */

#include <memory>

#include "atr_examples/AuxTools.h"
// List of Paths
#include "atr_interfaces/msg/atr_path_list.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace atr_examples
{
/**
 * @brief This class transforms the ATRPathList topic to a MarkerArray.
 *
 */
class PathListSubscriber : public rclcpp::Node, public AuxTools
{
public:
  // Subscribe to the PathList topic to visualize it as a MarkerArray
  using SubsATRPathList = rclcpp::Subscription<atr_interfaces::msg::ATRPathList>::SharedPtr;
  using ATRPathListShPt = atr_interfaces::msg::ATRPathList::SharedPtr;

private:
  SubsATRPathList atr_path_list_subs_;  ///< subscriber to the ATRPathList topic

  MarkerArrayPublisher v_marker_publisher_;  ///< Marker array pub. Publishes each ATR Path  as Line Markers

  std::string atr_path_topic_name_;  ///< ATR Path list topic name for the subscriber
  int period_ms_;                    ///< Sample time for the main process
  std::string frame_id_;             ///< reference frame for the paths
  // int atr_number_;                   ///< number of available ATRs (needed to create the clients)
  std::string marker_topic_name_;  ///< Marker Array topic name
  int color_id_;                   ///< Color ID to change the color of the published markers
  double m_size_;                  ///< Marker size
  double alpha_color_;             ///< alpha channel

public:
  /**
   * @brief standard constructor
   *
   */
  PathListSubscriber(/* args */);
  ~PathListSubscriber();

  /**
   * @brief Initialize the object
   *          It load the parameters from a yaml file
   */
  void init();

private:
  /**
   * @brief callback function for the ATR Path List subscriber
   *
   * @param msg ATRState list
   */
  void atr_path_subs_callback(const ATRPathListShPt msg);
};
}  // namespace atr_examples

#endif