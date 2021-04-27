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

#include <atr_examples/ATRFormationListPublisher.h>

namespace atr_examples
{
ATRFormationListPublisher::ATRFormationListPublisher(/* args */) : Node("atr_formation_list_publisher")
{
  publisher_ = this->create_publisher<atr_interfaces::msg::ATRFormationList>("atr_formations", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&ATRFormationListPublisher::timer_callback, this));
}

ATRFormationListPublisher::~ATRFormationListPublisher()
{
}

void ATRFormationListPublisher::timer_callback()
{
  auto message = atr_interfaces::msg::ATRFormationList();

  // We create two formations
  message.formations.resize(2);

  // First Formation with two ATRs (1,2)
  message.formations[0].atr_list.push_back(1);
  message.formations[0].atr_list.push_back(2);
  message.formations[0].formation_id = 0;

  // The Formation 1 should keep a distance between the two ATRs of 15 cm with a tolerance of 10% and an angle between
  // them of 45°
  message.formations[0].constraints.dist.tolerance = 0.1;
  message.formations[0].constraints.dist.d = 0.15;
  message.formations[0].constraints.angle.tolerance = 0.1;
  message.formations[0].constraints.angle.a = M_PI_4;

  // Second Formation with three ATRS (4,6,8) distance between ATRs 10 cm +- 5% and no constraint on the angle
  message.formations[1].atr_list.push_back(4);
  message.formations[1].atr_list.push_back(6);
  message.formations[1].atr_list.push_back(8);
  message.formations[0].formation_id = 0;
  message.formations[1].constraints.dist.tolerance = 0.05;
  message.formations[1].constraints.dist.d = 0.1;
  //   We don't need to add the value for the angle and its tolerance because by default it is set to 0 and -1 (in the
  //   message definition).

  RCLCPP_INFO_STREAM(get_logger(), "ATRFormationList- Publishing: " << now().nanoseconds());
  publisher_->publish(message);
}

}  // namespace atr_examples