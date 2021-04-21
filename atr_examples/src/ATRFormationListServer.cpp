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

#include <atr_examples/ATRFormationListServer.h>
#include <atr_interfaces/msg/atr_error.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace atr_examples
{
ATRFormationListServer::ATRFormationListServer(/* args */)
  : Node("atr_formation_list_server"), atr_formation_list_(atr_interfaces::msg::ATRFormationList())
{
  init();

  service_ = create_service<atr_interfaces::srv::GetATRFormation>(
      service_name_, std::bind(&ATRFormationListServer::sendATRFormationListCB, this, _1, _2));

  // Populate a dummy list of Formations
  // We create two formations
  atr_formation_list_.formations.resize(3);

  // First Formation with two ATRs (1,2)
  int idx = 0;
  atr_formation_list_.formations[idx].atr_list.push_back(1);
  atr_formation_list_.formations[idx].atr_list.push_back(2);
  atr_formation_list_.formations[idx].formation_id = 1;
  // Create the internal map to manage the information
  map_id_index_[atr_formation_list_.formations[idx].formation_id] = idx;
  // The Formation 1 should keep a distance between the two ATRs of 15 cm with a tolerance of 10% and an angle
  // between them of 45°
  atr_formation_list_.formations[idx].constraints.dist.tolerance = 0.1;
  atr_formation_list_.formations[idx].constraints.dist.d = 0.15;
  atr_formation_list_.formations[idx].constraints.angle.tolerance = 0.1;
  atr_formation_list_.formations[idx].constraints.angle.a = M_PI_4;

  // Second Formation with three ATRS (4,6,8) distance between ATRs 10 cm +- 5% and no constraint on the angle
  idx = 1;
  atr_formation_list_.formations[idx].atr_list.push_back(4);
  atr_formation_list_.formations[idx].atr_list.push_back(6);
  atr_formation_list_.formations[idx].atr_list.push_back(8);
  atr_formation_list_.formations[idx].formation_id = 2;
  map_id_index_[atr_formation_list_.formations[idx].formation_id] = idx;
  atr_formation_list_.formations[idx].constraints.dist.tolerance = 0.05;
  atr_formation_list_.formations[idx].constraints.dist.d = 0.1;
  //   We don't need to add the value for the angle and its tolerance because by default it is set to 0 and -1 (in the
  //   message definition).

  // Second Formation with three ATRS (4,6,8) distance between ATRs 10 cm +- 5% and no constraint on the angle
  idx = 2;
  atr_formation_list_.formations[idx].atr_list.push_back(5);
  atr_formation_list_.formations[idx].atr_list.push_back(7);
  atr_formation_list_.formations[idx].atr_list.push_back(10);
  atr_formation_list_.formations[idx].formation_id = 15;
  map_id_index_[atr_formation_list_.formations[idx].formation_id] = idx;

  RCLCPP_INFO(get_logger(), "ATR Formation List server ready!!");
}

ATRFormationListServer::~ATRFormationListServer()
{
}

void ATRFormationListServer::init()
{
  declare_parameter("service_name");
  service_name_ = get_parameter("service_name").as_string();
  RCLCPP_INFO_STREAM(get_logger(), "Service name: " << service_name_);
}

void ATRFormationListServer::sendATRFormationListCB(
    const std::shared_ptr<atr_interfaces::srv::GetATRFormation::Request> request,
    std::shared_ptr<atr_interfaces::srv::GetATRFormation::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), "Number of requested formations: " << request->formation_ids.size());

  response->success = true;

  size_t formation_size = request->formation_ids.size();

  if (formation_size == 0)
  {
    std::string error_message = "The requested formation id is empty [" + std::to_string(formation_size) + "]";
    RCLCPP_ERROR_STREAM(get_logger(), error_message);
    response->success = false;
    response->error.id = atr_interfaces::msg::ATRError::INVALID_ARGUMENT;
    response->error.message = error_message;
  }

  for (auto&& i : request->formation_ids)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Requested formation ID: " << i);

    // If Formation ID = -1, then the server should send the complete list
    if (-1 == i)
    {
      response->atr_formation_list = atr_formation_list_;
    }
    else
    {
      // Check if the id exist in the list of formations
      if (1 == map_id_index_.count(i))
      {
        response->atr_formation_list.formations.push_back(atr_formation_list_.formations.at(map_id_index_[i]));
      }
      else
      {
        // The Formation Id couldn't be found in the list
        response->success = false;

        std::string error_message;
        error_message = "Formation ID: " + std::to_string(i) + " not found in the list";
        response->error.message = error_message;
        response->error.id = atr_interfaces::msg::ATRError::DATA_NOT_AVAILABLE;

        break;
      }
    }
  }
}
}  // namespace atr_examples