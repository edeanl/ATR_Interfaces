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

#ifndef ATR_FORMATION_LIST_SERVER_H
#define ATR_FORMATION_LIST_SERVER_H

/*! \file ATRFormationListServer.h
 *  \brief Provides a service to request the ATR Formation List
 *  This class is the server version of ATRFormationListPublisher class
 *  Provides the following functionalities:
 *      - Service
 *          updates the ATR Formation List
 */

#include <memory>

#include "atr_interfaces/srv/get_atr_formation.hpp"
#include "rclcpp/rclcpp.hpp"

namespace atr_examples
{
/** \class ATRFormationListServer
 *
 * \brief Provides the ros interfaces to generate the ATR Formation information as a service
 */
class ATRFormationListServer : public rclcpp::Node
{
private:
  rclcpp::Service<atr_interfaces::srv::GetATRFormation>::SharedPtr service_;  ///< service to get formation

  atr_interfaces::msg::ATRFormationList atr_formation_list_;  ///< internal allocation of ATR formation list

  std::unordered_map<int, int> map_id_index_;  ///< Map to connect Formation ID to index in the local memory
                                               ///< (map[id]=index)

  std::string service_name_;  ///< name of the provided service

public:
  /**
   * @brief Default constructor
   *
   */
  ATRFormationListServer(/* args */);
  ~ATRFormationListServer();

private:
  /**
   * @brief Load parameters
   *
   */
  void init();

  /**
   * @brief callback function for the service
   *
   * @param request vector of formation ids
   * @param response list of atr formations
   */
  void sendATRFormationListCB(const std::shared_ptr<atr_interfaces::srv::GetATRFormation::Request> request,
                              std::shared_ptr<atr_interfaces::srv::GetATRFormation::Response> response);
};

}  // namespace atr_examples

#endif