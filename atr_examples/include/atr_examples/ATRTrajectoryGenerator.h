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

#ifndef ATR_TRAJECTORY_GENERATOR_H
#define ATR_TRAJECTORY_GENERATOR_H

/*! \file ATRTrajectoryGenerator.h
 *  \brief Provides dummy trajectory for the ATRs.
 *
 *  Provides the following functionalities:
 *      - List of paths for the ATRs, as publisher and client (to send info to the Fleet Ctrl)
 *      - Subscribers
 *          ATRState List
 *          Object List
 *      - Clients
 *          ATRFormation
 *          Path(not really implemented)
 *          NONA Object List
 *      - Service
 *          Update Predicted Object List
 */

#include <memory>

#include "atr_examples/AuxTools.h"
#include "atr_interfaces/msg/object_list.hpp"
#include "atr_interfaces/msg/atr_state_list_stamped.hpp"
#include "atr_interfaces/msg/atr_path_list.hpp"
#include "atr_interfaces/srv/get_atr_formation.hpp"
#include "atr_interfaces/srv/get_object_list.hpp"
#include "atr_interfaces/srv/update_predicted_object_list.hpp"
#include "atr_interfaces/srv/update_atr_path_list.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace atr_examples
{
/** \class ATRTrajectoryGenerator
 *
 * \brief This class provides the interfaces of the ATR Trajectory Generator
 */
class ATRTrajectoryGenerator : public rclcpp::Node, public AuxTools
{
public:
  using SrvRespFutureFormation = rclcpp::Client<atr_interfaces::srv::GetATRFormation>::SharedFuture;
  using ClientGetATRFormation = rclcpp::Client<atr_interfaces::srv::GetATRFormation>::SharedPtr;
  using SubsATRStateListStamped = rclcpp::Subscription<atr_interfaces::msg::ATRStateListStamped>::SharedPtr;
  using SubsObjectList = rclcpp::Subscription<atr_interfaces::msg::ObjectList>::SharedPtr;
  using ClientGetObjectList = rclcpp::Client<atr_interfaces::srv::GetObjectList>::SharedPtr;
  using SrvUpdatePredObjList = rclcpp::Service<atr_interfaces::srv::UpdatePredictedObjectList>::SharedPtr;
  using PubATRPathList = rclcpp::Publisher<atr_interfaces::msg::ATRPathList>::SharedPtr;
  using ClientUpdateATRPathList = rclcpp::Client<atr_interfaces::srv::UpdateATRPathList>::SharedPtr;

  using ATRFormationListShPt = atr_interfaces::msg::ATRFormationList::SharedPtr;
  using ATRStateListShPt = atr_interfaces::msg::ATRStateListStamped::SharedPtr;
  using ObjectListShPt = atr_interfaces::msg::ObjectList::SharedPtr;
  using PredObjEllipseListShPt = atr_interfaces::msg::PredictedObjectEllipseList::SharedPtr;
  using ATRPathListShPt = atr_interfaces::msg::ATRPathList::SharedPtr;
  using ReqUpdateATRPathList = atr_interfaces::srv::UpdateATRPathList::Request::SharedPtr;
  using SrvRespFutureUpdateATRPathList = rclcpp::Client<atr_interfaces::srv::UpdateATRPathList>::SharedFuture;

private:
  rclcpp::TimerBase::SharedPtr timer_;            ///< Timer to trigger the publisher
  ClientGetATRFormation formation_list_client_;   ///< client to request the ATR Formation list
  SubsATRStateListStamped atr_list_subs_;         ///< subscriber to the ATRStateList topic
  SubsObjectList obj_list_subs_;                  ///< subscriber for the dyn and static object list
  ClientGetObjectList nona_list_client_;          ///< client to request the ATR Formation list
  SrvUpdatePredObjList service_;                  ///< service to update the internal predicted object list
  PubATRPathList atr_path_list_pub_;              ///< publisher for the ATR Path list
  ClientUpdateATRPathList atr_path_list_client_;  ///< client to update the ATR Path List

  std::vector<std::string> v_data_names_;      ///< vector of data names to identify the different input signals
  std::unordered_map<int, int> map_id_index_;  ///< Map to connect Object ID to index in the local memory map[id] =
                                               ///< index)
  std::map<std::string, int> map_data_index_;  ///< Map to connect received data type with index (vector of flags and
                                               ///< mutex)
  std::vector<bool> v_data_flags;              ///< flags to control when the data is available

  std::string atr_formation_service_name_;  ///< Name of the service that provides the ATR Formation List
  std::string nona_service_name_;           ///< Name of the service that provides the NONA Object List

  std::string obj_topic_name_;         ///< Object list topic name for the subscriber
  std::string atr_topic_name_;         ///< ATR list topic name for the subscriber
  std::string atr_path_topic_name_;    ///< ATR Path list topic name for the publisher
  std::string service_name_;           ///< Service name to update the local predicted object list
  int atr_period_ms_;                  ///< Sample time for the publisher
  std::string frame_id_;               ///< reference frame for the paths
  std::string atr_path_service_name_;  ///< name for the service to update the ATR Path List

  ATRFormationListShPt atr_formation_list_;  ///< shared variable to allocate the obj list
  ATRStateListShPt atr_state_list_;          ///< shared variable to allocate the obj list
  ObjectListShPt obj_list_;                  ///< shared variable to allocate the obj list
  ObjectListShPt nona_list_;                 ///< shared variable to allocate the NONA obj list
  PredObjEllipseListShPt pred_obj_list_;     ///< shared variable to allocate the pred obj list
  ATRPathListShPt atr_path_list_;            ///< shared variable to allocate the atr path list (publish var)

  std::mutex data_mutex_;      ///< mutex to protect write/read access between subscribers/server and main process
  std::mutex atr_path_mutex_;  ///< mutex to protect write/read access between publisher and client

public:
  /**
   * @brief standard constructor
   *
   */
  ATRTrajectoryGenerator(/* args */);
  ~ATRTrajectoryGenerator();

  /**
   * @brief Initialize the object
   *          It load the parameters from a yaml file
   */
  void init();

private:
  /**
   * @brief callback function for the Obj List subscriber
   *
   * @param msg Object list
   */
  void obj_subs_callback(const atr_interfaces::msg::ObjectList::SharedPtr msg);

  /**
   * @brief callback function for the ATRState List subscriber
   *
   * @param msg ATRState list
   */
  void atr_subs_callback(const atr_interfaces::msg::ATRStateListStamped::SharedPtr msg);

  /**
   * @brief update predicted object list callback function
   *
   * @param request new predicted object list
   * @param response success flag with error message describing the failure
   */
  void updatePredObjectListCB(const std::shared_ptr<atr_interfaces::srv::UpdatePredictedObjectList::Request> request,
                              std::shared_ptr<atr_interfaces::srv::UpdatePredictedObjectList::Response> response);

  /**
   * @brief callback function triggered by the timer. It controls the publisher
   *
   */
  void timer_callback();

  /**
   * @brief Get the ATR Formation list. Requests the Formation list from the server
   *
   * @return true the formation list was received
   * @return false error
   */
  bool getFormation();

  /**
   * @brief Get the NONA object list. Requests the NONA list from the server
   *
   * @return true list of NONA objects received
   * @return false error
   */
  bool getNONA();

  /**
   * @brief Waits until the Formation List and the NONA list are received
   *
   * @return true received both lists
   * @return false error
   */
  bool getStaticData();
};
}  // namespace atr_examples

#endif