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

#ifndef ATR_FLEET_CONTROL_H
#define ATR_FLEET_CONTROL_H

/*! \file ATRFleetControl.h
 *  \brief Provides simulated trajectories for the ATRs.
 *
 *  Provides the following functionalities:
 *      - vector of ATR Paths to send target Path to each ATR
 *      - Subscribers
 *          ATRState List (to get the current ATRs pose)
 *      - Clients
 *          ATRFormation
 *      - Services
 *          Update ATRPath List (to get the ATR Paths from ATRTrajectoryGenerator)
 *      - Publishers
 *          ATRPathList, publishes all the target ATR Paths as a List (to visualize them in rviz)
 */

#include <memory>

#include "atr_examples/AuxTools.h"
#include "atr_interfaces/msg/object_list.hpp"
#include "atr_interfaces/msg/atr_state_list_stamped.hpp"
#include "atr_interfaces/msg/atr_path_list.hpp"
#include "atr_interfaces/msg/path_with_d_time.hpp"
#include "atr_interfaces/srv/get_atr_formation.hpp"
// List of Paths
#include "atr_interfaces/srv/update_atr_path_list.hpp"
// ATR paths
#include "atr_interfaces/srv/update_atr_path.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "atr_interfaces/srv/trigger_atr_signals.hpp"
#include "atr_interfaces/srv/trigger_atr_actuator.hpp"

using std::placeholders::_1;

namespace atr_examples
{
/** \class ATRFleetControl
 *
 * \brief Provides the ros interfaces to simulate the Fleet Control node
 */
class ATRFleetControl : public rclcpp::Node, public AuxTools
{
public:
  using ClientGetATRFormation = rclcpp::Client<atr_interfaces::srv::GetATRFormation>::SharedPtr;
  using SubsATRStateListStamped = rclcpp::Subscription<atr_interfaces::msg::ATRStateListStamped>::SharedPtr;
  using SrvUpdateATRPathList = rclcpp::Service<atr_interfaces::srv::UpdateATRPathList>::SharedPtr;
  using ClientUpdateATRPath = rclcpp::Client<atr_interfaces::srv::UpdateATRPath>::SharedPtr;
  using PubATRPathList = rclcpp::Publisher<atr_interfaces::msg::ATRPathList>::SharedPtr;
  using v_PoseWDTime =
      std::vector<atr_interfaces::msg::PoseWithDTime, std::allocator<atr_interfaces::msg::PoseWithDTime>>;

  // This will be replaced by the service
  using SubsATRPathList = rclcpp::Subscription<atr_interfaces::msg::ATRPathList>::SharedPtr;

  // We need to create multiple clients to send the target ATRPath to each ATR.
  using SrvRespFutureUpdateATRPath = rclcpp::Client<atr_interfaces::srv::UpdateATRPath>::SharedFuture;
  using v_ClientUpdateATRPath = std::vector<ClientUpdateATRPath, std::allocator<ClientUpdateATRPath>>;

  // shared objects definitions
  using ATRFormationListShPt = atr_interfaces::msg::ATRFormationList::SharedPtr;
  using ATRStateListShPt = atr_interfaces::msg::ATRStateListStamped::SharedPtr;
  using ATRPathListShPt = atr_interfaces::msg::ATRPathList::SharedPtr;
  using ATRPathWDtimeShPt = atr_interfaces::msg::PathWithDTime::SharedPtr;
  using v_ATRPathWDtimeShPt = std::vector<ATRPathWDtimeShPt, std::allocator<ATRPathWDtimeShPt>>;
  using v_ATRPathStamped =
      std::vector<atr_interfaces::msg::ATRPathStamped, std::allocator<atr_interfaces::msg::ATRPathStamped>>;

private:
  rclcpp::TimerBase::SharedPtr timer_;           ///< Timer to trigger the publisher (main process/thread)
  ClientGetATRFormation formation_list_client_;  ///< client to request the ATR Formation list
  SubsATRStateListStamped atr_list_subs_;        ///< subscriber to the ATRStateList topic
  SrvUpdateATRPathList atr_path_list_service_;   ///< service to receive the new ATRPath List

  PubATRPathList atr_path_list_pub_;  ///< publisher for the ATR Path list

  v_ClientUpdateATRPath v_atr_path_clients_;  ///< Vector of ATR Path Clients, each ATRPath from the ATRPathList will be
                                              ///< sent to the corresponding ATR

  std::unordered_map<int, int> map_id_index_;  ///< Map to connect Object ID to index in the local memory map[id] =
                                               ///< index)

  bool atr_state_list_flag;  ///< controls when the ATRState list is available

  std::string atr_formation_service_name_;  ///< Name of the service that provides the ATR Formation List
  std::string atr_topic_name_;              ///< ATR list topic name for the subscriber
  std::string atr_path_topic_name_;         ///< ATR Path list topic name for the subscriber
  std::string service_name_;                ///< Service name to update the Path in the ATRs
  int period_ms_;                           ///< Sample time for the main process
  std::string frame_id_;                    ///< reference frame for the paths
  std::string client_prefix_;               ///< Prefix for the ATR path clients

  ATRFormationListShPt atr_formation_list_;  ///< shared variable to allocate the atr formation list
  ATRStateListShPt atr_state_list_;          ///< shared variable to allocate the atr state list
  ATRPathListShPt atr_path_list_in_;         ///< shared variable to allocate the atr path list from ATRTrajectoryGen
  ATRPathListShPt atr_path_list_out_;        ///< shared variable to allocate the atr path list generated in this node
  ATRPathWDtimeShPt atr_path_;               ///< shared variable to allocate an atr path
  v_ATRPathWDtimeShPt v_atr_paths_;          ///< shared variable to allocate a vector of atr paths

  std::mutex atr_state_mutex_;  ///< mutex to protect write/read access between subscribers/server and main process

  std::mutex path_mutex_;  ///< mutex to protect write/read access between subscribers/server and main process

  // std::mutex atr_path_mutex_;                   ///< mutex to protect write/read access between publisher and client
  // std::vector<std::mutex> v_atr_path_mutexes_;  ///< vector of mutexes to protect write/read access between publisher
  //                                               ///< and client
  bool atr_path_list_available;  ///< defines when a path_list has been received

  Eigen::MatrixXd T_;             ///< Control variable for the BSpline function
  int bspline_samples_;           ///< number of samples for the Bspline
  Eigen::Matrix3d Mi_, Mm_, Mf_;  ///< Basic functions for the BSpline

public:
  /**
   * @brief standard constructor
   *
   */
  ATRFleetControl(/* args */);
  ~ATRFleetControl();

  /**
   * @brief Initialize the object
   *          It load the parameters from a yaml file
   */
  void init();

private:
  /**
   * @brief callback function for the ATRState List subscriber
   *
   * @param msg ATRState list
   */
  void atr_subs_callback(const atr_interfaces::msg::ATRStateListStamped::SharedPtr msg);

  /**
   * @brief Instantiates and initialize dynamically the clients to send the target paths to each ATR
   *
   * @param atr_path list of received paths
   * @return true all the clients were successfully created
   * @return false the path list was empty
   */
  bool updateClientList(v_ATRPathStamped& atr_path);

  /**
   * @brief Update the ATRPath list callback function
   *
   * @param request the new atr path list (e.g. from ATRTrajectoryGenerator)
   * @param response success false/true, and in case of error, it provides error information
   */
  void updateATRPathListCB(const std::shared_ptr<atr_interfaces::srv::UpdateATRPathList::Request> request,
                           std::shared_ptr<atr_interfaces::srv::UpdateATRPathList::Response> response);

  /**
   * @brief main function to re-calculate the ATR paths.
   *
   */
  void update_paths();

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
   * @brief Waits until the Formation List is received
   *
   * @return true received both lists
   * @return false error
   */
  bool getStaticData();

  /**
   * @brief Computes a non-uniform quadratic BSpline based on the path points
   *
   * @param paths sequence of points
   * @return Eigen::MatrixXd a matrix with the 2D positions of the BSpline
   */
  Eigen::MatrixXd compute_nuq_bspline(atr_interfaces::msg::ATRPathStamped& paths);

  /**
   * @brief Get the PosesWithTime structure based on a BSpline matrix
   *
   * @param m BSpline matrix
   * @param delta_time time to reach each 2D position
   * @return v_PoseWDTime 2D poses for the publisher
   */
  v_PoseWDTime getPosesWTime(Eigen::MatrixXd& m, float delta_time);
};
}  // namespace atr_examples

#endif