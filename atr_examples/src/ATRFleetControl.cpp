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

#include <atr_examples/ATRFleetControl.h>

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <atr_interfaces/convert_extended.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace atr_examples
{
ATRFleetControl::ATRFleetControl() : Node("atr_fleet_control"), atr_path_list_available(false)
{
  // Init parameters

  init();

  // Shared objects
  atr_formation_list_ = std::make_shared<atr_interfaces::msg::ATRFormationList>();
  atr_path_list_in_ = std::make_shared<atr_interfaces::msg::ATRPathList>();
  atr_path_list_out_ = std::make_shared<atr_interfaces::msg::ATRPathList>();
  atr_path_ = std::make_shared<atr_interfaces::msg::PathWithDTime>();
  atr_state_list_ = std::make_shared<atr_interfaces::msg::ATRStateListStamped>();

  // Formation List client (Static Info)
  formation_list_client_ = this->create_client<atr_interfaces::srv::GetATRFormation>(atr_formation_service_name_);

  // Subscription to ATR list topic
  atr_list_subs_ = create_subscription<atr_interfaces::msg::ATRStateListStamped>(
      atr_topic_name_, 10, std::bind(&ATRFleetControl::atr_subs_callback, this, _1));

  // Service to receive the new ATRPath List
  atr_path_list_service_ = create_service<atr_interfaces::srv::UpdateATRPathList>(
      service_name_, std::bind(&ATRFleetControl::updateATRPathListCB, this, _1, _2));

  // ATR Path List publisher
  atr_path_list_pub_ = this->create_publisher<atr_interfaces::msg::ATRPathList>(atr_path_topic_name_, 10);

  if (!getStaticData())
  {
    RCLCPP_ERROR(get_logger(), "Error getting static data");
  }

  // This timer triggers the publisher of the ObjectList message
  // timer_ =
  //     this->create_wall_timer(std::chrono::milliseconds(period_ms_), std::bind(&ATRFleetControl::timer_callback,
  //     this));

  RCLCPP_INFO(get_logger(), "Ready to generate Fleet trajectories!");
}

ATRFleetControl::~ATRFleetControl()
{
}

void ATRFleetControl::init()
{
  // Get paramters from a list
  std::vector<std::string> param_names = { "atr_path_topic_name",
                                           "formation_list_service_name",
                                           "atr_list_topic_name",
                                           "path_list_service_name",
                                           "period_ms",
                                           "frame_id",
                                           "atr_path_client_prefix",
                                           "bspline_samples" };
  for (auto&& i : param_names)
    declare_parameter(i);

  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  atr_path_topic_name_ = params.at(0).as_string();
  atr_formation_service_name_ = params.at(1).as_string();
  atr_topic_name_ = params.at(2).as_string();
  service_name_ = params.at(3).as_string();
  period_ms_ = params.at(4).as_int();
  frame_id_ = params.at(5).as_string();
  client_prefix_ = params.at(6).as_string();
  bspline_samples_ = params.at(7).as_int();

  // BSpline
  T_ = Eigen::MatrixXd::Zero(3, bspline_samples_ + 1);

  Mi_ << 2, -4, 2, -3, 4, 0, 1, 0, 0;
  Mi_ = Mi_ * 0.5;

  Mm_ << 1, -2, 1, -2, 2, 1, 1, 0, 0;
  Mm_ = Mm_ * 0.5;

  Mf_ << 1, -2, 1, -3, 2, 1, 2, 0, 0;
  Mf_ = Mf_ * 0.5;

  Eigen::VectorXd t, t2;
  t.resize(bspline_samples_ + 1);
  auto begin = t.data();
  auto end = t.data() + t.size();

  t2.resize(bspline_samples_ + 1);

  int count = 0;
  for (auto i = begin; i != end; ++i)
  {
    *i = count / (float)bspline_samples_;
    t2(count) = *i * (*i);
    count++;
  }

  T_.row(0) << t2.transpose();
  T_.row(1) << t.transpose();
  T_.row(2) << (Eigen::VectorXd::Ones(bspline_samples_ + 1));

  // RCLCPP_INFO_STREAM(get_logger(), " t vector: " << t.transpose());
  // RCLCPP_INFO_STREAM(get_logger(), " t2 vector: " << t2.transpose());
  // RCLCPP_INFO_STREAM(get_logger(), " T: \n" << T_);
  // RCLCPP_INFO_STREAM(get_logger(), " Mi: \n" << Mi_);
  // RCLCPP_INFO_STREAM(get_logger(), " Mm: \n" << Mm_);
  // RCLCPP_INFO_STREAM(get_logger(), " Mf: \n" << Mf_);
}

void ATRFleetControl::atr_subs_callback(const atr_interfaces::msg::ATRStateListStamped::SharedPtr msg)
{
  size_t msg_size = msg->list.atr_states.size();

  if (msg_size > 0)
  {
    // TODO: verify that full_state = true or pose_source = FUSED_ODOM before making the local copy
    // This is to verify that the trajectory generator uses the fused_odom to compute the paths
    atr_state_mutex_.lock();
    atr_state_list_->list = msg->list;
    atr_state_mutex_.unlock();

    // Set the ATR data flag to true
    atr_state_list_flag = true;
  }
  else
  {
    std::string error_message = "The ATR List is empty [" + std::to_string(msg_size) + "]";
    RCLCPP_ERROR_STREAM(get_logger(), error_message);
  }
}

bool ATRFleetControl::updateClientList(v_ATRPathStamped& v_atr_paths)
{
  // Get the number of paths
  size_t paths_size = v_atr_paths.size();

  bool new_data = false;

  if (paths_size > 0)
  {
    // Copy to the shared object
    // This mutex is only needed with the timer
    // path_mutex_.lock();
    atr_path_list_in_->paths = v_atr_paths;
    // path_mutex_.unlock();

    // if there are new paths, we need to register them and create a client for each of them
    // TODO: change this, even if the new path list is the same size as the vector of clients, we need to see if
    // there are new paths. The path list can have the same size with completely new atr paths
    if (v_atr_path_clients_.size() != paths_size)
    {
      // find the new path
      for (auto&& i : v_atr_paths)
      {
        // Get the ATR ID
        int16_t atr_id = i.atr_path.atr_id;
        // Check if this atr_id is not already registered
        std::unordered_map<int, int>::iterator it = map_id_index_.find(atr_id);

        // if the it = end then this is a new atr path, and we need to create its corresponding client
        if (it == map_id_index_.end())
        {
          std::string s;
          s = client_prefix_ + std::to_string(atr_id);
          ClientUpdateATRPath aux_client = this->create_client<atr_interfaces::srv::UpdateATRPath>(s);
          // Add the new client to the vector of clients
          v_atr_path_clients_.push_back(std::move(aux_client));

          // Add its corresponding shared object. PathWithDTime is the msg used to command the path to the ATR
          v_atr_paths_.push_back(std::make_shared<atr_interfaces::msg::PathWithDTime>());

          // populate map id -> index
          // we need to register the atr_id in the map. In this way, a new path for an already register atr_id will use
          // the same created client. New atr_ids are appended to the end of the list
          map_id_index_[atr_id] = v_atr_path_clients_.size() - 1;

          RCLCPP_WARN_STREAM(get_logger(),
                             "Created new client : " << s << " for ATR[" << atr_id << "], client("
                                                     << map_id_index_[atr_id] << ")");

        }  // if id not registered

      }  // for n paths

    }  // if new paths
    // RCLCPP_WARN_STREAM(get_logger(), "Got new atr path list");
    new_data = true;
  }  // if paths_size>0

  return new_data;
}

void ATRFleetControl::updateATRPathListCB(
    const std::shared_ptr<atr_interfaces::srv::UpdateATRPathList::Request> request,
    std::shared_ptr<atr_interfaces::srv::UpdateATRPathList::Response> response)
{
  if (updateClientList(request->list.paths))
  {
    // We will create two structures, one is a path list to publish the markers using the atr_path_list_subscriber node.
    // The second is the individual paths for each ATR. For this, we will split the input path list into single paths
    // one for each ATR. We will send these individual paths using the clients created with updateClientList().

    // RCLCPP_INFO_STREAM(get_logger(), "Got [" << atr_path_list_out_->paths.size() << "] atr paths ");
    atr_path_list_available = true;

    // Once a client has been created for each ATR, we will use them to send the target ATRPaths as a request for the
    // service
    update_paths();

    response->success = true;
  }
  else
  {
    response->error.message = "The ATR Path List is empty";
    RCLCPP_WARN_STREAM(get_logger(), response->error.message);
    response->success = false;
    response->error.id = atr_interfaces::msg::ATRError::DATA_NOT_AVAILABLE;
  }
}

void ATRFleetControl::update_paths()
{
  // RCLCPP_INFO(get_logger(), "Ready to process path list");

  // Get ATR states from subscriber thread
  // This info will be used to update the ATR paths
  atr_state_mutex_.lock();
  atr_interfaces::msg::ATRStateList local_atr_state_list = atr_state_list_->list;
  atr_state_mutex_.unlock();

  atr_interfaces::msg::ATRPathList local_atr_path_list;
  // path_mutex_.lock();
  // We copy the input path list to initialize the atr_ids, header, etc.
  local_atr_path_list.paths = atr_path_list_in_->paths;
  // path_mutex_.unlock();

  rclcpp::Time aux_time = now();
  int count = 0;
  float delta_time = 0.01;
  for (auto& paths : atr_path_list_in_->paths)
  {
    // Compute a non-uniform-quadratic BSpline based using the input path points as the control points
    auto m = compute_nuq_bspline(paths);
    // update the time stamp
    local_atr_path_list.paths[count].header.stamp = aux_time;
    local_atr_path_list.paths[count].header.frame_id = frame_id_;
    // define the new path
    local_atr_path_list.paths[count].atr_path.path_w_time.poses = getPosesWTime(m, delta_time);
    count++;
  }  // for paths

  // Divide the local atr path list in single atr paths
  for (auto& paths : local_atr_path_list.paths)
  {
    // get the atr_id to select the correct client
    int atr_id = paths.atr_path.atr_id;
    // get the index in the vector of clients corresponding to the atr_id
    int idx = map_id_index_[atr_id];

    // check if the ATR server is ready to receive a target Path
    if (v_atr_path_clients_[idx]->service_is_ready())
    {
      auto request = std::make_shared<atr_interfaces::srv::UpdateATRPath::Request>();
      // Copy the ATR Path from the path list to request
      v_atr_paths_[idx]->poses = paths.atr_path.path_w_time.poses;
      request->path = paths.atr_path.path_w_time;

      // Send the request to the ATR servers
      // Call the service (update the atr path list in the server)
      // We use the non blocking version
      // RCLCPP_INFO_STREAM(get_logger(), "Sending Path to ATR [" << atr_id << "], client (" << idx << ")");
      auto future_result = v_atr_path_clients_[idx]->async_send_request(
          request, std::bind(&ATRFleetControl::wait_for_srv_response<SrvRespFutureUpdateATRPath>, this, _1));
    }
    else
    {
      RCLCPP_INFO_STREAM(get_logger(), "ATR [" << atr_id << "], client (" << idx << ") not ready yet");
    }
  }

  // Publish the modified ATR Path list
  atr_path_list_pub_->publish(local_atr_path_list);
}

void ATRFleetControl::timer_callback()
{
  if (atr_path_list_available)
  {
    update_paths();
  }
  else
  {
    RCLCPP_INFO(get_logger(), "NO path list available");
  }
}

bool ATRFleetControl::getFormation()
{
  // Get the ATR Formation list
  rclcpp::Rate loop_rate(2);

  while (!formation_list_client_->service_is_ready())
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service ATR Formation List. Exiting.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "service  ATR Formation List not available, waiting again...");
    loop_rate.sleep();
  }

  auto formation_req = std::make_shared<atr_interfaces::srv::GetATRFormation::Request>();

  // Using -1 as formation id means that we request all the available formations
  formation_req->formation_ids.push_back(-1);

  // Call the Formation List service
  auto result = formation_list_client_->async_send_request(formation_req);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      atr_formation_list_->formations = result.get()->atr_formation_list.formations;
      // Set the Formation List data flag to true
      // v_data_flags[map_data_index_["formation"]] = true;
      return true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Error[" << result.get()->error.id
                                   << "]: getting formation list: " << result.get()->error.message);
    }
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Failed to call service ATR Formation List");
  }

  return false;
}

bool ATRFleetControl::getStaticData()
{
  rclcpp::Rate loop_rate(2);

  // Wait until we get the Formation list
  while (!getFormation())
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the ATR Formation List. Exiting.");
    }

    RCLCPP_INFO(get_logger(), "ATR Formation List not available, waiting ...");
    loop_rate.sleep();
  }

  return true;
}

Eigen::MatrixXd ATRFleetControl::compute_nuq_bspline(atr_interfaces::msg::ATRPathStamped& paths)
{
  v_PoseWDTime poses = paths.atr_path.path_w_time.poses;

  std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> v_B;
  Eigen::MatrixXd Bi, Bm, Bf, Bt, Btempo;

  size_t poses_size = poses.size();
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2, poses_size);

  for (size_t i = 0; i < poses_size; i++)
  {
    P.col(i) << poses[i].pose.position.x, poses[i].pose.position.y;
  }

  Btempo = P.block(0, 0, 2, 3) * Mi_ * T_;
  v_B.push_back(Btempo);

  for (size_t i = 1; i < poses_size - 3; i++)
  {
    Btempo = P.block(0, i, 2, 3) * Mm_ * T_;
    v_B.push_back(Btempo);
  }

  Btempo = P.block(0, poses_size - 3, 2, 3) * Mf_ * T_;
  v_B.push_back(Btempo);

  Bt = v_B.at(0);
  for (auto it = v_B.begin() + 1; it != v_B.end(); ++it)
  {
    Btempo = Bt;
    int n_cols = it->cols() - 1;
    Bt.conservativeResize(Eigen::NoChange, Bt.cols() + n_cols);
    Bt << Btempo, it->block(0, 1, 2, n_cols);
  }

  return Bt;
}

ATRFleetControl::v_PoseWDTime ATRFleetControl::getPosesWTime(Eigen::MatrixXd& m, float delta_time)
{
  v_PoseWDTime v_aux_poses;
  for (long int i = 0; i < m.cols(); i++)
  {
    atr_interfaces::msg::PoseWithDTime aux_pose;
    aux_pose.delta_time = delta_time;
    aux_pose.pose.position.x = m(0, i);
    aux_pose.pose.position.y = m(1, i);
    aux_pose.pose.position.z = 0;
    v_aux_poses.push_back(aux_pose);
  }

  return v_aux_poses;
}

}  // namespace atr_examples
