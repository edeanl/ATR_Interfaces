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

#include <atr_examples/ATRTrajectoryGenerator.h>

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <atr_interfaces/convert_extended.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace atr_examples
{
ATRTrajectoryGenerator::ATRTrajectoryGenerator() : Node("object_list_subscriber")
{
  // Init parameters
  init();

  // Shared objects
  atr_formation_list_ = std::make_shared<atr_interfaces::msg::ATRFormationList>();
  atr_state_list_ = std::make_shared<atr_interfaces::msg::ATRStateListStamped>();
  obj_list_ = std::make_shared<atr_interfaces::msg::ObjectList>();
  nona_list_ = std::make_shared<atr_interfaces::msg::ObjectList>();
  pred_obj_list_ = std::make_shared<atr_interfaces::msg::PredictedObjectEllipseList>();
  atr_path_list_ = std::make_shared<atr_interfaces::msg::ATRPathList>();

  // Create Map and initialize data flags
  v_data_names_ = { "formation", "atr_state", "objects", "nona", "pred_obj" };

  int idx = 0;
  for (auto&& nam : v_data_names_)
  {
    map_data_index_[nam] = idx;
    v_data_flags.push_back(false);
    idx++;
  }

  // Formation List client (Static Info)
  formation_list_client_ = this->create_client<atr_interfaces::srv::GetATRFormation>(atr_formation_service_name_);

  // Subscription to ATR list topic
  atr_list_subs_ = create_subscription<atr_interfaces::msg::ATRStateListStamped>(
      atr_topic_name_, 10, std::bind(&ATRTrajectoryGenerator::atr_subs_callback, this, _1));

  // Subscription to Object list topic
  obj_list_subs_ = create_subscription<atr_interfaces::msg::ObjectList>(
      obj_topic_name_, 10, std::bind(&ATRTrajectoryGenerator::obj_subs_callback, this, _1));

  // NONA List client (Static Info)
  nona_list_client_ = this->create_client<atr_interfaces::srv::GetObjectList>(nona_service_name_);

  // Update Local Predicted Object List
  service_ = create_service<atr_interfaces::srv::UpdatePredictedObjectList>(
      service_name_, std::bind(&ATRTrajectoryGenerator::updatePredObjectListCB, this, _1, _2));

  // Call the services with static data, e.g. ATR Formation and NONA Objects
  if (!getStaticData())
  {
    RCLCPP_ERROR(get_logger(), "Error getting static data");
  }

  // ATR Path List publisher
  atr_path_list_pub_ = this->create_publisher<atr_interfaces::msg::ATRPathList>(atr_path_topic_name_, 10);

  // UpdateATRPath client To send the new ATR Path List as a request to the service
  // provided by ATR Fleet Control
  atr_path_list_client_ = this->create_client<atr_interfaces::srv::UpdateATRPathList>(atr_path_service_name_);

  // This timer triggers the publisher of the ObjectList message
  timer_ = this->create_wall_timer(std::chrono::milliseconds(atr_period_ms_),
                                   std::bind(&ATRTrajectoryGenerator::timer_callback, this));

  RCLCPP_INFO(get_logger(), "Ready to generate trajectories!");
}

ATRTrajectoryGenerator::~ATRTrajectoryGenerator()
{
}

void ATRTrajectoryGenerator::init()
{
  std::vector<std::string> param_names = { "formation_list_service_name",
                                           "atr_list_topic_name",
                                           "object_list_topic_name",
                                           "nona_list_service_name",
                                           "predicted_obj_list_service_name",
                                           "atr_path_topic_name",
                                           "period_ms",
                                           "frame_id",
                                           "path_list_service_name" };
  for (auto&& i : param_names)
    declare_parameter(i);
  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  atr_formation_service_name_ = params.at(0).as_string();
  atr_topic_name_ = params.at(1).as_string();
  obj_topic_name_ = params.at(2).as_string();
  nona_service_name_ = params.at(3).as_string();
  service_name_ = params.at(4).as_string();
  atr_path_topic_name_ = params.at(5).as_string();
  atr_period_ms_ = params.at(6).as_int();
  frame_id_ = params.at(7).as_string();
  atr_path_service_name_ = params.at(8).as_string();
}

void ATRTrajectoryGenerator::atr_subs_callback(const atr_interfaces::msg::ATRStateListStamped::SharedPtr msg)
{
  size_t msg_size = msg->list.atr_states.size();

  if (msg_size > 0)
  {
    // TODO: verify that full_state = true or pose_source = FUSED_ODOM before making the local copy
    // This is to verify that the trajectory generator uses the fused_odom to compute the paths
    data_mutex_.lock();
    atr_state_list_->list = msg->list;
    data_mutex_.unlock();

    // Set the ATR data flag to true
    v_data_flags[map_data_index_["atr_state"]] = true;
  }
  else
  {
    std::string error_message = "The ATR List is empty [" + std::to_string(msg_size) + "]";
    RCLCPP_ERROR_STREAM(get_logger(), error_message);
  }
}

void ATRTrajectoryGenerator::obj_subs_callback(const atr_interfaces::msg::ObjectList::SharedPtr msg)
{
  size_t msg_size = msg->objects.size();

  if (msg_size > 0)
  {
    data_mutex_.lock();
    obj_list_->objects = msg->objects;
    data_mutex_.unlock();

    // Set the Object List data flag to true
    v_data_flags[map_data_index_["objects"]] = true;
  }
  else
  {
    std::string error_message = "The Object List is empty [" + std::to_string(msg_size) + "]";
    RCLCPP_ERROR_STREAM(get_logger(), error_message);
  }
}

void ATRTrajectoryGenerator::updatePredObjectListCB(
    const std::shared_ptr<atr_interfaces::srv::UpdatePredictedObjectList::Request> request,
    std::shared_ptr<atr_interfaces::srv::UpdatePredictedObjectList::Response> response)
{
  size_t data_size = request->p_list.p_object_ellipses.size();
  if (data_size > 0)
  {
    data_mutex_.lock();
    pred_obj_list_->p_object_ellipses = request->p_list.p_object_ellipses;
    data_mutex_.unlock();
    response->success = true;

    // Set the Predicted Object List data flag to true
    v_data_flags[map_data_index_["pred_obj"]] = true;
  }
  else
  {
    std::string error_message = "The Predicted Object List is empty [" + std::to_string(data_size) + "]";
    RCLCPP_ERROR_STREAM(get_logger(), error_message);
    response->success = false;
    response->error.id = atr_interfaces::msg::ATRError::DATA_NOT_AVAILABLE;
    response->error.message = error_message;
  }
}

void ATRTrajectoryGenerator::timer_callback()
{
  bool data_ready = true;

  // Verify that all the needed data is available at least once
  int idx = 0;
  for (auto&& flag : v_data_flags)
  {
    if (!flag)
    {
      data_ready = false;
      RCLCPP_WARN_STREAM(get_logger(), "(" << idx << ")Data [" << v_data_names_[idx] << "] missing, waiting ...");
      // is enough if one data type is missing
      break;
    }
    idx++;
  }

  if (data_ready)
  {
    atr_interfaces::msg::ATRPathList atr_path_msg;

    auto local_atr_formation_list = std::make_shared<atr_interfaces::msg::ATRFormationList>();
    auto local_atr_state_list = std::make_shared<atr_interfaces::msg::ATRStateListStamped>();
    auto local_obj_list = std::make_shared<atr_interfaces::msg::ObjectList>();
    auto local_nona_list = std::make_shared<atr_interfaces::msg::ObjectList>();
    auto local_pred_obj_list = std::make_shared<atr_interfaces::msg::PredictedObjectEllipseList>();

    // Collect Data
    // TODO: evaluate whether we need multiple mutex or not
    // "formation", "atr_state", "objects", "nona", "pred_obj"
    data_mutex_.lock();
    local_atr_formation_list = atr_formation_list_;
    local_atr_state_list = atr_state_list_;
    local_obj_list = obj_list_;
    local_nona_list = nona_list_;
    local_pred_obj_list = pred_obj_list_;
    data_mutex_.unlock();

    int atr_number = local_atr_state_list->list.atr_states.size();

    // Populate the PathList
    // auto atr_path_list_msg = std::make_shared<atr_interfaces::msg::ATRPathList>();
    atr_interfaces::msg::ATRPathList atr_path_list_msg;

    // The number of pats should be the number of ATRs
    // atr_path_list_msg->paths.resize(atr_number);
    atr_path_list_msg.paths.resize(atr_number);
    rclcpp::Time aux_time = now();

    // Defining 10 poses for now
    int poses_size = 14;

    // double inc = 3.0 / static_cast<double>(poses_size);

    double theta = DEG2RAD(45.0);

    std::vector<double> v_init_x = { 1.5, 1.0 };
    std::vector<double> v_init_y = { 0.8, -0.5 };

    Eigen::MatrixXd Points = Eigen::MatrixXd::Zero(4, poses_size);

    // Manually defined Paths
    // Rows 1-2 ATR 1 coordinates x and y
    // Rows 3-4 ATR 2 coordinates x and y
    Points.row(0) << 1.5, 1.2, 0.9, 0.6, 0.3, 0, -0.3, -0.6, -0.9, -1.2, -1.4, -1.6, -1.8, -2.0;
    Points.row(1) << 0.8, 0.8, 0.8, 0.8, 1.0, 1.0, 1.0, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8;
    Points.row(2) << 1, 0.7, 0.4, 0.1, -0.2, -0.5, -0.8, -1.1, -1.4, -1.7, -1.7, -1.7, -1.7, -1.7;
    Points.row(3) << -0.5, -0.5, -1.0, -1.0, -1.0, -0.5, -0.5, -0.5, -0.5, -0.5, -0.7, -0.9, -1.1, -1.3;

    tf2::Transform tf;
    geometry_msgs::msg::Transform ts;
    tf2::Quaternion q, q_z_pi2, qt;
    q.setRPY(0, 0, theta);  // Basic Rotation in z
    q.normalize();
    q_z_pi2.setRPY(0, 0, M_PI_2);
    q_z_pi2.normalize();

    // To correct the position and the mesh orientation
    qt = (q * q_z_pi2).normalize();

    // Pose 1
    int idx = 0;
    for (auto&& paths : atr_path_list_msg.paths)
    {
      paths.header.frame_id = frame_id_;
      paths.header.stamp = aux_time;
      paths.atr_path.atr_id = local_atr_state_list->list.atr_states[idx].atr_id;
      // TODO: define priority 1=max??
      paths.atr_path.priority = 1;

      paths.atr_path.path_w_time.poses.resize(poses_size);

      // Set dummy poses
      // double step = 0;
      int count = 0;
      for (auto&& poses : paths.atr_path.path_w_time.poses)
      {
        poses.delta_time = 0.1;  // in [sec]

        // tf.setOrigin(tf2::Vector3(v_init_x[idx] - (step * inc), v_init_y[idx], 0));
        tf.setOrigin(tf2::Vector3(Points(2 * idx, count), Points(2 * idx + 1, count), 0));
        count++;
        // step += 1.0;
        // For now fixed orientation
        // TODO: calculate the correct orientation
        tf.setRotation(qt);
        ts = tf2::toMsg(tf);
        // Setting the pose fused_odom using the ts information
        tf2::convert(ts, poses.pose);
      }
      // Increase the idx to get the next ATR info
      idx += 1;
    }

    // Copy data to share it with the client
    atr_path_mutex_.lock();
    atr_path_list_->paths = atr_path_list_msg.paths;
    atr_path_mutex_.unlock();

    // Publish data
    atr_path_list_pub_->publish(*atr_path_list_.get());

    // Send the new ATR Path list to the server
    if (atr_path_list_client_->service_is_ready())
    {
      ReqUpdateATRPathList request = std::make_shared<atr_interfaces::srv::UpdateATRPathList::Request>();
      // Copy the new atr path list as a request
      request->list = atr_path_list_msg;
      // Call the service (update the atr path list in the server)
      // We use the non blocking version
      auto future_result = atr_path_list_client_->async_send_request(
          request, std::bind(&ATRTrajectoryGenerator::wait_for_srv_response<SrvRespFutureUpdateATRPathList>, this, _1));
    }
    else
    {
      RCLCPP_WARN_STREAM(get_logger(), "Service: " + atr_path_service_name_ + " not ready yet, skipping data");
    }

  }  // if data ready
}

bool ATRTrajectoryGenerator::getFormation()
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
      v_data_flags[map_data_index_["formation"]] = true;
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

bool ATRTrajectoryGenerator::getNONA()
{
  // Get the NONA Object list
  rclcpp::Rate loop_rate(2);
  while (!nona_list_client_->service_is_ready())
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service NONA Object List. Exiting.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "service  NONA Object List not available, waiting again...");
    loop_rate.sleep();
  }

  auto nona_req = std::make_shared<atr_interfaces::srv::GetObjectList::Request>();

  // Call the Formation List service
  auto result = nona_list_client_->async_send_request(nona_req);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      nona_list_->objects = result.get()->list.objects;

      // Set the NONA Object List data flag to true
      v_data_flags[map_data_index_["nona"]] = true;
      return true;
    }
    else
    {
      RCLCPP_WARN_STREAM(get_logger(),
                         "Error[" << static_cast<int>(result.get()->error.id)
                                  << "]: getting nona object list: " << result.get()->error.message);
    }
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Failed to call service NONA Object List");
  }

  return false;
}

bool ATRTrajectoryGenerator::getStaticData()
{
  rclcpp::Rate loop_rate(2);

  // Wait until we get the nona objects
  while (!getNONA())
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the NONA Object List. Exiting.");
    }

    RCLCPP_INFO(get_logger(), "NONA Object List not available, waiting ...");
    loop_rate.sleep();
  }

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

}  // namespace atr_examples
