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

#include <atr_examples/ATRStateListPublisher.h>
#include <atr_interfaces/convert_extended.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

namespace atr_examples
{
ATRStateListPublisher::ATRStateListPublisher() : Node("atr_state_list_publisher"), tf_broadcaster_(this), count_(0)
{
  // Initialize object, loading parameters.

  init();

  publisher_ = this->create_publisher<atr_interfaces::msg::ATRStateListStamped>(list_topic_name_, 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(atr_period_ms_),
                                   std::bind(&ATRStateListPublisher::publisher_callback, this));

  // This timer is used to reset the full_state flag when a ATR doesn't update its state in atr_watchdog_time
  watchdog_timer_ = this->create_wall_timer(500ms, std::bind(&ATRStateListPublisher::watchdog_callback, this));

  // The frame_id doesn't change. It can be defined here
  atr_states_msg_.header.frame_id = list_frame_id_;
  // We created a list for atr_number_ ATRs
  atr_states_msg_.list.atr_states.resize(atr_number_);
  // Also, we need to create the shared_data object to allocate the information from the different ATRState subscribers.
  // This vector should be populated using the maps atr_id->index.
  // It is important to allocate this vector before the initialization of the ATRState subscribers
  v_atr_states_.resize(atr_number_);

  // We create a mutex for each ATRState topic.
  std::vector<std::mutex> aux_list_mutex(atr_number_);

  v_atr_states_mutex_.swap(aux_list_mutex);

  // Create a vector of clock, and initialize them
  v_tc_.resize(atr_number_);
  for (auto&& i : v_tc_)
  {
    i = now();
  }

  // Create ATRState subscribers
  ATRStateSubscriber aux_subscriber;

  for (size_t i = 0; i < atr_number_; i++)
  {
    // define the name of the topic based on the ATR id
    std::string atr_topic_name;
    atr_topic_name = "state_atr_" + std::to_string(atr_ids_.at(i));

    // Add the ATR id to the map id->index
    map_id_index_[atr_ids_.at(i)] = i;
    // and to the inverse map index->id
    map_index_id_[i] = atr_ids_.at(i);

    aux_subscriber = create_subscription<atr_interfaces::msg::ATRStateStamped>(
        atr_topic_name, 10, std::bind(&ATRStateListPublisher::topic_callback, this, _1));

    v_subscribers_.push_back(std::move(aux_subscriber));
  }
}

void ATRStateListPublisher::init()
{
  declare_parameter("topic_name");
  declare_parameter("frame_id");
  declare_parameter("atr_ids");
  declare_parameter("watchdog_time");
  declare_parameter("period_ms");

  // Get parameters
  list_topic_name_ = get_parameter("topic_name").as_string();
  list_frame_id_ = get_parameter("frame_id").as_string();
  atr_ids_ = get_parameter("atr_ids").as_integer_array();
  atr_number_ = atr_ids_.size();
  atr_watchdog_time = get_parameter("watchdog_time").as_double();
  atr_period_ms_ = get_parameter("period_ms").as_int();
}

void ATRStateListPublisher::publisher_callback()
{
  atr_states_msg_.header.stamp = now();

  for (size_t i = 0; i < atr_states_msg_.list.atr_states.size(); i++)
  {
    int64_t t_id = atr_ids_.at(i);
    int64_t t_idx = map_id_index_[t_id];
    atr_states_msg_.list.atr_states[i].atr_id = t_id;
    atr_states_msg_.list.atr_states[i].full_state = false;
    atr_states_msg_.list.atr_states[i].pose_source = atr_interfaces::msg::ATRState::OPTOM;

    // Generate a OPTOM pose
    double step = 0.01;
    double theta = count_ * step;
    count_ += 1;

    tf2::Transform tf;
    tf2::Quaternion q;
    geometry_msgs::msg::TransformStamped ts;

    double atr_radius = 1.0;
    int atr_period_ms = 33;

    q.setRPY(0, 0, theta);  // Basic Rotation in z
    q.normalize();

    tf2::Quaternion q_z_pi2;
    q_z_pi2.setRPY(0.0, 0.0, M_PI_2);
    q_z_pi2.normalize();

    tf.setOrigin(tf2::Matrix3x3(q) * tf2::Vector3(atr_radius, 0, 0));
    tf.setRotation((q * q_z_pi2).normalize());

    ts.transform = tf2::toMsg(tf);

    // Define the optom pose using Ts info
    tf2::convert(ts.transform, atr_states_msg_.list.atr_states[i].pose.optom);

    // Time derivative of the optom pose
    atr_states_msg_.list.atr_states[i].vel.optom.linear.x = -atr_radius * sin(theta);
    atr_states_msg_.list.atr_states[i].vel.optom.linear.y = atr_radius * cos(theta);
    atr_states_msg_.list.atr_states[i].vel.optom.linear.z = 0;
    atr_states_msg_.list.atr_states[i].vel.optom.angular.x = 0;
    atr_states_msg_.list.atr_states[i].vel.optom.angular.y = 0;
    atr_states_msg_.list.atr_states[i].vel.optom.angular.z = step / (0.001 * atr_period_ms);

    // Getting ODOM pose
    // If the ATR has already published data, we use its info to populate the atr_state_list (odom+fused_odom)
    // We have to guard again the shared object during the writing/reading process
    // std::lock_guard<std::mutex> guard(v_atr_states_mutex_[t_idx]);

    v_atr_states_mutex_[t_idx].lock();
    if (v_atr_states_.at(t_idx).full_state)
    {
      atr_states_msg_.list.atr_states[i].pose.fused_odom = v_atr_states_.at(t_idx).pose.fused_odom;
      atr_states_msg_.list.atr_states[i].pose.odom = v_atr_states_.at(t_idx).pose.odom;
      atr_states_msg_.list.atr_states[i].pose_source = atr_interfaces::msg::ATRState::FULL_POSE;
      atr_states_msg_.list.atr_states[i].full_state = v_atr_states_.at(t_idx).full_state;
    }
    v_atr_states_mutex_[t_idx].unlock();

    // Generating GOAL pose
    q.setRPY(0, 0, M_PI);

    // Just for debugging, we use the atr_id as goal
    double goal = static_cast<double>(t_id);
    tf.setOrigin(tf2::Vector3(goal, goal, 0));
    tf.setRotation(q);
    ts.transform = tf2::toMsg(tf);
    tf2::convert(tf2::toMsg(tf), atr_states_msg_.list.atr_states[i].goal);
  }
  publisher_->publish(atr_states_msg_);
}

void ATRStateListPublisher::topic_callback(const atr_interfaces::msg::ATRStateStamped::SharedPtr msg)
{
  int64_t t_idx = map_id_index_[msg->state.atr_id];
  // RCLCPP_INFO_STREAM(get_logger(), " Received: ATR[" << t_id << "]");

  v_tc_.at(t_idx) = now();
  // For debugging purposes
  // geometry_msgs::msg::Pose atr_p;
  // atr_p = msg->state.pose.fused_odom;
  // RCLCPP_INFO_STREAM(get_logger(),
  //                    " position: (" << atr_p.position.x << "," << atr_p.position.y << "," << atr_p.position.z <<
  //                    ")");

  // We protect the read/write of the atr_data. Everything after the mutex should be simple copy/assign functions. This
  // mutex will block the atr_state_list publisher
  std::lock_guard<std::mutex> guard(v_atr_states_mutex_[t_idx]);

  // v_atr_states_mutex_[t_idx].lock();

  // We need to get the pose from the message, use the atr_id in the message and append the pose/vel odom + fused_odom
  // to the shared_data (v_atr_states_). This will be copied into the general message with all the atrs info (list of
  // atrs). We need to create a vector of atrs_states with the same size of the atr_number and use the maps to put the
  // data in the correct index of this vector.
  v_atr_states_[t_idx] = msg->state;

  // Once we have received the state information from the ATRS (odom + fused_odom), we set the full_state flag to true.
  // We can do it here because when this cb_function is called, then we will have both the optom info (generated by this
  // node) and the odom info. If the optom info comes from another node, we will follow the same procedure, once both
  // arrays of atr states have published data, then we set the fll_state flag to true
  v_atr_states_[t_idx].full_state = true;
  // v_atr_states_mutex_[t_idx].unlock();

  // If full_state=false, then we should not try to copy the info of the atr state to the atr list msg, because is not
  // properly populated
}

void ATRStateListPublisher::watchdog_callback()
{
  rclcpp::Time tc = now();

  for (size_t i = 0; i < atr_number_; i++)
  {
    double elapsed_time = (tc - v_tc_.at(i)).nanoseconds() * 1E-9;

    // If the ATR has not updated its state, then reset the full_state flag
    if (elapsed_time > atr_watchdog_time)
    {
      int64_t t_idx = map_id_index_[atr_ids_.at(i)];
      v_atr_states_[t_idx].full_state = false;
    }
  }
}
}  // namespace atr_examples