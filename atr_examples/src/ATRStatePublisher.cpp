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

#include <atr_examples/ATRStatePublisher.h>
#include <atr_interfaces/convert_extended.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/parameter_client.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace atr_examples
{
ATRStatePublisher::ATRStatePublisher()
  : Node("atr_state_publisher")
  , atr_name_("dummy")
  , broadcast_data_flag(true)
  , atr_path_available(false) /* , tf_broadcaster_(this) */
  , count_(0)
{
  // initialize parameters
  init();

  // Service to receive the target ATR path
  atr_path_service_ = create_service<atr_interfaces::srv::UpdateATRPath>(
      path_service_name_, std::bind(&ATRStatePublisher::updateATRPathCB, this, _1, _2));

  // Publisher for the ATR state topic
  publisher_ = this->create_publisher<atr_interfaces::msg::ATRStateStamped>(atr_topic_name_, 10);

  // main thread
  timer_ = this->create_wall_timer(std::chrono::milliseconds(atr_period_ms_),
                                   std::bind(&ATRStatePublisher::timer_callback, this));

  // broadcaster to publish the ATR tf
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Service to toggle the publisher
  broadcast_data_srv_ = create_service<std_srvs::srv::Trigger>(
      atr_service_name_, std::bind(&ATRStatePublisher::toggle_publisher_cb, this, _1, _2));

  // Marker publisher
  // robot_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic_name, 10);
}

void ATRStatePublisher::init()
{
  // Get paramters from a list
  std::vector<std::string> param_names = { "id", "frame_id", "period_ms", "goal", "radius", "atr_path_service_prefix" };
  for (auto&& i : param_names)
    declare_parameter(i);

  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  atr_id_ = params.at(0).as_int();
  atr_frame_id_ = params.at(1).as_string();
  atr_period_ms_ = params.at(2).as_int();
  v_atr_goal_ = params.at(3).as_double_array();
  atr_radius_ = params.at(4).as_double();
  std::string service_prefix = params.at(5).as_string();

  // TODO: add namespaces??
  atr_name_ = "atr_" + std::to_string(atr_id_);
  atr_topic_name_ = "state_" + atr_name_;
  atr_service_name_ = "~/trigger_" + atr_topic_name_;
  path_service_name_ = service_prefix + std::to_string(atr_id_);

  /// Parameters hosted by another node
  //   rclcpp::SyncParametersClient::SharedPtr parameters_client;

  //   auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "remote_node_name");
  //   while (!parameters_client->wait_for_service(1s))
  //   {
  //     if (!rclcpp::ok())
  //     {
  //       RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
  //       rclcpp::shutdown();
  //     }
  //     RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  //   }
  //   auto parameters = parameters_client->get_parameters({ "remote_param1", "remote_param2" });
}

void ATRStatePublisher::updateATRPathCB(const std::shared_ptr<atr_interfaces::srv::UpdateATRPath::Request> request,
                                        std::shared_ptr<atr_interfaces::srv::UpdateATRPath::Response> response)
{
  atr_path_available = true;
  response->success = true;

  std::lock_guard<std::mutex> guard(path_mutex_);
  target_path_ = request->path;
}

void ATRStatePublisher::timer_callback()
{
  // We can control the ATRState publisher with the trigger service.
  // By default it is activated.
  if (broadcast_data_flag and atr_path_available)
  {
    // Local copy of the target path
    path_mutex_.lock();
    atr_interfaces::msg::PathWithDTime local_target_path = target_path_;
    path_mutex_.unlock();

    geometry_msgs::msg::TransformStamped ts;
    std::vector<geometry_msgs::msg::TransformStamped> v_ts;
    atr_interfaces::msg::ATRStateStamped message;

    int n_poses = local_target_path.poses.size();

    // Reset the counter when all the points have been covered
    count_ = (count_ > n_poses - 2) ? 0 : count_;

    // Generate a dummy pose
    // double step = 0.01;
    // double theta = count_ * step;

    double x = local_target_path.poses.at(count_).pose.position.x;
    double y = local_target_path.poses.at(count_).pose.position.y;

    double xf = local_target_path.poses.at(count_ + 1).pose.position.x;
    double yf = local_target_path.poses.at(count_ + 1).pose.position.y;

    double Dx = xf - x;
    double Dy = yf - y;

    double Dt = local_target_path.poses.at(count_).delta_time;

    double theta = atan2(Dy, Dx);

    // Using TF. Coordinate frame dummy with respect to map rotating by angle theta

    tf2::Transform tf;
    tf2::Quaternion q, q_z_pi2;

    // RPY = Rot_x*Rot_y*Rot_z
    q.setRPY(0, 0, theta);  // Basic Rotation in z
    q.normalize();

    // q_z_pi2.setRPY(0, 0, M_PI_2);
    // q_z_pi2.normalize();

    // Quaternion to Rotation Matrix (3x3)
    // tf2::Matrix3x3 Rh_m(q);
    // tf.setOrigin(Rh_m * tf2::Vector3(1, 0, 0));

    // x=cos(theta), y=sin(theta)
    //            Rh_m * [1,0,0]^T
    // tf.setOrigin(tf2::Matrix3x3(q) * tf2::Vector3(atr_radius_, 0, 0));

    tf.setOrigin(tf2::Vector3(x, y, 0));
    count_ += 1;

    // tf.setRotation((q * q_z_pi2).normalize());
    tf.setRotation(q);

    ts.transform = tf2::toMsg(tf);

    ts.header.frame_id = atr_frame_id_;
    ts.header.stamp = now();
    ts.child_frame_id = atr_name_;
    v_ts.push_back(ts);

    message.header.frame_id = atr_frame_id_;
    message.header.stamp = ts.header.stamp;
    message.state.atr_id = atr_id_;

    // Setting the pose fused_odom using the ts information
    tf2::convert(ts.transform, message.state.pose.fused_odom);
    // Dummy ATR is in Node 1 (charging station)
    message.state.pose.pose_id = 1;
    // Name of the current Node
    message.state.pose.name = "Charging Station";
    // Description of the node
    message.state.pose.description = "Charging station next to the main entrance";
    // Pose Type (what is the ATR doing in this Node)
    message.state.pose.type.id = atr_interfaces::msg::ATRPoseType::IDLE;

    // Time derivative of the fused_odom
    message.state.vel.fused_odom.linear.x = Dx / Dt;
    message.state.vel.fused_odom.linear.y = Dy / Dt;
    message.state.vel.fused_odom.linear.z = 0;
    message.state.vel.fused_odom.angular.x = 0;
    message.state.vel.fused_odom.angular.y = 0;
    message.state.vel.fused_odom.angular.z = 0.01 / (0.001 * atr_period_ms_);

    // message.state.full_state = false; Not needed since is set to false by default
    message.state.pose_source = atr_interfaces::msg::ATRState::ODOM;

    // Dummy goal
    double goal_x = local_target_path.poses.at(n_poses - 2).pose.position.x;
    double goal_y = local_target_path.poses.at(n_poses - 2).pose.position.y;

    q.setRPY(0, 0, M_PI);
    // tf.setOrigin(tf2::Vector3(v_atr_goal_.at(0), v_atr_goal_.at(1), 0));
    tf.setOrigin(tf2::Vector3(goal_x, goal_y, 0));
    tf.setRotation(q);
    ts.transform = tf2::toMsg(tf);
    ts.header.frame_id = atr_frame_id_;
    ts.header.stamp = now();
    ts.child_frame_id = atr_name_ + "_goal";
    v_ts.push_back(ts);

    tf2::convert(tf2::toMsg(tf), message.state.goal);

    // Over all state of the ATR
    message.state.overall.status.push_back(atr_interfaces::msg::ATRStateOverall::CHARGING);

    // ATR mission
    message.state.mission.status = atr_interfaces::msg::ATRStateMission::ARRIVED;

    // ATR Load status
    message.state.load.status = atr_interfaces::msg::ATRStateLoad::UNLOADED;

    // ATR Signals (which signals should be activated in the ATR)
    message.state.signal.types.push_back(atr_interfaces::msg::ATRStateSignals::CHARGING);

    // ATR Actuator
    message.state.actuator.type = atr_interfaces::msg::ATRStateActuator::LINEAR;
    message.state.actuator.status = atr_interfaces::msg::ATRStateActuator::CLOSED;
    message.state.actuator.value = 0.0;

    // ATR Emergency stopped?
    message.state.emerg_stop = false;

    // ATR Battery
    message.state.battery.status = atr_interfaces::msg::ATRBatteryState::FULL;
    // Battery charge (100%)
    message.state.battery.charge_state = 1.0;
    // Battery duration (5 hrs)
    message.state.battery.life_time = 5.0;
    // Battery capacity, in this case, we assume the battery has 100% capacity
    message.state.battery.health_state = 1.0;

    // Collision state

    // Collision sensors. We assume two type of sensors (two ultrasonic and one bumper)
    atr_interfaces::msg::ATRCollisionState collision_state;

    // No collision
    collision_state.status = atr_interfaces::msg::ATRCollisionState::NONE;
    // minimal distance to an obstacle (no collision we use the maximum value)
    collision_state.distance = 1000;
    collision_state.description = "Collision Free";

    atr_interfaces::msg::ATRCollisionSensor aux_sensor;

    // Ultrasonic 1
    aux_sensor.id = 0;
    aux_sensor.type = atr_interfaces::msg::ATRCollisionSensor::ULTRA_SONIC;
    aux_sensor.data = 1000.0;
    collision_state.sensors.push_back(aux_sensor);

    // Ultrasonic 2
    aux_sensor.id = 1;
    aux_sensor.type = atr_interfaces::msg::ATRCollisionSensor::ULTRA_SONIC;
    aux_sensor.data = 1000.0;
    collision_state.sensors.push_back(aux_sensor);

    message.state.collisions.push_back(collision_state);

    // Bumper (populating the message without aux instances)
    // add a new collision state
    message.state.collisions.push_back(atr_interfaces::msg::ATRCollisionState());
    // Fill in the variables
    message.state.collisions[1].status = atr_interfaces::msg::ATRCollisionState::NONE;
    message.state.collisions[1].distance = 1000;
    message.state.collisions[1].description = "Bumper off";

    // add a new sensor for this collision state
    message.state.collisions[1].sensors.push_back(atr_interfaces::msg::ATRCollisionSensor());
    // Fill in the parameters of the new sensor
    message.state.collisions[1].sensors[0].type = atr_interfaces::msg::ATRCollisionSensor::BUMPER;
    message.state.collisions[1].sensors[0].id = 3;
    message.state.collisions[1].sensors[0].data = 0;

    // Publish transformation (ATR reference frame relative to frame_id_)
    tf_broadcaster_->sendTransform(v_ts);

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("atr_examples");

    publisher_->publish(message);
  }
}

void ATRStatePublisher::toggle_publisher_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  broadcast_data_flag = !broadcast_data_flag;
  if (broadcast_data_flag)
  {
    // We use the success flag to report whether the topic is on or off
    response->success = true;
    response->message = "Broadcasting data ON";
    RCLCPP_INFO_STREAM(get_logger(), response->message);
  }
  else
  {
    response->success = false;
    response->message = "Broadcasting data OFF";
    RCLCPP_INFO_STREAM(get_logger(), response->message);
  }
  // Dummy if (hack to avoid unused variable warning)
  // clang-format off
  if (request){}
}

}  // namespace atr_examples
