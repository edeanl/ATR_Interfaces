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

#include <atr_examples/NONAGenerator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <atr_interfaces/msg/object_class.hpp>
#include <atr_interfaces/msg/object_type.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace atr_examples
{
NonaGenerator::NonaGenerator() : Node("nona_list_publisher")
{
  // Parameter initialization
  init();

  // Object List publisher (Static and Dynamic Objects)
  publisher_ = this->create_publisher<atr_interfaces::msg::ObjectList>(topic_name_, 10);

  // This timer triggers the publisher of the ObjectList message
  timer_ = this->create_wall_timer(1s, std::bind(&NonaGenerator::timer_callback, this));

  // Marker publisher
  v_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_name_, 10);

  // Nona Object list server
  service_ = create_service<atr_interfaces::srv::GetObjectList>(
      get_o_list_srv_name_, std::bind(&NonaGenerator::getObjectListCB, this, _1, _2));
}

void NonaGenerator::init()
{
  std::vector<std::string> param_names = { "topic_name", "marker_topic_name", "frame_id", "service_name" };
  for (auto& i : param_names)
    declare_parameter(i);
  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  topic_name_ = params.at(0).as_string();
  marker_topic_name_ = params.at(1).as_string();
  frame_id_ = params.at(2).as_string();
  get_o_list_srv_name_ = params.at(3).as_string();
}

void NonaGenerator::getObjectListCB(const std::shared_ptr<atr_interfaces::srv::GetObjectList::Request> request,
                                    std::shared_ptr<atr_interfaces::srv::GetObjectList::Response> response)
{
  auto local_message = atr_interfaces::msg::ObjectList();

  // copy the shared object locally
  data_mutex_.lock();
  local_message = nona_list_;
  data_mutex_.unlock();

  if (local_message.objects.size() > 0)
  {
    response->list = local_message;
    response->success = true;
  }
  else
  {
    std::string error_message = "No NONA objects available";
    response->success = false;
    response->error.id = atr_interfaces::msg::ATRError::DATA_NOT_AVAILABLE;
    response->error.message = error_message;
  }

  // clang-format off
  if (request){} //dummy if to prevent warning
  // clang-format on
}

void NonaGenerator::timer_callback()
{
  auto aux_message = atr_interfaces::msg::ObjectList();
  visualization_msgs::msg::MarkerArray m_marker_msg;

  std::vector<Eigen::MatrixXd> v_Points;
  std::vector<int8_t> v_classes;
  std::vector<int8_t> v_types;

  // Simulated Areas
  // Non accessible area (nonaa)
  v_Points.push_back(Eigen::MatrixXd(4, 3));
  v_Points[0] << 2, -1, 0, 3, -1, 0, 3, 1, 0, 2, 1, 0;
  v_classes.push_back(atr_interfaces::msg::ObjectClass::WALL);
  v_types.push_back(atr_interfaces::msg::ObjectType::NONAA);

  // Non accessible area (nonaa)
  v_Points.push_back(Eigen::MatrixXd(4, 3));
  v_Points[1] << 2, -1.5, 0, 2, -2.5, 0, -2, -2.5, 0, -2, -1.5, 0;
  v_classes.push_back(atr_interfaces::msg::ObjectClass::WALL);
  v_types.push_back(atr_interfaces::msg::ObjectType::NONAA);

  // Non accessible area (nonaa)
  v_Points.push_back(Eigen::MatrixXd(4, 3));
  v_Points[2] << -2.5, 1, 0, -3, 1, 0, -3, -2, 0, -2.5, -2, 0;
  v_classes.push_back(atr_interfaces::msg::ObjectClass::WALL);
  v_types.push_back(atr_interfaces::msg::ObjectType::NONAA);

  atr_interfaces::msg::ObjectStamped aux_obj;
  rclcpp::Time aux_time = now();

  visualization_msgs::msg::Marker aux_marker;
  aux_marker.header.frame_id = frame_id_;
  aux_marker.header.stamp = aux_time;
  aux_marker.ns = "points_and_lines";
  aux_marker.action = visualization_msgs::msg::Marker::ADD;
  aux_marker.pose.orientation.w = 1.0;
  aux_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  aux_marker.scale.x = 0.02;

  // Magenta color for NONA objects
  aux_marker.color.a = 1.0f;
  aux_marker.color.r = 1.0f;
  aux_marker.color.b = 1.0f;

  for (size_t i = 0; i < v_Points.size(); i++)
  {
    int64_t object_id = i + 1;
    aux_obj = set_object(frame_id_, aux_time, v_classes[i], v_types[i], object_id, i, v_Points[i]);

    // generate a closed polygon from the object points. This points will be used to create the Line markers
    get_polygon(aux_obj.polygon.points, aux_marker.points);

    // Use the same object ID for the marker ID
    aux_marker.id = object_id;

    aux_message.objects.push_back(aux_obj);
    m_marker_msg.markers.push_back(aux_marker);
  }

  // RCLCPP_INFO_STREAM(get_logger(), "MP- Publishing: " << now().nanoseconds());

  // TODO: add service to toggle the publishers
  publisher_->publish(aux_message);

  v_marker_publisher_->publish(m_marker_msg);

  // write the nona list to the share it with the server cb, we guard the write/read access
  std::lock_guard<std::mutex> guard(data_mutex_);
  nona_list_ = aux_message;

  /**
   * Examples how to use tf to transform from Rotation Matrix to Quaternion
   * using tf library and Eigen library.
   */

  // geometry_msgs::msg::TransformStamped ts;
  // std::vector<geometry_msgs::msg::TransformStamped> v_ts;

  // double theta = count_ * 0.1;
  // count_ += 1;

  // // Using TF. Coordinate frame dummy with respect to map rotating by angle theta

  // tf2::Transform tf;
  // tf2::Quaternion q;

  // // RPY = Rot_x*Rot_y*Rot_z
  // q.setRPY(0, 0, theta);  // Basic Rotation in z
  // q.normalize();

  // // Quaternion to Rotation Matrix (3x3)
  // // tf2::Matrix3x3 Rh_m(q);
  // // tf.setOrigin(Rh_m * tf2::Vector3(1, 0, 0));

  // // x=cos(theta), y=sin(theta)
  // //            Rh_m * [1,0,0]^T
  // tf.setOrigin(tf2::Matrix3x3(q) * tf2::Vector3(1, 0, 0));
  // tf.setRotation(q);

  // ts.transform = tf2::toMsg(tf);

  // ts.header.frame_id = "/map";
  // ts.header.stamp = this->now();
  // ts.child_frame_id = "/dummy";

  // v_ts.push_back(ts);

  // // Using Eigen Lib. Coordinate frame atr wtr dummy rotated around z 90°, and no translation

  // Eigen::Affine3d Th_m;

  // Th_m.linear() = Rz(M_PI_2f64);
  // Th_m.translation().setZero();
  // ts = tf2::eigenToTransform(Th_m);

  // ts.header.frame_id = "/dummy";
  // ts.header.stamp = this->now();
  // ts.child_frame_id = "/atr";

  // v_ts.push_back(ts);

  // // Publish transformations
  // tf_broadcaster_.sendTransform(v_ts);

  // // More examples of angle type conversions using tf2_ros package
  // /**< Declaration of quaternion */
  // tf2::Quaternion q1;
  // q1.setW(1);
  // q1.setX(0);
  // q1.setY(0);
  // q1.setZ(0);

  // tf2::Quaternion q2(0, 0, 0, 1);  // x, y, z, w in order

  // /**< Quaternion -> Rotation Matrix */
  // tf2::Matrix3x3 R1_0(q1);

  // tf2::Matrix3x3 R2;

  // q1.setRPY(0, 0, M_PI_2);

  // R2.setRotation(q1);

  // /**< Rotation Matrix - > Quaternion */
  // R1_0.getRotation(q1);

  // // Rotating a vector p1 using Rotation matrix R1_0 (90° in Z)
  // tf2::Vector3 p1(1, 0, 0);

  // RCLCPP_WARN_STREAM(get_logger(), "p1= [" << p1.getX() << ", " << p1.getY() << ", " << p1.getZ() << "]");

  // tf2::Vector3 p0;
  // p0 = R2 * p1;

  // RCLCPP_WARN_STREAM(get_logger(), "p0= [" << p0.getX() << ", " << p0.getY() << ", " << p0.getZ() << "]");

  // /**< rotation Matrix -> rpy */
  // double roll, pitch, yaw;
  // R1_0.getRPY(roll, pitch, yaw);

  // /**< rpy -> quaternion */
  // tf2::Quaternion q3;
  // q3.setRPY(roll, pitch, yaw);
  // q3.normalize();
}

}  // namespace atr_examples