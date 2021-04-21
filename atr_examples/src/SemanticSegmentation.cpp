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

#include <atr_examples/SemanticSegmentation.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <atr_interfaces/msg/object_class.hpp>
#include <atr_interfaces/msg/object_type.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace atr_examples
{
SemanticSegmentation::SemanticSegmentation() : Node("object_list_publisher")
{
  // Parameter initialization
  init();

  // Object List publisher (Static and Dynamic Objects)
  publisher_ = this->create_publisher<atr_interfaces::msg::ObjectList>(topic_name_, 10);

  // This timer triggers the publisher of the ObjectList message
  timer_ = this->create_wall_timer(1s, std::bind(&SemanticSegmentation::timer_callback, this));
}

void SemanticSegmentation::init()
{
  std::vector<std::string> param_names = { "topic_name", "frame_id" };
  for (auto&& i : param_names)
    declare_parameter(i);
  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  topic_name_ = params.at(0).as_string();
  frame_id_ = params.at(1).as_string();
}

void SemanticSegmentation::timer_callback()
{
  auto aux_message = atr_interfaces::msg::ObjectList();

  std::vector<Eigen::MatrixXd> v_Points;
  std::vector<int8_t> v_classes;
  std::vector<int8_t> v_types;

  static double inc = 0.0;

  /// Simulated Object Data
  // Human 1 (Dynamic)
  v_Points.push_back(Eigen::MatrixXd(5, 3));
  v_Points[0] << 0.1, -0.3, 0, 0, -0.2, 0, 0.1, -0.15, 0, 0.2, -0.2, 0, 0.25, -0.29, 0;
  // Adding noise and small motion
  v_Points[0] += 0.02 * Eigen::MatrixXd::Random(5, 3) + inc * Eigen::MatrixXd::Ones(5, 3);
  // z coordinate should be zero
  v_Points[0].col(2) << Eigen::VectorXd::Zero(5, 1);

  v_classes.push_back(atr_interfaces::msg::ObjectClass::HUMAN);
  v_types.push_back(atr_interfaces::msg::ObjectType::DYNAMIC);

  // Human 2 (Dynamic)
  v_Points.push_back(Eigen::MatrixXd(5, 3));
  v_Points[1] << 0.02, 0.12, 0.02, 0.02, 0.22, 0.02, 0.11, 0.27, 0.02, 0.18, 0.21, 0.02, 0.25, 0.11, 0.01;
  v_Points[1] += 0.02 * Eigen::MatrixXd::Random(5, 3) - inc * Eigen::MatrixXd::Ones(5, 3);
  // z coordinate should be zero
  v_Points[1].col(2) << Eigen::VectorXd::Zero(5, 1);

  // Reset increment when reaching 1.0
  inc > 1.0 ? inc = 0.0 : inc += 0.05;

  v_classes.push_back(atr_interfaces::msg::ObjectClass::HUMAN);
  v_types.push_back(atr_interfaces::msg::ObjectType::DYNAMIC);

  // Forklift (static)
  v_Points.push_back(Eigen::MatrixXd(4, 3));
  v_Points[2] << 1, 0.1, 0, 1.2, 0, 0, 1.2, 0.2, 0, 1, 0.2, 0;
  v_classes.push_back(atr_interfaces::msg::ObjectClass::FORKLIFT);
  v_types.push_back(atr_interfaces::msg::ObjectType::STATIC);

  // Container (static)
  v_Points.push_back(Eigen::MatrixXd(4, 3));
  // clang-format off
  v_Points[3] << -1.0, 0.0, 0, 
                 -1.0, 0.3, 0,
                 -1.5, 0.3, 0,
                 -1.5, 0.0, 0;
  // clang-format on
  v_classes.push_back(atr_interfaces::msg::ObjectClass::CONTAINER);
  v_types.push_back(atr_interfaces::msg::ObjectType::STATIC);

  atr_interfaces::msg::ObjectStamped aux_obj;
  rclcpp::Time aux_time = now();

  for (size_t i = 0; i < v_Points.size(); i++)
  {
    aux_obj = set_object(frame_id_, aux_time, v_classes[i], v_types[i], i + 1, i, v_Points[i]);
    aux_message.objects.push_back(aux_obj);
  }

  publisher_->publish(aux_message);
}

}  // namespace atr_examples

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