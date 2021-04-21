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

#include <atr_examples/AuxTools.h>

#include "rclcpp/rclcpp.hpp"

namespace atr_examples
{
AuxTools::AuxTools(/* args */)
{
  // Color Matrix (Assuming a maximum prediction horizon = 20)
  Colors_.resize(MAX_COLORS, 3);

  Colors_ << 0, 0, 0.6000, 0, 0, 0.8000, 0, 0, 1.0000, 0, 0.2000, 1.0000, 0, 0.4000, 1.0000, 0, 0.6000, 1.0000, 0,
      0.8000, 1.0000, 0, 1.0000, 1.0000, 0.2000, 1.0000, 0.8000, 0.4000, 1.0000, 0.6000, 0.6000, 1.0000, 0.4000, 0.8000,
      1.0000, 0.2000, 1.0000, 1.0000, 0, 1.0000, 0.8000, 0, 1.0000, 0.6000, 0, 1.0000, 0.4000, 0, 1.0000, 0.2000, 0,
      1.0000, 0, 0, 0.8000, 0, 0, 0.6000, 0, 0;
}

AuxTools::~AuxTools()
{
}

const visualization_msgs::msg::Marker AuxTools::createMarkerMesh(std::string frame_id, std::string name_space, int id,
                                                                 int shape, double x, double y, double z,
                                                                 /*position*/ double q_w, double q_x, double q_y,
                                                                 double q_z,
                                                                 /*orientation in quaternion*/ double s_x, double s_y,
                                                                 double s_z /*scale*/, float r, float g, float b,
                                                                 float a /*color*/, std::string meshFile) const
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame_id;
  //   marker.header.stamp = It must be defined before publishing the message;
  marker.ns = name_space;
  marker.id = id;
  marker.type = shape;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = q_x;
  marker.pose.orientation.y = q_y;
  marker.pose.orientation.z = q_z;
  marker.pose.orientation.w = q_w;
  marker.scale.x = s_x;
  marker.scale.y = s_y;
  marker.scale.z = s_z;
  marker.mesh_resource = meshFile;
  marker.mesh_use_embedded_materials = true;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.lifetime = rclcpp::Duration(0);
  marker.frame_locked = true;

  return marker;
}

Eigen::Matrix3d AuxTools::Rz(double theta_rad)
{
  Eigen::Matrix3d R;

  R << cos(theta_rad), -sin(theta_rad), 0, sin(theta_rad), cos(theta_rad), 0, 0, 0, 1;
  return R;
}

// TODO: move these functions to the AuxTools class
geometry_msgs::msg::Point32 AuxTools::set_point(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point32 aux_point;
  aux_point.x = x;
  aux_point.y = y;
  aux_point.z = z;

  return aux_point;
}

geometry_msgs::msg::Point32 AuxTools::set_point(const Eigen::Vector3d& point)
{
  geometry_msgs::msg::Point32 aux_point;
  aux_point.x = point[0];
  aux_point.y = point[1];
  aux_point.z = point[2];

  return aux_point;
}

geometry_msgs::msg::Point32 AuxTools::set_point(const Eigen::MatrixXd& Points, int64_t idx)
{
  geometry_msgs::msg::Point32 aux_point;
  aux_point.x = Points(idx, 0);
  aux_point.y = Points(idx, 1);
  aux_point.z = Points(idx, 2);

  return aux_point;
}

atr_interfaces::msg::ObjectStamped AuxTools::set_object(const std::string frame, const rclcpp::Time& c_time,
                                                        const int8_t o_class, const int8_t o_type, const int64_t o_id,
                                                        const int64_t o_idx,
                                                        const std::vector<Eigen::Vector3d>& v_points)
{
  atr_interfaces::msg::ObjectStamped aux_object;
  aux_object.header.frame_id = frame;
  aux_object.header.stamp = c_time;
  aux_object.object_c.o_class = o_class;
  aux_object.object_t.o_type = o_type;
  aux_object.object_id = o_id;
  aux_object.object_idx = o_idx;

  for (auto&& i : v_points)
  {
    aux_object.polygon.points.push_back(set_point(i));
  }

  return aux_object;
}

atr_interfaces::msg::ObjectStamped AuxTools::set_object(const std::string frame, const rclcpp::Time& c_time,
                                                        const int8_t o_class, const int8_t o_type, const int64_t o_id,
                                                        const int64_t o_idx, const Eigen::MatrixXd& M_points)
{
  atr_interfaces::msg::ObjectStamped aux_object;
  aux_object.header.frame_id = frame;
  aux_object.header.stamp = c_time;
  aux_object.object_c.o_class = o_class;
  aux_object.object_t.o_type = o_type;
  aux_object.object_id = o_id;
  aux_object.object_idx = o_idx;

  int64_t r = M_points.rows();

  // Create the list of points
  for (int64_t i = 0; i < r; i++)
  {
    aux_object.polygon.points.push_back(set_point(M_points, i));
  }

  // Calculate the centroid of the polygon
  float acc_x = 0.0;
  float acc_y = 0.0;
  float acc_z = 0.0;

  for (auto& j : aux_object.polygon.points)
  {
    acc_x += j.x;
    acc_y += j.y;
    acc_z += j.z;
  }

  size_t n = aux_object.polygon.points.size();

  aux_object.centroid.x = acc_x / n;
  aux_object.centroid.y = acc_y / n;
  aux_object.centroid.z = acc_z / n;

  return aux_object;
}

void AuxTools::get_polygon(const v_Point32& obj_points, v_Point& marker_points) const
{
  marker_points = transform_point<Point, Point32>(obj_points);
  marker_points.push_back(marker_points.front());
}

}  // namespace atr_examples

// explicit instantiation

// template <typename T, typename U>
// std::vector<T> AuxTools::transform_point(const std::vector<U>& in) const
// {
//   std::vector<T> out;
//   T aux;
//   for (auto&& i : in)
//   {
//     aux.x = i.x;
//     aux.y = i.y;
//     aux.z = i.z;
//     out.push_back(aux);
//   }

//   // add the first element of in to the last element of out (To complete the polygon)
//   aux.x = in.front().x;
//   aux.y = in.front().y;
//   aux.z = in.front().z;

//   out.push_back(aux);

//   return out;
// }

// template AuxTools::v_Point
// AuxTools::transform_point<AuxTools::Point, AuxTools::Point32>(const AuxTools::v_Point32&) const;