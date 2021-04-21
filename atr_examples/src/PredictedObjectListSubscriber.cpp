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

#include <atr_examples/PredictedObjectListSubscriber.h>

namespace atr_examples
{
PredictedObjectListSubscriber::PredictedObjectListSubscriber()
  : Node("pred_object_list_subscriber"), first_message_(true)
{
  // Init parameters
  init();

  // Subscription to Object list topic
  subscription_ = create_subscription<atr_interfaces::msg::PredictedObjectEllipseList>(
      pred_object_topic_name_, 10, std::bind(&PredictedObjectListSubscriber::topic_callback, this, _1));

  // Create generic markers for the predictions. These markers use the map as a frame ID.
  // This will be changed depending on the target object
  for (size_t i = 0; i < pred_horizon_; i++)
  {
    // clang-format off
    m_array_.markers.push_back(
        createMarkerMesh(frame_id_,
                         topic_namespace_, 
                         0,
                         visualization_msgs::msg::Marker::CYLINDER,
                         /*pos xyz:*/ 0.1 * i, 0.0, 0.0,
                         /*orientation quaternion wxyz*/ 0, 0, 0, 1,
                         /*scale s_x s_y s_z*/ 0.3, 0.15, 0.001,
                         /*color*/ 1, 0,0,0.4));
    // clang-format on
  }

  // Define different colors for each prediction. The alpha channel will be defined by the weight of the estimation
  size_t step = static_cast<size_t>(MAX_COLORS / pred_horizon_);

  for (size_t i = 0, count = 0; i < pred_horizon_ * step; i += step, count++)
  {
    m_array_.markers[count].color.r = Colors_(i, 0);
    m_array_.markers[count].color.g = Colors_(i, 1);
    m_array_.markers[count].color.b = Colors_(i, 2);
    m_array_.markers[count].color.a = 0.4;
  }

  // Tf publisher (needed for the MarkerArray)
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Marker publisher
  v_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(topic_name_, 10);
}

PredictedObjectListSubscriber::~PredictedObjectListSubscriber()
{
}

void PredictedObjectListSubscriber::init()
{
  std::vector<std::string> param_names = { "topic_name",          "topic_ns",          "frame_id", "pred_frame_id",
                                           "pred_obj_topic_name", "prediction_horizon" };
  for (auto&& i : param_names)
    declare_parameter(i);
  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  topic_name_ = params.at(0).as_string();
  topic_namespace_ = params.at(1).as_string();
  frame_id_ = params.at(2).as_string();
  pred_frame_name_ = params.at(3).as_string();
  pred_object_topic_name_ = params.at(4).as_string();
  pred_horizon_ = params.at(5).as_int();

  if (pred_horizon_ > MAX_COLORS)
  {
    // clang-format off
    RCLCPP_WARN_STREAM(get_logger(),
                       "Prediction Horizon (" << pred_horizon_ << ") exceeds max number [" 
                                              << MAX_COLORS << "]. Using {"
                                              << MAX_COLORS << "} instead.");
    // clang-format on
  }

  // Finish this class using the ObjectListServer
}

void PredictedObjectListSubscriber::topic_callback(const atr_interfaces::msg::PredictedObjectEllipseList::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray m_marker_msg;
  std::vector<geometry_msgs::msg::TransformStamped> l_v_ts_;

  // Publish TF for the markers, we use the position of the dynamic obstacle
  geometry_msgs::msg::TransformStamped ts;
  tf2::Transform tf;
  tf2::Quaternion q_z_pi2;
  q_z_pi2.setRPY(0, 0, M_PI_2);
  q_z_pi2.normalize();

  // Number of dynamic objects
  for (auto&& i : msg->p_object_ellipses)
  {
    // Number of predictions per object
    size_t cycles = pred_horizon_;

    size_t rec_predict_number = i.p_ellipse.size();

    if (rec_predict_number != pred_horizon_)
    {
      cycles = std::min(rec_predict_number, pred_horizon_);
      RCLCPP_WARN_STREAM(get_logger(),
                         "Number of received predictions (" << rec_predict_number
                                                            << ") not equal to prediction horizon [" << pred_horizon_
                                                            << "], using the min value: " << cycles);
    }

    for (size_t j = 0; j < cycles; j++)
    {
      visualization_msgs::msg::Marker aux_marker;
      // We get the prediction ellipse from the msg
      atr_interfaces::msg::Ellipse aux_elipse = i.p_ellipse[j];
      // Assign the corresponding base marker. This marker has the standard fields defined
      aux_marker = m_array_.markers[j];
      // We need to modify the position and scaling based on the ellipse msg info
      aux_marker.pose.position.x = aux_elipse.o_x;
      aux_marker.pose.position.y = aux_elipse.o_y;
      aux_marker.pose.position.z = 0;

      aux_marker.scale.x = aux_elipse.o_w;
      aux_marker.scale.y = aux_elipse.o_h;

      // We use the probability weight as alpha channel
      aux_marker.color.a = aux_elipse.weight;

      // We can use the frame_id defined in the message. This will be useful if we define the predicted poses
      // relative to the dynamic object coordinate frame
      aux_marker.header.frame_id = i.header.frame_id;

      aux_marker.id = i.object_id * MAX_COLORS + j;
      aux_marker.ns = topic_namespace_ + std::to_string(i.object_id);
      aux_marker.header.stamp = now();

      m_marker_msg.markers.push_back(aux_marker);
    }

    // Tf used as reference frame for the markers (Ellipses)
    tf.setOrigin(tf2::Vector3(i.centroid.x, i.centroid.y, 0));

    tf.setRotation(q_z_pi2);
    ts.transform = tf2::toMsg(tf);
    ts.header.frame_id = frame_id_;
    ts.header.stamp = now();

    // Tf coordinate frame for the markers
    ts.child_frame_id = pred_frame_name_ + std::to_string(i.object_id);

    // Push the new tf for the dynamic object
    l_v_ts_.push_back(ts);
  }  // for n dyn obj

  tf_broadcaster_->sendTransform(l_v_ts_);
  v_marker_publisher_->publish(m_marker_msg);
}
}  // namespace atr_examples
