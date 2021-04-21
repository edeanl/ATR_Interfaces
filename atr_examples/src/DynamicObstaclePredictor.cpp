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

#include <atr_examples/DynamicObstaclePredictor.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace atr_examples
{
DynamicObstaclePredictor::DynamicObstaclePredictor() : Node("dynamic_obstacle_predictor"), first_message_(true)
{
  // load parameters
  init();

  // Subscribe to ObjectList topic
  subscription_ = create_subscription<atr_interfaces::msg::ObjectList>(
      subs_topic_name_, 10, std::bind(&DynamicObstaclePredictor::topic_callback, this, _1));

  // Create publisher for the predicted objects
  pred_objects_publisher_ = create_publisher<atr_interfaces::msg::PredictedObjectEllipseList>(pred_obj_topic_name_, 10);

  // Update Predicted Object list client
  // create a client and request the object list update in the publisher (with thread function to avoid locking)
  // Predicted Object List client
  p_obj_client_ = this->create_client<atr_interfaces::srv::UpdatePredictedObjectList>(p_obj_service_name);

  RCLCPP_INFO_STREAM(get_logger(), " Ready to predict obstacles");
}

DynamicObstaclePredictor::~DynamicObstaclePredictor()
{
}

void DynamicObstaclePredictor::init()
{
  // clang-format off
  std::vector<std::string> param_names = { "p_obj_topic_name",
                                            "client_name",
                                            "frame_id",
                                            "prediction_horizon",
                                            "prediction_sample_time",
                                            "obj_list_topic_name",
                                            "pred_obj_service_name" };
  // clang-format on

  for (auto&& i : param_names)
    declare_parameter(i);

  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  pred_obj_topic_name_ = params.at(0).as_string();
  // TODO: remove this, it is the same as p_obj_service_name
  update_po_list_client_name_ = params.at(1).as_string();
  frame_id_ = params.at(2).as_string();
  pred_horizon_ = params.at(3).as_int();
  delta_time_ = params.at(4).as_double();
  subs_topic_name_ = params.at(5).as_string();
  p_obj_service_name = params.at(6).as_string();

  // We need to init a client to ATRTrajectoryGeneration node to send the new predicted obstacles
}

void DynamicObstaclePredictor::topic_callback(const atr_interfaces::msg::ObjectList::SharedPtr msg)
{
  // RCLCPP_INFO_STREAM(get_logger(), __FILE__ << ":" << __LINE__ << ": Got Objects: " << msg->objects.size());

  // Local containers
  std::vector<geometry_msgs::msg::TransformStamped> l_v_ts_;
  atr_interfaces::msg::PredictedObjectEllipseList l_pred_obj_msg_;

  // Get the current time
  rclcpp::Time c_t = now();
  for (auto&& obj : msg->objects)
  {
    // Get the dynamic objects only
    if (obj.object_t.o_type == atr_interfaces::msg::ObjectType::DYNAMIC)
    {
      // Populate the predictions for each dynamic object
      atr_interfaces::msg::PredictedObjectEllipseStamped aux_p_obj;

      // Set the header frame ID. In this case, we assume that all the info is wrt map (world c.f.)
      // TODO: revise if it is better to generate the predicted object data relative to the object_id frame
      // The stamp of the header will be defined when publishing this message
      aux_p_obj.header.frame_id = frame_id_;
      // Set the same time stamp for all the objects
      aux_p_obj.header.stamp = c_t;

      aux_p_obj.object_c = obj.object_c;
      aux_p_obj.object_t = obj.object_t;
      aux_p_obj.object_id = obj.object_id;
      aux_p_obj.object_idx = obj.object_idx;
      aux_p_obj.delta_t = delta_time_;
      aux_p_obj.centroid = obj.centroid;

      for (size_t j = 0; j < pred_horizon_; j++)
      {
        atr_interfaces::msg::Ellipse aux_ellipse;
        double re = rand() / (double)RAND_MAX;

        aux_ellipse.k = j;
        aux_ellipse.o_x = obj.centroid.x + 0.15 * re * j;
        aux_ellipse.o_y = obj.centroid.y + 0.1 * j;
        aux_ellipse.o_w = 0.15;
        aux_ellipse.o_h = 0.3;
        aux_ellipse.o_alpha = 0.0;
        // In this case, all the predicted poses have random probability.
        aux_ellipse.weight = re + 0.1 > 1.0 ? 1.0 : re + 0.1;

        aux_p_obj.p_ellipse.push_back(aux_ellipse);
      }
      // Push the predicted positions for that dynamic object
      l_pred_obj_msg_.p_object_ellipses.push_back(aux_p_obj);
    }  // if dyn object
  }    // for n objs

  // We will have three containers for the three different
  // types of objects, static, dynamic, nonaa we get the
  // type from the message and assign it to the appropiate
  // allocator

  // For the dynamic objects, we generate the Prediction
  // ellipses and call the service in ATR Trajectory Generator
  // We will publish the Predicted objects also as a topic and create a subscriber that generates the
  // array of markers.

  pred_objects_publisher_->publish(l_pred_obj_msg_);

  // At the same time, we call the updatePredictedObj
  if (p_obj_client_->service_is_ready())
  {
    auto request = std::make_shared<atr_interfaces::srv::UpdatePredictedObjectList::Request>();
    // Copy new pred obj list as the service request
    request->p_list = l_pred_obj_msg_;

    // Call the service (update the pred obj list in the server)
    // We use the non blocking version
    auto future_result = p_obj_client_->async_send_request(
        request, std::bind(&DynamicObstaclePredictor::wait_for_srv_response<SrvRespFutureUpdatePObj>, this, _1));
  }
  else
  {
    RCLCPP_WARN_STREAM(get_logger(), "Service " + p_obj_service_name + " not ready yet, skipping data");
  }
}

}  // namespace atr_examples