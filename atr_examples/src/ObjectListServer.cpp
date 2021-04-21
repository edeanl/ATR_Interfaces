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

#include <atr_examples/ObjectListServer.h>

#include <chrono>
#include <random>

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

namespace atr_examples
{
ObjectListServer::ObjectListServer(/* args */)
  : Node("object_list_server"), object_list_(atr_interfaces::msg::ObjectList()), got_data_(false)
{
  // Initialize Parameters
  init();

  // Service to receive the ObjectList
  service_ = create_service<atr_interfaces::srv::UpdateObjectList>(
      update_o_list_srv_name_, std::bind(&ObjectListServer::updateObjectListCB, this, _1, _2));

  // Publisher of the estimated dynamic obstacles as ellipses
  pred_objects_publisher_ = create_publisher<atr_interfaces::msg::PredictedObjectEllipseList>(pred_obj_topic_name_, 10);

  //
  timer_ = this->create_wall_timer(50ms, std::bind(&ObjectListServer::timer_callback, this));

  RCLCPP_INFO(get_logger(), "Ready to receive new object list!!");
}

ObjectListServer::~ObjectListServer()
{
}

void ObjectListServer::init()
{
  std::vector<std::string> param_names = { "p_obj_topic_name", "srv_name",           "client_name",
                                           "frame_id",         "prediction_horizon", "prediction_sample_time" };

  for (auto&& i : param_names)
    declare_parameter(i);

  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  pred_obj_topic_name_ = params.at(0).as_string();
  update_o_list_srv_name_ = params.at(1).as_string();
  update_po_list_client_name_ = params.at(2).as_string();
  frame_id_ = params.at(3).as_string();
  pred_horizon_ = params.at(4).as_int();
  delta_time_ = params.at(5).as_double();
}

void ObjectListServer::updateObjectListCB(const std::shared_ptr<atr_interfaces::srv::UpdateObjectList::Request> request,
                                          std::shared_ptr<atr_interfaces::srv::UpdateObjectList::Response> response)
{
  response->success = true;

  // Local containers
  std::vector<geometry_msgs::msg::TransformStamped> l_v_ts_;
  atr_interfaces::msg::PredictedObjectEllipseList l_pred_obj_msg_;

  for (auto&& obj : request->list.objects)
  {
    // For each dynamic object, we publish a TF with name "pred_obj_<object_id>". This coordinate frame will
    // be the reference frame for the ellipses (predicted objects) visualized by a MarkerArray.

    if (obj.object_t.o_type == atr_interfaces::msg::ObjectType::DYNAMIC)
    {
      // Populate the predictions for each dynamic object
      atr_interfaces::msg::PredictedObjectEllipseStamped aux_p_obj;

      // Set the header frame ID. In this case, we assume that all the info is wrt map (world c.f.)
      // TODO: revise if it is better to generate the predicted object data relative to the object_id frame
      // The stamp of the header will be defined when publishing this message
      aux_p_obj.header.frame_id = frame_id_;
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
    }

    got_data_ = true;

    // We will have two containers for the different
    // types of objects, static and dynamic we get the
    // type from the message and assign it to the appropiate
    // allocator

    // For the dynamic objects, we generate the Prediction
    // ellipses and call the service in ATR Trajectory Generator
    // We will publish the Predicted objects also as a topic and create a subscriber that generates the
    // array of markers.

    // At the same time, we call the updatePredictedObj
    // server with the new data
  }  // for n objects

  // copy the local data to the shared container with mutex
  data_mutex_.lock();
  pred_obj_msg_ = l_pred_obj_msg_;
  data_mutex_.unlock();
}

void ObjectListServer::timer_callback()
{
  if (got_data_)
  {
    data_mutex_.lock();
    // Update the time stamp for each ellipse msg
    for (auto&& i : pred_obj_msg_.p_object_ellipses)
    {
      i.header.stamp = now();
    }

    // pred_objects_publisher_->publish(pred_obj_msg_);
    data_mutex_.unlock();
  }
}

}  // namespace atr_examples