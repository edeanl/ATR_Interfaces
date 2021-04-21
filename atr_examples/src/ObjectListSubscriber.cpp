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

#include <atr_examples/ObjectListSubscriber.h>

namespace atr_examples
{
ObjectListSubscriber::ObjectListSubscriber() : Node("object_list_subscriber"), first_message_(true)
{
  // Init parameters
  init();

  // Subscription to Object list topic
  subscription_ = create_subscription<atr_interfaces::msg::ObjectList>(
      "object_list", 10, std::bind(&ObjectListSubscriber::topic_callback, this, _1));

  // Marker publisher
  v_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_name_, 10);
}

ObjectListSubscriber::~ObjectListSubscriber()
{
}

void ObjectListSubscriber::init()
{
  std::vector<std::string> param_names = { "topic_name_prefix", "marker_topic_name", "frame_id" };
  for (auto&& i : param_names)
    declare_parameter(i);
  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  topic_prefix_ = params.at(0).as_string();
  marker_topic_name_ = params.at(1).as_string();
  frame_id_ = params.at(2).as_string();
}

void ObjectListSubscriber::topic_callback(const atr_interfaces::msg::ObjectList::SharedPtr msg)
{
  // Dynamically create publishers according to the number of objects.
  // Steps:
  // 1) get the size of objects,
  // 2) compare with the number of publishers,
  // 3) if the sizes don't match, create the new publishers using the object_id for the name.
  // 4) use the map to identify the unregistered object (new object)
  // 5) populate the new object message and publish it

  // if there are new objects, we need to register them and create a publisher for each of them
  // TODO: change this, the number of objects can be the same as the number of publishers but they can be
  // different
  if (v_publishers_.size() != msg->objects.size())
  {
    // find the new object
    for (auto&& i : msg->objects)
    {
      int16_t obj_id = i.object_id;
      std::unordered_map<int, int>::iterator it = map_id_index_.find(obj_id);

      // if the it = end then this is the new obj
      if (it == map_id_index_.end())
      {
        std::string s;
        s = topic_prefix_ + std::to_string(obj_id);
        PolygonPublisher aux_publisher = this->create_publisher<geometry_msgs::msg::PolygonStamped>(s, 10);
        // Add the new publisher to the vector of publishers
        v_publishers_.push_back(std::move(aux_publisher));

        // populate map id -> index
        map_id_index_[obj_id] = v_publishers_.size() - 1;

        RCLCPP_WARN_STREAM(get_logger(), "Created new publisher : " << s);
      }
    }
  }

  geometry_msgs::msg::PolygonStamped aux_poly;

  // At this point, both the c_publishers and the objects must have the same size
  // for (size_t i = 0; i < msg->objects.size(); i++)

  visualization_msgs::msg::MarkerArray m_marker_msg;
  visualization_msgs::msg::Marker aux_marker;
  aux_marker.ns = "dynamic_static_objects";
  aux_marker.action = visualization_msgs::msg::Marker::ADD;
  aux_marker.pose.orientation.w = 1.0;
  aux_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  aux_marker.scale.x = 0.02;

  // Alpha channel
  aux_marker.color.a = 1.0f;

  rclcpp::Time aux_time = now();

  for (auto&& obj : msg->objects)
  {
    // RCLCPP_INFO_STREAM(get_logger(), "I hear: " << i.header.stamp.sec << "." << i.header.stamp.nanosec);

    // Get the object id
    int16_t obj_id = obj.object_id;
    // find its corresponding publisher (index)
    int16_t idx = map_id_index_[obj_id];

    // Populate the Polygon message
    aux_poly.header.frame_id = frame_id_;
    aux_poly.header.stamp = aux_time;
    aux_poly.polygon = obj.polygon;

    // Publish using the correct publisher
    v_publishers_[idx]->publish(aux_poly);

    // Populate the Marker Array to visualize the Polygons as Line Markers
    aux_marker.header.frame_id = frame_id_;
    aux_marker.header.stamp = aux_time;

    // Use the same object ID for the marker ID
    aux_marker.id = obj_id;

    switch (obj.object_t.o_type)
    {
      case atr_interfaces::msg::ObjectType::STATIC:
        // Blue color for STATIC objects
        aux_marker.color.r = 0.0f;
        aux_marker.color.g = 0.0f;
        aux_marker.color.b = 1.0f;
        break;
      case atr_interfaces::msg::ObjectType::DYNAMIC:
        // REd color for STATIC objects
        aux_marker.color.r = 1.0f;
        aux_marker.color.g = 0.0f;
        aux_marker.color.b = 0.0f;
        break;
      default:
        // Green color for UNKNOWN objects
        aux_marker.color.r = 0.0f;
        aux_marker.color.g = 0.0f;
        aux_marker.color.b = 1.0f;
        break;
    }

    /// Populate Points
    // generate a closed polygon from the object points. This points will be used to create the Line markers
    get_polygon(obj.polygon.points, aux_marker.points);

    // Create the msg
    m_marker_msg.markers.push_back(aux_marker);
  }
  // Publish Polygons as Line Markers
  v_marker_publisher_->publish(m_marker_msg);
}
}  // namespace atr_examples
