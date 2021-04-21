#ifndef TF2_CONVERT_EXTENDED_H
#define TF2_CONVERT_EXTENDED_H

/*! \file convert_extended.h
 *  \brief Provides additional conversions between ROS2 tf2 and geometry messages.
 *
 *  Provides the following functionalities:
 *    - Conversion from geometry_msgs::msg::Transform to geometry_msgs::msg::Pose
 *    - Conversion from geometry_msgs::msg::Pose to geometry_msgs::msg::Transform
 */

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf2
{
/**
 * @brief convert from Transform to Pose messages
 *
 * @param trans Transform message
 * @param pose Pose message
 */
inline void convert(const geometry_msgs::msg::Transform& trans, geometry_msgs::msg::Pose& pose)
{
  pose.orientation = trans.rotation;
  pose.position.x = trans.translation.x;
  pose.position.y = trans.translation.y;
  pose.position.z = trans.translation.z;
}

/**
 * @brief convert from Pose  to Transform messages
 *
 * @param pose Pose message
 * @param trans Transform message
 */
inline void convert(const geometry_msgs::msg::Pose& pose, geometry_msgs::msg::Transform& trans)
{
  trans.rotation = pose.orientation;
  trans.translation.x = pose.position.x;
  trans.translation.y = pose.position.y;
  trans.translation.z = pose.position.z;
}
}  // namespace tf2
#endif