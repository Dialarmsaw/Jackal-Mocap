/**
 * @file tf2_helpers.h
 * @brief Overloads to make tf2::convert support more datatypes
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 2 March 2019
 */
#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf2 {

inline void convert(const geometry_msgs::Transform &trans,
                    geometry_msgs::Point &position) {
  position.x = trans.translation.x;
  position.y = trans.translation.y;
  position.z = trans.translation.z;
}

inline void convert(const geometry_msgs::Point &position,
                    geometry_msgs::Transform &trans) {
  trans.translation.x = position.x;
  trans.translation.y = position.y;
  trans.translation.z = position.z;
}

inline void convert(const geometry_msgs::Transform &trans,
                    geometry_msgs::Pose &pose) {
  convert(trans, pose.position);
  pose.orientation = trans.rotation;
}

inline void convert(const geometry_msgs::Pose &pose,
                    geometry_msgs::Transform &trans) {
  convert(pose.position, trans);
  trans.rotation = pose.orientation;
}

inline void convert(const geometry_msgs::TransformStamped &trans,
                    geometry_msgs::PoseStamped &pose) {
  convert(trans.transform, pose.pose);
  pose.header = trans.header;
}

inline void convert(const geometry_msgs::PoseStamped &pose,
                    geometry_msgs::TransformStamped &trans) {
  convert(pose.pose, trans.transform);
  trans.header = pose.header;
}

inline void convert(const geometry_msgs::Point &position, tf2::Transform &T) {
  tf2::Vector3 origin;
  tf2::convert(position, origin);
  T.setOrigin(origin);
}

inline void convert(const tf2::Transform &T, geometry_msgs::Point &position) {
  position.x = T.getOrigin().x();
  position.y = T.getOrigin().y();
  position.z = T.getOrigin().z();
}

inline void convert(const tf2::Transform &T, geometry_msgs::Pose &pose) {
  pose.position.x = T.getOrigin().x();
  pose.position.y = T.getOrigin().y();
  pose.position.z = T.getOrigin().z();
  pose.orientation.x = T.getRotation().x();
  pose.orientation.y = T.getRotation().y();
  pose.orientation.z = T.getRotation().z();
  pose.orientation.w = T.getRotation().w();
}

} // namespace tf2
