/**
 * @file mocap_node.cpp
 * @brief Entry point for mocap ROS node
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 11 Dec 2019
 */

#include <ros/ros.h>

#include "mocap/mocap.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mocap");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  acl::mocap::Mocap node(nhtopics, nhparams);
  node.spin();
  return 0;
}
