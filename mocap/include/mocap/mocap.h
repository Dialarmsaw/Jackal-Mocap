/**
 * @file mocap.h
 * @brief Broadcasts mocap data onto ROS network
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 11 Dec 2019
 */

#pragma once

#include <algorithm>
#include <cctype>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <mocap/Cameras.h>
#include <mocap/Markers.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mocap/body.h"
#include "mocap/client.h"
#include "mocap/client/optitrack.h"
#include "mocap/client/vicon.h"
#include "mocap/rpc_client.h"

namespace acl {
namespace mocap {

class Mocap {
public:
  Mocap(const ros::NodeHandle &nh, const ros::NodeHandle &nhp);
  ~Mocap() = default;

  /**
   * @brief      Main loop. Synchronously checks for data and publishes it.
   */
  void spin();

private:
  ros::NodeHandle nh_, nhp_;
  ros::Publisher pub_unlabeledmarkers_, pub_cameras_;

  RPCClientPtr rpc_client_;
  bool should_pub_unlabeledmarkers_, should_pub_cameras_;

  std::unique_ptr<Client> client_; ///< ptr to mocap client specialization
  double mocap_dt_; ///< measured period at which we receive mocap

  /// \brief tracked rigid bodies (i.e., enabled in mocap GUI), keyed by name
  std::map<std::string, std::unique_ptr<Body>> bodyMap_;
  Body::Parameters bodyParams_; ///< user parameters for each rigid body

  /**
   * @brief      Initialize the mocap client with ROS parameters
   *
   * @return     True if successful
   */
  bool init();

  /**
   * @brief      Parse a string into a transform
   *
   * @param[in]  xyzYPR  String with "x y z Y P R"
   * @param      T       resulting transform object
   *
   * @return     Whether or not parsing was successful
   */
  bool parse_xyzYPR(const std::string &xyzYPR, tf2::Transform &T) const;

  /**
   * @brief      Print information to stdout
   */
  void screenPrint() const;

  void broadcastROS_cameras();

  void broadcastROS_unlabeled_markers();
};

} // namespace mocap
} // namespace acl
