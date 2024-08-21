/*!
 * \file body.h
 *
 * Class holding position and orientation filters for vicon objects.
 *
 * Created on: Oct 16, 2013
 *     Author: Mark Cutler
 *      Email: markjcutler@gmail.com
 * Updated further on: 12 Dec 2019
 *       Author: Parker Lusk
 *     Email: parkerclusk@gmail.com
 */

#ifndef BODY_H_
#define BODY_H_

#include <algorithm>
#include <deque>
#include <string>

#include <ros/ros.h>

#include <Eigen/Dense>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mocap/Marker.h>
#include <mocap/Status.h>

#include "mocap/GHKFilter.h"
#include "mocap/attitude_ekf.h"
#include "mocap/client.h"
#include "mocap/rpc_client.h"
#include "mocap/tf2_helpers.h"

namespace acl {
namespace mocap {

/**
 *  This class has position and orientation filters as members.
 */
class Body {
public:
  struct Parameters {
    std::string parent_frame; ///< name of (ROS) parent frame

    tf2::Transform T_PM;  ///< mocap origin w.r.t parent frame (internal)
    tf2::Transform T_MbB; ///< ROS body w.r.t mocap body (internal)

    // how many markers can be missing before measurement is considered bad?
    uint8_t num_markers_missing_tol = 4;

    // how large of an angle orientation jump before meas is considered bad?
    double q_diff_threshold = 0.1;

    // Filters reset after this many skipped measurement updates
    uint8_t skipped_meas_pos_tol = 3;  ///< for position
    uint8_t skipped_meas_att_tol = 20; ///< for attitude

    // should publish signals onto ROS network
    bool pub_twist = true;
    bool pub_accel = true;
    bool pub_status = true;

    bool tf_broadcast = true; ///< should broadcast pose on tf tree
  };

public:
  Body(const ros::NodeHandle &nh, const RPCClientPtr rpc_client,
       const std::string &name, const Parameters &params);
  ~Body() = default;

  void update(double dt, const RigidBodyMeasurement &rb);
  bool broadcastROS();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_pose_, pub_twist_, pub_accel_, pub_status_;
  tf2_ros::TransformBroadcaster br_;
  RPCClientPtr rpc_client_;

  /// \brief Internal state
  std::string name_;  ///< Body name_ as specified in Tracker software
  Parameters params_; ///< filtering, coordinate frames, broadcasting params
  ::mocap::Status status_;        ///< mocap and filter status
  uint64_t timestamp_ns_ = 0;     ///< latest meas timestamp from mocap server
  bool pub_initialized_ = false;  ///< Have ROS publishers been started?
  geometry_msgs::Pose last_meas_; ///< measurement from last iteration
  std::vector<Marker> model_markers_; ///< expected position of markers

  /// \brief Filters
  GHKFilter ghk;   ///< position, velocity, acceleration filter
  AttitudeEKF ekf; ///< attitude and attitude rate filter

  void initPublishers();

  void propagationUpdate(double dt);
  void measurementUpdate(double dt, const RigidBodyMeasurement &rb);

  geometry_msgs::Pose unpackMeasurement(const RigidBodyMeasurement &rb) const;
  bool getFlipProcessedQuaternion(const geometry_msgs::Quaternion &q_ref,
                                  geometry_msgs::Quaternion &q) const;
  double getEuclideanDistanceSq(const geometry_msgs::Quaternion &a,
                                const geometry_msgs::Quaternion &b) const;
  double getAngleDifference(const geometry_msgs::Quaternion &a,
                            const geometry_msgs::Quaternion &b) const;
};

} // namespace mocap
} // namespace acl

#endif /* BODY_H_ */
