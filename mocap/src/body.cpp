/*!
 * \file body.cpp
 *
 * @todo Brief file description
 *
 * Created on: Oct 16, 2013
 *     Author: Mark Cutler
 *     Email: markjcutler@gmail.com
 * Updated on: March, 2016
 *       Author: Shih-Yuan Liu
 *     Email: shihyuan.liu@gmail.com
 * Updated further on: 12 Dec 2019
 *       Author: Parker Lusk
 *     Email: parkerclusk@gmail.com
 */

#include "mocap/body.h"

namespace acl {
namespace mocap {

Body::Body(const ros::NodeHandle &nh, const RPCClientPtr rpc_client,
           const std::string &name, const Parameters &params)
    : nh_(nh), rpc_client_(rpc_client), name_(name), params_(params) {
  model_markers_ = rpc_client_->getModelMarkers(name_);
}

// ----------------------------------------------------------------------------

void Body::update(double dt, const RigidBodyMeasurement &rb) {
  // Initialize publishers once we get a valid measurement from mocap
  if (!pub_initialized_ && !rb.occluded) {
    initPublishers();
  }

  propagationUpdate(dt);
  measurementUpdate(dt, rb);

  // use the timestamp from the mocap server
  timestamp_ns_ = rb.time_ns;
}

// ----------------------------------------------------------------------------

bool Body::broadcastROS() {
  // Do nothing if publisher not initialized yet.
  if (!pub_initialized_) {
    ROS_WARN_THROTTLE(
        1, "Body::broadcastROS() -- publishers not initialized yet\n");
    return false;
  }

  // Use timestamp from mocap server
  ros::Time timestamp;
  timestamp.fromNSec(timestamp_ns_);

  //
  // Status
  //

  // Always publish status signal if desired
  status_.header.stamp = timestamp;
  if (params_.pub_status)
    pub_status_.publish(status_);

  // TODO: We might still want to publish the propagated signal
  // even if we didn't receive a measurement update this timestep
  if (status_.skip_update)
    return false;

  //
  // Pose
  //

  geometry_msgs::PoseStamped msgpose;
  msgpose.header.stamp = timestamp;
  msgpose.header.frame_id = params_.parent_frame;
  msgpose.pose.position = ghk.getPos();
  msgpose.pose.orientation = ekf.getAtt();
  pub_pose_.publish(msgpose);

  //
  // Twist
  //

  if (params_.pub_twist) {
    geometry_msgs::TwistStamped msgtwist;
    msgtwist.header.stamp = timestamp;
    msgtwist.header.frame_id = params_.parent_frame;
    msgtwist.twist.linear = ghk.getVel();
    msgtwist.twist.angular = ekf.getRate();
    pub_twist_.publish(msgtwist);
  }

  //
  // Accel
  //

  if (params_.pub_accel) {
    geometry_msgs::AccelStamped msgaccel;
    msgaccel.header.stamp = timestamp;
    msgaccel.header.frame_id = params_.parent_frame;
    msgaccel.accel.linear = ghk.getAcc();
    pub_accel_.publish(msgaccel);
  }

  //
  // Broadcast onto tf tree
  //

  if (params_.tf_broadcast) {
    geometry_msgs::TransformStamped msgtf;
    tf2::convert(msgpose, msgtf);
    msgtf.child_frame_id = name_;
    br_.sendTransform(msgtf);
  }

  return true;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void Body::initPublishers() {
  // construct topic names for this rigid body
  std::string topic_pose = name_ + "/" + params_.parent_frame;
  std::string topic_twist = name_ + "/mocap/twist";
  std::string topic_accel = name_ + "/mocap/accel";
  std::string topic_status = name_ + "/mocap/status";

  // initialize publishers
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_pose, 1);
  pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(topic_twist, 1);
  pub_accel_ = nh_.advertise<geometry_msgs::AccelStamped>(topic_accel, 1);

  if (params_.pub_status) {
    pub_status_ = nh_.advertise<::mocap::Status>(topic_status, 1);
  }

  pub_initialized_ = true;
}

// ----------------------------------------------------------------------------

void Body::propagationUpdate(double dt) {
  ghk.propUpdate(dt);
  ekf.propUpdate(dt);
}

// ----------------------------------------------------------------------------

void Body::measurementUpdate(double dt, const RigidBodyMeasurement &rb) {
  // unpack pose from raw measurement
  geometry_msgs::Pose meas = unpackMeasurement(rb);

  // first time, initialize last meas to current meas
  if (timestamp_ns_ == 0)
    last_meas_ = meas;

  // Handle possible quaternion representation flip
  bool didFlip = getFlipProcessedQuaternion(ekf.getAtt(), meas.orientation);

  // Populate status
  status_.occluded = rb.occluded;
  status_.num_markers = rb.nrMarkers;
  status_.num_visible_markers = rb.nrVisibleMarkers;
  status_.too_few_markers =
      (rb.nrMarkers - rb.nrVisibleMarkers > params_.num_markers_missing_tol);
  status_.q_diff = getAngleDifference(last_meas_.orientation, meas.orientation);
  status_.q_jumped = (status_.q_diff > params_.q_diff_threshold);
  status_.q_flipped = didFlip;

  status_.markers.clear();
  for (const auto &marker : rb.markers) {
    if (marker.occluded)
      continue;
    ::mocap::Marker markermsg;
    markermsg.name = marker.name;
    markermsg.position.x = marker.x;
    markermsg.position.y = marker.y;
    markermsg.position.z = marker.z;

    // mocap marker dot w.r.t mocap origin
    tf2::Transform T_Mm;
    tf2::convert(markermsg.position, T_Mm);

    // body w.r.t parent
    tf2::Transform T_PB;
    tf2::convert(meas, T_PB);

    // body marker w.r.t parent -- i.e., transform into ROS frame
    const tf2::Transform T_PBm = params_.T_PM * T_Mm * params_.T_MbB;

    // body marker w.r.t body
    const tf2::Transform T_BBm = T_PB.inverse() * T_PBm;
    tf2::convert(T_BBm, markermsg.position);

    status_.markers.push_back(markermsg);
  }

  status_.model_markers.clear();
  for (const auto &marker : model_markers_) {
    ::mocap::Marker markermsg;
    markermsg.name = marker.name;
    markermsg.position.x = marker.x;
    markermsg.position.y = marker.y;
    markermsg.position.z = marker.z;

    // mocap marker w.r.t mocap body
    tf2::Transform T_MbMm;
    tf2::convert(markermsg.position, T_MbMm);

    // body marker w.r.t body -- i.e., transform into ROS frame
    const tf2::Transform T_BBm =
        params_.T_MbB.inverse() * T_MbMm * params_.T_MbB;
    tf2::convert(T_BBm, markermsg.position);

    status_.model_markers.push_back(markermsg);
  }

  // should this measurement be used or skipped?
  if (status_.occluded || status_.q_jumped || status_.too_few_markers) {
    status_.skip_update = true;
    status_.skip_count++;
  } else {
    status_.skip_update = false;
    status_.skip_count = 0;
  }

  // if too many updates skipped, reset filters
  if (status_.skip_count > params_.skipped_meas_pos_tol)
    ghk.reset();
  if (status_.skip_count > params_.skipped_meas_att_tol)
    ekf.reset();

  // perform measurement update only if we are using this measurement
  if (!status_.skip_update) {
    ghk.measUpdate(dt, meas.position);
    ekf.measUpdate(meas.orientation);
  }

  // Save the last measurement if it's not occluded
  if (!status_.occluded)
    last_meas_ = meas;
}

// ----------------------------------------------------------------------------

geometry_msgs::Pose
Body::unpackMeasurement(const RigidBodyMeasurement &rb) const {
  // unpack measurement
  geometry_msgs::Pose pose;
  pose.position.x = rb.x;
  pose.position.y = rb.y;
  pose.position.z = rb.z;
  pose.orientation.x = rb.qx;
  pose.orientation.y = rb.qy;
  pose.orientation.z = rb.qz;
  pose.orientation.w = rb.qw;

  //
  // Transform measurement into desired coordinate frames
  //

  // mocap body w.r.t mocap origin
  tf2::Transform T_MMb;
  tf2::convert(pose, T_MMb);

  // body w.r.t parent
  const tf2::Transform T_PB = params_.T_PM * T_MMb * params_.T_MbB;
  tf2::convert(T_PB, pose);

  return pose;
}

// ----------------------------------------------------------------------------

bool Body::getFlipProcessedQuaternion(const geometry_msgs::Quaternion &q_ref,
                                      geometry_msgs::Quaternion &q) const {
  // Makes q consistent with q_ref (by flipping if necessary).

  geometry_msgs::Quaternion q_flip;
  q_flip.w = -q.w;
  q_flip.x = -q.x;
  q_flip.y = -q.y;
  q_flip.z = -q.z;

  // Compute the squared Euclidean distance between the reference quaternion
  // and the flipped/non-flipped version of the input quaternion.
  const double dist_sq = getEuclideanDistanceSq(q_ref, q);
  const double dist_sq_flipped = getEuclideanDistanceSq(q_ref, q_flip);

  if (dist_sq_flipped < dist_sq) {
    q = q_flip;
    return true;
  } else {
    return false;
  }
}

// ----------------------------------------------------------------------------

double Body::getEuclideanDistanceSq(const geometry_msgs::Quaternion &a,
                                    const geometry_msgs::Quaternion &b) const {
  return (a.w - b.w) * (a.w - b.w) + (a.x - b.x) * (a.x - b.x) +
         (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
}

// ----------------------------------------------------------------------------

double Body::getAngleDifference(const geometry_msgs::Quaternion &a,
                                const geometry_msgs::Quaternion &b) const {
  Eigen::Quaterniond q_a(a.w, a.x, a.y, a.z);
  Eigen::Quaterniond q_b(b.w, b.x, b.y, b.z);
  return q_a.angularDistance(q_b);
}

} // namespace mocap
} // namespace acl
