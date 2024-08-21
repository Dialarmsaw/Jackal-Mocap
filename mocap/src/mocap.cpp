/**
 * @file mocap.cpp
 * @brief Broadcasts mocap data onto ROS network
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 11 Dec 2019
 */

#include "mocap/mocap.h"

namespace acl {
namespace mocap {

Mocap::Mocap(const ros::NodeHandle &nh, const ros::NodeHandle &nhp)
    : nh_(nh), nhp_(nhp) {
  // attempt to initialize, shutdown if failed
  if (!init()) {
    ROS_ERROR("Initialization failed");
    ros::shutdown();
  }

  // attempt to initialize mocap client, shutdown if failed
  ROS_INFO_STREAM("Attempting to connect to " << client_->getName()
                                              << " server...");
  if (client_->init()) {
    ROS_INFO_STREAM("Connected!");
    ROS_INFO_STREAM("ver: " << client_->getSDKVersion());
  } else {
    ROS_ERROR_STREAM("Could not initiate communication with "
                     << client_->getName() << " server!");
    ros::shutdown();
  }
}

// ----------------------------------------------------------------------------

void Mocap::spin() {
  // Do some debouncing on the number of lost packets. It is not critical
  // unless many consecutive packets are being dropped.
  constexpr int MAX_LOST_PACKETS = 4;
  unsigned int lostPackets = 0;

  while (ros::ok()) {

    // allow mocap client to receive / check for data
    if (!client_->spinOnce()) {
      if (++lostPackets >= MAX_LOST_PACKETS) {
        ROS_ERROR_STREAM_THROTTLE(1, "Did not receive data from "
                                         << client_->getName()
                                         << " server! (Are cameras on?)");
      } else {
        ROS_WARN_THROTTLE(1, "Missing / dropped packets.");
      }
    } else {
      lostPackets = 0;
    }

    //
    // Process rigid bodies from mocap client
    //

    // calculate dt between mocap data frames
    static uint64_t last_time_ns = client_->getCurrentTimestampNs();
    mocap_dt_ = (client_->getCurrentTimestampNs() - last_time_ns) * 1e-9;

    // skip iteration if dt is too small (occasionally happens on first iter)
    if (mocap_dt_ < 1e-6) {
      ROS_INFO_STREAM_THROTTLE(1, "Skipping iteration due to small dt");
      continue;
    }

    // Each 'enabled' rigid body will be present in this list. If the rigid
    // body is not visible, then occluded will be true and the measurements
    // will not be valid.
    auto rbMeasurements = client_->getRigidBodyMeasurements();
    for (const auto &rb : rbMeasurements) {

      // if this measurement doesn't correspond to an existing rigid body, add.
      if (bodyMap_.find(rb.name) == bodyMap_.end()) {
        ROS_WARN_STREAM("Found '" << rb.name << "'");
        bodyMap_[rb.name].reset(
            new Body(nh_, rpc_client_, rb.name, bodyParams_));
      }

      // update body pose using latest mocap data
      bodyMap_[rb.name]->update(mocap_dt_, rb);

      // broadcast rigid body state/info onto ROS network
      bodyMap_[rb.name]->broadcastROS();
    }

    last_time_ns = client_->getCurrentTimestampNs();

    //
    // Remove any bodies that we didn't receive a measurement of
    //

    // NOTE: A "measurement" does not necessarily mean the object is visible.
    // Objects should only be removed from the body map if it is 'disabled' in
    // the motion capture software. In other words, invalid measurements do not
    // constitute removal from the body map.

    // For each body currently in the body map, was there a corresponding meas?
    for (auto it = bodyMap_.cbegin(); it != bodyMap_.cend(); /*manual inc*/) {
      if (std::find_if(rbMeasurements.begin(), rbMeasurements.end(),
                       [&](const RigidBodyMeasurement &rb) {
                         return rb.name == it->first;
                       }) == rbMeasurements.end()) {
        // not found, remove from the body map
        ROS_WARN_STREAM("Removing '" << it->first << "'");
        bodyMap_.erase(it++);
      } else {
        // found, advance to next body
        ++it;
      }
    }

    //
    // Process Cameras and Unlabeled Markers
    //

    if (should_pub_cameras_) {
      broadcastROS_cameras();
    }

    if (should_pub_unlabeledmarkers_) {
      broadcastROS_unlabeled_markers();
    }

    // print connection information
    screenPrint();
  }
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool Mocap::init() {

  //
  // RPC Client initialization
  //

  bool enable_rpc;
  std::string rpc_server_uri;
  nhp_.param<bool>("enable_rpc", enable_rpc, false);
  nhp_.param<std::string>("rpc_server_uri", rpc_server_uri,
                          "http://192.168.0.9:1250");
  rpc_client_.reset(new RPCClient(rpc_server_uri, enable_rpc));

  //
  // General mocap parameters
  //

  // what frame should everything be published w.r.t?
  nhp_.param<std::string>("tf_parent_frame", bodyParams_.parent_frame, "world");

  // allow a hidden transformation of mocap origin w.r.t ROS world origin
  std::string mocap_wrt_parent_frame;
  nhp_.param<std::string>("mocap_wrt_parent_frame", mocap_wrt_parent_frame,
                          "0 0 0 0 0 0");
  if (!parse_xyzYPR(mocap_wrt_parent_frame, bodyParams_.T_PM)) {
    ROS_ERROR_STREAM("rosparam 'mocap_wrt_parent_frame' expected format "
                     "'x y z Y P R' but got '"
                     << mocap_wrt_parent_frame << "'");
    return false;
  }

  // allow a hidden transformation of ROS body w.r.t mocap body
  std::string body_wrt_mocap_body;
  nhp_.param<std::string>("body_wrt_mocap_body", body_wrt_mocap_body,
                          "0 0 0 0 0 0");
  if (!parse_xyzYPR(body_wrt_mocap_body, bodyParams_.T_MbB)) {
    ROS_ERROR_STREAM("rosparam 'body_wrt_mocap_body' expected format "
                     "'x y z Y P R' but got '"
                     << body_wrt_mocap_body << "'");
    return false;
  }

  nhp_.param<bool>("should_pub_unlabeledmarkers", should_pub_unlabeledmarkers_,
                   false);
  nhp_.param<bool>("should_pub_cameras", should_pub_cameras_, false);

  if (should_pub_unlabeledmarkers_) {
    pub_unlabeledmarkers_ =
        nhp_.advertise<::mocap::Markers>("unlabeled_markers", 1);
  }

  if (should_pub_cameras_) {
    pub_cameras_ = nhp_.advertise<::mocap::Cameras>("cameras", 1);
  }

  //
  // Client specific parameters and initialization
  //

  std::string client;
  nhp_.param<std::string>("client", client, "vicon");

  // make the specified client lowercase
  std::transform(client.begin(), client.end(), client.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  if (client == "vicon") {

    VICON::Parameters params;
    nhp_.param<std::string>("host", params.host, "192.168.0.9:801");

    client_.reset(new VICON(params));

  } else if (client == "optitrack") {

    OptiTrack::Parameters params;
    nhp_.param<std::string>("local", params.localIP, "192.168.1.119");
    nhp_.param<std::string>("server", params.serverIP, "192.168.1.12");
    nhp_.param<std::string>("multicast_group", params.multicastIP,
                            "239.255.42.99");
    nhp_.param<int>("command_port", params.commandPort, 1510);
    nhp_.param<int>("data_port", params.dataPort, 1511);

    client_.reset(new OptiTrack(params));

  } else {
    ROS_ERROR_STREAM("Invalid mocap client '" << client << "'.");
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

bool Mocap::parse_xyzYPR(const std::string &xyzYPR, tf2::Transform &T) const {
  constexpr int expectedValues = 6; // x y z Y P R
  constexpr char delim = ' ';

  std::vector<double> values;
  std::stringstream ss(xyzYPR);

  // attempt to parse the string
  try {
    std::string tmp;
    while (std::getline(ss, tmp, delim)) {
      values.push_back(std::stod(tmp));
    }
  } catch (...) {
    return false;
  }

  if (values.size() != expectedValues) {
    return false;
  }

  // input: body YPR (intrinsic 3-2-1)
  // tf2: fixed RPY (extrinsic 1-2-3)

  // create transform
  tf2::Vector3 p(values[0], values[1], values[2]);
  tf2::Quaternion q;
  q.setRPY(values[5], values[4], values[3]);
  T.setOrigin(p);
  T.setRotation(q);

  return true;
}

// ----------------------------------------------------------------------------

void Mocap::screenPrint() const {
  const double latency = client_->getTotalLatency();

  ROS_INFO_STREAM_THROTTLE(
      10, "Receiving " << client_->getName() << " data at " << 1 / mocap_dt_
                       << " Hz (capture latency of " << std::fixed
                       << std::setprecision(2) << latency * 1e3 << " ms)");
}

// ----------------------------------------------------------------------------

void Mocap::broadcastROS_cameras() {
  static bool camerasmsg_initialized = false;
  std::vector<Camera> cameras = client_->getCameras();

  static ::mocap::Cameras camerasmsg;
  if (!camerasmsg_initialized) {

    const std::vector<Camera> calibs = rpc_client_->getCameras();

    for (const auto &camera : cameras) {
      ::mocap::Camera cameramsg;
      cameramsg.name = camera.name;
      cameramsg.type = camera.type;
      cameramsg.deviceid = camera.deviceid;

      //
      // Camera Calibration info
      //

      auto it = std::find_if(calibs.begin(), calibs.end(),
                             [&id = camera.deviceid](const Camera &cam) {
                               return id == cam.deviceid;
                             });
      if (it != calibs.end()) {
        const Camera &cam = *it;

        cameramsg.enabled = cam.enabled;
        cameramsg.index = cam.index;
        cameramsg.image_error = cam.image_error;
        tf2::convert(cam.position, cameramsg.pose.position);
        tf2::convert(cam.orientation, cameramsg.pose.orientation);

        // camera w.r.t mocap
        tf2::Transform T_MC;
        tf2::convert(cameramsg.pose, T_MC);

        // camera w.t.y parent
        const tf2::Transform T_PC = bodyParams_.T_PM * T_MC;
        tf2::convert(T_PC, cameramsg.pose);

        sensor_msgs::CameraInfo cinfo;
        cinfo.width = cam.image_width;
        cinfo.height = cam.image_height;
        cinfo.distortion_model =
            "vicon_radial"; // not sure what this is exactly
        cinfo.D.push_back(cam.radial_distortion(0));
        cinfo.D.push_back(cam.radial_distortion(1));
        cinfo.K[0] = cinfo.K[4] = cam.focal_length;
        cinfo.K[2] = cam.principal_point.x();
        cinfo.K[5] = cam.principal_point.y();
        cinfo.K[8] = 1;
        cameramsg.camera_info = cinfo;
      }

      camerasmsg.cameras.push_back(cameramsg);
    }

    camerasmsg_initialized = true;
  }

  // Use timestamp from mocap server
  ros::Time timestamp;
  timestamp.fromNSec(client_->getEstimatedTimeNsOffset() +
                     client_->getCurrentTimestampNs());

  camerasmsg.header.stamp = timestamp;
  camerasmsg.header.frame_id = bodyParams_.parent_frame;

  for (size_t i = 0; i < cameras.size(); ++i) {
    const Camera &camera = cameras[i];
    ::mocap::Camera &cameramsg = camerasmsg.cameras[i];

    // assumption: client_->getCameras() always returns a list of cameras
    //             in the same order
    assert(cameramsg.deviceid == camera.deviceid);

    cameramsg.camera_info.header = camerasmsg.header;
    cameramsg.centroids.clear();
    for (const auto &centroid : camera.centroids) {
      ::mocap::Centroid centroidmsg;
      centroidmsg.u = centroid.u;
      centroidmsg.v = centroid.v;
      centroidmsg.radius = centroid.radius;
      cameramsg.centroids.push_back(centroidmsg);
    }
  }

  pub_cameras_.publish(camerasmsg);
}

// ----------------------------------------------------------------------------

void Mocap::broadcastROS_unlabeled_markers() {
  std::vector<Marker> markers = client_->getUnlabeledMarkers();

  // Use timestamp from mocap server
  ros::Time timestamp;
  timestamp.fromNSec(client_->getEstimatedTimeNsOffset() +
                     client_->getCurrentTimestampNs());

  // build the markers msg
  ::mocap::Markers markersmsg;
  markersmsg.header.stamp = timestamp;
  markersmsg.header.frame_id = bodyParams_.parent_frame;
  for (const auto &marker : markers) {
    ::mocap::Marker markermsg;
    markermsg.name = marker.name;
    markermsg.position.x = marker.x;
    markermsg.position.y = marker.y;
    markermsg.position.z = marker.z;

    // mocap marker dot w.r.t mocap origin
    tf2::Transform T_Mm;
    tf2::convert(markermsg.position, T_Mm);

    // body marker w.r.t parent -- i.e., transform into ROS frame
    const tf2::Transform T_PBm = bodyParams_.T_PM * T_Mm * bodyParams_.T_MbB;
    tf2::convert(T_PBm, markermsg.position);

    markersmsg.markers.push_back(markermsg);
  }

  pub_unlabeledmarkers_.publish(markersmsg);
}

} // namespace mocap
} // namespace acl
