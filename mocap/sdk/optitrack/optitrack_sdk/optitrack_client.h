/**
 * MIT License
 *
 * Copyright (c) 2018 AgileDrones
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Originally from: https://github.com/mit-fast/OptiTrack-Motive-2-Client
 * @editedby Parker Lusk <parkerclusk@gmail.com>
 */

#ifndef MOTIONCAPTURECLIENTFRAMEWORK_H
#define MOTIONCAPTURECLIENTFRAMEWORK_H

#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include "optitrack_sdk/udp_socket.h"

#define MAX_NAMELENGTH 256
#define MAX_ANALOG_CHANNELS 32

// NATNET message ids
#define NAT_CONNECT 0
#define NAT_SERVERINFO 1
#define NAT_REQUEST 2
#define NAT_RESPONSE 3
#define NAT_REQUEST_MODELDEF 4
#define NAT_MODELDEF 5
#define NAT_REQUEST_FRAMEOFDATA 6
#define NAT_FRAMEOFDATA 7
#define NAT_MESSAGESTRING 8
#define NAT_DISCONNECT 9
#define NAT_KEEPALIVE 10
#define NAT_UNRECOGNIZED_REQUEST 100
#define UNDEFINED 999999.9999

#define MAX_PACKETSIZE                                                         \
  100000 // max size of packet (actual packet size is dynamic)

typedef struct {
  char szName[MAX_NAMELENGTH]; // sending app's name
  uint8_t Version[4];          // [major.minor.build.revision]
  uint8_t NatNetVersion[4];    // [major.minor.build.revision]
} sSender;

typedef struct sSender_Server {
  sSender Common;
  // host's high resolution clock frequency (ticks per second)
  uint64_t HighResClockFrequency;
  uint16_t DataPort;
  bool IsMulticast;
  uint8_t MulticastGroupAddress[4];
} sSender_Server;

typedef struct {
  uint16_t iMessage;   // message ID (e.g. NAT_FRAMEOFDATA)
  uint16_t nDataBytes; // Num bytes in payload

  // NOTE: Adding sSender_Server in the union causes the ptr to Data.Sender to
  // be 4 bytes later due to alignment. Removing it allows casting Sender to
  // SenderServer and accessing the fields directly.
  union {
    uint8_t cData[MAX_PACKETSIZE];
    char szData[MAX_PACKETSIZE];
    uint32_t lData[MAX_PACKETSIZE / sizeof(uint32_t)];
    float fData[MAX_PACKETSIZE / sizeof(float)];
    sSender Sender;
  } Data; // Payload incoming from NatNet Server
} sPacket;

struct Marker {
  int id;
  double x;
  double y;
  double z;
  double size;
  double residual;
};

struct Packet {
  int message_id;

  std::string model_name;
  int rigid_body_id;
  double pos[3];         // x, y, z
  double orientation[4]; // qx, qy, qz, qw
  std::vector<Marker> markers;
  std::vector<Marker> expected_markers;

  bool tracking_valid;
  float mean_marker_error;
  int labeled_marker_count;

  int frame_number;

  // NOTE: All are nanosecond timestamps
  uint64_t mid_exposure_timestamp;
  uint64_t camera_data_received_timestamp;
  uint64_t transmit_timestamp;
  uint64_t receive_timestamp; // measured on receive
};

class OptiTrackClient {
public:
  /**
   * @brief      Constructs the object.
   *
   * @param[in]  localIP           The local ip (for commands)
   * @param[in]  serverIP          The server ip (for commands)
   * @param[in]  multicastGroupIP  The multicast group ip (for rigid body data)
   * @param[in]  commandPort       The command port (for commands, server side)
   * @param[in]  dataPort          The multicast data port (for rigid body data,
   * client/server side)
   */
  OptiTrackClient(const std::string &localIP, const std::string &serverIP,
                  const std::string &multicastGroupIP, const int commandPort,
                  const int dataPort);

  /**
   * @brief      Initialize the sockets for communicating with Motive server
   *
   * @return     false if unsuccessful
   */
  bool initConnection();

  /**
   * @brief      Allow time for checking for command and rigid body data.
   *
   *             Three things happen:
   *                1) Check data socket for any rigid body packets
   *                2) Send a request via the command socket for all
   *                   model definitions (i.e., the model name)
   *                3) Wait (with timeout) to receive model descriptions.
   *
   * @return     false if unsuccessful (e.g., no packets received. This
   *             could indicate that the server is down or there is
   *             high network traffic.)
   */
  bool spinOnce();

  /**
   * @brief      Returns any packets received and processed after spinning.
   *
   * @return     The packets.
   */
  std::vector<Packet> getPackets() const { return processedPackets_; }

  /**
   * @brief      Returns the NatNet version number.
   *
   * @param      major  Major version
   * @param      minor  Minor version
   */
  void getVersion(int &major, int &minor);

  /**
   * @brief      Returns the latest server timestamp in nanoseconds.
   *             Note that this time is the time since software start.
   *
   * @return     Time in nanoseconds.
   */
  uint64_t getLatestServerTimeNs() const { return latest_server_timestamp_ns_; }

private:
  const std::string localIP_;     ///< IP addr of local NIC to use.
  const std::string serverIP_;    ///< IP addr of server (for commands)
  const std::string multicastIP_; ///< Multicast group (for UDP data)
  const int commandPort_;         ///< Port used for sending cmds to server
  const int dataPort_;            ///< Port used for multicast data from server

  std::unique_ptr<acl::utils::UDPSocket>
      datasock_; ///< UDP/Multicast socket for data
  std::unique_ptr<acl::utils::UDPSocket> cmdsock_; ///< UDP socket for commands

  sSender_Server
      serverInfo_; ///< The response of the inital connection request.

  /**
   * @brief      Requests (via command socket) server info
   *
   * @param      serverInfo  The server information
   *
   * @return     false if unsuccessful
   */
  bool getServerInfo(sSender_Server &serverInfo);

  /**
   * @brief      Gets the current time as seconds since the UNIX epoch.
   *
   * @return     The timestamp.
   */
  uint64_t getTimestamp();

  /**
   * @brief      Processes raw data into packets that can be easily handled
   *
   * @param      pData    The data received by a command / data socket
   * @param      outputs  Processes packets
   */
  char *Unpack(char *pData, std::vector<Packet> &outpus);

  std::vector<Packet> processedPackets_;

  uint64_t latest_server_timestamp_ns_ = 0; ///< time since software start, [ns]

  // Packet unpacking functions
  char *UnpackPacketHeader(char *ptr, int &messageID, int &nBytes,
                           int &nBytesTotal);
  // Frame data
  char *UnpackFrameData(char *inptr, int nBytes, int major, int minor,
                        uint64_t timestamp, int message_id,
                        std::vector<Packet> &outputs);
  char *UnpackFramePrefixData(char *ptr, int major, int minor,
                              int &frameNumber);
  char *UnpackMarkersetData(char *ptr, int major, int minor);
  char *UnpackRigidBodyData(char *ptr, int major, int minor, int frameNumber,
                            int message_id, uint64_t timestamp,
                            std::vector<Packet> &outputs);
  char *UnpackSkeletonData(char *ptr, int major, int minor);
  char *UnpackLabeledMarkerData(char *ptr, int major, int minor);
  char *UnpackForcePlateData(char *ptr, int major, int minor);
  char *UnpackDeviceData(char *ptr, int major, int minor);
  char *UnpackFrameSuffixData(char *ptr, int major, int minor,
                              std::vector<Packet> &outputs);
  // Descriptions
  char *UnpackDescription(char *inptr, int nBytes, int major, int minor,
                          std::vector<Packet> &outputs);
  char *UnpackMarkersetDescription(char *ptr, char *targetPtr, int major,
                                   int minor);
  char *UnpackRigidBodyDescription(
      char *ptr, char *targetPtr, int major, int minor,
      std::unordered_map<int, std::string> &rigidBodyMap,
      std::unordered_map<int, std::vector<Marker>> &markersMap);
  char *UnpackSkeletonDescription(char *ptr, char *targetPtr, int major,
                                  int minor);
  char *UnpackForcePlateDescription(char *ptr, char *targetPtr, int major,
                                    int minor);
  char *UnpackDeviceDescription(char *ptr, char *targetPtr, int major,
                                int minor);
  char *UnpackCameraDescription(char *ptr, char *targetPtr, int major,
                                int minor);
};

#endif // MOTIONCAPTURECLIENTFRAMEWORK_H
