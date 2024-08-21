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

#include "optitrack_sdk/optitrack_client.h"

OptiTrackClient::OptiTrackClient(const std::string &localIP,
                                 const std::string &serverIP,
                                 const std::string &multicastGroupIP,
                                 const int commandPort, const int dataPort)
    : localIP_(localIP), serverIP_(serverIP), multicastIP_(multicastGroupIP),
      commandPort_(commandPort), dataPort_(dataPort) {}

// ----------------------------------------------------------------------------

bool OptiTrackClient::initConnection() {

  try {
    cmdsock_.reset(new acl::utils::UDPSocket(localIP_, 0));
    cmdsock_->setReceiveTimeout(0, 500000); // 500 msec

    datasock_.reset(new acl::utils::UDPSocket(dataPort_));
    datasock_->setReceiveTimeout(0, 500000); // 500 msec
    datasock_->joinMulticastGroup(localIP_, multicastIP_);
  } catch (const std::exception &exc) {
    std::cerr << exc.what() << std::endl;
    return false;
  }

  // attempt to connect to the server to retrieve basic info
  return getServerInfo(serverInfo_);
}

// ----------------------------------------------------------------------------

bool OptiTrackClient::spinOnce() {
  // clear previous vector of processed packets
  processedPackets_.clear();

  // Receive one chunk of data (sRigidBodyData --- pose, mean error)
  {
    char data[MAX_PACKETSIZE];
    bool recvd = datasock_->receive(data, sizeof(data));

    if (recvd)
      Unpack(data, processedPackets_);
    else
      return false;
  }
  // Request current model descriptions from server (sRigidBodyDescription)
  {
    sPacket pkt{};
    pkt.iMessage = NAT_REQUEST_MODELDEF;
    pkt.nDataBytes = 0;
    const size_t pktlen = pkt.nDataBytes + 4;

    bool sent = cmdsock_->send(serverIP_, commandPort_, (char *)&pkt, pktlen);

    if (!sent)
      return false;
  }

  // Receive one chunk of command response (sRigidBodyDescription --- name)
  {
    char data[MAX_PACKETSIZE];

    bool recvd = cmdsock_->receive(data, sizeof(data));

    if (recvd)
      Unpack(data, processedPackets_);
    else
      return false;

    return true;
  }
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool OptiTrackClient::getServerInfo(sSender_Server &serverInfo) {
  constexpr int MAX_NUM_TRIES = 3;

  // attempt to send the connection request to the server nrTries times.
  int nrTries = MAX_NUM_TRIES;
  while (nrTries--) {

    //
    // send a message through the socket (non-blocking)
    //

    {
      // n.b.: the 4 is the size of the packet "header" (iMessage + nDataBytes)
      // and the nDataBytes is the actual size of the payload
      // create a packet with a connection request message
      sPacket pkt{};
      pkt.iMessage = NAT_CONNECT;
      pkt.nDataBytes = 0;
      const size_t pktlen = pkt.nDataBytes + 4;

      bool sent = cmdsock_->send(serverIP_, commandPort_, (char *)&pkt, pktlen);
      if (!sent)
        return false;
    }

    std::cout << "[OptiTrackClient] Attempting to connect "
                 "to OptiTrack server..."
              << std::flush;

    {
      sPacket pkt{};

      // wait (with timeout) for server response
      bool recvd = cmdsock_->receive((char *)&pkt, sizeof(pkt));

      if (!recvd) {
        std::cout << "timed out." << std::endl;
        continue;
      } else {
        std::cout << "done!" << std::endl;
      }

      if (pkt.iMessage != NAT_SERVERINFO) {
        std::cout << "[OptiTrackClient] Received unexpected command "
                     "response from server."
                  << std::endl;
        continue;
      }

      // Cast pkt.Data.Sender to sSender_Server
      serverInfo = (sSender_Server &)pkt.Data.Sender;
      // printf("Natnet version: %d.%d.%d.%d\n",
      //        serverInfo.Common.NatNetVersion[0],
      //        serverInfo.Common.NatNetVersion[1],
      //        serverInfo.Common.NatNetVersion[2],
      //        serverInfo.Common.NatNetVersion[3]);
    }

    return true;
  }

  // number of tries exceeded
  return false;
}

void OptiTrackClient::getVersion(int &major, int &minor) {
  major = serverInfo_.Common.NatNetVersion[0];
  minor = serverInfo_.Common.NatNetVersion[1];
}

uint64_t OptiTrackClient::getTimestamp() {
  return std::chrono::high_resolution_clock::now().time_since_epoch() /
         std::chrono::nanoseconds(1);
}

// Funtion that assigns a time code values to 5 variables passed as arguments
// Requires an integer from the packet as the timecode and timecodeSubframe
bool DecodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe,
                    int *hour, int *minute, int *second, int *frame,
                    int *subframe) {
  bool bValid = true;

  *hour = (inTimecode >> 24) & 255;
  *minute = (inTimecode >> 16) & 255;
  *second = (inTimecode >> 8) & 255;
  *frame = inTimecode & 255;
  *subframe = inTimecodeSubframe;

  return bValid;
}

// Takes timecode and assigns it to a string
bool TimecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe,
                       char *Buffer, int BufferSize) {
  bool bValid;
  int hour, minute, second, frame, subframe;
  bValid = DecodeTimecode(inTimecode, inTimecodeSubframe, &hour, &minute,
                          &second, &frame, &subframe);

  sprintf(Buffer, "%2d:%2d:%2d:%2d.%d", hour, minute, second, frame, subframe);
  for (unsigned int i = 0; i < strlen(Buffer); i++)
    if (Buffer[i] == ' ')
      Buffer[i] = '0';

  return bValid;
}

void DecodeMarkerID(int sourceID, int *pOutEntityID, int *pOutMemberID) {
  if (pOutEntityID)
    *pOutEntityID = sourceID >> 16;

  if (pOutMemberID)
    *pOutMemberID = sourceID & 0x0000ffff;
}

/*
 * MakeAlnum
 * For now, make sure the string is printable ascii.
 */
void MakeAlnum(char *szName, int len) {
  int i = 0, i_max = len;
  szName[len - 1] = 0;
  while ((i < len) && (szName[i] != 0)) {
    if (szName[i] == 0) {
      break;
    }
    if (isalnum(szName[i]) == 0) {
      szName[i] = ' ';
    }
    ++i;
  }
}

char *OptiTrackClient::UnpackDescription(char *inptr, int nBytes, int major,
                                         int minor,
                                         std::vector<Packet> &outputs) {
  char *ptr = inptr;
  char *targetPtr = ptr + nBytes;
  long long nBytesProcessed = (long long)ptr - (long long)inptr;

  // Used to convert rigid body ID to model name
  std::unordered_map<int, std::string> rigid_body_map;
  std::unordered_map<int, std::vector<Marker>> markers_map;

  // number of datasets
  int nDatasets = 0;
  memcpy(&nDatasets, ptr, 4);
  ptr += 4;
  // printf("Dataset Count : %d\n", nDatasets);
#ifdef VDEBUG
  int datasetCounts[7] = {0, 0, 0, 0, 0, 0, 0};
#endif
  bool errorDetected = false;
  for (int i = 0; i < nDatasets; i++) {
    // printf("Dataset %d\n", i);
#ifdef VDEBUG
    int nBytesUsed = (long long)ptr - (long long)inptr;
    int nBytesRemaining = nBytes - nBytesUsed;
    printf("Bytes Decoded: %d Bytes Remaining: %d)\n", nBytesUsed,
           nBytesRemaining);
#endif

    // Determine type and advance
    // The next type entry is inaccurate
    // if data descriptions are out of date
    int type = 0;
    memcpy(&type, ptr, 4);
    ptr += 4;
#ifdef VDEBUG
    if ((0 <= type) && (type <= 5)) {
      datasetCounts[type] += 1;
    } else {
      datasetCounts[6] += 1;
    }
#endif

    switch (type) {
    case 0: // Markerset
      // printf("Type: 0 Markerset\n");
      ptr = UnpackMarkersetDescription(ptr, targetPtr, major, minor);
      break;
    case 1: // rigid body
      // printf("Type: 1 Rigid Body\n");
      ptr = UnpackRigidBodyDescription(ptr, targetPtr, major, minor,
                                       rigid_body_map, markers_map);
      break;
    case 2: // skeleton
      // printf("Type: 2 Skeleton\n");
      ptr = UnpackSkeletonDescription(ptr, targetPtr, major, minor);
      break;
    case 3: // force plate
      // printf("Type: 3 Force Plate\n");
      ptr = UnpackForcePlateDescription(ptr, targetPtr, major, minor);
      break;
    case 4: // device
      // printf("Type: 4 Device\n");
      ptr = UnpackDeviceDescription(ptr, targetPtr, major, minor);
      break;
    case 5: // camera
      // printf("Type: 5 Camera\n");
      ptr = UnpackCameraDescription(ptr, targetPtr, major, minor);
      break;
    default: // unknown type
      printf("Type: %d UNKNOWN\n", type);
      printf("ERROR: Type decode failure\n");
      errorDetected = true;
      break;
    }
    if (errorDetected) {
      printf("ERROR: Stopping decode\n");
      break;
    }
    if (ptr > targetPtr) {
      printf("UnpackDescription: UNPACK ERROR DETECTED: STOPPING DECODE\n");
      return ptr;
    }
    // printf("\t%d datasets processed of %d\n", (i + 1), nDatasets);
    // printf("\t%lld bytes processed of %d\n",
    //       ((long long)ptr - (long long)inptr), nBytes);
  } // next dataset

  // Add names to rigid bodies
  for (int i = 0; i < outputs.size(); i++) {
    outputs[i].model_name = rigid_body_map[outputs[i].rigid_body_id];
    outputs[i].expected_markers = markers_map[outputs[i].rigid_body_id];
  }

  return ptr;
}

//
// UnpackMarkersetDescription
// (sMarkerSetDescription)
//
char *OptiTrackClient::UnpackMarkersetDescription(char *ptr, char *targetPtr,
                                                  int major, int minor) {
  // name
  char szName[MAX_NAMELENGTH];
  strcpy(szName, ptr);
  int nDataBytes = (int)strlen(szName) + 1;
  ptr += nDataBytes;
  MakeAlnum(szName, MAX_NAMELENGTH);
  // printf("Markerset Name: %s\n", szName);

  // marker data
  int nMarkers = 0;
  memcpy(&nMarkers, ptr, 4);
  ptr += 4;
  // printf("Marker Count : %d\n", nMarkers);

  for (int j = 0; j < nMarkers; j++) {
    char szName[MAX_NAMELENGTH];
    strcpy(szName, ptr);
    int nDataBytes = (int)strlen(ptr) + 1;
    ptr += nDataBytes;
    MakeAlnum(szName, MAX_NAMELENGTH);
    // printf("  %3.1d Marker Name: %s\n", j, szName);
    if (ptr > targetPtr) {
      // printf("UnpackMarkersetDescription: UNPACK ERROR DETECTED: STOPPING "
      //       "DECODE\n");
      return ptr;
    }
  }

  return ptr;
}

//
// UnpackRigidBodyDescription
// (sRigidBodyDescription)
//
char *OptiTrackClient::UnpackRigidBodyDescription(
    char *inptr, char *targetPtr, int major, int minor,
    std::unordered_map<int, std::string> &rigid_body_map,
    std::unordered_map<int, std::vector<Marker>> &markers_map) {
  char *ptr = inptr;
  int nBytes = 0; // common scratch variable
  char szName[MAX_NAMELENGTH];

  if ((major >= 2) || (major == 0)) {
    // RB name
    strcpy(szName, ptr);
    ptr += strlen(ptr) + 1;
    MakeAlnum(szName, MAX_NAMELENGTH);
    // printf("  Rigid Body Name: %s\n", szName);
  } else {
    // printf("  Rigid Body Name: <unknown>\n");
    return ptr;
  }

  int ID = 0;
  memcpy(&ID, ptr, 4);
  ptr += 4;
  // printf("  RigidBody ID   : %d\n", ID);

  rigid_body_map[ID] = szName;

  int parentID = 0;
  memcpy(&parentID, ptr, 4);
  ptr += 4;
  // printf("  Parent ID      : %d\n", parentID);

  // Offsets
  float xoffset = 0;
  memcpy(&xoffset, ptr, 4);
  ptr += 4;
  float yoffset = 0;
  memcpy(&yoffset, ptr, 4);
  ptr += 4;
  float zoffset = 0;
  memcpy(&zoffset, ptr, 4);
  ptr += 4;
  // printf("  Position       : %3.2f, %3.2f, %3.2f\n", xoffset, yoffset,
  // zoffset);

  if (ptr > targetPtr) {
    printf(
        "UnpackRigidBodyDescription: UNPACK ERROR DETECTED: STOPPING DECODE\n");
    return ptr;
  }

  if ((major >= 3) || (major == 0)) {
    int nMarkers = 0;
    memcpy(&nMarkers, ptr, 4);
    ptr += 4;
    // printf("  Number of Markers : %d\n", nMarkers);
    if (nMarkers > 16000) {
      int nBytesProcessed = (int)(targetPtr - ptr);
      printf("UnpackRigidBodyDescription: UNPACK ERROR DETECTED: STOPPING "
             "DECODE at %d processed\n",
             nBytesProcessed);
      printf("                           Unreasonable number of markers\n");
      return targetPtr + 4;
    }

    if (nMarkers > 0) {

      // printf("  Marker Positions:\n");
      char *ptr2 = ptr + (nMarkers * sizeof(float) * 3);
      char *ptr3 = ptr2 + (nMarkers * sizeof(int));
      for (int markerIdx = 0; markerIdx < nMarkers; ++markerIdx) {
        float xpos, ypos, zpos;
        int32_t label;
        char szMarkerNameUTF8[MAX_NAMELENGTH] = {0};
        char szMarkerName[MAX_NAMELENGTH] = {0};
        // marker positions
        memcpy(&xpos, ptr, 4);
        ptr += 4;
        memcpy(&ypos, ptr, 4);
        ptr += 4;
        memcpy(&zpos, ptr, 4);
        ptr += 4;

        // Marker Required activeLabels
        memcpy(&label, ptr2, 4);
        ptr2 += 4;

        // Marker Name
        szMarkerName[0] = 0;
        if ((major >= 4) || (major == 0)) {
          strcpy(szMarkerName, ptr3);
          ptr3 += strlen(ptr3) + 1;
        }

        Marker m;
        m.id = markerIdx;
        m.x = xpos;
        m.y = ypos;
        m.z = zpos;
        markers_map[ID].push_back(m);

        // printf("    %3.1d Marker Label: %3.1d Position: %6.6f %6.6f %6.6f
        // %s\n",
        //       markerIdx, label, xpos, ypos, zpos, szMarkerName);
        if (ptr3 > targetPtr) {
          printf("UnpackRigidBodyDescription: UNPACK ERROR DETECTED: STOPPING "
                 "DECODE\n");
          return ptr3;
        }
      }
      ptr = ptr3; // advance to the end of the labels & marker names
    }
  }

  if (ptr > targetPtr) {
    printf(
        "UnpackRigidBodyDescription: UNPACK ERROR DETECTED: STOPPING DECODE\n");
    return ptr;
  }
  // printf("UnpackRigidBodyDescription processed %lld bytes\n",
  //  ((long long)ptr - (long long)inptr));
  return ptr;
}

//
// UnpackSkeletonDescription
//
char *OptiTrackClient::UnpackSkeletonDescription(char *ptr, char *targetPtr,
                                                 int major, int minor) {
  char szName[MAX_NAMELENGTH];
  // Name
  strcpy(szName, ptr);
  ptr += strlen(ptr) + 1;
  MakeAlnum(szName, MAX_NAMELENGTH);
  // printf("Name: %s\n", szName);

  // ID
  int ID = 0;
  memcpy(&ID, ptr, 4);
  ptr += 4;
  // printf("ID : %d\n", ID);

  // # of RigidBodies
  int nRigidBodies = 0;
  memcpy(&nRigidBodies, ptr, 4);
  ptr += 4;
  // printf("RigidBody (Bone) Count : %d\n", nRigidBodies);

  if (ptr > targetPtr) {
    printf(
        "UnpackSkeletonDescription: UNPACK ERROR DETECTED: STOPPING DECODE\n");
    return ptr;
  }

  // We don't care about extracting the map out of skeleton data
  std::unordered_map<int, std::string> a_map;
  std::unordered_map<int, std::vector<Marker>> b_map;

  for (int i = 0; i < nRigidBodies; i++) {
    // printf("Rigid Body (Bone) %d:\n", i);
    ptr =
        UnpackRigidBodyDescription(ptr, targetPtr, major, minor, a_map, b_map);
    if (ptr > targetPtr) {
      printf("UnpackSkeletonDescription: UNPACK ERROR DETECTED: STOPPING "
             "DECODE\n");
      return ptr;
    }
  }
  return ptr;
}

char *OptiTrackClient::UnpackForcePlateDescription(char *ptr, char *targetPtr,
                                                   int major, int minor) {
  if ((major >= 3) || (major == 0)) {
    // ID
    int ID = 0;
    memcpy(&ID, ptr, 4);
    ptr += 4;
    // printf("ID : %d\n", ID);

    // Serial Number
    char strSerialNo[128];
    strcpy(strSerialNo, ptr);
    ptr += strlen(ptr) + 1;
    // printf("Serial Number : %s\n", strSerialNo);

    // Dimensions
    float fWidth = 0;
    memcpy(&fWidth, ptr, 4);
    ptr += 4;
    // printf("Width : %3.2f\n", fWidth);

    float fLength = 0;
    memcpy(&fLength, ptr, 4);
    ptr += 4;
    // printf("Length : %3.2f\n", fLength);

    // Origin
    float fOriginX = 0;
    memcpy(&fOriginX, ptr, 4);
    ptr += 4;
    float fOriginY = 0;
    memcpy(&fOriginY, ptr, 4);
    ptr += 4;
    float fOriginZ = 0;
    memcpy(&fOriginZ, ptr, 4);
    ptr += 4;
    // printf("Origin : %3.2f,  %3.2f,  %3.2f\n", fOriginX, fOriginY, fOriginZ);

    // Calibration Matrix
    const int kCalMatX = 12;
    const int kCalMatY = 12;
    float fCalMat[kCalMatX][kCalMatY];
    // printf("Cal Matrix\n");
    for (int calMatX = 0; calMatX < kCalMatX; ++calMatX) {
      // printf("  ");
      for (int calMatY = 0; calMatY < kCalMatY; ++calMatY) {
        memcpy(&fCalMat[calMatX][calMatY], ptr, 4);
        ptr += 4;
        // printf("%3.3e ", fCalMat[calMatX][calMatY]);
      }
      // printf("\n");
    }

    // Corners
    const int kCornerX = 4;
    const int kCornerY = 3;
    float fCorners[kCornerX][kCornerY] = {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    // printf("Corners\n");
    for (int cornerX = 0; cornerX < kCornerX; ++cornerX) {
      // printf("  ");
      for (int cornerY = 0; cornerY < kCornerY; ++cornerY) {
        memcpy(&fCorners[cornerX][cornerY], ptr, 4);
        ptr += 4;
        // printf("%3.3e ", fCorners[cornerX][cornerY]);
      }
      // printf("\n");
    }

    // Plate Type
    int iPlateType = 0;
    memcpy(&iPlateType, ptr, 4);
    ptr += 4;
    // printf("Plate Type : %d\n", iPlateType);

    // Channel Data Type
    int iChannelDataType = 0;
    memcpy(&iChannelDataType, ptr, 4);
    ptr += 4;
    // printf("Channel Data Type : %d\n", iChannelDataType);

    // Number of Channels
    int nChannels = 0;
    memcpy(&nChannels, ptr, 4);
    ptr += 4;
    // printf("  Number of Channels : %d\n", nChannels);
    if (ptr > targetPtr) {
      printf("UnpackSkeletonDescription: UNPACK ERROR DETECTED: STOPPING "
             "DECODE\n");
      return ptr;
    }

    for (int chNum = 0; chNum < nChannels; ++chNum) {
      char szName[MAX_NAMELENGTH];
      strcpy(szName, ptr);
      int nDataBytes = (int)strlen(szName) + 1;
      ptr += nDataBytes;
      // printf("    Channel Name %d: %s\n", chNum, szName);
      if (ptr > targetPtr) {
        printf("UnpackSkeletonDescription: UNPACK ERROR DETECTED: STOPPING "
               "DECODE\n");
        return ptr;
      }
    }
  }
  return ptr;
}

char *OptiTrackClient::UnpackDeviceDescription(char *ptr, char *targetPtr,
                                               int major, int minor) {
  if ((major >= 3) || (major == 0)) {
    int ID = 0;
    memcpy(&ID, ptr, 4);
    ptr += 4;
    // printf("ID : %d\n", ID);

    // Name
    char strName[128];
    strcpy(strName, ptr);
    ptr += strlen(ptr) + 1;
    // printf("Device Name :       %s\n", strName);

    // Serial Number
    char strSerialNo[128];
    strcpy(strSerialNo, ptr);
    ptr += strlen(ptr) + 1;
    // printf("Serial Number :     %s\n", strSerialNo);

    int iDeviceType = 0;
    memcpy(&iDeviceType, ptr, 4);
    ptr += 4;
    // printf("Device Type :        %d\n", iDeviceType);

    int iChannelDataType = 0;
    memcpy(&iChannelDataType, ptr, 4);
    ptr += 4;
    // printf("Channel Data Type : %d\n", iChannelDataType);

    int nChannels = 0;
    memcpy(&nChannels, ptr, 4);
    ptr += 4;
    // printf("Number of Channels : %d\n", nChannels);
    char szChannelName[MAX_NAMELENGTH];

    if (ptr > targetPtr) {
      printf(
          "UnpackDeviceDescription: UNPACK ERROR DETECTED: STOPPING DECODE\n");
      return ptr;
    }

    for (int chNum = 0; chNum < nChannels; ++chNum) {
      strcpy(szChannelName, ptr);
      ptr += strlen(ptr) + 1;
      // printf("  Channel Name %d:     %s\n", chNum, szChannelName);
      if (ptr > targetPtr) {
        printf("UnpackDeviceDescription: UNPACK ERROR DETECTED: STOPPING "
               "DECODE\n");
        return ptr;
      }
    }
  }

  return ptr;
}

//
// UnpackCameraDescription
//
char *OptiTrackClient::UnpackCameraDescription(char *ptr, char *targetPtr,
                                               int major, int minor) {

  // Name
  char szName[MAX_NAMELENGTH];
  strcpy(szName, ptr);
  ptr += strlen(ptr) + 1;
  MakeAlnum(szName, MAX_NAMELENGTH);
  // printf("Camera Name  : %s\n", szName);

  // Pos
  float cameraPosition[3];
  memcpy(cameraPosition + 0, ptr, 4);
  ptr += 4;
  memcpy(cameraPosition + 1, ptr, 4);
  ptr += 4;
  memcpy(cameraPosition + 2, ptr, 4);
  ptr += 4;
  // printf("  Position   : %3.2f, %3.2f, %3.2f\n", cameraPosition[0],
  //       cameraPosition[1], cameraPosition[2]);

  // Ori
  float cameraOriQuat[4]; // x, y, z, w
  memcpy(cameraOriQuat + 0, ptr, 4);
  ptr += 4;
  memcpy(cameraOriQuat + 1, ptr, 4);
  ptr += 4;
  memcpy(cameraOriQuat + 2, ptr, 4);
  ptr += 4;
  memcpy(cameraOriQuat + 3, ptr, 4);
  ptr += 4;
  // printf("  Orientation: %3.2f, %3.2f, %3.2f, %3.2f\n", cameraOriQuat[0],
  //        cameraOriQuat[1], cameraOriQuat[2], cameraOriQuat[3]);

  return ptr;
}

char *OptiTrackClient::UnpackFrameData(char *inptr, int nBytes, int major,
                                       int minor, uint64_t timestamp,
                                       int message_id,
                                       std::vector<Packet> &outputs) {
  char *ptr = inptr;
  // printf("MoCap Frame Begin\n-----------------\n");
  int frameNumber = 0;
  ptr = UnpackFramePrefixData(ptr, major, minor, frameNumber);

  ptr = UnpackMarkersetData(ptr, major, minor);

  ptr = UnpackRigidBodyData(ptr, major, minor, frameNumber, message_id,
                            timestamp, outputs);

  ptr = UnpackSkeletonData(ptr, major, minor);

  ptr = UnpackLabeledMarkerData(ptr, major, minor);

  ptr = UnpackForcePlateData(ptr, major, minor);

  ptr = UnpackDeviceData(ptr, major, minor);

  ptr = UnpackFrameSuffixData(ptr, major, minor, outputs);
  // printf("MoCap Frame End\n---------------- - \n");
  return ptr;
}

char *OptiTrackClient::UnpackFramePrefixData(char *ptr, int major, int minor,
                                             int &frame_number) {
  // Next 4 Bytes is the frame number
  int frameNumber = 0;
  memcpy(&frameNumber, ptr, 4);
  ptr += 4;
  // printf("Frame # : %d\n", frameNumber);
  frame_number = frameNumber;
  return ptr;
}

char *OptiTrackClient::UnpackMarkersetData(char *ptr, int major, int minor) {
  // First 4 Bytes is the number of data sets (markersets, rigidbodies, etc)
  int nMarkerSets = 0;
  memcpy(&nMarkerSets, ptr, 4);
  ptr += 4;
  // printf("Marker Set Count : %3.1d\n", nMarkerSets);

  // Loop through number of marker sets and get name and data
  for (int i = 0; i < nMarkerSets; i++) {
    // Markerset name
    char szName[MAX_NAMELENGTH];
    strcpy(szName, ptr);
    int nDataBytes = (int)strlen(szName) + 1;
    ptr += nDataBytes;
    MakeAlnum(szName, MAX_NAMELENGTH);
    // printf("Model Name       : %s\n", szName);

    // marker data
    int nMarkers = 0;
    memcpy(&nMarkers, ptr, 4);
    ptr += 4;
    // printf("Marker Count     : %3.1d\n", nMarkers);

    for (int j = 0; j < nMarkers; j++) {
      float x = 0;
      memcpy(&x, ptr, 4);
      ptr += 4;
      float y = 0;
      memcpy(&y, ptr, 4);
      ptr += 4;
      float z = 0;
      memcpy(&z, ptr, 4);
      ptr += 4;
      // printf("  Marker %3.1d : [x=%3.2f,y=%3.2f,z=%3.2f]\n", j, x, y, z);
    }
  }

  // Loop through unlabeled markers
  int nOtherMarkers = 0;
  memcpy(&nOtherMarkers, ptr, 4);
  ptr += 4;
  // OtherMarker list is Deprecated
  // printf("Unlabeled Markers Count : %d\n", nOtherMarkers);
  for (int j = 0; j < nOtherMarkers; j++) {
    float x = 0.0f;
    memcpy(&x, ptr, 4);
    ptr += 4;
    float y = 0.0f;
    memcpy(&y, ptr, 4);
    ptr += 4;
    float z = 0.0f;
    memcpy(&z, ptr, 4);
    ptr += 4;

    // Deprecated
    // printf("  Marker %3.1d : [x=%3.2f,y=%3.2f,z=%3.2f]\n", j, x, y, z);
  }

  return ptr;
}

char *OptiTrackClient::UnpackRigidBodyData(char *ptr, int major, int minor,
                                           int frameNumber, int message_id,
                                           uint64_t timestamp,
                                           std::vector<Packet> &outputs) {
  // Loop through rigidbodies
  int nRigidBodies = 0;
  memcpy(&nRigidBodies, ptr, 4);
  ptr += 4;
  // printf("Rigid Body Count : %3.1d\n", nRigidBodies);
  for (int j = 0; j < nRigidBodies; j++) {
    // Rigid body position and orientation
    int ID = 0;
    memcpy(&ID, ptr, 4);
    ptr += 4;
    float x = 0.0f;
    memcpy(&x, ptr, 4);
    ptr += 4;
    float y = 0.0f;
    memcpy(&y, ptr, 4);
    ptr += 4;
    float z = 0.0f;
    memcpy(&z, ptr, 4);
    ptr += 4;
    float qx = 0;
    memcpy(&qx, ptr, 4);
    ptr += 4;
    float qy = 0;
    memcpy(&qy, ptr, 4);
    ptr += 4;
    float qz = 0;
    memcpy(&qz, ptr, 4);
    ptr += 4;
    float qw = 0;
    memcpy(&qw, ptr, 4);
    ptr += 4;
    // printf("  RB: %3.1d ID : %3.1d\n", j, ID);
    // printf("    pos: [%3.2f,%3.2f,%3.2f]\n", x, y, z);
    // printf("    ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx, qy, qz, qw);

    Packet out_pkt;
    out_pkt.receive_timestamp = timestamp;
    out_pkt.message_id = message_id;
    out_pkt.frame_number = frameNumber;
    out_pkt.rigid_body_id = ID;
    out_pkt.pos[0] = x;
    out_pkt.pos[1] = y;
    out_pkt.pos[2] = z;
    out_pkt.orientation[0] = qx;
    out_pkt.orientation[1] = qy;
    out_pkt.orientation[2] = qz;
    out_pkt.orientation[3] = qw;
    // Marker positions removed as redundant (since they can be derived from RB
    // Pos/Ori plus initial offset) in NatNet 3.0 and later to optimize packet
    // size
    if (major < 3) {
      // Associated marker positions
      int nRigidMarkers = 0;
      memcpy(&nRigidMarkers, ptr, 4);
      ptr += 4;
      // printf("Marker Count: %d\n", nRigidMarkers);
      int nBytes = nRigidMarkers * 3 * sizeof(float);
      float *markerData = (float *)malloc(nBytes);
      memcpy(markerData, ptr, nBytes);
      ptr += nBytes;

      // NatNet Version 2.0 and later
      if (major >= 2) {
        // Associated marker IDs
        nBytes = nRigidMarkers * sizeof(int);
        int *markerIDs = (int *)malloc(nBytes);
        memcpy(markerIDs, ptr, nBytes);
        ptr += nBytes;

        // Associated marker sizes
        nBytes = nRigidMarkers * sizeof(float);
        float *markerSizes = (float *)malloc(nBytes);
        memcpy(markerSizes, ptr, nBytes);
        ptr += nBytes;

        for (int k = 0; k < nRigidMarkers; k++) {
          // printf("  Marker %d: id=%d  size=%3.1f  pos=[%3.2f,%3.2f,%3.2f]\n",
          // k,
          //  markerIDs[k], markerSizes[k], markerData[k * 3],
          //  markerData[k * 3 + 1], markerData[k * 3 + 2]);
        }

        if (markerIDs)
          free(markerIDs);
        if (markerSizes)
          free(markerSizes);

      }
      // Print marker positions for all rigid bodies
      else {
        int k3;
        for (int k = 0; k < nRigidMarkers; k++) {
          k3 = k * 3;
          // printf("  Marker %d: pos = [%3.2f,%3.2f,%3.2f]\n", k,
          // markerData[k3],
          //  markerData[k3 + 1], markerData[k3 + 2]);
        }
      }

      if (markerData)
        free(markerData);
    }

    // NatNet version 2.0 and later
    if ((major >= 2) || (major == 0)) {
      // Mean marker error
      float fError = 0.0f;
      memcpy(&fError, ptr, 4);
      ptr += 4;
      // printf("    Mean marker err: %3.2f\n", fError);
      out_pkt.mean_marker_error = fError;
    }

    // NatNet version 2.6 and later
    if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0)) {
      // params
      short params = 0;
      memcpy(&params, ptr, 2);
      ptr += 2;
      bool bTrackingValid =
          params &
          0x01; // 0x01 : rigid body was successfully tracked in this frame
      // printf("    Tracking Valid: %s\n", (bTrackingValid) ? "True" :
      // "False");
      out_pkt.tracking_valid = bTrackingValid;
    }

    outputs.push_back(out_pkt);
  } // Go to next rigid body

  return ptr;
}

char *OptiTrackClient::UnpackSkeletonData(char *ptr, int major, int minor) {
  // Skeletons (NatNet version 2.1 and later)
  if (((major == 2) && (minor > 0)) || (major > 2)) {
    int nSkeletons = 0;
    memcpy(&nSkeletons, ptr, 4);
    ptr += 4;
    // printf("Skeleton Count : %d\n", nSkeletons);

    // Loop through skeletons
    for (int j = 0; j < nSkeletons; j++) {
      // skeleton id
      int skeletonID = 0;
      memcpy(&skeletonID, ptr, 4);
      ptr += 4;
      // printf("  Skeleton %d ID=%d : BEGIN\n", j, skeletonID);

      // Number of rigid bodies (bones) in skeleton
      int nRigidBodies = 0;
      memcpy(&nRigidBodies, ptr, 4);
      ptr += 4;
      // printf("  Rigid Body Count : %d\n", nRigidBodies);

      // Loop through rigid bodies (bones) in skeleton
      for (int k = 0; k < nRigidBodies; k++) {
        // Rigid body position and orientation
        int ID = 0;
        memcpy(&ID, ptr, 4);
        ptr += 4;
        float x = 0.0f;
        memcpy(&x, ptr, 4);
        ptr += 4;
        float y = 0.0f;
        memcpy(&y, ptr, 4);
        ptr += 4;
        float z = 0.0f;
        memcpy(&z, ptr, 4);
        ptr += 4;
        float qx = 0;
        memcpy(&qx, ptr, 4);
        ptr += 4;
        float qy = 0;
        memcpy(&qy, ptr, 4);
        ptr += 4;
        float qz = 0;
        memcpy(&qz, ptr, 4);
        ptr += 4;
        float qw = 0;
        memcpy(&qw, ptr, 4);
        ptr += 4;
        // printf("    RB: %3.1d ID : %3.1d\n", k, ID);
        // printf("      pos: [%3.2f,%3.2f,%3.2f]\n", x, y, z);
        // printf("      ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx, qy, qz, qw);

        // Mean marker error (NatNet version 2.0 and later)
        if (major >= 2) {
          float fError = 0.0f;
          memcpy(&fError, ptr, 4);
          ptr += 4;
          // printf("    Mean marker error: %3.2f\n", fError);
        }

        // Tracking flags (NatNet version 2.6 and later)
        if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0)) {
          // params
          short params = 0;
          memcpy(&params, ptr, 2);
          ptr += 2;
          bool bTrackingValid =
              params &
              0x01; // 0x01 : rigid body was successfully tracked in this frame
        }
      }             // next rigid body
      // printf("  Skeleton %d ID=%d : END\n", j, skeletonID);

    } // next skeleton
  }

  return ptr;
}

char *OptiTrackClient::UnpackLabeledMarkerData(char *ptr, int major,
                                               int minor) {
  // labeled markers (NatNet version 2.3 and later)
  // labeled markers - this includes all markers: Active, Passive, and
  // 'unlabeled' (markers with no asset but a PointCloud ID)
  if (((major == 2) && (minor >= 3)) || (major > 2)) {
    int nLabeledMarkers = 0;
    memcpy(&nLabeledMarkers, ptr, 4);
    ptr += 4;
    // printf("Labeled Marker Count : %d\n", nLabeledMarkers);

    // Loop through labeled markers
    for (int j = 0; j < nLabeledMarkers; j++) {
      // id
      // Marker ID Scheme:
      // Active Markers:
      //   ID = ActiveID, correlates to RB ActiveLabels list
      // Passive Markers:
      //   If Asset with Legacy Labels
      //      AssetID 	(Hi Word)
      //      MemberID	(Lo Word)
      //   Else
      //      PointCloud ID
      int ID = 0;
      memcpy(&ID, ptr, 4);
      ptr += 4;
      int modelID, markerID;
      DecodeMarkerID(ID, &modelID, &markerID);

      // x
      float x = 0.0f;
      memcpy(&x, ptr, 4);
      ptr += 4;
      // y
      float y = 0.0f;
      memcpy(&y, ptr, 4);
      ptr += 4;
      // z
      float z = 0.0f;
      memcpy(&z, ptr, 4);
      ptr += 4;
      // size
      float size = 0.0f;
      memcpy(&size, ptr, 4);
      ptr += 4;

      // NatNet version 2.6 and later
      if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0)) {
        // marker params
        short params = 0;
        memcpy(&params, ptr, 2);
        ptr += 2;
        bool bOccluded = (params & 0x01) !=
                         0; // marker was not visible (occluded) in this frame
        bool bPCSolved =
            (params & 0x02) != 0; // position provided by point cloud solve
        bool bModelSolved =
            (params & 0x04) != 0; // position provided by model solve
        if ((major >= 3) || (major == 0)) {
          bool bHasModel =
              (params & 0x08) !=
              0; // marker has an associated asset in the data stream
          bool bUnlabeled =
              (params & 0x10) !=
              0; // marker is 'unlabeled', but has a point cloud ID
          bool bActiveMarker =
              (params & 0x20) != 0; // marker is an actively labeled LED marker
        }
      }

      // NatNet version 3.0 and later
      float residual = 0.0f;
      if ((major >= 3) || (major == 0)) {
        // Marker residual
        memcpy(&residual, ptr, 4);
        ptr += 4;
      }

      // printf("  ID  : [MarkerID: %d] [ModelID: %d]\n", markerID, modelID);
      // printf("    pos : [%3.2f,%3.2f,%3.2f]\n", x, y, z);
      // printf("    size: [%3.2f]\n", size);
      // printf("    err:  [%3.2f]\n", residual);
    }
  }
  return ptr;
}

char *OptiTrackClient::UnpackForcePlateData(char *ptr, int major, int minor) {
  // Force Plate data (NatNet version 2.9 and later)
  if (((major == 2) && (minor >= 9)) || (major > 2)) {
    int nForcePlates;
    const int kNFramesShowMax = 4;
    memcpy(&nForcePlates, ptr, 4);
    ptr += 4;
    // printf("Force Plate Count: %d\n", nForcePlates);
    for (int iForcePlate = 0; iForcePlate < nForcePlates; iForcePlate++) {
      // ID
      int ID = 0;
      memcpy(&ID, ptr, 4);
      ptr += 4;

      // Channel Count
      int nChannels = 0;
      memcpy(&nChannels, ptr, 4);
      ptr += 4;

      // printf("Force Plate %3.1d ID: %3.1d Num Channels: %3.1d\n",
      // iForcePlate,
      //  ID, nChannels);

      // Channel Data
      for (int i = 0; i < nChannels; i++) {
        // printf("  Channel %d : ", i);
        int nFrames = 0;
        memcpy(&nFrames, ptr, 4);
        ptr += 4;
        // printf("  %3.1d Frames - Frame Data: ", nFrames);

        // Force plate frames
        int nFramesShow = std::min(nFrames, kNFramesShowMax);
        for (int j = 0; j < nFrames; j++) {
          float val = 0.0f;
          memcpy(&val, ptr, 4);
          ptr += 4;
          // if (j < nFramesShow)
          // printf("%3.2f   ", val);
        }
        if (nFramesShow < nFrames) {
          // printf(" showing %3.1d of %3.1d frames", nFramesShow, nFrames);
        }
        // printf("\n");
      }
    }
  }

  return ptr;
}

char *OptiTrackClient::UnpackDeviceData(char *ptr, int major, int minor) {
  // Device data (NatNet version 3.0 and later)
  if (((major == 2) && (minor >= 11)) || (major > 2)) {
    const int kNFramesShowMax = 4;
    int nDevices;
    memcpy(&nDevices, ptr, 4);
    ptr += 4;
    // printf("Device Count: %d\n", nDevices);
    for (int iDevice = 0; iDevice < nDevices; iDevice++) {
      // ID
      int ID = 0;
      memcpy(&ID, ptr, 4);
      ptr += 4;

      // Channel Count
      int nChannels = 0;
      memcpy(&nChannels, ptr, 4);
      ptr += 4;

      // printf("Device %3.1d      ID: %3.1d Num Channels: %3.1d\n", iDevice,
      // ID,
      //  nChannels);

      // Channel Data
      for (int i = 0; i < nChannels; i++) {
        // printf("  Channel %d : ", i);
        int nFrames = 0;
        memcpy(&nFrames, ptr, 4);
        ptr += 4;
        // printf("  %3.1d Frames - Frame Data: ", nFrames);
        // Device frames
        int nFramesShow = std::min(nFrames, kNFramesShowMax);
        for (int j = 0; j < nFrames; j++) {
          float val = 0.0f;
          memcpy(&val, ptr, 4);
          ptr += 4;
          // if (j < nFramesShow)
          // printf("%3.2f   ", val);
        }
        if (nFramesShow < nFrames) {
          // printf(" showing %3.1d of %3.1d frames", nFramesShow,
          // nFrames);
        }
        // printf("\n");
      }
    }
  }

  return ptr;
}

char *OptiTrackClient::UnpackFrameSuffixData(char *ptr, int major, int minor,
                                             std::vector<Packet> &outputs) {

  // software latency (removed in version 3.0)
  if (major < 3) {
    float softwareLatency = 0.0f;
    memcpy(&softwareLatency, ptr, 4);
    ptr += 4;
    printf("software latency : %3.3f\n", softwareLatency);
  }

  // timecode
  unsigned int timecode = 0;
  memcpy(&timecode, ptr, 4);
  ptr += 4;
  unsigned int timecodeSub = 0;
  memcpy(&timecodeSub, ptr, 4);
  ptr += 4;
  char szTimecode[128] = "";
  TimecodeStringify(timecode, timecodeSub, szTimecode, 128);

  // timestamp
  double timestamp = 0.0f;

  // NatNet version 2.7 and later - increased from single to double precision
  if (((major == 2) && (minor >= 7)) || (major > 2)) {
    memcpy(&timestamp, ptr, 8);
    ptr += 8;
  } else {
    float fTemp = 0.0f;
    memcpy(&fTemp, ptr, 4);
    ptr += 4;
    timestamp = (double)fTemp;
  }
  // printf("Timestamp : %3.3f\n", timestamp);
  // Convert to nanoseconds: timestamp since software start
  latest_server_timestamp_ns_ = timestamp * 1e9;

  // high res timestamps (version 3.0 and later)
  if ((major >= 3) || (major == 0)) {
    uint64_t cameraMidExposureTimestamp = 0;
    memcpy(&cameraMidExposureTimestamp, ptr, 8);
    ptr += 8;
    // printf("Mid-exposure timestamp         : %lu\n",
    //  cameraMidExposureTimestamp);

    uint64_t cameraDataReceivedTimestamp = 0;
    memcpy(&cameraDataReceivedTimestamp, ptr, 8);
    ptr += 8;
    // printf("Camera data received timestamp : %lu\n",
    //        cameraDataReceivedTimestamp);

    uint64_t transmitTimestamp = 0;
    memcpy(&transmitTimestamp, ptr, 8);
    ptr += 8;
    // printf("Transmit timestamp             : %lu\n", transmitTimestamp);

    // Convert timestamps to nanoseconds and save them in the output packets
    const uint64_t server_frequency = serverInfo_.HighResClockFrequency;
    for (int i = 0; i < outputs.size(); i++) {
      outputs[i].mid_exposure_timestamp =
          (cameraMidExposureTimestamp * 1e9) / server_frequency;
      outputs[i].camera_data_received_timestamp =
          (cameraDataReceivedTimestamp * 1e9) / server_frequency;
      outputs[i].transmit_timestamp =
          (transmitTimestamp * 1e9) / server_frequency;
    }
  }

  // frame params
  short params = 0;
  memcpy(&params, ptr, 2);
  ptr += 2;
  bool bIsRecording = (params & 0x01) != 0; // 0x01 Motive is recording
  bool bTrackedModelsChanged =
      (params & 0x02) != 0; // 0x02 Actively tracked model list has changed

  // end of data tag
  int eod = 0;
  memcpy(&eod, ptr, 4);
  ptr += 4;
  /*End Packet*/

  return ptr;
}

char *OptiTrackClient::UnpackPacketHeader(char *ptr, int &messageID,
                                          int &nBytes, int &nBytesTotal) {
  // First 2 Bytes is message ID
  memcpy(&messageID, ptr, 2);
  ptr += 2;

  // Second 2 Bytes is the size of the packet
  memcpy(&nBytes, ptr, 2);
  ptr += 2;
  nBytesTotal = nBytes + 4;
  return ptr;
}

// *********************************************************************
//
//  Unpack:
//      Receives pointer to bytes that represent a packet of data
//
//      There are lots of print statements that show what
//      data is being stored
//
//      Most memcpy functions will assign the data to a variable.
//      Use this variable at your descretion.
//      Variables created for storing data do not exceed the
//      scope of this function.
//
// *********************************************************************
char *OptiTrackClient::Unpack(char *pData, std::vector<Packet> &outputs) {
  // Checks for NatNet Version number. Used later in
  // function.getServerInformation Packets may be different depending on NatNet
  // version.
  int major = 0;
  int minor = 0;
  getVersion(major, minor);
  uint64_t receiveTimestamp = getTimestamp();

  char *ptr = pData;

  // printf("Begin Packet\n-------\n");
  // printf("NatNetVersion %d %d\n", major, minor);

  int messageID = 0;
  int nBytes = 0;
  int nBytesTotal = 0;
  ptr = UnpackPacketHeader(ptr, messageID, nBytes, nBytesTotal);

  switch (messageID) {
  case NAT_CONNECT:
    // printf("Message ID  : %d NAT_CONNECT\n", messageID);
    // printf("Packet Size : %d\n", nBytes);
    break;
  case NAT_SERVERINFO:
    // printf("Message ID  : %d NAT_SERVERINFO\n", messageID);
    // printf("Packet Size : %d\n", nBytes);
    break;
  case NAT_REQUEST:
    // printf("Message ID  : %d NAT_REQUEST\n", messageID);
    // printf("Packet Size : %d\n", nBytes);
    break;
  case NAT_RESPONSE:
    // printf("Message ID  : %d NAT_RESPONSE\n", messageID);
    // printf("Packet Size : %d\n", nBytes);
    break;
  case NAT_REQUEST_MODELDEF:
    // printf("Message ID  : %d NAT_REQUEST_MODELDEF\n", messageID);
    // printf("Packet Size : %d\n", nBytes);
    break;
  case NAT_MODELDEF:
    // Data Descriptions
    // printf("Message ID  : %d NAT_MODELDEF\n", messageID);
    // printf("Packet Size : %d\n", nBytes);
    ptr = UnpackDescription(ptr, nBytes, major, minor, outputs);
    break;
  case NAT_REQUEST_FRAMEOFDATA:
    // printf("Message ID  : %d NAT_REQUEST_FRAMEOFDATA\n", messageID);
    // printf("Packet Size : %d\n", nBytes);
    break;
  case NAT_FRAMEOFDATA:
    // FRAME OF MOCAP DATA packet
    // printf("Message ID  : %d NAT_FRAMEOFDATA\n", messageID);
    // printf("Packet Size : %d\n", nBytes);
    ptr = UnpackFrameData(ptr, nBytes, major, minor, receiveTimestamp,
                          messageID, outputs);
    break;
  case NAT_MESSAGESTRING:
    // printf("Message ID  : %d NAT_MESSAGESTRING\n", messageID);
    // printf("Packet Size : %d\n", nBytes);
    break;
  case NAT_DISCONNECT:
    // printf("Message ID  : %d NAT_DISCONNECT\n", messageID);
    // printf("Packet Size : %d\n", nBytes);
    break;
  case NAT_KEEPALIVE:
    // printf("Message ID  : %d NAT_KEEPALIVE\n", messageID);
    // printf("Packet Size : %d\n", nBytes);
    break;
  case NAT_UNRECOGNIZED_REQUEST:
    printf("Message ID  : %d NAT_UNRECOGNIZED_REQUEST\n", messageID);
    printf("Packet Size : %d\n", nBytes);
    break;
  default:
    printf("Unrecognized Packet Type.\n");
    printf("Message ID  : %d\n", messageID);
    printf("Packet Size : %d\n", nBytes);
    break;
  }

  // printf("End Packet\n-------------\n");

  // check for full packet processing
  long long nBytesProcessed = (long long)ptr - (long long)pData;
  if (nBytesTotal != nBytesProcessed) {
    printf("WARNING: %d expected but %lld bytes processed\n", nBytesTotal,
           nBytesProcessed);
    if (nBytesTotal > nBytesProcessed) {
      int count = 0, countLimit = 8 * 25; // put on 8 byte boundary
      printf("Sample of remaining bytes:\n");
      char *ptr_start = ptr;
      int nCount = nBytesProcessed;
      char tmpChars[9] = {"        "};
      int charPos = ((long long)ptr % 8);
      char tmpChar;
      // add spaces for first row
      if (charPos > 0) {
        for (int i = 0; i < charPos; ++i) {
          printf("   ");
          if (i == 4) {
            printf("    ");
          }
        }
      }
      countLimit = countLimit - (charPos + 1);
      while (nCount < nBytesTotal) {
        tmpChar = ' ';
        if (isalnum(*ptr)) {
          tmpChar = *ptr;
        }
        tmpChars[charPos] = tmpChar;
        printf("%2.2x ", (unsigned char)*ptr);
        ptr += 1;
        charPos = (long long)ptr % 8;
        if (charPos == 0) {
          printf("    ");
          for (int i = 0; i < 8; ++i) {
            printf("%c", tmpChars[i]);
          }
          printf("\n");
        } else if (charPos == 4) {
          printf("    ");
        }
        if (++count > countLimit) {
          break;
        }
        ++nCount;
      }
      if ((long long)ptr % 8) {
        printf("\n");
      }
    }
  }
  // return the beginning of the possible next packet
  // assuming no additional termination
  ptr = pData + nBytesTotal;
  return ptr;
}
