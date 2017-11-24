#ifndef NETWORK_H
#define NETWORK_H

#include <SoftwareSerial.h>
#include "pose.h"
#include "sonars.h"

struct UpdatePacketData {
    sequence_num_t sequenceNum;
    PoseUpdate poseUpdate;
    sonar_reading_t sonarReadings[Sonars::NUM_SONAR];
    uint8_t activeBehaviourId;
};

/**
 * @brief The Network class
 */
class Network {
    union UpdatePacket {
        UpdatePacketData packetData;
        // data in byte form
        byte byteData[sizeof(UpdatePacketData)];
    };

    static constexpr char PACKET_START_BIT = 0xA1;

  public:
    /**
     * @brief Send packet from robot containing the pose update for this logic
     * cycle and sensor readings
     * @param poseUpdate
     * @param activeBehaviourId
     */
    static void sendReadingPacket(const PoseUpdate& poseUpdate,
                                  uint8_t activeBehaviourId);

  private:
    static sequence_num_t _sequenceNum;
    static UpdatePacket _updatePacket;
    static SoftwareSerial _blueTooth;
};

#endif // NETWORK_H
