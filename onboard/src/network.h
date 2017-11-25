#ifndef NETWORK_H
#define NETWORK_H

#include <SoftwareSerial.h>
#include "pose.h"
#include "sonars.h"

/**
 * Value associated with each PC packet to indicate intent
 * of the contained pose (waypoint or pose update)
 */
using pc_packet_intent_t = uint32_t;
struct PCPacketData {
    Pose pose;
    sequence_num_t sequenceNum;
    pc_packet_intent_t intent;
};

template <typename T>
void serialize(uint8_t* buffer, uint8_t& index, T value) {
    memcpy(buffer + index, &value, sizeof(T));
    index += sizeof(T);
}

/**
 * @brief The Network class
 */
class Network {
    struct R {
        static constexpr auto PACKET_SIZE =
            PoseUpdate::SERIALIZED_SIZE +
            sizeof(sonar_reading_t) * Sonars::NUM_SONAR +
            sizeof(sequence_num_t) + sizeof(uint8_t);
        static constexpr char PACKET_START_BYTE = 0xA1;
    };

    struct PC {
        static constexpr auto PACKET_SIZE = sizeof(PCPacketData);
        static constexpr char PACKET_START_BYTE = 0xF5;
        static constexpr char PACKET_END_BYTE = 0x53;

        union Packet {
            PCPacketData packetData;
            byte byteData[PACKET_SIZE];
        };
    };

    enum RxState { WAIT_FOR_START, READING, HAVE_VALID_PACKET };

  public:
    static void begin(uint32_t baudRate);
    /**
     * @brief Send packet from robot containing the pose update for this logic
     * cycle and sensor readings
     * @param poseUpdate
     * @param activeBehaviourId
     * @return Whether the packet was sent correctly
     */
    static bool sendRobotPacket(const PoseUpdate& poseUpdate,
                                uint8_t activeBehaviourId);

    /**
     * @brief Receive next byte of packet through bluetooth
     * @return Whether full packet is ready to be processed
     */
    static bool recvPcPacket();

    static PCPacketData getLatestPCPacket();
    /**
     * @brief Call after receiving and processing a PC packet.
     * Safe to call after call to getLatestPCPacket
     */
    static void resetPcPacket();

  private:
    // robot sending
    static sequence_num_t _sequenceNum;
    static uint8_t _robotPacketBuf[R::PACKET_SIZE];

    // pc sending
    static PC::Packet _pcPacket;
    static size_t _pcPacketIndex;
    static RxState _rxState;

    static SoftwareSerial _blueTooth;
};

#endif // NETWORK_H
