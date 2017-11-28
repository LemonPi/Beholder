#ifndef NETWORK_H
#define NETWORK_H

#include <SoftwareSerial.h>
#include "pose.h"
#include "sonars.h"

/**
 * Value associated with each PC packet to indicate intent
 * of the contained pose (waypoint or pose update)
 */
using pc_packet_intent_t = uint8_t;

/**
 * 0 - 29 means associated pose is a new waypoint with corresponding type
 * 30 - 59 means enable behaviour with this ID
 * 60 - 90 means disable behaviour with this ID
 * special intent values 250+
 */
namespace PCPacketIntent {
constexpr pc_packet_intent_t GROUP_OFFSET = 30;
constexpr pc_packet_intent_t TURN_ON = 250;
constexpr pc_packet_intent_t TURN_OFF = 251;
constexpr pc_packet_intent_t POSE_UPDATE = 252;
constexpr pc_packet_intent_t POSE_PING = 253;
constexpr pc_packet_intent_t CLEAR_TARGETS = 254;
}

struct PCPacketData {
    Pose pose;
    sequence_num_t sequenceNum;
    pc_packet_intent_t intent;

    static constexpr auto SERIALIZED_SIZE = Pose::SERIALIZED_SIZE +
                                            sizeof(sequence_num_t) +
                                            sizeof(pc_packet_intent_t);

    void deserializeObj(const uint8_t* const buffer, uint8_t& index);
};

template <typename T>
void serialize(uint8_t* buffer, uint8_t& index, T value) {
    memcpy(buffer + index, &value, sizeof(T));
    index += sizeof(T);
}

template <typename T>
void deserialize(const uint8_t* const buffer, uint8_t& index, T& value) {
    memcpy(&value, buffer + index, sizeof(T));
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
            sizeof(sequence_num_t) + sizeof(uint8_t) +
            sizeof(uint8_t); // CRC at end
        static constexpr uint8_t PACKET_START_BYTE = 0xA1;
    };

    struct PC {
        static constexpr auto PACKET_SIZE = PCPacketData::SERIALIZED_SIZE;
        static constexpr uint8_t PACKET_START_BYTE = 0xF5;
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

    static sequence_num_t getSequenceNum();

  private:
    // robot sending
    static sequence_num_t _sequenceNum;
    static uint8_t _robotPacketBuf[R::PACKET_SIZE];

    // pc sending
    static uint8_t _pcPacketBuf[PC::PACKET_SIZE];
    static size_t _pcPacketIndex;
    static RxState _rxState;

    // software serial's not working for RX; instead use HardwareSerial
    // Using Serial3 which is pins 14 and 15 for Mega
    //    static SoftwareSerial _blueTooth;
};

#endif // NETWORK_H
