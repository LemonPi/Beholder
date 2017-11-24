#include "network.h"
#include "constants.h"

sequence_num_t Network::_sequenceNum = 0;
SoftwareSerial Network::_blueTooth(BL_RX, BL_TX);

void Network::sendReadingPacket(const PoseUpdate& poseUpdate,
                                uint8_t activeBehaviourId) {
    // whenever we send a packet increment the sequence number
    // so that the time after sending the packet the sequence number refers to
    // the last sent packet
    _updatePacket.packetData.sequenceNum = ++_sequenceNum;
    _updatePacket.packetData.poseUpdate = poseUpdate;
    _updatePacket.packetData.activeBehaviourId = activeBehaviourId;
    for (auto i = 0; i < Sonars::NUM_SONAR; ++i) {
        _updatePacket.packetData.sonarReadings[i] =
            Sonars::getReading(static_cast<Sonars::SonarIndex>(i));
    }

    // TODO bluetooth sending code
}
