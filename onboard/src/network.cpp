#include "network.h"
#include "constants.h"
#include "debug.h"

sequence_num_t Network::_sequenceNum = 0;
SoftwareSerial Network::_blueTooth(BL_RX, BL_TX);
size_t Network::_pcPacketIndex = 0;
Network::RxState Network::_rxState = Network::RxState::WAIT_FOR_START;

Network::R::Packet Network::_robotPacket;
Network::PC::Packet Network::_pcPacket;

bool Network::begin(uint32_t baudRate) {
    _blueTooth.begin(baudRate);
}

bool Network::sendRobotPacket(const PoseUpdate& poseUpdate,
                              uint8_t activeBehaviourId) {
    // whenever we send a packet increment the sequence number
    // so that the time after sending the packet the sequence number refers to
    // the last sent packet
    _robotPacket.packetData.sequenceNum = ++_sequenceNum;
    _robotPacket.packetData.poseUpdate = poseUpdate;
    _robotPacket.packetData.activeBehaviourId = activeBehaviourId;
    for (auto i = 0; i < Sonars::NUM_SONAR; ++i) {
        _robotPacket.packetData.sonarReadings[i] =
            Sonars::getReading(static_cast<Sonars::SonarIndex>(i));
    }

    _blueTooth.write(R::PACKET_START_BYTE);
    const auto sentBytes =
        _blueTooth.write(_robotPacket.byteData, R::PACKET_SIZE);

    if (sentBytes != R::PACKET_SIZE) {
        ERROR(4);
        PRINTLN(sentBytes);
        return false;
    } else {
        return true;
    }
}

bool Network::recvPcPacket() {
    // don't read anything if we haven't processed the previous one
    if (_rxState == RxState::HAVE_VALID_PACKET) {
        return true;
    }

    // read into byte buffer of pcPacket until full
    if (_blueTooth.available()) {
        const auto c = _blueTooth.read();

        switch (_rxState) {
        case RxState::WAIT_FOR_START:
            if (c == PC::PACKET_START_BYTE) {
                _rxState = RxState::READING;
                _pcPacketIndex = 0;
            }
            break;
        case RxState::READING:
            // should've gotten all content, so check end byte
            // TODO consider making this end byte the CRC (xor all bytes)
            if (_pcPacketIndex == PC::PACKET_SIZE) {
                if (c == PC::PACKET_END_BYTE) {
                    _rxState = RxState::HAVE_VALID_PACKET;
                } else {
                    resetPcPacket();
                }
            } else {
                // otherwise just add to buffer
                _pcPacket.byteData[_pcPacketIndex++] = c;
                _pcPacketIndex++;
            }
            break;
        default:
            break;
        }
    }

    return _rxState == RxState::HAVE_VALID_PACKET;
}

PCPacketData Network::getLatestPCPacket() {
    return _pcPacket.packetData;
}

void Network::resetPcPacket() {
    _rxState = RxState::WAIT_FOR_START;
    _pcPacketIndex = 0;
}
