#include "network.h"
#include "constants.h"
#include "debug.h"

sequence_num_t Network::_sequenceNum = 0;
SoftwareSerial Network::_blueTooth(BL_RX, BL_TX);
size_t Network::_pcPacketIndex = 0;
Network::RxState Network::_rxState = Network::RxState::WAIT_FOR_START;

uint8_t Network::_robotPacketBuf[Network::R::PACKET_SIZE];
uint8_t Network::_pcPacketBuf[Network::PC::PACKET_SIZE];

void PCPacketData::deserializeObj(const uint8_t* const buffer, uint8_t& index) {
    pose.deserializeObj(buffer, index);
    deserialize(buffer, index, sequenceNum);
    deserialize(buffer, index, intent);
}

void Network::begin(uint32_t baudRate) {
    _blueTooth.begin(baudRate);
}

bool Network::sendRobotPacket(const PoseUpdate& poseUpdate,
                              uint8_t activeBehaviourId) {
    // whenever we send a packet increment the sequence number
    // so that the time after sending the packet the sequence number refers to
    // the last sent packet
    uint8_t serializationIndex = 0;
    serialize(_robotPacketBuf, serializationIndex, ++_sequenceNum);
    serialize(_robotPacketBuf, serializationIndex, poseUpdate.displacement);
    serialize(_robotPacketBuf, serializationIndex, poseUpdate.headingDiff);
    for (auto i = 0; i < Sonars::NUM_SONAR; ++i) {
        serialize(_robotPacketBuf, serializationIndex,
                  Sonars::getReading(static_cast<Sonars::SonarIndex>(i)));
    }
    serialize(_robotPacketBuf, serializationIndex, activeBehaviourId);

    if (serializationIndex != R::PACKET_SIZE) {
        ERROR(5);
        PRINTLN(serializationIndex);
        return false;
    }

    _blueTooth.write(R::PACKET_START_BYTE);
    const auto sentBytes =
        _blueTooth.write(_robotPacketBuf, serializationIndex);

    // CRC error checking
    auto crc = _robotPacketBuf[0];
    for (auto i = 1; i < serializationIndex; ++i) {
        crc ^= _robotPacketBuf[i];
    }
    _blueTooth.write(crc);

    if (sentBytes != serializationIndex) {
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
                // xor all bytes to get CRC for packet and compare against
                // incoming CRC
                auto crc = _pcPacketBuf[0];
                for (auto i = 1U; i < _pcPacketIndex; ++i) {
                    crc ^= _pcPacketBuf[i];
                }

                if (c == crc) {
                    _rxState = RxState::HAVE_VALID_PACKET;
                } else {
                    // something went wrong and we got back CRC
                    // just ignore packet
                    ERROR(7);
                    resetPcPacket();
                }
            } else {
                // otherwise just add to buffer
                _pcPacketBuf[_pcPacketIndex++] = c;
            }
            break;
        default:
            break;
        }
    }

    return _rxState == RxState::HAVE_VALID_PACKET;
}

PCPacketData Network::getLatestPCPacket() {
    PCPacketData pcPacket;
    uint8_t deserializationIndex = 0;
    pcPacket.deserializeObj(_pcPacketBuf, deserializationIndex);
    return pcPacket;
}

void Network::resetPcPacket() {
    _rxState = RxState::WAIT_FOR_START;
    _pcPacketIndex = 0;
}
sequence_num_t Network::getSequenceNum() {
    return _sequenceNum;
}
