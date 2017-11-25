#ifndef SERIALIZE_H
#define SERIALIZE_H

template <typename T>
void serialize(uint8_t* buffer, uint8_t& index, T value) {
  memcpy(buffer + index, &value, sizeof(T));
  index += sizeof(T);
}

#endif