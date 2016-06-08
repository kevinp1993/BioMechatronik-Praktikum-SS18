#include <ch.hpp>
#include <hal.h>

#include <amiro/bluetooth/bluetooth-descriptor.hpp>


using namespace chibios_rt;
using namespace amiro;


BluetoothDescriptor::BluetoothDescriptor() :
  linkId(0), length(0), buffer{0} {
}

//----------------------------------------------------------------


void BluetoothDescriptor::bluetoothDescriptorSetLinkId(uint8_t linkId) {
  this->linkId = linkId;
}

void BluetoothDescriptor::bluetoothDescriptorSetPayloadLength(size_t length) {
  this->length = length;
}

uint8_t BluetoothDescriptor::bluetoothDescriptorGetLinkId() {
  return this->linkId;
}

size_t BluetoothDescriptor::bluetoothDescriptorGetBufferLength() {
  return sizeof(this->buffer);
}

size_t BluetoothDescriptor::bluetoothDescriptorGetPayloadLength() {
  return this->length;
}

uint8_t* BluetoothDescriptor::bluetoothDescriptorGetBuffer() {
  return this->buffer;
}

uint8_t* BluetoothDescriptor::bluetoothDescriptorGetPayload() {
  return this->buffer;
}
