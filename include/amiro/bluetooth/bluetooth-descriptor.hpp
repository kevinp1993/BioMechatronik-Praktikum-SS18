#ifndef _BLUETOOTH_DESCRIPTOR_H_
#define _BLUETOOTH_DESCRIPTOR_H_

#include <string.h>

#define BLUETOOTH_DESCRIPTOR_BUFFER_SIZE 64

namespace amiro {

  class BluetoothDescriptor {

  public:
    BluetoothDescriptor();

    void bluetoothDescriptorSetLinkId(uint8_t linkId);
    void bluetoothDescriptorSetPayloadLength(size_t length);

    uint8_t bluetoothDescriptorGetLinkId();
    size_t bluetoothDescriptorGetBufferLength();
    size_t bluetoothDescriptorGetPayloadLength();

    uint8_t *bluetoothDescriptorGetBuffer();
    uint8_t *bluetoothDescriptorGetPayload();

  private:
//    unsigned char flags;
    uint8_t linkId;
    size_t length;
    uint8_t buffer[BLUETOOTH_DESCRIPTOR_BUFFER_SIZE];

  };
}

#endif /* _BLUETOOTH_DESCRIPTOR_H_ */
