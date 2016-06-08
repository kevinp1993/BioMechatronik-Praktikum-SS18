#ifndef _BLUETOOTH_TRANSPORT_H_
#define _BLUETOOTH_TRANSPORT_H_

#include <amiro/bluetooth/bluetooth-descriptor.hpp>

#define BLUETOOTH_TRANSPORT_STORAGE_MAILBOX_SIZE   10
#define BLUETOOTH_TRANSPORT_TRANSMIT_MAILBOX_SIZE   5

#define TRANSMIT_COMPLETED_EVENT  1

#define FRAME_HEADER_SIZE    4
#define FRAME_TRAILER_SIZE   1

namespace amiro {

  class BluetoothTransport : public chibios_rt::BaseStaticThread<128>, protected UARTConfig {
  public:
    BluetoothTransport(UARTDriver* uart);

    chibios_rt::Mailbox* bluetoothTransportGetStorageMailbox();
    chibios_rt::Mailbox* bluetoothTransportGetTransmitMailbox();

    void bluetoothTransportSetReceiveMailbox(uint8_t linkId, chibios_rt::Mailbox* mailbox);
    void bluetoothTransportResetState();
    bool bluetoothTransportIsMuxMode();

  protected:
    virtual msg_t main(void);

    virtual void bluetoothTransportTransmitCompleted();
    virtual void bluetoothTransportCharacterReceived(uint16_t c);
    virtual void bluetoothTransportReceiveCompleted();

  private:
    static void bluetoothTransportTransmitCompleted_cb(UARTDriver *uart);
    static void bluetoothTransportCharacterReceived_cb(UARTDriver *uart, uint16_t c);
    static void bluetoothTransportReceiveCompleted_cb(UARTDriver *uart);

    UARTDriver *uart;

    chibios_rt::Mailbox* receiveMailbox[10];
    chibios_rt::Mailbox storageMailbox;
    chibios_rt::Mailbox transmitMailbox;

    msg_t storageMailboxBuffer[BLUETOOTH_TRANSPORT_STORAGE_MAILBOX_SIZE];
    msg_t transmitMailboxBuffer[BLUETOOTH_TRANSPORT_TRANSMIT_MAILBOX_SIZE];

    enum BluetoothTransportState {
      BLUETOOTH_TRANSPORT_PLAIN   = 0,
      BLUETOOTH_TRANSPORT_HEADER  = 1,
      BLUETOOTH_TRANSPORT_DECODE  = 2,
      BLUETOOTH_TRANSPORT_PAYLOAD = 3,
      BLUETOOTH_TRANSPORT_TRAILER = 4,
      BLUETOOTH_TRANSPORT_IDLE    = 5
    }transmitState, receiveState;

    BluetoothDescriptor descriptor[BLUETOOTH_TRANSPORT_STORAGE_MAILBOX_SIZE];
    BluetoothDescriptor *receive;
    BluetoothDescriptor *transmit;

    size_t rcvlength;

    /* These arrays Contain header and trailer */
    uint8_t receiveFrameBuffer[5];
    uint8_t transmitFrameBuffer[5];

    void postReceiveDescriptorI();
    void postTransmitDescriptorI();

    uint8_t* getEncodeHeader();
    uint8_t* getEncodeTailer();
    int decodeHeader();
  };
}

#endif /* _BLUETOOTH_TRANSPORT_H_ */
