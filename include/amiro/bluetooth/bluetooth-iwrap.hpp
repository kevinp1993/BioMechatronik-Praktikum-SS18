#ifndef _BLUETOOTH_IWRAP_H_
#define _BLUETOOTH_IWRAP_H_

#include <amiro/bluetooth/bluetooth-transport.hpp>
#include <amiro/bluetooth/bluetooth-profile.hpp>

#if !CH_USE_QUEUES || !CH_USE_EVENTS
#error "Bluetooth Serial Port Profile requires CH_USE_QUEUES, CH_USE_EVENTS"
#endif

#define BLUETOOTH_IWRAP_MAILBOX_SIZE  5

#define HOST_IWRAP_PLAIN_LINKID       0xFF

namespace amiro {

  class BluetoothIwrap : public chibios_rt::BaseStaticThread<128> {
  public:
    BluetoothIwrap(UARTDriver* uart);

    void bluetoothIwrapSendCommand(const char* cmd);
    msg_t iwrapTransmit(uint8_t linkid, const uint8_t* txdata, size_t length);

    BluetoothTransport transport;

    BluetoothProfile profiles[8];

  protected:
    virtual msg_t main(void);

  private:
    msg_t iwrapReceive();

    chibios_rt::Mailbox mailbox;
    msg_t mailboxBuffer[BLUETOOTH_IWRAP_MAILBOX_SIZE];

    char connAddr[17];
    int iwrapblock;
  };
}

#endif /* _BLUETOOTH_IWRAP_H_ */
