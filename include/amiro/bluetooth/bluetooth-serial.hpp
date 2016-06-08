#ifndef _BLUETOOTH_SERIAL_H_
#define _BLUETOOTH_SERIAL_H_

#include <amiro/bluetooth/bluetooth-connector.hpp>

#define BLUETOOTH_SERIAL_MAILBOX_SIZE 5

#define SERIAL_START_EVENT  1
#define SERIAL_STOP_EVENT   2

namespace amiro {
  class BluetoothSerial : public chibios_rt::BaseStaticThread<128> {
  public:
    BluetoothSerial(BLUETOOTH *bluetooth, uint8_t rxtx);

    void bluetoothSerialStart(uint8_t linkid);
    void bluetoothSerialStop();
    bool bluetoothSerialIsConnected();

    void bluetoothSerialListen(const char *addr);
    void bluetoothSerialConnect(const char *addr);
    void bluetoothSerialDisconnect(const char *addr);

    msg_t serialTransmit(const uint8_t* serialdata, size_t length);

  protected:
    virtual msg_t main(void);

  private:
    msg_t serialReceive();

    BluetoothConnector serialConn;
    uint8_t rx_tx;

    chibios_rt::Mailbox mailbox;
    msg_t mailboxBuffer[BLUETOOTH_SERIAL_MAILBOX_SIZE];

    BluetoothIwrap *iwrap;
    uint8_t linkId;
    uint8_t stopflag;
  };
}
#endif /* _BLUETOOTH_SERIAL_H_ */
