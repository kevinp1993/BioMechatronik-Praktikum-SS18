#ifndef AMIRO_BLUETOOTH_H_
#define AMIRO_BLUETOOTH_H_

#include <amiro/bluetooth/bluetooth-iwrap.hpp>

namespace amiro {

  class BLUETOOTH {
  public:
    BLUETOOTH(UARTDriver* uart);

    void bluetoothStart();
    void bluetoothReset();
    void bluetoothEnableMux();
    void bluetoothDisableMux();
    void bluetoothDiscoverDevices();
    void bluetoothListAllConnections();
    void bluetoothSetName(const char *name);
    void bluetoothSetPin(const char *pin);
    void bluetoothCloseConnection(uint8_t linkid);
    void bluetoothSendCommand(const char *cmd);
    bool bluetoothIsMuxMode();

    BluetoothIwrap iwrap;
  };
}

#endif /* BLUETOOTH_H_ */
