#ifndef _BLUETOOTH_CONNECTOR_H_
#define _BLUETOOTH_CONNECTOR_H_

#include <amiro/bluetooth/bluetooth.hpp>

namespace amiro {

  class BluetoothConnector {
  public:
    BluetoothConnector(BLUETOOTH *bluetooth, void *bluetoothDev, const char *typeConn);

    void bluetoothConnectorListen(const char *addr);
    void bluetoothConnectorConnect(const char *addr);
    void bluetoothConnectorDisconnect(const char *addr);

  private:
    void bluetoothConnectorActivate(uint8_t linkid);
    void bluetoothConnectorDeactivate();

    BluetoothIwrap *iwrap;
    void * objDev;
    const char *typeDev;
  };
}

#endif /* _BLUETOOTH_CONNECTOR_H_ */
