#ifndef _BLUETOOTH_PROFILE_H_
#define _BLUETOOTH_PROFILE_H_

#define NO_RX_TX  0
#define TX_ONLY   1
#define RX_ONLY   2
#define RX_TX     3

namespace amiro {

  class BluetoothConnector;

  class BluetoothProfile {
  public:
    BluetoothProfile() {};

    const char *addr;
    void *connector;
    void (BluetoothConnector::*connect) (uint8_t linkid);
    void (BluetoothConnector::*disconnect) ();
    uint8_t linkid;
  };
}

#endif /* _BLUETOOTH_PROFILE_H_ */
