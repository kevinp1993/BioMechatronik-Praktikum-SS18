#ifndef _BLUETOOTH_WIIMOTE_H_
#define _BLUETOOTH_WIIMOTE_H_

#include <amiro/bluetooth/bluetooth-connector.hpp>

#define BLUETOOTH_WIIMOTE_MAILBOX_SIZE 5

#define WIIMOTE_START_EVENT  1
#define WIIMOTE_STOP_EVENT   2

namespace amiro {
  class BluetoothWiimote : public chibios_rt::BaseStaticThread<128> {
  public:
    BluetoothWiimote(BLUETOOTH *bluetooth, uint8_t rxtx);

    void bluetoothWiimoteStart(uint8_t linkid);
    void bluetoothWiimoteStop();
    bool bluetoothWiimoteIsConnected();

    void bluetoothWiimoteListen(const char *addr);
    void bluetoothWiimoteConnect(const char *addr);
    void bluetoothWiimoteDisconnect(const char *addr);

    msg_t wiimoteTransmit(const uint8_t* wiimotecmd, size_t length);
    void bluetoothWiimoteLedVibrate(uint8_t ledid_flag, uint8_t vibrate_flag);
    void bluetoothWiimoteStatusInfo();
    void bluetoothWiimoteDataBtn();
    void bluetoothWiimoteDataBtnAcc();

    class Accelerometer {
    public:
      int16_t x_axis, y_axis, z_axis;
    };
    Accelerometer *getAccelerometer();

    struct Buttons {
      uint16_t left : 1;
      uint16_t right : 1;
      uint16_t down : 1;
      uint16_t up : 1;
      uint16_t plus : 1;
      uint16_t two : 1;
      uint16_t one : 1;
      uint16_t B : 1;
      uint16_t A : 1;
      uint16_t minus : 1;
      uint16_t home : 1;
    };
    Buttons* getButtons();

  protected:
    virtual msg_t main(void);

  private:
    msg_t wiimoteReceive();

    BluetoothConnector wiimoteConn;
    uint8_t rx_tx;

    chibios_rt::Mailbox mailbox;
    msg_t mailboxBuffer[BLUETOOTH_WIIMOTE_MAILBOX_SIZE];

    BluetoothIwrap *iwrap;
    uint8_t linkId;
    uint8_t stopflag;

    Accelerometer accelerometer;
    Buttons buttons;
  };
}

#endif /* _BLUETOOTH_WIIMOTE_H_ */
