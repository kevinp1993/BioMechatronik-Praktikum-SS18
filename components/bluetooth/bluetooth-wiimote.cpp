#include <ch.hpp>
#include <hal.h>

#include <amiro/bluetooth/bluetooth-wiimote.hpp>

using namespace chibios_rt;
using namespace amiro;

/*
 * Class constructor
 */
BluetoothWiimote::BluetoothWiimote(BLUETOOTH *bluetooth, uint8_t rxtx) :
  BaseStaticThread<128>(), wiimoteConn(bluetooth, this, "WIIMOTE"),
  mailbox(mailboxBuffer, BLUETOOTH_WIIMOTE_MAILBOX_SIZE), accelerometer{0,0,0} {
  iwrap = &(bluetooth->iwrap);
  rx_tx = rxtx;
  linkId = 0xFF;
  stopflag = 0;
}

//----------------------------------------------------------------

msg_t BluetoothWiimote::main(void) {
  setName("BluetoothWiimote");

  while (!this->shouldTerminate()) {
    if (wiimoteReceive())
      this->requestTerminate();
  }
  return RDY_OK;
}

/*
 * Member functions
 */
msg_t BluetoothWiimote::wiimoteReceive() {
  BluetoothDescriptor* recv_descriptor = NULL;
  uint8_t *buffer;
  size_t length;
  msg_t msg;

  static uint8_t button_up;
  static uint8_t button_down;
  static uint8_t button_right;
  static uint8_t button_left;
  static uint8_t button_plus;
  static uint8_t button_home;
  static uint8_t button_minus;
  static uint8_t button_A;
  static uint8_t button_B;
  static uint8_t button_1;
  static uint8_t button_2;

  msg = mailbox.fetch((msg_t*) &recv_descriptor, TIME_INFINITE);
  if ((msg == RDY_RESET) || stopflag)
    return RDY_RESET;

  buffer = recv_descriptor->bluetoothDescriptorGetPayload();
  length = recv_descriptor->bluetoothDescriptorGetPayloadLength();

  if (buffer[0] == 0xA1 && buffer[1] == 0x31) {
    accelerometer.x_axis = (buffer[4] << 2) + ((buffer[2] & 0x60) >> 5) - 0x1EC;
    accelerometer.y_axis = (buffer[5] << 2) + ((buffer[3] & 0x20) >> 4) - 0x1EA;
    accelerometer.z_axis = (buffer[6] << 2) + ((buffer[3] & 0x40) >> 5) - 0x1EE;

    if (buffer[3] & 0x80) {             // Press home to return button reporting
      bluetoothWiimoteDataBtn();
      accelerometer.x_axis = 0;
      accelerometer.y_axis = 0;
      accelerometer.z_axis = 0;
    }

  } else if (buffer[0] == 0xA1 && buffer[1] == 0x30) {
    button_up    = (buffer[2] & 0x08) >> 3;
    button_down  = (buffer[2] & 0x04) >> 2;
    button_right = (buffer[2] & 0x02) >> 1;
    button_left  = (buffer[2] & 0x01) >> 0;
    button_plus  = (buffer[2] & 0x10) >> 4;
    button_home  = (buffer[3] & 0x80) >> 7;
    button_minus = (buffer[3] & 0x10) >> 4;
    button_A     = (buffer[3] & 0x08) >> 3;
    button_B     = (buffer[3] & 0x04) >> 2;
    button_1     = (buffer[3] & 0x02) >> 1;
    button_2     = (buffer[3] & 0x01) >> 0;

    if (button_up)
      chSequentialStreamPut((BaseSequentialStream*) &SD1, 'U');

    if (button_down)
      chSequentialStreamPut((BaseSequentialStream*) &SD1, 'D');

    if (button_right)
      chSequentialStreamPut((BaseSequentialStream*) &SD1, 'R');

    if (button_left)
      chSequentialStreamPut((BaseSequentialStream*) &SD1, 'L');

    if (button_plus)
      chSequentialStreamPut((BaseSequentialStream*) &SD1, '+');

    if (button_home)
      chSequentialStreamPut((BaseSequentialStream*) &SD1, 'H');

    if (button_minus)
      chSequentialStreamPut((BaseSequentialStream*) &SD1, '-');

    if (button_A)
      chSequentialStreamPut((BaseSequentialStream*) &SD1, 'A');

    if (button_B)
      chSequentialStreamPut((BaseSequentialStream*) &SD1, 'B');

    if (button_1)
      chSequentialStreamPut((BaseSequentialStream*) &SD1, '1');

    if (button_2)
      chSequentialStreamPut((BaseSequentialStream*) &SD1, '2');

    if (button_minus && button_plus)   // Press minus and plue to return accelerometer reporting
      bluetoothWiimoteDataBtnAcc();
  } else {
    chSequentialStreamWrite((BaseSequentialStream*) &SD1, buffer, length);
  }

  msg = iwrap->transport.bluetoothTransportGetStorageMailbox()->post((msg_t) recv_descriptor, TIME_INFINITE);
  if ((msg == RDY_RESET) || stopflag)
    return RDY_RESET;

  return RDY_OK;
}

msg_t BluetoothWiimote::wiimoteTransmit(const uint8_t* wiimotecmd, size_t length) {
  msg_t msg;

  if ((rx_tx & 0x01) && (linkId != 0xFF) && (!stopflag))
    msg = iwrap->iwrapTransmit(linkId, wiimotecmd, length);
  else
    msg = RDY_RESET;

  return msg;
}

void BluetoothWiimote::bluetoothWiimoteStart(uint8_t linkid) {
  linkId = linkid;
  iwrap->transport.bluetoothTransportSetReceiveMailbox(linkId, &this->mailbox);

  if ((rx_tx & 0x02)) {   // && (!stopflag)
    this->start(NORMALPRIO);
  }
  stopflag = 0;
}

void BluetoothWiimote::bluetoothWiimoteStop() {
  linkId = 0xFF;
  stopflag = 1;
  mailbox.post(RDY_RESET, TIME_INFINITE);        // TIME_IMMEDIATE TIME_INFINITE
}

bool BluetoothWiimote::bluetoothWiimoteIsConnected() {
  if (linkId == 0xFF)
    return false;

  return true;
}

void BluetoothWiimote::bluetoothWiimoteListen(const char *addr) {
  wiimoteConn.bluetoothConnectorListen(addr);
}

void BluetoothWiimote::bluetoothWiimoteConnect(const char *addr) {
  wiimoteConn.bluetoothConnectorConnect(addr);
}

void BluetoothWiimote::bluetoothWiimoteDisconnect(const char *addr) {
  wiimoteConn.bluetoothConnectorDisconnect(addr);
}

BluetoothWiimote::Accelerometer * BluetoothWiimote::getAccelerometer() {
  return &accelerometer;
}

/*
 * @brief :  On-off LEDs and Motor of Wiimote.
 *
 * @param[in] ledid_flag     On-off flag of LEDs 1,2,3,4
 *                           (LED1: 0x10, LED2: 0x20, LED3: 0x40, LED4: 0x80)
 * @param[in] vibrate_flag   On-off flag of Vibration Motor (On: 1, off: 0)
 */
void BluetoothWiimote::bluetoothWiimoteLedVibrate(uint8_t ledid_flag, uint8_t vibrate_flag) {
  const uint8_t data[] = {0xA2, 0x11, (uint8_t) (ledid_flag | vibrate_flag)};
  wiimoteTransmit(data, 3);
}

/*
 * @brief :  Show status information of Wiimote.
 */
void BluetoothWiimote::bluetoothWiimoteStatusInfo() {
  const uint8_t data[] = {0xA2, 0x15, 0x00};
  wiimoteTransmit(data, 3);
}

/*
 * @brief :  Send button status (change event) of Wiimote.
 */
void BluetoothWiimote::bluetoothWiimoteDataBtn() {
  const uint8_t data[] = {0xA2, 0x12, 0x00, 0x30};
  wiimoteTransmit(data, 4);
}

/*
 * @brief :  Send button & accelerometer status (change event) of Wiimote.
 */
void BluetoothWiimote::bluetoothWiimoteDataBtnAcc() {
  const uint8_t data[] = {0xA2, 0x12, 0x00, 0x31};
  wiimoteTransmit(data, 4);
}
