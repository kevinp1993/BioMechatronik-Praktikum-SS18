#include <ch.hpp>
#include <hal.h>

#include <amiro/bluetooth/bluetooth-serial.hpp>

#include <global.hpp>

using namespace chibios_rt;
using namespace amiro;

extern Global global;

/*
 * Class constructor
 */
BluetoothSerial::BluetoothSerial(BLUETOOTH *bluetooth, uint8_t rxtx) :
  BaseStaticThread<128>(), serialConn(bluetooth, this, "SERIAL"),
  mailbox(mailboxBuffer, BLUETOOTH_SERIAL_MAILBOX_SIZE) {
  iwrap = &(bluetooth->iwrap);
  rx_tx = rxtx;
  linkId = 0xFF;
  stopflag = 0;
}

//----------------------------------------------------------------

msg_t BluetoothSerial::main(void) {
  setName("BluetoothSerial");

  while (!this->shouldTerminate()) {
    if (serialReceive())
      this->requestTerminate();
  }
  return RDY_OK;
}

/*
 * Member functions
 */
msg_t BluetoothSerial::serialReceive() {
  BluetoothDescriptor* recv_descriptor = NULL;
  uint8_t *buffer;
  size_t length;
  msg_t msg;

  msg = mailbox.fetch((msg_t*) &recv_descriptor, TIME_INFINITE);
  if ((msg == RDY_RESET) || stopflag)
    return RDY_RESET;

  buffer = recv_descriptor->bluetoothDescriptorGetPayload();
  length = recv_descriptor->bluetoothDescriptorGetPayloadLength();

  for (size_t i = 0; i < length; i++)
    chSequentialStreamPut((BaseSequentialStream*) &global.sercanmux1, buffer[i]);

  msg = iwrap->transport.bluetoothTransportGetStorageMailbox()->post((msg_t) recv_descriptor, TIME_INFINITE);
  if ((msg == RDY_RESET) || stopflag)
    return RDY_RESET;

  return RDY_OK;
}

msg_t BluetoothSerial::serialTransmit(const uint8_t* serialdata, size_t length) {
  msg_t msg;

  if ((rx_tx & 0x01) && (linkId != 0xFF) && (!stopflag))
    msg = iwrap->iwrapTransmit(linkId, serialdata, length);
  else
    msg = RDY_RESET;

  return msg;
}

void BluetoothSerial::bluetoothSerialStart(uint8_t linkid) {
  linkId = linkid;
  iwrap->transport.bluetoothTransportSetReceiveMailbox(linkId, &this->mailbox);

  if ((rx_tx & 0x02)) { // && (!stopflag)) {
    this->start(NORMALPRIO);
  }
  stopflag = 0;
}

void BluetoothSerial::bluetoothSerialStop() {
  linkId = 0xFF;
  stopflag = 1;
  mailbox.post(RDY_RESET, TIME_INFINITE);        // TIME_IMMEDIATE TIME_INFINITE
}

bool BluetoothSerial::bluetoothSerialIsConnected() {
  if (linkId == 0xFF)
    return false;

  return true;
}

void BluetoothSerial::bluetoothSerialListen(const char *addr) {
  serialConn.bluetoothConnectorListen(addr);
}

void BluetoothSerial::bluetoothSerialConnect(const char *addr){
  serialConn.bluetoothConnectorConnect(addr);
}

void BluetoothSerial::bluetoothSerialDisconnect(const char *addr) {
  serialConn.bluetoothConnectorDisconnect(addr);
}
