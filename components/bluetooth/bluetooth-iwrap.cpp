#include <ch.hpp>
#include <hal.h>

#include <string.h>

#include <amiro/bluetooth/bluetooth-iwrap.hpp>
#include <amiro/bluetooth/bluetooth-connector.hpp>

#include <global.hpp>

using namespace chibios_rt;
using namespace amiro;

extern Global global;

/*
 * Class constructor
 */
BluetoothIwrap::BluetoothIwrap(UARTDriver* uart) :
  BaseStaticThread<128>(), transport(uart), profiles{},
  mailbox(mailboxBuffer, BLUETOOTH_IWRAP_MAILBOX_SIZE), connAddr{0} {
  iwrapblock = 0;
}

//----------------------------------------------------------------

msg_t BluetoothIwrap::main(void) {
  setName("BluetoothIwrap");

  // To set the receive mailbox[9] for plain communication (WT12->host)
  transport.bluetoothTransportSetReceiveMailbox(HOST_IWRAP_PLAIN_LINKID, &this->mailbox);

  transport.start(NORMALPRIO);

  while (!this->shouldTerminate()) {
    iwrapReceive();
  }
  return RDY_OK;
}

/*
 * Member functions
 */
msg_t BluetoothIwrap::iwrapReceive() {
  BluetoothDescriptor* descriptor = NULL;
  const char* buffer;
  size_t length;
  msg_t msg;

  msg = mailbox.fetch((msg_t*) &descriptor, TIME_INFINITE);
  if (msg == RDY_RESET)
    return RDY_RESET;

  buffer = (const char*)descriptor->bluetoothDescriptorGetPayload();
  length = descriptor->bluetoothDescriptorGetPayloadLength();

  for (size_t i = 0; i < length; i++) {
    // print out received chars
    chSequentialStreamPut((BaseSequentialStream*) &global.sercanmux1, buffer[i]);
  }

  // reset block
  iwrapblock = 0;

  // if RING or CONNECT event is received, then look for profile with correct MAC-address and connect.
  if (strstr(buffer, "CONNECT ") && (strstr(buffer, " RFCOMM 1") || strstr(buffer, " L2CAP 1"))) {
  // if CONNECT event serial RFCOMM 1 or wiimote (control) L2CAP 11 or wiimote (data) L2CAP 13
    for (int i = 0; i < 8; i++) {
      if ((profiles[i].addr != NULL) && strstr(connAddr, profiles[i].addr)) {
        BluetoothProfile* bp =  &(profiles[i]);
        bp->linkid = buffer[8] - '0';     // look for linkid and convert linkid to int
        (((BluetoothConnector*)(bp->connector))->*(bp->connect))(bp->linkid);
        break;
      }
    }
  } else if (strstr(buffer, "RING") && strstr(buffer, "1 RFCOMM ")) {
  // if RING event (be called by other device) in 1 RFCOMM mode
    for (int i = 0; i < 8; i++) {
      if ((profiles[i].addr != NULL) && (strstr(buffer, profiles[i].addr) || strstr(profiles[i].addr, "free"))) {
        BluetoothProfile* bp =  &(profiles[i]);
        bp->linkid  = buffer[5] - '0';    // look for linkid and convert linkid to int
        (((BluetoothConnector*)(bp->connector))->*(bp->connect))(bp->linkid );
        break;
      }
    }
  } else if (strstr(buffer, "NO CARRIER ") && strstr(buffer, "ERROR ")) {
  // if NO CARRIER ERROR 0 event, then look for profile and disconnect
    uint8_t linkid = buffer[11] - '0';
    for (int i = 0; i < 8; i++) {
      if ((profiles[i].addr != NULL) && (profiles[i].linkid == linkid)) {
        BluetoothProfile* bp =  &(profiles[i]);
        bp->linkid = 0xFF;
        (((BluetoothConnector*)(bp->connector))->*(bp->disconnect))();
        break;
      }
    }
  }

  msg = (transport.bluetoothTransportGetStorageMailbox())->post((msg_t) descriptor, TIME_INFINITE);
  if (msg == RDY_RESET)
    return RDY_RESET;

  return RDY_OK;
}

msg_t BluetoothIwrap::iwrapTransmit(uint8_t linkid, const uint8_t* txdata, size_t length) {
  BluetoothDescriptor* descriptor = NULL;
  uint8_t *buffer;
  msg_t msg;

  msg = transport.bluetoothTransportGetStorageMailbox()->fetch((msg_t*) &descriptor, TIME_INFINITE);
  if (msg == RDY_RESET)
    return RDY_RESET;

  buffer = descriptor->bluetoothDescriptorGetPayload();
  memcpy(buffer, txdata, length);

  descriptor->bluetoothDescriptorSetLinkId(linkid);
  descriptor->bluetoothDescriptorSetPayloadLength(length);

  msg = transport.bluetoothTransportGetTransmitMailbox()->post((msg_t) descriptor, TIME_INFINITE);
  if (msg == RDY_RESET)
    return RDY_RESET;

  return RDY_OK;
}

void BluetoothIwrap::bluetoothIwrapSendCommand(const char* cmd) {
  // Wait until another previous command finished.
  while(iwrapblock == 1) {
    BaseThread::sleep(MS2ST(500));
  }

  iwrapblock = 1;

  if (strstr(cmd, "CALL ") && (strstr(cmd, " RFCOMM") || strstr(cmd, " L2CAP"))) {
    memcpy(connAddr, cmd+5, 17);
  }

  size_t length = strlen(cmd);
//  for (size_t i = 0; i < length; i++)
//    chSequentialStreamPut((BaseSequentialStream*) &global.sercanmux1, cmd[i]);   // To display the command

  // To send the string command to WT12 bluetooth chip
  iwrapTransmit(HOST_IWRAP_PLAIN_LINKID,(uint8_t*) cmd, length);

  // Clear iwrapblock flag because these commands don't return an answer
  if(strstr(cmd, "SET BT NAME") || strstr(cmd, "SET BT AUTH") || strstr(cmd, "SET BT SSP")
    || strstr(cmd, "SET BT PAGEMODE") || strstr(cmd, "SET BT PAIR *")
    || strstr(cmd, "SET CONTROL MUX") || (strstr(cmd, "SET") && strlen(cmd) == 3)) {
    iwrapblock = 0;
  }

  BaseThread::sleep(MS2ST(500));
}
