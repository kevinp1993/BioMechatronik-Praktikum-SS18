#include <ch.hpp>
#include <hal.h>

#include <string.h>
#include <chprintf.h>

#include <amiro/bluetooth/bluetooth-connector.hpp>
#include <amiro/bluetooth/bluetooth-serial.hpp>
#include <amiro/bluetooth/bluetooth-wiimote.hpp>

using namespace chibios_rt;
using namespace amiro;

/*
 * Class constructor
 */
BluetoothConnector::BluetoothConnector(BLUETOOTH *bluetooth, void *bluetoothDev, const char *typeConn) {
  iwrap = &(bluetooth->iwrap);
  objDev = bluetoothDev;
  typeDev = typeConn;
}

//----------------------------------------------------------------


void BluetoothConnector::bluetoothConnectorListen(const char *addr) {
  BluetoothProfile profile;
  profile.addr = (strstr(addr, "ALL")) ? "free" : addr;
  profile.connector = this;
  profile.connect = &BluetoothConnector::bluetoothConnectorActivate;
  profile.disconnect = &BluetoothConnector::bluetoothConnectorDeactivate;
  profile.linkid = 0xFF;

  /* Listen all address can have only one profile at the last profile,
     otherwise a fixed address will take it at first */
  if (strstr(addr, "ALL") && (iwrap->profiles[7].addr == NULL)) {
    iwrap->profiles[7] = profile;
    return;
  }

  for(int i = 0; i < 8; i++) {
    if(iwrap->profiles[i].addr == NULL) {
      iwrap->profiles[i] = profile;
      break;
    }
  }
}

void BluetoothConnector::bluetoothConnectorConnect(const char *addr) {
  int size = strlen(addr) + 17;
  char cmd[size];
  strcpy(cmd, "CALL ");
  strcat(cmd, addr);

  for (int i = 0; i < 8; i++) {
    if (strstr(iwrap->profiles[i].addr, addr) && (iwrap->profiles[i].linkid == 0xFF)) {
      switch (typeDev[0]) {
      case 'S':           // 'S' for "SERIAL"
        strcat(cmd, " 1101 RFCOMM");
        break;
      case 'W':           // 'W' for "WIIMOTE"
        strcat(cmd, " 0013 L2CAP ");
        break;
      }
      iwrap->bluetoothIwrapSendCommand(cmd);
      BaseThread::sleep(MS2ST(1000));    // waiting connection establish
      break;
    }
  }
}

void BluetoothConnector::bluetoothConnectorDisconnect(const char *addr) {
  char cmd[8];
  strcpy(cmd, "CLOSE ");

  for (int i = 0; i < 8; i++) {
    if (strstr(iwrap->profiles[i].addr, addr) && (iwrap->profiles[i].linkid != 0xFF)) {
      cmd[6] = iwrap->profiles[i].linkid + 0x30;
      cmd[7]= '\0';
      iwrap->bluetoothIwrapSendCommand(cmd);
      BaseThread::sleep(MS2ST(2000));    // waiting disconnection
      break;
    }
  }
}

void BluetoothConnector::bluetoothConnectorActivate(uint8_t linkid) {
  if (strstr(typeDev, "SERIAL"))
    ((BluetoothSerial *) objDev)->bluetoothSerialStart(linkid);
  else
    ((BluetoothWiimote *) objDev)->bluetoothWiimoteStart(linkid);
}

void BluetoothConnector::bluetoothConnectorDeactivate() {
  if (strstr(typeDev, "SERIAL"))
    ((BluetoothSerial *) objDev)->bluetoothSerialStop();
  else
    ((BluetoothWiimote *) objDev)->bluetoothWiimoteStop();
}
