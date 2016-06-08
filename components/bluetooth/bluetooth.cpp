#include <ch.hpp>
#include <hal.h>

#include <string.h>

#include <amiro/bluetooth/bluetooth.hpp>

using namespace chibios_rt;
using namespace amiro;

/*
 * Class constructor
 */
BLUETOOTH::BLUETOOTH(UARTDriver* uart) : iwrap(uart) {
}

/*
 * Member functions
 */
void BLUETOOTH::bluetoothStart() {
  iwrap.start(NORMALPRIO);
  BaseThread::sleep(MS2ST(1000));
  // Reset bluetooth chip to get the first msg. (cmd/data mode or mux Mode)
  bluetoothReset();
}

void BLUETOOTH::bluetoothReset() {
  palWritePad(GPIOC, GPIOC_BT_RST, 1);
  BaseThread::sleep(MS2ST(10));
  palWritePad(GPIOC, GPIOC_BT_RST, 0);
  iwrap.transport.bluetoothTransportResetState();
    BaseThread::sleep(MS2ST(1000));
}

void BLUETOOTH::bluetoothEnableMux() {
  iwrap.bluetoothIwrapSendCommand("SET CONTROL MUX 1");
}

void BLUETOOTH::bluetoothDisableMux() {
  iwrap.bluetoothIwrapSendCommand("SET CONTROL MUX 0");
}

void BLUETOOTH::bluetoothDiscoverDevices() {
  iwrap.bluetoothIwrapSendCommand("INQUIRY 5");
  BaseThread::sleep(MS2ST(6000));
}

void BLUETOOTH::bluetoothListAllConnections() {
  iwrap.bluetoothIwrapSendCommand("List");
}

void BLUETOOTH::bluetoothSetName(const char *name) {
  int size = strlen(name) + 14;
  char cmd[size];
  strcpy(cmd, "SET BT NAME ");
  strcat(cmd, name);
  iwrap.bluetoothIwrapSendCommand(cmd);
}

void BLUETOOTH::bluetoothSetPin(const char *pin) {
  int size = strlen(pin) + 15;
  char cmd[size];
  strcpy(cmd, "SET BT AUTH * ");
  strcat(cmd, pin);
  iwrap.bluetoothIwrapSendCommand(cmd);
}

void BLUETOOTH::bluetoothCloseConnection(uint8_t linkid) {
  char cmd[8];
  strcpy(cmd, "CLOSE ");
  cmd[6] = linkid + '0';
  cmd[7]= '\0';
  iwrap.bluetoothIwrapSendCommand(cmd);
}

void BLUETOOTH::bluetoothSendCommand(const char *cmd) {
  iwrap.bluetoothIwrapSendCommand(cmd);
}

bool BLUETOOTH::bluetoothIsMuxMode() {
  return iwrap.transport.bluetoothTransportIsMuxMode();
}
