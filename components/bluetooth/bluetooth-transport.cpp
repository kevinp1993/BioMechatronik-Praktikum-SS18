#include <ch.hpp>
#include <hal.h>

#include <uart.h>
#include <chprintf.h>

#include <amiro/bluetooth/bluetooth-transport.hpp>

using namespace chibios_rt;
using namespace amiro;

/*
 * Class constructor
 */
BluetoothTransport::BluetoothTransport(UARTDriver* uart) :
  BaseStaticThread<128>(), UARTConfig(), uart(uart), receiveMailbox{NULL},
  storageMailbox(storageMailboxBuffer, BLUETOOTH_TRANSPORT_STORAGE_MAILBOX_SIZE),
  transmitMailbox(transmitMailboxBuffer, BLUETOOTH_TRANSPORT_TRANSMIT_MAILBOX_SIZE) {

  UARTConfig::txend1_cb = bluetoothTransportTransmitCompleted_cb;
  UARTConfig::txend2_cb = NULL;
  UARTConfig::rxend_cb  = bluetoothTransportReceiveCompleted_cb;
  UARTConfig::rxchar_cb = bluetoothTransportCharacterReceived_cb;
  UARTConfig::rxerr_cb  = NULL;
  speed = 115200;
  cr1   = 0;
  cr2   = 0;
  cr3   = 0;

  transmitState = BLUETOOTH_TRANSPORT_PLAIN;
  receiveState = BLUETOOTH_TRANSPORT_PLAIN;

  transmit = NULL;
  receive = NULL;
}

//----------------------------------------------------------------

msg_t BluetoothTransport::main(void) {
  setName("BluetoothTransport");

  for (int i = 0; i < BLUETOOTH_TRANSPORT_STORAGE_MAILBOX_SIZE; i++)
    storageMailbox.post((msg_t) &descriptor[i], TIME_INFINITE);

  BaseThread::sleep(MS2ST(200));
  uartStart(uart, this);

  while (!this->shouldTerminate()) {

    if (!this->transmitMailbox.fetch((msg_t *) &transmit, TIME_INFINITE)) {

      if (!transmit->bluetoothDescriptorGetPayloadLength()) {
        storageMailbox.post((msg_t) transmit, TIME_INFINITE);
            continue;
      }

      if (transmitState == BLUETOOTH_TRANSPORT_PLAIN) {
        uint8_t* buffer = transmit->bluetoothDescriptorGetPayload();
        size_t length = transmit->bluetoothDescriptorGetPayloadLength();

        memcpy(buffer+length, "\r\n", 2);

        uartStartSend(uart, length + 2, buffer);
      } else {
        transmitState = BLUETOOTH_TRANSPORT_PAYLOAD;
        uartStartSend(uart, FRAME_HEADER_SIZE, getEncodeHeader());
      }
      waitAnyEvent((eventmask_t) TRANSMIT_COMPLETED_EVENT);
    }
  }
  return RDY_OK;
}

/*
 * Interrupt callback handlers
 */
void BluetoothTransport::bluetoothTransportTransmitCompleted_cb(UARTDriver *uart) {
  BluetoothTransport *u = (BluetoothTransport *) uart->config;
  u->bluetoothTransportTransmitCompleted();
}

void BluetoothTransport::bluetoothTransportTransmitCompleted() {
  switch (transmitState) {
    case BLUETOOTH_TRANSPORT_PLAIN: {
      postTransmitDescriptorI();

      System::lockFromIsr();
      signalEventsI((eventmask_t) TRANSMIT_COMPLETED_EVENT);
      System::unlockFromIsr();
      break;
    }
    case BLUETOOTH_TRANSPORT_HEADER: {
      System::lockFromIsr();
      uartStartSendI(uart, FRAME_HEADER_SIZE, getEncodeHeader());
      System::unlockFromIsr();
      transmitState = BLUETOOTH_TRANSPORT_PAYLOAD;
      /* no break */
    }
    case BLUETOOTH_TRANSPORT_PAYLOAD: {
      System::lockFromIsr();
      uartStartSendI(uart, transmit->bluetoothDescriptorGetPayloadLength(), transmit->bluetoothDescriptorGetPayload());
      System::unlockFromIsr();
      transmitState = BLUETOOTH_TRANSPORT_TRAILER;
      break;
    }
    case BLUETOOTH_TRANSPORT_TRAILER: {
      postTransmitDescriptorI();

      System::lockFromIsr();
      uartStartSendI(uart, FRAME_TRAILER_SIZE, getEncodeTailer());

      // check whether any new transmision in the mailbox during the present tranmmision
      msg_t msg = transmitMailbox.fetchI((msg_t *) &transmit);
      System::unlockFromIsr();

      if (msg == RDY_OK) {
        if (transmit->bluetoothDescriptorGetPayloadLength()) {
          transmitState = BLUETOOTH_TRANSPORT_HEADER;
        } else {
          postTransmitDescriptorI();
          transmitState = BLUETOOTH_TRANSPORT_IDLE;
        }
      } else {
        transmitState = BLUETOOTH_TRANSPORT_IDLE;
      }
      break;
    }
    case BLUETOOTH_TRANSPORT_IDLE: {
      System::lockFromIsr();
      signalEventsI((eventmask_t) TRANSMIT_COMPLETED_EVENT);
      System::unlockFromIsr();
      break;
    }
    default: {
      break;
    }
  }
}

void BluetoothTransport::bluetoothTransportCharacterReceived_cb(UARTDriver *uart, uint16_t c) {
  BluetoothTransport *u = (BluetoothTransport *) uart->config;
  u->bluetoothTransportCharacterReceived(c);
}

void BluetoothTransport::bluetoothTransportCharacterReceived(uint16_t c) {
  if (c == 0xbf) {
    receiveFrameBuffer[1] = 0xbf;
    System::lockFromIsr();
    uartStartReceiveI(uart, 3, (void *) &receiveFrameBuffer[2]);
    System::unlockFromIsr();

    receiveState = BLUETOOTH_TRANSPORT_DECODE;
    if (transmitState == BLUETOOTH_TRANSPORT_PLAIN)
      transmitState = BLUETOOTH_TRANSPORT_IDLE;
  }

  if (receiveState != BLUETOOTH_TRANSPORT_PLAIN)
    return;

  if (!receive) {
    System::lockFromIsr();
    msg_t msg = storageMailbox.fetchI((msg_t *) &receive);
    System::unlockFromIsr();

    if (msg)
      return;

    receive->bluetoothDescriptorSetLinkId(0xFF);
    rcvlength = 0;
  }

  uint8_t* buffer = receive->bluetoothDescriptorGetPayload();
  buffer[rcvlength++] = c;

  if (rcvlength == receive->bluetoothDescriptorGetBufferLength() || c == '\n') {
    receive->bluetoothDescriptorSetPayloadLength(rcvlength);
    postReceiveDescriptorI();
  }
}

void BluetoothTransport::bluetoothTransportReceiveCompleted_cb(UARTDriver *uart) {
  BluetoothTransport *u = (BluetoothTransport *) uart->config;
  u->bluetoothTransportReceiveCompleted();
}

void BluetoothTransport::bluetoothTransportReceiveCompleted() {
  switch (receiveState) {
    case BLUETOOTH_TRANSPORT_HEADER: {
      if ((uint8_t) ~receiveFrameBuffer[0] != receive->bluetoothDescriptorGetLinkId()) {
        receiveState = BLUETOOTH_TRANSPORT_IDLE;
        break;
      }
      System::lockFromIsr();
      uartStartReceiveI(uart, 4, (void *) &receiveFrameBuffer[1]);
      System::unlockFromIsr();

      receiveState = BLUETOOTH_TRANSPORT_DECODE;
      break;
    }
    case BLUETOOTH_TRANSPORT_DECODE: {
      if (decodeHeader()) {
        receiveState = BLUETOOTH_TRANSPORT_IDLE;
        break;
      }
      receiveState = BLUETOOTH_TRANSPORT_PAYLOAD;
      /* no break */
    }
    case BLUETOOTH_TRANSPORT_PAYLOAD: {
      if (receive)
        postReceiveDescriptorI();

      System::lockFromIsr();
      msg_t msg = storageMailbox.fetchI((msg_t *) &receive);
      System::unlockFromIsr();

      if (msg) {
        receiveState = BLUETOOTH_TRANSPORT_IDLE;
        break;
      }

      receive->bluetoothDescriptorSetLinkId(receiveFrameBuffer[2]);
      if (rcvlength > receive->bluetoothDescriptorGetBufferLength())
        receive->bluetoothDescriptorSetPayloadLength(receive->bluetoothDescriptorGetBufferLength());
      else
        receive->bluetoothDescriptorSetPayloadLength(rcvlength);

      System::lockFromIsr();
      uartStartReceiveI(uart, receive->bluetoothDescriptorGetPayloadLength(), receive->bluetoothDescriptorGetPayload());
      System::unlockFromIsr();

      rcvlength -= receive->bluetoothDescriptorGetPayloadLength();

      if (!rcvlength)
        receiveState = BLUETOOTH_TRANSPORT_TRAILER;

      break;
    }
    case BLUETOOTH_TRANSPORT_TRAILER: {
      postReceiveDescriptorI();

      System::lockFromIsr();
      uartStartReceiveI(uart, 1, (void *) &receiveFrameBuffer[0]);
      System::unlockFromIsr();

      receiveState = BLUETOOTH_TRANSPORT_HEADER;
      break;
    }
    default: {
      break;
    }
  }
}

/*
 * Member functions
 */

void BluetoothTransport::bluetoothTransportResetState() {
  transmitState = BLUETOOTH_TRANSPORT_PLAIN;
  receiveState = BLUETOOTH_TRANSPORT_PLAIN;
}

bool BluetoothTransport::bluetoothTransportIsMuxMode() {
  if (receiveState == BLUETOOTH_TRANSPORT_PLAIN)
    return false;

  return true;
}

chibios_rt::Mailbox* BluetoothTransport::bluetoothTransportGetStorageMailbox() {
  return &storageMailbox;
}

chibios_rt::Mailbox* BluetoothTransport::bluetoothTransportGetTransmitMailbox() {
  return &transmitMailbox;
}

void BluetoothTransport::bluetoothTransportSetReceiveMailbox(uint8_t linkId, chibios_rt::Mailbox* mailbox) {
  System::lock();
  receiveMailbox[(linkId > 9) ? 9 : linkId] = mailbox;
  System::unlock();
}

void BluetoothTransport::postReceiveDescriptorI() {
  uint8_t linkId = receive->bluetoothDescriptorGetLinkId();
  linkId > 9 ? linkId = 9 : linkId;

  System::lockFromIsr();
  msg_t msg = receiveMailbox[linkId]->postI((msg_t) receive);
  if (msg)
    storageMailbox.postI((msg_t) receive);
  receive = NULL;
  System::unlockFromIsr();
}

void BluetoothTransport::postTransmitDescriptorI() {
  System::lockFromIsr();
  storageMailbox.postI((msg_t) transmit);
  transmit = NULL;
  System::unlockFromIsr();
}

/*
 * The header encode/decode are in compliance with of the multiplexing mode
 * Reference: iWRAP4 user guide Version 4.1 page 142-143
 */
uint8_t*  BluetoothTransport::getEncodeHeader() {
  transmitFrameBuffer[1] = 0xbf;
  transmitFrameBuffer[2] = transmit->bluetoothDescriptorGetLinkId();
  transmitFrameBuffer[3] = (transmit->bluetoothDescriptorGetPayloadLength() >> 8) & 0x3;
  transmitFrameBuffer[4] = transmit->bluetoothDescriptorGetPayloadLength() & 0xFF;
  return &transmitFrameBuffer[1];
}

uint8_t* BluetoothTransport::getEncodeTailer() {
  transmitFrameBuffer[0] = ~transmitFrameBuffer[2];
  return &transmitFrameBuffer[0];
}

int BluetoothTransport::decodeHeader() {
  if ((receiveFrameBuffer[1] != 0xbf) ||
    ((receiveFrameBuffer[2] > 8) && (receiveFrameBuffer[2] != 0xFF)))
    return -1;

  rcvlength = (receiveFrameBuffer[3] << 8) | receiveFrameBuffer[4];
  if (rcvlength > 1023)
    return -1;

  return RDY_OK;
}
