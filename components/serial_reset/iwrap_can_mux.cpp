#include <ch.h>
#include <hal.h>
#include <string.h>  // memcpy
#include <chprintf.h>
#include <amiro/Constants.h>

#include <amiro/serial_reset/iwrap_can_mux.hpp>

using namespace amiro;
using namespace chibios_rt;

IwrapCanMux::IwrapCanMux(SerialDriver *_sd_, CANDriver *can, const uint8_t boardId)
  : BaseSequentialStreamInterface(),
    sd_(_sd_),
    canDriver(can)
{
    myID = boardId;
    queryShellID = boardId;
    replyShellID = boardId;

    resetStatus = 0;
    resetAddBytes = 0;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0;

    bluetoothStatus = BLUETOOTH_STATUS_START;
}

IwrapCanMux::~IwrapCanMux() {

};


/* Check bluetooth status
void IwrapCanMux::checkBluetoothStatus(void) {

      BluetoothUartReceiveByte(&byteData);
      switch (bluetoothStatus) {

        case BLUETOOTH_STATUS_START:
          if (byteData == BLUETOOTH_START_FRAME) {
            bluetoothStatus++;
          }
          break;

        case BLUETOOTH_STATUS_LINK:
            bluetoothLinkID = byteData;
            bluetoothStatus++;
          break;

        case BLUETOOTH_STATUS_FLAGS:
          if (byteData == BLUETOOTH_FLAGS) {
            bluetoothStatus++;
          } else {
            bluetoothStatus = BLUETOOTH_STATUS_START;
          }
          break;

        case BLUETOOTH_STATUS_LENGTH:
          bluetoothStatus++;
          bluetoothDataLength = byteData;
          break;

        case BLUETOOTH_STATUS_DATA:
          if (--bluetoothDataLength == 0) {
            bluetoothStatus++;
          }
          if (bluetoothLinkID == 0xFF) {
          } else {
            // read data
          }
          break;

        default:
          bluetoothStatus = BLUETOOTH_STATUS_START;
          if (byteData == ~bluetoothLinkID) {
            // iWRAP check
          }
          break;
      }
}
*/


/*
 * resets system for entering in bootloader.
 */
void IwrapCanMux::restartSystem(void) {
  //TODO Set reset SYS_PD_N
  NVIC_SystemReset();
}

/*
 * checks bytes for reset command sequence.
 * if the last byte and the following identification number of the device have been sent,
 * the microcontroller will be reset
 */
bool IwrapCanMux::checkByteForBLReset(uint8_t *inputs, size_t size) {
  /* Run throug all given bytes */
  for (size_t idx=0; idx < size; idx++) {
    uint8_t input = *(inputs+idx);

    /*
     * if the last byte of reset command sequence has not been sent yet,
     * check new byte and handle reset status
     */
    if (resetStatus < INPUT_BL_RESET_LENGTH) {
      if (input == inputBLReset[resetStatus]) {
        resetStatus++;
        if (resetStatus == INPUT_BL_RESET_LENGTH) {
          chprintf((BaseSequentialStream*)sd_, "\n\nDo reset ...\n\n");
          uint8_t ledCount;
          for (ledCount=0; ledCount<20; ledCount++) {
//            boardWriteLed(1);
            chThdSleepMilliseconds(100);
//            boardWriteLed(0);
            chThdSleepMilliseconds(100);
          }
          restartSystem();
        }
      } else {
        resetStatus = 0;
        resetAddBytes = 0;
      }

    /*
     * if the last byte of reset command sequence has been sent,
     * save new byte as identification number of the device which shall be flashed and
     * reset microcontroller
     */
    } else {
      inputAddBytes[resetAddBytes] = input;
      resetAddBytes++;
      if (resetAddBytes >= INPUT_BL_RESET_ADDITIONAL_BYTES) {
        chprintf((BaseSequentialStream*)sd_, "\n\nAt device %x: Do reset ...\n\n", input);
        chThdSleepMilliseconds(OFFSET_TIME_RESET_MS);
        restartSystem();
      }
    }
  }

  return (resetStatus > 0);
}

msg_t IwrapCanMux::sendViaCan(uint32_t id, uint8_t *inputs, size_t size) {
  msg_t status;
  txmsg.SID = (id << CAN::DEVICE_ID_SHIFT) | myID;
  for (size_t i=0; i<size; i+=8) {
    txmsg.DLC = (size < (i + 8)) ? size-i : 8;
    memcpy(txmsg.data8, &inputs[i], txmsg.DLC);
    status = canTransmit(canDriver, CAN_TX_MAILBOXES, &txmsg, TIME_INFINITE);
    if (status!= RDY_OK)
      return status;
  }

  return RDY_OK;
}

/*
 * Interface implementation.
 */

size_t IwrapCanMux::write(const uint8_t *bp, size_t n) {
  size_t size;

  if (myID != replyShellID) {
     msg_t status = sendViaCan(CAN::SHELL_REPLY_ID(replyShellID), (uint8_t *)bp, n);
     BaseThread::sleep(US2ST(500));
     size = (status == RDY_OK) ? n : 0;
  } else {
    size = sdWrite(sd_, bp, n);
  }

  return size;
}

size_t IwrapCanMux::read(uint8_t *bp, size_t n) {
  size_t size = sdRead(sd_, inputChar, n);

  checkByteForBLReset(inputChar, size);

  if (myID != queryShellID)
    sendViaCan(CAN::SHELL_QUERY_ID(queryShellID),inputChar, size);
  else
    memcpy(bp, inputChar, size);

  return size;
}

msg_t IwrapCanMux::put(uint8_t b) {
  msg_t byte;

  if (myID != replyShellID) {
    byte = sendViaCan(CAN::SHELL_REPLY_ID(replyShellID), &b, 1);
    BaseThread::sleep(US2ST(200));
  } else {
    byte = sdPut(sd_, b);
  }

  return byte;
}

msg_t IwrapCanMux::get(void) {
  msg_t byte = sdGet(sd_);
  uint8_t b = (uint8_t) byte;

  checkByteForBLReset(&b, 1);

  if (myID != queryShellID)
    sendViaCan(CAN::SHELL_QUERY_ID(queryShellID), &b, 1);

  return byte;
}

//----------------------------------------------------------------

/*
 * Converse data from CAN bus into input queue of serial driver.
 */

void IwrapCanMux::convCan2Serial(uint8_t *data_can, size_t n) {
  chSysLock();
  for (size_t i=0; i < n; i++)
     sdIncomingDataI(sd_, data_can[i]);
  chSysUnlock();
}

void IwrapCanMux::sendSwitchCmd(uint8_t setid) {

  if (myID == setid)
    chprintf((BaseSequentialStream*)sd_, "Stay at the same shell\n");
  else if (replyShellID == setid)
    chprintf((BaseSequentialStream*)sd_, "Switch to the own shell\n");
  else
    chprintf((BaseSequentialStream*)sd_, "Switch to the shell on board %d\n", setid);

  if (myID != setid) {

    if (replyShellID != setid)  {
      // To set the new remote shell via CAN bus.
      txmsg.SID = ((CAN::SHELL_QUERY_ID(setid)) << CAN::DEVICE_ID_SHIFT) | replyShellID;
      txmsg.DLC = 0;
      canTransmit(canDriver, CAN_ANY_MAILBOX, &txmsg, US2ST(545));
    }

    if (replyShellID == myID) {
      queryShellID = setid;
    } else {
      if (replyShellID == setid)
        // To return the own shell via CAN bus.
        txmsg.SID = ((CAN::SHELL_QUERY_ID(replyShellID)) << CAN::DEVICE_ID_SHIFT) | replyShellID;
      else
        // To set the serial board to the new remote shell.
        txmsg.SID = ((CAN::SHELL_QUERY_ID(replyShellID)) << CAN::DEVICE_ID_SHIFT) | setid;

      txmsg.DLC = 0;
      canTransmit(canDriver, CAN_ANY_MAILBOX, &txmsg, US2ST(545));

      replyShellID = myID;
    }
  }
}

void IwrapCanMux::rcvSwitchCmd(uint8_t setid) {

  if (myID == setid) {
    chprintf((BaseSequentialStream*)sd_, "\nReturn own shell\n\n");
    queryShellID = setid;
  } else if (myID != queryShellID) {
    chprintf((BaseSequentialStream*)sd_, "\nSwitch to the shell on board %d\n", setid);
    queryShellID = setid;
  } else {
    chprintf((BaseSequentialStream*)sd_, "\nShell shown in board %d\n\n", setid);
    replyShellID = setid;
  }
}
