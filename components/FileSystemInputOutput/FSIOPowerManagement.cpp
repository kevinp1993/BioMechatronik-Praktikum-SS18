#include <amiro/FileSystemInputOutput/FSIOPowerManagement.hpp>

using namespace amiro;
using namespace amiro::fileSystemIo;

msg_t
FSIOPowerManagement::
getVcnl4020Offset (uint16_t *buffer, uint8_t idx) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(uint16_t);

  // Check if the vcnl index is in a valid range
  if (idx >= sizeof(PowerManagement_1_1::vcnl4020offset)) {
    return WRONG_INDEX;
  }

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIOPowerManagement::PowerManagement_1_1, vcnl4020offset[idx]));
      if (this->bsmv >= 0x01u) {
        bytesWritten = chSequentialStreamRead((BaseFileStream*)&at24c01, (uint8_t*) buffer, bufferSize);
      } else {
        return NOT_IMPLEMENTED;
      }
      break;
    }

    if (bytesWritten != bufferSize) {
      return IO_ERROR;
    } else {
      return OK;
    }
}

msg_t
FSIOPowerManagement::
setVcnl4020Offset (uint16_t buffer, uint8_t idx) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(uint16_t);

  // Check if the vcnl index is in a valid range
  if (idx >= sizeof(PowerManagement_1_1::vcnl4020offset)) {
    return WRONG_INDEX;
  }

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIOPowerManagement::PowerManagement_1_1, vcnl4020offset[idx]));
      if (this->bsmv >= 0x01u) {
        bytesWritten = chSequentialStreamWrite((BaseFileStream*)&at24c01, (const uint8_t*) &buffer, bufferSize);
      } else {
        return NOT_IMPLEMENTED;
      }
      break;
    }

    if (bytesWritten != bufferSize) {
      return IO_ERROR;
    } else {
      return OK;
    }
}
