#include <amiro/FileSystemInputOutput/FileSystemInputOutputBase.hpp>

using namespace amiro;
using namespace amiro::fileSystemIo;

msg_t
FileSystemIoBase::init()
{
  msg_t returnValue = OK;

  // Get the preamble
  const uint8_t bufferSize = sizeof(FileSystemIoBase::preamble_1_1);
  uint8_t buffer[bufferSize];
  chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FileSystemIoBase::preamble_1_1, magicByte));
  const size_t bytesRead = chSequentialStreamRead((BaseFileStream*)&at24c01, (uint8_t*) buffer, bufferSize);

  // reading failed -> error
  if (bytesRead != bufferSize) {
    returnValue |= IO_ERROR;
    return returnValue;
  }

  // magic byte is incorrect -> error
  if (buffer[offsetof(FileSystemIoBase::preamble_1_1, magicByte)] != magicByteValue()) {
    returnValue |= WRONG_MAGICBYTE_VALUE;
    return returnValue;
  }

  // BSMV is incorrect -> error
  if (buffer[offsetof(FileSystemIoBase::preamble_1_1, byteStructureMajorVersion)] != BSMV) {
    returnValue |= WRONG_BSMV;
    return returnValue;
  }

  // bsmv in smaller -> initialize
  if (buffer[offsetof(FileSystemIoBase::preamble_1_1, byteStructureMinorVersion)] < bsmv) {
    returnValue |= WRONG_bsmv;
    // initialize all missing data
    for (uint8_t v = bsmv; v > buffer[offsetof(FileSystemIoBase::preamble_1_1, byteStructureMinorVersion)]; --v)
    {
      returnValue |= initPreamble(BSMV, v, false);
    }
  }

  // HMV is incorrect -> error
  if (buffer[offsetof(FileSystemIoBase::preamble_1_1, hardwareMajorVersion)] != HMV) {
    returnValue |= WRONG_HMV;
    return returnValue;
  }

  // hmv is incorrect -> error
  if (buffer[offsetof(FileSystemIoBase::preamble_1_1, hardwareMinorVersion)] >= hmv) {
    returnValue |= WRONG_hmv;
  }

  return returnValue;
}

msg_t
FileSystemIoBase::
getCheck() {

  // Get the preamble
  const uint8_t bufferSize = sizeof(FileSystemIoBase::preamble_1_1);
  uint8_t buffer[bufferSize];
  chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FileSystemIoBase::preamble_1_1, magicByte));
  const size_t bytesRead = chSequentialStreamRead((BaseFileStream*)&at24c01, (uint8_t*) buffer, bufferSize);

  if (bytesRead != bufferSize) {
    return IO_ERROR;
  } else if (buffer[offsetof(FileSystemIoBase::preamble_1_1, magicByte)] != magicByteValue()) {
    return WRONG_MAGICBYTE_VALUE;
  } else if (buffer[offsetof(FileSystemIoBase::preamble_1_1, byteStructureMajorVersion)] != BSMV) {
    return WRONG_BSMV;
  } else if (buffer[offsetof(FileSystemIoBase::preamble_1_1, byteStructureMinorVersion)] < bsmv) {
    return WRONG_bsmv;
  } else if (buffer[offsetof(FileSystemIoBase::preamble_1_1, hardwareMajorVersion)] != HMV) {
    return WRONG_HMV;
  } else if (buffer[offsetof(FileSystemIoBase::preamble_1_1, hardwareMinorVersion)] < hmv) {
    return WRONG_hmv;
  } else {
    return OK;
  }
}

msg_t
FileSystemIoBase::initPreamble(const uint8_t BSMV, const uint8_t bsmv, const bool strict)
{
  uint8_t buffer[3] = {};

  if (strict) {
    // initialize the magic byte
    chFileStreamSeek((BaseFileStream*)&this->at24c01, offsetof(FileSystemIoBase::preamble_1_1, magicByte));
    buffer[0] = magicByteValue();
    if (chSequentialStreamWrite((BaseFileStream*)&this->at24c01, buffer, 1) != 1) {
      return IO_ERROR;
    }
  }

  switch (BSMV)
  {
    case 1:
      if (strict) {
        // initialize the BSMV
        chFileStreamSeek((BaseFileStream*)&this->at24c01, offsetof(FileSystemIoBase::preamble_1_1, byteStructureMajorVersion));
        buffer[0] = BSMV;
        if (chSequentialStreamWrite((BaseFileStream*)&this->at24c01, buffer, 1) != 1) {
          return IO_ERROR;
        }
      }
      // initialize the bsmv
      chFileStreamSeek((BaseFileStream*)&this->at24c01, offsetof(FileSystemIoBase::preamble_1_1, byteStructureMinorVersion));
      buffer[0] = bsmv;
      if (chSequentialStreamWrite((BaseFileStream*)&this->at24c01, buffer, 1) != 1) {
        return IO_ERROR;
      }
      // initialize the content
      switch (bsmv)
      {
        case 2:
          // initialize the board ID
          chFileStreamSeek((BaseFileStream*)&this->at24c01, offsetof(FileSystemIoBase::preamble_1_2, boardId));
          buffer[0] = 0;
          if (chSequentialStreamWrite((BaseFileStream*)&this->at24c01, buffer, 1) != 1) {
            return IO_ERROR;
          }
          if (!strict) {
            break;
          }

        case 1:
          // initialize the bsmv, the HMV and the hmv
          chFileStreamSeek((BaseFileStream*)&this->at24c01, offsetof(FileSystemIoBase::preamble_1_1, byteStructureMinorVersion));
          buffer[0] = bsmv;
          buffer[1] = this->HMV;
          buffer[2] = this->hmv;
          if (chSequentialStreamWrite((BaseFileStream*)&this->at24c01, buffer, 3) != 3) {
            return IO_ERROR;
          }
          if (!strict) {
            break;
          }

        default:
          break;

      }
      break;

    default:
      break;
  }

  return OK;
}

msg_t
FileSystemIoBase::
resetMemory() {
  return initPreamble(this->BSMV, this->bsmv, true);
}

msg_t
FileSystemIoBase::
getBoardId (uint8_t *buffer) {

  msg_t bytesRead = 0;
  // Work around, because stm32f1 cannot read a single byte
  const uint8_t bufferSize = 2;
  uint8_t bufferTmp[bufferSize] = {};

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FileSystemIoBase::preamble_1_2, boardId));
      if (this->bsmv >= 0x02u) {
        bytesRead = chSequentialStreamRead((BaseFileStream*)&at24c01, (uint8_t*) bufferTmp, bufferSize);
      } else {
        return NOT_IMPLEMENTED;
      }
      break;
    }

  // Store only the board id
  *buffer = bufferTmp[0];

    if (bytesRead != bufferSize) {
      return IO_ERROR;
    } else {
      return OK;
    }
}

msg_t
FileSystemIoBase::
setBoardId (uint8_t buffer) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(uint8_t);

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FileSystemIoBase::preamble_1_2, boardId));
      if (this->bsmv >= 0x02u) {
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
