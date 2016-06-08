#include <amiro/FileSystemInputOutput/FSIODiWheelDrive.hpp>

using namespace amiro;
using namespace amiro::fileSystemIo;

msg_t
FSIODiWheelDrive::
getVcnl4020Offset (uint16_t *buffer, uint8_t idx) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(uint16_t);

  // Check if the vcnl index is in a valid range
  if (idx >= sizeof(DiWheelDrive_1_1::vcnl4020offset)) {
    return WRONG_INDEX;
  }

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_1, vcnl4020offset[idx]));
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
FSIODiWheelDrive::
setVcnl4020Offset (uint16_t buffer, uint8_t idx) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(uint16_t);

  // Check if the vcnl index is in a valid range
  if (idx >= sizeof(DiWheelDrive_1_1::vcnl4020offset)) {
    return WRONG_INDEX;
  }

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_1, vcnl4020offset[idx]));
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


msg_t
FSIODiWheelDrive::
setwheelfactor (float buffer) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(float);

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_3, wheelfactor));
      if (this->bsmv >= 0x03u) {
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


msg_t
FSIODiWheelDrive::
getWheelFactor (float *buffer) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(float);

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_3, wheelfactor));
      if (this->bsmv >= 0x03u) {
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
FSIODiWheelDrive::
setiGain (float buffer) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(float);

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_3, igain));
      if (this->bsmv >= 0x03u) {
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


msg_t
FSIODiWheelDrive::
getiGain (float *buffer) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(float);

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_3, igain));
      if (this->bsmv >= 0x03u) {
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
FSIODiWheelDrive::
setdGain (float buffer) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(float);

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_3, dgain));
      if (this->bsmv >= 0x03u) {
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


msg_t
FSIODiWheelDrive::
getdGain (float *buffer) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(float);

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_3, dgain));
      if (this->bsmv >= 0x03u) {
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
FSIODiWheelDrive::
setpGain (int buffer) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(float);

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_3, pgain));
      if (this->bsmv >= 0x03u) {
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


msg_t
FSIODiWheelDrive::
getpGain (int *buffer) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(float);

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_3, pgain));
      if (this->bsmv >= 0x03u) {
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
FSIODiWheelDrive::
getEd (float *buffer) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(float);

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_2, Ed));
      if (this->bsmv >= 0x02u) {
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
FSIODiWheelDrive::
setEd (float buffer) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(float);

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_2, Ed));
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

msg_t
FSIODiWheelDrive::
getEb (float *buffer) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(float);

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_2, Eb));
      if (this->bsmv >= 0x02u) {
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
FSIODiWheelDrive::
setEb (float buffer) {

  msg_t bytesWritten = 0;
  const uint8_t bufferSize = sizeof(float);

  // Get the data out of the memory
  switch (this->BSMV) {
    case 0x01u:
      // Set the pointer to the address
      chFileStreamSeek((BaseFileStream*)&at24c01, offsetof(FSIODiWheelDrive::DiWheelDrive_1_2, Eb));
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
