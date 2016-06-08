#ifndef AMIRO_FILESYSTEMIO_H_
#define AMIRO_FILESYSTEMIO_H_

/*! \brief memory layouts for the eeproms
 *
 * This header defines the memory layout.
 * Every memory layout starts with a preamble
 * of 3 byte. The first one is a magic byte
 * for checking purpose. The second one declares
 * the byte structure major version (BSMV) and the third
 * the byte structure minor version (bsmv) of the subsequently
 * bytes in the memory. With different major versions, the
 * layout of the memory may totaly change, so one should
 * not read from such a different layout if the version
 * differs from the expected one. With greater minor
 * versions the old memory layout may not change, but
 * new elements emerge.
 */

#include <amiro/eeprom/at24.hpp>
#include <chprintf.h>

namespace amiro {
namespace fileSystemIo {

  class FileSystemIoBase {

  public:
    AT24 *at24c01;
    const uint8_t BSMV;
    const uint8_t bsmv;
    const uint8_t HMV;
    const uint8_t hmv;
    const uint8_t magicByteValue = 0xAAu;

    // TODO implement CRC check
    struct preamble {
      uint8_t magicByte;
      uint8_t byteStructureMajorVersion;
      uint8_t byteStructureMinorVersion;
      uint8_t hardwareMajorVersion;
      uint8_t hardwareMinorVersion;
      uint8_t reserved_0x04_0x14[15];
    };

    enum{
      OK                    = 0x00u,
      NOT_IMPLEMENTED       = 0x01u,
      WRONG_BSMV            = 0x02u,
      WRONG_bsmv            = 0x03u,
      IO_ERROR              = 0x04u,
      WRONG_MAGICBYTE_VALUE = 0x05u,
      WRONG_INDEX           = 0x06u,
      WRONG_HMV             = 0x07u,
      WRONG_hmv             = 0x08u,
    };

  public:

    FileSystemIoBase(AT24 *at24c01, uint8_t BSMV, uint8_t bsmv, uint8_t HMV, uint8_t hmv)
      : at24c01(at24c01),
        BSMV(BSMV),
        bsmv(bsmv),
        HMV(HMV),
        hmv(hmv) {
     }

    msg_t getCheck(){

      // Get the preamble
      const uint8_t bufferSize = sizeof(FileSystemIoBase::preamble);
      uint8_t buffer[bufferSize];
      chFileStreamSeek((BaseFileStream*)at24c01, offsetof(FileSystemIoBase::preamble, magicByte));
      msg_t bytesWritten = chSequentialStreamRead((BaseFileStream*)at24c01, (uint8_t*) buffer, sizeof(FileSystemIoBase::preamble));

      if (bytesWritten != bufferSize) {
        return IO_ERROR;
      } else if (buffer[offsetof(FileSystemIoBase::preamble, magicByte)] != magicByteValue) {
        return WRONG_MAGICBYTE_VALUE;
      } else if (buffer[offsetof(FileSystemIoBase::preamble, byteStructureMajorVersion)] != BSMV) {
        return WRONG_BSMV;
      } else if (buffer[offsetof(FileSystemIoBase::preamble, byteStructureMinorVersion)] < bsmv) {
        return WRONG_bsmv;
      } else if (buffer[offsetof(FileSystemIoBase::preamble, hardwareMajorVersion)] != HMV) {
        return WRONG_HMV;
      } else if (buffer[offsetof(FileSystemIoBase::preamble, hardwareMinorVersion)] < hmv) {
        return WRONG_hmv;
      } else {
        return OK;
      }
    }

    msg_t initMemory(){

      // Get the preamble
      const uint8_t bufferSize = offsetof(FileSystemIoBase::preamble, hardwareMinorVersion) + 1;
      uint8_t buffer[bufferSize];

      chFileStreamSeek((BaseFileStream*)at24c01, offsetof(FileSystemIoBase::preamble, magicByte));

      buffer[offsetof(FileSystemIoBase::preamble, magicByte)]                 = magicByteValue;
      buffer[offsetof(FileSystemIoBase::preamble, byteStructureMajorVersion)] = BSMV;
      buffer[offsetof(FileSystemIoBase::preamble, byteStructureMinorVersion)] = bsmv;
      buffer[offsetof(FileSystemIoBase::preamble, hardwareMajorVersion)]      = HMV;
      buffer[offsetof(FileSystemIoBase::preamble, hardwareMinorVersion)]      = hmv;

      msg_t bytesWritten = chSequentialStreamWrite((BaseFileStream*)at24c01, (uint8_t*) buffer, bufferSize);

      if (bytesWritten != bufferSize) {
        return IO_ERROR;
      } else {
        return OK;
      }
    }



};


class DiWheelDriveFSIO : public FileSystemIoBase {
  private:

    /** \brief Layout for DiWheelDriveFSIO with BSMV 1 and bsmv 1*/
    struct DiWheelDrive_1_1 {
      uint8_t  reserved_0x00_0x14[20];
      uint16_t vcnl4020offset[4];       // U [12, 13, 15, 16]
      uint8_t  generalPurpose[96];      // 1Cx_7Cx
      uint8_t  reserved_0x7D_0x80[4];   // TODO CRC
    };

  public:
    DiWheelDriveFSIO(AT24 *at24c01, uint8_t BSMV, uint8_t bsmv, uint8_t HMV, uint8_t hmv)
      : FileSystemIoBase(at24c01, BSMV, bsmv, HMV, hmv) {

    }

    msg_t getVcnl4020Offset (uint16_t *buffer, uint8_t idx) {

      msg_t bytesWritten;
      const uint8_t bufferSize = sizeof(uint16_t);

      // Check if the vcnl index is in a valid range
      if (idx >= sizeof(DiWheelDrive_1_1::vcnl4020offset)) {
        return WRONG_INDEX;
      }

      // Get the data out of the memory
      switch (this->BSMV) {
        case 0x01u:
          // Set the pointer to the address
          chFileStreamSeek((BaseFileStream*)at24c01, offsetof(DiWheelDriveFSIO::DiWheelDrive_1_1, vcnl4020offset[idx]));
          switch (this->bsmv) {
            default:
              bytesWritten = chSequentialStreamRead((BaseFileStream*)at24c01, (uint8_t*) buffer, bufferSize);
              break;
          }
          break;
        }

        if (bytesWritten != bufferSize) {
          return IO_ERROR;
        } else {
          return OK;
        }
    }

    msg_t setVcnl4020Offset (uint16_t *buffer, uint8_t idx) {

      msg_t bytesWritten;
      const uint8_t bufferSize = sizeof(uint16_t);

      // Check if the vcnl index is in a valid range
      if (idx >= sizeof(DiWheelDrive_1_1::vcnl4020offset)) {
        return WRONG_INDEX;
      }

      // Get the data out of the memory
      switch (this->BSMV) {
        case 0x01u:
          // Set the pointer to the address
          chFileStreamSeek((BaseFileStream*)at24c01, offsetof(DiWheelDriveFSIO::DiWheelDrive_1_1, vcnl4020offset[idx]));
          switch (this->bsmv) {
            default:
              bytesWritten = chSequentialStreamWrite((BaseFileStream*)at24c01, (const uint8_t*) buffer, bufferSize);
              break;
          }
          break;
        }

        if (bytesWritten != bufferSize) {
          return IO_ERROR;
        } else {
          return OK;
        }
    }
};

class PowerBoardFSIO : public FileSystemIoBase {
  private:

    /** \brief Layout for PowerBoardFSIO with BSMV 1 and bsmv 1*/
    struct PowerBoard_1_1 {
      uint8_t  reserved_0x00_0x14[20];
      uint16_t vcnl4020offset[8];        // U1.Proximity [11, 21, 31, 41, 12, 22, 32, 42]
      uint8_t  generalPurpose[88];       // 24x_7Cx
      uint8_t  reserved_0x7D_0x80[4];    // TODO CRC
    };

  public:
    PowerBoardFSIO(AT24 *at24c01, uint8_t BSMV, uint8_t bsmv, uint8_t HMV, uint8_t hmv)
      : FileSystemIoBase(at24c01, BSMV, bsmv, HMV, hmv) {

    }

    msg_t getVcnl4020Offset (uint16_t *buffer, uint8_t idx) {

      msg_t bytesWritten;
      const uint8_t bufferSize = sizeof(uint16_t);

      // Check if the vcnl index is in a valid range
      if (idx >= sizeof(PowerBoard_1_1::vcnl4020offset)) {
        return WRONG_INDEX;
      }

      // Get the data out of the memory
      switch (this->BSMV) {
        case 0x01u:
          // Set the pointer to the address
          chFileStreamSeek((BaseFileStream*)at24c01, offsetof(PowerBoardFSIO::PowerBoard_1_1, vcnl4020offset[idx]));
          switch (this->bsmv) {
            default:
              bytesWritten = chSequentialStreamRead((BaseFileStream*)at24c01, (uint8_t*) buffer, bufferSize);
              break;
          }
          break;
        }

        if (bytesWritten != bufferSize) {
          return IO_ERROR;
        } else {
          return OK;
        }
    }

    msg_t setVcnl4020Offset (uint16_t *buffer, uint8_t idx) {

      msg_t bytesWritten;
      const uint8_t bufferSize = sizeof(uint16_t);

      // Check if the vcnl index is in a valid range
      if (idx >= sizeof(PowerBoard_1_1::vcnl4020offset)) {
        return WRONG_INDEX;
      }

      // Get the data out of the memory
      switch (this->BSMV) {
        case 0x01u:
          // Set the pointer to the address
          chFileStreamSeek((BaseFileStream*)at24c01, offsetof(PowerBoardFSIO::PowerBoard_1_1, vcnl4020offset[idx]));
          switch (this->bsmv) {
            default:
              bytesWritten = chSequentialStreamWrite((BaseFileStream*)at24c01, (const uint8_t*) buffer, bufferSize);
              break;
          }
          break;
        }

        if (bytesWritten != bufferSize) {
          return IO_ERROR;
        } else {
          return OK;
        }
    }
};

class LightRingFSIO : public FileSystemIoBase {
  private:

    /** \brief Layout for LightRingFSIO with BSMV 1 and bsmv 1*/
    struct LightRing_1_1 {
      uint8_t  reserved_0x00_0x14[20];
      uint8_t  generalPurpose[104];      // 15x_80x
      uint8_t  reserved_0x7D_0x80[4];    // TODO CRC
    };

  public:
    LightRingFSIO(AT24 *at24c01, uint8_t BSMV, uint8_t bsmv, uint8_t HMV, uint8_t hmv)
      : FileSystemIoBase(at24c01, BSMV, bsmv, HMV, hmv) {

    }
};

}
}

#endif /* AMIRO_FILESYSTEMIO_H_ */
