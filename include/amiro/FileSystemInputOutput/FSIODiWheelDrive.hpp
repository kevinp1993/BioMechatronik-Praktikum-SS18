#ifndef AMIRO_FSIODIWHEELDRIVE_H_
#define AMIRO_FSIODIWHEELDRIVE_H_

/*! \brief memory layout for the eeproms
 *
 * This header defines the memory layout of the
 * at24c01 on the CiWheelDrive board.
 */

#include <amiro/eeprom/at24.hpp>
#include <amiro/FileSystemInputOutput/FileSystemInputOutputBase.hpp>

// using namespace chibios_rt;

namespace amiro {
namespace fileSystemIo {

class FSIODiWheelDrive : public FileSystemIoBase {
  private:

    /** \brief Layout for FSIODiWheelDrive with BSMV 1 and bsmv 1*/
    struct DiWheelDrive_1_1 {
      uint8_t  reserved_0x00_0x14[20];
      uint16_t vcnl4020offset[4];       // U [12, 13, 15, 16]
      uint8_t  generalPurpose[96];      // 1Cx_7Cx
      uint8_t  reserved_0x7D_0x80[4];
    };

    /** \brief Layout for FSIODiWheelDrive with BSMV 1 and bsmv 2*/
    struct DiWheelDrive_1_2 {
      uint8_t  reserved_0x00_0x14[28];
      float    Ed;
      float    Eb;
      uint8_t  generalPurpose[88];      // 24x_7Cx
      uint8_t  reserved_0x7D_0x80[4];
    };

    /** \brief Layout for FSIODiWheelDrive with BSMV 1 and bsmv 3*/
    struct DiWheelDrive_1_3 {
      uint8_t  reserved_0x00_0x14[36];
      float wheelfactor;
      float igain;
      int pgain;
      float dgain;
      uint8_t  generalPurpose[72];      // 24x_7Cx
      uint8_t  reserved_0x7D_0x80[4];
    };

  public:
    FSIODiWheelDrive(AT24 &at24c01, uint8_t BSMV, uint8_t bsmv, uint8_t HMV, uint8_t hmv)
      : FileSystemIoBase(at24c01, BSMV, bsmv, HMV, hmv) {}

    /**
     * \brief Read the VCNL4020 offset from memory
     * @param buffer Content to write in from the memory
     * @param idx Index of the VCNL4020 sensor
     * @return FSIO return types
     */
    msg_t getVcnl4020Offset (uint16_t *buffer, uint8_t idx);

    /**
     * \brief Write new data to the VCNL4020 offset
     * @param buffer Content to write to the memory
     * @param idx Index of the VCNL4020 sensor
     * @return FSIO return types
     */
    msg_t setVcnl4020Offset (uint16_t buffer, uint8_t idx);

    /**
     * \brief Read the Ed ratio from memory
     * @return FSIO return types
     */
    msg_t getEd (float *buffer);

    /**
     * \brief Write new data to the Ed ratio value
     * @param buffer Content to write to the memory
     * @return FSIO return types
     */
    msg_t setEd (float buffer);

    /**
     * \brief Read the Eb ratio from memory
     * @return FSIO return types
     */
    msg_t getEb (float *buffer);

    /**
     * \brief Write new data to the Eb ratio value
     * @param buffer Content to write to the memory
     * @return FSIO return types
     */
    msg_t setEb (float buffer);

    msg_t setwheelfactor (float buffer);
    msg_t getWheelFactor (float *buffer);

    msg_t setiGain (float buffer);
    msg_t getiGain (float *buffer);

    msg_t setdGain (float buffer);
    msg_t getdGain (float *buffer);

    msg_t setpGain (int buffer);
    msg_t getpGain (int *buffer);
};

}
}

#endif /* AMIRO_FSIODIWHEELDRIVE_H_ */
