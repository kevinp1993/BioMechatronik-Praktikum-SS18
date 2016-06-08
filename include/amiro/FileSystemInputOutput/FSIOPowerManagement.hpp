#ifndef AMIRO_FSIOPOWERMANAGEMENT_H_
#define AMIRO_FSIOPOWERMANAGEMENT_H_

/*! \brief memory layout for the eeproms
 *
 * This header defines the memory layout of the
 * at24c01 on the PowerManagement board.
 */

#include <amiro/eeprom/at24.hpp>
#include <amiro/FileSystemInputOutput/FileSystemInputOutputBase.hpp>

using namespace chibios_rt;

namespace amiro {
namespace fileSystemIo {

class FSIOPowerManagement : public FileSystemIoBase {
  private:

    /** \brief Layout for FSIOPowerManagement with BSMV 1 and bsmv 1*/
    struct PowerManagement_1_1 {
      uint8_t  reserved_0x00_0x14[20];
      uint16_t vcnl4020offset[8];        // U1.Proximity [11, 21, 31, 41, 12, 22, 32, 42]
      uint8_t  generalPurpose[88];       // 24x_7Cx
      uint8_t  reserved_0x7D_0x80[4];
    };

  public:
    FSIOPowerManagement(AT24 &at24c01, uint8_t BSMV, uint8_t bsmv, uint8_t HMV, uint8_t hmv)
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
};

}
}

#endif /* AMIRO_FSIOPOWERMANAGEMENT_H_ */
