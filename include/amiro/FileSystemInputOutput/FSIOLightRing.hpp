#ifndef AMIRO_FSIOLIGHTRING_H_
#define AMIRO_FSIOLIGHTRING_H_

/*! \brief memory layout for the eeproms
 *
 * This header defines the memory layout of the
 * at24c01 on the LightRing board.
 */

#include <amiro/eeprom/at24.hpp>
#include <amiro/FileSystemInputOutput/FileSystemInputOutputBase.hpp>

namespace amiro {
namespace fileSystemIo {

class FSIOLightRing : public FileSystemIoBase {
  private:

    /** \brief Layout for FSIOLightRing with BSMV 1 and bsmv 1*/
    struct LightRing_1_1 {
      uint8_t  reserved_0x00_0x14[20];
      uint8_t  generalPurpose[104];      // 15x_7Cx
      uint8_t  reserved_0x7D_0x80[4];
    };

  public:
    FSIOLightRing(AT24 &at24c01, uint8_t BSMV, uint8_t bsmv, uint8_t HMV, uint8_t hmv)
      : FileSystemIoBase(at24c01, BSMV, bsmv, HMV, hmv) {

    }
};

}
}

#endif /* AMIRO_FSIOLIGHTRING_H_ */
