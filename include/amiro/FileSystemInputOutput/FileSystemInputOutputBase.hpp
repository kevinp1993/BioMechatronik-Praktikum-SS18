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

namespace amiro {
namespace fileSystemIo {

class FileSystemIoBase {

  public:
    AT24 &at24c01;
    const uint8_t BSMV;
    const uint8_t bsmv;
    const uint8_t HMV;
    const uint8_t hmv;
    static constexpr uint8_t magicByteValue() { return 0xAAu;}

    // TODO implement CRC check at 0x7D to 0x80
    /** \brief Global memory structure */
    struct preamble_1_1 {
      uint8_t magicByte;
      uint8_t byteStructureMajorVersion;
      uint8_t byteStructureMinorVersion;
      uint8_t hardwareMajorVersion;
      uint8_t hardwareMinorVersion;
      uint8_t reserved_0x05_0x14[15];
    };

    struct preamble_1_2 {
      uint8_t reserved_0x00_0x04[5];
      uint8_t boardId;
      uint8_t reserved_0x06_0x14[14];
    };

    /** \brief FSIO return types */
    enum{
      OK                    = 0x00u,
      NOT_IMPLEMENTED       = 0x01u,
      WRONG_BSMV            = 0x02u,
      WRONG_bsmv            = 0x04u,
      IO_ERROR              = 0x08u,
      WRONG_MAGICBYTE_VALUE = 0x10u,
      WRONG_INDEX           = 0x20u,
      WRONG_HMV             = 0x40u,
      WRONG_hmv             = 0x80u,
    };

  public:

    FileSystemIoBase(AT24 &at24c01, uint8_t BSMV, uint8_t bsmv, uint8_t HMV, uint8_t hmv)
      : at24c01(at24c01),
        BSMV(BSMV),
        bsmv(bsmv),
        HMV(HMV),
        hmv(hmv) {
     }

    /**
     * \brief Checks the values stored in the memory and compares the set version numbers and the version numbers found.
     *        If the magic byte is incorrect WRONG_MAGICBYTE_VALUE is returned.
     *        If the BMSVs are not equal WRONG_BSMV is returned.
     *        If the bmsv found in the memory is smaller, the missing elements are initialized with default values and WRONG_bsmv is returned.
     *        If the bmsv found in the memory is equal or greater OK is returned.
     *        If the HMVs are not equal WRONG_HMV is returned.
     *        If the mmvs are not equal WRONG_hmv is returned.
     * @return FSIO return types
     */
    msg_t init();

    /**
     * \brief Check the memory structure
     * @return FSIO return types
     */
    msg_t getCheck();

    /**
     * @brief Initializes the memory of the given preamble with default values.
     * @param[in] BSMV The Byte Structure Major Version to initialize.
     * @param[in] bsvm The Byte Structure Minor Version to initialize.
     * @param[in] strict Specifies whether all (true) or only new (false) values are written to the memory.
     * @return FSIO return types
     */
    msg_t initPreamble(const uint8_t BSMV, const uint8_t bsmv, const bool strict = true);

    /**
     * \brief Initializes the memory with the given structure
     * @return FSIO return types
     */
    msg_t resetMemory();

    /**
     * \brief Writes the board id to the eeprom
     * @return FSIO return types
     */
    msg_t setBoardId (uint8_t buffer);

    /**
     * \brief Reads the board id from the eeprom
     * @return FSIO return types
     */
    msg_t getBoardId (uint8_t *buffer);

};

}
}

#endif /* AMIRO_FILESYSTEMIO_H_ */
