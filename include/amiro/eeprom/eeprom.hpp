#ifndef EEPROM_HPP_
#define EEPROM_HPP_

#include <ch.hpp>
#include <hal.h>

#include <amiro/bus/i2c/I2CParams.hpp>

namespace amiro {

class I2CDriver;
class HWSPIDriver;

class EEPROM {

  public:
    /**
     * Base File Stream
     * eepromInit initializes this member.
     */
    BaseFileStream bfs;

    msg_t error;
    fileoffset_t position;

    size_t size;
    uint8_t page_size;

    /**
     * Maximum Write Cycle time
     * Expressed in multiples of 10 µs.
     */
    uint16_t max_t_wr;

    union {

#if HAL_USE_I2C
      struct {
        I2CDriver* i2c_driver;
        I2CTxParams i2c_txparams;
      };
#endif /* HAL_USE_I2C */

#if HAL_USE_SPI
      struct {
        HWSPIDriver* spi_driver;
      };
#endif /* HAL_USE_SPI */

    }; /* union */

#if CH_USE_MUTEXES
    /**
     * Mutex Lock
     */
    Mutex mutex;
#elif CH_USE_SEMAPHORES
    /**
     * Semaphore
     */
    Semaphore semaphore;
#endif

    enum {
      EEPROM_ERROR_NONE = 0x00u,
    };

  protected:

#if HAL_USE_I2C
    EEPROM(const struct BaseFileStreamVMT* vmt, size_t size, uint8_t page_size, uint16_t max_t_wr, uint8_t sla, I2CDriver* i2c_driver);
#endif /* HAL_USE_I2C */
#if HAL_USE_SPI
    EEPROM(const struct BaseFileStreamVMT* vmt, size_t size, uint8_t page_size, uint16_t max_t_wr, HWSPIDriver* spi_driver);
#endif /* HAL_USE_SPI */
    ~EEPROM();

  public:

    void acquire();
    void release();

    static int geterror(void* instance);
    static fileoffset_t getsize(void* instance);
    static fileoffset_t getposition(void* instance);
    static fileoffset_t lseek(void *instance, fileoffset_t offset);

};

} /* amiro */

#endif /* EEPROM_HPP_ */
