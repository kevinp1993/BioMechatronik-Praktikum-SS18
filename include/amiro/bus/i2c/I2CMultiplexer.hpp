#ifndef AI2CMULTIPLEXER
#define AI2CMULTIPLEXER

#include <ch.hpp>
#include <hal.h>
#include <amiro/bus/i2c/I2CDriver.hpp>

/**
 * @brief   AMiRo
 */
namespace amiro {

  /**
   * @brief   I²C Bus Multiplexer
   */
  class I2CMultiplexer {

  public:

    I2CMultiplexer(I2CDriver *driver);
    virtual ~I2CMultiplexer() = 0;

    virtual i2cflags_t getErrors();
    virtual msg_t masterTransmit(const I2CTxParams *params, systime_t timeout = TIME_INFINITE);
    virtual msg_t masterReceive(const I2CRxParams *params, systime_t timeout = TIME_INFINITE);

    virtual void acquireBus();
    virtual void releaseBus();

    virtual msg_t select(const uint8_t bus_id) = 0;
    virtual msg_t deselect() = 0;

  private:

    I2CDriver *driver;

#if I2C_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
#if CH_USE_MUTEXES || defined(__DOXYGEN__)
    /**
    * @brief   Mutex protecting the bus.
    */
    ::Mutex                     mutex;
#elif CH_USE_SEMAPHORES
    ::Semaphore                 semaphore;
#endif
#endif /* I2C_USE_MUTUAL_EXCLUSION */

  };

} /* amiro */

#endif /* AI2CMULTIPLEXER */
