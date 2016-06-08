#ifndef MUXEDI2CDRIVER_HPP_
#define MUXEDI2CDRIVER_HPP_

#include <ch.hpp>
#include <hal.h>
#include <amiro/bus/i2c/I2CDriver.hpp>

/**
 * @brief   AMiRo
 */
namespace amiro {

  struct I2CTxParams;
  struct I2CRxParams;

  class I2CMultiplexer;

  /**
   * @brief   Virtual I²C Driver
   */
  class VI2CDriver : public I2CDriver {

  public:

    VI2CDriver(I2CMultiplexer *driver, const uint8_t bus_id);
    virtual ~VI2CDriver() {}
    virtual void start(const ::I2CConfig *config);
    virtual void stop();
    virtual i2cflags_t getErrors();
    virtual msg_t masterTransmit(const I2CTxParams *params, systime_t timeout = TIME_INFINITE);
    virtual msg_t masterReceive(const I2CRxParams *params, systime_t timeout = TIME_INFINITE);

    virtual void acquireBus();
    virtual void releaseBus();

  private:

    I2CMultiplexer *driver;
    uint8_t id;

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

#endif /* MUXEDI2CDRIVER_HPP_ */
