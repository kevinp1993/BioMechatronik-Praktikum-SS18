/*
 * HWI2CDriver.hpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Robert ABEL
 */

#ifndef HWI2CDRIVER_HPP_
#define HWI2CDRIVER_HPP_

#include <ch.hpp>
#include <hal.h>
#include <amiro/bus/i2c/I2CDriver.hpp>

/**
 * @brief   AMiRo
 */
namespace amiro {

  struct I2CTxParams;
  struct I2CRxParams;

  /**
   * @brief   I²C HW Driver Abstraction
   */
  class HWI2CDriver : public I2CDriver {

  public:

    HWI2CDriver(::I2CDriver *d);
    virtual ~HWI2CDriver() {};

    virtual void start(const ::I2CConfig *config);
    virtual void stop();
    virtual i2cflags_t getErrors();
    virtual msg_t masterTransmit(const I2CTxParams *params, systime_t timeout = TIME_INFINITE);
    virtual msg_t masterReceive(const I2CRxParams *params, systime_t timeout = TIME_INFINITE);
    virtual void acquireBus();
    virtual void releaseBus();

  private:

    ::I2CDriver *driver;

  };

} /* amiro */

#endif /* HWI2CDRIVER_HPP_ */
