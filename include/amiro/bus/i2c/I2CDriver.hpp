/*
 * AI2CDriver.hpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Robert ABEL
 */

#ifndef AI2CDRIVER_HPP_
#define AI2CDRIVER_HPP_

#include <ch.hpp>
#include <hal.h>

/**
 * @brief   AMiRo
 */
namespace amiro {

  struct I2CTxParams;
  struct I2CRxParams;

  /**
   * @brief   I²C Driver Interface
   */
  class I2CDriver {

  public:

    I2CDriver() {};
    virtual ~I2CDriver() {};

    virtual void start(const ::I2CConfig *config) = 0;
    virtual void stop() = 0;
    virtual i2cflags_t getErrors() = 0;
    virtual msg_t masterTransmit(const I2CTxParams *params, systime_t timeout = TIME_INFINITE) = 0;
    virtual msg_t masterReceive(const I2CRxParams *params, systime_t timeout = TIME_INFINITE) = 0;

    virtual void acquireBus() = 0;
    virtual void releaseBus() = 0;

  };

} /* amiro */

#endif /* AI2CDRIVER_HPP_ */
