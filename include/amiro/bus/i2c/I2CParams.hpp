/*
 * I2CParams.hpp
 *
 *  Created on: Jun 10, 2014
 *      Author: Robert ABEL
 */

#ifndef I2CPARAMS_HPP_
#define I2CPARAMS_HPP_

#include <ch.hpp>
#include <hal.h>

namespace amiro {

struct I2CTxParams {

  i2caddr_t      addr;
  const uint8_t *txbuf;
  size_t         txbytes;
  uint8_t       *rxbuf;
  size_t         rxbytes;

};

struct I2CRxParams {

  i2caddr_t      addr;
  uint8_t       *rxbuf;
  size_t         rxbytes;

};

} /* amiro */

#endif /* I2CPARAMS_HPP_ */
