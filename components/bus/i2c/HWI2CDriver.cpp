#include <amiro/bus/i2c/I2CParams.hpp>
#include <amiro/bus/i2c/HWI2CDriver.hpp>

namespace amiro {


HWI2CDriver::
HWI2CDriver(::I2CDriver *d) :
  driver(d) {

};

void
HWI2CDriver::
start(const ::I2CConfig *config) {

  i2cStart(this->driver, config);

};

void
HWI2CDriver::
stop() {

  i2cStop(this->driver);

}

i2cflags_t
HWI2CDriver::
getErrors() {

  return i2cGetErrors(this->driver);

}

msg_t
HWI2CDriver::
masterTransmit(const I2CTxParams *params, systime_t timeout) {

  return i2cMasterTransmitTimeout(
                                  this->driver,
                                  params->addr,
                                  params->txbuf,
                                  params->txbytes,
                                  params->rxbuf,
                                  params->rxbytes,
                                  timeout);

}

msg_t
HWI2CDriver::
masterReceive(const I2CRxParams *params, systime_t timeout) {

  return i2cMasterReceiveTimeout(
                                 this->driver,
                                 params->addr,
                                 params->rxbuf,
                                 params->rxbytes,
                                 timeout);

}

void
HWI2CDriver::
acquireBus() {

#if I2C_USE_MUTUAL_EXCLUSION
  i2cAcquireBus(this->driver);
#endif /* I2C_USE_MUTUAL_EXCLUSION */
}

void
HWI2CDriver::
releaseBus() {

#if I2C_USE_MUTUAL_EXCLUSION
  i2cReleaseBus(this->driver);
#endif /* I2C_USE_MUTUAL_EXCLUSION */

}

} /* amiro */
