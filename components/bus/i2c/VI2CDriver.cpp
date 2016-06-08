#include <chdebug.h>
#include <amiro/bus/i2c/I2CMultiplexer.hpp>
#include <amiro/bus/i2c/VI2CDriver.hpp>

namespace amiro {

VI2CDriver::
VI2CDriver(I2CMultiplexer *driver, const uint8_t bus_id) :
  driver(driver),
  id(bus_id) {

#if CH_USE_MUTEXES
  chMtxInit(&this->mutex);
#else
  chSemInit(&this->semaphore, 1);
#endif /* CH_USE_MUTEXES */

};


void
VI2CDriver::
start(const I2CConfig* config) {

  /* silently discard config */
  (void) config;
  /* ignore starts */

}

void
VI2CDriver::
stop() {

  /* ignore stops */

}

i2cflags_t
VI2CDriver::
getErrors() {

  return this->driver->getErrors();

}

msg_t
VI2CDriver::
masterTransmit(const I2CTxParams *params, systime_t timeout) {

  msg_t ret;

  this->driver->acquireBus();

  if (!(ret = this->driver->select(this->id))) {
    ret = this->driver->masterTransmit(params, timeout);
  }

  this->driver->deselect();
  this->driver->releaseBus();

  return ret;
}

msg_t
VI2CDriver::
masterReceive(const I2CRxParams *params, systime_t timeout) {

  msg_t ret;

  this->driver->acquireBus();

  if (!(ret = this->driver->select(this->id)))
    ret = this->driver->masterReceive(params, timeout);

  this->driver->deselect();
  this->driver->releaseBus();

  return ret;

}


void
VI2CDriver::
acquireBus() {

#if CH_USE_MUTEXES
  chMtxLock(&this->mutex);
#elif CH_USE_SEMAPHORES
  chSemWait(&this->semaphore);
#endif

}

void
VI2CDriver::
releaseBus() {

#if CH_USE_MUTEXES
  chMtxUnlock();
#elif CH_USE_SEMAPHORES
  chSemSignal(&this->semaphore);
#endif

}

}
/* amiro */
