#include <amiro/bus/i2c/I2CMultiplexer.hpp>

namespace amiro {

I2CMultiplexer::
I2CMultiplexer(I2CDriver *driver) :
  driver(driver) {

#if CH_USE_MUTEXES
  chMtxInit(&this->mutex);
#else
  chSemInit(&this->semaphore, 1);
#endif /* CH_USE_MUTEXES */

};

I2CMultiplexer::
~I2CMultiplexer() {

}

i2cflags_t
I2CMultiplexer::
getErrors() {

  return this->driver->getErrors();

}

msg_t
I2CMultiplexer::
masterTransmit(const I2CTxParams *params, systime_t timeout) {

  return this->driver->masterTransmit(params, timeout);

}

msg_t
I2CMultiplexer::
masterReceive(const I2CRxParams *params, systime_t timeout) {

  return this->driver->masterReceive(params, timeout);

}

void
I2CMultiplexer::
acquireBus() {

#if CH_USE_MUTEXES
  chMtxLock(&this->mutex);
#elif CH_USE_SEMAPHORES
  chSemWait(&this->semaphore);
#endif

  this->driver->acquireBus();
}

void
I2CMultiplexer::
releaseBus() {

  this->driver->releaseBus();

#if CH_USE_MUTEXES
  chMtxUnlock();
#elif CH_USE_SEMAPHORES
  chSemSignal(&this->semaphore);
#endif

}

} /* amiro */
