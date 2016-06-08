#include <ch.hpp>
#include <hal.h>

#include <amiro/eeprom/eeprom.hpp>

namespace amiro {

#if HAL_USE_I2C
EEPROM::
EEPROM(const struct BaseFileStreamVMT* vmt, size_t size, uint8_t page_size, uint16_t max_t_wr, uint8_t sla, I2CDriver* i2c_driver) :
  error(EEPROM_ERROR_NONE),
  position(0x00u),
  size(size),
  page_size(page_size),
  max_t_wr(max_t_wr),
  i2c_driver(i2c_driver) {

#if CH_USE_MUTEXES
  chMtxInit(&this->mutex);
#elif CH_USE_SEMAPHORES
  chSemInit(&this->semaphore, 1);
#endif

  this->bfs.vmt = vmt;
  this->i2c_txparams.addr = sla;

}
#endif /* HAL_USE_I2C */

#if HAL_USE_SPI
EEPROM::
EEPROM(const struct BaseFileStreamVMT* vmt, size_t size, uint8_t page_size, uint16_t max_t_wr, HWSPIDriver* spi_driver) :
  error(EEPROM_ERROR_NONE),
  position(0x00u),
  size(size),
  page_size(page_size),
  max_t_wr(max_t_wr),
  spi_driver(spi_driver) {

#if CH_USE_MUTEXES
  chMtxInit(&this->mutex);
#elif CH_USE_SEMAPHORES
  chSemInit(&this->semaphore, 1);
#endif

  this->bfs.vmt = vmt;

}
#endif /* HAL_USE_SPI */

EEPROM::
~EEPROM() {

}

void
EEPROM::
acquire() {

#if CH_USE_MUTEXES
  chMtxLock(&this->mutex);
#elif CH_USE_SEMAPHORES
  chSemWait(&this->semaphore);
#endif

}

void
EEPROM::
release() {

#if CH_USE_MUTEXES
  chMtxUnlock();
#elif CH_USE_SEMAPHORES
  chSemSignal(&this->semaphore);
#endif

}

int
EEPROM::
geterror(void* instance) {

  return (int) ((EEPROM*) (instance))->error;

}

fileoffset_t
EEPROM::
getsize(void *instance) {

  return ((EEPROM*) (instance))->size;

}

fileoffset_t
EEPROM::
getposition(void* instance) {

  return ((EEPROM*) (instance))->position;


}

fileoffset_t
EEPROM::
lseek(void *instance, fileoffset_t offset) {

  ((EEPROM*) (instance))->position = offset;
  return FILE_OK;

}

};
