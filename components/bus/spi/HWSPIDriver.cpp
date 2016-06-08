#include <hal.h>
#include <amiro/bus/spi/HWSPIDriver.hpp>

using namespace chibios_rt;

namespace amiro {

HWSPIDriver::
HWSPIDriver(::SPIDriver *driver, const ::SPIConfig *config) :
  driver(driver),
  config(config) {

}

HWSPIDriver::
~HWSPIDriver() {


}

void
HWSPIDriver::
exchange(uint8_t *readData, const uint8_t *writeData, size_t length) {
  spiAcquireBus(this->driver);
  spiStart(this->driver, this->config);
  spiSelect(this->driver);
  spiExchange(this->driver, length, writeData, readData);
  spiUnselect(this->driver);
  spiReleaseBus(this->driver);
}

void
HWSPIDriver::
read(uint8_t *data, size_t length) {
  spiAcquireBus(this->driver);
  spiStart(this->driver, this->config);
  spiSelect(this->driver);
  spiReceive(this->driver, length, data);
  spiUnselect(this->driver);
  spiReleaseBus(this->driver);
}

void
HWSPIDriver::
write(const uint8_t *data, size_t length) {
  spiAcquireBus(this->driver);
  spiStart(this->driver, this->config);
  spiSelect(this->driver);
  spiSend(this->driver, length, data);
  spiUnselect(this->driver);
  spiReleaseBus(this->driver);
}

} /* amiro */
