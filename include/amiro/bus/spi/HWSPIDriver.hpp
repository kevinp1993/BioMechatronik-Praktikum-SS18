#ifndef HWSPIDRIVER_HPP_
#define HWSPIDRIVER_HPP_

#include <ch.hpp>
#include <hal.h>

/**
 * @brief   AMiRo
 */
namespace amiro {

  /**
  * @brief   Input/Output driver class
  */
  class HWSPIDriver {
  public:
    HWSPIDriver(::SPIDriver *driver, const ::SPIConfig *config);
    virtual ~HWSPIDriver();
    void exchange(uint8_t *readData, const uint8_t *writeData, size_t length);
    void read(uint8_t *data, size_t length);
    void write(const uint8_t *data, size_t length);

  private:
    ::SPIDriver *driver;
    const ::SPIConfig *config;
  };

} /* amiro */

#endif /* HWSPIDRIVER_HPP_ */
