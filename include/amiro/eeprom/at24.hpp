#ifndef AT24_HPP_
#define AT24_HPP_

#include <ch.hpp>
#include <hal.h>

#include <amiro/eeprom/eeprom.hpp>

namespace amiro {

class I2CDriver;

class AT24 : public EEPROM {

  enum {
    AT24_SLA = 0x50u,
  };

  enum {
    AT24_MAX_PAGE_SIZE = 8,
  };

  public:

  /**
    * Return types of getCheck()
    */
   enum {
     CHECK_OK          = 0x00u,
     CHECK_READ_FAIL   = 0x01u,
     CHECK_WRITE_FAIL  = 0x02u,
     CHECK_CHKSUM_FAIL = 0x03u,
   };

    AT24(size_t size, uint8_t page_size, uint16_t max_t_wr, I2CDriver* i2c_driver);
    ~AT24();

    static msg_t poll_ack(void* instance);
    static size_t write(void* instance, const uint8_t* bp, size_t n);
    static size_t read(void* instance, uint8_t* bp, size_t n);
    static uint32_t close(void* instance);

};

} /* amiro */

#endif /* AT24_HPP_ */
