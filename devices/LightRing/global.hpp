#ifndef AMIRO_GLOBAL_HPP_
#define AMIRO_GLOBAL_HPP_

#include <hal.h>

#include <board.h>
#include <amiro/bus/i2c/HWI2CDriver.hpp>
#include <amiro/bus/spi/HWSPIDriver.hpp>
#include <amiro/leds/tlc5947.hpp>
#include <amiro/eeprom/at24.hpp>
#include <amiro/FileSystemInputOutput/FSIOLightRing.hpp>
#include <LightRing.h>
#include <amiro/Lidar.h>
#include <amiro/radio/a2500r24a.hpp>
#include <amiro/serial_reset/serial_can_mux.hpp>
#include <userthread.hpp>

namespace amiro {

class Global final
{
public:
  I2CConfig i2c2_config{
    /* I2C mode                 */ OPMODE_I2C,
    /* frequency                */ 400000,
    /* I2C fast mode duty cycle */ FAST_DUTY_CYCLE_2
  };

  SerialConfig sd1_config{
    /* speed        */ 115200,
    /* CR1 register */ 0,
    /* CR2 register */ 0,
    /* CR3 register */ 0
  };
  SerialConfig sd2_config{
    /* speed        */ 19200,
    /* CR1 register */ 0,
    /* CR2 register */ 0,
    /* CR3 register */ 0
  };

  SPIConfig spi1_config{
    /* callback function pointer   */ NULL,
    /* chip select line port       */ GPIOC,
    /* chip select line pad number */ GPIOC_LIGHT_XLAT,
    /* initialzation data          */ SPI_CR1_BR_0 | SPI_CR1_BR_1
  };
  SPIConfig spi2_config{
    /* callback function pointer   */ NULL,
    /* chip select line port       */ GPIOB,
    /* chip select line pad number */ GPIOB_WL_SS_N,
    /* initialzation data          */ SPI_CR1_BR_0
  };

  /**
   * @brief I2C Bus 2
   * Conected devices:
   *   AT24Cxx
   */
  HWI2CDriver HW_I2C2;

  HWSPIDriver HW_SPI1;
  HWSPIDriver HW_SPI2;

  TLC5947 tlc5947;

  AT24 at24c01;
  fileSystemIo::FSIOLightRing memory;

  LightRing robot;

  SerialCanMux sercanmux1;

  Lidar lidar;

  A2500R24A a2500r24a;

  UserThread userThread;

  uint8_t shellTermID;

public:
  Global() :
    HW_I2C2(&I2CD2),
    HW_SPI1(&SPID1, &spi1_config), HW_SPI2(&SPID2, &spi2_config),
    tlc5947(&HW_SPI1, GPIOA, GPIOA_LIGHT_BLANK),
    at24c01(0x400u / 0x08u, 0x08u, 500u, &HW_I2C2),
    memory(at24c01, /*BMSV*/ 1, /*bmsv*/ 2, /*HMV*/ 1, /*hmv*/ 0),
    robot(&CAND1, &tlc5947, &memory),
    sercanmux1(&SD1, &CAND1, CAN::LIGHT_RING_ID),
    lidar(CAN::LIGHT_RING_ID, Lidar::SETUP::POWER_ONLY),
    a2500r24a(&HW_SPI2),
    userThread()
  {
    return;
  }

  ~Global()
  {
    return;
  }
};

} // end of namespace amiro

#endif /* AMIRO_GLOBAL_HPP_ */
