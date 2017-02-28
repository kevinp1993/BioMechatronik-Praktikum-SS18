#ifndef AMIRO_GLOBAL_HPP_
#define AMIRO_GLOBAL_HPP_

#include <hal.h>
#include <qei.h>

#include <stdlib.h>
#include <array>

#include <board.h>
#include <amiro/proximity/vcnl4020.hpp>
#include <amiro/bus/i2c/HWI2CDriver.hpp>
#include <amiro/bus/i2c/mux/pca9544.hpp>
#include <amiro/bus/i2c/VI2CDriver.hpp>
#include <amiro/magneto/hmc5883l.hpp>
#include <amiro/bus/spi/HWSPIDriver.hpp>
#include <amiro/accel/lis331dlh.hpp>
#include <amiro/gyro/l3g4200d.hpp>
#include <amiro/power/ina219.hpp>
#include <amiro/power/ltc4412.hpp>
#include <amiro/eeprom/at24.hpp>
#include <amiro/FileSystemInputOutput/FSIODiWheelDrive.hpp>
#include <amiro/serial_reset/serial_can_mux.hpp>
#include <amiro/MotorIncrements.h>
#include <amiro/MotorControl.h>
#include <amiro/DistControl.h>
#include <amiro/Odometry.h>
#include <DiWheelDrive.h>
#include <userthread.hpp>

using namespace amiro;

class Global final
{
public:
  SerialConfig sd1_config{
    /* speed        */ 115200,
    /* CR1 register */ 0,
    /* CR2 register */ 0,
    /* CR3 register */ 0
  };

  I2CConfig i2c1_config{
    /* I2C mode                 */ OPMODE_I2C,
    /* frequency                */ 400000,
    /* I2C fast mode duty cycle */ FAST_DUTY_CYCLE_2
  };
  I2CConfig i2c2_config{
    /* I2C mode                 */ OPMODE_I2C,
    /* frequency                */ 400000,
    /* I2C fast mode duty cycle */ FAST_DUTY_CYCLE_2
  };

  VCNL4020::VCNL4020Config vcnl4020_config{
    /* command             */ VCNL4020::ALS_EN | VCNL4020::PROX_EN | VCNL4020::SELFTIMED_EN,
    /* ambient parameter   */ VCNL4020::AMBIENT_RATE_2 | VCNL4020::AMBIENT_AUTO_OFFSET | VCNL4020::AMBIENT_AVG_32,
    /* IR LED current [mA] */ 200u,
    /* proximity rate      */ VCNL4020::PROX_RATE_125
  };

  /**
   * @brief I2C Bus 1
   * Conected devices:
   *   HMC5883L
   */
  HWI2CDriver HW_I2C1;
  /**
   * @brief I2C Bus 2
   * Connected devices:
   *   PCA9544
   *   VCNL4020
   *   INA219
   *   AT24Cxx
   */
  HWI2CDriver HW_I2C2;

  PCA9544<true> HW_PCA9544;

  std::array<VI2CDriver, 4> V_I2C2;

  std::array<VCNL4020, 4> vcnl4020;

  HMC5883L::HMC5883LConfig hmc5883l_config{
    /* ctrlA */ HMC5883L::DO_20_HZ | HMC5883L::MS_NORMAL | HMC5883L::MA_AVG8,
    /* ctrlB */ HMC5883L::GN_5_GA,
    /* mode  */ HMC5883L::MD_CONTCV | HMC5883L::HS_DISABLE
  };
  HMC5883L hmc5883l;

  SPIConfig accel_spi1_config{
    /* callback function pointer   */ NULL,
    /* chip select line port       */ GPIOC,
    /* chip select line pad number */ GPIOC_ACCEL_SS_N,
    /* initialzation data          */ SPI_CR1_BR_0
  };

  SPIConfig gyro_spi1_config{
    /* callback function pointer   */ NULL,
    /* chip select line port       */ GPIOC,
    /* chip select line pad number */ GPIOC_GYRO_SS_N,
    /* initialzation data          */ SPI_CR1_BR_0
  };

  HWSPIDriver HW_SPI1_ACCEL;
  HWSPIDriver HW_SPI1_GYRO;

  LIS331DLH::LIS331DLHConfig accel_run_config{
    /* ctrl1                     */ LIS331DLH::PM_ODR | LIS331DLH::DR_50HZ_37LP | LIS331DLH::ZEN | LIS331DLH::YEN | LIS331DLH::XEN,
    /* ctrl2                     */ 0x00u,
    /* ctrl3                     */ LIS331DLH::INT_LOW | LIS331DLH::I1_CFG_DRY,
    /* ctrl4                     */ LIS331DLH::BDU_CONT | LIS331DLH::BLE_LE | LIS331DLH::FS_4G | LIS331DLH::SIM_4WI,
    /* ctrl5                     */ LIS331DLH::SLEEP_TO_WAKE_OFF,
    /* interrupt 1 configuration */ {/* configuration */ LIS331DLH::AOI_OR_INT,
                                     /* ths           */ 0x00u,
                                     /* duration      */ 0x00u},
    /* interrupt 2 configuration */ {/* configuration */ LIS331DLH::AOI_OR_INT,
                                     /* ths           */ 0x00u,
                                     /* duration      */ 0x00u}
  };
  LIS331DLH::LIS331DLHConfig accel_sleep_config{
    /* ctrl1                     */ LIS331DLH::PM_0_5_HZ | LIS331DLH::DR_50HZ_37LP | LIS331DLH::XEN,
    /* ctrl2                     */ 0x00u,
    /* ctrl3                     */ LIS331DLH::INT_LOW | LIS331DLH::I1_CFG_I1,
    /* ctrl4                     */ LIS331DLH::BDU_CONT | LIS331DLH::BLE_LE | LIS331DLH::FS_4G | LIS331DLH::SIM_4WI,
    /* ctrl5                     */ LIS331DLH::SLEEP_TO_WAKE_ON,
    /* interrupt 1 configuration */ {/* configuration */ LIS331DLH::AOI_OR_INT | LIS331DLH::XHIE,
                                     /* ths           */ 0x10u,
                                     /* duration      */ 0x02u},
    /* interrupt 2 configuration */ {/* configuration */ LIS331DLH::AOI_OR_INT,
                                     /* ths           */ 0x00u,
                                     /* duration      */ 0x00u}
  };
  LIS331DLH lis331dlh;

  L3G4200D::L3G4200DConfig gyro_run_config{
    /* ctrl reg 1 */ L3G4200D::DR_100_HZ | L3G4200D::BW_25 | L3G4200D::PD | L3G4200D::ZEN | L3G4200D::YEN | L3G4200D::XEN,
    /* ctrl reg 2 */ 0,
    /* ctrl reg 3 */ L3G4200D::I2_DRDY,
    /* ctrl reg 4 */ L3G4200D::FS_500_DPS | L3G4200D::SIM_4W,
    /* ctrl reg 5 */ 0
  };
  L3G4200D::L3G4200DConfig gyro_sleep_config{
    /* ctrl reg 1 */ L3G4200D::DR_100_HZ | L3G4200D::BW_25 | L3G4200D::PD,
    /* ctrl reg 2 */ gyro_run_config.ctrl2,
    /* ctrl reg 3 */ gyro_run_config.ctrl3,
    /* ctrl reg 4 */ gyro_run_config.ctrl4,
    /* ctrl reg 5 */ gyro_run_config.ctrl5
  };
  L3G4200D l3g4200d;

  INA219::Driver ina219;

  LTC4412wEN<(uintptr_t)GPIOC, GPIOC_PATH_DCSTAT, (uintptr_t)GPIOC, GPIOC_PATH_DCEN> ltc4412;

  AT24 at24c01;
  fileSystemIo::FSIODiWheelDrive memory;

  MotorIncrements increments;
  MotorControl motorcontrol;
  DistControl distcontrol;
  Odometry odometry;
  SerialCanMux sercanmux1;

  DiWheelDrive robot;

  UserThread userThread;

public:
  Global() :
    HW_I2C1(&I2CD1), HW_I2C2(&I2CD2),
    HW_PCA9544(&HW_I2C2, 0x07u),
    V_I2C2{{VI2CDriver(&HW_PCA9544, 0),
            VI2CDriver(&HW_PCA9544, 1),
            VI2CDriver(&HW_PCA9544, 2),
            VI2CDriver(&HW_PCA9544, 3)}
          },
    vcnl4020{{/* front left  */ VCNL4020(&V_I2C2[0], &vcnl4020_config),
              /* left wheel  */ VCNL4020(&V_I2C2[1], &vcnl4020_config),
              /* right wheel */ VCNL4020(&V_I2C2[2], &vcnl4020_config),
              /* front right */ VCNL4020(&V_I2C2[3], &vcnl4020_config)}
            },
    hmc5883l(&HW_I2C1, &hmc5883l_config),
    HW_SPI1_ACCEL(&SPID1, &accel_spi1_config), HW_SPI1_GYRO(&SPID1, &gyro_spi1_config),
    lis331dlh(&HW_SPI1_ACCEL),
    l3g4200d(&HW_SPI1_GYRO),
    ina219(HW_I2C2, 0x40u),
    ltc4412(),
    at24c01(0x400u / 0x08u, 0x08u, 500u, &HW_I2C2),
    memory(at24c01, /*BMSV*/ 1, /*bmsv*/ 3, /*HMV*/ 1, /*hmv*/ 0), // bmsv changed von 2 auf 3
    increments(&QEID3, &QEID4),
    motorcontrol(&PWMD2, &increments, GPIOB, GPIOB_POWER_EN, &memory),
    distcontrol(&motorcontrol, &increments),
    odometry(&increments, &l3g4200d),
    sercanmux1(&SD1, &CAND1, CAN::DI_WHEEL_DRIVE_ID),
    robot(&CAND1),
    userThread()
  {
    return;
  }

  ~Global()
  {
    return;
  }
};

#endif /* AMIRO_GLOBAL_HPP_ */
