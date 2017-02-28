#ifndef AMIRO_GLOBAL_HPP_
#define AMIRO_GLOBAL_HPP_

#include <hal.h>

#include <array>

#include <amiro/power/adconverter.hpp>
#include <amiro/bus/i2c/HWI2CDriver.hpp>
#include <amiro/bus/i2c/VI2CDriver.hpp>
#include <amiro/bus/i2c/mux/pca9544.hpp>
#include <amiro/proximity/vcnl4020.hpp>
#include <amiro/eeprom/at24.hpp>
#include <amiro/FileSystemInputOutput/FSIOPowerManagement.hpp>
#include <amiro/input/mpr121.hpp>
#include <amiro/power/ltc4412.hpp>
#include <amiro/power/ina219.hpp>
#include <amiro/power/bq27500.hpp>
#include <amiro/power/bq24103a.hpp>
#include <amiro/power/tps62113.hpp>
#include <amiro/bluetooth/bluetooth.hpp>
#include <amiro/bluetooth/bluetooth-serial.hpp>
#include <amiro/bluetooth/bluetooth-wiimote.hpp>
#include <amiro/serial_reset/iwrap_can_mux.hpp>
#include <amiro/serial_reset/serial_can_mux.hpp>
#include <PowerManagement.h>
#include <userthread.h>

namespace amiro {

class Global final
{
public:

  SerialConfig sd1_config{
    /* speed        */ 115200,
    /* CR1 register */ 0,
    /* CR2 register */ 0,
    /* CR3 register */ 0
  };

  PWMConfig pwm3_config{
    /* frequency [Hz]    */ 10000,
    /* period [ticks]    */ 100,
    /* callback          */ NULL,
    /* channels          */ {
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_ACTIVE_HIGH, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL}
    },
    /* CR2 register      */ 0,
    /* break & dead time */ 0
  };

  I2CConfig i2c1_config{
    /* I2C mode                 */ OPMODE_I2C,
    /* frequency                */ 100000,
    /* I2C fast mode duty cycle */ STD_DUTY_CYCLE
  };
  I2CConfig i2c2_config{
    /* I2C mode                 */ OPMODE_I2C,
    /* frequency                */ 100000,
    /* I2C fast mode duty cycle */ STD_DUTY_CYCLE
  };

  VCNL4020::VCNL4020Config vcnl4020_config{
    /* command             */ VCNL4020::ALS_EN | VCNL4020::PROX_EN | VCNL4020::SELFTIMED_EN,
    /* ambient parameter   */ VCNL4020::AMBIENT_RATE_2 | VCNL4020::AMBIENT_AUTO_OFFSET | VCNL4020::AMBIENT_AVG_32,
    /* IR LED current [mA] */ 200u,
    /* proximity rate      */ VCNL4020::PROX_RATE_125
  };

  adcsample_t adc1_buffer[1];
  ADCConversionGroup adc1_conversion_group{
    /* buffer type        */ TRUE, // TRUE=continuous; FALSE=linear
    /* number of channels */ 1,
    /* callback function  */ NULL,
    /* error callback     */ NULL,
    /* CR1                */ 0,
    /* CR2                */ ADC_CR2_SWSTART | ADC_CR2_CONT,
    /* SMPR1              */ 0,
    /* SMPR2              */ ADC_SMPR2_SMP_AN9(ADC_SAMPLE_480),
    /* HTR                */ ADC_HTR_HT,
    /* LTR                */ 0,
    /* SQR1               */ ADC_SQR1_NUM_CH(1),
    /* SQR2               */ 0,
    /* SQR3               */ ADC_SQR3_SQ1_N(ADC_CHANNEL_IN9)
  };
  ADConverter adc1_vsys;

  HWI2CDriver HW_I2C1;
  HWI2CDriver HW_I2C2;

  AT24 at24c01;
  fileSystemIo::FSIOPowerManagement memory;

  LTC4412<(uintptr_t)GPIOC, GPIOC_PATH_DC> ltc4412;

  std::array<INA219::Driver,5> ina219;

  std::array<BQ27500::Driver,2> bq27500;

private:
  BQ24103A<(uintptr_t)GPIOC, GPIOC_CHARGE_EN1_N, (uintptr_t)GPIOC, GPIOC_CHARGE_STAT1A> bq24103a_p7;
  BQ24103A<(uintptr_t)GPIOD, GPIOD_CHARGE_EN2_N, (uintptr_t)GPIOB, GPIOB_CHARGE_STAT2A> bq24103a_p8;
public:
  std::array<BaseBQ24103A*,2> bq24103a;

  TPS62113<(uintptr_t)GPIOA, GPIOA_SYS_REG_EN> tps62113_vio18_33_50;
  TPS62113<(uintptr_t)GPIOB, GPIOB_POWER_EN> tps62113_vsys42;

  BLUETOOTH wt12;

  std::array<PCA9544<true>, 2> HW_PCA9544;

  std::array<VI2CDriver, 4> V_I2C1;
  std::array<VI2CDriver, 4> V_I2C2;

  std::array<VCNL4020, 8> vcnl4020;

  MPR121::MPR121Config mpr121_run_config{
    /* global_config  */ MPR121::CDT_1 | MPR121::SFI_10 | MPR121::ESI_32 | MPR121::FFI_18 | 16,
    /* ele_config     */ MPR121::CL_ON_ALL | MPR121::ELEPROX_0 | 4,
    /* auto_config    */ MPR121::FFI_18 | MPR121::RETRY_2 | MPR121::BVA_ON_ALL | MPR121::AC_RECONF_EN | MPR121::AC_ENABLE,
    /* up_side_limit  */ 0x96u,
    /* low_side_limit */ 0x58u,
    /* target_level   */ 0x68u
  };
  MPR121::MPR121Config mpr121_stdby_config{
    /* global_config  */ MPR121::CDT_1 | MPR121::SFI_10 | MPR121::ESI_32 | MPR121::FFI_18 | 0,
    /* ele_config     */ MPR121::CL_ON_ALL | MPR121::ELEPROX_0 | 0, /* TODO: hack: turn buttons off for standby */
    /* auto_config    */ MPR121::FFI_18 | MPR121::RETRY_2 | MPR121::BVA_ON_ALL | MPR121::AC_RECONF_EN | MPR121::AC_ENABLE,
    /* up_side_limit  */ 0x96u,
    /* low_side_limit */ 0x58u,
    /* target_level   */ 0x68u
  };
  MPR121 mpr121;

  IwrapCanMux iwrapcanmux1;
  SerialCanMux sercanmux1;

  PowerManagement robot;

  UserThread userThread;

  uint8_t shellTermID;

public:
  Global() :
    adc1_vsys(ADCD1, adc1_conversion_group, 9000000),
    HW_I2C1(&I2CD1), HW_I2C2(&I2CD2),
    at24c01(0x400u / 0x08u, 0x08u, 500u, &HW_I2C2),
    memory(at24c01, /*BMSV*/ 1, /*bmsv*/ 2, /*HMV*/ 1, /*hmv*/ 1),
    ina219{{/* VDD      */ INA219::Driver(HW_I2C2, 0x45u),
            /* VIO 1.8V */ INA219::Driver(HW_I2C1, 0x44u),
            /* VIO 3.3V */ INA219::Driver(HW_I2C1, 0x40u),
            /* VIO 4.2V */ INA219::Driver(HW_I2C2, 0x40u),
            /* VIO 5.0V */ INA219::Driver(HW_I2C2, 0x44u)}
          },
    bq27500{{/* connector P7 */ BQ27500::Driver(HW_I2C2, *GPIOC, GPIOC_GAUGE_BATGD1_N, *GPIOC, GPIOC_GAUGE_BATLOW1),
             /* connector P8 */ BQ27500::Driver(HW_I2C1, *GPIOB, GPIOB_GAUGE_BATGD2_N, *GPIOB, GPIOB_GAUGE_BATLOW2)}
           },
    bq24103a_p7(), bq24103a_p8(),
    bq24103a{{&bq24103a_p7,
              &bq24103a_p8}
            },
    tps62113_vio18_33_50(), tps62113_vsys42(),
    wt12(&UARTD3),
    HW_PCA9544{{PCA9544<true>(&HW_I2C1, 0x07u),
                PCA9544<true>(&HW_I2C2, 0x07u)}
              },
    V_I2C1{{VI2CDriver(&HW_PCA9544[0], 0),
            VI2CDriver(&HW_PCA9544[0], 1),
            VI2CDriver(&HW_PCA9544[0], 2),
            VI2CDriver(&HW_PCA9544[0], 3)}
          },
    V_I2C2{{VI2CDriver(&HW_PCA9544[1], 0),
            VI2CDriver(&HW_PCA9544[1], 1),
            VI2CDriver(&HW_PCA9544[1], 2),
            VI2CDriver(&HW_PCA9544[1], 3)}
          },
    vcnl4020{{/* rear left        */ VCNL4020(&V_I2C1[1], &vcnl4020_config),
              /* left side rear   */ VCNL4020(&V_I2C1[3], &vcnl4020_config),
              /* left side front  */ VCNL4020(&V_I2C1[2], &vcnl4020_config),
              /* front left       */ VCNL4020(&V_I2C2[0], &vcnl4020_config),
              /* front right      */ VCNL4020(&V_I2C2[1], &vcnl4020_config),
              /* right side front */ VCNL4020(&V_I2C2[3], &vcnl4020_config),
              /* right side rear  */ VCNL4020(&V_I2C2[2], &vcnl4020_config),
              /* rear right       */ VCNL4020(&V_I2C1[0], &vcnl4020_config)}
            },
    mpr121(&HW_I2C2, 0),
    iwrapcanmux1(&SD1, &CAND1, CAN::POWER_MANAGEMENT_ID),
    sercanmux1(&SD1, &CAND1, CAN::POWER_MANAGEMENT_ID),
    robot(&CAND1),
    userThread()
  {
    memset(this->adc1_buffer, 0, sizeof(adc1_buffer));

    return;
  }

  ~Global()
  {
    return;
  }

};

} // end of namespace amiro

#endif /* AMIRO_GLOBAL_HPP_ */
