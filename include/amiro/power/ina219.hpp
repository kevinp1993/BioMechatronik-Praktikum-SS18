/*
 * INA219 - Power Monitor
 */

#ifndef AMIRO_INA219_H_
#define AMIRO_INA219_H_

#include <ch.hpp>
#include <amiro/bus/i2c/I2CParams.hpp>
#include <amiro/bus/i2c/I2CDriver.hpp>

#include <amiro/BaseSensor.hpp>

namespace amiro {

  namespace INA219 {

    namespace Configuration
    {
      union Register {
        uint16_t value = 0;
        struct {
          uint16_t mode : 3;
          uint16_t sadc : 4;
          uint16_t badc : 4;
          uint16_t pg : 2;
          uint16_t brng : 1;
          uint16_t rsvd : 1;
          uint16_t rst : 1;
        } content;
      };

      enum BusRange {
        BRNG_16V = 0,
        BRNG_32V = 1, BRNG_DEFAULT = 1
      };

      enum Pga {
        PGA_40mV = 0,
        PGA_80mV = 1,
        PGA_160mV = 2,
        PGA_320mV = 3, PGA_DEFAULT = 3
      };

      enum Adc {
        ADC_84us = 0x0u,
        ADC_148us = 0x1u,
        ADC_276us = 0x2u,
        ADC_532us = 0x3u, ADC_632us_alt = 0x8u, ADC_DEFAULT = 0x3u,
        ADC_1060us = 0x9u,
        ADC_2130us = 0xAu,
        ADC_4260us = 0xBu,
        ADC_8510us = 0xCu,
        ADC_17020us = 0xDu,
        ADC_34050us = 0xEu,
        ADC_68100us = 0xFu
      };

      enum Mode {
        MODE_PowerDown = 0x0u,
        MODE_ShuntV_Triggered = 0x1u,
        MODE_BusV_Triggered = 0x2u,
        MODE_ShuntBus_Triggered = 0x3u,
        MODE_AdcOff = 0x4u,
        MODE_ShuntV_Continuous = 0x5u,
        MODE_BusV_Continuous = 0x6u,
        MODE_ShuntBus_Continuous = 0x7u, MODE_DEFAULT = 0x7u
      };
    } // end of namespace Configuration



    struct InitData {
      Configuration::Register configuration;
      uint16_t calibration;
      uint16_t current_lsb_uA;
    };

    struct CalibData {
      struct CalibInput {
        float shunt_resistance_O;
        float max_expected_current_A;
        uint16_t current_lsb_uA;
        Configuration::Register configuration;
      } input;
      struct CalibOutput {
        float max_current_before_overflow_A;
        float max_shuntvoltage_before_overflow_V;
        uint16_t current_lsb_uA;
        uint16_t calibration_value;
      } output;

      uint16_t power_lsb_uW() const
      {
        return 20 * output.current_lsb_uA;
      }

      float maximum_power_W() const
      {
        return output.max_current_before_overflow_A * ((input.configuration.content.brng == Configuration::BusRange::BRNG_32V)? 32 : 16);}
    };

    struct BusVoltage {
      uint32_t voltage_uV : 25;
      uint32_t rsvd : 5;
      uint32_t conversion_ready : 1;
      uint32_t overflow : 1;
    };

    class Driver : public BaseSensor<InitData,CalibData>, public chibios_rt::BaseStaticThread<256>
    {
    public:
      enum SelftestResult {
        ST_OK = BaseSensor<>::OK,
        ST_FAIL_ANY = BaseSensor<>::FAIL,
        ST_FAIL_BACKUP = BaseSensor<>::FAIL + 1,
        ST_FAIL_RESET = BaseSensor<>::FAIL + 2,
        ST_FAIL_IS_DEFAULT = BaseSensor<>::FAIL + 3,
        ST_FAIL_WRITE_CONFIG = BaseSensor<>::FAIL + 4,
        ST_FAIL_WRITE_CALIB = BaseSensor<>::FAIL + 5,
        ST_FAIL_CHECK_CONFIG = BaseSensor<>::FAIL + 6,
        ST_FAIL_CHECK_CALIB = BaseSensor<>::FAIL + 7
      };

    private:
      union BusVoltage {
        uint16_t value = 0;
        struct {
          uint16_t overflow : 1;
          uint16_t conversion_ready : 1;
          uint16_t rsvd : 1;
          uint16_t voltage : 13;
        } content;
      };

      struct Status {
        Driver::BusVoltage bus_voltage;
        uint16_t power;

        Status() {bus_voltage.value = 0; power = 0;}
      };

    public:
      enum RegisterAddress {
        REG_CONFIGURATION = 0x00u,
        REG_SHUNT_VOLTAGE = 0x01u,
        REG_BUS_VOLTAGE = 0x02u,
        REG_POWER = 0x03u,
        REG_CURRENT = 0x04u,
        REG_CALIBRATION = 0x05u
      };

    private:
      enum RegisterMask {
        MASK_CONFIGURATION = 0x3FFFu,
        MASK_CALIBRATION = 0xFFFEu,
        MASK_RESET = 0x8000u
      };

      I2CDriver *const i2c_driver;
      I2CTxParams tx_params;
      Configuration::Register config;
      uint16_t current_lsb_uA;
      Status status;

    public:
      /* constructor & destructor */
      explicit Driver(I2CDriver &i2c_driver, const uint8_t i2c_address);
      virtual ~Driver();

      /* inherited functions from BaseSensor */
      msg_t init(InitData* initialization_data = NULL);
      msg_t update();
      msg_t wakeup();
      msg_t hibernate();
#ifndef AMIRO_NCALIBRATION
      msg_t calibration(CalibData* calibration_data = NULL);
#endif
#ifndef AMIRO_NSELFTEST
      msg_t selftest();
#endif

      INA219::BusVoltage getVoltage() const
      {
        return this->busVoltageReg2uV(this->status.bus_voltage);
      }

      uint32_t getPower_uW() const
      {
        return this->powerReg2uW(this->status.power);
      }

      uint32_t getCurrent_uA() const
      {
        return this->powerReg2uW(this->status.power) / this->busVoltageReg2uV(this->status.bus_voltage).voltage_uV;
      }



      inline Configuration::Register readConfiguration();
      inline int32_t readShuntVoltage_uV();
      inline INA219::BusVoltage readBusVoltage();
      inline uint32_t readPower_uW();
      inline int32_t readCurrent_uA();
      inline uint16_t readCalibration();

      uint8_t reset();

    protected:
      virtual msg_t main(void);

    public:
      msg_t readRegister(const RegisterAddress reg, uint16_t& dst);
      msg_t writeRegister(const RegisterAddress reg, const uint16_t& val);

    private:
      static inline INA219::BusVoltage busVoltageReg2uV(const INA219::Driver::BusVoltage reg_val)
      {
        INA219::BusVoltage bus_voltage;
        bus_voltage.conversion_ready = reg_val.content.conversion_ready;
        bus_voltage.overflow = reg_val.content.conversion_ready;
        /*
         * The least significant bit represents 4mV.
         * -> Multiply with 4000 in order to convert to uV.
         */
        bus_voltage.voltage_uV = reg_val.content.voltage * 4000;
        return bus_voltage;
      }

      inline uint32_t powerReg2uW(const uint16_t reg_val) const
      {
        /*
         * Multiply with the value of the least significant bit for the current and the constant 20.
         * -> see data sheet, chapter 'Programming the INA219 power measurement engine' for details
         */
        return reg_val * 20 * this->current_lsb_uA;
      }
    };

  } // end of namespace INA219

}

#endif /* AMIRO_INA219_H_ */
