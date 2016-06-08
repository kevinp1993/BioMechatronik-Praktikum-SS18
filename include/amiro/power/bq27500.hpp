/*
 * BQ27500 - Fuel Gauge
 */

#ifndef AMIRO_BQ27500_H_
#define AMIRO_BQ27500_H_

#include <ch.hpp>
#include <amiro/bus/i2c/I2CParams.hpp>
#include <amiro/bus/i2c/I2CDriver.hpp>

#include <amiro/BaseSensor.hpp>

namespace amiro {

  namespace BQ27500 {

    struct InitData {};

    struct CalibData
    {
      struct Configuration
      {
        struct Safety {
          enum SubclassID {SUB_ID = 2};
          const int16_t ot_chg = 450; // 0.1°C
          const uint8_t ot_chg_time = 2; // seconds
          const int16_t ot_chg_recovery = 400; // 0.1°C
          const int16_t ot_dsg = 600; // 0.1°C
          const uint8_t ot_dsg_time = 2; // seconds
          const int16_t ot_dsg_recovery = 500; // 0.1°C
          enum Offset {
            OFFSET_OtChg = 0,
            OFFSET_OtChgTime = 2,
            OFFSET_OtChgRecovery = 3,
            OFFSET_OtDsg = 5,
            OFFSET_OtDsgTime = 7,
            OFFSET_OtDsgRecovery = 8
          };
        } safety;

        struct ChargeInhibitConfig {
          enum SubclassID {SUB_ID = 32};
          const int16_t charge_inhibit_temp_low = 0; // 0.1°C
          const int16_t charge_inhibit_temp_high = 450; // 0.1°C
          const int16_t temp_hys = 50; // 0.1°C
          enum Offset {
            OFFSET_ChargeInhibitTempLow = 0,
            OFFSET_ChargeInhibitTempHigh = 2,
            OFFSET_TempHys = 4
          };
        } charge_inhibit_config;

        struct Charge {
          enum SubclassID {SUB_ID = 34};
          const int16_t charging_voltage = 4200; // mV
          const int16_t delta_temperature = 50; // 0.1°C
          const int16_t suspend_temperature_low = -200; // 0.1°C
          const int16_t suspend_temperature_high = 600; // 0.1°C
          enum Offset {
            OFFSET_ChargingVoltage = 2,
            OFFSET_DeltaTemperature = 4,
            OFFSET_SuspendTemperatureLow = 6,
            OFFSET_SuspendTemperatureHigh = 8
          };
        } charge;

        struct ChargeTermination {
          enum SubclassID {SUB_ID = 36};
          const int16_t taper_current = 100; // mA
          const int16_t minimum_taper_charge = 25; // 0.01mAh
          const int16_t taper_voltage = 100; // mV
          const uint16_t current_taper_window = 40; // seconds
          enum Offset {
            OFFSET_TaperCurrent = 2,
            OFFSET_MinimumTaperCharge = 4,
            OFFSET_TaperVoltage = 6,
            OFFSET_CurrentTaperWindow = 8
          };
        } charge_termination;

        struct Data {
          enum SubclassID {SUB_ID = 48};
          const int8_t initial_standby_current = -10; // mA
          const int16_t initial_max_load_current = -500; // mA
          const int16_t cc_threshold = 900; // mAh
          const int16_t design_capacity = 1950; // mAh
          const char device_name[8] = "bq27500";
          enum Offset {
            OFFSET_InitialStandbyCurrent = 4,
            OFFSET_InitialMaxLoadCurrent = 5,
            OFFSET_CCThreshold = 7,
            OFFSET_DesignCapacity = 10,
            OFFSET_DeviceName = 12
          };
        } data;

        struct Discharge {
          enum SublcassID {SUB_ID = 49};
          const uint8_t soc1_set_threshold = 150; // mAh
          const uint8_t soc1_clear_threshold = 175; // mAh
          const uint8_t socf_set_threshold = 75; // mAh
          const uint8_t socf_clear_threshold = 100; // mAh
          enum Offset {
            OFFSET_Soc1SetThreshold = 0,
            OFFSET_Soc1ClearThreshold = 1,
            OFFSET_SocfSetThreshold = 2,
            OFFSET_SocfClearThreshold = 3
          };
        } discharge;

        struct Registers {
          enum SubclassID {SUB_ID = 64};
          union OperationConfiguration {
            const uint16_t value = 0x0979u;
            struct {
              uint16_t temps : 1;
              uint16_t rsvd_1 : 1;
              uint16_t batg_pol : 1;
              uint16_t batl_pol : 1;
              uint16_t rmfcc : 1;
              uint16_t sleep : 1;
              uint16_t idselen : 1;
              uint16_t rsvd_7 : 1;
              uint16_t rsns : 2;
              uint16_t iwake : 1;
              uint16_t pfc : 2;
              uint16_t rsvd_13 : 1;
              uint16_t batg_ovr : 1;
              uint16_t rescap : 1;
            } content;
          } operation_configuration;
          enum Offset {
            OFFSET_OperationConfiguration = 0
          };
        } registers;

        struct Power {
          enum SubclassID {SUB_ID = 68};
          const int16_t flash_update_ok_voltage = 2800; // mV
          const int16_t sleep_current = 10; // mA
          const uint16_t hibernate_current = 8; // mA
          const uint16_t hibernate_voltage = 2550; // mV
          enum Offset {
            OFFSET_FlashUpdateOkVoltage = 0,
            OFFSET_SleepCurrent = 7,
            OFFSET_HibernateCurrent = 16,
            OFFSET_HibernateVoltage = 18
          };
        } power;
      } configuration;

      struct SystemData
      {
        struct ManufacturerInfo {
          enum SubclassID {SUB_ID = 57};
          // the very first byte is reserved as magic byte (0xAA)
          uint8_t version_major = 1;
          uint8_t version_minor = 0;
          enum Offset {
            OFFSET_VersionMajor = 1,
            OFFSET_VersionMinor = 2
          };
        } manufacturer_info;
      } system_data;

      struct FuelGauging
      {
        struct ITConfig {
          enum SubclassID {SUB_ID = 80};
          const uint8_t load_select = 1;
          const uint8_t load_mode = 0;
          const int16_t terminate_voltage = 3000; // mV
          const int16_t user_rate_mA = 0; // mA
          const int16_t user_rate_mW = 0; // mW
          const int16_t reserve_cap_mAh = 0; // mAh
          const int16_t reserve_cap_mWh = 0; // mWh
          enum Offset {
            OFFSET_LoadSelect = 0,
            OFFSET_LoadMode = 1,
            OFFSET_TerminateVoltage = 48,
            OFFSET_UserRate_mA = 53,
            OFFSET_UserRate_mW = 55,
            OFFSET_ReserveCap_mAh = 57,
            OFFSET_ReserveCap_mWh = 59
          };
        } it_config;

        struct CurrentThresholds {
          enum SubclassID {SUB_ID = 81};
          const int16_t dsg_current_threshold = 60; // mA
          const int16_t chg_current_threshold = 75; // mA
          const int16_t quit_current = 40; // mA
          const uint16_t dsg_relax_time = 60; // seconds
          const uint16_t chg_relax_time = 60; // seconds
          const uint8_t quit_relax_time = 1; // seconds
          enum Offset {
            OFFSET_DsgCurrentThreshold = 0,
            OFFSET_ChgCurrentThreshold = 2,
            OFFSET_QuitCurrent = 4,
            OFFSET_DsgRelaxTime = 6,
            OFFSET_ChgRelaxTime = 8,
            OFFSET_QuitRelaxTime = 10
          };
        } current_thresholds;

        struct State {
          enum SubclassID {SUB_ID = 82};
          const uint8_t it_enable = 0x00;
          const uint8_t application_status = 0x00;
          const int16_t qmax_0 = 1950; // mAh
          const uint16_t cycle_count_0 = 0;
          const uint8_t update_status_0 = 0x00;
          const int16_t qmax_1 = 1950; // mAh
          const uint16_t cycle_count_1 = 0;
          const uint8_t update_status_1 = 0x00;
          const int16_t avg_I_last_run = -299; // mA
          const int16_t avg_P_last_run = -1131; // mAh
          enum Offset {
            OFFSET_ItEnable = 0,
            OFFSET_ApplicationStatus = 1,
            OFFSET_Qmax0 = 2,
            OFFSET_CycleCount0 = 4,
            OFFSET_UpdateStatus0 = 6,
            OFFSET_Qmax1 = 7,
            OFFSET_CycleCount1 = 9,
            OFFSET_UpdateStatus1 = 11,
            OFFSET_AvgILastRun = 16,
            OFFSET_AvgPLastRun = 18
          };
        } state;
      } fuel_gauging;

      struct Security
      {
        struct Codes {
          enum SubclassID {SUB_ID = 112};
          const uint16_t unseal_key_0 = 0x3672;
          const uint16_t unseal_key_1 = 0x0414;
          const uint16_t full_access_key_0 = 0xFFFF;
          const uint16_t full_access_key_1 = 0xFFFF;
          enum Offset {
            OFFSET_UnsealKey0 = 0,
            OFFSET_UnsealKey1 = 2,
            OFFSET_FullAccessKey0 = 4,
            OFFSET_FullAccessKey1 = 6
          };
        } codes;
      } security;

    };

    class Driver : public BaseSensor<InitData,CalibData>, public chibios_rt::BaseStaticThread<256>
    {
    public:
      enum StandardCommand {
                                                                            //  unit    |      access
                                                                            //          | sealed  | unsealed
                                                                            // ---------+---------+----------
        STD_CMD_Control = 0x00u, STD_CMD_CNTL = 0x00u,                      // N/A      | R/W     | R/W
        STD_CMD_AtRate = 0x02u, STD_CMD_AR = 0x02u,                         // mA       | R/W     | R/W
        STD_CMD_AtRateTimeToEmpty = 0x04u, STD_CMD_ARTTE = 0x04u,           // minutes  | R       | R/W
        STD_CMD_Temperatur = 0x06u, STD_CMD_TEMP = 0x06u,                   // 0.1 K    | R       | R/W
        STD_CMD_Voltage = 0x08u, STD_CMD_VOLT = 0x08u,                      // mV       | R       | R/W
        STD_CMD_Flags = 0x0Au, STD_CMD_FLAGS = 0x0Au,                       // N/A      | R       | R/W
        STD_CMD_NominalAvailableCapacity = 0x0Cu, STD_CMD_NAC = 0x0Cu,      // mAh      | R       | R/W
        STD_CMD_FullAvailableCapacity = 0x0Eu, STD_CMD_FAC = 0x0Eu,         // mAh      | R       | R/W
        STD_CMD_RemainingCapacity = 0x10u, STD_CMD_RM = 0x10u,              // mAh      | R       | R/W
        STD_CMD_FullChargeCapacity = 0x12u, STD_CMD_FCC = 0x12u,            // mAh      | R       | R/W
        STD_CMD_AverageCurrent = 0x14u, STD_CMD_AI = 0x14u,                 // mA       | R       | R/W
        STD_CMD_TimeToEmpty = 0x16u, STD_CMD_TTE = 0x16u,                   // minutes  | R       | R/W
        STD_CMD_TimeToFull = 0x18u, STD_CMD_TTF = 0x18u,                    // minutes  | R       | R/W
        STD_CMD_StandbyCurrent = 0x1Au, STD_CMD_SI = 0x1Au,                 // mA       | R       | R/W
        STD_CMD_StandbyTimeToEmpty = 0x1Cu, STD_CMD_STTE = 0x1Cu,           // minutes  | R       | R/W
        STD_CMD_MaxLoadCurrent = 0x1Eu, STD_CMD_MLI = 0x1Eu,                // mA       | R       | R/W
        STD_CMD_MaxLoadTimeToEmpty = 0x20u, STD_CMD_MLTTE = 0x20u,          // minutes  | R       | R/W
        STD_CMD_AvailableEnergy = 0x22u, STD_CMD_AE = 0x22u,                // mWh      | R       | R/W
        STD_CMD_AveragePower = 0x24u, STD_CMD_AP = 0x24u,                   // mW       | R       | R/W
        STD_CMD_TimeToEmptyAtConstantPower = 0x26u, STD_CMD_TTECP = 0x26u,  // minutes  | R       | R/W
        STD_CMD_CycleCount = 0x2Au, STD_CMD_CC = 0x2Au,                     // counts   | R       | R/W
        STD_CMD_StateOfCharge = 0x2Cu, STD_CMD_SOC = 0x2Cu                  // %        | R       | R/W
      };

      enum ControlSubcommand {
                                            // sealed access
                                            // -------------
        SUB_CMD_CONTROL_STATUS = 0x0000u,   // yes
        SUB_CMD_DEVICE_TYPE = 0x0001u,      // yes
        SUB_CMD_FW_VERSION = 0x0002u,       // yes
        SUB_CMD_HW_VERSION = 0x0003u,       // yes
        SUB_CMD_DF_CHECKSUM = 0x0004u,      // no
        SUB_CMD_RESET_DATA = 0x0005u,       // yes
        SUB_CMD_PREV_MACWRITE = 0x0007u,    // yes
        SUB_CMD_CHEM_ID = 0x0008u,          // yes
        SUB_CMD_BOARD_OFFSET = 0x0009u,     // no
        SUB_CMD_CC_INT_OFFSET = 0x000Au,    // no
        SUB_CMD_WRITE_OFFSET = 0x000Bu,     // no
        SUB_CMD_SET_HIBERNATE = 0x0011u,    // yes
        SUB_CMD_CLEAR_HIBERNATE = 0x0012u,  // yes
        SUB_CMD_SET_SLEEPp = 0x0013u,       // yes
        SUB_CMD_CLEAR_SLEEPp = 0x0014u,     // yes
        SUB_CMD_SEALED = 0x0020u,           // no
        SUB_CMD_IT_ENABLE = 0x0021u,        // no
        SUB_CMD_IF_CHECKSUM = 0x0022u,      // no
        SUB_CMD_CAL_MODE = 0x0040u,         // no
        SUB_CMD_RESET = 0x0041u,            // no
      };

      enum ExtendedCommand {
                                                                    // length | unit | sealed | unsealed
                                                                    // -------+------+--------+----------
        EXT_CMD_DesignCapacity = 0x3Cu, EXT_CMD_DCAP = 0x3Cu,       //      2 |  mAh |     R  |      R
        EXT_CMD_DataFlashClass = 0x3Eu, EXT_CMD_DFCLS = 0x3Eu,      //      1 |  N/A |    N/A |     R/W
        EXT_CMD_DataFlashBlock = 0x3Fu, EXT_CMD_DFBLK = 0x3Fu,      //      1 |  N/A |    R/W |     R/W
        EXT_CMD_BlockData = 0x40u, EXT_CMD_DFD = 0x40u,             //     32 |  N/A |     R  |     R/W
        EXT_CMD_BlockDataCheckSum = 0x60u,EXT_CMD_DFDCKS = 0x60u,   //      1 |  N/A |    R/W |     R/W
        EXT_CMD_BlockDataControl = 0x61u, EXT_CMD_DFDCNTL = 0x61u,  //      1 |  N/A |    N/A |     R/W
        EXT_CMD_DeviceNameLength = 0x62u, EXT_CMD_DNAMELEN = 0x62u, //      1 |  N/A |     R  |      R
        EXT_CMD_DeviceName = 0x63u, EXT_CMD_DNAME = 0x63u,          //      7 |  N/A |     R  |      R
        EXT_CMD_ApplicationStatus = 0x6Au, EXT_CMD_APPSTAT = 0x6Au  //      1 |  N/A |     R  |      R
      };

      enum ExtendedCommandAccess {
        EXT_CMD_READ = 0,
        EXT_CMD_WRITE = 1,
      };

      enum SelftestResult {
        ST_OK = BaseSensor<>::OK,
        ST_ABORT_NO_BAT = BaseSensor<>::OK+1,
        ST_FAIL_ANY = BaseSensor<>::FAIL,
        ST_FAIL_READ_HW_VERSION = BaseSensor<>::FAIL + 1,
        ST_FAIL_READ_FW_VERSION = BaseSensor<>::FAIL + 2,
        ST_FAIL_READ_DEVICENAMELENGTH = BaseSensor<>::FAIL + 3,
        ST_FAIL_READ_DEVICENAME = BaseSensor<>::FAIL + 4,
        ST_FAIL_SET_ITENABLE = BaseSensor<>::FAIL + 5,
        ST_FAIL_READ_STATUS = BaseSensor<>::FAIL + 6,
        ST_FAIL_READ_FLAGS = BaseSensor<>::FAIL + 7,
        ST_FAIL_READ_TEMP = BaseSensor<>::FAIL + 8,
        ST_FAIL_READ_FAC = BaseSensor<>::FAIL + 9,
        ST_FAIL_READ_FCC = BaseSensor<>::FAIL + 10,
        ST_FAIL_READ_RM = BaseSensor<>::FAIL + 11,
        ST_FAIL_READ_SOC = BaseSensor<>::FAIL + 12,
        ST_FAIL_READ_VOLT = BaseSensor<>::FAIL + 13,
        ST_FAIL_READ_AI = BaseSensor<>::FAIL + 14,
        ST_FAIL_READ_AP = BaseSensor<>::FAIL + 15,
        ST_FAIL_READ_TTE = BaseSensor<>::FAIL + 16,
        ST_FAIL_READ_TTF = BaseSensor<>::FAIL + 17
      };

      union ControlStatus {
        uint16_t value = 0;
        struct {
          uint16_t qen : 1;
          uint16_t vok : 1;
          uint16_t rup_dis : 1;
          uint16_t ldmd : 1;
          uint16_t sleep : 1;
          uint16_t snooze : 1;
          uint16_t hibernate : 1;
          uint16_t rsvd_7_8_9 : 3;
          uint16_t bca : 1;
          uint16_t cca : 1;
          uint16_t csv : 1;
          uint16_t ss : 1;
          uint16_t fas : 1;
          uint16_t rsvd_15 : 1;
        } content;
      };

      union Flags {
        uint16_t value = 0;
        struct {
          uint16_t dsg : 1;
          uint16_t socf : 1;
          uint16_t soc1 : 1;
          uint16_t bat_det : 1;
          uint16_t wait_id : 1;
          uint16_t ocv_gd : 1;
          uint16_t rsvd_6_7 : 2;
          uint16_t chg : 1;
          uint16_t fc : 1;
          uint16_t xchg : 1;
          uint16_t chg_inh : 1;
          uint16_t rsvd_12_13 : 2;
          uint16_t otd : 1;
          uint16_t otc : 1;
        } content;
      };

      union Version {
        uint16_t value = 0;
        struct {
          uint16_t minor_low : 4;
          uint16_t minor_high : 4;
          uint16_t major_low : 4;
          uint16_t major_high : 4;
        } content;
      };

      struct UpdateData {
        uint16_t minutes_to_empty;
        uint16_t minutes_to_full;
        uint16_t average_power_mW;
        uint8_t state_of_charge; // %
      };

    private:
      enum I2CAddress {
        I2C_ADDR = 0x55u
      };

      enum DataFlashSubClassID {
        CONFIGURATION_Safety = 2,
        CONFIGURATION_ChargeInhibitConfig = 32,
        CONFIGURATION_Charge = 34,
        CONFIGURATION_ChargeTermination = 36,
        CONFIGURATION_Data = 48,
        CONFIGURATION_Discharge = 49,
        SYSTEM_DATA_ManufacturerInfo = 57,
        CONFIGURATION_Registers = 64,
        CONFIGURATION_Power = 68,
        FUEL_GAUGING_ItCfg = 80,
        FUEL_GAUGING_CurrentThresholds = 81,
        FUEL_GAUGING_State = 82,
        DEFAULT_RA_TABLES_Def0Ra = 87,
        DEFAULT_RA_TABLES_Def1Ra = 88,
        RA_TABLES_Pack0Ra = 91,
        RA_TABLES_Pack1Ra = 92,
        RA_TABLES_Pack0Rax = 93,
        RA_TABLES_Pack1Rax = 94,
        CALIBRATION_Data = 104,
        CALIBRATION_Current = 107,
        SECURITY_Codes = 112
      };

      union DataFlashBlock {
        uint8_t raw[33];
        struct Content {
          uint8_t data[32];
          uint8_t checksum;
        } content;
      };

      I2CDriver *const i2c_driver;
      I2CTxParams tx_params;

      chibios_rt::EvtSource eventSource;

      const GPIO_TypeDef* const batgd_pingrp;
      const uint8_t batgd_pin;
      const GPIO_TypeDef* const batlow_pingrp;
      const uint8_t batlow_pin;

      UpdateData status;

    public:
      Driver(I2CDriver &i2c_driver, const GPIO_TypeDef &batgd_pingrp, const uint8_t batgd_pin, const GPIO_TypeDef &batlow_pingrp, const uint8_t batlow_pin);
      virtual ~Driver();

      chibios_rt::EvtSource* getEventSource();

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

      bool isBatteryGood() const;
      bool isBatteryLow() const;

      const UpdateData& getStatus() const;

    protected:
      virtual msg_t main(void);

    private:
      msg_t stdCommand(const StandardCommand cmd, uint16_t &dst);
      msg_t readName();
      msg_t subCommand(const ControlSubcommand cmd, uint16_t* dst = NULL);
      msg_t extCommand(const ExtendedCommand cmd, const ExtendedCommandAccess rw, uint8_t* buf = NULL, const uint8_t length = 1, const uint8_t offset = 0);

      msg_t readDataFlashBlock(DataFlashBlock &block, const DataFlashSubClassID sub_id, const uint8_t sub_block = 0);

    };

  }

}

#endif /* AMIRO_BQ27500_H_ */
