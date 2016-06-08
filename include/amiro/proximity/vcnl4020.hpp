#ifndef AMIRO_VCNL4020_H_
#define AMIRO_VCNL4020_H_

#include <amiro/bus/i2c/I2CParams.hpp>

namespace amiro {

class I2CDriver;

/**
 * VCNL4020 IR Proximity/Ambient Light Sensor Driver
 * \todo Interrupt Support
 */
class VCNL4020 : public chibios_rt::BaseStaticThread<256> {

    enum { SLA = 0x13u };

    enum { PRODUCT_ID_REVISION = 0x21u };

    struct registers {

      uint8_t  reserved_0x00_0x7F[0x80u];
      uint8_t  command;
      uint8_t  revision;
      uint8_t  proximity_rate;
      uint8_t  ir_led_current;
      uint8_t  ambient_param;
      uint16_t ambient_result; /* BE */
      uint16_t proximity_result; /* BE */
      uint8_t  interrupt_ctrl;
      uint16_t low_thres; /* BE */
      uint16_t high_thres; /* BE */
      uint8_t  interrupt_status;
      uint8_t  proximity_adjust;
      uint8_t  ambient_ir_level; /* Do Not Use */
    } __attribute__((packed));

  public:

    struct VCNL4020Config {

      uint8_t command;
      uint8_t ambient_parameter;
      uint8_t ir_led_current_mA;
      uint8_t proximity_rate;

    };

    enum {

      PROX_RATE_1_95    = 0x00u, /* 250 / (2**(7-0)) */
      PROX_RATE_3_90625 = 0x01u, /* 250 / (2**(7-1)) */
      PROX_RATE_7_8125  = 0x02u, /* 250 / (2**(7-2)) */
      PROX_RATE_15_625  = 0x03u, /* 250 / (2**(7-3)) */
      PROX_RATE_31_25   = 0x04u, /* 250 / (2**(7-4)) */
      PROX_RATE_62_5    = 0x05u, /* 250 / (2**(7-5)) */
      PROX_RATE_125     = 0x06u, /* 250 / (2**(7-6)) */
      PROX_RATE_250     = 0x07u  /* 250 / (2**(7-7)) */

    };

    enum {

      AMBIENT_CONT_CONV   = 0x80u,
      AMBIENT_RATE_1      = 0x00u,
      AMBIENT_RATE_2      = 0x10u,
      AMBIENT_RATE_3      = 0x20u,
      AMBIENT_RATE_4      = 0x30u,
      AMBIENT_RATE_5      = 0x40u,
      AMBIENT_RATE_6      = 0x50u,
      AMBIENT_RATE_8      = 0x60u,
      AMBIENT_RATE_10     = 0x70u,
      AMBIENT_AUTO_OFFSET = 0x10u,
      AMBIENT_AVG_1       = 0x00u,
      AMBIENT_AVG_2       = 0x01u,
      AMBIENT_AVG_4       = 0x02u,
      AMBIENT_AVG_8       = 0x03u,
      AMBIENT_AVG_16      = 0x04u,
      AMBIENT_AVG_32      = 0x05u,
      AMBIENT_AVG_64      = 0x06u,
      AMBIENT_AVG_128     = 0x07u,

    };

    enum {
      INT_THRES_SEL     = 0x01u,
      INT_THRES_EN      = 0x02u,
      INT_ALS_READY_EN  = 0x04u,
      INT_PROX_READY_EN = 0x08u,
    };

    enum {
      INT_TH_HI      = 0x01u,
      INT_TH_LOW     = 0x02u,
      INT_ALS_READY  = 0x04u,
      INT_PROX_READY = 0x08u,
      INT_CNT_1      = 0x00u,
      INT_CNT_2      = 0x20u,
      INT_CNT_4      = 0x40u,
      INT_CNT_8      = 0x60u,
      INT_CNT_16     = 0x80u,
      INT_CNT_32     = 0xA0u,
      INT_CNT_64     = 0xC0u,
      INT_CNT_128    = 0xE0u,
    };

    enum {

      MOD_DLY_MASK    = 0x07u,
      MOD_DLY_SHIFT   = 0x05u,
      MOD_DEAD_MASK   = 0x07u,
      MOD_DEAD_SHIFT  = 0x00u,
      PROX_FREQ_390   = 0x00u,
      PROX_FREQ_781   = 0x08u,
      PROX_FREQ_1_562 = 0x10u,
      PROX_FREQ_3_125 = 0x18u,

    };

    enum {

      CONFIG_LOCK   = 0x80u,
      ALS_DATA_RDY  = 0x40u,
      PROX_DATA_RDY = 0x20u,
      ALS_OD        = 0x10u,
      PROX_OD       = 0x08u,
      ALS_EN        = 0x04u,
      PROX_EN       = 0x02u,
      SELFTIMED_EN  = 0x01u,

    };

    /**
     * Return types of getCheck()
     */
    enum {
      CHECK_OK   = 0x00u,
      CHECK_FAIL = 0x01u,
    };

    /**
     * Return types of calibration
     */
    enum {
      CALIB_OK   = 0x00u,
      CALIB_FAIL = 0x01u,
    };

  public:
    VCNL4020(I2CDriver *driver, const VCNL4020Config *config);
    virtual ~VCNL4020();

    chibios_rt::EvtSource* getEventSource();

    /**
     * Returns the last measured ambient light value.
     *
     * @return Measured ambient light value
     */
    uint16_t getAmbientLight();

    /**
     * Returns the last measured proximity value w/o offset.
     * Additionally, the value is scaled, so that full-scale
     * as a measurement can be reached.
     *
     * @return Measured proximity value without offset
     */
    uint16_t getProximityScaledWoOffset();

    /**
     * Returns the last measured proximity value.
     *
     * @return Measured proximity value
     */
    uint16_t getProximity();

    /**
     * Returns the offset of the vcnl4020.
     * calibrate() or setOffset() needs to be run first,
     * otherwise the return value is 0.
     *
     * @return Offset value of the device
     */
    uint16_t getProximityOffset();

    /**
     * Sets the offset value, which will be substracted from
     * the measured value.
     *
     * @param offset
     */
    void setProximityOffset(uint16_t offset);

    /**
     * Checks the vcnl4020 by reading the product
     * id from it an comparing it to the standard
     * value
     *
     * @return [CHECK_OK | CHECK_FAIL]
     */
    uint8_t getCheck();

    /**
     * Starts the calibration of the vcnl4020 and
     *
     * @return [ CALIB_OK | CALIB_FAIL]
     */
    uint8_t calibrate();

  protected:
    virtual msg_t main(void);

  private:
    inline msg_t readIntensities();
    inline msg_t writeIRConf();

    /**
     * Starts the offset calibration
     *
     * @return [ CALIB_OK | CALIB_FAIL]
     */
    uint8_t calibrateOffset(uint16_t &proximityFloorMeanValue);

    I2CDriver *driver;
    const VCNL4020Config *config;
    chibios_rt::EvtSource eventSource;
    uint16_t ambient;
    uint16_t proximity;
    uint16_t proximityOffset;
    I2CTxParams tx_params;
};

}

#endif /* AMIRO_VCNL4020_H_ */
