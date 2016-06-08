#ifndef LIS331DLH_HPP_
#define LIS331DLH_HPP_

#include <ch.hpp>

namespace amiro {

  class HWSPIDriver;

  class LIS331DLH : public chibios_rt::BaseStaticThread<256> {

  public:

  struct InterruptConfig {

    uint8_t config;
    uint8_t ths;
    uint8_t duration;

  } __attribute__((packed));

  struct LIS331DLHConfig {
    uint8_t         ctrl1;
    uint8_t         ctrl2;
    uint8_t         ctrl3;
    uint8_t         ctrl4;
    uint8_t         ctrl5;
    InterruptConfig int1_cfg;
    InterruptConfig int2_cfg;
  };

  enum {
    AOI_OR_INT  = 0x00u,
    AOI_6MOVE   = 0x40u,
    AOI_AND_INT = 0x80u,
    AOI_6POS    = 0xC0u,
    ZHIE        = 0x20u,
    ZLIE        = 0x10u,
    YHIE        = 0x08u,
    YLIE        = 0x04u,
    XHIE        = 0x02u,
    XLIE        = 0x01u,
  };

  enum {
    IA          = 0x40u,
    ZH          = 0x20u,
    ZL          = 0x10u,
    YH          = 0x08u,
    YL          = 0x04u,
    XH          = 0x02u,
    XL          = 0x01u,
  };

  enum {
    THS_MASK      = 0x7Fu,
  };

  enum {
    DURATION_MASK = 0x7Fu,
  };

  enum {
    PM_OFF          = 0x00u,
    PM_ODR          = 0x20u,
    PM_0_5_HZ       = 0x40u,
    PM_1_HZ         = 0x60u,
    PM_2_HZ         = 0x80u,
    PM_5_HZ         = 0xA0u,
    PM_10_HZ        = 0xC0u,
    DR_50HZ_37LP    = 0x00u,
    DR_100HZ_74LP   = 0x08u,
    DR_400HZ_292LP  = 0x10u,
    DR_1000HZ_780LP = 0x18u,
    ZEN             = 0x04u,
    YEN             = 0x02u,
    XEN             = 0x01u,
  };

  enum {
    BOOT_REBOOT  = 0x80u,
    HPM_NORMAL0  = 0x00u,
    HPM_REF      = 0x20u,
    HPM_NORMAL1  = 0x40u,
    FDS_FILTER   = 0x10u,
    HPEN_INT2    = 0x08u,
    HPEN_INT1    = 0x04u,
    HPCF_8       = 0x00u,
    HPCF_16      = 0x01u,
    HPCF_32      = 0x02u,
    HPCF_64      = 0x03u,
  };

  enum {
    INT_HIGH     = 0x00u,
    INT_LOW      = 0x80u,
    PUSH_PULL    = 0x00u,
    OPEN_DRAIN   = 0x40u,
    LIR2_LATCH   = 0x20u,
    I2_CFG_I2    = 0x00u,
    I2_CFG_I1_I2 = 0x08u,
    I2_CFG_DRY   = 0x10u,
    I2_CFG_BOOT  = 0x18u,
    LIR1_LATCH   = 0x04u,
    I1_CFG_I1    = 0x00u,
    I1_CFG_I1_I2 = 0x01u,
    I1_CFG_DRY   = 0x02u,
    I1_CFG_BOOT  = 0x03u,
  };

  enum {
    BDU_CONT = 0x00u,
    BDU_STOP = 0x80u,
    BLE_LE   = 0x00u,
    BLE_BE   = 0x40u,
    FS_2G    = 0x00u,
    FS_4G    = 0x10u,
    FS_8G    = 0x30u,
    STSIGN_POS = 0x00u,
    STSIGN_NEG = 0x08u,
    ST_DISABLE = 0x00u,
    ST_ENABLE  = 0x02u,
    SIM_4WI    = 0x00u,
    SIM_3WI    = 0x01u,
  };

  enum {
    SLEEP_TO_WAKE_OFF = 0x00u,
    SLEEP_TO_WAKE_ON  = 0x03u,
  };

  /**
   * Return types of getCheck()
   */
  enum {
    CHECK_OK   = 0x00,
    CHECK_FAIL = 0x01,
  };

  /**
  * Top view of the AMiRo with charger in the back (F:Front, B:Back).
  * Z is pointing into the ground (Apply right-hand-rule):
  *    ___________
  *   /    F      \
  *  /     X       \
  * |      |        |
  * |      Z---Y    |
  * |               |
  *  \             /
  *   \____B______/
  */
  enum {
    AXIS_X   = 0x00u,
    AXIS_Y   = 0x01u,
    AXIS_Z   = 0x02u,
  };

  //private:

  struct registers {
    uint8_t  reserved_0x00_0x0E[0x0Fu];
    uint8_t  who_am_i;
    uint8_t  reserved_0x10_0x1F[0x10u];
    uint8_t  ctrl_reg1;
    uint8_t  ctrl_reg2;
    uint8_t  ctrl_reg3;
    uint8_t  ctrl_reg4;
    uint8_t  ctrl_reg5;
    uint8_t  hp_filter_reset;
    uint8_t  reference;
    uint8_t  status_reg;
    uint16_t out_x; /* LE */
    uint16_t out_y; /* LE */
    uint16_t out_z; /* LE */
    uint8_t  reserved_0x2E_0x2F[0x02u];
    uint8_t  int1_cfg;
    uint8_t  int1_src;
    uint8_t  int1_ths;
    uint8_t  int1_duration;
    uint8_t  int2_cfg;
    uint8_t  int2_src;
    uint8_t  int2_ths;
    uint8_t  int2_duration;
    uint8_t  reserved_0x38_0x3F[0x08u];
  } __attribute__((packed));

  enum {
    LIS331DLH_ID = 0x32u
  };

  enum {
    ZYXOR = 0x80,
    ZOR   = 0x40,
    YOR   = 0x20,
    XOR   = 0x10,
    ZYXDA = 0x08,
    ZDA   = 0x04,
    YDA   = 0x02,
    XDA   = 0x01,
  };

  enum {
    SPI_MULT  = 0x40u,
    SPI_READ  = 0x80u,
    SPI_WRITE = 0x00u,
  };

  public:
    LIS331DLH(HWSPIDriver* driver);
    virtual ~LIS331DLH();

    chibios_rt::EvtSource* getEventSource();
    msg_t configure(LIS331DLHConfig* config);

    /**
    * Return the accelaration in LSB for the given axis.
    *
    * @param axis can be one of the axis [AXIS_X | AXIS_Y | AXIS_Z]
    *
    * @return Measured accelaration in in LSB for the given axis
    */
    int16_t getAcceleration(const uint8_t axis);

    /**
    * Return the accelaration in milli g for the given axis
    *
    * @param axis can be one of the axis [AXIS_X | AXIS_Y | AXIS_Z]
    *
    * @return Measured accelaration in in milli g for the given axis
    */
    int16_t getAccelerationForce(const uint8_t axis);

    /**
     * Check the presence of the accelerometer by reading
     * the identifier register and comparing it to the standard
     * value
     *
     * @return [CHECK_OK | CHECK_FAIL]
     */
    uint8_t getCheck();

    /**
     * Do the build in self-test of the device by applying
     * an electrostatic force and measuring the amplitude.
     * The result is then checked against the standard values
     * of the manual
     * TODO Check if it works correct
     *
     * @param config A working config as template (if NULL, predefined config is taken)
     */
    void printSelfTest(LIS331DLHConfig* config);

  protected:
    virtual msg_t main();

  private:
    inline void updateSensorData();

  private:

    HWSPIDriver* driver;
    chibios_rt::EvtSource eventSource;
    int16_t accelerations[AXIS_Z - AXIS_X + 1];
    /**
    * Stores the configuration [FS_2G | FS_4G | FS_8G] of
    * the current setup
    */
    uint8_t currentFullScaleConfiguration;


  };

} /* amiro */

#endif /* LIS331DLH_HPP_ */
