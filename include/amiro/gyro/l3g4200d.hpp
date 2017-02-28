#ifndef L3G4200D_HPP_
#define L3G4200D_HPP_

#include <ch.hpp>

namespace amiro {

class HWSPIDriver;

class L3G4200D : public chibios_rt::BaseStaticThread<256> {

 public:
  enum {
    DR_100_HZ   = 0x00,
    DR_200_HZ   = 0x40,
    DR_400_HZ   = 0x80,
    DR_800_HZ   = 0xC0,
    DR_MASK     = 0xC0,
  };
  enum {
    BW_12_5  = 0x00,
    BW_20    = 0x00,
    BW_25    = 0x10,
    BW_30    = 0x00,
    BW_35    = 0x10,
    BW_50    = 0x20,
    BW_70    = 0x30,
    BW_110   = 0x30,
    BW_MASK  = 0x30,
  };
  enum {
    PD       = 0x08,
    ZEN      = 0x04,
    YEN      = 0x02,
    XEN      = 0x01,
    EN_MASK  = 0x0F,
  };
  enum {
    HPM_NORMAL_RST = 0x00,
    HPM_REFERENCE  = 0x10,
    HPM_NORMAL     = 0x20,
    HPM_AUTO_RST   = 0x30,
  };
  enum {
    HPCF_2    = 0x00,
    HPCF_4    = 0x01,
    HPCF_8    = 0x02,
    HPCF_16   = 0x03,
    HPCF_32   = 0x04,
    HPCF_64   = 0x05,
    HPCF_128  = 0x06,
    HPCF_256  = 0x07,
    HPCF_512  = 0x08,
    HPCF_1024 = 0x09,
  };
  enum {
    I1_INT1   = 0x80,
    I1_BOOT   = 0x40,
    H_IACTIVE = 0x20,
    PP_OD     = 0x10,
    I2_DRDY   = 0x08,
    I2_WTM    = 0x04,
    I2_ORUN   = 0x02,
    I2_EMPTY  = 0x01,
  };
  enum {
    BDU_CONT    = 0x00,
    BDU_SINGLE  = 0x80,
    BLE_MSB     = 0x40,
    BLE_LSB     = 0x00,
    FS_250_DPS  = 0x00,
    FS_500_DPS  = 0x10,
    FS_2000_DPS = 0x20,
    ST_SIGN_M   = 0x04,
    ST_SIGN_P   = 0x00,
    ST_EN       = 0x02,
    SIM_3W      = 0x01,
    SIM_4W      = 0x00,
    FS_MASK     = 0x20,
  };
  enum {
    BOOT          = 0x80,
    FIFO_EN       = 0x40,
    HP_EN         = 0x10,
    INT1_SEL_NOHP = 0x00,
    INT1_SEL_HP   = 0x04,
    INT1_SEL_LP   = 0x08,
    OUT_SEL_NOHP  = 0x00,
    OUT_SEL_HP    = 0x01,
    OUT_SEL_LP    = 0x02,
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
    FM_BYPASS        = 0x00,
    FM_FMMODE        = 0x20,
    FM_STREAM        = 0x40,
    FM_STREAM2FIFO   = 0x60,
    FM_BYPASS2STREAM = 0x80,
    WTM_MASK         = 0x1F,
  };
  enum {
    WTM      = 0x80,
    OVRN     = 0x40,
    EMPTY    = 0x20,
    FSS_MASK = 0x1F,
  };
  enum {
    ANDOR = 0x80,
    LIR   = 0x40,
    ZHIE  = 0x20,
    ZLIE  = 0x10,
    YHIE  = 0x08,
    YLIE  = 0x04,
    XHIE  = 0x02,
    XLIE  = 0x01,
  };
  enum {
    IA = 0x40,
    ZH = 0x20,
    ZL = 0x10,
    YH = 0x08,
    YL = 0x04,
    XH = 0x02,
    XL = 0x01,
  };
  enum {
    THS_L_MASK = 0x7F
  };
  enum {
    INT1_WAIT          = 0x80,
    INT1_DURATION_MASK = 0x7F,
  };

  /**
   * Return types of getCheck()
   */
  enum {
    CHECK_OK = 0x00u,
    CHECK_FAIL = 0x01u,
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
    AXIS_X = 0x00u,
    AXIS_Y = 0x01u,
    AXIS_Z = 0x02u,
  };

  //private:

  struct registers {
    uint8_t RESERVED_0x00_0x0E[0x0Fu];
    uint8_t WHO_AM_I;
    uint8_t RESERVED_0x10_0x1F[0x10u];
    uint8_t CTRL_REG1;
    uint8_t CTRL_REG2;
    uint8_t CTRL_REG3;
    uint8_t CTRL_REG4;
    uint8_t CTRL_REG5;
    uint8_t REFERENCE;

    uint8_t OUT_TEMP;

    uint8_t STATUS_REG;

    uint16_t OUT_X; /* LE */
    uint16_t OUT_Y; /* LE */
    uint16_t OUT_Z; /* LE */

    uint8_t FIFO_CTRL_REG;
    uint8_t FIFO_SRC_REG;

    uint8_t INT1_CFG;
    uint8_t INT1_SRC;
    uint8_t INT1_THS_XH;
    uint8_t INT1_THS_XL;
    uint8_t INT1_THS_YH;
    uint8_t INT1_THS_YL;
    uint8_t INT1_THS_ZH;
    uint8_t INT1_THS_ZL;
    uint8_t INT1_DURATION;
    uint8_t RESERVED_0x38_0x3F[0x08u];
  }__attribute__((packed));

  enum {
    L3G4200D_ID = 0xD3u
  };

  enum {
    SPI_MULT = 0x40u,
    SPI_READ = 0x80u,
    SPI_WRITE = 0x00u,
  };

 public:

  struct L3G4200DConfig {
    uint8_t         ctrl1;
    uint8_t         ctrl2;
    uint8_t         ctrl3;
    uint8_t         ctrl4;
    uint8_t         ctrl5;
  };

  L3G4200D(HWSPIDriver* driver);
  virtual ~L3G4200D();

  chibios_rt::EvtSource* getEventSource();
  msg_t configure(const L3G4200DConfig* config);
  int16_t angularRate[AXIS_Z - AXIS_X + 1];
  int32_t angular[AXIS_Z - AXIS_X + 1];

  int16_t getAngularRate(const uint8_t axis);
  
  int32_t getAngularRate_udps(const uint8_t axis);
  int32_t getAngular(const uint8_t axis);
  int32_t getAngular_ud(const uint8_t axis);
  void angularReset();


  /**
   * Check the presence of the accelerometer by reading
   * the identifier register and comparing it to the standard
   * value
   */
  uint8_t getCheck();

 protected:
  virtual msg_t main();

 private:
  inline void updateSensorData();
  inline void calcAngular();

 private:

  HWSPIDriver* driver;
  chibios_rt::EvtSource eventSource;
  uint32_t integrationTic;
  uint32_t udpsPerTic; // Resolution: Micro-degree-per-second per digit
  uint32_t period_us;
  uint32_t period_ms;
  systime_t period_st;

};

} /* amiro */

#endif /* L3G4200D_HPP_ */
