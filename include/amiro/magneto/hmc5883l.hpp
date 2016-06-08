#ifndef HMC5883L_H_
#define HMC5883L_H_

#include <amiro/bus/i2c/I2CParams.hpp>

namespace amiro {

class I2CDriver;

/**
 * HMC5883L Magnetometer
 * \todo Interrupt Support, self-test
 */
class HMC5883L : public chibios_rt::BaseStaticThread<256> {

 public:

  struct registers {
    uint8_t ctrlA;
    uint8_t ctrlB;
    uint8_t mode;
    uint8_t xMsb;
    uint8_t xLsb;
    uint8_t zMsb;
    uint8_t zLsb;
    uint8_t yMsb;
    uint8_t yLsb;
    uint8_t status;
    uint8_t idA;
    uint8_t idB;
    uint8_t idC;
  }__attribute__((packed));

  struct HMC5883LConfig {
    uint8_t ctrlA;
    uint8_t ctrlB;
    uint8_t mode;
  };

  enum {
    SLA = 0x1E
  };

  enum {
    MA_AVG1 = 0x00,
    MA_AVG2 = 0x20,
    MA_AVG4 = 0x40,
    MA_AVG8 = 0x60,
  };

  enum {
    DO_0_5_HZ = 0x00,
    DO_1_HZ = 0x04,
    DO_2_HZ = 0x08,
    DO_5_HZ = 0x0C,
    DO_10_HZ = 0x10,
    DO_20_HZ = 0x14,
    DO_50_HZ = 0x18,
  };

  enum {
    MS_NORMAL = 0x00,
    MS_BIASPOS = 0x01,
    MS_BIASNEG = 0x02,
  };

  enum {
    GN_0_GA = 0x00,
    GN_1_GA = 0x20,
    GN_2_GA = 0x40,
    GN_3_GA = 0x60,
    GN_4_GA = 0x80,
    GN_5_GA = 0xA0,
    GN_6_GA = 0xC0,
    GN_7_GA = 0xE0,
  };

  enum {
    MD_CONTCV = 0x00,
    MD_SINGCV = 0x01,
    MD_IDLE = 0x02,
    MD_SLEEP = 0x03,
  };

  enum {
    HS_DISABLE = 0x00,
    HS_ENABLE = 0x80,
  };

  enum {
    SR_REN = 0x04,
    SR_LOCK = 0x02,
    SR_RDY = 0x01,
  };

  enum {
    ID_IRA = 'H',
    ID_IRB = '4',
    ID_IRC = '3',
  };

  enum {
    MAGNETO_AVG = 10
  };

  /**
   * Return types of getCheck()
   */
  enum {
    CHECK_OK = 0x00u,
    CHECK_FAIL = 0x01u,
  };

 public:
  HMC5883L(I2CDriver *driver, const HMC5883LConfig *config);
  virtual ~HMC5883L();

  msg_t configure(HMC5883LConfig* config);

  chibios_rt::EvtSource* getEventSource();

  /**
   * Check the presence of the magnetometer by reading
   * the identifier register and comparing it to the standard
   * value
   */
  uint8_t getCheck();

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
    AXIS_Y = 0x02u,
    AXIS_Z = 0x01u,
  };

  /**
   * Return the magnetization in LSB for the given axis.
   *
   * @param axis can be one of the axis [AXIS_X | AXIS_Y | AXIS_Z]
   *
   * @return Measured magnetization in in LSB for the given axis
   */
  int16_t getMagnetization(const uint8_t axis);

  /**
   * Return the magnetization in µGauss for the given axis.
   *
   * @param axis can be one of the axis [AXIS_X | AXIS_Y | AXIS_Z]
   *
   * @return Measured magnetization in µGauss for the given axis
   */
  int32_t getMagnetizationGauss(const uint8_t axis);

 protected:
  virtual msg_t main(void);

 private:

  /**
   * Update the sensor values by reading accessing
   * the sensor over I2C
   */
  inline void updateSensorData();

  /**
   * Writes the config to the HMC5883L
   *
   * @return I2C write return message
   */
  msg_t writeConf();

  /**
   * Stores the three orientations x,z,y in LSB
   */
  int16_t data[3];

  I2CDriver *driver;
  const HMC5883LConfig *config;
  chibios_rt::EvtSource eventSource;
  I2CTxParams txParams;
};

}

#endif /* HMC5883L_H_ */
