#include <string.h>

#include <amiro/util/util.h>
#include <amiro/bus/spi/HWSPIDriver.hpp>
#include <amiro/accel/lis331dlh.hpp>
#include <chprintf.h>
#include <cmath>  // abs()
#include <amiro/Constants.h>
#include <global.hpp>

namespace amiro {

extern Global global;

LIS331DLH::
LIS331DLH(HWSPIDriver *driver) :
  driver(driver),
  currentFullScaleConfiguration(FS_2G) {

}

LIS331DLH::
~LIS331DLH() {

}

chibios_rt::EvtSource*
LIS331DLH::
getEventSource() {
  return &this->eventSource;
}

int16_t
LIS331DLH::
getAcceleration(const uint8_t axis) {

  return this->accelerations[axis];

}

int16_t
LIS331DLH::
getAccelerationForce(const uint8_t axis) {

  int16_t result;

  switch(this->currentFullScaleConfiguration) {
  default:
  case(FS_2G):
    // Should be this->accelerations[axis] * 0.9765625
    result = this->accelerations[axis];
    break;
  case(FS_4G):
      // Should be this->accelerations[axis] * 1.953125
      result =  this->accelerations[axis] << 1;
    break;
  case(FS_8G):
      // Should be this->accelerations[axis] * 3.90625
      result =  this->accelerations[axis] << 2;
    break;
  }

  return result;

}

msg_t
LIS331DLH::
main() {

  this->setName("Lis331dlh");

  while (!this->shouldTerminate()) {

    updateSensorData();

    this->eventSource.broadcastFlags(0);

    this->waitAnyEventTimeout(ALL_EVENTS, CAN::UPDATE_PERIOD);

  }
  return RDY_OK;
}

void
LIS331DLH::
updateSensorData() {

  const size_t buffer_size = offsetof(LIS331DLH::registers, out_z)
                             - offsetof(LIS331DLH::registers, status_reg)
                             + MEMBER_SIZE(LIS331DLH::registers, out_z)
                             + 1; /* addressing */
  uint8_t buffer[buffer_size];
  uint8_t sreg;

  // this might be three-wire so we need to send ones
  memset(buffer, 0xFFu, buffer_size);
  buffer[0] = offsetof(LIS331DLH::registers, status_reg) | LIS331DLH::SPI_MULT | LIS331DLH::SPI_READ;
  this->driver->exchange(buffer, buffer, buffer_size);

  // assemble data
  sreg = buffer[1];

  if (sreg & LIS331DLH::XDA)
    this->accelerations[LIS331DLH::AXIS_X] = int16_t((buffer[3] << 8) | buffer[2]) >> 4;

  if (sreg & LIS331DLH::YDA)
    this->accelerations[LIS331DLH::AXIS_Y] = int16_t((buffer[5] << 8) | buffer[4]) >> 4;

  if (sreg & LIS331DLH::ZDA)
    this->accelerations[LIS331DLH::AXIS_Z] = int16_t((buffer[7] << 8) | buffer[6]) >> 4;

}

msg_t
LIS331DLH::
configure(LIS331DLHConfig *config) {

  const size_t ctrl_reg_size = offsetof(LIS331DLH::registers, ctrl_reg5)
                               - offsetof(LIS331DLH::registers, ctrl_reg1)
                               + MEMBER_SIZE(LIS331DLH::registers, ctrl_reg5)
                               + 1; /* addressing */
  const size_t int_reg_size = offsetof(LIS331DLH::registers, int1_duration)
                              - offsetof(LIS331DLH::registers, int1_cfg)
                              + MEMBER_SIZE(LIS331DLH::registers, int1_duration) /* linear interrupt config */
                              + MEMBER_SIZE(LIS331DLH::registers, int1_src)
                              + MEMBER_SIZE(LIS331DLH::registers, int1_src) /* 2x size of int1_src */
                              + 1; /* addressing */
  const size_t buffer_size = ctrl_reg_size > int_reg_size ? ctrl_reg_size : int_reg_size;

  uint8_t buffer[buffer_size];
  InterruptConfig *int_cfg = &config->int1_cfg;
  uint8_t i;

  // write interrupt config first
  for (i = 0x00u; i < 2; i++, int_cfg++) {
    // this might be three-wire so we need to send ones
    memset(buffer, 0xFFu, buffer_size);
    if (!i)
      buffer[0] = offsetof(LIS331DLH::registers, int1_cfg);
    else
      buffer[0] = offsetof(LIS331DLH::registers, int2_cfg);
    buffer[5] = buffer[0] + offsetof(LIS331DLH::registers, int1_src) - offsetof(LIS331DLH::registers, int1_cfg);
    buffer[5] |= LIS331DLH::SPI_READ;
    buffer[6] = 0xFFu,
    buffer[0] |= LIS331DLH::SPI_MULT | LIS331DLH::SPI_WRITE;
    buffer[1] = int_cfg->config;
    buffer[2] = 0xFFu; /* skip source register */
    buffer[3] = int_cfg->ths;
    buffer[4] = int_cfg->duration;
    this->driver->write(&buffer[0], 5);
    // read intN_src register to clear IA
    this->driver->exchange(&buffer[5], &buffer[5], 2);
  }

  // write control config
  // this might be three-wire so we need to send ones
  memset(buffer, 0xFFu, buffer_size);
  buffer[0] = offsetof(LIS331DLH::registers, ctrl_reg1) | LIS331DLH::SPI_MULT | LIS331DLH::SPI_WRITE;
  buffer[1] = config->ctrl1;
  buffer[2] = config->ctrl2;
  buffer[3] = config->ctrl3;
  buffer[4] = config->ctrl4;
  buffer[5] = config->ctrl5;
  this->driver->write(buffer, 6);

  // reset hp filter
  buffer[0] = offsetof(LIS331DLH::registers, hp_filter_reset) | LIS331DLH::SPI_WRITE;
  this->driver->write(buffer, 1);

  // Store the full scale configuration for acceleration decoding
  this->currentFullScaleConfiguration = config->ctrl4 & LIS331DLH::FS_8G;

  return RDY_OK;

}

uint8_t
LIS331DLH::
getCheck() {

  const size_t buffer_size = 1 /* addressing */
                             + 1; /* who am i */
  uint8_t buffer[buffer_size];

  // Exchange the data with the LIS331DLH accelerometer
  // Specify the adress and the mode
  buffer[0] = offsetof(LIS331DLH::registers, who_am_i) | LIS331DLH::SPI_READ;
  this->driver->exchange(buffer, buffer, buffer_size);
  // Check
  if (buffer[1] == LIS331DLH::LIS331DLH_ID) {
    return LIS331DLH::CHECK_OK;
  } else {
    return LIS331DLH::CHECK_FAIL;
  }

}

void
LIS331DLH::
printSelfTest(LIS331DLHConfig* config) {

  // Running the build in test from LIS331DLH manual and compare the
  // measured values against table 3

  const uint8_t amountOfMeanValues = 10;
  const uint32_t sleepBetweenMeasurementMs = 500;
  int32_t accel;

  // Choose a defined config, if none is given
  LIS331DLH::LIS331DLHConfig test_accel_run_cfg;
  if (config == NULL) {
    LIS331DLH::LIS331DLHConfig accel_run_cfg_tmp = {
    /* ctrl1    */LIS331DLH::PM_ODR | LIS331DLH::DR_50HZ_37LP | LIS331DLH::ZEN
        | LIS331DLH::YEN | LIS331DLH::XEN,
    /* ctrl2    */0x00u,
    /* ctrl3    */LIS331DLH::INT_LOW | LIS331DLH::I1_CFG_DRY,
    /* ctrl4    */LIS331DLH::BDU_CONT | LIS331DLH::BLE_LE | LIS331DLH::FS_8G
        | LIS331DLH::SIM_4WI,
    /* ctrl5    */LIS331DLH::SLEEP_TO_WAKE_OFF,
    /* int1_cfg */
    {
    /* config   */LIS331DLH::AOI_OR_INT,
    /* ths      */0x00u,
    /* duration */0x00u, },
    /* int2_cfg */
    {
    /* config   */LIS331DLH::AOI_OR_INT,
    /* ths      */0x00u,
    /* duration */0x00u, }, };
    test_accel_run_cfg = accel_run_cfg_tmp;
  } else {
    // Save the given config
    test_accel_run_cfg = *config;
  }

  // 1. Get some standard values
  // Configure for the test
  test_accel_run_cfg.ctrl4 = LIS331DLH::BDU_CONT | LIS331DLH::BLE_LE
      | LIS331DLH::FS_2G | LIS331DLH::SIM_4WI;
  this->configure(&test_accel_run_cfg);
  BaseThread::sleep(MS2ST(sleepBetweenMeasurementMs));
  // Grep some values and build the mean value
  chprintf((BaseSequentialStream*) &global.sercanmux1, "\nACC: Get acc std values\n");
  int32_t stdAccValues[3] = { this->getAcceleration(LIS331DLH::AXIS_X), this
      ->getAcceleration(LIS331DLH::AXIS_Y), this->getAcceleration(
      LIS331DLH::AXIS_Z) };

  for (uint8_t n = 1; n < amountOfMeanValues; ++n) {
    BaseThread::sleep(MS2ST(sleepBetweenMeasurementMs));
    for (uint8_t i = LIS331DLH::AXIS_X; i <= LIS331DLH::AXIS_Z; i++) {
      accel = int32_t(this->getAcceleration(i));
      stdAccValues[i] = (stdAccValues[i] * n + accel) / (n + 1);
      chprintf((BaseSequentialStream*) &global.sercanmux1, "%c%d:%d ", accel < 0 ? '-' : '+',
                accel < 0 ? -accel : accel, stdAccValues[i]);
    }
    chprintf((BaseSequentialStream*) &global.sercanmux1, "\n");
  }

  // 2. Apply negative offset
  // Configure for the test
  test_accel_run_cfg.ctrl4 = LIS331DLH::BDU_CONT | LIS331DLH::BLE_LE
      | LIS331DLH::FS_2G | LIS331DLH::SIM_4WI | LIS331DLH::ST_ENABLE
      | LIS331DLH::STSIGN_NEG;
  this->configure(&test_accel_run_cfg);
  BaseThread::sleep(MS2ST(sleepBetweenMeasurementMs));
  // Grep some values and build the mean value
  chprintf((BaseSequentialStream*) &global.sercanmux1, "\nACC: Get acc neg values\n");
  int16_t negAccValues[3] = { this->getAcceleration(LIS331DLH::AXIS_X), this
      ->getAcceleration(LIS331DLH::AXIS_Y), this->getAcceleration(
      LIS331DLH::AXIS_Z) };

  for (uint8_t n = 1; n < amountOfMeanValues; ++n) {
    BaseThread::sleep(MS2ST(sleepBetweenMeasurementMs));
    for (uint8_t i = LIS331DLH::AXIS_X; i <= LIS331DLH::AXIS_Z; i++) {
      accel = int32_t(this->getAcceleration(i));
      negAccValues[i] = (negAccValues[i] * n + accel) / (n + 1);
      chprintf((BaseSequentialStream*) &global.sercanmux1, "%c%d:%d ", accel < 0 ? '-' : '+',
                accel < 0 ? -accel : accel, negAccValues[i]);
    }
    chprintf((BaseSequentialStream*) &global.sercanmux1, "\n");
  }

  // 2. Apply positive offset
  // Configure for the test
  test_accel_run_cfg.ctrl4 = LIS331DLH::BDU_CONT | LIS331DLH::BLE_LE
      | LIS331DLH::FS_2G | LIS331DLH::SIM_4WI | LIS331DLH::ST_ENABLE
      | LIS331DLH::STSIGN_POS;
  this->configure(&test_accel_run_cfg);
  BaseThread::sleep(MS2ST(sleepBetweenMeasurementMs));
  // Grep some values and build the mean value
  chprintf((BaseSequentialStream*) &global.sercanmux1, "\nACC: Get acc pos values\n");
  int16_t posAccValues[3] = { this->getAcceleration(LIS331DLH::AXIS_X), this
      ->getAcceleration(LIS331DLH::AXIS_Y), this->getAcceleration(
      LIS331DLH::AXIS_Z) };

  for (uint8_t n = 1; n < amountOfMeanValues; ++n) {
    BaseThread::sleep(MS2ST(sleepBetweenMeasurementMs));
    for (uint8_t i = LIS331DLH::AXIS_X; i <= LIS331DLH::AXIS_Z; i++) {
      accel = int32_t(this->getAcceleration(i));
      posAccValues[i] = (posAccValues[i] * n + accel) / (n + 1);
      chprintf((BaseSequentialStream*) &global.sercanmux1, "%c%d:%d ", accel < 0 ? '-' : '+',
                accel < 0 ? -accel : accel, posAccValues[i]);
    }
    chprintf((BaseSequentialStream*) &global.sercanmux1, "\n");
  }

  // Get the amplitude change and compare it to the standard values from LIS331DLH manual table 3
  int32_t amp;
  const int32_t ampXmax = 550, ampYmax = 550, ampZmax = 750;
  const int32_t ampXmin = 120, ampYmin = 120, ampZmin = 140;

  // TEST
  chprintf((BaseSequentialStream*) &global.sercanmux1, "\n\nACC: Testresult\n");
  amp = std::abs(
      stdAccValues[LIS331DLH::AXIS_X] - negAccValues[LIS331DLH::AXIS_X]);
  if (amp < ampXmin || amp > ampXmax)
    chprintf((BaseSequentialStream*) &global.sercanmux1, "ACC: Negative x-axis faulty\n");
  amp = std::abs(
      stdAccValues[LIS331DLH::AXIS_Y] - negAccValues[LIS331DLH::AXIS_Y]);
  if (amp < ampYmin || amp > ampYmax)
    chprintf((BaseSequentialStream*) &global.sercanmux1, "ACC: Negative y-axis faulty\n");
  amp = std::abs(
      stdAccValues[LIS331DLH::AXIS_Z] - negAccValues[LIS331DLH::AXIS_Z]);
  if (amp < ampZmin || amp > ampZmax)
    chprintf((BaseSequentialStream*) &global.sercanmux1, "ACC: Negative z-axis faulty\n");
  amp = std::abs(
      stdAccValues[LIS331DLH::AXIS_X] - posAccValues[LIS331DLH::AXIS_X]);
  if (amp < ampXmin || amp > ampXmax)
    chprintf((BaseSequentialStream*) &global.sercanmux1, "ACC: Positive x-axis faulty\n");
  amp = std::abs(
      stdAccValues[LIS331DLH::AXIS_Y] - posAccValues[LIS331DLH::AXIS_Y]);
  if (amp < ampYmin || amp > ampYmax)
    chprintf((BaseSequentialStream*) &global.sercanmux1, "ACC: Positive y-axis faulty\n");
  amp = std::abs(
      stdAccValues[LIS331DLH::AXIS_Z] - posAccValues[LIS331DLH::AXIS_Z]);
  if (amp < ampZmin || amp > ampZmax)
    chprintf((BaseSequentialStream*) &global.sercanmux1, "ACC: Positive z-axis faulty\n");

  // Write back the original config
  this->configure(config);
}

} /* amiro */
