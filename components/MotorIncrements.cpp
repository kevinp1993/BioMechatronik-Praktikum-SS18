#include <ch.hpp>
#include <hal.h>

#include <qei.h>

#include <amiro/MotorIncrements.h>

using namespace chibios_rt;
using namespace amiro;

MotorIncrements::MotorIncrements(QEIDriver* qei0, QEIDriver* qei1) {
  this->qeiDriver[0] = qei0;
  this->qeiDriver[1] = qei1;

  this->qeiConfig.mode = QEI_COUNT_BOTH;
  this->qeiConfig.channels[0].mode = QEI_INPUT_INVERTED;
  this->qeiConfig.channels[1].mode = QEI_INPUT_NONINVERTED;
  this->qeiConfig.range = 0x10000;
}

void MotorIncrements::start() {
  qeiStart(this->qeiDriver[0], &this->qeiConfig);
  qeiStart(this->qeiDriver[1], &this->qeiConfig);

  qeiEnable(this->qeiDriver[0]);
  qeiEnable(this->qeiDriver[1]);
}

int MotorIncrements::qeiGetPosition(int idxQei) {
  return qeiGetPositionI(this->qeiDriver[idxQei]);
}

int MotorIncrements::getQeiConfigRange() {
  return this->qeiConfig.range;
}