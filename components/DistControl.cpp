#include <ch.hpp>
#include <hal.h>

#include <qei.h>

#include <amiro/DistControl.h>

#include <global.hpp>

using namespace chibios_rt;
using namespace amiro;
using namespace types;
using namespace constants;
using namespace constants::DiWheelDrive;

extern Global global;

DistControl::DistControl(MotorControl* mc, MotorIncrements* mi)
    : BaseStaticThread<256>(),
      motorControl(mc),
      motorIncrements(mi),
      period(10)
       {

  // initialize velocities
  this->maxVelocity.y = 0;
  this->maxVelocity.z = 0;
  this->maxVelocity.w_x = 0;
  this->maxVelocity.w_y = 0;
  this->targetVelocity.x = 0;
  this->targetVelocity.y = 0;
  this->targetVelocity.z = 0;
  this->targetVelocity.w_x = 0;
  this->targetVelocity.w_y = 0;
  this->targetVelocity.w_z = 0;
  this->minVelocity.y = 0;
  this->minVelocity.z = 0;
  this->minVelocity.w_x = 0;
  this->minVelocity.w_y = 0;
  this->minVelocity.w_z = 0;

  // set max and min velocities
  this->maxVelocity.x = 0.15 * 1e6; // 15 cm/s
  this->minVelocity.x = 0.02 * 1e6; // 2 cm/s
  this->maxVelocity.w_z = 2*maxVelocity.x / MotorControl::actualWheelBaseDistanceSI;
  this->minVelocity.w_z = 2*minVelocity.x / MotorControl::actualWheelBaseDistanceSI;
}

int DistControl::getCurrentTargetDist() {
  return (int)(targetDistance*1e6);
}

int DistControl::getCurrentTargetAngle() {
  return (int)(targetAngle*1e6);
}

void DistControl::setTargetPosition(int32_t distance, int32_t angle, uint16_t time) {
  chSysLock();
  targetDistance = distance; // um
  drivingForward = distance > 0;
  if (!drivingForward) {
    targetDistance *= -1;
  }
  targetAngle = angle; // urad
  turningLeft = angle > 0;
  if (!turningLeft) {
    targetAngle *= -1;
  }
  restTime = time * 1e3; // us
  controllerActive = true;
  chSysUnlock();
  fullDistance[LEFT_WHEEL] = 0;
  fullDistance[RIGHT_WHEEL] = 0;
  motorControl->updateIncrements(motorIncrements, increment, incrementDifference);
}

bool DistControl::isActive(void) {
  return controllerActive;
}

void DistControl::deactivateController(void) {
  chSysLock();
  controllerActive = false;
  targetDistance = 0;
  targetAngle = 0;
  restTime = 0;
  for (int idx=0; idx < 2; idx++) {
    increment[idx] = 0;
    incrementDifference[idx] = 0;
    actualDistance[idx] = 0;
    fullDistance[idx] = 0;
  }
  chSysUnlock();
}

msg_t DistControl::main(void) {
  systime_t time = System::getTime();
  systime_t printTime = time;
  this->setName("DistControl");

  DistControl::deactivateController();

  while (!this->shouldTerminate()) {
    time += MS2ST(this->period);

    if (controllerActive) {
      // get increment differences for each wheel
      motorControl->updateIncrements(motorIncrements, increment, incrementDifference); // ticks

      // calculate driven distance difference for each wheel
      motorControl->updateDistance(incrementDifference, actualDistance); // m

      // calculate full driven distance for each wheel
      for (int idxWheel = 0; idxWheel < 2; idxWheel++) {
        fullDistance[idxWheel] += (int32_t)(actualDistance[idxWheel]*1e6);
      }

      // calculate whole driven distance and angle
      realDistance = (fullDistance[LEFT_WHEEL] + fullDistance[RIGHT_WHEEL]) / 2.0; // um
      if (!drivingForward) {
        realDistance *= -1;
      }
      realAngle = (fullDistance[RIGHT_WHEEL] - fullDistance[LEFT_WHEEL]) / MotorControl::actualWheelBaseDistanceSI; // urad
      if (!turningLeft) {
        realAngle *= -1;
      }

      // calculate distance and angle to drive
      errorDistance = targetDistance - realDistance; // um
      if (errorDistance < 0) {
        errorDistance = 0;
      }
      errorAngle = targetAngle - realAngle; // urad
      if (errorAngle < 0) {
        errorAngle = 0;
      }

      // calculate velocities for motor control
      DistControl::calcVelocities();

      if (controllerActive && newVelocities) {
        // set target velocities
        this->motorControl->setTargetSpeed(targetVelocity);
        newVelocities = false;
      }

/*
      if (time-printTime > MS2ST(100)) {
        chprintf((BaseSequentialStream*) &global.sercanmux1, "dist = %i um, angle = %i urad, ed = %i um, ea = %i, v = %i um/s, w = %i urad/s\n", realDistance, realAngle, errorDistance, errorAngle, targetVelocity.x, targetVelocity.w_z);
        printTime = time;
      }
*/

      // reduce rest time (us)
      restTime -= period*1e3;
      if (restTime < 1) {
        restTime = 1;
      }

      // deactivate controller if necessary
      if (errorDistance == 0 && errorAngle == 0) {
        deactivateController();
      }

    }

    chThdSleepUntil(time);
  }

  return true;
}

void DistControl::calcVelocities(void) {
  // TODO calculate target velocities better

  // set intuitive velocities
  int32_t forwardSpeed = (int32_t) (errorDistance * 1e6 / (1.0f*restTime)); // um/s
  int32_t angleSpeed = (int32_t) (errorAngle * 1e6 / (1.0f*restTime)); // urad/s

  int32_t maxForward = maxVelocity.x;
  if (maxForward > errorDistance) {
    maxForward = errorDistance;
  }

  int32_t maxTurn = maxVelocity.w_z;
  if (maxTurn > errorAngle) {
    maxTurn = errorAngle;
  }

  // check max forward speed
  if (forwardSpeed > maxForward) {
    forwardSpeed = maxForward;
    angleSpeed = (int32_t) (forwardSpeed * ((1.0f*errorAngle) / (1.0f*errorDistance)));
  }

  // check max angle speed
  if (angleSpeed > maxTurn) {
    angleSpeed = maxTurn;
    forwardSpeed = (int32_t) (angleSpeed * ((1.0f*errorDistance) / (1.0f*errorAngle)));
  }

  // check for too small speeds
  if (errorDistance > 0 && forwardSpeed < minVelocity.x) {
    forwardSpeed = minVelocity.x;
  }
  if (errorAngle > 0 && errorDistance == 0 && angleSpeed < minVelocity.w_z) {
    angleSpeed = minVelocity.w_z;
  }

  // set velocity directions
  if (!drivingForward) {
    forwardSpeed *= -1;
  }
  if (!turningLeft) {
    angleSpeed *= -1;
  }

  // if nessecary set new target velocities
  if (forwardSpeed != targetVelocity.x || angleSpeed != targetVelocity.w_z) {
    newVelocities = true;
    targetVelocity.x = forwardSpeed;
    targetVelocity.w_z = angleSpeed;
  }
}
