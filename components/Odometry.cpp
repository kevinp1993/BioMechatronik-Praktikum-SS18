#include <ch.hpp>
#include <hal.h>

#include <qei.h>

#include <amiro/Odometry.h>

#include <math.h> // cos(), sin()
#include <Matrix.h> // Matrixoperations "Matrix::*"
#include <amiro/Constants.h> // Constants "constants::*"
#include <chprintf.h>
#include <global.hpp>

using namespace chibios_rt;
using namespace amiro;
using namespace constants::DiWheelDrive;

extern Global global;


Odometry::Odometry(MotorIncrements* mi, L3G4200D* gyroscope)
    : BaseStaticThread<512>(),
      motorIncrements(mi),
      gyro(gyroscope),
      eventSource(),
      period(50),
      incrementsPerRevolution(incrementsPerRevolution),
      updatesPerMinute(constants::secondsPerMinute * constants::millisecondsPerSecond / this->period),
      wheelCircumference(wheelCircumferenceSI),
      wheelBaseDistanceSI(wheelBaseDistanceSI) {


//  this-> = constants::secondsPerMinute * constants::millisecondsPerSecond / this->period;
//  this->wheelCircumference = constants::wheelCircumferenceSI;
//  this->wheelBaseDistanceSI = constants::wheelBaseDistanceSI;

  this->distance[LEFT_WHEEL] = 0.0f;
  this->distance[RIGHT_WHEEL] = 0.0f;
  this->increment[LEFT_WHEEL] = 0;
  this->increment[RIGHT_WHEEL] = 0;
  this->incrementDifference[LEFT_WHEEL] = 0.0f;
  this->incrementDifference[RIGHT_WHEEL] = 0.0f;
  this->distance[LEFT_WHEEL] = 0.0f;
  this->distance[RIGHT_WHEEL] = 0.0f;

  this->wheelError[LEFT_WHEEL] = wheelErrorSI[LEFT_WHEEL];
  this->wheelError[RIGHT_WHEEL] = wheelErrorSI[RIGHT_WHEEL];

  this->resetPosition(); // Init position

  this->resetError(); // Init error Cp

}

types::position Odometry::getPosition() {
  types::position robotPosition;
  const int32_t piScaled = int32_t(2 * M_PI * 1e6);
  chSysLock();
    // Conversion from standard unit to µ unit
    robotPosition.x = this->pX * 1e6;
    robotPosition.y = this->pY * 1e6;
    robotPosition.f_z = (int32_t(this->pPhi * 1e6) % piScaled) + ((this->pPhi < 0) ? piScaled : 0);  // Get only the postitve angel f_z in [0 .. 2 * pi]
  chSysUnlock();
//     chprintf((BaseSequentialStream*) &global.sercanmux1, "X:%d Y:%d Phi:%d", robotPosition.x,robotPosition.y, robotPosition.f_z);
//     chprintf((BaseSequentialStream*) &global.sercanmux1, "\r\n");
//     chprintf((BaseSequentialStream*) &global.sercanmux1, "X:%f Y:%f Phi:%f", this->pX,this->pY, this->pPhi);
//     chprintf((BaseSequentialStream*) &global.sercanmux1, "\r\n");
  return robotPosition;
}

void Odometry::setPosition(float pX, float pY, float pPhi) {
  chSysLock();
    this->pX = pX;
    this->pY = pY;
    this->pPhi = pPhi;
  chSysUnlock();
}

void Odometry::resetPosition() {
  this->setPosition(0.0f,0.0f,0.0f);
}

void Odometry::setError(float* Cp3x3) {
  chSysLock();
    Matrix::copy<float>(Cp3x3,3,3, &(this->Cp3x3[0]),3,3);
  chSysUnlock();
}

void Odometry::resetError() {
  Matrix::init<float>(&(this->Cp3x3[0]),3,3,0.0f);
}

EvtSource* Odometry::getEventSource() {
  return &this->eventSource;
}

msg_t Odometry::main(void) {
  systime_t time = System::getTime();
  this->setName("Odometry");

  while (!this->shouldTerminate()) {
    time += MS2ST(this->period);

    // Update the base distance, because it may have changed after a calibration
    this->updateWheelBaseDistance();

    // Get the actual speed
    this->updateDistance();

    // Calculate the odometry
    this->updateOdometry();


//     chprintf((BaseSequentialStream*) &global.sercanmux1, "X:%f Y:%f Phi:%f", this->pX,this->pY, this->pPhi);
//     chprintf((BaseSequentialStream*) &global.sercanmux1, "\r\n");
//     chprintf((BaseSequentialStream*) &global.sercanmux1, "distance_left:%f distance_right:%f", this->distance[0],this->distance[1]);
//     chprintf((BaseSequentialStream*) &global.sercanmux1, "\r\n");

    if (time >= System::getTime()) {
        chThdSleepUntil(time);
    } else {
        chprintf((BaseSequentialStream*) &global.sercanmux1, "WARNING Odometry: Unable to keep track\r\n");
    }
  }

  return true;
}

void Odometry::updateOdometry() {

  // Get the temporary position and error
  float Cp3x3[9];
  int32_t angular_ud;
  int32_t angularRate_udps;
  chSysLock();
    float pX = this->pX;
    float pY = this->pY;
    float pPhi = this->pPhi;
    Matrix::copy<float>(this->Cp3x3,3,3,Cp3x3,3,3);
    // TODO Get the gyro (or gyro rate) information and do something with it
    // angular_ud = gyro->getAngular_ud(L3G4200D::AXIS_Z);
    // angularRate_udps = gyro->getAngularRate_udps(L3G4200D::AXIS_Z);
  chSysUnlock();

  ////////////////
  // Temporary calculations
  ////////////////

  // TMP: Rotated angular
  float dPhi = (this->distance[RIGHT_WHEEL] - this->distance[LEFT_WHEEL]) / this->wheelBaseDistanceSI;
  // TODO Calculate the differential angel dPhi from either the angular (1. line) or angular rate (2.+3. line)
  // float dPhi = ((float(angular_ud * 1e-3) * M_PI ) * 1e-3) / 180.0f;
  // const float angular_md = float((angularRate_udps * this->period / constants::millisecondsPerSecond) * 1e-3);
  // float dPhi = ((angular_md * M_PI) * 1e-3) / 180.0f;

  // TMP: Moved distance
  float dDistance = (this->distance[RIGHT_WHEEL] + this->distance[LEFT_WHEEL]) / 2.0f;

  // TMP: Argument for the trigonometric functions
  float trigArg = pPhi + dPhi / 2.0f;

  // TMP: Trigonometric functions
  float cosArg = cos(trigArg);
  float sinArg = sin(trigArg);

  // TMP: Delta distance
  float dPX = dDistance * cosArg;
  float dPY = dDistance * sinArg;

  ////////////////
  // Position Update
  ////////////////

  // Update distance
  pX += dPX;
  pY += dPY;
  pPhi += dPhi;

  ////////////////
  // Temporary error calculations
  ////////////////

  // position propagation error (3x3 matrix)
  float Fp3x3[9]  = {1.0f, 0.0f, -dPY,
                     0.0f, 1.0f,  dPX,
                     0.0f, 0.0f, 1.0f};
  // steering error (2x2 matrix)
  float Cs2x2[4] = {abs(this->distance[RIGHT_WHEEL])*wheelError[RIGHT_WHEEL],0.0f,
                    0.0f, abs(this->distance[LEFT_WHEEL])*wheelError[LEFT_WHEEL]};
  // steering propagation error (3x2 matrix)
  float Fs3x2[6] = {(cosArg+dDistance*sinArg/this->wheelBaseDistanceSI)/2.0f, (sinArg+dDistance*cosArg/this->wheelBaseDistanceSI)/2.0f,
                 (sinArg-dDistance*cosArg/this->wheelBaseDistanceSI)/2.0f, (cosArg-dDistance*sinArg/this->wheelBaseDistanceSI)/2.0f,
                 -1.0f/this->wheelBaseDistanceSI                         , 1.0f/this->wheelBaseDistanceSI};

  ////////////////
  // Error calculations tmpCp = Fp*Cp*~Fp
  ////////////////
  // New position error
  float tmpCp3x3[9] = {0.0f};
  float tmpFpCp3x3[9] = {0.0f};
  // tmpFpCp = Fp*Cp
  Matrix::XdotY<float>(&(Fp3x3[0]),3,3,&(Cp3x3[0]),3,3,&(tmpFpCp3x3[0]),3,3);
  // tmpCp = tmpFpCp*~Fp
  Matrix::XdotYtrans<float>(&(tmpFpCp3x3[0]),3,3,&(Fp3x3[0]),3,3,&(tmpCp3x3[0]),3,3);

  ////////////////
  // Error calculations tmpCs = Fs*Cs*~Fs
  ////////////////
  // New steering error
  float tmpCs3x3[9] = {0.0f};
  float tmpFsCs3x2[6] = {0.0f};
  // tmpFsCs = Fs*Cs
  Matrix::XdotY<float>(&(Fs3x2[0]),3,2,&(Cs2x2[0]),2,2,&(tmpFsCs3x2[0]),3,2);
  // tmpCs = tmpFsCs*~Fs
  Matrix::XdotYtrans<float>(&(tmpFsCs3x2[0]),3,2,&(Fs3x2[0]),3,2,&(tmpCs3x3[0]),3,3);

  ////////////////
  // Error calculations Cp = Fp*Cp*~Fp + Fs*Cs*~Fs
  ////////////////
  Matrix::XplusY<float>(tmpCp3x3,3,3,tmpCs3x3,3,3,Cp3x3,3,3);

  ////////////////
  // Write back
  ////////////////

  // Write back
  this->setPosition(pX,pY,pPhi);
  chSysLock();
    Matrix::copy<float>(Cp3x3,3,3,this->Cp3x3,3,3);
  chSysUnlock();

}

void Odometry::updateWheelBaseDistance() {
  this->wheelBaseDistanceSI = MotorControl::actualWheelBaseDistanceSI;
}

void Odometry::updateDistance() {

  // Get the current increments of the QEI
  MotorControl::updateIncrements(this->motorIncrements, this->increment, this->incrementDifference);
//
//  chprintf((BaseSequentialStream*) &global.sercanmux1, "\ni_right = %d \t i_left = %d", this->increment[RIGHT_WHEEL], this->increment[LEFT_WHEEL]);
//  chprintf((BaseSequentialStream*) &global.sercanmux1, "\niDiff_right = %d \t iDiff_left = %d", this->incrementDifference[RIGHT_WHEEL], this->incrementDifference[LEFT_WHEEL]);

  // Get the driven distance for each wheel
  MotorControl::updateDistance(this->incrementDifference, this->distance);

//  chprintf((BaseSequentialStream*) &global.sercanmux1, "\nx_right = %f \t x_left = %f", this->distance[RIGHT_WHEEL], this->distance[LEFT_WHEEL]);
}
