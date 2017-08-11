#include <ch.hpp>
#include <hal.h>

#include <qei.h>
#include <chprintf.h>
#include <amiro/MotorControl.h>
#include <global.hpp>

using namespace chibios_rt;
using namespace amiro;
using namespace types;
using namespace constants;
using namespace constants::DiWheelDrive;

float MotorControl::wheelDiameterCorrectionFactor[2] = {1.0f, 1.0f};
float MotorControl::actualWheelBaseDistanceSI = wheelBaseDistanceSI;
extern Global global;

MotorControl::MotorControl(PWMDriver* pwm, MotorIncrements* mi, GPIO_TypeDef* port, int pad, fileSystemIo::FSIODiWheelDrive *memory)
    : BaseStaticThread<512>(),
      pwmDriver(pwm),
      motorIncrements(mi),
      powerEnablePort(port),
      powerEnablePad(pad),
      eventSource(),
      period(10),
      memory(memory) {

  this->pwmConfig.frequency = 7200000;
  this->pwmConfig.period = 360;
  this->pwmConfig.callback = NULL;
  this->pwmConfig.channels[0].mode = PWM_OUTPUT_ACTIVE_HIGH;
  this->pwmConfig.channels[0].callback = NULL;
  this->pwmConfig.channels[1].mode = PWM_OUTPUT_ACTIVE_HIGH;
  this->pwmConfig.channels[1].callback = NULL;
  this->pwmConfig.channels[2].mode = PWM_OUTPUT_ACTIVE_HIGH;
  this->pwmConfig.channels[2].callback = NULL;
  this->pwmConfig.channels[3].mode = PWM_OUTPUT_ACTIVE_HIGH;
  this->pwmConfig.channels[3].callback = NULL;
  this->pwmConfig.cr2 = 0;

  this->increment[0] = 0;
  this->increment[1] = 0;

  this->errorSum[0] = 0;
  this->errorSum[1] = 0;

  this->errorSumDiff = 0;

  // Init the velocities
  this->currentVelocity.x = 0;
  this->currentVelocity.y = 0;
  this->currentVelocity.z = 0;
  this->currentVelocity.w_x = 0;
  this->currentVelocity.w_y = 0;
  this->currentVelocity.w_z = 0;
  this->targetVelocity.x = 0;
  this->targetVelocity.w_z = 0;

  this->newTargetVelocities = false;
  this->lastVelocitiesV[0] = 0;
  this->lastVelocitiesV[1] = 0;
  this->lastVelocitiesV[2] = 0;
  this->lastVelocitiesV[3] = 0;
  this->lastVelocitiesV[4] = 0;
  this->lastVelocitiesV[5] = 0;

  this->newTargetVelocities = true;

  // calibration stuff;
  for (int i =0; i<3; i++){
    this->leftWValues[i] = i;
    this->rightWValues[i] = i;
  }

}

int MotorControl::getCurrentRPMLeft() {
  return this->actualSpeed[LEFT_WHEEL];
}

int MotorControl::getCurrentRPMRight() {
  return this->actualSpeed[RIGHT_WHEEL];
}

kinematic MotorControl::getCurrentVelocity() {
  return this->currentVelocity;
}

msg_t MotorControl::setWheelDiameterCorrectionFactor(float Ed /* = 1.0f */, bool_t storeEdToMemory /* = false */) {
  // cl (Eq. 17a)
  MotorControl::wheelDiameterCorrectionFactor[LEFT_WHEEL] = 2.0f / (Ed + 1.0f);
  // cl (Eq. 17a)
  MotorControl::wheelDiameterCorrectionFactor[RIGHT_WHEEL] = 2.0f / ((1.0f / Ed) + 1.0f);
  // Store Ed to memory
  if (storeEdToMemory)
    return memory->setEd(Ed);
  else
    return RDY_OK;
}

msg_t MotorControl::setActualWheelBaseDistance(float Eb /* = 1.0f */, bool_t storeEbToMemory /* = false */) {
  // bActual (Eq. 4)
  MotorControl::actualWheelBaseDistanceSI = wheelBaseDistanceSI * Eb;
  // Store Eb to memory
  if (storeEbToMemory)
    return memory->setEb(Eb);
  else
    return RDY_OK;
}

EvtSource* MotorControl::getEventSource() {
  return &this->eventSource;
}

void MotorControl::setTargetRPM(int32_t targetURpmLeft, int32_t targetURpmRight) {
  // Velocity in µm/s in x direction
  int32_t targetVelocityX = (wheelCircumferenceSI * (targetURpmLeft + targetURpmRight)) / secondsPerMinute / 2.0f;
  // Angular velocity around z in µrad/s
  int32_t targetVelocityWz = (wheelCircumferenceSI * (targetURpmRight - targetURpmLeft)) / secondsPerMinute / MotorControl::actualWheelBaseDistanceSI;
  chSysLock();
  this->targetVelocity.x = targetVelocityX;
  this->targetVelocity.w_z = targetVelocityWz;
  this->newTargetVelocities = true;
  chSysUnlock();
}

void MotorControl::setTargetSpeed(const kinematic &targetVelocity) {
  chSysLock();
  this->targetVelocity.x = targetVelocity.x;
  this->targetVelocity.w_z = targetVelocity.w_z;
  this->newTargetVelocities = true;
  chSysUnlock();
}

msg_t MotorControl::main(void) {
  systime_t time = System::getTime();
  this->setName("MotorControl");

  // load controller parameters from memory
  this->memory->getWheelFactor(&this->motorCalibrationFactor);
  this->memory->getpGain(&this->pGain);
  this->memory->getiGain(&this->iGain);
  this->memory->getdGain(&this->dGain);
  this->memory->getEb(&this->Eb);
  this->memory->getEd(&this->Ed);

  pwmStart(this->pwmDriver, &this->pwmConfig);

  palSetPad(this->powerEnablePort, this->powerEnablePad);

  while (!this->shouldTerminate()) {
    time += MS2ST(this->period);

    // Get the increments from the QEI
    MotorControl::updateIncrements(this->motorIncrements, this->increment, this->incrementDifference);

    // Get the actual speed from the gathered increments
    MotorControl::updateSpeed(this->incrementDifference, this->actualSpeed, this->period);

    // Calculate velocities
    this->calcVelocity();

    // updates past velocities for the derivate part of the controller
    updateDerivativeLastVelocities();

    controllerAndCalibrationLogic();

    // Write the calculated duty cycle to the pwm driver
    this->writePulseWidthModulation();

    chThdSleepUntil(time);

    delay ++;
    if (delay > 50){
      delay = 0;
    }

  }

  // Reset the PWM befor shutdown
  this->pwmPercentage[LEFT_WHEEL] = 0;
  this->pwmPercentage[RIGHT_WHEEL] = 0;
  this->writePulseWidthModulation();

  return true;
}

void MotorControl::controllerAndCalibrationLogic(){
  if (!isCalibrating){
    this->PIDController();
    startedZieglerCalibration = true;
    startedWheelCalibration = true;
  } else {
    if (motorCalibration){
      if (startedWheelCalibration){
        wheelCalibrationTime = System::getTime();
        startedWheelCalibration = false;
      }
      calibrate();
    } else {
      this->PIDController();
      this->calibrateZiegler();
      this->updatePastVelocities();

      if (startedZieglerCalibration){
        zieglerCalibrationTime = System::getTime();
        startedZieglerCalibration = false;
      }
    }
  }
}

void MotorControl::calibrate() {

  this->pwmPercentage[LEFT_WHEEL] = 3000;
  this->pwmPercentage[RIGHT_WHEEL] = 3000;

  this->rightWValues[0] = this->rightWValues[1];
  this->rightWValues[1] = this->rightWValues[2];
  this->rightWValues[2] = getRightWheelSpeed();

  this->leftWValues[0] = this->leftWValues[1];
  this->leftWValues[1] = this->leftWValues[2];
  this->leftWValues[2] = getLeftWheelSpeed();


  if (this->rightWValues[0] == this->rightWValues[1] &&
      this->rightWValues[1] == this->rightWValues[2] &&
      this->leftWValues[0] == this->leftWValues[1] &&
      this->leftWValues[1] == this->leftWValues[2] &&
      System::getTime() - this->wheelCalibrationTime > 1500) {
    this->motorCalibrationFactor = (float)this->rightWValues[0]/(float)this->leftWValues[0];

    chprintf((BaseSequentialStream*) &global.sercanmux1, "motorCalibrationFactor =   %f  \n" ,this->motorCalibrationFactor);
    chprintf((BaseSequentialStream*) &global.sercanmux1, "rw =   %i  \n" ,this->rightWValues[0]);
    chprintf((BaseSequentialStream*) &global.sercanmux1, "lw =   %i  \n" ,this->leftWValues[0]);

    this->pwmPercentage[LEFT_WHEEL] = 0;
    this->pwmPercentage[RIGHT_WHEEL] = 0;
    this->motorCalibration = false;
    this->memory->setwheelfactor(motorCalibrationFactor);
  }
}

void MotorControl::calibrateZiegler() {

    this->iGain =0;
    this->dGain = 0;
    int nsc  = this->getNumberofSignChanges();

    if (System::getTime() - this->zieglerCalibrationTime > 1000){
        this->zieglerCalibrationTime = System::getTime() ;
        this->ziegler2 =true;
    }

    if (ziegler && ziegler2){
        this->targetVelocity.x = 200000 * ((zieglerHelp%2 == 0) ? 1 : -1);
        this->pGain = 1000 + 100 * zieglerHelp;
        chprintf((BaseSequentialStream*) &global.sercanmux1, "pgain =  %i  \n" , this->pGain);
        zieglerHelp++;
        ziegler = false;
        ziegler2 = false;
    }

    if (!ziegler  && ziegler2){
        this->targetVelocity.x = 0;
        ziegler2= false;
        ziegler = true;
    }

    if (zieglerHelp > 20){
        this->isCalibrating = false;
        this->targetVelocity.x = 0;
        this->iGain = 0.08;
        this->pGain = 1000;
    }


    if ( nsc > 8){
        zieglerHelp2++;
        if (zieglerHelp2 > 20){
           this->zieglerPeriod  = numberOfLastVelocitiesV * this->period / nsc;
           chprintf((BaseSequentialStream*) &global.sercanmux1, "zieglerPeriod =   %f  \n" ,this->zieglerPeriod);

           this->targetVelocity.x = 0;
           this->pGain = (int) (this->pGain* 0.6);
           this->iGain = (float) ((1/(0.5*(this->zieglerPeriod/1000))/this->pGain));
           this->dGain = (float) ((1/(0.125*(this->zieglerPeriod/1000))/this->pGain));
           this->memory->setpGain(this->pGain);
           this->memory->setiGain(this->iGain);
           this->memory->setdGain(this->dGain);
           chprintf((BaseSequentialStream*) &global.sercanmux1, "pgain =   %i  \n" ,this->pGain);
           chprintf((BaseSequentialStream*) &global.sercanmux1, "igain =   %f  \n" ,this->iGain);
           chprintf((BaseSequentialStream*) &global.sercanmux1, "dgain =   %f  \n" ,this->dGain);

           this->motorCalibration = true;
           ziegler = true;
           ziegler2 = true;
           this->isCalibrating = false;
           zieglerHelp = 0;
           zieglerCalibrationTime = 0;
           zieglerHelp2 = 0;

           for (int i = 0; i< numberOfLastVelocitiesV ; i ++){
               lastVelocitiesVBig[i] = 0;
           }
        }
    }

}

void MotorControl::PIDController(){

    //pgain #####################################
    chSysLock();
    int diffv =this->targetVelocity.x - this->currentVelocity.x;
    int diffw = this->targetVelocity.w_z - this->currentVelocity.w_z;
    chSysUnlock();

    //igain ####################################
   this->accumulatedErrorV += diffv;
   this->accumulatedErrorW += diffw;

    if (this->accumulatedErrorV > this->antiWindupV)
      this->accumulatedErrorV = this->antiWindupV;
    else if (this->accumulatedErrorV < -this->antiWindupV)
      this->accumulatedErrorV = -this->antiWindupV;

    if (this->accumulatedErrorW > this->antiWindupW)
      this->accumulatedErrorW = this->antiWindupW;
    else if (this->accumulatedErrorW < -this->antiWindupW)
      this->accumulatedErrorW = -this->antiWindupW;

    diffv += (int) (this->accumulatedErrorV*this->iGain);
    diffw += (int) (this->accumulatedErrorW*this->iGain);

    //dgain ###################################
    int derivativeV;
    int derivativeW;
    int tmp1;
    int tmp2;

   tmp1 = static_cast<int32_t>((lastVelocitiesV[0]+lastVelocitiesV[1]+lastVelocitiesV[2])/3);
   tmp2 = static_cast<int32_t>((lastVelocitiesV[3]+lastVelocitiesV[4]+lastVelocitiesV[5])/3);
   derivativeV = static_cast<int32_t> ((tmp2-tmp1)/(int)(this->period));
   tmp1 = static_cast<int32_t>((lastVelocitiesW[0]+lastVelocitiesW[1]+lastVelocitiesW[2])/3);
   tmp2 = static_cast<int32_t>((lastVelocitiesW[3]+lastVelocitiesW[4]+lastVelocitiesW[5])/3);
   derivativeW = static_cast<int32_t> ((tmp2-tmp1)/(int)(this->period));


    diffv += (int) (dGain*derivativeV);
    diffw += (int) (dGain*derivativeW);

    setLeftWheelSpeed(diffv,diffw);
    setRightWheelSpeed(diffv, diffw);
}



void MotorControl::setLeftWheelSpeed(int diffv, int diffw){
    this->pwmPercentage[LEFT_WHEEL] = (int) (this->pGain*2*(diffv-0.5*diffw*wheelBaseDistanceSI*this->Eb)/(wheelDiameter*this->wheelDiameterCorrectionFactor[LEFT_WHEEL]));

}

void MotorControl::setRightWheelSpeed(int diffv, int diffw){
    this->pwmPercentage[RIGHT_WHEEL] = (int) (motorCalibrationFactor*this->pGain*2*(diffv+0.5*diffw*wheelBaseDistanceSI*this->Eb)/(wheelDiameter*this->wheelDiameterCorrectionFactor[RIGHT_WHEEL]));
}


// speed in um
int MotorControl::getRightWheelSpeed(){
    int omega = 2*M_PI*this->actualSpeed[RIGHT_WHEEL]/60; // syslock noetig ? todo
    return  omega * wheelDiameter*this->wheelDiameterCorrectionFactor[RIGHT_WHEEL];
}
// speed in um
int MotorControl::getLeftWheelSpeed(){
    int omega = 2*M_PI*this->actualSpeed[LEFT_WHEEL]/60;
    return  omega * wheelDiameter*this->wheelDiameterCorrectionFactor[LEFT_WHEEL];
}


void MotorControl::updatePastVelocities(){
    for (int i=0; i<numberOfLastVelocitiesV-1;i++){
        lastVelocitiesVBig[i] = lastVelocitiesVBig[i+1];
    }

    lastVelocitiesVBig[numberOfLastVelocitiesV-1] = this->currentVelocity.x;

}


void MotorControl::updateDerivativeLastVelocities(){
    for (int i=0; i<5;i++){
        lastVelocitiesV[i] = lastVelocitiesV[i+1];
        lastVelocitiesW[i] = lastVelocitiesW[i+1];
    }


    lastVelocitiesV[5] = this->currentVelocity.x;
    lastVelocitiesW[5] = this->currentVelocity.w_z;


}

int MotorControl::getNumberofSignChanges(){
    int nsc= 0;
    bool ispositive = true;
    bool tmpbool = true;
    if (lastVelocitiesVBig[0] < 0){
        ispositive =false;
        tmpbool =false;
    }
    for (int i=0; i<numberOfLastVelocitiesV-1; i++){
        if (lastVelocitiesVBig[i] < 0){
            ispositive= false;
        } else {
            ispositive = true;
        }
        if (ispositive != tmpbool){
            nsc++;
            tmpbool = ispositive;
        }

    }

    return nsc;
}

void MotorControl::printGains(){
    chprintf((BaseSequentialStream*)&global.sercanmux1, "motorCalibrationFactor %f\n", this->motorCalibrationFactor );
    chprintf((BaseSequentialStream*)&global.sercanmux1, "pGain %i\n", this->pGain );
    chprintf((BaseSequentialStream*)&global.sercanmux1, "iGain %f\n", this->iGain );
    chprintf((BaseSequentialStream*)&global.sercanmux1, "dGain %f\n", this->dGain );
}

void MotorControl::resetGains()
{
  // reset factors
  chSysLock();
  this->motorCalibrationFactor = 1.0f;
  this->pGain = 1000;
  this->iGain = 0.08f;
  this->dGain = 0.01f;
  chSysUnlock();

  // write reset factors to memory
  this->memory->setwheelfactor(this->motorCalibrationFactor);
  this->memory->setpGain(this->pGain);
  this->memory->setiGain(this->iGain);
  this->memory->setdGain(this->dGain);

  return;
}



void MotorControl::calcVelocity() {
  // Velocity in µm/s in x direction
  currentVelocity.x = (1.0f*wheelCircumference * (this->actualSpeed[LEFT_WHEEL] + this->actualSpeed[RIGHT_WHEEL])) / secondsPerMinute / 2.0f;
  // Angular velocity around z in µrad/s
  currentVelocity.w_z = (1.0f*wheelCircumference * (this->actualSpeed[RIGHT_WHEEL] - this->actualSpeed[LEFT_WHEEL])) / (1.0f*secondsPerMinute) / (wheelBaseDistanceSI*this->Eb);

}



void MotorControl::updateIncrements(MotorIncrements* motorIncrements, int32_t (&oldIncrement)[2], float (&incrementDifference)[2]) {
  int32_t currentIncrement[2];

  chSysLock();
  for (uint8_t idxWheel = 0; idxWheel < 2; idxWheel++) {
    currentIncrement[idxWheel] = motorIncrements->qeiGetPosition(idxWheel);
  }
  chSysUnlock();

  // Calculate the difference between the last and
  // actual increments and therefor the actual speed or distance
  for (uint8_t idxWheel = 0; idxWheel < 2; idxWheel++) {

    // Get the difference
    int32_t tmpIncrementDifference =  oldIncrement[idxWheel] - currentIncrement[idxWheel];

    // Handle overflow of increments
    int range = motorIncrements->getQeiConfigRange();
    if (tmpIncrementDifference > (range >> 1))
      tmpIncrementDifference -= motorIncrements->getQeiConfigRange();
    else if (tmpIncrementDifference < -(range >> 1))
      tmpIncrementDifference += motorIncrements->getQeiConfigRange();

    // Correct the difference
    incrementDifference[idxWheel] =  static_cast<float>(tmpIncrementDifference) * MotorControl::wheelDiameterCorrectionFactor[idxWheel];

    // Save the actual increments
    oldIncrement[idxWheel] = currentIncrement[idxWheel];
  }
}

void MotorControl::updateSpeed(const float (&incrementDifference)[2], int32_t (&actualSpeed)[2], const uint32_t period) {
  const int32_t updatesPerMinute = 60 * 1000 / period;

  for (uint8_t idxWheel = 0; idxWheel < 2; idxWheel++) {
    // Save the actual speed
    actualSpeed[idxWheel] = static_cast<int32_t>(static_cast<float>(updatesPerMinute * incrementDifference[idxWheel])) / incrementsPerRevolution;
  }
}

void MotorControl::updateDistance(const float (&incrementDifference)[2], float (&actualDistance)[2]) {

  for (uint8_t idxWheel = 0; idxWheel < 2; idxWheel++) {
    // Calc. the distance per wheel in meter
    actualDistance[idxWheel] = wheelCircumferenceSI * incrementDifference[idxWheel] / static_cast<float>(incrementsPerRevolution);
  }
}

void MotorControl::writePulseWidthModulation() {
  for (int idxWheel = 0; idxWheel < 2; idxWheel++) {
    int percentage = this->pwmPercentage[idxWheel];
    unsigned int widths[2];

    // 10000 is the max. duty cicle
    if (percentage > 10000) {
      percentage = 10000;
    } else if (percentage < -10000) {
      percentage = -10000;
    }

    if (percentage < 0) {
      widths[0] = 0;
      widths[1] = PWM_PERCENTAGE_TO_WIDTH(this->pwmDriver, -percentage);
    } else {
      widths[0] = PWM_PERCENTAGE_TO_WIDTH(this->pwmDriver, percentage);
      widths[1] = 0;
    }

    for (int idxPWM = 0; idxPWM < 2; idxPWM++)
      pwmEnableChannel(this->pwmDriver, (idxWheel * 2) + idxPWM, widths[idxPWM]);
  }
}
