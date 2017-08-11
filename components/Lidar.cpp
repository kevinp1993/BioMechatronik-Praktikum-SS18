#include <amiro/Lidar.h>

#include <global.hpp>

using namespace chibios_rt;
using namespace amiro;

extern Global global;

uint8_t Lidar::scannedData[NUMBER_OF_CHARACTERS + 1] = {};

Lidar::Lidar(const uint8_t boardId, Lidar::SETUP setup)
    : BaseStaticThread<256>(),
      boardId(boardId),
      setup(setup) {

}

Lidar::
~Lidar() {

  // Power down the LIDAR
  // TODO Is it correct anyway
  switch (boardId) {
    case(CAN::LIGHT_RING_ID):
      palWritePad(GPIOB, GPIOB_LASER_EN, PAL_LOW);
      break;
    default:
      break;
  }
  this->isReady = false;
};

void Lidar::flushSD2InputQueue() {
  chSysLock();
  chIQResetI(&SD2.iqueue);
  chSysUnlock();
}

msg_t Lidar::main(void) {


  switch (this->boardId) {
    case(CAN::LIGHT_RING_ID): {

      // Power up the LIDAR
      palWritePad(GPIOB, GPIOB_LASER_EN, PAL_HIGH);
      BaseThread::sleep(MS2ST(5000));

      // Setup the driver and lidar, if we want to communicate with it
      if (setup != SETUP::POWER_ONLY) {
        // Flush the queue because there is a "0" in it
        flushSD2InputQueue();

        // Configure LIDAR serial interface speed
//       chprintf((BaseSequentialStream*) &global.sercanmux1, "Speed switch to " STR(SD_SPEED) "\n");
        chprintf((BaseSequentialStream*) &SD2, "SS" STR(SD_SPEED_PREFIX) STR(SD_SPEED) LF);

        // Check if the switch went well, otherwise terminate the thread
        if (checkDataString("SS" STR(SD_SPEED_PREFIX) STR(SD_SPEED) "\n00P\n\n")) {
          chprintf((BaseSequentialStream*) &global.sercanmux1, "Lidar speed switch OK\n");
          // Configure serial interface of STM32
          sdStop(&SD2);
          SerialConfig sdLidarconf = { SD_SPEED, 0, 0, 0 };
          sdStart(&SD2, &sdLidarconf);
        } else {
          chprintf((BaseSequentialStream*) &global.sercanmux1, "Lidar speed switch NOT OK: Terminating Lidar \n");
          palWritePad(GPIOB, GPIOB_LASER_EN, PAL_LOW);
          return -1;
        }
      }
      break;
    }
    default:
      break;
  }

  evtInit(&this->evtimer, UPDATE_LIDAR_PERIOD_MSEC);

  EvtSource *eventTimerEvtSource = reinterpret_cast<EvtSource *>(&this->evtimer.et_es);

  EvtListener eventTimerEvtListener;

  // TODO Does PERIODIC_TIMER_ID has something to do with ControllerAreNetwork.h ?
  eventTimerEvtSource->registerOne(&eventTimerEvtListener, CAN::PERIODIC_TIMER_ID);

  evtStart(&this->evtimer);

  this->setName("Lidar");

  while (!this->shouldTerminate()) {

    eventmask_t eventMask = this->waitOneEvent(ALL_EVENTS);
    switch (eventMask) {
      case EVENT_MASK(CAN::PERIODIC_TIMER_ID):
//         printDetails();
        if (setup != SETUP::POWER_ONLY) {
          updateSensorVal();
        }
        break;
    }
  }

  evtStop(&this->evtimer);
  eventTimerEvtSource->unregister(&eventTimerEvtListener);

  return RDY_OK;
}

bool_t Lidar::getScan(uint16_t (&scannedData)[NUMBER_OF_STEPS]) {
  if (this->isReady && !this->isFail) {
    chSysLock();
      memcpy(&scannedData, &(Lidar::scannedData[0]), NUMBER_OF_CHARACTERS);
      this->isReady = false;
    chSysUnlock();
    return true;
  } else {
    return false;
  }
}

uint16_t Lidar::getNumberOfValues() {
  return NUMBER_OF_STEPS;
}

uint16_t Lidar::twoCharacterEncoding(uint8_t &char1, uint8_t &char2) {
  return uint16_t((((char1 - 0x30) & 0b00111111) << 6) | ((char2 - 0x30) & 0b00111111));
}

bool_t Lidar::getData(uint8_t &data, uint32_t timeoutMs) {

//     data = sdGet(&SD2);
    msg_t dataTmp = chIQGetTimeout(&SD2.iqueue, MS2ST(timeoutMs));
    if  (dataTmp == Q_TIMEOUT || dataTmp == Q_RESET) {
      return false;
    } else {
      data = uint8_t(dataTmp);
      ++this->dataCounter;
      return true;
    }
}

msg_t Lidar::updateSensorVal() {

  chSysLock();
    // Set this flag so that the everyone knows, that scannedData will be filed right now
    this->isReady = false;
    // Reset the FAIL flag
    this->isFail = false;
  chSysUnlock();

  while (!this->isReady) {
    switch(step) {
      case SEND_SCAN_CMD:
        flushSD2InputQueue();
//                     Read (725-(44-1))=682 values with two-character encoding what makes 1364 datapoints
//                     1456 in total:
//                     16 (Cmd-Echo) + ? (Remaining Scans) + 1 (LF) + ? (String Characters) + 1 (LF) + 2 (Status) + 1 (Sum) + 1 (LF) + ? (Timestamp) + 1 (LF) + 1364 (Data for a certain setup) + 1 (Sum) + 1 (LF) + 1 (LF)
        chprintf((BaseSequentialStream*) &SD2,DATA_ACQ_CODE STR(STARTING_STEP_PREFIX) STR(STARTING_STEP) STR(END_STEP_PREFIX) STR(END_STEP) STR(CLUSTER_COUNT) STR(SCAN_INTERVALL) STR(NUMBER_OF_INTERVALL) LF);
        step = DATA_VALID_CHECK;
        this->dataCounter = 0;
        break;
      case DATA_VALID_CHECK:
        if(getData(this->newInput, 200)) {
          // TODO check for last data, if scan has a few errors
          // HACK The timeout fix this issue at least, because after a while there
          //      wont be any transmitts
          if (this->newInput == validScanTag[this->checkStatusIdx]) {
            if (++this->checkStatusIdx == 5) {
              // scan_data is without error: Found the sting "\n99b\n"
              step = DATA_START_CHECK;
              this->checkStatusIdx = 0;
            }
          } else {
            this->checkStatusIdx = 0;
          }
        } else {
          step = FAIL;
        }
        break;
      case DATA_START_CHECK:
        // Check for the next linefeed to start the record
        if(getData(this->newInput, 200)) {
          if (this->newInput == 10) {
            step = DATA_RECORD;
            this->dataIdx = 0;
          }
        } else {
          step = FAIL;
        }
        break;
      case DATA_RECORD:
        if(getData(this->newInput, 200)) {
          if (this->lastInput == 10 && this->newInput == 10) {
            // end of data
            this->lastInput = 0x0;  // Just to delete this->lastInput value
            step = DATA_DECODE;
          } else if  (this->newInput != 10) {
            Lidar::scannedData[this->dataIdx++] = this->newInput;
            this->lastInput = this->newInput;
          } else {  // this->lastInput != 10 && this->newInput == 10
            // This is the case, where we know that the last character was the checksum.
            // Therefore, we need to decrement the counter
            --this->dataIdx;
            this->lastInput = this->newInput;
          }
          //chprintf((BaseSequentialStream*) &global.sercanmux1, "%d ", this->newInput);
        } else {
          step = FAIL;
        }
        break;
      case DATA_DECODE:
        // Decode the recorded data
        for (uint16_t idx=0; idx < NUMBER_OF_CHARACTERS; idx += 2) {
          //TODO Check if +1 is right
          *((uint16_t*) &(Lidar::scannedData[idx])) = twoCharacterEncoding(Lidar::scannedData[idx],Lidar::scannedData[idx+1]);
        }
        // To print out the data in this thread, choose "step = DATA_SHOW"
        step = FINISH;
        break;
      case DATA_SHOW:
        // Show the decoded data
        chprintf((BaseSequentialStream*) &global.sercanmux1, "\n%d", this->dataCounter);
        for (uint32_t idx=0; idx < this->dataIdx; idx+=2) {
          chprintf((BaseSequentialStream*) &global.sercanmux1, "\n%d", *((uint16_t*) &(Lidar::scannedData[idx])));
        }
        step = FINISH;
        break;
      case FAIL:
        // Set the FAIL flag
        chSysLock();
          // Set this flag so that the everyone knows, that scannedData holds wrong information
          this->isFail = true;
        chSysUnlock();
        break;
      case FINISH:
        // Clean up
        this->step = SEND_SCAN_CMD;
        chSysLock();
          // Set this flag so that the everyone knows, that scannedData holds the actual data
          this->isReady = true;
        chSysUnlock();
        break;
      default:
        break;
    }
  }

  return RDY_OK;
}

void Lidar::printData() {
  uint8_t lastInput = 0xFF, newInput = 0xFF;
  while (true) {
    if(getData(newInput, 200)) {
      if (lastInput == 10 && newInput == 10) {
        return;
      } else {
        chprintf((BaseSequentialStream*) &global.sercanmux1, "%c", newInput);
      }
      lastInput = newInput;
    } else {
      chprintf((BaseSequentialStream*) &global.sercanmux1, "TIMEOUT\n", newInput);
      return;
    }
  }
}

bool_t Lidar::checkDataString(const char compareString[]) {
  uint8_t lastInput = 0xFF, newInput = 0xFF, recIdx = 0;
  bool_t dataOk = true, processFlag = true;
  while (processFlag) {
    if (getData(newInput, 200)) {
      // End of data received, quit processing
      if (lastInput == 10 && newInput == 10) {
        processFlag = false;
      }
      // Compare data if the characters before have been ok
      // Otherwise do not compare anymore, because of possible
      // segmentation fault
      if (dataOk) {
        if (newInput != compareString[recIdx++]) {
          dataOk = false;
        }
      }
      // Save the received character
      lastInput = newInput;
    } else {
      processFlag = false;
      dataOk = false;
    }
  }
  return dataOk;
}

void Lidar::printDetails() {

  chprintf((BaseSequentialStream*) &global.sercanmux1, "Print sensor details:\n");

  // Tell the sensor to transmit its details
  chprintf((BaseSequentialStream*) &SD2, "VV\n");

  // Print the transmitted data
  printData();
}

void Lidar::printSpecification() {

  chprintf((BaseSequentialStream*) &global.sercanmux1, "Print sensor specification:\n");

  // Tell the sensor to transmit its specifications
  chprintf((BaseSequentialStream*) &SD2, "PP\n");

  // Print the transmitted data
  printData();
}

void Lidar::printInformation() {
  chprintf((BaseSequentialStream*) &global.sercanmux1, "Print sensor information:\n");

  // Tell the sensor to transmit its information
  chprintf((BaseSequentialStream*) &SD2, "II\n");

  // Print the transmitted data
  printData();
}

bool_t Lidar::getIsReady() {
  return this->isReady;
}
