#define BL_CALLBACK_TABLE_ADDR  (0x08000000 + 0x01C0)
#define BL_MAGIC_NUMBER         ((uint32_t)0xFF669900u)

#define SHUTDOWN_NONE             0
#define SHUTDOWN_TRANSPORTATION   1
#define SHUTDOWN_DEEPSLEEP        2
#define SHUTDOWN_HIBERNATE        3
#define SHUTDOWN_RESTART          4
#define SHUTDOWN_HANDLE_REQUEST   5

#include <ch.hpp>

#include <amiro/util/util.h>
#include <global.hpp>
#include <exti.hpp>

#include <chprintf.h>
#include <shell.h>

using namespace chibios_rt;

Global global;

struct blVersion_t {
  const uint8_t identifier;
  const uint8_t major;
  const uint8_t minor;
  const uint8_t patch;
} __attribute__((packed));

void systemShutdown() {
  types::kinematic k;
  uint8_t i;

//  // make sure we assert SYS_PD_N to delay shutdown until we're done.
//  boardRequestShutdown();

    // stop the user thread
  global.userThread.requestTerminate();
  global.userThread.wait();

  k.x = 0x00u;
  k.w_z = 0x00u;

  // stop wheels
  global.robot.setTargetSpeed(k);
  global.robot.terminate();

  for (i = 0x00; i < global.vcnl4020.size(); i++) {
    global.vcnl4020[i].requestTerminate();
    global.vcnl4020[i].wait();
  }

  global.ina219.requestTerminate();
  global.ina219.wait();
  global.hmc5883l.requestTerminate();
  global.hmc5883l.wait();
  global.l3g4200d.requestTerminate();
  global.l3g4200d.wait();

  global.motorcontrol.requestTerminate();
  global.motorcontrol.wait();
  global.odometry.requestTerminate();
  global.odometry.wait();

  // stop I²C
  for (i = 0; i < global.V_I2C2.size(); ++i)
    global.V_I2C2[i].stop();
  global.HW_I2C2.stop();

  global.lis331dlh.requestTerminate();
  global.lis331dlh.wait();

  global.lis331dlh.configure(&global.accel_sleep_config);
//  global.lis331dlh.start(NORMALPRIO +4);

//  boardWriteIoPower(0);
//  boardStandby();

  return;
}


//void (*shellcmd_t)(BaseSequentialStream *chp, int argc, char *argv[]);

void shellRequestShutdown(BaseSequentialStream *chp, int argc, char *argv[]) {

  chprintf(chp, "shellRequestShutdown\n");

  /* if nor argument was given, print some help text */
  if (argc == 0 || strcmp(argv[0], "help") == 0) {
    chprintf(chp, "\tUSAGE:\n");
    chprintf(chp, "> shutdown <type>\n");
    chprintf(chp, "\n");
    chprintf(chp, "\ttype\n");
    chprintf(chp, "The type of shutdown to perform.\n");
    chprintf(chp, "Choose one of the following types:\n");
    chprintf(chp, "  transportation - Ultra low-power mode with all wakeups disabled.\n");
    chprintf(chp, "                   The robot can not be charged.\n");
    chprintf(chp, "  deepsleep      - Ultra low-power mode with several wakeups enabled.\n");
    chprintf(chp, "                   The robot can only be charged via the power plug.\n");
    chprintf(chp, "  hibernate      - Medium low-power mode, but with full charging capabilities.\n");
    chprintf(chp, "  restart        - Performs a system restart.\n");
    chprintf(chp, "Alternatively, you can use the shortcuts 't', 'd', 'h', and 'r' respectively.");
    chprintf(chp, "\n");
    return;
  }

  if (strcmp(argv[0],"transportation") == 0 || strcmp(argv[0],"t") == 0) {
    shutdown_now = SHUTDOWN_TRANSPORTATION;
    chprintf(chp, "shutdown to transportation mode initialized\n");
  } else if (strcmp(argv[0],"deepsleep") == 0 || strcmp(argv[0],"d") == 0) {
    shutdown_now = SHUTDOWN_DEEPSLEEP;
    chprintf(chp, "shutdown to deepsleep mode initialized\n");
  } else if (strcmp(argv[0],"hibernate") == 0 || strcmp(argv[0],"h") == 0) {
    shutdown_now = SHUTDOWN_HIBERNATE;
    chprintf(chp, "shutdown to hibernate mode initialized\n");
  } else if (strcmp(argv[0],"restart") == 0 || strcmp(argv[0],"r") == 0) {
    chprintf(chp, "restart initialized\n");
    shutdown_now = SHUTDOWN_RESTART;
  } else {
    chprintf(chp, "ERROR: unknown argument!\n");
    shutdown_now = SHUTDOWN_NONE;
  }

  return;
}

void shellRequestWakeup(BaseSequentialStream *chp, int argc, char *argv[]) {
  int i;
  chprintf(chp, "shellRequestWakeup\n");

  for (i = 0x00u; i < argc; i++)
    chprintf(chp, "%s\n", argv[i]);

  boardWakeup();
}

void shellRequestGetMemoryData(BaseSequentialStream *chp, int argc, char *argv[]) {
  enum Type {HEX, U8, U16, U32, S8, S16, S32};

  chprintf(chp, "shellRequestReadData\n");

  if (argc < 2 || strcmp(argv[0],"help") == 0)
  {
    chprintf(chp, "Usage: %s\n","get_memory_data <type> <start> [<count>]");
    chprintf(chp, "\n");
    chprintf(chp, "\ttype\n");
    chprintf(chp, "The data type as which to interpret the data.\n");
    chprintf(chp, "Choose one of the following types:\n");
    chprintf(chp, "  hex - one byte as hexadecimal value\n");
    chprintf(chp, "  u8  - unsigned integer (8 bit)\n");
    chprintf(chp, "  u16 - unsigned integer (16 bit)\n");
    chprintf(chp, "  u32 - unsigned integer (32 bit)\n");
    chprintf(chp, "  s8  - signed integer (8 bit)\n");
    chprintf(chp, "  s16 - signed integer (16 bit)\n");
    chprintf(chp, "  s32 - signed integer (32 bit)\n");
    chprintf(chp, "\tstart\n");
    chprintf(chp, "The first byte to read from the memory.\n");
    chprintf(chp, "\tcount [default = 1]\n");
    chprintf(chp, "The number of elements to read.\n");
    chprintf(chp, "\n");
    chprintf(chp, "\tNOTE\n");
    chprintf(chp, "Type conversions of this function might fail.\n");
    chprintf(chp, "If so, use type=hex and convert by hand.\n");
    chprintf(chp, "\n");
    return;
  }

  uint8_t type_size = 0;
  Type type = HEX;
  if (strcmp(argv[0],"hex") == 0) {
    type_size = sizeof(unsigned char);
    type = HEX;
  } else if(strcmp(argv[0],"u8") == 0) {
    type_size = sizeof(uint8_t);
    type = U8;
  } else if(strcmp(argv[0],"u16") == 0) {
    type_size = sizeof(uint16_t);
    type = U16;
  } else if(strcmp(argv[0],"u32") == 0) {
    type_size = sizeof(uint32_t);
    type = U32;
  } else if(strcmp(argv[0],"s8") == 0) {
    type_size = sizeof(int8_t);
    type = S8;
  } else if(strcmp(argv[0],"s16") == 0) {
    type_size = sizeof(int16_t);
    type = S16;
  } else if(strcmp(argv[0],"s32") == 0) {
    type_size = sizeof(int32_t);
    type = S32;
  } else {
    chprintf(chp, "First argument invalid. Use 'get_memory_data help' for help.\n");
    return;
  }

  unsigned int start_byte = atoi(argv[1]);

  unsigned int num_elements = 1;
  if (argc >= 3)
    num_elements = atoi(argv[2]);

  const size_t eeprom_size = EEPROM::getsize(&global.at24c01);
  uint8_t buffer[eeprom_size];
  if (start_byte + (type_size * num_elements) > eeprom_size) {
    num_elements = (eeprom_size - start_byte) / type_size;
    chprintf(chp, "Warning: request exceeds eeprom size -> limiting to %u values.\n", num_elements);
  }

  chFileStreamSeek((BaseFileStream*)&global.at24c01, start_byte);

  // Work around, because stm32f1 cannot read a single byte
  if (type_size*num_elements < 2)
    type_size = 2;

  uint32_t bytes_read = chSequentialStreamRead((BaseFileStream*)&global.at24c01, buffer, type_size*num_elements);

  if (bytes_read != type_size*num_elements)
    chprintf(chp, "Warning: %u of %u requested bytes were read.\n", bytes_read, type_size*num_elements);

  for (unsigned int i = 0; i < num_elements; ++i) {
    switch (type) {
      case HEX:
        chprintf(chp, "%02X ", buffer[i]);
        break;
      case U8:
        chprintf(chp, "%03u ", ((uint8_t*)buffer)[i]);
        break;
      case U16:
        chprintf(chp, "%05u ", ((uint16_t*)buffer)[i]);
        break;
      case U32:
        chprintf(chp, "%010u ", ((uint32_t*)buffer)[i]);
        break;
      case S8:
        chprintf(chp, "%+03d ", ((int8_t*)buffer)[i]);
        break;
      case S16:
        chprintf(chp, "%+05d ", ((int16_t*)buffer)[i]);
        break;
      case S32:
        chprintf(chp, "%+010d ", ((int32_t*)buffer)[i]);
        break;
      default:
        break;
    }
  }
  chprintf(chp, "\n");

  return;
}

void shellRequestSetLights(BaseSequentialStream *chp, int argc, char *argv[]) {

  if (argc < 2 || argc == 3 ||strcmp(argv[0],"help") == 0) {
    chprintf(chp, "\tUSAGE:\n");
    chprintf(chp, "> set_lights <led mask> <white/red> [<green> <blue>]\n");
    chprintf(chp, "\n");
    chprintf(chp, "\tled mask\n");
    chprintf(chp, "The LEDs to be set.\n");
    chprintf(chp, "You can set multiple LEDs at once by adding the following values:\n");
    chprintf(chp, "  0x01 - rear left LED (SSW)\n");
    chprintf(chp, "  0x02 - left rear LED (WSW)\n");
    chprintf(chp, "  0x04 - left front LED (WNW)\n");
    chprintf(chp, "  0x08 - front left LED (NNW)\n");
    chprintf(chp, "  0x10 - front right LED (NNE)\n");
    chprintf(chp, "  0x20 - right front LED (ENE)\n");
    chprintf(chp, "  0x40 - right rear LED (ESE)\n");
    chprintf(chp, "  0x80 - rear right LED (SSE)\n");
    chprintf(chp, "\twhite/red\n");
    chprintf(chp, "If no optional argument is given, this arguments sets the white value of the selected LEDs.\n");
    chprintf(chp, "Otherwise this arguments sets the red color channel value.\n");
    chprintf(chp, "\tgreen\n");
    chprintf(chp, "Sets the green color channel value.\n");
    chprintf(chp, "\tblue\n");
    chprintf(chp, "Sets the blue color channel value.\n");
    chprintf(chp, "\n");
    chprintf(chp, "\tExample:\n");
    chprintf(chp, "This line will set the two most left and two most right LEDs to bright cyan.\n");
    chprintf(chp, "> set_lights 0x66 0 255 255\n");
    chprintf(chp, "\n");
    return;
  }

  int arg_mask = strtol(argv[0], NULL, 0);
  int red = strtol(argv[1], NULL, 0);
  int green = red;
  int blue = red;
  if (argc >= 4) {
    green = strtol(argv[2], NULL, 0);
    blue = strtol(argv[3], NULL, 0);
  }
  Color color(red, green, blue);

  if (arg_mask & 0x01) {
    global.robot.setLightColor(constants::LightRing::LED_SSW, color);
  }
  if (arg_mask & 0x02) {
    global.robot.setLightColor(constants::LightRing::LED_WSW, color);
  }
  if (arg_mask & 0x04) {
    global.robot.setLightColor(constants::LightRing::LED_WNW, color);
  }
  if (arg_mask & 0x08) {
    global.robot.setLightColor(constants::LightRing::LED_NNW, color);
  }
  if (arg_mask & 0x10) {
    global.robot.setLightColor(constants::LightRing::LED_NNE, color);
  }
  if (arg_mask & 0x20) {
    global.robot.setLightColor(constants::LightRing::LED_ENE, color);
  }
  if (arg_mask & 0x40) {
    global.robot.setLightColor(constants::LightRing::LED_ESE, color);
  }
  if (arg_mask & 0x80) {
    global.robot.setLightColor(constants::LightRing::LED_SSE, color);
  }

  return;
}

void boardPeripheryCheck(BaseSequentialStream *chp) {
  msg_t result;
  chprintf(chp, "\nCHECK: START\n");
  // Check the accelerometer
  result = global.lis331dlh.getCheck();
  if (result == global.lis331dlh.CHECK_OK)
    chprintf(chp, "LIS331DLH: OK\n");
  else
    chprintf(chp, "LIS331DLH: FAIL\n");

  // Self-test accelerometer
//  lis331dlh.printSelfTest(NULL);

  // Check the eeprom
  result = global.memory.getCheck();
  if ( result != global.memory.OK)
    chprintf(chp, "Memory Structure: FAIL\n");
  else
    chprintf(chp, "Memory Structure: OK\n");

  // Check the gyroscope
  result = global.l3g4200d.getCheck();
  if (result == global.l3g4200d.CHECK_OK)
    chprintf(chp, "L3G4200D: OK\n");
  else
    chprintf(chp, "L3G4200D: FAIL\n");

  // Check the magnetometer
  result = global.hmc5883l.getCheck();
  if (result == global.hmc5883l.CHECK_OK)
    chprintf(chp, "HMC5883L: OK\n");
  else
    chprintf(chp, "HMC5883L: FAIL\n");

  // Check the MUX
  result = global.HW_PCA9544.getCheck();
  if (result == global.HW_PCA9544.CHECK_OK)
    chprintf(chp, "PCA9544: OK\n");
  else
    chprintf(chp, "PCA9544: FAIL\n");

  // Check the power monitor
  chprintf(chp, "INA219:\tVDD (3.3V):\n");
  result = global.ina219.selftest();
  if (result == BaseSensor<>::NOT_IMPLEMENTED)
    chprintf(chp, "->\tnot implemented\n");
  else if (result != INA219::Driver::ST_OK)
    chprintf(chp, "->\tFAIL (error code 0x%02X)\n", result);
  else
    chprintf(chp, "->\tOK\n");

  // Check the proximitysensors
  for (uint8_t i = 0x00; i < global.vcnl4020.size(); i++) {
    result = global.vcnl4020[i].getCheck();
    if (result == global.vcnl4020[i].CHECK_OK)
      chprintf(chp, "VCNL4020: %d OK\n", i);
    else
      chprintf(chp, "VCNL4020: %d FAIL\n", i);
  }
  chprintf(chp, "CHECK: FINISH\n");
}

void shellRequestCheck(BaseSequentialStream *chp, int __unused argc, char __unused *argv[]) {
  chprintf(chp, "shellRequestCheck\n");
  boardPeripheryCheck(chp);
}

void shellRequestResetMemory(BaseSequentialStream *chp, int __unused argc, char __unused *argv[]) {
  chprintf(chp, "shellRequestInitMemory\n");

  msg_t res = global.memory.resetMemory();
  if ( res != global.memory.OK)
    chprintf(chp, "Memory Init: FAIL\n");
  else
    chprintf(chp, "Memory Init: OK\n");
}

void shellRequestGetBoardId(BaseSequentialStream *chp, int __unused argc, char __unused *argv[]) {
  chprintf(chp, "shellRequestGetBoardId\n");
  uint8_t id = 0xFFu;

  msg_t res = global.memory.getBoardId(&id);

  if (res != global.memory.OK)
    chprintf(chp, "Get Board ID: FAIL\n");
  else
    chprintf(chp, "Get Board ID: %u\n", id);
}

void shellRequestSetBoardId(BaseSequentialStream *chp, int argc, char *argv[]) {
  chprintf(chp, "shellRequestSetBoardId\n");

  if (argc == 0) {
    chprintf(chp, "Usage: %s\n","set_board_id <idx>");
  } else {
    msg_t res = global.memory.setBoardId(atoi(argv[0]));
    if (res != global.memory.OK)
      chprintf(chp, "Set Board ID: FAIL\n");
    else
      chprintf(chp, "Set Board ID: OK\n");
  }
}

void shellRequestResetCalibrationConstants(BaseSequentialStream *chp, int __unused argc, char __unused *argv[]) {
  chprintf(chp, "shellRequestResetCalibrationConstants\n");
  chprintf(chp, "Setting Ed=1.0f, Eb=1.0f\n");
  msg_t res;

  res = global.memory.setEd(1.0f);
  if (res != global.memory.OK)
    chprintf(chp, "Set Ed: FAIL\n");
  else
    chprintf(chp, "Set Ed: OK\n");

  res = global.memory.setEb(1.0f);
  if (res != global.memory.OK)
    chprintf(chp, "Set Eb: FAIL\n");
  else
    chprintf(chp, "Set Eb: OK\n");
}

void shellRequestGetCalibrationConstants(BaseSequentialStream *chp, int __unused argc, char __unused *argv[]) {
  chprintf(chp, "shellRequestGetCalibrationConstants\n");
  msg_t res;
  float Ed, Eb;

  res = global.memory.getEd(&Ed);
  if (res != global.memory.OK)
    chprintf(chp, "Get Ed: FAIL\n");
  else
    chprintf(chp, "Get Ed: OK \t Ed=%f\n", Ed);

  res = global.memory.getEb(&Eb);
  if (res != global.memory.OK)
    chprintf(chp, "Get Eb: FAIL\n");
  else
    chprintf(chp, "Get Eb: OK \t Eb=%f\n", Eb);
}

void shellRequestSetCalibrationConstants(BaseSequentialStream *chp, int argc, char *argv[]) {
  chprintf(chp, "shellRequestSetCalibrationConstants\n");
  msg_t res;

  if (argc != 3) {
    chprintf(chp, "Usage: %s\n","set_Ed_Eb <Ed> <Eb> <Write To Eeprom ? 1 : 0>");
    chprintf(chp, "(Call with floating point values for Ed and Eb values and write condition):\n");
    return;
  }
  // Get the write condition
  const float Ed = atof(argv[0]);
  const float Eb = atof(argv[1]);
  bool_t writeToMemory = atoi(argv[2]) == 1 ? true : false;

  res = global.motorcontrol.setWheelDiameterCorrectionFactor(Ed, writeToMemory);
  if (res != global.memory.OK)
    chprintf(chp, "Set Ed: FAIL\n");
  else
    chprintf(chp, "Set Ed: OK \t Ed=%f\n", Ed);

  res = global.motorcontrol.setActualWheelBaseDistance(Eb, writeToMemory);
  if (res != global.memory.OK)
    chprintf(chp, "Set Eb: FAIL\n");
  else
    chprintf(chp, "Set Eb: OK \t Ed=%f\n", Eb);
}

void shellRequestGetVcnl(BaseSequentialStream *chp, int argc, char *argv[]) {
  chprintf(chp, "shellRequestGetVcnl\n");
  // Print the sensor information
  if (argc != 1) {
    chprintf(chp, "Usage: %s\n","get_vcnl <rep>");
    return;
  }
  for (int32_t rep = 0x00; rep < atoi(argv[0]); ++rep) {
    for (uint8_t idx = 0x00; idx < global.vcnl4020.size(); idx++) {
     chprintf(chp, "%d: Ambi %d\tProx raw %d\tProx scaled %d\n", idx, global.vcnl4020[idx].getAmbientLight(), global.vcnl4020[idx].getProximity(), global.vcnl4020[idx].getProximityScaledWoOffset());
    }
    chprintf(chp, "\n\n");
    BaseThread::sleep(MS2ST(250));
  }
}

void shellRequestSetVcnlOffset(BaseSequentialStream *chp, int argc, char *argv[]) {
  chprintf(chp, "shellRequestSetVcnlOffset\n");
  if (argc != 2) {
    chprintf(chp, "Usage: %s\n","set_vcnl <idx> <offset>");
    return;
  }

  uint8_t vcnlIdx = static_cast<uint8_t>(atoi(argv[0]));
  uint16_t vcnlOffset = static_cast<uint16_t>(atoi(argv[1]));

  if (vcnlIdx >= global.vcnl4020.size()) {
    chprintf(chp, "Wrong VCNL index: Choose [0 .. %d]\n", global.vcnl4020.size()-1);
    return;
  }

  msg_t res = global.memory.setVcnl4020Offset(vcnlOffset, vcnlIdx);
  if (res != global.memory.OK) {
    chprintf(chp, "Set Offset: FAIL\n");
  } else {
    chprintf(chp, "Set Offset: OK\n");
    global.vcnl4020[vcnlIdx].setProximityOffset(vcnlOffset);
  }
}

void shellRequestResetVcnlOffset(BaseSequentialStream *chp, int argc, char *argv[]) {
  msg_t res = global.memory.OK;
  for (uint8_t idx = 0; idx < 4; ++idx) {
    msg_t r = global.memory.setVcnl4020Offset(0, idx);
    if (r == global.memory.OK) {
      global.vcnl4020[idx].setProximityOffset(0);
    } else {
      chprintf(chp, "Reset Offset %u: FAIL\n", idx);
      res = r;
    }
  }

  if (res == global.memory.OK) {
    chprintf(chp, "Reset Offset: DONE\n");
  }

  return;
}

void shellRequestGetVcnlOffset(BaseSequentialStream *chp, int argc, char *argv[]) {
  chprintf(chp, "shellRequestGetVcnlOffset\n");
  if (argc != 1) {
    chprintf(chp, "Call with decimal numbers: get_vcnl <idx>\n");
    return;
  }

  uint8_t vcnlIdx = static_cast<uint8_t>(atoi(argv[0]));

  if (vcnlIdx >= global.vcnl4020.size()) {
    chprintf(chp, "Wrong VCNL index: Choose [0 .. %d]\n", global.vcnl4020.size()-1);
    return;
  }

  uint16_t vcnlOffset;
  msg_t res = global.memory.getVcnl4020Offset(&vcnlOffset, vcnlIdx);
  if (res != global.memory.OK) {
    chprintf(chp, "Get Offset: FAIL\n");
  } else {
    chprintf(chp, "Get Offset: OK \t Offset=%d\n", vcnlOffset);
  }
}

void shellRequestCalib(BaseSequentialStream *chp, int __unused argc, char __unused *argv[]) {
  chprintf(chp, "shellRequestCalib\n");
  global.robot.calibrate();
}

void shellRequestGetRobotId(BaseSequentialStream *chp, int __unused argc, char __unused *argv[]) {
  chprintf(chp, "shellRequestGetRobotId\n");
  chprintf(chp, "Robot ID: %u\n", global.robot.getRobotID());
  if (global.robot.getRobotID() == 0)
    chprintf(chp, "Warning: The board ID seems to be uninitialized.\n");
}

void shellRequestGetSystemLoad(BaseSequentialStream *chp, int argc, char *argv[]) {
  chprintf(chp, "shellRequestGetSystemLoad\n");
  uint8_t seconds = 1;
  if (argc >= 1) {
    seconds = atoi(argv[0]);
  }
  chprintf(chp, "measuring CPU load for %u %s...\n", seconds, (seconds>1)? "seconds" : "second");

  const systime_t before = chThdGetTicks(chSysGetIdleThread());
  BaseThread::sleep(S2ST(seconds));
  const systime_t after = chThdGetTicks(chSysGetIdleThread());
  const float usage = 1.0f - (float(after - before) / float(seconds * CH_FREQUENCY));

  chprintf(chp, "CPU load: %3.2f%%\n", usage * 100);
  const uint32_t memory_total = 0x10000;
  const uint32_t memory_load = memory_total - chCoreStatus();
  chprintf(chp, "RAM load: %3.2f%% (%u / %u Byte)\n", float(memory_load)/float(memory_total) * 100, memory_load, memory_total);
}

void shellSwitchBoardCmd(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 1) {
    chprintf(chp, "Call with decimal numbers: shell_board <idx>\n");
    return;
  }
  uint8_t boardIdx = static_cast<uint8_t>(atoi(argv[0]));

  chprintf(chp, "shellSwitchBoardCmd\n");
  global.sercanmux1.sendSwitchCmd(boardIdx);
}

void shellRequestGetBootloaderInfo(BaseSequentialStream* chp, int argc, char *argv[]) {
  // check the magic number
  switch (*((uint32_t*)(BL_CALLBACK_TABLE_ADDR))) {
    case (('A'<<24) | ('-'<<16) | ('B'<<8) | ('L'<<0)):
      chprintf((BaseSequentialStream*) &SD1, "Bootloader %u.%u.%u\n",
               ((blVersion_t*)(BL_CALLBACK_TABLE_ADDR + (1*4)))->major,
               ((blVersion_t*)(BL_CALLBACK_TABLE_ADDR + (1*4)))->minor,
               ((blVersion_t*)(BL_CALLBACK_TABLE_ADDR + (1*4)))->patch);
      break;

    case BL_MAGIC_NUMBER:
      chprintf((BaseSequentialStream*) &SD1, "Bootloader %u.%u.%u\n",
               *((uint32_t*)(BL_CALLBACK_TABLE_ADDR + (1*4))),
               *((uint32_t*)(BL_CALLBACK_TABLE_ADDR + (1*4))),
               *((uint32_t*)(BL_CALLBACK_TABLE_ADDR + (1*4))));
      break;

    default:
      chprintf((BaseSequentialStream*) &SD1, "Bootloader incompatible\n");
      break;
  }

  return;
}

void shellRequestMotorDrive(BaseSequentialStream *chp, int argc, char *argv[]) {
  types::kinematic tmp;
  tmp.w_z = 0;
  tmp.x = 0;
  if (argc == 1){
    chprintf(chp, "Set speed to %i um/s \n", atoi(argv[0]));
    tmp.x = atoi(argv[0]);
  } else {
    if(argc == 2){
      chprintf(chp, "Set speed to %i \n um/s", atoi(argv[0]));
      chprintf(chp, "Set angular speed to %i \n urad/s", atoi(argv[1]));
      tmp.x = atoi(argv[0]);
      tmp.w_z= atoi(argv[1]);
    } else {
      chprintf(chp, "Wrong number of parameters given (%i), stopping robot \n", argc);
    }
  }

  global.motorcontrol.setTargetSpeed(tmp);
  return;
}

void shellRequestMotorStop(BaseSequentialStream *chp, int argc, char *argv[]) {
  types::kinematic tmp;
  tmp.x = 0;
  tmp.w_z = 0;

  global.motorcontrol.setTargetSpeed(tmp);

  chprintf(chp, "stop");
return;
}

void shellRequestMotorCalibrate(BaseSequentialStream *chp, int argc, char *argv[]) {
  global.motorcontrol.resetGains();
  chprintf((BaseSequentialStream*)&global.sercanmux1, "motor calibration starts in five seconds...\n");
  BaseThread::sleep(MS2ST(5000));
  global.motorcontrol.isCalibrating = true;

  return;
}

void shellRequestMotorGetGains(BaseSequentialStream *chp, int argc, char *argv[]){
  global.motorcontrol.printGains();

  return;
}

void shellRequestMotorResetGains(BaseSequentialStream *chp, int argc, char *argv[]) {
  global.motorcontrol.resetGains();;

  return;
}

static const ShellCommand commands[] = {
  {"shutdown", shellRequestShutdown},
  {"wakeup", shellRequestWakeup},
  {"check", shellRequestCheck},
  {"reset_memory", shellRequestResetMemory},
  {"get_board_id", shellRequestGetBoardId},
  {"set_board_id", shellRequestSetBoardId},
  {"get_memory_data", shellRequestGetMemoryData},
  {"get_vcnl", shellRequestGetVcnl},
  {"calib_vcnl_offset", shellRequestCalib},
  {"set_vcnl_offset", shellRequestSetVcnlOffset},
  {"reset_vcnl_offset", shellRequestResetVcnlOffset},
  {"get_vcnl_offset", shellRequestGetVcnlOffset},
  {"reset_Ed_Eb", shellRequestResetCalibrationConstants},
  {"get_Ed_Eb", shellRequestGetCalibrationConstants},
  {"set_Ed_Eb", shellRequestSetCalibrationConstants},
  {"get_robot_id", shellRequestGetRobotId},
  {"get_system_load", shellRequestGetSystemLoad},
  {"set_lights", shellRequestSetLights},
  {"shell_board", shellSwitchBoardCmd},
  {"get_bootloader_info", shellRequestGetBootloaderInfo},
  {"motor_drive", shellRequestMotorDrive},
  {"motor_stop", shellRequestMotorStop},
  {"motor_calibrate", shellRequestMotorCalibrate},
  {"motor_getGains", shellRequestMotorGetGains},
  {"motor_resetGains", shellRequestMotorResetGains},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *) &global.sercanmux1,
  commands
};

void initPowermonitor(INA219::Driver &ina219, const float shuntResistance_O, const float maxExpectedCurrent_A, const uint16_t currentLsb_uA)
{
  INA219::CalibData calibData;
  INA219::InitData initData;

  calibData.input.configuration.content.brng = INA219::Configuration::BRNG_16V;
  calibData.input.configuration.content.pg = INA219::Configuration::PGA_40mV;
  calibData.input.configuration.content.badc = INA219::Configuration::ADC_68100us;
  calibData.input.configuration.content.sadc = INA219::Configuration::ADC_68100us;
  calibData.input.configuration.content.mode = INA219::Configuration::MODE_ShuntBus_Continuous;
  calibData.input.shunt_resistance_O = shuntResistance_O;
  calibData.input.max_expected_current_A = maxExpectedCurrent_A;
  calibData.input.current_lsb_uA = currentLsb_uA;
  if (ina219.calibration(&calibData) != BaseSensor<>::SUCCESS)
  {
    chprintf((BaseSequentialStream*)&SD1, "WARNING: calibration of INA219 failed.\n");
  }

  initData.configuration.value = calibData.input.configuration.value;
  initData.calibration = calibData.output.calibration_value;
  initData.current_lsb_uA = calibData.output.current_lsb_uA;
  if (ina219.init(&initData) != BaseSensor<>::SUCCESS)
  {
    chprintf((BaseSequentialStream*)&SD1, "WARNING: initialization of INA219 failed.\n");
  }

  if (calibData.input.current_lsb_uA != initData.current_lsb_uA)
  {
    chprintf((BaseSequentialStream*)&SD1, "NOTE: LSB for current measurement was limited when initializing INA219 (%u -> %u)", calibData.input.current_lsb_uA, initData.current_lsb_uA);
  }

  return;
}

/*
 * Application entry point.
 */
int main(void) {

//  int16_t accel;
  Thread *shelltp = NULL;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  qeiInit();
  System::init();

//  boardWakeup();
//  boardWriteIoPower(1);

  /*
   * Activates the serial driver 2 using the driver default configuration.
   */
  sdStart(&SD1, &global.sd1_config);

  chprintf((BaseSequentialStream*) &SD1, "\n");
  chprintf((BaseSequentialStream*) &SD1, BOARD_NAME " " BOARD_VERSION "\n");
  switch (*((uint32_t*)(BL_CALLBACK_TABLE_ADDR))) {
    case (('A'<<24) | ('-'<<16) | ('B'<<8) | ('L'<<0)):
      chprintf((BaseSequentialStream*) &SD1, "Bootloader %u.%u.%u\n",
               ((blVersion_t*)(BL_CALLBACK_TABLE_ADDR + (1*4)))->major,
               ((blVersion_t*)(BL_CALLBACK_TABLE_ADDR + (1*4)))->minor,
               ((blVersion_t*)(BL_CALLBACK_TABLE_ADDR + (1*4)))->patch);
      break;

    case BL_MAGIC_NUMBER:
      chprintf((BaseSequentialStream*) &SD1, "Bootloader %u.%u.%u\n",
               *((uint32_t*)(BL_CALLBACK_TABLE_ADDR + (1*4))),
               *((uint32_t*)(BL_CALLBACK_TABLE_ADDR + (1*4))),
               *((uint32_t*)(BL_CALLBACK_TABLE_ADDR + (1*4))));
      break;

    default:
      chprintf((BaseSequentialStream*) &SD1, "Bootloader incompatible\n");
      break;
  }
  chprintf((BaseSequentialStream*) &SD1, "ChibiOS " CH_KERNEL_VERSION "\n");
  // make sure that the info text is completetly printed
  BaseThread::sleep(10);

  extStart(&EXTD1, &extcfg);

  /*
   * Wait for a certain amount of time, so that the PowerBoard can activate
   * the IO voltages for the I2C Bus
   */
  BaseThread::sleep(MS2ST(2000));

  boardClearI2CBus(GPIOB_COMPASS_SCL, GPIOB_COMPASS_SDA);
  boardClearI2CBus(GPIOB_IR_SCL, GPIOB_IR_SDA);

  global.HW_I2C1.start(&global.i2c1_config);
  global.HW_I2C2.start(&global.i2c2_config);

  global.memory.init();

  uint8_t i = 0;
  if (global.memory.getBoardId(&i) == fileSystemIo::FileSystemIoBase::OK) {
    chprintf((BaseSequentialStream*) &SD1, "Board ID: %u\n", i);
  } else {
    chprintf((BaseSequentialStream*) &SD1, "Error reading board ID\n");
  }
  chprintf((BaseSequentialStream*) &SD1, "\n");

  initPowermonitor(global.ina219, 0.1f, 0.075f, 10);

  for (i = 0x00u; i < global.vcnl4020.size(); i++) {
    uint16_t buffer;
    global.memory.getVcnl4020Offset(&buffer,i);
    global.vcnl4020[i].setProximityOffset(buffer);
    global.vcnl4020[i].start(NORMALPRIO);
  }

  global.ina219.start(NORMALPRIO);

  global.hmc5883l.start(NORMALPRIO + 8);

  global.increments.start();  // Start the qei driver

  global.odometry.start(NORMALPRIO + 20);

  global.robot.start(HIGHPRIO - 1);

  global.motorcontrol.start(NORMALPRIO + 7);

  global.distcontrol.start(NORMALPRIO + 9);

  // Set target velocity
  types::kinematic velocity;
  velocity.x = 0; // E.g.  "100*1e3" equals "10 cm/s"
  velocity.w_z = 0; // E.g. "2*1e6" equals "2 rad/s"
  global.motorcontrol.setTargetSpeed(velocity);

  // Start the three axes linear accelerometer
  global.lis331dlh.configure(&global.accel_run_config);
  global.lis331dlh.start(NORMALPRIO+4);

  // Start the three axes gyroscope
  global.l3g4200d.configure(&global.gyro_run_config);
  global.l3g4200d.start(NORMALPRIO+5);

  // Start the user thread
  global.userThread.start(NORMALPRIO);

  /* let the SYS_SYNC_N pin go, to signal that the initialization of the module is done */
  palWritePad(GPIOC, GPIOC_SYS_INT_N, PAL_HIGH);

  /* wait until all modules are done */
  while (palReadPad(GPIOC, GPIOC_SYS_INT_N) == PAL_LOW) {
    continue;
  }

  while (true) {

    if (!shelltp)
      shelltp = shellCreate(&shell_cfg1, THD_WA_SIZE(1024), NORMALPRIO);
    else if (chThdTerminated(shelltp)) {
      chThdRelease(shelltp);    /* Recovers memory of the previous shell. */
      shelltp = NULL;           /* Triggers spawning of a new shell.      */
    }

    // Let the LED just blink as an alive signal
    boardWriteLed(1);
    BaseThread::sleep(MS2ST(250));
    boardWriteLed(0);
    BaseThread::sleep(MS2ST(250));

    if (shutdown_now != SHUTDOWN_NONE) {
      if ((*((uint32_t*)(BL_CALLBACK_TABLE_ADDR)) != (('A'<<24) | ('-'<<16) | ('B'<<8) | ('L'<<0))) && (*((uint32_t*)(BL_CALLBACK_TABLE_ADDR)) != BL_MAGIC_NUMBER)) {
        chprintf((BaseSequentialStream*) &SD1, "ERROR: unable to shut down (bootloader deprecated).\n");
        shutdown_now = SHUTDOWN_NONE;
      } else {
        uint32_t blCallbackPtrAddr = BL_CALLBACK_TABLE_ADDR;
        // handle bootloader version 0.2.x
        if ((*((uint32_t*)(BL_CALLBACK_TABLE_ADDR)) == BL_MAGIC_NUMBER) &&
            (*((uint32_t*)(BL_CALLBACK_TABLE_ADDR + (1*4))) == 0 && *((uint32_t*)(BL_CALLBACK_TABLE_ADDR + (2*4))) == 2)) {
          switch (shutdown_now) {
            case SHUTDOWN_TRANSPORTATION:
              blCallbackPtrAddr += 6 * 4;
              break;
            case SHUTDOWN_DEEPSLEEP:
              blCallbackPtrAddr += 5 * 4;
              break;
            case SHUTDOWN_HIBERNATE:
              blCallbackPtrAddr += 4 * 4;
              break;
            case SHUTDOWN_HANDLE_REQUEST:
            case SHUTDOWN_RESTART:
              blCallbackPtrAddr += 10 * 4;
              break;
            default:
              blCallbackPtrAddr = 0;
              break;
          }
        }
        // handle bootloader version 0.3.x
        else if ((*((uint32_t*)(BL_CALLBACK_TABLE_ADDR)) == BL_MAGIC_NUMBER) &&
                 (*((uint32_t*)(BL_CALLBACK_TABLE_ADDR + (1*4))) == 0 && *((uint32_t*)(BL_CALLBACK_TABLE_ADDR + (2*4))) == 3)) {
          switch (shutdown_now) {
            case SHUTDOWN_TRANSPORTATION:
              blCallbackPtrAddr += 6 * 4;
              break;
            case SHUTDOWN_DEEPSLEEP:
              blCallbackPtrAddr += 5 * 4;
              break;
            case SHUTDOWN_HIBERNATE:
              blCallbackPtrAddr += 4 * 4;
              break;
            case SHUTDOWN_RESTART:
              blCallbackPtrAddr += 7 * 4;
              break;
            case SHUTDOWN_HANDLE_REQUEST:
              blCallbackPtrAddr += 8 * 4;
              break;
            default:
              blCallbackPtrAddr = 0;
              break;
          }
        }
        // handle bootloader version 1.0.x
        else if ((*((uint32_t*)(BL_CALLBACK_TABLE_ADDR)) == (('A'<<24) | ('-'<<16) | ('B'<<8) | ('L'<<0))) &&
                 ((blVersion_t*)(BL_CALLBACK_TABLE_ADDR + (1*4)))->major == 1 && ((blVersion_t*)(BL_CALLBACK_TABLE_ADDR + (1*4)))->minor == 0) {
          switch (shutdown_now) {
            case SHUTDOWN_TRANSPORTATION:
              blCallbackPtrAddr += 6 * 4;
              break;
            case SHUTDOWN_DEEPSLEEP:
              blCallbackPtrAddr += 5 * 4;
              break;
            case SHUTDOWN_HIBERNATE:
              blCallbackPtrAddr += 4 * 4;
              break;
            case SHUTDOWN_RESTART:
              blCallbackPtrAddr += 7 * 4;
              break;
            case SHUTDOWN_HANDLE_REQUEST:
              blCallbackPtrAddr += 8 * 4;
              break;
            default:
              blCallbackPtrAddr = 0;
              break;
          }
        }

        void (*blCallback)(void) = NULL;
        if (blCallbackPtrAddr > BL_CALLBACK_TABLE_ADDR) {
          blCallback = (void (*)(void))(*((uint32_t*)blCallbackPtrAddr));

          if (!blCallback) {
            chprintf((BaseSequentialStream*) &SD1, "ERROR: Requested shutdown not supported.\n");
            shutdown_now = SHUTDOWN_NONE;
          } else {
            chprintf((BaseSequentialStream*)&SD1, "initiating shutdown sequence...\n");
            palWritePad(GPIOC, GPIOC_SYS_INT_N, PAL_LOW);
            palWritePad(GPIOC, GPIOC_SYS_PD_N, PAL_LOW);

            chprintf((BaseSequentialStream*)&SD1, "stopping all threads and periphery...");
            systemShutdown();
            chprintf((BaseSequentialStream*)&SD1, "\tdone\n");
            BaseThread::sleep(MS2ST(10)); // sleep to print everything

            blCallback();
          }

        } else {
          chprintf((BaseSequentialStream*) &SD1, "ERROR: invalid shutdown requested (%u).\n", shutdown_now);
          shutdown_now = SHUTDOWN_NONE;
        }
      }

//    for (uint8_t i = LIS331DLH::AXIS_X; i <= LIS331DLH::AXIS_Z; i++) {
//        accel = lis331dlh.getAcceleration(i);
//        chprintf((BaseSequentialStream*) &SD1, "%c%04X ", accel < 0 ? '-' : '+', accel < 0 ? -accel : accel);
//    }
//
//    chprintf((BaseSequentialStream*) &SD1, "\n");
//
//    // Print out an alive signal
//    chprintf((BaseSequentialStream*) &SD1, ".");
    }
  }

  return 0;
}
