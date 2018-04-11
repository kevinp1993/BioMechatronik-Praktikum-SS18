#define BL_CALLBACK_TABLE_ADDR  (0x08000000 + 0x01C0)
#define BL_MAGIC_NUMBER         ((uint32_t)0xFF669900u)

#define SHUTDOWN_NONE             0
#define SHUTDOWN_TRANSPORTATION   1
#define SHUTDOWN_DEEPSLEEP        2
#define SHUTDOWN_HIBERNATE        3
#define SHUTDOWN_RESTART          4
#define SHUTDOWN_HANDLE_REQUEST   5

#include <ch.hpp>

#include "global.hpp"
#include <amiro/util/util.h>
#include <amiro/Color.h>
#include <exti.hpp>

#include <chprintf.h>
#include <shell.h>

using namespace amiro;
using namespace chibios_rt;

Global global;

struct blVersion_t {
  const uint8_t identifier;
  const uint8_t major;
  const uint8_t minor;
  const uint8_t patch;
} __attribute__((packed));

void shellRequestShutdown(BaseSequentialStream* chp, int argc, char *argv[]) {

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

void testLights() {
  for (int i = 0; i < (8 * 16 * 2); i++) {
    Color color;
    int brightness;

    switch (((i / 16) / 2) % 8) {
      case 0:
        color = Color::WHITE;
        break;
      case 1:
        color = Color::RED;
        break;
      case 2:
        color = Color::FUCHSIA;
        break;
      case 3:
        color = Color::BLUE;
        break;
      case 4:
        color = Color::AQUA;
        break;
      case 5:
        color = Color::LIME;
        break;
      case 6:
        color = Color::YELLOW;
        break;
      case 7:
      default:
        color = Color::BLACK;
        break;
    }

    if (i & (1 << 4))
      brightness = 10 + (i % 16) * 6;
    else
      brightness = 10;

    global.robot.setLightBrightness(brightness);
    global.robot.setLightColor((i / 2) % 8, color);

    BaseThread::sleep(MS2ST(250));
  }
}

void testColors() {
  Color color;

  global.robot.setLightBrightness(50);

  for (int i = 0; i < Color::YELLOWGREEN; i++) {
    color = Color(static_cast<Color::GlobalColor>(i));
    for (int j = 0; j < 8; j++)
      global.robot.setLightColor(j, color);

    BaseThread::sleep(MS2ST(250));
  }

}

void testFade() {
  for (int i = 0; i < 255; i += 16) {
    Color color(255 - i, i, 0);
    for (int j = 0; j < 8; j++) {
      global.robot.setLightColor(j, color);
      BaseThread::sleep(MS2ST(250));
    }
  }
}

void testBrightness() {
  for (int j = 0; j < 8; j++)
    global.robot.setLightColor(j, Color::WHITE);

  for (int i = 0; i < 200; i += 5) {
    int brightness = (i > 100) ? 200 - i : i;
    global.robot.setLightBrightness(brightness);
    BaseThread::sleep(MS2ST(100));
  }
}

static msg_t shutdownRequest(void *arg) {

  (void) arg;
  const size_t max_str_len = strlen("shutdown");
  uint8_t buffer[20];
  uint8_t num_bytes = 0x00u;
  uint8_t consume;

  while (true) {

    num_bytes += sdAsynchronousRead(&SD1, &buffer[num_bytes], sizeof(buffer) - num_bytes);

    if (num_bytes) {

      consume = 0x01u;

      if (num_bytes >= strlen("shutdown") && !strncmp((char*) buffer, "shutdown", strlen("shutdown"))) {

        boardRequestShutdown();
        consume = strlen("shutdown");

      }

      if (consume != 0x01 || num_bytes >= max_str_len) {

        num_bytes -= consume;
        memcpy(&buffer[0], &buffer[consume], num_bytes);

      }

    }

    BaseThread::sleep(MS2ST(1000));

  }

  return RDY_OK;

}

void systemShutdown() {

  global.userThread.requestTerminate();
  global.userThread.wait();

  global.robot.setLightBrightness(0);
  global.robot.terminate();

  global.tlc5947.requestTerminate();
  global.tlc5947.update();
  global.tlc5947.wait();

  global.lidar.requestTerminate();
  global.lidar.wait();

//  boardStandby();

  return;
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

void boardPeripheryCheck(BaseSequentialStream *chp) {
  msg_t result;
  chprintf(chp, "\nCHECK: START\n");

  // Check the radio
  result = global.a2500r24a.getCheck();
  if (result == global.a2500r24a.CHECK_OK)
    chprintf(chp, "A2500R24A: OK\n");
  else
    chprintf(chp, "A2500R24A: FAIL\n");

  // Check the eeprom
  result = global.memory.getCheck();
  if ( result != global.memory.OK)
    chprintf(chp, "Memory Structure: FAIL\n");
  else
    chprintf(chp, "Memory Structure: OK\n");

  // Check the lights
  chprintf(chp, "LEDs: Test colors\n");
  testColors();
  chprintf(chp, "LEDs: Off\n");
  global.robot.setLightBrightness(0);

  chprintf(chp, "CHECK: FINISH\n");
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

void shellRequestCheck(BaseSequentialStream *chp, int __unused argc, char __unused *argv[]) {
  chprintf(chp, "shellRequestCheck\n");
  boardPeripheryCheck(chp);
}

void shellRequestGetRobotId(BaseSequentialStream *chp, int __unused argc, char __unused *argv[]) {
  chprintf(chp, "shellRequestGetRobotId\n");
  chprintf(chp, "Robot ID: %u\n", global.robot.getRobotID());
  if (global.robot.getRobotID() == 0)
    chprintf(chp, "Warning: The ID seems to be uninitialized. Is CAN communication working correctly?\n");
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

static const ShellCommand commands[] = {
  {"shutdown", shellRequestShutdown},
  {"check", shellRequestCheck},
  {"reset_memory", shellRequestResetMemory},
  {"get_board_id", shellRequestGetBoardId},
  {"set_board_id", shellRequestSetBoardId},
  {"get_memory_data", shellRequestGetMemoryData},
  {"get_robot_id", shellRequestGetRobotId},
  {"get_system_load", shellRequestGetSystemLoad},
  {"shell_board", shellSwitchBoardCmd},
  {"get_bootloader_info", shellRequestGetBootloaderInfo},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *) &global.sercanmux1,
  commands
};



/*
 * Application entry point.
 */
int main(void) {

  Thread *shelltp = NULL;
//  global.shellTermID = CAN::LIGHT_RING_ID;
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  System::init();

  /*
   * Activates the serial driver 2 using the driver default configuration.
   */
  sdStart(&SD1, &global.sd1_config);
  sdStart(&SD2, &global.sd2_config);

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

  boardClearI2CBus(GPIOB_MEM_SCL, GPIOB_MEM_SDA);

  global.HW_I2C2.start(&global.i2c2_config);

  global.memory.init();

  uint8_t i = 0;
  if (global.memory.getBoardId(&i) == fileSystemIo::FileSystemIoBase::OK) {
    chprintf((BaseSequentialStream*) &SD1, "Board ID: %u\n", i);
  } else {
    chprintf((BaseSequentialStream*) &SD1, "Error reading board ID\n");
  }
  chprintf((BaseSequentialStream*) &SD1, "\n");

  global.tlc5947.start(NORMALPRIO + 5);
  global.tlc5947.enable();

  global.robot.start(HIGHPRIO - 1);

//   lidar.start(NORMALPRIO + 15); UNCOMMENT TO START LIDAR

  global.lidar.start(NORMALPRIO);

  global.userThread.start(NORMALPRIO);

  /* let the SYS_SYNC_N pin go, to signal that the initialization of the module is done */
  palWritePad(GPIOD, GPIOD_SYS_INT_N, PAL_HIGH);

  /* wait until all modules are done */
  while (palReadPad(GPIOD, GPIOD_SYS_INT_N) == PAL_LOW) {
    continue;
  }

  global.robot.setLightBrightness(10);
  global.robot.setLightColor(0, Color::RED);
  global.robot.setLightColor(1, Color::LIME);
  global.robot.setLightColor(2, Color::BLUE);
  global.robot.setLightColor(3, Color::WHITE);
  global.robot.setLightColor(4, Color::RED);
  global.robot.setLightColor(5, Color::LIME);
  global.robot.setLightColor(6, Color::BLUE);
  global.robot.setLightColor(7, Color::WHITE);

//   static uint16_t scannedData[NUMBER_OF_STEPS /*see lidar.h for this variable*/] = {}; UNCOMMENT TO START LIDAR
  while (true) {

    if (!shelltp)
      shelltp = shellCreate(&shell_cfg1, THD_WA_SIZE(1024), NORMALPRIO);
    else if (chThdTerminated(shelltp)) {
      chThdRelease(shelltp);    /* Recovers memory of the previous shell. */
      shelltp = NULL;           /* Triggers spawning of a new shell.      */
    }

//    testLights();
//    testColors();
//    testFade();
//    testBrightness();
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
        // handle bootloader version 1.0.x and 1.1.x
        else if ((*((uint32_t*)(BL_CALLBACK_TABLE_ADDR)) == (('A'<<24) | ('-'<<16) | ('B'<<8) | ('L'<<0))) &&
                 ((blVersion_t*)(BL_CALLBACK_TABLE_ADDR + (1*4)))->major == 1 && (((blVersion_t*)(BL_CALLBACK_TABLE_ADDR + (1*4)))->minor == 0 || ((blVersion_t*)(BL_CALLBACK_TABLE_ADDR + (1*4)))->minor == 1)) {
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
            palWritePad(GPIOD, GPIOD_SYS_INT_N, PAL_LOW);
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
    }

//     // Print LIDAR scan
//     if (global.lidar.getScan(scannedData)) {
//       for (uint16_t idx = 0; idx < NUMBER_OF_STEPS; ++idx) {
//         chprintf((BaseSequentialStream*) &SD1, "%d ", scannedData[idx]);
//       }
//       chprintf((BaseSequentialStream*) &SD1, "\n\n");
//     }
    BaseThread::sleep(MS2ST(500));
  }

  return 0;
}
