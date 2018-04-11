#ifndef IN_CCM
/*
 * @brief Makro to store data in the core coupled memory (ccm).
 *        Example:
 *        int compute_buffer[128] IN_CCM;
 *
 * @note The ccm is not connected to any bus system.
 */
#define IN_CCM  __attribute__((section(".ccm"))) __attribute__((aligned(4)))
#endif

#ifndef IN_ETH
/*
 * @brief Makro to store data in the ethernet memory (eth).
 *        Example:
 *        int dma_buffer[128] IN_ETH;
 *
 * @note The eth is a dedicated memory block with its own DMA controller.
 */
#define IN_ETH  __attribute__((section(".eth"))) __attribute__((aligned(4)))
#endif

#define BL_CALLBACK_TABLE_ADDR  (0x08000000 + 0x01C0)
#define BL_MAGIC_NUMBER         ((uint32_t)0xFF669900u)

#define SHUTDOWN_NONE             0
#define SHUTDOWN_TRANSPORTATION   1
#define SHUTDOWN_DEEPSLEEP        2
#define SHUTDOWN_HIBERNATE        3
#define SHUTDOWN_RESTART          4
#define SHUTDOWN_HANDLE_REQUEST   5

#include <ch.hpp>
#include <shell.h>
#include <chprintf.h>
#include <wakeup.h>
#include <cstdlib>
#include <cstring>
#include <amiro/util/util.h>
#include <global.hpp>
#include <exti.hpp>

using namespace amiro;
using namespace constants::PowerManagement;

Global global;

struct blVersion_t {
  const uint8_t identifier;
  const uint8_t major;
  const uint8_t minor;
  const uint8_t patch;
} __attribute__((packed));

void shutdownTimeoutISR(void *arg) {

  (void) arg;

}

void systemStop() {

//  VirtualTimer shutdownTimeout;
  uint8_t i;

  // tell all boards that it's time to shut down
  global.robot.broadcastShutdown();

  global.userThread.requestTerminate();
  global.userThread.wait();

  // kill bluetooth
  boardBluetoothSetState(0);

  global.adc1_vsys.requestTerminate();
  global.adc1_vsys.wait();

  for (i = 0; i < global.vcnl4020.size(); ++i) {
    global.vcnl4020[i].requestTerminate();
    global.vcnl4020[i].wait();
  }

  for (i = 0; i < global.bq27500.size(); ++i) {
    global.bq27500[i].requestTerminate();
    global.bq27500[i].wait();
  }

  for (i = 0; i < global.ina219.size(); ++i) {
    global.ina219[i].requestTerminate();
    global.ina219[i].wait();
  }

//  boardWriteIoPower(0);
  global.mpr121.configure(&global.mpr121_stdby_config);
  /* cannot shut down touch, b/c we need it to
   * clear any interrupt, so WKUP is not blocked
   */

  // stop I²C
  for (i = 0; i < global.V_I2C1.size(); ++i)
    global.V_I2C1[i].stop();
  for (i = 0; i < global.V_I2C2.size(); ++i)
    global.V_I2C2[i].stop();

  global.mpr121.requestTerminate();
  global.mpr121.wait();

  global.HW_I2C2.stop();
  global.HW_I2C1.stop();

  // stop all threads
  global.robot.terminate();

//  // 60 sec timeout
//  palWritePad(GPIOC, GPIOC_SYS_INT_N, PAL_HIGH);
//  chVTSet(&shutdownTimeout, MS2ST(60000), shutdownTimeoutISR, NULL);
//  // wait for all boards to release SYS_INT_N
//  while (palReadPad(GPIOC, GPIOC_SYS_INT_N)!=PAL_HIGH && chVTIsArmedI(&shutdownTimeout)) {
//    BaseThread::sleep(MS2ST(1)); /* must sleep for VT, else it will never fire */
//  }
//  chVTReset(&shutdownTimeout);

//  chprintf((BaseSequentialStream*) &SD1, "Stop\n");
//  boardWriteSystemPower(0);
//  boardWriteLed(1);
//  boardStop(0x00, 0x00);

//  /*
//   * HSI-PLL domain now.
//   */
//  //chprintf((BaseSequentialStream*) &SD1, "After Stop\n");
//  boardWriteLed(1);

//  while (true)
//    BaseThread::sleep(MS2ST(250));

  return;
}

void systemShutdown() {

  VirtualTimer shutdownTimeout;
  uint8_t i;

  // tell all boards that it's time to shut down
  global.robot.broadcastShutdown();

  // wait a little to make sure all boards got the message and had time to pull their SYS_PD_N pins down
  BaseThread::sleep(MS2ST(500));

  // stop the user thread
  global.userThread.requestTerminate();
  global.userThread.wait();

  // kill bluetooth
  boardBluetoothSetState(0);

  // stop all threads
  global.robot.terminate();

  global.adc1_vsys.requestTerminate();
  global.adc1_vsys.wait();

  for (i = 0; i < global.vcnl4020.size(); ++i) {
    global.vcnl4020[i].requestTerminate();
    global.vcnl4020[i].wait();
  }

  for (i = 0; i < global.bq27500.size(); ++i) {
    global.bq27500[i].requestTerminate();
    global.bq27500[i].wait();
  }

  for (i = 0; i < global.ina219.size(); ++i) {
    global.ina219[i].requestTerminate();
    global.ina219[i].wait();
  }

  // 60 sec timeout
  chVTSet(&shutdownTimeout, MS2ST(60000), shutdownTimeoutISR, NULL);

  // wait for all boards to release SYS_PD_N
  while (!palReadPad(GPIOC, GPIOC_SYS_PD_N) && chVTIsArmedI(&shutdownTimeout))
    BaseThread::sleep(MS2ST(1)); /* must sleep for VT, else it will never fire */

  chVTReset(&shutdownTimeout);
  boardWriteIoPower(0);
  global.mpr121.configure(&global.mpr121_stdby_config);
  /* cannot shut down touch, b/c we need it to
   * clear any interrupt, so WKUP is not blocked
   */

  // stop I²C
  for (i = 0; i < global.V_I2C1.size(); ++i)
    global.V_I2C1[i].stop();
  for (i = 0; i < global.V_I2C2.size(); ++i)
    global.V_I2C2[i].stop();

  boardWriteSystemPower(0);
  boardStandby();

}


void boardPeripheryCheck(BaseSequentialStream *chp) {

#ifndef AMIRO_NSELFTEST
  chprintf(chp, "\nCHECK: START\n");
  msg_t result = 0;

    // Check the proximitysensors
  for (uint8_t i = 0; i < global.vcnl4020.size(); i++) {
    result = global.vcnl4020[i].getCheck();
    if (result == global.vcnl4020[i].CHECK_OK)
      chprintf(chp, "VCNL4020: %d OK\n", i);
    else
      chprintf(chp, "VCNL4020: %d FAIL\n", i);
  }
  chprintf(chp, "----------------------------------------\n");

  // check the PowerPath controller
  chprintf(chp, "\n");
  if (global.ltc4412.isPluggedIn())
    chprintf(chp, "LTC4412: plugged in\n");
  else
    chprintf(chp, "LTC4412: not plugged in\n");
  chprintf(chp, "----------------------------------------\n");

  // Check the eeprom
  result = global.memory.getCheck();
  if ( result != global.memory.OK)
    chprintf(chp, "Memory Structure: FAIL\n");
  else
    chprintf(chp, "Memory Structure: OK\n");
  chprintf(chp, "----------------------------------------\n");

  // Check the power monitors
  INA219::BusVoltage bus_voltage;
  chprintf(chp, "\n");
  chprintf(chp, "INA219:\n");
  chprintf(chp, "\tVDD (3.3V):\n");
  uint8_t result_ina219_vdd = global.ina219[INA_VDD].selftest();
  chprintf(chp, "->\t");
  if (result_ina219_vdd == BaseSensor<>::NOT_IMPLEMENTED)
    chprintf(chp, "not implemented");
  else if (result_ina219_vdd != INA219::Driver::ST_OK)
    chprintf(chp, "FAIL (error code 0x%02X)", result_ina219_vdd);
  else
    chprintf(chp, "OK");

  chprintf(chp, "\n\n");
  chprintf(chp, "\tVIO1.8:\n");
  uint8_t result_ina219_vio18 = global.ina219[INA_VIO18].selftest();
  chprintf(chp, "->\t");
  if (result_ina219_vio18 == BaseSensor<>::NOT_IMPLEMENTED)
    chprintf(chp, "not implemented");
  else if (result_ina219_vio18 != INA219::Driver::ST_OK)
    chprintf(chp, "FAIL (error code 0x%02X)", result_ina219_vio18);
  else
    chprintf(chp, "OK");

  chprintf(chp, "\n\n");
  chprintf(chp, "\tVIO3.3:\n");
  uint8_t result_ina219_vio33 = global.ina219[INA_VIO33].selftest();
  chprintf(chp, "->\t");
  if (result_ina219_vio33 == BaseSensor<>::NOT_IMPLEMENTED)
    chprintf(chp, "not implemented");
  else if (result_ina219_vio33 != INA219::Driver::ST_OK)
    chprintf(chp, "FAIL (error code 0x%02X)", result_ina219_vio33);
  else
    chprintf(chp, "OK");

  chprintf(chp, "\n\n");
  chprintf(chp, "\tVIO4.2:\n");
  uint8_t result_ina219_vio42 = global.ina219[INA_VIO42].selftest();
  chprintf(chp, "->\t");
  if (result_ina219_vio42 == BaseSensor<>::NOT_IMPLEMENTED)
    chprintf(chp, "not implemented");
  else if (result_ina219_vio42 != INA219::Driver::ST_OK)
    chprintf(chp, "FAIL (error code 0x%02X)", result_ina219_vio42);
  else
    chprintf(chp, "OK");

  bus_voltage = global.ina219[INA_VIO42].readBusVoltage();
  chprintf(chp, "\n\n");
  chprintf(chp, "\tVIO5.0:\n");
  uint8_t result_ina219_vio50 = global.ina219[INA_VIO50].selftest();
  chprintf(chp, "->\t");
  if (result_ina219_vio50 == BaseSensor<>::NOT_IMPLEMENTED)
    chprintf(chp, "not implemented");
  else if (result_ina219_vio50 != INA219::Driver::ST_OK)
    chprintf(chp, "FAIL (error code 0x%02X)", result_ina219_vio50);
  else
    chprintf(chp, "OK");

  chprintf(chp, "\n\n");
  result = result_ina219_vdd | result_ina219_vio18 | result_ina219_vio33 | result_ina219_vio42 | result_ina219_vio50;
  if (result == BaseSensor<>::NOT_IMPLEMENTED)
    chprintf(chp, "->\tINA219: not implemented\n");
  else
    chprintf(chp, "->\tINA219: %s\n", (result != INA219::Driver::ST_OK)? "FAIL" : "OK");
  chprintf(chp, "----------------------------------------\n");

  // check the fuel gauges
  chprintf(chp, "\n");
  chprintf(chp, "BQ27500:\n");
  chprintf(chp, "\tP7:\n");
  msg_t result_bq27500_p7 = global.bq27500[BAT_P7].selftest();
  chprintf(chp, "->\tP7: ");
  if (result == BaseSensor<>::NOT_IMPLEMENTED)
    chprintf(chp, "not implemented");
  else if (result_bq27500_p7 == BQ27500::Driver::ST_ABORT_NO_BAT)
    chprintf(chp, "ABORT (no battery detected)");
  else if (result_bq27500_p7 != BQ27500::Driver::ST_OK)
    chprintf(chp, "FAIL (error code 0x%02X)", result);
  else
    chprintf(chp, "OK");

  chprintf(chp, "\n\n");
  chprintf(chp, "\tP8:\n");
  msg_t result_bq27500_p8 = global.bq27500[BAT_P8].selftest();
  chprintf(chp, "->\tP8: ");
  if (result == BaseSensor<>::NOT_IMPLEMENTED)
    chprintf(chp, "not implemented");
  else if (result_bq27500_p8 == BQ27500::Driver::ST_ABORT_NO_BAT)
      chprintf(chp, "ABORT (no battery detected)");
  else if (result_bq27500_p8 != BQ27500::Driver::ST_OK)
    chprintf(chp, "FAIL (error code 0x%02X)", result);
  else
    chprintf(chp, "OK");

  chprintf(chp, "\n");
  result = result_bq27500_p7 | result_bq27500_p8;
  if (result == BaseSensor<>::NOT_IMPLEMENTED)
    chprintf(chp, "\n->\tBQ27500: not implemented\n");
  else
    chprintf(chp, "\n->\tBQ27500: %s\n", (result != BQ27500::Driver::ST_OK)? "FAIL" : "OK");
  chprintf(chp, "----------------------------------------\n");

  // check the chargers
  chprintf(chp, "\n");
  chprintf(chp, "BQ24103A:\n");
  if (!global.ltc4412.isPluggedIn())
    chprintf(chp, "This test is skipped. Rerun when plugged in.\n");
  else {
    bool status1, status2, status3;
    chprintf(chp, "\tP7:\n");
    bool status_p7 = global.bq27500[BAT_P7].isBatteryGood();
    chprintf(chp, "Battery good: %s\n", (status_p7? "yes" : "no"));
    if (!status_p7) {
      chprintf(chp, "-> Rerun test with (another) battery!\n");
      status_p7 = true;
    } else {
      status1 = global.bq24103a[BAT_P7]->isCharging();
      chprintf(chp, "status:%scharging\n", (status1? " " : " not "));
      chprintf(chp, "%sabling charger...\n", (status1? "dis" : "en"));
      global.bq24103a[BAT_P7]->enable(!status1);
      BaseThread::sleep(MS2ST(1500));
      status2 = global.bq24103a[BAT_P7]->isCharging();
      chprintf(chp, "status:%scharging\n", (status2? " " : " not "));
      chprintf(chp, "%sabling charger...\n", (!status1? "dis" : "en"));
      global.bq24103a[BAT_P7]->enable(status1);
      BaseThread::sleep(MS2ST(1500));
      status3 = global.bq24103a[BAT_P7]->isCharging();
      chprintf(chp, "status:%scharging\n", (status3? " " : " not "));
      status_p7 = status2 != status1 && status3 == status1;
      chprintf(chp, "->\t");
      if (status_p7) {
        chprintf(chp, "OK");
      } else {
        chprintf(chp, "FAIL");
      }
      chprintf(chp, "\n");
    }
    chprintf(chp, "\n");
    chprintf(chp, "\tP8:\n");
    bool status_p8 = global.bq27500[BAT_P8].isBatteryGood();
    chprintf(chp, "Battery good: %s\n", (status_p8? "yes" : "no"));
    if (!status_p8) {
      chprintf(chp, "-> Rerun test with (another) battery!\n");
      status_p8 = true;
    } else {
      status1 = global.bq24103a[BAT_P8]->isCharging();
      chprintf(chp, "status:%scharging\n", (status1? " " : " not "));
      chprintf(chp, "%sabling charger...\n", (status1? "dis" : "en"));
      global.bq24103a[BAT_P8]->enable(!status1);
      BaseThread::sleep(MS2ST(1500));
      status2 = global.bq24103a[BAT_P8]->isCharging();
      chprintf(chp, "status:%scharging\n", (status2? " " : " not "));
      chprintf(chp, "%sabling charger...\n", (!status1? "dis" : "en"));
      global.bq24103a[BAT_P8]->enable(status1);
      BaseThread::sleep(MS2ST(1500));
      status3 = global.bq24103a[BAT_P8]->isCharging();
      chprintf(chp, "status:%scharging\n", (status3? " " : "not "));
      status_p8 = status2 != status1 && status3 == status1;
      chprintf(chp, "->\t");
      if (status_p8)
        chprintf(chp, "OK");
      else
        chprintf(chp, "FAIL");
      chprintf(chp, "\n");
    }
    chprintf(chp, "\n");
    chprintf(chp, "->\tBQ24103A: ");
    if (status_p7 && status_p8)
      chprintf(chp, "OK");
    else
      chprintf(chp, "FAIL");
    chprintf(chp, "\n");
  }
  chprintf(chp, "----------------------------------------\n");

  // check Bluetooth (TODO: move this check to driver)
  chprintf(chp, "\n");
  chprintf(chp, "WT12-A-AI:\n");

  chprintf(chp, "testing for MUX mode:\t");

  if (global.wt12.bluetoothIsMuxMode()) {
    chprintf(chp, "PASSED\n");
  } else {
    chprintf(chp, "FAILED -> setting MUX mode now\n");
    /* initialise the WT-12 bluetooth chip on AMIRO (Please, run this processes once) */
    global.wt12.bluetoothSendCommand("SET BT AUTH *");
    global.wt12.bluetoothSendCommand("SET BT PAIR *");
    global.wt12.bluetoothSendCommand("SET BT SSP 3 0");
    global.wt12.bluetoothEnableMux();
    global.wt12.bluetoothReset();
  }

  global.wt12.bluetoothSendCommand("TEMP");
  global.wt12.bluetoothSendCommand("SET");

  chprintf(chp, "----------------------------------------\n");

  // check the buzzer
  chprintf(chp, "\n");
  chprintf(chp, "PKLCS1212E4001:\n");
  chprintf(chp, "you should hear the buzzer for one second...\n");
  pwmEnableChannel(&PWMD3, 1, 50);
  BaseThread::sleep(MS2ST(1000));
  pwmDisableChannel(&PWMD3, 1);
  chprintf(chp, "----------------------------------------\n");

  chprintf(chp, "CHECK: FINISH\n");
#endif

  return;
}

void shellRequestShutdown(BaseSequentialStream* chp, int __unused argc, char __unused *argv[]) {

  chprintf(chp, "shellRequestShutdown\n");

  /* if no argument was given, print some help text */
  if (argc == 0 || strcmp(argv[0],"help") == 0) {
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
  if (argc >= 3) {
    num_elements = atoi(argv[2]);
  }

  const size_t eeprom_size = EEPROM::getsize(&global.at24c01);
  uint8_t buffer[eeprom_size];
  if (start_byte + (type_size * num_elements) > eeprom_size) {
    num_elements = (eeprom_size - start_byte) / type_size;
    chprintf(chp, "Warning: request exceeds eeprom size -> limiting to %u values.\n", num_elements);
  }

  chFileStreamSeek((BaseFileStream*)&global.at24c01, start_byte);
  uint32_t bytes_read = chSequentialStreamRead((BaseFileStream*)&global.at24c01, buffer, type_size*num_elements);

  if (bytes_read != type_size*num_elements) {
    chprintf(chp, "Warning: %u of %u requested bytes were read.\n", bytes_read, type_size*num_elements);
  }

  for (unsigned int i = 0; i < num_elements; ++i)
  {
    switch (type)
    {
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

void shellRequestSetVcnlOffset(BaseSequentialStream *chp, int argc, char *argv[]) {
  chprintf(chp, "shellRequestSetVcnlOffset\n");
  if (argc != 2) {
    chprintf(chp, "Usage: %s\n","set_vcnl <idx> <offset>");
    return;
  }

  uint8_t vcnlIdx = static_cast<uint8_t>(atoi(argv[0]));
  uint16_t vcnlOffset = static_cast<uint16_t>(atoi(argv[1]));

  if (vcnlIdx >= global.vcnl4020.size()) {
    chprintf((BaseSequentialStream *)&SD1, "Wrong VCNL index: Choose [0 .. %d]\n", global.vcnl4020.size()-1);
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
  for (uint8_t idx = 0; idx < 8; ++idx) {
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
    chprintf(chp, "Usage: %s\n","get_vcnl_offset <idx>");
    return;
  }

  uint8_t vcnlIdx = static_cast<uint8_t>(atoi(argv[0]));

  if (vcnlIdx >= global.vcnl4020.size()) {
    chprintf((BaseSequentialStream *)&SD1, "Wrong VCNL index: Choose [0 .. %d]\n", global.vcnl4020.size()-1);
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

void shellRequestCheck(BaseSequentialStream *chp, int __unused argc, char __unused *argv[]) {
  chprintf(chp, "shellRequestCheck\n");
  boardPeripheryCheck(chp);

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

void shellRequestCalib(BaseSequentialStream *chp, int __unused argc, char __unused *argv[]) {
  chprintf(chp, "shellRequestCalib\n");
  global.robot.calibrate();

}

void shellRequestGetRobotId(BaseSequentialStream *chp, int __unused argc, char __unused *argv[]) {
  chprintf(chp, "shellRequestGetRobotId\n");
  chprintf(chp, "Robot ID: %u\n", global.robot.getRobotID());
  if (global.robot.getRobotID() == 0) {
    chprintf(chp, "Warning: The ID seems to be uninitialized. Is CAN communication working correctly?\n");
  }
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
  const uint32_t memory_total = 0x1C000;
  const uint32_t memory_load = memory_total - chCoreStatus();
  chprintf(chp, "RAM load: %3.2f%% (%u / %u Byte)\n", float(memory_load)/float(memory_total) * 100, memory_load, memory_total);
}

void shellRequestSetCharging(BaseSequentialStream *chp, int argc, char *argv[]) {
  chprintf(chp, "shellRequestSetCharging\n");
  if (argc < 1) {
    chprintf(chp, "Usage: %s\n","set_charging <enable>");
    chprintf(chp, "\n");
    chprintf(chp, "\tenable\n");
    chprintf(chp, "Whether to enable (1) or to disable (0) the power path controller of the DiWheelDrive board.\n");
    chprintf(chp, "\n");
    return;
  }

  const bool enable = atoi(argv[0]);
  chprintf(chp, "%s power path controller\n", enable ? "enabling" : "disabling");
  global.robot.getPowerStatus().charging_flags.content.diwheeldrive_enable_power_path = enable;

  return;
}

void shellRequestPrintVCNL(BaseSequentialStream *chp, int argc, char *argv[]) {
  chprintf(chp, "shellRequestPrintVCNL\n");
  if (argc < 1) {
    chprintf(chp, "Warning: no arguments specified.\n");
    chprintf(chp, "Using default values (prints one measurement).\n");
    chprintf(chp, "Type 'print_vcnl help' for help.\n");
    chprintf(chp, "----------------------------------------\n");
  }

  if (strcmp(argv[0],"help") == 0) {
    chprintf(chp, "Usage: %s\n","print_vcnl [<seconds>] [<frequency>]");
    chprintf(chp, "\n");
    chprintf(chp, "\tseconds\n");
    chprintf(chp, "Number of seconds to print the VCNL values. (default: 1)\n");
    chprintf(chp, "\n");
    chprintf(chp, "\tfrequency\n");
    chprintf(chp, "Number prints per second. (default: 1)\n");
    return;
  }

  const uint32_t seconds = (argc >= 1) ? atoi(argv[0]) : 1;
  const uint32_t freq = (argc >= 2) ? atoi(argv[1]) : 1;

  uint16_t ambient = 0;
  uint16_t proximity = 0;

  for (uint32_t loop = 0; loop < seconds*freq; ++loop) {
    // Print proximity and ambilight values
    for (uint8_t i = 0x00; i < global.vcnl4020.size(); ++i) {
      ambient = global.vcnl4020[i].getAmbientLight();
      proximity = global.vcnl4020[i].getProximity();
      chprintf(chp, "#%d: ambient=0x%04X\tproximity=0x%04X\n", i, ambient, proximity);
    }
    chprintf(chp, "----------------------------------------\n");

    BaseThread::sleep(US2ST(1000000 / freq));
  }

  return;
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

void shellRequestWiiSteering(BaseSequentialStream* chp, int argc, char *argv[]) {
  // if Wii steering is currently active, stop it
  if (global.userThread.getCurrenState() == UserThread::WII_STEERING) {
    global.userThread.setNextState(UserThread::IDLE);
  }
  // check arguments and (try to) enable Wii steering
  else {
    // if arguments invalid
    if (argc < 1 || argc > 2 || global.userThread.setWiiAddress(argv[0]) != RDY_OK) {
      chprintf(chp, "Warning: invalid arguments\n");
      chprintf(chp, "Usage: %s\n", "wii_steering <address> [<deadzone>]");
      chprintf(chp, "\n");
      chprintf(chp, "\taddress\n");
      chprintf(chp, "bluetooth address of the Wiimote controller to pair with.\n");
      chprintf(chp, "\tdeadzone\n");
      chprintf(chp, "deadzone to set for the Wiimote controller [default = 10%%].\n");
      return;
    }
    else {
      // set deadzone
      const float deadzone = global.userThread.setWiiDeadzone((argc == 2) ? std::atof(argv[1]) : 0.1f);
      chprintf(chp, "deadzone set to %u%%\n", (unsigned int)(deadzone * 100.0f));

      // start Wii steering behaviour
      global.userThread.setNextState(UserThread::WII_STEERING);
    }
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
  {"get_vcnl", shellRequestGetVcnl},
  {"calib_vcnl_offset", shellRequestCalib},
  {"set_vcnl_offset", shellRequestSetVcnlOffset},
  {"reset_vcnl_offset", shellRequestResetVcnlOffset},
  {"get_vcnl_offset", shellRequestGetVcnlOffset},
  {"get_robot_id", shellRequestGetRobotId},
  {"get_system_load", shellRequestGetSystemLoad},
  {"set_charging", shellRequestSetCharging},
  {"print_vcnl", shellRequestPrintVCNL},
  {"shell_board", shellSwitchBoardCmd},
  {"get_bootloader_info", shellRequestGetBootloaderInfo},
  {"wii_steering", shellRequestWiiSteering},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *) &global.sercanmux1,
  commands
};

void charger_logic() {
  /*
   * if supply connected, activate charger, else
   * deactivate charger
   */
  if (global.ltc4412.isPluggedIn()) {
    boardChargerSetState(0x03u, 1), chprintf((BaseSequentialStream*) &SD1, "Charging.\n");
  } else {
    boardChargerSetState(0x03u, 0), chprintf((BaseSequentialStream*) &SD1, "Not charging.\n");
  }
}

void init_powermonitor(INA219::Driver &ina219, const float shunt_resistance_O, const float max_expected_current_A, const uint16_t current_lsb_uA)
{
  INA219::CalibData calib_data;
  INA219::InitData init_data;

  calib_data.input.configuration.content.brng = INA219::Configuration::BRNG_16V;
  calib_data.input.configuration.content.pg = INA219::Configuration::PGA_40mV;
  calib_data.input.configuration.content.badc = INA219::Configuration::ADC_68100us;
  calib_data.input.configuration.content.sadc = INA219::Configuration::ADC_68100us;
  calib_data.input.configuration.content.mode = INA219::Configuration::MODE_ShuntBus_Continuous;
  calib_data.input.shunt_resistance_O = shunt_resistance_O;
  calib_data.input.max_expected_current_A = max_expected_current_A;
  calib_data.input.current_lsb_uA = current_lsb_uA;
  if (ina219.calibration(&calib_data) != BaseSensor<>::SUCCESS)
    chprintf((BaseSequentialStream*)&SD1, "WARNING: calibration of INA219 failed.\n");

  init_data.configuration.value = calib_data.input.configuration.value;
  init_data.calibration = calib_data.output.calibration_value;
  init_data.current_lsb_uA = calib_data.output.current_lsb_uA;
  if (ina219.init(&init_data) != BaseSensor<>::SUCCESS)
    chprintf((BaseSequentialStream*)&SD1, "WARNING: initialization of INA219 failed.\n");

  if (calib_data.input.current_lsb_uA != init_data.current_lsb_uA)
  {
    chprintf((BaseSequentialStream*)&SD1, "NOTE: LSB for current measurement was limited when initializing INA219 (%u -> %u)", calib_data.input.current_lsb_uA, init_data.current_lsb_uA);
  }

  return;
}

/*
 * Application entry point.
 */
int main(void) {

  Thread *shelltp = NULL;

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
   * TODO: detect the reason why the system was started by reading the 1st backup register of the RTC.
   *       To do that, it is probably required to extend the RTC LLD by the required methods.
   *       For further details, how to interprete the data in the backu register, please have a look at the bootloader project.
   */

  /*
   * Activates the serial driver 1 using the driver default configuration.
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

//  boardWriteSystemPower(1);
//  boardWriteIoPower(1);
//  boardWriteWarmRestart(0);

  global.robot.start(HIGHPRIO - 1);

  pwmStart(&PWMD3, &global.pwm3_config);
  pwmDisableChannel(&PWMD3, 1);

  adcStart(&ADCD1, NULL);

  extStart(&EXTD1, &extcfg);

  boardClearI2CBus(GPIOB_GAUGE_SCL1, GPIOB_GAUGE_SDA1);
  boardClearI2CBus(GPIOB_GAUGE_SCL2, GPIOB_GAUGE_SDA2);

  global.HW_I2C1.start(&global.i2c1_config);
  global.HW_I2C2.start(&global.i2c2_config);

  uint16_t i2c_test = 0;
  while (global.ina219[INA_VIO18].readRegister(INA219::Driver::REG_BUS_VOLTAGE, i2c_test) != RDY_OK) {
    chprintf((BaseSequentialStream*)&global.sercanmux1, "I2C #1 stalled! trying to clear the bus... (this will take 20 seconds)\n");
    boardWriteLed(1);
    boardResetBQ27500I2C(GPIOB_GAUGE_SCL2, GPIOB_GAUGE_SDA2);
    boardWriteLed(0);
  }
  while (global.ina219[INA_VDD].readRegister(INA219::Driver::REG_BUS_VOLTAGE, i2c_test) != RDY_OK) {
    chprintf((BaseSequentialStream*)&global.sercanmux1, "I2C #2 stalled! trying to clear the bus... (this will take 20 seconds)\n");
    boardWriteLed(1);
    boardResetBQ27500I2C(GPIOB_GAUGE_SCL1, GPIOB_GAUGE_SDA1);
    boardWriteLed(0);
  }

  global.memory.init();
  uint8_t i = 0;
  if (global.memory.getBoardId(&i) == fileSystemIo::FileSystemIoBase::OK) {
    chprintf((BaseSequentialStream*) &SD1, "Board ID: %u\n", i);
  } else {
    chprintf((BaseSequentialStream*) &SD1, "Error reading board ID\n");
  }
  chprintf((BaseSequentialStream*) &SD1, "\n");

  shelltp = shellCreate(&shell_cfg1, THD_WA_SIZE(1024), NORMALPRIO);

  // initialize the power monitors
  init_powermonitor(global.ina219[INA_VDD], 0.1f, 0.075f, 10);
  init_powermonitor(global.ina219[INA_VIO18], 0.01f, 1.5f, 100);
  init_powermonitor(global.ina219[INA_VIO33], 0.01f, 1.5f, 100);
  init_powermonitor(global.ina219[INA_VIO42], 0.01f, 1.5f, 100);
  init_powermonitor(global.ina219[INA_VIO50], 0.01f, 1.5f, 100);

  // start the ADC watchdog
  global.adc1_vsys.start(NORMALPRIO);

  // start the ina threads
  for (i = 0; i < global.ina219.size(); ++i)
    global.ina219[i].start(NORMALPRIO);

//  // calibrate the fuel gauges (TODO)
//  BQ27500::CalibData bq27500_calib_data;
//  bq27500_p7.calibration(&bq27500_calib_data);

  // start the fuel gauge threads
  for (i = 0; i < global.bq27500.size(); ++i)
    global.bq27500[i].start(NORMALPRIO);

  // start the proximity sensor threads
  for (i = 0; i < global.vcnl4020.size(); ++i) {
    uint16_t buffer;
    global.memory.getVcnl4020Offset(&buffer,i);
    global.vcnl4020[i].setProximityOffset(buffer);
    global.vcnl4020[i].start(NORMALPRIO);
  }

  /* Start uart port connecting bluetooth chip */
  global.wt12.bluetoothStart();

  global.mpr121.configure(&global.mpr121_run_config);
  global.mpr121.start(NORMALPRIO);

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
      chThdRelease(shelltp); /* Recovers memory of the previous shell. */
      shelltp = NULL; /* Triggers spawning of a new shell.      */
    }

    // Let the LED just blink as an alive signal
    boardWriteLed(1);
    BaseThread::sleep(MS2ST(250));
    boardWriteLed(0);
    BaseThread::sleep(MS2ST(250));

    /*
     * Charger logic before shutdown logic,
     * so we don't need to call it twice.
     */
    if (pathdc_change) {
      pathdc_change = 0x00u;
//      charger_logic();
    }

    /*
     * Shutdown logic before user logic,
     * so user does not get hopes up and
     * prepares wheels to drive into the
     * next wall or something like that.
     */
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
            chprintf((BaseSequentialStream*) &SD1, "initiating shutdown sequence...\n");
            palWritePad(GPIOC, GPIOC_SYS_INT_N, PAL_LOW);
            palWritePad(GPIOC, GPIOC_SYS_PD_N, PAL_LOW);

            chprintf((BaseSequentialStream*) &SD1, "stopping all threads and periphery...");
            systemStop();
            chprintf((BaseSequentialStream*) &SD1, "\tdone\n");
            BaseThread::sleep(MS2ST(10)); // sleep to print everything

            blCallback();
          }

        } else {
          chprintf((BaseSequentialStream*) &SD1, "ERROR: invalid shutdown requested (%u).\n", shutdown_now);
          shutdown_now = SHUTDOWN_NONE;
        }
      }

//      shutdown_now = 0x00u;
//      if (palReadPad(GPIOC, GPIOC_PATH_DC))
//        systemStop();
//      else
//        systemShutdown();
//      //boardStop(0x00u, 0x00u);
//      //chprintf((BaseSequentialStream*) &SD1, "Stop exit\n");
    }

  }

  global.HW_I2C2.stop();
  global.HW_I2C1.stop();

  return 0;
}
