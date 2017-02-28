#include "ch.h"
#include "hal.h"

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config =
{
  {VAL_GPIOAODR, VAL_GPIOACRL, VAL_GPIOACRH},
  {VAL_GPIOBODR, VAL_GPIOBCRL, VAL_GPIOBCRH},
  {VAL_GPIOCODR, VAL_GPIOCCRL, VAL_GPIOCCRH},
  {VAL_GPIODODR, VAL_GPIODCRL, VAL_GPIODCRH},
  {VAL_GPIOEODR, VAL_GPIOECRL, VAL_GPIOECRH},
  {VAL_GPIOFODR, VAL_GPIOFCRL, VAL_GPIOFCRH},
  {VAL_GPIOGODR, VAL_GPIOGCRL, VAL_GPIOGCRH},
};

#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {

  stm32_clock_init();
}

/*
 * Board-specific initialization code.
 */
void boardInit(void) {
  /*
   * Several I/O pins are re-mapped:
   *   JTAG disabled and SWD enabled
   */
  AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE |
               AFIO_MAPR_USART3_REMAP_PARTIALREMAP;
}

inline void boardRequestShutdown(void)
{
  palClearPad(GPIOC, GPIOC_SYS_PD_N);
}

inline void boardStandby(void)
{

  palSetPad(GPIOC, GPIOC_SYS_PD_N);
  chSysLock();
  // Standby
  // set deepsleep bit
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  // set PDDS, clear WUF, clear SBF
  PWR->CR |= (PWR_CR_CWUF | PWR_CR_PDDS | PWR_CR_CSBF);
  // clear RTC wakeup source flags
  RTC->CRL &= ~(RTC_CRL_ALRF);
  // Wait for Interrupt
  __WFI();

}

inline void boardClearI2CBus(const uint8_t scl_pad, const uint8_t sda_pad) {

  uint8_t i;

  // configure I²C SCL and SDA open drain
  palSetPadMode(GPIOB, scl_pad, PAL_MODE_OUTPUT_OPENDRAIN);
  palSetPadMode(GPIOB, sda_pad, PAL_MODE_OUTPUT_OPENDRAIN);

  // perform a 2-wire software reset for the eeprom (see AT24C01BN-SH-B datasheet, chapter 3)
  // note: clock is ~50kHz (20us per cycle)
  palSetPad(GPIOB, sda_pad);
  palClearPad(GPIOB, scl_pad);
  chThdSleepMicroseconds(10);
  palSetPad(GPIOB, scl_pad);
  chThdSleepMicroseconds(5);
  palClearPad(GPIOB, sda_pad);
  chThdSleepMicroseconds(5);
  palClearPad(GPIOB, scl_pad);
  chThdSleepMicroseconds(5);
  palSetPad(GPIOB, sda_pad);
  chThdSleepMicroseconds(5);
  for (i = 0; i < 9; ++i) {
    palSetPad(GPIOB, scl_pad);
    chThdSleepMicroseconds(10);
    palClearPad(GPIOB, scl_pad);
    chThdSleepMicroseconds(10);
  }
  palSetPad(GPIOB, scl_pad);
  chThdSleepMicroseconds(5);
  palClearPad(GPIOB, sda_pad);
  chThdSleepMicroseconds(5);
  palClearPad(GPIOB, scl_pad);
  chThdSleepMicroseconds(10);
  palSetPad(GPIOB, scl_pad);
  chThdSleepMicroseconds(5);
  palSetPad(GPIOB, sda_pad);
  chThdSleepMicroseconds(5);
  palClearPad(GPIOB, scl_pad);
  chThdSleepMicroseconds(10);

  // perform bus clear as per I²C Specification v6 3.1.16
  // note: clock is 100kHz (10us per cycle)
  for (i = 0; i < 10; i++) {
    palClearPad(GPIOB, scl_pad);
    chThdSleepMicroseconds(5);
    palSetPad(GPIOB, scl_pad);
    chThdSleepMicroseconds(5);
  }

  // reconfigure I²C SCL
  palSetPadMode(GPIOB, scl_pad, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
  palSetPadMode(GPIOB, sda_pad, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);

  return;
}
