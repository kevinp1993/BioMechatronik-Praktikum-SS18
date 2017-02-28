#include "ch.h"
#include "hal.h"
#include "board.h"

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config =
{
  {VAL_GPIOA_MODER, VAL_GPIOA_OTYPER, VAL_GPIOA_OSPEEDR, VAL_GPIOA_PUPDR, VAL_GPIOA_ODR, VAL_GPIOA_AFRL, VAL_GPIOA_AFRH},
  {VAL_GPIOB_MODER, VAL_GPIOB_OTYPER, VAL_GPIOB_OSPEEDR, VAL_GPIOB_PUPDR, VAL_GPIOB_ODR, VAL_GPIOB_AFRL, VAL_GPIOB_AFRH},
  {VAL_GPIOC_MODER, VAL_GPIOC_OTYPER, VAL_GPIOC_OSPEEDR, VAL_GPIOC_PUPDR, VAL_GPIOC_ODR, VAL_GPIOC_AFRL, VAL_GPIOC_AFRH},
  {VAL_GPIOD_MODER, VAL_GPIOD_OTYPER, VAL_GPIOD_OSPEEDR, VAL_GPIOD_PUPDR, VAL_GPIOD_ODR, VAL_GPIOD_AFRL, VAL_GPIOD_AFRH},
  {VAL_GPIOE_MODER, VAL_GPIOE_OTYPER, VAL_GPIOE_OSPEEDR, VAL_GPIOE_PUPDR, VAL_GPIOE_ODR, VAL_GPIOE_AFRL, VAL_GPIOE_AFRH},
  {VAL_GPIOF_MODER, VAL_GPIOF_OTYPER, VAL_GPIOF_OSPEEDR, VAL_GPIOF_PUPDR, VAL_GPIOF_ODR, VAL_GPIOF_AFRL, VAL_GPIOF_AFRH},
  {VAL_GPIOG_MODER, VAL_GPIOG_OTYPER, VAL_GPIOG_OSPEEDR, VAL_GPIOG_PUPDR, VAL_GPIOG_ODR, VAL_GPIOG_AFRL, VAL_GPIOG_AFRH},
  {VAL_GPIOH_MODER, VAL_GPIOH_OTYPER, VAL_GPIOH_OSPEEDR, VAL_GPIOH_PUPDR, VAL_GPIOH_ODR, VAL_GPIOH_AFRL, VAL_GPIOH_AFRH},
  {VAL_GPIOI_MODER, VAL_GPIOI_OTYPER, VAL_GPIOI_OSPEEDR, VAL_GPIOI_PUPDR, VAL_GPIOI_ODR, VAL_GPIOI_AFRL, VAL_GPIOI_AFRH}
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

}

inline void boardWriteIoPower(int value)
{
    palWritePad(GPIOA, GPIOA_SYS_REG_EN, value);
    if (value) {
        // drive pins
        palSetPadMode(GPIOA, GPIOA_CAN_TX, PAL_MODE_ALTERNATE(9));
        palSetPadMode(GPIOA, GPIOA_SYS_UART_TX, PAL_MODE_ALTERNATE(7));
        palSetPadMode(GPIOB, GPIOB_BT_CTS, PAL_MODE_ALTERNATE(7));
    } else {
        // float pins
        palSetPadMode(GPIOA, GPIOA_CAN_TX, PAL_MODE_INPUT);
        palSetPadMode(GPIOA, GPIOA_SYS_UART_TX, PAL_MODE_INPUT);
        palSetPadMode(GPIOB, GPIOB_BT_CTS, PAL_MODE_INPUT);
    }
    chThdSleepMilliseconds(50);
}

inline void boardWriteLed(int value)
{
    palWritePad(GPIOB, GPIOB_LED, !value);
}

inline void boardWriteSystemPower(int value)
{
    palWritePad(GPIOB, GPIOB_POWER_EN, value);
    chThdSleepMilliseconds(50);
}

inline void boardWriteWarmRestart(const uint8_t value)
{
    palWritePad(GPIOC, GPIOC_SYS_WARMRST_N, ~value);
    chThdSleepMilliseconds(50);
}

inline void boardChargerSetState(uint8_t chrg_mask, uint8_t state)
{
  if (chrg_mask & (1u << 0))
    palWritePad(GPIOC, GPIOC_CHARGE_EN1_N, ~state);
  if (chrg_mask & (1u << 1))
    palWritePad(GPIOD, GPIOD_CHARGE_EN2_N, ~state);
}

inline void boardBluetoothSetState(uint8_t state)
{
    palWritePad(GPIOC, GPIOC_BT_RST, ~state);
}

inline void boardRequestShutdown(void)
{
  palClearPad(GPIOC, GPIOC_SYS_PD_N);
}

#define RTC_ISR_TAMP2F ((uint32_t)0x00004000)

inline void boardStandby(void)
{

  chSysLock();
  // set deepsleep bit
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  // enable wakeup pin
  PWR->CSR |= PWR_CSR_EWUP;
  // set PDDS, clear WUF, clear SBF
  PWR->CR |= (PWR_CR_CWUF | PWR_CR_PDDS | PWR_CR_CSBF);
  // clear RTC wakeup source flags
  RTC->ISR &= ~(RTC_ISR_ALRBF | RTC_ISR_ALRAF | RTC_ISR_WUTF | RTC_ISR_TAMP1F | RTC_ISR_TAMP2F |
                RTC_ISR_TSOVF | RTC_ISR_TSF);
  // Wait for Interrupt
  __WFI();

}

inline void boardStop(const uint8_t lpds, const uint8_t fpds)
{

  chSysLock();
  // set deepsleep bit
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  // enable wakeup pin
  //PWR->CSR |= PWR_CSR_EWUP;
  // clear PDDS, clear LPDS, clear FPDS
  PWR->CR &= ~(PWR_CR_PDDS | PWR_CR_LPDS | PWR_CR_FPDS);
  // clear WUF, clear SBF
  PWR->CR |= (PWR_CR_CWUF | PWR_CR_CSBF);
  if (lpds)
    PWR->CR |= (PWR_CR_LPDS);
  if (fpds)
    PWR->CR |= (PWR_CR_FPDS);
  // clear RTC wakeup source flags
  RTC->ISR &= ~(RTC_ISR_ALRBF | RTC_ISR_ALRAF | RTC_ISR_WUTF | RTC_ISR_TAMP1F | RTC_ISR_TAMP2F |
                RTC_ISR_TSOVF | RTC_ISR_TSF);
  // clear pending interrupts
  EXTI->PR = ~0;
  // Wait for Interrupt
  __WFI();

}

#undef RTC_ISR_TAMP2F

inline void boardWakeup(void) {

  palClearPad(GPIOC, GPIOC_SYS_PD_N);
  chThdSleepMicroseconds(10);
  palSetPad(GPIOC, GPIOC_SYS_PD_N);
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
  palSetPadMode(GPIOB, scl_pad, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
  palSetPadMode(GPIOB, sda_pad, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

  return;
}

inline void boardResetBQ27500I2C(const uint8_t scl_pad, const uint8_t sda_pad) {

  // configure I²C SCL and SDA open drain
  palSetPadMode(GPIOB, scl_pad, PAL_MODE_OUTPUT_OPENDRAIN);
  palSetPadMode(GPIOB, sda_pad, PAL_MODE_OUTPUT_OPENDRAIN);

  // BQ27500: reset by holding bus low for t_BUSERR (17.3 - 21.2 seconds)
  palClearPad(GPIOB, scl_pad);
  palClearPad(GPIOB, sda_pad);
  chThdSleepSeconds(20);

  boardClearI2CBus(scl_pad, sda_pad);

  return;
}
