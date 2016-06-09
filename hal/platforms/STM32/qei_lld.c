/**
 * @file    STM32/qei_lld.c
 * @brief   STM32 QEI subsystem low level driver.
 *
 * @addtogroup QEI
 * @{
 */

#include "ch.h"
#include "hal.h"

#include "qei.h"

#if HAL_USE_QEI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   QEID1 driver identifier.
 * @note    The driver QEID1 allocates the complex timer TIM1 when enabled.
 */
#if STM32_QEI_USE_TIM1 || defined(__DOXYGEN__)
QEIDriver QEID1;
#endif

/**
 * @brief   QEID2 driver identifier.
 * @note    The driver QEID1 allocates the timer TIM2 when enabled.
 */
#if STM32_QEI_USE_TIM2 || defined(__DOXYGEN__)
QEIDriver QEID2;
#endif

/**
 * @brief   QEID3 driver identifier.
 * @note    The driver QEID1 allocates the timer TIM3 when enabled.
 */
#if STM32_QEI_USE_TIM3 || defined(__DOXYGEN__)
QEIDriver QEID3;
#endif

/**
 * @brief   QEID4 driver identifier.
 * @note    The driver QEID4 allocates the timer TIM4 when enabled.
 */
#if STM32_QEI_USE_TIM4 || defined(__DOXYGEN__)
QEIDriver QEID4;
#endif

/**
 * @brief   QEID5 driver identifier.
 * @note    The driver QEID5 allocates the timer TIM5 when enabled.
 */
#if STM32_QEI_USE_TIM5 || defined(__DOXYGEN__)
QEIDriver QEID5;
#endif

/**
 * @brief   QEID8 driver identifier.
 * @note    The driver QEID8 allocates the timer TIM8 when enabled.
 */
#if STM32_QEI_USE_TIM8 || defined(__DOXYGEN__)
QEIDriver QEID8;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level QEI driver initialization.
 *
 * @notapi
 */
void qei_lld_init(void) {

#if STM32_QEI_USE_TIM1
  /* Driver initialization.*/
  qeiObjectInit(&QEID1);
  QEID1.tim = STM32_TIM1;
#endif

#if STM32_QEI_USE_TIM2
  /* Driver initialization.*/
  qeiObjectInit(&QEID2);
  QEID2.tim = STM32_TIM2;
#endif

#if STM32_QEI_USE_TIM3
  /* Driver initialization.*/
  qeiObjectInit(&QEID3);
  QEID3.tim = STM32_TIM3;
#endif

#if STM32_QEI_USE_TIM4
  /* Driver initialization.*/
  qeiObjectInit(&QEID4);
  QEID4.tim = STM32_TIM4;
#endif

#if STM32_QEI_USE_TIM5
  /* Driver initialization.*/
  qeiObjectInit(&QEID5);
  QEID5.tim = STM32_TIM5;
#endif

#if STM32_QEI_USE_TIM8
  /* Driver initialization.*/
  qeiObjectInit(&QEID8);
  QEID8.tim = STM32_TIM8;
#endif
}

/**
 * @brief   Configures and activates the QEI peripheral.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @notapi
 */
void qei_lld_start(QEIDriver *qeip) {
  uint32_t arr, ccer;

  if (qeip->state == QEI_STOP) {
    /* Clock activation and timer reset.*/
#if STM32_QEI_USE_TIM1
    if (&QEID1 == qeip) {
      rccEnableTIM1(FALSE);
      rccResetTIM1();
    }
#endif
#if STM32_QEI_USE_TIM2
    if (&QEID2 == qeip) {
      rccEnableTIM2(FALSE);
      rccResetTIM2();
    }
#endif
#if STM32_QEI_USE_TIM3
    if (&QEID3 == qeip) {
      rccEnableTIM3(FALSE);
      rccResetTIM3();
    }
#endif
#if STM32_QEI_USE_TIM4
    if (&QEID4 == qeip) {
      rccEnableTIM4(FALSE);
      rccResetTIM4();
    }
#endif

#if STM32_QEI_USE_TIM5
    if (&QEID5 == qeip) {
      rccEnableTIM5(FALSE);
      rccResetTIM5();
    }
#endif
#if STM32_QEI_USE_TIM8
    if (&QEID8 == qeip) {
      rccEnableTIM8(FALSE);
      rccResetTIM8();
    }
#endif
  }
  else {
    /* Driver re-configuration scenario, it must be stopped first.*/
    qeip->tim->CR1    = 0;                  /* Timer disabled.              */
    qeip->tim->DIER   = 0;                  /* All IRQs disabled.           */
    qeip->tim->SR     = 0;                  /* Clear eventual pending IRQs. */
    qeip->tim->CCR[0] = 0;                  /* Comparator 1 disabled.       */
    qeip->tim->CCR[1] = 0;                  /* Comparator 2 disabled.       */
    qeip->tim->CNT    = 0;                  /* Counter reset to zero.       */
  }

  /* Timer configuration.*/
  qeip->tim->PSC  = 0;
  arr = qeip->config->range - 1;
  chDbgAssert((arr <= 0xFFFF), "qei_lld_start(), #1", "invalid range");
  qeip->tim->ARR  = arr & 0xFFFF;

  /* CCMR1_CC1S = 01 - CH1 Input on TI1.
     CCMR1_CC2S = 01 - CH2 Input on TI2.*/
  qeip->tim->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

  ccer = 0;
  if (qeip->config->channels[0].mode == QEI_INPUT_INVERTED)
    ccer |= TIM_CCER_CC1P;
  if (qeip->config->channels[1].mode == QEI_INPUT_INVERTED)
    ccer |= TIM_CCER_CC2P;
  qeip->tim->CCER = ccer;

  if (qeip->config->mode == QEI_COUNT_CH1)
    qeip->tim->SMCR  = TIM_SMCR_SMS_1;
  else if (qeip->config->mode == QEI_COUNT_CH2)
    qeip->tim->SMCR  = TIM_SMCR_SMS_0;
  else
    qeip->tim->SMCR  = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
}

/**
 * @brief   Deactivates the QEI peripheral.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @notapi
 */
void qei_lld_stop(QEIDriver *qeip) {

  if (qeip->state == QEI_READY) {
    /* Clock deactivation.*/
    qeip->tim->CR1  = 0;                    /* Timer disabled.              */

#if STM32_QEI_USE_TIM1
    if (&QEID1 == qeip) {
      rccDisableTIM1(FALSE);
    }
#endif
#if STM32_QEI_USE_TIM2
    if (&QEID2 == qeip) {
      rccDisableTIM2(FALSE);
    }
#endif
#if STM32_QEI_USE_TIM3
    if (&QEID3 == qeip) {
      rccDisableTIM3(FALSE);
    }
#endif
#if STM32_QEI_USE_TIM4
    if (&QEID4 == qeip) {
      rccDisableTIM4(FALSE);
    }
#endif
#if STM32_QEI_USE_TIM5
    if (&QEID5 == qeip) {
      rccDisableTIM5(FALSE);
    }
#endif
  }
#if STM32_QEI_USE_TIM8
    if (&QEID8 == qeip) {
      rccDisableTIM8(FALSE);
    }
#endif
}

/**
 * @brief   Enables the quadrature encoder.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @notapi
 */
void qei_lld_enable(QEIDriver *qeip) {

  qeip->tim->CR1  = TIM_CR1_CEN;
}

/**
 * @brief   Disables the quadrature encoder.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @notapi
 */
void qei_lld_disable(QEIDriver *qeip) {

  qeip->tim->CR1  = 0;
}

#endif /* HAL_USE_QEI */

/** @} */
