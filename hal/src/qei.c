/**
 * @file    qei.c
 * @brief   QEI Driver code.
 *
 * @addtogroup QEI
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "qei.h"

#if HAL_USE_QEI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   QEI Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void qeiInit(void) {

  qei_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p QEIDriver structure.
 *
 * @param[out] qeip     pointer to the @p QEIDriver object
 *
 * @init
 */
void qeiObjectInit(QEIDriver *qeip) {

  qeip->state  = QEI_STOP;
  qeip->config = NULL;
}

/**
 * @brief   Configures and activates the QEI peripheral.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 * @param[in] config    pointer to the @p QEIConfig object
 *
 * @api
 */
void qeiStart(QEIDriver *qeip, const QEIConfig *config) {

  chDbgCheck((qeip != NULL) && (config != NULL), "qeiStart");

  chSysLock();
  chDbgAssert((qeip->state == QEI_STOP) || (qeip->state == QEI_READY),
              "qeiStart(), #1", "invalid state");
  qeip->config = config;
  qei_lld_start(qeip);
  qeip->state = QEI_READY;
  chSysUnlock();
}

/**
 * @brief   Deactivates the QEI peripheral.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @api
 */
void qeiStop(QEIDriver *qeip) {

  chDbgCheck(qeip != NULL, "qeiStop");

  chSysLock();
  chDbgAssert((qeip->state == QEI_STOP) || (qeip->state == QEI_READY),
              "qeiStop(), #1", "invalid state");
  qei_lld_stop(qeip);
  qeip->state = QEI_STOP;
  chSysUnlock();
}

/**
 * @brief   Enables the quadrature encoder.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @api
 */
void qeiEnable(QEIDriver *qeip) {

  chDbgCheck(qeip != NULL, "qeiEnable");

  chSysLock();
  chDbgAssert(qeip->state == QEI_READY, "qeiEnable(), #1", "invalid state");
  qei_lld_enable(qeip);
  qeip->state = QEI_ACTIVE;
  chSysUnlock();
}

/**
 * @brief   Disables the quadrature encoder.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @api
 */
void qeiDisable(QEIDriver *qeip) {

  chDbgCheck(qeip != NULL, "qeiDisable");

  chSysLock();
  chDbgAssert((qeip->state == QEI_READY) || (qeip->state == QEI_ACTIVE),
              "qeiDisable(), #1", "invalid state");
  qei_lld_disable(qeip);
  qeip->state = QEI_READY;
  chSysUnlock();
}

#endif /* HAL_USE_QEI */

/** @} */
