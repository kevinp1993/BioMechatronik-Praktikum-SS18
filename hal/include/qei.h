/**
 * @file    qei.h
 * @brief   QEI Driver macros and structures.
 *
 * @addtogroup QEI
 * @{
 */

#ifndef _QEI_H_
#define _QEI_H_

#if HAL_USE_QEI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  QEI_UNINIT = 0,                   /**< Not initialized.                   */
  QEI_STOP = 1,                     /**< Stopped.                           */
  QEI_READY = 2,                    /**< Ready.                             */
  QEI_ACTIVE = 4,                   /**< Active.                            */
} qeistate_t;

/**
 * @brief   Type of a structure representing an QEI driver.
 */
typedef struct QEIDriver QEIDriver;

#include "qei_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Enables the quadrature encoder.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @iclass
 */
#define qeiEnableI(qeip) qei_lld_enable(qeip)

/**
 * @brief   Disables the quadrature encoder.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @iclass
 */
#define qeiDisableI(qeip) qei_lld_disable(qeip)

/**
 * @brief   Returns the direction of the last transition.
 * @details The direction is defined as boolean and is
 *          calculated at each transition on any input.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 * @return              The request direction.
 * @retval FALSE        Position counted up.
 * @retval TRUE         Position counted down.
 * @iclass
 */
#define qeiGetDirectionI(qeip) qei_lld_get_direction(qeip)

/**
 * @brief   Returns the position of the encoder.
 * @details The position is defined as number of pulses since last reset.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 * @return              The number of pulses.
 *
 * @iclass
 */
#define qeiGetPositionI(qeip) qei_lld_get_position(qeip)

/**
 * @brief   Returns the range of the encoder.
 * @details The range is defined as number of maximum pulse count.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 * @return              The number of pulses.
 *
 * @iclass
 */
#define qeiGetRangeI(qeip) qei_lld_get_range(qeip)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void qeiInit(void);
  void qeiObjectInit(QEIDriver *qeip);
  void qeiStart(QEIDriver *qeip, const QEIConfig *config);
  void qeiStop(QEIDriver *qeip);
  void qeiEnable(QEIDriver *qeip);
  void qeiDisable(QEIDriver *qeip);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_QEI */

#endif /* _QEI_H_ */

/** @} */
