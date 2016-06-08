#ifndef WAKEUP_H_
#define WAKEUP_H_

enum wakeup_mode {

  /**
   * Wakeup POR.
   */
  WAKE_MODE_POR,
  /**
   * Wakeup from stop mode.
   */
  WAKE_MODE_STOP,
  /**
   * Wakeup from standby mode.
   */
  WAKE_MODE_STDBY,

};

enum wakeup_source {

  /**
   * Wakeup source unknown.
   */
  WAKE_SRC_UNKN                 = 0x00000000u,
  /**
   * Wakeup source wakeup pin.
   */
  WAKE_SRC_WKUP                 = 0x00000001u,
  /**
   * Wakeup source DC jack connected.
   */
  WAKE_SRC_PATHDC_CONNECT       = 0x00000002u,
  /**
   * Wakeup source DC jack disconnected.
   */
  WAKE_SRC_PATHDC_DISCONNECT    = 0x00000004u,
  /**
   * Wakeup source touch button interrupt.
   */
  WAKE_SRC_TOUCH_INT            = 0x00000008u,
  /**
   * Wakeup source other board.
   */
  WAKE_SRC_SYS_PD_N             = 0x00000010u,
  /**
   * Wakeup source any RTC event.
   */
  WAKE_SRC_RTC                  = 0x00000060u,
  /**
   * Wakeup source RTC Tamper/Timestamp event.
   */
  WAKE_SRC_RTC_TAMPER_TIMESTAMP = 0x00000020u,
  /**
   * Wakeup source RTC Wakeup.
   */
  WAKE_SRC_RTC_WAKEUP           = 0x00000040u,

};

extern enum wakeup_mode   wakeup_mode;
extern enum wakeup_source wakeup_source;

#ifdef __cplusplus
extern "C" {
#endif
  void standby_wakeup_source(void);
#ifdef __cplusplus
}
#endif

#endif /* WAKEUP_H_ */
