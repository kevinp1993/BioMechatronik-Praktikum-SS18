#include <ch.h>
#include <hal.h>

#include <board.h>
#include "wakeup.h"

enum wakeup_mode   wakeup_mode;
enum wakeup_source wakeup_source;

void standby_wakeup_source(void) {

  uint32_t csr = PWR->CSR;
  uint32_t gpioc_val = palReadPort(GPIOC);

  // default to POR
  wakeup_mode = WAKE_MODE_POR;
  wakeup_source = WAKE_SRC_UNKN;

  // make sure reset while powered on looks like POR
  PWR->CR |= (PWR_CR_CSBF);

  if ((csr & (PWR_CSR_SBF | PWR_CSR_WUF)) == (PWR_CSR_SBF | PWR_CSR_WUF)) {

    // wakeup or RTC event from standby mode
    // default to WKUP
    wakeup_mode = WAKE_MODE_STDBY;
    wakeup_source = WAKE_SRC_WKUP;

    if (gpioc_val & (1u << GPIOC_PATH_DC))
      wakeup_source |= WAKE_SRC_PATHDC_CONNECT;

    if (!(gpioc_val & (1u << GPIOC_TOUCH_INT_N)))
      wakeup_source |= WAKE_SRC_TOUCH_INT;

    /*
     * default to SYS_PD_N source when
     * no other reason for wakeup was found.
     */
    if (wakeup_source == WAKE_SRC_WKUP)
      wakeup_source |= WAKE_SRC_SYS_PD_N;

  }

}

void stop_wakeup_source(void) {

  wakeup_mode = WAKE_MODE_STOP;
  wakeup_source = WAKE_SRC_UNKN;

}
