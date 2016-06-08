#include <ch.hpp>
#include <hal.h>

#include <board.h>

#include "exti.hpp"

volatile uint32_t shutdown_now = 0x00000000u; // = BL_SHUTDOWN_NONE in main.cpp

EXTConfig extcfg = {

  {
    /* channel 0 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 1 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 2 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 3 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 4 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 5 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 6 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 7 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 8 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 9 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 10 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 11 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 12 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 13 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 14 */
    {
      /* mode */ EXT_MODE_GPIOC | EXT_CH_MODE_AUTOSTART | EXT_CH_MODE_FALLING_EDGE,
      /* cb   */ power_down_cb,
    },
    /* channel 15 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 16 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 17 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 18 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
  }
};

void power_down_cb(EXTDriver *extp, expchannel_t channel) {

  (void) extp;
  (void) channel;
  // make sure we don't accidentally power down when somebody tries to wake us
  if (!palReadPad(GPIOC, GPIOC_SYS_PD_N))
    palWritePad(GPIOD, GPIOD_SYS_INT_N, PAL_LOW); // indicate that the module needs some time to shut down
    shutdown_now = 5; // = SHUTDOWN_HANDLE_REQUEST in main.cpp
}
