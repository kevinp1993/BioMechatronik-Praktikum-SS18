#include <ch.h>
#include <hal.h>

/*
 * System_halt
 * error code
 *
 *            - SV#1, misplaced @p chSysDisable().
 *            - SV#2, misplaced @p chSysSuspend()
 *            - SV#3, misplaced @p chSysEnable().
 *            - SV#4, misplaced @p chSysLock().
 *            - SV#5, misplaced @p chSysUnlock().
 *            - SV#6, misplaced @p chSysLockFromIsr().
 *            - SV#7, misplaced @p chSysUnlockFromIsr().
 *            - SV#8, misplaced @p CH_IRQ_PROLOGUE().
 *            - SV#9, misplaced @p CH_IRQ_EPILOGUE().
 *            - SV#10, misplaced I-class function.
 *            - SV#11, misplaced S-class function.
 */

static inline void serialWaitForEmpty(void) {
  while (!((SD1.usart->SR) & USART_SR_TC)) {
    ;
  }
}

void haltErrorCode(void) {
#if CH_DBG_SYSTEM_STATE_CHECK
  char errorCode[26]="\nSystem halt! Error Code:\0";
  int i=0;
  rccEnableUSART1(FALSE);
  nvicEnableVector(USART1_IRQn,CORTEX_PRIORITY_MASK(STM32_SERIAL_USART1_PRIORITY));
  SD1.state = SD_READY;

  SD1.usart->CR1 = 0;
  SD1.usart->CR2 = 0;
  SD1.usart->CR3 = 0;

  SD1.usart->CR1 |= USART_CR1_UE | USART_CR1_TE;
  SD1.usart->CR2 |= USART_CR2_STOP;
  SD1.usart->BRR = STM32_PCLK2 / 115200;

  while(errorCode[i]!='\0') {
    SD1.usart->DR = errorCode[i];
    serialWaitForEmpty();
    i++;
  }
  i=0;
  while(dbg_panic_msg[i]!='\0') {
    SD1.usart->DR = dbg_panic_msg[i];
    serialWaitForEmpty();
    i++;
  }

  SD1.usart->DR = '\n';
  serialWaitForEmpty();
#endif
}
