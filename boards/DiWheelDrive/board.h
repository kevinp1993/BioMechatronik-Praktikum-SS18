#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for AMiRo DiWheelDrive board.
 */

/*
 * Board identifier.
 */
#define BOARD_DI_WHEEL_DRIVE
#define BOARD_NAME              "AMiRo DiWheelDrive"
#define BOARD_VERSION           "1.1"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            0
#define STM32_HSECLK            8000000

/*
 * MCU type as defined in the ST header file stm32f1xx.h.
 */
#define STM32F10X_HD

/*
 * IO pins assignments.
 */
#define GPIOA_WKUP              0
#define GPIOA_LED               1
#define GPIOA_DRIVE_PWM1A       2
#define GPIOA_DRIVE_PWM1B       3
#define GPIOA_MOTION_SCLK       5
#define GPIOA_MOTION_MISO       6
#define GPIOA_MOTION_MOSI       7
#define GPIOA_PROG_RX           9
#define GPIOA_PROG_TX           10
#define GPIOA_CAN_RX            11
#define GPIOA_CAN_TX            12
#define GPIOA_SWDIO             13
#define GPIOA_SWCLK             14
#define GPIOA_DRIVE_PWM2B       15


#define GPIOB_DRIVE_SENSE2      1
#define GPIOB_POWER_EN          2
#define GPIOB_DRIVE_PWM2A       3
#define GPIOB_COMPASS_DRDY      5
#define GPIOB_DRIVE_ENC1A       6
#define GPIOB_DRIVE_ENC1B       7
#define GPIOB_COMPASS_SCL       8
#define GPIOB_COMPASS_SDA       9
#define GPIOB_IR_SCL            10
#define GPIOB_IR_SDA            11
#define GPIOB_IR_INT            12
#define GPIOB_GYRO_DRDY         13
#define GPIOB_SYS_UART_UP       14
#define GPIOB_ACCEL_INT_N       15

#define GPIOC_DRIVE_SENSE1      0
#define GPIOC_SYS_INT_N         1
#define GPIOC_PATH_DCSTAT       3
#define GPIOC_PATH_DCEN         5
#define GPIOC_DRIVE_ENC2B       6
#define GPIOC_DRIVE_ENC2A       7
#define GPIOC_SYS_PD_N          8
#define GPIOC_SYS_REG_EN        9
#define GPIOC_SYS_UART_RX       10
#define GPIOC_SYS_UART_TX       11
#define GPIOC_ACCEL_SS_N        13
#define GPIOC_GYRO_SS_N         14

#define GPIOD_OSC_IN            0
#define GPIOD_OSC_OUT           1
#define GPIOD_SYS_WARMRST_N     2

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 */
#define PIN_MODE_INPUT(n)               (0x4U << (((n) % 8) * 4))
#define PIN_MODE_INPUT_PULLX(n)         (0x8U << (((n) % 8) * 4))
#define PIN_MODE_INPUT_ANALOG(n)        (0x0U << (((n) % 8) * 4))
/* Push Pull output 50MHz */
#define PIN_MODE_OUTPUT_PUSHPULL(n)     (0x3U << (((n) % 8) * 4))
/* Open Drain output 50MHz */
#define PIN_MODE_OUTPUT_OPENDRAIN(n)    (0x7U << (((n) % 8) * 4))
/* Alternate Push Pull output 50MHz */
#define PIN_MODE_ALTERNATE_PUSHPULL(n)  (0xbU << (((n) % 8) * 4))
/* Alternate Open Drain output 50MHz */
#define PIN_MODE_ALTERNATE_OPENDRAIN(n) (0xfU << (((n) % 8) * 4))

/*
 * Port A setup.
 */
#define VAL_GPIOACRL            (PIN_MODE_INPUT(GPIOA_WKUP) | \
                                 PIN_MODE_OUTPUT_OPENDRAIN(GPIOA_LED) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOA_DRIVE_PWM1A) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOA_DRIVE_PWM1B) | \
                                 PIN_MODE_INPUT_PULLX(4) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOA_MOTION_SCLK) | \
                                 PIN_MODE_INPUT_PULLX(GPIOA_MOTION_MISO) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOA_MOTION_MOSI))
#define VAL_GPIOACRH            (PIN_MODE_INPUT_PULLX(8) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOA_PROG_RX) | \
                                 PIN_MODE_INPUT_PULLX(GPIOA_PROG_TX) | \
                                 PIN_MODE_INPUT_PULLX(GPIOA_CAN_RX) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOA_CAN_TX) | \
                                 PIN_MODE_INPUT_PULLX(GPIOA_SWDIO) | \
                                 PIN_MODE_INPUT_PULLX(GPIOA_SWCLK) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOA_DRIVE_PWM2B))
#define VAL_GPIOAODR            0xF7FF /* prevent power over CAN bug */

/*
 * Port B setup.
 */
#define VAL_GPIOBCRL            (PIN_MODE_INPUT_PULLX(0) | \
                                 PIN_MODE_INPUT_ANALOG(GPIOB_DRIVE_SENSE2) | \
                                 PIN_MODE_OUTPUT_PUSHPULL(GPIOB_POWER_EN) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOB_DRIVE_PWM2A) | \
                                 PIN_MODE_INPUT_PULLX(4) | \
                                 PIN_MODE_INPUT_PULLX(GPIOB_COMPASS_DRDY) | \
                                 PIN_MODE_INPUT(GPIOB_DRIVE_ENC1A) | \
                                 PIN_MODE_INPUT(GPIOB_DRIVE_ENC1B))
#define VAL_GPIOBCRH            (PIN_MODE_ALTERNATE_OPENDRAIN(GPIOB_COMPASS_SCL) | \
                                 PIN_MODE_ALTERNATE_OPENDRAIN(GPIOB_COMPASS_SDA) | \
                                 PIN_MODE_ALTERNATE_OPENDRAIN(GPIOB_IR_SCL) | \
                                 PIN_MODE_ALTERNATE_OPENDRAIN(GPIOB_IR_SDA) | \
                                 PIN_MODE_INPUT(GPIOB_IR_INT) | \
                                 PIN_MODE_INPUT_PULLX(GPIOB_GYRO_DRDY) | \
                                 PIN_MODE_OUTPUT_OPENDRAIN(GPIOB_SYS_UART_UP) | \
                                 PIN_MODE_INPUT_PULLX(GPIOB_ACCEL_INT_N))
#define VAL_GPIOBODR            0xFFFB /* initially the motors are not powered */

/*
 * Port C setup.
 */
#define VAL_GPIOCCRL            (PIN_MODE_INPUT_ANALOG(GPIOC_DRIVE_SENSE1) | \
                                 PIN_MODE_OUTPUT_OPENDRAIN(GPIOC_SYS_INT_N) | \
                                 PIN_MODE_INPUT_PULLX(2) | \
                                 PIN_MODE_INPUT(GPIOC_PATH_DCSTAT) | \
                                 PIN_MODE_INPUT_PULLX(4) | \
                                 PIN_MODE_OUTPUT_PUSHPULL(GPIOC_PATH_DCEN) | \
                                 PIN_MODE_INPUT(GPIOC_DRIVE_ENC2B) | \
                                 PIN_MODE_INPUT(GPIOC_DRIVE_ENC2A))
#define VAL_GPIOCCRH            (PIN_MODE_OUTPUT_OPENDRAIN(GPIOC_SYS_PD_N) | \
                                 PIN_MODE_INPUT(GPIOC_SYS_REG_EN) | \
                                 PIN_MODE_INPUT(GPIOC_SYS_UART_RX) | \
                                 PIN_MODE_INPUT(GPIOC_SYS_UART_TX) | \
                                 PIN_MODE_INPUT_PULLX(12) | \
                                 PIN_MODE_OUTPUT_PUSHPULL(GPIOC_ACCEL_SS_N) | \
                                 PIN_MODE_OUTPUT_PUSHPULL(GPIOC_GYRO_SS_N) | \
                                 PIN_MODE_INPUT_PULLX(15))
#define VAL_GPIOCODR            0xFFDD /* initially charging via the pins is disabled and SYSNIN_N indicates that the OS is busy */

/*
 * Port D setup.
 */
#define VAL_GPIODCRL            (PIN_MODE_INPUT(GPIOD_OSC_IN) | \
                                 PIN_MODE_INPUT(GPIOD_OSC_OUT) | \
                                 PIN_MODE_OUTPUT_OPENDRAIN(GPIOD_SYS_WARMRST_N) | \
                                 PIN_MODE_INPUT_PULLX(3) | \
                                 PIN_MODE_INPUT_PULLX(4) | \
                                 PIN_MODE_INPUT_PULLX(5) | \
                                 PIN_MODE_INPUT_PULLX(6) | \
                                 PIN_MODE_INPUT_PULLX(7))
#define VAL_GPIODCRH            0x88888888
#define VAL_GPIODODR            0xFFFF

/*
 * Port E setup.
 */
#define VAL_GPIOECRL            0x88888888 /*  PE7...PE0 */
#define VAL_GPIOECRH            0x88888888 /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFF

/*
 * Port F setup.
 */
#define VAL_GPIOFCRL            0x88888888 /*  PF7...PF0 */
#define VAL_GPIOFCRH            0x88888888 /* PF15...PF8 */
#define VAL_GPIOFODR            0xFFFF

/*
 * Port G setup.
 */
#define VAL_GPIOGCRL            0x88888888 /*  PG7...PG0 */
#define VAL_GPIOGCRH            0x88888888 /* PG15...PG8 */
#define VAL_GPIOGODR            0xFFFF

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
  void boardWriteIoPower(const uint8_t value);
  void boardWriteLed(int value);
  void boardRequestShutdown(void);
  void boardStandby(void);
  void boardWakeup(void);
  void boardClearI2CBus(const uint8_t scl_pad, const uint8_t sda_pad);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
