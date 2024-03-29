/**
 * @file bsp_delay.c
 * @brief delays
 */

#include "bsp_delay.h"

/**
 * @brief Initialize DWT
 * @note It may need to enable access to DWT registers on Cortex-M7
 *       DWT->LAR = 0xC5ACCE55
 */
void DWT_Init(void) {
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}

/**
 * @brief Provide delays in microseconds
 * @param nus Specifies the delay time length in microseconds
 */
void delay_us(u32 nus) {
  uc32 startTick = DWT->CYCCNT, delayTicks = nus * (SystemCoreClock / 1000000);
  while ((u32)(DWT->CYCCNT - startTick) < delayTicks)
    ;
}

/**
 * @brief Provide delays in milliseconds
 * @param nms Specifies the delay time length in milliseconds
 */
void delay_ms(u32 nms) {
  OS_ERR err;
  OSTimeDly(nms, OS_OPT_TIME_PERIODIC, &err);
}
