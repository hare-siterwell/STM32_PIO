/**
 * @file bsp_app.c
 * @brief Application main task body
 */

#include "bsp_delay.h"
#include "bsp_led.h"
#include "bsp_spi_tftlcd.h"
#include "bsp_tim_a4988.h"
#include "bsp_usart.h"

static OS_TCB Task1TCB;
static CPU_STK Task1Stk[512];
static OS_TCB Task2TCB;
static CPU_STK Task2Stk[512];
static OS_TCB Task3TCB;
static CPU_STK Task3Stk[512];
static OS_TCB Task4TCB;
static CPU_STK Task4Stk[512];

/**
 * @brief The application main task
 */
void app_main(void *p_arg) {
  OS_ERR err;

  DWT_Init();
  USART_Enable();
  LL_TIM_EnableIT_UPDATE(TIM3);
  LL_TIM_EnableCounter(TIM3);
  motor_arg_init();

#if OS_CFG_SCHED_ROUND_ROBIN_EN
  OSSchedRoundRobinCfg(DEF_ENABLED, 0, &err);
#endif

  OSTaskCreate(&Task1TCB, "Task1", lpuart1_task, 0, 3, Task1Stk, 512 / 10, 512,
               0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);

  OSTaskCreate(&Task2TCB, "Task2", usart3_task, 0, 4, Task2Stk, 512 / 10, 512,
               0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);

  OSTaskCreate(&Task3TCB, "Task3", lcd_task, 0, 5, Task3Stk, 512 / 10, 512, 0,
               0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);

  OSTaskCreate(&Task4TCB, "Task4", led_task, 0, 5, Task4Stk, 512 / 10, 512, 0,
               0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
}

uint32_t HAL_GetTick(void) {
  OS_ERR os_err;
  return OSTimeGet(&os_err);
}
