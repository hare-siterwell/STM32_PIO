/**
 * @file app_task2.c
 * @brief Application task body
 */

#include "a4988.h"
#include "bsp_usart.h"

static void verify_buf(uc8 *kBuf, uc8 *kFormatString);

/**
 * @brief LPUART1 communication with PC
 */
void lpuart1_task(void *p_arg) {
  OS_ERR err;
  OSSemCreate(&lur1.sta, "lur1 sta", 0, &err);
  /*
  {
    "step_move": 100000,
    "step_spmax": 1500,
    "step_accel": 1000,
    "divnum": 8,
    "motor_id": 1
  }
  */
  uc8 kFormatString[] = "\
  {\
    \"step_move\": %d,\
    \"step_spmax\": %hu,\
    \"step_accel\": %hu,\
    \"divnum\": %hu,\
    \"motor_id\": %hu\
  }";
  printf("TaskLpuart1 running!\r\n");
  while (1) {
    OSSemPend(&lur1.sta, 0, OS_OPT_PEND_BLOCKING, 0, &err);

    verify_buf(lur1.buf, kFormatString);

    USART_ReEnable(LPUART1);
  }
}

/**
 * @brief USART1 communication with BLE
 */
void usart1_task(void *p_arg) {
  OS_ERR err;
  OSSemCreate(&ur1.sta, "ur1 sta", 0, &err);
  uc8 kFormatString[] = "\
  {\
    \"to\": %d,\
    \"sp\": %hu,\
    \"ac\": %hu,\
    \"su\": %hu,\
    \"mo\": %hu\
  }"; // 蓝牙数据传输受限

  printf("TaskUsart1 running!\r\n");
  while (1) {
    OSSemPend(&ur1.sta, 0, OS_OPT_PEND_BLOCKING, 0, &err);

    verify_buf(ur1.buf, kFormatString);

    USART_ReEnable(USART1);
  }
}

/**
 * @brief 验证数据,驱动步进电机
 * @param kBuf 接收缓冲
 * @param kFormatString 格式字符串
 */
static void verify_buf(uc8 *kBuf, uc8 *kFormatString) {
  s32 step_move = 0;
  u16 motor_id = 0, divnum = 0, step_spmax = 0, step_accel = 0;

  if (sscanf((char *)kBuf, (char *)kFormatString, &step_move, &step_spmax,
             &step_accel, &divnum, &motor_id)) {
    if (motor_id == 1) { // 电机1
      if (motor_move(&motor1, divnum, step_move, step_spmax, step_accel)) {
        printf("Stopped!\r\n");
      } else {
        printf("motorId=%hu,divnum=%hu\r\nstepMove=%d,stepSpmax="
               "%hu,stepAccel=%hu\r\n",
               motor_id, motor1.divnum, motor1.step_move, motor1.step_spmax,
               motor1.step_accel);
      }
    } else {
      printf("Input Error!\r\n");
    }
  } else {
    printf("%s\r\n", kBuf);
  }
}
