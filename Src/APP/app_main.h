/**
 * @file app_main.h
 * @brief This file contains all the function prototypes for
 *        the app_main.c file
 */

#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "a4988.h"
#include "bsp_delay.h"
#include "bsp_usart.h"

void lpuart1_task(void *p_arg);
void usart1_task(void *p_arg);
void lcd_task(void *p_arg);
void led_task(void *p_arg);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MAIN_H */
