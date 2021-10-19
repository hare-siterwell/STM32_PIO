/**
 * @file bsp_usart.h
 * @brief This file contains all the function prototypes for
 *        the bsp_usart.c file
 */

#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "includes.h"

#define USART_RXSIZE 1024
struct UsartRx {
  OS_SEM sta;           // Received flag
  u16 len;              // Received length
  u8 buf[USART_RXSIZE]; // Received buffer
};
extern struct UsartRx lur1, ur3;

void lpuart1_task(void *p_arg);
void usart3_task(void *p_arg);

void USART_Enable(void);
void USART_RxIdleCallback(USART_TypeDef *USARTx);
void USART_ReEnable(USART_TypeDef *USARTx);
void USART_Send(USART_TypeDef *USARTx, u8 *pData, u32 Size);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_USART_H__ */
