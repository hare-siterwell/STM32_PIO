/**
 * @file bsp_led.c
 * @brief USART task body
 */

#include "bsp_usart.h"

struct UsartRx lur1, ur1;

/**
 * @brief Enable USART
 */
void USART_Enable(void) {
  LL_LPUART_EnableIT_RXNE(LPUART1);
  LL_LPUART_EnableIT_IDLE(LPUART1);

  LL_DMA_SetPeriphAddress(
      DMA1, LL_DMA_STREAM_0,
      LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE));
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, *(u32 *)ur1.buf);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, USART_RXSIZE);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
  LL_USART_EnableDMAReq_RX(USART1);
  LL_USART_EnableIT_IDLE(USART1);

  LL_DMA_SetPeriphAddress(
      DMA1, LL_DMA_STREAM_0,
      LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT));
  LL_USART_EnableDMAReq_TX(USART1);
}

/**
 * @brief Rx Transfer IDLE callbacks
 * @param USARTx Universal Synchronous Asynchronous Receiver Transmitter
 */
void USART_RxIdleCallback(USART_TypeDef *USARTx) {
  OS_ERR err;
  if (USARTx == LPUART1 && LL_LPUART_IsActiveFlag_RXNE_RXFNE(LPUART1)) {
    lur1.buf[lur1.len++] = LL_LPUART_ReceiveData8(LPUART1);
  } else if (USARTx == LPUART1 && LL_LPUART_IsActiveFlag_IDLE(LPUART1)) {
    LL_LPUART_ClearFlag_IDLE(LPUART1);
    if (lur1.len) {
      OSSemPost(&lur1.sta, OS_OPT_POST_1, &err); // Processing data
    } else {
      USART_ReEnable(LPUART1);
    }
  } else if (USARTx == USART1 && LL_USART_IsActiveFlag_IDLE(USART1)) {
    LL_USART_ClearFlag_IDLE(USART1);
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
    ur1.len = USART_RXSIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_0);
    if (ur1.len) {
      OSSemPost(&ur1.sta, OS_OPT_POST_1, &err); // Processing data
    } else {
      USART_ReEnable(USART1);
    }
  }
}

/**
 * @brief Re-enable DMA mode for reception
 * @param USARTx Universal Synchronous Asynchronous Receiver Transmitter
 */
void USART_ReEnable(USART_TypeDef *USARTx) {
  if (USARTx == LPUART1) {
    memset(lur1.buf, 0, lur1.len);
    lur1.len = 0;
  } else if (USARTx == USART1) {
    memset(ur1.buf, 0, ur1.len);
    ur1.len = 0;
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, USART_RXSIZE);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
  }
}

/**
 * @brief Sends an amount of data in DMA mode
 * @note Pay attention to bdma access address permissions
 * @param USARTx Universal Synchronous Asynchronous Receiver Transmitter
 * @param pData Pointer to data buffer
 * @param Size Amount of data elements
 */
void USART_Send(USART_TypeDef *USARTx, u8 *pData, u32 Size) {
  if (USARTx == USART1) {
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, *(u32 *)pData);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, Size);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
  }
}

/* Support "printf()" */
#ifdef __GNUC__
int _write(int fd, char *pBuffer, int size) {
  for (int i = 0; i < size; i++) {
    while (!LL_LPUART_IsActiveFlag_TXE_TXFNF(LPUART1))
      ;
    LL_LPUART_TransmitData8(LPUART1, pBuffer[i]);
  }
  return size;
}
#else
FILE __stdout;
void _sys_exit(int x) {}
int fputc(int ch, FILE *f) {
  LL_LPUART_TransmitData8(LPUART1, ch);
  while (!LL_LPUART_IsActiveFlag_TXE_TXFNF(LPUART1))
    ;
  return ch;
}
#endif
