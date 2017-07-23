#ifndef __MCU_UART_H__
#define __MCU_UART_H__
#include <stdint.h>
#include <stdio.h>


/*
   @brief mcu_uart_init: Uart init function. Here is where we initialize
                         a circular buffer, USART hardware, default USART FIFO
                         configuration, baudrate and enable transmition.
   @param baudRate: Transmition baudRate
*/
void mcu_uart_init(int32_t baudRate);

/*
   @brief mcu_uart_disable: This function disables both, RX and TX interrupts
*/
void mcu_uart_disable(void);

/*
   @brief mcu_uart_setBaud: Set a given TX / RX baudRate.
   @param baudRate: baudrate value to be set
*/
void mcu_uart_setBaud(int32_t baudRate);

/*
   @brief mcu_uart_read: Read a received buffer.
   @param buffer: pointer to a buffer where to store received data.
   @param size: size of the buffer.
   @return amount of bytes read.
*/
int32_t mcu_uart_read(uint8_t* buffer, size_t const size);

/*
   @brief mcu_uart_write: Send a buffer over UART. It will be block up to
                          all buffer is sent
   @param buffer: Pointer to a buffer where information to be sent is stored.
   @param size: Amount of bytes to be sent.
*/
void mcu_uart_write(uint8_t const * const buffer, size_t const size);

#endif
