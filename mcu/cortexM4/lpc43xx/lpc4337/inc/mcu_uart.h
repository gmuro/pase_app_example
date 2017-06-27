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
   @brief mcu_uart_enable: This function enables RX IRQ and reset FIFO
                           configuration. After this function call, RX ISR
                           will be called when we receive at least 1 byte.
                           (FIFO trigger level is set to 0).
*/
void mcu_uart_enable(void);

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
   @brief mcu_uart_setFifoTriggerLevel: Set how much bytes should be received
                                        to trigger RX interrupt.
   @param level: amount of bytes to be received to trigger ISR
*/
void mcu_uart_setFifoTriggerLevel(int32_t level);

/*
   @brief mcu_uart_enableTXInterrupt: Enable / Disable Transmition Interrupt.
                                      Because UART inner buffer is too short,
                                      we need to wait after one transmition
                                      to transmit other element. First
                                      transmition is done on write, and next
                                      ones are done on ISR.
   @param isEnable: true: Enables TX interrupt
                    false: Disable TX interrupt
*/
void mcu_uart_enableTXInterrupt(bool isEnable);

/*
   @brief mcu_uart_read: Read a received buffer.
   @param buffer: pointer to a buffer where to store received data.
   @param size: size of the buffer.
   @return amount of bytes read.
*/
int32_t mcu_uart_read(uint8_t* buffer, size_t const size);

/*
   @brief mcu_uart_write: Send a buffer over UART. This function doesn't send
                          inmediately the information. First, we store the
                          buffer into a circular buffer, we enable TX
                          interrupts and then we send the first byte. After
                          that the function returns, but the transmition is
                          in charge of the ISR, up to circular buffer is empty.
   @param buffer: Pointer to a buffer where information to be sent is stored.
   @param size: Amount of bytes to be sent.
*/
void mcu_uart_write(uint8_t const * const buffer, size_t const size);

#endif
