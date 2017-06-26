#ifndef __MCU_UART_H__
#define __MCU_UART_H__
#include <stdint.h>
#include <stdio.h>

void mcu_uart_init(int32_t baudRate);
void mcu_uart_enable(void);
void mcu_uart_disable(void);
void mcu_uart_setBaud(int32_t baudRate);
void mcu_uart_setFifoTriggerLevel(int32_t level);
void mcu_uart_enableTXInterrupt(bool isEnable);
int32_t mcu_uart_read(uint8_t* buffer, size_t const size);
int32_t mcu_uart_write(uint8_t const * const buffer, size_t const size);

#endif
