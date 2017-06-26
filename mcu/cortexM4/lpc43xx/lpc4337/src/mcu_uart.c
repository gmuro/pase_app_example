/* Copyright 2014, Pablo Ridolfi (UTN-FRBA)
 * Copyright 2014, Juan Cecconi
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief CIAA UART Driver for LPC4337
 **
 ** Implements the UART Driver for LPC4337
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Drivers CIAA Drivers
 ** @{ */
/** \addtogroup UART UART Drivers
 ** @{ */

/*==================[inclusions]=============================================*/
#include "chip.h"
#include "os.h"
#include "mcu.h"
/*==================[macros and definitions]=================================*/

#define UART_RX_FIFO_SIZE       (16)
#define UART_AVAILABLES 1

typedef struct {
   uint8_t hwbuf[UART_RX_FIFO_SIZE];
   uint8_t rxcnt;
} uartControl_type;

typedef struct
{
   uint8_t port;
   uint8_t pin;
}portPin_type;

typedef struct
{
   portPin_type tx;
   portPin_type rx;
   uint16_t modefunc;
}p_uart_type;

static const p_uart_type p_uart[] =
{
   {{7,1},   {7,2},   FUNC6},       /*TX:{port-pin} RX:{port-pin}  modeFunc*/
};



/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/** \brief Buffers */
uartControl_type uartControl[UART_AVAILABLES];
/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
extern void mcu_uart_enable(void)
{
   /* Restart FIFOS: set Enable, Reset content, set trigger level */
   Chip_UART_SetupFIFOS(LPC_USART2,
                        UART_FCR_FIFO_EN |
                        UART_FCR_TX_RS   |
                        UART_FCR_RX_RS   |
                        UART_FCR_TRG_LEV0);
   /* dummy read */
   Chip_UART_ReadByte(LPC_USART2);
   /* enable rx interrupt */
   Chip_UART_IntEnable(LPC_USART2,
                       UART_IER_RBRINT);
}

extern void mcu_uart_disable()
{
   /* disable tx and rx interrupt */
   Chip_UART_IntDisable(LPC_USART2,
                        UART_IER_THREINT |
                        UART_IER_RBRINT);
}

extern void mcu_uart_setBaud(int32_t baudRate)
{
    Chip_UART_SetBaud(LPC_USART2, (int32_t)baudRate);
}

/* Set FIFO trigger level will set an interrupt after receiving "level"
   characters throgh UART*/
extern void mcu_uart_setFifoTriggerLevel(int32_t level)
{
    Chip_UART_SetupFIFOS(LPC_USART2,
                         UART_FCR_FIFO_EN |
                         UART_FCR_TX_RS   |
                         UART_FCR_RX_RS   |
                         (int32_t)level);
}

//extern void mcu_uart_enableTXInterrupt(bool isEnable)
//{
//    if(isEnable)
//    {
//        /* enable THRE irq (TX) */
//        Chip_UART_IntEnable(LPC_USART2, UART_IER_THREINT);
//    }
//    else
//    {
//        /* disable THRE irq (TX) */
//        Chip_UART_IntDisable(LPC_USART2, UART_IER_THREINT);
//    }
//}

extern void mcu_uart_enableRXInterrupt(bool isEnable)
{
    if(isEnable)
    {
        /* enable RBR irq (RX) */
        Chip_UART_IntEnable(LPC_USART2, UART_IER_RBRINT);
    }
    else
    {
        /* disable RBR irq (RX) */
        Chip_UART_IntDisable(LPC_USART2, UART_IER_RBRINT);
    }
}

extern int32_t mcu_uart_read(uint8_t* buffer, size_t const size)
{
   int32_t ret = -1;
   uint8_t i;

   if(size != 0)
   {
      if(size > uartControl[0].rxcnt)
      {
         /* buffer has enough space */
         ret = uartControl->rxcnt;
         uartControl->rxcnt = 0;
      }
      else
      {
         /* buffer hasn't enough space */
         ret = size;
         uartControl->rxcnt -= size;
      }
      for(i = 0; i < ret; i++)
      {
         buffer[i] = uartControl->hwbuf[i];
      }
      if(uartControl->rxcnt != 0)
      {
         /* We removed data from the buffer, it is time to reorder it */
         for(i = 0; i < uartControl->rxcnt ; i++)
         {
            uartControl->hwbuf[i] = uartControl->hwbuf[i + ret];
         }
      }
   }
   return ret;
}

extern int32_t mcu_uart_write(uint8_t const * const buffer, size_t const size)
{
   int32_t ret = 0;

   while((Chip_UART_ReadLineStatus(LPC_USART2) & UART_LSR_THRE) &&
         (ret < size))
   {
      /* send first byte */
      Chip_UART_SendByte(LPC_USART2,
                         buffer[ret]);
      /* bytes written */
      ret++;
   }
   return ret;
}

void mcu_uart_init(int32_t baudRate)
{
    /* init hardware */
    /* UART2 (USB-UART) */
    Chip_UART_Init(LPC_USART2);
    Chip_UART_SetBaud(LPC_USART2, baudRate);

    Chip_UART_SetupFIFOS(LPC_USART2,
                         UART_FCR_FIFO_EN |
                         UART_FCR_TRG_LEV0);

    Chip_UART_TXEnable(LPC_USART2);
    Chip_SCU_PinMux(p_uart[0].tx.port,
                    p_uart[0].tx.pin,
                    MD_PDN,
                    p_uart[0].modefunc);
    Chip_SCU_PinMux(p_uart[0].rx.port,
                    p_uart[0].rx.pin,
                    MD_PLN|MD_EZI|MD_ZI,
                    p_uart[0].modefunc);
}

/*==================[interrupt handlers]=====================================*/
ISR(UART2_IRQHandler)
{
   uint8_t status = Chip_UART_ReadLineStatus(LPC_USART2);

   if(status & UART_LSR_RDR)
   {
      do
      {
         uartControl[0].hwbuf[uartControl[0].rxcnt] =
                                                Chip_UART_ReadByte(LPC_USART2);
         uartControl[0].rxcnt++;
      }while((Chip_UART_ReadLineStatus(LPC_USART2) & UART_LSR_RDR) &&
             (uartControl[0].rxcnt < UART_RX_FIFO_SIZE));
   }
   if((status & UART_LSR_THRE) &&
      (Chip_UART_GetIntsEnabled(LPC_USART2) &
      UART_IER_THREINT))
   {
      if(Chip_UART_ReadLineStatus(LPC_USART2) &
         UART_LSR_THRE)
      {
         /* There is not more bytes to send, disable THRE irq */
         Chip_UART_IntDisable(LPC_USART2, UART_IER_THREINT);
      }
   }
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

