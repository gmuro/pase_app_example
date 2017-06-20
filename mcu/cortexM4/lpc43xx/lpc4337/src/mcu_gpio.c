/* Copyright 2017, Gustavo Muro
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

/** \brief source para MCU
 **
 ** archivo de inicilización del microcontrolador
 **
 **/

/** \addtogroup PASE_APP_EXAMPLE
 ** @{ */
/** \addtogroup MCU
 ** @{ */

/*==================[inclusions]=============================================*/
#include "mcu.h"
#include "stdint.h"
#include "chip.h"
#include "os.h"

/*==================[macros and definitions]=================================*/

/** \brief cantidad de callback que se pueden registrar */
#define MCU_GPIO_IN_EVENT_TOTAL     8

/** \brief Dio Type */
typedef struct
{
   uint8_t port;
   uint8_t pin;
}portPin_type;

typedef struct
{
   portPin_type p;
   portPin_type gpio;
   uint16_t modefunc;
}p_gpio_type;

typedef struct
{
   mcu_gpio_eventInput_callBack_type cb;
   mcu_gpio_eventTypeInput_enum evType;
   mcu_gpio_pinId_enum pinId;
}eventsInputs_type;

/*==================[internal data declaration]==============================*/

static const p_gpio_type p_gpio[] =
{
   {{2,0},   {5,0},   FUNC4},
   {{2,1},   {5,1},   FUNC3},
   {{2,2},   {5,2},   FUNC4},
   {{2,10},  {0,14},  FUNC0},
   {{2,11},  {1,11},  FUNC0},
   {{2,12},  {1,12},  FUNC0},
   {{1,0},   {0,4},   FUNC0},
   {{1,1},   {0,8},   FUNC0},
   {{1,2},   {0,9},   FUNC0},
   {{1,6},   {1,9},   FUNC0},
};

static eventsInputs_type eventsInputs[MCU_GPIO_IN_EVENT_TOTAL];

static const LPC43XX_IRQn_Type pinIntIrqMap[] =
{
   PIN_INT0_IRQn,
   PIN_INT1_IRQn,
   PIN_INT2_IRQn,
   PIN_INT3_IRQn,
   PIN_INT4_IRQn,
   PIN_INT5_IRQn,
   PIN_INT6_IRQn,
   PIN_INT7_IRQn,
};

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
extern void mcu_gpio_init(void)
{
   int32_t i;

   Chip_Clock_Enable(CLK_MX_GPIO);
   Chip_GPIO_Init(LPC_GPIO_PORT);

   for (i = 0 ; i < MCU_GPIO_IN_EVENT_TOTAL ; i++)
   {
      eventsInputs[i].cb = NULL;
   }
}

extern void mcu_gpio_setDirection(mcu_gpio_pinId_enum id,
                                  mcu_gpio_direction_enum dir)
{
   if(dir == MCU_GPIO_DIRECTION_INPUT)
   {
	   Chip_SCU_PinMux(p_gpio[id].p.port,
	                   p_gpio[id].p.pin,
	                   MD_EZI,
	                   p_gpio[id].modefunc);
   }
   else
   {
	   Chip_SCU_PinMux(p_gpio[id].p.port,
	                   p_gpio[id].p.pin,
	                   MD_PLN | MD_EZI,
	                   p_gpio[id].modefunc);
   }

   Chip_GPIO_SetDir(LPC_GPIO_PORT,
                    p_gpio[id].gpio.port,
                    (1<<p_gpio[id].gpio.pin),
                    ((uint8_t)dir == MCU_GPIO_DIRECTION_OUTPUT));
}

extern void mcu_gpio_toggleOut(mcu_gpio_pinId_enum id)
{
   Chip_GPIO_SetPortToggle(LPC_GPIO_PORT,
                           p_gpio[id].gpio.port,
                           ((uint32_t)1) << (p_gpio[id].gpio.pin));
}

extern void mcu_gpio_setOut(mcu_gpio_pinId_enum id, bool state)
{
   Chip_GPIO_SetPinState(LPC_GPIO_PORT,
                         p_gpio[id].gpio.port,
                         p_gpio[id].gpio.pin,
                         state);
}

extern bool mcu_gpio_readPin(mcu_gpio_pinId_enum id)
{
   return Chip_GPIO_GetPinState(LPC_GPIO_PORT,
                                p_gpio[id].gpio.port,
                                p_gpio[id].gpio.pin);
}

extern int32_t mcu_gpio_setEventInput(mcu_gpio_pinId_enum id,
                                      mcu_gpio_eventTypeInput_enum evType,
                                      mcu_gpio_eventInput_callBack_type cb)
{
   int32_t i;
   int32_t ret = -1;

   for (i = 0 ; (i < MCU_GPIO_IN_EVENT_TOTAL) && (ret == -1) ; i++)
   {
      if (eventsInputs[i].cb == NULL)
      {
         eventsInputs[i].cb = cb;
         eventsInputs[i].evType = evType;
         eventsInputs[i].pinId = id;
         ret = i;
      }
   }

   if (ret >= 0)
   {
      Chip_PININT_Init(LPC_GPIO_PIN_INT);
      /* Configure interrupt channel for the GPIO pin in SysCon block */
      Chip_SCU_GPIOIntPinSel(ret,
                             p_gpio[id].gpio.port,
                             p_gpio[id].gpio.pin);

      /* Configure channel interrupt as edge sensitive and falling edge
         interrupt */
      switch(evType)
      {
         case MCU_GPIO_EVENT_TYPE_INPUT_FALLING_EDGE:
            /* Configure the pins as edge sensitive in Pin interrupt block */
            Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(ret));
            Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(ret));
            break;

         case MCU_GPIO_EVENT_TYPE_INPUT_RISING_EDGE:
            /* Configure the pins as edge sensitive in Pin interrupt block */
            Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(ret));
            Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(ret));
            break;

         /* TODO: interrupt for low or high level */

         default:
            break;
      }

      /* Enable interrupt in the NVIC */
      NVIC_EnableIRQ(pinIntIrqMap[ret]);
   }

   return ret;
}

ISR(GPIOINTHandler0)
{
   Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH0);
   eventsInputs[0].cb(eventsInputs[0].pinId, eventsInputs[0].evType);
}

ISR(GPIOINTHandler1)
{
   Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH1);
   eventsInputs[1].cb(eventsInputs[1].pinId, eventsInputs[1].evType);
}

ISR(GPIOINTHandler2)
{
   Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH2);
   eventsInputs[2].cb(eventsInputs[2].pinId, eventsInputs[2].evType);
}

ISR(GPIOINTHandler3)
{
   Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH3);
   eventsInputs[3].cb(eventsInputs[3].pinId, eventsInputs[3].evType);
}

ISR(GPIOINTHandler4)
{
   Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH4);
   eventsInputs[4].cb(eventsInputs[4].pinId, eventsInputs[4].evType);
}

ISR(GPIOINTHandler5)
{
   Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH5);
   eventsInputs[5].cb(eventsInputs[5].pinId, eventsInputs[5].evType);
}

ISR(GPIOINTHandler6)
{
   Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH6);
   eventsInputs[6].cb(eventsInputs[6].pinId, eventsInputs[6].evType);
}

ISR(GPIOINTHandler7)
{
   Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH7);
   eventsInputs[7].cb(eventsInputs[7].pinId, eventsInputs[7].evType);
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
