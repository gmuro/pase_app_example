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

/*==================[macros and definitions]=================================*/
/* PININT index used for GPIO mapping */
#define PININT_INDEX 0
/* GPIO interrupt NVIC interrupt name */
#define PININT_NVIC_NAME    PIN_INT0_IRQn


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
   {{2,10}, {0,14}, FUNC0},
   {{2,11}, {1,11}, FUNC0},
   {{1,0}, {0,4}, FUNC0},
   {{1,1}, {0,8}, FUNC0},
};

static eventsInputs_type eventsInputs[MCU_GPIO_IN_EVENT_TOTAL];

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
extern void mcu_gpio_init(void)
{
   int32_t i;

   Chip_GPIO_Init(LPC_GPIO_PORT);
   Chip_Clock_Enable(CLK_MX_GPIO);
   for (i = 0 ; i < MCU_GPIO_IN_EVENT_TOTAL ; i++)
   {
      eventsInputs[i].cb = NULL;
   }
}

extern void mcu_gpio_setDirection(mcu_gpio_pinId_enum id,
                                  mcu_gpio_direction_enum dir)
{
   Chip_SCU_PinMux(p_gpio[id].p.port,
                   p_gpio[id].p.pin,
                   MD_PLN,
                   p_gpio[id].modefunc);

   Chip_GPIO_SetDir(LPC_GPIO_PORT,
                    p_gpio[id].gpio.port,
                    (1<<p_gpio[id].gpio.pin),
                    (dir == MCU_GPIO_DIRECTION_OUTPUT));
}

extern void mcu_gpio_setOut(mcu_gpio_pinId_enum id, bool state)
{
   Chip_GPIO_SetPinState(LPC_GPIO_PORT,
                         p_gpio[id].gpio.port,
                         p_gpio[id].gpio.pin,
                         state);
}

extern bool mcu_gpio_readInput(mcu_gpio_pinId_enum id)
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
         ret = 0;
      }
   }

   if (ret != -1)
   {
      /* TODO: configurar interrupción de ese pin */
   }
}


/* TODO: isr del los pines */
#if 0

void nombre_de_isr(void)
{
   mcu_gpio_pinId_enum idPin;
   mcu_gpio_eventTypeInput_enum evType;

   // determinar pin que interrumpio y guardarlo en idPin
   // determinar flanco y guardarlo en evType


   eventsInputs[idPin].cb(idPin, evType);
}

#endif

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
