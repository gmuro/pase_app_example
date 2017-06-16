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
#include "board.h"
#include "mcu.h"
#include "stdint.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
static const mcu_gpio_pinId_enum ledMap[] =
{
   MCU_GPIO_PIN_ID_75,
   MCU_GPIO_PIN_ID_81,
   MCU_GPIO_PIN_ID_84,
   MCU_GPIO_PIN_ID_104,
   MCU_GPIO_PIN_ID_105,
   MCU_GPIO_PIN_ID_106,
};

static const mcu_gpio_pinId_enum switchMap[] =
{
   MCU_GPIO_PIN_ID_38,
   MCU_GPIO_PIN_ID_42,
};

static const int8_t totalLeds = sizeof(ledMap) / sizeof(ledMap[0]);
static const int8_t totalSwitches = sizeof(switchMap) / sizeof(switchMap[0]);

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
void callback_1(mcu_gpio_pinId_enum id,
                mcu_gpio_eventTypeInput_enum evType)
{
   //Dummy
   return;
}

extern void board_init(void)
{
   int8_t i;

   for (i = 0 ; i < totalLeds ; i++)
   {
      mcu_gpio_setDirection(ledMap[i], MCU_GPIO_DIRECTION_OUTPUT);
   }

   for (i = 0 ; i < totalSwitches ; i++)
   {
      mcu_gpio_setDirection(switchMap[i], MCU_GPIO_DIRECTION_INPUT);
   }
}

extern void board_ledSet(board_ledId_enum id, board_ledState_enum state)
{
   mcu_gpio_setOut(ledMap[id], state == BOARD_LED_STATE_ON);
}

extern board_switchState_enum board_switchGet(board_switchId_enum id)
{
   return (mcu_gpio_readInput(switchMap[id])?BOARD_TEC_NON_PRESSED:BOARD_TEC_PRESSED);
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
